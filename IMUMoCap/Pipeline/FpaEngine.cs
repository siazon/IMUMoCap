// IMUMoCap/Pipeline/FpaEngine.cs
using System;
using System.Collections.Generic;
using System.Diagnostics;
using IMUMoCap.Pipeline.Models;

namespace IMUMoCap.Pipeline
{
    /// <summary>
    /// 门控 + 落地稳定后结算，输出步级 FpaResult。
    ///
    /// 采集与输出分离：
    ///   采集：只要脚在 stance 就持续采集 yaw，gate 失败不丢弃已采集数据。
    ///   输出：数据驱动结算——落地满 MinStanceFramesFloor 帧后，一旦该脚角速度连续
    ///         SettleQuietFrames 帧低于 SettleGyroThreshold（真正静止，不是固定帧数瞎猜）
    ///         AND 当前帧 gate 全通过 → 立即输出 FPA；若一直不安静，到 MaxStanceFramesForSettle
    ///         硬上限强制结算，防止个别落地冲击大的步子卡住不输出。
    ///
    /// FPA 计算：
    ///   FPA = mean(foot_yaw，落地到结算这段帧) - PD.DirectionRad，转换为度
    ///   正值 = toe-out，负值 = toe-in
    /// </summary>
    public sealed class FpaEngine
    {
        // ── 门控阈值 ──────────────────────────────────────────────────────────
        public float ContextConfidenceThreshold { get; set; } = 0.7f;
        public float PdStabilityThreshold { get; set; } = 0.7f;

        // ── 结算参数（数据驱动）──────────────────────────────────────────────
        // 落地后至少攒够这么多帧才可能结算，防止单帧抖动/刚过 debounce 边界那一帧被当成"已安静"
        public int MinStanceFramesFloor { get; set; } = 2;
        // 角速度模长连续低于此值达到 SettleQuietFrames 帧，才认为脚真正静止（比 GaitEventDetector
        // 判定"算不算落地"用的 GyroThreshold=1.0 更紧）。0.35 是在两份真实录制（一份转身较多的
        // 测试走动、一份 188s 的 Baseline+Training 正式录制）上验证过的值：0.2 时正常走路(Straight)
        // 下仍有 4-8% 的落地因为一直没连续 3 帧低于阈值而被静默丢弃（docs/task12、task13），
        // 0.3-0.4 基本消除该风险，0.35 取中间留一点余量。
        public float SettleGyroThreshold { get; set; } = 0.35f; // rad/s
        public int SettleQuietFrames { get; set; } = 3;
        // 硬上限：即使一直不安静也不能无限等（落地冲击大/有回弹的步子），到点强制结算
        public int MaxStanceFramesForSettle { get; set; } = 20; // frames (200ms @ 100Hz)

        // ── BaselineProfile（Training 阶段设置，Baseline 阶段为 null）────────
        public BaselineProfile? Baseline { get; set; }

        // ── 容差带系数 α：w = max(w_min, α·SD)，随训练块递减实现渐进式难度 ──────
        public float ToleranceAlpha { get; set; } = 1.5f;

        // ── 步级结果事件：每个真正落地(stance)完成的周期，无论有没有输出 FpaResult ──
        // emitted=true 时 reason 为 null；emitted=false 时 reason 说明被门控排除的原因
        // （用于统计 Turning 等排除比例，供 QoE 分析用，不影响 FpaResult 本身的输出逻辑）
        public event Action<string, bool, StepExclusionReason?>? OnStepOutcome;

        // ── 内部 stance 追踪（per foot）───────────────────────────────────────
        private readonly StanceSampler _leftSampler = new();
        private readonly StanceSampler _rightSampler = new();

        public FpaResult? Process(ValidFrame frame, GaitEvent gait,
                                  MotionContext context, PdEstimate pd,
                                  CalibrationProfile? calibration = null)
        {
            // ── Step 1: 无条件更新 stance 状态（采集与 gate 无关）────────────────
            // stance→swing 的转换帧上，如果这一步从未成功输出过 FPA，视为一次排除。
            if (gait.LeftStance)
                _leftSampler.AddFrame(NormalizeAngle(
                    ExtractYaw(frame.LeftFoot.Quaternion) -
                    ExtractYaw(calibration?.LeftFootRef ?? System.Numerics.Quaternion.Identity)),
                    frame.LeftFoot.RateOfTurn.Length(), SettleGyroThreshold);
            else
            {
                var excludedL = _leftSampler.MarkSwing();
                if (excludedL.HasValue) OnStepOutcome?.Invoke("L", false, excludedL);
            }

            if (gait.RightStance)
                _rightSampler.AddFrame(NormalizeAngle(
                    ExtractYaw(frame.RightFoot.Quaternion) -
                    ExtractYaw(calibration?.RightFootRef ?? System.Numerics.Quaternion.Identity)),
                    frame.RightFoot.RateOfTurn.Length(), SettleGyroThreshold);
            else
            {
                var excludedR = _rightSampler.MarkSwing();
                if (excludedR.HasValue) OnStepOutcome?.Invoke("R", false, excludedR);
            }

            // ── Step 2: Gate 检查——仅影响输出，不影响采集 ──────────────────────
            bool contextOk = context.State == ContextState.Straight
                          && context.Confidence >= ContextConfidenceThreshold;
            bool pdOk = pd.IsValid && pd.Stability >= PdStabilityThreshold;

            // 门控失败原因（仅在 !contextOk 时才会被 NoteGate 采用；contextOk 但 !pdOk 时用 PdInvalid）
            StepExclusionReason blockReason = context.State switch
            {
                ContextState.Turning       => StepExclusionReason.Turning,
                ContextState.ReacquiringPd => StepExclusionReason.ReacquiringPd,
                _ /* Straight 但 confidence 不够，或 pd 不合格 */ =>
                    contextOk ? StepExclusionReason.PdInvalid : StepExclusionReason.LowConfidence,
            };
            _leftSampler.NoteGate(MinStanceFramesFloor, SettleQuietFrames, MaxStanceFramesForSettle, contextOk && pdOk, blockReason);
            _rightSampler.NoteGate(MinStanceFramesFloor, SettleQuietFrames, MaxStanceFramesForSettle, contextOk && pdOk, blockReason);

            if (!contextOk || !pdOk) return null;
            // 两脚都在 swing（双悬空）时无意义，不输出
            if (!gait.LeftStance && !gait.RightStance) return null;

            // ── Step 3: 尝试结算：脚已安静（或触发硬上限）+ gate 通过 → 输出 ──────
            float? fpaL = _leftSampler.TrySettle(MinStanceFramesFloor, SettleQuietFrames, MaxStanceFramesForSettle);
            if (fpaL.HasValue) OnStepOutcome?.Invoke("L", true, null);
            float? fpaR = _rightSampler.TrySettle(MinStanceFramesFloor, SettleQuietFrames, MaxStanceFramesForSettle);
            if (fpaR.HasValue) OnStepOutcome?.Invoke("R", true, null);

            if (fpaL == null && fpaR == null) return null;

            // 通过 gate 才能走到这里，此时的 confidence/stability 即决定本次输出的原始质量
            bool marginal = (context.Confidence - ContextConfidenceThreshold < 0.1f)
                         || (pd.Stability - PdStabilityThreshold < 0.1f);
            string quality = marginal ? "Marginal" : "High";

            float fpaLDeg = fpaL.HasValue
                ? RadToDeg(NormalizeAngle(fpaL.Value - pd.DirectionRad))
                : float.NaN;
            float fpaRDeg = fpaR.HasValue
                ? RadToDeg(NormalizeAngle(fpaR.Value - pd.DirectionRad))
                : float.NaN;

            bool onTargetL = false, onTargetR = false;
            float errorL = float.NaN, errorR = float.NaN;

            if (Baseline != null)
            {
                if (!float.IsNaN(fpaLDeg))
                {
                    errorL = fpaLDeg - Baseline.Target_L;
                    onTargetL = MathF.Abs(errorL) <= Baseline.ToleranceL(ToleranceAlpha);
                }
                if (!float.IsNaN(fpaRDeg))
                {
                    errorR = fpaRDeg - Baseline.Target_R;
                    onTargetR = MathF.Abs(errorR) <= Baseline.ToleranceR(ToleranceAlpha);
                }
            }

            return new FpaResult
            {
                PacketId = frame.PacketId,
                Fpa_L = fpaLDeg,
                Fpa_R = fpaRDeg,
                OnTarget_L = onTargetL,
                OnTarget_R = onTargetR,
                Error_L = errorL,
                Error_R = errorR,
                Tolerance_L = Baseline?.ToleranceL(ToleranceAlpha) ?? float.NaN,
                Tolerance_R = Baseline?.ToleranceR(ToleranceAlpha) ?? float.NaN,
                ContextConfidence = context.Confidence,
                PdStability = pd.Stability,
                Quality = quality,
            };
        }

        public void Reset()
        {
            _leftSampler.Reset();
            _rightSampler.Reset();
        }

        // ── 工具方法 ──────────────────────────────────────────────────────────

        private static float ExtractYaw(System.Numerics.Quaternion q)
            => MathF.Atan2(2f * (q.W * q.Z + q.X * q.Y),
                           1f - 2f * (q.Y * q.Y + q.Z * q.Z));

        private static float NormalizeAngle(float rad)
        {
            while (rad > MathF.PI) rad -= 2f * MathF.PI;
            while (rad < -MathF.PI) rad += 2f * MathF.PI;
            return rad;
        }

        private static float RadToDeg(float rad) => rad * (180f / MathF.PI);

        // ── StanceSampler（内嵌私有类）────────────────────────────────────────
        // 单脚状态机：检测 swing→stance 转换，落地后采集，直到脚真正安静下来（或触发硬上限）
        // 再输出一次 FPA。

        private sealed class StanceSampler
        {
            private float _cosSum = 0f;
            private float _sinSum = 0f;
            private int _count = 0;
            private bool _inStance = false;
            private bool _emittedThisStance = false;

            // ── 排除原因追踪：这一步是否已经达到过结算所需帧数，以及当时被什么原因挡住 ──
            private bool _reachedReady = false;
            private StepExclusionReason? _pendingReason = null;

            // ── 数据驱动结算：角速度连续低于阈值的帧数 ──────────────────────────
            private int _quietStreak = 0;

            public void AddFrame(float yaw, float gyroMagRadPerSec, float settleGyroThreshold)
            {
                if (!_inStance)
                {
                    _inStance = true;
                    _emittedThisStance = false;
                    _reachedReady = false;
                    _pendingReason = null;
                    _cosSum = 0f;
                    _sinSum = 0f;
                    _count = 0;
                    _quietStreak = 0;
                }
                _cosSum += MathF.Cos(yaw);
                _sinSum += MathF.Sin(yaw);
                _count++;
                _quietStreak = gyroMagRadPerSec < settleGyroThreshold ? _quietStreak + 1 : 0;
            }

            // 已达最小帧数下限，且（连续 settleQuietFrames 帧真正安静 或 触及硬上限）→ 可以结算。
            private bool HasSettled(int minFramesFloor, int settleQuietFrames, int maxFrames) =>
                _count >= minFramesFloor && (_quietStreak >= settleQuietFrames || _count >= maxFrames);

            /// <summary>每帧调用一次，记录门控状态；已安静但门控未过时，暂存排除原因。</summary>
            public void NoteGate(int minFramesFloor, int settleQuietFrames, int maxFrames,
                                  bool gateOk, StepExclusionReason reasonIfBlocked)
            {
                if (!_inStance || _emittedThisStance || !HasSettled(minFramesFloor, settleQuietFrames, maxFrames)) return;
                _reachedReady = true;
                _pendingReason = gateOk ? null : reasonIfBlocked;
            }

            /// <summary>
            /// 落地→抬脚的转换帧上调用。若这一步曾经达标却从未成功输出过 FPA，
            /// 返回最后一次记录的排除原因；否则返回 null（要么仍在摆动，要么已正常输出）。
            /// </summary>
            public StepExclusionReason? MarkSwing()
            {
                StepExclusionReason? excludedReason = null;
                if (_inStance && !_emittedThisStance && _reachedReady)
                    excludedReason = _pendingReason;

                _inStance = false;
                _reachedReady = false;
                _pendingReason = null;
                return excludedReason;
            }

            public float? TrySettle(int minFramesFloor, int settleQuietFrames, int maxFrames)
            {
                if (!_inStance || _emittedThisStance) return null;
                if (!HasSettled(minFramesFloor, settleQuietFrames, maxFrames)) return null;

                _emittedThisStance = true;
                return MathF.Atan2(_sinSum, _cosSum);
            }

            public void Reset()
            {
                _cosSum = _sinSum = 0f;
                _count = 0;
                _inStance = false;
                _emittedThisStance = false;
                _reachedReady = false;
                _pendingReason = null;
                _quietStreak = 0;
            }
        }
    }
}
