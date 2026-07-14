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
    ///   输出：swing→stance 后累积 MinStanceFramesForSettle 帧 AND 当前帧 gate 全通过
    ///         → 立即输出 FPA，本 stance 周期不再重复输出。
    ///
    /// FPA 计算：
    ///   FPA = mean(foot_yaw，落地后前 n 帧) - PD.DirectionRad，转换为度
    ///   正值 = toe-out，负值 = toe-in
    /// </summary>
    public sealed class FpaEngine
    {
        // ── 门控阈值 ──────────────────────────────────────────────────────────
        public float ContextConfidenceThreshold { get; set; } = 0.7f;
        public float PdStabilityThreshold { get; set; } = 0.7f;

        // ── 结算参数 ──────────────────────────────────────────────────────────
        // swing→stance 后累积此帧数再输出 FPA（@100Hz，10帧=100ms）
        public int MinStanceFramesForSettle { get; set; } = 10;

        // ── BaselineProfile（Training 阶段设置，Baseline 阶段为 null）────────
        public BaselineProfile? Baseline { get; set; }

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
                    ExtractYaw(calibration?.LeftFootRef ?? System.Numerics.Quaternion.Identity)));
            else
            {
                var excludedL = _leftSampler.MarkSwing();
                if (excludedL.HasValue) OnStepOutcome?.Invoke("L", false, excludedL);
            }

            if (gait.RightStance)
                _rightSampler.AddFrame(NormalizeAngle(
                    ExtractYaw(frame.RightFoot.Quaternion) -
                    ExtractYaw(calibration?.RightFootRef ?? System.Numerics.Quaternion.Identity)));
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
            _leftSampler.NoteGate(MinStanceFramesForSettle, contextOk && pdOk, blockReason);
            _rightSampler.NoteGate(MinStanceFramesForSettle, contextOk && pdOk, blockReason);

            if (!contextOk || !pdOk) return null;
            // 两脚都在 swing（双悬空）时无意义，不输出
            if (!gait.LeftStance && !gait.RightStance) return null;

            // ── Step 3: 尝试结算：落地后 n 帧已满 + gate 通过 → 输出 ──────────
            float? fpaL = _leftSampler.TrySettle(MinStanceFramesForSettle);
            if (fpaL.HasValue) OnStepOutcome?.Invoke("L", true, null);
            float? fpaR = _rightSampler.TrySettle(MinStanceFramesForSettle);
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
                    onTargetL = MathF.Abs(errorL) <= Baseline.Tolerance_L;
                }
                if (!float.IsNaN(fpaRDeg))
                {
                    errorR = fpaRDeg - Baseline.Target_R;
                    onTargetR = MathF.Abs(errorR) <= Baseline.Tolerance_R;
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
                Tolerance_L = Baseline?.Tolerance_L ?? float.NaN,
                Tolerance_R = Baseline?.Tolerance_R ?? float.NaN,
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
        // 单脚状态机：检测 swing→stance 转换，落地后采集 n 帧，输出一次 FPA。

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

            public void AddFrame(float yaw)
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
                }
                _cosSum += MathF.Cos(yaw);
                _sinSum += MathF.Sin(yaw);
                _count++;
            }

            /// <summary>每帧调用一次，记录门控状态；帧数达标但门控未过时，暂存排除原因。</summary>
            public void NoteGate(int minFrames, bool gateOk, StepExclusionReason reasonIfBlocked)
            {
                if (!_inStance || _emittedThisStance || _count < minFrames) return;
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

            public float? TrySettle(int minFrames)
            {
                if (!_inStance || _emittedThisStance) return null;
                if (_count < minFrames) return null;

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
            }
        }
    }
}
