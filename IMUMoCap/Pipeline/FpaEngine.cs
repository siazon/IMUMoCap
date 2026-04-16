// IMUMoCap/Pipeline/FpaEngine.cs
using System;
using System.Collections.Generic;
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
        public float PdStabilityThreshold       { get; set; } = 0.7f;

        // ── 结算参数 ──────────────────────────────────────────────────────────
        // swing→stance 后累积此帧数再输出 FPA（@100Hz，10帧=100ms）
        public int MinStanceFramesForSettle { get; set; } = 10;

        // ── BaselineProfile（Training 阶段设置，Baseline 阶段为 null）────────
        public BaselineProfile? Baseline { get; set; }

        // ── 内部 stance 追踪（per foot）───────────────────────────────────────
        private readonly StanceSampler _leftSampler  = new();
        private readonly StanceSampler _rightSampler = new();

        public FpaResult? Process(ValidFrame frame, GaitEvent gait,
                                  MotionContext context, PdEstimate pd,
                                  CalibrationProfile? calibration = null)
        {
            // ── Step 1: 无条件更新 stance 状态（采集与 gate 无关）────────────────
            if (gait.LeftStance)
                _leftSampler.AddFrame(ExtractYaw(CalibrateQuaternion(frame.LeftFoot.Quaternion, calibration?.LeftFootRef)));
            else
                _leftSampler.MarkSwing();

            if (gait.RightStance)
                _rightSampler.AddFrame(ExtractYaw(CalibrateQuaternion(frame.RightFoot.Quaternion, calibration?.RightFootRef)));
            else
                _rightSampler.MarkSwing();

            // ── Step 2: Gate 检查——仅影响输出，不影响采集 ──────────────────────
            bool contextOk = context.State == ContextState.Straight
                          && context.Confidence >= ContextConfidenceThreshold;
            bool pdOk = pd.IsValid && pd.Stability >= PdStabilityThreshold;
            if (!contextOk || !pdOk) return null;

            // 两脚都在 swing（双悬空）时无意义，不输出
            if (!gait.LeftStance && !gait.RightStance) return null;

            // ── Step 3: 尝试结算：落地后 n 帧已满 + gate 通过 → 输出 ──────────
            float? fpaL = _leftSampler.TrySettle(MinStanceFramesForSettle);
            float? fpaR = _rightSampler.TrySettle(MinStanceFramesForSettle);

            if (fpaL == null && fpaR == null) return null;

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
                    errorL    = fpaLDeg - Baseline.Target_L;
                    onTargetL = MathF.Abs(errorL) <= Baseline.Tolerance_L;
                }
                if (!float.IsNaN(fpaRDeg))
                {
                    errorR    = fpaRDeg - Baseline.Target_R;
                    onTargetR = MathF.Abs(errorR) <= Baseline.Tolerance_R;
                }
            }

            return new FpaResult
            {
                PacketId   = frame.PacketId,
                Fpa_L      = fpaLDeg,
                Fpa_R      = fpaRDeg,
                OnTarget_L = onTargetL,
                OnTarget_R = onTargetR,
                Error_L    = errorL,
                Error_R    = errorR,
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
            while (rad >  MathF.PI) rad -= 2f * MathF.PI;
            while (rad < -MathF.PI) rad += 2f * MathF.PI;
            return rad;
        }

        private static float RadToDeg(float rad) => rad * (180f / MathF.PI);

        private static System.Numerics.Quaternion CalibrateQuaternion(
            System.Numerics.Quaternion measured, System.Numerics.Quaternion? reference)
        {
            if (!reference.HasValue) return measured;
            return System.Numerics.Quaternion.Multiply(
                System.Numerics.Quaternion.Inverse(reference.Value), measured);
        }

        // ── StanceSampler（内嵌私有类）────────────────────────────────────────
        // 单脚状态机：检测 swing→stance 转换，落地后采集 n 帧，输出一次 FPA。

        private sealed class StanceSampler
        {
            private readonly List<float> _yaws     = new();
            private bool _inStance                 = false;
            private bool _emittedThisStance        = false; // 本 stance 周期已输出过

            /// <summary>脚处于 stance 时每帧调用，首次调用即为 swing→stance 转换。</summary>
            public void AddFrame(float yaw)
            {
                if (!_inStance)
                {
                    // swing→stance 转换：重置本步采集状态
                    _inStance          = true;
                    _emittedThisStance = false;
                    _yaws.Clear();
                }
                _yaws.Add(yaw);
            }

            /// <summary>脚处于 swing 时每帧调用。</summary>
            public void MarkSwing()
            {
                _inStance = false;
            }

            /// <summary>
            /// 尝试输出本步 FPA（rad）。
            /// 条件：处于 stance + 已累积 minFrames 帧 + 本 stance 尚未输出。
            /// 满足则返回均值并标记已输出；否则返回 null。
            /// </summary>
            public float? TrySettle(int minFrames)
            {
                if (!_inStance || _emittedThisStance)  return null;
                if (_yaws.Count < minFrames)            return null;

                _emittedThisStance = true;
                return Mean(_yaws);
            }

            public void Reset()
            {
                _yaws.Clear();
                _inStance          = false;
                _emittedThisStance = false;
            }

            private static float Mean(List<float> vs)
            {
                float sum = 0f;
                foreach (var v in vs) sum += v;
                return sum / vs.Count;
            }
        }
    }
}
