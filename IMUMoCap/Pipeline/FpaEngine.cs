// IMUMoCap/Pipeline/FpaEngine.cs
using System;
using System.Collections.Generic;
using IMUMoCap.Pipeline.Models;

namespace IMUMoCap.Pipeline
{
    /// <summary>
    /// 四重门控 + 早-稳结算，输出步级 FpaResult。
    ///
    /// 四重门控（全部满足才进入结算）：
    ///   1. 帧有效（ValidFrame，已由 DataQualityGate 保证）
    ///   2. 步有效：GaitEvent 显示完整 stance 周期（左脚或右脚处于 stance）
    ///   3. 上下文允许：MotionContext.State == Straight 且 Confidence >= ContextConfidenceThreshold
    ///   4. PD 有效：PdEstimate.IsValid == true 且 Stability >= PdStabilityThreshold
    ///
    /// 早-稳结算（per foot，独立）：
    ///   在 stance 期间持续估计 FPA，监测滑动窗口方差；
    ///   方差 < VarianceThreshold 时立即结算；
    ///   超过 StancePhaseUpperLimit（0–1，stance 相位比例）时强制结算。
    ///
    /// FPA 计算：
    ///   FPA = mean(foot_yaw during stance) - PD.DirectionRad，转换为度
    ///   正值 = toe-out，负值 = toe-in
    /// </summary>
    public sealed class FpaEngine
    {
        // ── 门控阈值 ──────────────────────────────────────────────────────────
        public float ContextConfidenceThreshold { get; set; } = 0.7f;
        public float PdStabilityThreshold       { get; set; } = 0.7f;

        // ── 早-稳结算参数 ──────────────────────────────────────────────────────
        public float VarianceThreshold        { get; set; } = 1.0f;  // rad²，方差收敛阈值
        public float StancePhaseUpperLimit    { get; set; } = 0.70f; // stance 时长的 70% 强制结算
        public int   MinStanceFramesForSettle { get; set; } = 5;     // 最少帧数才允许结算

        // ── BaselineProfile（Training 阶段设置，Baseline 阶段为 null）────────
        public BaselineProfile? Baseline { get; set; }

        // ── 内部 stance 追踪（per foot）───────────────────────────────────────
        private StanceSampler _leftSampler  = new();
        private StanceSampler _rightSampler = new();

        public FpaResult? Process(ValidFrame frame, GaitEvent gait,
                                  MotionContext context, PdEstimate pd,
                                  CalibrationProfile? calibration = null)
        {
            // 门控 3 & 4：上下文和 PD
            bool contextOk = context.State == ContextState.Straight
                          && context.Confidence >= ContextConfidenceThreshold;
            bool pdOk = pd.IsValid && pd.Stability >= PdStabilityThreshold;

            if (!contextOk || !pdOk)
            {
                _leftSampler.Reset();
                _rightSampler.Reset();
                return null;
            }

            // 门控 2：step valid（至少一脚在 stance）
            if (!gait.LeftStance && !gait.RightStance)
            {
                _leftSampler.Reset();
                _rightSampler.Reset();
                return null;
            }

            // 采样
            if (gait.LeftStance)
                _leftSampler.AddFrame(ExtractYaw(CalibrateQuaternion(frame.LeftFoot.Quaternion, calibration?.LeftFootRef)));
            else
                _leftSampler.MarkSwing();

            if (gait.RightStance)
                _rightSampler.AddFrame(ExtractYaw(CalibrateQuaternion(frame.RightFoot.Quaternion, calibration?.RightFootRef)));
            else
                _rightSampler.MarkSwing();

            // 检查是否有脚完成了结算
            float? fpaL = _leftSampler.TrySettle(VarianceThreshold, StancePhaseUpperLimit,
                                                   MinStanceFramesForSettle);
            float? fpaR = _rightSampler.TrySettle(VarianceThreshold, StancePhaseUpperLimit,
                                                   MinStanceFramesForSettle);

            // 只有至少一脚完成结算时才输出结果
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

        /// <summary>
        /// 用校准参考四元数修正测量四元数。
        /// 返回相对参考姿态的相对旋转：q_corrected = Inverse(q_ref) * q_measured
        /// </summary>
        private static System.Numerics.Quaternion CalibrateQuaternion(System.Numerics.Quaternion measured,
                                                                       System.Numerics.Quaternion? reference)
        {
            if (!reference.HasValue) return measured;
            return System.Numerics.Quaternion.Multiply(System.Numerics.Quaternion.Inverse(reference.Value), measured);
        }

        // ── StanceSampler（内嵌私有类）────────────────────────────────────────
        // 追踪单脚在 stance 期间的 yaw 样本，实现早-稳结算逻辑

        private sealed class StanceSampler
        {
            private readonly List<float> _yaws = new();
            private bool  _inStance                   = false;
            private bool  _settled                    = false;
            private float _settledYaw                 = 0f;
            private int   _totalStanceFramesEstimate  = 0; // 用前次 stance 长度估算

            public void AddFrame(float yaw)
            {
                if (!_inStance)
                {
                    _inStance = true;
                    _settled  = false;
                    _yaws.Clear();
                }
                _yaws.Add(yaw);
            }

            public void MarkSwing()
            {
                if (_inStance)
                {
                    // stance 结束，如果还没结算，记录当前均值
                    if (!_settled && _yaws.Count > 0)
                    {
                        _settledYaw = Mean(_yaws);
                        _settled    = true;
                    }
                    // 更新 stance 长度估计（取最近一次）
                    if (_yaws.Count > 0)
                        _totalStanceFramesEstimate = _yaws.Count;
                    _inStance = false;
                }
            }

            /// <summary>
            /// 在 stance 中途尝试提前结算。
            /// 返回结算值（rad）或 null（尚未结算/上次已消费）。
            /// </summary>
            public float? TrySettle(float varianceThreshold, float phaseUpperLimit, int minFrames)
            {
                // 已结算且 stance 刚结束（MarkSwing 触发）：返回结果并消费
                if (_settled && !_inStance)
                {
                    _settled = false;
                    return _settledYaw;
                }

                if (!_inStance || _settled) return null;
                if (_yaws.Count < minFrames)  return null;

                // 计算当前方差
                float mean = Mean(_yaws);
                float var  = Variance(_yaws, mean);

                // 方差收敛
                if (var < varianceThreshold)
                {
                    _settledYaw = mean;
                    _settled    = true;
                    return null; // 等 swing 开始后再报告，避免重复
                }

                // 超过相位上限，强制结算
                if (_totalStanceFramesEstimate > 0)
                {
                    float phase = (float)_yaws.Count / _totalStanceFramesEstimate;
                    if (phase >= phaseUpperLimit)
                    {
                        _settledYaw = mean;
                        _settled    = true;
                    }
                }

                return null;
            }

            public void Reset()
            {
                _yaws.Clear();
                _inStance = false;
                _settled  = false;
            }

            private static float Mean(List<float> vs)
            {
                float sum = 0f;
                foreach (var v in vs) sum += v;
                return sum / vs.Count;
            }

            private static float Variance(List<float> vs, float mean)
            {
                float sumSq = 0f;
                foreach (var v in vs) sumSq += (v - mean) * (v - mean);
                return sumSq / vs.Count;
            }
        }
    }
}
