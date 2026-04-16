// IMUMoCap/Pipeline/ProgressionDirEstimator.cs
using System;
using System.Collections.Generic;
using IMUMoCap.Pipeline.Models;

namespace IMUMoCap.Pipeline
{
    /// <summary>
    /// 在线估计行进方向（Progression Direction, PD）。
    ///
    /// 仅在 MotionContext.State == Straight 或 ReacquiringPd 时更新。
    /// 融合三路信号（权重可调）：
    ///   - Pelvis heading（yaw）：主信号，稳定性好
    ///   - Left foot heading（stance 相位均值 yaw）
    ///   - Right foot heading（stance 相位均值 yaw）
    ///
    /// 稳定性（Stability）= 1 / (1 + variance)，滑动窗口内各步估计值的方差。
    ///
    /// ReacquiringPd 退出条件（两者同时满足）：
    ///   1. Stability >= StabilityThreshold
    ///   2. 已积累 MinStepsBeforeValid 步
    /// 满足后调用 motionContext.ConfirmStraight()。
    /// </summary>
    public sealed class ProgressionDirEstimator
    {
        // ── 可调参数 ──────────────────────────────────────────────────────────
        public float PelvisWeight        { get; set; } = 0.6f;
        public float LeftFootWeight      { get; set; } = 0.2f;
        public float RightFootWeight     { get; set; } = 0.2f;
        public int   StabilityWindow     { get; set; } = 10;    // 步数滑动窗口
        public float StabilityThreshold  { get; set; } = 0.85f;
        public int   MinStepsBeforeValid { get; set; } = 5;

        // ── 内部状态 ──────────────────────────────────────────────────────────
        private float  _currentPd       = 0f;
        private bool   _hasEstimate     = false;
        private int    _stepCount       = 0;
        private readonly Queue<float> _pdHistory    = new();
        private float  _pdHistorySumCos = 0f;  // 用于圆形方差（处理角度 ±π 环绕）
        private float  _pdHistorySumSin = 0f;

        private bool  _leftInStance     = false;
        private bool  _rightInStance    = false;
        private float _leftStanceYaw    = 0f;
        private float _rightStanceYaw   = 0f;
        private int   _leftStanceFrames  = 0;
        private int   _rightStanceFrames = 0;
        private bool  _leftYawReady     = false;
        private bool  _rightYawReady    = false;
        private float _lastLeftYaw      = 0f;
        private float _lastRightYaw     = 0f;

        public PdEstimate Update(ValidFrame frame, GaitEvent gait,
                                 MotionContext context, MotionContextDetector contextDetector,
                                 CalibrationProfile? calibration = null)
        {
            if (context.State != ContextState.Straight && context.State != ContextState.ReacquiringPd)
                return BuildEstimate();

            float pelvisYaw = frame.Pelvis.HasQuaternion
                ? ExtractYaw(CalibrateQuaternion(frame.Pelvis.Quaternion, calibration?.PelvisRef))
                : _currentPd;

            TrackStanceYaw(frame, gait, calibration);

            if (!TryConsumeStep(out float leftYaw, out float rightYaw))
                return BuildEstimate();

            _stepCount++;
            float fused = FuseDirection(pelvisYaw, leftYaw, rightYaw);
            UpdateHistory(fused);
            _currentPd   = fused;
            _hasEstimate = true;

            if (context.State == ContextState.ReacquiringPd)
            {
                float stability = ComputeStability();
                if (stability >= StabilityThreshold && _stepCount >= MinStepsBeforeValid)
                    contextDetector.ConfirmStraight();
            }

            return BuildEstimate();
        }

        public void Reset()
        {
            _currentPd   = 0f;
            _hasEstimate = false;
            _stepCount   = 0;
            _pdHistory.Clear();
            _pdHistorySumCos = 0f;
            _pdHistorySumSin = 0f;
            _leftInStance = _rightInStance = false;
            _leftStanceFrames = _rightStanceFrames = 0;
            _leftYawReady = _rightYawReady = false;
            _lastLeftYaw  = _lastRightYaw  = 0f;
        }

        // ── 私有方法 ──────────────────────────────────────────────────────────

        private void TrackStanceYaw(ValidFrame frame, GaitEvent gait, CalibrationProfile? calibration)
        {
            if (gait.LeftStance)
            {
                if (!_leftInStance) { _leftInStance = true; _leftStanceYaw = 0f; _leftStanceFrames = 0; }
                _leftStanceYaw += ExtractYaw(CalibrateQuaternion(frame.LeftFoot.Quaternion, calibration?.LeftFootRef));
                _leftStanceFrames++;
            }
            else if (_leftInStance)
            {
                _leftInStance  = false;
                _lastLeftYaw   = _leftStanceFrames > 0 ? _leftStanceYaw / _leftStanceFrames : _lastLeftYaw;
                _leftYawReady  = true;
            }

            if (gait.RightStance)
            {
                if (!_rightInStance) { _rightInStance = true; _rightStanceYaw = 0f; _rightStanceFrames = 0; }
                _rightStanceYaw += ExtractYaw(CalibrateQuaternion(frame.RightFoot.Quaternion, calibration?.RightFootRef));
                _rightStanceFrames++;
            }
            else if (_rightInStance)
            {
                _rightInStance  = false;
                _lastRightYaw   = _rightStanceFrames > 0 ? _rightStanceYaw / _rightStanceFrames : _lastRightYaw;
                _rightYawReady  = true;
            }
        }

        private bool TryConsumeStep(out float leftYaw, out float rightYaw)
        {
            leftYaw  = _lastLeftYaw;
            rightYaw = _lastRightYaw;
            if (_leftYawReady && _rightYawReady)
            {
                _leftYawReady = _rightYawReady = false;
                return true;
            }
            return false;
        }

        private float FuseDirection(float pelvisYaw, float leftYaw, float rightYaw)
        {
            // 加权平均，注意角度的圆周性（用向量和再取 atan2）
            float totalW = PelvisWeight + LeftFootWeight + RightFootWeight;
            float wx = (MathF.Cos(pelvisYaw) * PelvisWeight
                      + MathF.Cos(leftYaw)   * LeftFootWeight
                      + MathF.Cos(rightYaw)  * RightFootWeight) / totalW;
            float wy = (MathF.Sin(pelvisYaw) * PelvisWeight
                      + MathF.Sin(leftYaw)   * LeftFootWeight
                      + MathF.Sin(rightYaw)  * RightFootWeight) / totalW;
            return MathF.Atan2(wy, wx);
        }

        private void UpdateHistory(float pd)
        {
            _pdHistory.Enqueue(pd);
            _pdHistorySumCos += MathF.Cos(pd);
            _pdHistorySumSin += MathF.Sin(pd);
            if (_pdHistory.Count > StabilityWindow)
            {
                float old = _pdHistory.Dequeue();
                _pdHistorySumCos -= MathF.Cos(old);
                _pdHistorySumSin -= MathF.Sin(old);
            }
        }

        private float ComputeStability()
        {
            int n = _pdHistory.Count;
            if (n < 2) return 0f;
            // 平均合矢量长度 R̄ ∈ [0,1]：1 = 方向完全一致，0 = 方向随机分布
            // 圆形方差 = 1 - R̄，代入线性公式保持阈值语义不变
            float rBar    = MathF.Sqrt(_pdHistorySumCos * _pdHistorySumCos +
                                       _pdHistorySumSin * _pdHistorySumSin) / n;
            float circVar = 1f - rBar;
            return 1f / (1f + circVar);
        }

        private PdEstimate BuildEstimate()
        {
            float stability = ComputeStability();
            return new PdEstimate
            {
                DirectionRad = _currentPd,
                IsValid      = _hasEstimate && stability >= StabilityThreshold,
                Stability    = stability,
            };
        }

        private static float ExtractYaw(System.Numerics.Quaternion q)
            => MathF.Atan2(2f * (q.W * q.Z + q.X * q.Y),
                           1f - 2f * (q.Y * q.Y + q.Z * q.Z));

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
    }
}
