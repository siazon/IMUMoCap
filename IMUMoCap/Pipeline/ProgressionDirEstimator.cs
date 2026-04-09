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
        private readonly Queue<float> _pdHistory = new();
        private float  _pdHistorySum    = 0f;
        private float  _pdHistorySumSq  = 0f;

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
                                 MotionContext context, MotionContextDetector contextDetector)
        {
            if (context.State != ContextState.Straight && context.State != ContextState.ReacquiringPd)
                return BuildEstimate();

            float pelvisYaw = frame.Pelvis.HasQuaternion
                ? ExtractYaw(frame.Pelvis.Quaternion)
                : _currentPd;

            TrackStanceYaw(frame, gait);

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
            _pdHistorySum   = 0f;
            _pdHistorySumSq = 0f;
            _leftInStance = _rightInStance = false;
            _leftStanceFrames = _rightStanceFrames = 0;
            _leftYawReady = _rightYawReady = false;
            _lastLeftYaw  = _lastRightYaw  = 0f;
        }

        // ── 私有方法 ──────────────────────────────────────────────────────────

        private void TrackStanceYaw(ValidFrame frame, GaitEvent gait)
        {
            if (gait.LeftStance)
            {
                if (!_leftInStance) { _leftInStance = true; _leftStanceYaw = 0f; _leftStanceFrames = 0; }
                _leftStanceYaw += ExtractYaw(frame.LeftFoot.Quaternion);
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
                _rightStanceYaw += ExtractYaw(frame.RightFoot.Quaternion);
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
            _pdHistorySum   += pd;
            _pdHistorySumSq += pd * pd;
            if (_pdHistory.Count > StabilityWindow)
            {
                float old = _pdHistory.Dequeue();
                _pdHistorySum   -= old;
                _pdHistorySumSq -= old * old;
            }
        }

        private float ComputeStability()
        {
            int n = _pdHistory.Count;
            if (n < 2) return 0f;
            float mean     = _pdHistorySum / n;
            float variance = _pdHistorySumSq / n - mean * mean;
            variance = MathF.Max(variance, 0f);
            return 1f / (1f + variance);
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
    }
}
