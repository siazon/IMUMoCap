// IMUMoCap/Pipeline/MotionContextDetector.cs
using System;
using System.Collections.Generic;
using IMUMoCap.Pipeline.Models;

namespace IMUMoCap.Pipeline
{
    /// <summary>
    /// 识别运动上下文：Straight / Turning / ReacquiringPd。
    ///
    /// 四路信号加权投票，持续 TurningConfirmFrames 帧满足才切换到 Turning：
    /// 1. Pelvis yaw rate 绝对值 > YawRateThreshold（主信号）
    /// 2. 短窗内 pelvis heading 累积变化 > HeadingDeltaThreshold
    /// 3. DeltaQ 累积 yaw 增量 > DeltaQYawThreshold
    /// 4. （辅助）IsWalking 用于确认仍在步行
    ///
    /// 转弯结束后进入 ReacquiringPd，等 ProgressionDirEstimator 调用 ConfirmStraight() 退出。
    /// </summary>
    public sealed class MotionContextDetector
    {
        // ── 可调参数 ──────────────────────────────────────────────────────────
        public float YawRateThreshold_rads { get; set; } = 0.26f; // ~15°/s
        public float HeadingDeltaThreshold { get; set; } = 0.17f; // ~10° 累积
        public float DeltaQYawThreshold    { get; set; } = 0.17f; // rad
        public int   TurningConfirmFrames  { get; set; } = 10;    // 持续帧数才判 Turning
        public int   StraightConfirmFrames { get; set; } = 20;    // 持续帧数才退出 Turning
        public int   HeadingWindowFrames   { get; set; } = 20;    // 短窗大小

        // ── 内部状态 ──────────────────────────────────────────────────────────
        private ContextState         _state            = ContextState.Straight;
        private int                  _turningFrames    = 0;
        private int                  _straightFrames   = 0;
        private float                _deltaQYawAccum   = 0f;
        private readonly Queue<float> _headingWindow   = new();
        private float                _headingWindowSum  = 0f;
        private float                _lastPelvisYaw    = float.NaN;

        public MotionContext Detect(ValidFrame frame, GaitEvent gait)
        {
            float pelvisYawRate = frame.Pelvis.HasRateOfTurn
                ? MathF.Abs(frame.Pelvis.RateOfTurn.Y)   // Y = vertical axis in world frame
                : 0f;

            // 累积 deltaQ yaw
            if (frame.Pelvis.HasDeltaQ)
                _deltaQYawAccum += ExtractYawFromDeltaQ(frame.Pelvis.DeltaQ);

            // 短窗 heading 变化
            float pelvisYaw = frame.Pelvis.HasQuaternion
                ? ExtractYaw(frame.Pelvis.Quaternion)
                : float.NaN;

            float headingDelta = 0f;
            if (!float.IsNaN(pelvisYaw) && !float.IsNaN(_lastPelvisYaw))
            {
                float delta = NormalizeAngle(pelvisYaw - _lastPelvisYaw);
                _headingWindow.Enqueue(delta);
                _headingWindowSum += MathF.Abs(delta);
                if (_headingWindow.Count > HeadingWindowFrames)
                    _headingWindowSum -= MathF.Abs(_headingWindow.Dequeue());
                headingDelta = _headingWindowSum;
            }
            _lastPelvisYaw = pelvisYaw;

            bool turningSignal =
                pelvisYawRate > YawRateThreshold_rads ||
                headingDelta  > HeadingDeltaThreshold ||
                MathF.Abs(_deltaQYawAccum) > DeltaQYawThreshold;

            return _state switch
            {
                ContextState.Straight      => HandleStraight(turningSignal),
                ContextState.Turning       => HandleTurning(turningSignal),
                ContextState.ReacquiringPd => HandleReacquiring(turningSignal),
                _                          => new MotionContext { State = _state, Confidence = 1f }
            };
        }

        /// <summary>由 ProgressionDirEstimator 调用，PD 稳定后退出 ReacquiringPd。</summary>
        public void ConfirmStraight()
        {
            if (_state == ContextState.ReacquiringPd)
                _state = ContextState.Straight;
        }

        public void Reset()
        {
            _state            = ContextState.Straight;
            _turningFrames    = 0;
            _straightFrames   = 0;
            _deltaQYawAccum   = 0f;
            _headingWindow.Clear();
            _headingWindowSum = 0f;
            _lastPelvisYaw    = float.NaN;
        }

        // ── 状态处理 ──────────────────────────────────────────────────────────

        private MotionContext HandleStraight(bool turningSignal)
        {
            if (turningSignal)
            {
                _turningFrames++;
                if (_turningFrames >= TurningConfirmFrames)
                {
                    _state = ContextState.Turning;
                    _turningFrames  = 0;
                    _straightFrames = 0;
                    _deltaQYawAccum = 0f;
                    return new MotionContext { State = ContextState.Turning, Confidence = 1f };
                }
                float conf = 1f - (float)_turningFrames / TurningConfirmFrames;
                return new MotionContext { State = ContextState.Straight, Confidence = conf };
            }
            // 直行时重置累积量，防止 deltaQYaw 随时间无限积累后永久触发 turningSignal
            _turningFrames  = 0;
            _deltaQYawAccum = 0f;
            return new MotionContext { State = ContextState.Straight, Confidence = 1f };
        }

        private MotionContext HandleTurning(bool turningSignal)
        {
            if (!turningSignal)
            {
                _straightFrames++;
                if (_straightFrames >= StraightConfirmFrames)
                {
                    _state = ContextState.ReacquiringPd;
                    _straightFrames = 0;
                    _deltaQYawAccum = 0f;
                    return new MotionContext { State = ContextState.ReacquiringPd, Confidence = 0f };
                }
            }
            else
            {
                _straightFrames = 0;
            }
            return new MotionContext { State = ContextState.Turning, Confidence = 1f };
        }

        private MotionContext HandleReacquiring(bool turningSignal)
        {
            if (turningSignal)
            {
                // 再次转弯
                _state = ContextState.Turning;
                _straightFrames = 0;
            }
            return new MotionContext { State = _state, Confidence = 0f };
        }

        // ── 工具方法 ──────────────────────────────────────────────────────────

        private static float ExtractYaw(System.Numerics.Quaternion q)
            => MathF.Atan2(2f * (q.W * q.Z + q.X * q.Y),
                           1f - 2f * (q.Y * q.Y + q.Z * q.Z));

        private static float ExtractYawFromDeltaQ(System.Numerics.Quaternion dq)
            => 2f * MathF.Atan2(dq.Z, dq.W);  // 近似：小角度 deltaQ 的 yaw 增量

        private static float NormalizeAngle(float rad)
        {
            while (rad >  MathF.PI) rad -= 2f * MathF.PI;
            while (rad < -MathF.PI) rad += 2f * MathF.PI;
            return rad;
        }
    }
}
