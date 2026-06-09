// IMUMoCap/Pipeline/MotionContextDetector.cs
using System;
using IMUMoCap.Pipeline.Models;

namespace IMUMoCap.Pipeline
{
    /// <summary>
    /// 识别运动上下文：Straight / Turning / ReacquiringPd。
    ///
    /// 两路信号 OR，持续 TurningConfirmFrames 帧满足才切换到 Turning：
    /// 1. Pelvis world-frame yaw rate 绝对值 > YawRateThreshold（主信号）
    /// 2. DeltaQ 累积 yaw 增量 > DeltaQYawThreshold
    ///
    /// 转弯结束后进入 ReacquiringPd，等 ProgressionDirEstimator 调用 ConfirmStraight() 退出。
    /// </summary>
    public sealed class MotionContextDetector
    {
        // ── 可调参数 ──────────────────────────────────────────────────────────
        public float YawRateThreshold_rads { get; set; } = 0.70f; // ~40°/s (raised from 0.26: normal gait pelvic yaw peaks ~0.5 rad/s)
        public float DeltaQYawThreshold { get; set; } = 0.17f; // rad
        public int TurningConfirmFrames { get; set; } = 10; // 持续帧数才判 Turning
        public int StraightConfirmFrames { get; set; } = 10; // 持续帧数才退出 Turning
        // Prevents gait-sway yaw spikes from immediately bouncing ReacquiringPd back to Turning.
        public int ReacqTurningConfirmFrames { get; set; } = 10;

        // ── 内部状态 ──────────────────────────────────────────────────────────
        private ContextState _state = ContextState.Straight;
        private int _turningFrames = 0;
        private int _straightFrames = 0;
        private int _reacqTurningFrames = 0;
        private float _deltaQYawAccum = 0f;

        public MotionContext Detect(ValidFrame frame, GaitEvent gait, CalibrationProfile? calibration = null)
        {
            float pelvisYawRate = (frame.Pelvis.HasRateOfTurn && frame.Pelvis.HasQuaternion)
                ? MathF.Abs(System.Numerics.Vector3.Transform(frame.Pelvis.RateOfTurn, frame.Pelvis.Quaternion).Z)
                : 0f;

            // 累积 deltaQ yaw
            if (frame.Pelvis.HasDeltaQ)
                _deltaQYawAccum += ExtractYawFromDeltaQ(frame.Pelvis.DeltaQ);

            bool turningSignal =
                pelvisYawRate > YawRateThreshold_rads ||
                MathF.Abs(_deltaQYawAccum) > DeltaQYawThreshold;

            var ctx = _state switch
            {
                ContextState.Straight => HandleStraight(turningSignal),
                ContextState.Turning => HandleTurning(turningSignal),
                ContextState.ReacquiringPd => HandleReacquiring(turningSignal),
                _ => new MotionContext { State = _state, Confidence = 1f }
            };

            // IsWalking as auxiliary vote: halve confidence when not walking (doesn't block state transitions)
            if (!gait.IsWalking && ctx.State == ContextState.Straight)
                return new MotionContext { State = ctx.State, Confidence = ctx.Confidence * 0.5f };
            return ctx;
        }

        /// <summary>由 ProgressionDirEstimator 调用，PD 稳定后退出 ReacquiringPd。</summary>
        public void ConfirmStraight()
        {
            if (_state == ContextState.ReacquiringPd)
                _state = ContextState.Straight;
        }

        public void Reset()
        {
            _state = ContextState.Straight;
            _turningFrames = 0;
            _straightFrames = 0;
            _reacqTurningFrames = 0;
            _deltaQYawAccum = 0f;
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
                    _turningFrames = 0;
                    _straightFrames = 0;
                    _deltaQYawAccum = 0f;
                    return new MotionContext { State = ContextState.Turning, Confidence = 1f };
                }
                float conf = 1f - (float)_turningFrames / TurningConfirmFrames;
                return new MotionContext { State = ContextState.Straight, Confidence = conf };
            }
            // 直行时重置累积量，防止 deltaQYaw 随时间无限积累后永久触发 turningSignal
            _turningFrames = 0;
            _deltaQYawAccum = 0f;
            return new MotionContext { State = ContextState.Straight, Confidence = 1f };
        }

        private MotionContext HandleTurning(bool turningSignal)
        {
            // deltaQYawAccum was designed to detect turn onset from Straight.
            // Inside Turning it would just accumulate the full turn angle and permanently
            // block exit. Reset it each frame so only yawRate gates exit.
            _deltaQYawAccum = 0f;

            if (!turningSignal)
            {
                _straightFrames++;
                if (_straightFrames >= StraightConfirmFrames)
                {
                    _state = ContextState.ReacquiringPd;
                    _straightFrames = 0;
                    _reacqTurningFrames = 0;
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
            _deltaQYawAccum = 0f;

            if (turningSignal)
            {
                _reacqTurningFrames++;
                if (_reacqTurningFrames >= ReacqTurningConfirmFrames)
                {
                    // Confirmed re-turn: transition back to Turning.
                    _state = ContextState.Turning;
                    _straightFrames = 0;
                    _reacqTurningFrames = 0;
                }
            }
            else
            {
                _reacqTurningFrames = 0;
            }
            return new MotionContext { State = _state, Confidence = 0f };
        }

        // ── 工具方法 ──────────────────────────────────────────────────────────

        private static float ExtractYawFromDeltaQ(System.Numerics.Quaternion dq)
            => 2f * MathF.Atan2(dq.Z, dq.W);  // 近似：小角度 deltaQ 的 yaw 增量
    }
}
