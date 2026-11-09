// IMUMoCap/Pipeline/GaitEventDetector.cs
using System.Diagnostics;
using System.Numerics;
using IMUMoCap.Pipeline.Models;

namespace IMUMoCap.Pipeline
{
    /// <summary>
    /// 逐帧判定步态事件：stance/swing、stomp、isWalking。
    ///
    /// Stance 判定（三重条件）：
    ///   1. free acc 模长 &lt; FreeAccStanceThreshold（足部无明显线加速度）
    ///   2. 角速度 &lt; GyroThreshold（足部无明显旋转）
    ///   3. 足部 pitch 与校准参考差 &lt; FootPitchThreshold（防慢走时朝向接近但脚仍倾斜）
    ///   且持续 MinStanceFrames 帧以上防抖。
    ///
    /// Pitch 检查：q_rel = Inverse(calRef) * q_current，提取绕 X 轴旋转角；
    /// 校准参考为 null 或无四元数时跳过此条件。
    ///
    /// IsWalking：左右脚在 WalkingWindowFrames 内都至少有一次 stance 切换。
    /// </summary>
    public sealed class GaitEventDetector
    {
        // ── 可调参数 ──────────────────────────────────────────────────────────
        public float FreeAccStanceThreshold { get; set; } = 2.5f;   // m/s²，free acc 模长
        public float GyroThreshold          { get; set; } = 1.0f;   // rad/s
        public float FootPitchThreshold     { get; set; } = 0.35f;   // rad，~20°，与校准参考的重力方向偏转上限（10°对转弯后left foot过紧）
        public int   MinStanceFrames        { get; set; } = 5;
        public int   WalkingWindowFrames    { get; set; } = 300;     // 3s @100Hz

        // ── 内部状态 ──────────────────────────────────────────────────────────
        private int  _leftStanceCount   = 0;
        private int  _rightStanceCount  = 0;
        private bool _leftStancePrev    = false;
        private bool _rightStancePrev   = false;

        private int  _leftTransitions   = 0;   // stance↔swing 切换次数（滑动窗内）
        private int  _rightTransitions  = 0;
        private int  _walkingFrameCount = 0;
        private bool _isWalking         = false;

        public GaitEvent Detect(ValidFrame frame, CalibrationProfile? calibration)
        {
            bool leftStance  = IsStance(frame.LeftFoot,  calibration?.LeftFootRef);
            bool rightStance = IsStance(frame.RightFoot, calibration?.RightFootRef);

            // 防抖：需连续 MinStanceFrames 帧才确认
            _leftStanceCount  = leftStance  ? _leftStanceCount  + 1 : 0;
            _rightStanceCount = rightStance ? _rightStanceCount + 1 : 0;

            bool leftStanceConfirmed  = _leftStanceCount  >= MinStanceFrames;
            bool rightStanceConfirmed = _rightStanceCount >= MinStanceFrames;

            // 切换计数（用于 IsWalking 判定）
            if (leftStanceConfirmed  != _leftStancePrev)  { _leftTransitions++;  _leftStancePrev  = leftStanceConfirmed; }
            if (rightStanceConfirmed != _rightStancePrev) { _rightTransitions++; _rightStancePrev = rightStanceConfirmed; }

            _walkingFrameCount++;
            if (_walkingFrameCount >= WalkingWindowFrames)
            {
                // 3s 窗口内左右各至少有 2 次切换（一个完整 stance-swing 周期）
                _isWalking         = _leftTransitions >= 2 && _rightTransitions >= 2;
                _walkingFrameCount = 0;
                _leftTransitions   = 0;
                _rightTransitions  = 0;
            }

            return new GaitEvent
            {
                LeftStance  = leftStanceConfirmed,
                RightStance = rightStanceConfirmed,
                LeftSwing   = !leftStanceConfirmed,
                RightSwing  = !rightStanceConfirmed,
                IsWalking   = _isWalking,
            };
        }

        public void Reset()
        {
            _leftStanceCount = _rightStanceCount = 0;
            _leftStancePrev  = _rightStancePrev  = false;
            _leftTransitions = _rightTransitions = 0;
            _walkingFrameCount = 0;
            _isWalking       = false;
        }

        // ── 私有辅助 ──────────────────────────────────────────────────────────

        private bool IsStance(ImuSampleFrame f, Quaternion? calRef)
        {
            if (!f.HasFreeAcceleration || !f.HasRateOfTurn) return false;
            bool accOk  = f.FreeAcceleration.Length() < FreeAccStanceThreshold;
            bool gyroOk = f.RateOfTurn.Length()       < GyroThreshold;
            bool pitchOk = true;
            if (calRef.HasValue && f.HasQuaternion)
            {
                // Measure tilt as angle between gravity direction in sensor frame now vs. at
                // calibration. This is yaw-independent: a flat foot at any heading has the
                // same gravity vector in sensor frame. The old qRel pitch extraction broke
                // after turns because heading change coupled into the relative pitch angle.
                var gRef = Vector3.Transform(-Vector3.UnitZ, Quaternion.Inverse(calRef.Value));
                var gNow = Vector3.Transform(-Vector3.UnitZ, Quaternion.Inverse(f.Quaternion));
                float cosTilt = Math.Clamp(Vector3.Dot(gRef, gNow), -1f, 1f);
                pitchOk = MathF.Acos(cosTilt) < FootPitchThreshold;
            }
            return accOk && gyroOk && pitchOk;
        }

    }
}
