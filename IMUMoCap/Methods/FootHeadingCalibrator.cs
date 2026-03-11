using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using XDA;

namespace IMUMoCap.Methods
{
    /// <summary>
    /// Maintains per-device orientation q_WS by integrating orientationIncrement at 100Hz,
    /// and performs a static standing calibration to estimate yaw/heading offsets for feet.
    /// World frame assumed ENU (Z up). Heading computed from projected forward vector.
    /// </summary>
    public sealed class FootHeadingCalibrator
    {
        private readonly string _pelvisId, _leftFootId, _rightFootId;

        // Per device orientation q_WS (integrated)
        private readonly Dictionary<string, Quaternion> _qWs = new();

        // Forward axis per device (sensor frame)
        private readonly Dictionary<string, Vector3> _forwardAxis = new();

        // ---- Calibration state ----
        private bool _calibrating;
        private long _startPacketId;

        private float _delaySec = 1.0f;        // 延迟开始
        private float _collectSec = 6.0f;      // 统计窗口时长（不含 delay）
        private bool _requireStill = true;     // 是否要求静止才计入样本

        // Static thresholds for "still" gating (tuned for foot/pelvis in quiet standing)
        private float _stillGyroTh = 0.15f;    // rad/s
        private float _stillAccDevTh = 0.35f;  // m/s^2 deviation from g

        private const float Fs = 100f;
        private float _gEst = 9.81f;           // gravity magnitude estimate

        // Accumulators (circular mean)
        private readonly AngleAccumulator _pelvisAcc = new();
        private readonly AngleAccumulator _leftAcc = new();
        private readonly AngleAccumulator _rightAcc = new();

        public FootHeadingCalibrator(string pelvisId, string leftFootId, string rightFootId)
        {
            _pelvisId = pelvisId;
            _leftFootId = leftFootId;
            _rightFootId = rightFootId;

            _qWs[_pelvisId] = Quaternion.Identity;
            _qWs[_leftFootId] = Quaternion.Identity;
            _qWs[_rightFootId] = Quaternion.Identity;

            // You confirmed:
            // foot: +X forward
            // pelvis: -Z forward
            _forwardAxis[_leftFootId] = new Vector3(1, 0, 0);
            _forwardAxis[_rightFootId] = new Vector3(1, 0, 0);
            _forwardAxis[_pelvisId] = new Vector3(0, 0, -1);
        }

        /// <summary>
        /// Start a standing calibration with delayed collection and automatic failure checks.
        /// </summary>
        public void BeginStandingCalibration(
            long startPacketId,
            float delaySec = 1.0f,
            float collectSec = 6.0f,
            bool requireStill = true,
            float stillGyroThRad = 0.15f,
            float stillAccDevTh = 0.35f
        )
        {
            _calibrating = true;
            _startPacketId = startPacketId;

            _delaySec = MathF.Max(0f, delaySec);
            _collectSec = MathF.Max(1.0f, collectSec);
            _requireStill = requireStill;

            _stillGyroTh = stillGyroThRad;
            _stillAccDevTh = stillAccDevTh;

            _qWs[_pelvisId] = Quaternion.Identity;
            _qWs[_leftFootId] = Quaternion.Identity;
            _qWs[_rightFootId] = Quaternion.Identity;

            _pelvisAcc.Reset();
            _leftAcc.Reset();
            _rightAcc.Reset();

            _gEst = 9.81f;
        }

        /// <summary>
        /// Feed per-device samples. Returns an outcome only once when calibration finishes (success or failure).
        /// </summary>
        public ImuCalibrationOutcome? OnSample(long packetId, string deviceId, XsSdiData sdi, XsCalibratedData cal, bool isUseConjugate)
        {
            // Update q(t) by orientation increment
            Quaternion dq = ToNumericsQuaternion(sdi.orientationIncrement());


            Quaternion qOld = _qWs.TryGetValue(deviceId, out var existing) ? existing : Quaternion.Identity;
            Quaternion qNew = Utils.Normalize(Quaternion.Multiply(qOld, dq));
            // Optional: quaternion sign continuity (avoid q/-q flips)
            if (Quaternion.Dot(qNew, qOld) < 0)
                qNew = new Quaternion(-qNew.X, -qNew.Y, -qNew.Z, -qNew.W);

            _qWs[deviceId] = qNew;

            if (!_calibrating) return null;

            float t = PacketIdToTimeSec(packetId, _startPacketId);

            // Delay phase: do nothing, let subject settle
            if (t < _delaySec) return null;

            // Collection window time (relative to delayed start)
            float tc = t - _delaySec;

            // If beyond collection time, finalize (but only when we have enough samples)
            if (tc > _collectSec)
            {
                _calibrating = false;
                return FinalizeCalibration(packetId);
            }

            // Still gating (recommended)
            if (_requireStill)
            {
                float gyroMag = (float)cal.m_gyr.cartesianLength();     // rad/s (you confirmed)
                float accMag = (float)cal.m_acc.cartesianLength();      // m/s^2 (you confirmed)

                // Update g estimate slowly (only when gyro is low-ish)
                if (gyroMag < 0.25f)
                    _gEst = Lerp(_gEst, accMag, 0.01f);

                float accDev = MathF.Abs(accMag - _gEst);

                if (gyroMag > _stillGyroTh || accDev > _stillAccDevTh)
                {
                    // Not still -> skip this sample
                    return null;
                }
            }

            // Compute heading for this device
            if (!_forwardAxis.TryGetValue(deviceId, out var fwd))
                fwd = new Vector3(1, 0, 0);

            float heading = Utils.HeadingENU_Consistent(qNew, fwd, isUseConjugate);

            // Accumulate
            if (deviceId == _pelvisId) _pelvisAcc.Add(heading);
            else if (deviceId == _leftFootId) _leftAcc.Add(heading);
            else if (deviceId == _rightFootId) _rightAcc.Add(heading);

            return null;
        }

        private ImuCalibrationOutcome FinalizeCalibration(long packetId)
        {
            // Basic sample count check
            int n = Math.Min(_pelvisAcc.Count, Math.Min(_leftAcc.Count, _rightAcc.Count));
            if (n < 150) // ~1.5s effective still samples @100Hz
            {
                return new ImuCalibrationOutcome
                {
                    Success = false,
                    Reason = $"Calibration failed: not enough still samples (used={n}). Stand still longer / loosen thresholds.",
                    SamplesUsed = n
                };
            }

            float Hp = _pelvisAcc.MeanAngle();
            float Hl = _leftAcc.MeanAngle();
            float Hr = _rightAcc.MeanAngle();

            float deltaL = Utils.WrapPi(Hp - Hl);
            float deltaR = Utils.WrapPi(Hp - Hr);

            // Sanity: apply deltas and compute residual errors
            float errL = Utils.WrapPi((Hl + deltaL) - Hp);
            float errR = Utils.WrapPi((Hr + deltaR) - Hp);

            float errLdeg = Rad2Deg(errL);
            float errRdeg = Rad2Deg(errR);

            // This error should be near 0 if math is consistent.
            // If not near 0, something is inconsistent in your quaternion direction/order.
            if (MathF.Abs(errLdeg) > 2f || MathF.Abs(errRdeg) > 2f)
            {
                return new ImuCalibrationOutcome
                {
                    Success = false,
                    Reason = $"Calibration failed: internal inconsistency (errL={errLdeg:F1}°, errR={errRdeg:F1}°). Check quaternion order / multiply order / conjugate usage.",
                    ErrLeftDeg = errLdeg,
                    ErrRightDeg = errRdeg,
                    SamplesUsed = n
                };
            }

            // Practical quality gate: even if internal consistency is OK,
            // we want feet aligned with pelvis in the calibration posture.
            // Here we check raw-vs-pelvis offsets magnitude (i.e., -delta) is not absurdly varying between feet.
            // More importantly: you can enforce that corrected headings are close to pelvis by checking during the same window,
            // but since corr==pelvis by construction in mean, we instead gate delta difference.
            float deltaDiffDeg = MathF.Abs(Rad2Deg(Utils.WrapPi(deltaL - deltaR)));

            // If left/right delta differ too much, likely the posture was not symmetric / sensors moved / not aligned.
            if (deltaDiffDeg > 25f)
            {
                return new ImuCalibrationOutcome
                {
                    Success = false,
                    Reason = $"Calibration failed: left/right offsets differ too much (|deltaL-deltaR|={deltaDiffDeg:F1}°). Re-wear or re-stand with both feet pointing forward.",
                    SamplesUsed = n
                };
            }

            // Optional gate: delta too close to +/-180 can indicate forward axis mismatch
            // (not always wrong, but often suspicious). We don't hard-fail, just warn.
            // You can decide to fail if you want.
            // if (MathF.Abs(MathF.Abs(Rad2Deg(deltaL)) - 180f) < 10f) ...

            var result = new ImuCalibrationResult
            {
                DeltaLeftRad = deltaL,
                DeltaRightRad = deltaR,
                PelvisHeadingRad = Hp,
                LeftRawHeadingRad = Hl,
                RightRawHeadingRad = Hr,
                SamplesUsed = n
            };

            return new ImuCalibrationOutcome
            {
                Success = true,
                Reason = "OK",
                Result = result,
                ErrLeftDeg = errLdeg,
                ErrRightDeg = errRdeg,
                SamplesUsed = n
            };
        }

        // ---------- math helpers ----------
        private static float PacketIdToTimeSec(long packetId, long startPacketId)
            => (packetId - startPacketId) / Fs;

        private static float Lerp(float a, float b, float t) => a + (b - a) * t;
        private static float Rad2Deg(float r) => r * 180f / MathF.PI;

        // TODO: replace with your SDK mapping
        private static Quaternion ToNumericsQuaternion(XsQuaternion inc)
            => new Quaternion((float)inc.x(), (float)inc.y(), (float)inc.z(), (float)inc.w());
    }
    public enum ImuRole { Pelvis, Left, Right }

    public sealed class ImuCalibrationOutcome
    {
        public bool Success { get; init; }
        public string Reason { get; init; } = "";

        public ImuCalibrationResult? Result { get; init; }

        // Optional extra debug
        public float ErrLeftDeg { get; init; }
        public float ErrRightDeg { get; init; }
        public int SamplesUsed { get; init; }
    }

    public sealed class ImuCalibrationResult
    {
        public float DeltaLeftRad { get; init; }
        public float DeltaRightRad { get; init; }
        public float PelvisHeadingRad { get; init; }
        public float LeftRawHeadingRad { get; init; }
        public float RightRawHeadingRad { get; init; }
        public int SamplesUsed { get; init; }
        public override string ToString()
        {
            return $"DeltaLeftRad: {DeltaLeftRad * 180 / MathF.PI}, DeltaRightRad: {DeltaRightRad * 180 / MathF.PI},{Environment.NewLine}PelvisHeadingRad: {PelvisHeadingRad * 180 / MathF.PI}, LeftRawHeadingRad: {LeftRawHeadingRad * 180 / MathF.PI}, RightRawHeadingRad:{RightRawHeadingRad * 180 / MathF.PI}, SamplesUsed: {SamplesUsed} ";
        }
    }

    public sealed class AngleAccumulator
    {
        private double _sumSin;
        private double _sumCos;
        public int Count { get; private set; }

        public void Reset()
        {
            _sumSin = 0;
            _sumCos = 0;
            Count = 0;
        }

        public void Add(float angleRad)
        {
            _sumSin += Math.Sin(angleRad);
            _sumCos += Math.Cos(angleRad);
            Count++;
        }

        public float MeanAngle()
        {
            if (Count == 0) return 0f;
            return (float)Math.Atan2(_sumSin / Count, _sumCos / Count);
        }
    }
}
