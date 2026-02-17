using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using XDA;

namespace IMUMoCap.Methods
{
    public enum ImuRole { Pelvis, LeftFoot, RightFoot }

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

    /// <summary>
    /// Maintains per-device orientation q_WS by integrating orientationIncrement at 100Hz,
    /// and performs a static standing calibration to estimate yaw/heading offsets for feet.
    /// World frame assumed ENU (Z up). Heading computed from projected forward vector.
    /// </summary>
    public sealed class FootHeadingCalibrator
    {
        // --- Configure these 3 IDs in your app ---
        private readonly string _pelvisId;
        private readonly string _leftFootId;
        private readonly string _rightFootId;


        // 100 Hz
        private const float Fs = 100f;

        // Per device orientation state: q_WS (sensor frame rotated into world frame)
        private readonly Dictionary<string, Quaternion> _qWs = new();

        // Calibration window buffers
        private bool _calibrating;
        private long _calibStartPacketId;
        private float _calibDurationSec;

        private readonly AngleAccumulator _pelvisHeadingAcc = new();
        private readonly AngleAccumulator _leftHeadingAcc = new();
        private readonly AngleAccumulator _rightHeadingAcc = new();
        private readonly Dictionary<string, Vector3> _forwardAxis = new();


        public FootHeadingCalibrator(string pelvisId, string leftFootId, string rightFootId)
        {
            _pelvisId = pelvisId;
            _leftFootId = leftFootId;
            _rightFootId = rightFootId;

            // Initialize orientations to identity (world == sensor at t0).
            _qWs[_pelvisId] = Quaternion.Identity;
            _qWs[_leftFootId] = Quaternion.Identity;
            _qWs[_rightFootId] = Quaternion.Identity;

            // Forward axis in SENSOR frame for each device:
            _forwardAxis[_leftFootId] = new Vector3(1, 0, 0);   // +X forward
            _forwardAxis[_rightFootId] = new Vector3(1, 0, 0);   // +X forward
            _forwardAxis[_pelvisId] = new Vector3(0, 0, -1);  // -Z forward (your腰部安装)
        }

        /// <summary>
        /// Call this once you instruct the subject to stand still and align feet forward.
        /// durationSec: e.g., 6 seconds.
        /// </summary>
        public void BeginStandingCalibration(long startPacketId, float durationSec = 6f)
        {
            _calibrating = true;
            _calibStartPacketId = startPacketId;
            _calibDurationSec = durationSec;

            _pelvisHeadingAcc.Reset();
            _leftHeadingAcc.Reset();
            _rightHeadingAcc.Reset();
        }

        /// <summary>
        /// Feed samples from your 100Hz callback.
        /// Expect: all 3 devices share same packetId for a given time.
        /// </summary>
        public ImuCalibrationResult? OnSample(
            long packetId,
            string deviceId,
            XsSdiData sdi,
            XsCalibratedData cal // not used in this basic calibration, kept for extensibility
        )
        {
            // 1) Update q_WS by integrating orientationIncrement
            // NOTE: orientationIncrement must be converted into System.Numerics.Quaternion (X,Y,Z,W).
            Quaternion dq = ToNumericsQuaternion(sdi.orientationIncrement());

            // Common integration: q(t+dt) = normalize(q(t) * dq)
            Quaternion q = _qWs.TryGetValue(deviceId, out var existing) ? existing : Quaternion.Identity;
            q = Normalize(Quaternion.Multiply(q, dq));
            _qWs[deviceId] = q;

            // 2) If calibrating, accumulate heading for this device
            if (_calibrating)
            {
                float t = PacketIdToTimeSec(packetId, _calibStartPacketId);

                // Only accumulate within [0, duration]
                if (t >= 0 && t <= _calibDurationSec)
                {
                    Vector3 fwd = _forwardAxis.TryGetValue(deviceId, out var v) ? v : new Vector3(1, 0, 0);
                    float heading = ComputeHeadingRadFromQuaternionENU(q, fwd);

                    if (deviceId == _pelvisId) _pelvisHeadingAcc.Add(heading);
                    else if (deviceId == _leftFootId) _leftHeadingAcc.Add(heading);
                    else if (deviceId == _rightFootId) _rightHeadingAcc.Add(heading);
                }

                // 3) Finish when time exceeds duration AND we have enough samples from all three
                if (t > _calibDurationSec &&
                    _pelvisHeadingAcc.Count > 50 &&
                    _leftHeadingAcc.Count > 50 &&
                    _rightHeadingAcc.Count > 50)
                {
                    _calibrating = false;

                    float Hp = _pelvisHeadingAcc.MeanAngle();
                    float Hl = _leftHeadingAcc.MeanAngle();
                    float Hr = _rightHeadingAcc.MeanAngle();

                    // Offsets so that corrected foot heading aligns to pelvis heading in the neutral stance.
                    float deltaLeft = WrapPi(Hp - Hl);
                    float deltaRight = WrapPi(Hp - Hr);

                    return new ImuCalibrationResult
                    {
                        DeltaLeftRad = deltaLeft,
                        DeltaRightRad = deltaRight,
                        PelvisHeadingRad = Hp,
                        LeftRawHeadingRad = Hl,
                        RightRawHeadingRad = Hr,
                        SamplesUsed = Math.Min(_pelvisHeadingAcc.Count, Math.Min(_leftHeadingAcc.Count, _rightHeadingAcc.Count))
                    };
                }
            }

            return null;
        }

        /// <summary>
        /// Convert packetId to seconds with 100Hz.
        /// If packetId is incremental and can wrap, uint subtraction still works with wrap if window is short.
        /// </summary>
        private  float PacketIdToTimeSec(long packetId, long startPacketId)
        {
            long diff = packetId - startPacketId; // handles wrap-around in uint arithmetic for short intervals
            return diff / Fs;
        }

        /// <summary>
        /// Compute heading from sensor forward axis rotated to world frame, projected to horizontal plane.
        /// World ENU: Z is up, horizontal plane is XY.
        /// </summary>
        private static float ComputeHeadingRadFromQuaternionENU(Quaternion qWs, Vector3 sensorForwardAxis)
        {
            // Rotate the chosen forward axis from SENSOR frame into WORLD frame
            Vector3 fw = Vector3.Transform(sensorForwardAxis, qWs);

            // Project to horizontal plane (ENU => Z up)
            fw.Z = 0;

            if (fw.LengthSquared() < 1e-8f)
                return 0f;

            fw = Vector3.Normalize(fw);

            // Heading in [-pi, pi]
            return MathF.Atan2(fw.Y, fw.X);
        }


        private Quaternion Normalize(Quaternion q)
        {
            float n = MathF.Sqrt(q.X * q.X + q.Y * q.Y + q.Z * q.Z + q.W * q.W);
            if (n < 1e-12f) return Quaternion.Identity;
            float inv = 1f / n;
            return new Quaternion(q.X * inv, q.Y * inv, q.Z * inv, q.W * inv);
        }

        private  float WrapPi(float a)
        {
            while (a > MathF.PI) a -= 2f * MathF.PI;
            while (a < -MathF.PI) a += 2f * MathF.PI;
            return a;
        }

        /// <summary>
        /// Implement this conversion based on your SDK struct layout.
        /// System.Numerics.Quaternion expects (X,Y,Z,W).
        /// If your SDK gives (w,x,y,z), reorder here.
        /// </summary>
        private  Quaternion ToNumericsQuaternion(XsQuaternion inc)
        {
            // Example assumes inc has fields X,Y,Z,W already.
            // If your SDK provides W,X,Y,Z: return new Quaternion(inc.X, inc.Y, inc.Z, inc.W) accordingly.
            return new Quaternion((float)inc.x(), (float)inc.y(), (float)inc.z(), (float)inc.w());
        }

        // Circular mean accumulator: mean angle robustly over wrap-around
        private sealed class AngleAccumulator
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
}
