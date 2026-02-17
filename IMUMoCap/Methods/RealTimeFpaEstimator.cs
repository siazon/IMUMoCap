using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using XDA;

namespace IMUMoCap.Methods
{
    public sealed class RealTimeFpaEstimator
    {
        private readonly string _pelvisId, _leftId, _rightId;

        // per-device orientation q_WS (integrated)
        private readonly Dictionary<string, Quaternion> _qWs = new();

        // forward axis in sensor frame
        private readonly Dictionary<string, Vector3> _forwardAxis = new();

        // calibration offsets (rad)
        private readonly float _deltaLeft, _deltaRight;

        // stance window accumulators (per foot)
        private readonly StepWindow _leftStep = new(windowFrames: 15);   // 150ms @ 100Hz
        private readonly StepWindow _rightStep = new(windowFrames: 15);

        // last stance state to detect transitions
        private bool _leftWasStance = false;
        private bool _rightWasStance = false;

        public event Action<StepFpaResult>? OnStepFpa;  // fired when a step value is finalized (TO)

        public RealTimeFpaEstimator(string pelvisId, string leftId, string rightId,
                                    float deltaLeftRad, float deltaRightRad)
        {
            _pelvisId = pelvisId; _leftId = leftId; _rightId = rightId;
            _deltaLeft = deltaLeftRad; _deltaRight = deltaRightRad;

            _qWs[_pelvisId] = Quaternion.Identity;
            _qWs[_leftId] = Quaternion.Identity;
            _qWs[_rightId] = Quaternion.Identity;

            // You confirmed:
            // foot: +X forward
            // pelvis: -Z forward
            _forwardAxis[_leftId] = new Vector3(1, 0, 0);
            _forwardAxis[_rightId] = new Vector3(1, 0, 0);
            _forwardAxis[_pelvisId] = new Vector3(0, 0, -1);
        }

        /// <summary>
        /// Process one synchronized packet (all 3 devices same packetId).
        /// stanceLeft/stanceRight supplied by your existing detector.
        /// Returns instantaneous FPA (for UI) when available.
        /// </summary>
        public (float? fpaLeftRad, float? fpaRightRad) ProcessBundle(
            PacketBundle b,
            bool stanceLeft,
            bool stanceRight)
        {
            // 1) update qWs for each device
            UpdateOrientation(b.Pelvis!.Value.deviceId, b.Pelvis!.Value.sdi);
            UpdateOrientation(b.Left!.Value.deviceId, b.Left!.Value.sdi);
            UpdateOrientation(b.Right!.Value.deviceId, b.Right!.Value.sdi);

            // 2) compute headings (ENU Z-up projection)
            float pelvisHeading = HeadingENU(_qWs[_pelvisId], _forwardAxis[_pelvisId]);

            float leftRaw = HeadingENU(_qWs[_leftId], _forwardAxis[_leftId]);
            float rightRaw = HeadingENU(_qWs[_rightId], _forwardAxis[_rightId]);

            float leftCorr = WrapPi(leftRaw + _deltaLeft);
            float rightCorr = WrapPi(rightRaw + _deltaRight);

            float fpaLeft = WrapPi(leftCorr - pelvisHeading);
            float fpaRight = WrapPi(rightCorr - pelvisHeading);

            // 3) step finalization logic (stance mid window)
            HandleFootStep(ImuRole.LeftFoot, b.PacketId, stanceLeft, fpaLeft, _leftStep, ref _leftWasStance);
            HandleFootStep(ImuRole.RightFoot, b.PacketId, stanceRight, fpaRight, _rightStep, ref _rightWasStance);

            // 4) instantaneous values for UI (optional: only when stance)
            float? uiLeft = stanceLeft ? fpaLeft : null;
            float? uiRight = stanceRight ? fpaRight : null;

            return (uiLeft, uiRight);
        }

        private void HandleFootStep(ImuRole role, long packetId, bool isStance, float fpaRad,
                                    StepWindow window, ref bool wasStance)
        {
            if (isStance)
            {
                // accumulate within stance; window will keep only last N frames
                window.Add(fpaRad);
            }

            // transition: stance -> swing means TO; finalize a per-step value
            if (wasStance && !isStance)
            {
                if (window.Count >= window.WindowFrames)
                {
                    float stepFpa = window.MeanAngle();
                    OnStepFpa?.Invoke(new StepFpaResult
                    {
                        Role = role,
                        PacketId = packetId,
                        FpaRad = stepFpa
                    });
                }
                window.Reset();
            }

            // transition: swing -> stance means HS; start collecting anew (reset)
            if (!wasStance && isStance)
            {
                window.Reset();
            }

            wasStance = isStance;
        }

        private void UpdateOrientation(string deviceId, XsSdiData sdi)
        {
            Quaternion dq = ToNumericsQuaternion(sdi.orientationIncrement());

            // Common integration: q(t+dt) = normalize(q(t) * dq)
            Quaternion q = _qWs[deviceId];
            q = Normalize(Quaternion.Multiply(q, dq));
            _qWs[deviceId] = q;
        }

        private static float HeadingENU(Quaternion qWs, Vector3 sensorForwardAxis)
        {
            Vector3 fw = Vector3.Transform(sensorForwardAxis, qWs);
            fw.Z = 0;
            if (fw.LengthSquared() < 1e-8f) return 0f;
            fw = Vector3.Normalize(fw);
            return MathF.Atan2(fw.Y, fw.X);
        }

        private static Quaternion Normalize(Quaternion q)
        {
            float n = MathF.Sqrt(q.X * q.X + q.Y * q.Y + q.Z * q.Z + q.W * q.W);
            if (n < 1e-12f) return Quaternion.Identity;
            float inv = 1f / n;
            return new Quaternion(q.X * inv, q.Y * inv, q.Z * inv, q.W * inv);
        }

        private static float WrapPi(float a)
        {
            while (a > MathF.PI) a -= 2f * MathF.PI;
            while (a < -MathF.PI) a += 2f * MathF.PI;
            return a;
        }

        private static Quaternion ToNumericsQuaternion(XsQuaternion inc)
        {
            // Adjust here if your SDK is WXYZ etc.
            return new Quaternion((float)inc.x(), (float)inc.y(), (float)inc.z(), (float)inc.w());
        }
    }

    public sealed class StepWindow
    {
        private double _sumSin, _sumCos;
        private readonly Queue<float> _buf = new();

        public int WindowFrames { get; }
        public int Count => _buf.Count;

        public StepWindow(int windowFrames) => WindowFrames = windowFrames;

        public void Reset()
        {
            _buf.Clear();
            _sumSin = 0; _sumCos = 0;
        }

        public void Add(float angleRad)
        {
            _buf.Enqueue(angleRad);
            _sumSin += Math.Sin(angleRad);
            _sumCos += Math.Cos(angleRad);

            while (_buf.Count > WindowFrames)
            {
                float old = _buf.Dequeue();
                _sumSin -= Math.Sin(old);
                _sumCos -= Math.Cos(old);
            }
        }

        public float MeanAngle()
        {
            if (_buf.Count == 0) return 0f;
            return (float)Math.Atan2(_sumSin / _buf.Count, _sumCos / _buf.Count);
        }
    }

    public sealed class StepFpaResult
    {
        public ImuRole Role { get; init; }
        public long PacketId { get; init; }
        public float FpaRad { get; init; }
    }

}
