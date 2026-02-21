using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Reflection.Metadata;
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
        private readonly StepDebugWindow _leftStep = new(windowFrames: 15);   // 150ms @ 100Hz
        private readonly StepDebugWindow _rightStep = new(windowFrames: 15);

        // last stance state to detect transitions
        private bool _leftWasStance = false;
        private bool _rightWasStance = false;

        public event Action<StepFpaResult>? OnStepFpa;  // fired when a step value is finalized (TO)

        public readonly CircularMovingAverage _pelvisProgDir = new CircularMovingAverage(300); // 3秒@100Hz
        public bool EnableAntiJumpGate { get; set; } = false;  // 防突变总开关

        private readonly HeadingQualityGate _leftFootGate = new HeadingQualityGate(fs: 100f);
        private readonly HeadingQualityGate _rightFootGate = new HeadingQualityGate(fs: 100f);

        private const int _minFramesForStep = 8;              // stance里至少80ms有效样本再出step
        private const int _maxFootRejectStreakForAdd = 6;     // 短时拒绝仍允许用hold值入窗，避免窗口断裂


        private float? _pelvisHeadingPrev = null;
        private float? _pelvisHeadingUsedPrev = null;
        private long _pelvisPrevPacket = 0;
        private int _pelvisRejectStreak = 0;
        // ---- Magnetometer quality gate (pelvis) ----
        private float _pelvisMagBaseline = 0f;     // |mag| baseline, learned when still
        private int _pelvisMagBaseCount = 0;
        private const float _magLearnGyroTh = 0.25f;   // rad/s, learn baseline only when still-ish
        private const float _magRelBadTh = 0.20f;      // 20% deviation => likely disturbance (tune 0.15~0.30)

        // ---- Yaw-rate consistency gate (pelvis) ----
        private const float _yawRateRatioBad = 4.0f;   // heading-rate > gyro-rate*4 + margin => suspicious
        private const float _yawRateMargin = 0.5f;     // rad/s margin
        private const float _maxJumpDeg = 35f;         // your existing jump gate


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

            // 脚部门控稍微放宽，否则在磁扰/瞬态下容易导致窗口样本不足
            _leftFootGate.MaxJumpDeg = 35f;
            _leftFootGate.YawRateRatioBad = 5f;
            _leftFootGate.MagRelBadTh = 0.35f;

            _rightFootGate.MaxJumpDeg = 35f;
            _rightFootGate.YawRateRatioBad = 5f;
            _rightFootGate.MagRelBadTh = 0.35f;
        }

        /// <summary>
        /// Process one synchronized packet (all 3 devices same packetId).
        /// stanceLeft/stanceRight supplied by your existing detector.
        /// Returns instantaneous FPA (for UI) when available.
        /// </summary>
        public (float? fpaLeftRad, float? fpaRightRad, float? pelvisHeading) ProcessBundle(
            PacketBundle b,
            bool stanceLeft,
            bool stanceRight, bool isUseConjugate)
        {
            // 1) update qWs for each device
            UpdateOrientation(b.Pelvis!.Value.deviceId, b.Pelvis!.Value.sdi);
            UpdateOrientation(b.Left!.Value.deviceId, b.Left!.Value.sdi);
            UpdateOrientation(b.Right!.Value.deviceId, b.Right!.Value.sdi);

            // 2) compute headings (ENU Z-up projection)
            Quaternion qPelvis = _qWs[_pelvisId];
            float pelvisHeading = Utils.HeadingENU_Consistent(qPelvis, _forwardAxis[_pelvisId], isUseConjugate);

            float pelvisGyroMag = (float)b.Pelvis!.Value.cal.m_gyr.cartesianLength();
            float pelvisMagNorm = (float)b.Pelvis!.Value.cal.m_mag.cartesianLength();

            float pelvisHeadingUsed = AddPelvisHeadingByGate(pelvisHeading, b.PacketId, pelvisGyroMag, pelvisMagNorm);

            Quaternion qLeft = _qWs[_leftId];
            float leftRaw = Utils.HeadingENU_Consistent(qLeft, _forwardAxis[_leftId], isUseConjugate);

            Quaternion qRight = _qWs[_rightId];
            float rightRaw = Utils.HeadingENU_Consistent(qRight, _forwardAxis[_rightId], isUseConjugate);

            float leftCorr = Utils.WrapPi(leftRaw + _deltaLeft);
            float rightCorr = Utils.WrapPi(rightRaw + _deltaRight);

            float fpaLeft = Utils.WrapPi(leftCorr - pelvisHeading);
            float fpaRight = Utils.WrapPi(rightCorr - pelvisHeading);

            float leftGyroMag = (float)b.Left!.Value.cal.m_gyr.cartesianLength();
            float rightGyroMag = (float)b.Right!.Value.cal.m_gyr.cartesianLength();


            float leftMagNorm = (float)b.Left!.Value.cal.m_mag.cartesianLength();
            float rightMagNorm = (float)b.Right!.Value.cal.m_mag.cartesianLength();

            _leftFootGate.EnableAntiJumpGate = EnableAntiJumpGate;
            _rightFootGate.EnableAntiJumpGate = EnableAntiJumpGate;

            bool leftOk = _leftFootGate.Update(leftCorr, leftGyroMag, leftMagNorm, out float leftCorrUsed);
            bool rightOk = _rightFootGate.Update(rightCorr, rightGyroMag, rightMagNorm, out float rightCorrUsed);

            // 如果拒绝：最保守做法是“该帧不写入 step window”
            // 只要你在 Add(...) 前做判断即可，HandleFootStep 里会做 Add
            // 所以我们这里把 isStance 改成 false 来阻止 Add（不改变真实stance状态机可选）
            bool stanceLForAdd = stanceLeft && (leftOk || _leftFootGate.RejectStreak <= _maxFootRejectStreakForAdd);
            bool stanceRForAdd = stanceRight && (rightOk || _rightFootGate.RejectStreak <= _maxFootRejectStreakForAdd);


            // 然后用 leftCorrUsed/rightCorrUsed 继续走流程（避免突然跳变导致FPA炸）
            HandleFootStep(ImuRole.LeftFoot, b.PacketId, stanceLeft, stanceLForAdd,
                          fpaLeft, pelvisHeadingUsed, leftCorrUsed, leftGyroMag,
                            _leftStep, ref _leftWasStance);

            HandleFootStep(ImuRole.RightFoot, b.PacketId, stanceRight, stanceRForAdd,
                           fpaRight, pelvisHeadingUsed, rightCorrUsed, rightGyroMag,
                            _rightStep, ref _rightWasStance);

            // 4) instantaneous values for UI (optional: only when stance)
            float? uiLeft = stanceLeft ? fpaLeft : null;
            float? uiRight = stanceRight ? fpaRight : null;

            return (uiLeft, uiRight, pelvisHeading);
        }

        private float AddPelvisHeadingByGate(float pelvisHeading, long packetId, float pelvisGyroMag, float pelvisMagNorm)
        {
            bool accept = true;

            // --- A) jump gate (your existing idea) ---
            if (EnableAntiJumpGate && _pelvisHeadingPrev.HasValue)
            {
                float d = Utils.WrapPi(pelvisHeading - _pelvisHeadingPrev.Value);
                float dDeg = d * 180f / MathF.PI;
                if (MathF.Abs(dDeg) > _maxJumpDeg) accept = false;

                // --- B) yaw-rate consistency gate (more robust than jump alone) ---
                // heading-derived yaw-rate (rad/s) assuming 100Hz
                float yawRateFromHeading = MathF.Abs(d) * 100f;

                // gyro magnitude is an upper bound of actual yaw-rate (rough but useful)
                float yawRateFromGyro = pelvisGyroMag;

                if (yawRateFromHeading > yawRateFromGyro * _yawRateRatioBad + _yawRateMargin)
                    accept = false;
            }

            // --- C) learn mag baseline when still-ish ---
            if (pelvisGyroMag < _magLearnGyroTh && pelvisMagNorm > 1e-6f)
            {
                if (_pelvisMagBaseCount == 0) _pelvisMagBaseline = pelvisMagNorm;
                else _pelvisMagBaseline = _pelvisMagBaseline * 0.99f + pelvisMagNorm * 0.01f;
                _pelvisMagBaseCount++;
            }

            // --- D) mag disturbance gate ---
            if (_pelvisMagBaseCount > 200 && _pelvisMagBaseline > 1e-6f) // baseline ready (~2s)
            {
                float relDev = MathF.Abs(pelvisMagNorm - _pelvisMagBaseline) / _pelvisMagBaseline;
                if (relDev > _magRelBadTh)
                    accept = false;
            }

            // commit
            if (accept)
            {
                _pelvisProgDir.Add(pelvisHeading);
                _pelvisRejectStreak = 0;
                _pelvisHeadingUsedPrev = pelvisHeading;
            }
            else
            {
                _pelvisRejectStreak++;
                // 这里不更新 progDir，避免污染
            }

            _pelvisHeadingPrev = pelvisHeading;
            _pelvisPrevPacket = packetId;

            return accept ? pelvisHeading : (_pelvisHeadingUsedPrev ?? pelvisHeading);
        }

        private uint _lastPelvisAxisLogPacket = 0;
        private const uint _logEveryPackets = 100; // 100Hz -> 1秒

        private void HandleFootStep(ImuRole role, long packetId, bool isStance, bool allowAdd,
                              float fpaRad, float pelvisHeadingRad, float footCorrHeadingRad, float gyroMag,
                              StepDebugWindow window,
                              ref bool wasStance)
        {
            if (isStance && allowAdd)
                window.Add(pelvisHeadingRad, footCorrHeadingRad, gyroMag);

            // transition: stance -> swing means TO; finalize a per-step value
            if (wasStance && !isStance)
            {
                int subWindowN = Math.Min(window.WindowFrames, window.Count);
                if (subWindowN >= _minFramesForStep)
                {
                    var best = window.ComputeBestSubwindowFpaByCorrMinusPelvis(packetId, subWindowN: subWindowN);

                    if (best.HasValue)
                    {
                        float progDir = _pelvisProgDir.MeanRad;
                        float diff = Utils.WrapPi(best.CorrMeanRad - progDir);
                        float stepFpa = Utils.WrapPi(best.CorrMeanRad - progDir);

                        bool ok = _pelvisRejectStreak < 80 &&
                            MathF.Abs(diff) <= (140f * MathF.PI / 180f) &&
                            best.GyroMean < 0.45f &&
                            MathF.Abs(stepFpa * 180f / MathF.PI) < 50f &&
                            _pelvisRejectStreak < 50;

                        if (ok)
                        {
                            OnStepFpa?.Invoke(new StepFpaResult
                            {
                                Role = role,
                                PacketId = packetId,
                                FpaRad = stepFpa,
                                DebugBest = best,
                                PelvisProgDirRad = progDir
                            });
                        }
                    }
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
        public void ResetProgressionDirection()
        {
            _pelvisProgDir.Reset();
            _leftStep.Reset();
            _rightStep.Reset();
            _leftWasStance = false;
            _rightWasStance = false;

            _pelvisHeadingPrev = null;
            _pelvisHeadingUsedPrev = null;
            _pelvisRejectStreak = 0;
            _pelvisMagBaseline = 0;
            _pelvisMagBaseCount = 0;

            _leftFootGate.Reset();
            _rightFootGate.Reset();

            // 如果你有 stance detector / gait event detector，也在这里 Reset
            // stanceLeftDet.Reset(); stanceRightDet.Reset(); ...
        }

       
        private void UpdateOrientation(string deviceId, XsSdiData sdi)
        {
            Quaternion dq = ToNumericsQuaternion(sdi.orientationIncrement());

            // Common integration: q(t+dt) = normalize(q(t) * dq)
            Quaternion q = _qWs[deviceId];
            q = Utils.Normalize(Quaternion.Multiply(q, dq));
            _qWs[deviceId] = q;
        }

        private static Quaternion ToNumericsQuaternion(XsQuaternion inc)
        {
            // Adjust here if your SDK is WXYZ etc.
            return new Quaternion((float)inc.x(), (float)inc.y(), (float)inc.z(), (float)inc.w());
        }
    }

    public sealed class StepDebugWindow
    {
        public int WindowFrames { get; }
        public int Count => _corr.Count;

        // 存 stance 内每帧数据（不做滑动删除；我们要在 stance 全段里选最佳子窗）
        private readonly List<float> _pelvis = new();
        private readonly List<float> _corr = new();
        private readonly List<float> _gyro = new();

        public StepDebugWindow(int windowFrames)
        {
            WindowFrames = Math.Max(5, windowFrames);
        }

        public void Reset()
        {
            _pelvis.Clear();
            _corr.Clear();
            _gyro.Clear();
        }

        public void Add(float pelvisHeadingRad, float footCorrHeadingRad, float gyroMagRadPerSec)
        {
            _pelvis.Add(pelvisHeadingRad);
            _corr.Add(footCorrHeadingRad);
            _gyro.Add(gyroMagRadPerSec);
        }

        /// <summary>
        /// 在 stance 全段里找 gyroMean 最小的连续子窗口（长度 = subWindowN），
        /// 然后返回 stepFPA = wrap( mean(corr) - mean(pelvis) )。
        /// 同时返回该子窗口的 gyroMean/gyroMax 方便 debug。
        /// </summary>
        public BestWindowResult ComputeBestSubwindowFpaByCorrMinusPelvis(long packetId, int subWindowN)
        {
            subWindowN = Math.Max(5, subWindowN);

            int n = Count;
            if (n < subWindowN)
            {
                return new BestWindowResult
                {
                    PacketId = packetId,
                    FramesTotal = n,
                    FramesUsed = 0,
                    HasValue = false
                };
            }

            // 1) 找 gyroMean 最小的子窗口
            int bestStart = 0;
            double bestGyroMean = double.PositiveInfinity;
            float bestGyroMax = 0f;

            // 为了效率：用滑动和
            double gyroSum = 0;
            float gyroMax = 0;

            // init first window
            for (int i = 0; i < subWindowN; i++)
            {
                float g = _gyro[i];
                gyroSum += g;
                if (g > gyroMax) gyroMax = g;
            }
            bestGyroMean = gyroSum / subWindowN;
            bestGyroMax = gyroMax;

            // slide
            for (int start = 1; start <= n - subWindowN; start++)
            {
                float gOut = _gyro[start - 1];
                float gIn = _gyro[start + subWindowN - 1];
                gyroSum = gyroSum - gOut + gIn;

                // 维护 max：如果 max 被滑出，重算一次（窗口不大，开销可接受）
                if (Math.Abs(gOut - gyroMax) < 1e-6f || gIn > gyroMax)
                {
                    gyroMax = 0f;
                    for (int k = start; k < start + subWindowN; k++)
                    {
                        float g = _gyro[k];
                        if (g > gyroMax) gyroMax = g;
                    }
                }

                double mean = gyroSum / subWindowN;
                if (mean < bestGyroMean)
                {
                    bestGyroMean = mean;
                    bestGyroMax = gyroMax;
                    bestStart = start;
                }
            }

            // 2) 在 bestStart..bestStart+subWindowN 内分别算 mean(pelvis) 和 mean(corr)（圆均值）
            float meanPelvis = CircularMean(_pelvis, bestStart, subWindowN);
            float meanCorr = CircularMean(_corr, bestStart, subWindowN);

            float stepFpa = Utils.WrapPi(meanCorr - meanPelvis);

            return new BestWindowResult
            {
                PacketId = packetId,
                HasValue = true,
                FramesTotal = n,
                FramesUsed = subWindowN,
                BestStart = bestStart,
                GyroMean = (float)bestGyroMean,
                GyroMax = bestGyroMax,
                PelvisMeanRad = meanPelvis,
                CorrMeanRad = meanCorr,
                StepFpaRad = stepFpa
            };
        }

        private static float CircularMean(List<float> a, int start, int len)
        {
            double s = 0, c = 0;
            for (int i = start; i < start + len; i++)
            {
                double v = a[i];
                s += Math.Sin(v);
                c += Math.Cos(v);
            }
            return (float)Math.Atan2(s / len, c / len);
        }

    }
    public sealed class BestWindowResult
    {
        public long PacketId { get; init; }
        public bool HasValue { get; init; }

        public int FramesTotal { get; init; }
        public int FramesUsed { get; init; }
        public int BestStart { get; init; }

        public float GyroMean { get; init; }
        public float GyroMax { get; init; }

        public float PelvisMeanRad { get; init; }
        public float CorrMeanRad { get; init; }
        public float StepFpaRad { get; init; }

        public float PelvisDeg => PelvisMeanRad * 180f / MathF.PI;
        public float CorrDeg => CorrMeanRad * 180f / MathF.PI;
        public float StepFpaDeg => StepFpaRad * 180f / MathF.PI;
    }
    public sealed class StepDebugSummary
    {
        public long PacketId { get; init; }
        public int Frames { get; init; }

        public float FpaMeanRad { get; init; }
        public float PelvisHeadingMeanRad { get; init; }
        public float FootRawHeadingMeanRad { get; init; }
        public float FootCorrHeadingMeanRad { get; init; }

        public float GyroMean { get; init; } // rad/s
        public float GyroMax { get; init; }  // rad/s

        public float FpaDeg => FpaMeanRad * 180f / MathF.PI;
        public float PelvisDeg => PelvisHeadingMeanRad * 180f / MathF.PI;
        public float FootRawDeg => FootRawHeadingMeanRad * 180f / MathF.PI;
        public float FootCorrDeg => FootCorrHeadingMeanRad * 180f / MathF.PI;
    }
    public sealed class StepFpaResult
    {
        public ImuRole Role { get; init; }
        public long PacketId { get; init; }
        public float FpaRad { get; init; }
        public BestWindowResult? DebugBest { get; init; } // 可选
        public StepDebugSummary Debug { get; init; } = new StepDebugSummary();
        public float PelvisProgDirRad { get; init; }
    }

    public sealed class HeadingQualityGate
    {
        private readonly float _fs;

        // --- state ---
        private float? _prevHeading;
        private int _rejectStreak;

        private float _magBaseline = 0f;
        private int _magBaseCount = 0;

        // --- parameters (tune) ---
        public float MaxJumpDeg { get; set; } = 25f;          // 单帧超过25°基本是突变（100Hz）
        public float YawRateRatioBad { get; set; } = 4.0f;    // heading导出的yaw rate > gyroMag*ratio + margin => 可疑
        public float YawRateMargin { get; set; } = 0.5f;      // rad/s
        public float MagLearnGyroTh { get; set; } = 0.35f;    // rad/s，脚在stance静止时学习基线
        public float MagRelBadTh { get; set; } = 0.25f;       // |mag|相对偏差阈值（20~30%）
        public int MagBaselineMinCount { get; set; } = 200;   // 至少2秒@100Hz
        public bool EnableAntiJumpGate { get; set; } = true;  // 仅控制jump/yaw-rate突变门控

        public HeadingQualityGate(float fs)
        {
            _fs = fs;
        }

        public int RejectStreak => _rejectStreak;
        public bool HasPrev => _prevHeading.HasValue;

        public void Reset()
        {
            _prevHeading = null;
            _rejectStreak = 0;
            _magBaseline = 0f;
            _magBaseCount = 0;
        }

        /// <summary>
        /// 输入：heading(弧度), gyroMag(rad/s), magNorm(任意单位)
        /// 返回：accepted? 以及可用的 heading（若拒绝则返回 prevHeading 以保持连续）
        /// </summary>
        public bool Update(float headingRad, float gyroMag, float magNorm, out float headingUsed)
        {
            bool accept = true;

            // A) jump gate + yaw-rate consistency gate
            if (EnableAntiJumpGate && _prevHeading.HasValue)
            {
                float d = WrapPi(headingRad - _prevHeading.Value);
                float dDeg = d * 180f / MathF.PI;

                if (MathF.Abs(dDeg) > MaxJumpDeg)
                    accept = false;

                float yawRateFromHeading = MathF.Abs(d) * _fs; // rad/s
                float yawRateFromGyro = gyroMag;               // 上界近似

                if (yawRateFromHeading > yawRateFromGyro * YawRateRatioBad + YawRateMargin)
                    accept = false;
            }

            // B) learn magnetic baseline when still-ish (stance时更容易满足)
            if (gyroMag < MagLearnGyroTh && magNorm > 1e-6f)
            {
                if (_magBaseCount == 0) _magBaseline = magNorm;
                else _magBaseline = _magBaseline * 0.99f + magNorm * 0.01f;
                _magBaseCount++;
            }

            // C) mag disturbance gate
            if (_magBaseCount >= MagBaselineMinCount && _magBaseline > 1e-6f && magNorm > 1e-6f)
            {
                float relDev = MathF.Abs(magNorm - _magBaseline) / _magBaseline;
                if (relDev > MagRelBadTh)
                    accept = false;
            }

            if (accept)
            {
                _prevHeading = headingRad;
                _rejectStreak = 0;
                headingUsed = headingRad;
                return true;
            }
            else
            {
                _rejectStreak++;
                headingUsed = _prevHeading ?? headingRad; // 没prev就只能用当前
                return false;
            }
        }

        private static float WrapPi(float a)
        {
            while (a > MathF.PI) a -= 2f * MathF.PI;
            while (a < -MathF.PI) a += 2f * MathF.PI;
            return a;
        }
    }

}
