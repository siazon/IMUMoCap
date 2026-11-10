// IMUMoCap/Pipeline/MagMonitor.cs
using System;
using System.Numerics;

namespace IMUMoCap.Pipeline
{
    public enum MagLevel { Unknown, Ok, Warn, Bad }

    public readonly record struct MagSensorSnapshot(
        bool HasData, float Norm, float DipDeg, bool HasBaseline, float NormDevPct, float DipDevDeg, MagLevel Level);

    public readonly record struct MagSnapshot(
        MagSensorSnapshot Pelvis, MagSensorSnapshot Left, MagSensorSnapshot Right,
        bool HasHeading, float PelvisMinusFeetDeg, MagLevel HeadingLevel);

    /// <summary>
    /// 实时磁场/航向一致性监视(只用于界面显示,不进入 pipeline,不写文件)。
    ///
    /// 磁场:每个传感器的模长 |B| 和磁倾角 dip(磁场与水平面的夹角,世界系),用 1 s 指数平滑;
    ///   校准完成后前 3 s 的均值作为"自身基线",之后显示相对基线的偏差。没有绝对的"正常值"
    ///   (实测三个传感器模长/倾角本来就互不相同),所以只看相对自己基线的变化。
    /// 航向:骨盆相对校准姿态的朝向 − 双脚相对校准姿态朝向的圆周均值,3 s 指数平滑;
    ///   脚只在近似静止(角速度小于 StanceGyroThreshold,即 stance)时更新,避免摆动相干扰。
    ///   对称的内/外八在双脚均值里抵消,所以这个量反映的是"骨盆相对双脚"的偏移(漂移指示)。
    ///
    /// 采样率按 100 Hz 假设(与 pipeline 其它帧数阈值一致)。
    /// 线程安全:Update 在 SDK 线程,GetSnapshot 在 UI 线程。
    /// </summary>
    public sealed class MagMonitor
    {
        public const float NormWarnPct = 15f, NormBadPct = 30f;
        public const float DipWarnDeg = 10f, DipBadDeg = 20f;
        public const float HeadingWarnDeg = 10f, HeadingBadDeg = 20f;
        public const float StanceGyroThreshold = 1.0f; // rad/s,与 GaitEventDetector.GyroThreshold 一致

        private const int SampleRateHz = 100;
        private const int BaselineSamples = 3 * SampleRateHz;
        private static readonly float MagAlpha = 1f - MathF.Exp(-1f / (SampleRateHz * 1.0f));
        private static readonly float HeadingAlpha = 1f - MathF.Exp(-1f / (SampleRateHz * 3.0f));

        private sealed class Sensor
        {
            public bool HasData;
            public float Norm, Dip;
            public int BaseCount;
            public double BaseNormSum, BaseDipSum;
            public bool HasBaseline;
            public float BaseNorm, BaseDip;
            public bool HasHeading;   // 朝向单位向量的指数平滑
            public float Cos, Sin;
        }

        private readonly object _lock = new();
        private readonly Sensor _pelvis = new(), _left = new(), _right = new();
        private Quaternion? _refPelvis, _refLeft, _refRight;

        private Sensor? Of(ImuRole r) => r switch
        {
            ImuRole.Pelvis => _pelvis,
            ImuRole.Left => _left,
            ImuRole.Right => _right,
            _ => null,
        };

        /// <summary>新一轮校准开始:清掉基线和平滑状态。</summary>
        public void Reset()
        {
            lock (_lock)
            {
                foreach (var s in new[] { _pelvis, _left, _right })
                {
                    s.HasData = false; s.BaseCount = 0; s.BaseNormSum = s.BaseDipSum = 0;
                    s.HasBaseline = false; s.HasHeading = false;
                }
                _refPelvis = _refLeft = _refRight = null;
            }
        }

        /// <summary>校准完成:记下三个参考姿态,并开始 3 s 的磁场基线采集。</summary>
        public void StartBaseline(Quaternion pelvisRef, Quaternion leftRef, Quaternion rightRef)
        {
            lock (_lock)
            {
                _refPelvis = pelvisRef; _refLeft = leftRef; _refRight = rightRef;
                foreach (var s in new[] { _pelvis, _left, _right })
                {
                    s.BaseCount = 0; s.BaseNormSum = s.BaseDipSum = 0; s.HasBaseline = false; s.HasHeading = false;
                }
            }
        }

        public void Update(ImuSampleFrame sample)
        {
            var s = Of(sample.Role);
            if (s == null) return;
            lock (_lock)
            {
                if (sample.HasMagneticField && sample.HasQuaternion)
                {
                    var b = sample.MagneticField;
                    float norm = b.Length();
                    if (norm > 1e-6f)
                    {
                        var w = Vector3.Transform(b, sample.Quaternion);       // 传感器系 → 世界系
                        float dip = MathF.Asin(Math.Clamp(w.Z / norm, -1f, 1f)) * 180f / MathF.PI;
                        if (!s.HasData) { s.Norm = norm; s.Dip = dip; s.HasData = true; }
                        else { s.Norm += MagAlpha * (norm - s.Norm); s.Dip += MagAlpha * (dip - s.Dip); }

                        if (!s.HasBaseline && _refPelvis.HasValue)
                        {
                            s.BaseNormSum += norm; s.BaseDipSum += dip;
                            if (++s.BaseCount >= BaselineSamples)
                            {
                                s.BaseNorm = (float)(s.BaseNormSum / s.BaseCount);
                                s.BaseDip = (float)(s.BaseDipSum / s.BaseCount);
                                s.HasBaseline = true;
                            }
                        }
                    }
                }

                var reference = sample.Role switch { ImuRole.Pelvis => _refPelvis, ImuRole.Left => _refLeft, _ => _refRight };
                if (reference.HasValue && sample.HasQuaternion)
                {
                    bool isFoot = sample.Role != ImuRole.Pelvis;
                    bool still = !isFoot || (sample.HasRateOfTurn && sample.RateOfTurn.Length() < StanceGyroThreshold);
                    if (still)
                    {
                        float h = ProgressionDirEstimator.WorldHeading(sample.Quaternion, reference);
                        float c = MathF.Cos(h), sn = MathF.Sin(h);
                        if (!s.HasHeading) { s.Cos = c; s.Sin = sn; s.HasHeading = true; }
                        else { s.Cos += HeadingAlpha * (c - s.Cos); s.Sin += HeadingAlpha * (sn - s.Sin); }
                    }
                }
            }
        }

        public MagSnapshot GetSnapshot()
        {
            lock (_lock)
            {
                bool hasHeading = _pelvis.HasHeading && _left.HasHeading && _right.HasHeading;
                float rel = 0f;
                var headingLevel = MagLevel.Unknown;
                if (hasHeading)
                {
                    float pel = MathF.Atan2(_pelvis.Sin, _pelvis.Cos);
                    float feet = MathF.Atan2(_left.Sin + _right.Sin, _left.Cos + _right.Cos);
                    rel = Wrap180((pel - feet) * 180f / MathF.PI);
                    float a = MathF.Abs(rel);
                    headingLevel = a > HeadingBadDeg ? MagLevel.Bad : a > HeadingWarnDeg ? MagLevel.Warn : MagLevel.Ok;
                }
                return new MagSnapshot(Snap(_pelvis), Snap(_left), Snap(_right), hasHeading, rel, headingLevel);
            }
        }

        private static MagSensorSnapshot Snap(Sensor s)
        {
            if (!s.HasData) return new MagSensorSnapshot(false, 0, 0, false, 0, 0, MagLevel.Unknown);
            if (!s.HasBaseline) return new MagSensorSnapshot(true, s.Norm, s.Dip, false, 0, 0, MagLevel.Unknown);
            float normDev = (s.Norm / s.BaseNorm - 1f) * 100f;
            float dipDev = s.Dip - s.BaseDip;
            var level = Classify(MathF.Abs(normDev), NormWarnPct, NormBadPct);
            var dipLevel = Classify(MathF.Abs(dipDev), DipWarnDeg, DipBadDeg);
            if (dipLevel > level) level = dipLevel;
            return new MagSensorSnapshot(true, s.Norm, s.Dip, true, normDev, dipDev, level);
        }

        private static MagLevel Classify(float value, float warn, float bad)
            => value > bad ? MagLevel.Bad : value > warn ? MagLevel.Warn : MagLevel.Ok;

        private static float Wrap180(float deg)
        {
            while (deg > 180f) deg -= 360f;
            while (deg < -180f) deg += 360f;
            return deg;
        }
    }
}
