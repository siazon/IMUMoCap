using System.Numerics;
using IMUMoCap.Pipeline;

namespace IMUMoCap.Tests.Unit
{
    public class MagMonitorTests
    {
        private static ImuSampleFrame Sample(ImuRole role, Vector3 mag, Quaternion q) => new()
        {
            Role = role,
            HasMagneticField = true, MagneticField = mag,
            HasQuaternion = true, Quaternion = q,
            HasRateOfTurn = true, RateOfTurn = Vector3.Zero,
        };

        private static void Feed(MagMonitor m, int n, Vector3 pelvisMag, Vector3 mag, Quaternion pelvisQ)
        {
            for (int i = 0; i < n; i++)
            {
                m.Update(Sample(ImuRole.Pelvis, pelvisMag, pelvisQ));
                m.Update(Sample(ImuRole.Left, mag, Quaternion.Identity));
                m.Update(Sample(ImuRole.Right, mag, Quaternion.Identity));
            }
        }

        [Fact]
        public void NormAndDip_AreComputed_AndBaselineDeviationIsClassified()
        {
            var m = new MagMonitor();
            var b = new Vector3(0.6f, 0f, -0.8f);                 // |B| = 1, dip = asin(-0.8) = -53.13°
            m.StartBaseline(Quaternion.Identity, Quaternion.Identity, Quaternion.Identity);
            Feed(m, 400, b, b, Quaternion.Identity);              // > 3 s: baseline captured

            var s = m.GetSnapshot();
            Assert.True(s.Left.HasBaseline);
            Assert.Equal(1f, s.Left.Norm, 2);
            Assert.Equal(-53.13f, s.Left.DipDeg, 1);
            Assert.Equal(MagLevel.Ok, s.Left.Level);

            // |B| +40% → Bad (EMA tau = 1 s, 10 s of data is plenty)
            Feed(m, 1000, b, b * 1.4f, Quaternion.Identity);
            s = m.GetSnapshot();
            Assert.InRange(s.Left.NormDevPct, 35f, 41f);
            Assert.Equal(MagLevel.Bad, s.Left.Level);
            Assert.Equal(MagLevel.Ok, s.Pelvis.Level);            // pelvis unchanged
        }

        [Fact]
        public void DipDeviation_IsWarnedByThreshold()
        {
            var m = new MagMonitor();
            var b = new Vector3(0.6f, 0f, -0.8f);                 // dip -53.13°
            m.StartBaseline(Quaternion.Identity, Quaternion.Identity, Quaternion.Identity);
            Feed(m, 400, b, b, Quaternion.Identity);

            // dip -42° (≈ +11° vs baseline), same |B| → Warn (> 10°, <= 20°)
            float rad = 42f * MathF.PI / 180f;
            var tilted = new Vector3(MathF.Cos(rad), 0f, -MathF.Sin(rad));
            Feed(m, 1000, b, tilted, Quaternion.Identity);
            var s = m.GetSnapshot();
            Assert.InRange(s.Left.DipDevDeg, 10f, 12f);
            Assert.Equal(MagLevel.Warn, s.Left.Level);
        }

        [Fact]
        public void PelvisMinusFeet_ReflectsPelvisHeadingOffset()
        {
            var m = new MagMonitor();
            var b = new Vector3(0.6f, 0f, -0.8f);
            m.StartBaseline(Quaternion.Identity, Quaternion.Identity, Quaternion.Identity);

            // feet stay at their calibration heading; pelvis yawed +30° about the vertical axis
            var yaw30 = Quaternion.CreateFromAxisAngle(Vector3.UnitZ, 30f * MathF.PI / 180f);
            Feed(m, 2000, b, b, yaw30);
            var s = m.GetSnapshot();
            Assert.True(s.HasHeading);
            Assert.Equal(30f, s.PelvisMinusFeetDeg, 0);
            Assert.Equal(MagLevel.Bad, s.HeadingLevel);

            // small offset → Ok
            var m2 = new MagMonitor();
            m2.StartBaseline(Quaternion.Identity, Quaternion.Identity, Quaternion.Identity);
            Feed(m2, 2000, b, b, Quaternion.CreateFromAxisAngle(Vector3.UnitZ, 5f * MathF.PI / 180f));
            Assert.Equal(MagLevel.Ok, m2.GetSnapshot().HeadingLevel);
        }

        [Fact]
        public void Reset_ClearsEverything()
        {
            var m = new MagMonitor();
            var b = new Vector3(0.6f, 0f, -0.8f);
            m.StartBaseline(Quaternion.Identity, Quaternion.Identity, Quaternion.Identity);
            Feed(m, 400, b, b, Quaternion.Identity);
            m.Reset();
            var s = m.GetSnapshot();
            Assert.False(s.Pelvis.HasData);
            Assert.False(s.HasHeading);
            Assert.Equal(MagLevel.Unknown, s.Left.Level);
        }
    }
}
