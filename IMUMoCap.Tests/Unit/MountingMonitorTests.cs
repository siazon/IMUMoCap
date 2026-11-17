using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using IMUMoCap.Pipeline;
using IMUMoCap.Pipeline.Models;

namespace IMUMoCap.Tests.Unit
{
    public class MountingMonitorTests
    {
        // Pelvis worn with its X axis up: pitch the sensor −90° about Y, so sensor X points along world Z.
        private static readonly Quaternion PelvisWorn = Quaternion.CreateFromAxisAngle(Vector3.UnitY, -MathF.PI / 2);
        // Foot: Z up, tilted 20° (instep slope).
        private static readonly Quaternion FootWorn = Tilt(20f);

        private static Quaternion Tilt(float deg, Quaternion? on = null)
            => Quaternion.Normalize(Quaternion.CreateFromAxisAngle(Vector3.UnitY, deg * MathF.PI / 180f) * (on ?? Quaternion.Identity));

        private static CalibrationProfile Profile(Quaternion pelvis, Quaternion left, Quaternion right)
            => new() { PelvisRef = pelvis, LeftFootRef = left, RightFootRef = right };

        [Fact]
        public void UpInSensor_FlatSensor_IsZ()
        {
            var up = MountingMonitor.UpInSensor(Quaternion.Identity);
            Assert.Equal(1f, up.Z, 3);
        }

        [Fact]
        public void UpInSensor_PelvisWorn_IsX()
        {
            var up = MountingMonitor.UpInSensor(PelvisWorn);
            Assert.Equal(1f, up.X, 3);
        }

        [Fact]
        public void CheckCalibration_CorrectMounting_Passes()
        {
            Assert.Null(MountingMonitor.CheckCalibration(Profile(PelvisWorn, FootWorn, FootWorn)));
        }

        [Fact]
        public void CheckCalibration_PelvisLyingFlat_Fails()
        {
            string? msg = MountingMonitor.CheckCalibration(Profile(Quaternion.Identity, FootWorn, FootWorn));
            Assert.NotNull(msg);
            Assert.Contains("Pelvis", msg);
            Assert.DoesNotContain("Left", msg);
        }

        [Fact]
        public void CheckCalibration_FootTilted50_Fails()
        {
            string? msg = MountingMonitor.CheckCalibration(Profile(PelvisWorn, FootWorn, Tilt(50f)));
            Assert.NotNull(msg);
            Assert.Contains("Right", msg);
        }

        [Fact]
        public void CheckCalibration_PelvisTilted25_Passes()
        {
            Assert.Null(MountingMonitor.CheckCalibration(Profile(Tilt(25f, PelvisWorn), FootWorn, FootWorn)));
        }

        private static MountingMonitor Started()
        {
            var m = new MountingMonitor();
            m.Start(Profile(PelvisWorn, FootWorn, FootWorn));
            return m;
        }

        private static readonly Vector3 Swinging = new(3f, 0f, 0f);   // rad/s

        private static ImuSampleFrame Frame(Quaternion q, bool still) => new() { Quaternion = q, RateOfTurn = still ? Vector3.Zero : Swinging };

        // Walking: the feet take turns, 0.5 s still / 0.5 s swinging. Not walking: both feet stay still.
        // The tilt passed for a foot applies while it is still; swingTilt (if given) while it swings.
        private static MountingAlert[] Walk(MountingMonitor m, int frames, Quaternion pelvis, Quaternion left, Quaternion right,
                                            bool walking = true, Quaternion? leftSwingTilt = null)
        {
            var alerts = new List<MountingAlert>();
            for (int i = 0; i < frames; i++)
            {
                bool leftStill = !walking || (i / 50) % 2 == 0;
                bool rightStill = !walking || (i / 50) % 2 == 1;
                alerts.AddRange(m.Update(Frame(pelvis, true), Frame(leftStill ? left : leftSwingTilt ?? left, leftStill), Frame(right, rightStill)));
            }
            return alerts.ToArray();
        }

        [Fact]
        public void NoShift_NoAlert()
        {
            var m = Started();
            Assert.Empty(Walk(m, 3000, PelvisWorn, FootWorn, FootWorn));
        }

        [Fact]
        public void FootShift15_WarnsOnce()
        {
            var m = Started();
            var alerts = Walk(m, 3000, PelvisWorn, Tilt(35f), FootWorn);
            var a = Assert.Single(alerts);
            Assert.Equal(ImuRole.Left, a.Role);
            Assert.Equal(MountingLevel.Warn, a.Level);
            Assert.InRange(a.ShiftDeg, 14f, 16f);
        }

        [Fact]
        public void PelvisShift25_WarnsThenPauses_EachOnce()
        {
            var m = Started();
            var alerts = Walk(m, 3000, Tilt(25f, PelvisWorn), FootWorn, FootWorn);
            Assert.Equal(new[] { MountingLevel.Warn, MountingLevel.Pause }, alerts.Select(a => a.Level).ToArray());
            Assert.All(alerts, a => Assert.Equal(ImuRole.Pelvis, a.Role));
        }

        [Fact]
        public void ShortBlip_ShorterThanWindow_NoAlert()
        {
            var m = Started();
            Walk(m, 1200, PelvisWorn, FootWorn, FootWorn);
            Assert.Empty(Walk(m, 100, PelvisWorn, Tilt(45f), FootWorn));   // one 0.5 s stance
            Assert.Empty(Walk(m, 1200, PelvisWorn, FootWorn, FootWorn));
        }

        [Fact]
        public void NotWalking_Ignored()
        {
            var m = Started();
            Assert.Empty(Walk(m, 3000, Tilt(40f, PelvisWorn), Tilt(60f), Tilt(60f), walking: false));
        }

        [Fact]
        public void FootTiltWhileSwinging_Ignored()
        {
            var m = Started();
            Assert.Empty(Walk(m, 3000, PelvisWorn, FootWorn, FootWorn, leftSwingTilt: Tilt(60f)));
        }

        [Fact]
        public void Start_NewCalibration_ResetsReferenceAndAlerts()
        {
            var m = Started();
            Assert.NotEmpty(Walk(m, 3000, PelvisWorn, Tilt(45f), FootWorn));
            m.Start(Profile(PelvisWorn, Tilt(45f), FootWorn));                    // remounted and recalibrated
            Assert.Empty(Walk(m, 3000, PelvisWorn, Tilt(45f), FootWorn));
            Assert.NotEmpty(Walk(m, 3000, PelvisWorn, Tilt(70f), FootWorn));      // and alerts again later
        }

        [Fact]
        public void BeforeStart_NoAlert()
        {
            var m = new MountingMonitor();
            Assert.Empty(Walk(m, 3000, Tilt(40f, PelvisWorn), Tilt(60f), Tilt(60f)));
        }
    }
}
