// IMUMoCap.Tests/Unit/FilterRestartDetectionTests.cs
using System.Collections.Generic;
using System.Numerics;
using IMUMoCap.Pipeline;
using Xunit;

namespace IMUMoCap.Tests.Unit
{
    /// <summary>
    /// GaitPipeline spots an MTw filter restart (StatusWord OrientationValid 1 → 0) before DataQualityGate drops the
    /// invalid frames, so FpaEngine / the Events log can react (2026-11-17: restarts near gyro saturation).
    /// </summary>
    public class FilterRestartDetectionTests
    {
        private const uint Valid = (uint)XDA.XsStatusFlag.XSF_OrientationValid;

        private static ImuFrameBundle Bundle(long pid, uint leftStatus, uint otherStatus = Valid)
        {
            var b = new ImuFrameBundle(pid, pid / 100.0);
            foreach (var role in new[] { ImuRole.Pelvis, ImuRole.Left, ImuRole.Right })
                b.AddOrUpdate(new ImuSampleFrame
                {
                    PacketId = pid, Role = role, StatusWord = role == ImuRole.Left ? leftStatus : otherStatus,
                    HasQuaternion = true, Quaternion = Quaternion.Identity,
                    HasRateOfTurn = true, RateOfTurn = Vector3.Zero,
                });
            return b;
        }

        [Fact]
        public void ValidToInvalid_IsReportedOncePerRestart_StartUpIsNot()
        {
            var p = new GaitPipeline();
            var restarts = new List<(ImuRole Role, long Packet)>();
            p.OnFilterRestart += (r, pkt) => restarts.Add((r, pkt));

            for (long i = 0; i < 5; i++) p.Process(Bundle(i, 0u, 0u));        // start-up initialisation: not a restart
            for (long i = 5; i < 10; i++) p.Process(Bundle(i, Valid));
            for (long i = 10; i < 15; i++) p.Process(Bundle(i, 0u));          // left restarts (several invalid frames)
            for (long i = 15; i < 20; i++) p.Process(Bundle(i, Valid));
            for (long i = 20; i < 22; i++) p.Process(Bundle(i, 0u));          // and again

            Assert.Equal(new[] { (ImuRole.Left, 10L), (ImuRole.Left, 20L) }, restarts);
        }
    }
}
