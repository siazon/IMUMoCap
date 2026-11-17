using System.Numerics;
using IMUMoCap.Pipeline;

namespace IMUMoCap.Tests.Unit
{
    public class WarmupMonitorTests
    {
        private const uint Valid = (uint)XDA.XsStatusFlag.XSF_OrientationValid;
        private static readonly Vector3 Gravity = new(0f, 0f, 9.81f);

        private static ImuSampleFrame Sample(ImuRole role, long packetId, uint? status, Vector3 gyro) => new()
        {
            Role = role, PacketId = packetId, StatusWord = status,
            HasRateOfTurn = true, RateOfTurn = gyro,
            HasAcceleration = true, Acceleration = Gravity,
        };

        // Feeds all three sensors for packets [from, from + n).
        private static void Feed(WarmupMonitor m, long from, int n, uint? status = Valid, Vector3 gyro = default)
        {
            for (long p = from; p < from + n; p++)
                foreach (var role in new[] { ImuRole.Pelvis, ImuRole.Left, ImuRole.Right })
                    m.Update(Sample(role, p, status, gyro));
        }

        [Fact]
        public void Still10s_Completes()
        {
            var m = new WarmupMonitor();
            Feed(m, 1000, 900);                        // 9 s
            Assert.False(m.IsComplete);
            Feed(m, 1900, 200);                        // 11 s total
            var s = m.GetSnapshot();
            Assert.True(s.IsComplete);
            Assert.Equal(1000, s.MeasurementStartPacketId);
            Assert.Equal(2000, s.CompletedPacketId);   // 10 s after the first valid packet
        }

        [Fact]
        public void Motion_RestartsTheCount()
        {
            var m = new WarmupMonitor();
            Feed(m, 0, 800);                           // 8 s still
            m.Update(Sample(ImuRole.Left, 800, Valid, new Vector3(0f, 0f, 0.2f))); // ~11 °/s on one foot
            Feed(m, 801, 800);                         // 8 s still again
            Assert.False(m.IsComplete);
            Assert.InRange(m.GetSnapshot().StillSeconds, 7.9, 8.0);
            Feed(m, 1601, 300);
            Assert.True(m.IsComplete);
        }

        [Fact]
        public void FramesBeforeOrientationValid_AreNotCounted()
        {
            var m = new WarmupMonitor();
            Feed(m, 0, 200, status: 0u);               // filter not yet valid (first ~2 s)
            Feed(m, 200, 950);
            Assert.False(m.IsComplete);
            Assert.Equal(200, m.GetSnapshot().MeasurementStartPacketId);
            Feed(m, 1150, 100);
            Assert.True(m.IsComplete);
        }

        [Fact]
        public void Reset_And_LostOrientationValid_ClearCompletion()
        {
            var m = new WarmupMonitor();
            Feed(m, 0, 1600);
            Assert.True(m.IsComplete);
            Feed(m, 1600, 500, gyro: new Vector3(1f, 0f, 0f)); // walking around after warm-up: stays complete
            Assert.True(m.IsComplete);

            m.Update(Sample(ImuRole.Right, 2100, 0u, Vector3.Zero)); // filter invalid → warm-up void
            Assert.False(m.IsComplete);

            Feed(m, 2101, 1600);
            Assert.True(m.IsComplete);
            m.Reset();                                 // measurement restarted
            var s = m.GetSnapshot();
            Assert.False(s.IsComplete);
            Assert.Null(s.MeasurementStartPacketId);
        }

        [Fact]
        public void Snapshot_ReportsEachSensorsStillSeconds_AndNullWithoutValidData()
        {
            var m = new WarmupMonitor();
            for (long p = 0; p < 1000; p++)
            {
                m.Update(Sample(ImuRole.Pelvis, p, Valid, Vector3.Zero));                       // still 10 s
                m.Update(Sample(ImuRole.Left, p, Valid, p < 500 ? new Vector3(0.5f, 0, 0) : Vector3.Zero)); // still for the last 5 s
                m.Update(Sample(ImuRole.Right, p, 0u, Vector3.Zero));                           // filter never valid
            }
            var s = m.GetSnapshot();
            Assert.InRange(s.PelvisSec!.Value, 9.9, 10.0);
            Assert.InRange(s.LeftSec!.Value, 4.9, 5.0);
            Assert.Null(s.RightSec);
            Assert.Equal(0, s.StillSeconds);
        }

        [Fact]
        public void EveryFilterRestart_IsReportedOnce_IncludingSimultaneousOnes()
        {
            // 2026-11-17 Po9: both feet re-initialised at the same packet; only "Left" was logged.
            var m = new WarmupMonitor();
            Feed(m, 0, 200, status: 0u);               // start-up initialisation: not a restart
            Assert.Empty(m.TakeRestartNotes());
            Feed(m, 200, 1600);
            foreach (var role in new[] { ImuRole.Left, ImuRole.Right })
                for (long p = 1800; p < 1803; p++) m.Update(Sample(role, p, 0u, Vector3.Zero));  // 3 invalid samples each

            var notes = m.TakeRestartNotes();
            Assert.Equal(2, notes.Count);
            Assert.Contains(notes, n => n.Contains("Left") && n.Contains("1800"));
            Assert.Contains(notes, n => n.Contains("Right") && n.Contains("1800"));
            Assert.Empty(m.TakeRestartNotes());       // taken once
        }

        [Fact]
        public void EachSensorLatchesItsOwnWarmup_SoOnlyARestartedFilterHasToWarmUpAgain()
        {
            // 2026-11-16 15:30: all three warmed up on the table, then the left foot's filter re-initialised while the
            // IMUs were being strapped on; strapping kept all three moving, so the old "all still together" rule never finished.
            var m = new WarmupMonitor();
            Feed(m, 0, 1600);
            Assert.True(m.IsComplete);
            m.Update(Sample(ImuRole.Left, 1600, 0u, Vector3.Zero));       // left filter restarts
            Assert.False(m.IsComplete);
            Feed(m, 1601, 300, gyro: new Vector3(1f, 0f, 0f));            // mounting: everyone moves
            var moving = new Vector3(1f, 0f, 0f);
            for (long p = 1901; p < 3500; p++)                            // left still, pelvis/right keep moving
            {
                m.Update(Sample(ImuRole.Pelvis, p, Valid, moving));
                m.Update(Sample(ImuRole.Left, p, Valid, Vector3.Zero));
                m.Update(Sample(ImuRole.Right, p, Valid, moving));
            }
            var s = m.GetSnapshot();
            Assert.True(s.IsComplete);
            Assert.Equal(10.0, s.PelvisSec);   // latched: shown as done although it's moving
            Assert.Equal(2901, s.CompletedPacketId);
        }

        [Fact]
        public void NullStatusWord_IsTreatedAsValid()
        {
            var m = new WarmupMonitor();
            Feed(m, 0, 1600, status: null);
            Assert.True(m.IsComplete);
        }
    }
}
