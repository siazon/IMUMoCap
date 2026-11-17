using System.Numerics;
using IMUMoCap.Pipeline;

namespace IMUMoCap.Tests.Unit
{
    public class StrideDirectionTrackerTests
    {
        private static Vector3 Dir(float deg, float mag)
        {
            float r = deg * MathF.PI / 180f;
            return new Vector3(MathF.Cos(r) * mag, MathF.Sin(r) * mag, 0f);
        }

        // Stance (zero acc) for 20 frames from packet `start`, then a swing of `swingFrames` frames whose horizontal
        // acceleration is +a along `deg` for the first half and −a for the second half (+ optional constant bias),
        // then stance again. Returns the result produced on the landing frame.
        private static StrideResult? RunStride(StrideDirectionTracker t, long start, int swingFrames, float deg, float a,
                                               Vector3 bias = default, long skipFrom = -1, int skipCount = 0)
        {
            long p = start;
            for (int i = 0; i < 20; i++) Assert.Null(t.Update(p++, true, Vector3.Zero));
            for (int i = 0; i < swingFrames; i++, p++)
            {
                if (p >= skipFrom && p < skipFrom + skipCount) continue; // dropped packets
                var acc = (i < swingFrames / 2 ? Dir(deg, a) : Dir(deg, -a)) + bias;
                Assert.Null(t.Update(p, false, acc));
            }
            return t.Update(p, true, Vector3.Zero);
        }

        [Fact]
        public void Swing_GivesDirectionLengthAndDuration()
        {
            var t = new StrideDirectionTracker();
            var r = RunStride(t, 1000, 60, 30f, 10f);  // 0.6 s swing: 0.5·10·0.3² ·2 = 0.9 m
            Assert.NotNull(r);
            Assert.Equal(30f, r!.Value.DirectionRad * 180f / MathF.PI, 0);
            Assert.InRange(r.Value.LengthM, 0.85f, 0.95f);
            Assert.InRange(r.Value.SwingSec, 0.6f, 0.62f);
            Assert.Equal(0, r.Value.MissingPackets);
            Assert.Equal(1080, r.Value.LandingPacketId);
        }

        [Fact]
        public void ConstantAccelerationBias_IsRemovedByZupt()
        {
            var t = new StrideDirectionTracker();
            var r = RunStride(t, 0, 60, -20f, 10f, bias: new Vector3(0.5f, -0.8f, 0f));
            Assert.Equal(-20f, r!.Value.DirectionRad * 180f / MathF.PI, 0);
            Assert.InRange(r.Value.LengthM, 0.85f, 0.95f);
            Assert.InRange(r.Value.ResidualSpeed, 0.5f, 0.6f); // |bias|·T ≈ 0.94·0.6 before the ZUPT correction
        }

        [Fact]
        public void VerticalAcceleration_IsIgnored()
        {
            var t = new StrideDirectionTracker();
            var r = RunStride(t, 0, 60, 90f, 10f, bias: new Vector3(0f, 0f, 3f));
            Assert.Equal(90f, r!.Value.DirectionRad * 180f / MathF.PI, 0);
            Assert.InRange(r.Value.ResidualSpeed, 0f, 0.01f);
        }

        [Fact]
        public void DroppedPackets_AreCountedAndTimedByPacketId()
        {
            var t = new StrideDirectionTracker();
            var r = RunStride(t, 0, 60, 0f, 10f, skipFrom: 40, skipCount: 2);
            Assert.Equal(2, r!.Value.MissingPackets);
            Assert.InRange(r.Value.SwingSec, 0.6f, 0.62f);
            Assert.InRange(r.Value.LengthM, 0.8f, 1.0f);
        }

        [Fact]
        public void SwingWithoutPrecedingStance_GivesNoResult()
        {
            var t = new StrideDirectionTracker();
            for (long p = 0; p < 30; p++) Assert.Null(t.Update(p, false, Dir(0f, 5f)));
            Assert.Null(t.Update(30, true, Vector3.Zero));   // no anchor: first landing after start-up
            var r = RunStride(t, 31, 60, 45f, 10f);          // the next full stride works
            Assert.Equal(45f, r!.Value.DirectionRad * 180f / MathF.PI, 0);
        }

        [Fact]
        public void Reset_DropsTheSwingInProgress()
        {
            var t = new StrideDirectionTracker();
            for (long p = 0; p < 10; p++) t.Update(p, true, Vector3.Zero);
            for (long p = 10; p < 40; p++) t.Update(p, false, Dir(0f, 10f));
            t.Reset();
            Assert.Null(t.Update(40, true, Vector3.Zero));
        }
    }
}
