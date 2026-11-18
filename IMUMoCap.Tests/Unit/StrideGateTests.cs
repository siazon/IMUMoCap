using IMUMoCap.Pipeline;
using IMUMoCap.Pipeline.Models;

namespace IMUMoCap.Tests.Unit
{
    public class StrideGateTests
    {
        private static float Rad(float deg) => deg * MathF.PI / 180f;

        private static StrideResult Stride(float dirDeg = 10f, float len = 1.2f, float swing = 0.5f,
                                           float residual = 0.2f, int missing = 0)
            => new(Rad(dirDeg), len, swing, residual, missing, 0);

        private static StepExclusionReason? Check(StrideResult s, float? prevSame = 12f, bool otherStraight = true,
                                                  bool turning = false, float? fpa = 5f)
            => StrideGate.Check(s, prevSame is float p ? Rad(p) : null, otherStraight, turning, fpa);

        [Fact]
        public void NormalStraightStride_Passes() => Assert.Null(Check(Stride()));

        [Theory]
        [InlineData(0.2f)]   // G1: too short (one stance split in two)
        [InlineData(1.6f)]   // G1: too long
        public void SwingDurationOutOfRange_IsInvalid(float swing)
            => Assert.Equal(StepExclusionReason.StrideInvalid, Check(Stride(swing: swing)));

        [Fact]
        // G2 (2026-11-18, was 5): the gaps are wireless drops of any of the three sensors (frames are processed only
        // when all three arrive), short and scattered (longest gap per stride median 2–3 frames); strides with 6–12
        // missing frames had the same direction noise as 0–2 in the P3 / Pl89 / Po9 recordings.
        public void MoreThanTwelveMissingPackets_IsInvalid()
        {
            Assert.Null(Check(Stride(missing: 12)));
            Assert.Equal(StepExclusionReason.StrideInvalid, Check(Stride(missing: 13)));   // G2
        }

        [Theory]
        [InlineData(0.25f)]  // G3: shuffle
        [InlineData(2.6f)]   // G3: bad integration
        public void StrideLengthOutOfRange_IsInvalid(float len)
            => Assert.Equal(StepExclusionReason.StrideInvalid, Check(Stride(len: len)));

        [Fact]
        public void LargeResidualVelocity_IsInvalid()
            => Assert.Equal(StepExclusionReason.StrideInvalid, Check(Stride(residual: 1.0f)));  // G4

        [Fact]
        public void DirectionChangeVsSameFootPreviousStride_IsNotStraight()
        {
            Assert.Equal(StepExclusionReason.StrideNotStraight, Check(Stride(dirDeg: 10f), prevSame: 30f)); // G5
            Assert.Equal(StepExclusionReason.StrideNotStraight, Check(Stride(), prevSame: null));          // no history
        }

        // G6: each sensor has its own heading reference (VRU: arbitrary, drifting independently), so the feet's absolute
        // stride directions can't be compared. G6 instead requires the other foot's latest stride to be straight by its own G5.
        [Fact]
        public void OtherFootNotWalkingStraight_IsNotStraight()
            => Assert.Equal(StepExclusionReason.StrideNotStraight, Check(Stride(), otherStraight: false));

        [Fact]
        public void DirectionComparison_WrapsAround180()
            => Assert.Null(Check(Stride(dirDeg: 179f), prevSame: -178f));

        [Fact]
        public void Turning_IsExcludedAsTurning()
            => Assert.Equal(StepExclusionReason.Turning, Check(Stride(), turning: true));   // G7

        [Fact]
        public void ImplausibleFpa_IsInvalid_AndSkippedWhenUnknown()
        {
            Assert.Equal(StepExclusionReason.StrideInvalid, Check(Stride(), fpa: -61f));   // G8
            Assert.Null(Check(Stride(), fpa: null));   // zero not yet estimated
        }
    }
}
