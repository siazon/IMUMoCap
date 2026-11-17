using IMUMoCap.Pipeline;

namespace IMUMoCap.Tests.Unit
{
    public class StrideZeroEstimatorTests
    {
        private static float Rad(float deg) => deg * MathF.PI / 180f;
        private static float Deg(float rad) => rad * 180f / MathF.PI;

        // 5 strides (was 10): small rooms need several passes per stride count, and the zero is compared with the stand
        // heading, so sensor heading drift (VRU: up to ~10°/min) grows with the time it takes. Ps2: N=5 vs 10 differed by 0.3°.
        [Fact]
        public void BecomesReadyOnTheFifthSample_WithCircularMean()
        {
            var z = new StrideZeroEstimator();
            float[] d = { 18, 22, 19, 21, 20 };
            for (int i = 0; i < 4; i++) Assert.False(z.Add(Rad(d[i])));
            Assert.False(z.IsReady);
            Assert.True(z.Add(Rad(d[4])));
            Assert.True(z.IsReady);
            Assert.Equal(20f, Deg(z.OffsetRad), 1);
            Assert.InRange(z.SdDeg, 1.0f, 2.0f);
        }

        [Fact]
        public void Mean_WrapsAround180()
        {
            var z = new StrideZeroEstimator();
            foreach (float d in new[] { 178f, -178f, 180f, 179f, -179f }) z.Add(Rad(d));   // symmetric around 180
            Assert.Equal(180f, MathF.Abs(Deg(z.OffsetRad)), 1);
            Assert.InRange(z.SdDeg, 0f, 3f);
        }

        [Fact]
        public void LargeSpread_IsReportedForTheWarning()
        {
            var z = new StrideZeroEstimator();
            for (int i = 0; i < 10; i++) z.Add(Rad(i % 2 == 0 ? 10f : -10f));
            Assert.True(z.SdDeg > StrideZeroEstimator.WarnSdDeg);
        }

        [Fact]
        public void SamplesAfterReady_AreIgnored_UntilReset()
        {
            var z = new StrideZeroEstimator();
            for (int i = 0; i < 10; i++) z.Add(Rad(5f));
            Assert.False(z.Add(Rad(40f)));
            Assert.Equal(5f, Deg(z.OffsetRad), 1);

            z.Reset();
            Assert.False(z.IsReady);
            for (int i = 0; i < 10; i++) z.Add(Rad(-7f));
            Assert.Equal(-7f, Deg(z.OffsetRad), 1);
        }
    }
}
