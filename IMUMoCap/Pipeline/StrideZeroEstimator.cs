// IMUMoCap/Pipeline/StrideZeroEstimator.cs
using System;

namespace IMUMoCap.Pipeline
{
    /// <summary>
    /// 步幅法零点 φ_f(docs/stride_fpa_design.md 一、方案 A):每只脚一个实例,
    /// 输入每步 dₖ = (ψₖ − sₖ) − 当前方法 raw FPA,前 RequiredSamples 个的圆周均值即 φ_f,之后固定到 Reset()(新 condition)。
    /// SdDeg 为圆周标准差,大于 WarnSdDeg 时调用方报警。
    /// </summary>
    public sealed class StrideZeroEstimator
    {
        // 2026-11-17: 10 → 5. Small rooms take several passes to collect straight strides, and the plan B zero compares
        // them with the stand heading, so sensor heading drift grows with the time taken; on Ps2 N=5 vs 10 differed by 0.3°.
        public const int RequiredSamples = 5;
        public const float WarnSdDeg = 5f;

        private double _cos, _sin;
        private int _n;

        public bool  IsReady   { get; private set; }
        public float OffsetRad { get; private set; }
        public float SdDeg     { get; private set; }

        /// <summary>返回 true 表示这一次调用使 φ 估计完成。</summary>
        public bool Add(float dRad)
        {
            if (IsReady) return false;
            _cos += Math.Cos(dRad);
            _sin += Math.Sin(dRad);
            if (++_n < RequiredSamples) return false;

            OffsetRad = (float)Math.Atan2(_sin, _cos);
            double r = Math.Min(1.0, Math.Sqrt(_cos * _cos + _sin * _sin) / _n);
            SdDeg = (float)(Math.Sqrt(-2.0 * Math.Log(r)) * 180.0 / Math.PI);
            IsReady = true;
            return true;
        }

        public void Reset()
        {
            _cos = _sin = 0;
            _n = 0;
            IsReady = false;
            OffsetRad = SdDeg = 0f;
        }
    }
}
