// IMUMoCap/Pipeline/BaselineProcessor.cs
using System.Collections.Generic;
using System.Linq;
using IMUMoCap.Pipeline.Models;

namespace IMUMoCap.Pipeline
{
    /// <summary>
    /// 收集 Baseline 阶段的步级 FPA，计算统计量并生成 BaselineProfile。
    ///
    /// 只接受来自 FpaEngine 的有效 FpaResult（已经过四重门控）。
    /// 达到 MinValidSteps 后可提前调用 TryFinalize 生成结果；
    /// 3 分钟时间到后由 GaitPipeline 强制调用 TryFinalize。
    /// </summary>
    public sealed class BaselineProcessor
    {
        public int MinValidSteps { get; set; } = 20;

        private readonly List<float> _fpaL = new();
        private readonly List<float> _fpaR = new();

        public int CollectedSteps_L => _fpaL.Count;
        public int CollectedSteps_R => _fpaR.Count;
        public bool IsReady => _fpaL.Count >= MinValidSteps && _fpaR.Count >= MinValidSteps;

        /// <summary>每次 FpaEngine 输出一个 FpaResult 时调用。</summary>
        public void AddStep(FpaResult result)
        {
            // float.NaN 表示该脚本次无效（单脚无效时另一脚仍记录）
            if (!float.IsNaN(result.Fpa_L)) _fpaL.Add(result.Fpa_L);
            if (!float.IsNaN(result.Fpa_R)) _fpaR.Add(result.Fpa_R);
        }

        /// <summary>
        /// 尝试生成 BaselineProfile。
        /// IsReady 为 false 时返回 null（步数不足）。
        /// 调用方可在 Baseline 结束时强制调用，不足时显示错误。
        /// </summary>
        public BaselineProfile? TryFinalize()
        {
            if (!IsReady) return null;

            float meanL = Mean(_fpaL);
            float sdL   = Sd(_fpaL, meanL);
            float meanR = Mean(_fpaR);
            float sdR   = Sd(_fpaR, meanR);

            return BaselineProfile.Create(
                meanL, sdL, _fpaL.Count,
                meanR, sdR, _fpaR.Count);
        }

        public void Reset()
        {
            _fpaL.Clear();
            _fpaR.Clear();
        }

        // ── 工具方法 ──────────────────────────────────────────────────────────

        private static float Mean(List<float> values)
            => values.Count == 0 ? 0f : values.Sum() / values.Count;

        private static float Sd(List<float> values, float mean)
        {
            if (values.Count < 2) return 0f;
            float sumSq = values.Sum(v => (v - mean) * (v - mean));
            return System.MathF.Sqrt(sumSq / (values.Count - 1));
        }
    }
}
