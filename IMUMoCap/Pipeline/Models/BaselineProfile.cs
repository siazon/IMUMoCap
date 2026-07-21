// IMUMoCap/Pipeline/Models/BaselineProfile.cs
namespace IMUMoCap.Pipeline.Models
{
    public enum TrainingDirection { ToeIn, ToeOut }

    /// <summary>
    /// 基线统计结果 + 个性化训练目标。
    /// 由 BaselineProcessor 在 Baseline 阶段结束后生成，之后只读。
    /// </summary>
    public sealed class BaselineProfile
    {
        // 基线统计
        public float MeanFpa_L  { get; init; }
        public float MeanFpa_R  { get; init; }
        public float SdFpa_L    { get; init; }
        public float SdFpa_R    { get; init; }
        public float Asymmetry  { get; init; }   // |μ_L - μ_R|
        public int   ValidSteps_L { get; init; }
        public int   ValidSteps_R { get; init; }

        // 训练目标：Tᵢ = μᵢ ± k·SDᵢ（k 固定 = 1.0，个体化偏移量，见 Research Overview §2.4）
        public const float TargetK          = 1.0f;
        public const float ToleranceWMinDeg = 4f;   // w_min

        public float Target_L    { get; init; }
        public float Target_R    { get; init; }
        public TrainingDirection Direction_L { get; init; }
        public TrainingDirection Direction_R { get; init; }

        /// <summary>
        /// 当前训练块的容差半宽：w = max(w_min, α·SD)。α 随训练块递减实现渐进式难度，
        /// 由调用方（FpaEngine.ToleranceAlpha）传入，与 target 的 k 解耦，故不作为固定字段存储。
        /// </summary>
        public float ToleranceL(float alpha) => MathF.Max(ToleranceWMinDeg, alpha * SdFpa_L);
        public float ToleranceR(float alpha) => MathF.Max(ToleranceWMinDeg, alpha * SdFpa_R);

        public int  MinRequiredSteps { get; init; }
        public bool IsValid => ValidSteps_L >= MinRequiredSteps && ValidSteps_R >= MinRequiredSteps;

        // ── 左右步数不对称检查（跟上面 FPA 角度的 Asymmetry 是两回事）────────
        public float StepCountRatio              { get; init; } // min/max，越接近1越均衡
        public float ImbalanceRatioThresholdUsed { get; init; } // 生成时生效的阈值（可调，记录下来便于事后复核）
        public bool  StepCountImbalanceWarning   { get; init; } // StepCountRatio < ImbalanceRatioThresholdUsed

        /// <summary>
        /// 从统计数据计算训练目标，规则：
        /// μ > 10° → ToeIn，T = μ - k·SD
        /// μ ≤ 10° → ToeOut，T = μ + k·SD
        /// （方向判定规则本身文档未规定，沿用既有的 10° 阈值；偏移量按文档改为 k·SD）
        /// </summary>
        public static BaselineProfile Create(
            float meanL, float sdL, int stepsL,
            float meanR, float sdR, int stepsR,
            int minRequiredSteps = 20,
            float imbalanceRatioThreshold = 0.7f)
        {
            static (float target, TrainingDirection dir) ComputeTarget(float mean, float sd)
            {
                if (mean > 10f)
                    return (mean - TargetK * sd, TrainingDirection.ToeIn);
                else
                    return (mean + TargetK * sd, TrainingDirection.ToeOut);
            }

            var (tL, dL) = ComputeTarget(meanL, sdL);
            var (tR, dR) = ComputeTarget(meanR, sdR);

            float stepRatio = (stepsL == 0 || stepsR == 0)
                ? 0f
                : (float)Math.Min(stepsL, stepsR) / Math.Max(stepsL, stepsR);

            return new BaselineProfile
            {
                MeanFpa_L       = meanL, SdFpa_L   = sdL, ValidSteps_L = stepsL,
                MeanFpa_R       = meanR, SdFpa_R   = sdR, ValidSteps_R = stepsR,
                Asymmetry       = MathF.Abs(meanL - meanR),
                Target_L        = tL,    Direction_L = dL,
                Target_R        = tR,    Direction_R = dR,
                MinRequiredSteps = minRequiredSteps,
                StepCountRatio              = stepRatio,
                ImbalanceRatioThresholdUsed = imbalanceRatioThreshold,
                StepCountImbalanceWarning   = stepRatio < imbalanceRatioThreshold,
            };
        }
    }
}
