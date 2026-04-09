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

        // 训练目标（Δ = 5° 固定）
        public float Target_L    { get; init; }  // T = μ ± 5°
        public float Target_R    { get; init; }
        public float Tolerance_L { get; init; }  // clamp(SD, 2°, 5°)
        public float Tolerance_R { get; init; }
        public TrainingDirection Direction_L { get; init; }
        public TrainingDirection Direction_R { get; init; }

        public bool IsValid => ValidSteps_L >= 20 && ValidSteps_R >= 20;

        /// <summary>
        /// 从统计数据计算训练目标，规则：
        /// μ > 10° → ToeIn，T = μ - 5°
        /// μ ≤ 10° → ToeOut，T = μ + 5°
        /// </summary>
        public static BaselineProfile Create(
            float meanL, float sdL, int stepsL,
            float meanR, float sdR, int stepsR)
        {
            static (float target, TrainingDirection dir) ComputeTarget(float mean)
            {
                if (mean > 10f)
                    return (mean - 5f, TrainingDirection.ToeIn);
                else
                    return (mean + 5f, TrainingDirection.ToeOut);
            }

            static float ComputeTolerance(float sd) => Math.Clamp(sd, 2f, 5f);

            var (tL, dL) = ComputeTarget(meanL);
            var (tR, dR) = ComputeTarget(meanR);

            return new BaselineProfile
            {
                MeanFpa_L   = meanL, SdFpa_L   = sdL, ValidSteps_L = stepsL,
                MeanFpa_R   = meanR, SdFpa_R   = sdR, ValidSteps_R = stepsR,
                Asymmetry   = MathF.Abs(meanL - meanR),
                Target_L    = tL,    Direction_L = dL, Tolerance_L = ComputeTolerance(sdL),
                Target_R    = tR,    Direction_R = dR, Tolerance_R = ComputeTolerance(sdR),
            };
        }
    }
}
