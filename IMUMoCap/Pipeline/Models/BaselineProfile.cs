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

        // 训练目标：Tᵢ = μᵢ − Δ（统一 ToeIn，Δ 为固定值，与 SD 无关）；容差为固定半宽
        public const float TargetDeltaDeg = 10f;   // Δ
        public const float ToleranceDeg   = 4f;    // 容差半宽 w
        public const float TargetMinDeg   = -25f;  // target 下限（过度内八保护）：μ−Δ 低于它时截断（实际 Δ < TargetDeltaDeg）；μ 本身低于它时改为 ToeOut

        public float Target_L    { get; init; }
        public float Target_R    { get; init; }
        /// <summary>实际生效的 Δ = μ − Target（ToeIn 为正；未被截断时等于 TargetDeltaDeg；ToeOut 时为负），分析时作为协变量。</summary>
        public float ActualDelta_L => MeanFpa_L - Target_L;
        public float ActualDelta_R => MeanFpa_R - Target_R;
        public TrainingDirection Direction_L { get; init; }
        public TrainingDirection Direction_R { get; init; }

        /// <summary>
        /// 容差半宽：所有训练块使用同一个固定值 ToleranceDeg，与 SD 无关。
        /// </summary>
        public float ToleranceL => ToleranceDeg;
        public float ToleranceR => ToleranceDeg;

        public int  MinRequiredSteps { get; init; }
        public bool IsValid => ValidSteps_L >= MinRequiredSteps && ValidSteps_R >= MinRequiredSteps;

        // ── 左右步数不对称检查（跟上面 FPA 角度的 Asymmetry 是两回事）────────
        public float StepCountRatio              { get; init; } // min/max，越接近1越均衡
        public float ImbalanceRatioThresholdUsed { get; init; } // 生成时生效的阈值（可调，记录下来便于事后复核）
        public bool  StepCountImbalanceWarning   { get; init; } // StepCountRatio < ImbalanceRatioThresholdUsed

        /// <summary>
        /// 从统计数据计算训练目标，规则：
        /// 默认 ToeIn，T = max(μ − Δ, TargetMinDeg)（Δ = TargetDeltaDeg），不使用 SD。
        /// 若 μ 本身已低于 TargetMinDeg（过度内八），改为 ToeOut，T = μ + Δ。
        /// </summary>
        public static BaselineProfile Create(
            float meanL, float sdL, int stepsL,
            float meanR, float sdR, int stepsR,
            int minRequiredSteps = 20,
            float imbalanceRatioThreshold = 0.7f)
        {
            static (float target, TrainingDirection dir) ComputeTarget(float mean) =>
                mean < TargetMinDeg
                    ? (mean + TargetDeltaDeg, TrainingDirection.ToeOut)
                    : (MathF.Max(mean - TargetDeltaDeg, TargetMinDeg), TrainingDirection.ToeIn);

            var (tL, dL) = ComputeTarget(meanL);
            var (tR, dR) = ComputeTarget(meanR);

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
