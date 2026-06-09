// IMUMoCap/Pipeline/Models/PipelineParams.cs
namespace IMUMoCap.Pipeline.Models
{
    /// <summary>
    /// Live-tunable pipeline parameters. GaitPipeline reads these on every Process() call
    /// and pushes them into sub-processors, so changes take effect immediately.
    /// </summary>
    public sealed class PipelineParams
    {
        // CalibrationProcessor
        public float StompThreshold_ms2      { get; set; } = 15f;   // 10–50
        public float StaticGyroThreshold     { get; set; } = 0.3f;  // 0.05–1.0

        // GaitEventDetector
        public float StanceFreeAccThreshold  { get; set; } = 2.5f;  // 0.5–5.0
        public float StanceGyroThreshold     { get; set; } = 1.0f;  // 0.1–3.0
        public float StanceFootPitchThreshold { get; set; } = 0.35f;  // rad，~20°，转弯后left foot安装倾斜角大，10°过紧

        // FpaEngine gate thresholds
        public float PdConfidenceThreshold   { get; set; } = 0.5f;  // 0.3–1.0
        public float PdStabilityThreshold    { get; set; } = 0.5f;  // 0.3–1.0

        // BaselineProcessor
        public int   MinBaselineSteps        { get; set; } = 5;    // 10–50
    }
}
