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
        public float StaticGyroThreshold     { get; set; } = 0.3f;  // 0.05–1.0

        // GaitEventDetector
        public float StanceFreeAccThreshold  { get; set; } = 2.5f;  // 0.5–5.0
        public float StanceGyroThreshold     { get; set; } = 1.0f;  // 0.1–3.0
        public float StanceFootPitchThreshold { get; set; } = 0.35f;  // rad，~20°，转弯后left foot安装倾斜角大，10°过紧

        // FpaEngine gate thresholds
        public float PdConfidenceThreshold   { get; set; } = 0.7f;  // 0.3–1.0
        public float PdStabilityThreshold    { get; set; } = 0.7f;  // 0.3–1.0

        // BaselineProcessor / Training / Retention：每脚每阶段的目标有效步数
        public int   TargetStepsPerFoot      { get; set; } = 100;  // 1–150
        public float BaselineImbalanceRatioThreshold { get; set; } = 0.7f;  // 0.3–1.0，L/R步数比低于此值时警示

        /// <summary>独立副本(所有字段都是值类型)。</summary>
        public PipelineParams Clone() => (PipelineParams)MemberwiseClone();

        /// <summary>
        /// 两份参数之间每个发生变化的参数(名称、旧值、新值)。按反射遍历所有公开属性,所以新增参数无需改这里;
        /// 数值一律用不随区域设置变化的格式,事件日志里不会出现 "0,7"。
        /// </summary>
        public static System.Collections.Generic.IReadOnlyList<ParamChange> Diff(PipelineParams before, PipelineParams after)
        {
            var changes = new System.Collections.Generic.List<ParamChange>();
            foreach (var prop in typeof(PipelineParams).GetProperties())
            {
                var o = prop.GetValue(before);
                var n = prop.GetValue(after);
                if (!Equals(o, n))
                    changes.Add(new ParamChange(prop.Name, Format(o), Format(n)));
            }
            return changes;
        }

        private static string Format(object? v) =>
            System.Convert.ToString(v, System.Globalization.CultureInfo.InvariantCulture) ?? "";
    }

    /// <summary>一次参数变化,写入事件日志时的文本形如 "TargetStepsPerFoot=100->5"。</summary>
    public readonly record struct ParamChange(string Name, string Old, string New)
    {
        public string Describe() => $"{Name}={Old}->{New}";
    }
}
