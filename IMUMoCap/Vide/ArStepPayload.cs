using System;

namespace IMUMoCap
{
    internal sealed class ArStepPayload
    {
        public string CMD { get; init; } = string.Empty;
        public string Foot { get; init; } = string.Empty;
        public float FpaDegree { get; init; }
        public bool IsStance { get; init; }
        public string status { get; init; } = string.Empty;
        public double? TargetFpaDegree { get; init; }
        public double? DeltaFromTarget { get; init; }
        public int BaselineLeftCount { get; init; }
        public int BaselineRightCount { get; init; }
        public int BaselineRequiredPerFoot { get; init; }
        public string? BaselineProgress { get; init; }
        public string? FootBaselineProgress { get; init; }

        public override string ToString()
        {
            return "ArStepPayload{" +
                   $"CMD={CMD}, " +
                   $"Foot={Foot}, " +
                   $"FpaDegree={FpaDegree:F1}, " +
                   $"IsStance={IsStance}, " +
                   $"status={status}, " +
                   $"TargetFpaDegree={Format(TargetFpaDegree)}, " +
                   $"DeltaFromTarget={Format(DeltaFromTarget)}, " +
                   $"BaselineLeftCount={BaselineLeftCount}, " +
                   $"BaselineRightCount={BaselineRightCount}, " +
                   $"BaselineRequiredPerFoot={BaselineRequiredPerFoot}, " +
                   $"BaselineProgress={BaselineProgress ?? "null"}, " +
                   $"FootBaselineProgress={FootBaselineProgress ?? "null"}" +
                   "}";
        }

        private static string Format(double? value)
            => value.HasValue ? value.Value.ToString("F1") : "-";
    }
}
