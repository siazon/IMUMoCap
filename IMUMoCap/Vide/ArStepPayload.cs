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
        public string Msg { get; init; } = string.Empty;

        public override string ToString()
        {
            return "ArStepPayload{" +
                   $"CMD={CMD}, " +
                   $"Foot={Foot}, " +
                   $"FpaDegree={FpaDegree:F1}, " +
                   $"IsStance={IsStance}, " +
                   $"Color={status}, " +
                   $"Msg={Msg}, " +
                   "}";
        }

        private static string Format(double? value)
            => value.HasValue ? value.Value.ToString("F1") : "-";
    }
}
