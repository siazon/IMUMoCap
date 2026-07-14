// IMUMoCap/Pipeline/Models/ConditionMeta.cs
using System;
using System.Collections.Generic;

namespace IMUMoCap.Pipeline.Models
{
    /// <summary>
    /// 一次暂停的记录：阶段、起止时间、原因、恢复时操作员的判断（Continue/Redo）。
    /// </summary>
    public sealed class PauseEvent
    {
        public string Stage { get; set; } = "";
        public DateTime StartTime { get; set; }
        public DateTime? EndTime { get; set; }
        public string Reason { get; set; } = "";
        public string OperatorDecision { get; set; } = ""; // "Continue" | "Redo"
    }

    /// <summary>
    /// 某个 stage 内的 step 门控结果统计：有多少步正常输出，多少步因为 Turning/PD 等原因被排除。
    /// 用于事后计算"转身相关排除率"这类 QoE 指标，而不是把这些排除静默丢弃。
    /// </summary>
    public sealed class StepExclusionStats
    {
        public int Emitted               { get; set; }
        public int ExcludedTurning       { get; set; }
        public int ExcludedReacquiring   { get; set; }
        public int ExcludedLowConfidence { get; set; }
        public int ExcludedPdInvalid     { get; set; }

        public int TotalAttempted =>
            Emitted + ExcludedTurning + ExcludedReacquiring + ExcludedLowConfidence + ExcludedPdInvalid;

        // Turning + ReacquiringPd 合并为"转身相关排除率"（ReacquiringPd 是转身后的方向重稳定期）
        public float TurningExclusionRate =>
            TotalAttempted == 0 ? 0f : (float)(ExcludedTurning + ExcludedReacquiring) / TotalAttempted;
    }

    /// <summary>
    /// 每个 condition（EF/IF）一份的元数据文件内容：
    /// 个性化基线/目标参数 + 暂停记录 + 各阶段的最终有效 Attempt 号 + 各阶段 step 排除统计。
    /// </summary>
    public sealed class ConditionMeta
    {
        public string ParticipantId { get; set; } = "";
        public string Condition { get; set; } = "";
        public string OrderGroup { get; set; } = "";
        public BaselineProfile? Baseline { get; set; }
        public List<PauseEvent> PauseEvents { get; set; } = new();
        public Dictionary<string, int> FinalAttempt { get; set; } = new();
        public Dictionary<string, StepExclusionStats> StageStepStats { get; set; } = new();
    }
}
