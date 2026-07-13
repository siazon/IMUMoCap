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
    /// 每个 condition（EF/IF）一份的元数据文件内容：
    /// 个性化基线/目标参数 + 暂停记录 + 各阶段的最终有效 Attempt 号。
    /// </summary>
    public sealed class ConditionMeta
    {
        public string ParticipantId { get; set; } = "";
        public string Condition { get; set; } = "";
        public string OrderGroup { get; set; } = "";
        public BaselineProfile? Baseline { get; set; }
        public List<PauseEvent> PauseEvents { get; set; } = new();
        public Dictionary<string, int> FinalAttempt { get; set; } = new();
    }
}
