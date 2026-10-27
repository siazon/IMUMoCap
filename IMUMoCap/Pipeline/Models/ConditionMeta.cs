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
    /// 某个 stage 的耗时：起止墙钟时间 + 活跃时长（已扣除该阶段内的操作员暂停时长）。
    /// </summary>
    public sealed class StageTiming
    {
        public DateTime StartTime { get; set; }
        public DateTime? EndTime { get; set; }
        public double ActiveSeconds { get; set; }
    }

    /// <summary>
    /// 纯审计用的时间戳标记：Manipulation Check / NASA-TLX / IMI-PC 等由 RA 在纸面上主持的量表，
    /// 软件不渲染其内容，只在对应时刻记一笔 {Stage, TimestampUtc}。
    /// </summary>
    public sealed class StageMarker
    {
        public string Stage { get; set; } = "";
        public DateTime TimestampUtc { get; set; }
    }

    /// <summary>
    /// 每个 condition（EF/IF）一份的元数据文件内容：
    /// 个性化基线/目标参数 + 暂停记录 + 各阶段的最终有效 Attempt 号 + 各阶段 step 排除统计 + 各阶段耗时 + 审计标记。
    /// </summary>
    public sealed class ConditionMeta
    {
        public string ParticipantId { get; set; } = "";
        public string Condition { get; set; } = "";
        public BaselineProfile? Baseline { get; set; }
        public List<PauseEvent> PauseEvents { get; set; } = new();
        public Dictionary<string, int> FinalAttempt { get; set; } = new();
        public Dictionary<string, StepExclusionStats> StageStepStats { get; set; } = new();
        public Dictionary<string, StageTiming> StageTimings { get; set; } = new();
        public List<StageMarker> StageMarkers { get; set; } = new();

        // ── 以下为增量字段(T2):旧字段不得改动,分析脚本依赖它们 ──
        public RunConfig? RunConfig { get; set; }
        public List<RecordedEvent> Events { get; set; } = new();
    }

    /// <summary>
    /// 统一事件日志的一条记录(暂停/重校准/Mark/阶段切换等)。PacketId 是写入时最近处理的 IMU 包号,
    /// 与 Session CSV 的 PacketId 列对齐;事件真实发生时刻与该包号相差最多是 UI 线程调度延迟内的若干包,
    /// 分析时取事件之后的第一个包为参照。时间一律 UTC。
    /// </summary>
    public sealed class RecordedEvent
    {
        public int      Seq      { get; set; }
        public string   Type     { get; set; } = "";
        public long     PacketId { get; set; }
        public DateTime TsUtc    { get; set; }
        public string   Stage    { get; set; } = "";
        public int      Attempt  { get; set; } = 1;
        public string   Detail   { get; set; } = "";

        public static string CsvHeader => "Seq,Type,PacketId,TsUtc,Stage,Attempt,Detail";

        public string ToCsvRow() =>
            $"{Seq},{CsvEscape(Type)},{PacketId},{TsUtc.ToString("o", System.Globalization.CultureInfo.InvariantCulture)}," +
            $"{CsvEscape(Stage)},{Attempt},{CsvEscape(Detail)}";

        private static string CsvEscape(string s) =>
            s.IndexOfAny(new[] { ',', '"', '\n', '\r' }) < 0 ? s : "\"" + s.Replace("\"", "\"\"") + "\"";
    }

    /// <summary>
    /// 开始一个 condition 时的运行配置快照:软件版本(含 git commit)、PipelineParams 全部属性、
    /// 以及写在代码里的协议常量(Rest 时长、超时、容差等),事后可以确认这份数据是在什么设置下采集的。
    /// </summary>
    public sealed class RunConfig
    {
        public string AppVersion { get; set; } = "";
        public PipelineParams Params { get; set; } = new();
        public Dictionary<string, object> Constants { get; set; } = new();
    }
}
