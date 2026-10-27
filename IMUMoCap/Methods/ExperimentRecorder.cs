// IMUMoCap/Methods/ExperimentRecorder.cs
using System;
using System.IO;
using System.Text;
using System.Text.Json;
using IMUMoCap.Pipeline.Models;

namespace IMUMoCap.Methods
{
    /// <summary>
    /// 按 Participant/Condition 管理实验数据落盘。
    /// 一个 condition（EF/IF）对应一个连续写入的 Session CSV，
    /// 各自额外对应一个 Meta JSON（BaselineProfile + 暂停记录 + Redo标记），
    /// 以及一个 LiveVsFpaConsistency CSV（live 角度 vs 落地确认 fpa 值，供离线一致性分析）。
    /// Session/LiveVsFpaConsistency CSV 均采用流式写入（每帧/每次事件直接 append），不在内存里攒整段再导出。
    /// </summary>
    public sealed class ExperimentRecorder : IDisposable
    {
        public string ParticipantId { get; private set; } = "";
        public string Condition { get; private set; } = ""; // "EF" | "IF"

        public string? CurrentSessionFilePath { get; private set; }
        public string? CurrentMetaFilePath { get; private set; }
        public string? CurrentQoeFilePath { get; private set; }
        public string? CurrentLiveConsistencyFilePath { get; private set; }

        public bool IsRecording => _sessionWriter != null;

        private StreamWriter? _sessionWriter;
        private StreamWriter? _liveConsistencyWriter;
        private ConditionMeta? _meta;
        private PauseEvent? _openPause;
        private string _eventStage = ""; // 事件日志里 Stage 列的默认值:最近一次 BeginStage 的阶段

        /// <summary>
        /// 最近一次处理的 IMU 包号,由 MainWindow 在每个 DiagnosticsRow 到达时更新(不受暂停影响)。
        /// 事件日志用它与 Session CSV 的 PacketId 对齐。
        /// </summary>
        public long LatestPacketId { get; set; }

        public IReadOnlyList<RecordedEvent> Events =>
            (IReadOnlyList<RecordedEvent>?)_meta?.Events ?? Array.Empty<RecordedEvent>();

        private static string DataRoot =>
            Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Data");

        public TimeSpan PauseElapsed =>
            _openPause == null ? TimeSpan.Zero : DateTime.Now - _openPause.StartTime;

        /// <summary>True if StartCondition for this participant/condition would overwrite an existing Session CSV.</summary>
        public static bool SessionFileExists(string participantId, string condition)
        {
            var dir = Path.Combine(DataRoot, $"P{participantId}");
            var path = Path.Combine(dir, $"P{participantId}_{condition}_Session.csv");
            return File.Exists(path);
        }

        private static string ScreeningFilePath(string participantId) =>
            Path.Combine(DataRoot, $"P{participantId}", $"P{participantId}_VisualScreening.json");

        private static ScreeningRecord? ReadScreeningRecord(string participantId)
        {
            var path = ScreeningFilePath(participantId);
            if (!File.Exists(path)) return null;
            try { return JsonSerializer.Deserialize<ScreeningRecord>(File.ReadAllText(path, Encoding.UTF8)); }
            catch (JsonException) { return null; }
        }

        /// <summary>
        /// Latest visual-screening result for this participant: "Pass", "Fail", or null if never screened.
        /// A participant with no result file but already-recorded sessions pre-dates the screening feature
        /// and is treated as "Pass" (nothing is written), so ongoing participants aren't blocked.
        /// </summary>
        public static string? GetScreeningResult(string participantId)
        {
            var rec = ReadScreeningRecord(participantId);
            if (rec != null && rec.Result.Length > 0) return rec.Result;

            var dir = Path.Combine(DataRoot, $"P{participantId}");
            if (Directory.Exists(dir) && Directory.EnumerateFiles(dir, "*_Session.csv").Any()) return "Pass";
            return null;
        }

        /// <summary>Records a screening outcome: overwrites the latest result and appends to Attempts (audit trail).</summary>
        public static void SaveScreeningResult(string participantId, bool passed)
        {
            Directory.CreateDirectory(Path.Combine(DataRoot, $"P{participantId}"));
            var rec = ReadScreeningRecord(participantId) ?? new ScreeningRecord();
            var now = DateTime.UtcNow;
            rec.Result = passed ? "Pass" : "Fail";
            rec.TimestampUtc = now;
            rec.Attempts.Add(new ScreeningAttempt { TimestampUtc = now, Result = rec.Result });
            File.WriteAllText(ScreeningFilePath(participantId),
                JsonSerializer.Serialize(rec, new JsonSerializerOptions { WriteIndented = true }), Encoding.UTF8);
        }

        public void StartCondition(string participantId, string condition)
        {
            Close();

            ParticipantId = participantId;
            Condition = condition;
            _eventStage = "";
            _openPause = null;

            var dir = Path.Combine(DataRoot, $"P{participantId}");
            Directory.CreateDirectory(dir);

            CurrentSessionFilePath = Path.Combine(dir, $"P{participantId}_{condition}_Session.csv");
            CurrentMetaFilePath = Path.Combine(dir, $"P{participantId}_{condition}_Meta.json");
            CurrentQoeFilePath = Path.Combine(dir, $"P{participantId}_{condition}_QoE.csv");
            CurrentLiveConsistencyFilePath = Path.Combine(dir, $"P{participantId}_{condition}_LiveVsFpaConsistency.csv");

            _sessionWriter = new StreamWriter(CurrentSessionFilePath, append: false, Encoding.UTF8);
            _sessionWriter.WriteLine(DiagnosticsRow.CsvHeader);
            _sessionWriter.Flush();

            _liveConsistencyWriter = new StreamWriter(CurrentLiveConsistencyFilePath, append: false, Encoding.UTF8);
            _liveConsistencyWriter.WriteLine("PacketId,TsUtcMs,Foot,LiveAngleDeg,FpaErrorDeg,FpaDeg,TargetDeg");
            _liveConsistencyWriter.Flush();

            _meta = new ConditionMeta { ParticipantId = participantId, Condition = condition };
            RecordEvent("ConditionStart", $"P{participantId} {condition}");
        }

        /// <summary>
        /// 追加一条事件并落盘。仅在录制中生效;须在 UI 线程调用(与 WriteRow/SaveMeta 同线程,不加锁)。
        /// stage 缺省取最近一次 BeginStage 的阶段(Rest 期间仍是刚结束的训练阶段)。
        /// </summary>
        public void RecordEvent(string type, string detail = "", string? stage = null)
        {
            if (AddEvent(type, detail, stage)) SaveMeta();
        }

        // 只追加不落盘,供本身随后会 SaveMeta 的方法使用,避免重复写文件。
        private bool AddEvent(string type, string detail, string? stage)
        {
            if (_meta == null || !IsRecording) return false;
            stage ??= _eventStage;
            _meta.Events.Add(new RecordedEvent
            {
                Seq      = _meta.Events.Count + 1,
                Type     = type,
                PacketId = LatestPacketId,
                TsUtc    = DateTime.UtcNow,
                Stage    = stage,
                Attempt  = _meta.FinalAttempt.GetValueOrDefault(stage, 1),
                Detail   = detail,
            });
            return true;
        }

        /// <summary>
        /// 录制中 IMU 测量的启停写进事件日志(操作员点 Measure、掉线后重连等都会改变设备状态)。
        /// 事件的 PacketId 是状态变化前最后处理的包;按"取事件之后的第一个包"解读。
        /// 设备侧的 RECORDING/FLUSHING 仍属于测量中,不算启停。
        /// </summary>
        public void RecordMeasurementTransition(States previous, States next)
        {
            static bool Measuring(States s) => s is States.MEASURING or States.RECORDING or States.FLUSHING;
            bool was = Measuring(previous), now = Measuring(next);
            if (!was && now && next == States.MEASURING) RecordEvent("MeasureStart", $"{previous} -> {next}");
            else if (was && !now) RecordEvent("MeasureStop", $"{previous} -> {next}");
        }

        public void SetRunConfig(RunConfig config)
        {
            if (_meta == null) return;
            _meta.RunConfig = config;
            SaveMeta();
        }

        /// <summary>把事件日志导出为 CSV(Seq,Type,PacketId,TsUtc,Stage,Attempt,Detail),供与 Session CSV 按 PacketId 对齐分析。</summary>
        public void WriteEventsCsv(string path)
        {
            using var sw = new StreamWriter(path, append: false, Encoding.UTF8);
            sw.WriteLine(RecordedEvent.CsvHeader);
            foreach (var e in Events) sw.WriteLine(e.ToCsvRow());
        }

        public void WriteRow(DiagnosticsRow row, string stage, int attempt)
        {
            if (_sessionWriter == null) return;
            row.Stage = stage;
            row.Attempt = attempt;
            _sessionWriter.WriteLine(row.ToCsvRow());
        }

        /// <summary>
        /// One row per foot per fpa settlement event: that foot's most recently received `live`
        /// angle vs this event's final confirmed fpa value — offline input
        /// for a live-vs-fpa consistency analysis, not shown live in the UI. LiveAngleDeg is NaN
        /// whenever no `live` frame has been received yet (e.g. EF condition, or IF before the
        /// first throttled live frame of the block).
        /// </summary>
        public void WriteLiveVsFpaConsistency(long packetId, long tsUtcMs, string foot,
            float liveAngleDeg, float fpaErrorDeg, float fpaDeg, float targetDeg)
        {
            if (_liveConsistencyWriter == null) return;
            _liveConsistencyWriter.WriteLine(
                $"{packetId},{tsUtcMs},{foot},{liveAngleDeg:F3},{fpaErrorDeg:F3},{fpaDeg:F3},{targetDeg:F3}");
        }

        public void SaveBaseline(BaselineProfile profile)
        {
            if (_meta == null) return;
            _meta.Baseline = profile;
            SaveMeta();
        }

        public void BeginPause(string stage)
        {
            _openPause = new PauseEvent { Stage = stage, StartTime = DateTime.Now };
            RecordEvent("PauseStart", "", stage);
        }

        // 原因现在在恢复时（Continue/Redo 都要填）才收集，而不是暂停当下。
        public void EndPause(string operatorDecision, string reason)
        {
            if (_openPause == null || _meta == null) return;
            _openPause.EndTime = DateTime.Now;
            _openPause.OperatorDecision = operatorDecision;
            _openPause.Reason = reason;
            _meta.PauseEvents.Add(_openPause);
            AddEvent("PauseEnd", $"decision={operatorDecision}; reason={reason}", _openPause.Stage);
            _openPause = null;
            SaveMeta();
        }

        public void MarkRedo(string stage, int newAttempt)
        {
            if (_meta == null) return;
            _meta.FinalAttempt[stage] = newAttempt;
            AddEvent("Redo", $"attempt={newAttempt}", stage);
            SaveMeta();
        }

        /// <summary>
        /// 纯审计标记：记录一个由 RA 在纸面上主持的量表/检查（Manipulation Check、NASA-TLX、IMI-PC 等）
        /// 发生的 UTC 时间戳。软件不呈现这些量表的内容，只留痕。
        /// </summary>
        public void MarkStageEvent(string stage)
        {
            if (_meta == null) return;
            _meta.StageMarkers.Add(new StageMarker { Stage = stage, TimestampUtc = DateTime.UtcNow });
            const string recalPrefix = "Recalibrate_Before"; // written by MainWindow.AdvanceStage when the recalibration prompt is armed
            if (stage.StartsWith(recalPrefix, StringComparison.Ordinal))
                AddEvent("RecalPrompt", stage[recalPrefix.Length..], null);
            else
                AddEvent("Marker", stage, null);
            SaveMeta();
        }

        public void BeginStage(string stage)
        {
            if (_meta == null) return;
            _meta.StageTimings[stage] = new StageTiming { StartTime = DateTime.Now };
            _eventStage = stage;
            RecordEvent("StageStart", "", stage);
        }

        /// <summary>结算 stage 的活跃时长：墙钟时长减去该阶段内已结束的操作员暂停时长。</summary>
        public void EndStage(string stage)
        {
            if (_meta == null) return;
            if (!_meta.StageTimings.TryGetValue(stage, out var timing)) return;
            timing.EndTime = DateTime.Now;
            double pausedSeconds = 0;
            foreach (var p in _meta.PauseEvents)
                if (p.Stage == stage && p.EndTime.HasValue)
                    pausedSeconds += (p.EndTime.Value - p.StartTime).TotalSeconds;
            timing.ActiveSeconds = (timing.EndTime.Value - timing.StartTime).TotalSeconds - pausedSeconds;
            RecordEvent("StageEnd", FormattableString.Invariant($"activeSeconds={timing.ActiveSeconds:F1}"), stage);
        }

        /// <summary>
        /// 每个 step 判定结果调用一次（emitted 或被排除）。只更新内存计数，不落盘——
        /// 高频调用不适合每次都写文件，落盘统一在 FlushMeta() 里做（阶段切换/关闭时调用）。
        /// </summary>
        public void NoteStepOutcome(string stage, bool emitted, StepExclusionReason? reason)
        {
            if (_meta == null) return;
            if (!_meta.StageStepStats.TryGetValue(stage, out var stats))
                _meta.StageStepStats[stage] = stats = new StepExclusionStats();

            if (emitted) { stats.Emitted++; return; }
            switch (reason)
            {
                case StepExclusionReason.Turning:       stats.ExcludedTurning++; break;
                case StepExclusionReason.ReacquiringPd: stats.ExcludedReacquiring++; break;
                case StepExclusionReason.LowConfidence: stats.ExcludedLowConfidence++; break;
                case StepExclusionReason.PdInvalid:     stats.ExcludedPdInvalid++; break;
            }
        }

        public StepExclusionStats? GetStageStats(string stage) =>
            _meta?.StageStepStats.GetValueOrDefault(stage);

        /// <summary>把当前内存里的 Meta（含 StageStepStats）落盘一次，供阶段切换/关闭时调用。</summary>
        public void FlushMeta() => SaveMeta();

        private void SaveMeta()
        {
            if (_meta == null || CurrentMetaFilePath == null) return;
            var json = JsonSerializer.Serialize(_meta, new JsonSerializerOptions { WriteIndented = true });
            File.WriteAllText(CurrentMetaFilePath, json, Encoding.UTF8);
        }

        /// <summary>
        /// 重新开始 condition 时归档当前这次的全部文件(不删除):移到 Data/P{id}/Aborted/{UTC 时间戳}/。
        /// 放进子文件夹而不是改文件名,是因为 GetScreeningResult 会把被试目录下任意 *_Session.csv 当作隐式 Pass,
        /// SessionFileExists 也按原文件名判断——归档后两者都不应被旧数据影响。返回归档目录;没有开始过则返回 null。
        /// </summary>
        public string? ArchiveCurrentFiles()
        {
            Close();
            if (CurrentSessionFilePath == null) return null;

            var dest = Path.Combine(Path.GetDirectoryName(CurrentSessionFilePath)!, "Aborted",
                DateTime.UtcNow.ToString("yyyyMMdd'T'HHmmssfff'Z'"));
            Directory.CreateDirectory(dest);
            foreach (var path in new[] { CurrentSessionFilePath, CurrentMetaFilePath, CurrentQoeFilePath, CurrentLiveConsistencyFilePath })
                if (path != null && File.Exists(path))
                    File.Move(path, Path.Combine(dest, Path.GetFileName(path)));

            CurrentSessionFilePath = CurrentMetaFilePath = CurrentQoeFilePath = CurrentLiveConsistencyFilePath = null;
            _meta = null;
            _openPause = null;
            return dest;
        }

        public void Close()
        {
            _sessionWriter?.Flush();
            _sessionWriter?.Dispose();
            _sessionWriter = null;

            _liveConsistencyWriter?.Flush();
            _liveConsistencyWriter?.Dispose();
            _liveConsistencyWriter = null;
        }

        public void Dispose() => Close();
    }

    /// <summary>Contents of P{id}_VisualScreening.json.</summary>
    public sealed class ScreeningRecord
    {
        public string Result { get; set; } = "";
        public DateTime TimestampUtc { get; set; }
        public List<ScreeningAttempt> Attempts { get; set; } = new();
    }

    public sealed class ScreeningAttempt
    {
        public DateTime TimestampUtc { get; set; }
        public string Result { get; set; } = "";
    }
}
