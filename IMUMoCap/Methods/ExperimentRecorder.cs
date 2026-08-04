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
    /// 各自额外对应一个 Meta JSON（BaselineProfile + 暂停记录 + Redo标记）。
    /// Session CSV 采用流式写入（每帧直接 append），不在内存里攒整段再导出。
    /// </summary>
    public sealed class ExperimentRecorder : IDisposable
    {
        public string ParticipantId { get; private set; } = "";
        public string Condition { get; private set; } = ""; // "EF" | "IF"
        public string? OrderGroup { get; private set; }

        public string? CurrentSessionFilePath { get; private set; }
        public string? CurrentMetaFilePath { get; private set; }
        public string? CurrentQoeFilePath { get; private set; }

        public bool IsRecording => _sessionWriter != null;

        private StreamWriter? _sessionWriter;
        private ConditionMeta? _meta;
        private PauseEvent? _openPause;

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

        public void StartCondition(string participantId, string condition, string? orderGroup)
        {
            Close();

            ParticipantId = participantId;
            Condition = condition;
            OrderGroup = orderGroup;

            var dir = Path.Combine(DataRoot, $"P{participantId}");
            Directory.CreateDirectory(dir);

            CurrentSessionFilePath = Path.Combine(dir, $"P{participantId}_{condition}_Session.csv");
            CurrentMetaFilePath = Path.Combine(dir, $"P{participantId}_{condition}_Meta.json");
            CurrentQoeFilePath = Path.Combine(dir, $"P{participantId}_{condition}_QoE.csv");

            _sessionWriter = new StreamWriter(CurrentSessionFilePath, append: false, Encoding.UTF8);
            _sessionWriter.WriteLine(DiagnosticsRow.CsvHeader);
            _sessionWriter.Flush();

            _meta = new ConditionMeta { ParticipantId = participantId, Condition = condition, OrderGroup = orderGroup ?? "" };
        }

        public void WriteRow(DiagnosticsRow row, string stage, int attempt)
        {
            if (_sessionWriter == null) return;
            row.Stage = stage;
            row.Attempt = attempt;
            _sessionWriter.WriteLine(row.ToCsvRow());
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
        }

        // 原因现在在恢复时（Continue/Redo 都要填）才收集，而不是暂停当下。
        public void EndPause(string operatorDecision, string reason)
        {
            if (_openPause == null || _meta == null) return;
            _openPause.EndTime = DateTime.Now;
            _openPause.OperatorDecision = operatorDecision;
            _openPause.Reason = reason;
            _meta.PauseEvents.Add(_openPause);
            _openPause = null;
            SaveMeta();
        }

        public void MarkRedo(string stage, int newAttempt)
        {
            if (_meta == null) return;
            _meta.FinalAttempt[stage] = newAttempt;
            SaveMeta();
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

        public void Close()
        {
            _sessionWriter?.Flush();
            _sessionWriter?.Dispose();
            _sessionWriter = null;
        }

        public void Dispose() => Close();
    }
}
