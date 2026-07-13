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
    /// 一个 condition（EF/IF/Washout）对应一个连续写入的 Session CSV，
    /// EF/IF 各自额外对应一个 Meta JSON（BaselineProfile + 暂停记录 + Redo标记）。
    /// Session CSV 采用流式写入（每帧直接 append），不在内存里攒整段再导出。
    /// </summary>
    public sealed class ExperimentRecorder : IDisposable
    {
        public string ParticipantId { get; private set; } = "";
        public string Condition { get; private set; } = ""; // "EF" | "IF" | "Washout"
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

        public void StartCondition(string participantId, string condition, string? orderGroup)
        {
            Close();

            ParticipantId = participantId;
            Condition = condition;
            OrderGroup = orderGroup;

            var dir = Path.Combine(DataRoot, $"P{participantId}");
            Directory.CreateDirectory(dir);

            CurrentSessionFilePath = Path.Combine(dir, $"P{participantId}_{condition}_Session.csv");
            bool hasMeta = condition != "Washout";
            CurrentMetaFilePath = hasMeta ? Path.Combine(dir, $"P{participantId}_{condition}_Meta.json") : null;
            CurrentQoeFilePath = hasMeta ? Path.Combine(dir, $"P{participantId}_{condition}_QoE.csv") : null;

            _sessionWriter = new StreamWriter(CurrentSessionFilePath, append: false, Encoding.UTF8);
            _sessionWriter.WriteLine(DiagnosticsRow.CsvHeader);
            _sessionWriter.Flush();

            _meta = hasMeta
                ? new ConditionMeta { ParticipantId = participantId, Condition = condition, OrderGroup = orderGroup ?? "" }
                : null;
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

        public void BeginPause(string stage, string reason)
        {
            _openPause = new PauseEvent { Stage = stage, StartTime = DateTime.Now, Reason = reason };
        }

        public void EndPause(string operatorDecision)
        {
            if (_openPause == null || _meta == null) return;
            _openPause.EndTime = DateTime.Now;
            _openPause.OperatorDecision = operatorDecision;
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
