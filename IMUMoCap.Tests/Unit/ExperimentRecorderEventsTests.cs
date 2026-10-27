// IMUMoCap.Tests/Unit/ExperimentRecorderEventsTests.cs
using System;
using System.IO;
using System.Linq;
using System.Text.Json;
using IMUMoCap.Methods;
using IMUMoCap.Pipeline.Models;
using Xunit;

namespace IMUMoCap.Tests.Unit
{
    /// <summary>
    /// 统一事件日志(docs/flow-review-2026-10-27.md §7.7、T2):每条事件带 PacketId + UTC 时间,
    /// 用于把暂停/重校准/Mark 等操作和 Session CSV 里的 IMU 样本按 PacketId 对齐。
    /// </summary>
    public class ExperimentRecorderEventsTests : IDisposable
    {
        private readonly string _participantId = "T_" + Guid.NewGuid().ToString("N")[..8];
        private readonly ExperimentRecorder _recorder = new();

        private string DataDir => Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Data", $"P{_participantId}");

        public void Dispose()
        {
            _recorder.Dispose();
            if (Directory.Exists(DataDir)) Directory.Delete(DataDir, recursive: true);
        }

        private void Start(string condition = "EF") => _recorder.StartCondition(_participantId, condition);

        [Fact]
        public void StartCondition_RecordsConditionStartEvent_AsSeq1()
        {
            Start("IF");

            var e = Assert.Single(_recorder.Events);
            Assert.Equal(1, e.Seq);
            Assert.Equal("ConditionStart", e.Type);
            Assert.Contains("IF", e.Detail);
            Assert.Contains(_participantId, e.Detail);
        }

        [Fact]
        public void RecordEvent_StampsPacketId_UtcTime_Stage_AndIncrementingSeq()
        {
            Start();
            _recorder.BeginStage("Baseline");
            _recorder.LatestPacketId = 4242;
            var before = DateTime.UtcNow.AddSeconds(-1);

            _recorder.RecordEvent("Custom", "some detail");

            var e = _recorder.Events.Last();
            Assert.Equal("Custom", e.Type);
            Assert.Equal("some detail", e.Detail);
            Assert.Equal(4242, e.PacketId);
            Assert.Equal("Baseline", e.Stage);
            Assert.Equal(1, e.Attempt);
            Assert.Equal(DateTimeKind.Utc, e.TsUtc.Kind);
            Assert.InRange(e.TsUtc, before, DateTime.UtcNow.AddSeconds(1));
            Assert.Equal(Enumerable.Range(1, _recorder.Events.Count), _recorder.Events.Select(x => x.Seq));
        }

        [Fact]
        public void MarkRedo_ChangesAttemptOfSubsequentEvents_AndRecordsRedoEvent()
        {
            Start();
            _recorder.BeginStage("Training1");

            _recorder.MarkRedo("Training1", 2);
            _recorder.RecordEvent("After");

            var redo = _recorder.Events.Single(e => e.Type == "Redo");
            Assert.Equal("attempt=2", redo.Detail);
            Assert.Equal(2, redo.Attempt);
            Assert.Equal(2, _recorder.Events.Last().Attempt);
        }

        [Fact]
        public void StageAndPauseHooks_ProduceEventsInOrder()
        {
            Start();
            _recorder.BeginStage("Training1");
            _recorder.BeginPause("Training1");
            _recorder.EndPause("Redo", "IMU slipped");
            _recorder.EndStage("Training1");

            Assert.Equal(
                new[] { "ConditionStart", "StageStart", "PauseStart", "PauseEnd", "StageEnd" },
                _recorder.Events.Select(e => e.Type));

            var pauseEnd = _recorder.Events.Single(e => e.Type == "PauseEnd");
            Assert.Contains("decision=Redo", pauseEnd.Detail);
            Assert.Contains("reason=IMU slipped", pauseEnd.Detail);
            Assert.Equal("Training1", pauseEnd.Stage);
        }

        [Fact]
        public void MarkStageEvent_RecordsMarker_AndMapsRecalibratePromptToRecalPrompt()
        {
            Start();
            _recorder.BeginStage("Training1");

            _recorder.MarkStageEvent("NASA-TLX_Admin1");
            _recorder.MarkStageEvent("Recalibrate_BeforeTraining2");

            var marker = _recorder.Events.Single(e => e.Type == "Marker");
            Assert.Equal("NASA-TLX_Admin1", marker.Detail);
            var prompt = _recorder.Events.Single(e => e.Type == "RecalPrompt");
            Assert.Equal("Training2", prompt.Detail);
            // 旧的 StageMarkers 照常写入(契约不变)
            Assert.Equal(2, JsonDocument.Parse(File.ReadAllText(_recorder.CurrentMetaFilePath!))
                .RootElement.GetProperty("StageMarkers").GetArrayLength());
        }

        [Fact]
        public void RecordEvent_IsNoOp_WhenNotRecording()
        {
            _recorder.RecordEvent("Orphan");
            Assert.Empty(_recorder.Events);

            Start();
            _recorder.Close();
            int countAfterClose = _recorder.Events.Count;
            _recorder.RecordEvent("AfterClose");
            Assert.Equal(countAfterClose, _recorder.Events.Count);
        }

        [Fact]
        public void SetRunConfig_IsPersistedInMeta()
        {
            Start();
            var cfg = new RunConfig { AppVersion = "1.0.0+abc1234" };
            cfg.Params.TargetStepsPerFoot = 5;
            cfg.Constants["RestDurationSec"] = 120;

            _recorder.SetRunConfig(cfg);

            using var doc = JsonDocument.Parse(File.ReadAllText(_recorder.CurrentMetaFilePath!));
            var rc = doc.RootElement.GetProperty("RunConfig");
            Assert.Equal("1.0.0+abc1234", rc.GetProperty("AppVersion").GetString());
            Assert.Equal(5, rc.GetProperty("Params").GetProperty("TargetStepsPerFoot").GetInt32());
            Assert.Equal(120, rc.GetProperty("Constants").GetProperty("RestDurationSec").GetInt32());
        }

        [Fact]
        public void WriteEventsCsv_WritesHeaderAndRows_AndEscapesDetail()
        {
            Start();
            _recorder.LatestPacketId = 77;
            _recorder.RecordEvent("Tricky", "a,b \"c\"\nline2");
            var path = Path.Combine(DataDir, "events.csv");

            _recorder.WriteEventsCsv(path);

            var text = File.ReadAllText(path);
            Assert.StartsWith("Seq,Type,PacketId,TsUtc,Stage,Attempt,Detail", text);
            Assert.Contains("\"a,b \"\"c\"\"\nline2\"", text);
            Assert.Contains(",77,", text);
        }

        [Fact]
        public void EventPacketId_CanBeFoundInSessionCsv()
        {
            Start();
            for (long pid = 100; pid <= 110; pid++)
                _recorder.WriteRow(new DiagnosticsRow { PacketId = pid }, "Baseline", 1);
            _recorder.LatestPacketId = 105;
            _recorder.RecordEvent("Probe");
            _recorder.Close();

            var packetIdsInSession = File.ReadLines(_recorder.CurrentSessionFilePath!)
                .Skip(1).Select(l => long.Parse(l.Split(',')[1])).ToList();
            var probe = _recorder.Events.Single(e => e.Type == "Probe");

            Assert.Contains(probe.PacketId, packetIdsInSession);
        }
    }
}
