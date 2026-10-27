// IMUMoCap.Tests/Unit/ExperimentRecorderMetaTests.cs
using System;
using System.IO;
using System.Linq;
using System.Text.Json;
using IMUMoCap.Methods;
using Xunit;

namespace IMUMoCap.Tests.Unit
{
    /// <summary>
    /// Meta JSON 的顶层字段是分析脚本依赖的契约(docs/flow-review-2026-10-27.md §7.1-5):
    /// 只允许新增字段,不得改名/删除。Order Group 已于 2026-10-27 决定移除(顺序由同一被试的 condition 记录推导)。
    /// </summary>
    public class ExperimentRecorderMetaTests : IDisposable
    {
        private readonly string _participantId = "T_" + Guid.NewGuid().ToString("N")[..8];
        private readonly ExperimentRecorder _recorder = new();

        private string DataDir => Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Data", $"P{_participantId}");

        public void Dispose()
        {
            _recorder.Dispose();
            if (Directory.Exists(DataDir)) Directory.Delete(DataDir, recursive: true);
        }

        [Fact]
        public void StartCondition_WritesMetaWithExpectedTopLevelFields_AndNoOrderGroup()
        {
            _recorder.StartCondition(_participantId, "EF");
            _recorder.FlushMeta();

            using var doc = JsonDocument.Parse(File.ReadAllText(_recorder.CurrentMetaFilePath!));
            var keys = doc.RootElement.EnumerateObject().Select(p => p.Name).OrderBy(n => n).ToArray();

            // 旧字段(分析脚本依赖)原样保留;Events、RunConfig 是 T2 新增的增量字段。
            var expected = new[]
            {
                "Baseline", "Condition", "Events", "FinalAttempt", "ParticipantId",
                "PauseEvents", "RunConfig", "StageMarkers", "StageStepStats", "StageTimings",
            };
            Assert.Equal(expected, keys);
        }

        [Fact]
        public void StartCondition_KeepsParticipantAndConditionInMeta()
        {
            _recorder.StartCondition(_participantId, "IF");
            _recorder.FlushMeta();

            using var doc = JsonDocument.Parse(File.ReadAllText(_recorder.CurrentMetaFilePath!));
            Assert.Equal(_participantId, doc.RootElement.GetProperty("ParticipantId").GetString());
            Assert.Equal("IF", doc.RootElement.GetProperty("Condition").GetString());
        }
    }
}
