// IMUMoCap.Tests/Unit/ExperimentRecorderArchiveTests.cs
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
    /// 重新开始 condition(原 Start Calibration,docs/flow-review-2026-10-27.md §7.8、Q3):
    /// 旧数据归档而不是删除,且归档不能影响 SessionFileExists / GetScreeningResult 的判断。
    /// </summary>
    public class ExperimentRecorderArchiveTests : IDisposable
    {
        private readonly string _pid = "T_" + Guid.NewGuid().ToString("N")[..8];
        private readonly ExperimentRecorder _recorder = new();

        private string DataDir => Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Data", $"P{_pid}");

        public void Dispose()
        {
            _recorder.Dispose();
            if (Directory.Exists(DataDir)) Directory.Delete(DataDir, recursive: true);
        }

        private void StartWithSomeData()
        {
            _recorder.StartCondition(_pid, "EF");
            _recorder.BeginStage("Baseline");
            _recorder.WriteRow(new DiagnosticsRow { PacketId = 42 }, "Baseline", 1);
            _recorder.WriteLiveVsFpaConsistency(42, 1, "L", 1f, 1f, 1f, 1f);
        }

        [Fact]
        public void Archive_MovesEveryFileIntoATimestampedSubfolder_AndKeepsTheContent()
        {
            StartWithSomeData();
            var session = _recorder.CurrentSessionFilePath!;
            var meta = _recorder.CurrentMetaFilePath!;
            var live = _recorder.CurrentLiveConsistencyFilePath!;
            _recorder.RecordEvent("Abort", "test");

            var dest = _recorder.ArchiveCurrentFiles();

            Assert.NotNull(dest);
            Assert.StartsWith(Path.Combine(DataDir, "Aborted"), dest);
            Assert.False(File.Exists(session));
            Assert.False(File.Exists(meta));
            Assert.False(File.Exists(live));
            Assert.Contains(",42,", File.ReadAllText(Path.Combine(dest!, Path.GetFileName(session))));
            using var doc = JsonDocument.Parse(File.ReadAllText(Path.Combine(dest, Path.GetFileName(meta))));
            Assert.Contains(doc.RootElement.GetProperty("Events").EnumerateArray(), e => e.GetProperty("Type").GetString() == "Abort");
        }

        [Fact]
        public void Archive_ClosesTheRecorder_AndLeavesNothingToOverwrite()
        {
            StartWithSomeData();

            _recorder.ArchiveCurrentFiles();

            Assert.False(_recorder.IsRecording);
            Assert.False(ExperimentRecorder.SessionFileExists(_pid, "EF"));
            Assert.Empty(_recorder.Events);
        }

        [Fact]
        public void Archive_DoesNotCountAsAnExistingSession_ForTheScreeningFallback()
        {
            StartWithSomeData();
            _recorder.ArchiveCurrentFiles();

            // GetScreeningResult treats any *_Session.csv in the participant folder as an implicit Pass;
            // an archived (aborted) session must not do that.
            Assert.Null(ExperimentRecorder.GetScreeningResult(_pid));
        }

        [Fact]
        public void Archive_ThenStartConditionAgain_BeginsAFreshSessionWithoutAnOverwritePrompt()
        {
            StartWithSomeData();
            _recorder.ArchiveCurrentFiles();

            _recorder.StartCondition(_pid, "EF");

            Assert.True(_recorder.IsRecording);
            Assert.Single(_recorder.Events);   // just the new ConditionStart
            Assert.Equal(1, Directory.GetFiles(DataDir, "*_Session.csv").Length);
        }

        [Fact]
        public void Archive_TwiceInARow_KeepsBothArchives()
        {
            StartWithSomeData();
            var first = _recorder.ArchiveCurrentFiles();
            StartWithSomeData();
            var second = _recorder.ArchiveCurrentFiles();

            Assert.NotEqual(first, second);
            Assert.Equal(2, Directory.GetDirectories(Path.Combine(DataDir, "Aborted")).Length);
        }

        [Fact]
        public void Archive_WhenNothingWasStarted_ReturnsNull()
        {
            Assert.Null(_recorder.ArchiveCurrentFiles());
        }
    }
}
