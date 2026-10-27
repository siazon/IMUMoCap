// IMUMoCap.Tests/Unit/ShadowMonitorTests.cs
using System.Collections.Generic;
using System.Linq;
using IMUMoCap.Model;
using IMUMoCap.Protocol;
using Xunit;

namespace IMUMoCap.Tests.Unit
{
    /// <summary>
    /// 影子模式(docs/flow-review-2026-10-27.md §8 T6):ProtocolFlow 与旧标志位并行运行,只比对不生效。
    /// 这里测的是比对与日志逻辑本身。
    /// </summary>
    public class ShadowMonitorTests
    {
        private static readonly string[] Markers =
            { FlowMarkers.Mc1, FlowMarkers.Mc3, FlowMarkers.NasaTlx1, FlowMarkers.ImiPc, FlowMarkers.NasaTlx2 };

        /// <summary>构造一份与 flow 完全一致的"旧状态"快照。</summary>
        private static LegacySnapshot SnapshotFrom(ProtocolFlow f) => new(
            CurrentStage: f.CurrentStage,
            InRest: f.InRest,
            IsRetention: f.IsRetention,
            RetentionComplete: f.RetentionComplete,
            RestBlockJustFinished: f.RestBlockJustFinished,
            PendingStageEntry: f.Phase.Kind is PhaseKind.RecalPending or PhaseKind.Recalibrating ? f.Phase.Stage.ToString() : null,
            IsPaused: f.Paused,
            IsRecording: f.IsRecording,
            Marked: Markers.ToDictionary(m => m, f.IsMarked),
            CanMark: Markers.ToDictionary(m => m, f.CanMark),
            Attempts: ProtocolFlow.StageNames.ToDictionary(s => s, f.AttemptOf),
            Checklist: f.Checklist(),
            ArState: f.ArState);

        private static ProtocolFlow AtRest1()
        {
            var f = new ProtocolFlow();
            foreach (var e in new[]
            {
                FlowEvent.StartCondition, FlowEvent.CalibrationStarted, FlowEvent.CalibrationCompleted,
                FlowEvent.BaselineCompleted(true), FlowEvent.CalibrationStarted, FlowEvent.CalibrationCompleted,
                FlowEvent.BlockComplete,
            })
                Assert.True(f.Fire(e).Accepted);
            return f;
        }

        // ── Differences ──────────────────────────────────────────────────────

        [Fact]
        public void Differences_AreEmpty_AtEveryStepOfTheFullProtocol()
        {
            var f = new ProtocolFlow();
            var script = new[]
            {
                FlowEvent.StartCondition, FlowEvent.CalibrationStarted, FlowEvent.CalibrationCompleted,
                FlowEvent.BaselineCompleted(true), FlowEvent.CalibrationStarted, FlowEvent.CalibrationCompleted,
                FlowEvent.BlockComplete, FlowEvent.Mark(FlowMarkers.Mc1), FlowEvent.RestElapsed,
                FlowEvent.CalibrationStarted, FlowEvent.CalibrationCompleted,
                FlowEvent.Pause, FlowEvent.ResumeRedo, FlowEvent.CalibrationStarted, FlowEvent.CalibrationCompleted,
                FlowEvent.BlockComplete, FlowEvent.CalibrationStarted, FlowEvent.CalibrationCompleted,
                FlowEvent.BlockComplete, FlowEvent.Mark(FlowMarkers.Mc3), FlowEvent.RestElapsed,
                FlowEvent.CalibrationStarted, FlowEvent.CalibrationCompleted,
                FlowEvent.RetentionTargetReached, FlowEvent.Mark(FlowMarkers.NasaTlx2), FlowEvent.EndRecording,
            };

            Assert.Empty(ShadowMonitor.Differences(f, SnapshotFrom(f)));
            foreach (var e in script)
            {
                Assert.True(f.Fire(e).Accepted, e.Kind.ToString());
                Assert.Empty(ShadowMonitor.Differences(f, SnapshotFrom(f)));
            }
        }

        [Fact]
        public void Differences_NameEachMismatchedField()
        {
            var f = AtRest1();
            var ok = SnapshotFrom(f);

            string One(LegacySnapshot s) => Assert.Single(ShadowMonitor.Differences(f, s));

            Assert.Contains("CurrentStage", One(ok with { CurrentStage = "Training2" }));
            Assert.Contains("InRest", One(ok with { InRest = false }));
            Assert.Contains("IsRetention", One(ok with { IsRetention = true }));
            Assert.Contains("RetentionComplete", One(ok with { RetentionComplete = true }));
            Assert.Contains("RestBlockJustFinished", One(ok with { RestBlockJustFinished = 0 }));
            Assert.Contains("Pending", One(ok with { PendingStageEntry = "Training2" }));
            Assert.Contains("Paused", One(ok with { IsPaused = true }));
            Assert.Contains("Recording", One(ok with { IsRecording = false }));
            Assert.Contains("ArState", One(ok with { ArState = "training" }));
        }

        [Fact]
        public void Differences_NameTheMarkerAttemptAndChecklistRow()
        {
            var f = AtRest1();
            var ok = SnapshotFrom(f);

            var marked = new Dictionary<string, bool>(ok.Marked) { [FlowMarkers.Mc1] = true };
            Assert.Contains("Marked[" + FlowMarkers.Mc1 + "]", Assert.Single(ShadowMonitor.Differences(f, ok with { Marked = marked })));

            var can = new Dictionary<string, bool>(ok.CanMark) { [FlowMarkers.Mc1] = false };
            Assert.Contains("CanMark[" + FlowMarkers.Mc1 + "]", Assert.Single(ShadowMonitor.Differences(f, ok with { CanMark = can })));

            var attempts = new Dictionary<string, int>(ok.Attempts) { ["Training1"] = 2 };
            Assert.Contains("Attempt[Training1]", Assert.Single(ShadowMonitor.Differences(f, ok with { Attempts = attempts })));

            var rows = ok.Checklist.Select(c => c.Label == "Training1" ? (c.Label, ChecklistStatus.Pending) : c).ToList();
            Assert.Contains("Checklist[Training1]", Assert.Single(ShadowMonitor.Differences(f, ok with { Checklist = rows })));
        }

        [Fact]
        public void Differences_PendingStageNameMustAlsoMatch()
        {
            var f = AtRest1();
            Assert.True(f.Fire(FlowEvent.RestElapsed).Accepted); // RecalPending(Training2)
            var ok = SnapshotFrom(f);

            Assert.Empty(ShadowMonitor.Differences(f, ok));
            Assert.Contains("Pending", Assert.Single(ShadowMonitor.Differences(f, ok with { PendingStageEntry = "Training3" })));
        }

        // ── Monitor: logging policy ──────────────────────────────────────────

        [Fact]
        public void Observe_LogsAMismatchOnlyAfterItPersists_AndOnlyOnce()
        {
            var f = AtRest1();
            var lines = new List<string>();
            var monitor = new ShadowMonitor(f, lines.Add, debounceTicks: 3);
            var bad = SnapshotFrom(f) with { InRest = false };

            monitor.Observe(bad);
            monitor.Observe(bad);
            Assert.Empty(lines);               // transient: not reported yet

            monitor.Observe(bad);
            var line = Assert.Single(lines);   // persisted for 3 ticks
            Assert.Contains("SHADOW MISMATCH", line);
            Assert.Contains("InRest", line);

            monitor.Observe(bad);
            monitor.Observe(bad);
            Assert.Single(lines);              // not repeated
        }

        [Fact]
        public void Observe_TransientMismatchThatClearsInTime_IsNeverReported()
        {
            var f = AtRest1();
            var lines = new List<string>();
            var monitor = new ShadowMonitor(f, lines.Add, debounceTicks: 3);
            var good = SnapshotFrom(f);
            var bad = good with { InRest = false };

            monitor.Observe(bad);
            monitor.Observe(bad);
            monitor.Observe(good);
            monitor.Observe(good);
            monitor.Observe(good);

            Assert.Empty(lines);
        }

        [Fact]
        public void Observe_ReportsRecovery_AndReportsAChangedMismatchAgain()
        {
            var f = AtRest1();
            var lines = new List<string>();
            var monitor = new ShadowMonitor(f, lines.Add, debounceTicks: 2);
            var good = SnapshotFrom(f);

            for (int i = 0; i < 2; i++) monitor.Observe(good with { InRest = false });
            Assert.Single(lines);

            for (int i = 0; i < 2; i++) monitor.Observe(good with { IsPaused = true });   // different mismatch
            Assert.Equal(2, lines.Count);
            Assert.Contains("Paused", lines[1]);

            for (int i = 0; i < 2; i++) monitor.Observe(good);
            Assert.Equal(3, lines.Count);
            Assert.Contains("SHADOW OK", lines[2]);

            for (int i = 0; i < 5; i++) monitor.Observe(good);
            Assert.Equal(3, lines.Count);                                                  // stays quiet
        }

        [Fact]
        public void MismatchLine_NamesTheLastEventFed()
        {
            var f = new ProtocolFlow();
            var lines = new List<string>();
            var monitor = new ShadowMonitor(f, lines.Add, debounceTicks: 1);

            monitor.Fire(FlowEvent.StartCondition);
            monitor.Fire(FlowEvent.Mark("not-a-marker")); // ignored → still "last event"
            monitor.Observe(SnapshotFrom(f) with { CurrentStage = "Training1" });

            Assert.Contains(lines, l => l.Contains("SHADOW MISMATCH") && l.Contains("Mark(not-a-marker)"));
        }

        // ── Monitor: Fire ────────────────────────────────────────────────────

        [Fact]
        public void Fire_ForwardsToTheFlow_AndStaysQuietWhenAccepted()
        {
            var f = new ProtocolFlow();
            var lines = new List<string>();
            var monitor = new ShadowMonitor(f, lines.Add);

            var r = monitor.Fire(FlowEvent.StartCondition);

            Assert.True(r.Accepted);
            Assert.Equal(PhaseKind.Armed, f.Phase.Kind);
            Assert.Empty(lines);
        }

        [Fact]
        public void Fire_LogsIgnoredEvents_WithPhaseAndReason()
        {
            var f = new ProtocolFlow();
            var lines = new List<string>();
            var monitor = new ShadowMonitor(f, lines.Add);

            var r = monitor.Fire(FlowEvent.BlockComplete);

            Assert.False(r.Accepted);
            var line = Assert.Single(lines);
            Assert.Contains("SHADOW ignored", line);
            Assert.Contains("BlockComplete", line);
            Assert.Contains("Idle", line);
        }

        [Fact]
        public void Fire_DescribesBaselineValidity_AndMarkerNames()
        {
            var f = new ProtocolFlow();
            var lines = new List<string>();
            var monitor = new ShadowMonitor(f, lines.Add);

            monitor.Fire(FlowEvent.BaselineCompleted(false)); // ignored in Idle → logged with its description
            monitor.Fire(FlowEvent.Mark(FlowMarkers.Mc1));

            Assert.Contains("BaselineCompleted(valid=False)", lines[0]);
            Assert.Contains("Mark(" + FlowMarkers.Mc1 + ")", lines[1]);
        }

        // ── summary: silence must be provable ────────────────────────────────

        [Fact]
        public void ReportSummary_CountsFedIgnoredTicksAndMismatchReports()
        {
            var f = new ProtocolFlow();
            var lines = new List<string>();
            var monitor = new ShadowMonitor(f, lines.Add, debounceTicks: 2);

            monitor.Fire(FlowEvent.StartCondition);   // accepted
            monitor.Fire(FlowEvent.BlockComplete);    // ignored
            var good = SnapshotFrom(f);
            monitor.Observe(good);
            monitor.Observe(good);
            var bad = good with { InRest = true };
            monitor.Observe(bad);
            monitor.Observe(bad);                      // one MISMATCH report
            lines.Clear();

            monitor.ReportSummary();

            var line = Assert.Single(lines);
            Assert.Contains("SHADOW summary", line);
            Assert.Contains("events=2", line);
            Assert.Contains("ignored=1", line);
            Assert.Contains("ticks=4", line);
            Assert.Contains("mismatches=1", line);
        }

        [Fact]
        public void StartCondition_ResetsTheSummaryCounters()
        {
            var f = new ProtocolFlow();
            var lines = new List<string>();
            var monitor = new ShadowMonitor(f, lines.Add);
            monitor.Fire(FlowEvent.StartCondition);
            monitor.Fire(FlowEvent.BlockComplete);
            monitor.Observe(SnapshotFrom(f));
            lines.Clear();

            monitor.Fire(FlowEvent.StartCondition);    // a new condition starts counting from scratch
            monitor.ReportSummary();

            var line = Assert.Single(lines);
            Assert.Contains("events=1", line);
            Assert.Contains("ignored=0", line);
            Assert.Contains("ticks=0", line);
        }

        // ── the shadow must never disturb the experiment ─────────────────────

        [Fact]
        public void Observe_NeverThrows_EvenIfTheSinkDoes()
        {
            var f = AtRest1();
            var monitor = new ShadowMonitor(f, _ => throw new System.IO.IOException("disk full"), debounceTicks: 1);

            var ex = Record.Exception(() => monitor.Observe(SnapshotFrom(f) with { InRest = false }));

            Assert.Null(ex);
        }

        [Fact]
        public void Fire_NeverThrows_EvenIfTheSinkDoes()
        {
            var monitor = new ShadowMonitor(new ProtocolFlow(), _ => throw new System.IO.IOException("disk full"));

            var ex = Record.Exception(() => monitor.Fire(FlowEvent.BlockComplete)); // ignored → sink called → throws

            Assert.Null(ex);
        }
    }
}
