// IMUMoCap.Tests/Unit/ShadowMonitorTests.cs
using System.Collections.Generic;
using IMUMoCap.Protocol;
using Xunit;

namespace IMUMoCap.Tests.Unit
{
    /// <summary>
    /// ShadowMonitor 起初是 T6 的影子比对;T8 删除旧标志位后(T8b 清理),它只剩下一个职责:
    /// 把事件转发给 ProtocolFlow(唯一的流程状态来源),记录被拒绝的事件,并在 condition 结束时输出汇总。
    /// </summary>
    public class ShadowMonitorTests
    {
        // ── Fire ─────────────────────────────────────────────────────────────

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

        [Fact]
        public void Fire_DescribesTheStageOfThePhaseTheEventWasRefusedIn()
        {
            var f = new ProtocolFlow();
            var lines = new List<string>();
            var monitor = new ShadowMonitor(f, lines.Add);
            monitor.Fire(FlowEvent.StartCondition);

            monitor.Fire(FlowEvent.RestElapsed);

            Assert.Contains("Armed(Baseline)", Assert.Single(lines));
        }

        // ── summary: the absence of "ignored" lines must be provable ─────────

        [Fact]
        public void ReportSummary_CountsEventsAndIgnoredOnes()
        {
            var f = new ProtocolFlow();
            var lines = new List<string>();
            var monitor = new ShadowMonitor(f, lines.Add);

            monitor.Fire(FlowEvent.StartCondition);   // accepted
            monitor.Fire(FlowEvent.BlockComplete);    // ignored
            monitor.Fire(FlowEvent.NextStage);        // accepted (legacy semantics)
            lines.Clear();

            monitor.ReportSummary();

            var line = Assert.Single(lines);
            Assert.Contains("SHADOW summary", line);
            Assert.Contains("events=3", line);
            Assert.Contains("ignored=1", line);
        }

        [Fact]
        public void ReportSummary_HasNoLeftoversFromTheRemovedComparison()
        {
            var lines = new List<string>();
            var monitor = new ShadowMonitor(new ProtocolFlow(), lines.Add);

            monitor.ReportSummary();

            Assert.DoesNotContain("ticks", lines[0]);
            Assert.DoesNotContain("mismatches", lines[0]);
        }

        [Fact]
        public void StartCondition_ResetsTheSummaryCounters()
        {
            var f = new ProtocolFlow();
            var lines = new List<string>();
            var monitor = new ShadowMonitor(f, lines.Add);
            monitor.Fire(FlowEvent.StartCondition);
            monitor.Fire(FlowEvent.BlockComplete);
            lines.Clear();

            monitor.Fire(FlowEvent.StartCondition);    // a new condition starts counting from scratch
            monitor.ReportSummary();

            var line = Assert.Single(lines);
            Assert.Contains("events=1", line);
            Assert.Contains("ignored=0", line);
        }

        // ── the monitor must never disturb the experiment ────────────────────

        [Fact]
        public void Fire_NeverThrows_EvenIfTheSinkDoes()
        {
            var monitor = new ShadowMonitor(new ProtocolFlow(), _ => throw new System.IO.IOException("disk full"));

            var ex = Record.Exception(() => monitor.Fire(FlowEvent.BlockComplete)); // ignored → sink called → throws

            Assert.Null(ex);
        }

        [Fact]
        public void ReportSummary_NeverThrows_EvenIfTheSinkDoes()
        {
            var monitor = new ShadowMonitor(new ProtocolFlow(), _ => throw new System.IO.IOException("disk full"));

            Assert.Null(Record.Exception(() => monitor.ReportSummary()));
        }
    }
}
