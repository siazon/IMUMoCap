// IMUMoCap/Protocol/ShadowMonitor.cs
using System;
using System.Collections.Generic;
using IMUMoCap.Model;

namespace IMUMoCap.Protocol
{
    /// <summary>MainWindow 里旧标志位在某一时刻的取值,供与 ProtocolFlow 比对。</summary>
    public sealed record LegacySnapshot(
        string CurrentStage,
        bool InRest,
        bool IsRetention,
        bool RetentionComplete,
        int RestBlockJustFinished,
        string? PendingStageEntry,
        bool IsPaused,
        bool IsRecording,
        IReadOnlyDictionary<string, bool> Marked,
        IReadOnlyDictionary<string, bool> CanMark,
        IReadOnlyDictionary<string, int> Attempts,
        IReadOnlyList<(string Label, ChecklistStatus Status)> Checklist,
        IReadOnlyList<string> MissingMarkers,
        string ArState);

    /// <summary>
    /// 影子模式(docs/flow-review-2026-10-27.md §8 T6):ProtocolFlow 与旧标志位并行运行,**只比对、不生效**。
    /// 旧代码每走一步,MainWindow 就把对应事件喂给 <see cref="Fire"/>;每个定时器 tick 再用 <see cref="Observe"/>
    /// 交一份旧状态快照。两边不一致且持续数个 tick 才写一条 SHADOW MISMATCH(避免瞬时抖动刷屏);
    /// 被 ProtocolFlow 忽略的事件也记一条 SHADOW ignored,便于人工判断是否合理。
    /// 所有方法都不会向调用方抛异常——影子逻辑绝不能影响正在进行的实验。
    /// </summary>
    public sealed class ShadowMonitor
    {
        private static readonly string[] MarkerNames =
            { FlowMarkers.Mc1, FlowMarkers.Mc3, FlowMarkers.NasaTlx1, FlowMarkers.ImiPc, FlowMarkers.NasaTlx2 };

        private readonly ProtocolFlow _flow;
        private readonly Action<string> _sink;
        private readonly int _debounceTicks;

        private string _lastEvent = "(none)";
        private string _key = "";        // 当前这一串连续相同的差异
        private int _count;              // 该差异已连续出现的 tick 数
        private string _loggedKey = "";  // 最近一次已写入日志的差异("" = 当前处于一致状态)

        // 每个 condition 一份的计数(StartCondition 时清零),用于在结束时证明\"没有输出\"是因为真的一致,而不是监视器没在跑。
        private int _eventsFed, _eventsIgnored, _ticksObserved, _mismatchReports;

        public ShadowMonitor(ProtocolFlow flow, Action<string> sink, int debounceTicks = 3)
        {
            _flow = flow;
            _sink = sink;
            _debounceTicks = Math.Max(1, debounceTicks);
        }

        /// <summary>把旧代码刚做的一步喂给 ProtocolFlow。被忽略的事件记一条日志。</summary>
        public FireResult Fire(FlowEvent e)
        {
            _lastEvent = Describe(e);
            if (e.Kind == FlowEventKind.StartCondition)
                _eventsFed = _eventsIgnored = _ticksObserved = _mismatchReports = 0;
            _eventsFed++;
            FireResult result;
            try
            {
                result = _flow.Fire(e);
            }
            catch (Exception ex)
            {
                Say($"SHADOW error while firing {_lastEvent}: {ex.Message}");
                return new FireResult(false, ex.Message);
            }
            if (!result.Accepted)
            {
                _eventsIgnored++;
                Say($"SHADOW ignored: {_lastEvent} in {DescribePhase(_flow.Phase)} — {result.Reason}");
            }
            return result;
        }

        /// <summary>写一行汇总(事件数、被忽略数、观察的 tick 数、不一致报告数)。在 condition 结束时调用。</summary>
        public void ReportSummary() =>
            Say($"SHADOW summary: events={_eventsFed} ignored={_eventsIgnored} ticks={_ticksObserved} mismatches={_mismatchReports}");

        /// <summary>每个定时器 tick 调用一次。</summary>
        public void Observe(LegacySnapshot legacy)
        {
            try
            {
                _ticksObserved++;
                string key = string.Join("; ", Differences(_flow, legacy));
                if (key == _key) _count = Math.Min(_count + 1, int.MaxValue - 1);
                else { _key = key; _count = 1; }
                if (_count < _debounceTicks) return;

                if (key.Length == 0)
                {
                    if (_loggedKey.Length > 0)
                    {
                        _loggedKey = "";
                        Say("SHADOW OK — legacy and flow agree again");
                    }
                    return;
                }
                if (key != _loggedKey)
                {
                    _loggedKey = key;
                    _mismatchReports++;
                    Say($"SHADOW MISMATCH (last event: {_lastEvent}; flow phase: {DescribePhase(_flow.Phase)}): {key}");
                }
            }
            catch (Exception ex)
            {
                Say($"SHADOW error while observing: {ex.Message}");
            }
        }

        /// <summary>旧状态与 ProtocolFlow 的逐项差异;空表示完全一致。</summary>
        public static IReadOnlyList<string> Differences(ProtocolFlow f, LegacySnapshot l)
        {
            var d = new List<string>();
            void Cmp<T>(string name, T legacy, T flow)
            {
                if (!EqualityComparer<T>.Default.Equals(legacy, flow))
                    d.Add($"{name} legacy={legacy} flow={flow}");
            }

            Cmp("CurrentStage", l.CurrentStage, f.CurrentStage);
            Cmp("InRest", l.InRest, f.InRest);
            Cmp("IsRetention", l.IsRetention, f.IsRetention);
            Cmp("RetentionComplete", l.RetentionComplete, f.RetentionComplete);
            Cmp("RestBlockJustFinished", l.RestBlockJustFinished, f.RestBlockJustFinished);

            string? flowPending = f.Phase.Kind is PhaseKind.RecalPending or PhaseKind.Recalibrating
                ? f.Phase.Stage.ToString() : null;
            Cmp("Pending", l.PendingStageEntry ?? "-", flowPending ?? "-");

            Cmp("Paused", l.IsPaused, f.Paused);
            Cmp("Recording", l.IsRecording, f.IsRecording);

            foreach (var m in MarkerNames)
            {
                Cmp($"Marked[{m}]", l.Marked.GetValueOrDefault(m), f.IsMarked(m));
                Cmp($"CanMark[{m}]", l.CanMark.GetValueOrDefault(m), f.CanMark(m));
            }
            foreach (var s in ProtocolFlow.StageNames)
                Cmp($"Attempt[{s}]", l.Attempts.GetValueOrDefault(s, 1), f.AttemptOf(s));

            foreach (var (label, status) in f.Checklist())
            {
                string legacyStatus = "(missing)";
                foreach (var row in l.Checklist)
                    if (row.Label == label) { legacyStatus = row.Status.ToString(); break; }
                Cmp($"Checklist[{label}]", legacyStatus, status.ToString());
            }

            Cmp("MissingMarkers", string.Join("|", l.MissingMarkers), string.Join("|", f.MissingExpectedMarkers()));
            Cmp("ArState", l.ArState, f.ArState);
            return d;
        }

        private void Say(string line)
        {
            try { _sink(line); } catch { /* never let diagnostics disturb the experiment */ }
        }

        private static string DescribePhase(Phase p) =>
            p.Stage is { } s ? $"{p.Kind}({s})" : p.Kind.ToString();

        private static string Describe(FlowEvent e) => e.Kind switch
        {
            FlowEventKind.Mark => $"Mark({e.Marker})",
            FlowEventKind.BaselineCompleted => $"BaselineCompleted(valid={e.Valid})",
            _ => e.Kind.ToString(),
        };
    }
}
