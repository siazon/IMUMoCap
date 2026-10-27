// IMUMoCap/Protocol/ShadowMonitor.cs
using System;

namespace IMUMoCap.Protocol
{
    /// <summary>
    /// 流程事件的唯一入口:MainWindow 的每个状态转换都经过 <see cref="Fire"/>,由它转发给 ProtocolFlow(流程状态的唯一来源),
    /// 并把被状态机拒绝的事件记成一条 "SHADOW ignored";condition 结束时 <see cref="ReportSummary"/> 输出一行汇总,
    /// 使"日志里没有 ignored"可以被证明,而不是监视器没在跑。
    ///
    /// 名字是历史遗留:它起初(T6)是把 ProtocolFlow 与旧标志位并行运行的影子比对;T8 删除旧标志位、T8b 删除比对代码后,
    /// 只剩转发与记录。日志前缀 SHADOW 与 Data/shadow_flow.log 的文件名保持不变,以免改变操作员的习惯。
    /// 所有方法都不会向调用方抛异常——记录逻辑绝不能影响正在进行的实验。
    /// </summary>
    public sealed class ShadowMonitor
    {
        private readonly ProtocolFlow _flow;
        private readonly Action<string> _sink;

        private string _lastEvent = "(none)";
        // 每个 condition 一份的计数(StartCondition 时清零)。
        private int _eventsFed, _eventsIgnored;

        public ShadowMonitor(ProtocolFlow flow, Action<string> sink)
        {
            _flow = flow;
            _sink = sink;
        }

        /// <summary>把一个流程事件交给状态机;被拒绝的事件记一条日志。返回状态机的裁决,调用方据此决定要不要执行副作用。</summary>
        public FireResult Fire(FlowEvent e)
        {
            _lastEvent = Describe(e);
            if (e.Kind == FlowEventKind.StartCondition)
                _eventsFed = _eventsIgnored = 0;
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

        /// <summary>写一行汇总(事件数、被拒绝数)。在 condition 结束时调用。</summary>
        public void ReportSummary() =>
            Say($"SHADOW summary: events={_eventsFed} ignored={_eventsIgnored}");

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
