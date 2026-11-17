// IMUMoCap.Tests/Integration/StrideFpaBatchReplayTests.cs
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Runtime.CompilerServices;
using IMUMoCap.Methods;
using IMUMoCap.Pipeline;
using Xunit;
using Xunit.Abstractions;

namespace IMUMoCap.Tests.Integration
{
    /// <summary>
    /// 步幅法批量回放报告:docs/ 下 2026-11-09 之后、能与 Events 配对的全部录制(此后流程不做阶段间重校准)。
    /// 每份按 Events 的 CalibrationStart / StageStart 驱动 GaitPipeline,输出两种方法分阶段的中位数,
    /// 以及 Baseline(无反馈的自然行走)内的线性趋势。录制不在 git 里,缺文件时跳过。运行约 1 分钟。
    /// </summary>
    [Trait("Category", "LongReplay")]
    public class StrideFpaBatchReplayTests
    {
        private const int Rate = 100;
        private readonly ITestOutputHelper _out;
        public StrideFpaBatchReplayTests(ITestOutputHelper output) => _out = output;

        private static string DocsDir([CallerFilePath] string thisFile = "")
            => Path.GetFullPath(Path.Combine(Path.GetDirectoryName(thisFile)!, "..", "..", "docs"));

        private sealed record Ev(string Type, long Packet, string Stage, string Detail);
        private sealed record Step(long Packet, string Stage, string Foot, float Stride, float Pd);

        [Fact]
        public void AllRecordingsSince1109_Report()
        {
            string docs = DocsDir();
            if (!Directory.Exists(docs)) { _out.WriteLine("SKIPPED: no docs dir"); return; }

            var events = Directory.GetFiles(docs, "Diagnostics_*_Events.csv")
                .Where(f => string.CompareOrdinal(Path.GetFileName(f), "Diagnostics_20261109") >= 0)
                .ToDictionary(f => f, ReadEvents);
            var rows = new List<string>();
            int replayed = 0;

            foreach (var samplesPath in Directory.GetFiles(docs, "ImuSamples_*.csv").OrderBy(f => f)
                         .Where(f => string.CompareOrdinal(Path.GetFileName(f), "ImuSamples_20261109") >= 0))
            {
                var samples = new CsvUtil().ReadImuSamplesCsv(samplesPath);
                long first = samples.Min(s => s.PacketId), last = samples.Max(s => s.PacketId);
                string name = Path.GetFileNameWithoutExtension(samplesPath).Replace("ImuSamples_", "");
                // PacketIds restart across days, so pair by file name: the first Events file of the same day saved at or
                // after the samples file (both are saved together at the end), whose ConditionStart lies in the samples' range.
                var evs = events.OrderBy(kv => kv.Key)
                    .Where(kv => Path.GetFileName(kv.Key).Substring(12, 8) == name[..8]
                                 && string.CompareOrdinal(Path.GetFileName(kv.Key).Substring(12, 15), name) >= 0)
                    .Select(kv => kv.Value)
                    .FirstOrDefault(e => e.FirstOrDefault(x => x.Type == "ConditionStart") is { } cs && cs.Packet >= first && cs.Packet <= last);
                if (evs == null) { rows.Add($"{name}: no matching Events, skipped"); continue; }
                if (evs.Any(e => e.Type == "CalibrationStart" && e.Detail != "initial"))
                { rows.Add($"{name}: has inter-stage recalibration, skipped"); continue; }

                var steps = Replay(CsvUtil.GroupIntoBundles(samples), evs, out var zeros, out var diag);
                replayed++;
                rows.Add($"== {name}  ({(last - first) / (float)Rate / 60f:F1} min)  zero: {zeros}");
                rows.Add($"  {diag}");
                foreach (var foot in new[] { "L", "R" })
                {
                    var fs = steps.Where(s => s.Foot == foot).ToList();
                    var stageText = fs.GroupBy(s => s.Stage).Select(g =>
                        $"{g.Key}: stride {Med(g.Select(s => s.Stride)),6:F1} (n={g.Count(s => !float.IsNaN(s.Stride))}) " +
                        $"pd {Med(g.Select(s => s.Pd)),6:F1} (n={g.Count(s => !float.IsNaN(s.Pd))})");
                    var b = fs.Where(s => s.Stage == "Baseline").ToList();
                    rows.Add($"  {foot} Baseline trend: stride {Slope(b, s => s.Stride),5:F2} °/min, pd {Slope(b, s => s.Pd),5:F2} °/min");
                    foreach (var t in stageText) rows.Add($"    {foot} {t}");
                }
            }
            foreach (var r in rows) _out.WriteLine(r);
            _out.WriteLine($"replayed {replayed} recordings");
        }

        private static List<Step> Replay(List<ImuFrameBundle> bundles, List<Ev> evs, out string zeros, out string diag)
        {
            var p = new GaitPipeline { SampleRateHz = Rate, UseStrideDirection = true, CalibrationArmed = true };
            var steps = new List<Step>();
            var z = new List<string>();
            string stage = "Baseline";
            var byId = bundles.ToDictionary(b => b.PacketId);
            // Independent stride integration on the same stance flags, only to explain which gate stops the zero estimate.
            var tL = new StrideDirectionTracker();
            var strides = new List<StrideResult>();
            float? lastDir = null;
            int straight = 0, rowsN = 0;
            var calStates = new List<string>();
            p.OnStrideZeroEstimated += (f, o, sd) => z.Add($"{f} {o:F1}±{sd:F1}");
            p.OnCalibrationStateChanged += s => { calStates.Add(s.ToString()); if (s == Pipeline.CalibrationState.Completed) p.StartBaseline(); };
            p.OnDiagnosticsFrame += d =>
            {
                rowsN++;
                if (tL.Update(d.PacketId, d.LeftStance, byId[d.PacketId].LeftFoot!.FreeAcceleration) is StrideResult r)
                {
                    strides.Add(r);
                    if (StrideGate.IsStraight(r, lastDir)) straight++;
                    if (StrideGate.IsIntegrationValid(r)) lastDir = r.DirectionRad;
                }
                if (!float.IsNaN(d.FpaLeft_StrideDeg) || !float.IsNaN(d.FpaLeft_PdDeg))
                    steps.Add(new Step(d.PacketId, stage, "L", d.FpaLeft_StrideDeg, d.FpaLeft_PdDeg));
                if (!float.IsNaN(d.FpaRight_StrideDeg) || !float.IsNaN(d.FpaRight_PdDeg))
                    steps.Add(new Step(d.PacketId, stage, "R", d.FpaRight_StrideDeg, d.FpaRight_PdDeg));
            };

            long calAt = evs.First(e => e.Type == "CalibrationStart").Packet;
            var stageStarts = evs.Where(e => e.Type == "StageStart" && e.Stage != "Baseline").ToList();
            bool calTriggered = false;
            int nextStage = 0;
            foreach (var b in bundles)
            {
                if (!calTriggered && b.PacketId >= calAt) calTriggered = p.TriggerCalibration();
                while (nextStage < stageStarts.Count && b.PacketId >= stageStarts[nextStage].Packet)
                {
                    stage = stageStarts[nextStage++].Stage;
                    if (stage == "Training1")
                    {
                        if (p.InBaseline) p.FinalizeBaseline();
                        if (p.BaselineProfile != null) p.StartTraining();
                    }
                }
                p.Process(b);
            }
            zeros = z.Count == 0 ? "none" : string.Join(", ", z);
            var lens = strides.Select(s => s.LengthM).OrderBy(v => v).ToArray();
            diag = $"calibration [{string.Join(">", calStates)}] at pkt {calAt} (file {bundles[0].PacketId}..{bundles[^1].PacketId}), " +
                   $"diag rows {rowsN}; L strides {strides.Count}, median length {(lens.Length > 0 ? lens[lens.Length / 2] : float.NaN):F2} m, " +
                   $"G1–G4 ok {strides.Count(StrideGate.IsIntegrationValid)}, G5 straight {straight}, " +
                   $"swing<0.25s {strides.Count(s => s.SwingSec < 0.25f)}, len<0.3 {strides.Count(s => s.LengthM < 0.3f)}, miss>5 {strides.Count(s => s.MissingPackets > 5)}";
            return steps;
        }

        private static List<Ev> ReadEvents(string path) => File.ReadAllLines(path).Skip(1)
            .Select(l => l.Split(','))
            .Where(c => c.Length >= 7)
            .Select(c => new Ev(c[1], long.Parse(c[2], CultureInfo.InvariantCulture), c[4], c[6]))
            .ToList();

        private static float Med(IEnumerable<float> xs)
        {
            var a = xs.Where(v => !float.IsNaN(v)).OrderBy(v => v).ToArray();
            return a.Length == 0 ? float.NaN : a[a.Length / 2];
        }

        // least-squares slope of value vs time, °/min; NaN if fewer than 10 points
        private static float Slope(List<Step> s, System.Func<Step, float> v)
        {
            var pts = s.Where(x => !float.IsNaN(v(x))).Select(x => (T: x.Packet / (double)Rate / 60.0, Y: (double)v(x))).ToList();
            if (pts.Count < 10) return float.NaN;
            double mt = pts.Average(q => q.T), my = pts.Average(q => q.Y);
            double num = pts.Sum(q => (q.T - mt) * (q.Y - my)), den = pts.Sum(q => (q.T - mt) * (q.T - mt));
            return den == 0 ? float.NaN : (float)(num / den);
        }
    }
}
