// IMUMoCap.Tests/Integration/StrideFpaReplayTests.cs
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.CompilerServices;
using IMUMoCap.Methods;
using IMUMoCap.Pipeline;
using IMUMoCap.Pipeline.Models;
using Xunit;
using Xunit.Abstractions;

namespace IMUMoCap.Tests.Integration
{
    /// <summary>
    /// 步幅法回放验收(docs/stride_fpa_design.md 三-4):2026-11-12 16:38 的自然步态录制
    /// (约 4 分钟正常 + 1 分钟内八 + 4 分钟正常,来回直走)。当前方法在这份数据里骨盆漂移 25–35°、后 4 分钟无输出。
    /// 录制文件在 docs/ 下,未纳入 git;文件不存在时本测试直接返回(输出里会注明)。
    /// </summary>
    public class StrideFpaReplayTests
    {
        private const string SamplesFile = "ImuSamples_20261112_164938.csv";
        private const long CalibrationStartPacket = 784171;   // Events: CalibrationStart
        private const long TrainingStartPacket = 810668;      // Events: StageStart Training1
        private const int Rate = 100;

        private readonly ITestOutputHelper _out;
        public StrideFpaReplayTests(ITestOutputHelper output) => _out = output;

        private static string DocsDir([CallerFilePath] string thisFile = "")
            => Path.GetFullPath(Path.Combine(Path.GetDirectoryName(thisFile)!, "..", "..", "docs"));

        private sealed record Step(long PacketId, string Foot, float Stride, float Pd);

        [Fact]
        public void NaturalGait_1638_StrideFpaIsStable_DetectsToeIn_AndKeepsOutputting()
        {
            string path = Path.Combine(DocsDir(), SamplesFile);
            if (!File.Exists(path)) { _out.WriteLine($"SKIPPED: {path} not found"); return; }

            var bundles = CsvUtil.GroupIntoBundles(new CsvUtil().ReadImuSamplesCsv(path));
            var p = new GaitPipeline { SampleRateHz = Rate, UseStrideDirection = true, CalibrationArmed = true };
            var steps = new List<Step>();
            var outcomes = new List<(string Foot, bool Emitted, StepExclusionReason? Reason)>();
            var zeros = new List<string>();
            p.OnStepOutcome += (f, e, r) => outcomes.Add((f, e, r));
            p.OnStrideZeroEstimated += (f, o, sd) => zeros.Add($"{f} phi={o:F1} sd={sd:F1}");
            p.OnDiagnosticsFrame += d =>
            {
                if (!float.IsNaN(d.FpaLeft_StrideDeg)) steps.Add(new Step(d.PacketId, "L", d.FpaLeft_StrideDeg, d.FpaLeft_PdDeg));
                if (!float.IsNaN(d.FpaRight_StrideDeg)) steps.Add(new Step(d.PacketId, "R", d.FpaRight_StrideDeg, d.FpaRight_PdDeg));
            };
            p.OnCalibrationStateChanged += s => { if (s == CalibrationState.Completed) p.StartBaseline(); };

            bool calTriggered = false, trainingStarted = false;
            foreach (var b in bundles)
            {
                if (!calTriggered && b.PacketId >= CalibrationStartPacket) calTriggered = p.TriggerCalibration();
                if (!trainingStarted && b.PacketId >= TrainingStartPacket)
                {
                    trainingStarted = true;   // what MainWindow does at the stage change (FpaEngine.Reset, zero kept)
                    if (p.InBaseline) p.FinalizeBaseline();
                    if (p.BaselineProfile != null) p.StartTraining();
                }
                p.Process(b);
            }

            long end = bundles[^1].PacketId;
            _out.WriteLine($"bundles {bundles.Count}, calibration {(calTriggered ? p.CalibrationProfile != null ? "done" : "triggered" : "not triggered")}, " +
                           $"stride steps {steps.Count}, outcomes " +
                           string.Join(" ", outcomes.GroupBy(x => x.Emitted ? "emitted" : x.Reason.ToString()).Select(g => $"{g.Key}={g.Count()}")));
            _out.WriteLine($"zeros: {string.Join("; ", zeros)}");
            Assert.Equal(2, zeros.Count);

            // 30 s bins of the per-step stride FPA, per foot. Stability is judged on bins with >= 10 steps
            // (a 7-step bin's median has a standard error of ~1.5° at 3–4° per-step noise).
            var report = new List<string>();
            var failures = new List<string>();
            foreach (var foot in new[] { "L", "R" })
            {
                var fs = steps.Where(s => s.Foot == foot).ToList();
                var bins = fs.GroupBy(s => (s.PacketId - CalibrationStartPacket) / (30 * Rate))
                             .Where(g => g.Count() >= 5)
                             .Select(g => (Bin: g.Key, N: g.Count(), Med: Median(g.Select(s => s.Stride)),
                                           PdMed: Median(g.Select(s => s.Pd).Where(v => !float.IsNaN(v)))))
                             .OrderBy(x => x.Bin).ToList();
                foreach (var x in bins)
                    _out.WriteLine($"{foot} t={x.Bin * 30,4}s n={x.N,3} stride {x.Med,6:F1}  pd {x.PdMed,6:F1}");

                var toeIn = bins.MinBy(x => x.Med);
                // Normal = outside [toe-in − 1 min, toe-in + 2 min]: right after the toe-in minute both feet (computed from
                // independent sensors) drift back together over ~1.5 min — the walker relaxing, not sensor drift.
                var normal = bins.Where(x => (x.Bin < toeIn.Bin - 2 || x.Bin > toeIn.Bin + 4) && x.N >= 10).ToList();
                float normalMed = Median(normal.Select(x => x.Med));
                float worst = normal.Max(x => System.Math.Abs(x.Med - normalMed));
                int lastFourMin = fs.Count(s => s.PacketId > end - 240 * Rate);
                var o = outcomes.Where(x => x.Foot == foot).ToList();
                int invalid = o.Count(x => x.Reason == StepExclusionReason.StrideInvalid);
                int notStraight = o.Count(x => x.Reason == StepExclusionReason.StrideNotStraight);
                int turning = o.Count(x => x.Reason == StepExclusionReason.Turning);
                report.Add($"{foot}: normal median {normalMed:F1}, worst normal-bin deviation {worst:F1}, " +
                           $"toe-in bin t={toeIn.Bin * 30}s {toeIn.Med:F1} (change {toeIn.Med - normalMed:F1}), " +
                           $"steps in last 4 min {lastFourMin}, outcomes {o.Count}: emitted {o.Count(x => x.Emitted)}, " +
                           $"invalid {invalid}, notStraight {notStraight}, turning {turning}");

                if (worst >= 3f) failures.Add($"{foot}: normal 30 s medians vary by {worst:F1}° (limit 3°)");
                if (toeIn.Med - normalMed > -10f) failures.Add($"{foot}: toe-in change only {toeIn.Med - normalMed:F1}°");
                if (lastFourMin < 50) failures.Add($"{foot}: only {lastFourMin} stride FPAs in the last 4 min");
                if (invalid >= 0.2 * o.Count) failures.Add($"{foot}: {invalid}/{o.Count} strides failed G1–G4/G8");
            }
            foreach (var line in report) _out.WriteLine(line);
            Assert.True(failures.Count == 0, string.Join("; ", failures));
        }

        private static float Median(IEnumerable<float> xs)
        {
            var a = xs.OrderBy(v => v).ToArray();
            return a.Length == 0 ? float.NaN : a[a.Length / 2];
        }
    }
}
