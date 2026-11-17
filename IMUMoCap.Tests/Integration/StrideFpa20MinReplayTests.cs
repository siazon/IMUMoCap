// IMUMoCap.Tests/Integration/StrideFpa20MinReplayTests.cs
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using IMUMoCap.Methods;
using IMUMoCap.Pipeline;
using Xunit;
using Xunit.Abstractions;

namespace IMUMoCap.Tests.Integration
{
    /// <summary>
    /// 单份录制的回放报告(只输出,不判通过与否),阶段时间点从同名 Events 读取:
    /// 每分钟 / 每阶段两种方法的 FPA;步态指标(步长、摆动时长、步频、FPA 离散度);开头校准站姿与结尾静止站姿间各传感器航向变化。
    /// - Pl89 2026-11-16:~20 分钟验证(自然 + 内八 + 外八)
    /// - Po9  2026-11-17:完整实验流程(每 block 50 步),参与者全程未看反馈
    /// </summary>
    [Trait("Category", "LongReplay")]
    public class StrideFpa20MinReplayTests
    {
        private const int Rate = 100;
        private readonly ITestOutputHelper _out;
        public StrideFpa20MinReplayTests(ITestOutputHelper o) => _out = o;

        private static string DocsDir([CallerFilePath] string f = "")
            => Path.GetFullPath(Path.Combine(Path.GetDirectoryName(f)!, "..", "..", "docs"));

        private sealed record Step(long Pkt, string Stage, string Foot, float Stride, float Pd);

        [Theory]
        [InlineData("20261116_143722")]
        [InlineData("20261117_121715")]
        [InlineData("20261117_144014")]   // Pf2: IF, 40 s Baseline only
        [InlineData("20261117_130600")]   // Ps2: 1-minute straight-feet test (feet deliberately along the path, stand on parallel tape)
        public void Report(string stamp)
        {
            string samplesPath = Path.Combine(DocsDir(), $"ImuSamples_{stamp}.csv");
            string eventsPath = Path.Combine(DocsDir(), $"Diagnostics_{stamp}_Events.csv");
            if (!File.Exists(samplesPath) || !File.Exists(eventsPath)) { _out.WriteLine($"SKIPPED: {stamp} not found"); return; }

            var ev = File.ReadAllLines(eventsPath).Skip(1).Select(l => l.Split(',')).Where(c => c.Length >= 7)
                .Select(c => (Type: c[1], Pkt: long.Parse(c[2], CultureInfo.InvariantCulture), Stage: c[4])).ToList();
            long calStart = ev.First(e => e.Type == "CalibrationStart").Pkt;
            long calDone = ev.First(e => e.Type == "CalibrationDone").Pkt;
            var stageStarts = ev.Where(e => e.Type == "StageStart" && e.Stage != "Baseline").ToList();
            long? endPkt = ev.Where(e => e.Type == "End").Select(e => (long?)e.Pkt).FirstOrDefault();

            var bundles = CsvUtil.GroupIntoBundles(new CsvUtil().ReadImuSamplesCsv(samplesPath));
            var byId = bundles.ToDictionary(b => b.PacketId);
            var p = new GaitPipeline { SampleRateHz = Rate, UseStrideDirection = true, CalibrationArmed = true };
            var steps = new List<Step>();
            var zeros = new List<string>();
            var strides = new Dictionary<string, List<(long Pkt, string Stage, StrideResult S)>> { ["L"] = new(), ["R"] = new() };
            var trackers = new Dictionary<string, StrideDirectionTracker> { ["L"] = new(), ["R"] = new() };
            string stage = "Baseline";
            p.OnStrideZeroEstimated += (f, o, sd) => zeros.Add($"{f} {o:F1}±{sd:F1}");
            p.OnStrideZeroCrossCheck += (f, diff, usedB) => zeros.Add($"{f} A−B {diff:F1} → {(usedB ? "B" : "A")}");
            p.OnCalibrationStateChanged += s => { if (s == CalibrationState.Completed) p.StartBaseline(); };
            p.OnDiagnosticsFrame += d =>
            {
                if (!float.IsNaN(d.FpaLeft_StrideDeg) || !float.IsNaN(d.FpaLeft_PdDeg)) steps.Add(new(d.PacketId, stage, "L", d.FpaLeft_StrideDeg, d.FpaLeft_PdDeg));
                if (!float.IsNaN(d.FpaRight_StrideDeg) || !float.IsNaN(d.FpaRight_PdDeg)) steps.Add(new(d.PacketId, stage, "R", d.FpaRight_StrideDeg, d.FpaRight_PdDeg));
                var b = byId[d.PacketId];
                if (trackers["L"].Update(d.PacketId, d.LeftStance, b.LeftFoot!.FreeAcceleration) is StrideResult sl) strides["L"].Add((d.PacketId, stage, sl));
                if (trackers["R"].Update(d.PacketId, d.RightStance, b.RightFoot!.FreeAcceleration) is StrideResult sr) strides["R"].Add((d.PacketId, stage, sr));
            };
            bool cal = false;
            int next = 0;
            foreach (var b in bundles)
            {
                if (!cal && b.PacketId >= calStart) cal = p.TriggerCalibration();
                while (next < stageStarts.Count && b.PacketId >= stageStarts[next].Pkt)
                {
                    stage = stageStarts[next++].Stage;
                    if (stage == "Training1")
                    {
                        if (p.InBaseline) p.FinalizeBaseline();
                        if (p.BaselineProfile != null) p.StartTraining();
                    }
                }
                if (endPkt is long ep && b.PacketId > ep) stage = "AfterEnd";
                p.Process(b);
            }
            _out.WriteLine($"zeros: {string.Join(", ", zeros)}   (recording {bundles[0].PacketId}..{bundles[^1].PacketId}, calibration {calStart}..{calDone})");

            // per minute
            _out.WriteLine(" min | L n  stride     pd  | R n  stride     pd   | stage");
            int maxMin = (int)((bundles[^1].PacketId - calDone) / (60 * Rate));
            for (int m = 0; m <= maxMin; m++)
            {
                long a = calDone + m * 60L * Rate, e = a + 60L * Rate;
                string Cell(string foot)
                {
                    var s = steps.Where(x => x.Foot == foot && x.Pkt >= a && x.Pkt < e).ToList();
                    return $"{s.Count(x => !float.IsNaN(x.Stride)),3} {Med(s.Select(x => x.Stride)),6:F1} {Med(s.Select(x => x.Pd)),6:F1}";
                }
                string st = string.Join("/", steps.Where(x => x.Pkt >= a && x.Pkt < e).Select(x => x.Stage).Distinct());
                _out.WriteLine($" {m,3} | {Cell("L")} | {Cell("R")} | {st}");
            }

            // per stage: FPA level + spread, gait metrics
            _out.WriteLine("stage      | foot | stride FPA med (IQR)  n | pd med | stride len med  swing s  | landings/min");
            foreach (var stg in steps.Select(x => x.Stage).Distinct())
                foreach (var foot in new[] { "L", "R" })
                {
                    var s = steps.Where(x => x.Stage == stg && x.Foot == foot).ToList();
                    var fs = s.Select(x => x.Stride).Where(v => !float.IsNaN(v)).OrderBy(v => v).ToArray();
                    var st = strides[foot].Where(x => x.Stage == stg).Select(x => x.S).ToList();
                    var valid = st.Where(StrideGate.IsIntegrationValid).ToList();
                    float minutes = st.Count < 2 ? float.NaN
                        : (strides[foot].Where(x => x.Stage == stg).Max(x => x.Pkt) - strides[foot].Where(x => x.Stage == stg).Min(x => x.Pkt)) / (60f * Rate);
                    string iqr = fs.Length >= 4 ? $"{fs[fs.Length / 4]:F1}..{fs[fs.Length * 3 / 4]:F1}" : "-";
                    _out.WriteLine($"{stg,-10} | {foot}    | {Med(fs),6:F1} ({iqr,-12}) {fs.Length,3} | {Med(s.Select(x => x.Pd)),6:F1} | " +
                                   $"{Med(valid.Select(x => x.LengthM)),5:F2} m  {Med(valid.Select(x => x.SwingSec)),5:F2} s | {st.Count / minutes,5:F0}");
                }

            // stands
            var calFrames = bundles.Where(b => b.IsComplete && b.PacketId >= calStart && b.PacketId <= calDone).ToList();
            var endStand = new List<ImuFrameBundle>();
            for (int i = bundles.Count - 1; i >= 0; i--)
            {
                var b = bundles[i];
                if (!b.IsComplete) continue;
                bool still = new[] { b.Pelvis!, b.LeftFoot!, b.RightFoot! }.All(f => f.RateOfTurn.Length() < 0.3f);
                if (still) endStand.Add(b);
                else if (endStand.Count >= 200) break;
                else endStand.Clear();
            }
            if (endStand.Count >= 200)
            {
                var y0 = Headings(calFrames); var y1 = Headings(endStand);
                float dP = Wrap(y1.P - y0.P), dL = Wrap(y1.L - y0.L), dR = Wrap(y1.R - y0.R);
                _out.WriteLine($"end stand: {endStand.Count / (float)Rate:F1} s ending at {(endStand[0].PacketId - calDone) / (float)Rate / 60f:F1} min");
                _out.WriteLine($"heading change (end − start): pelvis {dP:F1}, left {dL:F1}, right {dR:F1}; " +
                               $"relative: left−pelvis {Wrap(dL - dP):F1}, right−pelvis {Wrap(dR - dP):F1}, left−right {Wrap(dL - dR):F1}");
            }
            else _out.WriteLine("end stand: none ≥ 2 s found");
        }

        private static (float P, float L, float R) Headings(List<ImuFrameBundle> fs)
            => (CircMean(fs.Select(b => Yaw(b.Pelvis!.Quaternion))), CircMean(fs.Select(b => Yaw(b.LeftFoot!.Quaternion))),
                CircMean(fs.Select(b => Yaw(b.RightFoot!.Quaternion))));

        // same yaw definition as FpaEngine.ExtractYaw, in degrees
        private static float Yaw(Quaternion q)
            => System.MathF.Atan2(2f * (q.W * q.Z + q.X * q.Y), 1f - 2f * (q.Y * q.Y + q.Z * q.Z)) * 180f / System.MathF.PI;

        private static float CircMean(IEnumerable<float> deg)
        {
            double s = 0, c = 0;
            foreach (var d in deg) { s += System.Math.Sin(d * System.Math.PI / 180); c += System.Math.Cos(d * System.Math.PI / 180); }
            return (float)(System.Math.Atan2(s, c) * 180 / System.Math.PI);
        }

        private static float Wrap(float d) { while (d > 180) d -= 360; while (d < -180) d += 360; return d; }

        private static float Med(IEnumerable<float> xs)
        {
            var a = xs.Where(v => !float.IsNaN(v)).OrderBy(v => v).ToArray();
            return a.Length == 0 ? float.NaN : a[a.Length / 2];
        }
    }
}
