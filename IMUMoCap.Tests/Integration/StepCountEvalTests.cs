// IMUMoCap.Tests/Integration/StepCountEvalTests.cs
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Runtime.CompilerServices;
using Xunit;
using Xunit.Abstractions;

namespace IMUMoCap.Tests.Integration
{
    /// <summary>
    /// Baseline / Retention 每只脚取 20、50、100 步的差别(只输出报告,不判通过与否)。
    /// 数据:P4 2026-11-19 EF / IF,两者 Baseline、Retention 都是每只脚 100 步。读 Diagnostics 里实时输出的步幅法 FPA:
    /// Baseline 只取零点定下后的步(之前的是 PD FPA),Retention 取每只脚最初 100 步(IF 在 RetentionDone 后还在录)。
    /// 为了不让"100 步"自己当标准答案,精度用相邻、互不重叠的 N 步窗口之间的差异来估(包含步与步之间的相关和慢漂移)。
    /// 录制在 IMUMoCap/bin/Debug/net8.0-windows/Data/P4 下,未纳入 git;不存在时直接返回。
    /// </summary>
    [Trait("Category", "LongReplay")]
    public class StepCountEvalTests
    {
        private const int Rate = 100;
        private readonly ITestOutputHelper _out;
        public StepCountEvalTests(ITestOutputHelper o) => _out = o;

        private static string DataDir([CallerFilePath] string f = "")
            => Path.GetFullPath(Path.Combine(Path.GetDirectoryName(f)!, "..", "..", "IMUMoCap", "bin", "Debug", "net8.0-windows", "Data", "P4"));

        private sealed record Series(string Cond, string Stage, string Foot, long StageStart, List<(long Pkt, double Fpa)> Steps);

        [Fact]
        public void P4_Baseline_And_Retention_20_vs_50_vs_100_Steps()
        {
            var all = new List<Series>();
            foreach (var (cond, stamp) in new[] { ("EF", "20261119_142425"), ("IF", "20261119_145438") })
            {
                string diag = Path.Combine(DataDir(), $"Diagnostics_{stamp}.csv");
                string events = Path.Combine(DataDir(), $"Diagnostics_{stamp}_Events.csv");
                if (!File.Exists(diag) || !File.Exists(events)) { _out.WriteLine($"SKIPPED: {diag} not found"); return; }

                var starts = File.ReadAllLines(events).Skip(1).Select(l => l.Split(','))
                    .Where(c => c.Length >= 7 && c[1] == "StageStart")
                    .ToDictionary(c => c[4], c => long.Parse(c[2], CultureInfo.InvariantCulture));

                var rows = ReadDiagnostics(diag);
                foreach (var stage in new[] { "Baseline", "Retention" })
                    foreach (var foot in new[] { "L", "R" })
                    {
                        var steps = rows.Where(r => r.Stage == stage)
                            .Select(r => (r.Pkt, Fb: foot == "L" ? r.FbL : r.FbR, Stride: foot == "L" ? r.StrideL : r.StrideR))
                            .Where(x => !double.IsNaN(x.Fb) && !double.IsNaN(x.Stride) && Math.Abs(x.Fb - x.Stride) < 1e-3)
                            .Select(x => (x.Pkt, Fpa: x.Stride)).Take(100).ToList();
                        all.Add(new Series(cond, stage, foot, starts[stage], steps));
                    }
            }

            _out.WriteLine("== 1. Per series: means of the first N steps, 20-step block means, linear trend");
            foreach (var s in all)
            {
                var x = s.Steps.Select(t => t.Fpa).ToArray();
                double m100 = x.Average();
                var blocks = Blocks(x, 20).Select(b => b.Average()).ToArray();
                var (slope, slopeSe) = Trend(x);
                _out.WriteLine($"{s.Cond} {s.Stage,-9} {s.Foot}: n {x.Length,3}  sd {Sd(x):F1}  " +
                               $"μ20 {Mean(x, 20),6:F2}  μ50 {Mean(x, 50),6:F2}  μall {m100,6:F2}  " +
                               $"(μ20−μall {Mean(x, 20) - m100,5:F2}, μ50−μall {Mean(x, 50) - m100,5:F2})  " +
                               $"20-step blocks [{string.Join(", ", blocks.Select(b => b.ToString("F1", CultureInfo.InvariantCulture)))}]  " +
                               $"trend {slope:F2}±{slopeSe:F2} °/100 steps");
            }

            _out.WriteLine("\n== 2. Precision of an N-step mean, pooled over EF/IF × L/R (empirical = spread between adjacent independent windows)");
            foreach (var stage in new[] { "Baseline", "Retention" })
            {
                var ss = all.Where(s => s.Stage == stage).Select(s => s.Steps.Select(t => t.Fpa).ToArray()).ToList();
                double sdPooled = Math.Sqrt(ss.Average(x => Sd(x) * Sd(x)));
                // N = 20: variance of the complete 20-step block means within each series
                double se20 = Math.Sqrt(ss.Average(x => Var(Blocks(x, 20).Select(b => b.Average()).ToArray())));
                // N = 50 (baseline has 93–98 post-zero steps: two halves of n/2): half-difference² / 2 estimates SE² of a half
                var halves = ss.Select(x => { int h = Math.Min(50, x.Length / 2); return (H: h, D: x.Take(h).Average() - x.Skip(h).Take(h).Average()); }).ToList();
                double se50 = Math.Sqrt(halves.Average(t => t.D * t.D / 2));
                double infl = se20 * se20 / (sdPooled * sdPooled / 20);
                _out.WriteLine($"{stage}: per-step sd {sdPooled:F2}; " +
                               $"SE(20) empirical {se20:F2} vs independent-steps {sdPooled / Math.Sqrt(20):F2} (variance inflation ×{infl:F1}); " +
                               $"SE({halves.Min(t => t.H)}–{halves.Max(t => t.H)}) empirical {se50:F2} (from {halves.Count} half-differences: " +
                               $"{string.Join(", ", halves.Select(t => t.D.ToString("F2", CultureInfo.InvariantCulture)))}) vs independent {sdPooled / Math.Sqrt(50):F2}; " +
                               $"projected with the inflation: SE(50) {Math.Sqrt(infl) * sdPooled / Math.Sqrt(50):F2}, SE(100) {Math.Sqrt(infl) * sdPooled / Math.Sqrt(100):F2}");
            }

            _out.WriteLine("\n== 3. Retention change Δ = Retention − Baseline (both with the first N steps)");
            foreach (var cond in new[] { "EF", "IF" })
                foreach (var foot in new[] { "L", "R" })
                {
                    var b = all.Single(s => s.Cond == cond && s.Stage == "Baseline" && s.Foot == foot).Steps.Select(t => t.Fpa).ToArray();
                    var r = all.Single(s => s.Cond == cond && s.Stage == "Retention" && s.Foot == foot).Steps.Select(t => t.Fpa).ToArray();
                    _out.WriteLine($"{cond} {foot}: " + string.Join("  ", new[] { 20, 50, 100 }.Select(n => $"Δ{n} {Mean(r, n) - Mean(b, n),6:F2}")) +
                                   $"   retention 20-step blocks minus baseline μall [{string.Join(", ", Blocks(r, 20).Select(x => (x.Average() - b.Average()).ToString("F1", CultureInfo.InvariantCulture)))}]");
                }

            _out.WriteLine("\n== 4. Walking time from stage start until both feet have N steps (s)");
            foreach (var cond in new[] { "EF", "IF" })
                foreach (var stage in new[] { "Baseline", "Retention" })
                {
                    var pair = all.Where(s => s.Cond == cond && s.Stage == stage).ToList();
                    _out.WriteLine($"{cond} {stage,-9}: " + string.Join("  ", new[] { 20, 50, 100 }.Select(n =>
                        pair.All(s => s.Steps.Count >= n)
                            ? $"N{n} {pair.Max(s => s.Steps[n - 1].Pkt - s.StageStart) / (double)Rate,5:F0}"
                            : $"N{n}   n/a")));
                }
        }

        /// <summary>
        /// 合适的 Baseline 步数:P3(每段 50 步,零点后 42–45 步)+ P4(每段 100 步,零点后 93–97 步),共 8 条 Baseline 序列。
        /// 看三件事:N 步均值的 SE 随 N 怎么降(相邻不重叠窗口 + 合并自相关两种估法);开头几步是否有系统偏差(丢前 k 步的效果);
        /// 以及凑够 N 步要走多久。
        /// </summary>
        [Fact]
        public void P3_P4_Baseline_Step_Count()
        {
            var sets = new[] { ("P3", "EF", "20261117_155209"), ("P3", "IF", "20261117_160036"),
                               ("P4", "EF", "20261119_142425"), ("P4", "IF", "20261119_145438") };
            var series = new List<(string Name, double[] X, double[] T)>();
            foreach (var (p, cond, stamp) in sets)
            {
                string dir = Path.Combine(DataDir(), "..", p);
                string diag = Path.Combine(dir, $"Diagnostics_{stamp}.csv");
                string events = Path.Combine(dir, $"Diagnostics_{stamp}_Events.csv");
                if (!File.Exists(diag) || !File.Exists(events)) { _out.WriteLine($"SKIPPED: {diag} not found"); return; }
                long start = File.ReadAllLines(events).Skip(1).Select(l => l.Split(','))
                    .Where(c => c.Length >= 7 && c[1] == "StageStart" && c[4] == "Baseline").Select(c => long.Parse(c[2], CultureInfo.InvariantCulture)).First();
                var rows = ReadDiagnostics(diag).Where(r => r.Stage == "Baseline").ToList();
                foreach (var foot in new[] { "L", "R" })
                {
                    var st = rows.Select(r => (r.Pkt, Fb: foot == "L" ? r.FbL : r.FbR, S: foot == "L" ? r.StrideL : r.StrideR))
                        .Where(x => !double.IsNaN(x.Fb) && !double.IsNaN(x.S) && Math.Abs(x.Fb - x.S) < 1e-3).ToList();
                    series.Add(($"{p} {cond} {foot}", st.Select(x => x.S).ToArray(), st.Select(x => (x.Pkt - start) / (double)Rate).ToArray()));
                }
            }

            _out.WriteLine("== A. Series (post-zero stride steps): n, sd, 10-step block means minus own mean, trend");
            foreach (var (name, x, t) in series)
            {
                var (sl, se) = Trend(x);
                _out.WriteLine($"{name}: n {x.Length,3}  sd {Sd(x):F1}  first step at {t[0],4:F0}s  " +
                               $"10-blocks [{string.Join(", ", Blocks(x, 10).Select(b => (b.Average() - x.Average()).ToString("F1", CultureInfo.InvariantCulture)))}]  trend {sl:F1}±{se:F1}°/100");
            }

            _out.WriteLine("\n== B. Warm-up: mean of steps 1–k minus mean of steps k+1..min(n,45) (P3 and P4 on the same 45-step window)");
            foreach (int k in new[] { 5, 10, 15, 20 })
            {
                var d = series.Select(s => s.X.Take(k).Average() - s.X.Skip(k).Take(45 - k).Average()).ToArray();
                _out.WriteLine($"k {k,2}: [{string.Join(", ", d.Select(v => v.ToString("F1", CultureInfo.InvariantCulture)))}]  mean {d.Average():F2}  SE {Sd(d) / Math.Sqrt(d.Length):F2}  negative {d.Count(v => v < 0)}/{d.Length}");
            }

            _out.WriteLine("\n== C. SE of an N-step mean (per series sd-normalised then rescaled by pooled sd)");
            double sdPool = Math.Sqrt(series.Average(s => Sd(s.X) * Sd(s.X)));
            // pooled autocorrelation from detrended-by-mean series, lags 1..15
            var rho = Enumerable.Range(1, 15).Select(lag =>
            {
                double num = 0, den = 0;
                foreach (var (_, x, _) in series)
                {
                    double m = x.Average();
                    for (int i = 0; i + lag < x.Length; i++) num += (x[i] - m) * (x[i + lag] - m);
                    den += x.Sum(v => (v - m) * (v - m));
                }
                return num / den;
            }).ToArray();
            _out.WriteLine($"pooled sd {sdPool:F2}; autocorrelation lag1..15 [{string.Join(", ", rho.Select(r => r.ToString("F2", CultureInfo.InvariantCulture)))}]");
            foreach (int n in new[] { 10, 15, 20, 25, 30, 40, 50, 60, 80, 100 })
            {
                // ACF model: Var(mean) = σ²/N · (1 + 2 Σ_{h<N} (1 − h/N) ρ_h), ρ beyond lag 15 taken as 0
                double f = 1 + 2 * Enumerable.Range(1, Math.Min(15, n - 1)).Sum(h => (1 - h / (double)n) * rho[h - 1]);
                double seAcf = sdPool * Math.Sqrt(f / n);
                // empirical: adjacent non-overlapping N-windows, Var(diff)/2, pooled over series that hold ≥ 2 windows
                var diffs = series.Where(s => s.X.Length >= 2 * n)
                    .SelectMany(s => Enumerable.Range(0, s.X.Length / n - 1).Select(i => s.X.Skip(i * n).Take(n).Average() - s.X.Skip((i + 1) * n).Take(n).Average())).ToArray();
                string emp = diffs.Length > 0 ? $"{Math.Sqrt(diffs.Average(d => d * d) / 2):F2} ({diffs.Length} pairs)" : "  n/a";
                _out.WriteLine($"N {n,3}: independent {sdPool / Math.Sqrt(n):F2}  ACF-model {seAcf:F2} (×{f:F2})  empirical {emp}");
            }

            _out.WriteLine("\n== D. Walking time from Baseline StageStart until both feet have N post-zero steps (s); first [P3 EF, P3 IF, P4 EF, P4 IF]");
            foreach (int n in new[] { 20, 30, 40, 50, 60, 80 })
                _out.WriteLine($"N {n,3}: [{string.Join(", ", Enumerable.Range(0, 4).Select(i => { var (l, r) = (series[2 * i], series[2 * i + 1]); return l.X.Length >= n && r.X.Length >= n ? Math.Max(l.T[n - 1], r.T[n - 1]).ToString("F0", CultureInfo.InvariantCulture) : "n/a"; }))}]");
        }

        private static List<(long Pkt, string Stage, double FbL, double StrideL, double FbR, double StrideR)> ReadDiagnostics(string path)
        {
            var rows = new List<(long, string, double, double, double, double)>();
            string[]? h = null;
            int iPkt = 0, iStage = 0, iFbL = 0, iStL = 0, iFbR = 0, iStR = 0;
            foreach (var line in File.ReadLines(path))
            {
                var c = line.Split(',');
                if (h == null)
                {
                    h = c.Select(x => x.Trim('﻿')).ToArray();
                    iPkt = Array.IndexOf(h, "PacketId"); iStage = Array.IndexOf(h, "Stage");
                    iFbL = Array.IndexOf(h, "FPA_Left_Deg"); iStL = Array.IndexOf(h, "FPA_Left_StrideDeg");
                    iFbR = Array.IndexOf(h, "FPA_Right_Deg"); iStR = Array.IndexOf(h, "FPA_Right_StrideDeg");
                    continue;
                }
                if (c.Length < h.Length) continue;
                if (c[iFbL].Length == 0 && c[iFbR].Length == 0) continue;
                rows.Add((long.Parse(c[iPkt], CultureInfo.InvariantCulture), c[iStage],
                          D(c[iFbL]), D(c[iStL]), D(c[iFbR]), D(c[iStR])));
            }
            return rows;
        }

        private static double D(string s) => s.Length == 0 ? double.NaN : double.Parse(s, CultureInfo.InvariantCulture);
        private static double Mean(double[] x, int n) => x.Take(n).Average();
        private static double Var(double[] x) { double m = x.Average(); return x.Sum(v => (v - m) * (v - m)) / (x.Length - 1); }
        private static double Sd(double[] x) => Math.Sqrt(Var(x));
        private static IEnumerable<double[]> Blocks(double[] x, int size)
            => Enumerable.Range(0, x.Length / size).Select(i => x.Skip(i * size).Take(size).ToArray());

        /// <summary>OLS slope in ° per 100 steps and its SE (SE assumes independent residuals, so it is optimistic).</summary>
        private static (double Slope, double Se) Trend(double[] y)
        {
            int n = y.Length;
            double mx = (n - 1) / 2.0, my = y.Average();
            double sxx = Enumerable.Range(0, n).Sum(i => (i - mx) * (i - mx));
            double b = Enumerable.Range(0, n).Sum(i => (i - mx) * (y[i] - my)) / sxx;
            double rss = Enumerable.Range(0, n).Sum(i => Math.Pow(y[i] - my - b * (i - mx), 2));
            return (b * 100, Math.Sqrt(rss / (n - 2) / sxx) * 100);
        }
    }
}
