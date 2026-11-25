// IMUMoCap.Tests/Integration/BaselinePostZeroReplayTests.cs
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
    /// 回放 P4 EF(2026-11-19)的 Baseline:步幅法模式下 Baseline 只数零点之后的步,每脚 50 步就结束,
    /// 均值只由步幅法 FPA 组成(录制时零点前的 PD 步被混入,Pk12 偏了 2.3°)。
    /// 录制在 IMUMoCap/bin/Debug/net8.0-windows/Data/P4 下,未纳入 git;不存在时直接返回。
    /// </summary>
    [Trait("Category", "LongReplay")]
    public class BaselinePostZeroReplayTests
    {
        private const long CalibrationStartPacket = 374892;   // Events: CalibrationStart
        private readonly ITestOutputHelper _out;
        public BaselinePostZeroReplayTests(ITestOutputHelper o) => _out = o;

        private static string SamplesPath([CallerFilePath] string f = "")
            => Path.GetFullPath(Path.Combine(Path.GetDirectoryName(f)!, "..", "..", "IMUMoCap", "bin", "Debug", "net8.0-windows",
                                             "Data", "P4", "ImuSamples_20261119_142425.csv"));

        [Fact]
        public void P4_EF_Baseline_Counts50PostZeroStrideSteps_AndExcludesPdSteps()
        {
            string path = SamplesPath();
            if (!File.Exists(path)) { _out.WriteLine($"SKIPPED: {path} not found"); return; }

            var bundles = CsvUtil.GroupIntoBundles(new CsvUtil().ReadImuSamplesCsv(path));
            var p = new GaitPipeline { SampleRateHz = 100, UseStrideDirection = true, CalibrationArmed = true };
            var zeroed = new HashSet<string>();
            var stride = new Dictionary<string, List<float>> { ["L"] = new(), ["R"] = new() };
            int pdStepsBeforeZero = 0;
            p.OnStrideZeroEstimated += (foot, _, _) => zeroed.Add(foot);
            p.OnDiagnosticsFrame += d =>
            {
                if (!p.InBaseline) return;
                foreach (var (foot, fb, st) in new[] { ("L", d.FpaLeft_Deg, d.FpaLeft_StrideDeg), ("R", d.FpaRight_Deg, d.FpaRight_StrideDeg) })
                {
                    if (float.IsNaN(fb)) continue;
                    // the step that decides the zero already has a stride FPA, but its feedback is still the PD FPA
                    if (!zeroed.Contains(foot) || fb != st) pdStepsBeforeZero++;
                    else stride[foot].Add(st);
                }
            };
            p.OnCalibrationStateChanged += s => { if (s == CalibrationState.Completed) p.StartBaseline(); };

            bool calTriggered = false;
            foreach (var b in bundles)
            {
                if (!calTriggered && b.PacketId >= CalibrationStartPacket) calTriggered = p.TriggerCalibration();
                p.Process(b);
                if (p.BaselineProfile != null) break;
            }

            var bp = p.BaselineProfile;
            Assert.NotNull(bp);
            _out.WriteLine($"PD feedback steps before the zero {pdStepsBeforeZero}; profile L n={bp!.ValidSteps_L} μ={bp.MeanFpa_L:F2}, " +
                           $"R n={bp.ValidSteps_R} μ={bp.MeanFpa_R:F2}; stride-only means L {stride["L"].Take(bp.ValidSteps_L).Average():F2} " +
                           $"R {stride["R"].Take(bp.ValidSteps_R).Average():F2}");
            Assert.True(pdStepsBeforeZero > 0);                               // the recording does have PD steps before the zero
            Assert.Equal(50, System.Math.Min(bp.ValidSteps_L, bp.ValidSteps_R));
            Assert.Equal(stride["L"].Take(bp.ValidSteps_L).Average(), bp.MeanFpa_L, 3);
            Assert.Equal(stride["R"].Take(bp.ValidSteps_R).Average(), bp.MeanFpa_R, 3);
        }

        /// <summary>
        /// Pk12 EF(2026-11-19):TODO 1 就是在这份录制上发现的——两种方法差约 8°,录制时混入 PD 步后的 Baseline
        /// 为 L −6.97 / R −10.89(Meta.json,每脚 20 步)。录制时 Baseline 每脚 20 步,零点后到 StageEnd 只有约 33 s,
        /// 所以这里同样用 20 步,且只用 Baseline 阶段的数据(不能借 Training 的步)。
        /// </summary>
        [Fact]
        public void Pk12_EF_Baseline_ExcludesPdSteps_MeanIsStrideOnly()
        {
            const long pk12CalibrationStart = 79061;   // Events: 第二次 CalibrationStart(第一次因骨盆没戴好被拒)
            const long pk12BaselineEnd = 84246;        // Events: Baseline StageEnd
            string path = Path.Combine(Path.GetDirectoryName(SamplesPath())!, "..", "Pk12", "ImuSamples_20261119_132355.csv");
            if (!File.Exists(path)) { _out.WriteLine($"SKIPPED: {path} not found"); return; }

            var bundles = CsvUtil.GroupIntoBundles(new CsvUtil().ReadImuSamplesCsv(path));
            var p = new GaitPipeline { SampleRateHz = 100, UseStrideDirection = true, CalibrationArmed = true };
            p.Params.BaselineStepsPerFoot = 20;
            var zeroed = new HashSet<string>();
            var stride = new Dictionary<string, List<float>> { ["L"] = new(), ["R"] = new() };
            int pdStepsBeforeZero = 0;
            p.OnStrideZeroEstimated += (foot, _, _) => zeroed.Add(foot);
            p.OnDiagnosticsFrame += d =>
            {
                if (!p.InBaseline) return;
                foreach (var (foot, fb, st) in new[] { ("L", d.FpaLeft_Deg, d.FpaLeft_StrideDeg), ("R", d.FpaRight_Deg, d.FpaRight_StrideDeg) })
                {
                    if (float.IsNaN(fb)) continue;
                    if (!zeroed.Contains(foot) || fb != st) pdStepsBeforeZero++;
                    else stride[foot].Add(st);
                }
            };
            p.OnCalibrationStateChanged += s => { if (s == CalibrationState.Completed) p.StartBaseline(); };

            bool calTriggered = false;
            long lastPacket = 0;
            foreach (var b in bundles)
            {
                if (b.PacketId > pk12BaselineEnd) break;
                if (!calTriggered && b.PacketId >= pk12CalibrationStart) calTriggered = p.TriggerCalibration();
                p.Process(b);
                lastPacket = b.PacketId;
                if (p.BaselineProfile != null) break;
            }

            var bp = p.BaselineProfile;
            _out.WriteLine($"PD feedback steps before the zero {pdStepsBeforeZero}; stride steps L {stride["L"].Count} R {stride["R"].Count}; " +
                           $"stopped at packet {lastPacket}" + (bp == null ? "; no profile" :
                           $"; profile L n={bp.ValidSteps_L} μ={bp.MeanFpa_L:F2}, R n={bp.ValidSteps_R} μ={bp.MeanFpa_R:F2} (recorded mixed: L −6.97, R −10.89)"));
            Assert.NotNull(bp);                                              // 20 post-zero steps/foot fit inside the recorded Baseline
            Assert.True(pdStepsBeforeZero > 0);
            Assert.Equal(20, System.Math.Min(bp!.ValidSteps_L, bp.ValidSteps_R));
            Assert.Equal(stride["L"].Take(bp.ValidSteps_L).Average(), bp.MeanFpa_L, 3);
            Assert.Equal(stride["R"].Take(bp.ValidSteps_R).Average(), bp.MeanFpa_R, 3);
        }
    }
}
