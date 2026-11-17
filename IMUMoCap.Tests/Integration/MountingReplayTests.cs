// IMUMoCap.Tests/Integration/MountingReplayTests.cs
using System;
using System.Collections.Generic;
using System.Numerics;
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
    /// 佩戴检查在真实录制上的表现:正常佩戴的整场实验不得被拒、不得报警;
    /// Pf2(2026-11-17 14:40)骨盆 IMU 全程平放在桌上,校准必须被拒。
    /// </summary>
    [Trait("Category", "LongReplay")]
    public class MountingReplayTests
    {
        private readonly ITestOutputHelper _out;
        public MountingReplayTests(ITestOutputHelper o) => _out = o;

        private static string DocsDir([CallerFilePath] string f = "")
            => Path.GetFullPath(Path.Combine(Path.GetDirectoryName(f)!, "..", "..", "docs"));

        /// <summary>Alerts are those raised while the experiment was still running (before RetentionDone, if any);
        /// afterwards the participant walks around filling in questionnaires and the app doesn't pause any more.</summary>
        private (List<string> Rejected, List<MountingAlert> Alerts, bool Calibrated)? Replay(string stamp, Action<ImuFrameBundle, long>? tamper = null)
        {
            string samples = Path.Combine(DocsDir(), $"ImuSamples_{stamp}.csv");
            string events = Path.Combine(DocsDir(), $"Diagnostics_{stamp}_Events.csv");
            if (!File.Exists(samples) || !File.Exists(events)) { _out.WriteLine($"SKIPPED: {stamp} not found"); return null; }

            var ev = File.ReadAllLines(events).Skip(1).Select(l => l.Split(',')).Where(c => c.Length >= 7).ToList();
            long calStart = long.Parse(ev.First(c => c[1] == "CalibrationStart")[2], CultureInfo.InvariantCulture);
            long doneAt = ev.Where(c => c[1] == "RetentionDone").Select(c => long.Parse(c[2], CultureInfo.InvariantCulture)).DefaultIfEmpty(long.MaxValue).First();
            var p = new GaitPipeline { SampleRateHz = 100, UseStrideDirection = true, CalibrationArmed = true };
            var rejected = new List<string>();
            var alerts = new List<MountingAlert>();
            bool calibrated = false;
            long pkt = 0;
            p.OnMountingRejected += rejected.Add;
            p.OnMountingAlert += a =>
            {
                _out.WriteLine($"  alert at {(pkt - calStart) / 6000f:F1} min: {a.Role} {a.Level} {a.ShiftDeg:F1}°{(pkt >= doneAt ? " (after RetentionDone)" : "")}");
                if (pkt < doneAt) alerts.Add(a);
            };
            p.OnCalibrationStateChanged += s =>
            {
                if (s == CalibrationState.Completed) { calibrated = true; p.StartBaseline(); }
            };
            bool triggered = false;
            foreach (var b in CsvUtil.GroupIntoBundles(new CsvUtil().ReadImuSamplesCsv(samples)))
            {
                if (!triggered && b.PacketId >= calStart) triggered = p.TriggerCalibration();
                pkt = b.PacketId;
                tamper?.Invoke(b, calStart);
                p.Process(b);
            }
            _out.WriteLine($"{stamp}: calibrated={calibrated} rejected=[{string.Join(" | ", rejected)}] " +
                           $"alerts=[{string.Join(", ", alerts.Select(a => $"{a.Role} {a.Level} {a.ShiftDeg:F1}°"))}]");
            return (rejected, alerts, calibrated);
        }

        [Theory]
        [InlineData("20261116_143722", true)]    // Pl89, ~21 min: pelvis + right foot both pass 12° for a few seconds at 15.8 min
                                                 // (Training2; the session mixed natural / toe-in / toe-out walking) and come back — a warning, never a pause
        [InlineData("20261117_121715", false)]   // Po9, full experiment
        [InlineData("20261117_130600", false)]   // Ps2, straight-line test
        public void CorrectlyWorn_CalibratesAndNeverPauses(string stamp, bool warningSeen)
        {
            if (Replay(stamp) is not { } r) return;
            Assert.True(r.Calibrated);
            Assert.Empty(r.Rejected);
            Assert.DoesNotContain(r.Alerts, a => a.Level == MountingLevel.Pause);
            Assert.Equal(warningSeen, r.Alerts.Count > 0);
        }

        [Fact]
        public void LeftFootImuTiltedOnTheShoe_MidExperiment_Pauses()
        {
            // From 5 min after calibration the left IMU sits 25° pitched on the shoe (rotation in the sensor frame).
            var slip = Quaternion.CreateFromAxisAngle(Vector3.UnitY, 25f * MathF.PI / 180f);
            var r = Replay("20261117_121715", (b, calStart) =>
            {
                if (b.PacketId >= calStart + 5 * 6000 && b.LeftFoot is { } l) l.Quaternion = l.Quaternion * slip;
            });
            if (r is not { } res) return;
            Assert.Contains(res.Alerts, a => a.Role == ImuRole.Left && a.Level == MountingLevel.Pause);
            Assert.DoesNotContain(res.Alerts, a => a.Role != ImuRole.Left && a.Level == MountingLevel.Pause);
        }

        [Fact]
        public void PelvisLeftOnTheTable_CalibrationRejected()
        {
            if (Replay("20261117_144014") is not { } r) return;
            Assert.False(r.Calibrated);
            Assert.Contains("Pelvis", Assert.Single(r.Rejected));
        }
    }
}
