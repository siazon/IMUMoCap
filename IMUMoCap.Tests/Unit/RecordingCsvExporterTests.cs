// IMUMoCap.Tests/Unit/RecordingCsvExporterTests.cs
using System;
using System.Collections.Generic;
using System.IO;
using IMUMoCap.Methods;
using IMUMoCap.Pipeline.Models;
using Xunit;

namespace IMUMoCap.Tests.Unit
{
    /// <summary>End Recording / Retention 结束时自动保存 ImuSamples、Diagnostics、Events 三个 CSV。</summary>
    public class RecordingCsvExporterTests : IDisposable
    {
        private readonly string _dir = Path.Combine(Path.GetTempPath(), "rce_" + Guid.NewGuid().ToString("N")[..8]);
        private readonly string _participantId = "T_" + Guid.NewGuid().ToString("N")[..8];
        private readonly ExperimentRecorder _recorder = new();

        private string DataDir => Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Data", $"P{_participantId}");

        public void Dispose()
        {
            _recorder.Dispose();
            if (Directory.Exists(_dir)) Directory.Delete(_dir, recursive: true);
            if (Directory.Exists(DataDir)) Directory.Delete(DataDir, recursive: true);
        }

        private static List<ImuSampleFrame> Samples(int n)
        {
            var list = new List<ImuSampleFrame>();
            for (int i = 0; i < n; i++) list.Add(new ImuSampleFrame { PacketId = 100 + i, Role = ImuRole.Pelvis });
            return list;
        }

        private static List<DiagnosticsRow> Rows(int n)
        {
            var list = new List<DiagnosticsRow>();
            for (int i = 0; i < n; i++) list.Add(new DiagnosticsRow { PacketId = 100 + i, Stage = "Baseline" });
            return list;
        }

        [Fact]
        public void Save_WritesThreeCsvsWithTheSameStamp()
        {
            _recorder.StartCondition(_participantId, "EF");
            _recorder.RecordEvent("CalibrationStart", "initial");

            var paths = RecordingCsvExporter.Save(_dir, "20261116_101500", Samples(3), Rows(2), _recorder);

            Assert.Equal(Path.Combine(_dir, "ImuSamples_20261116_101500.csv"), paths.ImuSamples);
            Assert.Equal(Path.Combine(_dir, "Diagnostics_20261116_101500.csv"), paths.Diagnostics);
            Assert.Equal(Path.Combine(_dir, "Diagnostics_20261116_101500_Events.csv"), paths.Events);
            Assert.Equal(1 + 3, File.ReadAllLines(paths.ImuSamples).Length);
            var diag = File.ReadAllLines(paths.Diagnostics);
            Assert.Equal(DiagnosticsRow.CsvHeader, diag[0]);
            Assert.Equal(1 + 2, diag.Length);
            Assert.Contains(File.ReadAllLines(paths.Events!), l => l.Contains("CalibrationStart"));
        }

        [Fact]
        public void SavingAgainWithTheSameStamp_OverwritesWithTheLongerData()
        {
            _recorder.StartCondition(_participantId, "EF");
            RecordingCsvExporter.Save(_dir, "s", Samples(3), Rows(2), _recorder);      // Retention done
            var p = RecordingCsvExporter.Save(_dir, "s", Samples(5), Rows(4), _recorder); // End Recording
            Assert.Equal(3, Directory.GetFiles(_dir).Length);
            Assert.Equal(1 + 5, File.ReadAllLines(p.ImuSamples).Length);
            Assert.Equal(1 + 4, File.ReadAllLines(p.Diagnostics).Length);
        }

        [Fact]
        public void NoEvents_SkipsTheEventsFile()
        {
            var p = RecordingCsvExporter.Save(_dir, "s", Samples(1), Rows(1), _recorder);   // no condition → no events
            Assert.Null(p.Events);
            Assert.Equal(2, Directory.GetFiles(_dir).Length);
        }
    }
}
