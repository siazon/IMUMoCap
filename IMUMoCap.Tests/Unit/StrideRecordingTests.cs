// IMUMoCap.Tests/Unit/StrideRecordingTests.cs
using System;
using System.IO;
using IMUMoCap.Methods;
using IMUMoCap.Pipeline.Models;
using Xunit;

namespace IMUMoCap.Tests.Unit
{
    /// <summary>步幅法的记录:两种 FPA 都写入 Diagnostics,新的排除原因进入 QoE 统计(docs/stride_fpa_design.md)。</summary>
    public class StrideRecordingTests : IDisposable
    {
        private readonly string _participantId = "T_" + Guid.NewGuid().ToString("N")[..8];
        private readonly ExperimentRecorder _recorder = new();

        private string DataDir => Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Data", $"P{_participantId}");

        public void Dispose()
        {
            _recorder.Dispose();
            if (Directory.Exists(DataDir)) Directory.Delete(DataDir, recursive: true);
        }

        [Fact]
        public void DiagnosticsCsv_HasBothMethodsPerFoot_AfterTheExistingFpaColumns()
        {
            Assert.Contains("FPA_Quality,FPA_Left_PdDeg,FPA_Left_StrideDeg,FPA_Right_PdDeg,FPA_Right_StrideDeg,Stage,Attempt",
                            DiagnosticsRow.CsvHeader);

            var row = new DiagnosticsRow
            {
                FpaLeft_Deg = 5f, FpaLeft_Error = float.NaN, FpaRight_Deg = float.NaN, FpaRight_Error = float.NaN,
                FpaLeft_PdDeg = 4.5f, FpaLeft_StrideDeg = 5f, FpaRight_PdDeg = float.NaN, FpaRight_StrideDeg = -3.25f,
                Stage = "Baseline",
            };
            string csv = row.ToCsvRow();
            Assert.Equal(DiagnosticsRow.CsvHeader.Split(',').Length, csv.Split(',').Length);
            Assert.EndsWith(",4.50,5.00,,-3.25,Baseline,1", csv);
        }

        [Fact]
        public void StrideExclusions_AreCountedInStageStats()
        {
            _recorder.StartCondition(_participantId, "EF");
            _recorder.NoteStepOutcome("Baseline", true, null);
            _recorder.NoteStepOutcome("Baseline", false, StepExclusionReason.StrideInvalid);
            _recorder.NoteStepOutcome("Baseline", false, StepExclusionReason.StrideNotStraight);
            _recorder.NoteStepOutcome("Baseline", false, StepExclusionReason.StrideNotStraight);

            var s = _recorder.GetStageStats("Baseline")!;
            Assert.Equal(1, s.ExcludedStrideInvalid);
            Assert.Equal(2, s.ExcludedStrideNotStraight);
            Assert.Equal(4, s.TotalAttempted);
        }
    }
}
