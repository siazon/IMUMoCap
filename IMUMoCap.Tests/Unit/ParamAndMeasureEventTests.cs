// IMUMoCap.Tests/Unit/ParamAndMeasureEventTests.cs
using System;
using System.IO;
using System.Linq;
using IMUMoCap.Methods;
using IMUMoCap.Pipeline.Models;
using Xunit;

namespace IMUMoCap.Tests.Unit
{
    /// <summary>
    /// T13(docs/flow-review-2026-10-27.md Q2、Q6):录制中的 Measure 停止/启动和 Parameters 修改要写进事件日志,
    /// 带 PacketId,这样事后能解释\"为什么这段数据变了\"。
    /// </summary>
    public class ParamChangeTests
    {
        [Fact]
        public void Clone_IsIndependentOfTheOriginal()
        {
            var a = new PipelineParams { TargetStepsPerFoot = 5, StaticGyroThreshold = 0.4f };
            var b = a.Clone();

            a.TargetStepsPerFoot = 99;

            Assert.Equal(5, b.TargetStepsPerFoot);
            Assert.Equal(0.4f, b.StaticGyroThreshold);
        }

        [Fact]
        public void Diff_IsEmpty_WhenNothingChanged()
        {
            var a = new PipelineParams();
            Assert.Empty(PipelineParams.Diff(a, a.Clone()));
        }

        [Fact]
        public void Diff_ListsEachChangedParameter_WithOldAndNewValue()
        {
            var before = new PipelineParams();
            var after = before.Clone();
            after.TargetStepsPerFoot = 5;
            after.StaticGyroThreshold = 0.45f;

            var d = PipelineParams.Diff(before, after).OrderBy(c => c.Name).ToList();

            Assert.Equal(2, d.Count);
            Assert.Equal(new ParamChange("StaticGyroThreshold", "0.3", "0.45"), d[0]);
            Assert.Equal(new ParamChange("TargetStepsPerFoot", "100", "5"), d[1]);
        }

        [Fact]
        public void Diff_FormatsNumbersIndependentlyOfTheCurrentCulture()
        {
            var old = System.Globalization.CultureInfo.CurrentCulture;
            try
            {
                System.Globalization.CultureInfo.CurrentCulture = new System.Globalization.CultureInfo("de-DE");
                var before = new PipelineParams();
                var after = before.Clone();
                after.PdConfidenceThreshold = 0.55f;

                var change = Assert.Single(PipelineParams.Diff(before, after));

                Assert.Equal("0.7", change.Old);   // not "0,7"
                Assert.Equal("0.55", change.New);
            }
            finally { System.Globalization.CultureInfo.CurrentCulture = old; }
        }

        [Fact]
        public void Describe_IsTheTextWrittenIntoTheEventLog()
        {
            Assert.Equal("TargetStepsPerFoot=100->5", new ParamChange("TargetStepsPerFoot", "100", "5").Describe());
        }

        [Fact]
        public void Diff_CoversEveryPublicParameter_SoANewOneCannotBeForgotten()
        {
            var before = new PipelineParams();
            var after = before.Clone();
            foreach (var prop in typeof(PipelineParams).GetProperties())
            {
                if (prop.PropertyType == typeof(int)) prop.SetValue(after, (int)prop.GetValue(after)! + 1);
                else if (prop.PropertyType == typeof(float)) prop.SetValue(after, (float)prop.GetValue(after)! + 0.1f);
            }

            var names = PipelineParams.Diff(before, after).Select(c => c.Name).ToHashSet();
            var all = typeof(PipelineParams).GetProperties().Select(p => p.Name).ToHashSet();

            Assert.Equal(all, names);
        }
    }

    public class MeasurementEventTests : IDisposable
    {
        private readonly string _pid = "T_" + Guid.NewGuid().ToString("N")[..8];
        private readonly ExperimentRecorder _recorder = new();
        private string DataDir => Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Data", $"P{_pid}");

        public void Dispose()
        {
            _recorder.Dispose();
            if (Directory.Exists(DataDir)) Directory.Delete(DataDir, recursive: true);
        }

        [Fact]
        public void LeavingMeasuring_RecordsMeasureStop_WithThePacketIdOfTheLastSample()
        {
            _recorder.StartCondition(_pid, "EF");
            _recorder.LatestPacketId = 777;

            _recorder.RecordMeasurementTransition(States.MEASURING, States.OPERATIONAL);

            var e = _recorder.Events.Last();
            Assert.Equal("MeasureStop", e.Type);
            Assert.Equal("MEASURING -> OPERATIONAL", e.Detail);
            Assert.Equal(777, e.PacketId);
        }

        [Fact]
        public void ReachingMeasuring_RecordsMeasureStart()
        {
            _recorder.StartCondition(_pid, "EF");

            _recorder.RecordMeasurementTransition(States.AWAIT_MEASUREMENT_START, States.MEASURING);

            Assert.Equal("MeasureStart", _recorder.Events.Last().Type);
        }

        [Theory]
        [InlineData(States.OPERATIONAL, States.AWAIT_MEASUREMENT_START)]  // still starting up
        [InlineData(States.MEASURING, States.MEASURING)]                  // no change
        [InlineData(States.MEASURING, States.RECORDING)]                  // device-side logging is still measuring
        [InlineData(States.RECORDING, States.MEASURING)]
        [InlineData(States.FLUSHING, States.MEASURING)]
        [InlineData(States.DETECTING, States.CONNECTING)]
        public void OtherTransitions_RecordNothing(States prev, States next)
        {
            _recorder.StartCondition(_pid, "EF");
            int before = _recorder.Events.Count;

            _recorder.RecordMeasurementTransition(prev, next);

            Assert.Equal(before, _recorder.Events.Count);
        }

        [Fact]
        public void NothingIsRecorded_WhenNoConditionIsRecording()
        {
            _recorder.RecordMeasurementTransition(States.MEASURING, States.OPERATIONAL);

            Assert.Empty(_recorder.Events);
        }
    }
}
