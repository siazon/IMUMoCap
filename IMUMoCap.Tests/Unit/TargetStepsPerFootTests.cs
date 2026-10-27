// IMUMoCap.Tests/Unit/TargetStepsPerFootTests.cs
using System.Text.Json;
using IMUMoCap.Pipeline;
using IMUMoCap.Pipeline.Models;
using Xunit;

namespace IMUMoCap.Tests.Unit
{
    /// <summary>
    /// 每脚目标有效步数统一由 PipelineParams.TargetStepsPerFoot 控制（Baseline / Training / Retention 共用）。
    /// docs/flow-review-2026-10-27.md §7.12
    /// </summary>
    public class TargetStepsPerFootTests
    {
        private static FpaResult Step(bool leftValid, bool rightValid) => new()
        {
            Fpa_L = leftValid  ? 1f : float.NaN,
            Fpa_R = rightValid ? 1f : float.NaN,
        };

        [Fact]
        public void Default_IsProtocolValue100()
        {
            Assert.Equal(100, new PipelineParams().TargetStepsPerFoot);
        }

        [Fact]
        public void OldParamsFileWithRenamedKey_FallsBackToDefault()
        {
            // pipeline_params.json written before the rename carries "MinBaselineSteps";
            // System.Text.Json ignores unknown members, so the value falls back to the default.
            var p = JsonSerializer.Deserialize<PipelineParams>("{\"MinBaselineSteps\":5,\"StaticGyroThreshold\":0.4}")!;
            Assert.Equal(100, p.TargetStepsPerFoot);
            Assert.Equal(0.4f, p.StaticGyroThreshold);
        }

        [Fact]
        public void TrainingBlock_CompletesAtParamValue_WhenBothFeetReachIt()
        {
            var pipeline = new GaitPipeline();
            pipeline.Params.TargetStepsPerFoot = 5;
            pipeline.SetTrainingBlock(1);
            int completed = 0;
            pipeline.OnTrainingBlockComplete += () => completed++;

            for (int i = 0; i < 4; i++) pipeline.CountTrainingStep(Step(true, true));
            Assert.Equal(0, completed);

            pipeline.CountTrainingStep(Step(true, true));
            Assert.Equal(1, completed);
            Assert.Equal(5, pipeline.TrainingStepsL);
            Assert.Equal(5, pipeline.TrainingStepsR);
        }

        [Fact]
        public void TrainingBlock_DoesNotComplete_WhenOnlyOneFootReachesTarget()
        {
            var pipeline = new GaitPipeline();
            pipeline.Params.TargetStepsPerFoot = 5;
            pipeline.SetTrainingBlock(1);
            int completed = 0;
            pipeline.OnTrainingBlockComplete += () => completed++;

            for (int i = 0; i < 8; i++) pipeline.CountTrainingStep(Step(true, false));

            Assert.Equal(0, completed);
        }

        [Fact]
        public void TrainingBlock_FiresOnlyOnce_PerBlock()
        {
            var pipeline = new GaitPipeline();
            pipeline.Params.TargetStepsPerFoot = 3;
            pipeline.SetTrainingBlock(1);
            int completed = 0;
            pipeline.OnTrainingBlockComplete += () => completed++;

            for (int i = 0; i < 6; i++) pipeline.CountTrainingStep(Step(true, true));

            Assert.Equal(1, completed);
        }

        [Fact]
        public void TrainingBlock_WithDefaultParam_NeedsFull100()
        {
            var pipeline = new GaitPipeline();
            pipeline.SetTrainingBlock(1);
            int completed = 0;
            pipeline.OnTrainingBlockComplete += () => completed++;

            for (int i = 0; i < 99; i++) pipeline.CountTrainingStep(Step(true, true));
            Assert.Equal(0, completed);

            pipeline.CountTrainingStep(Step(true, true));
            Assert.Equal(1, completed);
        }
    }
}
