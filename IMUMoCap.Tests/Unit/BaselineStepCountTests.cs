// IMUMoCap.Tests/Unit/BaselineStepCountTests.cs
using System.Text.Json;
using IMUMoCap.Pipeline;
using IMUMoCap.Pipeline.Models;
using Xunit;

namespace IMUMoCap.Tests.Unit
{
    /// <summary>
    /// Baseline 只数步幅法零点定下之后的步,每脚 50 步(与 Training / Retention 的 TargetStepsPerFoot 分开)。
    /// 原因:Pk12 零点前约 5 步用的是 PD FPA,与步幅法差约 ±8°,混入后 Baseline 均值偏 2.3°;
    /// 50 步的依据见 StepCountEvalTests.P3_P4_Baseline_Step_Count。
    /// </summary>
    public class BaselineStepCountTests
    {
        private static FpaResult Step(float l, float r, bool preZeroL = false, bool preZeroR = false) => new()
        {
            Fpa_L = l, Fpa_R = r, PreZero_L = preZeroL, PreZero_R = preZeroR,
        };

        [Fact]
        public void Default_BaselineStepsPerFoot_Is50_TrainingTargetUnchanged()
        {
            var p = new PipelineParams();
            Assert.Equal(50, p.BaselineStepsPerFoot);
            Assert.Equal(100, p.TargetStepsPerFoot);
        }

        [Fact]
        public void ParamsDialogDefault_MatchesThePipelineDefault()
        {
            // the dialog's VM value is pushed into Params on change / load, so the two defaults must agree
            Assert.Equal(new PipelineParams().BaselineStepsPerFoot, new IMUMoCap.MainPageVM().BaselineStepsPerFoot);
        }

        [Fact]
        public void OldParamsFileWithoutTheKey_FallsBackTo50()
        {
            var p = JsonSerializer.Deserialize<PipelineParams>("{\"TargetStepsPerFoot\":100}")!;
            Assert.Equal(50, p.BaselineStepsPerFoot);
        }

        [Fact]
        public void PreZeroSteps_AreNotCounted_PerFoot()
        {
            var b = new BaselineProcessor { MinValidSteps = 2 };
            b.AddStep(Step(30f, 1f, preZeroL: true));            // left still on PD FPA, right already on stride FPA
            Assert.Equal(0, b.CollectedSteps_L);
            Assert.Equal(1, b.CollectedSteps_R);
        }

        [Fact]
        public void PreZeroSteps_DoNotEnterTheBaselineMean()
        {
            var b = new BaselineProcessor { MinValidSteps = 2 };
            for (int i = 0; i < 5; i++) b.AddStep(Step(30f, 30f, preZeroL: true, preZeroR: true));
            b.AddStep(Step(2f, -2f));
            b.AddStep(Step(4f, -4f));
            var profile = b.TryFinalize();
            Assert.NotNull(profile);
            Assert.Equal(3f, profile!.MeanFpa_L, 3);
            Assert.Equal(-3f, profile.MeanFpa_R, 3);
            Assert.Equal(2, profile.ValidSteps_L);
        }
    }
}
