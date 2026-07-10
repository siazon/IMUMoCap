// IMUMoCap.Tests/Integration/GaitPipelineReplayTests.cs
using System.Collections.Generic;
using System.IO;
using IMUMoCap.Pipeline;
using IMUMoCap.Pipeline.Models;
using Xunit;

namespace IMUMoCap.Tests.Integration
{
    /// <summary>
    /// 集成测试：从录制的 JSONL 回放，验证流水线端到端输出。
    /// Fixture 文件放在 Fixtures/ 目录下，并设置 CopyToOutputDirectory。
    /// </summary>
    public class GaitPipelineReplayTests
    {
        private static string FixturePath(string name)
            => Path.Combine("Fixtures", name);

        /// <summary>
        /// 基础冒烟测试：回放直走录制，确认流水线能输出 FPA 结果且值在合理范围内。
        /// 录制文件需包含：校准（跺脚+静立）→ Baseline 阶段 → Training 阶段的完整流程。
        /// </summary>
        [Fact(Skip = "需要先录制 walk_straight.jsonl fixture")]
        public void WalkStraight_ProducesFpaResultsInReasonableRange()
        {
            var pipeline = new GaitPipeline();
            var results  = new List<FpaResult>();

            // 录制文件里已包含完整流程；这里直接回放
            pipeline.StartBaseline();
            foreach (var bundle in BundlePlayer.Load(FixturePath("walk_straight.jsonl")))
                pipeline.Process(bundle);

            Assert.NotEmpty(results);
            Assert.All(results, r =>
            {
                if (!float.IsNaN(r.Fpa_L)) Assert.InRange(r.Fpa_L, -45f, 45f);
                if (!float.IsNaN(r.Fpa_R)) Assert.InRange(r.Fpa_R, -45f, 45f);
            });
        }
    }
}
