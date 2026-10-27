// IMUMoCap.Tests/Unit/CalibrationRetryTests.cs
using System.Collections.Generic;
using System.Numerics;
using IMUMoCap.Pipeline;
using Xunit;

namespace IMUMoCap.Tests.Unit
{
    /// <summary>
    /// 特征测试(docs/flow-review-2026-10-27.md §2-17):校准一旦 Failed,GaitPipeline 就没有任何入口可以重试。
    /// 操作员已决定\"校准失败应重试直到成功\"(O5),T12 需要先让这里成为可能。标 [KnownIssue] 的断言固定的是有缺陷的现状。
    /// </summary>
    public class CalibrationRetryTests
    {
        private static ImuFrameBundle Bundle(long pid, Vector3 gyro)
        {
            var b = new ImuFrameBundle(pid, pid / 100.0);
            foreach (var role in new[] { ImuRole.Pelvis, ImuRole.Left, ImuRole.Right })
                b.AddOrUpdate(new ImuSampleFrame
                {
                    PacketId = pid, Role = role,
                    HasQuaternion = true, Quaternion = Quaternion.Identity,
                    HasRateOfTurn = true, RateOfTurn = gyro,
                });
            return b;
        }

        private sealed class Rig
        {
            public readonly GaitPipeline Pipeline = new();
            public CalibrationState Last = CalibrationState.WaitingForStart;
            private long _pid = 1000;

            public Rig()
            {
                Pipeline.AwaitingRecalibration = true; // keeps frames after calibration from reaching the gait modules
                Pipeline.OnCalibrationStateChanged += s => Last = s;
            }

            /// <summary>Feeds frames until the calibration reaches <paramref name="target"/> (cap: 3000 frames).</summary>
            public void FeedUntil(CalibrationState target, Vector3 gyro)
            {
                for (int i = 0; i < 3000 && Last != target; i++)
                    Pipeline.Process(Bundle(_pid++, gyro));
                Assert.Equal(target, Last);
            }
        }

        private static readonly Vector3 Still = Vector3.Zero;
        private static readonly Vector3 Moving = new(1f, 0f, 0f); // > 0.3 rad/s: never counts as static

        [Fact]
        public void Baseline_StillParticipant_CalibratesSuccessfully()
        {
            var rig = new Rig { };
            rig.Pipeline.CalibrationArmed = true;
            Assert.True(rig.Pipeline.TriggerCalibration());

            rig.FeedUntil(CalibrationState.Completed, Still);

            Assert.NotNull(rig.Pipeline.CalibrationProfile);
        }

        [Fact]
        public void ParticipantNeverStandsStill_TimesOutAndFails()
        {
            var rig = new Rig();
            rig.Pipeline.CalibrationArmed = true;
            rig.Pipeline.TriggerCalibration();

            rig.FeedUntil(CalibrationState.Failed, Moving);
        }

        [Fact, Trait("Category", "KnownIssue")]
        public void KnownIssue_2_17_AfterAFailedFirstCalibration_NeitherReadyPathCanRetry()
        {
            var rig = new Rig();
            rig.Pipeline.CalibrationArmed = true;
            rig.Pipeline.TriggerCalibration();
            rig.FeedUntil(CalibrationState.Failed, Moving);

            // MainWindow.HandleReadyForCalibration calls TriggerCalibration (first calibration) or TriggerRecalibration.
            Assert.False(rig.Pipeline.TriggerCalibration());     // T12: should be able to retry
            Assert.False(rig.Pipeline.TriggerRecalibration());   // T12: should be able to retry
        }

        [Fact, Trait("Category", "KnownIssue")]
        public void KnownIssue_2_17_AfterAFailedRecalibration_ReadyIsRejected_ButARIsToldToTapReadyAgain()
        {
            var rig = new Rig();
            rig.Pipeline.CalibrationArmed = true;
            rig.Pipeline.TriggerCalibration();
            rig.FeedUntil(CalibrationState.Completed, Still);

            // between stages: recalibration starts, then the participant keeps moving until it times out
            Assert.True(rig.Pipeline.TriggerRecalibration());
            rig.Last = CalibrationState.CollectingStaticPose;
            rig.FeedUntil(CalibrationState.Failed, Moving);

            // MainWindow then broadcasts state "armed" ("Tap Ready when you are") — but tapping Ready does:
            Assert.False(rig.Pipeline.TriggerRecalibration());   // T12: should be accepted
            Assert.False(rig.Pipeline.TriggerCalibration());
        }

        [Fact]
        public void Reset_IsTheOnlyWayOut_AndItDiscardsTheBaseline()
        {
            var rig = new Rig();
            rig.Pipeline.CalibrationArmed = true;
            rig.Pipeline.TriggerCalibration();
            rig.FeedUntil(CalibrationState.Failed, Moving);

            rig.Pipeline.Reset();                                // what the Start Calibration button does

            Assert.True(rig.Pipeline.TriggerCalibration());
        }
    }
}
