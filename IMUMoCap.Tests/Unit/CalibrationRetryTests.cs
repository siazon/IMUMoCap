// IMUMoCap.Tests/Unit/CalibrationRetryTests.cs
using System.Numerics;
using IMUMoCap.Pipeline;
using Xunit;

namespace IMUMoCap.Tests.Unit
{
    /// <summary>
    /// 校准失败后必须能重试(docs/flow-review-2026-10-27.md §2-17、O5):
    /// 操作员的规则是\"校准失败应重试直到成功,多次不成功才由操作员判断重启软件\"。
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

            /// <param name="awaitingRecalibration">true keeps frames after calibration from reaching the gait modules
            /// (what MainWindow does between stages); false mirrors the very first calibration.</param>
            public Rig(bool awaitingRecalibration = true)
            {
                Pipeline.AwaitingRecalibration = awaitingRecalibration;
                Pipeline.CalibrationArmed = true;
                Pipeline.OnCalibrationStateChanged += s => Last = s;
            }

            public void Feed(int frames, Vector3 gyro)
            {
                for (int i = 0; i < frames; i++) Pipeline.Process(Bundle(_pid++, gyro));
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
        public void StillParticipant_CalibratesSuccessfully()
        {
            var rig = new Rig();
            Assert.True(rig.Pipeline.TriggerCalibration());

            rig.FeedUntil(CalibrationState.Completed, Still);

            Assert.NotNull(rig.Pipeline.CalibrationProfile);
        }

        [Fact]
        public void ParticipantNeverStandsStill_TimesOutAndFails()
        {
            var rig = new Rig();
            rig.Pipeline.TriggerCalibration();

            rig.FeedUntil(CalibrationState.Failed, Moving);
        }

        [Fact]
        public void AfterAFailedFirstCalibration_ReadyRetriesAndCanSucceed()
        {
            var rig = new Rig(awaitingRecalibration: false);
            rig.Pipeline.TriggerCalibration();
            rig.FeedUntil(CalibrationState.Failed, Moving);

            Assert.True(rig.Pipeline.TriggerCalibration());      // Ready → retry
            Assert.Equal(CalibrationState.CollectingStaticPose, rig.Last);
            rig.FeedUntil(CalibrationState.Completed, Still);

            Assert.NotNull(rig.Pipeline.CalibrationProfile);
        }

        [Fact]
        public void AfterAFailedRecalibration_ReadyRetriesAndCanSucceed()
        {
            var rig = new Rig();
            rig.Pipeline.TriggerCalibration();
            rig.FeedUntil(CalibrationState.Completed, Still);

            Assert.True(rig.Pipeline.TriggerRecalibration());    // between stages
            rig.FeedUntil(CalibrationState.Failed, Moving);

            Assert.True(rig.Pipeline.TriggerRecalibration());    // Ready → retry
            rig.FeedUntil(CalibrationState.Completed, Still);

            Assert.NotNull(rig.Pipeline.CalibrationProfile);
        }

        [Fact]
        public void RetryIsUnlimited_ThreeFailuresThenSuccess()
        {
            var rig = new Rig(awaitingRecalibration: false);
            rig.Pipeline.TriggerCalibration();

            for (int attempt = 0; attempt < 3; attempt++)
            {
                rig.FeedUntil(CalibrationState.Failed, Moving);
                Assert.True(rig.Pipeline.TriggerCalibration());
                rig.Last = CalibrationState.CollectingStaticPose;
            }
            rig.FeedUntil(CalibrationState.Completed, Still);

            Assert.NotNull(rig.Pipeline.CalibrationProfile);
        }

        [Fact]
        public void FramesKeepArrivingWhileWaitingForTheRetry_WithoutThrowing()
        {
            var rig = new Rig(awaitingRecalibration: false); // first calibration: nothing else gates the later modules
            rig.Pipeline.TriggerCalibration();
            rig.FeedUntil(CalibrationState.Failed, Moving);

            var ex = Record.Exception(() => rig.Feed(50, Moving));

            Assert.Null(ex);
            Assert.Null(rig.Pipeline.CalibrationProfile);
        }

        [Fact]
        public void RetryIsRefused_WhileACalibrationIsAlreadyRunning()
        {
            var rig = new Rig();
            rig.Pipeline.TriggerCalibration();
            rig.Feed(10, Moving);

            Assert.False(rig.Pipeline.TriggerCalibration());
            Assert.False(rig.Pipeline.TriggerRecalibration());
        }

        [Fact]
        public void FirstCalibration_StillRequiresStartConditionToArmIt()
        {
            var pipeline = new GaitPipeline();           // CalibrationArmed is false
            Assert.False(pipeline.TriggerCalibration());
        }

        [Fact]
        public void Reset_StillWorksAfterAFailedCalibration()
        {
            var rig = new Rig();
            rig.Pipeline.TriggerCalibration();
            rig.FeedUntil(CalibrationState.Failed, Moving);

            rig.Pipeline.Reset();

            Assert.True(rig.Pipeline.TriggerCalibration());
        }
    }
}
