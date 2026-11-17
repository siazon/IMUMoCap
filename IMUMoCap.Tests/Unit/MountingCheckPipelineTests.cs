// IMUMoCap.Tests/Unit/MountingCheckPipelineTests.cs
using System;
using System.Collections.Generic;
using System.Numerics;
using IMUMoCap.Pipeline;
using Xunit;

namespace IMUMoCap.Tests.Unit
{
    /// <summary>校准完成时检查佩戴方向(Pelvis X 朝上、两脚 Z 朝上),不对就判校准失败。</summary>
    public class MountingCheckPipelineTests
    {
        private static readonly Quaternion PelvisWorn = Quaternion.CreateFromAxisAngle(Vector3.UnitY, -MathF.PI / 2);

        private static ImuFrameBundle Bundle(long pid, Quaternion pelvis)
        {
            var b = new ImuFrameBundle(pid, pid / 100.0);
            foreach (var (role, q) in new[] { (ImuRole.Pelvis, pelvis), (ImuRole.Left, Quaternion.Identity), (ImuRole.Right, Quaternion.Identity) })
                b.AddOrUpdate(new ImuSampleFrame { PacketId = pid, Role = role, HasQuaternion = true, Quaternion = q, HasRateOfTurn = true });
            return b;
        }

        private static (GaitPipeline P, List<CalibrationState> States, List<string> Rejections) Calibrate(Quaternion pelvis)
        {
            var p = new GaitPipeline { CalibrationArmed = true };
            var states = new List<CalibrationState>();
            var rejections = new List<string>();
            p.OnCalibrationStateChanged += states.Add;
            p.OnMountingRejected += rejections.Add;
            p.TriggerCalibration();
            for (long pid = 1000; pid < 1400; pid++) p.Process(Bundle(pid, pelvis));
            return (p, states, rejections);
        }

        [Fact]
        public void PelvisXUp_Completes()
        {
            var (p, states, rejections) = Calibrate(PelvisWorn);
            Assert.Equal(CalibrationState.Completed, states[^1]);
            Assert.Empty(rejections);
            Assert.NotNull(p.CalibrationProfile);
        }

        [Fact]
        public void PelvisLyingFlat_CalibrationFailsWithReason_AndCanBeRetried()
        {
            var (p, states, rejections) = Calibrate(Quaternion.Identity);
            Assert.Equal(CalibrationState.Failed, states[^1]);
            Assert.DoesNotContain(CalibrationState.Completed, states);
            Assert.Contains("Pelvis", Assert.Single(rejections));
            Assert.Null(p.CalibrationProfile);
            Assert.True(p.TriggerCalibration());   // Ready → retry, as for any failed calibration
        }
    }
}
