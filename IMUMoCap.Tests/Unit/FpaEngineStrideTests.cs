using System.Collections.Generic;
using System.Numerics;
using IMUMoCap.Pipeline;
using IMUMoCap.Pipeline.Models;

namespace IMUMoCap.Tests.Unit
{
    /// <summary>步幅法 FPA 接入 FpaEngine(docs/stride_fpa_design.md 三、实现方式)。</summary>
    public class FpaEngineStrideTests
    {
        private const int StanceFrames = 40, SwingFrames = 60, Cycle = StanceFrames + SwingFrames;

        private static float Rad(float deg) => deg * MathF.PI / 180f;
        private static Quaternion Yaw(float deg) => Quaternion.CreateFromAxisAngle(Vector3.UnitZ, Rad(deg));

        /// <summary>Per-cycle walking conditions (all angles in degrees, world frame).</summary>
        /// <param name="RightFrameDeg">Heading of the right sensor's own world frame relative to the left's (each MTw has
        /// its own heading reference; under VRU it is arbitrary). Rotates both its yaw and its free acceleration.</param>
        /// <param name="Accel">Swing acceleration magnitude, m/s² (10 → ~0.9 m strides; 0.2 → ~2 cm, i.e. shuffling in place).</param>
        private sealed record Walk(float FootYawL = 10f, float FootYawR = -10f, float StrideDir = 0f, float Pd = 0f,
                                   bool PdValid = true, ContextState Ctx = ContextState.Straight, float RightFrameDeg = 0f,
                                   float Accel = 10f);

        private sealed class Rig
        {
            public readonly FpaEngine Engine = new();
            public readonly List<FpaResult> Results = new();
            public readonly List<(string Foot, bool Emitted, StepExclusionReason? Reason)> Outcomes = new();
            public readonly List<(string Foot, float OffsetDeg, float SdDeg)> Zeros = new();
            public CalibrationProfile Cal = new() { PelvisRef = Quaternion.Identity, LeftFootRef = Quaternion.Identity, RightFootRef = Quaternion.Identity };
            private long _pid = 1000;

            public Rig(bool useStride)
            {
                Engine.UseStrideDirection = useStride;
                Engine.OnStepOutcome += (f, e, r) => Outcomes.Add((f, e, r));
                Engine.OnStrideZeroEstimated += (f, o, sd) => Zeros.Add((f, o, sd));
            }

            /// <summary>One stance + swing cycle for both feet (same timing; the engine treats feet independently).</summary>
            public void Run(int cycles, Walk w)
            {
                for (int c = 0; c < cycles; c++)
                    for (int i = 0; i < Cycle; i++)
                    {
                        bool stance = i < StanceFrames;
                        Vector3 accL = Vector3.Zero, accR = Vector3.Zero;
                        if (!stance)
                        {
                            float a = (i - StanceFrames) < SwingFrames / 2 ? w.Accel : -w.Accel;
                            accL = new Vector3(MathF.Cos(Rad(w.StrideDir)) * a, MathF.Sin(Rad(w.StrideDir)) * a, 0f);
                            float dirR = w.StrideDir + w.RightFrameDeg;
                            accR = new Vector3(MathF.Cos(Rad(dirR)) * a, MathF.Sin(Rad(dirR)) * a, 0f);
                        }
                        var frame = Frame(_pid++, w.FootYawL, w.FootYawR + w.RightFrameDeg, accL, accR);
                        var gait = new GaitEvent { LeftStance = stance, RightStance = stance, LeftSwing = !stance, RightSwing = !stance };
                        var ctx = new MotionContext { State = w.Ctx, Confidence = 1f };
                        var pd = new PdEstimate { DirectionRad = Rad(w.Pd), IsValid = w.PdValid, Stability = 1f };
                        var r = Engine.Process(frame, gait, ctx, pd, Cal);
                        if (r != null) Results.Add(r);
                    }
            }

            private static ValidFrame Frame(long pid, float yawL, float yawR, Vector3 accL, Vector3 accR)
            {
                var b = new ImuFrameBundle(pid, pid / 100.0);
                b.AddOrUpdate(Sample(pid, ImuRole.Pelvis, Quaternion.Identity, Vector3.Zero));
                b.AddOrUpdate(Sample(pid, ImuRole.Left, Yaw(yawL), accL));
                b.AddOrUpdate(Sample(pid, ImuRole.Right, Yaw(yawR), accR));
                return new ValidFrame(b);
            }

            private static ImuSampleFrame Sample(long pid, ImuRole role, Quaternion q, Vector3 freeAcc) => new()
            {
                PacketId = pid, Role = role,
                HasQuaternion = true, Quaternion = q,
                HasRateOfTurn = true, RateOfTurn = Vector3.Zero,
                HasFreeAcceleration = true, FreeAcceleration = freeAcc,
            };
        }

        [Fact]
        public void PdMode_FeedbackIsUnchanged_AndStrideIsRecordedOnceZeroIsKnown()
        {
            var rig = new Rig(useStride: false);
            rig.Run(15, new Walk());
            Assert.All(rig.Results, r => Assert.Equal(r.FpaPd_L, r.Fpa_L));
            Assert.All(rig.Results, r => Assert.Equal(10f, r.Fpa_L, 1));
            Assert.All(rig.Results, r => Assert.Equal(10f, r.Fpa_R, 1));   // right foot: positive = toe-out
            Assert.True(float.IsNaN(rig.Results[0].FpaStride_L));            // zero not yet estimated
            Assert.Equal(10f, rig.Results[^1].FpaStride_L, 1);
            Assert.Equal(10f, rig.Results[^1].FpaStride_R, 1);
        }

        [Fact]
        public void ZeroIsEstimatedOncePerFoot_FromTenPairedStrides()
        {
            var rig = new Rig(useStride: false);
            rig.Cal = new CalibrationProfile { PelvisRef = Quaternion.Identity, LeftFootRef = Yaw(5f), RightFootRef = Yaw(-5f) };
            rig.Run(15, new Walk());
            Assert.Equal(2, rig.Zeros.Count);
            var left = rig.Zeros.Find(z => z.Foot == "L");
            Assert.Equal(5f, left.OffsetDeg, 1);   // ψ_abs − s − FPA_pd,raw = 10 − 0 − (10 − 5 − 0)
            Assert.InRange(left.SdDeg, 0f, 0.5f);
            Assert.Equal(5f, rig.Results[^1].FpaPd_L, 1);
            Assert.Equal(5f, rig.Results[^1].FpaStride_L, 1);  // same definition as the PD method at calibration
        }

        [Fact]
        public void PlanB_Zero_IgnoresAPelvisOffsetBetweenStandAndWalking()
        {
            // Ps2 2026-11-17: walking, the pelvis is turned ~10° relative to its stand orientation while the feet aren't.
            // Plan A copied that into the zero (L −3.6°, R +3.5°); plan B uses only each foot's sensor.
            var rig = new Rig(useStride: true);
            var checks = new List<(string Foot, float DiffDeg, bool UsedB)>();
            rig.Engine.OnStrideZeroCrossCheck += (f, d, b) => checks.Add((f, d, b));
            rig.Run(15, new Walk(Pd: 10f));
            Assert.Equal(0f, rig.Results[^1].FpaPd_L, 1);      // pelvis-referenced: biased
            Assert.Equal(20f, rig.Results[^1].FpaPd_R, 1);
            Assert.Equal(10f, rig.Results[^1].FpaStride_L, 1);  // stride, plan B zero: unbiased
            Assert.Equal(10f, rig.Results[^1].FpaStride_R, 1);
            Assert.Equal(2, checks.Count);                       // plan A − plan B, reported once per foot
            Assert.All(checks, c => { Assert.Equal(10f, System.MathF.Abs(c.DiffDeg), 1); Assert.True(c.UsedB); });
        }

        [Fact]
        public void StandNotFacingThePath_FallsBackToPlanA_InsteadOfLosingAllOutput()
        {
            // Po9 / Pl89: the participant stood ~40–90° off the path at calibration and turned before walking.
            // Plan B would then be off by that much (all strides fail G8 → no feedback at all); plan A is unaffected.
            var rig = new Rig(useStride: true);
            var checks = new List<(string Foot, float DiffDeg, bool UsedB)>();
            rig.Engine.OnStrideZeroCrossCheck += (f, d, b) => checks.Add((f, d, b));
            rig.Cal = new CalibrationProfile { PelvisRef = Quaternion.Identity, LeftFootRef = Yaw(-40f), RightFootRef = Yaw(-40f) };
            rig.Run(15, new Walk(Pd: 40f));                      // walking direction is 40° from the stand's facing
            Assert.Equal(2, checks.Count);
            Assert.All(checks, c => { Assert.Equal(40f, System.MathF.Abs(c.DiffDeg), 1); Assert.False(c.UsedB); });
            Assert.Equal(10f, rig.Results[^1].FpaStride_L, 1);
            Assert.Equal(10f, rig.Results[^1].FpaStride_R, 1);
            Assert.Equal(rig.Results[^1].FpaStride_L, rig.Results[^1].Fpa_L);
        }

        [Fact]
        public void PlanA_NotAvailable_PlanBIsUsedAfterTheTimeout_WithoutACrossCheck()
        {
            // PD stuck from the start (e.g. ReacquiringPd): plan A never gets a sample, plan B mustn't wait forever.
            var rig = new Rig(useStride: true);
            var checks = new List<(string Foot, float DiffDeg, bool UsedB)>();
            rig.Engine.OnStrideZeroCrossCheck += (f, d, b) => checks.Add((f, d, b));
            rig.Run(10, new Walk(PdValid: false));
            Assert.Empty(rig.Zeros);                             // plan B is in, still waiting for plan A
            rig.Run(15, new Walk(PdValid: false));
            Assert.Equal(2, rig.Zeros.Count);
            Assert.All(checks, c => { Assert.True(float.IsNaN(c.DiffDeg)); Assert.True(c.UsedB); });
            Assert.Equal(10f, rig.Results[^1].FpaStride_L, 1);
            Assert.Equal(10f, rig.Results[^1].FpaStride_R, 1);
        }

        [Fact]
        public void PlanB_EarlyWalkingDirection_FoldsReturnTripsBy180()
        {
            // the first 10 qualifying strides can include walking back along the same line
            var rig = new Rig(useStride: true);
            rig.Cal = new CalibrationProfile { PelvisRef = Quaternion.Identity, LeftFootRef = Yaw(5f), RightFootRef = Yaw(-5f) };
            rig.Run(4, new Walk());
            rig.Run(14, new Walk(StrideDir: 180f, FootYawL: 190f, FootYawR: 170f, Pd: 180f));
            Assert.Equal(2, rig.Zeros.Count);
            Assert.Equal(5f, rig.Zeros.Find(z => z.Foot == "L").OffsetDeg, 1);   // ψ_cal − s_early = 5 − 0
            Assert.Equal(5f, rig.Results[^1].FpaStride_L, 1);
            Assert.Equal(5f, rig.Results[^1].FpaStride_R, 1);
        }

        // ── filter restarts (XDA re-initialises an MTw's filter, e.g. near gyro saturation) ──────────────────────

        [Fact]
        public void FootFilterRestart_BeforeTheZero_StopsThatFoot_UntilANewCalibration()
        {
            // the stand heading (old filter frame) can no longer be compared with strides in the new frame
            var rig = new Rig(useStride: true);
            var invalidated = new List<string>();
            rig.Engine.OnStrideZeroInvalidated += (f, why) => invalidated.Add(f);
            rig.Run(3, new Walk());                               // zero not decided yet
            rig.Engine.NotifyFilterRestart(ImuRole.Left);
            int n = rig.Results.Count;
            rig.Run(10, new Walk());
            Assert.Equal(new[] { "L" }, invalidated);
            var after = rig.Results.GetRange(n, rig.Results.Count - n);
            Assert.NotEmpty(after);
            Assert.All(after, r => Assert.True(float.IsNaN(r.Fpa_L)));    // left stopped
            Assert.Contains(after, r => !float.IsNaN(r.FpaStride_R));      // right unaffected
            Assert.DoesNotContain(rig.Zeros, z => z.Foot == "L");

            rig.Cal = new CalibrationProfile { PelvisRef = Quaternion.Identity, LeftFootRef = Yaw(1f), RightFootRef = Quaternion.Identity };
            rig.Run(10, new Walk());                              // recalibrated: left zero estimated again
            Assert.Contains(rig.Zeros, z => z.Foot == "L");
            Assert.Equal(9f, rig.Results[^1].FpaStride_L, 1);    // ψ − s − (1° stand heading − 0)
        }

        [Fact]
        public void FootFilterRestart_AfterTheZero_ChangesNothing()
        {
            var rig = new Rig(useStride: true);
            var invalidated = new List<string>();
            rig.Engine.OnStrideZeroInvalidated += (f, why) => invalidated.Add(f);
            rig.Run(15, new Walk());
            rig.Engine.NotifyFilterRestart(ImuRole.Left);
            int n = rig.Results.Count;
            rig.Run(5, new Walk());
            Assert.Empty(invalidated);
            var leftAfter = rig.Results.GetRange(n, rig.Results.Count - n).Where(r => !float.IsNaN(r.FpaStride_L)).ToList();
            Assert.NotEmpty(leftAfter);
            Assert.All(leftAfter, r => Assert.Equal(10f, r.FpaStride_L, 1));
        }

        [Fact]
        public void PelvisFilterRestart_BeforeTheZero_UsesPlanBWithoutCrossCheck_AndNoPdFeedback()
        {
            var rig = new Rig(useStride: true);
            var checks = new List<(string Foot, float DiffDeg, bool UsedB)>();
            rig.Engine.OnStrideZeroCrossCheck += (f, d, b) => checks.Add((f, d, b));
            rig.Run(2, new Walk());
            rig.Engine.NotifyFilterRestart(ImuRole.Pelvis);
            int n = rig.Results.Count;
            rig.Run(15, new Walk());
            Assert.Equal(2, checks.Count);
            Assert.All(checks, c => { Assert.True(float.IsNaN(c.DiffDeg)); Assert.True(c.UsedB); });
            Assert.All(rig.Results.GetRange(n, rig.Results.Count - n),
                       r => Assert.True(!float.IsNaN(r.FpaStride_L) || !float.IsNaN(r.FpaStride_R)));  // no PD-only output
            Assert.Equal(10f, rig.Results[^1].FpaStride_L, 1);
        }

        [Fact]
        public void StrideMode_BeforeTheZero_OnlyCountsStepsWithAValidStride()
        {
            // Pf2 2026-11-17: feet barely moving (1 cm "strides", 0.2 s swings) — the PD method counted 35 baseline
            // steps per foot from that; in stride mode those steps mustn't reach the baseline before the zero is known.
            var stride = new Rig(useStride: true);
            stride.Run(10, new Walk(Accel: 0.2f));
            Assert.Empty(stride.Results);
            Assert.Contains(stride.Outcomes, o => !o.Emitted && o.Reason == StepExclusionReason.StrideInvalid);

            var pd = new Rig(useStride: false);                  // switch off: old behaviour unchanged
            pd.Run(10, new Walk(Accel: 0.2f));
            Assert.NotEmpty(pd.Results);
        }

        [Fact]
        public void StrideMode_UsesPdFpaUntilZeroIsKnown_ThenStrideFpa()
        {
            var rig = new Rig(useStride: true);
            rig.Run(15, new Walk());
            Assert.Equal(rig.Results[0].FpaPd_L, rig.Results[0].Fpa_L);
            Assert.True(float.IsNaN(rig.Results[0].FpaStride_L));
            Assert.Equal(rig.Results[^1].FpaStride_L, rig.Results[^1].Fpa_L);
        }

        [Fact]
        public void StrideMode_MarksPdFeedbackStepsBeforeTheZero_AsPreZero()
        {
            var rig = new Rig(useStride: true);
            rig.Run(15, new Walk());
            Assert.True(rig.Results[0].PreZero_L);
            Assert.True(rig.Results[0].PreZero_R);
            Assert.False(rig.Results[^1].PreZero_L);
            Assert.False(rig.Results[^1].PreZero_R);

            var pd = new Rig(useStride: false);                  // PD mode: no zero needed, nothing is pre-zero
            pd.Run(15, new Walk());
            Assert.All(pd.Results, r => Assert.False(r.PreZero_L || r.PreZero_R));
        }

        [Fact]
        public void StrideMode_IgnoresPelvisPdDrift()
        {
            var rig = new Rig(useStride: true);
            rig.Run(15, new Walk());
            int n = rig.Results.Count;
            rig.Run(5, new Walk(Pd: 20f));   // pelvis heading drifted 20°: PD FPA goes wrong, stride FPA must not
            var after = rig.Results.GetRange(n, rig.Results.Count - n);
            Assert.NotEmpty(after);
            Assert.All(after, r => Assert.Equal(10f, r.Fpa_L, 1));
            Assert.All(after, r => Assert.Equal(10f, r.Fpa_R, 1));
            Assert.All(after, r => Assert.Equal(-10f, r.FpaPd_L, 1));
        }

        [Fact]
        public void StrideMode_StillOutputs_WhilePdInvalidOrReacquiring()
        {
            var rig = new Rig(useStride: true);
            rig.Run(15, new Walk());
            int n = rig.Results.Count;
            rig.Run(5, new Walk(PdValid: false, Ctx: ContextState.ReacquiringPd));
            var after = rig.Results.GetRange(n, rig.Results.Count - n);
            Assert.Equal(5, after.Count);     // both feet settle on the same frame here → one result per cycle
            Assert.All(after, r => Assert.True(float.IsNaN(r.FpaPd_L)));
        }

        [Fact]
        public void StrideMode_WorksWhenTheFeetHaveDifferentHeadingReferences()
        {
            // Replay of 2026-11-12 16:38 showed left strides at ~73° and right at ~−30° in their own frames.
            var rig = new Rig(useStride: true);
            rig.Cal = new CalibrationProfile { PelvisRef = Quaternion.Identity, LeftFootRef = Quaternion.Identity, RightFootRef = Yaw(100f) };
            rig.Run(15, new Walk(RightFrameDeg: 100f));
            Assert.Equal(2, rig.Zeros.Count);
            Assert.Equal(10f, rig.Results[^1].FpaStride_L, 1);
            Assert.Equal(10f, rig.Results[^1].FpaStride_R, 1);
        }

        [Fact]
        public void StrideMode_ExcludesTurningSteps()
        {
            var rig = new Rig(useStride: true);
            rig.Run(15, new Walk());
            int n = rig.Results.Count, m = rig.Outcomes.Count;
            rig.Run(3, new Walk(Ctx: ContextState.Turning));
            Assert.Equal(n, rig.Results.Count);
            Assert.All(rig.Outcomes.GetRange(m, rig.Outcomes.Count - m),
                       o => Assert.Equal(StepExclusionReason.Turning, o.Reason));
        }

        [Fact]
        public void StrideMode_DirectionChange_ExcludesFirstStrideAsNotStraight()
        {
            var rig = new Rig(useStride: true);
            rig.Run(15, new Walk());
            int m = rig.Outcomes.Count;
            // Walked off in a new direction. Cycle 1 still lands the old-direction swing; the first new-direction
            // stride lands in cycle 2 (excluded by G5/G6), the second in cycle 3 (accepted).
            rig.Run(3, new Walk(StrideDir: 90f, FootYawL: 100f, FootYawR: 80f));
            var o = rig.Outcomes.GetRange(m, rig.Outcomes.Count - m);
            Assert.Contains(o, x => x.Reason == StepExclusionReason.StrideNotStraight);
            Assert.Contains(o, x => x.Emitted);   // second stride in the new direction is accepted again
        }

        [Fact]
        public void Reset_KeepsTheZero_ResetCondition_ClearsIt()
        {
            var rig = new Rig(useStride: true);
            rig.Run(15, new Walk());
            rig.Engine.Reset();                    // recalibration / training start: zero stays
            int n = rig.Results.Count;
            rig.Run(4, new Walk());
            var afterReset = rig.Results.GetRange(n, rig.Results.Count - n);
            Assert.NotEmpty(afterReset);
            // stride history restarts (G5/G6 need a previous stride), but no PD fallback: every output is a stride FPA
            Assert.All(afterReset, r => Assert.True(!float.IsNaN(r.FpaStride_L) || !float.IsNaN(r.FpaStride_R)));
            Assert.Equal(2, rig.Zeros.Count);

            rig.Engine.ResetCondition();           // new condition: zero estimated again
            n = rig.Results.Count;
            rig.Run(4, new Walk());
            Assert.All(rig.Results.GetRange(n, rig.Results.Count - n), r => Assert.True(float.IsNaN(r.FpaStride_L)));
        }
    }
}
