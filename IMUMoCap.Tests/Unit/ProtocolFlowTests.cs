// IMUMoCap.Tests/Unit/ProtocolFlowTests.cs
using System.Collections.Generic;
using System.Linq;
using IMUMoCap.Model;
using IMUMoCap.Protocol;
using Xunit;

namespace IMUMoCap.Tests.Unit
{
    /// <summary>
    /// ProtocolFlow 的"当前行为版"测试(docs/flow-review-2026-10-27.md §8 T1):
    /// 复刻 MainWindow 里 _currentStage/_inRest/_pendingStageEntry/... 的现有语义,包括 §2 记录的已知缺陷。
    /// 标 [KnownIssue] 的测试固定的是**有缺陷的现状**,对应修复任务(T9–T14)落地时会翻转断言。
    /// </summary>
    public class ProtocolFlowTests
    {
        // ── helpers ──────────────────────────────────────────────────────────

        private static Phase P(PhaseKind kind, Stage? stage = null) => new(kind, stage);

        private static void Go(ProtocolFlow f, params FlowEvent[] events)
        {
            foreach (var e in events)
            {
                var r = f.Fire(e);
                Assert.True(r.Accepted, $"{e.Kind} was ignored in {f.Phase}: {r.Reason}");
            }
        }

        private static readonly FlowEvent[] Recal =
            { FlowEvent.CalibrationStarted, FlowEvent.CalibrationCompleted };

        private static ProtocolFlow AtArmed()
        {
            var f = new ProtocolFlow();
            Go(f, FlowEvent.StartCondition);
            return f;
        }

        private static ProtocolFlow AtBaseline()
        {
            var f = AtArmed();
            Go(f, Recal);
            return f;
        }

        private static ProtocolFlow AtTraining1()
        {
            var f = AtBaseline();
            Go(f, FlowEvent.BaselineCompleted(true));
            Go(f, Recal);
            return f;
        }

        private static ProtocolFlow AtRest1()
        {
            var f = AtTraining1();
            Go(f, FlowEvent.BlockComplete);
            return f;
        }

        private static ProtocolFlow AtTraining2()
        {
            var f = AtRest1();
            Go(f, FlowEvent.RestElapsed);
            Go(f, Recal);
            return f;
        }

        private static ProtocolFlow AtTraining3()
        {
            var f = AtTraining2();
            Go(f, FlowEvent.BlockComplete);
            Go(f, Recal);
            return f;
        }

        private static ProtocolFlow AtRest3()
        {
            var f = AtTraining3();
            Go(f, FlowEvent.BlockComplete);
            return f;
        }

        private static ProtocolFlow AtRetention()
        {
            var f = AtRest3();
            Go(f, FlowEvent.RestElapsed);
            Go(f, Recal);
            return f;
        }

        private static ProtocolFlow AtRetentionDone()
        {
            var f = AtRetention();
            Go(f, FlowEvent.RetentionTargetReached);
            return f;
        }

        private static ChecklistStatus Status(ProtocolFlow f, string label) =>
            f.Checklist().Single(c => c.Label == label).Status;

        // ── happy path ───────────────────────────────────────────────────────

        [Fact]
        public void NewFlow_IsIdle_AndNotRecording()
        {
            var f = new ProtocolFlow();

            Assert.Equal(P(PhaseKind.Idle), f.Phase);
            Assert.False(f.IsRecording);
            Assert.False(f.Paused);
            Assert.Equal("", f.CurrentStage);
            Assert.Equal("waiting", f.ArState);
        }

        [Fact]
        public void FullProtocol_WalksThroughEveryPhase_InOrder()
        {
            var f = new ProtocolFlow();

            Go(f, FlowEvent.StartCondition);
            Assert.Equal(P(PhaseKind.Armed, Stage.Baseline), f.Phase);
            Assert.Equal("armed", f.ArState);

            Go(f, FlowEvent.CalibrationStarted);
            Assert.Equal(P(PhaseKind.Calibrating, Stage.Baseline), f.Phase);
            Assert.Equal("calibrating", f.ArState);

            Go(f, FlowEvent.CalibrationCompleted);
            Assert.Equal(P(PhaseKind.Baseline, Stage.Baseline), f.Phase);
            Assert.Equal("baseline", f.ArState);

            Go(f, FlowEvent.BaselineCompleted(true));
            Assert.Equal(P(PhaseKind.RecalPending, Stage.Training1), f.Phase);
            Assert.Equal("armed", f.ArState);
            Assert.Equal("Baseline", f.CurrentStage); // legacy: _currentStage stays the old stage until EnterStage

            Go(f, FlowEvent.CalibrationStarted);
            Assert.Equal(P(PhaseKind.Recalibrating, Stage.Training1), f.Phase);
            Go(f, FlowEvent.CalibrationCompleted);
            Assert.Equal(P(PhaseKind.Training, Stage.Training1), f.Phase);
            Assert.Equal("training", f.ArState);
            Assert.Equal("Training1", f.CurrentStage);

            Go(f, FlowEvent.BlockComplete);
            Assert.Equal(P(PhaseKind.Rest, Stage.Training1), f.Phase);
            Assert.True(f.InRest);
            Assert.Equal("rest", f.ArState);
            Assert.Equal(1, f.RestBlockJustFinished);
            Assert.Equal("Training1", f.CurrentStage); // legacy: rest is not a stage

            Go(f, FlowEvent.RestElapsed);
            Assert.Equal(P(PhaseKind.RecalPending, Stage.Training2), f.Phase);
            Assert.False(f.InRest);
            Assert.Equal("Training1", f.CurrentStage);
            Go(f, Recal);
            Assert.Equal(P(PhaseKind.Training, Stage.Training2), f.Phase);

            Go(f, FlowEvent.BlockComplete);
            Assert.Equal(P(PhaseKind.RecalPending, Stage.Training3), f.Phase); // no rest between Block2 and Block3
            Assert.False(f.InRest);
            Assert.Equal(1, f.RestBlockJustFinished);
            Go(f, Recal);
            Assert.Equal(P(PhaseKind.Training, Stage.Training3), f.Phase);

            Go(f, FlowEvent.BlockComplete);
            Assert.Equal(P(PhaseKind.Rest, Stage.Training3), f.Phase);
            Assert.Equal(3, f.RestBlockJustFinished);

            Go(f, FlowEvent.RestElapsed);
            Assert.Equal(P(PhaseKind.RecalPending, Stage.Retention), f.Phase);
            Go(f, Recal);
            Assert.Equal(P(PhaseKind.Retention, Stage.Retention), f.Phase);
            Assert.True(f.IsRetention);
            Assert.False(f.RetentionComplete);
            Assert.Equal("retention", f.ArState);

            Go(f, FlowEvent.RetentionTargetReached);
            Assert.Equal(P(PhaseKind.RetentionDone, Stage.Retention), f.Phase);
            Assert.True(f.IsRetention);
            Assert.True(f.RetentionComplete);
            Assert.Equal("retention", f.ArState); // AR is not told retention finished

            Go(f, FlowEvent.EndRecording);
            Assert.Equal(P(PhaseKind.Ended, Stage.Retention), f.Phase);
            Assert.False(f.IsRecording);
            Assert.False(f.IsRetention);
            Assert.Equal("ended", f.ArState);
            Assert.Equal("Retention", f.CurrentStage); // legacy: _currentStage is kept after End
            Assert.True(f.RetentionComplete);          // legacy: _retentionComplete is not cleared by End
        }

        [Fact]
        public void RetentionTimeout_AndTargetReached_AreTheSameEvent_AndIdempotent()
        {
            var f = AtRetention();
            Go(f, FlowEvent.RetentionTargetReached);

            var again = f.Fire(FlowEvent.RetentionTargetReached);

            Assert.False(again.Accepted);
            Assert.Equal(P(PhaseKind.RetentionDone, Stage.Retention), f.Phase);
        }

        // ── ignored / misplaced events ───────────────────────────────────────

        [Fact]
        public void EventsInWrongPhase_AreIgnored_AndLeaveStateUntouched()
        {
            var f = AtBaseline();
            var before = f.Phase;

            foreach (var e in new[] { FlowEvent.BlockComplete, FlowEvent.RestElapsed,
                                      FlowEvent.RetentionTargetReached, FlowEvent.CalibrationCompleted })
                Assert.False(f.Fire(e).Accepted, e.Kind.ToString());

            Assert.Equal(before, f.Phase);
        }

        [Fact]
        public void DuplicateBlockComplete_IsIgnored_AfterTheBlockAlreadyEnded()
        {
            var f = AtTraining1();
            Go(f, FlowEvent.BlockComplete);

            var dup = f.Fire(FlowEvent.BlockComplete);

            Assert.False(dup.Accepted);
            Assert.Equal(P(PhaseKind.Rest, Stage.Training1), f.Phase);
        }

        [Fact]
        public void BlockComplete_DuringPendingRecalibration_IsIgnored()
        {
            var f = AtRest1();
            Go(f, FlowEvent.RestElapsed); // RecalPending(Training2), CurrentStage still Training1

            Assert.False(f.Fire(FlowEvent.BlockComplete).Accepted);
            Assert.Equal(P(PhaseKind.RecalPending, Stage.Training2), f.Phase);
        }

        [Fact]
        public void EverythingExceptStartCondition_IsIgnored_WhenNotRecording()
        {
            var f = new ProtocolFlow();

            foreach (var e in new[] { FlowEvent.CalibrationStarted, FlowEvent.NextStage, FlowEvent.Pause,
                                      FlowEvent.EndRecording, FlowEvent.BlockComplete })
                Assert.False(f.Fire(e).Accepted, e.Kind.ToString());

            Assert.Equal(P(PhaseKind.Idle), f.Phase);
        }

        // ── failures ─────────────────────────────────────────────────────────

        [Fact]
        public void FirstCalibrationFailure_GoesBackToArmed_SoReadyCanRetryWithoutLimit()
        {
            var f = AtArmed();
            for (int attempt = 0; attempt < 3; attempt++)
            {
                Go(f, FlowEvent.CalibrationStarted, FlowEvent.CalibrationFailed);
                Assert.Equal(P(PhaseKind.Armed, Stage.Baseline), f.Phase);
                Assert.Equal("armed", f.ArState);   // the AR shows its Ready button again
            }

            Go(f, Recal);
            Assert.Equal(P(PhaseKind.Baseline, Stage.Baseline), f.Phase);
        }

        [Fact]
        public void FailedPhase_IsOnlyForInsufficientBaseline_AndOnlyEndCanLeaveIt()
        {
            var f = AtBaseline();
            Go(f, FlowEvent.BaselineCompleted(false));

            Assert.Equal(P(PhaseKind.Failed, Stage.Baseline), f.Phase);
            Assert.False(f.Fire(FlowEvent.CalibrationStarted).Accepted);
            Assert.False(f.Fire(FlowEvent.CalibrationCompleted).Accepted);

            Go(f, FlowEvent.EndRecording);
            Assert.Equal(PhaseKind.Ended, f.Phase.Kind);
        }

        [Fact]
        public void InsufficientBaseline_GoesToFailed()
        {
            var f = AtBaseline();
            Go(f, FlowEvent.BaselineCompleted(false));

            Assert.Equal(P(PhaseKind.Failed, Stage.Baseline), f.Phase);
            Assert.Equal("error", f.ArState);
            Assert.Equal("Baseline", f.CurrentStage);
            Assert.Equal(ChecklistStatus.Pending, Status(f, "Baseline"));
        }

        [Fact]
        public void RecalibrationFailure_GoesBackToRecalPending_ForTheSameTarget()
        {
            var f = AtRest1();
            Go(f, FlowEvent.RestElapsed, FlowEvent.CalibrationStarted, FlowEvent.CalibrationFailed);

            Assert.Equal(P(PhaseKind.RecalPending, Stage.Training2), f.Phase);
            Assert.Equal("armed", f.ArState);

            Go(f, Recal); // retry succeeds
            Assert.Equal(P(PhaseKind.Training, Stage.Training2), f.Phase);
        }

        // ── NextStage (legacy AdvanceStage semantics) ────────────────────────

        [Fact]
        public void NextStage_WhilePendingRecalibration_IsIgnored()
        {
            var f = AtRest1();
            Go(f, FlowEvent.RestElapsed);

            Assert.False(f.Fire(FlowEvent.NextStage).Accepted);
            Assert.Equal(P(PhaseKind.RecalPending, Stage.Training2), f.Phase);
        }

        [Fact]
        public void NextStage_InRetention_IsIgnored_BecauseItIsTheFinalStage()
        {
            var f = AtRetention();
            Assert.False(f.Fire(FlowEvent.NextStage).Accepted);

            Go(f, FlowEvent.RetentionTargetReached);
            Assert.False(f.Fire(FlowEvent.NextStage).Accepted);
        }

        [Fact]
        public void NextStage_InTraining2_GoesToRecalPendingTraining3()
        {
            var f = AtTraining2();
            Go(f, FlowEvent.NextStage);
            Assert.Equal(P(PhaseKind.RecalPending, Stage.Training3), f.Phase);
        }

        // ── Pause / Resume ───────────────────────────────────────────────────

        [Fact]
        public void Pause_IsAnOverlay_ThatKeepsThePhase_AndReportsPausedToAr()
        {
            var f = AtTraining1();
            var phase = f.Phase;

            Go(f, FlowEvent.Pause);
            Assert.True(f.Paused);
            Assert.Equal(phase, f.Phase);
            Assert.Equal("paused", f.ArState);

            Go(f, FlowEvent.ResumeContinue);
            Assert.False(f.Paused);
            Assert.Equal("training", f.ArState);
            Assert.Equal(1, f.AttemptOf("Training1"));
        }

        // ── Redo (T9): only while data is being collected; restarts the stage from a fresh calibration ──

        [Fact]
        public void ResumeRedo_InTraining_BumpsTheAttempt_AndWaitsForARecalibrationOfTheSameStage()
        {
            var f = AtTraining1();
            Go(f, FlowEvent.Pause, FlowEvent.ResumeRedo);

            Assert.Equal(2, f.AttemptOf("Training1"));
            Assert.Equal(1, f.AttemptOf("Training2"));
            Assert.False(f.Paused);
            Assert.Equal(P(PhaseKind.RecalPending, Stage.Training1), f.Phase);
            Assert.Equal("Training1", f.CurrentStage);   // still the stage being redone, not the previous one
            Assert.Equal("armed", f.ArState);            // the AR shows its Ready button
            Assert.False(f.InRest);
        }

        [Fact]
        public void ResumeRedo_ThenReady_RestartsTheSameStage()
        {
            var f = AtTraining1();
            Go(f, FlowEvent.Pause, FlowEvent.ResumeRedo);

            Go(f, Recal);

            Assert.Equal(P(PhaseKind.Training, Stage.Training1), f.Phase);
            Assert.Equal("training", f.ArState);
            Assert.Equal(2, f.AttemptOf("Training1"));
        }

        [Fact]
        public void ResumeRedo_CanBeRepeated_AndEachRedoIsANewAttempt()
        {
            var f = AtTraining1();
            for (int i = 0; i < 3; i++)
            {
                Go(f, FlowEvent.Pause, FlowEvent.ResumeRedo);
                Go(f, Recal);
            }

            Assert.Equal(4, f.AttemptOf("Training1"));
            Assert.Equal(P(PhaseKind.Training, Stage.Training1), f.Phase);
        }

        [Fact]
        public void ResumeRedo_InBaseline_RestartsTheBaseline()
        {
            var f = AtBaseline();
            Go(f, FlowEvent.Pause, FlowEvent.ResumeRedo);

            Assert.Equal(P(PhaseKind.RecalPending, Stage.Baseline), f.Phase);
            Assert.Equal("Baseline", f.CurrentStage);
            Assert.Equal(2, f.AttemptOf("Baseline"));

            Go(f, Recal);
            Assert.Equal(P(PhaseKind.Baseline, Stage.Baseline), f.Phase);
            Assert.Equal("baseline", f.ArState);
            Assert.Equal(ChecklistStatus.Pending, Status(f, "Baseline"));
        }

        [Fact]
        public void ResumeRedo_InRetention_StaysARetentionStage_AndItsNotComplete()
        {
            var f = AtRetention();
            Go(f, FlowEvent.Pause, FlowEvent.ResumeRedo);

            Assert.Equal(P(PhaseKind.RecalPending, Stage.Retention), f.Phase);
            Assert.True(f.IsRetention);                  // AR feedback must stay off while waiting
            Assert.False(f.RetentionComplete);
            Assert.Equal("Retention", f.CurrentStage);

            Go(f, Recal);
            Assert.Equal(P(PhaseKind.Retention, Stage.Retention), f.Phase);
            Assert.Equal(2, f.AttemptOf("Retention"));
        }

        [Fact]
        public void ResumeRedo_DoesNotTouchMarkersOrTheRestHistory()
        {
            var f = AtRest1();
            Go(f, FlowEvent.Mark(FlowMarkers.Mc1), FlowEvent.RestElapsed);
            Go(f, Recal);                                  // Training2
            Go(f, FlowEvent.Pause, FlowEvent.ResumeRedo);

            Assert.True(f.IsMarked(FlowMarkers.Mc1));
            Assert.Equal(1, f.RestBlockJustFinished);
            Assert.Equal(ChecklistStatus.Done, Status(f, "Training1"));
        }

        [Fact]
        public void RecalibrationFailureDuringARedo_RetriesTheSameStage()
        {
            var f = AtTraining1();
            Go(f, FlowEvent.Pause, FlowEvent.ResumeRedo, FlowEvent.CalibrationStarted, FlowEvent.CalibrationFailed);

            Assert.Equal(P(PhaseKind.RecalPending, Stage.Training1), f.Phase);
            Assert.Equal("Training1", f.CurrentStage);

            Go(f, Recal);
            Assert.Equal(P(PhaseKind.Training, Stage.Training1), f.Phase);
        }

        [Fact]
        public void EndingWhileWaitingForARedoRecalibration_KeepsTheStageBeingRedone()
        {
            var f = AtTraining1();
            Go(f, FlowEvent.Pause, FlowEvent.ResumeRedo, FlowEvent.EndRecording);

            Assert.Equal("Training1", f.CurrentStage);
            Assert.Equal(PhaseKind.Ended, f.Phase.Kind);
        }

        public static IEnumerable<object[]> PhasesWhereRedoIsNotLegal() => new[]
        {
            new object[] { "Armed" }, new object[] { "Calibrating" }, new object[] { "Rest" },
            new object[] { "RecalPending" }, new object[] { "Recalibrating" }, new object[] { "RetentionDone" },
            new object[] { "Failed" },
        };

        private static ProtocolFlow InPhase(string name)
        {
            switch (name)
            {
                case "Armed": return AtArmed();
                case "Calibrating": { var f = AtArmed(); Go(f, FlowEvent.CalibrationStarted); return f; }
                case "Rest": return AtRest1();
                case "RecalPending": { var f = AtRest1(); Go(f, FlowEvent.RestElapsed); return f; }
                case "Recalibrating": { var f = AtRest1(); Go(f, FlowEvent.RestElapsed, FlowEvent.CalibrationStarted); return f; }
                case "RetentionDone": return AtRetentionDone();
                case "Failed": { var f = AtBaseline(); Go(f, FlowEvent.BaselineCompleted(false)); return f; }
            }
            throw new ArgumentException(name);
        }

        [Theory]
        [MemberData(nameof(PhasesWhereRedoIsNotLegal))]
        public void ResumeRedo_IsRefused_OutsideDataCollection_AndThePauseStays(string phaseName)
        {
            var f = InPhase(phaseName);
            var phase = f.Phase;
            Go(f, FlowEvent.Pause);

            Assert.False(f.CanRedo);
            var r = f.Fire(FlowEvent.ResumeRedo);

            Assert.False(r.Accepted);
            Assert.True(f.Paused);                       // nothing happened; the operator can still Continue
            Assert.Equal(phase, f.Phase);
            Assert.Equal(1, f.AttemptOf(f.CurrentStage));
        }

        [Fact]
        public void PendingStageEntry_IsTheTargetOfAnUpcomingRecalibration_AndNullOtherwise()
        {
            var f = AtRest1();
            Assert.Null(f.PendingStageEntry);                       // resting

            Go(f, FlowEvent.RestElapsed);
            Assert.Equal("Training2", f.PendingStageEntry);         // waiting for Ready
            Go(f, FlowEvent.CalibrationStarted);
            Assert.Equal("Training2", f.PendingStageEntry);         // recalibrating
            Go(f, FlowEvent.CalibrationCompleted);
            Assert.Null(f.PendingStageEntry);                       // entered
            Assert.Null(new ProtocolFlow().PendingStageEntry);
        }

        [Fact]
        public void PendingStageEntry_ForARedo_IsTheStageBeingRedone()
        {
            var f = AtTraining1();
            Go(f, FlowEvent.Pause, FlowEvent.ResumeRedo);

            Assert.Equal("Training1", f.PendingStageEntry);
            Assert.True(f.IsRedoRecalibration);
        }

        [Fact]
        public void IsRedoRecalibration_IsFalseForOrdinaryStageChanges_AndOnceTheRedoStarts()
        {
            var f = AtRest1();
            Go(f, FlowEvent.RestElapsed);
            Assert.False(f.IsRedoRecalibration);                    // a normal recalibration before Training2

            var g = AtTraining1();
            Go(g, FlowEvent.Pause, FlowEvent.ResumeRedo);
            Assert.True(g.IsRedoRecalibration);
            Go(g, Recal);
            Assert.False(g.IsRedoRecalibration);                    // the stage is running again
            Assert.False(new ProtocolFlow().IsRedoRecalibration);
        }

        [Fact]
        public void CanRedo_IsTrueExactlyWhileBaselineTrainingOrRetentionIsRunning()
        {
            Assert.True(AtBaseline().CanRedo);
            Assert.True(AtTraining1().CanRedo);
            Assert.True(AtTraining3().CanRedo);
            Assert.True(AtRetention().CanRedo);
            Assert.False(new ProtocolFlow().CanRedo);
            Assert.False(AtRest3().CanRedo);
        }

        [Fact]
        public void Resume_WithoutPause_AndDoublePause_AreIgnored()
        {
            var f = AtTraining1();

            Assert.False(f.Fire(FlowEvent.ResumeContinue).Accepted);
            Assert.False(f.Fire(FlowEvent.ResumeRedo).Accepted);
            Go(f, FlowEvent.Pause);
            Assert.False(f.Fire(FlowEvent.Pause).Accepted);
        }

        // ── markers ──────────────────────────────────────────────────────────

        [Fact]
        public void Mc1_IsMarkableOnlyInsideRest1()
        {
            var f = AtTraining1();
            Assert.False(f.CanMark(FlowMarkers.Mc1));

            Go(f, FlowEvent.BlockComplete);
            Assert.True(f.CanMark(FlowMarkers.Mc1));
            Assert.False(f.CanMark(FlowMarkers.Mc3));
            Assert.False(f.CanMark(FlowMarkers.NasaTlx1));

            Go(f, FlowEvent.Mark(FlowMarkers.Mc1));
            Assert.True(f.IsMarked(FlowMarkers.Mc1));
        }

        [Fact]
        public void Rest3_OpensMc3_Tlx1_AndImiPc_ButNotMc1()
        {
            var f = AtRest3();

            Assert.True(f.CanMark(FlowMarkers.Mc3));
            Assert.True(f.CanMark(FlowMarkers.NasaTlx1));
            Assert.True(f.CanMark(FlowMarkers.ImiPc));
            Assert.False(f.CanMark(FlowMarkers.Mc1));
            Assert.False(f.CanMark(FlowMarkers.NasaTlx2));
        }

        [Fact]
        public void Tlx2_OpensOnceRetentionIsComplete_AndStaysOpenAfterEnd()
        {
            var f = AtRetention();
            Assert.False(f.CanMark(FlowMarkers.NasaTlx2));

            Go(f, FlowEvent.RetentionTargetReached);
            Assert.True(f.CanMark(FlowMarkers.NasaTlx2));

            Go(f, FlowEvent.EndRecording);
            Assert.True(f.CanMark(FlowMarkers.NasaTlx2)); // legacy: CanMark is not cleared by End
        }

        [Fact]
        public void MissingExpectedMarkers_InTraining2_StillListsAnUnmarkedMc1()
        {
            var f = AtTraining2(); // Rest1 was entered → MC1 is due, and it was never marked
            Assert.Equal(new[] { "Manipulation Check (Block1)" }, f.MissingExpectedMarkers());
        }

        [Fact]
        public void MissingExpectedMarkers_FollowsTheReachedWindows()
        {
            var f = AtTraining1();
            Assert.Empty(f.MissingExpectedMarkers());

            Go(f, FlowEvent.BlockComplete);
            Assert.Equal(new[] { "Manipulation Check (Block1)" }, f.MissingExpectedMarkers());

            Go(f, FlowEvent.Mark(FlowMarkers.Mc1));
            Assert.Empty(f.MissingExpectedMarkers());
        }

        [Fact]
        public void MissingExpectedMarkers_AfterRest3_AndAfterRetention()
        {
            var f = AtRest3();
            Assert.Equal(
                new[] { "Manipulation Check (Block1)", "Manipulation Check (Block3)", "NASA-TLX Admin 1", "IMI-PC" },
                f.MissingExpectedMarkers());

            Go(f, FlowEvent.Mark(FlowMarkers.Mc3), FlowEvent.Mark(FlowMarkers.NasaTlx1), FlowEvent.Mark(FlowMarkers.ImiPc));
            Assert.Equal(new[] { "Manipulation Check (Block1)" }, f.MissingExpectedMarkers());

            Go(f, FlowEvent.RestElapsed);
            Go(f, Recal);
            Go(f, FlowEvent.RetentionTargetReached);
            Assert.Equal(
                new[] { "Manipulation Check (Block1)", "NASA-TLX Admin 2" },
                f.MissingExpectedMarkers());
        }

        [Fact]
        public void Mark_IsIgnored_WhenItsWindowIsNotOpen()
        {
            var f = AtTraining1();
            Assert.False(f.Fire(FlowEvent.Mark(FlowMarkers.Mc1)).Accepted);
            Assert.False(f.Fire(FlowEvent.Mark("not-a-marker")).Accepted);
            Assert.False(f.IsMarked(FlowMarkers.Mc1));
        }

        // ── checklist (replicates MainWindow.RefreshChecklist) ───────────────

        [Fact]
        public void Checklist_StartsAllPending_InProtocolOrder()
        {
            var f = AtArmed();
            var list = f.Checklist();

            Assert.Equal(
                new[] { "Baseline", "Training1", "MC (Block1)", "Training2", "Training3",
                        "MC (Block3)", "NASA-TLX #1", "IMI-PC", "Retention", "NASA-TLX #2" },
                list.Select(c => c.Label));
            Assert.All(list, c => Assert.Equal(ChecklistStatus.Pending, c.Status));
        }

        [Fact]
        public void Checklist_ProgressesWithTheProtocol()
        {
            var f = AtBaseline();
            Go(f, FlowEvent.BaselineCompleted(true));
            Assert.Equal(ChecklistStatus.Done, Status(f, "Baseline"));
            Assert.Equal(ChecklistStatus.Pending, Status(f, "Training1"));

            Go(f, Recal);
            Go(f, FlowEvent.BlockComplete); // Rest1
            Assert.Equal(ChecklistStatus.Done, Status(f, "Training1"));
            Assert.Equal(ChecklistStatus.Missing, Status(f, "MC (Block1)")); // due, not marked
            Assert.Equal(ChecklistStatus.Pending, Status(f, "Training2"));

            Go(f, FlowEvent.Mark(FlowMarkers.Mc1));
            Assert.Equal(ChecklistStatus.Done, Status(f, "MC (Block1)"));
        }

        [Fact]
        public void Checklist_Training2IsDone_OnlyOnceTraining3HasBeenEntered()
        {
            var f = AtTraining2();
            Go(f, FlowEvent.BlockComplete); // RecalPending(Training3); CurrentStage is still Training2
            Assert.Equal(ChecklistStatus.Pending, Status(f, "Training2"));

            Go(f, Recal);
            Assert.Equal(ChecklistStatus.Done, Status(f, "Training2"));
        }

        [Fact]
        public void Checklist_AfterEnd_ClearsBaselineButKeepsTheRest()
        {
            var f = AtRetentionDone();
            Go(f, FlowEvent.EndRecording);

            // legacy: EndFlow resets the pipeline, so BaselineProfile is null again → Baseline shows Pending.
            Assert.Equal(ChecklistStatus.Pending, Status(f, "Baseline"));
            Assert.Equal(ChecklistStatus.Done, Status(f, "Training1"));
            Assert.Equal(ChecklistStatus.Done, Status(f, "Retention"));
        }

        // ── conditions are isolated ──────────────────────────────────────────

        [Fact]
        public void StartingASecondCondition_DoesNotInheritAnythingFromTheFirst()
        {
            var f = AtRest3();
            Go(f, FlowEvent.Mark(FlowMarkers.Mc3));
            Go(f, FlowEvent.Pause, FlowEvent.ResumeContinue);
            Go(f, FlowEvent.EndRecording);

            Go(f, FlowEvent.StartCondition);

            Assert.Equal(P(PhaseKind.Armed, Stage.Baseline), f.Phase);
            Assert.Equal(0, f.RestBlockJustFinished);
            Assert.False(f.RetentionComplete);
            Assert.False(f.IsMarked(FlowMarkers.Mc3));
            Assert.Equal(1, f.AttemptOf("Training3"));
            Assert.Empty(f.MissingExpectedMarkers());
            Assert.All(f.Checklist(), c => Assert.Equal(ChecklistStatus.Pending, c.Status));
        }

        // ── End ──────────────────────────────────────────────────────────────

        [Fact]
        public void EndRecording_FromAnyRecordingPhase_EndsAndKeepsTheCurrentStage()
        {
            var f = AtRest1();
            Go(f, FlowEvent.EndRecording);

            Assert.Equal(P(PhaseKind.Ended, Stage.Training1), f.Phase);
            Assert.Equal("Training1", f.CurrentStage);
            Assert.False(f.InRest);
            Assert.Equal(new[] { "Manipulation Check (Block1)" }, f.MissingExpectedMarkers());
            Assert.False(f.Fire(FlowEvent.EndRecording).Accepted);
        }

        [Fact]
        public void EndRecording_WhilePendingRecalibration_KeepsThePreviousStage()
        {
            var f = AtTraining2();
            Go(f, FlowEvent.BlockComplete); // RecalPending(Training3)
            Go(f, FlowEvent.EndRecording);

            Assert.Equal("Training2", f.CurrentStage);
        }

        // ── Transitioned event ───────────────────────────────────────────────

        [Fact]
        public void Transitioned_IsRaisedForAcceptedEventsOnly_WithBeforeAndAfter()
        {
            var f = AtArmed();
            var seen = new List<(FlowEventKind Kind, Phase Before, Phase After)>();
            f.Transitioned += (e, before, after) => seen.Add((e.Kind, before, after));

            f.Fire(FlowEvent.BlockComplete);        // ignored in Armed
            f.Fire(FlowEvent.CalibrationStarted);   // accepted

            var only = Assert.Single(seen);
            Assert.Equal(FlowEventKind.CalibrationStarted, only.Kind);
            Assert.Equal(P(PhaseKind.Armed, Stage.Baseline), only.Before);
            Assert.Equal(P(PhaseKind.Calibrating, Stage.Baseline), only.After);
        }

        [Fact]
        public void Transitioned_IsAlsoRaisedForAcceptedEventsThatKeepThePhase()
        {
            var f = AtTraining1();
            int count = 0;
            f.Transitioned += (_, _, _) => count++;

            f.Fire(FlowEvent.Pause);
            f.Fire(FlowEvent.ResumeContinue);

            Assert.Equal(2, count);
        }

        // ═════════════════════════════════════════════════════════════════════
        //  [KnownIssue] 以下固定的是**有缺陷的现状**(docs/flow-review-2026-10-27.md §2),
        //  对应修复任务落地时翻转断言。
        // ═════════════════════════════════════════════════════════════════════

        [Fact, Trait("Category", "KnownIssue")]
        public void KnownIssue_2_2_NextStage_IsAccepted_WhilePaused()
        {
            var f = AtTraining1();
            Go(f, FlowEvent.Pause);

            var r = f.Fire(FlowEvent.NextStage);

            Assert.True(r.Accepted);                 // T10: should be rejected while paused
            Assert.True(f.Paused);
            Assert.Equal(P(PhaseKind.RecalPending, Stage.Training2), f.Phase);
        }

        // §2-4 (fixed 2026-10-27 after it deadlocked a real run): resuming from a pause must put the AR client back
        // in the state it was in before the pause, not a state recomputed from CurrentStage.
        [Fact]
        public void ArStateAfterResume_InRest_IsRest()
        {
            var f = AtRest1();
            Go(f, FlowEvent.Pause);

            Assert.Equal("rest", f.ArStateAfterResume);
        }

        [Fact]
        public void ArStateAfterResume_WhileWaitingForRecalibration_IsArmed_SoTheReadyButtonComesBack()
        {
            var f = AtRest1();
            Go(f, FlowEvent.RestElapsed, FlowEvent.Pause);

            Assert.Equal("armed", f.ArStateAfterResume);
        }

        [Fact]
        public void ArStateAfterResume_AlwaysEqualsTheStateBeforeThePause()
        {
            var phases = new Dictionary<string, ProtocolFlow>
            {
                ["Armed"] = AtArmed(),
                ["Baseline"] = AtBaseline(),
                ["Training1"] = AtTraining1(),
                ["Rest1"] = AtRest1(),
                ["Training3"] = AtTraining3(),
                ["Rest3"] = AtRest3(),
                ["Retention"] = AtRetention(),
                ["RetentionDone"] = AtRetentionDone(),
            };
            // plus phases that are mid-recalibration or waiting for Ready
            var pending = AtRest1(); Go(pending, FlowEvent.RestElapsed);
            phases["RecalPending"] = pending;
            var recal = AtRest1(); Go(recal, FlowEvent.RestElapsed, FlowEvent.CalibrationStarted);
            phases["Recalibrating"] = recal;
            var failed = AtBaseline(); Go(failed, FlowEvent.BaselineCompleted(false));
            phases["Failed"] = failed;

            foreach (var (name, f) in phases)
            {
                string before = f.ArState;
                Go(f, FlowEvent.Pause);
                Assert.Equal(before, f.ArStateAfterResume);
                Assert.Equal("paused", f.ArState);
                Go(f, FlowEvent.ResumeContinue);
                Assert.Equal(before, f.ArState);
            }
        }

        [Fact, Trait("Category", "KnownIssue")]
        public void KnownIssue_2_5_NextStage_FromTraining1_SkipsRest_AndNoMarkerEverBecomesDue()
        {
            var f = AtTraining1();
            Go(f, FlowEvent.NextStage);

            Assert.Equal(P(PhaseKind.RecalPending, Stage.Training2), f.Phase); // T11: skip is recorded explicitly
            Assert.Equal(0, f.RestBlockJustFinished);
            Assert.Empty(f.MissingExpectedMarkers());                          // silent: no MC1 reminder at End
            Assert.Equal(ChecklistStatus.Pending, Status(f, "Training1"));     // even though Training1 was left
        }

        [Fact, Trait("Category", "KnownIssue")]
        public void KnownIssue_2_5_NextStage_FromTraining3_SkipsRest3AndItsQuestionnaires()
        {
            var f = AtTraining3();
            Go(f, FlowEvent.NextStage);

            Assert.Equal(P(PhaseKind.RecalPending, Stage.Retention), f.Phase);
            Assert.Equal(1, f.RestBlockJustFinished);                          // only Rest1 happened, Rest3 never did
            // Only the (unmarked) MC1 from the Rest1 that did happen is listed; MC3 / NASA-TLX#1 / IMI-PC never appear.
            Assert.Equal(new[] { "Manipulation Check (Block1)" }, f.MissingExpectedMarkers());
        }

        [Fact, Trait("Category", "KnownIssue")]
        public void KnownIssue_2_5_NextStage_InRest_SkipsTheRemainingRestTime()
        {
            var f = AtRest1();
            Go(f, FlowEvent.NextStage);

            Assert.Equal(P(PhaseKind.RecalPending, Stage.Training2), f.Phase);
            Assert.False(f.CanMark(FlowMarkers.Mc1)); // window closes immediately
        }

        [Fact, Trait("Category", "KnownIssue")]
        public void KnownIssue_2_6_NextStage_InBaseline_EntersTraining1_WithoutAnyBaselineProfile()
        {
            var f = AtBaseline();
            Go(f, FlowEvent.NextStage);

            Assert.Equal(P(PhaseKind.RecalPending, Stage.Training1), f.Phase); // T11: ignored in Baseline
            Assert.Equal(ChecklistStatus.Pending, Status(f, "Baseline"));
        }

        [Fact, Trait("Category", "KnownIssue")]
        public void KnownIssue_2_6_NextStage_FromArmed_ArmsARecalibrationThatCanNeverBeTriggered()
        {
            var f = AtArmed(); // never calibrated
            Go(f, FlowEvent.NextStage);

            Assert.Equal(P(PhaseKind.RecalPending, Stage.Training1), f.Phase);
        }

        [Fact, Trait("Category", "KnownIssue")]
        public void KnownIssue_2_12_MarkersCannotBeAddedAfterTheirWindowCloses()
        {
            var f = AtRest1();
            Go(f, FlowEvent.RestElapsed);

            Assert.False(f.CanMark(FlowMarkers.Mc1));                          // T14: should still be markable (late)
            Assert.False(f.Fire(FlowEvent.Mark(FlowMarkers.Mc1)).Accepted);
            Assert.Equal(ChecklistStatus.Missing, Status(f, "MC (Block1)"));   // due, but unclickable
            Assert.Equal(new[] { "Manipulation Check (Block1)" }, f.MissingExpectedMarkers());
        }

        // §2-14 (fixed with the Restart-Condition work): a pause must not outlive the recording it belongs to,
        // otherwise the next condition starts "paused" and silently writes no data rows.
        [Fact]
        public void EndRecording_ClearsAPendingPause()
        {
            var f = AtTraining1();
            Go(f, FlowEvent.Pause, FlowEvent.EndRecording);

            Assert.False(f.Paused);
            Assert.Equal("ended", f.ArState);
        }

        [Fact]
        public void StartCondition_ClearsAPause()
        {
            var f = AtTraining1();
            Go(f, FlowEvent.Pause);

            Go(f, FlowEvent.StartCondition);          // e.g. restart after an accident while paused

            Assert.False(f.Paused);
            Assert.Equal("armed", f.ArState);
        }

        [Fact]
        public void RestartCondition_FromAnyRecordingPhase_ReturnsToAGenuinelyFreshArmedState()
        {
            var f = AtRest3();
            Go(f, FlowEvent.Mark(FlowMarkers.Mc3), FlowEvent.Pause);

            Go(f, FlowEvent.EndRecording, FlowEvent.StartCondition);   // what MainWindow's restart does

            Assert.Equal(P(PhaseKind.Armed, Stage.Baseline), f.Phase);
            Assert.False(f.Paused);
            Assert.Equal(0, f.RestBlockJustFinished);
            Assert.False(f.IsMarked(FlowMarkers.Mc3));
            Assert.Empty(f.MissingExpectedMarkers());
            Assert.All(f.Checklist(), c => Assert.Equal(ChecklistStatus.Pending, c.Status));
        }
    }
}
