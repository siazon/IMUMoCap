// IMUMoCap/Protocol/ProtocolFlow.cs
using System;
using System.Collections.Generic;
using IMUMoCap.Model;

namespace IMUMoCap.Protocol
{
    /// <summary>实验阶段;名字与 MainWindow.ConditionStages 里的字符串一致。</summary>
    public enum Stage { Baseline, Training1, Training2, Training3, Retention }

    public enum PhaseKind
    {
        Idle, Armed, Calibrating, Baseline, Failed,
        RecalPending, Recalibrating,
        Training, Rest, Retention, RetentionDone,
        Ended,
    }

    /// <summary>
    /// 流程所处的位置。Stage 的含义随 Kind 而定:Training/Retention/RetentionDone 是该阶段本身;
    /// Rest 是刚结束的训练阶段(Training1 或 Training3);RecalPending/Recalibrating 是即将进入的目标阶段;
    /// Ended 是结束时的 CurrentStage;Idle 为 null。
    /// </summary>
    public readonly record struct Phase(PhaseKind Kind, Stage? Stage = null);

    public enum FlowEventKind
    {
        StartCondition,
        CalibrationStarted, CalibrationCompleted, CalibrationFailed,
        BaselineCompleted, BlockComplete, RestElapsed, RetentionTargetReached,
        NextStage, Pause, ResumeContinue, ResumeRedo, Mark, EndRecording,
    }

    public readonly record struct FlowEvent(FlowEventKind Kind, string? Marker = null, bool Valid = true)
    {
        public static FlowEvent StartCondition => new(FlowEventKind.StartCondition);
        public static FlowEvent CalibrationStarted => new(FlowEventKind.CalibrationStarted);
        public static FlowEvent CalibrationCompleted => new(FlowEventKind.CalibrationCompleted);
        public static FlowEvent CalibrationFailed => new(FlowEventKind.CalibrationFailed);
        public static FlowEvent BlockComplete => new(FlowEventKind.BlockComplete);
        public static FlowEvent RestElapsed => new(FlowEventKind.RestElapsed);
        public static FlowEvent RetentionTargetReached => new(FlowEventKind.RetentionTargetReached);
        public static FlowEvent NextStage => new(FlowEventKind.NextStage);
        public static FlowEvent Pause => new(FlowEventKind.Pause);
        public static FlowEvent ResumeContinue => new(FlowEventKind.ResumeContinue);
        public static FlowEvent ResumeRedo => new(FlowEventKind.ResumeRedo);
        public static FlowEvent EndRecording => new(FlowEventKind.EndRecording);
        public static FlowEvent BaselineCompleted(bool valid) => new(FlowEventKind.BaselineCompleted, Valid: valid);
        public static FlowEvent Mark(string marker) => new(FlowEventKind.Mark, marker);
    }

    public readonly record struct FireResult(bool Accepted, string Reason);

    /// <summary>审计标记名,与 ExperimentRecorder.MarkStageEvent / ChecklistItem.MarkerStage 使用的字符串一致。</summary>
    public static class FlowMarkers
    {
        public const string Mc1 = "ManipulationCheck_AfterBlock1";
        public const string Mc3 = "ManipulationCheck_AfterBlock3";
        public const string NasaTlx1 = "NASA-TLX_Admin1";
        public const string ImiPc = "IMI-PC";
        public const string NasaTlx2 = "NASA-TLX_Admin2";
    }

    /// <summary>
    /// 实验流程状态机(纯逻辑,不依赖 WPF/硬件/计时器)。
    ///
    /// 这是**当前行为版**(docs/flow-review-2026-10-27.md §8 T1):复刻 MainWindow 里
    /// _currentStage / _inRest / _pendingStageEntry / _isRetention / _retentionComplete /
    /// _restBlockJustFinished / *Marked / IsPaused 的现有语义,**包括 §2 记录的已知缺陷**,
    /// 这样 T6 的影子模式才能与旧代码逐项比对。新规则由 T9–T14 逐条修改这里的转换并加测试。
    ///
    /// 只存"事实"(Phase、Paused、已 Mark 集合、各阶段 attempt、几个历史标志);
    /// InRest / IsRetention / RetentionComplete / RestBlockJustFinished / CanMark / 缺失 marker /
    /// Flow 清单 / AR state 都由事实推导,不单独存储。
    /// </summary>
    public sealed class ProtocolFlow
    {
        /// <summary>与 MainWindow.ConditionStages 相同的阶段名顺序。</summary>
        public static readonly string[] StageNames = { "Baseline", "Training1", "Training2", "Training3", "Retention" };

        private static readonly FireResult Accept = new(true, "");
        private static FireResult Ignore(string why) => new(false, why);

        // ── 事实 ─────────────────────────────────────────────────────────────

        public Phase Phase { get; private set; } = new(PhaseKind.Idle);

        /// <summary>暂停叠加标志。StartCondition / EndRecording 会清除它(§2-14)。</summary>
        public bool Paused { get; private set; }

        private readonly HashSet<string> _marked = new();
        private readonly Dictionary<string, int> _attempts = new();
        private bool _rest1Entered, _rest3Entered;
        private bool _retentionCompleted;
        private bool _baselineDone; // 旧代码里是 _pipeline.BaselineProfile != null

        /// <summary>每个被接受的事件触发一次(包括不改变 Phase 的,如 Pause/Mark)。参数:事件、之前的 Phase、之后的 Phase。</summary>
        public event Action<FlowEvent, Phase, Phase>? Transitioned;

        // ── 推导值 ────────────────────────────────────────────────────────────

        public bool IsRecording => Phase.Kind is not (PhaseKind.Idle or PhaseKind.Ended);
        public bool InRest => Phase.Kind == PhaseKind.Rest;
        public bool IsRetention => Phase.Kind is PhaseKind.Retention or PhaseKind.RetentionDone;
        public bool RetentionComplete => _retentionCompleted;

        /// <summary>旧 _restBlockJustFinished:进入过 Rest3 为 3,进入过 Rest1 为 1,否则 0。</summary>
        public int RestBlockJustFinished => _rest3Entered ? 3 : _rest1Entered ? 1 : 0;

        private Stage? CurrentStageValue => Phase.Kind switch
        {
            PhaseKind.Armed or PhaseKind.Calibrating or PhaseKind.Baseline or PhaseKind.Failed => Stage.Baseline,
            // 旧行为:等待/进行阶段间重校准期间 _currentStage 仍是上一个阶段,直到 EnterStage
            PhaseKind.RecalPending or PhaseKind.Recalibrating => Previous(Phase.Stage!.Value),
            PhaseKind.Training or PhaseKind.Rest or PhaseKind.Retention or PhaseKind.RetentionDone or PhaseKind.Ended => Phase.Stage,
            _ => null,
        };

        /// <summary>与旧 _currentStage 相同的字符串;Idle 时为 ""。</summary>
        public string CurrentStage => CurrentStageValue?.ToString() ?? "";

        public int AttemptOf(string stage) => _attempts.GetValueOrDefault(stage, 1);
        public bool IsMarked(string marker) => _marked.Contains(marker);

        /// <summary>旧行为:marker 只在其窗口内可点(Rest 窗口随离开 Rest 关闭);NASA-TLX#2 在 Retention 完成后一直可点。</summary>
        public bool CanMark(string marker) => marker switch
        {
            FlowMarkers.Mc1 => Phase is { Kind: PhaseKind.Rest, Stage: Stage.Training1 },
            FlowMarkers.Mc3 or FlowMarkers.NasaTlx1 or FlowMarkers.ImiPc => Phase is { Kind: PhaseKind.Rest, Stage: Stage.Training3 },
            FlowMarkers.NasaTlx2 => _retentionCompleted,
            _ => false,
        };

        /// <summary>复刻 MainWindow.MissingExpectedMarkers:只提示流程实际走到过的窗口。</summary>
        public IReadOnlyList<string> MissingExpectedMarkers()
        {
            var missing = new List<string>();
            if (RestBlockJustFinished >= 1 && !IsMarked(FlowMarkers.Mc1))
                missing.Add("Manipulation Check (Block1)");
            if (RestBlockJustFinished == 3)
            {
                if (!IsMarked(FlowMarkers.Mc3)) missing.Add("Manipulation Check (Block3)");
                if (!IsMarked(FlowMarkers.NasaTlx1)) missing.Add("NASA-TLX Admin 1");
                if (!IsMarked(FlowMarkers.ImiPc)) missing.Add("IMI-PC");
            }
            if (RetentionComplete && !IsMarked(FlowMarkers.NasaTlx2))
                missing.Add("NASA-TLX Admin 2");
            return missing;
        }

        /// <summary>复刻 MainWindow.RefreshChecklist 的 10 行状态(顺序与 ChecklistDefs 一致)。</summary>
        public IReadOnlyList<(string Label, ChecklistStatus Status)> Checklist()
        {
            int idx = Array.IndexOf(StageNames, CurrentStage);
            bool block1Window = RestBlockJustFinished >= 1;
            bool block3Window = RestBlockJustFinished == 3;

            static ChecklistStatus Done(bool done) => done ? ChecklistStatus.Done : ChecklistStatus.Pending;
            ChecklistStatus Marker(string marker, bool due) =>
                IsMarked(marker) ? ChecklistStatus.Done : due ? ChecklistStatus.Missing : ChecklistStatus.Pending;

            return new (string, ChecklistStatus)[]
            {
                ("Baseline", Done(_baselineDone)),
                ("Training1", Done(block1Window)),
                ("MC (Block1)", Marker(FlowMarkers.Mc1, block1Window)),
                ("Training2", Done(idx >= (int)Stage.Training3)),
                ("Training3", Done(block3Window)),
                ("MC (Block3)", Marker(FlowMarkers.Mc3, block3Window)),
                ("NASA-TLX #1", Marker(FlowMarkers.NasaTlx1, block3Window)),
                ("IMI-PC", Marker(FlowMarkers.ImiPc, block3Window)),
                ("Retention", Done(RetentionComplete)),
                ("NASA-TLX #2", Marker(FlowMarkers.NasaTlx2, RetentionComplete)),
            };
        }

        /// <summary>应当呈现给 AR 眼镜的 state;暂停(且正在录制)时为 "paused"。</summary>
        public string ArState => Paused && IsRecording ? "paused" : ArStateIgnoringPause;

        private string ArStateIgnoringPause => Phase.Kind switch
        {
            PhaseKind.Idle => "waiting",
            PhaseKind.Armed or PhaseKind.RecalPending => "armed",
            PhaseKind.Calibrating or PhaseKind.Recalibrating => "calibrating",
            PhaseKind.Baseline => "baseline",
            PhaseKind.Failed => "error",
            PhaseKind.Training => "training",
            PhaseKind.Rest => "rest",
            PhaseKind.Retention or PhaseKind.RetentionDone => "retention",
            PhaseKind.Ended => "ended",
            _ => "waiting",
        };

        /// <summary>
        /// Resume 后应发给 AR 的 state:暂停前 AR 所处的状态,即不含暂停叠加的 <see cref="ArState"/>。
        /// (§2-4:旧实现按 CurrentStage 重新推一个 state,在等待 Ready / Rest 期间会发出 baseline/training,
        /// 导致眼镜上没有 Ready 按钮、PC 的 Ready 按钮也被禁用,流程死锁。2026-10-27 已修,MainWindow 改为重发暂停前的 state。)
        /// </summary>
        public string ArStateAfterResume => ArStateIgnoringPause;

        // ── 事件入口 ──────────────────────────────────────────────────────────

        /// <summary>唯一的状态变更入口。不合法/不适用的事件返回 Accepted=false,状态保持不变。</summary>
        public FireResult Fire(FlowEvent e)
        {
            var before = Phase;
            var result = Apply(e);
            if (result.Accepted) Transitioned?.Invoke(e, before, Phase);
            return result;
        }

        private FireResult Apply(FlowEvent e)
        {
            switch (e.Kind)
            {
                case FlowEventKind.StartCondition:
                    // 任何时候都可以开始新 condition(含\"重新开始\");重置每个 condition 的事实,并清除暂停(§2-14)。
                    ResetConditionFacts();
                    Paused = false;
                    Phase = new(PhaseKind.Armed, Stage.Baseline);
                    return Accept;

                case FlowEventKind.CalibrationStarted:
                    switch (Phase.Kind)
                    {
                        case PhaseKind.Armed: Phase = new(PhaseKind.Calibrating, Stage.Baseline); return Accept;
                        case PhaseKind.RecalPending: Phase = new(PhaseKind.Recalibrating, Phase.Stage); return Accept;
                    }
                    return Ignore($"calibration cannot start in {Phase.Kind}");

                case FlowEventKind.CalibrationCompleted:
                    switch (Phase.Kind)
                    {
                        case PhaseKind.Calibrating:
                            Phase = new(PhaseKind.Baseline, Stage.Baseline);
                            return Accept;
                        case PhaseKind.Recalibrating:
                            EnterStage(Phase.Stage!.Value);
                            return Accept;
                    }
                    return Ignore($"calibration cannot complete in {Phase.Kind}");

                case FlowEventKind.CalibrationFailed:
                    switch (Phase.Kind)
                    {
                        // 校准失败一律可重试(O5、§2-17):首次校准失败回到 Armed,再点 Ready 即可
                        case PhaseKind.Calibrating: Phase = new(PhaseKind.Armed, Stage.Baseline); return Accept;
                        case PhaseKind.Recalibrating: Phase = new(PhaseKind.RecalPending, Phase.Stage); return Accept;
                    }
                    return Ignore($"calibration cannot fail in {Phase.Kind}");

                case FlowEventKind.BaselineCompleted:
                    if (Phase.Kind != PhaseKind.Baseline) return Ignore($"baseline cannot complete in {Phase.Kind}");
                    if (!e.Valid)
                    {
                        Phase = new(PhaseKind.Failed, Stage.Baseline);
                        return Accept;
                    }
                    _baselineDone = true;
                    Phase = new(PhaseKind.RecalPending, Stage.Training1);
                    return Accept;

                case FlowEventKind.BlockComplete:
                    if (Phase.Kind != PhaseKind.Training) return Ignore($"no training block is running in {Phase.Kind}");
                    switch (Phase.Stage)
                    {
                        case Stage.Training1: _rest1Entered = true; Phase = new(PhaseKind.Rest, Stage.Training1); break;
                        case Stage.Training3: _rest3Entered = true; Phase = new(PhaseKind.Rest, Stage.Training3); break;
                        default: Phase = new(PhaseKind.RecalPending, Next(Phase.Stage!.Value)); break; // Block2 → Block3: no rest
                    }
                    return Accept;

                case FlowEventKind.RestElapsed:
                    if (Phase.Kind != PhaseKind.Rest) return Ignore($"not resting in {Phase.Kind}");
                    Phase = new(PhaseKind.RecalPending, Next(Phase.Stage!.Value));
                    return Accept;

                case FlowEventKind.RetentionTargetReached:
                    if (Phase.Kind != PhaseKind.Retention) return Ignore($"retention is not running in {Phase.Kind}");
                    _retentionCompleted = true;
                    Phase = new(PhaseKind.RetentionDone, Stage.Retention);
                    return Accept;

                case FlowEventKind.NextStage:
                    // 旧 AdvanceStage:无条件推进到 CurrentStage 的下一个阶段,不经过 Rest(§2-5)、
                    // 不检查 Paused(§2-2)、不检查 BaselineProfile(§2-6);只有"已在等待重校准"和"已是最后阶段"时才不动。
                    if (!IsRecording) return Ignore("no active condition recording");
                    if (Phase.Kind is PhaseKind.RecalPending or PhaseKind.Recalibrating)
                        return Ignore("already awaiting recalibration for the next stage");
                    var next = Next(CurrentStageValue!.Value);
                    if (next == null) return Ignore("already at the final stage");
                    Phase = new(PhaseKind.RecalPending, next);
                    return Accept;

                case FlowEventKind.Pause:
                    if (!IsRecording) return Ignore("no active recording to pause");
                    if (Paused) return Ignore("already paused");
                    Paused = true;
                    return Accept;

                case FlowEventKind.ResumeContinue:
                    if (!IsRecording) return Ignore("no active recording");
                    if (!Paused) return Ignore("not paused");
                    Paused = false;
                    return Accept;

                case FlowEventKind.ResumeRedo:
                    if (!IsRecording) return Ignore("no active recording");
                    if (!Paused) return Ignore("not paused");
                    // 旧行为(§2-1、§2-4):任何 Phase 下都只是 attempt+1,不重置计数器,Rest/等待校准期间也一样。
                    _attempts[CurrentStage] = AttemptOf(CurrentStage) + 1;
                    Paused = false;
                    return Accept;

                case FlowEventKind.Mark:
                    if (e.Marker == null || !CanMark(e.Marker)) return Ignore($"marker window not open: {e.Marker}");
                    _marked.Add(e.Marker);
                    return Accept;

                case FlowEventKind.EndRecording:
                    if (!IsRecording) return Ignore("no active recording");
                    // 旧 EndFlow:_currentStage 保持不变;_retentionComplete / _restBlockJustFinished / Marked 保持;
                    // 但 _pipeline.Reset() 会清掉 BaselineProfile。
                    Phase = new(PhaseKind.Ended, CurrentStageValue);
                    _baselineDone = false;
                    Paused = false; // a pause must not outlive its recording (§2-14)
                    return Accept;
            }
            return Ignore($"unknown event {e.Kind}");
        }

        // 旧 EnterStage:阶段间重校准完成后真正进入该阶段。
        private void EnterStage(Stage stage)
        {
            if (stage == Stage.Retention)
            {
                _retentionCompleted = false;
                Phase = new(PhaseKind.Retention, Stage.Retention);
            }
            else
            {
                Phase = new(PhaseKind.Training, stage);
            }
        }

        private void ResetConditionFacts()
        {
            _marked.Clear();
            _attempts.Clear();
            _rest1Entered = _rest3Entered = false;
            _retentionCompleted = false;
            _baselineDone = false;
        }

        private static Stage? Next(Stage s) => s == Stage.Retention ? null : (Stage)((int)s + 1);
        private static Stage Previous(Stage s) => (Stage)((int)s - 1);
    }
}
