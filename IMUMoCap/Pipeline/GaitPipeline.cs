// IMUMoCap/Pipeline/GaitPipeline.cs
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using IMUMoCap.Methods;
using IMUMoCap.Pipeline.Models;
using XDA;

namespace IMUMoCap.Pipeline
{
    /// <summary>
    /// 流水线编排类。持有所有模块实例，对外暴露：
    ///   - ProcessPacket(ImuRole, uint, XsDataPacket)：主入口，每包调用
    ///   - OnFpaResult 事件：每步输出一个 FpaResult
    ///   - OnCalibrationStateChanged 事件
    ///   - OnBaselineProgress 事件（当前有效步数）
    ///
    /// MainWindow 只与 GaitPipeline 交互，不直接接触任何分析模块。
    /// </summary>
    public sealed class GaitPipeline
    {
        // ── 子模块 ────────────────────────────────────────────────────────────
        private readonly ImuFrameCollector       _frameCollector = new();
        private readonly DataQualityGate         _gate           = new();
        private readonly CalibrationProcessor    _calibration    = new();
        private readonly GaitEventDetector       _gait           = new();
        private readonly MotionContextDetector   _motion         = new();
        private readonly ProgressionDirEstimator _pd             = new();
        private readonly BaselineProcessor       _baseline       = new();
        private readonly FpaEngine               _fpa            = new();

        // ── 可调参数（UI 实时更新）──────────────────────────────────────────────
        public PipelineParams Params { get; } = new();

        // ── 诊断数据缓冲区（滑动窗，最长 20 分钟 @100Hz）────────────────────────
        private const int DiagnosticsCapacity = 120_000;
        private readonly Queue<DiagnosticsRow> _diagnosticsBuffer = new();

        // TEMP：帧时间戳→AR广播延迟测量，用完删除。
        private readonly ConcurrentDictionary<long, long> _frameEntryTicks = new();

        // ── 事件 ──────────────────────────────────────────────────────────────
        public event Action<FpaResult,bool >?        OnFpaResult;
        public event Action<CalibrationState>? OnCalibrationStateChanged;
        public event Action<int, int>?         OnBaselineProgress;  // (stepsL, stepsR)
        public event Action<BaselineProfile?>? OnBaselineCompleted; // auto- or manual-finalize
        public event Action<string>?           OnLog;
        public event Action<DiagnosticsRow>?   OnDiagnosticsFrame;
        public event Action<string, bool, StepExclusionReason?>? OnStepOutcome;
        public event Action?                   OnTrainingBlockComplete; // both feet reached TrainingBlockTargetSteps

        public GaitPipeline()
        {
            _fpa.OnStepOutcome += (foot, emitted, reason) => OnStepOutcome?.Invoke(foot, emitted, reason);
        }

        // ── 状态 ──────────────────────────────────────────────────────────────
        public CalibrationProfile? CalibrationProfile => _calibration.Profile;
        public BaselineProfile?    BaselineProfile    { get; private set; }
        public bool InBaseline  { get; private set; }
        public bool InTraining  { get; private set; }

        /// <summary>
        /// 只有 Start Condition 打开录制后才为 true。为 false 时 TriggerCalibration() 不生效，
        /// 防止操作员在正式开始前误触发校准+baseline，导致这段数据没被录制却已耗尽（数据漏存）。
        /// </summary>
        public bool CalibrationArmed { get; set; } = false;

        // 协议 §2.4 渐进式难度：容差带系数 α 随训练块递减（Block1=1.5, Block2=1.0, Block3=0.5）
        private static readonly float[] TrainingBlockAlphas = { 1.5f, 1.0f, 0.5f };
        public float CurrentToleranceAlpha => _fpa.ToleranceAlpha;

        // 协议 §2.6：每个 training block 在两脚均达到 100 个有效步时自动结束（另有停滞保护超时上限，由 MainWindow 的计时器负责，非并列退出条件）
        public const int TrainingBlockTargetSteps = 100;
        private int  _trainingStepsL, _trainingStepsR;
        private bool _trainingBlockDone;

        public int TrainingStepsL => _trainingStepsL;
        public int TrainingStepsR => _trainingStepsR;

        /// <summary>操作员切换到 Training1/2/3 时调用，收紧本训练块的容差带并重置步数计数。</summary>
        public void SetTrainingBlock(int blockNumber1Based)
        {
            int idx = Math.Clamp(blockNumber1Based - 1, 0, TrainingBlockAlphas.Length - 1);
            _fpa.ToleranceAlpha = TrainingBlockAlphas[idx];
            _trainingStepsL = _trainingStepsR = 0;
            _trainingBlockDone = false;
            OnLog?.Invoke($"Training block {blockNumber1Based}: tolerance α={_fpa.ToleranceAlpha:F1} " +
                $"(w=max({IMUMoCap.Pipeline.Models.BaselineProfile.ToleranceWMinDeg:F0}°, α·SD)), target {TrainingBlockTargetSteps} valid steps/foot");
        }

        // ── 主入口 ────────────────────────────────────────────────────────────

        /// <summary>每个 IMU 的单包调用，内部组装 bundle 再进流水线。</summary>
        public void ProcessPacket(ImuRole role, uint deviceId, XsDataPacket packet)
        {
            var (_, bundle) = _frameCollector.Process(role, deviceId, packet);
            if (bundle?.IsComplete == true)
                Process(bundle);
        }

        private void SyncParams()
        {
            // CalibrationProcessor
            _calibration.StaticGyroThreshold   = Params.StaticGyroThreshold;

            // GaitEventDetector
            _gait.FreeAccStanceThreshold = Params.StanceFreeAccThreshold;
            _gait.GyroThreshold          = Params.StanceGyroThreshold;
            _gait.FootPitchThreshold     = Params.StanceFootPitchThreshold;

            // FpaEngine
            _fpa.ContextConfidenceThreshold = Params.PdConfidenceThreshold;
            _fpa.PdStabilityThreshold       = Params.PdStabilityThreshold;

            // BaselineProcessor
            _baseline.MinValidSteps = Params.MinBaselineSteps;
            _baseline.ImbalanceRatioThreshold = Params.BaselineImbalanceRatioThreshold;
        }

        // ── 录制器（可选，仅供录制/测试用）─────────────────────────────────────
        public BundleRecorder? Recorder { get; set; }

        /// <summary>
        /// 操作员点击触发校准（替代原来的跺脚检测）。仅在 CalibrationArmed 且当前
        /// 处于 WaitingForStart 时生效，否则返回 false 且不做任何事。
        /// </summary>
        public bool TriggerCalibration()
        {
            if (!CalibrationArmed) return false;
            if (_calibration.State != CalibrationState.WaitingForStart) return false;
            _calibration.TriggerStart();
            OnCalibrationStateChanged?.Invoke(_calibration.State);
            return true;
        }

        internal void Process(ImuFrameBundle bundle)
        {
            _frameEntryTicks[bundle.PacketId] = Environment.TickCount64; // TEMP：延迟测量起点

            Recorder?.Record(bundle);
            SyncParams();

            // Step 2: DataQualityGate
            var validFrame = _gate.Evaluate(bundle);
            if (validFrame == null) return;

            // Step 3: CalibrationProcessor — static pose collection (post-gate only)
            if (_calibration.State != CalibrationState.Completed &&
                _calibration.State != CalibrationState.Failed)
            {
                bool stateChanged = _calibration.Process(validFrame);
                if (stateChanged)
                {
                    OnCalibrationStateChanged?.Invoke(_calibration.State);
                    if (_calibration.State == CalibrationState.Completed)
                        OnLog?.Invoke("Calibration completed.");
                    else if (_calibration.State == CalibrationState.Failed)
                        OnLog?.Invoke("Calibration failed — please restart.");
                }
                // 校准完成前，后续模块不运行
                if (_calibration.State != CalibrationState.Completed) return;
            }

            var profile = _calibration.Profile!;

            // Step 4: GaitEventDetector
            var gaitEvent = _gait.Detect(validFrame, profile);

            // Step 5: MotionContextDetector
            var motionCtx = _motion.Detect(validFrame, gaitEvent);

            // Step 6: ProgressionDirEstimator
            var pdEstimate = _pd.Update(validFrame, gaitEvent, motionCtx, _motion, profile);

            // Step 7: FpaEngine
            var fpaResult = _fpa.Process(validFrame, gaitEvent, motionCtx, pdEstimate, profile);

            // ── 诊断数据采集（每帧一行，FPA 可为 null）────────────────────────
            var diagRow = new DiagnosticsRow
            {
                Timestamp         = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds(),
                PacketId          = validFrame.PacketId,
                PelvisAcc         = validFrame.Pelvis.HasAcceleration  ? validFrame.Pelvis.Acceleration  : System.Numerics.Vector3.Zero,
                PelvisGyr         = validFrame.Pelvis.HasRateOfTurn     ? validFrame.Pelvis.RateOfTurn     : System.Numerics.Vector3.Zero,
                PelvisQuat        = validFrame.Pelvis.HasQuaternion     ? validFrame.Pelvis.Quaternion     : System.Numerics.Quaternion.Identity,
                LeftAcc           = validFrame.LeftFoot.HasAcceleration ? validFrame.LeftFoot.Acceleration : System.Numerics.Vector3.Zero,
                LeftGyr           = validFrame.LeftFoot.HasRateOfTurn    ? validFrame.LeftFoot.RateOfTurn    : System.Numerics.Vector3.Zero,
                LeftQuat          = validFrame.LeftFoot.HasQuaternion    ? validFrame.LeftFoot.Quaternion    : System.Numerics.Quaternion.Identity,
                RightAcc          = validFrame.RightFoot.HasAcceleration? validFrame.RightFoot.Acceleration: System.Numerics.Vector3.Zero,
                RightGyr          = validFrame.RightFoot.HasRateOfTurn   ? validFrame.RightFoot.RateOfTurn   : System.Numerics.Vector3.Zero,
                RightQuat         = validFrame.RightFoot.HasQuaternion   ? validFrame.RightFoot.Quaternion   : System.Numerics.Quaternion.Identity,
                LeftStance        = gaitEvent.LeftStance,
                RightStance       = gaitEvent.RightStance,
                IsWalking         = gaitEvent.IsWalking,
                MotionState       = motionCtx.State,
                MotionConfidence  = motionCtx.Confidence,
                PdDirectionDeg    = pdEstimate.DirectionRad * (180f / MathF.PI),
                PdStability       = pdEstimate.Stability,
                PdIsValid         = pdEstimate.IsValid,
                FpaLeft_Deg       = fpaResult?.Fpa_L      ?? float.NaN,
                FpaLeft_Error     = fpaResult?.Error_L    ?? float.NaN,
                FpaLeft_OnTarget  = fpaResult?.OnTarget_L ?? false,
                FpaRight_Deg      = fpaResult?.Fpa_R      ?? float.NaN,
                FpaRight_Error    = fpaResult?.Error_R    ?? float.NaN,
                FpaRight_OnTarget = fpaResult?.OnTarget_R ?? false,
                FpaQuality        = fpaResult?.Quality ?? "",
            };
            _diagnosticsBuffer.Enqueue(diagRow);
            if (_diagnosticsBuffer.Count > DiagnosticsCapacity) _diagnosticsBuffer.Dequeue();
            OnDiagnosticsFrame?.Invoke(diagRow);

            if (fpaResult == null) return;

            // Baseline 阶段：收集步级 FPA
            if (InBaseline)
            {
                _baseline.AddStep(fpaResult);
                OnBaselineProgress?.Invoke(_baseline.CollectedSteps_L, _baseline.CollectedSteps_R);
                OnFpaResult?.Invoke(fpaResult, false);  // UI 可选显示

                // 自动检查完成条件：两脚均达阈值时自动结束
                if (_baseline.IsReady)
                {
                    FinalizeBaseline();
                    OnLog?.Invoke("Baseline auto-finalized.");
                }
                // 提前止损：一脚已达阈值、另一脚严重滞后——TryFinalize() 因 IsReady 为 false 会返回
                // null，走既有的失败上报路径，交给操作员决定是否重新开始 baseline。
                else if (_baseline.IsStalled)
                {
                    FinalizeBaseline();
                    OnLog?.Invoke($"Baseline stalled (L={_baseline.CollectedSteps_L} R={_baseline.CollectedSteps_R}) — reporting failure.");
                }
                return;
            }

            // Training 阶段：直接输出供 AR 反馈
            if (InTraining)
            {
                OnFpaResult?.Invoke(fpaResult, true);

                // 当前 training block 的两脚有效步数统计，达标（100/foot）后自动通知 MainWindow 结束该 block
                if (!_trainingBlockDone)
                {
                    if (!float.IsNaN(fpaResult.Fpa_L)) _trainingStepsL++;
                    if (!float.IsNaN(fpaResult.Fpa_R)) _trainingStepsR++;

                    if (_trainingStepsL >= TrainingBlockTargetSteps && _trainingStepsR >= TrainingBlockTargetSteps)
                    {
                        _trainingBlockDone = true;
                        OnLog?.Invoke($"Training block target reached (L={_trainingStepsL} R={_trainingStepsR} valid steps).");
                        OnTrainingBlockComplete?.Invoke();
                    }
                }
            }
        }

        /// <summary>返回诊断数据快照（副本），用于 CSV 导出。不清空缓冲区。</summary>
        public List<DiagnosticsRow> GetDiagnosticsSnapshot()
            => [.._diagnosticsBuffer];

        // TEMP：帧时间戳→AR广播延迟测量，用完删除。取出并清除该 PacketId 对应的耗时（ms）。
        public long? TakeFrameLatencyMs(long packetId) =>
            _frameEntryTicks.TryRemove(packetId, out var t0) ? Environment.TickCount64 - t0 : null;

        // ── 阶段控制（由 MainWindow 的状态机调用）────────────────────────────

        public void StartBaseline()
        {
            _baseline.Reset();
            _fpa.Baseline = null;
            InBaseline = true;
            InTraining = false;
            OnLog?.Invoke("Baseline started.");
        }

        /// <summary>
        /// 结束 Baseline 采集，尝试生成 BaselineProfile。
        /// 返回 null 表示步数不足（调用方负责显示错误）。
        /// </summary>
        public BaselineProfile? FinalizeBaseline()
        {
            InBaseline  = false;
            BaselineProfile = _baseline.TryFinalize();
            if (BaselineProfile != null)
            {
                OnLog?.Invoke($"Baseline done. μL={BaselineProfile.MeanFpa_L:F1}° μR={BaselineProfile.MeanFpa_R:F1}°");
                if (BaselineProfile.StepCountImbalanceWarning)
                    OnLog?.Invoke($"[Warning] Baseline L/R step count imbalance: L={BaselineProfile.ValidSteps_L} R={BaselineProfile.ValidSteps_R}" +
                        $" (ratio={BaselineProfile.StepCountRatio:P0}, threshold={BaselineProfile.ImbalanceRatioThresholdUsed:P0}) — consider redoing baseline.");
            }
            else
                OnLog?.Invoke("Baseline failed — insufficient valid steps.");
            OnBaselineCompleted?.Invoke(BaselineProfile);
            return BaselineProfile;
        }

        public void StartTraining()
        {
            if (BaselineProfile == null)
                throw new InvalidOperationException("Must finalize baseline before starting training.");
            _fpa.Baseline = BaselineProfile;
            _fpa.ToleranceAlpha = TrainingBlockAlphas[0]; // reset to Block 1 default each time training (re)starts
            _fpa.Reset();
            InTraining = true;
            InBaseline = false;
            OnLog?.Invoke("Training started.");
        }

        public void Reset()
        {
            _gate.Reset();
            _calibration.Reset();
            _gait.Reset();
            _motion.Reset();
            _pd.Reset();
            _baseline.Reset();
            _fpa.Reset();
            BaselineProfile = null;
            InBaseline = InTraining = false;
            _diagnosticsBuffer.Clear();
            _frameEntryTicks.Clear(); // TEMP：延迟测量，用完删除。
        }
    }
}
