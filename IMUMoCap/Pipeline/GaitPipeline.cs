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
        private readonly MountingMonitor         _mounting       = new();

        // ── 可调参数（UI 实时更新）──────────────────────────────────────────────
        public PipelineParams Params { get; } = new();

        // ── 诊断数据缓冲区（滑动窗，最长 30 分钟 @100Hz）────────────────────────
        private const int DiagnosticsCapacity = 180_000;
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
        public event Action?                   OnTrainingBlockComplete; // both feet reached Params.TargetStepsPerFoot
        public event Action<string, float, float>? OnStrideZeroEstimated; // (foot, φ°, sd°),每 condition 每只脚一次
        public event Action<string, float, bool>?  OnStrideZeroCrossCheck; // (foot, φ_A − φ_B °, 采用方案 B),每 condition 每只脚一次
        public event Action<string, string>?       OnStrideZeroInvalidated; // (foot, 说明):零点定下前这只脚滤波器重启,需重新校准
        /// <summary>MTw 滤波器重启(StatusWord OrientationValid 1 → 0;开机初始化不算):(role, PacketId)。</summary>
        public event Action<ImuRole, long>?        OnFilterRestart;
        /// <summary>校准姿态采到了,但有 IMU 佩戴方向不对(说明),校准随即判为 Failed。</summary>
        public event Action<string>?               OnMountingRejected;
        /// <summary>行走中某个 IMU 的倾斜偏离校准时 &gt; 10°(Warn)/ &gt; 20°(Pause),每次校准后每级只报一次。</summary>
        public event Action<MountingAlert>?        OnMountingAlert;

        // per role: has the filter been valid since the last restart (for detecting 1 → 0)
        private readonly Dictionary<ImuRole, bool> _filterValid = new();

        public GaitPipeline()
        {
            _fpa.OnStepOutcome += (foot, emitted, reason) => OnStepOutcome?.Invoke(foot, emitted, reason);
            _fpa.OnStrideZeroEstimated += (foot, offset, sd) => OnStrideZeroEstimated?.Invoke(foot, offset, sd);
            _fpa.OnStrideZeroCrossCheck += (foot, diff, usedB) => OnStrideZeroCrossCheck?.Invoke(foot, diff, usedB);
            _fpa.OnStrideZeroInvalidated += (foot, why) => OnStrideZeroInvalidated?.Invoke(foot, why);
        }

        /// <summary>步幅法 FPA 做反馈(默认关:反馈用当前方法,步幅法只记录)。见 docs/stride_fpa_design.md。</summary>
        public bool UseStrideDirection
        {
            get => _fpa.UseStrideDirection;
            set => _fpa.UseStrideDirection = value;
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

        /// <summary>
        /// MainWindow 在"等待参与者站定、触发下一个 stage 前的重新校准"期间设为 true。
        /// InTraining 在这段等待期内本来就是 true（StartTraining 早就调过），如果不在这里拦一下，
        /// Process() 会继续用旧校准产出 FpaResult 并触发 OnFpaResult——AR 端收到 fpa 广播后
        /// OnFpaUpdate() 会调 ShowGraphicGroup()，把刚弹出的"Tap Start"确认按钮盖掉。
        /// 一旦参与者真正点击开始（TriggerRecalibration 把 _calibration.State 拨出 Completed），
        /// 下面既有的"校准完成前不运行后续模块"检查会自动接管，这个 flag 只覆盖"武装等待点击"这一小段。
        /// </summary>
        public bool AwaitingRecalibration { get; set; } = false;

        // 协议 §2.6：每个 training block 在两脚均达到 Params.TargetStepsPerFoot（默认 100）个有效步时自动结束（另有停滞保护超时上限，由 MainWindow 的计时器负责，非并列退出条件）
        private int  _trainingStepsL, _trainingStepsR;
        private bool _trainingBlockDone;

        public int TrainingStepsL => _trainingStepsL;
        public int TrainingStepsR => _trainingStepsR;

        /// <summary>操作员切换到 Training1/2/3 时调用，重置步数计数（容差为所有训练块共用的固定值）。</summary>
        public void SetTrainingBlock(int blockNumber1Based)
        {
            _trainingStepsL = _trainingStepsR = 0;
            _trainingBlockDone = false;
            OnLog?.Invoke($"Training block {blockNumber1Based}: tolerance w={IMUMoCap.Pipeline.Models.BaselineProfile.ToleranceDeg:F0}° (fixed), " +
                $"target {Params.TargetStepsPerFoot} valid steps/foot");
        }

        // ── 主入口 ────────────────────────────────────────────────────────────

        /// <summary>每个 IMU 的单包调用，内部组装 bundle 再进流水线。</summary>
        public void ProcessPacket(ImuRole role, uint deviceId, XsDataPacket packet)
        {
            var (_, bundle) = _frameCollector.Process(role, deviceId, packet);
            if (bundle?.IsComplete == true)
                Process(bundle);
        }

        /// <summary>IMU 实际更新率。落地防抖 / 结算下限按毫秒定义,据此换算成帧数(其余按帧数写的阈值仍假定 100 Hz)。</summary>
        public int SampleRateHz { get; set; } = 100;
        /// <summary>落地防抖:脚需连续静止这么久才算落地(100 Hz = 5 帧)。</summary>
        public const double StanceDebounceMs = 50;
        /// <summary>落地后至少过这么久才结算 FPA(100 Hz = 10 帧)。</summary>
        public const double SettleMinStanceMs = 100;

        private int FramesFor(double ms) => Math.Max(1, (int)Math.Round(ms * SampleRateHz / 1000.0));

        private void SyncParams()
        {
            _gait.MinStanceFrames        = FramesFor(StanceDebounceMs);
            _fpa.MinStanceFramesFloor    = FramesFor(SettleMinStanceMs);
            _fpa.SampleRateHz            = SampleRateHz;

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
            _baseline.MinValidSteps = Params.BaselineStepsPerFoot;
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
            // A failed first calibration is retried by tapping Ready again (flow-review §2-17).
            if (_calibration.State == CalibrationState.Failed) _calibration.Reset();
            if (_calibration.State != CalibrationState.WaitingForStart) return false;
            _calibration.TriggerStart();
            OnCalibrationStateChanged?.Invoke(_calibration.State);
            return true;
        }

        /// <summary>
        /// 在每个 training block / Retention 开始前重新校准零点，对抗 IMU 长时间漂移。
        /// 与 TriggerCalibration() 不同：只重置校准本身 + 依赖校准参考的内部状态（PD 历史、
        /// FPA 结算采样），不touch BaselineProfile/_fpa.Baseline，训练目标保留。
        /// </summary>
        public bool TriggerRecalibration()
        {
            // Completed = normal between-stage recalibration; Failed = retry after a recalibration that timed out.
            if (_calibration.State != CalibrationState.Completed && _calibration.State != CalibrationState.Failed) return false;
            _calibration.Reset();
            _pd.Reset();
            _fpa.Reset();
            _calibration.TriggerStart();
            OnCalibrationStateChanged?.Invoke(_calibration.State);
            return true;
        }

        internal void Process(ImuFrameBundle bundle)
        {
            _frameEntryTicks[bundle.PacketId] = Environment.TickCount64; // TEMP：延迟测量起点

            Recorder?.Record(bundle);
            SyncParams();
            DetectFilterRestarts(bundle);   // before DataQualityGate, which drops the invalid frames

            // Step 2: DataQualityGate
            var validFrame = _gate.Evaluate(bundle);
            if (validFrame == null) return;

            // Step 3: CalibrationProcessor — static pose collection (post-gate only)
            if (_calibration.State != CalibrationState.Completed &&
                _calibration.State != CalibrationState.Failed)
            {
                bool stateChanged = _calibration.Process(validFrame);
                string? mountingProblem = null;
                if (stateChanged && _calibration.Profile is { } collected)
                {
                    mountingProblem = MountingMonitor.CheckCalibration(collected);
                    if (mountingProblem != null) _calibration.Reject();
                    else _mounting.Start(collected);
                }
                if (stateChanged)
                {
                    if (mountingProblem != null)
                    {
                        OnLog?.Invoke($"Calibration rejected — IMU mounted wrong: {mountingProblem}. Fix the IMU and tap Ready to retry.");
                        OnMountingRejected?.Invoke(mountingProblem);
                    }
                    OnCalibrationStateChanged?.Invoke(_calibration.State);
                    if (_calibration.State == CalibrationState.Completed)
                        OnLog?.Invoke("Calibration completed.");
                    else if (_calibration.State == CalibrationState.Failed && mountingProblem == null)
                        OnLog?.Invoke("Calibration failed — stand still and tap Ready to retry.");
                }
                // 校准完成前，后续模块不运行
                if (_calibration.State != CalibrationState.Completed) return;
            }

            // Failed (waiting for the retry) has no profile — nothing downstream can run yet.
            if (_calibration.Profile == null) return;
            var profile = _calibration.Profile;
            if (AwaitingRecalibration) return;

            // Step 4: GaitEventDetector
            var gaitEvent = _gait.Detect(validFrame, profile);
            foreach (var alert in _mounting.Update(validFrame.Pelvis, validFrame.LeftFoot, validFrame.RightFoot))
                OnMountingAlert?.Invoke(alert);

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
                FpaLeft_PdDeg      = fpaResult?.FpaPd_L     ?? float.NaN,
                FpaLeft_StrideDeg  = fpaResult?.FpaStride_L ?? float.NaN,
                FpaRight_PdDeg     = fpaResult?.FpaPd_R     ?? float.NaN,
                FpaRight_StrideDeg = fpaResult?.FpaStride_R ?? float.NaN,
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

                CountTrainingStep(fpaResult);
            }
        }

        private void DetectFilterRestarts(ImuFrameBundle bundle)
        {
            foreach (var (role, sample) in bundle.Samples)
            {
                if (sample.StatusWord is not uint sw) continue;
                bool valid = (sw & (uint)XsStatusFlag.XSF_OrientationValid) != 0;
                if (!valid && _filterValid.GetValueOrDefault(role))
                {
                    _fpa.NotifyFilterRestart(role);
                    OnFilterRestart?.Invoke(role, bundle.PacketId);
                }
                _filterValid[role] = valid;
            }
        }

        /// <summary>
        /// 当前 training block 的两脚有效步数统计，达标（Params.TargetStepsPerFoot）后自动通知 MainWindow 结束该 block。
        /// 从 Process() 抽出，仅为让这段计数逻辑可以单独测试，行为不变。
        /// </summary>
        internal void CountTrainingStep(FpaResult fpaResult)
        {
            if (_trainingBlockDone) return;

            if (!float.IsNaN(fpaResult.Fpa_L)) _trainingStepsL++;
            if (!float.IsNaN(fpaResult.Fpa_R)) _trainingStepsR++;

            if (_trainingStepsL >= Params.TargetStepsPerFoot && _trainingStepsR >= Params.TargetStepsPerFoot)
            {
                _trainingBlockDone = true;
                OnLog?.Invoke($"Training block target reached (L={_trainingStepsL} R={_trainingStepsR} valid steps).");
                OnTrainingBlockComplete?.Invoke();
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
            _fpa.ResetCondition(); // 新 condition:步幅法零点也重估
            _mounting.Reset();
            BaselineProfile = null;
            InBaseline = InTraining = false;
            _diagnosticsBuffer.Clear();
            _frameEntryTicks.Clear(); // TEMP：延迟测量，用完删除。
        }
    }
}
