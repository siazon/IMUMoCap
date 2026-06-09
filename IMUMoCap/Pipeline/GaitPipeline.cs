// IMUMoCap/Pipeline/GaitPipeline.cs
using System;
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

        // ── 诊断数据缓冲区 ────────────────────────────────────────────────────
        private readonly List<DiagnosticsRow> _diagnosticsBuffer = new();

        // ── 事件 ──────────────────────────────────────────────────────────────
        public event Action<FpaResult,bool >?        OnFpaResult;
        public event Action<CalibrationState>? OnCalibrationStateChanged;
        public event Action<int, int>?         OnBaselineProgress;  // (stepsL, stepsR)
        public event Action<BaselineProfile?>? OnBaselineCompleted; // auto- or manual-finalize
        public event Action<string>?           OnLog;

        // ── 状态 ──────────────────────────────────────────────────────────────
        public CalibrationProfile? CalibrationProfile => _calibration.Profile;
        public BaselineProfile?    BaselineProfile    { get; private set; }
        public bool InBaseline  { get; private set; }
        public bool InTraining  { get; private set; }

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
            _calibration.StompAccThreshold_ms2 = Params.StompThreshold_ms2;
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
        }

        public void Process(ImuFrameBundle bundle)
        {
            SyncParams();

            // Step 1 (pre-gate): Stomp detection for calibration trigger.
            // Stomp frames have StatusWord with all axes clipping + OrientationValid=0,
            // which DataQualityGate rejects. Stomp detection must run on the raw bundle.
            if (_calibration.State == CalibrationState.WaitingForStart)
            {
                bool stompDetected = _calibration.ProcessPreGate(bundle);
                if (stompDetected)
                {
                    OnCalibrationStateChanged?.Invoke(_calibration.State);
                    OnLog?.Invoke("Stomp detected — collecting static pose...");
                }
            }

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
            _diagnosticsBuffer.Add(new DiagnosticsRow
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
            });

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
                return;
            }

            // Training 阶段：直接输出供 AR 反馈
            if (InTraining)
                OnFpaResult?.Invoke(fpaResult, true);
        }

        /// <summary>返回诊断数据快照（副本），用于 CSV 导出。不清空缓冲区。</summary>
        public List<DiagnosticsRow> GetDiagnosticsSnapshot()
            => new List<DiagnosticsRow>(_diagnosticsBuffer);

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
                OnLog?.Invoke($"Baseline done. μL={BaselineProfile.MeanFpa_L:F1}° μR={BaselineProfile.MeanFpa_R:F1}°");
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
            _fpa.Reset();
            BaselineProfile = null;
            InBaseline = InTraining = false;
            _diagnosticsBuffer.Clear();
        }
    }
}
