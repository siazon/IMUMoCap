// IMUMoCap/Pipeline/GaitPipeline.cs
using System;
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

        // ── 事件 ──────────────────────────────────────────────────────────────
        public event Action<FpaResult>?        OnFpaResult;
        public event Action<CalibrationState>? OnCalibrationStateChanged;
        public event Action<int, int>?         OnBaselineProgress;  // (stepsL, stepsR)
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

        private void Process(ImuFrameBundle bundle)
        {
            // Step 1: DataQualityGate
            var validFrame = _gate.Evaluate(bundle);
            if (validFrame == null) return;

            // Step 2: CalibrationProcessor（仅在未完成校准时运行）
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

            // Step 3: GaitEventDetector
            var gaitEvent = _gait.Detect(validFrame, profile);

            // Step 4: MotionContextDetector
            var motionCtx = _motion.Detect(validFrame, gaitEvent);

            // Step 5: ProgressionDirEstimator
            var pdEstimate = _pd.Update(validFrame, gaitEvent, motionCtx, _motion);

            // Step 6: FpaEngine
            var fpaResult = _fpa.Process(validFrame, gaitEvent, motionCtx, pdEstimate);

            if (fpaResult == null) return;

            // Baseline 阶段：收集步级 FPA
            if (InBaseline)
            {
                _baseline.AddStep(fpaResult);
                OnBaselineProgress?.Invoke(_baseline.CollectedSteps_L, _baseline.CollectedSteps_R);
                OnFpaResult?.Invoke(fpaResult);  // UI 可选显示
                return;
            }

            // Training 阶段：直接输出供 AR 反馈
            if (InTraining)
                OnFpaResult?.Invoke(fpaResult);
        }

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
        }
    }
}
