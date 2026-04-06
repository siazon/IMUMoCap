using System;
using System.Diagnostics;
using GaitTraining.Imu;
using IMUMoCap.Methods;

namespace GaitTraining.Gait
{
    // ─────────────────────────────────────────────────────────────
    //  管线配置（聚合所有模块配置）
    // ─────────────────────────────────────────────────────────────

    public sealed class GaitPipelineConfig
    {
        public AggregatorConfig Aggregator { get; set; } = new();
        public QualityGateConfig Quality { get; set; } = new();
        public StanceDetectorConfig StanceL { get; set; } = new();
        public StanceDetectorConfig StanceR { get; set; } = new();
        public CalibrationConfig Calibration { get; set; } = new();
        public ProgDirConfig ProgDir { get; set; } = new();
        public FpaConfig FpaL { get; set; } = new();
        public FpaConfig FpaR { get; set; } = new();
    }

    // ─────────────────────────────────────────────────────────────
    //  GaitPipeline
    // ─────────────────────────────────────────────────────────────

    /// <summary>
    /// 步态分析管线。串联全部五个模块，对外暴露：
    /// <list type="bullet">
    ///   <item>IMU 数据入口 <see cref="OnImuData"/></item>
    ///   <item>标定控制 <see cref="BeginCalibration"/></item>
    ///   <item>步态输出事件 <see cref="OnStepFpa"/></item>
    ///   <item>各模块诊断访问器</item>
    /// </list>
    /// <para>
    /// 数据流（同一 Xsens 回调线程，无锁）：
    /// IMU 回调
    ///   → ImuFrameAggregator（packetId 聚合）
    ///   → QualityGate（帧级质量过滤）
    ///   → ProgDirTracker（progDir 更新）
    ///   → StanceDetector × 2（左右脚 stance 检测）
    ///   → StandingCalibrator（标定采集）
    ///   → FpaComputer × 2（FPA 计算，stance 结束时触发事件）
    /// </para>
    /// </summary>
    public sealed class GaitPipeline
    {
        // ── 模块实例 ─────────────────────────────────────────────
        public ImuFrameAggregator Aggregator { get; }
        public QualityGate Quality { get; }
        public StanceDetector StanceL { get; }
        public StanceDetector StanceR { get; }
        public StandingCalibrator Calibrator { get; }
        public ProgDirTracker ProgDir { get; }
        public FpaComputer FpaL { get; }
        public FpaComputer FpaR { get; }
        public FootstepImpactDetector FootstepDetector { get; }
        public readonly struct StanceStatus
        {
            public readonly bool LeftInStance;
            public readonly bool RightInStance;

            public StanceStatus(bool left, bool right)
            {
                LeftInStance = left;
                RightInStance = right;
            }

            public override string ToString() =>
                $"L:{LeftInStance} R:{RightInStance}";
        }

        // ── 对外事件 ─────────────────────────────────────────────

        /// <summary>每步输出一次 FPA（左脚或右脚）。在 Xsens 回调线程触发。</summary>
        public event Action<StepFpaResult>? OnStepFpa;

        /// <summary>标定开始。</summary>
        public event Action? OnCalibrationStarted;

        /// <summary>标定成功。</summary>
        public event Action<CalibrationResult>? OnCalibrationDone;

        /// <summary>标定失败，携带原因字符串供 UI 显示后重试提示。</summary>
        public event Action<CalibrationFailReason, string>? OnCalibrationFailed;

        /// <summary>标定 sanity 警告（不阻止完成）。</summary>
        public event Action<CalibrationResult, string>? OnSanityWarning;

        /// <summary>标定状态变更（供 UI 进度显示）。</summary>
        public event Action<CalibrationState>? OnCalibrationStateChanged;

        public event Action<StanceStatus>? OnStanceStatusChanged;

        // ── 内部状态 ─────────────────────────────────────────────
        private CalibrationResult? _calibration;
        private bool? _lastLeftInStance;
        private bool? _lastRightInStance;
        
        // ── 跺脚检测与延迟标定 ─────────────────────────────────
        private DateTimeOffset? _footstepDetectedTime;
        private readonly int _delayAfterFootstepMs = 500;

        // ── 构造 ─────────────────────────────────────────────────
        public GaitPipeline(GaitPipelineConfig? config = null)
        {
            config ??= new GaitPipelineConfig();

            Aggregator = new ImuFrameAggregator(config.Aggregator);
            Quality = new QualityGate(config.Quality);
            StanceL = new StanceDetector(config.StanceL);
            StanceR = new StanceDetector(config.StanceR);
            Calibrator = new StandingCalibrator(config.Calibration);
            ProgDir = new ProgDirTracker(config.ProgDir);
            FpaL = new FpaComputer(ImuRole.Left, ProgDir, config.FpaL);
            FpaR = new FpaComputer(ImuRole.Right, ProgDir, config.FpaR);
            FootstepDetector = new FootstepImpactDetector(
                baselineCollectionDurationMs: 500f,
                waitBeforeDetectDurationMs: 500f,
                accelPeakThresholdMs2: 15f,
                accelRatioThreshold: 2.0f,
                imuSamplingRateHz: 100);

            // 聚合完整帧 → 主处理链
            Aggregator.OnFrameReady += ProcessFrame;

            // 标定事件转发
            Calibrator.OnCalibrationDone += result =>
            {
                _calibration = result;
                OnCalibrationDone?.Invoke(result);
            };
            Calibrator.OnCalibrationFailed += (reason, msg) =>
                OnCalibrationFailed?.Invoke(reason, msg);
            Calibrator.OnSanityWarning += (result, msg) =>
                OnSanityWarning?.Invoke(result, msg);
            Calibrator.OnStateChanged += state =>
                OnCalibrationStateChanged?.Invoke(state);

            // FPA 事件转发
            FpaL.OnStepFpa += result => OnStepFpa?.Invoke(result);
            FpaR.OnStepFpa += result => OnStepFpa?.Invoke(result);
        }

        // ─────────────────────────────────────────────────────────
        //  IMU 数据入口（从 Xsens 回调调用）
        // ─────────────────────────────────────────────────────────

        /// <summary>
        /// 从 Xsens 的 OnDataAvailable 回调中调用。
        /// </summary>
        public void OnImuData(ImuRole sensor, long packetId, ImuRawData raw)
            => Aggregator.OnImuData(sensor, packetId, raw);

        // ─────────────────────────────────────────────────────────
        //  标定控制
        // ─────────────────────────────────────────────────────────

        /// <summary>
        /// 开始站立标定。建议在 UI 提示受试者"请站直朝向行进方向"后调用。
        /// </summary>
        public void BeginCalibration()
        {
            OnCalibrationStarted?.Invoke();
            Calibrator.BeginStandingCalibration();
        }

        // ─────────────────────────────────────────────────────────
        //  主处理链
        // ─────────────────────────────────────────────────────────
        private int _debugFrameCount = 0;
        private void ProcessFrame(SyncedFrame frame)
        {
            if (!frame.IsComplete)
            {
                Debug.WriteLine($"[Pipeline] Incomplete frame pkt={frame.PacketId}, skipped.");
                return;
            }
            _debugFrameCount++;

            // 1. 质量门控
            var quality = Quality.Evaluate(frame);

            // 2. progDir 更新（只用质量通过的帧，不限 stance）
            ProgDir.Update(quality.PelvisHeading, quality.IsValid);

            // 3. 跺脚检测（左脚原地跺脚）
            bool leftFootstepDetected = FootstepDetector.Update(frame.Left);
            if (leftFootstepDetected)
            {
                _footstepDetectedTime = DateTimeOffset.UtcNow;
                Debug.WriteLine($"[Pipeline] Left footstep detected at frame {_debugFrameCount}");
            }

            // 3.5 检查是否该触发标定（跺脚后延迟500ms）
            if (_footstepDetectedTime.HasValue)
            {
                double msSinceFootstep = (DateTimeOffset.UtcNow - _footstepDetectedTime.Value).TotalMilliseconds;
                if (msSinceFootstep >= _delayAfterFootstepMs)
                {
                    Debug.WriteLine($"[Pipeline] Triggering CalibrationStart {msSinceFootstep:F0}ms after footstep");
                    BeginCalibration();
                    _footstepDetectedTime = null;  // 清空标志，只触发一次
                }
            }

            // 4. stance 检测（质量不通过的帧仍然喂入，让状态机自然运行；
            //    FpaComputer 内部只收质量通过的帧）
            var stateL = StanceL.Update(frame.Left);
            var stateR = StanceR.Update(frame.Right);

            // 5. 标定采集
            Calibrator.Update(frame, stateL, stateR, quality);

            // 6. FPA 计算（标定完成前 calibration 为 null，FpaComputer 内部跳过）
            FpaL.Update(frame, stateL, quality, _calibration);
            FpaR.Update(frame, stateR, quality, _calibration);

            // ── 每100帧打印一次诊断快照 ──────────────────────────────
            if (_debugFrameCount % 100 == 0)
            {
                var qd = Quality.GetDiagnostics();
                var pd = ProgDir.GetDiagnostics();
                var fl = FpaL.GetDiagnostics();
                var fr = FpaR.GetDiagnostics();
                var sl = StanceL.GetDiagnostics();
                var sr = StanceR.GetDiagnostics();
                var fd = FootstepDetector.GetDiagnostics();

                Debug.WriteLine($"[Pipeline @{_debugFrameCount}]");
                Debug.WriteLine($"  Quality:  Total={qd.TotalFrames} Invalid={qd.InvalidFrames} " +
                                $"Jump={qd.JumpFailFrames} Yaw={qd.YawConsistFailFrames} " +
                                $"Mag={qd.MagFailFrames} Acc={qd.AccFailFrames}");
                Debug.WriteLine($"  ProgDir:  {pd}");
                Debug.WriteLine($"  StanceL:  {sl}");
                Debug.WriteLine($"  StanceR:  {sr}");
                Debug.WriteLine($"  FpaL:     {fl}");
                Debug.WriteLine($"  FpaR:     {fr}");
                Debug.WriteLine($"  Footstep: {fd}");
            }

            // 仅在 stance 状态发生变化时广播，避免每帧重复推送相同状态。
            bool leftInStance = stateL == StanceState.Stance;
            bool rightInStance = stateR == StanceState.Stance;
            if (_lastLeftInStance != leftInStance || _lastRightInStance != rightInStance)
            {
                _lastLeftInStance = leftInStance;
                _lastRightInStance = rightInStance;
                OnStanceStatusChanged?.Invoke(new StanceStatus(leftInStance, rightInStance));
            }
        }

        // ─────────────────────────────────────────────────────────
        //  全局 Reset
        // ─────────────────────────────────────────────────────────

        /// <summary>
        /// 重置所有模块。会话切换、重新开始时调用。
        /// 注意：_calibration 也会清除，需要重新标定。
        /// </summary>
        public void Reset()
        {
            Aggregator.Reset();
            Quality.Reset();
            StanceL.Reset();
            StanceR.Reset();
            Calibrator.Reset();
            ProgDir.Reset();
            FpaL.Reset();
            FpaR.Reset();
            FootstepDetector.Reset();
            _calibration = null;
            _lastLeftInStance = null;
            _lastRightInStance = null;
            _footstepDetectedTime = null;
            Debug.WriteLine("[Pipeline] Full reset.");
        }

        /// <summary>
        /// 仅重置步态检测状态，保留标定结果。
        /// 适用于同一受试者换场地/换方向时。
        /// </summary>
        public void ResetGaitOnly()
        {
            Aggregator.Reset();
            Quality.Reset();
            StanceL.Reset();
            StanceR.Reset();
            ProgDir.Reset();
            FpaL.Reset();
            FpaR.Reset();
            FootstepDetector.Reset();
            _lastLeftInStance = null;
            _lastRightInStance = null;
            _footstepDetectedTime = null;
            Debug.WriteLine("[Pipeline] Gait-only reset (calibration preserved).");
        }
    }
}
