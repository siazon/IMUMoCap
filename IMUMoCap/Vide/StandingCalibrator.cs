using System;
using System.Collections.Generic;
using System.Diagnostics;
using GaitTraining.Imu;

namespace GaitTraining.Gait
{
    // ─────────────────────────────────────────────────────────────
    //  状态枚举
    // ─────────────────────────────────────────────────────────────

    public enum CalibrationState
    {
        Idle,        // 未开始
        Waiting,     // 延迟等待阶段（受试者站稳）
        Collecting,  // 数据收集阶段
        Computing,   // 计算中（同步，极短）
        Done,        // 成功完成
        Failed,      // 失败（有效帧不足）
    }

    // ─────────────────────────────────────────────────────────────
    //  失败原因
    // ─────────────────────────────────────────────────────────────

    public enum CalibrationFailReason
    {
        None,
        InsufficientValidFrames,   // 有效帧数不足 MinValidFrames
        SanityCheckFailed,         // 保留扩展，当前作警告不失败
    }

    // ─────────────────────────────────────────────────────────────
    //  配置
    // ─────────────────────────────────────────────────────────────

    public sealed class CalibrationConfig
    {
        /// <summary>延迟开始时间（s）。调用 Begin 后等待此时长再开始采集。</summary>
        public float DelaySeconds { get; set; } = 1f;

        /// <summary>数据收集时长（s）。</summary>
        public float CollectSeconds { get; set; } = 6f;

        /// <summary>IMU 采样率（Hz）。</summary>
        public int SampleRateHz { get; set; } = 100;

        /// <summary>
        /// 收集阶段最小有效帧数。
        /// 不足时标定失败，通知 UI 重试。
        /// 默认 400 = 6s × 100Hz × 67%，留有 33% 的静止门控余量。
        /// </summary>
        public int MinValidFrames { get; set; } = 400;

        /// <summary>
        /// Sanity check 警告阈值（°）。
        /// 校正后残差超过此值时触发 OnSanityWarning，不阻止标定完成。
        /// </summary>
        public float SanityWarnDeg { get; set; } = 5f;

        /// <summary>
        /// requireStill：true = 只收双脚均为 Stance 且质量门控通过的帧。
        /// false = 只要质量门控通过即收集（调试用）。
        /// </summary>
        public bool RequireStill { get; set; } = true;

        // 派生
        internal int WaitFrames => (int)(DelaySeconds * SampleRateHz);
        internal int CollectFrames => (int)(CollectSeconds * SampleRateHz);
    }

    // ─────────────────────────────────────────────────────────────
    //  标定结果
    // ─────────────────────────────────────────────────────────────

    public sealed class CalibrationResult
    {
        /// <summary>左脚安装偏差（rad）。FPA 计算：corrected = footHeading - DeltaL。</summary>
        public float DeltaL { get; init; }

        /// <summary>右脚安装偏差（rad）。</summary>
        public float DeltaR { get; init; }

        /// <summary>DeltaL 转度数（显示用）。</summary>
        public float DeltaLDeg => DeltaL * (180f / MathF.PI);

        /// <summary>DeltaR 转度数（显示用）。</summary>
        public float DeltaRDeg => DeltaR * (180f / MathF.PI);

        /// <summary>左脚 sanity 残差（°）。理想值接近 0。</summary>
        public float SanityErrLDeg { get; init; }

        /// <summary>右脚 sanity 残差（°）。</summary>
        public float SanityErrRDeg { get; init; }

        /// <summary>实际收集到的有效帧数。</summary>
        public int ValidFramesCollected { get; init; }

        /// <summary>标定完成时间。</summary>
        public DateTimeOffset Timestamp { get; init; }

        /// <summary>Sanity check 是否通过（残差均在阈值内）。</summary>
        public bool SanityPassed { get; init; }

        public override string ToString() =>
            $"DeltaL={DeltaLDeg:F1}° DeltaR={DeltaRDeg:F1}° " +
            $"ErrL={SanityErrLDeg:F2}° ErrR={SanityErrRDeg:F2}° " +
            $"Frames={ValidFramesCollected} Sanity={SanityPassed}";
    }

    // ─────────────────────────────────────────────────────────────
    //  诊断快照
    // ─────────────────────────────────────────────────────────────

    public sealed class CalibrationDiagnostics
    {
        public CalibrationState State { get; internal set; }
        public int ElapsedFrames { get; internal set; }
        public int ValidFramesCollected { get; internal set; }
        public int SkippedFrames { get; internal set; }
        /// <summary>收集阶段进度 0–1。</summary>
        public float CollectProgress { get; internal set; }

        public override string ToString() =>
            $"State={State} Progress={CollectProgress:P0} " +
            $"Valid={ValidFramesCollected} Skipped={SkippedFrames}";
    }

    // ─────────────────────────────────────────────────────────────
    //  StandingCalibrator
    // ─────────────────────────────────────────────────────────────

    /// <summary>
    /// 站立标定模块。
    /// <para>
    /// 使用方式：
    /// <code>
    /// calibrator.BeginStandingCalibration();
    /// // 每帧调用：
    /// calibrator.Update(frame, stateL, stateR, quality);
    /// // 订阅事件获取结果：
    /// calibrator.OnCalibrationDone   += result => ...;
    /// calibrator.OnCalibrationFailed += reason => ...;
    /// calibrator.OnSanityWarning     += (result, msg) => ...;
    /// calibrator.OnStateChanged      += state => ...;
    /// </code>
    /// </para>
    /// <para>线程假设：单线程调用（Xsens 回调线程）。</para>
    /// </summary>
    public sealed class StandingCalibrator
    {
        // ── 配置 ─────────────────────────────────────────────────
        public CalibrationConfig Config { get; set; }

        // ── 事件 ─────────────────────────────────────────────────

        /// <summary>标定成功完成。携带结果，即使 sanity 警告也会触发此事件。</summary>
        public event Action<CalibrationResult>? OnCalibrationDone;

        /// <summary>
        /// 标定失败（有效帧不足）。
        /// UI 应显示提示并在用户调整后调用 BeginStandingCalibration 重试。
        /// </summary>
        public event Action<CalibrationFailReason, string>? OnCalibrationFailed;

        /// <summary>Sanity 残差超出阈值时触发（不阻止完成）。</summary>
        public event Action<CalibrationResult, string>? OnSanityWarning;

        /// <summary>状态变更通知，供 UI 驱动进度显示。</summary>
        public event Action<CalibrationState>? OnStateChanged;

        // ── 最近一次成功结果（失败时保留上次） ──────────────────
        public CalibrationResult? LastResult { get; private set; }

        // ── 诊断 ─────────────────────────────────────────────────
        private readonly CalibrationDiagnostics _diag = new();
        public CalibrationDiagnostics GetDiagnostics() => _diag;

        // ── 内部状态 ─────────────────────────────────────────────
        private CalibrationState _state = CalibrationState.Idle;
        private int _elapsedFrames;

        // 收集缓冲：每帧存一个差值对 (leftDiff, rightDiff)，单位 rad
        // 用 List 而非固定数组，避免提前分配过多内存
        private readonly List<float> _leftDiffs = new(700);
        private readonly List<float> _rightDiffs = new(700);

        // ── 构造 ─────────────────────────────────────────────────
        public StandingCalibrator(CalibrationConfig? config = null)
        {
            Config = config ?? new CalibrationConfig();
        }

        // ─────────────────────────────────────────────────────────
        //  开始标定
        // ─────────────────────────────────────────────────────────

        /// <summary>
        /// 开始一次站立标定流程。可在任意状态下调用（自动重置）。
        /// 建议在 UI 层提示受试者"请站直并保持静止"后调用。
        /// </summary>
        public void BeginStandingCalibration()
        {
            ResetInternal();
            TransitionTo(CalibrationState.Waiting);
            Debug.WriteLine($"[Calibrator] Begin. Delay={Config.DelaySeconds}s " +
                            $"Collect={Config.CollectSeconds}s " +
                            $"MinValid={Config.MinValidFrames}");
        }

        // ─────────────────────────────────────────────────────────
        //  主入口：每帧调用
        // ─────────────────────────────────────────────────────────

        /// <summary>
        /// 每帧调用一次。在 Idle / Done / Failed 状态下直接返回，不做任何处理。
        /// </summary>
        /// <param name="frame">聚合后的同步帧（必须 IsComplete）。</param>
        /// <param name="stateL">左脚当前 stance 状态。</param>
        /// <param name="stateR">右脚当前 stance 状态。</param>
        /// <param name="quality">当前帧质量门控结果。</param>
        public void Update(
            in SyncedFrame frame,
            StanceState stateL,
            StanceState stateR,
            in QualityResult quality)
        {
            if (_state == CalibrationState.Idle ||
                _state == CalibrationState.Done ||
                _state == CalibrationState.Failed)
                return;

            _elapsedFrames++;

            switch (_state)
            {
                case CalibrationState.Waiting:
                    HandleWaiting();
                    break;

                case CalibrationState.Collecting:
                    HandleCollecting(frame, stateL, stateR, quality);
                    break;
            }

            // 更新诊断（Computing 状态极短，不单独更新）
            _diag.ElapsedFrames = _elapsedFrames;
            _diag.ValidFramesCollected = _leftDiffs.Count;
            _diag.CollectProgress = _state == CalibrationState.Collecting
                ? Math.Clamp((float)_leftDiffs.Count / Config.MinValidFrames, 0f, 1f)
                : (_state == CalibrationState.Done ? 1f : 0f);
        }

        // ─────────────────────────────────────────────────────────
        //  Waiting 阶段
        // ─────────────────────────────────────────────────────────

        private void HandleWaiting()
        {
            if (_elapsedFrames >= Config.WaitFrames)
            {
                _elapsedFrames = 0;   // 重置计数，用于 Collecting 阶段
                TransitionTo(CalibrationState.Collecting);
                Debug.WriteLine("[Calibrator] Collecting started.");
            }
        }

        // ─────────────────────────────────────────────────────────
        //  Collecting 阶段
        // ─────────────────────────────────────────────────────────

        private void HandleCollecting(
            in SyncedFrame frame,
            StanceState stateL,
            StanceState stateR,
            in QualityResult quality)
        {
            // 判断是否接受本帧
            bool qualityOk = quality.IsValid;
            bool stillOk = !Config.RequireStill ||
                             (stateL == StanceState.Stance &&
                              stateR == StanceState.Stance);

            if (qualityOk && stillOk)
            {
                // 计算帧级差值（圆域）
                float leftDiff = HeadingUtil.AngleDiff(quality.LeftHeading,
                                                        quality.PelvisHeading);
                float rightDiff = HeadingUtil.AngleDiff(quality.RightHeading,
                                                        quality.PelvisHeading);
                _leftDiffs.Add(leftDiff);
                _rightDiffs.Add(rightDiff);
            }
            else
            {
                _diag.SkippedFrames++;
            }

            // 收集时间到（按总 elapsed 帧数判断，不依赖有效帧数）
            if (_elapsedFrames >= Config.CollectFrames)
                Finalize();
        }

        // ─────────────────────────────────────────────────────────
        //  计算与 Sanity Check
        // ─────────────────────────────────────────────────────────

        private void Finalize()
        {
            TransitionTo(CalibrationState.Computing);

            int validCount = _leftDiffs.Count;
            Debug.WriteLine($"[Calibrator] Finalizing. ValidFrames={validCount} " +
                            $"MinRequired={Config.MinValidFrames}");

            // 有效帧不足 → 失败
            if (validCount < Config.MinValidFrames)
            {
                string reason = $"Insufficient valid frames (collected {validCount} frames)" +
                                $"At least {Config.MinValidFrames} frames are required.）。" +
                                $"Please keep your feet still and try again.";
                Debug.WriteLine($"[Calibrator] FAILED: {reason}");
                TransitionTo(CalibrationState.Failed);
                OnCalibrationFailed?.Invoke(
                    CalibrationFailReason.InsufficientValidFrames, reason);
                return;
            }

            // 计算 delta（圆均值）
            float deltaL = HeadingUtil.CircularMean(_leftDiffs.ToArray());
            float deltaR = HeadingUtil.CircularMean(_rightDiffs.ToArray());

            // Sanity check：用 delta 校正后的残差应接近 0
            float sanityErrL = ComputeSanityError(_leftDiffs, deltaL);
            float sanityErrR = ComputeSanityError(_rightDiffs, deltaR);

            float sanityErrLDeg = sanityErrL * (180f / MathF.PI);
            float sanityErrRDeg = sanityErrR * (180f / MathF.PI);

            bool sanityPassed = sanityErrLDeg <= Config.SanityWarnDeg &&
                                sanityErrRDeg <= Config.SanityWarnDeg;

            var result = new CalibrationResult
            {
                DeltaL = deltaL,
                DeltaR = deltaR,
                SanityErrLDeg = sanityErrLDeg,
                SanityErrRDeg = sanityErrRDeg,
                ValidFramesCollected = validCount,
                Timestamp = DateTimeOffset.UtcNow,
                SanityPassed = sanityPassed,
            };

            Debug.WriteLine($"[Calibrator] Done: {result}");

            LastResult = result;
            TransitionTo(CalibrationState.Done);

            // Sanity 警告（不阻止完成）
            if (!sanityPassed)
            {
                string warn = $"Calibration residuals are excessively large (left foot{sanityErrLDeg:F1}°，" +
                              $"right foot {sanityErrRDeg:F1}°，Threshold {Config.SanityWarnDeg}°）。" +
                              $"Recommended to recalibrate after checking the sensor installation orientation.";
                Debug.WriteLine($"[Calibrator] SANITY WARNING: {warn}");
                OnSanityWarning?.Invoke(result, warn);
            }

            OnCalibrationDone?.Invoke(result);
        }

        // ─────────────────────────────────────────────────────────
        //  Sanity Error 计算
        // ─────────────────────────────────────────────────────────

        /// <summary>
        /// 用 delta 校正每帧差值后，计算校正残差的圆均值绝对值。
        /// 理想情况下应为 0（即 delta 完美对齐）。
        /// corrected[i] = diff[i] - delta
        /// sanityErr = |CircularMean(corrected)|
        /// </summary>
        private static float ComputeSanityError(List<float> diffs, float delta)
        {
            var corrected = new float[diffs.Count];
            for (int i = 0; i < diffs.Count; i++)
                corrected[i] = HeadingUtil.AngleDiff(diffs[i], delta);
            return MathF.Abs(HeadingUtil.CircularMean(corrected));
        }

        // ─────────────────────────────────────────────────────────
        //  Reset & 辅助
        // ─────────────────────────────────────────────────────────

        /// <summary>
        /// 完全重置，回到 Idle。不清除 LastResult（保留上次成功结果）。
        /// </summary>
        public void Reset()
        {
            ResetInternal();
            TransitionTo(CalibrationState.Idle);
            Debug.WriteLine("[Calibrator] Reset to Idle.");
        }

        private void ResetInternal()
        {
            _elapsedFrames = 0;
            _leftDiffs.Clear();
            _rightDiffs.Clear();
            _diag.ElapsedFrames = 0;
            _diag.ValidFramesCollected = 0;
            _diag.SkippedFrames = 0;
            _diag.CollectProgress = 0f;
        }

        private void TransitionTo(CalibrationState next)
        {
            if (_state == next) return;
            Debug.WriteLine($"[Calibrator] {_state} → {next}");
            _state = next;
            _diag.State = next;
            OnStateChanged?.Invoke(next);
        }
    }
}