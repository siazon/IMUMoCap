using System;
using System.Collections.Generic;
using System.Diagnostics;
using GaitTraining.Imu;
using IMUMoCap.Methods;

namespace GaitTraining.Gait
{
    // ─────────────────────────────────────────────────────────────
    //  配置
    // ─────────────────────────────────────────────────────────────

    public sealed class FpaConfig
    {
        /// <summary>
        /// 最低 gyro 子窗口帧数。在 stance 内选连续 N 帧使 mean(gyroMag) 最小。
        /// 默认 15 帧 = 150ms。
        /// </summary>
        public int BestWindowFrames { get; set; } = 15;

        /// <summary>
        /// 步输出门控：stance 最小持续帧数。
        /// 低于此值丢弃本步（防止短暂静止误触发）。默认 20 帧 = 200ms，适应正常步速。
        /// </summary>
        public int MinStanceFrames { get; set; } = 20;

        /// <summary>
        /// 是否把 Entering / Exiting 也视为可收集的支撑段。
        /// 开启后可覆盖更完整的脚掌着地过程，适合正常步速下 stance 稳定段较短的场景。
        /// </summary>
        public bool IncludeTransitionFrames { get; set; } = true;

        /// <summary>IMU 采样率（Hz）。</summary>
        public int SampleRateHz { get; set; } = 100;
    }

    // ─────────────────────────────────────────────────────────────
    //  步输出结果
    // ─────────────────────────────────────────────────────────────

    public sealed class StepFpaResult
    {
        /// <summary>哪只脚。</summary>
        public ImuRole Foot { get; init; }

        /// <summary>FPA（°）。正值 = 外八，负值 = 内八。</summary>
        public float FpaDeg { get; init; }

        /// <summary>本步使用的 progDir（°，显示用）。</summary>
        public float ProgDirDeg { get; init; }

        /// <summary>最优子窗口内校正后脚部 heading 圆均值（°）。</summary>
        public float CorrectedHeadingDeg { get; init; }

        /// <summary>本次 stance 总有效帧数。</summary>
        public int StanceFrameCount { get; init; }

        /// <summary>最优子窗口在 stance 帧序列中的起始偏移（调试用）。</summary>
        public int BestWindowStartIndex { get; init; }

        /// <summary>最优子窗口实际帧数（< BestWindowFrames 时表示 stance 过短）。</summary>
        public int BestWindowActualFrames { get; init; }

        /// <summary>stance 开始的 packetId。</summary>
        public long PacketIdStart { get; init; }

        /// <summary>stance 结束的 packetId。</summary>
        public long PacketIdEnd { get; init; }
        public bool inStance { get; set; }

        public DateTimeOffset Timestamp { get; init; }

        public override string ToString() =>
            $"[{Foot}] FPA={FpaDeg:+0.0;-0.0}° inStance={inStance}" +
            $"CorrH={CorrectedHeadingDeg:F1}° ProgDir={ProgDirDeg:F1}° " +
            $"StanceFrames={StanceFrameCount} BestWin=[{BestWindowStartIndex}+{BestWindowActualFrames}] " +
            $"pkt={PacketIdStart}~{PacketIdEnd}";
    }

    // ─────────────────────────────────────────────────────────────
    //  内部 stance 帧记录
    // ─────────────────────────────────────────────────────────────

    internal readonly struct StanceFrameRecord
    {
        public readonly float CorrectedHeading;   // rad，已减 delta
        public readonly float GyrMag;             // rad/s
        public readonly long PacketId;

        public StanceFrameRecord(float corrH, float gyrMag, long packetId)
        {
            CorrectedHeading = corrH;
            GyrMag = gyrMag;
            PacketId = packetId;
        }
    }

    // ─────────────────────────────────────────────────────────────
    //  诊断
    // ─────────────────────────────────────────────────────────────

    public sealed class FpaDiagnostics
    {
        public ImuRole Foot { get; internal set; }
        public bool InStance { get; internal set; }
        public int CurrentStanceFrames { get; internal set; }
        public int TotalStepsEmitted { get; internal set; }
        public int TotalStepsDropped { get; internal set; }
        public float LastFpaDeg { get; internal set; }

        public override string ToString() =>
            $"[{Foot}] InStance={InStance} StanceFrames={CurrentStanceFrames} " +
            $"Steps={TotalStepsEmitted} Dropped={TotalStepsDropped} " +
            $"LastFPA={LastFpaDeg:+0.0;-0.0}°";
    }

    // ─────────────────────────────────────────────────────────────
    //  FpaComputer
    // ─────────────────────────────────────────────────────────────

    /// <summary>
    /// 单脚 FPA 计算器。
    /// <para>
    /// 使用方式：每帧调用 <see cref="Update"/>，stance 结束时通过
    /// <see cref="OnStepFpa"/> 事件输出 <see cref="StepFpaResult"/>。
    /// </para>
    /// <para>
    /// 依赖：<see cref="ProgDirTracker"/>（外部共享实例），
    ///       <see cref="CalibrationResult"/>（外部传入）。
    /// </para>
    /// <para>线程假设：单线程调用。</para>
    /// </summary>
    public sealed class FpaComputer
    {
        // ── 配置 ─────────────────────────────────────────────────
        public FpaConfig Config { get; set; }

        // ── 依赖 ─────────────────────────────────────────────────
        private readonly ImuRole _foot;
        private readonly ProgDirTracker _progDir;

        // ── 事件 ─────────────────────────────────────────────────
        /// <summary>每步 stance 结束且通过质量门控后触发一次。</summary>
        public event Action<StepFpaResult>? OnStepFpa;

        // ── 诊断 ─────────────────────────────────────────────────
        private readonly FpaDiagnostics _diag;
        public FpaDiagnostics GetDiagnostics() => _diag;

        // ── 内部状态 ─────────────────────────────────────────────
        private bool _wasInStance;
        private long _stanceStartPacketId;
        private long _lastPacketId;

        // stance 期间收集的帧（只收质量门控通过的帧）
        private readonly List<StanceFrameRecord> _stanceFrames = new(300);

        // ── 构造 ─────────────────────────────────────────────────
        public FpaComputer(ImuRole foot, ProgDirTracker progDir, FpaConfig? config = null)
        {
            _foot = foot;
            _progDir = progDir;
            Config = config ?? new FpaConfig();
            _diag = new FpaDiagnostics { Foot = foot };
        }

        // ─────────────────────────────────────────────────────────
        //  主入口：每帧调用
        // ─────────────────────────────────────────────────────────

        /// <summary>
        /// 每帧调用一次。
        /// </summary>
        /// <param name="frame">当前同步帧（用于取 packetId 和对应脚的 ImuSample）。</param>
        /// <param name="stanceState">本脚当前 stance 状态（来自 StanceDetector）。</param>
        /// <param name="quality">当前帧质量结果。</param>
        /// <param name="calibration">标定结果（可为 null，null 时跳过处理）。</param>
        public void Update(
            in SyncedFrame frame,
            StanceState stanceState,
            in QualityResult quality,
            CalibrationResult? calibration)
        {
            if (calibration is null) return;

            bool inStance = stanceState == StanceState.Stance;
            bool inSupport = IsSupportPhase(stanceState);
            _lastPacketId = frame.PacketId;

            // ── 支撑段开始 ───────────────────────────────────────
            if (inSupport && !_wasInStance)
            {
                _stanceFrames.Clear();
                _stanceStartPacketId = frame.PacketId;
            }

            // ── 支撑段期间：收集有效帧 ───────────────────────────
            if (inSupport && quality.IsValid)
            {
                float footHeading = _foot == ImuRole.Left
                    ? quality.LeftHeading
                    : quality.RightHeading;

                float delta = _foot == ImuRole.Left
                    ? calibration.DeltaL
                    : calibration.DeltaR;

                float corrected = HeadingUtil.AngleDiff(footHeading, delta);

                float gyrMag = _foot == ImuRole.Left
                    ? frame.Left.GyrMag
                    : frame.Right.GyrMag;

                _stanceFrames.Add(new StanceFrameRecord(corrected, gyrMag, frame.PacketId));
            }

            // ── 支撑段结束：计算并输出 ───────────────────────────
            if (!inSupport && _wasInStance)
                TryEmitStep(frame.PacketId - 1, inStance);

            // ── 更新诊断 ─────────────────────────────────────────
            _wasInStance = inSupport;
            _diag.InStance = inStance;
            _diag.CurrentStanceFrames = inSupport ? _stanceFrames.Count : 0;
        }

        // ─────────────────────────────────────────────────────────
        //  步计算
        // ─────────────────────────────────────────────────────────

        private void TryEmitStep(long packetIdEnd,bool inStance)
        {
            int frameCount = _stanceFrames.Count;

            // ── 门控1：stance 持续时间 ────────────────────────────
            if (frameCount < Config.MinStanceFrames)
            {
                string reason = $"stance 帧数不足（{frameCount} < {Config.MinStanceFrames}）";
                Debug.WriteLine($"[FPA/{_foot}] Step dropped: {reason}");
                _diag.TotalStepsDropped++;
                return;
            }

            // ── 门控2：progDir 是否就绪 ───────────────────────────
            if (!_progDir.IsReady)
            {
                Debug.WriteLine($"[FPA/{_foot}] Step dropped: progDir not ready " +
                                $"(frames={_progDir.GetDiagnostics().WindowFrameCount})");
                _diag.TotalStepsDropped++;
                return;
            }

            // ── 门控3：转弯中不输出 ───────────────────────────────
            if (_progDir.GetDiagnostics().IsTurning)
            {
                Debug.WriteLine($"[FPA/{_foot}] Step dropped: turning in progress.");
                _diag.TotalStepsDropped++;
                return;
            }

            // ── 选最低 gyro 子窗口 ────────────────────────────────
            int winSize = Math.Min(Config.BestWindowFrames, frameCount);
            int bestStart = FindBestGyroWindow(winSize);

            // ── 计算子窗口内校正 heading 圆均值 ───────────────────
            float sumSin = 0f, sumCos = 0f;
            for (int i = bestStart; i < bestStart + winSize; i++)
            {
                float h = _stanceFrames[i].CorrectedHeading;
                sumSin += MathF.Sin(h);
                sumCos += MathF.Cos(h);
            }
            float meanCorrectedH = MathF.Atan2(sumSin / winSize, sumCos / winSize);

            // ── FPA = corrected heading - progDir ─────────────────
            float progDir = _progDir.ProgDir;
            float fpaRad = HeadingUtil.AngleDiff(meanCorrectedH, progDir);
            float fpaDeg = fpaRad * (180f / MathF.PI);
            if (_foot == ImuRole.Right) { 
            fpaDeg = -fpaDeg;
            }

            var result = new StepFpaResult
            {
                Foot = _foot,
                FpaDeg = fpaDeg,
                ProgDirDeg = progDir * (180f / MathF.PI),
                CorrectedHeadingDeg = meanCorrectedH * (180f / MathF.PI),
                StanceFrameCount = frameCount,
                BestWindowStartIndex = bestStart,
                BestWindowActualFrames = winSize,
                PacketIdStart = _stanceStartPacketId,
                PacketIdEnd = packetIdEnd,
                Timestamp = DateTimeOffset.UtcNow,
                inStance=inStance
            };

            Debug.WriteLine($"[FPA/{_foot}] Step: {result}");

            _diag.TotalStepsEmitted++;
            _diag.LastFpaDeg = fpaDeg;

            OnStepFpa?.Invoke(result);
        }

        // ─────────────────────────────────────────────────────────
        //  最低 gyro 子窗口选择（滑动窗口 O(n)）
        // ─────────────────────────────────────────────────────────

        /// <summary>
        /// 在 _stanceFrames 中找连续 winSize 帧使 mean(gyroMag) 最小的起始索引。
        /// 用滑动窗口 O(n) 实现，避免 O(n²)。
        /// </summary>
        private int FindBestGyroWindow(int winSize)
        {
            int frameCount = _stanceFrames.Count;

            // 计算第一个窗口的 gyroMag 总和
            float windowSum = 0f;
            for (int i = 0; i < winSize; i++)
                windowSum += _stanceFrames[i].GyrMag;

            float minSum = windowSum;
            int bestStart = 0;

            // 滑动窗口
            for (int start = 1; start <= frameCount - winSize; start++)
            {
                windowSum += _stanceFrames[start + winSize - 1].GyrMag;
                windowSum -= _stanceFrames[start - 1].GyrMag;

                if (windowSum < minSum)
                {
                    minSum = windowSum;
                    bestStart = start;
                }
            }

            return bestStart;
        }

        private bool IsSupportPhase(StanceState stanceState)
        {
            if (!Config.IncludeTransitionFrames)
                return stanceState == StanceState.Stance;

            return stanceState != StanceState.Swing;
        }

        // ─────────────────────────────────────────────────────────
        //  Reset
        // ─────────────────────────────────────────────────────────

        public void Reset()
        {
            _wasInStance = false;
            _stanceStartPacketId = 0;
            _lastPacketId = 0;
            _stanceFrames.Clear();
            _diag.InStance = false;
            _diag.CurrentStanceFrames = 0;
            _diag.TotalStepsEmitted = 0;
            _diag.TotalStepsDropped = 0;
            _diag.LastFpaDeg = 0f;
            Debug.WriteLine($"[FPA/{_foot}] Reset.");
        }
    }
}
