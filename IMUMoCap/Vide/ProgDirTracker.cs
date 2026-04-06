using System;
using System.Diagnostics;

namespace GaitTraining.Gait
{
    // ─────────────────────────────────────────────────────────────
    //  配置
    // ─────────────────────────────────────────────────────────────

    public sealed class ProgDirConfig
    {
        /// <summary>滑动窗口长度（s）。默认 3s = 300 帧。</summary>
        public float WindowSeconds { get; set; } = 3f;

        /// <summary>
        /// 转弯检测：骨盆 yaw rate 超过此值（°/s）认为正在转弯，冻结 progDir。
        /// 正常行走骨盆 yaw rate 约 30-60°/s，默认 60°/s 作为转弯阈值。
        /// 调参：室内窄空间转弯快可降到 45°/s；直线跑道可升到 80°/s。
        /// </summary>
        public float TurnYawRateThreshDeg { get; set; } = 60f;

        /// <summary>
        /// 转弯结束判定：yaw rate 低于阈值连续此帧数后认为转弯结束，清空窗口重新积累。
        /// 默认 50 帧 = 0.5s。
        /// </summary>
        public int TurnExitStableFrames { get; set; } = 50;

        /// <summary>
        /// 进入转弯保护所需最小持续帧数。
        /// yaw rate 超过阈值必须持续此帧数才认为是真正转弯。
        /// 默认 10 帧 = 100ms，单次晃动通常 1-3 帧，不会触发。
        /// </summary>
        public int TurnEnterMinFrames { get; set; } = 10;

        /// <summary>
        /// 转弯证据窗口长度（s）。
        /// 用短时间内的累计净转角和同向性，区分真实转弯与普通步态中的骨盆摆动。
        /// </summary>
        public float TurnEvidenceSeconds { get; set; } = 0.8f;

        /// <summary>
        /// 转弯证据：窗口内累计净转角阈值（°）。
        /// 越大越不容易把普通摆动误判成转弯。
        /// </summary>
        public float TurnMinNetYawDeg { get; set; } = 15f;

        /// <summary>
        /// 转弯证据：窗口内旋转同向性阈值，范围 0-1。
        /// 1 表示几乎全程同向；越低越容易进入转弯。
        /// </summary>
        public float TurnMinConsistencyRatio { get; set; } = 0.75f;

        /// <summary>
        /// progDir 有效所需最小窗口帧数。
        /// 低于此值时 IsReady = false，步输出被压制。
        /// 默认 100 帧 = 1s。
        /// </summary>
        public int MinReadyFrames { get; set; } = 100;

        /// <summary>
        /// 转弯结束后重新建窗时的最小 ready 帧数。
        /// 用更短的稳定段快速恢复 FPA 输出，后续再继续自然积累完整窗口。
        /// </summary>
        public int MinReadyFramesAfterTurn { get; set; } = 25;

        /// <summary>IMU 采样率（Hz）。</summary>
        public int SampleRateHz { get; set; } = 100;

        internal int WindowFrames => (int)(WindowSeconds * SampleRateHz);
        internal int TurnEvidenceFrames => Math.Max(2, (int)(TurnEvidenceSeconds * SampleRateHz));
    }

    // ─────────────────────────────────────────────────────────────
    //  诊断
    // ─────────────────────────────────────────────────────────────

    public sealed class ProgDirDiagnostics
    {
        public float ProgDirDeg { get; internal set; }
        public bool IsReady { get; internal set; }
        public bool IsTurning { get; internal set; }
        public int WindowFrameCount { get; internal set; }
        public float LastYawRateDeg { get; internal set; }
        public float TurnEvidenceNetYawDeg { get; internal set; }
        public float TurnEvidenceConsistencyRatio { get; internal set; }

        public override string ToString() =>
            $"ProgDir={ProgDirDeg:F1}° Ready={IsReady} Turning={IsTurning} " +
            $"WinFrames={WindowFrameCount} YawRate={LastYawRateDeg:F1}°/s " +
            $"NetYaw={TurnEvidenceNetYawDeg:F1}° Cons={TurnEvidenceConsistencyRatio:F2}";
    }

    // ─────────────────────────────────────────────────────────────
    //  ProgDirTracker
    // ─────────────────────────────────────────────────────────────

    /// <summary>
    /// 骨盆 heading 圆滑动均值，输出行进方向 progDir（rad）。
    /// <para>
    /// 机制1：质量门控通过且 yaw rate 低于转弯阈值时，写入滑动窗口并更新 progDir。
    /// 机制2：yaw rate 持续超过阈值（转弯中），冻结 progDir；转弯稳定结束后清空窗口重新积累。
    /// </para>
    /// <para>线程假设：单线程调用。</para>
    /// </summary>
    public sealed class ProgDirTracker
    {
        public ProgDirConfig Config { get; set; }

        // ── 诊断 ─────────────────────────────────────────────────
        private readonly ProgDirDiagnostics _diag = new();
        public ProgDirDiagnostics GetDiagnostics() => _diag;

        // 内部状态里加一个进入计数器
        private int _turnEnterFrameCount;  // 新增

        // ── 滑动窗口（sin/cos 累加，O(1) 圆均值） ────────────────
        // 用循环缓冲存原始 heading，同时维护 sin/cos 累加和，避免每帧全量重算
        private float[] _sinBuf;
        private float[] _cosBuf;
        private int _head;
        private int _count;
        private float _sinSum;
        private float _cosSum;

        // ── 转弯证据窗口（短窗累计净转角） ────────────────────────
        private float[] _turnDiffBuf;
        private int _turnDiffHead;
        private int _turnDiffCount;
        private float _turnDiffSum;
        private float _turnAbsDiffSum;

        // ── 转弯状态 ─────────────────────────────────────────────
        private bool _isTurning;
        private int _stableFrameCount;   // 转弯后连续稳定帧计数

        // ── 上一帧 heading（用于 yaw rate 计算） ─────────────────
        private float _prevHeading = float.NaN;

        // ── 当前 progDir ─────────────────────────────────────────
        private float _progDir;            // rad，圆均值结果
        public float ProgDir => _progDir; // rad
        private int _readyFrameTarget;

        public bool IsReady => _count >= _readyFrameTarget;

        // ── 构造 ─────────────────────────────────────────────────
        public ProgDirTracker(ProgDirConfig? config = null)
        {
            Config = config ?? new ProgDirConfig();
            int cap = Math.Max(1, Config.WindowFrames);
            _sinBuf = new float[cap];
            _cosBuf = new float[cap];
            _turnDiffBuf = new float[Config.TurnEvidenceFrames];
            _readyFrameTarget = Config.MinReadyFrames;
        }

        // ─────────────────────────────────────────────────────────
        //  主入口：每帧调用
        // ─────────────────────────────────────────────────────────

        /// <summary>
        /// 传入当前帧骨盆 heading（rad，来自 QualityResult.PelvisHeading）和质量标志。
        /// 质量门控不通过时直接跳过，不更新任何状态。
        /// </summary>
        public void Update(float pelvisHeading, bool qualityValid)
        {
            if (!qualityValid) return;

            // ── 计算瞬时 yaw rate ────────────────────────────────
            float yawRateDeg = 0f;
            bool hasPrev = !float.IsNaN(_prevHeading);

            if (hasPrev)
            {
                float diffRad = HeadingUtil.AngleDiff(pelvisHeading, _prevHeading);
                float diffDegSigned = diffRad * (180f / MathF.PI);
                yawRateDeg = MathF.Abs(diffDegSigned) * Config.SampleRateHz;
                PushTurnEvidence(diffDegSigned);
            }

            float turnThresh = Config.TurnYawRateThreshDeg;
            bool turnEvidenceStrong = HasStrongTurnEvidence();
            bool turnCandidate = hasPrev && yawRateDeg > turnThresh && turnEvidenceStrong;

            // ── 转弯状态机 ───────────────────────────────────────
            if (!_isTurning)
            {
                if (turnCandidate)
                {
                    _turnEnterFrameCount++;
                    if (_turnEnterFrameCount >= Config.TurnEnterMinFrames)
                    {
                        // 持续超过阈值足够长，才真正进入转弯
                        _isTurning = true;
                        _stableFrameCount = 0;
                        _turnEnterFrameCount = 0;
                        _readyFrameTarget = Config.MinReadyFrames;
                        Debug.WriteLine(
                            $"[ProgDir] Turn detected. YawRate={yawRateDeg:F1}°/s");
                    }
                    // 未达到最小帧数：暂时不写窗口，不让转弯起始段污染 progDir
                }
                else
                {
                    // 证据不足：视为正常步态摆动，正常写窗口
                    _turnEnterFrameCount = 0;
                    PushToWindow(pelvisHeading);
                    if (_count > 0)
                        _progDir = MathF.Atan2(_sinSum / _count, _cosSum / _count);
                }
            }
            else
            {
                // 转弯中：等待稳定
                if (yawRateDeg <= turnThresh)
                {
                    _stableFrameCount++;
                    if (_stableFrameCount >= Config.TurnExitStableFrames)
                    {
                        // 转弯结束：清空窗口，重新积累
                        Debug.WriteLine(
                            $"[ProgDir] Turn ended. Clearing window, re-accumulating.");
                        ClearWindow();
                        _isTurning = false;
                        _stableFrameCount = 0;
                        _readyFrameTarget = Math.Max(1, Math.Min(
                            Config.MinReadyFramesAfterTurn,
                            Config.MinReadyFrames));
                        ResetTurnEvidence();

                        // 把当前帧作为第一个新样本写入
                        PushToWindow(pelvisHeading);
                        _progDir = pelvisHeading;
                    }
                    // 未达到稳定帧数：继续等待，不写窗口
                }
                else
                {
                    // 仍在转弯：重置稳定计数
                    _stableFrameCount = 0;
                }
            }

            _prevHeading = pelvisHeading;

            // ── 更新诊断 ─────────────────────────────────────────
            _diag.ProgDirDeg = _progDir * (180f / MathF.PI);
            _diag.IsReady = IsReady;
            _diag.IsTurning = _isTurning;
            _diag.WindowFrameCount = _count;
            _diag.LastYawRateDeg = yawRateDeg;
            _diag.TurnEvidenceNetYawDeg = _turnDiffSum;
            _diag.TurnEvidenceConsistencyRatio =
                _turnAbsDiffSum > 1e-5f ? MathF.Abs(_turnDiffSum) / _turnAbsDiffSum : 0f;
        }

        // ─────────────────────────────────────────────────────────
        //  滑动窗口（O(1) sin/cos 累加）
        // ─────────────────────────────────────────────────────────

        private void PushToWindow(float heading)
        {
            EnsureWindowCapacity();
            int cap = _sinBuf.Length;

            if (_count == cap)
            {
                // 移除最老元素
                _sinSum -= _sinBuf[_head];
                _cosSum -= _cosBuf[_head];
            }

            _sinBuf[_head] = MathF.Sin(heading);
            _cosBuf[_head] = MathF.Cos(heading);
            _sinSum += _sinBuf[_head];
            _cosSum += _cosBuf[_head];
            _head = (_head + 1) % cap;
            if (_count < cap) _count++;
        }

        private void ClearWindow()
        {
            Array.Clear(_sinBuf, 0, _sinBuf.Length);
            Array.Clear(_cosBuf, 0, _cosBuf.Length);
            _head = 0;
            _count = 0;
            _sinSum = 0f;
            _cosSum = 0f;
        }

        private void PushTurnEvidence(float diffDegSigned)
        {
            EnsureTurnEvidenceCapacity();
            int cap = _turnDiffBuf.Length;

            if (_turnDiffCount == cap)
            {
                float oldest = _turnDiffBuf[_turnDiffHead];
                _turnDiffSum -= oldest;
                _turnAbsDiffSum -= MathF.Abs(oldest);
            }
            else
            {
                _turnDiffCount++;
            }

            _turnDiffBuf[_turnDiffHead] = diffDegSigned;
            _turnDiffSum += diffDegSigned;
            _turnAbsDiffSum += MathF.Abs(diffDegSigned);
            _turnDiffHead = (_turnDiffHead + 1) % cap;
        }

        private void ResetTurnEvidence()
        {
            _turnDiffHead = 0;
            _turnDiffCount = 0;
            _turnDiffSum = 0f;
            _turnAbsDiffSum = 0f;
            Array.Clear(_turnDiffBuf, 0, _turnDiffBuf.Length);
        }

        private bool HasStrongTurnEvidence()
        {
            if (_turnDiffCount < Config.TurnEvidenceFrames)
                return false;

            float netYawDeg = MathF.Abs(_turnDiffSum);
            if (netYawDeg < Config.TurnMinNetYawDeg)
                return false;

            if (_turnAbsDiffSum <= 1e-5f)
                return false;

            float consistency = netYawDeg / _turnAbsDiffSum;
            return consistency >= Config.TurnMinConsistencyRatio;
        }

        // ─────────────────────────────────────────────────────────
        //  窗口容量自适应
        // ─────────────────────────────────────────────────────────

        private int _lastWindowFrames = -1;
        private int _lastTurnEvidenceFrames = -1;

        private void EnsureWindowCapacity()
        {
            int needed = Config.WindowFrames;
            if (needed == _lastWindowFrames) return;

            // 容量变化：重建（清空，重新积累）
            _sinBuf = new float[needed];
            _cosBuf = new float[needed];
            ClearWindow();
            _lastWindowFrames = needed;
            Debug.WriteLine($"[ProgDir] Window resized to {needed} frames.");
        }

        private void EnsureTurnEvidenceCapacity()
        {
            int needed = Config.TurnEvidenceFrames;
            if (needed == _lastTurnEvidenceFrames) return;

            _turnDiffBuf = new float[needed];
            ResetTurnEvidence();
            _lastTurnEvidenceFrames = needed;
            Debug.WriteLine($"[ProgDir] Turn evidence window resized to {needed} frames.");
        }

        // ─────────────────────────────────────────────────────────
        //  Reset
        // ─────────────────────────────────────────────────────────

        public void Reset()
        {
            ClearWindow();
            _isTurning = false;
            _stableFrameCount = 0;
            _prevHeading = float.NaN;
            _progDir = 0f;
            _readyFrameTarget = Config.MinReadyFrames;
            _diag.ProgDirDeg = 0f;
            _diag.IsReady = false;
            _diag.IsTurning = false;
            _diag.WindowFrameCount = 0;
            _diag.LastYawRateDeg = 0f;
            _diag.TurnEvidenceNetYawDeg = 0f;
            _diag.TurnEvidenceConsistencyRatio = 0f;
            _turnEnterFrameCount = 0;
            ResetTurnEvidence();
            Debug.WriteLine("[ProgDir] Reset.");
        }
    }
}
