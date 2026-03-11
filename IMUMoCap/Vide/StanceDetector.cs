using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace GaitTraining.Gait
{
    // ─────────────────────────────────────────────────────────────
    //  状态枚举
    // ─────────────────────────────────────────────────────────────

    public enum StanceState
    {
        Swing,     // 摆动期
        Entering,  // 进入缓冲（条件满足但未达 MinEnterMs）
        Stance,    // 支撑期（稳定）
        Exiting,   // 退出缓冲（条件满足但未达 MinExitMs）
    }

    // ─────────────────────────────────────────────────────────────
    //  配置
    // ─────────────────────────────────────────────────────────────

    /// <summary>
    /// StanceDetector 可调参数。所有阈值均可在运行时修改，下一帧立即生效。
    /// </summary>
    public sealed class StanceDetectorConfig
    {
        // ── 进入阈值（窗口 median） ───────────────────────────────
        /// <summary>进入 stance：窗口 median GyrMag 上限（rad/s）。降低 → 更难进入。</summary>
        public float EnterGyrThresh { get; set; } = 0.5f;

        /// <summary>进入 stance：窗口 median |AccMag-g| 上限（m/s²）。降低 → 更难进入。</summary>
        public float EnterAccDevThresh { get; set; } = 0.8f;

        // ── 退出阈值（瞬时值） ───────────────────────────────────
        /// <summary>退出 stance：瞬时 GyrMag 下限（rad/s）。降低 → 更容易退出。</summary>
        public float ExitGyrThresh { get; set; } = 1.0f;

        /// <summary>退出 stance：瞬时 |AccMag-g| 下限（m/s²）。降低 → 更容易退出。</summary>
        public float ExitAccDevThresh { get; set; } = 1.5f;

        /// <summary>
        /// 退出条件逻辑。true = 两个信号都超标才退出（更保守）；
        /// false = 任一超标即退出（更敏捷，默认）。
        /// </summary>
        public bool ExitRequireBoth { get; set; } = false;

        // ── 窗口与持续时间 ───────────────────────────────────────
        /// <summary>Median 计算窗口（ms）。增大 → 对冲击更鲁棒，但响应变慢。</summary>
        public int MedianWindowMs { get; set; } = 80;

        /// <summary>进入 stance 所需最小持续时间（ms）。增大 → 减少误触发。</summary>
        public int MinEnterMs { get; set; } = 80;

        /// <summary>退出 stance 所需最小持续时间（ms）。增大 → stance 内抖动容忍度更高。</summary>
        public int MinExitMs { get; set; } = 20;

        // ── 物理常数 ─────────────────────────────────────────────
        /// <summary>重力加速度（m/s²）。</summary>
        public float G { get; set; } = 9.81f;

        // ── 采样率 ───────────────────────────────────────────────
        /// <summary>IMU 采样率（Hz），用于 ms → 帧数换算。</summary>
        public int SampleRateHz { get; set; } = 100;

        // ── 派生帧数（只读，内部使用） ───────────────────────────
        internal int MedianWindowFrames => Math.Max(1, MedianWindowMs * SampleRateHz / 1000);
        internal int MinEnterFrames => Math.Max(1, MinEnterMs * SampleRateHz / 1000);
        internal int MinExitFrames => Math.Max(1, MinExitMs * SampleRateHz / 1000);
    }

    // ─────────────────────────────────────────────────────────────
    //  诊断快照（供 UI 实时显示，无需加锁，同一线程调用）
    // ─────────────────────────────────────────────────────────────

    public sealed class StanceDiagnostics
    {
        /// <summary>当前状态机状态。</summary>
        public StanceState State { get; internal set; }

        /// <summary>当前帧 median GyrMag（rad/s）。</summary>
        public float LastGyrMedian { get; internal set; }

        /// <summary>当前帧 median |AccMag-g|（m/s²）。</summary>
        public float LastAccDevMedian { get; internal set; }

        /// <summary>当前帧瞬时 GyrMag（rad/s）。</summary>
        public float LastGyrInstant { get; internal set; }

        /// <summary>当前帧瞬时 |AccMag-g|（m/s²）。</summary>
        public float LastAccDevInstant { get; internal set; }

        /// <summary>当前缓冲状态已持续帧数（Entering/Exiting 时有意义）。</summary>
        public int BufferFrameCount { get; internal set; }

        /// <summary>累计进入 stance 次数（步数参考）。</summary>
        public int TotalStanceEnterCount { get; internal set; }

        public override string ToString() =>
            $"State={State} GyrMed={LastGyrMedian:F3} AccDevMed={LastAccDevMedian:F3} " +
            $"GyrInst={LastGyrInstant:F3} AccDevInst={LastAccDevInstant:F3} " +
            $"BufFrames={BufferFrameCount} Enters={TotalStanceEnterCount}";
    }

    // ─────────────────────────────────────────────────────────────
    //  StanceDetector
    // ─────────────────────────────────────────────────────────────

    /// <summary>
    /// 单脚 stance/swing 检测器。
    /// <para>
    /// 使用方式：每帧调用 <see cref="Update"/>，返回当前 <see cref="StanceState"/>。
    /// 调用方（标定模块、FPA模块）根据返回状态决定业务逻辑，本类不持有任何业务数据。
    /// </para>
    /// <para>
    /// 线程假设：与 ImuFrameAggregator 相同，在 Xsens 回调线程单线程调用。
    /// </para>
    /// </summary>
    public sealed class StanceDetector
    {
        // ── 配置（可运行时替换） ──────────────────────────────────
        public StanceDetectorConfig Config { get; set; }

        // ── 诊断 ─────────────────────────────────────────────────
        private readonly StanceDiagnostics _diag = new();
        public StanceDiagnostics GetDiagnostics() => _diag;

        // ── 内部状态 ─────────────────────────────────────────────
        private StanceState _state = StanceState.Swing;

        // 缓冲状态计时（帧数）
        private int _bufferFrameCount;

        // Median 滑动窗口（循环缓冲）
        private readonly CircularBuffer<float> _gyrWindow;
        private readonly CircularBuffer<float> _accDevWindow;

        // 排序缓冲（复用，避免每帧 alloc）
        private float[] _sortBuf;

        // ── 构造 ─────────────────────────────────────────────────
        public StanceDetector(StanceDetectorConfig? config = null)
        {
            Config = config ?? new StanceDetectorConfig();

            // 窗口初始容量按默认配置分配；Reset 时若配置变化会重建
            int cap = Config.MedianWindowFrames;
            _gyrWindow = new CircularBuffer<float>(cap);
            _accDevWindow = new CircularBuffer<float>(cap);
            _sortBuf = new float[cap];
        }

        // ─────────────────────────────────────────────────────────
        //  主入口：每帧调用
        // ─────────────────────────────────────────────────────────

        /// <summary>
        /// 传入当前帧脚部 IMU 样本，返回更新后的 <see cref="StanceState"/>。
        /// </summary>
        /// <param name="sample">来自 SyncedFrame 的单脚 ImuSample（Left 或 Right）。</param>
        public StanceState Update(in Imu.ImuSample sample)
        {
            // 1. 计算瞬时信号
            float gyrInst = sample.GyrMag;
            float accDevInst = MathF.Abs(sample.AccMag - Config.G);

            // 2. 更新 median 窗口
            EnsureWindowCapacity();
            _gyrWindow.Push(gyrInst);
            _accDevWindow.Push(accDevInst);

            float gyrMed = Median(_gyrWindow, ref _sortBuf);
            float accDevMed = Median(_accDevWindow, ref _sortBuf);

            // 3. 计算进入/退出条件
            bool enterCond = gyrMed < Config.EnterGyrThresh
                          && accDevMed < Config.EnterAccDevThresh;

            bool exitGyr = gyrInst > Config.ExitGyrThresh;
            bool exitAccDev = accDevInst > Config.ExitAccDevThresh;
            bool exitCond = Config.ExitRequireBoth
                                ? (exitGyr && exitAccDev)
                                : (exitGyr || exitAccDev);

            // 4. 状态机转移
            _state = Transition(_state, enterCond, exitCond);

            // 5. 更新诊断
            _diag.State = _state;
            _diag.LastGyrMedian = gyrMed;
            _diag.LastAccDevMedian = accDevMed;
            _diag.LastGyrInstant = gyrInst;
            _diag.LastAccDevInstant = accDevInst;
            _diag.BufferFrameCount = _bufferFrameCount;

            return _state;
        }

        // ─────────────────────────────────────────────────────────
        //  状态机
        // ─────────────────────────────────────────────────────────

        private StanceState Transition(StanceState current, bool enterCond, bool exitCond)
        {
            switch (current)
            {
                // ── SWING ────────────────────────────────────────
                case StanceState.Swing:
                    if (enterCond)
                    {
                        _bufferFrameCount = 1;
                        // 进入条件满足且 MinEnterFrames=1，直接进 Stance
                        if (_bufferFrameCount >= Config.MinEnterFrames)
                        {
                            _diag.TotalStanceEnterCount++;
                            _bufferFrameCount = 0;
                            return StanceState.Stance;
                        }
                        return StanceState.Entering;
                    }
                    return StanceState.Swing;

                // ── ENTERING ─────────────────────────────────────
                case StanceState.Entering:
                    if (enterCond)
                    {
                        _bufferFrameCount++;
                        if (_bufferFrameCount >= Config.MinEnterFrames)
                        {
                            _diag.TotalStanceEnterCount++;
                            _bufferFrameCount = 0;
                            return StanceState.Stance;
                        }
                        return StanceState.Entering;
                    }
                    else
                    {
                        // 条件中断，回退到 Swing
                        _bufferFrameCount = 0;
                        return StanceState.Swing;
                    }

                // ── STANCE ───────────────────────────────────────
                case StanceState.Stance:
                    if (exitCond)
                    {
                        _bufferFrameCount = 1;
                        if (_bufferFrameCount >= Config.MinExitFrames)
                        {
                            _bufferFrameCount = 0;
                            return StanceState.Swing;
                        }
                        return StanceState.Exiting;
                    }
                    return StanceState.Stance;

                // ── EXITING ──────────────────────────────────────
                case StanceState.Exiting:
                    if (exitCond)
                    {
                        _bufferFrameCount++;
                        if (_bufferFrameCount >= Config.MinExitFrames)
                        {
                            _bufferFrameCount = 0;
                            return StanceState.Swing;
                        }
                        return StanceState.Exiting;
                    }
                    else
                    {
                        // 退出条件中断，回到 Stance
                        _bufferFrameCount = 0;
                        return StanceState.Stance;
                    }

                default:
                    return StanceState.Swing;
            }
        }

        // ─────────────────────────────────────────────────────────
        //  Reset
        // ─────────────────────────────────────────────────────────

        /// <summary>
        /// 重置状态机和所有窗口缓冲。标定开始前、会话切换时调用。
        /// </summary>
        public void Reset()
        {
            _state = StanceState.Swing;
            _bufferFrameCount = 0;

            int cap = Config.MedianWindowFrames;
            _gyrWindow.Reset(cap);
            _accDevWindow.Reset(cap);

            if (_sortBuf.Length < cap)
                _sortBuf = new float[cap];

            _diag.State = StanceState.Swing;
            _diag.LastGyrMedian = 0;
            _diag.LastAccDevMedian = 0;
            _diag.LastGyrInstant = 0;
            _diag.LastAccDevInstant = 0;
            _diag.BufferFrameCount = 0;
            _diag.TotalStanceEnterCount = 0;

            Debug.WriteLine("[StanceDetector] Reset.");
        }

        // ─────────────────────────────────────────────────────────
        //  窗口容量自适应（配置变更时）
        // ─────────────────────────────────────────────────────────

        private int _lastWindowFrames = -1;

        private void EnsureWindowCapacity()
        {
            int needed = Config.MedianWindowFrames;
            if (needed == _lastWindowFrames) return;

            _gyrWindow.Reset(needed);
            _accDevWindow.Reset(needed);
            if (_sortBuf.Length < needed)
                _sortBuf = new float[needed];
            _lastWindowFrames = needed;

            Debug.WriteLine($"[StanceDetector] MedianWindow resized to {needed} frames.");
        }

        // ─────────────────────────────────────────────────────────
        //  Median 计算（部分排序，O(n log n)，n≤20，完全可接受）
        // ─────────────────────────────────────────────────────────

        private static float Median(CircularBuffer<float> buf, ref float[] sortBuf)
        {
            int count = buf.Count;
            if (count == 0) return 0f;

            // 确保 sortBuf 够大
            if (sortBuf.Length < count)
                sortBuf = new float[count];

            buf.CopyTo(sortBuf, 0);
            Array.Sort(sortBuf, 0, count);

            return (count % 2 == 1)
                ? sortBuf[count / 2]
                : (sortBuf[count / 2 - 1] + sortBuf[count / 2]) * 0.5f;
        }
    }

    // ─────────────────────────────────────────────────────────────
    //  CircularBuffer<T>（内部工具类）
    // ─────────────────────────────────────────────────────────────

    /// <summary>
    /// 定长循环缓冲，满后覆盖最老元素。不做线程同步。
    /// </summary>
    public sealed class CircularBuffer<T>
    {
        private T[] _buf;
        private int _head;   // 下一个写入位置
        private int _count;

        public int Count => _count;
        public int Capacity => _buf.Length;

        public CircularBuffer(int capacity)
        {
            _buf = new T[Math.Max(1, capacity)];
        }

        public void Push(T value)
        {
            _buf[_head] = value;
            _head = (_head + 1) % _buf.Length;
            if (_count < _buf.Length) _count++;
        }

        /// <summary>按从旧到新的顺序复制到目标数组。</summary>
        public void CopyTo(T[] dest, int destIndex)
        {
            if (_count == 0) return;
            int cap = _buf.Length;
            // 最老元素的起始索引
            int start = (_count < cap) ? 0 : _head;
            for (int i = 0; i < _count; i++)
                dest[destIndex + i] = _buf[(start + i) % cap];
        }

        /// <summary>清空并可选重置容量（容量变更时调用）。</summary>
        public void Reset(int newCapacity = -1)
        {
            if (newCapacity > 0 && newCapacity != _buf.Length)
                _buf = new T[newCapacity];
            _head = 0;
            _count = 0;
        }
    }
}