using IMUMoCap.Methods;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;

namespace GaitTraining.Imu
{
    // ─────────────────────────────────────────────────────────────
    //  枚举 & 原始数据结构
    // ─────────────────────────────────────────────────────────────


    /// <summary>
    /// 从 Xsens 回调中提取的单颗 IMU 原始数据。
    /// 调用方负责填充；本层不依赖任何 Xsens SDK 类型，方便单元测试。
    /// </summary>
    public readonly struct ImuRawData
    {
        // 传感器系 → 世界系（ENU, Z-up），直接来自 XsQuaternion
        public readonly Quaternion Orientation;   // (X, Y, Z, W)

        // 来自 XsCalibratedData
        public readonly Vector3 Acc;   // m/s²
        public readonly Vector3 Gyr;   // rad/s
        public readonly Vector3 Mag;   // μT（或设备归一化单位，保持一致即可）

        public ImuRawData(Quaternion orientation, Vector3 acc, Vector3 gyr, Vector3 mag)
        {
            Orientation = orientation;
            Acc = acc;
            Gyr = gyr;
            Mag = mag;
        }
    }

    /// <summary>
    /// 聚合后单颗 IMU 的样本，含预计算标量，供下游模块直接使用。
    /// </summary>
    public readonly struct ImuSample
    {
        public readonly ImuRole Sensor;
        public readonly Quaternion Orientation;
        public readonly Vector3 Acc;
        public readonly Vector3 Gyr;
        public readonly Vector3 Mag;

        // 预计算，避免下游重复开方
        public readonly float AccMag;   // |acc|  m/s²
        public readonly float GyrMag;   // |gyr|  rad/s
        public readonly float MagNorm;  // |mag|  μT

        public ImuSample(ImuRole sensor, ImuRawData raw)
        {
            Sensor = sensor;
            Orientation = raw.Orientation;
            Acc = raw.Acc;
            Gyr = raw.Gyr;
            Mag = raw.Mag;
            AccMag = raw.Acc.Length();
            GyrMag = raw.Gyr.Length();
            MagNorm = raw.Mag.Length();
        }
    }

    /// <summary>
    /// 三颗 IMU 在同一 packetId 下的同步帧。
    /// IsComplete = true 表示三颗均到齐；false 表示超时后强制发出（含缺失颗）。
    /// 下游模块应在 IsComplete = false 时跳过处理。
    /// </summary>
    public readonly struct SyncedFrame
    {
        public readonly long PacketId;
        public readonly DateTimeOffset Timestamp;   // 最后一颗到达时的本机时间
        public readonly ImuSample Pelvis;
        public readonly ImuSample Left;
        public readonly ImuSample Right;
        public readonly bool IsComplete;

        public SyncedFrame(
            long packetId,
            DateTimeOffset timestamp,
            ImuSample pelvis,
            ImuSample left,
            ImuSample right,
            bool isComplete)
        {
            PacketId = packetId;
            Timestamp = timestamp;
            Pelvis = pelvis;
            Left = left;
            Right = right;
            IsComplete = isComplete;
        }
    }

    // ─────────────────────────────────────────────────────────────
    //  配置 & 诊断
    // ─────────────────────────────────────────────────────────────

    public sealed class AggregatorConfig
    {
        /// <summary>等待同一 packetId 其余颗的超时帧数（@100 Hz，5帧=50 ms）。</summary>
        public int TimeoutFrames { get; set; } = 5;

        /// <summary>pending buffer 最大保留的 packetId 数量。超出时强制清理最老条目。</summary>
        public int MaxPendingPackets { get; set; } = 10;
    }

    public sealed class AggregatorDiagnostics
    {
        public long TotalFramesEmitted { get; internal set; }
        public long IncompleteFrames { get; internal set; }   // 超时强制发出
        public long DroppedLateArrivals { get; internal set; }   // 帧已发出后迟到的颗
        public int CurrentPendingCount { get; internal set; }

        public override string ToString() =>
            $"Emitted={TotalFramesEmitted} Incomplete={IncompleteFrames} " +
            $"LateArrivals={DroppedLateArrivals} Pending={CurrentPendingCount}";
    }

    // ─────────────────────────────────────────────────────────────
    //  内部 Pending 条目
    // ─────────────────────────────────────────────────────────────

    public sealed class PendingEntry
    {
        public long PacketId;
        public long ArrivalFrameCounter;   // 用于超时判断
        public DateTimeOffset LastArrivalTime;
        public ImuSample? Pelvis;
        public ImuSample? Left;
        public ImuSample? Right;

        public bool IsComplete => Pelvis.HasValue && Left.HasValue && Right.HasValue;

        public int PresentCount =>
            (Pelvis.HasValue ? 1 : 0) +
            (Left.HasValue ? 1 : 0) +
            (Right.HasValue ? 1 : 0);
    }

    // ─────────────────────────────────────────────────────────────
    //  ImuFrameAggregator
    // ─────────────────────────────────────────────────────────────

    /// <summary>
    /// 按 packetId 聚合三颗 IMU（Pelvis / Left / Right）的回调数据，
    /// 三颗全到齐后通过 <see cref="OnFrameReady"/> 事件发出 <see cref="SyncedFrame"/>。
    /// <para>
    /// 线程安全假设：三颗 IMU 的回调在同一线程串行调用，无需额外加锁。
    /// 若日后改为多线程回调，在 <see cref="OnImuData"/> 入口加 lock(_sync) 即可。
    /// </para>
    /// </summary>
    public sealed class ImuFrameAggregator
    {
        // ── 配置 ──────────────────────────────────────────────────
        public AggregatorConfig Config { get; }

        // ── 事件 ──────────────────────────────────────────────────
        /// <summary>完整或超时帧就绪。订阅者应检查 <see cref="SyncedFrame.IsComplete"/>。</summary>
        public event Action<SyncedFrame>? OnFrameReady;

        // ── 内部状态 ──────────────────────────────────────────────
        // key = packetId，按插入顺序维护（Dictionary + 插入序列表）
        private readonly Dictionary<long, PendingEntry> _pending = new();
        private readonly LinkedList<long> _insertionOrder = new();   // 维护到达顺序，用于超时清理

        private long _frameCounter;      // 每次 OnImuData 调用递增，用于超时判断
        private long _latestPacketId = long.MinValue;

        // 已完成发出的 packetId 集合（小型循环缓冲，防止迟到颗误建新条目）
        private readonly Queue<long> _emittedRecent = new();
        private const int EmittedCacheSize = 20;

        // ── 诊断 ──────────────────────────────────────────────────
        private readonly AggregatorDiagnostics _diag = new();
        public AggregatorDiagnostics GetDiagnostics()
        {
            _diag.CurrentPendingCount = _pending.Count;
            return _diag;
        }

        // ── 构造 ──────────────────────────────────────────────────
        public ImuFrameAggregator(AggregatorConfig? config = null)
        {
            Config = config ?? new AggregatorConfig();
        }

        // ─────────────────────────────────────────────────────────
        //  主入口：IMU 回调
        // ─────────────────────────────────────────────────────────

        /// <summary>
        /// 从 Xsens 回调中调用。同一 packetId 的三颗都到齐后立即触发 OnFrameReady。
        /// </summary>
        public void OnImuData(ImuRole sensor, long packetId, ImuRawData raw)
        {
            _frameCounter++;

            // 1. 超时清理（在处理新数据前先扫描 pending 中过期条目）
            FlushTimedOut();

            // 2. 迟到颗检查（帧已发出后又来了同 packetId 的数据）
            if (IsAlreadyEmitted(packetId))
            {
                _diag.DroppedLateArrivals++;
                Debug.WriteLine(
                    $"[Aggregator] Late arrival dropped: sensor={sensor} packetId={packetId}");
                return;
            }

            // 3. 获取或创建 pending 条目
            if (!_pending.TryGetValue(packetId, out var entry))
            {
                entry = new PendingEntry
                {
                    PacketId = packetId,
                    ArrivalFrameCounter = _frameCounter,
                };
                _pending[packetId] = entry;
                _insertionOrder.AddLast(packetId);

                // 维护 MaxPendingPackets 上限
                EnforceMaxPending();
            }

            // 4. 填充对应颗
            var sample = new ImuSample(sensor, raw);
            switch (sensor)
            {
                case ImuRole.Pelvis: entry.Pelvis = sample; break;
                case ImuRole.Left: entry.Left = sample; break;
                case ImuRole.Right: entry.Right = sample; break;
            }
            entry.LastArrivalTime = DateTimeOffset.UtcNow;

            // 追踪最新 packetId（用于诊断/外部查询）
            if (packetId > _latestPacketId)
                _latestPacketId = packetId;

            // 5. 三颗全到齐，立即发出
            if (entry.IsComplete)
                EmitAndRemove(entry, isComplete: true);
        }

        // ─────────────────────────────────────────────────────────
        //  超时清理
        // ─────────────────────────────────────────────────────────

        private void FlushTimedOut()
        {
            // 遍历插入序（最老在前），找到超时的条目强制发出
            var node = _insertionOrder.First;
            while (node != null)
            {
                var next = node.Next;
                long pid = node.Value;

                if (_pending.TryGetValue(pid, out var entry))
                {
                    long age = _frameCounter - entry.ArrivalFrameCounter;
                    if (age >= Config.TimeoutFrames)
                    {
                        Debug.WriteLine(
                            $"[Aggregator] Timeout flush: packetId={pid} " +
                            $"present={entry.PresentCount}/3 age={age}frames");
                        EmitAndRemove(entry, isComplete: false);
                    }
                }
                node = next;
            }
        }

        // ─────────────────────────────────────────────────────────
        //  MaxPending 强制清理（防堆积）
        // ─────────────────────────────────────────────────────────

        private void EnforceMaxPending()
        {
            while (_pending.Count > Config.MaxPendingPackets && _insertionOrder.First != null)
            {
                long oldest = _insertionOrder.First.Value;
                if (_pending.TryGetValue(oldest, out var entry))
                {
                    Debug.WriteLine(
                        $"[Aggregator] MaxPending evict: packetId={oldest} " +
                        $"present={entry.PresentCount}/3");
                    EmitAndRemove(entry, isComplete: false);
                }
                else
                {
                    _insertionOrder.RemoveFirst();
                }
            }
        }

        // ─────────────────────────────────────────────────────────
        //  发出帧并清理
        // ─────────────────────────────────────────────────────────

        private void EmitAndRemove(PendingEntry entry, bool isComplete)
        {
            // 用 default(ImuSample) 填充缺失颗（IsComplete=false 时下游应跳过）
            var frame = new SyncedFrame(
                packetId: entry.PacketId,
                timestamp: entry.LastArrivalTime == default
                                ? DateTimeOffset.UtcNow
                                : entry.LastArrivalTime,
                pelvis: entry.Pelvis ?? default,
                left: entry.Left ?? default,
                right: entry.Right ?? default,
                isComplete: isComplete
            );

            // 从 pending 移除
            _pending.Remove(entry.PacketId);
            _insertionOrder.Remove(entry.PacketId);   // O(n)，n 极小（≤10），可接受

            // 记录到已发出缓存
            MarkEmitted(entry.PacketId);

            // 更新诊断
            _diag.TotalFramesEmitted++;
            if (!isComplete) _diag.IncompleteFrames++;

            // 触发事件（在调用线程，即 Xsens 回调线程）
            OnFrameReady?.Invoke(frame);
        }

        // ─────────────────────────────────────────────────────────
        //  已发出缓存（防迟到颗）
        // ─────────────────────────────────────────────────────────

        private void MarkEmitted(long packetId)
        {
            _emittedRecent.Enqueue(packetId);
            if (_emittedRecent.Count > EmittedCacheSize)
                _emittedRecent.Dequeue();
        }

        private bool IsAlreadyEmitted(long packetId)
        {
            // 线性扫描，队列长度固定 ≤20，开销可忽略
            foreach (var id in _emittedRecent)
                if (id == packetId) return true;
            return false;
        }

        // ─────────────────────────────────────────────────────────
        //  Reset
        // ─────────────────────────────────────────────────────────

        public void Reset()
        {
            _pending.Clear();
            _insertionOrder.Clear();
            _emittedRecent.Clear();
            _frameCounter = 0;
            _latestPacketId = long.MinValue;
            _diag.TotalFramesEmitted = 0;
            _diag.IncompleteFrames = 0;
            _diag.DroppedLateArrivals = 0;
            Debug.WriteLine("[Aggregator] Reset.");
        }
    }
}