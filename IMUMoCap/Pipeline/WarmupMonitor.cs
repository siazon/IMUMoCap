// IMUMoCap/Pipeline/WarmupMonitor.cs
using System;
using System.Linq;

namespace IMUMoCap.Pipeline
{
    /// <param name="PelvisSec">各传感器的预热秒数(已完成的显示 RequiredSeconds);null = 还没收到滤波器有效的数据。</param>
    public readonly record struct WarmupSnapshot(
        bool IsComplete, double StillSeconds, long? MeasurementStartPacketId, long? CompletedPacketId,
        double? PelvisSec, double? LeftSec, double? RightSec);

    /// <summary>
    /// MTw 滤波器预热门控(docs/stride_fpa_design.md 第六节)。
    ///
    /// XKF3-hm 在 PC 端 XDA 里运行,每次测量开始都从头初始化;白皮书要求开始后 10–15 s 低运动。
    /// 规则(每个传感器各自独立,因为每个 MTw 有自己的滤波器):从滤波器有效(StatusWord OrientationValid)起,
    /// 连续低运动(|gyro| &lt; GyroThresholdRadS 且 ||acc| − g| &lt; AccThreshold)满 RequiredSeconds → 该传感器完成并锁定,
    /// 之后再动也不影响;只有它自己的 OrientationValid 消失(滤波器重新初始化)才作废、重新计时。
    /// 三个传感器都完成 → 预热完成。Reset()(测量重新开始)全部清零。
    /// 计时用 PacketId 差 / 100 Hz,丢包不影响。StatusWord 为空(回放 CSV)视为有效。
    /// 线程安全:Update 与 GetSnapshot 可在不同线程调用。
    /// </summary>
    public sealed class WarmupMonitor
    {
        // 10 s = lower end of the whitepaper's 10–15 s (2026-11-17, was 15): on the table the outputs are already
        // stable once the filter is valid (tilt < 0.1°, heading ≈ 0°/s, free acc constant over the next 15 s).
        public const double RequiredSeconds = 10.0;
        public const float GyroThresholdRadS = 3f * MathF.PI / 180f; // 3 °/s
        public const float AccThreshold = 0.3f;                      // m/s²,||acc| − g|
        private const float Gravity = 9.81f;
        private const int SampleRateHz = 100;

        private sealed class Sensor
        {
            public long? LastPacketId;
            public long? StillSince; // 当前连续低运动段的起点 PacketId
            public long? DoneAt;     // 该传感器完成预热的 PacketId(锁定)

            public void Clear() { LastPacketId = StillSince = DoneAt = null; }
        }

        private readonly object _lock = new();
        private readonly Sensor _pelvis = new(), _left = new(), _right = new();
        private long? _measurementStart, _completedAt;

        private Sensor? Of(ImuRole r) => r switch
        {
            ImuRole.Pelvis => _pelvis,
            ImuRole.Left => _left,
            ImuRole.Right => _right,
            _ => null,
        };

        /// <summary>测量(重新)开始:滤波器会重新初始化,预热从头算。</summary>
        public void Reset()
        {
            lock (_lock)
            {
                foreach (var s in new[] { _pelvis, _left, _right }) s.Clear();
                _measurementStart = _completedAt = null;
            }
        }

        public void Update(ImuSampleFrame sample)
        {
            var s = Of(sample.Role);
            if (s == null) return;
            lock (_lock)
            {
                bool valid = sample.StatusWord is not uint sw
                             || (sw & (uint)XDA.XsStatusFlag.XSF_OrientationValid) != 0;
                if (!valid)
                {
                    // 这个传感器的滤波器重新初始化:只有它重新预热;整体完成状态随之作废。
                    // 之前有过有效数据才算"重启"(开机初始化不算);每次重启记一条,同时重启的几个各记一条。
                    if (s.LastPacketId != null)
                        _restartNotes.Add($"{sample.Role}: filter re-initialised (StatusWord OrientationValid=0) at packet {sample.PacketId}" +
                                          (_completedAt != null ? ", warm-up voided" : ""));
                    s.Clear();
                    _completedAt = null;
                    return;
                }

                _measurementStart ??= sample.PacketId;
                s.LastPacketId = sample.PacketId;
                if (s.DoneAt != null) return;   // 已锁定,动也不影响

                bool still = sample.HasRateOfTurn && sample.RateOfTurn.Length() < GyroThresholdRadS
                             && sample.HasAcceleration && MathF.Abs(sample.Acceleration.Length() - Gravity) < AccThreshold;
                if (!still) { s.StillSince = null; return; }
                s.StillSince ??= sample.PacketId;
                if ((sample.PacketId - s.StillSince.Value) / (double)SampleRateHz < RequiredSeconds) return;

                s.DoneAt = sample.PacketId;
                if (_pelvis.DoneAt != null && _left.DoneAt != null && _right.DoneAt != null)
                    _completedAt = sample.PacketId;
            }
        }

        public WarmupSnapshot GetSnapshot()
        {
            lock (_lock)
            {
                double?[] secs = { SensorSeconds(_pelvis), SensorSeconds(_left), SensorSeconds(_right) };
                double still = secs.Min() is double m && secs.All(x => x != null) ? m : 0;
                return new WarmupSnapshot(_completedAt != null, still, _measurementStart, _completedAt, secs[0], secs[1], secs[2]);
            }
        }

        public bool IsComplete { get { lock (_lock) return _completedAt != null; } }

        private readonly System.Collections.Generic.List<string> _restartNotes = new();

        /// <summary>取走自上次调用以来的滤波器重启记录(每个重启的传感器一条),供界面写日志。</summary>
        public System.Collections.Generic.List<string> TakeRestartNotes()
        {
            lock (_lock)
            {
                var notes = new System.Collections.Generic.List<string>(_restartNotes);
                _restartNotes.Clear();
                return notes;
            }
        }

        private static double? SensorSeconds(Sensor s)
            => s.LastPacketId is not long last ? null
             : s.DoneAt != null ? RequiredSeconds
             : s.StillSince is long since ? (last - since) / (double)SampleRateHz : 0;
    }
}
