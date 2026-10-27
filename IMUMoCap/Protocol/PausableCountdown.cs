// IMUMoCap/Protocol/PausableCountdown.cs
using System;
using System.Diagnostics;

namespace IMUMoCap.Protocol
{
    /// <summary>
    /// 一个可暂停的倒计时:只统计"运行中"的时间,暂停期间不走。
    /// 用来驱动基线超时(flow-review §2-19)——MainWindow 的 DispatcherTimer 只负责到点触发,
    /// 剩余多久由这里算,暂停时停表、恢复时按剩余时间重新起表。时钟可注入以便测试。
    /// </summary>
    public sealed class PausableCountdown
    {
        private readonly Func<TimeSpan> _now;
        private TimeSpan _total;
        private TimeSpan _elapsedBeforeThisRun;
        private TimeSpan? _runningSince;
        private bool _started;

        public PausableCountdown() : this(() => Stopwatch.GetElapsedTime(0)) { }
        public PausableCountdown(Func<TimeSpan> now) => _now = now;

        public bool IsRunning => _runningSince != null;
        public bool IsPaused => _started && _runningSince == null;

        public TimeSpan Remaining
        {
            get
            {
                if (!_started) return TimeSpan.Zero;
                var elapsed = _elapsedBeforeThisRun + (_runningSince is { } t ? _now() - t : TimeSpan.Zero);
                var left = _total - elapsed;
                return left < TimeSpan.Zero ? TimeSpan.Zero : left;
            }
        }

        /// <summary>(重新)开始,从完整时长起算。</summary>
        public void Start(TimeSpan total)
        {
            _total = total;
            _elapsedBeforeThisRun = TimeSpan.Zero;
            _runningSince = _now();
            _started = true;
        }

        public void Pause()
        {
            if (_runningSince is not { } t) return;
            _elapsedBeforeThisRun += _now() - t;
            _runningSince = null;
        }

        public void Resume()
        {
            if (!IsPaused) return;
            _runningSince = _now();
        }

        public void Stop()
        {
            _started = false;
            _runningSince = null;
            _elapsedBeforeThisRun = TimeSpan.Zero;
        }
    }
}
