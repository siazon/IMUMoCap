// IMUMoCap.Tests/Unit/PausableCountdownTests.cs
using System;
using IMUMoCap.Protocol;
using Xunit;

namespace IMUMoCap.Tests.Unit
{
    /// <summary>
    /// 基线 10 分钟超时在暂停期间必须停表(docs/flow-review-2026-10-27.md §2-19):
    /// 暂停时数据包被丢弃、步数不涨,如果计时器照走,暂停多久就少多久的可用行走时间。
    /// </summary>
    public class PausableCountdownTests
    {
        private readonly TimeSpan _ten = TimeSpan.FromMinutes(10);
        private TimeSpan _now = TimeSpan.Zero;
        private PausableCountdown Make() => new(() => _now);
        private void Advance(double minutes) => _now += TimeSpan.FromMinutes(minutes);

        [Fact]
        public void Remaining_CountsDown_WhileRunning()
        {
            var c = Make();
            c.Start(_ten);
            Advance(4);

            Assert.True(c.IsRunning);
            Assert.Equal(TimeSpan.FromMinutes(6), c.Remaining);
        }

        [Fact]
        public void Pause_FreezesTheRemainingTime()
        {
            var c = Make();
            c.Start(_ten);
            Advance(4);
            c.Pause();
            Advance(100);                       // a very long pause

            Assert.True(c.IsPaused);
            Assert.False(c.IsRunning);
            Assert.Equal(TimeSpan.FromMinutes(6), c.Remaining);
        }

        [Fact]
        public void Resume_ContinuesFromWhereItStopped()
        {
            var c = Make();
            c.Start(_ten);
            Advance(4);
            c.Pause();
            Advance(100);
            c.Resume();
            Advance(5);

            Assert.True(c.IsRunning);
            Assert.Equal(TimeSpan.FromMinutes(1), c.Remaining);
            Advance(1);
            Assert.Equal(TimeSpan.Zero, c.Remaining);
        }

        [Fact]
        public void SeveralPauses_AccumulateOnlyTheRunningTime()
        {
            var c = Make();
            c.Start(_ten);
            for (int i = 0; i < 3; i++)
            {
                Advance(2);
                c.Pause();
                Advance(30);
                c.Resume();
            }

            Assert.Equal(TimeSpan.FromMinutes(4), c.Remaining);
        }

        [Fact]
        public void Remaining_NeverGoesNegative()
        {
            var c = Make();
            c.Start(_ten);
            Advance(25);

            Assert.Equal(TimeSpan.Zero, c.Remaining);
        }

        [Fact]
        public void PauseAndResume_AreIdempotent_AndIgnoredWhenNotApplicable()
        {
            var c = Make();
            c.Resume();                          // never started
            c.Pause();
            Assert.False(c.IsPaused);

            c.Start(_ten);
            Advance(1);
            c.Resume();                          // running, not paused → no effect
            Advance(1);
            c.Pause();
            c.Pause();                           // second pause must not move the freeze point
            Advance(10);
            c.Resume();
            c.Resume();

            Assert.Equal(TimeSpan.FromMinutes(8), c.Remaining);
        }

        [Fact]
        public void Stop_EndsIt_AndAPausedCountdownCannotBeResumedAfterwards()
        {
            var c = Make();
            c.Start(_ten);
            Advance(1);
            c.Pause();
            c.Stop();
            c.Resume();

            Assert.False(c.IsRunning);
            Assert.False(c.IsPaused);
        }

        [Fact]
        public void Start_AgainRestartsFromTheFullDuration()
        {
            var c = Make();
            c.Start(_ten);
            Advance(7);
            c.Pause();

            c.Start(_ten);

            Assert.True(c.IsRunning);
            Assert.False(c.IsPaused);
            Assert.Equal(_ten, c.Remaining);
        }
    }
}
