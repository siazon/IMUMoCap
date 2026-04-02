using System;
using System.Diagnostics;
using GaitTraining.Imu;

namespace IMUMoCap.Methods
{
    /// <summary>
    /// 检测左脚原地跺脚事件。
    /// 
    /// 工作流程：
    /// 1. 前 500ms 采集加速度基线（静止状态）
    /// 2. 经过 500ms 检查是否发生跺脚事件
    /// 3. 跺脚由两个条件触发：竖直加速度峰值 > 15 m/s² 且 加速度模长 > 基线×2
    /// </summary>
    public sealed class FootstepImpactDetector
    {
        // ── 配置 ─────────────────────────────────────────────────
        private readonly float _baselineCollectionDurationMs;
        private readonly float _waitBeforeDetectDurationMs;
        private readonly float _accelPeakThreshold;      // m/s²
        private readonly float _accelRatioThreshold;     // 倍数
        /// <summary>
        /// IMU 输入频率，用于计算 ms 对应的样本数。
        /// Xsens 默认 100Hz。
        /// </summary>
        private readonly int _imuSamplingRateHz;

        // ── 运行状态 ─────────────────────────────────────────────
        private long _accumulatedFrameCount;
        private float _baselineAccMag;
        private bool _baselineValid;
        private float _peakAccMag;
        private bool _stepDetected;
        private Stopwatch _timer = null!;

        // ── 诊断 ─────────────────────────────────────────────────
        public struct Diagnostics
        {
            public long FrameCount;
            public float BaselineAccMag;
            public bool BaselineValid;
            public float PeakAccMag;
            public bool StepDetected;
            public double ElapsedMs;
            public override string ToString() =>
                $"Frame={FrameCount} BaselineValid={BaselineValid} BaselineMag={BaselineAccMag:F2} " +
                $"PeakMag={PeakAccMag:F2} StepDetected={StepDetected} ElapsedMs={ElapsedMs:F0}";
        }

        public FootstepImpactDetector(
            float baselineCollectionDurationMs = 500f,
            float waitBeforeDetectDurationMs = 500f,
            float accelPeakThresholdMs2 = 15f,
            float accelRatioThreshold = 2.0f,
            int imuSamplingRateHz = 100)
        {
            _baselineCollectionDurationMs = baselineCollectionDurationMs;
            _waitBeforeDetectDurationMs = waitBeforeDetectDurationMs;
            _accelPeakThreshold = accelPeakThresholdMs2;
            _accelRatioThreshold = accelRatioThreshold;
            _imuSamplingRateHz = Math.Max(1, imuSamplingRateHz);

            Reset();
        }

        public void Reset()
        {
            _accumulatedFrameCount = 0;
            _baselineAccMag = 0f;
            _baselineValid = false;
            _peakAccMag = 0f;
            _stepDetected = false;
            _timer = Stopwatch.StartNew();
        }

        /// <summary>
        /// 逐帧处理左脚 IMU 数据。
        /// 返回 true 表示检测到跺脚事件（仅一次返回 true）。
        /// 后续帧会继续返回 false，直到 Reset() 重置。
        /// </summary>
        public bool Update(ImuSample leftFootSample)
        {
            // 已经检测到跺脚，之后始终返回 false
            if (_stepDetected)
                return false;

            double elapsedMs = _timer.Elapsed.TotalMilliseconds;
            _accumulatedFrameCount++;

            float accMag = leftFootSample.AccMag;
            _peakAccMag = Math.Max(_peakAccMag, accMag);

            // ── 阶段 1：采集基线（前 500ms）────────────────────────
            if (elapsedMs < _baselineCollectionDurationMs)
            {
                // 简单方案：用第一帧作为基线（假设初期静止）
                // 也可以改成均值：_baselineAccMag = moving average of accMag
                if (_accumulatedFrameCount == 1)
                {
                    _baselineAccMag = accMag;
                }
                else
                {
                    // 取前 500ms 的最小值作为"静止基线"
                    _baselineAccMag = Math.Min(_baselineAccMag, accMag);
                }
                return false;
            }

            // ── 阶段 1 结束：验证基线 ──────────────────────────────
            if (!_baselineValid && elapsedMs >= _baselineCollectionDurationMs)
            {
                _baselineValid = true;
                // 基线应该接近重力加速度（~9.8 m/s²）
                // 如果基线太小或太大，可能是异常
                Debug.WriteLine($"[FootstepImpactDetector] Baseline established: {_baselineAccMag:F2} m/s²");
            }

            // ── 阶段 2：检测跺脚（500ms 之后）──────────────────────
            if (elapsedMs >= _waitBeforeDetectDurationMs && _baselineValid)
            {
                // 双条件判定：
                // 1. 加速度模长超过阈值（绝对值）
                // 2. 加速度模长超过基线的倍数
                bool peakExceeds = accMag > _accelPeakThreshold;
                bool ratioExceeds = accMag > _baselineAccMag * _accelRatioThreshold;

                if (peakExceeds && ratioExceeds)
                {
                    _stepDetected = true;
                    Debug.WriteLine(
                        $"[FootstepImpactDetector] STEP DETECTED at {elapsedMs:F0}ms. " +
                        $"AccMag={accMag:F2} m/s² (peak={_peakAccMag:F2}, threshold={_accelPeakThreshold}, " +
                        $"ratio={accMag / _baselineAccMag:F2}x baseline)");
                    return true;
                }
            }

            return false;
        }

        public Diagnostics GetDiagnostics() => new()
        {
            FrameCount = _accumulatedFrameCount,
            BaselineAccMag = _baselineAccMag,
            BaselineValid = _baselineValid,
            PeakAccMag = _peakAccMag,
            StepDetected = _stepDetected,
            ElapsedMs = _timer.Elapsed.TotalMilliseconds
        };
    }
}
