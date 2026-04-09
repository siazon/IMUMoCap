// IMUMoCap/Pipeline/CalibrationProcessor.cs
using System;
using System.Collections.Generic;
using System.Numerics;
using IMUMoCap.Pipeline.Models;

namespace IMUMoCap.Pipeline
{
    public enum CalibrationState
    {
        WaitingForStart,
        CollectingStaticPose,
        Completed,
        Failed
    }

    /// <summary>
    /// 校准流程：等待左脚 stomp → 采集静立数据 → 求解 CalibrationProfile。
    ///
    /// Stomp 检测：左脚垂直加速度峰值 > StompAccThreshold（约 3g），
    /// 持续时间 < StompMaxFrames，之后恢复平静。
    /// 无需 CalibrationProfile 即可检测（与安装误差无关）。
    ///
    /// 静立检测：连续 StaticRequiredFrames 帧内三个 IMU 角速度均 < StaticGyroThreshold。
    /// 采集 StaticCollectFrames 帧后，对每个 IMU 取四元数的分量中位数作为参考姿态。
    /// </summary>
    public sealed class CalibrationProcessor
    {
        // ── 可调参数 ──────────────────────────────────────────────────────────
        public float StompAccThreshold_ms2   { get; set; } = 25f;  // ~2.5g
        public int   StompMaxFrames          { get; set; } = 20;   // 0.2s @100Hz
        public float StaticGyroThreshold     { get; set; } = 0.3f; // rad/s
        public int   StaticRequiredFrames    { get; set; } = 30;   // 0.3s 连续静止才开始采集
        public int   StaticCollectFrames     { get; set; } = 300;  // 采集 3s 数据
        public int   StaticTimeoutFrames     { get; set; } = 1000; // 10s 超时 → Failed

        // ── 状态 ──────────────────────────────────────────────────────────────
        public CalibrationState State { get; private set; } = CalibrationState.WaitingForStart;
        public CalibrationProfile? Profile { get; private set; }

        private int _stompFrameCount      = 0;
        private bool _stompPeakSeen       = false;
        private int _staticConsecutive    = 0;
        private int _staticTimeoutCounter = 0;
        private readonly List<Quaternion> _pelvisBuf    = new();
        private readonly List<Quaternion> _leftBuf      = new();
        private readonly List<Quaternion> _rightBuf     = new();

        /// <summary>
        /// 输入一帧 ValidFrame，推进校准状态机。
        /// 返回 true 表示本帧触发了状态变化（供调用方记录日志）。
        /// </summary>
        public bool Process(ValidFrame frame)
        {
            return State switch
            {
                CalibrationState.WaitingForStart      => ProcessWaiting(frame),
                CalibrationState.CollectingStaticPose => ProcessCollecting(frame),
                _ => false
            };
        }

        public void Reset()
        {
            State            = CalibrationState.WaitingForStart;
            Profile          = null;
            _stompFrameCount = 0;
            _stompPeakSeen   = false;
            _staticConsecutive    = 0;
            _staticTimeoutCounter = 0;
            _pelvisBuf.Clear(); _leftBuf.Clear(); _rightBuf.Clear();
        }

        // ── WaitingForStart ───────────────────────────────────────────────────

        private bool ProcessWaiting(ValidFrame frame)
        {
            float leftVertAcc = GetVerticalAcceleration(frame.LeftFoot);

            if (!_stompPeakSeen)
            {
                if (leftVertAcc > StompAccThreshold_ms2)
                {
                    _stompPeakSeen   = true;
                    _stompFrameCount = 1;
                }
            }
            else
            {
                _stompFrameCount++;
                // 峰值结束后（加速度回落），确认为 stomp
                if (leftVertAcc < StompAccThreshold_ms2 * 0.4f)
                {
                    if (_stompFrameCount <= StompMaxFrames)
                    {
                        State = CalibrationState.CollectingStaticPose;
                        _stompPeakSeen   = false;
                        _stompFrameCount = 0;
                        return true;  // 状态变化
                    }
                    // 持续太久不像 stomp，重置
                    _stompPeakSeen   = false;
                    _stompFrameCount = 0;
                }
                else if (_stompFrameCount > StompMaxFrames)
                {
                    _stompPeakSeen   = false;
                    _stompFrameCount = 0;
                }
            }
            return false;
        }

        // ── CollectingStaticPose ──────────────────────────────────────────────

        private bool ProcessCollecting(ValidFrame frame)
        {
            _staticTimeoutCounter++;
            if (_staticTimeoutCounter > StaticTimeoutFrames)
            {
                State = CalibrationState.Failed;
                return true;
            }

            bool isStatic = IsStatic(frame);
            if (isStatic)
            {
                _staticConsecutive++;
                if (_staticConsecutive >= StaticRequiredFrames)
                {
                    // 开始/继续收集
                    if (frame.Pelvis.HasQuaternion)
                    {
                        _pelvisBuf.Add(frame.Pelvis.Quaternion);
                        _leftBuf.Add(frame.LeftFoot.Quaternion);
                        _rightBuf.Add(frame.RightFoot.Quaternion);
                    }

                    if (_pelvisBuf.Count >= StaticCollectFrames)
                    {
                        Profile = new CalibrationProfile
                        {
                            PelvisRef    = MedianQuaternion(_pelvisBuf),
                            LeftFootRef  = MedianQuaternion(_leftBuf),
                            RightFootRef = MedianQuaternion(_rightBuf),
                        };
                        State = CalibrationState.Completed;
                        return true;
                    }
                }
            }
            else
            {
                _staticConsecutive = 0;
                // 抖动时暂停采集但不清空缓冲区（允许短时抖动）
            }
            return false;
        }

        // ── 工具方法 ──────────────────────────────────────────────────────────

        private bool IsStatic(ValidFrame f)
        {
            float gyroThSq = StaticGyroThreshold * StaticGyroThreshold;
            return f.Pelvis.RateOfTurn.LengthSquared()    < gyroThSq
                && f.LeftFoot.RateOfTurn.LengthSquared()  < gyroThSq
                && f.RightFoot.RateOfTurn.LengthSquared() < gyroThSq;
        }

        /// <summary>垂直加速度 = 自由加速度的 Y 分量绝对值（Xsens 世界系 Y 轴朝上）</summary>
        private static float GetVerticalAcceleration(ImuSampleFrame f)
        {
            if (f.HasFreeAcceleration)
                return MathF.Abs(f.FreeAcceleration.Y);
            if (f.HasAcceleration)
                return MathF.Abs(f.Acceleration.Y - 9.81f);
            return 0f;
        }

        /// <summary>
        /// 对四元数列表取分量中位数，再归一化。
        /// 比均值四元数对离群值更稳健。
        /// </summary>
        private static Quaternion MedianQuaternion(List<Quaternion> qs)
        {
            if (qs.Count == 0) return Quaternion.Identity;
            var ws = new float[qs.Count];
            var xs = new float[qs.Count];
            var ys = new float[qs.Count];
            var zs = new float[qs.Count];
            for (int i = 0; i < qs.Count; i++)
            {
                // 确保同半球（避免符号翻转）
                var q = qs[i].W < 0 ? Quaternion.Negate(qs[i]) : qs[i];
                ws[i] = q.W; xs[i] = q.X; ys[i] = q.Y; zs[i] = q.Z;
            }
            Array.Sort(ws); Array.Sort(xs); Array.Sort(ys); Array.Sort(zs);
            int mid = qs.Count / 2;
            return Quaternion.Normalize(new Quaternion(xs[mid], ys[mid], zs[mid], ws[mid]));
        }
    }
}
