// IMUMoCap/Pipeline/CalibrationProcessor.cs
using System;
using System.Collections.Generic;
using System.Numerics;
using IMUMoCap.Pipeline.Models;
using IMUMoCap;

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
    /// 校准流程：操作员点击触发 → 采集静立数据 → 求解 CalibrationProfile。
    ///
    /// 静立检测：连续 StaticRequiredFrames 帧内三个 IMU 角速度均 < StaticGyroThreshold。
    /// 采集 StaticCollectFrames 帧后，对每个 IMU 取四元数的分量中位数作为参考姿态。
    /// </summary>
    public sealed class CalibrationProcessor
    {
        // ── 可调参数 ──────────────────────────────────────────────────────────
        public float StaticGyroThreshold     { get; set; } = 0.3f; // rad/s
        public int   StaticRequiredFrames    { get; set; } = 30;   // 0.3s 连续静止才开始采集
        public int   StaticCollectFrames     { get; set; } = 300; // 采集 3s 数据
        public int   StaticTimeoutFrames     { get; set; } = 1000; // 10s 超时 → Failed

        // ── 状态 ──────────────────────────────────────────────────────────────
        public CalibrationState State { get; private set; } = CalibrationState.WaitingForStart;
        public CalibrationProfile? Profile { get; private set; }

        private int _staticConsecutive    = 0;
        private int _staticTimeoutCounter = 0;
        private readonly List<Quaternion> _pelvisBuf    = new();
        private readonly List<Quaternion> _leftBuf      = new();
        private readonly List<Quaternion> _rightBuf     = new();

        /// <summary>操作员触发校准：WaitingForStart → CollectingStaticPose。已在别的状态则忽略。</summary>
        public void TriggerStart()
        {
            if (State != CalibrationState.WaitingForStart) return;
            State = CalibrationState.CollectingStaticPose;
        }

        /// <summary>
        /// 输入一帧 ValidFrame，推进校准状态机。
        /// 返回 true 表示本帧触发了状态变化（供调用方记录日志）。
        /// </summary>
        public bool Process(ValidFrame frame)
        {
            return State switch
            {
                CalibrationState.CollectingStaticPose => ProcessCollecting(frame),
                _ => false
            };
        }

        public void Reset()
        {
            State            = CalibrationState.WaitingForStart;
            Profile          = null;
            _staticConsecutive    = 0;
            _staticTimeoutCounter = 0;
            _pelvisBuf.Clear(); _leftBuf.Clear(); _rightBuf.Clear();
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
