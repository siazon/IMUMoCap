# Gait Pipeline — Plan 2: Analysis Modules

> **For agentic workers:** Execute inline in this session (executing-plans). Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 实现 CalibrationProcessor、GaitEventDetector、MotionContextDetector、ProgressionDirEstimator 四个分析模块。

**Architecture:** 每个模块一个操作类，无状态工具方法抽取为同文件私有静态方法。所有模块只依赖 Plan 1 定义的模型类型，不相互依赖（GaitPipeline 在 Plan 3 负责编排调用顺序）。

**Tech Stack:** C# 12 / .NET 8，System.Numerics，无新 NuGet 包。

**前置条件：** Plan 1 已完成，`IMUMoCap/Pipeline/Models/` 下的模型类型已存在。

---

## 文件结构

| 操作 | 路径 | 职责 |
|------|------|------|
| Create | `IMUMoCap/Pipeline/CalibrationProcessor.cs` | stomp 触发 + 静立采集 + CalibrationProfile 求解 |
| Create | `IMUMoCap/Pipeline/GaitEventDetector.cs` | stance/swing/stomp/isWalking 判定 |
| Create | `IMUMoCap/Pipeline/MotionContextDetector.cs` | Straight/Turning/ReacquiringPd + 置信度 |
| Create | `IMUMoCap/Pipeline/ProgressionDirEstimator.cs` | PD 在线估计 + 有效性 + 稳定性 |

---

## Task 1: CalibrationProcessor

**Files:**
- Create: `IMUMoCap/Pipeline/CalibrationProcessor.cs`

- [ ] **Step 1: 创建文件**

```csharp
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
                CalibrationState.WaitingForStart     => ProcessWaiting(frame),
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
                        _stompPeakSeen = false;
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
            return f.Pelvis.RateOfTurn.LengthSquared()   < gyroThSq
                && f.LeftFoot.RateOfTurn.LengthSquared() < gyroThSq
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
```

- [ ] **Step 2: Build 验证**

```
dotnet build IMUMoCap/IMUMoCap.csproj
```
期望：0 errors。

- [ ] **Step 3: Commit**

```bash
git add IMUMoCap/Pipeline/CalibrationProcessor.cs
git commit -m "feat: add CalibrationProcessor — stomp trigger, static pose collection, profile solving"
```

---

## Task 2: GaitEventDetector

**Files:**
- Create: `IMUMoCap/Pipeline/GaitEventDetector.cs`

- [ ] **Step 1: 创建文件**

```csharp
// IMUMoCap/Pipeline/GaitEventDetector.cs
using System.Numerics;
using IMUMoCap.Pipeline.Models;

namespace IMUMoCap.Pipeline
{
    /// <summary>
    /// 逐帧判定步态事件：stance/swing、stomp、isWalking。
    ///
    /// Stance 判定：足部垂直加速度 ≈ 1g（free acc ≈ 0）AND 角速度 < GyroThreshold，
    /// 且持续 MinStanceFrames 帧以上防抖。
    ///
    /// IsWalking：左右脚在 WalkingWindowFrames 内都至少有一次 stance 切换。
    ///
    /// StompDetected（校准后版本）：左脚垂直加速度峰值 > StompAccThreshold，
    /// 持续 < StompMaxFrames，此版本比校准前更精确。
    /// </summary>
    public sealed class GaitEventDetector
    {
        // ── 可调参数 ──────────────────────────────────────────────────────────
        public float FreeAccStanceThreshold { get; set; } = 2.5f;   // m/s²，free acc 模长 < 此值 = stance
        public float GyroThreshold          { get; set; } = 1.0f;   // rad/s
        public int   MinStanceFrames        { get; set; } = 5;
        public float StompAccThreshold_ms2  { get; set; } = 25f;
        public int   StompMaxFrames         { get; set; } = 20;
        public int   WalkingWindowFrames    { get; set; } = 300;     // 3s @100Hz

        // ── 内部状态 ──────────────────────────────────────────────────────────
        private int  _leftStanceCount   = 0;
        private int  _rightStanceCount  = 0;
        private bool _leftStancePrev    = false;
        private bool _rightStancePrev   = false;
        private int  _leftTransitions   = 0;   // stance↔swing 切换次数（滑动窗内）
        private int  _rightTransitions  = 0;
        private int  _walkingFrameCount = 0;

        private bool _stompPeakSeen    = false;
        private int  _stompFrameCount  = 0;

        public GaitEvent Detect(ValidFrame frame, CalibrationProfile _)
        {
            // CalibrationProfile 预留参数，目前 stance 检测在世界系下直接用 free acc，
            // 不需要安装偏差修正（世界系重力方向固定）

            bool leftStance  = IsStance(frame.LeftFoot);
            bool rightStance = IsStance(frame.RightFoot);

            // 防抖：需连续 MinStanceFrames 帧才确认
            _leftStanceCount  = leftStance  ? _leftStanceCount  + 1 : 0;
            _rightStanceCount = rightStance ? _rightStanceCount + 1 : 0;

            bool leftStanceConfirmed  = _leftStanceCount  >= MinStanceFrames;
            bool rightStanceConfirmed = _rightStanceCount >= MinStanceFrames;

            // 切换计数（用于 IsWalking 判定）
            if (leftStanceConfirmed  != _leftStancePrev)  { _leftTransitions++;  _leftStancePrev  = leftStanceConfirmed; }
            if (rightStanceConfirmed != _rightStancePrev) { _rightTransitions++; _rightStancePrev = rightStanceConfirmed; }

            _walkingFrameCount++;
            bool isWalking = false;
            if (_walkingFrameCount >= WalkingWindowFrames)
            {
                // 3s 窗口内左右各至少有 2 次切换（一个完整 stance-swing 周期）
                isWalking = _leftTransitions >= 2 && _rightTransitions >= 2;
                _walkingFrameCount = 0;
                _leftTransitions   = 0;
                _rightTransitions  = 0;
            }

            bool stomp = DetectStomp(frame.LeftFoot);

            return new GaitEvent
            {
                LeftStance    = leftStanceConfirmed,
                RightStance   = rightStanceConfirmed,
                LeftSwing     = !leftStanceConfirmed,
                RightSwing    = !rightStanceConfirmed,
                StompDetected = stomp,
                IsWalking     = isWalking,
            };
        }

        public void Reset()
        {
            _leftStanceCount = _rightStanceCount = 0;
            _leftStancePrev  = _rightStancePrev  = false;
            _leftTransitions = _rightTransitions = 0;
            _walkingFrameCount = 0;
            _stompPeakSeen   = false;
            _stompFrameCount = 0;
        }

        // ── 私有辅助 ──────────────────────────────────────────────────────────

        private bool IsStance(ImuSampleFrame f)
        {
            if (!f.HasFreeAcceleration || !f.HasRateOfTurn) return false;
            bool accOk  = f.FreeAcceleration.Length() < FreeAccStanceThreshold;
            bool gyroOk = f.RateOfTurn.Length()       < GyroThreshold;
            return accOk && gyroOk;
        }

        private bool DetectStomp(ImuSampleFrame leftFoot)
        {
            float vertAcc = leftFoot.HasFreeAcceleration
                ? MathF.Abs(leftFoot.FreeAcceleration.Y)
                : 0f;

            if (!_stompPeakSeen)
            {
                if (vertAcc > StompAccThreshold_ms2) { _stompPeakSeen = true; _stompFrameCount = 1; }
                return false;
            }

            _stompFrameCount++;
            if (vertAcc < StompAccThreshold_ms2 * 0.4f)
            {
                bool valid = _stompFrameCount <= StompMaxFrames;
                _stompPeakSeen  = false;
                _stompFrameCount = 0;
                return valid;
            }
            if (_stompFrameCount > StompMaxFrames) { _stompPeakSeen = false; _stompFrameCount = 0; }
            return false;
        }
    }
}
```

- [ ] **Step 2: Build 验证**

```
dotnet build IMUMoCap/IMUMoCap.csproj
```
期望：0 errors。

- [ ] **Step 3: Commit**

```bash
git add IMUMoCap/Pipeline/GaitEventDetector.cs
git commit -m "feat: add GaitEventDetector — stance/swing/stomp/isWalking detection"
```

---

## Task 3: MotionContextDetector

**Files:**
- Create: `IMUMoCap/Pipeline/MotionContextDetector.cs`

- [ ] **Step 1: 创建文件**

```csharp
// IMUMoCap/Pipeline/MotionContextDetector.cs
using System;
using System.Collections.Generic;
using IMUMoCap.Pipeline.Models;

namespace IMUMoCap.Pipeline
{
    /// <summary>
    /// 识别运动上下文：Straight / Turning / ReacquiringPd。
    ///
    /// 四路信号加权投票，持续 TurningConfirmFrames 帧满足才切换到 Turning：
    /// 1. Pelvis yaw rate 绝对值 > YawRateThreshold（主信号）
    /// 2. 短窗内 pelvis heading 累积变化 > HeadingDeltaThreshold
    /// 3. DeltaQ 累积 yaw 增量 > DeltaQYawThreshold
    /// 4. （辅助）IsWalking 用于确认仍在步行
    ///
    /// 转弯结束后进入 ReacquiringPd，等 ProgressionDirEstimator 调用 ConfirmStraight() 退出。
    /// </summary>
    public sealed class MotionContextDetector
    {
        // ── 可调参数 ──────────────────────────────────────────────────────────
        public float YawRateThreshold_rads    { get; set; } = 0.26f; // ~15°/s
        public float HeadingDeltaThreshold    { get; set; } = 0.17f; // ~10° 累积
        public float DeltaQYawThreshold       { get; set; } = 0.17f; // rad
        public int   TurningConfirmFrames     { get; set; } = 10;    // 持续帧数才判 Turning
        public int   StraightConfirmFrames    { get; set; } = 20;    // 持续帧数才退出 Turning
        public int   HeadingWindowFrames      { get; set; } = 20;    // 短窗大小

        // ── 内部状态 ──────────────────────────────────────────────────────────
        private ContextState       _state         = ContextState.Straight;
        private int                _turningFrames = 0;
        private int                _straightFrames = 0;
        private float              _deltaQYawAccum = 0f;
        private readonly Queue<float> _headingWindow = new();
        private float              _headingWindowSum = 0f;
        private float              _lastPelvisYaw = float.NaN;

        public MotionContext Detect(ValidFrame frame, GaitEvent gait)
        {
            float pelvisYawRate = frame.Pelvis.HasRateOfTurn
                ? MathF.Abs(frame.Pelvis.RateOfTurn.Y)   // Y = vertical axis in world frame
                : 0f;

            // 累积 deltaQ yaw
            if (frame.Pelvis.HasDeltaQ)
                _deltaQYawAccum += ExtractYawFromDeltaQ(frame.Pelvis.DeltaQ);

            // 短窗 heading 变化
            float pelvisYaw = frame.Pelvis.HasQuaternion
                ? ExtractYaw(frame.Pelvis.Quaternion)
                : float.NaN;

            float headingDelta = 0f;
            if (!float.IsNaN(pelvisYaw) && !float.IsNaN(_lastPelvisYaw))
            {
                float delta = NormalizeAngle(pelvisYaw - _lastPelvisYaw);
                _headingWindow.Enqueue(delta);
                _headingWindowSum += MathF.Abs(delta);
                if (_headingWindow.Count > HeadingWindowFrames)
                    _headingWindowSum -= MathF.Abs(_headingWindow.Dequeue());
                headingDelta = _headingWindowSum;
            }
            _lastPelvisYaw = pelvisYaw;

            bool turningSignal =
                pelvisYawRate > YawRateThreshold_rads ||
                headingDelta  > HeadingDeltaThreshold ||
                MathF.Abs(_deltaQYawAccum) > DeltaQYawThreshold;

            return _state switch
            {
                ContextState.Straight        => HandleStraight(turningSignal),
                ContextState.Turning         => HandleTurning(turningSignal),
                ContextState.ReacquiringPd   => HandleReacquiring(turningSignal),
                _                            => new MotionContext { State = _state, Confidence = 1f }
            };
        }

        /// <summary>由 ProgressionDirEstimator 调用，PD 稳定后退出 ReacquiringPd。</summary>
        public void ConfirmStraight()
        {
            if (_state == ContextState.ReacquiringPd)
                _state = ContextState.Straight;
        }

        public void Reset()
        {
            _state           = ContextState.Straight;
            _turningFrames   = 0;
            _straightFrames  = 0;
            _deltaQYawAccum  = 0f;
            _headingWindow.Clear();
            _headingWindowSum = 0f;
            _lastPelvisYaw   = float.NaN;
        }

        // ── 状态处理 ──────────────────────────────────────────────────────────

        private MotionContext HandleStraight(bool turningSignal)
        {
            if (turningSignal)
            {
                _turningFrames++;
                if (_turningFrames >= TurningConfirmFrames)
                {
                    _state = ContextState.Turning;
                    _turningFrames  = 0;
                    _straightFrames = 0;
                    _deltaQYawAccum = 0f;
                    return new MotionContext { State = ContextState.Turning, Confidence = 1f };
                }
                float conf = 1f - (float)_turningFrames / TurningConfirmFrames;
                return new MotionContext { State = ContextState.Straight, Confidence = conf };
            }
            _turningFrames = 0;
            return new MotionContext { State = ContextState.Straight, Confidence = 1f };
        }

        private MotionContext HandleTurning(bool turningSignal)
        {
            if (!turningSignal)
            {
                _straightFrames++;
                if (_straightFrames >= StraightConfirmFrames)
                {
                    _state = ContextState.ReacquiringPd;
                    _straightFrames = 0;
                    _deltaQYawAccum = 0f;
                    return new MotionContext { State = ContextState.ReacquiringPd, Confidence = 0f };
                }
            }
            else
            {
                _straightFrames = 0;
            }
            return new MotionContext { State = ContextState.Turning, Confidence = 1f };
        }

        private MotionContext HandleReacquiring(bool turningSignal)
        {
            if (turningSignal)
            {
                // 再次转弯
                _state = ContextState.Turning;
                _straightFrames = 0;
            }
            return new MotionContext { State = _state, Confidence = 0f };
        }

        // ── 工具方法 ──────────────────────────────────────────────────────────

        private static float ExtractYaw(System.Numerics.Quaternion q)
            => MathF.Atan2(2f * (q.W * q.Z + q.X * q.Y),
                           1f - 2f * (q.Y * q.Y + q.Z * q.Z));

        private static float ExtractYawFromDeltaQ(System.Numerics.Quaternion dq)
            => 2f * MathF.Atan2(dq.Z, dq.W);  // 近似：小角度 deltaQ 的 yaw 增量

        private static float NormalizeAngle(float rad)
        {
            while (rad >  MathF.PI) rad -= 2f * MathF.PI;
            while (rad < -MathF.PI) rad += 2f * MathF.PI;
            return rad;
        }
    }
}
```

- [ ] **Step 2: Build 验证**

```
dotnet build IMUMoCap/IMUMoCap.csproj
```
期望：0 errors。

- [ ] **Step 3: Commit**

```bash
git add IMUMoCap/Pipeline/MotionContextDetector.cs
git commit -m "feat: add MotionContextDetector — Straight/Turning/ReacquiringPd with multi-signal fusion"
```

---

## Task 4: ProgressionDirEstimator

**Files:**
- Create: `IMUMoCap/Pipeline/ProgressionDirEstimator.cs`

- [ ] **Step 1: 创建文件**

```csharp
// IMUMoCap/Pipeline/ProgressionDirEstimator.cs
using System;
using System.Collections.Generic;
using IMUMoCap.Pipeline.Models;

namespace IMUMoCap.Pipeline
{
    /// <summary>
    /// 在线估计行进方向（Progression Direction, PD）。
    ///
    /// 仅在 MotionContext.State == Straight 时更新。
    /// 融合三路信号（权重可调）：
    ///   - Pelvis heading（yaw）：主信号，稳定性好
    ///   - Left foot heading（stance 相位中点）
    ///   - Right foot heading（stance 相位中点）
    ///
    /// 稳定性（Stability）= 1 / (1 + variance)，滑动窗口内各步估计值的方差。
    ///
    /// ReacquiringPd 退出条件（两者同时满足）：
    ///   1. Stability >= StabilityThreshold
    ///   2. 已积累 MinStepsBeforeValid 步
    /// 满足后调用 motionContext.ConfirmStraight()。
    /// </summary>
    public sealed class ProgressionDirEstimator
    {
        // ── 可调参数 ──────────────────────────────────────────────────────────
        public float PelvisWeight      { get; set; } = 0.6f;
        public float LeftFootWeight    { get; set; } = 0.2f;
        public float RightFootWeight   { get; set; } = 0.2f;
        public int   StabilityWindow   { get; set; } = 10;   // 步数滑动窗口
        public float StabilityThreshold { get; set; } = 0.85f;
        public int   MinStepsBeforeValid { get; set; } = 5;

        // ── 内部状态 ──────────────────────────────────────────────────────────
        private float  _currentPd     = 0f;
        private bool   _hasEstimate   = false;
        private int    _stepCount     = 0;
        private readonly Queue<float> _pdHistory = new();
        private float  _pdHistorySum  = 0f;
        private float  _pdHistorySumSq = 0f;

        private bool _leftInStance    = false;
        private bool _rightInStance   = false;
        private float _leftStanceYaw  = 0f;
        private float _rightStanceYaw = 0f;
        private int   _leftStanceFrames  = 0;
        private int   _rightStanceFrames = 0;

        public PdEstimate Update(ValidFrame frame, GaitEvent gait,
                                 MotionContext context, MotionContextDetector contextDetector)
        {
            // 只在直行时更新
            if (context.State != ContextState.Straight && context.State != ContextState.ReacquiringPd)
                return BuildEstimate();

            float pelvisYaw = ExtractYaw(frame.Pelvis.Quaternion);

            // 追踪 stance 中点 yaw（用左脚 swing→stance 过渡）
            TrackStanceYaw(frame, gait);

            // 每当一个完整的步出现（left swing→stance 或 right swing→stance），更新 PD
            bool stepCompleted = TryConsumeStep(out float leftYaw, out float rightYaw);
            if (!stepCompleted) return BuildEstimate();

            _stepCount++;

            float fused = FuseDirection(pelvisYaw, leftYaw, rightYaw);
            UpdateHistory(fused);
            _currentPd   = fused;
            _hasEstimate = true;

            // ReacquiringPd 退出检查
            if (context.State == ContextState.ReacquiringPd)
            {
                float stability = ComputeStability();
                if (stability >= StabilityThreshold && _stepCount >= MinStepsBeforeValid)
                    contextDetector.ConfirmStraight();
            }

            return BuildEstimate();
        }

        public void Reset()
        {
            _currentPd    = 0f;
            _hasEstimate  = false;
            _stepCount    = 0;
            _pdHistory.Clear();
            _pdHistorySum   = 0f;
            _pdHistorySumSq = 0f;
            _leftInStance = _rightInStance = false;
            _leftStanceFrames = _rightStanceFrames = 0;
        }

        // ── 内部方法 ──────────────────────────────────────────────────────────

        private void TrackStanceYaw(ValidFrame frame, GaitEvent gait)
        {
            // 累积 stance 相位内的 yaw，取均值作为该步的 foot heading
            if (gait.LeftStance)
            {
                _leftInStance = true;
                _leftStanceYaw += ExtractYaw(frame.LeftFoot.Quaternion);
                _leftStanceFrames++;
            }
            else if (_leftInStance)
            {
                // swing 开始，stance 结束，保存均值
                if (_leftStanceFrames > 0)
                    _leftStanceYaw /= _leftStanceFrames;
                _leftInStance    = false;
                _leftStanceFrames = 0;  // 已消费标记，见 TryConsumeStep
                // 注：这里 frames==0 表示"有新的 leftYaw 可用"
                // 实际通过 _leftStanceFrames 的正负来判断，简化实现：用 bool 标记
            }

            if (gait.RightStance)
            {
                _rightInStance = true;
                _rightStanceYaw += ExtractYaw(frame.RightFoot.Quaternion);
                _rightStanceFrames++;
            }
            else if (_rightInStance)
            {
                if (_rightStanceFrames > 0)
                    _rightStanceYaw /= _rightStanceFrames;
                _rightInStance    = false;
                _rightStanceFrames = 0;
            }
        }

        private bool _leftYawReady = false, _rightYawReady = false;
        private float _lastLeftYaw = 0f, _lastRightYaw = 0f;

        // 重写 TrackStanceYaw 使用更清晰的 ready 标志
        private void TrackStanceYawV2(ValidFrame frame, GaitEvent gait)
        {
            if (gait.LeftStance)
            {
                if (!_leftInStance) { _leftInStance = true; _leftStanceYaw = 0f; _leftStanceFrames = 0; }
                _leftStanceYaw += ExtractYaw(frame.LeftFoot.Quaternion);
                _leftStanceFrames++;
            }
            else if (_leftInStance)
            {
                _leftInStance  = false;
                _lastLeftYaw   = _leftStanceFrames > 0 ? _leftStanceYaw / _leftStanceFrames : _lastLeftYaw;
                _leftYawReady  = true;
            }

            if (gait.RightStance)
            {
                if (!_rightInStance) { _rightInStance = true; _rightStanceYaw = 0f; _rightStanceFrames = 0; }
                _rightStanceYaw += ExtractYaw(frame.RightFoot.Quaternion);
                _rightStanceFrames++;
            }
            else if (_rightInStance)
            {
                _rightInStance  = false;
                _lastRightYaw   = _rightStanceFrames > 0 ? _rightStanceYaw / _rightStanceFrames : _lastRightYaw;
                _rightYawReady  = true;
            }
        }

        private bool TryConsumeStep(out float leftYaw, out float rightYaw)
        {
            leftYaw  = _lastLeftYaw;
            rightYaw = _lastRightYaw;
            if (_leftYawReady && _rightYawReady)
            {
                _leftYawReady = _rightYawReady = false;
                return true;
            }
            return false;
        }

        // 实际 Process 用 V2 版本，修正 Task 1 中 TrackStanceYaw 的实现
        public PdEstimate UpdateClean(ValidFrame frame, GaitEvent gait,
                                      MotionContext context, MotionContextDetector contextDetector)
        {
            if (context.State != ContextState.Straight && context.State != ContextState.ReacquiringPd)
                return BuildEstimate();

            float pelvisYaw = frame.Pelvis.HasQuaternion ? ExtractYaw(frame.Pelvis.Quaternion) : _currentPd;

            TrackStanceYawV2(frame, gait);

            if (!TryConsumeStep(out float leftYaw, out float rightYaw))
                return BuildEstimate();

            _stepCount++;
            float fused = FuseDirection(pelvisYaw, leftYaw, rightYaw);
            UpdateHistory(fused);
            _currentPd   = fused;
            _hasEstimate = true;

            if (context.State == ContextState.ReacquiringPd)
            {
                float stability = ComputeStability();
                if (stability >= StabilityThreshold && _stepCount >= MinStepsBeforeValid)
                    contextDetector.ConfirmStraight();
            }

            return BuildEstimate();
        }

        private float FuseDirection(float pelvisYaw, float leftYaw, float rightYaw)
        {
            // 加权平均，注意角度的圆周性（用向量和再取 atan2）
            float totalW = PelvisWeight + LeftFootWeight + RightFootWeight;
            float wx = (MathF.Cos(pelvisYaw) * PelvisWeight
                      + MathF.Cos(leftYaw)   * LeftFootWeight
                      + MathF.Cos(rightYaw)  * RightFootWeight) / totalW;
            float wy = (MathF.Sin(pelvisYaw) * PelvisWeight
                      + MathF.Sin(leftYaw)   * LeftFootWeight
                      + MathF.Sin(rightYaw)  * RightFootWeight) / totalW;
            return MathF.Atan2(wy, wx);
        }

        private void UpdateHistory(float pd)
        {
            _pdHistory.Enqueue(pd);
            _pdHistorySum   += pd;
            _pdHistorySumSq += pd * pd;
            if (_pdHistory.Count > StabilityWindow)
            {
                float old = _pdHistory.Dequeue();
                _pdHistorySum   -= old;
                _pdHistorySumSq -= old * old;
            }
        }

        private float ComputeStability()
        {
            int n = _pdHistory.Count;
            if (n < 2) return 0f;
            float mean = _pdHistorySum / n;
            float variance = _pdHistorySumSq / n - mean * mean;
            variance = MathF.Max(variance, 0f);
            return 1f / (1f + variance);
        }

        private PdEstimate BuildEstimate()
        {
            float stability = ComputeStability();
            return new PdEstimate
            {
                DirectionRad = _currentPd,
                IsValid      = _hasEstimate && stability >= StabilityThreshold,
                Stability    = stability,
            };
        }

        private static float ExtractYaw(System.Numerics.Quaternion q)
            => MathF.Atan2(2f * (q.W * q.Z + q.X * q.Y),
                           1f - 2f * (q.Y * q.Y + q.Z * q.Z));
    }
}
```

**注意：** 上面代码中 `Update` 方法有一个草稿实现和一个 `UpdateClean` 修正版本。只保留 `UpdateClean`，删除 `Update` 和 `TrackStanceYaw`（非 V2 版本）。在实际写文件时只写干净版本：

最终文件只包含：`Reset`、`UpdateClean`（重命名为 `Update`）、`TrackStanceYawV2`（重命名为 `TrackStanceYaw`）、`TryConsumeStep`、`FuseDirection`、`UpdateHistory`、`ComputeStability`、`BuildEstimate`、`ExtractYaw`。

- [ ] **Step 2: Build 验证**

```
dotnet build IMUMoCap/IMUMoCap.csproj
```
期望：0 errors。

- [ ] **Step 3: Commit**

```bash
git add IMUMoCap/Pipeline/ProgressionDirEstimator.cs
git commit -m "feat: add ProgressionDirEstimator — multi-IMU PD fusion with stability-driven ReacquiringPd exit"
```

---

## Plan 2 完成

Plan 2 产出：CalibrationProcessor + GaitEventDetector + MotionContextDetector + ProgressionDirEstimator。

**继续执行 Plan 3（Output + Integration）：**
`docs/superpowers/plans/2026-04-09-gait-pipeline-plan3-output.md`
