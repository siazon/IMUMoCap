# Gait Pipeline — Plan 3: Output + Integration

> **For agentic workers:** Execute inline in this session (executing-plans). Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 实现 BaselineProcessor、FpaEngine、GaitPipeline 编排类，并将流水线接入 MainWindow，完成完整的端到端数据流。

**Architecture:** `GaitPipeline` 持有所有模块实例，对外暴露单一 `Process(ImuFrameBundle)` 入口，输出 `PipelineOutput`。`MainWindow` 订阅 `GaitPipeline.OnFpaResult` 事件即可，不直接接触任何分析模块。系统状态机在 `MainWindow` 维护，通过 `GaitPipeline` 的状态方法驱动。

**Tech Stack:** C# 12 / .NET 8，System.Numerics，无新 NuGet 包。

**前置条件：** Plan 1 和 Plan 2 已完成。

---

## 文件结构

| 操作 | 路径 | 职责 |
|------|------|------|
| Create | `IMUMoCap/Pipeline/BaselineProcessor.cs` | 收集基线步 → BaselineProfile |
| Create | `IMUMoCap/Pipeline/FpaEngine.cs` | 四重门控 + 早-稳结算 → FpaResult |
| Create | `IMUMoCap/Pipeline/GaitPipeline.cs` | 编排所有模块，暴露事件给 MainWindow |
| Modify | `IMUMoCap/MainWindow.xaml.cs` | 接入 GaitPipeline，实现系统状态机 |

---

## Task 1: BaselineProcessor

**Files:**
- Create: `IMUMoCap/Pipeline/BaselineProcessor.cs`

- [ ] **Step 1: 创建文件**

```csharp
// IMUMoCap/Pipeline/BaselineProcessor.cs
using System.Collections.Generic;
using System.Linq;
using IMUMoCap.Pipeline.Models;

namespace IMUMoCap.Pipeline
{
    /// <summary>
    /// 收集 Baseline 阶段的步级 FPA，计算统计量并生成 BaselineProfile。
    ///
    /// 只接受来自 FpaEngine 的有效 FpaResult（已经过四重门控）。
    /// 达到 MinValidSteps 后可提前调用 TryFinalize 生成结果；
    /// 3 分钟时间到后由 GaitPipeline 强制调用 TryFinalize。
    /// </summary>
    public sealed class BaselineProcessor
    {
        public int MinValidSteps { get; set; } = 20;

        private readonly List<float> _fpaL = new();
        private readonly List<float> _fpaR = new();

        public int CollectedSteps_L => _fpaL.Count;
        public int CollectedSteps_R => _fpaR.Count;
        public bool IsReady => _fpaL.Count >= MinValidSteps && _fpaR.Count >= MinValidSteps;

        /// <summary>每次 FpaEngine 输出一个 FpaResult 时调用。</summary>
        public void AddStep(FpaResult result)
        {
            // FpaResult 中 float.NaN 表示该脚本次无效（单脚无效时另一脚仍记录）
            if (!float.IsNaN(result.Fpa_L)) _fpaL.Add(result.Fpa_L);
            if (!float.IsNaN(result.Fpa_R)) _fpaR.Add(result.Fpa_R);
        }

        /// <summary>
        /// 尝试生成 BaselineProfile。
        /// IsReady 为 false 时返回 null（步数不足）。
        /// 调用方可在 Baseline 结束时强制调用，不足时显示错误。
        /// </summary>
        public BaselineProfile? TryFinalize()
        {
            if (!IsReady) return null;

            float meanL = Mean(_fpaL);
            float sdL   = Sd(_fpaL, meanL);
            float meanR = Mean(_fpaR);
            float sdR   = Sd(_fpaR, meanR);

            return BaselineProfile.Create(
                meanL, sdL, _fpaL.Count,
                meanR, sdR, _fpaR.Count);
        }

        public void Reset()
        {
            _fpaL.Clear();
            _fpaR.Clear();
        }

        // ── 工具方法 ──────────────────────────────────────────────────────────

        private static float Mean(List<float> values)
            => values.Count == 0 ? 0f : values.Sum() / values.Count;

        private static float Sd(List<float> values, float mean)
        {
            if (values.Count < 2) return 0f;
            float sumSq = values.Sum(v => (v - mean) * (v - mean));
            return System.MathF.Sqrt(sumSq / (values.Count - 1));
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
git add IMUMoCap/Pipeline/BaselineProcessor.cs
git commit -m "feat: add BaselineProcessor — collects step-level FPA and computes BaselineProfile"
```

---

## Task 2: FpaEngine

**Files:**
- Create: `IMUMoCap/Pipeline/FpaEngine.cs`

- [ ] **Step 1: 创建文件**

```csharp
// IMUMoCap/Pipeline/FpaEngine.cs
using System;
using System.Collections.Generic;
using IMUMoCap.Pipeline.Models;

namespace IMUMoCap.Pipeline
{
    /// <summary>
    /// 四重门控 + 早-稳结算，输出步级 FpaResult。
    ///
    /// 四重门控（全部满足才进入结算）：
    ///   1. 帧有效（ValidFrame，已由 DataQualityGate 保证）
    ///   2. 步有效：GaitEvent 显示完整 stance 周期（左脚或右脚处于 stance）
    ///   3. 上下文允许：MotionContext.State == Straight 且 Confidence >= ContextConfidenceThreshold
    ///   4. PD 有效：PdEstimate.IsValid == true 且 Stability >= PdStabilityThreshold
    ///
    /// 早-稳结算（per foot，独立）：
    ///   在 stance 期间持续估计 FPA，监测滑动窗口方差；
    ///   方差 < VarianceThreshold 时立即结算；
    ///   超过 StancePhaseUpperLimit（0–1，stance 相位比例）时强制结算。
    ///
    /// FPA 计算：
    ///   FPA = mean(foot_yaw during stance) - PD.DirectionRad，转换为度
    ///   正值 = toe-out，负值 = toe-in
    /// </summary>
    public sealed class FpaEngine
    {
        // ── 门控阈值 ──────────────────────────────────────────────────────────
        public float ContextConfidenceThreshold { get; set; } = 0.7f;
        public float PdStabilityThreshold       { get; set; } = 0.7f;

        // ── 早-稳结算参数 ──────────────────────────────────────────────────────
        public float VarianceThreshold          { get; set; } = 1.0f;  // rad²，方差收敛阈值
        public float StancePhaseUpperLimit      { get; set; } = 0.70f; // stance 时长的 70% 强制结算
        public int   MinStanceFramesForSettle   { get; set; } = 5;     // 最少帧数才允许结算

        // ── BaselineProfile（Training 阶段设置，Baseline 阶段为 null）────────
        public BaselineProfile? Baseline { get; set; }

        // ── 内部 stance 追踪（per foot）───────────────────────────────────────
        private StanceSampler _leftSampler  = new();
        private StanceSampler _rightSampler = new();

        public FpaResult? Process(ValidFrame frame, GaitEvent gait,
                                  MotionContext context, PdEstimate pd)
        {
            // 门控 3 & 4：上下文和 PD
            bool contextOk = context.State == ContextState.Straight
                          && context.Confidence >= ContextConfidenceThreshold;
            bool pdOk = pd.IsValid && pd.Stability >= PdStabilityThreshold;

            if (!contextOk || !pdOk)
            {
                _leftSampler.Reset();
                _rightSampler.Reset();
                return null;
            }

            // 门控 2：step valid（至少一脚在 stance）
            if (!gait.LeftStance && !gait.RightStance)
            {
                _leftSampler.Reset();
                _rightSampler.Reset();
                return null;
            }

            // 采样
            if (gait.LeftStance)
                _leftSampler.AddFrame(ExtractYaw(frame.LeftFoot.Quaternion));
            else
                _leftSampler.MarkSwing();

            if (gait.RightStance)
                _rightSampler.AddFrame(ExtractYaw(frame.RightFoot.Quaternion));
            else
                _rightSampler.MarkSwing();

            // 检查是否有脚完成了结算
            float? fpaL = _leftSampler.TrySettle(VarianceThreshold, StancePhaseUpperLimit,
                                                   MinStanceFramesForSettle);
            float? fpaR = _rightSampler.TrySettle(VarianceThreshold, StancePhaseUpperLimit,
                                                   MinStanceFramesForSettle);

            // 只有至少一脚完成结算时才输出结果
            if (fpaL == null && fpaR == null) return null;

            float fpaLDeg = fpaL.HasValue
                ? RadToDeg(NormalizeAngle(fpaL.Value - pd.DirectionRad))
                : float.NaN;
            float fpaRDeg = fpaR.HasValue
                ? RadToDeg(NormalizeAngle(fpaR.Value - pd.DirectionRad))
                : float.NaN;

            bool onTargetL = false, onTargetR = false;
            float errorL = float.NaN, errorR = float.NaN;

            if (Baseline != null)
            {
                if (!float.IsNaN(fpaLDeg))
                {
                    errorL     = fpaLDeg - Baseline.Target_L;
                    onTargetL  = MathF.Abs(errorL) <= Baseline.Tolerance_L;
                }
                if (!float.IsNaN(fpaRDeg))
                {
                    errorR     = fpaRDeg - Baseline.Target_R;
                    onTargetR  = MathF.Abs(errorR) <= Baseline.Tolerance_R;
                }
            }

            return new FpaResult
            {
                PacketId   = frame.PacketId,
                Fpa_L      = fpaLDeg,
                Fpa_R      = fpaRDeg,
                OnTarget_L = onTargetL,
                OnTarget_R = onTargetR,
                Error_L    = errorL,
                Error_R    = errorR,
            };
        }

        public void Reset()
        {
            _leftSampler.Reset();
            _rightSampler.Reset();
        }

        // ── 工具方法 ──────────────────────────────────────────────────────────

        private static float ExtractYaw(System.Numerics.Quaternion q)
            => MathF.Atan2(2f * (q.W * q.Z + q.X * q.Y),
                           1f - 2f * (q.Y * q.Y + q.Z * q.Z));

        private static float NormalizeAngle(float rad)
        {
            while (rad >  MathF.PI) rad -= 2f * MathF.PI;
            while (rad < -MathF.PI) rad += 2f * MathF.PI;
            return rad;
        }

        private static float RadToDeg(float rad) => rad * (180f / MathF.PI);

        // ── StanceSampler（内嵌私有类）────────────────────────────────────────
        // 追踪单脚在 stance 期间的 yaw 样本，实现早-稳结算逻辑

        private sealed class StanceSampler
        {
            private readonly List<float> _yaws = new();
            private bool  _inStance       = false;
            private bool  _settled        = false;
            private float _settledYaw     = 0f;
            private int   _totalStanceFramesEstimate = 0; // 用前两次 stance 长度估算

            public void AddFrame(float yaw)
            {
                if (!_inStance)
                {
                    _inStance  = true;
                    _settled   = false;
                    _yaws.Clear();
                }
                _yaws.Add(yaw);
            }

            public void MarkSwing()
            {
                if (_inStance)
                {
                    // stance 结束，如果还没结算，记录当前均值
                    if (!_settled && _yaws.Count > 0)
                    {
                        _settledYaw = Mean(_yaws);
                        _settled    = true;
                    }
                    // 更新 stance 长度估计（取最近一次）
                    if (_yaws.Count > 0)
                        _totalStanceFramesEstimate = _yaws.Count;
                    _inStance = false;
                }
            }

            /// <summary>
            /// 在 stance 中途尝试提前结算。
            /// 返回结算值（rad）或 null（尚未结算/上次已消费）。
            /// </summary>
            public float? TrySettle(float varianceThreshold, float phaseUpperLimit, int minFrames)
            {
                // 已结算且 stance 刚结束（MarkSwing 触发）：返回结果并消费
                if (_settled && !_inStance)
                {
                    _settled = false;
                    return _settledYaw;
                }

                if (!_inStance || _settled) return null;
                if (_yaws.Count < minFrames)  return null;

                // 计算当前方差
                float mean = Mean(_yaws);
                float var  = Variance(_yaws, mean);

                // 方差收敛
                if (var < varianceThreshold)
                {
                    _settledYaw = mean;
                    _settled    = true;
                    return null; // 等 swing 开始后再报告，避免重复
                }

                // 超过相位上限，强制结算
                if (_totalStanceFramesEstimate > 0)
                {
                    float phase = (float)_yaws.Count / _totalStanceFramesEstimate;
                    if (phase >= phaseUpperLimit)
                    {
                        _settledYaw = mean;
                        _settled    = true;
                    }
                }

                return null;
            }

            public void Reset()
            {
                _yaws.Clear();
                _inStance  = false;
                _settled   = false;
            }

            private static float Mean(List<float> vs)
            {
                float sum = 0f;
                foreach (var v in vs) sum += v;
                return sum / vs.Count;
            }

            private static float Variance(List<float> vs, float mean)
            {
                float sumSq = 0f;
                foreach (var v in vs) sumSq += (v - mean) * (v - mean);
                return sumSq / vs.Count;
            }
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
git add IMUMoCap/Pipeline/FpaEngine.cs
git commit -m "feat: add FpaEngine — four-gate control, early-stable settling, step-level FpaResult"
```

---

## Task 3: GaitPipeline（编排类）

**Files:**
- Create: `IMUMoCap/Pipeline/GaitPipeline.cs`

- [ ] **Step 1: 创建文件**

```csharp
// IMUMoCap/Pipeline/GaitPipeline.cs
using System;
using IMUMoCap.Pipeline.Models;

namespace IMUMoCap.Pipeline
{
    /// <summary>
    /// 流水线编排类。持有所有模块实例，对外暴露：
    ///   - Process(ImuFrameBundle)：主入口，每帧调用一次
    ///   - OnFpaResult 事件：每步输出一个 FpaResult
    ///   - OnCalibrationStateChanged 事件
    ///   - OnBaselineProgress 事件（当前有效步数）
    ///
    /// MainWindow 只与 GaitPipeline 交互，不直接接触任何分析模块。
    /// </summary>
    public sealed class GaitPipeline
    {
        // ── 子模块 ────────────────────────────────────────────────────────────
        private readonly DataQualityGate         _gate        = new();
        private readonly CalibrationProcessor    _calibration = new();
        private readonly GaitEventDetector       _gait        = new();
        private readonly MotionContextDetector   _motion      = new();
        private readonly ProgressionDirEstimator _pd          = new();
        private readonly BaselineProcessor       _baseline    = new();
        private readonly FpaEngine               _fpa         = new();

        // ── 事件 ──────────────────────────────────────────────────────────────
        public event Action<FpaResult>?         OnFpaResult;
        public event Action<CalibrationState>?  OnCalibrationStateChanged;
        public event Action<int, int>?          OnBaselineProgress;  // (stepsL, stepsR)
        public event Action<string>?            OnLog;

        // ── 状态 ──────────────────────────────────────────────────────────────
        public CalibrationProfile? CalibrationProfile => _calibration.Profile;
        public BaselineProfile?    BaselineProfile    { get; private set; }
        public bool InBaseline  { get; private set; }
        public bool InTraining  { get; private set; }

        // ── 主入口 ────────────────────────────────────────────────────────────

        /// <summary>每帧调用一次，传入来自 ImuFrameCollector 的完整帧。</summary>
        public void Process(ImuFrameBundle bundle)
        {
            // Step 1: DataQualityGate
            var validFrame = _gate.Evaluate(bundle);
            if (validFrame == null) return;

            // Step 2: CalibrationProcessor（仅在未完成校准时运行）
            if (_calibration.State != CalibrationState.Completed &&
                _calibration.State != CalibrationState.Failed)
            {
                bool stateChanged = _calibration.Process(validFrame);
                if (stateChanged)
                {
                    OnCalibrationStateChanged?.Invoke(_calibration.State);
                    if (_calibration.State == CalibrationState.Completed)
                        OnLog?.Invoke("Calibration completed.");
                    else if (_calibration.State == CalibrationState.Failed)
                        OnLog?.Invoke("Calibration failed — please restart.");
                }
                // 校准完成前，后续模块不运行
                if (_calibration.State != CalibrationState.Completed) return;
            }

            var profile = _calibration.Profile!;

            // Step 3: GaitEventDetector
            var gaitEvent = _gait.Detect(validFrame, profile);

            // Step 4: MotionContextDetector
            var motionCtx = _motion.Detect(validFrame, gaitEvent);

            // Step 5: ProgressionDirEstimator
            var pdEstimate = _pd.Update(validFrame, gaitEvent, motionCtx, _motion);

            // Step 6: FpaEngine
            var fpaResult = _fpa.Process(validFrame, gaitEvent, motionCtx, pdEstimate);

            if (fpaResult == null) return;

            // Baseline 阶段：收集步级 FPA
            if (InBaseline)
            {
                _baseline.AddStep(fpaResult);
                OnBaselineProgress?.Invoke(_baseline.CollectedSteps_L, _baseline.CollectedSteps_R);
                OnFpaResult?.Invoke(fpaResult);  // UI 可选显示
                return;
            }

            // Training 阶段：直接输出供 AR 反馈
            if (InTraining)
                OnFpaResult?.Invoke(fpaResult);
        }

        // ── 阶段控制（由 MainWindow 的状态机调用）────────────────────────────

        public void StartBaseline()
        {
            _baseline.Reset();
            _fpa.Baseline = null;
            InBaseline = true;
            InTraining = false;
            OnLog?.Invoke("Baseline started.");
        }

        /// <summary>
        /// 结束 Baseline 采集，尝试生成 BaselineProfile。
        /// 返回 null 表示步数不足（调用方负责显示错误）。
        /// </summary>
        public BaselineProfile? FinalizeBaseline()
        {
            InBaseline  = false;
            BaselineProfile = _baseline.TryFinalize();
            if (BaselineProfile != null)
                OnLog?.Invoke($"Baseline done. μL={BaselineProfile.MeanFpa_L:F1}° μR={BaselineProfile.MeanFpa_R:F1}°");
            else
                OnLog?.Invoke("Baseline failed — insufficient valid steps.");
            return BaselineProfile;
        }

        public void StartTraining()
        {
            if (BaselineProfile == null)
                throw new InvalidOperationException("Must finalize baseline before starting training.");
            _fpa.Baseline = BaselineProfile;
            _fpa.Reset();
            InTraining = true;
            InBaseline = false;
            OnLog?.Invoke("Training started.");
        }

        public void Reset()
        {
            _gate.Reset();
            _calibration.Reset();
            _gait.Reset();
            _motion.Reset();
            _pd.Reset();
            _baseline.Reset();
            _fpa.Reset();
            BaselineProfile = null;
            InBaseline = InTraining = false;
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
git add IMUMoCap/Pipeline/GaitPipeline.cs
git commit -m "feat: add GaitPipeline — orchestrates all modules, exposes events for MainWindow"
```

---

## Task 4: MainWindow 集成

**Files:**
- Modify: `IMUMoCap/MainWindow.xaml.cs`

- [ ] **Step 1: 添加 GaitPipeline 字段和初始化**

在 `MainWindow` 类的字段区（`_deviceManager` 和 `_slotRegistry` 旁边）添加：

```csharp
private readonly GaitPipeline _pipeline = new();
```

在构造函数末尾（`StartScanAsync()` 之前）添加：

```csharp
// 流水线事件订阅
_pipeline.OnLog += msg => Dispatcher.BeginInvoke(() => log(msg));
_pipeline.OnCalibrationStateChanged += s => Dispatcher.BeginInvoke(() => OnCalibrationStateChanged(s));
_pipeline.OnBaselineProgress += (l, r) => Dispatcher.BeginInvoke(() =>
    log($"Baseline: L={l} steps, R={r} steps"));
_pipeline.OnFpaResult += result => Dispatcher.BeginInvoke(() => OnFpaResult(result));
```

在 `using` 区添加：
```csharp
using IMUMoCap.Pipeline;
using IMUMoCap.Pipeline.Models;
```

- [ ] **Step 2: 在 OnDataPacket 中接入流水线**

找到 `MainWindow` 中的 `OnDataPacket` 方法，在 `OnXsensData(...)` 调用之后添加：

```csharp
// 流水线处理
_pipeline.Process(ev.Bundle);
```

其中 `ev` 是 `DataPacketEvent`，需要在 `DataPacketEvent` 中暴露 `ImuFrameBundle`。

在 `IMUMoCap/Services/ImuDeviceManager.cs` 的 `DataPacketEvent` record 定义处添加 Bundle 字段：

```csharp
public record DataPacketEvent(uint DeviceId, ImuViewModel Slot, XsDataPacket Packet, ImuFrameBundle Bundle);
```

并在 `OnDataAvailable` 中构造 event 时传入 bundle（需要把 bundle 引用传进来——在 `OnDataAvailable` 的参数中，`e.Device` 对应的 bundle 是当前 `SyncFrame`）。

**注意：** `ImuDeviceManager.OnDataAvailable` 当前直接操作 `XsDataPacket`，没有持有 `ImuFrameBundle` 引用。需要将 bundle 传递路径打通：

在 `ImuDeviceManager` 中，`OnDataAvailable` 目前逐包处理，但 `ImuFrameCollector` 已经在 MainWindow 的 `OnXsensData` 路径中组装 bundle。

最简洁的修改方案：**不改动 DataPacketEvent**，改为在 GaitPipeline 内部通过另一个入口接收单包，让 GaitPipeline 内部持有一个 `ImuFrameCollector`：

在 `GaitPipeline` 添加一个 `ProcessPacket` 入口：

```csharp
private readonly ImuFrameCollector _frameCollector = new();

/// <summary>每个 IMU 的单包调用，内部组装 bundle 再进流水线。</summary>
public void ProcessPacket(ImuRole role, uint deviceId, XsDataPacket packet)
{
    var (_, bundle) = _frameCollector.Process(role, deviceId, packet);
    if (bundle?.IsComplete == true)
        Process(bundle);
}
```

在 `MainWindow.OnXsensData` 中改为：

```csharp
void OnXsensData(ImuRole sensor, uint deviceId, XsDataPacket packet)
{
    var (sample, _) = _imuFrameCollector.Process(sensor, deviceId, packet);
    _imuSamples.Add(sample);
    _pipeline.ProcessPacket(sensor, deviceId, packet);
}
```

这样 `GaitPipeline` 自己维护一个 `ImuFrameCollector` 做帧聚合（与用于录制的 `_imuFrameCollector` 并行），不需要改动 `DataPacketEvent`。

- [ ] **Step 3: 实现系统状态机响应方法**

在 `MainWindow` 添加以下方法：

```csharp
// ── 校准状态变化 ──────────────────────────────────────────────────────────
private void OnCalibrationStateChanged(CalibrationState state)
{
    switch (state)
    {
        case CalibrationState.CollectingStaticPose:
            _content.StatusLabel = "Calibrating... Stand still.";
            _sessionState = TestState.Calibrating;
            break;

        case CalibrationState.Completed:
            _content.StatusLabel = "Calibration done. Starting baseline walk.";
            _content.CalibrationState = "Calibrated";
            _sessionState = TestState.Calibrated;
            // 自动开始 Baseline
            _pipeline.StartBaseline();
            _baselineTimer = new System.Windows.Threading.DispatcherTimer
            {
                Interval = TimeSpan.FromMinutes(3)
            };
            _baselineTimer.Tick += (_, _) =>
            {
                _baselineTimer.Stop();
                FinalizeBaseline();
            };
            _baselineTimer.Start();
            _sessionState = TestState.Baseline;
            _content.StatusLabel = "Baseline: Walk naturally for 3 minutes.";
            break;

        case CalibrationState.Failed:
            _content.StatusLabel = "Calibration failed. Please restart the application.";
            _sessionState = TestState.Launching;
            break;
    }
}

private System.Windows.Threading.DispatcherTimer? _baselineTimer;

private void FinalizeBaseline()
{
    var profile = _pipeline.FinalizeBaseline();
    if (profile == null)
    {
        _content.StatusLabel = "Baseline insufficient (< 20 valid steps). Please restart.";
        return;
    }
    _sessionState = TestState.Step; // 复用 Step 表示 Training
    _content.StatusLabel =
        $"Training started. Target L={profile.Target_L:F1}° ({profile.Direction_L})  " +
        $"R={profile.Target_R:F1}° ({profile.Direction_R})";
    _pipeline.StartTraining();
}

// ── FPA 结果处理（每步一次）─────────────────────────────────────────────
private void OnFpaResult(FpaResult result)
{
    // 更新 UI 状态
    if (!float.IsNaN(result.Fpa_L))
        _content.LeftFpaDeg = result.Fpa_L;
    if (!float.IsNaN(result.Fpa_R))
        _content.RightFpaDeg = result.Fpa_R;

    _content.StanceSummary =
        $"L: {result.Fpa_L:F1}° {(result.OnTarget_L ? "✓" : $"err={result.Error_L:+0.0;-0.0}°")}  " +
        $"R: {result.Fpa_R:F1}° {(result.OnTarget_R ? "✓" : $"err={result.Error_R:+0.0;-0.0}°")}";

    // 将 FPA 结果推送给 AR 端（WebSocket）
    if (_wsServer != null)
    {
        _ = _wsServer.BroadcastJsonAsync(new
        {
            type      = "fpa",
            packetId  = result.PacketId,
            fpaL      = result.Fpa_L,
            fpaR      = result.Fpa_R,
            onTargetL = result.OnTarget_L,
            onTargetR = result.OnTarget_R,
            errorL    = result.Error_L,
            errorR    = result.Error_R,
        });
    }
}
```

- [ ] **Step 4: 处理 TestState 枚举（如需新增状态）**

查看 `TestState` 枚举的当前定义（在 `IMUMoCap/Model/DeviceModel.cs` 或类似文件），确认包含以下状态，不足时添加：

```csharp
public enum TestState
{
    Launching,
    Connected,
    Calibrating,
    Calibrated,
    Baseline,
    Step        // 复用为 Training
}
```

- [ ] **Step 5: 在 `OnClosed` 中停止 baselineTimer**

在 `OnClosed` 方法的 `_imuLoopCts?.Cancel()` 之后添加：

```csharp
_baselineTimer?.Stop();
```

- [ ] **Step 6: Build 验证**

```
dotnet build IMUMoCap/IMUMoCap.csproj
```
期望：0 errors。修复所有编译错误后继续。

- [ ] **Step 7: Commit**

```bash
git add IMUMoCap/MainWindow.xaml.cs IMUMoCap/Pipeline/GaitPipeline.cs IMUMoCap/Services/ImuDeviceManager.cs
git commit -m "feat: integrate GaitPipeline into MainWindow — full end-to-end data flow"
```

---

## Plan 3 完成 — 全系统就绪

三个 Plan 完成后，完整流水线：

```
Sensor → ImuFrameCollector → DataQualityGate → CalibrationProcessor
       → GaitEventDetector → MotionContextDetector → ProgressionDirEstimator
       → FpaEngine → (BaselineProcessor | AR WebSocket)
```

**系统状态机：**
```
Launching → Connected → Calibrating → Baseline（3分钟）→ Training
```

**下一步可扩展：**
- AR 端 WebSocket 消息格式细化
- GaitEventDetector 参数调优（通过实验数据）
- FrameQualityReport 在实验结束时导出
