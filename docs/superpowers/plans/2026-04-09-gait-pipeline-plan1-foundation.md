# Gait Pipeline — Plan 1: Foundation

> **For agentic workers:** Execute inline in this session (executing-plans). Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 建立流水线的数据模型层、增强 ImuFrameCollector 的 gap 统计能力、实现 DataQualityGate。

**Architecture:** 所有模型放在 `IMUMoCap/Pipeline/Models/`；操作类放在 `IMUMoCap/Pipeline/`；`ImuFrameCollector` 原地增强，不移动文件。Plan 2 和 Plan 3 依赖本 Plan 的类型定义，因此类型签名一旦确定不得随意更改。

**Tech Stack:** C# 12 / .NET 8，System.Numerics，无新 NuGet 包。

---

## 文件结构

| 操作 | 路径 | 职责 |
|------|------|------|
| Create | `IMUMoCap/Pipeline/Models/PipelineModels.cs` | ValidFrame、GaitEvent、MotionContext、PdEstimate、FpaResult |
| Create | `IMUMoCap/Pipeline/Models/CalibrationProfile.cs` | 校准结果 |
| Create | `IMUMoCap/Pipeline/Models/BaselineProfile.cs` | 基线统计 + 训练目标 |
| Create | `IMUMoCap/Pipeline/DataQualityGate.cs` | 帧质量过滤 |
| Modify | `IMUMoCap/Methods/ImuFrameCollector.cs` | 增加 gap 统计 |

---

## Task 1: 创建 Pipeline/Models 目录和核心模型

**Files:**
- Create: `IMUMoCap/Pipeline/Models/PipelineModels.cs`

- [ ] **Step 1: 创建文件**

```csharp
// IMUMoCap/Pipeline/Models/PipelineModels.cs
using System.Numerics;

namespace IMUMoCap.Pipeline.Models
{
    // ── ValidFrame ────────────────────────────────────────────────────────────
    // ImuFrameBundle 通过 DataQualityGate 后的包装，表示该帧数据可信
    public sealed class ValidFrame
    {
        public ValidFrame(ImuFrameBundle bundle) => Bundle = bundle;
        public ImuFrameBundle Bundle { get; }
        public long   PacketId  => Bundle.PacketId;
        public double TimeSec   => Bundle.TimeSec;
        public ImuSampleFrame Pelvis    => Bundle.Pelvis!;
        public ImuSampleFrame LeftFoot  => Bundle.LeftFoot!;
        public ImuSampleFrame RightFoot => Bundle.RightFoot!;
    }

    // ── GaitEvent ─────────────────────────────────────────────────────────────
    // 每帧的步态事件，由 GaitEventDetector 输出
    public sealed class GaitEvent
    {
        public bool LeftStance      { get; set; }
        public bool RightStance     { get; set; }
        public bool LeftSwing       { get; set; }
        public bool RightSwing      { get; set; }
        public bool StompDetected   { get; set; }   // 左脚跺脚（校准后精确版）
        public bool IsWalking       { get; set; }
    }

    // ── MotionContext ─────────────────────────────────────────────────────────
    // 运动上下文，由 MotionContextDetector 输出
    public enum ContextState { Straight, Turning, ReacquiringPd }

    public sealed class MotionContext
    {
        public ContextState State      { get; set; }
        public float        Confidence { get; set; }  // 0–1
    }

    // ── PdEstimate ────────────────────────────────────────────────────────────
    // 行进方向估计，由 ProgressionDirEstimator 输出
    public sealed class PdEstimate
    {
        public float DirectionRad { get; set; }
        public bool  IsValid      { get; set; }
        public float Stability    { get; set; }  // 0–1，方差归一化倒数
    }

    // ── FpaResult ─────────────────────────────────────────────────────────────
    // 步级 FPA 结果，由 FpaEngine 输出（每步一个）
    public sealed class FpaResult
    {
        public long  PacketId    { get; set; }
        public float Fpa_L       { get; set; }  // 度，正值=toe-out
        public float Fpa_R       { get; set; }
        public bool  OnTarget_L  { get; set; }
        public bool  OnTarget_R  { get; set; }
        public float Error_L     { get; set; }  // FPA - Target，带符号，供AR反馈
        public float Error_R     { get; set; }
    }

    // ── FrameQualityReport ────────────────────────────────────────────────────
    // 实验结束后的帧质量报告，由 ImuFrameCollector 生成
    public sealed class FrameQualityReport
    {
        public int TotalFrames        { get; set; }
        public int CompletedFrames    { get; set; }
        public int PelvisGapFrames    { get; set; }
        public int LeftFootGapFrames  { get; set; }
        public int RightFootGapFrames { get; set; }
        public float CompletionRate   => TotalFrames == 0 ? 0f :
            (float)CompletedFrames / TotalFrames;
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
git add IMUMoCap/Pipeline/Models/PipelineModels.cs
git commit -m "feat: add pipeline model types (ValidFrame, GaitEvent, MotionContext, PdEstimate, FpaResult)"
```

---

## Task 2: 创建 CalibrationProfile 和 BaselineProfile

**Files:**
- Create: `IMUMoCap/Pipeline/Models/CalibrationProfile.cs`
- Create: `IMUMoCap/Pipeline/Models/BaselineProfile.cs`

- [ ] **Step 1: 创建 CalibrationProfile.cs**

```csharp
// IMUMoCap/Pipeline/Models/CalibrationProfile.cs
using System.Numerics;

namespace IMUMoCap.Pipeline.Models
{
    /// <summary>
    /// 校准结果：三个 IMU 各自在静立时的参考姿态四元数。
    /// 用于后续所有模块消除安装误差。
    /// </summary>
    public sealed class CalibrationProfile
    {
        public Quaternion PelvisRef    { get; init; }
        public Quaternion LeftFootRef  { get; init; }
        public Quaternion RightFootRef { get; init; }
    }
}
```

- [ ] **Step 2: 创建 BaselineProfile.cs**

```csharp
// IMUMoCap/Pipeline/Models/BaselineProfile.cs
namespace IMUMoCap.Pipeline.Models
{
    public enum TrainingDirection { ToeIn, ToeOut }

    /// <summary>
    /// 基线统计结果 + 个性化训练目标。
    /// 由 BaselineProcessor 在 Baseline 阶段结束后生成，之后只读。
    /// </summary>
    public sealed class BaselineProfile
    {
        // 基线统计
        public float MeanFpa_L  { get; init; }
        public float MeanFpa_R  { get; init; }
        public float SdFpa_L    { get; init; }
        public float SdFpa_R    { get; init; }
        public float Asymmetry  { get; init; }   // |μ_L - μ_R|
        public int   ValidSteps_L { get; init; }
        public int   ValidSteps_R { get; init; }

        // 训练目标（Δ = 5° 固定）
        public float Target_L    { get; init; }  // T = μ ± 5°
        public float Target_R    { get; init; }
        public float Tolerance_L { get; init; }  // clamp(SD, 2°, 5°)
        public float Tolerance_R { get; init; }
        public TrainingDirection Direction_L { get; init; }
        public TrainingDirection Direction_R { get; init; }

        public bool IsValid => ValidSteps_L >= 20 && ValidSteps_R >= 20;

        /// <summary>
        /// 从统计数据计算训练目标，规则：
        /// μ > 15° → ToeIn，T = μ - 5°
        /// μ < 5°  → ToeOut，T = μ + 5°
        /// 5°–15°  → 朝 10° 靠拢（取近的方向），T = μ ± 5°
        /// </summary>
        public static BaselineProfile Create(
            float meanL, float sdL, int stepsL,
            float meanR, float sdR, int stepsR)
        {
            static (float target, TrainingDirection dir) ComputeTarget(float mean)
            {
                if (mean > 10f)
                    return (mean - 5f, TrainingDirection.ToeIn);
                else
                    return (mean + 5f, TrainingDirection.ToeOut);
            }

            static float ComputeTolerance(float sd) => Math.Clamp(sd, 2f, 5f);

            var (tL, dL) = ComputeTarget(meanL);
            var (tR, dR) = ComputeTarget(meanR);

            return new BaselineProfile
            {
                MeanFpa_L   = meanL, SdFpa_L   = sdL, ValidSteps_L = stepsL,
                MeanFpa_R   = meanR, SdFpa_R   = sdR, ValidSteps_R = stepsR,
                Asymmetry   = MathF.Abs(meanL - meanR),
                Target_L    = tL,    Direction_L = dL, Tolerance_L = ComputeTolerance(sdL),
                Target_R    = tR,    Direction_R = dR, Tolerance_R = ComputeTolerance(sdR),
            };
        }
    }
}
```

- [ ] **Step 3: Build 验证**

```
dotnet build IMUMoCap/IMUMoCap.csproj
```
期望：0 errors。

- [ ] **Step 4: Commit**

```bash
git add IMUMoCap/Pipeline/Models/CalibrationProfile.cs IMUMoCap/Pipeline/Models/BaselineProfile.cs
git commit -m "feat: add CalibrationProfile and BaselineProfile models"
```

---

## Task 3: ImuFrameCollector — 增加 gap 统计

**Files:**
- Modify: `IMUMoCap/Methods/ImuFrameCollector.cs`

- [ ] **Step 1: 在文件顶部添加 using，并在类中添加 gap 计数字段和 GenerateReport 方法**

在 `ImuFrameCollector` 类内添加以下字段和方法（在现有代码末尾 `}` 之前插入）：

```csharp
// ── Gap 统计字段（在现有字段之后添加）────────────────────────────────────
private int _totalFramesSeen   = 0;
private int _completedFrames   = 0;
private int _pelvisGapFrames   = 0;
private int _leftFootGapFrames = 0;
private int _rightFootGapFrames = 0;
```

在 `UpsertFrameBundle` 方法内，当 `bundle.IsComplete` 为 true 时（即 `_lastCompletedPacketId = sample.PacketId;` 那行之前），在 `return bundle;` 之前插入：

```csharp
_completedFrames++;
```

在 `CleanupStaleFrameBundles` 方法内，在 `_frameBundles.Remove(stalePacketId)` 之前插入：

```csharp
// 统计哪个 IMU 缺失导致超时丢弃
var staleBundle = _frameBundles[stalePacketId];
if (staleBundle.Pelvis    == null) _pelvisGapFrames++;
if (staleBundle.LeftFoot  == null) _leftFootGapFrames++;
if (staleBundle.RightFoot == null) _rightFootGapFrames++;
_totalFramesSeen++;
```

在类末尾添加：

```csharp
public FrameQualityReport GenerateReport()
{
    return new IMUMoCap.Pipeline.Models.FrameQualityReport
    {
        TotalFrames        = _totalFramesSeen + _completedFrames,
        CompletedFrames    = _completedFrames,
        PelvisGapFrames    = _pelvisGapFrames,
        LeftFootGapFrames  = _leftFootGapFrames,
        RightFootGapFrames = _rightFootGapFrames,
    };
}
```

在文件顶部添加：
```csharp
using IMUMoCap.Pipeline.Models;
```

- [ ] **Step 2: Build 验证**

```
dotnet build IMUMoCap/IMUMoCap.csproj
```
期望：0 errors。

- [ ] **Step 3: Commit**

```bash
git add IMUMoCap/Methods/ImuFrameCollector.cs
git commit -m "feat: add gap statistics and FrameQualityReport to ImuFrameCollector"
```

---

## Task 4: DataQualityGate

**Files:**
- Create: `IMUMoCap/Pipeline/DataQualityGate.cs`

- [ ] **Step 1: 创建文件**

```csharp
// IMUMoCap/Pipeline/DataQualityGate.cs
using System.Numerics;
using IMUMoCap.Pipeline.Models;

namespace IMUMoCap.Pipeline
{
    /// <summary>
    /// 对完整的 ImuFrameBundle 进行质量检查。
    /// 通过检查则返回 ValidFrame，否则返回 null（帧被丢弃）。
    ///
    /// 检查项：
    /// 1. StatusWord 异常位（bit 1 = clipFlag, bit 2 = syncLost）
    /// 2. 任一 IMU 的 RSSI 低于阈值
    /// 3. 任一 IMU 的加速度与上帧差超突变阈值（孤立尖峰）
    /// 4. 任一 IMU 的角速度与上帧差超突变阈值
    /// </summary>
    public sealed class DataQualityGate
    {
        // 可调参数（实验前根据环境设定，不需要运行时动态修改）
        public int   RssiThresholdDbm          { get; set; } = -85;
        public float AccDeltaThreshold_ms2     { get; set; } = 50f;   // m/s²
        public float GyroDeltaThreshold_rads   { get; set; } = 20f;   // rad/s

        private ImuSampleFrame? _prevPelvis;
        private ImuSampleFrame? _prevLeft;
        private ImuSampleFrame? _prevRight;

        public ValidFrame? Evaluate(ImuFrameBundle bundle)
        {
            if (!bundle.IsComplete) return null;

            var p = bundle.Pelvis!;
            var l = bundle.LeftFoot!;
            var r = bundle.RightFoot!;

            // 1. StatusWord 检查
            if (HasStatusError(p) || HasStatusError(l) || HasStatusError(r))
            {
                UpdatePrev(p, l, r);
                return null;
            }

            // 2. RSSI 检查
            if (p.Rssi < RssiThresholdDbm || l.Rssi < RssiThresholdDbm || r.Rssi < RssiThresholdDbm)
            {
                UpdatePrev(p, l, r);
                return null;
            }

            // 3 & 4. 突变检查（与上帧比较）
            if (_prevPelvis != null && (HasAccSpike(p, _prevPelvis) || HasGyroSpike(p, _prevPelvis)))
            {
                UpdatePrev(p, l, r);
                return null;
            }
            if (_prevLeft != null && (HasAccSpike(l, _prevLeft) || HasGyroSpike(l, _prevLeft)))
            {
                UpdatePrev(p, l, r);
                return null;
            }
            if (_prevRight != null && (HasAccSpike(r, _prevRight) || HasGyroSpike(r, _prevRight)))
            {
                UpdatePrev(p, l, r);
                return null;
            }

            UpdatePrev(p, l, r);
            return new ValidFrame(bundle);
        }

        public void Reset()
        {
            _prevPelvis = _prevLeft = _prevRight = null;
        }

        // ── 私有辅助 ──────────────────────────────────────────────────────────

        private static bool HasStatusError(ImuSampleFrame f)
        {
            if (f.StatusWord == null) return false;
            uint s = f.StatusWord.Value;
            // bit 1: clipFlag (sensor saturation), bit 2: syncLost
            return (s & 0b0110u) != 0;
        }

        private bool HasAccSpike(ImuSampleFrame curr, ImuSampleFrame prev)
        {
            if (!curr.HasAcceleration || !prev.HasAcceleration) return false;
            return Vector3.Distance(curr.Acceleration, prev.Acceleration) > AccDeltaThreshold_ms2;
        }

        private bool HasGyroSpike(ImuSampleFrame curr, ImuSampleFrame prev)
        {
            if (!curr.HasRateOfTurn || !prev.HasRateOfTurn) return false;
            return Vector3.Distance(curr.RateOfTurn, prev.RateOfTurn) > GyroDeltaThreshold_rads;
        }

        private void UpdatePrev(ImuSampleFrame p, ImuSampleFrame l, ImuSampleFrame r)
        {
            _prevPelvis = p;
            _prevLeft   = l;
            _prevRight  = r;
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
git add IMUMoCap/Pipeline/DataQualityGate.cs
git commit -m "feat: add DataQualityGate — filters status errors, RSSI drops, and motion spikes"
```

---

## Plan 1 完成

Plan 1 产出：模型类型层 + ImuFrameCollector gap 统计 + DataQualityGate。

**继续执行 Plan 2（Analysis）：**
`docs/superpowers/plans/2026-04-09-gait-pipeline-plan2-analysis.md`
