# Demo 模式调参说明

> 目的：快速展示流程，优先响应速度，放宽精度要求。  
> 以下参数均可在代码中直接修改默认值，部分参数通过 `PipelineParams` 可在 UI 实时调整。

---

## 一览表

| 参数 | 所在类 | 默认值 | Demo 推荐值 | 效果 |
|------|--------|--------|------------|------|
| `StaticCollectFrames` | `CalibrationProcessor` | 300（3 s） | **50（0.5 s）** | 标定采集时间大幅缩短 |
| `StaticRequiredFrames` | `CalibrationProcessor` | 30（0.3 s） | **10（0.1 s）** | 连续静止门槛降低，更容易触发采集 |
| `StaticTimeoutFrames` | `CalibrationProcessor` | 1000（10 s） | **300（3 s）** | 标定超时提前，尽快失败重试 |
| `MinValidSteps` | `BaselineProcessor` / `PipelineParams` | 20 步 | **5 步** | Baseline 所需步数 |
| `StabilityWindow` | `ProgressionDirEstimator` | 10 步 | **3 步** | PD 稳定性计算窗口，影响 FPA 启动速度和转弯恢复速度 |
| `StabilityThreshold` | `ProgressionDirEstimator` | 0.85 | **0.65** | PD 稳定性阈值，同时决定 FPA 能否输出、转弯后何时恢复直线 |
| `MinStepsBeforeValid` | `ProgressionDirEstimator` | 2 步 | **1 步** | ReacquiringPd → Straight 最少需要的步数 |
| `StraightConfirmFrames` | `MotionContextDetector` | 20 帧（0.2 s） | **10 帧（0.1 s）** | 转弯结束后退出 Turning 所需帧数，加快进入 ReacquiringPd |
| `MinStanceFrames` | `GaitEventDetector` | 5 帧 | **3 帧** | 站立相防抖帧数，FPA 响应更快 |
| `MinStanceFramesForSettle` | `FpaEngine` | 10 帧（100 ms） | **5 帧（50 ms）** | 落地稳定后多少帧输出 FPA，减少启动延迟 |
| `PdStabilityThreshold` | `FpaEngine` / `PipelineParams` | 0.7 | **0.5** | FPA 输出的 PD 稳定性门限 |
| `ContextConfidenceThreshold` | `FpaEngine` / `PipelineParams` | 0.7 | **0.5** | FPA 输出的运动置信度门限 |
| `StanceFootPitchThreshold` | `GaitEventDetector` / `PipelineParams` | 0.35 rad（20°） | 保持 **0.35 rad** | 转弯后 left foot 安装倾斜角大，已修为 20°，无需再改 |
| `WalkingWindowFrames` | `GaitEventDetector` | 300 帧（3 s） | **100 帧（1 s）** | `IsWalking` 判定窗口，启动后 1 s 即可识别行走 |

---

## 分项说明

### 1. 标定（Calibration）加速

**修改文件：** `IMUMoCap/Pipeline/CalibrationProcessor.cs`

```csharp
public int StaticCollectFrames  { get; set; } = 50;   // 原 300，0.5s 即完成
public int StaticRequiredFrames { get; set; } = 10;   // 原 30，0.1s 连续静止
public int StaticTimeoutFrames  { get; set; } = 300;  // 原 1000，3s 内完不成就失败
```

**代价：** 校准参考姿态样本少，足部安装偏差补偿略差；Demo 场景可接受。

---

### 2. Baseline 步数

**修改方式：** `PipelineParams.MinBaselineSteps`（UI 实时可调，或修改默认值）

```csharp
// PipelineParams.cs
public int MinBaselineSteps { get; set; } = 5;   // 原 20
```

**代价：** 5 步统计均值方差不够稳定，目标角度误差可能偏大。Demo 展示流程够用。

---

### 3. FPA 启动无效步数缩短

FPA 在以下两个条件同时满足后才开始输出：
- `MotionContext.State == Straight` 且 `Confidence >= ContextConfidenceThreshold`
- `PD.IsValid == true`（需 PD 稳定性 ≥ `StabilityThreshold` 且已积累 ≥ `StabilityWindow` 步）

加速方法：

```csharp
// ProgressionDirEstimator.cs
public int   StabilityWindow    { get; set; } = 3;    // 原 10，减少预热步数
public float StabilityThreshold { get; set; } = 0.65f; // 原 0.85

// FpaEngine.cs
public int MinStanceFramesForSettle { get; set; } = 5;  // 原 10，落地 50ms 后即输出
```

同时在 `PipelineParams`（UI 可调）：
```csharp
public float PdStabilityThreshold  { get; set; } = 0.5f; // 原 0.7
public float PdConfidenceThreshold { get; set; } = 0.5f; // 原 0.7
```

---

### 4. 转弯后恢复直线行走加速

恢复路径：`Turning → ReacquiringPd → (ConfirmStraight) → Straight`

`ConfirmStraight()` 触发条件：
- PD 稳定性 ≥ `StabilityThreshold`
- 已积累步数 ≥ `MinStepsBeforeValid`

调整：

```csharp
// MotionContextDetector.cs
public int StraightConfirmFrames { get; set; } = 10;  // 原 20，更快退出 Turning

// ProgressionDirEstimator.cs
public int   MinStepsBeforeValid { get; set; } = 1;    // 原 2，1 步即可尝试恢复
public int   StabilityWindow     { get; set; } = 3;    // 原 10
public float StabilityThreshold  { get; set; } = 0.65f; // 原 0.85
```

---

### 5. 站立相响应加速

```csharp
// GaitEventDetector.cs
public int   MinStanceFrames        { get; set; } = 3;    // 原 5，30ms 防抖
public int   WalkingWindowFrames    { get; set; } = 100;  // 原 300，1s 识别行走
```

---

## 修改位置汇总

| 需要代码改 | 文件 |
|-----------|------|
| 标定速度 | `Pipeline/CalibrationProcessor.cs` |
| FPA 启动、转弯恢复 | `Pipeline/ProgressionDirEstimator.cs` |
| 转弯退出速度 | `Pipeline/MotionContextDetector.cs` |
| 站立相响应 | `Pipeline/GaitEventDetector.cs` |
| FPA 落地稳定帧数 | `Pipeline/FpaEngine.cs` |

| UI 实时可调（`PipelineParams`） | 对应参数 |
|--------------------------------|---------|
| Baseline 步数 | `MinBaselineSteps` |
| FPA PD 稳定性门限 | `PdStabilityThreshold` |
| FPA 置信度门限 | `PdConfidenceThreshold` |
| 足部倾斜角门限 | `StanceFootPitchThreshold` |

---

## 注意事项

1. **`StanceFootPitchThreshold`** 已在代码中修为 `0.35f`（20°），`PipelineParams` 默认值同步已修正，无需再改。
2. `StabilityWindow` 和 `StabilityThreshold` 目前**不在** `PipelineParams` 中，需要直接改 `ProgressionDirEstimator.cs`。如需 UI 实时调整，可扩展 `PipelineParams` 并在 `GaitPipeline.SyncParams()` 中推入。
3. `StaticCollectFrames` 等标定参数同样不在 `PipelineParams` 中，需改源码。
4. Demo 结束后建议恢复默认值或另建 Demo 配置分支，避免影响正式精度测试。
