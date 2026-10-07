# Demo 模式调参说明

> 目的：快速展示流程，优先响应速度，放宽精度要求。
> 以下参数均可在代码中直接修改默认值，部分参数通过 `PipelineParams` 可在 UI 实时调整。
>
> **本次更新**：核对了一遍当前代码里的实际默认值——`MinBaselineSteps`/`MinValidSteps` 的研究
> 默认值实际是 **100 步**（不是本文之前写的 20），`MinStanceFrames` 当前默认已经是 **2 帧**
> （不是 5，已经比本文之前的"Demo 推荐值 3"还快，Demo 场景不需要再改）。另外 `FpaEngine` 的落地
> 结算逻辑已经从"固定帧数"改成"数据驱动结算"，`MinStanceFramesForSettle` 这个参数**已经不存在
> 了**，见下面第 3 节的重写说明。

---

## 一览表

| 参数 | 所在类 | 当前默认值 | Demo 推荐值 | 效果 |
|------|--------|--------|------------|------|
| `StaticCollectFrames` | `CalibrationProcessor` | 300（3 s） | **50（0.5 s）** | 标定采集时间大幅缩短 |
| `StaticRequiredFrames` | `CalibrationProcessor` | 30（0.3 s） | **10（0.1 s）** | 连续静止门槛降低，更容易触发采集 |
| `StaticTimeoutFrames` | `CalibrationProcessor` | 1000（10 s） | **300（3 s）** | 标定超时提前，尽快失败重试 |
| `MinValidSteps` / `MinBaselineSteps` | `BaselineProcessor` / `PipelineParams` | **100 步** | **5 步** | Baseline 所需步数 |
| `StabilityWindow` | `ProgressionDirEstimator` | 10 步 | **3 步** | PD 稳定性计算窗口，影响 FPA 启动速度和转弯恢复速度 |
| `StabilityThreshold` | `ProgressionDirEstimator` | 0.85 | **0.65** | PD 稳定性阈值，同时决定 FPA 能否输出、转弯后何时恢复直线 |
| `MinStepsBeforeValid` | `ProgressionDirEstimator` | 2 步 | **1 步** | ReacquiringPd → Straight 最少需要的步数（注意：实测数据显示这一步几乎总是瓶颈所在，见下方"转弯恢复"一节） |
| `StraightConfirmFrames` | `MotionContextDetector` | 20 帧（0.2 s） | **10 帧（0.1 s）** | 转弯结束后退出 Turning 所需帧数，加快进入 ReacquiringPd |
| `MinStanceFrames` | `GaitEventDetector` | **2 帧**（已是较快值） | 保持 **2 帧**，不建议再降 | 站立相防抖帧数；再降低会显著增加"一次落地被拆成多次假 stance"的风险（详见下方注意事项） |
| `MinStanceFramesFloor` / `SettleGyroThreshold` / `SettleQuietFrames` / `MaxStanceFramesForSettle` | `FpaEngine` | 2 帧 / 0.35 rad/s / 3 帧 / 20 帧(200ms) | 见下方第 3 节 | 数据驱动的落地结算逻辑，已替代原来单一的 `MinStanceFramesForSettle` |
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
// PipelineParams.cs / BaselineProcessor.cs
public int MinBaselineSteps { get; set; } = 5;   // 当前默认 100，不是之前文档写的 20
```

**代价：** 5 步统计均值方差不够稳定，目标角度误差可能偏大。Demo 展示流程够用。

---

### 3. FPA 落地结算加速（已改为数据驱动，非固定帧数）

`FpaEngine` 里原来"落地后固定攒 N 帧就结算"的 `MinStanceFramesForSettle` 已经被替换成一套
数据驱动的结算逻辑——不再是数帧数，而是看**该脚角速度是否真的降下来了**：

```csharp
// FpaEngine.cs（当前代码，非 Demo 特化）
public int   MinStanceFramesFloor    { get; set; } = 2;     // 落地后至少攒够这么多帧才可能结算
public float SettleGyroThreshold     { get; set; } = 0.35f; // rad/s，角速度连续低于此值才算"真安静"
public int   SettleQuietFrames       { get; set; } = 3;     // 需要连续多少帧安静
public int   MaxStanceFramesForSettle{ get; set; } = 20;    // 硬上限（200ms），防止落地冲击大的步子一直等不到
```

`0.35 rad/s` 这个阈值是在两份真实录制数据上验证过的（`docs/task12_settle_latency_risk_check.py`、
`docs/task13_settle_latency_risk_check_diagnostics.py`）——`0.2` 时哪怕正常走路(Straight 状态)也
有 4-8% 的落地会因为一直凑不够 3 帧连续安静而被**静默丢弃**（不产生任何 FpaResult，也不算作排除
统计），`0.35` 基本消除了这个风险。**Demo 场景如果想再快，不建议简单调高 `SettleGyroThreshold`
（会重新引入上面的丢弃风险，而且没有重新跑数据验证过）**；更安全的 Demo 加速点是调低
`MaxStanceFramesForSettle`（比如 10 帧=100ms），只压缩"落地冲击特别大、一直不安静"这类少数步子
的等待上限，不影响正常步子的结算逻辑本身。

```csharp
// Demo 建议（未经数据验证，仅工程上合理，展示前建议实测走两步看看效果）
public int MaxStanceFramesForSettle { get; set; } = 10;  // 原 20，100ms 硬上限
```

---

### 4. FPA 启动无效步数缩短

FPA 在以下两个条件同时满足后才开始输出：
- `MotionContext.State == Straight` 且 `Confidence >= ContextConfidenceThreshold`
- `PD.IsValid == true`（需 PD 稳定性 ≥ `StabilityThreshold` 且已积累 ≥ `StabilityWindow` 步）

加速方法：

```csharp
// ProgressionDirEstimator.cs
public int   StabilityWindow    { get; set; } = 3;    // 原 10，减少预热步数
public float StabilityThreshold { get; set; } = 0.65f; // 原 0.85
```

同时在 `PipelineParams`（UI 可调）：
```csharp
public float PdStabilityThreshold  { get; set; } = 0.5f; // 原 0.7
public float PdConfidenceThreshold { get; set; } = 0.5f; // 原 0.7
```

---

### 5. 转弯后恢复直线行走加速

恢复路径：`Turning → ReacquiringPd → (ConfirmStraight) → Straight`

`ConfirmStraight()` 触发条件：
- PD 稳定性 ≥ `StabilityThreshold`
- 已积累步数 ≥ `MinStepsBeforeValid`
- 骨盆瞬时朝向与融合方向夹角 ≤ `PelvisAgreementThreshold_deg`（15°）

**实测发现**（`docs/task14_reacquisition_timing.py`，17 次真实转身）：当前恢复耗时中位数
**0.86s**（0.74~1.07s），而且**每一次**都是被 `MinStepsBeforeValid=2` 卡住——第 1 步时
`stability` 恒为 0（样本数<2，方差算不出来，代码里是硬编码返回 0，不是真的不稳），但角度一致性
(`agree_deg`) 在第 1 步就已经远小于 15° 阈值。如果放宽到 1 步（同时把 stability 检查在样本数=1
时的强制拒绝去掉，只留 `agree_deg` 这道真实的一致性检查），17 次里全部能在第 1 步就通过，
中位数能降到 **0.37s**——但这个改动没有在生产代码里落地（讨论过，用户决定"先不动"），如果
Demo 场景想要这个效果，需要同时改：

```csharp
// MotionContextDetector.cs
public int StraightConfirmFrames { get; set; } = 10;  // 原 20，更快退出 Turning

// ProgressionDirEstimator.cs
public int   MinStepsBeforeValid { get; set; } = 1;    // 原 2，1 步即可尝试恢复
public int   StabilityWindow     { get; set; } = 3;    // 原 10
public float StabilityThreshold  { get; set; } = 0.65f; // 原 0.85
```

**注意**：`MinStepsBeforeValid=1` 只在这一份录制数据上验证过（一个人、比较规整的转身动作）；
生产环境如果要采用，建议先看 `docs/task14_reacquisition_timing.py` 的分析方法，用更多参与者/
更多转身数据复核一遍 `agree_deg` 在第 1 步是否总能达标，Demo 场景不需要这么谨慎，可以直接用。

---

### 6. 站立相响应加速

```csharp
// GaitEventDetector.cs
public int   MinStanceFrames        { get; set; } = 2;    // 当前默认已经是 2（不是原来的 5），Demo 不建议再降
public int   WalkingWindowFrames    { get; set; } = 100;  // 原 300，1s 识别行走
```

**`MinStanceFrames` 为什么不建议再降**：这个防抖帧数如果太短，容易把摆动相中间的瞬时低值
误判成"已落地"，把一次真实落地拆成好几次"假 stance→swing→stance"——每一段假 stance 现在都
可能单独触发结算（尤其是配合上面第 3 节较松的 `MaxStanceFramesForSettle`），导致**同一次物理
落地被重复计数、AR 反馈对同一步闪烁多次**。当前 2 帧已经是研究团队权衡过后采用的值，Demo 场景
没有必要为了再快一点点去冒这个风险。

---

## 修改位置汇总

| 需要代码改 | 文件 |
|-----------|------|
| 标定速度 | `Pipeline/CalibrationProcessor.cs` |
| FPA 启动、转弯恢复 | `Pipeline/ProgressionDirEstimator.cs` |
| 转弯退出速度 | `Pipeline/MotionContextDetector.cs` |
| 站立相响应 | `Pipeline/GaitEventDetector.cs` |
| FPA 落地结算（数据驱动，见第 3 节） | `Pipeline/FpaEngine.cs` |

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
4. `FpaEngine.MinStanceFramesForSettle` **已不存在**——改动这块前先看第 3 节，不要按旧文档直接找这个字段名。
5. `MinStanceFrames`（`GaitEventDetector`）当前研究默认值是 2 帧，比本文历史版本记录的 5 帧更激进；这是研究团队自己改的，不是 Demo 专用调整，Demo 不需要在此基础上再降。
6. Demo 结束后建议恢复默认值或另建 Demo 配置分支，避免影响正式精度测试。
