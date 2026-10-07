# 数据模型 (Data Models)

本模块不含业务逻辑，只是贯穿其他模块的数据结构 / DTO 定义，分布在两个目录：
`IMUMoCap/Model/`（偏 UI 绑定 / 设备层的旧模型）和 `IMUMoCap/Pipeline/Models/`（步态分析流水线的核心数据契约，被 [Gait Pipeline 模块](02-gait-pipeline.md) 产出和消费）。

## 一、设备 / IMU 相关模型（`IMUMoCap/Model/`）

### `DeviceModel.cs` —— `IMUMoCap.DeviceModel`
`INotifyPropertyChanged` 实现的属性包，字段为 `DeviceName`、`XsTime`、`X/Y/Z`（三轴数值）、`packetId`、`Angle`/`AngleXZ`/`AngleYZ`。命名风格（`x`/`y`/`z` 私有字段配合 `X`/`Y`/`Z` 公开属性）和用途（早期按设备直接绑定 UI 的角度展示）与下面的 `ImuSampleFrame` 明显是两套不同时期的模型。

### `IMUData.cs` —— `IMUMoCap.Model.IMUData` / `RecordedData`
- `IMUData`：`PackageId` + 一堆松散字段（`x/y/z/w`、`X/Y/Z`、`aX/aY/aZ`、`Angle`、`Q1`~`Q6` 六个 `Quaternion`、固定长度 8 的 `RecordedData[] RecordedDatas`）。字段命名重复且含义不完全自解释（`Q1..Q6` 用途在类型定义里看不出来）。
- `RecordedData`：`PackageId` + `Quaternion` + 三个 `Vector3`（`Accelerate`/`Orientation`/`AHRS`/`MadgwickAHRS`，共 4 个，注意有两个是姿态角输出，命名上区分"AHRS"和"MadgwickAHRS"两种算法结果）。
- **观察**：这两个类型只被 `MainWindow.xaml.cs`、`VM/MainPageVM.cs`、`AHRS/CvsUntil.cs` 引用，字段结构与 `Pipeline/Models` 下的新模型（`ImuSampleFrame` 等）不重叠、不复用，看起来是 Pipeline 重构之前的旧数据路径，与当前 `GaitPipeline` 的主数据流（见 §二）并行存在。文档任务范围内不做删改，仅作记录，供后续判断这条路径是否仍在被实际使用。

### `ImuFrameModels.cs` —— `IMUMoCap.ImuSampleFrame` / `ImuFrameBundle`
这是当前 Pipeline 主数据流的**入口模型**，由 `ImuFrameCollector` 从 Xsens `XsDataPacket` 解析填充（详见 [Gait Pipeline 模块](02-gait-pipeline.md)）。

- `ImuSampleFrame`：单个 IMU 单帧的解析结果。`PacketId`/`PacketCounter`/`TimeSec` 用于跨传感器对齐；`DeviceId`/`Role`（`ImuRole`，定义在 `Enums/DeviceStatesEnum.cs`：`Pelvis`/`Left`/`Right`）标识来源；`StatusWord`/`Rssi` 是质量相关字段；`Quaternion`/`RateOfTurn`/`FreeAcceleration`/`Acceleration`/`MagneticField`/`DeltaQ`/`DeltaV` 是实际姿态与运动数据，每个都配一个 `Has*` 布尔标记该字段本次是否实际解析出来（可能因数据包类型不同而缺失某些字段）。
- `ImuFrameBundle`：以 `PacketId` 为键，把同一时刻 Pelvis/Left/Right 三个 `ImuSampleFrame` 装进一个字典（`Samples`），`IsComplete` 判断三者是否到齐，`Pelvis`/`LeftFoot`/`RightFoot` 是按角色取值的便捷属性。

### `ImuViewModel.cs` —— 顶层 `ImuViewModel`（无命名空间）
UI 绑定用的单个 IMU 槽位视图模型：`SlotName`（"IMU-1" 等展示名）、`DeviceId`、`Role`、`Quat`、`IsConnected`、`LastUpdateUtc`，`BindDevice(deviceId)` 完成设备绑定后触发属性通知。被 [Services 层](01-device-access.md) 管理的槽位（`ImuSlotRegistry`）和 UI 层共同使用，是设备接入层与 UI 层之间的桥接模型。

### `PipelineConfigModels.cs`
文件内容目前只有一行注释：
```csharp
// Pipeline config removed for the minimal Sensor Interface -> Frame Aggregation baseline.
```
即该文件当前是空壳，原有的 pipeline 配置模型已被移除（配置改由 `Pipeline/Models/PipelineParams.cs` 承担，见下）。

## 二、Pipeline / 实验相关模型（`IMUMoCap/Pipeline/Models/`）

这些类型构成了 `GaitPipeline` 各处理阶段之间传递数据的契约，全部是不可变或半不可变的 sealed 类。

### `PipelineModels.cs` —— 流水线逐帧/逐步的核心 DTO 集合
- `ValidFrame`：包装通过 `DataQualityGate` 校验的 `ImuFrameBundle`，只读透传 `Pelvis`/`LeftFoot`/`RightFoot`（非空断言 `!`，因为能构造出 `ValidFrame` 前提是 bundle 已 `IsComplete`）。
- `GaitEvent`：`LeftStance`/`RightStance`/`LeftSwing`/`RightSwing`/`IsWalking`，由 `GaitEventDetector` 逐帧输出。
- `ContextState`（枚举 `Straight`/`Turning`/`ReacquiringPd`）+ `MotionContext`（`State` + `Confidence` 0–1），由 `MotionContextDetector` 输出。
- `PdEstimate`：`DirectionRad`/`IsValid`/`Stability`（0–1），由 `ProgressionDirEstimator` 输出，行进方向估计结果。
- `FpaResult`：单步的最终结果，`Fpa_L/R`（度，正值 toe-out）、`OnTarget_L/R`、`Error_L/R`（FPA − Target）、`Tolerance_L/R`（无 baseline 时为 NaN）、`ContextConfidence`/`PdStability`（门控通过时的原始置信度快照）、`Quality`（"High"/"Marginal"）。
- `StepExclusionReason`（枚举 `Turning`/`ReacquiringPd`/`LowConfidence`/`PdInvalid`）：一步完整落地但因门控未过而未输出 `FpaResult` 时的排除原因。
- `FrameQualityReport`：`TotalFrames`/`CompletedFrames`/`PelvisGapFrames`/`LeftFootGapFrames`/`RightFootGapFrames`，`CompletionRate` 计算属性，由 `ImuFrameCollector` 生成的丢帧统计报告。

### `CalibrationProfile.cs`
标定结果：`PelvisRef`/`LeftFootRef`/`RightFootRef` 三个参考姿态四元数，由 `CalibrationProcessor` 产出，后续所有相对姿态计算都以此为零点。

### `BaselineProfile.cs`
基线阶段结束后的统计与个性化训练目标，由 `BaselineProcessor` 生成后只读：
- 统计量：`MeanFpa_L/R`、`SdFpa_L/R`、`Asymmetry`（左右均值之差绝对值）、`ValidSteps_L/R`。
- 训练目标：`Target_L/R` + `Direction_L/R`（`TrainingDirection` 枚举 `ToeIn`/`ToeOut`），由静态工厂 `Create(...)` 按规则计算——`mean > 10°` 判定习惯性 toe-out → `ToeIn` 方向、`Target = mean − k·SD`；否则 → `ToeOut` 方向、`Target = mean + k·SD`，`k` 固定为常量 `TargetK = 1.0f`。`ToleranceWMinDeg = 4f` 是容差下限常量；`ToleranceL/R(alpha)` 按 `w = max(w_min, alpha·SD)` 计算某个训练 block 的容差半宽（`alpha` 由调用方 `FpaEngine.ToleranceAlpha` 传入,不在本类存储)。
- 步数均衡性：`StepCountRatio`（左右有效步数 min/max）、`ImbalanceRatioThresholdUsed`（生成时生效的阈值,记录下来便于事后复核)、`StepCountImbalanceWarning`。
- `MinRequiredSteps`/`IsValid`：`Create(...)` 的 `minRequiredSteps` 参数默认值是 `20`，但注意实际调用方（`PipelineParams.MinBaselineSteps`,见下)当前生产配置传入的是 `100`，以调用处为准。

### `DiagnosticsRow.cs`
每帧诊断行，是 CSV 落盘（`ExperimentRecorder`，见 [数据记录模块](06-data-recording.md)）的行结构：三个 IMU 各自的 `Acc`/`Gyr`/`Quat`、`GaitEvent`（`LeftStance`/`RightStance`/`IsWalking`）、`MotionContext`（`MotionState`/`MotionConfidence`）、`PdEstimate`（`PdDirectionDeg`/`PdStability`/`PdIsValid`）、`FpaResult`（左右 `Fpa_Deg`/`Error`/`OnTarget` + `FpaQuality`）、实验标签 `Stage`/`Attempt`（由 `ExperimentRecorder` 落盘时设置，不是 pipeline 本身产出）。`CsvHeader` 静态属性和 `ToCsvRow()` 方法保证表头与数据顺序严格对应；FPA 相关数值为 `NaN` 时序列化为空字符串（表示当前帧无 FPA 输出）。

### `ConditionMeta.cs`
Condition（EF/IF）级别的元数据，落盘为 Meta JSON：
- `PauseEvent`：一次暂停的 `Stage`/`StartTime`/`EndTime`/`Reason`/`OperatorDecision`（"Continue"|"Redo"）。
- `StepExclusionStats`：某 stage 内 `Emitted` vs 各类排除计数（`ExcludedTurning`/`ExcludedReacquiring`/`ExcludedLowConfidence`/`ExcludedPdInvalid`），`TotalAttempted`/`TurningExclusionRate` 是计算属性（转身+方向重获取合并为"转身相关排除率"）。
- `StageTiming`：某 stage 的起止墙钟时间 + 扣除暂停后的 `ActiveSeconds`。
- `StageMarker`：`{Stage, TimestampUtc}`，Manipulation Check / NASA-TLX / IMI-PC 等纸面量表的审计时间戳（软件不渲染内容）。
- `ConditionMeta` 本体：`ParticipantId`/`Condition`/`OrderGroup` + `Baseline`（`BaselineProfile?`）+ 上述四类记录的容器（`PauseEvents` 列表、`FinalAttempt`/`StageStepStats`/`StageTimings` 字典、`StageMarkers` 列表）。

### `PipelineParams.cs`
实时可调参数容器，`GaitPipeline` 每次 `Process()` 调用都会读取并下发给各子处理器，修改立即生效：

| 属性 | 默认值 | 所属处理器 |
|---|---|---|
| `StaticGyroThreshold` | 0.3f | CalibrationProcessor |
| `StanceFreeAccThreshold` | 2.5f | GaitEventDetector |
| `StanceGyroThreshold` | 1.0f | GaitEventDetector |
| `StanceFootPitchThreshold` | 0.35f (rad) | GaitEventDetector |
| `PdConfidenceThreshold` | 0.7f | FpaEngine 门控 |
| `PdStabilityThreshold` | 0.7f | FpaEngine 门控 |
| `MinBaselineSteps` | 100 | BaselineProcessor |
| `BaselineImbalanceRatioThreshold` | 0.7f | BaselineProcessor |

（各阈值的算法含义见 [Gait Pipeline 模块](02-gait-pipeline.md) 对应小节；这里只列出数据结构层面的默认值。）此文件当前有未提交改动（`git status`），以上数值为读取时的当前代码状态。

## 与其他模块的关系

- `ImuSampleFrame`/`ImuFrameBundle` 由 [设备接入层](01-device-access.md) 的解析代码产出，是进入 [Gait Pipeline](02-gait-pipeline.md) 的第一站数据。
- `Pipeline/Models/` 下的类型全部服务于 Gait Pipeline 内部各阶段之间的传递，最终 `DiagnosticsRow`/`FpaResult`/`ConditionMeta` 被 [数据记录模块](06-data-recording.md) 落盘、被 [UI 层](04-ui-layer.md) 展示、被 [WebSocket 通信模块](05-websocket-protocol.md) 广播给 AR 客户端。
- `ImuViewModel` 同时被设备层（`ImuSlotRegistry`）写入和 UI 层读取，是两者之间的共享视图模型。
