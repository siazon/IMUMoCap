# 数据记录与持久化 (Data Recording & Persistence)

## 模块概述

本模块负责把实验运行过程中的数据落盘，核心类是 `IMUMoCap/Methods/ExperimentRecorder.cs`。它按 **Participant × Condition（EF/IF）** 为单位管理一次连续录制：一个 condition 对应一个流式写入的 Session CSV（每帧 append，不在内存里攒整段），以及一个随阶段/事件推进不断重写的 Meta JSON。`IMUMoCap/Methods/CsvUtil.cs` 是另一个独立的 CSV 工具类，与 `ExperimentRecorder` 没有调用关系，用于原始 IMU 样本（`ImuSampleFrame`）的读写（供 CSV 回放功能读文件、以及导出原始采样数据用）。

## 输出路径与目录结构

`ExperimentRecorder.DataRoot` 固定为：

```
Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Data")
```

即**可执行文件所在目录**（编译输出目录）下的 `Data` 文件夹，而不是源码树里的 `IMUMoCap/Data/`（源码树下的 `IMUMoCap/Data/record.csv` 只是一个遗留文件，与运行时输出路径无关，构建产物目录未纳入版本控制，仓库里看不到实际录制过的 participant 子目录）。

每个 participant 一个子目录：`Data/P{participantId}/`，目录在 `StartCondition()` 时通过 `Directory.CreateDirectory` 创建（已存在则忽略）。

## 输出文件类型

### Session CSV — `P{participantId}_{condition}_Session.csv`

- 每次 `StartCondition(participantId, condition, orderGroup)` 调用时以 `append: false` 打开（**覆盖重建**，不是追加到旧文件），先写入一行 header：`DiagnosticsRow.CsvHeader`（该类型属于「数据模型」模块，此处不重复定义）。
- 之后每帧调用一次 `WriteRow(DiagnosticsRow row, string stage, int attempt)`：先把 `row.Stage`/`row.Attempt` 写回该行对象，再 `WriteLine(row.ToCsvRow())` 追加写入。写入是同步的、逐行 `WriteLine`，没有内部缓冲攒批，落盘时机完全跟随调用节奏（即 pipeline 每帧产出诊断行的频率，见「步态分析核心流水线」模块）。
- `SessionFileExists(participantId, condition)` 是静态方法，仅检查文件是否已存在，供 UI 层在开始录制前判断"是否会覆盖已有数据"并提示操作员。

### Meta JSON — `P{participantId}_{condition}_Meta.json`

- 内容类型是 `ConditionMeta`（定义于 `Pipeline/Models/ConditionMeta.cs`，属于「数据模型」模块），序列化时用 `JsonSerializerOptions { WriteIndented = true }`，每次 `SaveMeta()` 调用都是**整份重写**（`File.WriteAllText`），不是增量 patch。
- `ConditionMeta` 内包含：`Baseline`（个性化基线/目标）、`PauseEvents`（暂停记录列表）、`FinalAttempt`（各阶段最终生效的 attempt 号，支持 redo）、`StageStepStats`（各阶段 step 门控排除统计）、`StageTimings`（各阶段起止时间与扣除暂停后的活跃时长）、`StageMarkers`（纯审计时间戳标记列表）。这些类型的字段定义属于「数据模型」模块，此处只说明写入触发点。

### QoE CSV — 已预留路径，当前未实际写入

`CurrentQoeFilePath` 在 `StartCondition()` 中被赋值为 `P{participantId}_{condition}_QoE.csv`，但通读 `ExperimentRecorder.cs` 全文没有任何方法向这个路径写入内容——路径已经规划好，写入逻辑尚未实现。

## `ExperimentRecorder` 关键方法一览

| 方法 | 触发时机 / 作用 |
|---|---|
| `StartCondition(participantId, condition, orderGroup)` | 开始一个新的 condition 录制：先 `Close()` 关闭上一个 writer，再创建目录、打开新 Session CSV（覆盖）、初始化内存中的 `ConditionMeta` |
| `WriteRow(row, stage, attempt)` | 逐帧写 Session CSV 一行 |
| `SaveBaseline(profile)` | 把 `BaselineProfile` 写入 `_meta.Baseline` 并立即 `SaveMeta()` |
| `BeginPause(stage)` / `EndPause(operatorDecision, reason)` | 暂停开始只记 `StartTime`；暂停结束时才一次性补全 `EndTime`/`OperatorDecision`/`Reason` 并 append 进 `PauseEvents`——原因是"恢复时（Continue/Redo）才收集原因，而不是暂停发生的当下"，代码注释明确说明这一设计 |
| `MarkRedo(stage, newAttempt)` | 记录某阶段被重做后的新 attempt 号 |
| `MarkStageEvent(stage)` | **纯审计标记**：记录一个由研究助理（RA）在纸面上主持的量表/检查（Manipulation Check、NASA-TLX、IMI-PC 等）发生的 UTC 时间戳，软件本身不呈现这些量表内容，只留痕 |
| `BeginStage(stage)` / `EndStage(stage)` | 记录阶段起止墙钟时间；`EndStage` 会遍历 `PauseEvents` 中属于该 stage 且已结束的暂停，从总时长里扣除，得到 `ActiveSeconds` |
| `NoteStepOutcome(stage, emitted, reason)` | 每个 step 判定结果（输出或被排除，排除原因为 `Turning`/`ReacquiringPd`/`LowConfidence`/`PdInvalid`）调用一次，**只更新内存计数，不落盘**——注释说明高频调用不适合每次都写文件，落盘统一交给 `FlushMeta()` |
| `GetStageStats(stage)` | 读取某阶段当前的 `StepExclusionStats` |
| `FlushMeta()` | 把内存中的 `_meta`（含最新的 `StageStepStats`）落盘一次，供阶段切换/关闭时调用 |
| `Close()` / `Dispose()` | Flush 并释放 `_sessionWriter` |

## `CsvUtil.cs`

与 `ExperimentRecorder` 相互独立，服务于原始 IMU 样本层面的 CSV 读写：

- `ReadImuSamplesCsv(filePath)`：解析一份 `ImuSampleFrame` CSV（固定 23 列，`PacketId,Role,StatusWord,Qx..Qw,Gx..Gz,FreeAx..Az,Ax..Az,DQx..DQw,DVx..DVz`），每个可选字段组（四元数/角速度/加速度等）用是否为空字符串来判断该样本是否携带该数据（对应 `ImuSampleFrame` 上的 `HasXxx` 标志位）。用于 UI 层的 CSV 回放功能加载历史数据。
- `WriteImuSamplesCsv(filePath, samples)`：反向写出同样格式的 CSV，一次性用 `StringBuilder` 拼好再 `File.WriteAllText`（与 `ExperimentRecorder` 的逐行流式写入不同，这里是全量重写）。
- `GroupIntoBundles(samples)`：静态方法，按 `PacketId` 把展开的样本重新分组回 `ImuFrameBundle` 列表（含未集齐三个 IMU 的不完整 bundle），供回放时重建帧序列、交给 `DataQualityGate` 等下游组件处理。

## 与其他模块的关系

- **数据来源**：`DiagnosticsRow`（每帧诊断行）、`BaselineProfile`、`ConditionMeta` 及其内部子类型均由「步态分析核心流水线」产出或在「数据模型」模块中定义，本模块只负责序列化落盘，不重新定义这些结构。
- **调用方**：`ExperimentRecorder` 的所有方法均由 `MainWindow.xaml.cs`（见「WPF UI 层」模块）在会话/阶段状态机推进的各个节点上调用（开始录制、逐帧写入、暂停/恢复、阶段切换、审计标记按钮点击、结束录制）。
- **CSV 回放**：`CsvUtil` 读取的历史数据经 `GroupIntoBundles` 重建后，会重新喂给 `DataQualityGate`→…→`FpaEngine` 的完整 pipeline（见「步态分析核心流水线」模块），用于离线复现/调试。
