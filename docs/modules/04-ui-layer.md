# WPF UI 层 (UI Layer)

## 模块概述

本模块是 IMUMoCap WPF 桌面应用的界面与交互编排层：单一主窗口 `MainWindow` + 两个弹窗（`ParamsDialog` 参数调整、`PauseResumeDialog` 暂停恢复）+ 一个 ViewModel（`MainPageVM`）+ 一个 UI 专用模型（`ChecklistItem`，驱动 Flow 一览面板）。它是整个应用的"业务流程总控台"：设备连接、静态标定触发、实验阶段状态机（Baseline → Training1~3 → Rest/MC → Retention → Ended）、数据记录起止、WebSocket 广播时机、UI 实时刷新（自绘时间线、Flow checklist）全部在这里编排调用。

截至当前代码：`MainWindow.xaml.cs` 1975 行，是事实上的"上帝类"；`MainWindow.xaml` 379 行；`VM/MainPageVM.cs` 525 行；其余文件均较小。

## 架构特点：不是标准 MVVM

`MainPageVM`（`IMUMoCap/VM/MainPageVM.cs`）实现 `INotifyPropertyChanged`，但**没有任何 `ICommand`/`RelayCommand`**——它纯粹是一个属性包（property bag），每个属性的 setter 只做字段赋值 + `OnPropertyChanged`。XAML 里的按钮全部用传统的 `Click="BtnXxx_Click"` 事件绑定，事件处理方法直接写在 `MainWindow.xaml.cs` 的 code-behind 里，由 code-behind 直接调用 `GaitPipeline`、`ExperimentRecorder`、`WebSocketBroadcastServer`、`ImuDeviceManager` 等下游对象。也就是说 UI 数据绑定（用于展示）和业务逻辑（用于响应用户操作）是分离的两条路径，MainWindow 既是 View 的 code-behind 又是事实上的 Controller/Presenter。这是当前代码的既有风格，不是缺陷或遗漏。

`MainPageVM` 与 `MainWindow` 的耦合点：构造函数里 `this.DataContext = _content`（`_content` 是 `MainPageVM` 实例），随后 `_content.PropertyChanged += (_, e) => SyncParamToPipeline(e.PropertyName)`——参数滑块类属性变化时反向同步进 `GaitPipeline.Params`（唯一一处 VM→Model 的主动同步逻辑，其余全是 MainWindow→VM 的单向展示更新）。

## MainWindow 职责分解

### 1. 构造函数：装配与事件订阅

`MainWindow()` 一次性完成：
- 启动 `WebSocketBroadcastServer`（监听 `http://+:8765/ws/`），订阅其 `OnTextMessage`/`OnClientConnected`/`OnClientDisconnected`
- 硬编码构造 3 个 `ImuViewModel`（Pelvis `0x00B43CAB`、Left `0x10b41913`、Right `0x10B41904`），装入 `ImuSlotRegistry`，再用它构造 `ImuDeviceManager` 并订阅其 6 个事件（`Log`/`StateChanged`/`UpdateRatesAvailable`/`MtwConnected`/`MtwDisconnected`/`DataPacketReceived`/`BatteryLevelChanged`）
- 按 `ChecklistDefs`（10 项静态定义，见下文）填充 `_content.FlowChecklist`
- 订阅 `GaitPipeline` 的 7 个事件（`OnLog`/`OnCalibrationStateChanged`/`OnBaselineProgress`/`OnBaselineCompleted`/`OnFpaResult`/`OnDiagnosticsFrame`/`OnStepOutcome`/`OnTrainingBlockComplete`）
- 启动一个 60ms 间隔的 `_timelineTimer`：每 tick 重绘时间线 (`DrawTimeline`)、刷新阶段计时显示、刷新丢包计数、检查训练/Retention 的超时兜底、调用 `RefreshChecklist()`
- 加载已保存的 pipeline 参数（`LoadParamsIfExists`），启动设备扫描（`StartScanAsync`）

### 2. 设备连接与数据回调

`OnMtwConnected`/`OnMtwDisconnected`/`OnBattery`/`OnDataPacket` 响应 `ImuDeviceManager` 的事件：更新 `_slotRegistry` 中对应 IMU 的 `IsConnected`、刷新连接指示灯（`UpdateImuStatusIndicators`）、三个 IMU 全部连接后自动调用 `_deviceManager.StartMeasurement(desiredRate)`。`OnDataPacket` 取出数据快照后调用 `OnXsensData(role, deviceId, packet)`——这是原始 Xsens 包进入本应用逻辑的唯一入口，内部依次：替身录制（若非回放且未暂停）→ `_imuFrameCollector.Process(...)` 生成单传感器样本 → `_pipeline.ProcessPacket(...)` 送入步态分析流水线。

### 3. 实验流程状态机（会话/阶段管理）

这是 MainWindow 最核心的职责类别，围绕静态字段 `ConditionStages = {Baseline, Training1, Training2, Training3, Retention}` 展开：

- **`BtnStartCondition_Click`**：校验 Participant ID / 设备测量中，调用 `_recorder.StartCondition(...)` 开新的 session 文件，重置所有阶段/标记状态（`_stageAttempt`、5 个"是否已点击"标记 `_mc1Marked`/`_mc3Marked`/`_nasaTlx1Marked`/`_imiPcMarked`/`_nasaTlx2Marked`、`_restBlockJustFinished`），把 `_currentStage` 设为 `"Baseline"`，广播 `state=armed` 通知 AR 端可以显示"准备标定"按钮。
- **`BtnStartCal_Click`**：重置 pipeline/缓冲区，调用 `_pipeline.TriggerCalibration()`。
- **`OnCalibrationStateChanged`**：响应 `CalibrationProcessor` 的状态变化，标定完成后自动 `_pipeline.StartBaseline()` 并广播 `state=baseline`，同时启动 `StartBaselineTimeoutTimer()`（10 分钟兜底强制结束，见 `BaselineMaxDuration`）。
- **`HandleBaselineCompleted`**：baseline 步数达标后，调用 `_pipeline.StartTraining()`，若已有 recording 在跑则通过 `AdvanceStage()` 正式推进到 `Training1`；否则只在本地展示目标角（无 recording 的"仅标定测试"场景）。
- **`AdvanceStage()`**：`ConditionStages` 数组顺序推进一格，结束/开始对应 stage 的记录（`_recorder.EndStage`/`BeginStage`），重启阶段计时；若进入 Training1/2/3 则调用 `_pipeline.SetTrainingBlock(n)`（容差随 block 收紧）并广播 `state=training`；若进入 Retention 则设 `_isRetention=true` 并广播 `state=retention`（此后 AR 端不再收到 fpa 反馈）。
- **`EndTrainingBlock()`**：由 `_pipeline.OnTrainingBlockComplete` 事件触发（100 步/脚达标）或 60ms 定时器超时兜底触发。Training1/Training3 后进入 `EnterRest(...)`（Rest+MC 窗口，Block1 用 120s / `RestDurationSec`，Block3 用 300s / `PostBlock3RestDurationSec`）；Training2 直接 `AdvanceStage()`（Block2→Block3 之间按设计无 Rest 窗口）。用 `_trainingBlockEnding` 标志防止后台线程的步数达标事件与 UI 线程定时器超时事件在同一个 block 上重复触发。
- **`EnterRest`/`HandleContinueTraining`/`BtnContinueRest_Click`**：Rest 窗口不自动跳过——必须等 AR 端发 `continueTraining` 命令或操作员点击 Continue 按钮，且服务端会用 `_stageStopwatch` 校验最短时长真的走完（不信任客户端自己的计时）。
- **`CompleteRetention`/`EndFlow`**：Retention 达到 100 步/脚（或超时兜底）后仅停止计步，不自动关闭 recording；必须操作员标记 NASA-TLX Admin2 并点击 "End Recording"（`BtnEndCondition_Click` → `MissingExpectedMarkers()` 非阻断检查缺失的手动标记 → `EndFlow`）才真正落盘关闭。
- **`BtnPauseResume_Click`**：暂停时冻结数据处理（`_content.IsPaused` 由 `OnXsensData` 检查后直接 return），恢复时弹出 `PauseResumeDialog` 强制填写原因，可选择"Redo"（阶段 attempt 计数 +1，广播 `redo` 事件）或"Continue"。

### 4. WebSocket 广播调用点

MainWindow 直接持有 `WebSocketBroadcastServer` 实例并在状态机推进的各个节点调用广播方法：`BroadcastArState`（`state` 消息，统一承载阶段转换）、`OnDiagnosticsFrame` 回调内联的 `live` 广播（10Hz 节流，仅 IF 条件+Training 阶段发送原始脚部朝向角）、`OnFpaResult`（`fpa` 消息，仅 Training 阶段）、`BroadcastStepProgress`（`stepProgress`，Baseline/Retention 用）、`BroadcastRedo`（`redo`）、`HandleWsMessage` 里对 `ping`/`ReadyForCalibration`/`continueTraining` 命令的处理与 `pong` 单播回复。所有广播内容与协议字段细节属于「WebSocket 通信」模块范畴，本模块只关注**调用时机**。另有 `LogArEvent` 把每条广播记录写入独立的 AR event log（`_content.ArEventLog`），供操作员事后核对算法输出与 AR 实际呈现是否一致。

### 5. UI 实时刷新

- **`DrawTimeline()`**：用 `DrawingVisual` + `RenderTargetBitmap` 手绘一个 6 车道时间线（PdIsValid / L.Stance / R.Stance / IsWalking / MotionContext / FPA gate 原因），数据源是最近 600 帧的 `DiagnosticsRow` 环形缓冲（`_timelineBuffer`），由 60ms 定时器和窗口 `SizeChanged` 共同触发重绘。
- **`RefreshChecklist()`/`SetChecklist`/`SetChecklistMarker`**：Flow 一览面板（10 行，见 `ChecklistDefs`：Baseline/Training1/MC(Block1)/Training2/Training3/MC(Block3)/NASA-TLX#1/IMI-PC/Retention/NASA-TLX#2）。**整体重算**而非增量推送——每次直接读取 `_restBlockJustFinished`/`_retentionComplete`/5 个手动标记布尔值/`_pipeline.BaselineProfile` 等原始状态推导出每行的 `Pending`/`Done`/`Missing`，保证永不漂移。3 个手动审计标记行内嵌 "Mark" 按钮，统一走 `BtnMarkFlowItem_Click`（从被点击行的 `DataContext`，即绑定的 `ChecklistItem.MarkerStage`，分发到 `_recorder.MarkStageEvent(stage)`）。
- **`UpdateImuStatusIndicators`**：3 个 IMU 连接指示灯（Pelvis/Left/Right 圆点变色）。
- **`BtnLoadData_Click`**：CSV 回放功能，后台线程解析 `ImuSamples_*.csv` 并重建 `ImuFrameBundle` 序列，逐帧调用 `_pipeline.Process(bundle)`（不走 `ProcessPacket`，因为回放没有原始 `XsDataPacket`），支持 0.2×/1×/5×/Max 四档速度（`BtnReplaySlower_Click`/`BtnReplayFaster_Click`/`BtnReplayPause_Click`）。回放期间会自行 arm+trigger 校准（因为没有操作员点击）。回放结束后调用 `BuildReplayTimeline(...)` 生成一份 ASCII 表格日志（转弯/站立/FPA有效窗口等事件时间轴），纯离线诊断用途，不写入正式 session 文件。
- **`Button_SaveData`/`BtnSaveDiagnostics_Click`/`BtnOpenDataFolder_Click`**：手动导出 IMU 原始样本 CSV / 诊断行 CSV，或打开当前 participant/condition 的数据文件夹。

## 其他文件

- **`ParamsDialog.xaml`/`.xaml.cs`**：非模态参数调整弹窗，`DataContext` 直接复用 `MainWindow` 的 `_content`（`MainPageVM`），所以滑块改动通过 VM 的 `PropertyChanged` 直接联动 `SyncParamToPipeline`。自身只有一个 `BtnSaveParams_Click`，转发 `SaveParamsClicked` 事件给 `MainWindow` 处理实际的 JSON 落盘（`pipeline_params.json`）。同一时刻只允许一个实例（`MainWindow._paramsDialog` 判空短路 + `Activate()`）。
- **`PauseResumeDialog.xaml.cs`**：暂停恢复弹窗，强制要求填写暂停原因（`ValidateReason`），提供 Continue / Redo 两个出口，通过 `DialogResult`/`Reason`/`Redo` 三个属性把结果带回调用方。
- **`VM/MainPageVM.cs`**：约 50+ 个可绑定属性，覆盖设备状态、IMU 数据展示、FPA 大数字卡（含目标/误差/背景色三色态）、pipeline 可调参数（`StaticGyroThreshold`/`StanceFreeAccThreshold`/`StanceGyroThreshold`/`PdConfidenceThreshold`/`PdStabilityThreshold`/`MinBaselineSteps`/`BaselineImbalanceRatioThreshold`）、实验会话展示字段（`CurrentStageLabel`/`StageElapsedDisplay`/`BlockStepsDisplay` 等）、`FlowChecklist` 集合。另有一个孤立的 `DataReceived(RecordedData, int)` 方法（按 `PackageId` 归并到 `IMUData.RecordedDatas[dataSource]`）——**在 MainWindow 中未见调用**，疑似早期原型遗留代码。
- **`Model/ChecklistItem.cs`**：Flow 面板的行模型，`Label`（展示名）+`MarkerStage`（可选，`MarkStageEvent` 用的阶段字符串）+`Status`（`Pending`/`Done`/`Missing` 三态，驱动 `DotBrush` 颜色）+`CanMark`（Mark 按钮是否可点）。`IsMarkable = MarkerStage != null` 决定该行是否渲染按钮。
- **`App.xaml.cs`**：应用入口，除标准 `Application` 分部类声明外无自定义逻辑。

## 与其他模块的关系

- **← 设备接入层**：MainWindow 持有 `ImuDeviceManager`/`ImuSlotRegistry` 实例，订阅其事件驱动连接状态展示和自动开始测量。
- **← 步态分析核心流水线**：MainWindow 持有唯一的 `GaitPipeline` 实例，是流水线所有输出事件（标定状态、baseline 进度、FPA 结果、诊断帧、步骤结果、训练 block 完成）的唯一消费方与状态机驱动者；也是唯一调用 `_pipeline.ProcessPacket`/`Process`/`TriggerCalibration`/`StartBaseline`/`StartTraining`/`SetTrainingBlock`/`Reset` 等推进方法的地方。
- **→ WebSocket 通信**：MainWindow 持有 `WebSocketBroadcastServer` 实例，在状态机的每个关键节点调用广播方法；具体消息结构见「WebSocket 通信协议」模块。
- **→ 数据记录与持久化**：MainWindow 持有 `ExperimentRecorder` 实例，在阶段推进/暂停/标记的各处调用其方法写 CSV/Meta JSON；具体持久化格式见「数据记录与持久化」模块。
- **→ 数据模型**：`ChecklistItem` 属于本模块，但 `DeviceModel`/`IMUData`/`ImuFrameModels`/`PipelineConfigModels` 等由「数据模型」模块统一说明，本模块只描述它们如何被 UI 使用/展示。
