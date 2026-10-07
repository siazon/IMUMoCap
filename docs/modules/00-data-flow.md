# 端到端数据流 (End-to-End Data Flow)

## 这篇文档解决什么问题

其余 8 篇模块文档是**按组件切**的（"这个类是干什么的"），本篇是**按时间线切**的（"一个数据包/一次广播从哪来、经过谁、到哪去"），两者正交、互补。这里不重复各组件内部的算法细节，只负责把散落在各模块文档"与其他模块的关系"小节里的衔接点串成完整链路，并标注每一跳发生在哪个线程、哪种触发频率。所有内容均来自对应模块文档已核实过的源码事实，未新增未经核实的信息。

系统里同时存在三种粒度的数据流，混在一起读容易搞混，本文分别叙述：

- **帧级**（100Hz，每个 IMU 数据包一次）
- **步级**（每次脚 stance→swing 转换，约 1-2 秒一次）
- **阶段级**（Baseline/Training1-3/Rest/Retention 等状态切换，几分钟到十几分钟一次）

## 一、主链路：IMU 硬件 → FPA 结果（帧级，100Hz）

```
[Xsens MTw 传感器硬件]
   │  (Xsens SDK 内部线程)
   ▼
MyMtwCallback.onLiveDataAvailable                                  ── 01-device-access
   │
   ▼
ImuDeviceManager.OnDataAvailable
   → ImuSlotRegistry.GetOrAssign(deviceId) 查槽位
   → 触发 DataPacketReceived 事件 (仍在 SDK 线程上，非 UI 线程)
   │
   ▼
MainWindow.OnDataPacket → OnXsensData(role, deviceId, packet)      ── 04-ui-layer
   │  （原始 XsDataPacket 进入本应用逻辑的唯一入口）
   ├─ 若正在录制原始样本：CsvUtil 写入 ImuSamples_*.csv（可选，见 06）
   ▼
ImuFrameCollector.Process → BuildImuSample → UpsertFrameBundle     ── 02-gait-pipeline
   │  （按 PacketId 聚合 Pelvis/LeftFoot/RightFoot 三个样本；
   │   三者未在 MaxPendingPacketLag=16 包内凑齐 → 计入丢帧统计，不再往下游传）
   ▼  ImuFrameBundle（IsComplete=true）
GaitPipeline.ProcessPacket → GaitPipeline.Process(bundle)          ── 02-gait-pipeline
   │  （标定未完成时，Process 直接 return，下面的组件都不会运行）
   ├─→ DataQualityGate.Evaluate        （坏帧丢弃）→ ValidFrame
   ├─→ CalibrationProcessor.Process    （标定态机）→ CalibrationProfile
   ├─→ GaitEventDetector.Detect        （逐脚 stance/swing）→ GaitEvent
   ├─→ MotionContextDetector.Detect    （转弯识别）→ MotionContext
   ├─→ ProgressionDirEstimator.Update  （行进方向融合）→ PdEstimate
   ├─→ FpaEngine.Process               （门控+结算）→ FpaResult?（可能为 null）
   └─→ 组装 DiagnosticsRow（无论 FPA 是否产出，每帧一行）→ 存入 20 分钟环形缓冲
   │
   ▼  事件广播（仍在原调用线程，即 SDK 回调线程，不是 UI 线程）
GaitPipeline 触发：OnFpaResult / OnDiagnosticsFrame / OnBaselineProgress /
                   OnCalibrationStateChanged / OnStepOutcome / OnTrainingBlockComplete
   │
   ▼
MainWindow 订阅这些事件，扇出到三个方向（见下文二、三、四）
```

**关键点**：从硬件回调到 `GaitPipeline` 处理，全程发生在 Xsens SDK 的内部线程上；`MainWindow` 订阅的所有 pipeline 事件也因此**不是**在 WPF UI 线程触发的——`04-ui-layer.md` 提到"设备接入层不做任何线程封送"，调用方需自行 `Dispatcher.Invoke` 编排回 UI 线程更新界面控件；日志和纯数据类的事件处理器则可以直接在原线程执行。

## 二、扇出分支 A：广播给 AR 前端（帧级 + 步级 + 阶段级）

```
GaitPipeline 事件 → MainWindow 组装消息 → WebSocketBroadcastServer.BroadcastJsonAsync
                                                    │ (纯 JSON 文本, fire-and-forget)
                                                    ▼
                                        每个已连接的 AR 客户端 socket
                                                    │ (ws://<PC-IP>:8765/ws/)
                                                    ▼
                                  FootWebSocketClient.ReceiveLoop（AR 端后台线程）
                                                    │ 塞进 ConcurrentQueue<Action>
                                                    ▼
                                  Unity 主线程 Update() 里取出执行
                                                    ▼
                                  FootHudController.OnStateChange/OnFpaUpdate/
                                                    OnLiveUpdate/OnStepProgress/OnRedo
                                                    ▼
                                        HUD 状态机 + 脚部图形旋转/变色/进度条渲染
```

触发时机与消息类型对照（详细字段见 `05-websocket-protocol.md`）：

| 数据来源（帧/步/阶段） | 消息类型 | 触发点 |
|---|---|---|
| `GaitPipeline` 阶段切换 | `state` | `MainWindow.BroadcastArState(...)`，各状态机方法（`AdvanceStage`/`EnterRest`/...）内调用 |
| 每帧诊断帧（仅 IF 条件 + training 阶段，10Hz 节流） | `live` | `OnDiagnosticsFrame` 回调内联广播 |
| 每步 `FpaResult`（仅 training 阶段） | `fpa` | `OnFpaResult` 回调 |
| baseline/retention 每步进度 | `stepProgress` | `OnBaselineProgress` / retention 每次结算 |
| 操作员选择 Redo | `redo` | `BtnPauseResume` → `PauseResumeDialog` → `BroadcastRedo` |

**反向流（AR → PC 命令，见下）与这条链路共用同一条 WebSocket 连接，但方向相反、触发方不同，不要和上面的广播流混淆。**

## 三、扇出分支 B：写入磁盘（帧级 + 阶段级）

```
GaitPipeline.OnDiagnosticsFrame（每帧一次）
   ▼
MainWindow → ExperimentRecorder.WriteRow(row, stage, attempt)      ── 06-data-recording
   ▼
同步 WriteLine 追加写入 Session CSV
   (Data/P{id}/P{id}_{EF|IF}_Session.csv，编译输出目录下，非源码树 IMUMoCap/Data/)

────────────────────────────────────────────────────────────────

阶段切换 / 暂停 / 审计标记按钮点击（阶段级、低频事件）
   ▼
MainWindow → ExperimentRecorder.SaveBaseline / BeginPause / EndPause /
                                MarkStageEvent / BeginStage / EndStage / FlushMeta
   ▼
整份重写 Meta JSON（Data/P{id}/P{id}_{EF|IF}_Meta.json）
```

`DiagnosticsRow`/`BaselineProfile`/`ConditionMeta` 等落盘的数据结构本身在 `03-data-models.md` 定义，`GaitPipeline`（`02`）产出，`ExperimentRecorder`（`06`）只负责序列化，不重新定义字段。

## 四、扇出分支 C：UI 自身刷新（帧级，非算法链路的一部分）

```
GaitPipeline 事件 / 60ms 定时器 tick
   ▼
MainWindow.DrawTimeline()        —— 读最近 600 帧 DiagnosticsRow 环形缓冲，手绘 6 车道时间线
MainWindow.RefreshChecklist()    —— 整体重算 Flow 面板 10 行状态（不是增量推送，见 04-ui-layer）
MainWindow.UpdateImuStatusIndicators() —— 连接指示灯
```

这条分支只影响 WPF 界面展示，不产生任何持久化或对外广播的数据，因此不与分支 A/B 冲突。

## 五、反向流：AR → PC 命令（低频，事件驱动）

```
参与者/操作员点击 AR 端确认按钮
   ▼
FootHudController → FootWebSocketClient.SendReadyForCalibrationAsync() / SendContinueTrainingAsync(fromBlock)
   ▼ (JSON: {"cmd": "..."})
WebSocketBroadcastServer.ReceiveLoopAsync → OnTextMessage 事件
   ▼
MainWindow.HandleWsMessage(id, json)                                ── 04-ui-layer / 05-websocket-protocol
   ├─ "ping"               → 单播回 pong
   ├─ "ReadyForCalibration"→ _pipeline.TriggerCalibration()（若未 armed 则仅记日志）
   └─ "continueTraining"   → HandleContinueTraining(fromBlock)
                              （校验 _inRest && fromBlock 匹配 && 最短时长已过，
                               通过才 AdvanceStage()；否则静默拒绝）
```

操作员在 PC 端直接点击 "Continue Rest" 按钮（`BtnContinueRest_Click`）会调用**同一个** `HandleContinueTraining`，是 AR 命令的等价降级路径——AR 掉线时实验仍可由操作员手动推进，两条触发路径最终汇合到同一段校验逻辑。

## 六、旁路：参数实时调整（UI → Pipeline，非主数据流方向）

```
操作员在 ParamsDialog 拖动滑块
   ▼
MainPageVM 属性 setter → PropertyChanged 事件
   ▼
MainWindow 订阅的 lambda → SyncParamToPipeline(propertyName)
   ▼
写入 GaitPipeline.Params（PipelineParams 实例）
   ▼
下一帧 GaitPipeline.Process 时各组件读取最新阈值
```

这是全系统里**唯一一处方向相反**的数据流（UI → 算法层），其余所有帧级数据都是"设备 → 算法 → UI/广播/持久化"单向流动。`ParamsDialog` 的 `DataContext` 直接复用 `MainWindow` 的 `MainPageVM` 实例，所以滑块变化不需要额外的事件转发就能触发上面的同步。

## 七、CSV 回放：一条绕开硬件的替代入口

```
操作员点击 BtnLoadData_Click，选择历史 ImuSamples_*.csv
   ▼
CsvUtil.ReadImuSamplesCsv → CsvUtil.GroupIntoBundles                ── 06-data-recording
   ▼  重建的 ImuFrameBundle 序列（可能不完整，跳过了 ImuFrameCollector 的实时聚合）
MainWindow 后台线程逐帧调用 _pipeline.Process(bundle)                ── 02-gait-pipeline
   （注意：调用的是 Process，不是 ProcessPacket——回放没有原始 XsDataPacket，
    从入口开始就绕开了本文档「一、主链路」里 ImuFrameCollector 之前的所有步骤）
   ▼
后续与「一、主链路」的 GaitPipeline.Process 之后完全相同：
   触发同一批事件 → 走向分支 A（广播）/ C（UI 刷新），
   但**不会**走分支 B（不写入正式 Session CSV/Meta JSON，属于离线诊断用途）
```

## 三种粒度速查表

| 粒度 | 典型频率 | 代表事件/方法 | 主要消费方 |
|---|---|---|---|
| 帧级 | 100Hz（源头），广播降到 10Hz | `OnDiagnosticsFrame`, `live` 消息 | Session CSV、AR 实时角度、自绘时间线 |
| 步级 | 约 0.5-2Hz（走路节奏） | `OnFpaResult`/`OnStepOutcome`, `fpa`/`stepProgress` 消息 | AR 反馈渲染、baseline/QoE 统计 |
| 阶段级 | 分钟到十几分钟一次 | `OnCalibrationStateChanged`/`OnTrainingBlockComplete`, `state`/`redo` 消息, Meta JSON 重写 | AR 状态机切换、Meta JSON、Flow checklist |

## 延伸阅读

- 各环节的算法细节 → [02-gait-pipeline.md](02-gait-pipeline.md)
- 广播消息的完整字段定义 → [05-websocket-protocol.md](05-websocket-protocol.md)
- AR 端如何解析并渲染 → [08-ar-frontend.md](08-ar-frontend.md)
- 落盘文件的具体格式 → [06-data-recording.md](06-data-recording.md)
- 完整实验协议时间线（业务背景） → `docs/PROJECT_CONTEXT.md`
