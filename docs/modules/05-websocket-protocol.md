# WebSocket 通信协议 (WebSocket Protocol)

## 模块概述

PC 端（`IMUMoCap/` WPF 应用）内置一个轻量 WebSocket 广播服务端，负责把训练流程状态、逐帧朝向、逐步 FPA 反馈等实时推送给 AR 头显客户端（`IMUARFPA/`），并接收 AR 端发回的少量控制命令。

- 服务端实现：`IMUMoCap/Methods/WebSocketBroadcastServer.cs`（纯传输层，只管连接管理和 JSON 收发，不含任何业务语义）
- 业务侧消息组装与调用：全部在 `IMUMoCap/MainWindow.xaml.cs` 中
- 监听地址：`http://+:8765/ws/`（`HttpListener` 前缀，客户端以 `ws://` 连接），在 `MainWindow` 构造函数中 `_wsServer.Start(...)` 启动，日志固定打印 `ws://192.168.137.1:8765/ws/`（硬编码的 ICS 热点 IP，仅用于日志提示，实际监听前缀是 `+`，不限定网卡）
- PC 是服务端，AR 头显是客户端；协议为纯文本 JSON，无鉴权，假设运行在可信局域网内

## 服务端传输层（`WebSocketBroadcastServer`）

- 基于 `HttpListener` + `System.Net.WebSockets`，`AcceptLoopAsync` 循环 `GetContextAsync()`，非 WebSocket 请求直接 400 拒绝，握手成功后为每个连接分配 `Guid` 并存入 `_clients` 字典（`lock (_gate)` 保护）。
- `BroadcastJsonAsync(payload)`：序列化后向所有当前连接的客户端发送；发送失败或状态非 `Open` 的客户端会被就地移除（`RemoveClient`），不重试。
- `SendJsonAsync(id, payload)`：只发给单个客户端，用于连接/重连时的状态快照下发。
- `ReceiveLoopAsync`：每个客户端各有一个后台接收循环，收到 Text 消息即通过 `OnTextMessage` 事件转发给上层（`MainWindow.HandleWsMessage`），收到 Close 或异常则清理连接并触发 `OnClientDisconnected`。
- 连接建立时触发 `OnClientConnected`，`MainWindow` 在回调里调用 `SendStateSnapshot(id)`，让新连接/重连的客户端立即拿到当前阶段状态，不用等下一次广播。
- 没有心跳发送逻辑在服务端一侧——心跳由 AR 客户端主动发 `ping`，服务端只是原样回 `pong`（见下）。也没有消息级别的顺序保证或重试机制，纯 best-effort 广播。

## PC → AR 消息类型

所有广播消息都带 `ts`（服务端广播时刻，Unix 毫秒），供 AR 端在重连后丢弃过期消息。

| 类型 (`type`) | 触发时机 | 关键字段 | 组装/调用点 |
|---|---|---|---|
| `state` | 每次阶段切换（`waiting/armed/calibrating/baseline/training/rest/retention/paused/ended/error`），以及新客户端（重）连接时单播一次快照 | `state`, `condition`(`""`/`EF`/`IF`), `block`(0-3), `restDurationSec`, `reason`, `targetL/directionL/targetR/directionR`(个性化目标角，baseline 完成前为哨兵值 `0`/`""`) | `BroadcastArState(...)`（广播）/ `SendStateSnapshot(id)`（单播，内容与最近一次 `BroadcastArState` 的参数完全一致，靠 `_lastState/_lastCondition/_lastBlock/_lastRestDurationSec/_lastReason` 四个字段缓存） |
| `live` | **仅 `IF` 条件、且处于 training block、非 retention、非 rest** 时，随每个 IMU 诊断帧节流到 10Hz 广播（`_lastLiveAngleSentTicks` 节流，间隔 `LiveAngleIntervalMs`） | `packetId`, `angleL/angleR`(相对**当前骨盆朝向**的原始脚部 yaw，度,已扣除标定时的脚/骨盆安装偏移,`RawFootYawDeg`), `confidence`, `stability` | `OnDiagnosticsFrame` 回调内联广播 |
| `fpa` | 仅在 training block（非 baseline/retention/rest）内，每次 `FpaEngine` 产出一次结算结果时 | `stage`("training"), `block`, `packetId`, `fpaL/fpaR`, `onTargetL/onTargetR`, `errorL/errorR`, `confidence`, `stability`, `quality` | `OnFpaResult` 回调内 |
| `stepProgress` | baseline 阶段每收集一步（`OnBaselineProgress`），以及 retention 阶段每产出一次结果时 | `stage`("baseline"/"retention"), `stepsL/stepsR`, `requiredL/requiredR`(baseline 用 `_pipeline.Params.MinBaselineSteps`，retention 用 `RetentionTargetSteps`) | `BroadcastStepProgress(...)` |
| `redo` | 操作员在 Pause 对话框选择 Redo 某阶段时 | `stage`, `attempt`, `reason` | `BroadcastRedo(...)` |
| `pong` | 收到客户端 `{"cmd":"ping"}` 时，单播回给发送者 | `ts` | `HandleWsMessage` 的 `"ping"` 分支 |

**关于 `live` 的重要说明（与旧协议文档的差异）**：`docs/superpowers/specs/2026-07-29-ws-protocol.md` 描述 `live` 由 UI 上的 `ChkLiveAngleToAr` 复选框控制、且 EF/IF 均发送。经核对当前 `MainWindow.xaml.cs` 源码，**该复选框已不存在**（全文搜索 `ChkLiveAngleToAr` 无匹配），`live` 广播条件已改为硬编码的 `ConditionForAr() == "IF"` 门控（即只有 IF 条件才发送 `live`，EF 条件完全不发送）。这份规范文档已过时，以代码为准。

`live` 中的 `angleL/angleR` 与 `state.targetL/R`、`fpa.errorL/R` 是**不同的参考系**：前者相对"实时骨盆朝向"，后者相对"行进方向估计（PD）"，两者在直线行走时近似重合但不完全一致（`RawFootYawDeg` 函数注释明确说明）。

## AR → PC 命令

客户端以 `{"cmd": "..."}` 发送，服务端 `HandleWsMessage` 按 `cmd` 分发（`cmd` 大小写不统一是历史遗留，代码注释未做修正）：

| `cmd` | 参数 | 行为 |
|---|---|---|
| `ping` | 无 | 单播回 `pong` |
| `Start` | 无 | 仅打日志，无实际动作 |
| `ReadyForCalibration` | 无 | 调用 `_pipeline.TriggerCalibration()`；若标定尚未 armed 则仅记日志不触发 |
| `continueTraining` | `fromBlock`(int) | 交给 `HandleContinueTraining(fromBlock)`：必须当前处于 `_inRest` 且 `fromBlock == _restBlockJustFinished`（防止过期/重复命令）且 `_stageStopwatch.Elapsed` 已超过该窗口要求的最短时长（Block1 后 120s / Block3 后 300s，见 `RestDurationSec`/`PostBlock3RestDurationSec`）才会调用 `AdvanceStage()`；否则记日志拒绝，不报错给客户端 |
| `stop` / `Stop` | 无 | 仅打日志，无实际动作 |
| 其他未知 `cmd` | — | 记录 `WS: unknown cmd=...` 日志，不做任何动作 |

操作员点击 UI 上的 "Continue Rest" 按钮（`BtnContinueRest_Click`）会调用同一个 `HandleContinueTraining`，走完全相同的门控逻辑，是 AR 端 Continue 命令的等价降级路径（AR 掉线时操作员仍可手动推进）。

## 关键实现细节

- **线程安全**：`WebSocketBroadcastServer` 内部用 `lock (_gate)` 保护 `_clients` 字典；每个客户端的收发运行在独立 `Task.Run` 后台任务里；`MainWindow` 侧的所有广播调用（`_ = _wsServer.BroadcastJsonAsync(...)`）都是 fire-and-forget（不 await），失败会被 `BroadcastJsonAsync` 内部 catch 掉并移除对应客户端。
- **`live` 的节流**：源数据是 100Hz 的 IMU 诊断帧，但 `WebSocket.SendAsync` 同一个 socket 不能并发调用两次，逐帧发送有触发并发异常导致断线的风险，因此代码把 `live` 节流到约 10Hz（`_lastLiveAngleSentTicks` 时间戳比较），中间帧直接丢弃，不做队列缓冲。
- **重连快照**：新客户端连接后立即收到当前 `state` 快照（不含 `live`/`fpa`/`stepProgress` 的最近值），所以 AR 端重连后能恢复到正确的阶段文案，但连续型的角度/反馈流要等下一次自然广播才会有数据。
- **临时调试代码**：`OnFpaResult` 广播 `fpa` 之前有一段延迟测量日志（`_pipeline.TakeFrameLatencyMs`，注释写着"TEMP：延迟测量，用完删除"），是尚未清理的调试代码，不影响协议本身。
- **`live` 帧率与旧文档不一致**：规范文档称 EF/IF 均发送 `live` 用于驱动连续旋转，但当前实现只有 IF 会收到 `live`；EF 端的持续旋转效果如何实现（是否完全依赖 `fpa` 离散更新）需要对照 AR 端代码（见 `08-ar-frontend.md`）确认，本文档不做超出 PC 端代码范围的推测。

## 与其他模块的关系

- 广播的业务数据来自 `02-gait-pipeline.md`（`GaitPipeline` 产出的 `FpaResult`、`DiagnosticsRow`、`BaselineProfile` 等）。
- 所有广播调用点和命令处理都写在 UI 层的 `MainWindow.xaml.cs`（见 `04-ui-layer.md`），`WebSocketBroadcastServer` 本身不感知任何步态/训练语义。
- AR 端（Unity）如何解析和消费这些消息见 `08-ar-frontend.md`。
