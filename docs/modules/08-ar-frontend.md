# AR 前端 (AR Frontend - Unity/XREAL)

## 模块概述

`IMUARFPA/` 是一个独立的 Unity 工程（Unity 2022.3.62f3 + XREAL XR SDK `com.xreal.xr` 本地 tarball 依赖 + `com.unity.xr.interaction.toolkit` 2.6.5 + `com.unity.xr.hands` 1.8.0），运行在 XREAL AR 眼镜上，作为步态训练的可视化客户端。它不做任何独立的算法计算，纯粹是一个"渲染 + 状态机"消费端：所有角度、阶段、进度数据都来自 PC 端（`IMUMoCap/`）通过 WebSocket 推送。UI 用 uGUI（`Canvas` + TextMeshPro），网络与 JSON 均用内置方案（`System.Net.WebSockets.ClientWebSocket` + Unity `JsonUtility`），不依赖第三方库。

该工程真正属于业务逻辑的脚本只有两个文件（其余 `Assets/Samples/`、`Assets/TextMesh Pro/` 等均为第三方包/示例，不属于本模块）。

## 关键文件

| 文件 | 职责 |
|---|---|
| `IMUARFPA/Assets/FootWebSocketClient.cs` | 通信层：连接/自动重连/心跳、JSON 解析、把解析结果通过 `ConcurrentQueue<Action>` 搬到主线程分发给 HUD |
| `IMUARFPA/Assets/FootHudController.cs` | 表现层：消费解析后的消息，驱动 UI 状态机、脚部图形旋转/变色、进度条 |
| `IMUARFPA/Assets/Scenes/FPATraining.unity` | 唯一训练场景；已核实 `ProjectSettings/EditorBuildSettings.asset` 中 build index 0 且唯一 `enabled: 1` 的场景，`HelloMR.unity` 仍为 disabled |

## 通信层要点（`FootWebSocketClient`）

- **连接地址**：`serverUrl` 字段硬编码默认值 `ws://192.168.137.1:8765/ws/`（Inspector 可改，无环境切换机制），PC 是 WS 服务端，眼镜是客户端。
- **连接生命周期**：`Start()` 用 `Task.Run(ConnectionLoop)` 起一个后台循环，不阻塞 Unity 主线程；`ConnectionLoop` 在断线/连接失败后自动重连，退避延迟从 `reconnectInitialDelaySec`(1s) 翻倍增长到 `reconnectMaxDelaySec`(10s) 上限；每次成功连接后用 `Interlocked.Increment(ref _sessionGen)` 生成新的会话代号，`ReceiveLoop`/清理逻辑用它判断是否是"当前这一代"连接，避免旧连接的收尾逻辑覆盖新连接状态。
- **心跳**：`enableHeartbeat` 开启时每 `heartbeatIntervalSec`(10s) 发送 `{"cmd":"ping"}`，服务端以 `{"type":"pong"}` 回应（客户端收到后为 no-op）。
- **线程模型**：Socket 收发（`ConnectionLoop`/`ReceiveLoop`/`HeartbeatLoop`）全部跑在后台 `Task` 上；对 Unity API（`RectTransform`/`Image`/`Slider` 等）的调用必须在主线程执行，所以所有回调都包成 `Action` 塞进 `ConcurrentQueue<Action> mainThreadActions`，在 `Update()` 里逐个取出执行。
- **消息格式**：服务端消息统一是带 `type` 字段的 JSON，反序列化进同一个非多态的 `ServerMessage` 类（所有类型共用字段，不相关字段保持默认值），`type` 取值：`state`/`fpa`/`live`/`stepProgress`/`redo`/`pong`，分别路由给 `FootHudController` 的 `OnStateChange`/`OnFpaUpdate`/`OnLiveUpdate`/`OnStepProgress`/`OnRedo`。
- **NaN/无数据处理**：某只脚本帧无数据时服务端会发裸 `NaN` 或字符串 `"NaN"`；`JsonUtility` 无法解析这些进 `float` 字段，所以 `HandleMessage` 在反序列化前先扫描原始 JSON 记录 `fpaLIsNaN`/`fpaRIsNaN` 标志，再把 `NaN`/`null` 统一替换成 `0` 后才解析，下游用这两个标志区分"真实值为 0"和"这帧该脚没数据、跳过渲染"。
- **AR→PC 命令**：`SendReadyForCalibrationAsync()` 发 `{"cmd":"ReadyForCalibration"}`，`SendContinueTrainingAsync(fromBlock)` 发 `{"cmd":"continueTraining","fromBlock":N}`，均由 HUD 的确认按钮点击触发，客户端从不自动发送。

## HUD 状态机要点（`FootHudController`）

- **状态机**（`OnStateChange`，与协议 `state` 字段一一对应）：`waiting → armed → calibrating → baseline → training ⇄ rest ⇄ retention → ended`，`paused`/`error`/未知值（含初始的 `disconnected`）走 default 分支。`armed` 态和 `rest` 态倒计时结束后都需要显式点击确认按钮（`ShowConfirmButton("Start"/...)`），客户端从不自动推进到下一阶段。
- **`rest` 态倒计时**：进入 `rest` 时记录 `_restStartTime` 并按服务端下发的 `restDurationSec`（缺省 120s）用 `InvokeRepeating(CheckRestElapsed, 0f, 1f)` 每秒刷新剩余时间文案；倒计时清零后才显示 "Continue" 按钮。
- **`paused → 非paused` 恢复处理**：切出 `paused` 状态时置位 `_awaitingResumeFpa = true`；下一条 `training` 态的 `state` 消息只显示"Resuming..."文案而不立即显示脚部图形，真正的图形恢复要等到下一条 `fpa` 包到达（`OnFpaUpdate` 里检测到 `_awaitingResumeFpa` 才调用 `ShowGraphicGroup()`），避免暂停期间残留的旧角度图形先闪一下。
- **EF/IF 两种训练条件的渲染差异**（`_condition` 字段，来自 `state.condition`）：
  - **EF（stepping stones）**：训练目标角度是固定的，仅在 `state` 消息带 `directionL/R` 时更新一次 `_targetL/R`/`_directionL/R`，进入 `training` 态时 `ApplyStoneTargets()` 把方向换算成带符号角度（`toe-in` 取负）设置石头的旋转；每条 `fpa` 包只是把实际落脚角度（`msg.fpaL/R`）临时叠加到 `footOutL/R` 上做对比展示 `FootOutDisplaySec`(0.2s，代码注释里说明该值来自对 `docs/task8_footprint_display_duration.py` 分析结果的调整) 后自动隐藏，不改变石头本身。
  - **IF（footprint）**：每条 `fpa` 包都直接旋转并按 `onTargetL/onTargetR` 变色（绿=达标/红=偏差），旋转角度用 `errorL/errorR`（= FPA − 目标角）而不是绝对角度。
  - **`live` 消息**：与 EF/IF 无关，任何时候收到都会强制切到脚印图形显示 `angleL/angleR`（绝对角度），不带颜色信号（`onTarget`/`error` 不适用），用于连续实时角度展示。
- **进度条**：`OnStepProgress` 用 `(stepsL+stepsR)/(requiredL+requiredR)` 算总体百分比，只在 `baseline`/`retention` 两个状态显示。
- **`OnRedo`**：仅更新提示文案（stage + attempt + reason），不改变状态机本身。

## 详细文档指引

- 完整架构细节（场景层级结构、字段命名约定、已知问题列表）见 `docs/AR-FRONTEND.md`。
- WebSocket 协议契约（PC 端广播的消息类型、字段含义、AR 端命令）见 `docs/modules/05-websocket-protocol.md`；`FootWebSocketClient.cs` 中的 `ServerMessage` 类是 AR 侧对该协议的具体落地。
