# 模块文档索引 (Modules Index)

本目录按代码模块拆分了 IMUMoCap 项目的技术文档，每个文件聚焦一个模块，便于按需查阅，无需通读整份 `docs/PROJECT_CONTEXT.md`（后者仍然是完整的项目背景文档，继续保留、不受本目录影响，两者互为补充，如有出入以本目录内文档为准，因为它们是对照当前源码逐一核实后写的）。

系统由两部分组成：**PC 端 = WPF 桌面应用**（`IMUMoCap/`，模块 1-7）+ **AR 前端 = Unity/XREAL 工程**（`IMUARFPA/`，模块 8），通过模块 5 的 WebSocket 协议解耦通信。

| # | 模块 | 文档 | 一句话说明 |
|---|---|---|---|
| 0 | 端到端数据流 | [00-data-flow.md](00-data-flow.md) | 按时间线串联全部模块：一个数据包/一次广播从哪来、到哪去，含帧级/步级/阶段级三种粒度 |
| 1 | 设备接入层 | [01-device-access.md](01-device-access.md) | 封装 Xsens XDA SDK，管理 IMU 硬件连接/断线生命周期，向上层暴露数据包与状态事件 |
| 2 | 步态分析核心流水线 | [02-gait-pipeline.md](02-gait-pipeline.md) | 系统算法核心：帧同步 → 质量门控 → 标定 → 站立/摆动检测 → 转弯过滤 → FPA 计算 → 基线生成 |
| 3 | 数据模型 | [03-data-models.md](03-data-models.md) | 贯穿全系统的数据结构/DTO 定义（设备层模型 + Pipeline 数据契约） |
| 4 | WPF UI 层 | [04-ui-layer.md](04-ui-layer.md) | `MainWindow` 业务流程总控台：设备连接、实验阶段状态机、UI 刷新、各模块调用编排 |
| 5 | WebSocket 通信协议 | [05-websocket-protocol.md](05-websocket-protocol.md) | PC 端 WebSocket 服务端实现，PC↔AR 消息类型与命令契约 |
| 6 | 数据记录与持久化 | [06-data-recording.md](06-data-recording.md) | 实验数据落盘：Session CSV、Meta JSON、审计标记写入 |
| 7 | AHRS / 姿态计算工具 | [07-ahrs-utils.md](07-ahrs-utils.md) | 角度换算、四元数运算等零散数学工具 |
| 8 | AR 前端 (Unity/XREAL) | [08-ar-frontend.md](08-ar-frontend.md) | 独立 Unity 工程，消费 WebSocket 消息驱动 HUD 渲染与状态机，无独立算法逻辑 |

## 跨模块数据流

完整的端到端数据流（含 AR→PC 反向命令、UI 参数同步、CSV 回放等分支，以及帧级/步级/阶段级三种粒度的区分）见 [00-data-flow.md](00-data-flow.md)，此处不再重复。

## 阅读建议

- 只想了解某一层具体做什么、有哪些类/关键设计点 → 直接看对应模块文档。
- 想了解完整实验协议流程（阶段划分、问卷时间点等业务背景）→ 看 `docs/PROJECT_CONTEXT.md`。
- 想深入 AR 端架构细节（场景结构、已知问题等）→ 看 `docs/AR-FRONTEND.md`，`08-ar-frontend.md` 只是速览。
- 各模块文档中标注的"未使用的遗留代码""与旧文档不一致之处"等发现，均基于当前源码核实得出，未做任何代码改动。
