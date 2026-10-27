# 实验流程梳理记录(2026-10-27)

> 目的:梳理操作员视角下的实验流程与按钮关系,为后续"阶段状态机"重构定规则。本文只记录结论与待办,**尚未改动任何代码**。
> 标注"已读代码"的条目来自读源码,未运行复现;"待确认"的条目需要操作员/研究者回答。

## 1. 已确认的规则(操作员 2026-10-27 确认)

| 项 | 规则 |
|---|---|
| Measure | 3 个 IMU 连齐后自动开始;按钮用于测试过程中暂停/重启测量 |
| Start Calibration | 语义 = 测试出现意外、一切从头开始(等同 Start Condition),并清除脏数据 |
| Save Diagnostics | 应保存各 stage 的标记:是否暂停过、是否重新标定过、是否 Mark 问卷,以便按 PacketId 对 IMU samples 分析 |
| Screening | 只允许在特定阶段使用(具体阶段待定,见 §3) |
| AR 眼镜 Continue | 不再需要 |
| Order Group | 不再需要,从界面移除;顺序由同一 Participant ID 下已有的 condition 记录推导 |
| Parameters | 需要记录到 Meta JSON |
| Redo | 仅表示重新开始**当前采集阶段** |

## 2. 已读代码得出的现状问题(待复现/待修)

均为读代码推断,行号为 2026-10-27 的 `MainWindow.xaml.cs`。

1. **Redo 不重置步数**:Redo 分支(`:1662-1671`)只改 attempt、记录 marker、重启秒表;`SetTrainingBlock` 只在 `EnterStage:1486` 调用,Retention 的 `_retentionStepsL/R` 也只在 `EnterStage` 清零。作废尝试的有效步数可能计入 100 步目标。
2. **暂停中点 Next Stage**:`BtnNextStage_Click` 不检查 `IsPaused`;`WriteRow` 受 `!IsPaused` 门控,新阶段进入后数据行可能不写入。
3. **暂停期间 block 完成**:`OnTrainingBlockComplete`(`:289`)不检查 `IsPaused`,pipeline 步数计数也不受暂停影响。
4. **Rest/重校准期间 Pause → Resume(Continue)**:Resume 调用 `ArStateForStage(_currentStage)`(`:1681`),Rest 时 `_currentStage` 仍是 `Training1`,眼镜被发成 `training`,PC 实际在 Rest。Rest 期间选 Redo 的语义也不成立。
5. **Next Stage 绕过协议窗口**:直接调 `AdvanceStage`,不经 `EnterRest`;Training1/3 点它会跳过 Rest 与 marker 窗口,且 `MissingExpectedMarkers()` 不会提醒。
6. **Baseline 阶段点 Next Stage**:未检查 `BaselineProfile`,可能在无 target 的情况下进入 Training1,`StartTraining()`/`SaveBaseline` 被跳过,`_baselineTimer` 不停。
7. **Start Calibration 名不副实**:只做 `_pipeline.Reset()`(会清掉 BaselineProfile),不重置阶段/recorder/marker 标志/AR 状态,且无确认弹窗、不检查是否录制中。
8. **Screening 无阶段门控**:录制中点它会把筛查页推给眼镜。
9. **Meta 时间基准混用**:`StageMarker` 用 `DateTime.UtcNow`(`ExperimentRecorder.cs:176`),`PauseEvent`/`StageTiming` 用 `DateTime.Now`(`:147、:183`);且所有事件**没有 PacketId**。重校准只记录了"提示开始"(`Recalibrate_Before…`),未记录完成/失败。
10. **Measure 是开关**:`StartMeasurement` 在 MEASURING 状态下会调 `gotoConfig()` 停止测量(`ImuDeviceManager.cs:134-146`);停止/重启没有任何记录。**待确认**:重启后 PacketId 是否归零、滤波器 yaw 参考是否重置(若是,Session CSV 的 PacketId 会不唯一,原校准也可能失效)。
11. **Parameters 未入 Meta**:实验中改参数实时生效(`SyncParamToPipeline`),无记录。
12. **漏点的 marker 事后无法补点,而 End 弹窗却建议"回去点 Mark"**:`AdvanceStage` 在离开 Rest 时把 MC1/MC3/NASA-TLX#1/IMI-PC 的 `CanMark` 全部置 false(`:1436-1439`),只有 Rest 窗口内可点。窗口过了,Flow 变橙(due)但标签不可点,`BtnEndCondition_Click` 弹窗里的"go back and click the matching Mark button first"(`:1518`)实际做不到。NASA-TLX#2 的窗口是 `CompleteRetention` 之后至 End,不受此影响。
13. **`GetScreeningResult` 有隐式回退**:`ExperimentRecorder.cs:71` 在被试目录下只要存在任意 `*_Session.csv` 即返回 "Pass"。归档 Abort 的旧文件时,命名必须避开这个通配符(见 §7.8)。

## 3. 待决定(截至本文,尚未回答)

| # | 问题 | 建议 |
|---|---|---|
| Q1 | Rest/重校准期间 Pause 的规则 | **已定(2026-10-27)**:允许 Pause + Continue,禁用 Redo |
| Q2 | 录制中 Measure Stop 的处理 | **已定(2026-10-27)**:只记录事件(Measure Stop/Start,含 PacketId),不自动 Pause、不强制重校准。**遗留风险**:若 §2-10 的实测证实重启后 PacketId 归零或 yaw 参考重置,重启之后的数据可能不可用而系统不会提醒,届时需重新评估此决定 |
| Q3 | Start Calibration(Abort)的旧数据 | **已定(2026-10-27)**:改名归档(`_aborted_时间戳`)而非删除;点击前弹确认(2026-10-27 操作员确认要弹窗) |
| Q4 | Screening 允许的阶段;已 Pass 能否重筛 | **已定(2026-10-27)**:方案 A——流程保持"先 Screening → Pass → 再 Start Condition";Screening 按钮仅在没有进行中的 condition 时可用(录制中禁用)。重筛沿用现状:结果追加到已有 `Attempts`(`ExperimentRecorder.cs:82-83`) |
| Q5 | 事件日志落盘位置 | **按建议采用(2026-10-27)**:事实源 = Meta JSON 中的 `Events` 列表(与现有 marker/暂停同处一份文件);Save Diagnostics 导出时,在诊断 CSV 旁额外写一份 `*_Events.csv`,便于按 PacketId 与 Session CSV 对齐。若实际分析只读其中一份,再收敛 |
| Q6 | Parameters 记录粒度 | **按建议采用(2026-10-27)**:Start Condition 时记一份参数快照 + 实验中每次修改记一条事件(带 PacketId);并记录软件版本(git commit),因 ±4°、Δ=10° 等阈值是代码常量 |

## 4. 设计方向(状态机重构,待上述问题定稿后再动手)

- 阶段改为显式 Phase(含 Rest、等待重校准),Pause 作为叠加在 Phase 上的另一个维度;所有转换走一个入口 + 转换表,非法事件忽略并记日志。
- 只存"事实"(当前 Phase、已 Mark 的集合),Flow 清单、`CanMark`、`MissingExpectedMarkers`、AR state 全部推导。
- 每个 condition 新建一个状态对象,取代手写 reset 清单。
- 统一事件日志:每次转换自动写一条事件(类型、PacketId、UTC 时间、Stage、Attempt、详情),覆盖暂停/重校准/Mark/Measure 启停/参数变更/Abort。
- Abort(Start Calibration)复用 Start Condition 的重置路径。
- 迁移顺序:先写"刻画当前行为"的测试 → 实现纯逻辑类 → 影子模式比对 → 切换读取侧 → 切换转换入口 → **单独提交**修复 §2 的问题。行为变更须在两批被试之间部署。
- Meta JSON 里现有的阶段名/marker 名(`Training1`、`NASA-TLX_Admin1` 等)不得改动,分析脚本依赖它们。

## 5. 暂缓清理清单(流程梳理完成后再处理,**现在不动**)

- AR 眼镜(Unity `IMUARFPA`)端残留的 Continue 按钮/逻辑。
- 文档中关于 `continueTraining`、`BtnContinueRest`、"Rest 必须手动 Continue"的描述:`docs/PROJECT_CONTEXT.md`、`docs/summry/PROJECT_CONTEXT.md`、`docs/superpowers/specs/2026-07-29-ws-protocol.md`、`docs/AR-FRONTEND.md`(及 `IMUARFPA/docs/` 同名文件)。
- 服务端 `HandleWsMessage` 中仅写日志、不做事的 `Start` / `stop` 命令分支。
- Order Group:`cmbOrderGroup` 下拉框、`StartCondition(..., orderGroup)` 参数、Meta 中对应字段(删除前确认分析脚本是否读取该字段)。
- 文档与代码的已知不一致:`PROJECT_CONTEXT.md` 称 Rest 需手动 Continue、MC 由 `EnterRest()` 自动记录;代码实际为 Rest 到时自动推进、MC 由操作员点击 Flow 标签手动记录。`MainWindow.xaml.cs:99` 注释"between Training2 and Training3"过期(实际是 Block1 后与 Block3 后)。
- `docs/PROJECT_CONTEXT.md` 与 `docs/summry/PROJECT_CONTEXT.md` 两份内容不同的"同一份"文档;文档称 `MainWindow.xaml.cs` 1753 行,实际 2260 行。

## 6. 备注

- 去掉 Order Group 后,condition 顺序由同一被试目录下各 condition 的开始时间推导。**更正**:Meta 里现有的 `StageTimings.StartTime` 是本地时间(`DateTime.Now`),不是 UTC;推导顺序应使用 §7.7 新增的 `ConditionStart` 事件(UTC)。Abort 归档的旧文件不应被当成"第一个 condition",归档文件名带 `aborted`,易于排除。

---

## 7. 完整设计(待确认后实施;**尚未改动任何代码**)

> 规则来源:§1 已确认规则 + §3 Q1–Q6 的决定。凡 §7.11 列出的"待确认"项,表格里先用建议默认值,标 `(默认)`。

### 7.1 设计原则

1. **只存事实,其余推导**:存 `Phase`、`Paused`、已 Mark 集合、各 Phase 的 attempt;`IsRetention/InRest/RetentionComplete/RestBlockJustFinished/CanMark/due/missing/AR state/Flow 状态` 全部由它们算出。
2. **每个 condition 一个 `ConditionSession` 对象**(含上述事实 + recorder 句柄),Start Condition 时新建,取代手写 reset 清单。
3. **所有状态变化走唯一入口 `Fire(event)`**,查转换表;非法事件忽略并写日志,不抛异常、不改状态。
4. **逻辑和副作用分离**:`ProtocolFlow` 为不依赖 WPF/硬件的纯 C# 类,通过 `PhaseChanged` 等事件通知;`recorder.BeginStage`、`BroadcastArState`、`pipeline.SetTrainingBlock` 等副作用留在 `MainWindow` 订阅执行(Surgical Changes:不拆 God class)。
5. **对外格式不破坏**:Meta JSON 里现有字段和阶段名/marker 名(`Training1`、`NASA-TLX_Admin1`、`PauseEvents`、`StageMarkers`、`StageTimings`、`FinalAttempt`)原样保留,新信息只做**增量字段**。

### 7.2 Phase(取代 `_currentStage` + `_inRest` + `_pendingStageEntry` + `_isRetention` + `_retentionComplete` + `_sessionState`)

| Phase | 含义 | AR state | 是否采集数据 |
|---|---|---|---|
| `Idle` | 无进行中的 condition | `waiting` / `ended` | 否 |
| `Armed` | 已 Start Condition,等 Ready(首次校准) | `armed` | 否 |
| `Calibrating` | 首次静态校准中 | `calibrating` | 否 |
| `Baseline` | 基线行走 | `baseline` | 是 |
| `Failed` | 基线步数不足(当前代码:显示 error,只能重启程序)。只允许 Abort 或 End(O5) | `error` | 否 |
| `RecalPending(target)` | 阶段间等 Ready,`target` ∈ Training1/2/3、Retention | `armed` | 否 |
| `Recalibrating(target)` | 阶段间重校准中 | `calibrating` | 否 |
| `Training(n)` n=1,2,3 | 训练 block | `training` | 是 |
| `Rest(n)` n=1,3 | Rest + MC(+Block3 后的 TLX#1/IMI-PC) | `rest` | 否 |
| `Retention` | 保留测试(无 AR 反馈) | `retention` | 是 |
| `RetentionDone` | 步数达标/超时,等 TLX#2 与 End | `retention` | 否 |
| `Ended` | 已 End,session 已关闭 | `ended` | 否 |

- **脚本顺序**(线性进度,供 Flow 推导):`Baseline → Training1 → Rest1 → Training2 → Training3 → Rest3 → Retention → RetentionDone`,相邻阶段之间插入 `RecalPending/Recalibrating`。
- **`Paused` 是叠加标志,不是 Phase**:`Paused = true` 时 Phase 不变,Resume 后按 Phase 推导 AR state(修复 §2-4)。

### 7.3 事件(输入)

| 来源 | 事件 |
|---|---|
| 操作员 | `StartCondition`, `Ready`, `NextStage`, `Pause`, `ResumeContinue`, `ResumeRedo`, `Mark(x)`, `EndRecording`, `Abort`(现 Start Calibration), `MeasureStop`, `MeasureStart`, `ParamChanged` |
| AR 眼镜 | `Ready`(cmd `ReadyForCalibration`) |
| Pipeline | `CalibrationStarted`, `CalibrationCompleted`, `CalibrationFailed`, `BaselineCompleted(valid/invalid)`, `BlockComplete`, `RetentionTargetReached` |
| 定时器 | `RestElapsed`, `StallTimeout`(Baseline/Training/Retention 的 10 分钟兜底) |

### 7.4 转换表(Paused = false)

| 当前 Phase | 事件 | 新 Phase / 动作 |
|---|---|---|
| `Idle`,`Ended` | `StartCondition`(前置检查通过) | `Armed`;新建 `ConditionSession` |
| `Armed` | `Ready` | `Calibrating` |
| `Calibrating` | `CalibrationCompleted` | `Baseline`(启动基线计步与 10 分钟兜底) |
| `Calibrating` | `CalibrationFailed` | `Armed`(重试直到成功;多次失败由操作员判断是否重启软件。现状为"显示 error、只能重启程序") |
| `Baseline` | `BaselineCompleted(valid)` | `RecalPending(Training1)` |
| `Baseline` | `BaselineCompleted(invalid)` / `StallTimeout` | `Failed` |
| `Failed` | `Abort` | 见 §7.8,结果 `Armed`(整个 condition 重来) |
| `Failed` | `ResumeRedo` | **不合法**,只能 Abort 或 End |
| `RecalPending(x)` | `Ready` | `Recalibrating(x)` |
| `Recalibrating(x)` | `CalibrationCompleted` | `x` |
| `Recalibrating(x)` | `CalibrationFailed` | `RecalPending(x)`(与现状一致) |
| `Training(1)` | `BlockComplete` / `StallTimeout` | `Rest(1)` |
| `Training(2)` | `BlockComplete` / `StallTimeout` | `RecalPending(Training3)` |
| `Training(3)` | `BlockComplete` / `StallTimeout` | `Rest(3)` |
| `Rest(1)` | `RestElapsed` | `RecalPending(Training2)` |
| `Rest(3)` | `RestElapsed` | `RecalPending(Retention)` |
| `Retention` | `RetentionTargetReached` / `StallTimeout` | `RetentionDone` |
| `Training(n)` / `Rest(n)` | `NextStage`(仅测试用,O1) | 硬跳:按 `ConditionStages` 顺序进入下一阶段的 `RecalPending`,**不经过 Rest**;在 Rest 中则立即结束 Rest。写 `NextStage(fromPhase, skippedWindows)` 事件并在 Meta 标记 `OperatorSkips`;录制中先弹确认 |
| `Baseline`/`Retention`/`RetentionDone`/`Armed`/`Calibrating`/`RecalPending`/`Recalibrating`/`Failed` | `NextStage` | 忽略并写日志(Baseline 需要真实的 BaselineProfile,配合 §7.12 步数参数即可快速完成) |
| 任一非 `Idle/Ended` | `EndRecording` | `Ended`(缺 marker 时按 §7.9 弹确认) |
| 任一非 `Idle/Ended` | `Abort` | 见 §7.8,结果 `Armed` |
| `Training(n)`/`Baseline`/`Retention` | `ResumeRedo` | (O3;"Redo 从校准开始"已于 2026-10-27 确认)先弹窗提示"Redo 用于 IMU 脱落/移位等意外,确认后将重新校准";确认后:attempt+1、**重置该 Phase 的计数器与秒表**(修复 §2-1),进入 `RecalPending(同一 Phase)`(Baseline 则回 `Armed`);Ready + 校准完成后重新开始该 Phase |
| `Rest`/`RecalPending`/`Recalibrating`/`Calibrating`/`Armed`/`RetentionDone`/`Failed` | `ResumeRedo` | **不合法**,Resume 对话框中 Redo 灰掉(Q1) |
| 任一非 `Idle/Ended` | `Pause` / `ResumeContinue` | 切换 `Paused`;Resume 后 AR state 由 Phase 推导 |
| 任一 | `MeasureStop/MeasureStart/ParamChanged` | 只写事件,Phase 不变(Q2、Q6) |
| 任一非 `Idle/Ended` | `Mark(x)` | 加入已 Mark 集合;窗口规则见 §7.6 |

### 7.5 Paused 期间

- 允许:`ResumeContinue`、`ResumeRedo`(受上表合法性约束)、`EndRecording`、`Abort`、`Mark`、`MeasureStop/Start`、`ParamChanged`。
- `NextStage`、`Ready` 一律拒绝(修复 §2-2)。
- Pipeline 事件(`BlockComplete`、`BaselineCompleted`、`RetentionTargetReached`、`CalibrationCompleted/Failed`)**挂起并在 Resume 后按到达顺序重放**(默认,§7.11-O2),修复 §2-3;每个被挂起的事件同样写入事件日志。
- 秒表在 Paused 期间停止(现有行为),因此 Rest 计时和 StallTimeout 自然暂停。
- 数据行不写入(现有行为);PacketId 缺口 + `PauseStart/PauseEnd` 事件即可解释。

### 7.6 推导值(不存储)

| 推导值 | 规则 |
|---|---|
| `InRest` | Phase 是 `Rest(*)` |
| `IsRetention` | Phase 是 `Retention` 或 `RetentionDone` |
| `RetentionComplete` | Phase 是 `RetentionDone` |
| IF `live` 广播门控 | `condition==IF` 且 Phase 是 `Training(*)`(沿用现有"不含 Paused 判断"的行为,不在本次重构里改) |
| Flow 圆点 done | 脚本进度已越过该项 |
| Marker `due` | MC1:进度 ≥ `Rest(1)`;MC3/TLX#1/IMI-PC:进度 ≥ `Rest(3)`;TLX#2:进度 ≥ `RetentionDone` |
| `MissingExpectedMarkers` | `due` 且不在已 Mark 集合中 |
| `CanMark(x)` | 默认:`due` 且尚未 Mark(窗口过后仍可补点,事件带 `late=true`,§7.11-O4;修复 §2-12) |
| AR state | 取 §7.2 表,`Paused` 时为 `paused` |
| `BtnReady` 可用 | Phase 是 `Armed` 或 `RecalPending(*)` 且未 Paused |

### 7.7 事件日志(满足"按 PacketId 分析 IMU samples")

**落盘**:Meta JSON 新增增量字段 `Events`(列表);Save Diagnostics 导出时,在诊断 CSV 旁额外写 `<同名>_Events.csv`(Q5)。

**字段**:`Seq`(自增)、`Type`、`PacketId`(日志写入时最近一次已处理的 `DiagnosticsRow.PacketId`)、`TsUtc`、`Stage`(沿用现有阶段名)、`Attempt`、`Detail`(文本/JSON)。`PacketId` 是"最近处理的包",与真实事件相差最多是 UI 线程调度延迟内的若干包。

**类型**:`ConditionStart`(含参数快照、软件版本、`restartOf`)、`StageStart/StageEnd`、`PauseStart/PauseEnd(decision,reason)`、`Redo(newAttempt)`、`RecalPrompt/RecalStart/RecalDone/RecalFailed`、`Marker(name,late)`、`NextStage(fromPhase)`(操作员越权推进)、`MeasureStop/MeasureStart`、`ParamChange(name,old,new)`、`Abort(archivedTo)`、`BlockComplete(reason)`、`End(reason)`、被挂起事件的 `Deferred(type)`。

**时间基准**:新增字段一律 UTC。**现有** `PauseEvents/StageTimings` 仍是本地时间(`DateTime.Now`),为不破坏分析脚本**不修改**;在文档里标明差异,分析时对齐到 `Events` 的 UTC。

**参数快照**(`ConditionStart` 的 Detail):`PipelineParams` 全部公开属性、Rest/兜底超时/目标步数等 `MainWindow` 常量、IF 渲染模式;`ParamChange` 只记变化项。软件版本:优先用构建时写入的 git commit(`AssemblyInformationalVersion`),实现时确认项目格式是否支持。

**与现有字段的关系**:`StageMarkers/PauseEvents/FinalAttempt` 继续写;`Events` 是同一信息的统一视图。待分析脚本全部迁到 `Events` 后,再讨论去重。

### 7.8 Abort(现 Start Calibration)

1. 仅在非 `Idle/Ended` 时可用;点击先弹确认(显示被试 ID、condition、"当前数据将归档并从头开始")。
2. 关闭 recorder,把当前 `Session.csv`、`Meta.json`、LiveConsistency 等同 condition 文件**改名归档**,后缀形如 `.aborted-20261027T101500Z`;归档名**不得**匹配 `*_Session.csv` 通配符(`ExperimentRecorder.cs:71`),也不得让 `SessionFileExists` 命中。
3. 新建 `ConditionSession`、重置 pipeline,沿用同一被试 ID 与 condition,等同再点一次 Start Condition → `Armed`(因旧文件已归档,不再弹覆盖确认)。
4. 新 session 的 `ConditionStart` 事件写 `restartOf=<归档文件名>`,保留来源。
5. Screening 结果按被试保存,不受 Abort 影响。
6. 建议把按钮更名为"Restart Condition"以匹配语义(仅界面文案,待你确认)。

### 7.9 按钮启用矩阵

| 按钮 | 可用条件 |
|---|---|
| Start Condition | Phase ∈ `Idle/Ended`,且原有前置检查(ID、Measuring、已 Pass、非筛查中) |
| Screening 及 `< > Pass Fail` | Phase ∈ `Idle/Ended`(Q4);录制中禁用 |
| Ready (AR) | Phase ∈ `Armed/RecalPending` 且未 Paused |
| Pause / Resume | Phase ∉ `Idle/Ended`;Resume 对话框的 Redo 受 §7.4 合法性约束 |
| Next Stage | 仅测试用(O1):仅 `Training(n)`/`Rest(n)` 可用;录制中弹确认 |
| End Recording | Phase ∉ `Idle/Ended`;存在 `due` 未 Mark 的 marker 时弹确认,确认文案改为"这些 marker 未记录,仍要结束?"(因 `CanMark` 现允许补点,文案可保留"回去点 Mark") |
| Start Calibration(Abort) | Phase ∉ `Idle/Ended`,二次确认 |
| Measure | 始终可用;录制中使用时写 `MeasureStop/Start` 事件(Q2) |
| Parameters | 始终可用;修改写 `ParamChange`(Q6) |
| Mark(Flow 标签) | 见 §7.6 `CanMark` |
| Open Data Folder | 有当前 session 路径时 |
| Load IMU Samples / 回放 | 不在录制且 IMU 未测量(现有) |
| Save IMU Samples / Save Diagnostics | 始终可用;Save Diagnostics 同时导出 Events CSV |

### 7.10 迁移步骤与验证

```
M0 决策(本文档 §1、§3、§7.11)                           → verify: O1–O6 回答完毕
M1 新增 ProtocolFlow(纯逻辑,暂不接入)+ 单元测试           → verify: 测试通过;覆盖见下表
M2 新增 EventLog 与 Meta 增量字段(Events/Params/Version) → verify: 旧 Meta 字段逐项不变;旧 Meta 仍能被现有分析脚本读取
M2.5 步数参数统一(见 §7.12),独立小提交                   → verify: 参数设 5 时 Training/Retention 在双脚各 5 步结束;默认 100 时现有 replay 测试结果不变
M3 影子模式:接入 ProtocolFlow 与旧标志位并行,每 tick 比对 → verify: 找一名同事戴 IMU 完整走完全程(含 Pause/Redo/Next Stage/Abort),零不一致
M4 读取侧切换:Flow 清单、MissingExpectedMarkers、CanMark、AR state 改读 ProtocolFlow
M5 转换入口切换:AdvanceStage/EnterRest/EnterStage/CompleteRetention/EndFlow 改走 Fire;删除旧标志位(_inRest、_isRetention、_retentionComplete、_restBlockJustFinished、_pendingStageEntry、_mc*/_nasaTlx*/_imiPcMarked、_sessionState)
M6 行为修复,每项单独提交且带"先失败后通过"的测试:
   Redo 重置计数(§2-1)、Pause 期间事件挂起(§2-2/3)、Rest/Recal 的 Resume AR 状态(§2-4)、
   Next Stage 策略(§2-5/6)、Abort 重写(§2-7)、Screening 门控(§2-8)、Measure 事件(§2-10)、
   Parameters 入 Meta(§2-11)、marker 可补点(§2-12)、移除 Order Group
```

- 流程测试用例(M1):全程走通;Block2→3 无 Rest;Training1/3 后进入 Rest,Rest 到点进入 `RecalPending`;连续跑第二个 condition 不被上一个污染;重复 `BlockComplete` 被忽略;Paused 时 `NextStage` 被拒;Paused 期间 `BlockComplete` 被挂起并在 Resume 后重放;Rest 期间 `ResumeRedo` 被拒;提前 End 只提示已 `due` 的 marker;Abort 后回到 `Armed` 且 Mark 集合清空。
- **部署窗口**:M3 之后每一步的上线都在两批被试之间;M5 前打 git tag 以便回退。
- Replay 模式下 `EndTrainingBlock` 被 `!_isReplaying` 屏蔽,无法用回放验证阶段流程,所以 M3 必须有一次真人完整演练(约 40 分钟:基线 + 3×训练 100 步 + Rest 120s/300s + Retention)。
- 前提假设(M1 前确认):`IMUMoCap.Tests` 能引用 `ProtocolFlow` 所在的程序集(它已能回放 pipeline,推测可以)。

### 7.11 待确认项(Q1–Q6 之外;默认值已写入上文,你否决则改)

| # | 问题 | 默认 | 备选 |
|---|---|---|---|
| O1 | Next Stage 的语义 | 等同"当前阶段提前完成",保持 Rest 与 marker 窗口;Baseline 未完成、校准类 Phase 中忽略 | (b) 保留"硬跳"但写 `NextStage` 事件并让 End 弹窗知晓被跳过的窗口;(c) 在需要 Rest 的 block 内直接禁用按钮 |
| O2 | Paused 期间到达的 Pipeline 事件 | 挂起,Resume 后重放 | Paused 时直接冻结 pipeline 步数 |
| O3 | Redo 前是否强制重校准 | 不强制 | 强制(校准漂移问题见 `docs/task19_calibration_duration_tuning.py`) |
| O4 | Marker 窗口关闭后能否补点 | 可补点,事件标 `late=true` | 窗口外不可点(现状),End 弹窗改为"仅提示,无法补" |
| O5 | 首次校准失败 / 基线不足的恢复 | 校准失败回到 `Armed`;基线不足可经 Pause→Redo 重做 | 维持现状"重启程序" |
| O6 | 旧 Meta 字段的本地时间 | 不改,只在文档标注 | 迁移为 UTC(需同步改分析脚本) |

#### 7.11a O1–O6 决议(2026-10-27)

| # | 操作员的回答 | 记录为 |
|---|---|---|
| O1 | "测试时想快速到下一阶段才会使用" / "主要就是为了跳过无用的等待时间" | **已定**:Next Stage 是测试用的硬跳,要能越过 Rest,所以不采用原默认"等同提前完成"。跳过的窗口写进事件与 Meta,避免静默漏标;录制中弹确认防误触 |
| O2 | "暂停时一般让被试者坐下休息" | **已定**:保留"挂起并重放"作兜底(逻辑很小);被试坐下时 block 基本不会在暂停中完成 |
| O3 | "还未发生 redo,为了防止 IMU 安装脱落等意外,要弹窗" | **理解(待确认)**:Redo 前弹窗提示,确认后强制重新校准再重做(IMU 脱落/移位后旧校准必然失效) |
| O4 | "不要禁用" | **已定**:窗口过后仍可补点,事件标 `late=true` |
| O5 | "失败时整个 condition 重来";随后更正:"阶段之前的校准失败应该重试直到成功,如果多次不成功由操作员判断重启软件" | **已定**:所有校准失败(首次校准与阶段间重校准)一律回到可点 Ready 的状态重试,不设次数上限;多次不成功由操作员决定重启软件。**只有基线步数不足**进入 `Failed`,只允许 Abort(整个 condition 重来)或 End |
| O6 | "读"(分析脚本读旧字段的时间) | **已定**:旧 Meta 字段保持本地时间不动;新增字段用 UTC |
| Q5 | "可以导出额外的 events.csv,只要能用于分析 IMU samples" | **已定**:`Events.csv` 必须含 `PacketId` 与 `TsUtc`,可与 Session CSV 的 `Timestamp,PacketId` 对齐 |
| Q3 弹窗 | "要弹窗" | **已定**:Abort 前弹确认 |
| O7(新) | "pause 久了 redo 肯定就是从 calibration 开始,因为现在每个测试阶段最开始就是 calibration" | **已定(2026-10-27 确认)**:暂停很久时由操作员选 Redo,Redo 本来就从校准开始;Continue 不重新校准。同时确认了 O3 里"Redo 先重新校准"的部分 |

### 7.12 步数参数统一(2026-10-27 已定,为 M3 真人演练省时间)

**决定**
- Baseline、Training1/2/3、Retention 的"每脚目标有效步数"统一读同一个参数,不再各写各的 100。
- 参数改名:`MinBaselineSteps` → `TargetStepsPerFoot`(界面文案 "Target Steps / Foot"),因为它不再只管 Baseline。
- 不加"忘记改回 100"的保护或弹窗(操作员:改了这个参数通常会重启软件)。参数值会进入 §7.7 的 `ConditionStart` 参数快照,事后可查。

**现状(已读代码)**
- Baseline 已读 `Params.MinBaselineSteps`(`GaitPipeline.cs:122`);Training 用常量 `TrainingBlockTargetSteps = 100`(`GaitPipeline.cs:81、:94、:271`);Retention 用常量 `RetentionTargetSteps = 100`(`MainWindow.xaml.cs:138、:2204-2205`)。
- AR 眼镜端不写死步数,`requiredL/R` 由 PC 下发(`FootHudController.cs:301-308`),**AR 端无需改动**。

**改动范围(实施时)**:`PipelineParams.cs:23`、`MainPageVM.cs:417-420`(及其私有字段)、`ParamsDialog.xaml:38、41`、`MainWindow.xaml.cs:226、968-969、990、1823、1994`、`GaitPipeline.cs:81、94、122、271`、`MainWindow.xaml.cs:138、2204-2205`;`docs/task15_*.py` 里两处注释同步。`BaselineProcessor.MinValidSteps` 与 `BaselineProfile.MinRequiredSteps` 是内部名,保留不动。

**持久化注意(已读代码)**
- 参数只在 Parameters 窗口点 **Save** 时写入 `ParamsFilePath`(`MainWindow.xaml.cs:995-1001`),启动时读取(`:980-990`);窗口内的修改即时生效。
- 因此:演练时把它设成小值且**不点 Save**,重启后自动恢复文件里的值;如果点了 Save,下次启动仍是小值。
- 改名后,旧参数文件里的 `MinBaselineSteps` 键将不再被识别,首次启动会回落到默认 100(安全方向),其余参数不受影响。该行为按 `System.Text.Json` 默认忽略未知成员推断,实施时验证。

**不改**:Rest 时长(120s/300s)仍写死,真人演练里它们合计约 7 分钟。

**测试**:参数设 5 → Training block 在双脚各 5 步时触发 `OnTrainingBlockComplete`,Retention 同;参数为默认 100 时现有 `IMUMoCap.Tests` 结果不变。

**预估**:演练时间由约 40 分钟降到约 15–20 分钟(估算:步数部分约 2 分钟,Rest 合计 7 分钟,另有 5 次重校准和需要故意触发的边缘操作)。

---

## 8. 任务拆分(文档冻结后实施;**此刻不写代码**)

**原则**
- 一个任务一组提交,独立可验证、可回退;上线只在两批被试之间;T8 之前打 git tag。
- **重构类任务(T1、T6–T8)不改变行为;行为变更类任务(T9–T14)各自单独提交,并带"先失败后通过"的测试。**
- **重要澄清**:§7.4 的转换表描述的是**最终目标**。T1 实现的是**当前行为**那一版(Redo 只 attempt+1、Next Stage 走 `AdvanceStage`、marker 仅窗口内可点、Pause 只门控写数据),这样 T6 的影子模式才能与旧代码逐项比对。新规则由 T9–T14 逐条修改转换表并加测试。T1 里针对 §2 已知问题的"刻画当前行为"测试标 `[KnownIssue]`,对应修复任务落地时翻转断言。

### 8.1 任务总览

| 波次 | ID | 任务 | 依赖 | 对应 | 规模 |
|---|---|---|---|---|---|
| 0 准备 | P0 | 基线确认:`dotnet build` + `dotnet test` 通过并记录结果;确认测试项目能引用 `ProtocolFlow` 所在程序集;确认项目格式是否支持 `AssemblyInformationalVersion`;整理工作区(未提交的 Ready (AR)/Data Folder 位置等 WPF 改动单独提交,与 Unity 设置改动分开,统一换行符) | — | §7.10 前提 | S |
| 0 准备 | P1 | **[操作员]** 重启测量实测:录制中 Measure Stop→Start,观察 PacketId 是否归零、FPA 是否突变;结果写回 §2-10 | — | Q2 | S |
| 1 独立任务 | T1 | `ProtocolFlow` 纯逻辑类(Phase、Paused 叠加、`Fire`、推导值、**当前行为版**转换表)+ 单元测试 | P0 | M1 | L |
| 1 | T2 | 事件日志与 Meta 增量字段:`Events`、`ConditionStart` 参数快照 + 版本、`OperatorSkips`;最近 PacketId 追踪;Save Diagnostics 额外导出 `*_Events.csv` | P0 | §7.7、Q5、Q6 | M |
| 1 | T3 | 步数参数统一并改名 `TargetStepsPerFoot` | P0 | §7.12 | S |
| 1 | T4 | 移除 Order Group(UI、`StartCondition` 参数、Meta 字段);先 grep 确认分析脚本不读该字段 | P0 | §1 | S |
| 1 | T5 | Screening 按钮门控:录制中禁用(先用 `_recorder.IsRecording`,T8 后改读 Phase) | P0 | §2-8、Q4 | S |
| 2 重构 | T6 | 影子模式接入 `ProtocolFlow`,每 tick 比对旧标志位;**真人演练一次**(步数参数 5),只覆盖现有行为 | T1、T3 | M3 | M |
| 2 | T7 | 读取侧切换:Flow 清单、`MissingExpectedMarkers`、`CanMark`、AR state 改读 `ProtocolFlow` | T6 | M4 | M |
| 2 | T8 | 转换入口切换(`AdvanceStage/EnterRest/EnterStage/CompleteRetention/EndFlow` 走 `Fire`),删除旧标志位 | T7 | M5 | L |
| 3 行为修复 | T9 | Redo:先弹窗 → 重新校准 → 重置该 Phase 计数器与秒表 | T8、T2 | §2-1、O3 | M |
| 3 | T10 | Pause 叠加规则:Paused 时拒绝 Next Stage/Ready,pipeline 事件挂起重放,Rest/Recal 期间禁用 Redo,Resume 后 AR state 由 Phase 推导 | T8 | §2-2/3/4、Q1、O2 | M |
| 3 | T11 | Next Stage 硬跳:越过 Rest、录制中确认、写 `NextStage`/`OperatorSkips`;Baseline 等阶段忽略 | T8、T2 | §2-5/6、O1 | S |
| 3 | T12 | Abort(Restart Condition):确认弹窗、归档(`.aborted-时间戳`,避开 `*_Session.csv` 通配符)、复用 Start Condition 路径;`Failed`(基线不足);所有校准失败回到可重试状态 | T8、T2 | §2-7/13、Q3、O5 | M |
| 3 | T13 | Measure 启停与 Parameters 修改写事件 | T2 | §2-10/11、Q2、Q6 | S |
| 3 | T14 | marker 窗口过后可补点,事件标 `late=true`;End 弹窗文案同步 | T8、T2 | §2-12、O4 | S |
| 4 收尾 | T15 | 清理(§5 暂缓清单):AR 端 Continue、过期文档、`Start/stop` 空分支、文档与代码不一致 | T9–T14 | §5 | M |
| 4 | T16 | **[操作员]** 最终回归真人演练:覆盖 Pause/Redo/Next Stage/Abort/漏点 marker 后 End/校准失败重试 | T15 | — | M |

### 8.2 验收标准

| ID | 验收 |
|---|---|
| P0 | 现有测试的通过/失败数记入本文档;工作区只剩与本次无关的改动 |
| P1 | 结论写入 §2-10:重启后 PacketId 是否归零、FPA 是否异常 |
| T1 | §7.10 的流程测试用例全部通过;类中不引用 WPF/硬件;[KnownIssue] 测试覆盖 §2-1…§2-6 |
| T2 | 旧 Meta 字段逐项与改动前一致(用现有样本对比);`Events.csv` 的每个 `PacketId` 能在对应 Session CSV 中找到相邻行;`ConditionStart` 含完整参数快照与版本 |
| T3 | 参数=5 时 Training、Retention 在双脚各 5 步结束;默认 100 时现有 replay 测试结果不变;旧参数文件首次启动回落默认 100 |
| T4 | grep 证明分析脚本不读 OrderGroup;界面无该下拉框;Meta 不再写该字段 |
| T5 | 录制中 Screening 及 `< > Pass Fail` 不可用;无 condition 时可用 |
| T6 | 真人演练全程 `SHADOW MISMATCH` 日志为零 |
| T7、T8 | 每个切换点前后行为一致:同一份演练操作清单,Flow 清单与 AR state 与切换前逐步相同 |
| T9 | 先写失败测试:Redo 后该 Phase 步数从 0 开始;Redo 必经校准;Rest 期间 Redo 被拒 |
| T10 | 先写失败测试:Paused 时 Next Stage 被拒;Paused 期间 `BlockComplete` Resume 后才生效;Rest 期间 Pause→Resume 后 AR state 为 `rest` |
| T11 | Training/Rest 中 Next Stage 直达下一阶段的 `RecalPending`;录制中有确认;事件与 `OperatorSkips` 记录跳过的窗口 |
| T12 | Abort 后旧文件已归档、新 session 从 `Armed` 开始、Mark 集合清空;归档名不触发 `GetScreeningResult` 回退;校准连续失败可无限重试;基线不足进入 `Failed` |
| T13 | 录制中 Measure Stop/Start 与参数修改在 `Events` 中出现且带 PacketId |
| T14 | Rest 窗口结束后点击 marker 成功,事件 `late=true`;End 弹窗不再建议无法执行的操作 |
| T15 | §5 清单逐项勾掉;文档与代码一致 |
| T16 | 演练操作清单全部通过,Meta/Events 内容与实际操作一致 |

### 8.3 推荐顺序与并行
1. P0、P1 先做。P1 需要操作员空档。
2. 第 1 波五个任务彼此独立,可以并行或任意顺序。建议顺序:T3 → T4 → T5 → T2 → T1(由小到大,先拿到小的可验证收益)。
3. 第 2 波严格串行:T6 → T7 → T8,每步之间都在两批被试之间上线。
4. 第 3 波六个任务彼此基本独立(都依赖 T8),建议顺序:T10 → T9 → T11 → T12 → T14 → T13(由影响数据完整性的最高到最低)。
5. 第 4 波最后。

### 8.4 风险提示
- T6 的影子模式只能比对"新旧都有"的行为;Redo/Next Stage/Abort/Pause 的新语义要等到第 3 波才能整体验证,由 T16 兜底。
- T8 是改动面最大的一步,务必打 tag,出问题时可整体回退。
- T2 的 `PacketId` 是"最近处理的包",与事件真实发生时刻差几个包,用于分析时请取事件之后的第一个包为参照。

### 8.5 P0 结果(2026-10-27,已执行)

| 项 | 结果 |
|---|---|
| `dotnet test IMUMoCap.Tests`(含构建) | 构建成功(仅 nullable 与未使用字段的警告,无错误)。**测试:通过 0、失败 0、跳过 1、总计 1**。SDK:9.0.318 与 10.0.301 均已安装,项目目标 `net8.0-windows` |
| 唯一的测试 | `GaitPipelineReplayTests.WalkStraight_ProducesFpaResultsInReasonableRange` 被 `Skip` 标记("需要先录制 walk_straight.jsonl fixture"),`Fixtures/` 目录是空的。**即当前没有任何可执行的自动化测试**。另外它的 `results` 列表从未订阅 `OnFpaResult`,即使解除 Skip 也会因 `Assert.NotEmpty` 失败,属于未完成的桩 |
| 测试项目能否引用被测程序集 | 能。`IMUMoCap.Tests.csproj` 对 `IMUMoCap.csproj` 有 `ProjectReference`,且已成功构建;`ProtocolFlow` 放在 `IMUMoCap` 项目内即可被测 |
| 版本号是否含 git commit | 已含。构建产物 `IMUMoCap.dll` 的 `ProductVersion` 为 `1.0.0+a3fc94766bbaa335c2920c66f19a0c04c0070661`(与当前 HEAD 一致),T2 记录软件版本无需额外配置 |
| 工作区 | 构建与测试未产生新的被跟踪改动。待整理项见下 |
| 换行符 | 无 `.gitattributes`;`core.autocrlf=true`,所以改动文件会出现 LF→CRLF 警告 |

**对任务的影响**
- 此前"只有一个 replay 测试"的说法需要更正:实际是**零个可执行测试**。T1 将是项目里第一批真正运行的测试,验收标准里"现有 replay 测试结果不变"(T3)也因此改为:对 T3 另写测试,不依赖这个被跳过的测试。
- T2 不再需要研究版本号方案,直接读 `ProductVersion`。

**工作区待整理(尚未处理,需操作员决定)**
- 已修改且属于本次流程梳理上下文的:`IMUMoCap/MainWindow.xaml`、`MainWindow.xaml.cs`(Ready (AR) 按钮、Data Folder 位置、Screening 文案)。
- 已修改的 Unity 设置:`IMUARFPA/Assets/Settings/URP-*.asset`、`XRGeneralSettingsPerBuildTarget.asset`、`ProjectSettings/BurstAotSettings_StandaloneWindows.json`、`ProjectSettings.asset`。
- 未跟踪:`IMUARFPA/FeedBackWin11/`(疑似 Unity 构建产物)、`IMUARFPA/logo.png`、`IMUARFPA/Assets/Textures/Footprints/logo.png`、`docs/footprint.png`、`docs/steppingStone.png`、两张微信图片、本文档。
