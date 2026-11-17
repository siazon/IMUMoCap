# 步幅法 FPA:零点与门控设计(草案,2026-11-12)

状态:已实现(2026-11-13),开关 `MainWindow.UseStrideFpa` 默认关(反馈仍用当前方法,步幅法只记录)。
代码:`Pipeline/StrideDirectionTracker.cs`、`StrideGate.cs`、`StrideZeroEstimator.cs`、`FpaEngine.cs`;
测试:`StrideDirectionTrackerTests`、`StrideGateTests`、`StrideZeroEstimatorTests`、`FpaEngineStrideTests`、
`StrideRecordingTests`、回放验收 `Integration/StrideFpaReplayTests`。实现中与草案不同之处标注为"(实现变更)"。

## 背景

当前 FPA = 脚偏航 − PD,PD 约 60% 来自骨盆朝向。任何一个传感器相对另一个的航向漂移都会进入 FPA。
实测(human profile)几分钟内骨盆相对双脚可漂 25–35°,造成:

- 左右脚 FPA 同时反向偏离(显示为一正一负),参与者被迫"追" target;
- 漂移过大时 ReacquiringPd 无法退出(PD 与骨盆夹角 ≤15° 的条件满足不了),FPA 完全不输出。

步幅法(参考 Wouda et al. 2021, J NeuroEng Rehabil, https://www.ncbi.nlm.nih.gov/pmc/articles/PMC7888122/):
每只脚只用自己的传感器,FPA = 脚偏航 − 这只脚这一步的位移方向。两者来自同一传感器、相隔不到 1 秒,航向漂移相减抵消。

## 符号

对脚 f 的第 k 步(上一次着地结束 → 这一次着地开始):

- ψₖ:这次着地期间脚传感器的绝对航向(四元数绕世界 Z 轴的转角),用与现在相同的结算窗口取圆周均值。
- sₖ:步幅方向。摆动期对 free acceleration 水平分量二次积分,两端速度强制为 0(ZUPT,线性去漂),取 atan2(Dy, Dx)。
- φ_f:零点偏移(传感器航向与"脚的前方"之间的固定夹角)。

FPA_f = sign_f × wrap(ψₖ − sₖ − φ_f),左脚 sign = +1,右脚 sign = −1(与现有约定一致,正 = toe-out)。

## 一、零点 φ_f

目标:保持现有定义——校准站姿时脚与骨盆朝向一致 → FPA = 0,使基线、目标值含义不变。

### 方案 A(推荐)

- 每个 condition 在 Baseline 前 N 步(N = 10,2026-11-17 起 N = 5,见下文"现行实现")估计一次,之后整个 condition 固定。
- 对每个有效步:dₖ = (ψₖ − sₖ) − FPA_当前方法,raw(未取反);φ_f = dₖ 的圆周均值。
- 理由:刚校准完骨盆尚未漂移,当前方法可信;离线验证两份录制开头差值在 ±1° 内。
- 估计误差约 3°/√10 ≈ 1°。
- 不随阶段间重校准更新:步幅法不靠重校准对抗漂移;每个 block 开头重估会被训练行为污染;安装角只在传感器滑动时才变。
- 前 N 步(实现变更):φ_f 估出前,开关打开时也用当前方法的 FPA 做反馈/计入基线,不做补算。
  理由:φ_f 定义为这 N 步"步幅法 − 当前方法"的平均差,补算出的 N 步均值与当前方法这 N 步的均值相同,对 Baseline 均值无差别,省掉补算机制。
- ψₖ 用脚的绝对航向(不减校准参考),所以阶段间重校准不影响 φ_f。
- 检查:N 个 dₖ 的标准差 > 5° 时日志报警(说明前 N 步里已有漂移或离群)。

### 现行实现(2026-11-17):方案 B 优先,与方案 A 交叉检查,不一致时退回 A

- 零点估计阶段(反馈仍用当前方法)两种零点同时估:
  - 方案 B:φ_B = 校准时脚的航向 − 校准后最初 5 个直线步(G1–G7)的平均方向;返程步(与第一个相差 > 90°)按 180° 折回,
    所以房间小、需要多个来回也能凑齐。
  - N 从 10 改为 5(`StrideZeroEstimator.RequiredSamples`,方案 A 同用):φ_B 拿这些步与校准时的航向比较,
    VRU 下传感器航向每分钟漂 2–10°,凑步越久偏差越大;小房间每个来回可用的直线步少,10 步可能要 40–60 s。
    Ps2:N = 5 与 10 的 φ_B 相差 0.3°(凑齐用时 8 s 对 17 s);随机误差 3.2°/√N 约 1.4° 对 1.0°。
  - 方案 A:同上文(借当前方法,骨盆参照)。
- 两个都估出后定下每只脚的零点:|φ_A − φ_B| ≤ 15°(`FpaEngine.ZeroCrossCheckMaxDeg`)用 B,否则退回 A 并报警。
- 零点定下前(步幅法模式)只把积分有效(G1–G4)的步计入:Pf2(11-17,脚几乎没动,每步 1 cm)时当前方法曾把晃动当成 35 个基线步。
- MTw 滤波器重启(StatusWord OrientationValid 1 → 0,`GaitPipeline` 在 DataQualityGate 前检测,Events 记 `FilterRestart`):
  - 原因线索:Po9 两脚同时重启前后角速度达 1748 / 1984 °/s(量程 2000 °/s),即佩戴时被甩、翻、磕碰。
  - 每次:约 2 s 无数据(整帧被丢),前后再丢 1–2 步(G2 缺帧、G5 方向跳变),之后 10–15 s 精度稍差。
  - 零点定下后:不处理(FPA 只比较同一传感器同一步内的量,φ 与坐标系无关)。
  - 零点定下前,脚:校准航向属于旧坐标系,这只脚零点作废、停止输出,Events `StrideZeroInvalidated`,日志提示重新校准;
    该脚校准参考改变(重新校准)后自动重估。骨盆:不再输出零点前的 PD 结果、不再收集方案 A,方案 B 估出即用(不交叉检查)。
- 超时:φ_B 估出后再过 10 个步(`FpaEngine.ZeroCrossCheckTimeoutStrides`)方案 A 仍未估出(开头 PD 一直无效 / 卡在
  ReacquiringPd),不做交叉检查直接用 B,日志与 `StrideZeroCheck` 注明 "not cross-checked"。
  Events 记 `StrideZero`(采用的 φ)和 `StrideZeroCheck`(差值与采用哪个)。
- 原因:Ps2(1 分钟直线测试,校准面朝路线、双脚贴平行胶带、走路时刻意踩直):方案 A 左 −2.5、右 +7.7,方案 B 左 +1.1、右 +4.0。
  A 的左右反向偏差约 ±3.5°,来自走路时骨盆相对站立转了约 10° 而脚没有;B 只用脚自己的传感器,不受骨盆影响。
- B 的前提:校准时面朝行走路线,校准后直接往前直走。Po9、Pl89 的校准没有面朝路线(从站姿到最初几步脚转了 30–70°),
  B 偏 80–150°,若直接采用会让所有步被 G8 排除、整场无输出;退回 A 后结果与改之前相同。
- 操作要求:地上贴行走路线和站位胶带,校准时面朝路线、双脚平行脚尖朝前,校准后直接往前走。

### 方案 B(备选,不用骨盆)

- φ_f = 校准时脚的航向 − 早期直行的行走方向(按 180° 折叠)。
- 缺点:校准时身体整体斜对路线(如 20°)会让两脚 FPA 都偏 20°;方案 A 不会。

## 二、门控

步幅法模式下不再使用 `PdEstimate.IsValid` / `PdStabilityThreshold`,也不再依赖"ReacquiringPd 等 PD 与骨盆夹角 ≤15°"。

| 编号 | 条件 | 初始阈值 | 作用 |
|---|---|---|---|
| G1 | 摆动时长 | 0.25–1.5 s | 排除一次着地被拆成两次、长时间不动 |
| G2 | 摆动期缺帧 | ≤ 5 帧(实现变更,原 ≤ 2) | 缺帧使积分出错 |
| G3 | 水平步长 | 0.3–2.5 m | 排除碎步、原地转、坏积分 |
| G4 | ZUPT 前末端残余速度 | < 1.0 m/s | 积分质量 |
| G5 | 与同一只脚上一步方向差 | < 15° | 排除转弯中及转弯后第一步 |
| G6 | 另一只脚最近一步也是直走(它自己的 G5 通过)(实现变更) | — | 两脚都在直走 |
| G7 | 运动状态 | 非 Turning | 骨盆角速度判定转弯,不受航向漂移影响,保留 |
| G8 | FPA 合理范围 | \|FPA\| < 60° | 兜底 |

- ReacquiringPd:步幅法模式下只要 G5/G6 成立就照常输出。
- 结算时机不变:第 k 步步幅在着地时已完整,FPA 仍在着地后约 100 ms(`SettleMinStanceMs`)输出。
- 新增 `StepExclusionReason`:`StrideInvalid`(G1–G4、G8)、`StrideNotStraight`(G5–G6),进入 QoE 统计。
- 阈值为离线数据初值,需在回放测试中调整。
- 回放 11-12 16:38 后的两处实现变更:
  - G6 原为"与另一只脚最近一步方向差 < 15°"。每个传感器的航向参考各自独立(VRU 下任意且各自漂移),
    回放中左脚步幅方向约 73°、右脚约 −30°,原 G6 让所有步都失败。改为只看另一只脚是否也在直走,不比较两脚绝对方向。
  - G2 原为 ≤ 2。缺帧多数是 `DataQualityGate` 突变检查在摆动期拒掉的帧(不是无线丢包),只有 37% 的步缺 0 帧;
    缺 3–5 帧的步与缺 0–2 帧的步,相邻步方向差相同(中位数 2.9° vs 2.8°,p90 8.1° vs 8.3°),故放宽到 5。

## 三、实现方式

1. 新增 `StrideDirectionTracker`(每脚一个):摆动期积分 free acceleration,着地时给出 sₖ、步长、残余速度、丢包情况。
2. `FpaEngine` 增加开关 `UseStrideDirection`(默认关)。打开时用步幅法 FPA 做反馈,当前方法只记录;关闭时行为不变。
   `FpaEngine.Reset()`(重校准、训练开始)保留 φ_f;`ResetCondition()`(新 condition,由 `GaitPipeline.Reset()` 调用)重估。
3. 两种 FPA 都写入 DiagnosticsRow(新列 `FPA_Left_PdDeg`、`FPA_Left_StrideDeg`、`FPA_Right_PdDeg`、`FPA_Right_StrideDeg`,
   `FPA_Left_Deg`/`FPA_Right_Deg` 仍是反馈用的值);RunConfig 记录开关 `UseStrideFpa`;φ_f 估出时写 Events `StrideZero`
   (RunConfig 在 Start Condition 时写入,那时 φ_f 还没有),标准差 > 5° 时日志报警。
4. 回放测试(用 2026-11-12 16:38 的 11 分钟自然步态录制 `Diagnostics_20261112_164947.csv` / `ImuSamples_20261112_164938.csv`):
   - 正常段每 30 s 中位数波动 < 3°;
   - 内八段检测到 ≥ 10° 变化;
   - 后 4 分钟(当前方法卡在 ReacquiringPd)有输出;
   - 被丢弃的步 < 20%。

## 四、已知问题与相关改动

- 每步噪声约 2–4°(文献平均误差约 2.6°),相对 ±4° 容差不小,需与当前方法同时记录对比。
- 只适用于地面行走;跑步机不可用。步长 < 0.3 m(拖步、极慢)时方向不可靠。
- 更依赖着地检测质量(决定积分起止)。
- VRU profile 下 free acceleration 是否正常输出待离线确认。(着地检测 `GaitEventDetector` 本来就依赖 free acceleration,回放数据中全部帧都有)
- FPA 定义变为"相对本脚行进方向",旧数据不能直接比较。
- IF 条件的 `live` 实时角度(`MainWindow.xaml.cs`,"NOTE(2026-11-12) 暂时停用"注释处)仍基于骨盆,受漂移影响;计划改为"脚偏航 − 该脚上一步步幅方向"。

## 五、离线验证结论(2026-11-12)

| 录制 | 结果 |
|---|---|
| 11-12 15:04 | 当前方法左 −6 → −24、右 +11 → +35;步幅法左约 −9、右约 +11 平稳 |
| 11-12 14:44 | Baseline/Training1 两法一致;Training2 左脚分开(差约 −11),推断左脚传感器漂移 |
| 11-10 14:50 | 当前方法右脚 +9 → −24;步幅法右脚 +2…+3 平稳,解释了"需外八才能触达 target"的体感 |
| 11-12 16:38(4 分钟正常 + 1 分钟内八 + 4 分钟正常) | 步幅法正常段左约 −9、右约 +11(±2°),内八段左 −15°、右 −14°,结束后回到原值;当前方法正常段漂 25–35°,后 4 分钟无输出 |

C# 实现回放(`StrideFpaReplayTests`,开关打开,2026-11-13):

| | 左脚 | 右脚 |
|---|---|---|
| φ_f(标准差) | 12.8°(2.7°) | 1.9°(2.7°) |
| 正常段中位数 / 30 s 中位数最大偏差 | −8.5° / 2.2° | 10.7° / 1.3° |
| 内八段变化 | −17.9° | −16.8° |
| 后 4 分钟步数(当前方法此时无输出) | 135 | 139 |
| 着地 575 / 573 中:输出 / G1–G4,G8 / G5–G6 / 转弯 | 383 / 38 / 102 / 45 | 390 / 59 / 79 / 38 |

- 验收标准的两处调整:"正常段"排除内八最低点前 1 分钟到后 2 分钟,只统计 ≥ 10 步的 30 s 格。
  内八结束后约 1.5 分钟内两脚(各自独立的传感器)一起慢慢回到原值,判断为步态恢复而非漂移;
  按原定义(只排除内八 ±1 分钟)右脚 t=450 s 一格偏差 3.2°,超过 3°。
- 当前方法同一份数据:正常段左脚 −7 → −29、右脚 +15 → +54(骨盆漂移),约 t=450 s 后无输出。

同一份 16:38 数据的纯陀螺模拟(不用磁力计,校准时估一次零偏):10 分钟骨盆约漂 45°、左脚约 10°、右脚 20–30°;human 滤波骨盆第 2 段约漂 110°。步幅法只要求约 1 秒内航向一致,两者都不影响它。

## 六、滤波器预热(2026-11-13)

### 文档依据

MTw Awinda 白皮书(https://www.movella.com/hubfs/Downloads/Whitepapers/MTwAwinda_WhitePaper.pdf)§IV-A:
"a short period of time just after startup (10 s to 15 s) is needed with no or low dynamic movements and preferably
without magnetic distortions. This process is called 'filter warm-up'. After filter warm-up, the data acquisition can start."

- 白皮书没有定义 "startup" 和 "low dynamic" 的具体含义,也没有给阈值。
- 姿态滤波器 XKF3-hm 运行在 PC 端的 XDA 里(白皮书 §II-C、Fig. 3;代码里接口名为 `setXdaFilterProfile`),
  传感器端只做采样和 SDI。因此 "startup" 应理解为 XDA 开始为该传感器运行滤波,即软件进入测量(`gotoMeasurement()`),
  而不是传感器上电。

### 实测(最近 8 份 ImuSamples)

- 每份录制的第一个包之后 2.01 s,三个传感器的 StatusWord 才出现 0x2(滤波器有效),之前为 0x0:
  录制从测量开始录起,滤波器每次测量都从头开始。
- 16:38 录制开头约 15 s 静止站立:航向游走 1–2°;free acc 残余 0.11–0.16 → 0.05–0.06 m/s²(约 12 s),
  站定后 0.03–0.06 m/s²。收敛过程可见,但量级远小于几分钟内 25–35° 的漂移,不是漂移主因。
- 对步幅法:0.1 m/s² 残余在 0.6 s 摆动期内积分约 2 cm,ZUPT 去漂后更小,可忽略。
- 校准时间(从测量开始算):11-12 16:38 22.4 s、15:04 12.4 s、14:44 22.9 s、14:33 50.5 s;
  11-10 15:09 29.2 s、14:50 14.4 s、14:34 31.6 s、13:51 21.3 s。两份早于 15 s,但 22 s 才校准的 16:38 同样漂移,不能据此归因。

### 预热方式(已决定:桌上预热)

- 现状:所有绑定角色的 IMU 连上后软件自动 `StartMeasurement`(`MainWindow.xaml.cs` OnMtwConnected),
  滤波器从"传感器连上 station"那一刻开始。
- 决定(2026-11-13):传感器连上后先平放在桌上预热,完成后再给参与者佩戴。
  理由:至今未观察到 IMU 断连;由操作员完成,不占参与者时间;代码改动小。
- 前提:测量从预热到 condition 结束不中断;桌面附近避开铁磁物体和电脑(human profile 下影响初始航向,VRU 下基本无关);
  佩戴后仍做现有的 2 s 静态校准。
- 备选(佩戴后站立预热)仍然可行,门控设计对两种方式都适用,见下文。
- "低运动"无官方阈值。推断含义:加速度计读数接近重力(用于初始化倾角)、陀螺读数接近零偏(用于估计零偏)。
  实测安静站立 |gyro| 0.6–1 °/s、|acc| − g 约 0.01 m/s²;建议门槛 |gyro| < 3 °/s、||acc| − g| < 0.3 m/s²(待验证)。
  `CalibrationProcessor.StaticGyroThreshold` 0.3 rad/s(约 17 °/s)作为预热判定过松。

### 预热门控设计(已实现 2026-11-13,`Pipeline/WarmupMonitor.cs`,测试 `WarmupMonitorTests`)

**判定规则**

- 起点:每个传感器第一个 StatusWord 含 OrientationValid(0x2)的包。之前的帧 `DataQualityGate` 已经丢弃,保持不变。
- 计时用 PacketId 差 / 采样率(100 Hz),丢包不影响计时。
- 每个传感器各自判定(2026-11-17 修改,原为"三个同时连续静止 15 s"):自己连续低运动
  (|gyro| < 3 °/s 且 ||acc| − g| < 0.3 m/s²)满 15 s → 该传感器完成并锁定,之后再动也不影响;三个都完成 → 预热完成。
  中途有动作只让该传感器从 0 重新计。桌上静置时总是一次满足;如果传感器是在佩戴途中才连上,则自动退化为"佩戴后站立 15 s"。
- 某个传感器的 OrientationValid 消失(其滤波器重新初始化)→ 只有它重新预热,其它已完成的保持;
  测量重新开始(`DeviceState` 重新进入 MEASURING)→ 全部重置。
  修改原因:11-16 15:30 三个 IMU 桌上预热完成后,佩戴时左脚滤波器被 XDA 重新初始化(无断线),
  按原规则要三个同时静止 15 s,佩戴动作让三个反复归零,一直完成不了。
- StatusWord 为空(回放 CSV)时视为有效。

**拦截位置**

- 只拦截 `BtnStartCondition_Click`:与现有 "IMUs are not measuring" 检查并列,未完成时提示并返回。
- 不拦截阶段间重校准(`TriggerRecalibration`)和 `RestartCondition`:它们发生在同一次测量中,预热早已完成。
- 如果 condition 进行中测量被重新开始(例如误点 Measure,它是开关),预热会重置;此时不拦截校准(不能卡住实验),
  只在 Events 里记 `WarmupIncomplete`,事后可识别。

**需要修改的地方**

1. 新增 `Pipeline/WarmupMonitor.cs`(约 50 行):`Update(ImuSample)`、`Reset()`、`IsComplete`、`StillSeconds`。
2. `MainWindow.xaml.cs`:
   - `OnXsensData` 里在 `_magMonitor.Update(sample)` 旁调用 `_warmup.Update(sample)`(暂停时数据不进来,预热也暂停计时,符合预期);
   - `OnDeviceStateChanged` 进入 MEASURING 时 `_warmup.Reset()`;
   - `BtnStartCondition_Click` 增加检查;
   - 状态栏显示 "Warm-up 8/15 s — keep IMUs still" / "Warm-up done";
   - RunConfig 记录阈值、`MeasurementStartPacketId`、`WarmupDonePacketId`;
     "测量开始 → 首次校准"的秒数 = (Events 中 CalibrationStart 的 PacketId − MeasurementStartPacketId) / 采样率。
3. 单元测试 `WarmupMonitorTests`:静止 15 s → 完成;中途动作 → 重新计;测量重新开始 → 重置;OrientationValid 前的帧不计;StatusWord 为空视为有效。
4. 操作流程写入 `docs/PROJECT_CONTEXT.md` §3.9:"传感器连上后在桌上静置,状态栏显示 Warm-up done 后再佩戴"。

**影响**

- 操作流程:多一步桌上静置。一般佩戴本身就超过 15 s,只要连上后先别拿起来,几乎不增加时间。
- 算法与数据:不改校准、FPA、PD 任何逻辑;ImuSamples、Diagnostics 格式不变;RunConfig 多两个字段。回放与已有测试不受影响(门控在 MainWindow,不在 `GaitPipeline`)。
- 不能解决航向漂移:实测预热期间误差仅 1–2°,门控只保证校准参考姿态取自已稳定的滤波器。
- 未验证的假设:
  - 桌上收敛的状态能延续到佩戴后。传感器从室温到贴身温度,陀螺零偏会变;MTw 有温度补偿,剩余变化由滤波器跟踪。
    验证方法:对比桌上预热的录制与 16:38(佩戴后预热)在校准站立时的 free acc 残余和航向游走。
  - 测量不重新开始时 XDA 滤波器状态不会重置(目前只确认了每份录制开头都会重新初始化)。
- 阈值 3 °/s、0.3 m/s²、15 s 为初值;桌上静置远低于阈值,主要风险是佩戴后站立的情况下判得过严,需实测。

## 待确认

1. 零点用方案 A。(已确认 2026-11-13)
2. 开关打开时步幅法 FPA 做反馈,当前方法只记录。(已确认)
3. 门控阈值先用初值,回放测试中调。(已确认;G2、G6 已按回放调整,见二)
4. 预热门控:只拦 Start Condition、连续 15 s、阈值 3 °/s / 0.3 m/s²(预热方式已定为桌上预热)。
