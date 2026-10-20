# IMUMoCap 项目背景文档 (for prime-agent)

> 本文档基于 2026-09-04 首次调研整理,并于 2026-09-06 根据后续改动(3-block 训练流程恢复、Rest+MC/问卷审计标记、AR 前端 Unity 工程落地)更新,又于 2026-09-16 根据后续改动(`live` 广播改为自动门控取代 `ChkLiveAngleToAr` 手动开关、EF 新增实际脚印短暂叠加显示、Session 卡片新增 Open Data Folder 按钮)更新。2026-10-20 又对照当前代码整体校对了一遍(标定/站立/结算参数、rest 自动推进 + 赛前重标定、`continueTraining` 移除、IMU 配置外置、AR 反馈渲染三模式等,见各节)。所有结论均来自实际读取源码 / 文档得出,而非猜测。

## 1. 项目是什么

IMUMoCap 是一个基于 Xsens MTw Awinda 无线 IMU 传感器的**步态分析实验系统**,核心场景是评估/训练受试者的 **足部前进角 (Foot Progression Angle, FPA)**:先做静态标定和基线采集,得到受试者个性化的目标角度,再在训练阶段给受试者实时反馈(EF = 显式反馈 / IF = 隐式反馈两种实验条件),同时通过 WebSocket 把状态和角度数据广播给一个 **AR 头显客户端** 做可视化。

系统分两部分,**均已在仓库内实现**:
- **PC 端后端 = WPF 桌面应用**(`IMUMoCap/`,完整实现,含 UI,详见第 3 节)
- **AR 前端 = Unity + XREAL 工程**(`IMUARFPA/`,完整实现,详见第 4 节;首次调研时该目录尚不存在,是后续新增的)

## 2. 代码结构总览

顶层目录(`D:\SourceCode\IMUMoCap`):

```
IMUMoCap/            # WPF 应用主体
IMUMoCap.Tests/       # 测试项目(pipeline 回放测试)
IMUMoCap.sln
docs/                 # 项目文档 + 示例数据 + Python 原型脚本
```

`IMUMoCap/` 下关键子目录:

| 目录 | 内容 |
|---|---|
| `Pipeline/` | 步态分析核心流水线(见第 3 节) |
| `Pipeline/Models/` | 流水线用到的数据模型 / DTO |
| `VM/` | `MainPageVM.cs` —— 唯一的 ViewModel |
| `Model/` | `DeviceModel.cs`, `IMUData.cs`, `ImuFrameModels.cs`, `ImuViewModel.cs`, `PipelineConfigModels.cs` |
| `Methods/` | `CsvUtil.cs`, `ExperimentRecorder.cs`, `ImuFrameCollector.cs`, `Utils.cs`, `WebSocketBroadcastServer.cs` |
| `Device/` | `MyXda.cs`(Xsens XDA 封装)、`MyEventArgs.cs` —— 底层 Xsens SDK 胶水代码 |
| `Services/` | `ImuDeviceManager.cs`, `ImuSlotRegistry.cs` —— 设备连接/槽位管理 |
| `AHRS/` | 姿态角计算辅助类 |
| `wrap_csharp64/` | SWIG 自动生成的 Xsens Device API (XDA) C# 绑定 |
| `extralibs/` | Xsens 原生 DLL(`xsensdeviceapi64.dll` 等) |

顶层文件:`App.xaml(.cs)`, `MainWindow.xaml(.cs)`(约 2100 行,几乎所有业务逻辑都在这里)、`imu_sensors.json`(IMU 槽位配置,见 §3.1), `ParamsDialog.xaml(.cs)`, `PauseResumeDialog.xaml(.cs)`。

**架构特点**:没有用 MVVM 的 ICommand/RelayCommand 模式,`MainPageVM` 只是一个属性包(property bag),按钮点击直接绑定到 `MainWindow.xaml.cs` 的事件处理方法,由 code-behind 直接调用 pipeline / recorder / WebSocket server。

## 3. PC 端后端(WPF 应用)详解

### 3.1 IMU 数据接入

不是自定义网络/串口/蓝牙接收器,而是通过官方 **Xsens Device API (XDA)**:
- `Device/MyXda.cs` 封装 `XsControl`/`XsScanner`,回调类 `MyMtwCallback`(单传感器数据回调)、`MyWirelessMasterCallback`(设备连接/断开事件)
- `Services/ImuDeviceManager.cs` 在其上层暴露高层事件:`MtwConnected`、`MtwDisconnected`、`DataPacketReceived`、`BatteryLevelChanged`、`StateChanged`
- 传感器槽位由 `IMUMoCap/imu_sensors.json` 配置(`ImuSensorConfigEntry`:`SlotName`/`DeviceId`(十六进制字符串)/`Role`),启动时由 `MainWindow.LoadOrCreateImuConfig()` 读入 `ImuSlotRegistry`;文件不存在时自动用旧的 3 个硬编码传感器生成。现有 8 个槽位,`ImuRole` 枚举新增 `None`:Pelvis `0x00B43CAB`、Left `0x10B41913`、Right `0x10B41904` 已绑定角色,其余 5 个为备用(`Role=None`)。**只要求已绑定角色的传感器全部连上就自动开始测量**;在参数对话框里改角色后点 Save Params 会回写该文件(`SaveImuConfig()`)。默认 100Hz
  - 注:以上 IMU 配置外置属于 2026-10-20 的**未提交**改动(`imu_sensors.json`、`ImuSensorConfigEntry.cs`、`ImuRole.None`、`ParamsDialog.xaml` 的角色选择 UI)。

数据流:`OnDataPacket` → `OnXsensData(role, deviceId, packet)` → `ImuFrameCollector.Process(...)` 同步汇总三个传感器 → `GaitPipeline.ProcessPacket(...)`

### 3.2 IMU 原始数据字段(`ImuSampleFrame`,由 `ImuFrameCollector.BuildImuSample` 从 Xsens `XsDataPacket` 解出)

| 字段 | 来源 | 说明 |
|---|---|---|
| `PacketId` / `PacketCounter` | `packet.packetId()` / `packetCounter()` | 用于跨传感器按包序号对齐(见下) |
| `TimeSec` | `(PacketId - firstPacketId) / SampleRateHz` | 相对起始时间,100Hz 采样 |
| `StatusWord` | `packet.status()` | 位标志:方向有效性、饱和/削波等,见 §3.3 |
| `Rssi` | `packet.rssi()` | 无线信号强度(dBm),质量门控中当前**已注释关闭**(见 §3.3) |
| `Quaternion` | `orientationQuaternion()` | 传感器姿态四元数(Xsens 板载 Kalman 滤波器输出) |
| `RateOfTurn` | `calibratedGyroscopeData()` | 角速度(rad/s),标定后 |
| `FreeAcceleration` | `freeAcceleration()` | 已扣除重力的线加速度(m/s²),stance 判定主信号 |
| `Acceleration` | `calibratedAcceleration()` | 标定后原始加速度(含重力) |
| `MagneticField` | `calibratedMagneticField()` | 磁力计读数(标定后) |
| `DeltaQ` | `sdiData().orientationIncrement()` | 姿态增量四元数(SDI, strapdown integration),用于转弯检测的辅助信号 |
| `DeltaV` | `sdiData().velocityIncrement()` | 速度增量(当前 pipeline 未使用) |

**跨传感器帧同步**(`ImuFrameCollector.UpsertFrameBundle`):以 `PacketId` 为键,把同一时刻的 Pelvis/LeftFoot/RightFoot 三个 `ImuSampleFrame` 聚合进一个 `ImuFrameBundle`;三者到齐(`IsComplete`)才产出完整帧。若某个 packetId 等了超过 `MaxPendingPacketLag`(16 包)仍未到齐,视为丢帧(`PelvisGapFrames`/`LeftFootGapFrames`/`RightFootGapFrames` 计数,最终生成 `FrameQualityReport`,含 `CompletionRate`)。

### 3.3 数据质量门控(`DataQualityGate.Evaluate`,逐 bundle 校验,任一失败整帧丢弃)

1. **StatusWord 检查**(`HasStatusError`):`XSF_OrientationValid` 位为 0(滤波器未收敛)→ 拒绝;`XSF_ClippingDetected` 位为 1(任一轴加速度计/陀螺仪/磁力计饱和削波)→ 拒绝。三个 IMU 任一不合格则整帧丢弃。
2. **RSSI 检查**:代码中存在(`RssiThresholdDbm = -50`)但当前**被注释掉未生效**。
3. **加速度突变检查**(`HasAccSpike`):与上一帧比较,`Acceleration` 欧氏距离 > `AccDeltaThreshold_ms2`(100 m/s²,专门调高以避免正常跺脚触发)→ 拒绝。
4. **角速度突变检查**(`HasGyroSpike`):与上一帧比较,`RateOfTurn` 欧氏距离 > `GyroDeltaThreshold_rads`(20 rad/s)→ 拒绝。

通过后包装为 `ValidFrame`(仅是 bundle 的只读视图,不复制数据),进入下游 pipeline。

### 3.4 静态标定(`CalibrationProcessor`)

操作员点击触发(`TriggerStart()`:`WaitingForStart → CollectingStaticPose`)。每帧检查三个 IMU 的 `RateOfTurn` 模长是否都小于 `StaticGyroThreshold`(0.3 rad/s);连续静止满 `StaticRequiredFrames`(20 帧,0.2s)才开始采集四元数样本,采满 `StaticCollectFrames`(200 帧,2s)后完成;短暂抖动只暂停采集、不清空已收集的缓冲区;超过 `StaticTimeoutFrames`(1000 帧,10s)未完成则 `Failed`。三个 IMU 各自的参考姿态 `PelvisRef`/`LeftFootRef`/`RightFootRef` 取采集样本**四元数分量中位数后归一化**(比均值更抗离群值),构成 `CalibrationProfile`,后续所有朝向计算都以此为零点(相对旋转 `q_rel = Inverse(q_ref) * q_measured`)。**每个训练/Retention 阶段进入前都会再标定一次**(约束 IMU yaw 漂移,见 §3.8),标定流程与首次标定相同。

### 3.5 Stance/摆动检测(`GaitEventDetector.Detect`,每帧每脚独立判定)

三重 AND 条件同时满足才算 stance(站立):
1. `FreeAcceleration.Length() < FreeAccStanceThreshold`(2.5 m/s²)—— 脚无明显线加速度
2. `RateOfTurn.Length() < GyroThreshold`(1.0 rad/s)—— 脚无明显旋转
3. **姿态倾斜检查**(yaw 无关):取标定参考下的重力方向 `gRef = Rot⁻¹(calRef)·(-Z)` 与当前姿态下的重力方向 `gNow = Rot⁻¹(q_current)·(-Z)`,两向量夹角 `acos(gRef·gNow) < FootPitchThreshold`(0.35 rad ≈ 20°)。用夹角而非旧版"提取相对四元数的 pitch 分量"是因为后者在转身后会与朝向变化耦合出错(代码注释明确说明是修过的 bug)。

三条件都满足后还要求**连续 `MinStanceFrames`(2 帧)防抖**才确认 stance(研究团队已从 5 帧下调到 2 帧;再调低会把一次落地拆成多次假 stance)。`IsWalking` 标志:3 秒滑动窗口(`WalkingWindowFrames=300`)内左右脚各自的 stance↔swing 切换次数都 ≥ 2(即至少完成一个完整周期)才判定为"正在走"。

### 3.6 转弯过滤(`MotionContextDetector` + `ProgressionDirEstimator` 协同,这是 FPA 计算能否输出的关键门控)

**目的**:FPA(足部前进角)只有在"直线行走"时才有意义 —— 转弯时脚的朝向相对身体前进方向会剧烈变化,若不过滤会把转弯动作误判成极端的 toe-in/toe-out。因此系统用一个三态状态机 `Straight → Turning → ReacquiringPd → Straight` 来识别并剔除转弯及其后的方向重建期。

**转弯信号判定**(`MotionContextDetector.Detect`,双路 OR):
1. **骨盆 yaw 角速度**(主信号):把 Pelvis 传感器系下的 `RateOfTurn` 用当前姿态四元数旋到世界系,取 Z 分量绝对值,超过 `YawRateThreshold_rads`(0.70 rad/s ≈ 40°/s;代码注释说明该值特意从 0.26 调高,因为正常步行时骨盆 yaw 峰值本身就到 0.5 rad/s,阈值太低会把正常摆髋也误判为转弯)。
2. **DeltaQ 累积增量**(辅助信号):对 Pelvis 的姿态增量四元数逐帧提取 yaw 分量(`2·atan2(dq.Z, dq.W)` 小角度近似)并累加,累积值绝对值超过 `DeltaQYawThreshold`(0.17 rad)。

**状态转换逻辑**:
- `Straight → Turning`:转弯信号连续满足 `TurningConfirmFrames`(10 帧)才切换,置信度随累积帧数线性下降(`1 - frames/threshold`),防止瞬时噪声误触发;确认切换时清空 DeltaQ 累积量。
- `Turning → ReacquiringPd`:转弯信号连续**消失** `StraightConfirmFrames`(20 帧,比进入转弯的确认帧数更长,非对称设计防止转弯尾声抖动来回跳变)才退出,进入"重新获取行进方向"过渡态(此态置信度恒为 0,FPA 引擎不会输出)。
- `ReacquiringPd → Turning`(重新转弯):在 ReacquiringPd 态若转弯信号又连续出现 `ReacqTurningConfirmFrames`(10 帧)则判定为二次转弯,退回 Turning(防止步态摆动产生的 yaw 尖峰把 ReacquiringPd 立即弹回 Turning)。
- `ReacquiringPd → Straight`:**不由 MotionContextDetector 自己判定**,而是由 `ProgressionDirEstimator.ConfirmStraight()` 外部调用触发(见下)。
- 辅助投票:若当前 `Straight` 但 `GaitEvent.IsWalking == false`(比如站着不动),置信度打 5 折,但不会改变状态本身。

**行进方向(PD)重建**(`ProgressionDirEstimator.Update`,仅在 `Straight`/`ReacquiringPd` 态更新):
- 融合三路朝向信号加权平均(圆周角度用向量和 `atan2` 处理,避免 ±180° 环绕问题):骨盆瞬时 yaw(权重 0.6,主信号、最稳定)、左脚 stance 相位内的平均 yaw(权重 0.1)、右脚 stance 相位内的平均 yaw(权重 0.1)。**2026-10-20 起脚的权重由 0.2 降为 0.1,且只有双脚同时偏向骨盆朝向的同一侧才起作用(见 §3.7 的 PD 融合说明)。**每只脚的朝向在其整个 stance 周期内用向量和累积后 `atan2` 求平均,swing→stance/stance→swing 转换时才产出一次"该脚这一步的朝向"。
- **每当左右脚都各自产出一次新朝向**(双脚都完成了一次 stance→swing 转换)才触发一次融合更新、计入一次"PD 步数"。
- **稳定性(Stability)**:滑动窗口(`StabilityWindow=10` 步)内历次融合朝向的圆形方差 `circVar = 1 - R̄`(R̄ 为平均合成向量长度,1=完全一致方向,0=随机分布),`Stability = 1/(1+circVar)`。
- **Turning → ReacquiringPd 切换时的处理**:清空 PD 历史窗口和步数计数,不让转弯前后的方向混在一起算稳定性,避免"旧方向 + 新方向"平均出一个错误的过渡值。
- **ReacquiringPd → Straight 退出条件**(三者同时满足,由 `ProgressionDirEstimator` 判定后调用 `contextDetector.ConfirmStraight()`):
  1. `Stability >= StabilityThreshold`(0.85)
  2. 已累积 `MinStepsBeforeValid`(2)步新的融合估计
  3. 当前融合 PD 与骨盆瞬时朝向夹角 `<= PelvisAgreementThreshold_deg`(15°)—— 防止双脚朝向还没跟上骨盆转身进度时就提前锁定一个偏差很大的 PD

**IMU 传感器安装轴自适应**(`DetectHeadingAxis`):标定时通过参考四元数把世界系重力向量反变换回传感器坐标系,判断哪个传感器本体轴与重力(竖直方向)最接近对齐,该轴即视为"朝向/yaw 轴"。这样即使传感器绑扎方向(躺放/立放)不同,也能自动选对提取 yaw 的分量公式,而不是写死假设 Z 轴垂直。

### 3.7 FPA(足部前进角)计算(`FpaEngine.Process`,每帧调用)

核心公式:**`FPA = mean(该脚 stance 期间的 yaw) - PD.DirectionRad`,弧度转角度;正值 = toe-out(外八),负值 = toe-in(内八)。**

**采集与门控分离设计**(注释明确说明的设计原则):采集只要脚在 stance 就持续进行,不受 gate 状态影响(哪怕转弯中也在采集,便于事后统计排除比例);只有"是否允许输出"受 gate 控制。

**采集**(`StanceSampler`,左右脚各一个独立状态机):
- 每帧若 `GaitEvent.LeftStance`/`RightStance` 为真,取该脚四元数相对标定参考的 yaw 差(`ExtractYaw(q) - ExtractYaw(calRef)`,归一化到 ±π),用向量和(`cosSum`/`sinSum`)累积。
- swing→stance 边沿触发时重置累积器(视为新的一步开始)。

**输出门控**(`gate` = `contextOk && pdOk`):
- `contextOk`:`MotionContext.State == Straight` 且 `Confidence >= ContextConfidenceThreshold`(0.7)—— **这就是转弯被排除在 FPA 输出之外的直接机制**:只要处于 `Turning` 或 `ReacquiringPd` 状态,或虽是 `Straight` 但置信度不够,`contextOk` 就是 false,当前 stance 周期无论采集了多少数据都不会产出 `FpaResult`。
- `pdOk`:`PdEstimate.IsValid` 且 `Stability >= PdStabilityThreshold`(0.7)。
- 门控失败时会记录 `StepExclusionReason`(`Turning` / `ReacquiringPd` / `LowConfidence` / `PdInvalid`),供 QoE 统计"转弯排除了多少步",不影响 `FpaResult` 输出本身,只是审计用。

**结算时机**(`TrySettle`,**数据驱动**,原来的固定帧数参数 `MinStanceFramesForSettle` 已不存在):脚从 swing 转入 stance 后,须累积满 `MinStanceFramesFloor`(2 帧),且该脚角速度模长**连续 `SettleQuietFrames`(3 帧)低于 `SettleGyroThreshold`(0.35 rad/s)**(脚真正静止),**且**当前帧 gate 通过,才立即输出该步的 `FpaResult`;若一直不安静,到 `MaxStanceFramesForSettle`(20 帧,200ms)硬上限强制结算。同一个 stance 周期只输出一次(`_emittedThisStance`)。阈值 0.35 是在两份真实录制上验证的(0.2 时正常走路也有 4–8% 的落地因凑不够安静帧被静默丢弃)。若落地后一直没等到 gate 通过(比如整个站立期都在转弯),`MarkSwing()` 在抬脚瞬间会把这一步记为"排除"而非"输出"。

**FPA 值与目标比对**(仅 Training 阶段,`Baseline != null` 时):
- `Error = FPA - Baseline.Target`(带符号,供 AR 端做左/右偏纠正提示)
- 【2026-10-19 起已改为固定 ±4°,见下方新规则】容差带 `w = max(w_min, α·SD)`(`w_min`=4°,`α` = `FpaEngine.ToleranceAlpha`,随训练 block 从 1.5→1.0→0.5 递减,难度递增)
- `OnTarget = |Error| <= w`
- 质量标签 `Quality`:`context.Confidence` 或 `pd.Stability` 任一项距离各自阈值不到 0.1 时标记 `"Marginal"`,否则 `"High"`(即便通过了 gate,余量小也会被标注,供事后按质量过滤数据)

**Baseline 阶段的目标角计算【2026-10-19 起已被下方新规则取代,本段仅作旧规则记录】**(`BaselineProfile.Create`,由 `BaselineProcessor` 在采够 `MinRequiredSteps`(默认 100,2026-09-06 起由 20 调整为 100)步后调用):对每只脚的基线 FPA 样本求均值 `μ` 和标准差 `SD`;若 `μ > 10°` 判定该脚习惯性 toe-out,训练方向定为 `ToeIn`,目标 `T = μ - SD`;若 `μ ≤ 10°` 判定习惯性 toe-in/中性,训练方向定为 `ToeOut`,目标 `T = μ + SD`(即让受试者始终朝"回到中性区间"的方向调整,`k` 固定为 1.0)。同时计算左右步数比 `StepCountRatio = min(stepsL,stepsR)/max(...)`,低于 `ImbalanceRatioThresholdUsed`(默认 0.7)会标 `StepCountImbalanceWarning`,提示双脚数据量不均衡。

**2026-10-19 起生效的 target / 容差规则(取代上文旧规则,代码见提交 `16f5496c`)**:
- Target:默认 ToeIn,`T = max(μ − Δ, TargetMinDeg)`,`Δ = TargetDeltaDeg = 10°`,`TargetMinDeg = −25°`;若 `μ < −25°`(过度内八)改为 ToeOut,`T = μ + Δ`。不再使用 `μ > 10°` 判方向,也不再使用 `SD`(`SD` 仍计算并记录)。`BaselineProfile.ActualDelta_L/R = μ − Target` 记录实际生效的 Δ(被截断者 < 10°,ToeOut 者为负),分析时作为协变量。
- 容差:所有训练 block 共用固定 `ToleranceDeg = ±4°`(`OnTarget = |Error| <= 4°`),已删除 `FpaEngine.ToleranceAlpha` 与 `GaitPipeline.TrainingBlockAlphas`。
- PD 朝向:`ProgressionDirEstimator.WorldHeading` 取相对校准姿态绕世界竖直轴的 twist,取代按传感器轴取欧拉角(旧法对 180° 折返低估约 4°,造成往返方向 FPA 差异 2–5°;新法 ≤2°,依据 `task21_multicalib_pd_replay.py`)。
- AR 端:`TargetForAr()` 发送 `|target|`,`directionL/R` 按 target 符号定(负 = `"toe-in"`),因为 AR 的 `SignedTargetAngle` 把 target 当幅值、由 direction 决定旋转符号。
- **PD 融合(2026-10-20 起)**:脚的权重 0.2 → 0.1(骨盆 0.6 不变),且双脚必须同时偏向骨盆朝向的同一侧才起作用、取较小偏差,否则脚的贡献等于骨盆朝向。依据 2026-10-19 外八测试:右脚刻意转到 −25° 时旧融合把 PD 拉到 −5°,使左脚 FPA 虚增约 5°(0.4° 显示成 4.8°);新规则下 PD 保持在 ±0.4° 内,Training2 的 L 均值 4.8° → −0.1°。2026-10-12 折返数据中往返方向差仍 ≤1.9°,各 block 步数不变。
- **右脚符号约定(2026-10-20 起)**:`FpaEngine` 对右脚的 FPA 取反(`Fpa_R = −(右脚 yaw − PD)`),`live` 的 `angleR` 同样取反,所以左右脚统一为"正 = toe-out,负 = toe-in"。依据:2026-10-19 的双脚外八测试 baseline 为 L = +11.4°、R = −12.1°(几乎对称),说明旧代码里 yaw 以逆时针为正、右脚外八是负值,"正 = toe-out"只对左脚成立。AR 端(`FootHudController`)因此对**右脚所有旋转取反**(stone 目标、`live.angleR`、`fpa.errorR`),因为 Unity Z 轴正向旋转在屏幕上是逆时针,对左脚是 toe-out、对右脚却是 toe-in(俯视图)。2026-10-20 之前录制的 Diagnostics 里右脚列(`FPA_Right_*`)是旧符号,与之后的数据相差一个负号。
- 未处理(用户 2026-10-19 决定暂缓):baseline 100 步内 FPA 并不平稳(示例录制中 L 约 +11°/min、R 约 −9°/min,两脚都在向 0 靠近,疑为行为适应而非传感器漂移),`μ`/`SD` 是被趋势平均后的值。

**设计决策:全部被试统一以 toe-in 为 target 方向(2026-10-19 14:04 记录)**。保持统一方向的理由(用户给出):(1) 内部效度——固定方向使 Δ 在被试间可比;(2) 健康成人以 toe-out 为主、toe-in 人群少(用户 2026-10-19 补充确认),因此多数被试的 toe-in 目标方向有调整空间;μ 本身已很内八者(μ < −25°)按上述规则改为 toe-out。研究问题是"AR 反馈能否让人达到 target",不主张 toe-in 带来生物力学/健康收益。

**已接受的两个代价(2026-10-19 14:04 记录)**:
1. **无法回答"为什么选 toe-in"。** 审稿人可合理追问,而目前没有针对健康人的证据支持 toe-in 优于 toe-out。已核实的相反方向证据:Uhlrich 等 2018(DOI 10.1016/j.jbiomech.2017.11.003)、2022(10.1016/j.jbiomech.2022.111312)、2025(10.1016/S2665-9913(25)00151-1)均显示个体化的 toe-in/toe-out 优于统一方案(结局为膝 OA 患者的 KAM,不是"达到 target 的能力",因此不直接否定本设计,但论文中不得声称统一 toe-in 更优或有健康收益)。
2. **结论无法外推到 toe-out。** 统一方向意味着无法说明"AR 反馈帮助达到 target"的结果对 toe-out 目标是否同样成立;论文需把它列为局限。若日后要补,可改为方向平衡分配,但需要同时为 toe-out 一侧加对称上限(现在只有 `−25°` 下限)。

### 3.8 Pipeline 数据流总览(`GaitPipeline.ProcessPacket` 为主入口,每包调用一次)

```
ImuFrameCollector          (3 IMU 按 PacketId 打包同步为 ImuFrameBundle,见 §3.2)
  → DataQualityGate         (质量校验,坏帧丢弃 → ValidFrame,见 §3.3)
  → CalibrationProcessor    (静态标定状态机 → CalibrationProfile,见 §3.4)
  → GaitEventDetector       (每脚站立/摆动检测 → GaitEvent, IsWalking,见 §3.5)
  → MotionContextDetector   (直行/转弯/PD重获取分类 → MotionContext,见 §3.6)
  → ProgressionDirEstimator (在线前进方向估计 → PdEstimate,见 §3.6)
  → FpaEngine               (门控 + 结算 → FpaResult,逐步产出足部前进角,见 §3.7)
  → DiagnosticsRow          (每帧诊断行,20分钟环形缓冲,OnDiagnosticsFrame 事件)
```

- **Baseline 阶段**:`BaselineProcessor.AddStep(fpaResult)` 累积双脚步数,达到 `MinValidSteps`(=100,`PipelineParams.MinBaselineSteps`)后 `TryFinalize()` 产出 `BaselineProfile`(个性化目标角,见 §3.7 末尾公式)
- **Training 阶段**:`FpaEngine.Baseline` 设为该 profile,结果通过 `OnFpaResult` 广播给 AR 客户端;容差三个 block 共用固定 `BaselineProfile.ToleranceDeg = ±4°`(原 `TrainingBlockAlphas = {1.5, 1.0, 0.5}` 的逐 block 收窄已于 2026-10-19 删除);单 block 在 `GaitPipeline.TrainingBlockTargetSteps`(=100)步/脚有效步数达标后立即结束,另有 `TrainingBlockMaxDuration`(10 分钟)作为纯粹的**停滞保护超时**(非并列退出条件,只是防止流程卡死的兜底)——Baseline 阶段同理有独立的 `BaselineMaxDuration`(10 分钟)兜底
- **阶段流程**(2026-09-06 起已恢复完整 3 个 training block,并按最终协议重新安排了 Rest+问卷窗口位置):

  ```
  Baseline → Training1 → [Rest+MC, 120s 起] → Training2 → Training3
    → [Rest+MC+NASA-TLX(Admin1)+IMI-PC, 300s 起] → Retention → [NASA-TLX(Admin2) 标记] → Ended
  ```

  由 `MainWindow` 的 `ConditionStages = {Baseline, Training1, Training2, Training3, Retention}`/`AdvanceStage()`/`EndTrainingBlock()`/`EnterRest()`/`CompleteRetention()` 管理。要点:
  - **Block2→Block3 之间没有 Rest/MC 窗口**,是刻意设计(`EndTrainingBlock()` 对 `_currentStage=="Training2"` 直接 `AdvanceStage()`);Rest+MC 只出现在 Block1 后和 Block3 后。
  - 两个 Rest 窗口(`_inRest`)由 PC 自己的 stopwatch 计时,**到点自动推进**(`MainWindow` 60ms 定时器里 `_stageStopwatch.Elapsed >= requiredRestSec` 即调用 `AdvanceStage()`),不再需要 AR 端 `continueTraining` 或操作员 Continue 按钮(二者均已移除)。Block1 后窗口 120s(`RestDurationSec`),Block3 后窗口(含 NASA-TLX Admin1 + IMI-PC,纸面/RA 主持)300s(`PostBlock3RestDurationSec`,协议只规定了问卷本身约 5 分钟,窗口整体下限是估算值,需研究者确认)。
  - **赛前重标定门控(每个 training block / Retention 进入前)**:`AdvanceStage()` 只收尾当前 stage,置 `_pendingStageEntry = 下一 stage`、`_pipeline.AwaitingRecalibration = true`(冻结 fpa/训练输出),向 AR 广播 `armed`(`reason` 为 "Quick recheck before continuing. Tap Ready when you are."),并写审计标记 `Recalibrate_Before{stage}`;参与者在 AR 点 `Start`(`ReadyForCalibration`)→ PC 走 `TriggerRecalibration()`(首次校准则走 `TriggerCalibration()`)→ 校准完成后 `EnterStage()` 才真正开始该 stage。该 `armed` 点击同时充当原来 `continueTraining` 的"恢复"信号。目的是约束 IMU yaw 漂移(依据 `docs/task19_calibration_duration_tuning.py`)。
  - **Retention 完成后不会自动结束录制**:达到 100 步/脚或超时后 `CompleteRetention()` 只停止计步、启用"Mark NASA-TLX #2"按钮,真正关闭 session 文件仍需操作员手动点击既有的 "End Recording" 按钮。
  - **End Recording 前的非阻断完整性检查**(2026-09-08 新增,`MissingExpectedMarkers()`):只检查 3 个**需要操作员手动点击**的标记(`NASA-TLX_Admin1`/`IMI-PC`/`NASA-TLX_Admin2`;两个 `ManipulationCheck_*` 是 `EnterRest()` 自动写入的,不会漏),且只在流程**实际走到过**对应窗口时才视为"应有"——`_restBlockJustFinished==3` 才检查前两者,`_retentionComplete==true` 才检查第三者,避免在提前终止(退出/设备故障)场景里对根本不该存在的标记持续误报。缺失时弹出 Yes/No 确认框而非硬拦截,操作员可选择"仍要结束"覆盖。`_nasaTlx1Marked`/`_imiPcMarked`/`_nasaTlx2Marked` 三个"是否已点击"标志只在 `BtnStartCondition_Click`(开新 condition)时清零,暂停/恢复/Redo 均不影响(纸面量表是否做过是既成事实,不因数据重录而改变)。**`_restBlockJustFinished` 本身也必须在 `BtnStartCondition_Click` 清零到 0**(2026-09-08 补的一处遗漏修复)——否则同一次程序运行里连续跑第二个 condition(比如 EF 跑完接着跑 IF)时,这个字段会带着上一个 condition 结束时的值(如 3)进入新 condition,导致新 condition 明明还没走到 Block3 就被误判"该有 NASA-TLX1/IMI-PC 了",完整性检查和 Flow 面板都会被这个陈旧值污染。
  - **审计标记(纯留痕,不渲染问卷内容)**:`ManipulationCheck_AfterBlock1`/`ManipulationCheck_AfterBlock3` 在进入对应 Rest 窗口时自动记录;`NASA-TLX_Admin1`/`IMI-PC`/`NASA-TLX_Admin2` 由操作员点击对应的 3 个小按钮手动记录(RA 在纸面上实际主持这些量表,软件只留时间戳),写入 `ConditionMeta.StageMarkers`(见 §3.10)。

### 3.9 UI(`MainWindow.xaml` + `.xaml.cs`)

- 顶部深蓝 header:标题 + 已连接 WS/AR 客户端列表(绿点)
- 左侧面板:Session 卡片(Participant ID、Condition EF/IF、Order Group、Start/Pause/Resume/End 按钮;原 Continue-Rest 按钮已移除)、左右 FPA 大数字卡(达标/未达标背景变色)
- **Flow 一览面板**(2026-09-08 新增,`FlowChecklist`/`ChecklistItem`,`Model/ChecklistItem.cs`):Session 卡片内的一行小圆点+标签清单,横向自动换行,10 项对应完整协议流程(Baseline/Training1/MC(Block1)/Training2/Training3/MC(Block3)/NASA-TLX#1/IMI-PC/Retention/NASA-TLX#2)。灰=未到、绿=完成、橙=窗口已到但审计标记还没点。**3 个手动审计标记(NASA-TLX#1/IMI-PC/NASA-TLX#2)对应的"Mark"按钮直接内嵌在各自的流程条目里**(2026-09-08 由独立的一排按钮改为内嵌,`ChecklistItem.MarkerStage`/`IsMarkable`/`CanMark` 驱动,`IsMarkable` 为 true 的行才渲染按钮、`CanMark` 控制是否可点),点哪个标记就在流程图上哪个位置点,不用再去别处找对应按钮;其余 7 行是纯状态展示,没有按钮。所有 3 个按钮共用一个处理器 `BtnMarkFlowItem_Click`(从被点击行的 `DataContext` 读出 `MarkerStage` 分发,而不是各自独立的 3 个 handler)。由 `MainWindow.RefreshChecklist()` 每次 60ms 定时器 tick 时整体重算状态圆点(不是在各处散落推送),纯读现有的 `_restBlockJustFinished`/`_retentionComplete`/`_nasaTlx*Marked`/`_pipeline.BaselineProfile` 等信号,保证不会和真实状态漂移;点击 Mark 按钮时额外立即调用一次 `RefreshChecklist()`,不用等 60ms。让操作员在点 "End Recording" 之前就能持续看到有没有漏项,而不是只在点击那一刻才通过前面提到的确认弹窗(`MissingExpectedMarkers()`)得知。
- 状态栏:3 个 IMU 连接指示灯(Pelvis/Left/Right)+ 丢包计数
- 自绘时间线(`DrawTimeline()`,`DrawingVisual`/`RenderTargetBitmap`,6 条 lane:PD valid / L stance / R stance / IsWalking / motion context / FPA gate reason)
- 底部分栏日志:操作日志 + 独立的 **AR event log**(`ArEventLogBox`,记录每条广播给 AR 的事件,供事后审计)
- `live` 消息广播不再有手动开关(原 `ChkLiveAngleToAr` 复选框已移除);现自动门控为仅 IF 条件 + 训练 block 阶段(`_pipeline.InTraining && !_isRetention && !_inRest`)才发送,EF 不再消费连续 `live` 流(见 §4.1、`docs/AR-FRONTEND.md`)
- Session 卡片内新增 **Open Data Folder** 按钮(`BtnOpenDataFolder_Click`),打开当前 participant/condition 的数据文件夹;原先显示 Session/Meta/QoE 文件名的三行文本已被上面的 Flow 一览面板取代
- 支持 CSV 回放(`BtnLoadData_Click`,0.2x/1x/5x/Max 速度)

### 3.10 `ExperimentRecorder.cs`(数据持久化)

按 participant/condition 管理:
- Session CSV(`P{id}_{EF|IF}_Session.csv`,header = `DiagnosticsRow.CsvHeader`,逐帧写入)
- Meta JSON(`P{id}_{cond}_Meta.json`,`ConditionMeta` 结构:`BaselineProfile`、暂停事件(`PauseEvents`)、redo/attempt 记录(`FinalAttempt`)、每阶段耗时/有效秒数(`StageTimings`)、步数排除统计(`StageStepStats`)、**审计标记列表**(`StageMarkers: List<{Stage, TimestampUtc}>`,2026-09-06 新增,纯追加字段,记录 Manipulation Check / NASA-TLX / IMI-PC 等纸面量表的发生时刻,由 `ExperimentRecorder.MarkStageEvent(stage)` 写入))
- QoE CSV 路径已预留(`CurrentQoeFilePath`)但**目前代码中未见实际写入**
- `P{id}_{cond}_LiveVsFpaConsistency.csv`(`WriteLiveVsFpaConsistency`):每个脚每次 fpa 结算写一行(`PacketId,TsUtcMs,Foot,LiveAngleDeg,FpaErrorDeg,FpaDeg,TargetDeg`),记录该脚最近一次 `live` 角度与最终 fpa 的对比,供事后一致性分析(Task 15),UI 不显示;EF 条件或 IF 首个 live 帧之前 `LiveAngleDeg` 为 NaN
- 文件存于 `Data/P{participantId}/`

### 3.11 `PipelineParams`(可实时调参)

`MainPageVM` 暴露若干可调参数属性(`StaticGyroThreshold`, `StanceFreeAccThreshold`, `PdConfidenceThreshold`, `MinBaselineSteps` 等),经 `MainWindow.SyncParamToPipeline` 实时同步进 `GaitPipeline.Params`,配合 `ParamsDialog` 在 UI 中调整。另有 `docs/summry/demo_params.md` 记录了一套"Demo 模式"(牺牲精度换取响应速度)的参数对照表。

## 4. WebSocket 协议与 AR 前端

> 2026-09-04 首次调研时 AR 前端仓库内完全没有实现代码,只有过时计划文档。**该状态已过时** —— `IMUARFPA/` 目录已作为独立的 Unity 工程加入仓库,协议在双端均已落地。以下按当前状态重写。

### 4.1 协议(PC 端与 AR 端均已完整实现,非纸面设计)

规范文档:`docs/superpowers/specs/2026-07-29-ws-protocol.md`(取代了更早的 `2026-04-09-ui-design.md`/`2026-04-09-ar-unity-xreal.md` 中的旧协议形状),文档自述"18 个流程点已全部实现" —— 这是**回溯性记录已上线行为的规范**,不是前瞻计划。

服务端实现:`IMUMoCap/Methods/WebSocketBroadcastServer.cs`,基于 `HttpListener` + `System.Net.WebSockets`,监听 `http://+:8765/ws/`,`MainWindow` 构造函数中启动。

**PC → AR 广播消息类型**:
- `state`:统一承载所有阶段转换(`waiting/armed/calibrating/baseline/training/rest/retention/paused/ended/error`),含 `condition`(EF/IF)、`block`(1-3)、`restDurationSec`、`reason`、个性化 `targetL/directionL/targetR/directionR`(PC 发 `|target|`,`direction` 由有符号 target 的符号定,见 §3.7)、`ifRenderMode`(`"placeholder"`|`"live"`|`"lastResult"`,IF 脚印渲染模式,由 PC 端 `ParamsDialog` 的两个互斥复选框 `IsIfLiveMode`/`IsIfLastResultMode` 实时切换,默认 `"lastResult"`;切换时会立即重发当前 state)。每次 stage 切换进入 training/retention 前会先广播带重标定 `reason` 的 `armed`(见 §3.8)。`rest` 现在会在 block=1(Training1 后)和 block=3(Training3 后)两个时机各广播一次(2026-09-06 起,原先只有 block=2 一次),AR 端对 `block` 字段完全通用处理(仅用于显示文案),**无需为此改动**。
- `live`:10Hz 连续每脚 yaw 角。**仅 IF 条件、且处于训练 block 阶段(非 baseline/rest/retention)时发送**(`MainWindow.xaml.cs` 内自动门控,原 `ChkLiveAngleToAr` 手动开关已移除);PC 端**无论 `ifRenderMode` 是什么都照常发送**(也供 `LiveVsFpaConsistency.csv` 对比用);只有 `ifRenderMode=="live"` 时 AR 才消费它来驱动 IF 脚印在两次 `fpa` 之间的连续旋转。`angleL/angleR` 为脚相对**实时骨盆朝向**的 yaw(`RawFootYawDeg`,已扣除标定偏置),右脚已取反(正 = toe-out),与 `fpa.errorL/R`(相对 PD)不是同一参考系。EF 不使用该流(见 §4.2)
- `fpa`:逐步反馈(仅 training 阶段),含 `fpaL/R`、`onTargetL/R`、`errorL/R`、`confidence`、`stability`、`quality`、`stage`、`block`
- `stepProgress`:baseline/retention 阶段步数进度条
- `redo`:操作员重做某阶段的通知
- `pong`:心跳回复(单播)

**AR → PC 命令**:`{"cmd":"ping"}`、`{"cmd":"ReadyForCalibration"}`(`_pendingStageEntry != null` 时走 `TriggerRecalibration()`,否则走 `TriggerCalibration()`)。`HandleWsMessage` 里还保留了 `Start`/`stop` 两个只写日志的占位 case。**`continueTraining` 已移除**——rest 到点由 PC 自动推进到赛前重标定 `armed`,AR 的 `Start` 点击已充当恢复信号。注意 `cmd` 大小写不统一,文档中说明是有意保留现状;`docs/superpowers/specs/2026-07-29-ws-protocol.md` **尚未同步**这些变化(`continueTraining` 移除、`ifRenderMode`、`live` 重新启用),以代码为准。

代码侧广播调用点与规范逐一对应:`BroadcastArState(...)`(state)、`OnDiagnosticsFrame` 内联的 `live` 广播、`OnFpaResult`(fpa)、`BroadcastStepProgress`、`BroadcastRedo`、`HandleWsMessage` 中的 `pong` 回复 —— **协议在 PC 端代码中已 1:1 完整实现**。

Manipulation Check / NASA-TLX / IMI-PC 等纸面量表**不通过 WebSocket 传输**,纯粹是 PC 端 `ConditionMeta.StageMarkers` 里的审计时间戳(见 §3.8、§3.10),AR 端完全不感知这些量表的存在。

### 4.2 AR 前端(`IMUARFPA/`)—— 已实现,独立 Unity 工程

**技术栈**:Unity 2022.3.62f3、XREAL XR SDK(`com.xreal.xr`,本地 tarball 依赖,非注册表包)、`com.unity.xr.interaction.toolkit` 2.6.5、`com.unity.xr.hands` 1.8.0;UI 用 uGUI(`Canvas`+`TextMeshPro`);网络/JSON 均用内置方案(`System.Net.WebSockets.ClientWebSocket` + Unity `JsonUtility`),无第三方库。

**关键文件**:
| 文件 | 作用 |
|---|---|
| `Assets/FootWebSocketClient.cs` | WebSocket 传输层:连接/重连/心跳、JSON 解析,把解析后的消息在主线程分发给 HUD |
| `Assets/FootHudController.cs` | 消费解析后的消息,驱动 UI 状态机与脚部图形渲染 |
| `Assets/Scenes/FPATraining.unity` | 唯一的训练场景,build index 0(`EditorBuildSettings.asset`,2026-09-06 修复,见下) |
| `Assets/XR/Settings/XREALSettings.asset` | XREAL SDK 配置(追踪模式、支持设备) |

**通信**:单条常驻 WebSocket,`ws://192.168.137.1:8765/ws/`(硬编码 ICS 热点 IP,`FootWebSocketClient` 的 Inspector 字段 `serverUrl` 可改但无环境切换机制);PC 是 WS 服务端,眼镜是客户端;无鉴权,假设可信局域网。断线自动重连(指数退避 1s→10s),心跳 10s 一次。所有 socket I/O 在后台线程,通过 `ConcurrentQueue<Action>` 在 `Update()` 里搬回主线程(Unity API 非线程安全)。

**状态机**(`FootHudController.OnStateChange`,与 §4.1 的 `state` 值完全对应):`waiting → armed → calibrating → baseline → training ⇄ rest → armed → calibrating → training ⇄ ... → retention → ended`,`paused`/`error` 可从多数状态进入;首次校准和每次赛前重标定都靠 `armed` 里的 `Start` 点击(`ReadyForCalibration`)触发——**客户端从不自动开始校准**。`rest` 不再需要 Continue 点击,由 PC 到点自动推进。

**训练条件差异**(`condition` 字段 `EF`/`IF`):两种条件的图形在**整个 training block 内都可见**(dwell-time 匹配,避免曝光时长成为混淆变量),只在收到 `fpa`(真实落地事件)时做一次"脉冲",两者共用 `PulseScale`(1.05x)的放大,`FootOutDisplaySec`(0.35s)后缩放一律还原(颜色/旋转是否保留见下)。
- **EF(stepping stones)**:石头显示固定目标朝向(`state.targetL/R`+`directionL/R`,每 block 设置一次),不旋转、不隐藏;`fpa` 时石头原地放大并按 `onTargetL/R` 染色(`FeedbackColors.OnTarget` 蓝 / `OffTarget` 橙,Wong 色盲友好色板),0.35s 后**只还原缩放,颜色保持上一步结果直到下一次 `fpa` 覆盖**(与 IF `lastResult` 一致;每个 block 开始时才复位为贴图原色)。**原来的 `footOutL/R` 实际脚印叠加已移除**(文档里旧的"EF 额外叠加实际脚印、0.2s 隐藏"描述已作废)。
- **IF(footprint)**:脚印始终可见;`fpa` 时旋转到 `errorL/R`(= FPA − target)、放大并按 `onTargetL/R` 染色。旋转/颜色之后是否还原取决于 `ifRenderMode`:`placeholder`(待命姿势 + 脉冲后全部还原)、`live`(额外消费 `live` 流连续旋转,脉冲后只还原颜色;0.5s 无更新则整体回到待命姿势)、`lastResult`(**默认**:不消费 `live`,脉冲后旋转/颜色都不还原,一直显示上一步的确认结果直到下一次 `fpa` 覆盖)。`live` 与 `fpa` 参考系不同,`live` 模式仅限 pilot 对比,见 `docs/task15_live_vs_fpa_consistency_report.md`。
- **右脚符号**:PC 端右脚 FPA/`live.angleR` 已取反(正 = toe-out),AR 端对右脚的 stone 目标、`live.angleR`、`fpa.errorR` 的旋转再取反一次(Unity Z 轴正向旋转为逆时针),两端必须成对修改(见 §3.7)。
- **Focus reminder**:每个 training block 开始时(暂停恢复不触发)在图形下方叠加 3s 的条件专属提示文本(EF/IF 各 10 词,字数匹配),然后只隐藏文本。`focusReminderPanel` 是手工在 Unity Editor 加的,不由脚本创建。
- **NaN 处理**:后端对无数据的脚会发 `NaN`,`JsonUtility` 解析不了,`HandleMessage` 先扫描置 `fpaLIsNaN/RIsNaN`、`angleLIsNaN/RIsNaN` 再把 `NaN` 替换为 `0`。

**场景结构**(`FPATraining.unity`,单场景单 Canvas):`FootprintRoot`(IF,`FootL/R`)、`StoneRoot`(EF,`StoneL/R`,已贴图)、`CalibOverlay`(文案 + 复用的 Button + 步数进度 Slider),三者互斥显示。`FootOutlineL/R`、`FootOutL/R` 仍在场景里但已无 C# 引用,是惰性死对象。

详细架构文档:`docs/summry/AR-FRONTEND.md`(最新);`IMUARFPA/docs/AR-FRONTEND.md` 是另一份副本,可能落后,以 summry 下的为准。

**已知问题/待办**(详见该文档"Known issues"一节):
- `serverUrl` 硬编码热点 IP,无环境切换。
- `XREALSettings.asset` 的 `InitialTrackingType`/`SupportDevices` 改动是否有意,尚未确认。
- ~~`EditorBuildSettings.asset` 引用已删除的 `SampleScene.unity`、缺失 `FPATraining.unity`~~ —— **已于 2026-09-06 修复**:该条目的 `guid` 其实一直指向 `FPATraining.unity`(同一 Unity 资产被改名,`.meta` 里的 guid 未变),只是 `path` 字段过期;已更正 `path`,`FPATraining.unity` 现为 build index 0 唯一启用场景,`HelloMR.unity` 保持禁用(全仓库找不到任何代码/场景引用它,无证据支持重新启用)。

## 5. 相关文档索引

`docs/superpowers/specs/`:
- `2026-04-08-ui-system-design.md`
- `2026-04-09-ui-design.md`(旧,部分已被 §4.1 的协议规范取代)
- `2026-07-29-ws-protocol.md`(当前有效的 WS 协议契约,见 §4.1)

`docs/superpowers/plans/`:
- `2026-04-08-thread-safety-and-refactor.md`
- `2026-04-09-ar-unity-xreal.md`(**过时**的 AR 实现计划,针对更早的旧协议,已被 AR 前端的实际实现和 §4.1 的当前协议规范取代 —— 不要用它作为 IMUARFPA 的实现依据)
- `2026-04-09-gait-pipeline-plan1-foundation.md` / `plan2-analysis.md` / `plan3-output.md`
- `2026-04-09-wpf-ui-pc-side.md`

`IMUMoCap/docs/superpowers/plans/2026-04-08-calibration-manager.md` —— 嵌在 app 目录内部的单独计划文档。

`docs/summry/demo_params.md` —— Demo 模式调参对照表(已于 2026-10-20 校对,当前生产默认值见 §3.4/§3.5/§3.7/§3.11)。

`docs/summry/AR-FRONTEND.md` —— AR 前端架构详解(2026-10-20 已更新),见第 4 节引用。

`docs/modules/`(`README.md`、`00-data-flow.md` ~ `08-ar-frontend.md`)—— 按模块拆分的详细文档,与本文互补。

`docs/task15_live_vs_fpa_consistency_report.md`、`docs/task*.py` —— 各项数据分析脚本及报告(task21 为多次校准 PD 重放)。

`docs/` 内另有示例采集数据(`Diagnostics_*.csv`、`ImuSamples_*.csv`)和 Python 原型脚本(`layerB2_axis_check.py`、`task2_stance.py` ~ `task7_green_red.py`),推测是算法(站立检测/FPA/运动上下文)在移植到 C# 之前的原型验证。

`IMUARFPA/` —— AR 前端 Unity 工程根目录(与 `IMUMoCap/` 平级,同一仓库内),自带 `CLAUDE.md`(与根目录相同的通用编码准则)。

## 6. 给 prime-agent 的要点提示

- 这是单机 WPF 应用(`IMUMoCap/`)+ 独立 Unity/XREAL AR 客户端(`IMUARFPA/`)的两段式架构,两者通过 WebSocket(端口 8765,PC 是服务端)解耦通信,协议双端均已落地并交叉验证一致。
- 训练流程为 3 个 training block(每 block 100 步/脚为退出标准,容差 α 依次 1.5→1.0→0.5【2026-10-19 起三个 block 共用固定 ±4°,见 §3.7】),Block1 后与 Block3 后各有一个 Rest+问卷窗口(Block3 后还含 NASA-TLX Admin1 + IMI-PC),Block2→Block3 之间无窗口;Rest 到点自动推进,之后每个 stage 进入前都有一次参与者点 Ready 的赛前重标定;Retention 完成后需操作员手动标记 NASA-TLX Admin2 并点击 End Recording 才真正结束(详见 §3.8)。这套时间线和步数目标目前有几处**研究者尚未最终确认的假设值**(停滞保护超时的具体分钟数、Block3 后窗口下限秒数)——继续改动这部分时应提醒用户复核。
- `MainWindow.xaml.cs` 是事实上的"上帝类",承载了大部分业务逻辑而非分散在 ViewModel/Command 中 —— 这是当前代码的既有风格,不是缺陷。
- AR 前端(`IMUARFPA/`)的 `FootHudController`/`FootWebSocketClient` 对协议字段(尤其是 `block`)处理得很通用,PC 端后续再调整阶段编号/窗口位置大概率不需要改 AR 端代码——但改动前仍应对照 `docs/summry/AR-FRONTEND.md` 确认,不要假设。
- **符号约定**:左右脚统一"正 = toe-out、负 = toe-in"(右脚在 PC 端取反);2026-10-20 之前录制的 Diagnostics 里右脚列是旧符号。分析历史数据时先确认录制日期。
