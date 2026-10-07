# 步态分析核心流水线 (Gait Analysis Pipeline)

## 模块概述

本模块是整个系统的算法核心，负责把三个 IMU（Pelvis/LeftFoot/RightFoot）的原始数据包逐帧转换为足部前进角（FPA, Foot Progression Angle）结果。所有代码位于 `IMUMoCap/Pipeline/`，加上上游的帧同步组件 `IMUMoCap/Methods/ImuFrameCollector.cs`。对外只暴露一个门面类 `GaitPipeline`，`MainWindow` 只与它交互，不直接触碰任何内部分析组件。

对应测试：`IMUMoCap.Tests/BundlePlayer.cs`、`IMUMoCap.Tests/Integration/GaitPipelineReplayTests.cs`（回放式集成测试，目前唯一的用例被标记 `[Fact(Skip=...)]`，因为所需的 `Fixtures/walk_straight.jsonl` 录制文件尚未生成，实际并未在 CI/测试运行中执行）。

## 数据流总览

```
XsDataPacket (每个 IMU 逐包)
  → ImuFrameCollector.Process        按 PacketId 聚合三只 IMU → ImuFrameBundle
      → GaitPipeline.Process(bundle) （门面类主循环，逐帧调用下列组件）
          → DataQualityGate.Evaluate         质量校验，坏帧丢弃 → ValidFrame
          → CalibrationProcessor.Process     静态标定状态机 → CalibrationProfile（仅未完成标定前运行）
          → GaitEventDetector.Detect         逐脚站立/摆动检测 → GaitEvent（含 IsWalking）
          → MotionContextDetector.Detect     直行/转弯/PD重获取分类 → MotionContext
          → ProgressionDirEstimator.Update   在线前进方向融合估计 → PdEstimate
          → FpaEngine.Process                门控 + 结算 → FpaResult?（可能为 null）
          → DiagnosticsRow                   每帧诊断行，20 分钟环形缓冲，触发 OnDiagnosticsFrame
      → BaselineProcessor.AddStep (Baseline 阶段) / 直接转发 (Training 阶段)
```

标定完成之前（`CalibrationProcessor.State != Completed`），后续所有组件都不会运行——`GaitPipeline.Process` 里遇到未完成标定会直接 `return`。

## 组件详解

### `ImuFrameCollector`（帧同步，`Methods/ImuFrameCollector.cs`）

- `BuildImuSample`：从 `XsDataPacket` 解出一个 `ImuSampleFrame`，字段包括 `PacketId`/`PacketCounter`、`TimeSec`（相对第一个包的时间，`(PacketId - firstPacketId) / SampleRateHz`，默认 100Hz）、`StatusWord`、`Rssi`、`Quaternion`（板载姿态）、`RateOfTurn`（标定后角速度）、`FreeAcceleration`（已扣重力）、`Acceleration`（含重力）、`MagneticField`、`DeltaQ`/`DeltaV`（SDI 姿态/速度增量）。每个字段有对应的 `HasXxx` 标志，取决于该 `XsDataPacket` 是否包含该数据类型。
- `UpsertFrameBundle`：以 `PacketId` 为键把 Pelvis/LeftFoot/RightFoot 三个样本聚合进 `ImuFrameBundle`；`IsComplete` 为真（三者齐全）才当作完整帧交给下游，并从字典移除、更新 `_lastCompletedPacketId`（防止迟到的重复包）。
- `CleanupStaleFrameBundles`：某个 `PacketId` 等待超过 `MaxPendingPacketLag`（默认 16 包）仍未凑齐，视为丢帧，按缺失的传感器分别计入 `PelvisGapFrames`/`LeftFootGapFrames`/`RightFootGapFrames`，可通过 `GenerateReport()` 取得 `FrameQualityReport`（含 `CompletedFrames`/`TotalFrames`）。

### `DataQualityGate`（质量门控）

逐 bundle 校验，任一检查失败整帧丢弃（返回 `null`），三个 IMU 任一不合格则整帧作废：

1. **StatusWord 检查**（`HasStatusError`）：`XSF_OrientationValid` 位为 0（滤波器未收敛）→ 拒绝；`XSF_ClippingDetected` 位为 1（任一轴饱和削波）→ 拒绝。
2. **RSSI 检查**：字段 `RssiThresholdDbm`（-50）仍存在，但对应代码块被整段注释掉，**当前未生效**。
3. **加速度突变**（`HasAccSpike`）：与上一帧比较，`Acceleration` 欧氏距离 > `AccDeltaThreshold_ms2`（100 m/s²，注释说明特意调高以避免正常跺脚触发）。
4. **角速度突变**（`HasGyroSpike`）：与上一帧比较，`RateOfTurn` 欧氏距离 > `GyroDeltaThreshold_rads`（20 rad/s）。

通过后包装为 `ValidFrame`（bundle 的只读视图，不复制数据）。

### `CalibrationProcessor`（静态标定）

状态机：`WaitingForStart → CollectingStaticPose → Completed/Failed`，由操作员调用 `TriggerStart()` 触发进入采集态。

- 静止判定（`IsStatic`）：三个 IMU 的 `RateOfTurn` 模长同时 `< StaticGyroThreshold`（0.3 rad/s）。
- 连续静止满 `StaticRequiredFrames`（30 帧，0.3s）才开始把四元数样本压入缓冲区；抖动只是暂停采集（`_staticConsecutive` 清零），**不清空已收集的缓冲区**。
- 采满 `StaticCollectFrames`（300 帧，3s）后完成，取每个 IMU 四元数样本的**分量中位数**（先按 `W<0` 取反统一半球避免符号翻转，再对 X/Y/Z/W 四个分量分别排序取中位数、归一化）作为参考姿态，比均值更抗离群值。
- 超过 `StaticTimeoutFrames`（1000 帧，10s）仍未完成 → `Failed`。
- 产出 `CalibrationProfile{ PelvisRef, LeftFootRef, RightFootRef }`，后续所有朝向计算都以此为零点。

### `GaitEventDetector`（Stance/摆动检测）

每帧每脚独立判定，三重 AND 条件同时满足才算 stance：

1. `FreeAcceleration.Length() < FreeAccStanceThreshold`（2.5 m/s²）
2. `RateOfTurn.Length() < GyroThreshold`（1.0 rad/s）
3. **姿态倾斜检查**（yaw 无关，`FootPitchThreshold` 默认 0.35 rad ≈ 20°）：取标定参考下的重力方向 `gRef = Inverse(calRef)·(-Z)` 与当前姿态下的重力方向 `gNow = Inverse(q_current)·(-Z)`，两向量夹角 `acos(gRef·gNow) < FootPitchThreshold`。代码注释明确说明用夹角而非"提取相对四元数的 pitch 分量"，是因为旧方法在转身后会与朝向变化耦合出错。

三条件都满足后还要求连续 `MinStanceFrames`（5 帧）防抖才确认 stance。`IsWalking`：`WalkingWindowFrames`（300 帧，3s）滑动窗口内左右脚 stance↔swing 切换次数（`_leftTransitions`/`_rightTransitions`）都 `>= 2`（即至少一次完整周期）才判定为"正在走"，窗口结束后重新计数。

### `MotionContextDetector`（转弯识别）

三态状态机 `Straight → Turning → ReacquiringPd → Straight`，目的是把转弯及转弯后的方向重建期从 FPA 输出中剔除。

转弯信号判定（双路 OR）：
1. **骨盆 yaw 角速度**（主信号）：把 Pelvis 传感器系下的 `RateOfTurn` 用当前姿态四元数旋到世界系，取 Z 分量绝对值，超过 `YawRateThreshold_rads`（0.70 rad/s ≈ 40°/s；注释说明该值特意从 0.26 调高，因为正常步行时骨盆 yaw 峰值本身就到 0.5 rad/s，阈值太低会把正常摆髋误判为转弯）。
2. **DeltaQ 累积增量**（辅助信号）：对 Pelvis 姿态增量四元数逐帧提取 yaw 分量（`2·atan2(dq.Z, dq.W)` 小角度近似）并累加，绝对值超过 `DeltaQYawThreshold`（0.17 rad）。**注意**：这个累积量只在 `Straight` 状态下真正累积；一旦进入 `Turning` 或 `ReacquiringPd` 状态，代码会在每帧处理开头把它重置为 0（`HandleTurning`/`HandleReacquiring` 内 `_deltaQYawAccum = 0f`），代码注释解释是为了防止它在 Turning 态内持续累积整个转弯角度、永久阻塞退出——因此在 Turning/ReacquiringPd 态内，实际只有 yaw 角速度信号在起作用。

状态转换：
- `Straight → Turning`：信号连续满足 `TurningConfirmFrames`（10 帧）才切换，置信度随累积帧数线性下降（`1 - frames/threshold`）；切换时清空 DeltaQ 累积量。
- `Turning → ReacquiringPd`：信号连续**消失** `StraightConfirmFrames`（20 帧，比进入转弯的确认帧数更长，非对称设计防止转弯尾声抖动来回跳变）才退出；此态置信度恒为 0。
- `ReacquiringPd → Turning`：在 ReacquiringPd 态若信号又连续出现 `ReacqTurningConfirmFrames`（10 帧）判定为二次转弯，退回 Turning。
- `ReacquiringPd → Straight`：不由本类自己判定，由 `ProgressionDirEstimator.ConfirmStraight()` 外部调用触发。
- 辅助投票：若当前 `Straight` 但 `GaitEvent.IsWalking == false`，置信度打 5 折，不改变状态本身。

### `ProgressionDirEstimator`（行进方向 PD 估计）

仅在 `Straight`/`ReacquiringPd` 态更新。融合三路朝向信号加权平均（角度用向量和 `cos`/`sin` 累加再 `atan2`，避免 ±180° 环绕问题）：骨盆瞬时 yaw（权重 0.6）、左脚 stance 相位内的平均 yaw（权重 0.2）、右脚同理（权重 0.2）。

- 每只脚的朝向在其整个 stance 周期内用向量和累积（`TrackStanceYaw`），stance→swing 转换时才产出一次该脚这一步的朝向（`_leftYawReady`/`_rightYawReady`）。
- **每当左右脚都各自产出一次新朝向**（`TryConsumeStep` 两者都 ready）才触发一次融合更新，计入一次 PD 步数（`_stepCount++`）。
- **IMU 安装轴自适应**（`DetectHeadingAxis`）：标定时用参考四元数把世界重力向量反变换回传感器坐标系，取分量绝对值最大的轴作为该传感器的"朝向/yaw 轴"（默认 Z），据此选择 `ExtractHeading` 中 Roll/Pitch/Yaw 三种公式之一，兼容传感器绑扎方向不同的情况。
- **稳定性**：滑动窗口（`StabilityWindow=10` 步）内历次融合朝向的圆形方差 `circVar = 1 - R̄`（`R̄` 为平均合成向量长度），`Stability = 1/(1+circVar)`。
- **Turning → ReacquiringPd 切换**时清空 PD 历史窗口、步数计数、双脚 stance 累积器，避免转弯前后方向混算稳定性，也避免转弯瞬间正处于 stance 中的脚把转弯期间的朝向带入新估计。
- **ReacquiringPd → Straight 退出条件**（三者同时满足，满足后调用 `contextDetector.ConfirmStraight()`）：
  1. `Stability >= StabilityThreshold`（0.85）
  2. 已累积 `MinStepsBeforeValid`（2）步新融合估计
  3. 当前融合 PD 与骨盆瞬时朝向夹角 `<= PelvisAgreementThreshold_deg`（15°）——防止双脚朝向还没跟上骨盆转身进度就提前锁定偏差很大的 PD

### `FpaEngine`（FPA 计算与门控结算）

**采集与门控分离**（注释明确的设计原则）：采集只要脚在 stance 就持续进行（`StanceSampler.AddFrame`，内部类，逐脚独立状态机），不受 gate 状态影响；只有"是否允许输出"受 gate 控制。

- **采集**：每帧若 `LeftStance`/`RightStance` 为真，取该脚四元数相对标定参考的 yaw 差（`ExtractYaw(q) - ExtractYaw(calRef)`，归一化到 ±π），用向量和累积；swing→stance 边沿触发重置累积器。
- **门控** `gate = contextOk && pdOk`：
  - `contextOk`：`MotionContext.State == Straight` 且 `Confidence >= ContextConfidenceThreshold`（0.7）。
  - `pdOk`：`PdEstimate.IsValid` 且 `Stability >= PdStabilityThreshold`（0.7）。
  - 门控失败时记录 `StepExclusionReason`（`Turning`/`ReacquiringPd`/`LowConfidence`/`PdInvalid`），通过 `OnStepOutcome` 事件上报，供 QoE 统计"转弯排除了多少步"，不影响 `FpaResult` 输出本身，仅用于审计。
- **结算**（`TrySettle`）：脚从 swing 转入 stance 后，累积满 `MinStanceFramesForSettle`（10 帧，@100Hz=100ms）且当前帧 gate 通过，才输出该步 `FpaResult`；同一 stance 周期只输出一次（`_emittedThisStance`）。若落地后一直没等到 gate 通过，`MarkSwing()` 会在抬脚瞬间把这一步记为"排除"（返回 `_pendingReason`）而非"输出"。
- **FPA 值**：`FPA = mean(该脚 stance 期间 yaw) - PD.DirectionRad`，弧度转角度，正值 = toe-out，负值 = toe-in。
- **与 Baseline 比对**（仅 `Baseline != null` 即 Training 阶段）：`Error = FPA - Baseline.Target`；容差带 `w = max(w_min, α·SD)`（`w_min = BaselineProfile.ToleranceWMinDeg = 4°`，`α = FpaEngine.ToleranceAlpha`）；`OnTarget = |Error| <= w`。
- **质量标签**：门控通过后，若 `context.Confidence - ContextConfidenceThreshold < 0.1` 或 `pd.Stability - PdStabilityThreshold < 0.1`，标记 `Quality = "Marginal"`，否则 `"High"`——即使通过了 gate，余量小也会被标注，供事后按质量过滤数据。

### `BaselineProcessor` + `BaselineProfile`（基线统计与个性化目标）

- `AddStep(FpaResult)`：`Fpa_L`/`Fpa_R` 为 `NaN` 表示该脚本次无效，单脚无效时另一脚仍记录。
- `IsReady`：两脚累积步数都 `>= MinValidSteps`（默认 100）。
- `IsStalled`（提前止损）：领先脚已达 `MinValidSteps`，但落后脚不到 `MinValidSteps / 2` —— 判定为该脚采集异常（走廊太短/转弯排除过多/传感器问题），直接触发失败上报交给操作员决定是否重来，而不是让领先脚被迫走到 2× 阈值、用严重失衡数据凑目标。
- `TryFinalize()`：对每只脚样本求均值 `μ`、标准差 `SD`（`Sd` 用 n-1 无偏估计，样本数 <2 时返回 0），调用 `BaselineProfile.Create` 生成结果。
- **目标角规则**（`BaselineProfile.Create`）：`μ > 10°` → 判定习惯性 toe-out，训练方向 `ToeIn`，`Target = μ - TargetK·SD`；`μ ≤ 10°` → 判定习惯性 toe-in/中性，训练方向 `ToeOut`，`Target = μ + TargetK·SD`；`TargetK` 固定为 1.0。
- 同时计算左右步数比 `StepCountRatio = min(stepsL,stepsR)/max(...)`，低于 `imbalanceRatioThreshold`（默认 0.7，由调用方传入并原样记录为 `ImbalanceRatioThresholdUsed`）标记 `StepCountImbalanceWarning`。

### `BundleRecorder`（回放录制，测试用）

把 `ImuFrameBundle` 逐行序列化为 JSONL（`BundleDto`/`SampleDto`，字段与 `ImuSampleFrame` 一一对应），供测试从磁盘回放。`GaitPipeline.Recorder` 是可选属性，设置后 `Process()` 每次都会先调用 `Recorder.Record(bundle)`。测试侧 `IMUMoCap.Tests/BundlePlayer.cs` 负责反向读回并 `yield return` 出 `ImuFrameBundle` 序列（依赖 `InternalsVisibleTo` 访问 internal 的 `BundleDto`）。

`IMUMoCap.Tests/Integration/GaitPipelineReplayTests.cs` 是唯一的集成测试用例，用于回放录制数据验证端到端 FPA 输出范围（`-45°~45°`），但目前标记 `[Fact(Skip = "需要先录制 walk_straight.jsonl fixture")]`——**尚无可用 fixture，该测试实际未执行**。

## `GaitPipeline` 门面类：阶段编排

`GaitPipeline.Process(bundle)` 是唯一的每帧入口（`ProcessPacket` 是每包入口，内部先调用 `ImuFrameCollector` 攒齐 bundle）。除了串联上述组件，还负责：

- **训练块容差调度**：`SetTrainingBlock(n)` 按 `TrainingBlockAlphas = {1.5, 1.0, 0.5}`（对应 Block1/2/3）设置 `FpaEngine.ToleranceAlpha`，并重置本块步数计数。
- **训练块自动完成**：`InTraining` 时统计 `_trainingStepsL`/`_trainingStepsR`（有效 FPA 才计数），两脚都达到 `TrainingBlockTargetSteps`（100）时触发 `OnTrainingBlockComplete` 事件，由 `MainWindow` 负责推进阶段。
- **Baseline 自动完成/失败**：每帧检查 `_baseline.IsReady`（自动 `FinalizeBaseline()`）和 `_baseline.IsStalled`（同样调用 `FinalizeBaseline()`，因 `IsReady` 为 false 会返回 `null`，走既有失败上报路径）。
- **诊断行**：每帧无论 FPA 是否产出都构造一行 `DiagnosticsRow`（含双足加速度/角速度/四元数、stance 状态、`MotionState`/`MotionConfidence`、`PdDirectionDeg`/`PdStability`/`PdIsValid`、FPA 值/误差/是否达标/质量标签），存入最长 20 分钟（`DiagnosticsCapacity = 120,000` 帧 @100Hz）的环形缓冲区，并通过 `OnDiagnosticsFrame` 事件广播；`GetDiagnosticsSnapshot()` 返回副本供 CSV 导出。
- 代码中还有一段标记为 `TEMP` 的帧延迟测量逻辑（`_frameEntryTicks`/`TakeFrameLatencyMs`），用于测量帧从进入 pipeline 到广播给 AR 端的耗时，注释注明"用完删除"，属于临时调试代码，尚未清理。

## 与其他模块的关系

- **上游**：设备接入层（见 [`01-device-access.md`](01-device-access.md)）的 `ImuDeviceManager.DataPacketReceived` 事件触发 `GaitPipeline.ProcessPacket`。
- **下游 / UI**：`MainWindow`（见 [`04-ui-layer.md`](04-ui-layer.md)）订阅 `OnFpaResult`/`OnCalibrationStateChanged`/`OnBaselineProgress`/`OnDiagnosticsFrame`/`OnTrainingBlockComplete` 等事件驱动界面更新与阶段流转；`PipelineParams`（见 [`03-data-models.md`](03-data-models.md)）经 `SyncParams()` 实时同步进各组件的可调阈值。
- **下游 / 广播**：`OnFpaResult`、`OnDiagnosticsFrame` 中的 `live` 角度等最终经 `MainWindow` 转发给 [`05-websocket-protocol.md`](05-websocket-protocol.md) 描述的 `WebSocketBroadcastServer` 推给 AR 客户端。
- **下游 / 持久化**：`DiagnosticsRow` 是 [`06-data-recording.md`](06-data-recording.md) 中 `ExperimentRecorder` 写入 Session CSV 的行格式来源；`BaselineProfile` 被写入 Meta JSON。
