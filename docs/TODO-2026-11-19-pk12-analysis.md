# TODO — Pk1 / Pk12 数据分析发现的问题（2026-11-19）

数据：`IMUMoCap/bin/Debug/net8.0-windows/Data/Pk1/*_20261119_130812*`、`Data/Pk12/*_20261119_132355*`

## 背景结论：右脚 IMU 前后颠倒，对 FPA 没有影响（无需处理）

- Pk12 右脚传感器 X 轴朝向 − 步幅方向 = 176°（左脚 2.6°），StrideZero `R phi=152.8°`。
- 算法里没有任何地方假设"X 轴朝前"：stance 检测只看大小和倾斜；步幅积分用的是世界系 free acc；PD 法用 `yaw − yaw_calib`，步幅法的 180° 偏移被零点 φ 吸收；`MountingMonitor` 只检查 Z 轴朝上。
- 数据验证：右脚零点 sd 2.4°（左脚 2.3°）；同一步 Stride 与 PD 的相关性右脚 0.69/0.96/0.85，左脚 0.56/0.80/0.93。

---

## [x] TODO 1 — 步幅法模式下 Baseline 混用了 PD 法和步幅法的 FPA（优先）

> **已完成**：commit `bc496096`（2026-11-20）采用改法 A——`FpaResult.PreZero_L/R` 标记零点前的 PD 步，`BaselineProcessor.AddStep` 不计入（仍作反馈）。
> 回放验证（`IMUMoCap.Tests/Integration/BaselinePostZeroReplayTests.cs`）：
> - P4 EF：排除 12 个零点前 PD 步，Baseline（每脚 50 步）均值 L −2.56 / R 2.59，与纯步幅法一致。
> - Pk12 EF（2026-11-25 补）：排除 10 个 PD 步，Baseline（每脚 20 步，只用 Baseline 阶段数据）均值 **L −3.87 / R −13.04**（录制时混合值 L −6.97 / R −10.89），符合下文预期 L ≈ −4°、R ≈ −13°。
> - Pk1 未回放（两种方法本来就一致，改动前后差别很小）。

**现象**：零点定下之前，前 4–5 步用 PD FPA 作为反馈（`FpaEngine.SettleFoot` 里 pre-zero 的分支），这些步也计入了 Baseline。Pk12 上两种方法差了约 8°：

| 脚 | PD | Stride | 混合后 Baseline |
|---|---|---|---|
| Left | ≈ −14° | ≈ −4° | mean −7.0, SD 5.0 |
| Right | ≈ −5° | ≈ −13° | mean −10.9, SD 5.3 |

- 结果：target 偏了约 2–3°，SD 被放大；连"哪只脚更 toe-in"都反过来了。
- Pk1 两种方法一致（差 −0.1 / −0.7°），所以 Pk1 上看不出问题（SD 1.8 / 2.8）。

**可能的改法**（待定）：
- A. 步幅法模式下，零点定下之前的 PD 步不计入 Baseline；
- B. 零点定下之后才开始累计 Baseline。

**验证**：回放 Pk12 / Pk1，对比改动前后 Baseline 的 mean/SD/target；Pk12 应接近纯步幅法的值（L ≈ −4°，R ≈ −13°）。

## [ ] TODO 2 — 查明方案 A / B 零点之间约 8° 共模偏移的来源

- Pk12 事件：`pelvisZero-standZero = 8.1°(R) / 8.4°(L)`；左右脚 raw 偏移同向，所以是共模偏移，跟右脚装反无关。
- 方案 B 假设"一开始的直走方向 = 校准时站立面对的方向"，方案 A 以骨盆为参考。可能原因：被试出发方向偏离校准站姿方向，或骨盆 heading 偏差。
- 差值在 `ZeroCrossCheckMaxDeg = 15°` 以内，所以用了 B。在同一个 condition 内只看相对 Δ 时会抵消掉（前提是 TODO 1 修好）。
- 待办：确认是否需要在操作规程里要求"出发方向与校准站姿一致"，或者这个阈值是否合适。

## [ ] TODO 3（观察）— PD 法受各传感器之间 heading drift 影响

- Pk12 的 Stride − PD 偏移在各阶段间漂移：右脚 −7.9 → −4.6 → +1.2，左脚 8.2 → 3.9 → 11.6（Baseline → Training1 → Training2）。
- 步幅法在同一个传感器坐标系内做差，不受影响，这支持默认用步幅法。待办：确认还有没有地方依赖 PD FPA（例如方案 A 交叉检查、pre-zero 反馈）。

## [ ] TODO 4（运行记录，确认即可）

- Pk1 / Pk12 第一次校准都被拒：`Pelvis X axis 89–90°`，骨盆 IMU 当时还平放着，没戴好。佩戴检查按设计工作。
- Pk1 Training1 休息期间骨盆倾斜偏了 20° → 触发 Pause → 骨盆 FilterRestart → 录制中关了程序（`application closed while recording`）。骨盆 IMU 可能松了，需要检查固定方式。
- Pk12 在 Training3 之前等待时，左右脚都发生了 FilterRestart（13:23:32）。那里本来就要重新校准，不影响数据。
