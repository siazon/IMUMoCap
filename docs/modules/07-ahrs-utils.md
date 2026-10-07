# AHRS / 姿态计算工具 (AHRS Utilities)

## 模块概述

本模块是一组分散的数学/格式转换小工具，服务于 IMU 数据处理链路中的角度换算、四元数运算、朝向计算和 CSV 读写，本身不包含业务状态机。物理上分布在两处：`IMUMoCap/AHRS/`（角度换算、旧版 CSV 工具、四元数与欧拉角互转扩展方法）和 `IMUMoCap/Methods/Utils.cs`（四元数/向量运算 + Xsens SDK 类型到 `System.Numerics` 类型的转换），后者虽物理路径不在 `AHRS/` 下，但功能上属于同一类别，故并入本文档。

| 文件 | 用途 |
|---|---|
| `AHRS/AngleCalculater.cs` | 角度→弧度转换的扩展方法 |
| `AHRS/CvsUntil.cs` | 早期 CSV 读写工具类（当前未见调用） |
| `AHRS/ExtensionMethod.cs` | 弧度→角度换算、四元数↔欧拉角互转的扩展方法 |
| `Methods/Utils.cs` | 四元数/向量归一化与旋转运算、朝向提取、Xsens SDK 类型转换 |

## `AHRS/AngleCalculater.cs`

只有一个静态扩展方法：`double.DegreesToRadians()`（`AngleCalculaterExtr` 类）。实际被 `MainWindow.xaml.cs` 用于把 Xsens `snap.Orientation` 的角度值（度）转换为弧度，拼进一个 4 元素数组（用途是给某个调试/显示用途的欧拉角构造，具体见 UI 层文档）。

## `AHRS/CvsUntil.cs`

`CvsUntil` 类提供 `ReadCsv(path, fileName)` 和 `WriteCVS(path, fileName, List<RecordedData>)` 两个方法，用于把 `IMUMoCap.Model.IMUData.cs` 中定义的 `RecordedData`（含 `quaternion`、`Accelerate`、`Orientation`、`MadgwickAHRS`、`AHRS` 等字段）写成固定列的 CSV。

**已知情况**：全仓库搜索未找到任何地方实例化 `CvsUntil` 或调用 `WriteCVS`/`ReadCsv`，属于当前未被使用的遗留代码（推测是早期数据采集原型阶段的工具，现有的 CSV 记录已由"数据记录与持久化"模块的 `ExperimentRecorder`/`CsvUtil` 取代）。按项目约定，未在本次任务范围内要求清理，故只记录不删除。

## `AHRS/ExtensionMethod.cs`

一组 `double`/`float`/`Vector3`/`Quaternion` 的扩展方法（命名空间 `IMUMoCap`，非 `IMUMoCap.AHRS`）：

- `ConvertRadiansToDegrees()`（`float`/`double` 两个重载）：弧度转角度。
- `RoundTwo()`：`Math.Round(x, 2)` 的简写，用于显示时保留两位小数。**实际被 `MainWindow.xaml.cs` 使用**（`snap.Orientation.x().RoundTwo()` 等，用于状态文本 `XsTime` 的显示）。
- `ToQuaternion()`（`Vector3` → `Quaternion`）：按 yaw/pitch/roll 顺序做欧拉角转四元数。
- `ToEulerAngles()`（`Quaternion` → `Vector3`）：四元数转欧拉角（roll/pitch/yaw）。

**未见调用**：全仓库搜索未找到 `ToQuaternion()`/`ToEulerAngles()` 的调用点，当前处于未使用状态。此外 `ToEulerAngles()` 的 yaw 分量计算里 `cosy_cosp = 1 - 2 * (q.Y * q.Y + q.Y * q.Z)`，按标准四元数转欧拉角公式此处应为 `q.Y*q.Y + q.Z*q.Z`，与其余项目里的 yaw 提取方式（见"步态分析核心流水线"模块中 `ExtractYaw` 一类实现）不一致；由于该方法当前无调用点，这个问题目前不影响运行时行为，仅作记录。

## `Methods/Utils.cs`

这是本模块中**唯一被核心数据链路依赖**的部分，作用是把 Xsens SDK 原生类型（`XsQuaternion`/`XsVector`/`XsVector3`，来自 `wrap_csharp64`）转换成 `System.Numerics.Quaternion`/`Vector3`，供 Pipeline 使用：

- `ToNumericsQuaternion(XsQuaternion)`、`ToNumericsVector3(XsVector / XsVector3)`：类型转换，**在 `Methods/ImuFrameCollector.cs` 的 `BuildImuSample` 中被密集调用**，用于解出 `Quaternion`、`RateOfTurn`、`FreeAcceleration`、`Acceleration`、`MagneticField`、`DeltaQ`、`DeltaV` 等字段（字段含义见"数据模型"模块），是设备原始数据进入统一处理链路的必经转换点。
- `Normalize(Quaternion)`：四元数归一化（除以模长，模长过小时返回单位四元数）。
- `ApplyRotationOffset(source, offset)` / `RotateVector(source, rotation)` / `QForHeading` / `HeadingENU_Consistent` / `WrapPi`：四元数旋转叠加、向量旋转、朝向角提取、角度环绕归一化等通用运算。

**未见调用**：除 `ToNumericsQuaternion`/`ToNumericsVector3` 外，`Normalize`、`ApplyRotationOffset`、`RotateVector`、`QForHeading`、`HeadingENU_Consistent`、`WrapPi` 在当前代码中未发现外部调用点（Pipeline 各组件如 `GaitEventDetector`/`MotionContextDetector`/`FpaEngine` 中的类似运算是各自内联实现的，未复用这里的工具方法）。推测是为某次重构预留但未完成迁移，或是早期原型遗留；同样只记录、不代为清理。

## 与其他模块的关系

- **设备接入层**：产出 `wrap_csharp64` 的原始 SDK 类型（`XsQuaternion`/`XsVector` 等）。
- **本模块（`Utils.ToNumericsQuaternion`/`ToNumericsVector3`）**：作为类型转换桥梁，是设备层数据进入统一处理链路前的最后一步。
- **步态分析核心流水线**（`ImuFrameCollector.BuildImuSample`）：唯一的实际调用方，转换结果直接写入 `ImuSampleFrame` 字段，供后续 `DataQualityGate`/`GaitEventDetector`/`FpaEngine` 等消费。
- **WPF UI 层**：`MainWindow.xaml.cs` 使用 `AngleCalculater`/`ExtensionMethod` 中的 `DegreesToRadians`/`RoundTwo` 做少量显示用的角度格式化，与核心算法链路无关。
