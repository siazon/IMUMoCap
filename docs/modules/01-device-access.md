# 设备接入层 (Device Access)

## 概述

本模块负责与 Xsens MTw Awinda 无线 IMU 传感器硬件通信：扫描/连接 Wireless Master(Awinda Station)、管理 MTw 传感器的连接生命周期、接收原始数据包并做初步统计(有效更新率、电量等)，最终把数据以 C# 事件的形式向上层(Pipeline / UI)暴露。所有与 Xsens 官方 SDK(XDA, Xsens Device API)的直接交互都封装在这一层，上层代码不直接触碰 SWIG 生成的 API。

## 文件列表

| 路径 | 职责 |
|---|---|
| `IMUMoCap/Device/MyXda.cs` | 对 `XsControl`(XDA 顶层入口)的薄封装：端口扫描(`scanPorts`)、开端口(`openPort`)、按 ID 查找设备(`getDevice`)。同文件内还定义了两个 `XsCallback` 子类：`MyMtwCallback`(单个 MTw 的数据/电量回调)、`MyWirelessMasterCallback`(Wireless Master 的连接状态、测量状态、设备错误等回调)，以及数据缓存结构 `ConnectedMTwData`。 |
| `IMUMoCap/Device/MyEventArgs.cs` | 上述回调用到的 `EventArgs` 子类集合(`DataAvailableArgs`、`BatteryLevelChangedArgs`、`DeviceErrorArgs`、`DeviceIdArg`、`PortInfoArg`、`ProgressUpdateArgs`)，纯数据载体，无逻辑。 |
| `IMUMoCap/Services/ImuDeviceManager.cs` | 面向上层的高层设备管理服务：维护一个有限状态机(`States`)，把 `MyXda`/`MyWirelessMasterCallback` 的底层回调转译为业务事件(`MtwConnected`/`MtwDisconnected`/`DataPacketReceived`/`BatteryLevelChanged`/`StateChanged`/`UpdateRatesAvailable`)。 |
| `IMUMoCap/Services/ImuSlotRegistry.cs` | 把硬件 `deviceId`(uint)映射到预先声明好的 `ImuViewModel` 槽位(Pelvis/Left/Right)，首次收到某设备的数据包时完成绑定(`GetOrAssign`)，线程安全。 |
| `IMUMoCap/Enums/DeviceStatesEnum.cs` | `States`(设备生命周期状态)、`TestState`(实验流程状态，与本模块耦合但语义上更偏 UI/流程)、`ImuRole`(Pelvis/Left/Right)三个枚举。 |
| `IMUMoCap/wrap_csharp64/`(约 200 个文件) | SWIG 自动生成的 Xsens XDA C# 绑定，不建议手动阅读全部文件。关键入口类：`XsControl`(SDK 总入口，管理设备列表)、`XsScanner`(静态方法 `scanPorts()` 扫描串口/USB 设备)、`XsDevice`(单个物理设备的句柄，测量/配置状态切换、回调注册)、`XsDataPacket`(单帧原始数据，包含四元数/加速度/角速度/磁场等字段)、`XsCallback`(回调基类，`MyMtwCallback`/`MyWirelessMasterCallback` 继承自它)。这一层纯粹是 C++ XDA 库的机械映射，没有业务逻辑，改动前应查阅 Xsens 官方 XDA 文档而非猜测行为。 |
| `IMUMoCap/extralibs/*.dll` | XDA 原生依赖(`xsensdeviceapi64.dll`、`xstypes64.dll`、滤波器 DLL 等)，运行时按需加载，非托管代码，不在本模块文档范围内。 |

## 关键设计点

### 两层封装结构

`MyXda`(端口级) + `MyWirelessMasterCallback`/`MyMtwCallback`(设备级回调) 是对 XDA 原始 API 的**第一层**封装，几乎是 Xsens 官方示例代码的直接移植，保留了原始的 `internal class`、无 nullable 标注等风格。`ImuDeviceManager` 是**第二层**封装，构造时把自己订阅到 `MyXda` 和一个 `MyWirelessMasterCallback` 实例上，对外只暴露强类型的 `event Action<T>?`，调用方（`MainWindow`）不需要认识 XDA 的类型。

### 设备生命周期状态机(`States`)

`DETECTING → CONNECTING → CONNECTED → ENABLED → OPERATIONAL → AWAIT_MEASUREMENT_START → MEASURING → AWAIT_RECORDING_START → RECORDING → FLUSHING`，由 `ImuDeviceManager.State`(私有 setter，变更时触发 `StateChanged` 事件)驱动。`ScanPorts()`/`StartMeasurement()`/`StopMeasurement()` 等公开方法内部都用 `switch(State)` 判断当前状态是否允许该操作，非法状态下操作被静默忽略（只写日志，不抛异常）。

### 数据包处理路径(`OnDataAvailable`)

`MyMtwCallback.onLiveDataAvailable`(SDK 线程触发) → `ImuDeviceManager.OnDataAvailable`：
1. 用 `e.Device.deviceId().legacyDeviceId()` 取出 32 位设备 ID，在 `_mtwData`(`ConcurrentDictionary<uint, ConnectedMTwData>`)中查表，查不到则丢弃(设备已断开但仍有在途数据包)。
2. 用每设备一把锁(`_mtwLocks`，`ConcurrentDictionary<uint, object>`)保护对该设备 `ConnectedMTwData` 字段的读写，避免与 `GetMtwDataSnapshot` 等读取方产生数据竞争。
3. 基于 `frameRange().first()/last()` 计算跳帧数(`frameSkips`)，用滑动窗口（最多累计到 99 个包）估算“有效更新率”百分比，用于 UI 展示，与后续 Pipeline 的丢帧统计(`ImuFrameCollector`)是两套独立的机制，不要混淆。
4. 通过 `ImuSlotRegistry.GetOrAssign(deviceId)` 拿到该设备绑定的 `ImuViewModel` 槽位，连同原始 `XsDataPacket` 一起通过 `DataPacketReceived` 事件抛给上层。

### 设备 ID 与角色的绑定

本模块本身**不硬编码** Pelvis/Left/Right 具体的设备 ID —— `ImuSlotRegistry` 只是通用的“ID → 预声明槽位”映射器，遇到未在 `Imus` 列表中声明的 `deviceId` 会直接抛 `InvalidOperationException`。三个传感器的具体十六进制 ID 是在 `MainWindow` 构造函数中声明 `ImuViewModel` 列表时写死的（见 UI 层模块文档），设备接入层对此完全无感知，这样切换传感器只需改 `MainWindow` 一处。

### 线程模型

`MyMtwCallback`/`MyWirelessMasterCallback` 的所有回调方法都运行在 Xsens SDK 的内部线程上，`ImuDeviceManager` 对外抛出的事件（`Log`/`StateChanged`/`DataPacketReceived` 等）因此**也是在非 UI 线程触发**，调用方必须自行 `Dispatcher.Invoke` 编排回 UI 线程——这一层本身不做任何线程封送。

## 与其他模块的关系

- **向上游（消费方）**：`Services/ImuDeviceManager` 的 `DataPacketReceived` 事件是 Gait Pipeline 模块的数据入口——`MainWindow` 订阅该事件后，把 `XsDataPacket` 交给 `Methods/ImuFrameCollector`（见「步态分析核心流水线」模块文档）做跨传感器帧同步。
- **依赖的数据模型**：`ImuSlotRegistry` 依赖 `Model/ImuViewModel`（见「数据模型」模块文档）作为槽位类型。
- **与 UI 层的耦合**：`States`/`ImuRole` 枚举、`ImuDeviceManager` 的事件均由 `MainWindow.xaml.cs` 直接订阅和展示（连接指示灯、日志框等），设备接入层本身不引用任何 WPF 类型。
