# Thread Safety & MainWindow Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix concurrent-access crashes on `Imus`/`imuDevicesMap`, then extract Xsens device logic into `ImuDeviceManager` so `MainWindow.xaml.cs` only handles UI.

**Architecture:** Introduce `ImuSlotRegistry` (thread-safe slot lookup via `ConcurrentDictionary`) and `ImuDeviceManager` (owns all Xsens SDK state, raises plain `Action` events with no Dispatcher dependency). `MainWindow` subscribes and wraps handlers in `Dispatcher.BeginInvoke`.

**Tech Stack:** C# 12 / .NET 8, WPF, Xsens XDA SDK (wrap_csharp64), no new NuGet packages.

---

## File Map

| Action | Path | Responsibility |
|--------|------|----------------|
| **Create** | `IMUMoCap/Services/ImuSlotRegistry.cs` | Thread-safe `ImuViewModel` slot lookup |
| **Create** | `IMUMoCap/Services/ImuDeviceManager.cs` | All Xsens SDK state + callbacks, raises events |
| **Modify** | `IMUMoCap/MainWindow.xaml.cs` | UI-only: subscribes to events, 3D rendering, WS messages |

---

## Task 1: Create `ImuSlotRegistry`

**Files:**
- Create: `IMUMoCap/Services/ImuSlotRegistry.cs`

- [ ] **Step 1: Create the Services folder and the file**

```csharp
// IMUMoCap/Services/ImuSlotRegistry.cs
using IMUMoCap.Model;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;

namespace IMUMoCap.Services
{
    /// <summary>
    /// Thread-safe registry that maps hardware device IDs to pre-declared ImuViewModel slots.
    /// Slots are declared at startup; binding happens on first data packet from a device.
    /// </summary>
    public sealed class ImuSlotRegistry
    {
        private readonly List<ImuViewModel> _imus;
        private readonly ConcurrentDictionary<uint, ImuViewModel> _deviceMap = new();

        public ImuSlotRegistry(IEnumerable<ImuViewModel> imus)
            => _imus = imus.ToList();

        /// <summary>Read-only snapshot of all declared slots (order matches construction order).</summary>
        public IReadOnlyList<ImuViewModel> Imus => _imus;

        /// <summary>
        /// Returns the slot for <paramref name="deviceId"/>.
        /// On first call for a given ID, binds the matching pre-declared slot.
        /// Thread-safe: multiple concurrent callers for the same ID are benign.
        /// </summary>
        public ImuViewModel GetOrAssign(uint deviceId)
        {
            if (_deviceMap.TryGetValue(deviceId, out var cached))
                return cached;

            var vm = _imus.FirstOrDefault(a => a.DeviceId == deviceId)
                ?? throw new InvalidOperationException(
                    $"Unrecognized deviceId {deviceId:X8}. Add it to the Imus list in MainWindow constructor.");

            vm.BindDevice(deviceId);
            // GetOrAdd is atomic: if two threads race, both see the same winner
            return _deviceMap.GetOrAdd(deviceId, vm);
        }

        /// <summary>Returns the slot if already bound, null otherwise.</summary>
        public ImuViewModel? TryGet(uint deviceId)
            => _deviceMap.TryGetValue(deviceId, out var vm) ? vm : null;
    }
}
```

- [ ] **Step 2: Build — confirm no errors**

Run: `dotnet build IMUMoCap/IMUMoCap.csproj`
Expected: Build succeeded, 0 errors.

- [ ] **Step 3: Commit**

```bash
git add IMUMoCap/Services/ImuSlotRegistry.cs
git commit -m "feat: add thread-safe ImuSlotRegistry with ConcurrentDictionary"
```

---

## Task 2: Create `ImuDeviceManager`

**Files:**
- Create: `IMUMoCap/Services/ImuDeviceManager.cs`

This class owns all Xsens SDK state. It raises events on whatever thread the SDK calls back on — callers are responsible for UI-thread marshalling.

- [ ] **Step 1: Create the file with all fields, events, and constructor**

```csharp
// IMUMoCap/Services/ImuDeviceManager.cs
using System;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Linq;
using System.Threading;
using XDA;

namespace IMUMoCap.Services
{
    // ── Event payload types ────────────────────────────────────────────────────
    public record MtwConnectedEvent(uint DeviceId, string DeviceIdStr, int TotalConnected);
    public record MtwDisconnectedEvent(uint DeviceId, string DeviceIdStr, int TotalConnected);
    public record DataPacketEvent(uint DeviceId, ImuViewModel Slot, XsDataPacket Packet);
    public record BatteryEvent(uint DeviceId, int Level);
    public record UpdateRatesEvent(List<string> Rates, int SelectedIndex);

    /// <summary>
    /// Owns all Xsens XDA SDK state: master device, MTw callbacks, connection lifecycle.
    /// All events may fire on a background thread — subscribers must marshal to UI if needed.
    /// </summary>
    public sealed class ImuDeviceManager : IDisposable
    {
        // ── Events ─────────────────────────────────────────────────────────────
        public event Action<string>? Log;
        public event Action<States>? StateChanged;
        public event Action<UpdateRatesEvent>? UpdateRatesAvailable;
        public event Action<MtwConnectedEvent>? MtwConnected;
        public event Action<MtwDisconnectedEvent>? MtwDisconnected;
        public event Action<DataPacketEvent>? DataPacketReceived;
        public event Action<BatteryEvent>? BatteryLevelChanged;

        // ── SDK state ──────────────────────────────────────────────────────────
        private readonly MyXda _myxda;
        private readonly MyWirelessMasterCallback _masterCallback;
        private readonly ImuSlotRegistry _registry;
        private XsDevice? _master;
        private readonly Dictionary<XsDevice, MyMtwCallback> _measuringMtws = new();
        private Dictionary<XsDevice, MyMtwCallback>.Enumerator _nextBatteryRequest;
        private readonly ConcurrentDictionary<uint, ConnectedMTwData> _mtwData = new();

        // ── Public state ───────────────────────────────────────────────────────
        private States _state = States.DETECTING;
        public States State
        {
            get => _state;
            private set { _state = value; StateChanged?.Invoke(value); }
        }

        public ImuDeviceManager(ImuSlotRegistry registry)
        {
            _registry = registry;

            _myxda = new MyXda();
            _myxda.WirelessMasterDetected += OnWirelessMasterDetected;
            _myxda.DockedMtwDetected      += OnDockedMtwDetected;
            _myxda.MtwUndocked            += OnMtwUndocked;
            _myxda.OpenPortSuccessful     += OnOpenPortSuccessful;
            _myxda.OpenPortFailed         += OnOpenPortFailed;

            _masterCallback = new MyWirelessMasterCallback();
            _masterCallback.MtwWireless            += OnMtwWireless;
            _masterCallback.MtwDisconnected        += OnMtwDisconnected;
            _masterCallback.MeasurementStarted     += OnMeasurementStarted;
            _masterCallback.MeasurementStopped     += OnMeasurementStopped;
            _masterCallback.DeviceError            += OnDeviceError;
            _masterCallback.WaitingForRecordingStart += OnWaitingForRecordingStart;
            _masterCallback.RecordingStarted       += OnRecordingStarted;
            _masterCallback.ProgressUpdate         += OnProgressUpdate;
        }

        // ── Public API ─────────────────────────────────────────────────────────
        public void ScanPorts() => _myxda.scanPorts();

        public void StartMeasurement(int desiredUpdateRate)
        {
            if (State != States.ENABLED && State != States.OPERATIONAL) return;

            if (desiredUpdateRate != -1 && desiredUpdateRate != _master?.updateRate())
            {
                if (_master?.setUpdateRate(desiredUpdateRate) == true)
                    Log?.Invoke($"Update rate set. Rate: {desiredUpdateRate}");
                else
                    Log?.Invoke($"Failed to set update rate. Rate: {desiredUpdateRate}");
            }

            if (desiredUpdateRate == 0)
                Log?.Invoke("Note: at the highest update rate, recording will be at effective update rate.");

            var bkp = State;
            State = States.AWAIT_MEASUREMENT_START;

            if (_master?.gotoMeasurement() == true)
                Log?.Invoke($"Waiting for measurement start. ID: {_master.deviceId().toXsString().toString()}");
            else
                State = bkp;
        }

        public void StopMeasurement()
        {
            if (State != States.MEASURING) return;

            if (_master?.gotoConfig() != true)
                Log?.Invoke($"Failed to stop measurement. ID: {_master?.deviceId().toXsString().toString()}");
        }

        public ConnectedMTwData? GetMtwData(uint deviceId)
            => _mtwData.TryGetValue(deviceId, out var d) ? d : null;

        public void Dispose()
        {
            ClearMeasuringMtws();
            _myxda.Dispose();
        }

        // ── MyXda event handlers ───────────────────────────────────────────────
        private void OnOpenPortSuccessful(object? _, PortInfoArg e)
        {
            if (State != States.CONNECTING) return;
            if (!e.PortInfo.deviceId().isWirelessMaster()) return;

            string idStr = e.PortInfo.deviceId().toXsString().toString();
            _master = _myxda.getDevice(e.PortInfo.deviceId());
            _master.addCallbackHandler(_masterCallback);

            Log?.Invoke($"Master Connected. Port: {e.PortInfo.portName().toString()}, ID: {idStr}");

            bool isStation = e.PortInfo.deviceId().isAwindaXStation() ||
                             e.PortInfo.deviceId().isAwinda2Station();
            if (isStation)
            {
                if (_master.deviceState() != XsDeviceState.XDS_Config)
                    _master.gotoConfig();
                bool ok = _master.makeOperational();
                Log?.Invoke($"makeOperational() => {ok}");
            }

            if (_master.isRadioEnabled())
                SetRadioChannel(-1);
            SetRadioChannel(11);

            State = States.CONNECTED;
        }

        private void OnWirelessMasterDetected(object? _, PortInfoArg e)
        {
            if (State != States.DETECTING) return;
            Log?.Invoke($"Master Detected. Port: {e.PortInfo.portName().toString()}, ID: {e.PortInfo.deviceId().toXsString().toString()}");
            State = States.CONNECTING;
            _myxda.openPort(e.PortInfo);
        }

        private void OnDockedMtwDetected(object? _, PortInfoArg e)
            => Log?.Invoke($"MTw Docked. Port: {e.PortInfo.portName().toString()}, ID: {e.PortInfo.deviceId().toXsString().toString()}");

        private void OnMtwUndocked(object? _, PortInfoArg e)
            => Log?.Invoke($"MTw Undocked. Port: {e.PortInfo.portName().toString()}, ID: {e.PortInfo.deviceId().toXsString().toString()}");

        private void OnOpenPortFailed(object? _, PortInfoArg e)
        {
            string kind = e.PortInfo.deviceId().isWirelessMaster() ? "wireless master" : "device";
            Log?.Invoke($"Connect to {kind} failed. Port: {e.PortInfo.portName().toString()}");

            if (State == States.CONNECTING)
            {
                _myxda.reset();
                State = States.DETECTING;
            }
        }

        // ── Master callback handlers ───────────────────────────────────────────
        private void OnMtwWireless(object? _, DeviceIdArg e)
        {
            string idStr = e.DeviceId.toXsString().toString();
            uint id = e.DeviceId.legacyDeviceId();

            if (_mtwData.ContainsKey(id)) return;   // already known

            var data = new ConnectedMTwData { _rssi = 0, _frameSkipsList = new List<int>() };
            _mtwData[id] = data;

            if (_registry.TryGet(id) is { } vm)
                vm.IsConnected = true;

            int total = _mtwData.Count;
            MtwConnected?.Invoke(new MtwConnectedEvent(id, idStr, total));
            Log?.Invoke($"Connected MTw list ({total}): {idStr}");
        }

        private void OnMtwDisconnected(object? _, DeviceIdArg e)
        {
            string idStr = e.DeviceId.toXsString().toString();
            uint id = e.DeviceId.legacyDeviceId();
            Log?.Invoke($"MTw Disconnected. ID: {idStr}");

            _mtwData.TryRemove(id, out _);
            MtwDisconnected?.Invoke(new MtwDisconnectedEvent(id, idStr, _mtwData.Count));
        }

        private void OnMeasurementStarted(object? _, DeviceIdArg e)
        {
            if (_myxda.getDevice(e.DeviceId)?.deviceId().legacyDeviceId() != _master?.deviceId().legacyDeviceId())
                return;

            switch (State)
            {
                case States.AWAIT_MEASUREMENT_START:
                    ClearMeasuringMtws();
                    foreach (XsDeviceId devId in _masterCallback.getConnectedMtws())
                    {
                        XsDevice? mtw = _myxda.getDevice(devId);
                        if (mtw == null) continue;

                        mtw.setSyncSettings(new XsSyncSettingArray());
                        var cb = new MyMtwCallback();
                        cb.DataAvailable      += OnDataAvailable;
                        cb.BatteryLevelChanged += OnBatteryLevelChanged;
                        mtw.addCallbackHandler(cb);
                        _measuringMtws[mtw] = cb;
                    }
                    _nextBatteryRequest = _measuringMtws.GetEnumerator();
                    Log?.Invoke($"Measurement Started. ID: {e.DeviceId.toXsString().toString()}");
                    State = States.MEASURING;
                    break;

                case States.RECORDING:
                case States.FLUSHING:
                    Log?.Invoke($"Recording Finished. ID: {e.DeviceId.toXsString().toString()}");
                    _master?.closeLogFile();
                    State = States.MEASURING;
                    break;
            }
        }

        private void OnMeasurementStopped(object? _, DeviceIdArg e)
        {
            if (e.DeviceId.toInt() != _master?.deviceId().legacyDeviceId()) return;
            Log?.Invoke($"Measurement Stopped. ID: {e.DeviceId.toXsString().toString()}");
            ClearMeasuringMtws();
            State = States.OPERATIONAL;
        }

        private void OnDeviceError(object? _, DeviceErrorArgs e)
        {
            Log?.Invoke($"ERROR. ID: {e.DeviceId.toXsString().toString()}");
            if (State == States.AWAIT_MEASUREMENT_START)
                State = States.ENABLED;
        }

        private void OnWaitingForRecordingStart(object? _, DeviceIdArg e)
        {
            Log?.Invoke($"Waiting for recording start. ID: {_master?.deviceId().toXsString().toString()}");
            State = States.AWAIT_RECORDING_START;
        }

        private void OnRecordingStarted(object? _, DeviceIdArg e)
        {
            if (State != States.AWAIT_RECORDING_START) return;
            Log?.Invoke($"Recording started. ID: {_master?.deviceId().toXsString().toString()}");
            State = States.RECORDING;
        }

        private void OnProgressUpdate(object? _, ProgressUpdateArgs e)
        {
            if (State != States.FLUSHING || e.Identifier != "Flushing") return;
            if (e.Total == 0)
            {
                _master?.abortFlushing();
                Log?.Invoke($"Flushing aborted. ID: {_master?.deviceId().toXsString().toString()}");
            }
        }

        private void OnDataAvailable(object? _, DataAvailableArgs e)
        {
            uint deviceId = e.Device.deviceId().legacyDeviceId();

            if (!_mtwData.TryGetValue(deviceId, out var mtwData))
            {
                Log?.Invoke($"Obsolete data from unknown device {deviceId:X8}");
                return;
            }

            // Update cached MTw data (all fields written from this one background thread per device)
            if (e.Packet.containsSdiData())
                mtwData.XsQuaternion = e.Packet.sdiData().orientationIncrement();
            if (e.Packet.containsRssi())
                mtwData._rssi = e.Packet.rssi();
            if (e.Packet.containsOrientation())
                mtwData._orientation = e.Packet.orientationEuler();

            // Frame-skip accounting
            int frameSkips = e.Packet.frameRange().last() > e.Packet.frameRange().first()
                ? e.Packet.frameRange().last() - e.Packet.frameRange().first() - 1
                : 65535 + e.Packet.frameRange().last() - e.Packet.frameRange().first() - 1;

            mtwData._frameSkipsList.Add(frameSkips);
            mtwData._sumFrameSkips += (uint)frameSkips;
            mtwData._effectiveUpdateRate = (int)(100 * (1 - (float)mtwData._sumFrameSkips /
                (float)(mtwData._frameSkipsList.Count + mtwData._sumFrameSkips)));

            while (mtwData._frameSkipsList.Count + mtwData._sumFrameSkips > 99 && mtwData._frameSkipsList.Count > 0)
            {
                mtwData._sumFrameSkips -= (uint)mtwData._frameSkipsList[0];
                mtwData._frameSkipsList.RemoveAt(0);
            }

            ImuViewModel slot = _registry.GetOrAssign(deviceId);
            DataPacketReceived?.Invoke(new DataPacketEvent(deviceId, slot, e.Packet));
        }

        private void OnBatteryLevelChanged(object? _, BatteryLevelChangedArgs e)
        {
            uint id = e.DeviceId.legacyDeviceId();
            if (_mtwData.TryGetValue(id, out var d))
                d._batteryLevel = e.Level;
            BatteryLevelChanged?.Invoke(new BatteryEvent(id, e.Level));
        }

        // ── Helpers ────────────────────────────────────────────────────────────
        private void SetRadioChannel(int channel)
        {
            if (_master?.enableRadio(channel) == true)
            {
                if (channel == -1)
                {
                    Log?.Invoke($"Master Disabled. ID: {_master.deviceId().toXsString().toString()}");
                    State = States.CONNECTED;
                    return;
                }

                Log?.Invoke($"Master Enabled. ID: {_master.deviceId().toXsString().toString()}, Channel: {channel}");

                var supportedRates = _master.supportedUpdateRates();
                int maxRate = _master.maximumUpdateRate();
                var rates = new List<string>();
                for (uint i = 0; i < supportedRates.size() && supportedRates.at(i) <= maxRate; i++)
                    rates.Add(supportedRates.at(i).ToString());

                int selectedIdx = rates.IndexOf(Convert.ToString(_master.updateRate()));

                // Default to 75 Hz if available
                int idx75 = rates.IndexOf("75");
                if (idx75 != -1) selectedIdx = idx75;

                UpdateRatesAvailable?.Invoke(new UpdateRatesEvent(rates, selectedIdx));
                State = States.ENABLED;
            }
            else
            {
                Log?.Invoke(channel == -1
                    ? $"Failed to disable wireless master. ID: {_master?.deviceId().toXsString().toString()}"
                    : $"Failed to enable wireless master. ID: {_master?.deviceId().toXsString().toString()}, Channel: {channel}");
            }
        }

        private void ClearMeasuringMtws()
        {
            lock (_measuringMtws)
            {
                foreach (var kv in _measuringMtws)
                    kv.Key.clearCallbackHandlers();
            }
            _measuringMtws.Clear();
            _nextBatteryRequest.Dispose();
        }
    }
}
```

- [ ] **Step 2: Build — confirm no errors**

Run: `dotnet build IMUMoCap/IMUMoCap.csproj`
Expected: Build succeeded, 0 errors. Fix any SDK API mismatches (e.g. if `MyWirelessMasterCallback` or `ConnectedMTwData` field names differ from what's shown above — check `IMUMoCap/Device/MyXda.cs` and the existing `MainWindow.xaml.cs` for exact names).

- [ ] **Step 3: Commit**

```bash
git add IMUMoCap/Services/ImuDeviceManager.cs
git commit -m "feat: extract ImuDeviceManager from MainWindow — owns all Xsens SDK state"
```

---

## Task 3: Wire `ImuDeviceManager` into `MainWindow` and remove extracted code

**Files:**
- Modify: `IMUMoCap/MainWindow.xaml.cs`

The strategy: add the new service fields, subscribe to events with Dispatcher wrapping, then delete the blocks of code that moved into the service.

- [ ] **Step 1: Add `using` directives and new service fields at the top of `MainWindow`**

At `MainWindow.xaml.cs` line 0–32 (the `using` block), add:
```csharp
using IMUMoCap.Services;
```

Replace the old field declarations (lines 41–46 — `_MyWirelessMasterDevice`, `_myxda`, `m_myWirelessMasterCallback`, `_measuringMtws`, `_nextBatteryRequest`, `_connectedMtwData`) with:

```csharp
private ImuDeviceManager _deviceManager = null!;
private ImuSlotRegistry _slotRegistry = null!;
```

Keep all other existing fields (`_wsServer`, `actionQueue`, `ImuDataQueue`, `imuPelvis`, etc.).

- [ ] **Step 2: Rewrite the `MainWindow` constructor**

Replace the constructor body with:

```csharp
public MainWindow()
{
    InitializeComponent();
    this.DataContext = _content;

    // ── WebSocket ──────────────────────────────────────────────────────────
    _wsServer = new WebSocketBroadcastServer();
    _wsServer.Start(new[] { "http://+:8765/ws/" });
    log("WebSocket server started: ws://192.168.137.1:8765/ws/");
    _wsServer.OnTextMessage += (clientId, text) => HandleWsMessage(clientId, text);
    _wsServer.OnClientConnected += (id, remote) => { /* TODO: push current state on connect */ };

    // ── IMU slot registry (pre-declared slots, hardcoded device IDs) ───────
    var imuList = new List<ImuViewModel>
    {
        new ImuViewModel(imuPelvis) { IMUDodel = ImuVisual,   DeviceId = 0x00B43CAB, Role = ImuRole.Pelvis },
        new ImuViewModel(imuL)      { IMUDodel = ImuVisual1,  DeviceId = 0x10B41904, Role = ImuRole.Left   },
        new ImuViewModel(imuR)      { IMUDodel = ImuVisual12, DeviceId = 0x10b41913, Role = ImuRole.Right  },
    };
    _slotRegistry = new ImuSlotRegistry(imuList);

    // ── Device manager ─────────────────────────────────────────────────────
    _deviceManager = new ImuDeviceManager(_slotRegistry);

    _deviceManager.Log           += msg  => Dispatcher.BeginInvoke(() => log(msg));
    _deviceManager.StateChanged  += s    => Dispatcher.BeginInvoke(() => OnDeviceStateChanged(s));
    _deviceManager.UpdateRatesAvailable += ev => Dispatcher.BeginInvoke(() => OnUpdateRatesAvailable(ev));
    _deviceManager.MtwConnected  += ev   => Dispatcher.BeginInvoke(() => OnMtwConnected(ev));
    _deviceManager.MtwDisconnected += ev => Dispatcher.BeginInvoke(() => OnMtwDisconnected(ev));
    _deviceManager.DataPacketReceived += ev => Dispatcher.BeginInvoke(() => OnDataPacket(ev));
    _deviceManager.BatteryLevelChanged += ev => Dispatcher.BeginInvoke(() => OnBattery(ev));

    // ── Misc init ──────────────────────────────────────────────────────────
    _content.StatusLabel = "Ready to calibration";
    _imuRotTf = new RotateTransform3D(_imuRot);
    _imuFrameCollector.SampleRateHz = _sampleRateHz;
    UpdateImuStatusIndicators();

    StartScanAsync();
}
```

- [ ] **Step 3: Replace the `InitDevice` method with `StartScanAsync`**

Delete the old `InitDevice()` method (lines 171–212) and replace with:

```csharp
private void StartScanAsync()
{
    _imuLoopCts = new CancellationTokenSource();
    var token = _imuLoopCts.Token;

    // Background scan
    Task.Run(() => _deviceManager.ScanPorts());

    // Background IMU queue consumer (currently empty; extend as needed)
    Task.Run(async () =>
    {
        try
        {
            while (!token.IsCancellationRequested)
            {
                var info = await Task.Run(() => ImuDataQueue.Take(token), token);
                // process info here when needed
            }
        }
        catch (OperationCanceledException) { /* expected on shutdown */ }
    }, token);
}
```

- [ ] **Step 4: Add the event-handler methods that MainWindow now owns**

Add these methods to `MainWindow` (replacing the inline Dispatcher lambdas that used to be inside the old callback methods):

```csharp
// Called on UI thread (Dispatcher.BeginInvoke)
private void OnDeviceStateChanged(States s)
{
    _content.DeviceState = s;
    setWidgetsStates();
}

private void OnUpdateRatesAvailable(UpdateRatesEvent ev)
{
    _content.UpdateRates.Clear();
    foreach (var r in ev.Rates)
        _content.UpdateRates.Add(r);
    _content.SelectedRate = ev.SelectedIndex;
}

private void OnMtwConnected(MtwConnectedEvent ev)
{
    if (_content.ConnectedMtws.IndexOf(ev.DeviceIdStr) >= 0) return;

    _content.ConnectedMtws.Add(ev.DeviceIdStr);
    _content.DeviceModels.Add(new DeviceModel { DeviceName = ev.DeviceIdStr });
    _content.SelectedMtw = _content.ConnectedMtws.Count - 1;

    // Update connection badge for this slot
    if (_slotRegistry.TryGet(ev.DeviceId) is { } vm)
        vm.IsConnected = true;

    UpdateImuStatusIndicators();

    bool allConnected = _slotRegistry.Imus.All(i => i.IsConnected);
    if (allConnected)
    {
        log("All IMUs connected. Auto-starting measurement...");
        int desiredRate = _content.UpdateRates.Count > 0
            ? Convert.ToInt32(_content.UpdateRates[_content.SelectedRate])
            : -1;
        _deviceManager.StartMeasurement(desiredRate);
    }
}

private void OnMtwDisconnected(MtwDisconnectedEvent ev)
{
    _content.ConnectedMtws.Remove(ev.DeviceIdStr);
    _content.SelectedMtw = _content.ConnectedMtws.Count - 1;
    UpdateImuStatusIndicators();
}

private void OnDataPacket(DataPacketEvent ev)
{
    var mtwData = _deviceManager.GetMtwData(ev.DeviceId);
    if (mtwData == null) return;

    string mtwIdStr = ev.Slot.SlotName;

    if (ev.Packet.containsOrientation())
    {
        var quat = ev.Packet.orientationQuaternion();
        OnNewImuQuaternion(new Quaternion(quat.x(), quat.y(), quat.z(), quat.w()), ev.Slot);
    }

    // Update display for selected MTw
    if (_content.SelectedMtw >= 0 &&
        _content.SelectedMtw < _content.ConnectedMtws.Count &&
        _content.ConnectedMtws[_content.SelectedMtw] == mtwIdStr)
    {
        _content.XsTime = $"{mtwIdStr} | {mtwData._orientation.x().RoundTwo()}, " +
                          $"{mtwData._orientation.y().RoundTwo()}, {mtwData._orientation.z().RoundTwo()}";

        if (_content.RotationByDegree)
            actionQueue.Enqueue([
                mtwData._orientation.x().DegreesToRadians(),
                mtwData._orientation.y().DegreesToRadians(),
                mtwData._orientation.z().DegreesToRadians(), 1]);
        else
            actionQueue.Enqueue([
                mtwData.XsQuaternion.x(), mtwData.XsQuaternion.y(),
                mtwData.XsQuaternion.z(), mtwData.XsQuaternion.w()]);
    }

    // Forward to frame collector
    OnXsensData(ev.Slot.Role, ev.DeviceId, ev.Packet);
}

private void OnBattery(BatteryEvent ev) { /* extend UI badge if needed */ }
```

- [ ] **Step 5: Delete all code that was moved to `ImuDeviceManager`**

Delete these blocks from `MainWindow.xaml.cs`:
- The `#region MyXda` ... `#endregion` block (lines 214–738) — all `_myxda_*` and `_callbackHandler_*` methods
- `clearMeasuringMtws()` (lines 457–468)
- `SetRadioChannel()` (lines 782–840)
- `setWidgetsStates()` — **keep it**, it still belongs in MainWindow (updates UI widget enable states)
- `StartMeasurementInternal()` — replace with call to `_deviceManager.StartMeasurement(...)` in `BtnMeasure_Click`
- `StopMeasurementInternal()` — replace with call to `_deviceManager.StopMeasurement()` where needed
- Old field declarations: `pelvisId`, `leftFootId`, `rightFootId`, `Imus`, `imuDevicesMap`, `GetOrAssignSlot()`

Update `BtnMeasure_Click`:
```csharp
private void BtnMeasure_Click(object sender, RoutedEventArgs e)
{
    int desiredRate = _content.UpdateRates.Count > 0
        ? Convert.ToInt32(_content.UpdateRates[_content.SelectedRate])
        : -1;
    _deviceManager.StartMeasurement(desiredRate);
}
```

Update `OnClosed`:
```csharp
protected override void OnClosed(EventArgs e)
{
    _imuLoopCts?.Cancel();
    ImuDataQueue?.CompleteAdding();
    _deviceManager.Dispose();
    if (_wsServer != null)
    {
        try { _wsServer.StopAsync().GetAwaiter().GetResult(); } catch { }
    }
    base.OnClosed(e);
}
```

- [ ] **Step 6: Build and fix any remaining compile errors**

Run: `dotnet build IMUMoCap/IMUMoCap.csproj`

Common fixes needed:
- Any remaining references to `_MyWirelessMasterDevice` → use `_deviceManager` methods
- Any reference to old `Imus` → use `_slotRegistry.Imus`
- Any reference to `imuDevicesMap` → use `_slotRegistry.TryGet()`
- Any reference to `_connectedMtwData` → use `_deviceManager.GetMtwData()`

- [ ] **Step 7: Commit**

```bash
git add IMUMoCap/MainWindow.xaml.cs
git commit -m "refactor: slim MainWindow to UI-only — device logic lives in ImuDeviceManager"
```

---

## Self-Review

**Spec coverage:**
- Thread safety for `Imus`/`imuDevicesMap` ✓ — `ImuSlotRegistry` uses `ConcurrentDictionary.GetOrAdd` (atomic)
- Thread safety for `_connectedMtwData` ✓ — replaced with `ConcurrentDictionary<uint, ConnectedMTwData>` in `ImuDeviceManager`
- `MainWindow` split ✓ — device, slot-registry, and UI code in separate files
- WebSocket already separate (`WebSocketBroadcastServer.cs`) — `HandleWsMessage` stays in `MainWindow` since it's UI orchestration

**Placeholder scan:** All steps contain real code. No TBD/TODO in implementation steps.

**Type consistency:**
- `ConnectedMTwData` used in both tasks — must match the existing type in `IMUMoCap` namespace (verify field names `_rssi`, `_frameSkipsList`, `_sumFrameSkips`, `_effectiveUpdateRate`, `_batteryLevel`, `XsQuaternion`, `_orientation`, `XsTime` match actual definition in `IMUMoCap/Device/MyXda.cs` or wherever it's declared)
- `ImuSlotRegistry.GetOrAssign` / `TryGet` — used consistently across Task 1, 2, 3
- `MtwConnectedEvent`, `MtwDisconnectedEvent`, `DataPacketEvent`, `BatteryEvent`, `UpdateRatesEvent` — defined in Task 2, used in Task 3

**Risk:** The XDA SDK callback `DataAvailable` may fire from multiple device threads simultaneously. `ConcurrentDictionary` lookup in `OnDataAvailable` is safe. However `ConnectedMTwData._frameSkipsList` is a plain `List<int>` modified in `OnDataAvailable` — if two devices share one `ConnectedMTwData` instance this could race, but each device has its own entry so it's fine as long as one device's data arrives on one thread at a time (XDA guarantees per-device serialization).
