using IMUMoCap.Model;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System;
using XDA;

namespace IMUMoCap.Services
{
    // ── Event payload records ──────────────────────────────────────────────────
    public record MtwConnectedEvent(uint DeviceId, string DeviceIdStr, int TotalConnected);
    public record MtwDisconnectedEvent(uint DeviceId, string DeviceIdStr, int TotalConnected);
    public record DataPacketEvent(uint DeviceId, ImuViewModel Slot, XsDataPacket Packet);
    public record BatteryEvent(uint DeviceId, int Level);
    public record UpdateRatesEvent(List<string> Rates, int SelectedIndex);

    public sealed class ImuDeviceManager : IDisposable
    {
        // ── Dependencies ───────────────────────────────────────────────────────
        private readonly ImuSlotRegistry _registry;

        // ── Xsens SDK objects ──────────────────────────────────────────────────
        private MyXda _myxda;
        private MyWirelessMasterCallback _callbackHandler;
        private XsDevice? _wirelessMasterDevice;

        // ── Thread-safe data stores ────────────────────────────────────────────
        private readonly ConcurrentDictionary<uint, ConnectedMTwData> _mtwData = new();

        // ── Measuring MTws — only accessed from SDK callbacks, protected by lock ──
        private readonly Dictionary<XsDevice, MyMtwCallback> _measuringMtws = new();
        private readonly object _measuringMtwsLock = new();
        private Dictionary<XsDevice, MyMtwCallback>.Enumerator _nextBatteryRequest;
        private bool _batteryEnumeratorInitialized = false;

        // ── Per-device locks for cross-thread data mutations ───────────────────
        private readonly ConcurrentDictionary<uint, object> _mtwLocks = new();

        // ── Events (fired on SDK/background threads — callers must marshal to UI) ──
        public event Action<string>? Log;
        public event Action<States>? StateChanged;
        public event Action<UpdateRatesEvent>? UpdateRatesAvailable;
        public event Action<MtwConnectedEvent>? MtwConnected;
        public event Action<MtwDisconnectedEvent>? MtwDisconnected;
        public event Action<DataPacketEvent>? DataPacketReceived;
        public event Action<BatteryEvent>? BatteryLevelChanged;

        // ── State ──────────────────────────────────────────────────────────────
        private volatile States _state = States.DETECTING;
        public States State
        {
            get => _state;
            private set
            {
                _state = value;
                StateChanged?.Invoke(value);
            }
        }

        // ── Constructor ────────────────────────────────────────────────────────
        public ImuDeviceManager(ImuSlotRegistry registry)
        {
            _registry = registry;

            _myxda = new MyXda();
            _myxda.WirelessMasterDetected += OnWirelessMasterDetected;
            _myxda.DockedMtwDetected += OnDockedMtwDetected;
            _myxda.MtwUndocked += OnMtwUndocked;
            _myxda.OpenPortSuccessful += OnOpenPortSuccessful;
            _myxda.OpenPortFailed += OnOpenPortFailed;

            _callbackHandler = new MyWirelessMasterCallback();
            _callbackHandler.MtwWireless += OnMtwWireless;
            _callbackHandler.MtwDisconnected += OnMtwDisconnected;
            _callbackHandler.MeasurementStarted += OnMeasurementStarted;
            _callbackHandler.MeasurementStopped += OnMeasurementStopped;
            _callbackHandler.DeviceError += OnDeviceError;
            _callbackHandler.WaitingForRecordingStart += OnWaitingForRecordingStart;
            _callbackHandler.RecordingStarted += OnRecordingStarted;
            _callbackHandler.ProgressUpdate += OnProgressUpdate;
        }

        // ── Public API ─────────────────────────────────────────────────────────

        public void ScanPorts()
        {
            if (State != States.MEASURING &&
                State != States.AWAIT_MEASUREMENT_START &&
                State != States.RECORDING &&
                State != States.FLUSHING)
            {
                _myxda.scanPorts();
            }
        }

        public void StartMeasurement(int desiredUpdateRate)
        {
            switch (State)
            {
                case States.ENABLED:
                case States.OPERATIONAL:
                {
                    // Set the update rate
                    if (desiredUpdateRate != -1 && desiredUpdateRate != _wirelessMasterDevice?.updateRate())
                    {
                        if (_wirelessMasterDevice?.setUpdateRate(desiredUpdateRate) == true)
                        {
                            Log?.Invoke(string.Format("Update rate set. ID: {0}, Rate: {1}",
                                _wirelessMasterDevice?.deviceId().toXsString().toString(), desiredUpdateRate));
                        }
                        else
                        {
                            Log?.Invoke(string.Format("Failed to set update rate. ID: {0}, Rate: {1}",
                                _wirelessMasterDevice?.deviceId().toXsString().toString(), desiredUpdateRate));
                        }
                    }

                    States bkpState = State;
                    State = States.AWAIT_MEASUREMENT_START;

                    if (_wirelessMasterDevice?.gotoMeasurement() == true)
                    {
                        Log?.Invoke(string.Format("Waiting for measurement start. ID: {0}",
                            _wirelessMasterDevice.deviceId().toXsString().toString()));
                    }
                    else
                    {
                        // If gotoMeasurement fails revert the state
                        State = bkpState;
                    }
                }
                break;

                case States.MEASURING:
                {
                    if (_wirelessMasterDevice?.gotoConfig() == true)
                    {
                        // ICC (optional — commented out in original)
                    }
                    else
                    {
                        Log?.Invoke(string.Format("Failed to stop measurement. ID: {0}",
                            _wirelessMasterDevice?.deviceId().toXsString().toString()));
                    }
                }
                break;
            }
        }

        public void StopMeasurement()
        {
            switch (State)
            {
                case States.MEASURING:
                {
                    if (_wirelessMasterDevice?.gotoConfig() == true)
                    {
                        // ICC (optional — commented out in original)
                    }
                    else
                    {
                        Log?.Invoke(string.Format("Failed to stop measurement. ID: {0}",
                            _wirelessMasterDevice?.deviceId().toXsString().toString()));
                    }
                }
                break;
            }
        }

        internal ConnectedMTwData? GetMtwData(uint deviceId)
        {
            _mtwData.TryGetValue(deviceId, out var data);
            return data;
        }

        public void Dispose()
        {
            // Unhook master callback
            if (_wirelessMasterDevice != null)
            {
                try { _wirelessMasterDevice.clearCallbackHandlers(); } catch { }
            }

            ClearMeasuringMtws();

            _callbackHandler.MtwWireless            -= OnMtwWireless;
            _callbackHandler.MtwDisconnected        -= OnMtwDisconnected;
            _callbackHandler.MeasurementStarted     -= OnMeasurementStarted;
            _callbackHandler.MeasurementStopped     -= OnMeasurementStopped;
            _callbackHandler.DeviceError            -= OnDeviceError;
            _callbackHandler.WaitingForRecordingStart -= OnWaitingForRecordingStart;
            _callbackHandler.RecordingStarted       -= OnRecordingStarted;
            _callbackHandler.ProgressUpdate         -= OnProgressUpdate;

            _myxda.WirelessMasterDetected -= OnWirelessMasterDetected;
            _myxda.DockedMtwDetected -= OnDockedMtwDetected;
            _myxda.MtwUndocked -= OnMtwUndocked;
            _myxda.OpenPortSuccessful -= OnOpenPortSuccessful;
            _myxda.OpenPortFailed -= OnOpenPortFailed;

            _myxda.Dispose();
        }

        // ── MyXda callbacks ────────────────────────────────────────────────────

        private void OnOpenPortSuccessful(object? sender, PortInfoArg e)
        {
            switch (State)
            {
                case States.CONNECTING:
                    if (e.PortInfo.deviceId().isWirelessMaster())
                    {
                        _wirelessMasterDevice = _myxda.getDevice(e.PortInfo.deviceId());

                        // Attach the callback handler
                        _wirelessMasterDevice.addCallbackHandler(_callbackHandler);

                        State = States.CONNECTED;
                        Log?.Invoke(string.Format("Master Connected. Port: {0}, ID: {1}",
                            e.PortInfo.portName().toString(),
                            e.PortInfo.deviceId().toXsString().toString()));

                        // Ensure station enters operational state
                        var did = e.PortInfo.deviceId();
                        bool isStation = did.isAwindaXStation() || did.isAwinda2Station();
                        if (isStation)
                        {
                            if (_wirelessMasterDevice.deviceState() != XsDeviceState.XDS_Config)
                                _wirelessMasterDevice.gotoConfig();

                            bool ok = _wirelessMasterDevice.makeOperational();
                            Log?.Invoke($"makeOperational() => {ok}");
                        }

                        // Be sure to start with radio disabled
                        if (_wirelessMasterDevice.isRadioEnabled())
                        {
                            SetRadioChannel(-1);
                        }
                        SetRadioChannel(11);
                    }
                    break;
                default:
                    break;
            }
        }

        private void OnWirelessMasterDetected(object? sender, PortInfoArg e)
        {
            if (_myxda != null)
            {
                switch (State)
                {
                    case States.DETECTING:
                        Log?.Invoke(string.Format("Master Detected. Port: {0}, ID: {1}",
                            e.PortInfo.portName().toString(),
                            e.PortInfo.deviceId().toXsString().toString()));
                        State = States.CONNECTING;
                        _myxda.openPort(e.PortInfo);
                        break;
                    default:
                        break;
                }
            }
        }

        private void OnDockedMtwDetected(object? sender, PortInfoArg e)
        {
            Log?.Invoke(string.Format("MTw Docked. Port: {0}, ID: {1}",
                e.PortInfo.portName().toString(),
                e.PortInfo.deviceId().toXsString().toString()));
        }

        private void OnMtwUndocked(object? sender, PortInfoArg e)
        {
            Log?.Invoke(string.Format("MTw Undocked. Port: {0}, ID: {1}",
                e.PortInfo.portName().toString(),
                e.PortInfo.deviceId().toXsString().toString()));
        }

        private void OnOpenPortFailed(object? sender, PortInfoArg e)
        {
            if (e.PortInfo.deviceId().isWirelessMaster())
            {
                Log?.Invoke(string.Format("Connect to wireless master failed. Port: {0}",
                    e.PortInfo.portName().toString()));
            }
            else
            {
                Log?.Invoke(string.Format("Connect to device failed. Port: {0}",
                    e.PortInfo.portName().toString()));
            }

            switch (State)
            {
                case States.CONNECTING:
                    Log?.Invoke("Closing XDA");
                    _myxda.reset();
                    State = States.DETECTING;
                    break;
                default:
                    break;
            }
        }

        // ── MyWirelessMasterCallback callbacks ─────────────────────────────────

        private void OnMtwWireless(object? sender, DeviceIdArg e)
        {
            uint id = e.DeviceId.legacyDeviceId();
            string mtwIdStr = e.DeviceId.toXsString().toString();

            bool isNew = !_mtwData.ContainsKey(id);
            if (isNew)
            {
                var connectedMtwData = new ConnectedMTwData
                {
                    _rssi = 0,
                    _frameSkipsList = new List<int>()
                };
                _mtwData[id] = connectedMtwData;

                int totalConnected = _mtwData.Count;
                Log?.Invoke(string.Format("Connected MTw list ({0}):{1}", totalConnected, mtwIdStr));
                MtwConnected?.Invoke(new MtwConnectedEvent(id, mtwIdStr, totalConnected));
            }
        }

        private void OnMtwDisconnected(object? sender, DeviceIdArg e)
        {
            uint id = e.DeviceId.legacyDeviceId();
            string mtwIdStr = e.DeviceId.toXsString().toString();

            bool wasPresent = _mtwData.TryRemove(id, out _);
            if (wasPresent)
            {
                _mtwLocks.TryRemove(id, out _);
                int totalConnected = _mtwData.Count;
                Log?.Invoke(string.Format("MTw Disconnected. ID: {0}", mtwIdStr));
                Log?.Invoke(string.Format("Connected MTw list ({0}):", totalConnected));
                MtwDisconnected?.Invoke(new MtwDisconnectedEvent(id, mtwIdStr, totalConnected));
            }
        }

        private void OnMeasurementStarted(object? sender, DeviceIdArg e)
        {
            Log?.Invoke(string.Format("Measurement Started. ID: {0}", e.DeviceId.toXsString().toString()));

            if (_myxda.getDevice(e.DeviceId)?.deviceId().legacyDeviceId() == _wirelessMasterDevice?.deviceId().legacyDeviceId())
            {
                switch (State)
                {
                    case States.AWAIT_MEASUREMENT_START:
                    {
                        ClearMeasuringMtws();
                        List<XsDeviceId> deviceIds = _callbackHandler.getConnectedMtws();
                        lock (_measuringMtwsLock)
                        {
                            foreach (XsDeviceId devId in deviceIds)
                            {
                                XsDevice mtw = _myxda.getDevice(devId);
                                if (mtw != null)
                                {
                                    mtw.setSyncSettings(new XsSyncSettingArray());

                                    MyMtwCallback callback = new MyMtwCallback();
                                    callback.DataAvailable += OnDataAvailable;
                                    callback.BatteryLevelChanged += OnBatteryLevelChanged;

                                    mtw.addCallbackHandler(callback);
                                    _measuringMtws[mtw] = callback;
                                }
                            }
                            _nextBatteryRequest = _measuringMtws.GetEnumerator();
                            _batteryEnumeratorInitialized = true;
                        }
                        State = States.MEASURING;
                    }
                    break;

                    case States.RECORDING:
                    case States.FLUSHING:
                        Log?.Invoke(string.Format("Recording Finished. ID: {0}", e.DeviceId.toXsString().toString()));
                        _wirelessMasterDevice?.closeLogFile();
                        State = States.MEASURING;
                        break;

                    default:
                        break;
                }
            }
        }

        private void OnMeasurementStopped(object? sender, DeviceIdArg e)
        {
            Log?.Invoke(string.Format("Measurement Stopped. ID: {0}", e.DeviceId.toXsString().toString()));
            if (e.DeviceId.toInt() == _wirelessMasterDevice?.deviceId().legacyDeviceId())
            {
                ClearMeasuringMtws();
                State = States.OPERATIONAL;
            }
        }

        private void OnDeviceError(object? sender, DeviceErrorArgs e)
        {
            Log?.Invoke(string.Format("ERROR. ID: {0}", e.DeviceId.toXsString().toString()));
            switch (State)
            {
                case States.AWAIT_MEASUREMENT_START:
                    State = States.ENABLED;
                    break;
                default:
                    break;
            }
        }

        private void OnWaitingForRecordingStart(object? sender, DeviceIdArg e)
        {
            Log?.Invoke(string.Format("Waiting for recording start. ID: {0}",
                _wirelessMasterDevice?.deviceId().toXsString().toString()));
            State = States.AWAIT_RECORDING_START;
        }

        private void OnRecordingStarted(object? sender, DeviceIdArg e)
        {
            if (State == States.AWAIT_RECORDING_START)
            {
                Log?.Invoke(string.Format("Waiting for recording start. ID: {0}",
                    _wirelessMasterDevice?.deviceId().toXsString().toString()));
                State = States.RECORDING;
            }
        }

        private void OnProgressUpdate(object? sender, ProgressUpdateArgs e)
        {
            if (State == States.FLUSHING && e.Identifier == "Flushing")
            {
                // Nothing to flush when at the highest update rate (SelectedRate == 0 in old code).
                // Manager doesn't know SelectedRate, so expose the raw event and let caller decide.
                // For now replicate: if total == 0, abort flushing
                if (e.Total == 0)
                {
                    _wirelessMasterDevice?.abortFlushing();
                    Log?.Invoke(string.Format("Flushing aborted. ID: {0}",
                        _wirelessMasterDevice?.deviceId().toXsString().toString()));
                }
            }
        }

        private void OnDataAvailable(object? sender, DataAvailableArgs e)
        {
            uint deviceId = e.Device.deviceId().legacyDeviceId();
            string mtwIdStr = e.Device.deviceId().toXsString().toString();

            if (!_mtwData.TryGetValue(deviceId, out var mtwData))
            {
                Log?.Invoke(string.Format("Obsolete data received of an MTw {0} that's no longer in the list.", mtwIdStr));
                return;
            }

            if (!e.Packet.containsSdiData())
            {
                Log?.Invoke(string.Format("Packet received of an MTw {0} not containing SDI data.", mtwIdStr));
            }

            var mtwLock = _mtwLocks.GetOrAdd(deviceId, _ => new object());
            lock (mtwLock)
            {
                if (e.Packet.containsSdiData())
                {
                    XsSdiData sdiData = e.Packet.sdiData();
                    mtwData.XsQuaternion = sdiData.orientationIncrement();
                }

                if (e.Packet.containsRssi())
                    mtwData._rssi = e.Packet.rssi();

                if (e.Packet.containsOrientation())
                {
                    XsEuler oriEuler = e.Packet.orientationEuler();
                    mtwData._orientation = oriEuler;
                }

                // Determine effective update rate percentage
                int frameSkips;
                if (e.Packet.frameRange().last() > e.Packet.frameRange().first())
                {
                    frameSkips = e.Packet.frameRange().last() - e.Packet.frameRange().first() - 1;
                }
                else
                {
                    // Rollover (internal framecounter is unsigned 16 bits integer)
                    frameSkips = 65535 + e.Packet.frameRange().last() - e.Packet.frameRange().first() - 1;
                }

                mtwData._frameSkipsList.Add(frameSkips);
                mtwData._sumFrameSkips = mtwData._sumFrameSkips + (uint)frameSkips;
                mtwData._effectiveUpdateRate = (int)(100 * (1 - (float)mtwData._sumFrameSkips /
                    (float)(mtwData._frameSkipsList.Count + mtwData._sumFrameSkips)));

                while (mtwData._frameSkipsList.Count + mtwData._sumFrameSkips > 99 &&
                       mtwData._frameSkipsList.Count > 0)
                {
                    mtwData._sumFrameSkips = mtwData._sumFrameSkips - (uint)mtwData._frameSkipsList[0];
                    mtwData._frameSkipsList.RemoveAt(0);
                }
            }

            ImuViewModel slot = _registry.GetOrAssign(deviceId);
            DataPacketReceived?.Invoke(new DataPacketEvent(deviceId, slot, e.Packet));
        }

        private void OnBatteryLevelChanged(object? sender, BatteryLevelChangedArgs e)
        {
            uint id = e.DeviceId.legacyDeviceId();
            string mtwIdStr = e.DeviceId.toXsString().toString();

            if (!_mtwData.TryGetValue(id, out var data))
            {
                Log?.Invoke(string.Format("Obsolete data received of an MTw {0} that's no longer in the list.", mtwIdStr));
                return;
            }

            data._batteryLevel = e.Level;
            BatteryLevelChanged?.Invoke(new BatteryEvent(id, e.Level));
        }

        // ── Private helpers ────────────────────────────────────────────────────

        private void ClearMeasuringMtws()
        {
            lock (_measuringMtwsLock)
            {
                foreach (var item in _measuringMtws)
                {
                    item.Key.clearCallbackHandlers();
                }
                _measuringMtws.Clear();
                if (_batteryEnumeratorInitialized)
                {
                    _nextBatteryRequest.Dispose();
                    _batteryEnumeratorInitialized = false;
                }
            }
        }

        private void SetRadioChannel(int channel)
        {
            if (State != States.CONNECTED)
            {
                Log?.Invoke($"Not Connected, Device State is {State}");
                return;
            }

            if (_wirelessMasterDevice?.enableRadio(channel) == true)
            {
                if (channel != -1)
                {
                    Log?.Invoke(string.Format("Master Enabled. ID: {0}, Channel: {1}",
                        _wirelessMasterDevice?.deviceId().toXsString().toString(), channel));

                    XsIntArray supportedRates = _wirelessMasterDevice?.supportedUpdateRates();
                    int maxUpdateRate = _wirelessMasterDevice?.maximumUpdateRate() ?? 0;

                    var rates = new List<string>();
                    for (uint i = 0; i < supportedRates.size() && supportedRates.at(i) <= maxUpdateRate; ++i)
                    {
                        rates.Add(supportedRates.at(i).ToString());
                    }

                    // Select the current update rate of the station.
                    int updateRateIndex = rates.IndexOf(Convert.ToString(_wirelessMasterDevice?.updateRate()));

                    State = States.ENABLED;

                    // Set a default update rate of 75 (if available)
                    int defaultIndex = rates.IndexOf(Convert.ToString(75));
                    if (defaultIndex != -1)
                        updateRateIndex = defaultIndex;

                    UpdateRatesAvailable?.Invoke(new UpdateRatesEvent(rates, updateRateIndex));
                }
                else
                {
                    Log?.Invoke(string.Format("Master Disabled. ID: {0}",
                        _wirelessMasterDevice?.deviceId().toXsString().toString()));
                    State = States.CONNECTED;
                }
            }
            else
            {
                if (channel != -1)
                {
                    Log?.Invoke(string.Format("Failed to enable wireless master. ID: {0}, Channel: {1}",
                        _wirelessMasterDevice?.deviceId().toXsString().toString(), channel));
                }
                else
                {
                    Log?.Invoke(string.Format("Failed to disable wireless master. ID: {0}",
                        _wirelessMasterDevice?.deviceId().toXsString().toString()));
                }
            }
        }
    }
}
