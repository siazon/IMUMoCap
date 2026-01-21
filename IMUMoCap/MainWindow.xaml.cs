using IMUMoCap.AHRS;
using System.Net.Sockets;
using System.Numerics;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Windows.Threading;
using XDA;
using static System.Net.Mime.MediaTypeNames;
using Application = System.Windows.Application;

namespace IMUMoCap
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private MainPageVM _content = new MainPageVM();
        private XsDevice? _MyWirelessMasterDevice;
        private MyXda _myxda;
        private MyWirelessMasterCallback m_myWirelessMasterCallback;
        private Dictionary<XsDevice, MyMtwCallback> _measuringMtws;
        private Dictionary<XsDevice, MyMtwCallback>.Enumerator _nextBatteryRequest;
        private Dictionary<uint, ConnectedMTwData> _connectedMtwData;

        Queue<double[]> actionQueue = new Queue<double[]>();
        Queue<RecoredData> ImuDataQueue = new Queue<RecoredData>();
        public MainWindow()
        {
            InitializeComponent();
            this.DataContext = _content;

            _measuringMtws = new Dictionary<XsDevice, MyMtwCallback>();
            _connectedMtwData = new Dictionary<uint, ConnectedMTwData>();

            _myxda = new MyXda();
            _myxda.WirelessMasterDetected += new EventHandler<PortInfoArg>(_myxda_WirelessMasterDetected);
            _myxda.DockedMtwDetected += new EventHandler<PortInfoArg>(_myxda_DockedMtwDetected);
            _myxda.MtwUndocked += new EventHandler<PortInfoArg>(_myxda_MtwUndocked);
            _myxda.OpenPortSuccessful += new EventHandler<PortInfoArg>(_myxda_OpenPortSuccessful);
            _myxda.OpenPortFailed += new EventHandler<PortInfoArg>(_myxda_OpenPortFailed);

            m_myWirelessMasterCallback = new MyWirelessMasterCallback();
            m_myWirelessMasterCallback.MtwWireless += new EventHandler<DeviceIdArg>(_callbackHandler_MtwWireless);
            m_myWirelessMasterCallback.MtwDisconnected += new EventHandler<DeviceIdArg>(_callbackHandler_MtwDisconnected);
            m_myWirelessMasterCallback.MeasurementStarted += new EventHandler<DeviceIdArg>(_callbackHandler_MeasurementStarted);
            m_myWirelessMasterCallback.MeasurementStopped += new EventHandler<DeviceIdArg>(_callbackHandler_MeasurementStopped);
            m_myWirelessMasterCallback.DeviceError += new EventHandler<DeviceErrorArgs>(_callbackHandler_DeviceError);
            m_myWirelessMasterCallback.WaitingForRecordingStart += new EventHandler<DeviceIdArg>(_callbackHandler_WaitingForRecordingStart);
            m_myWirelessMasterCallback.RecordingStarted += new EventHandler<DeviceIdArg>(_callbackHandler_RecordingStarted);
            m_myWirelessMasterCallback.ProgressUpdate += new EventHandler<ProgressUpdateArgs>(_callbackHandler_ProgressUpdate);
            InitDevice();
        }

        

        private void InitDevice()
        {
            Task.Run(() =>
            {
                if (_content.DeviceState != States.MEASURING || _content.DeviceState != States.AWAIT_MEASUREMENT_START || _content.DeviceState != States.RECORDING || _content.DeviceState != States.FLUSHING)
                {
                    _myxda.scanPorts();
                }
                Thread.Sleep(1000);

                if (_measuringMtws.Count == 0)
                    return;
                // It is impossible to request battery status for all MTWs at once. So cycle between them
                if (!_nextBatteryRequest.MoveNext())
                    _nextBatteryRequest = _measuringMtws.GetEnumerator();
                else
                    _nextBatteryRequest.Current.Key.requestBatteryLevel();

                Thread.Sleep(1000);
            });
            int idx = 0;
            Task.Run(() =>
            {
                while (true)
                {
                    RecoredData? info = ImuDataQueue.Count > 0 ? ImuDataQueue.Dequeue() : null;
                    if (info != null)
                    {
                        double[] qua = new double[] { info.Querternion.x, info.Querternion.y, info.Querternion.z, info.Querternion.w };
                        double[] anu = new double[] { info.Orientation.X, info.Orientation.Y, info.Orientation.Z };
                        QuaternionHelper quaternionHelper = new QuaternionHelper();
                        var angle1 = quaternionHelper.GetYawAngle(qua, anu);
                        var angle2 = quaternionHelper.GetPitchAngle(qua, anu);
                        var angle3 = quaternionHelper.GetRollAngle(qua, anu);
                        idx++;
                        if (idx % 100 == 0)
                        {
                            log($"data:{info.PackageId},{info.Orientation}");

                            var q = new System.Windows.Media.Media3D.Quaternion(info.Querternion.x, info.Querternion.y, info.Querternion.z, info.Querternion.w);
                            var rot = new QuaternionRotation3D(q);
                            var transform = new RotateTransform3D(rot);
                            //Application.Current.Dispatcher.BeginInvoke(() =>
                            //{
                            //_content.Transform3D = transform;
                            //deviceBox.Transform = transform;
                            //forwardArrow.Transform = transform;
                            //});
                        }
                    }
                    Thread.Sleep(10);
                }
            });
        }

        #region MyXda
        void _myxda_OpenPortSuccessful(object? sender, PortInfoArg e)
        {
            this.Dispatcher.BeginInvoke(() =>
            {
                // Update the UI
                switch (_content.DeviceState)
                {
                    case States.CONNECTING:
                        if (e.PortInfo.deviceId().isWirelessMaster())
                        {
                            // Set the label to indicate the ID of the station.
                            _content.ConnecetBtn = e.PortInfo.deviceId().toXsString().toString();
                            _MyWirelessMasterDevice = _myxda.getDevice(e.PortInfo.deviceId());

                            // Attach the callback handler. This causes events to arrive in m_myWirelessMasterCallback.
                            _MyWirelessMasterDevice.addCallbackHandler(m_myWirelessMasterCallback);

                            _content.DeviceState = States.CONNECTED;
                            log(string.Format("Master Connected. Port: {0}, ID: {1}", e.PortInfo.portName().toString(), e.PortInfo.deviceId().toXsString().toString()));

                            // Be sure to start with radio disabled
                            if (_MyWirelessMasterDevice.isRadioEnabled())
                            {
                                SetRadioChannel(-1);
                            }
                            SetRadioChannel(11);
                            setWidgetsStates();
                        }
                        break;
                    default:
                        break;
                }
            });
        }
        void _myxda_WirelessMasterDetected(object? sender, PortInfoArg e)
        {
            if (_myxda != null)
            {
                this.Dispatcher.BeginInvoke(() =>
                {
                    switch (_content.DeviceState)
                    {
                        case States.DETECTING:
                            log(string.Format("Master Detected. Port: {0}, ID: {1}", e.PortInfo.portName().toString(), e.PortInfo.deviceId().toXsString().toString()));
                            _content.DeviceState = States.CONNECTING;
                            _myxda.openPort(e.PortInfo);
                            setWidgetsStates();
                            break;
                        default:
                            break;
                    }
                });
            }
        }
        void _myxda_DockedMtwDetected(object? sender, PortInfoArg e)
        {
            this.Dispatcher.BeginInvoke(() =>
            {
                log(string.Format("MTw Docked. Port: {0}, ID: {1}", e.PortInfo.portName().toString(), e.PortInfo.deviceId().toXsString().toString()));

                string mtwId = e.PortInfo.deviceId().toXsString().toString();
                //if (dockedMtwList.FindStringExact(mtwId) == ListBox.NoMatches)
                //{
                //    dockedMtwList.Items.Add(mtwId);
                //}
                //dockedMtwListGroupBox.Text = string.Format("Docked MTw list ({0}):", dockedMtwList.Items.Count);
            });
        }
        void _myxda_MtwUndocked(object? sender, PortInfoArg e)
        {
            this.Dispatcher.BeginInvoke(() =>
            {
                log(string.Format("MTw Undocked. Port: {0}, ID: {1}", e.PortInfo.portName().toString(), e.PortInfo.deviceId().toXsString().toString()));

                string mtwId = e.PortInfo.deviceId().toXsString().toString();

                //dockedMtwList.Items.Remove(mtwId);
                //dockedMtwListGroupBox.Text = string.Format("Docked MTw list ({0}):", dockedMtwList.Items.Count);
            });
        }
        void _myxda_OpenPortFailed(object? sender, PortInfoArg e)
        {
            this.Dispatcher.BeginInvoke(() =>
            {
                if (e.PortInfo.deviceId().isWirelessMaster())
                {
                    log(string.Format("Connect to wireless master failed. Port: {0}", e.PortInfo.portName().toString()));
                }
                else
                {
                    log(string.Format("Connect to device failed. Port: {0}", e.PortInfo.portName().toString()));
                }

                switch (_content.DeviceState)
                {
                    case States.CONNECTING:
                        log("Closing XDA");
                        _myxda.reset();
                        _content.DeviceState = States.DETECTING;
                        setWidgetsStates();
                        break;
                    default:
                        break;
                }
            });
        }

        void _callbackHandler_MtwWireless(object? sender, DeviceIdArg e)
        {
            this.Dispatcher.BeginInvoke(() =>
            {
                //log(string.Format("MTw Connected. ID: {0}", e.DeviceId.toXsString().toString()));

                string mtwIdStr = e.DeviceId.toXsString().toString();
                ConnectedMTwData connectedMtwData = new ConnectedMTwData();
                if (_content.ConnectedMtws.IndexOf(mtwIdStr) < 0)
                {
                    _content.ConnectedMtws.Add(mtwIdStr);
                    _content.DeviceModels.Add(new DeviceModel() { DeviceName = mtwIdStr });
                    _content.SelectedMtw = _content.ConnectedMtws.Count - 1;
                    // This is a new MTw, add it.
                    connectedMtwData._rssi = 0;
                    connectedMtwData._frameSkipsList = new List<int>();
                    _connectedMtwData[e.DeviceId.legacyDeviceId()] = connectedMtwData;

                    log(string.Format("Connected MTw list ({0}):{1}", _content.ConnectedMtws.Count, mtwIdStr));
                }
                //btnMeasure.Enabled =_content.DeviceState == States.ENABLED && connectedMtwList.Items.Count > 0;
            });
        }

        void _callbackHandler_MtwDisconnected(object? sender, DeviceIdArg e)
        {
            this.Dispatcher.BeginInvoke(() =>
            {
                string mtwIdStr = e.DeviceId.toXsString().toString();
                log(string.Format("MTw Disconnected. ID: {0}", mtwIdStr));

                Int32 index = _content.ConnectedMtws.IndexOf(mtwIdStr);
                if (index < 0)
                {
                    // Found --> delete
                    _content.ConnectedMtws.Remove(mtwIdStr);
                    _connectedMtwData.Remove(e.DeviceId.legacyDeviceId());

                    log(string.Format("Connected MTw list ({0}):", _content.ConnectedMtws.Count));
                    //btnMeasure.Enabled =_content.DeviceState == States.ENABLED && connectedMtwList.Items.Count > 0;
                }
            });
        }
        void _callbackHandler_MeasurementStarted(object? sender, DeviceIdArg e)
        {


            this.Dispatcher.BeginInvoke(() =>
            {
                log(string.Format("Measurement Started. ID: {0}", e.DeviceId.toXsString().toString()));

                if (_myxda.getDevice(e.DeviceId).deviceId().legacyDeviceId() == _MyWirelessMasterDevice?.deviceId().legacyDeviceId())
                {
                    switch (_content.DeviceState)
                    {
                        case States.AWAIT_MEASUREMENT_START:
                            {
                                // Get the MTws that are measuring and attach callback handlers
                                clearMeasuringMtws();
                                List<XsDeviceId> deviceIds = m_myWirelessMasterCallback.getConnectedMtws();
                                foreach (XsDeviceId devId in deviceIds)
                                {
                                    XsDevice mtw = _myxda.getDevice(devId);

                                    if (mtw != null)
                                    {
                                        mtw.setSyncSettings(new XsSyncSettingArray());

                                        MyMtwCallback callback = new MyMtwCallback();

                                        // connect signals
                                        callback.DataAvailable += new EventHandler<DataAvailableArgs>(_callbackHandler_DataAvailable);
                                        callback.BatteryLevelChanged += new EventHandler<BatteryLevelChangedArgs>(_callbackHandler_BatteryLevelChanged);

                                        mtw.addCallbackHandler(callback);
                                        _measuringMtws[mtw] = callback;
                                    }
                                }
                                _nextBatteryRequest = _measuringMtws.GetEnumerator();
                                _content.DeviceState = States.MEASURING;
                                setWidgetsStates();
                            }
                            break;

                        case States.RECORDING:
                        case States.FLUSHING:
                            log(string.Format("Recording Finished. ID: {0}", e.DeviceId.toXsString().toString()));
                            // Ready recording (flushing also ready), so file can be closed.
                            _MyWirelessMasterDevice?.closeLogFile();
                            _content.DeviceState = States.MEASURING;
                            setWidgetsStates();
                            break;
                        default:
                            break;
                    }
                }
            });
        }
        private void clearMeasuringMtws()
        {
            lock (_measuringMtws)
            {
                foreach (KeyValuePair<XsDevice, MyMtwCallback> item in _measuringMtws)
                {
                    item.Key.clearCallbackHandlers();
                }
            }
            _measuringMtws.Clear();
            _nextBatteryRequest.Dispose();
        }
        void _callbackHandler_MeasurementStopped(object? sender, DeviceIdArg e)
        {
            this.Dispatcher.BeginInvoke(() =>
            {
                log(string.Format("Measurement Stopped. ID: {0}", e.DeviceId.toXsString().toString()));
                if (e.DeviceId.toInt() == _MyWirelessMasterDevice?.deviceId().legacyDeviceId())
                {
                    clearMeasuringMtws();
                    _content.DeviceState = States.OPERATIONAL;
                    setWidgetsStates();
                }
            });
        }
        void _callbackHandler_DeviceError(object? sender, DeviceErrorArgs e)
        {
            this.Dispatcher.BeginInvoke(() =>
            {
                log(string.Format("ERROR. ID: {0}", e.DeviceId.toXsString().toString()));
                switch (_content.DeviceState)
                {
                    case States.AWAIT_MEASUREMENT_START:
                        _content.DeviceState = States.ENABLED;
                        setWidgetsStates();
                        break;
                    default:
                        break;
                }
            });
        }
        void _callbackHandler_WaitingForRecordingStart(object? sender, DeviceIdArg e)
        {
            this.Dispatcher.BeginInvoke(() =>
            {
                log(string.Format("Waiting for recording start. ID: {0}", _MyWirelessMasterDevice?.deviceId().toXsString().toString()));
                _content.DeviceState = States.AWAIT_RECORDING_START;
                setWidgetsStates();
            });
        }
        void _callbackHandler_RecordingStarted(object? sender, DeviceIdArg e)
        {
            this.Dispatcher.BeginInvoke(() =>
            {
                if (_content.DeviceState == States.AWAIT_RECORDING_START)
                {
                    log(string.Format("Waiting for recording start. ID: {0}", _MyWirelessMasterDevice?.deviceId().toXsString().toString()));
                    _content.DeviceState = States.RECORDING;
                    setWidgetsStates();
                }
            });
        }
        void _callbackHandler_ProgressUpdate(object? sender, ProgressUpdateArgs e)
        {
            this.Dispatcher.BeginInvoke(() =>
            {
                if (_content.DeviceState == States.FLUSHING && e.Identifier == "Flushing")
                {
                    if (_content.SelectedRate == 0)
                    {
                        // Nothing to flush when at the highest update rate.
                        _MyWirelessMasterDevice?.abortFlushing();
                        log(string.Format("Flushing aborted. ID: {0}", _MyWirelessMasterDevice?.deviceId().toXsString().toString()));
                    }

                    if (e.Total != 0 && _content.SelectedRate != 0)
                    {
                        // Only do this when there is still data to be flushed
                        // and not the highest update rate was selected.
                        //progressBarFlushing.Maximum = e.Total;
                        //progressBarFlushing.Value = e.Current;
                    }
                }
            });
        }
        int qty = 0;
        bool docalibrate = false;
        void _callbackHandler_DataAvailable(object? sender, DataAvailableArgs e)
        {
            this.Dispatcher.BeginInvoke(() =>
            {
                string mtwIdStr = e.Device.deviceId().toXsString().toString();
                int index = _content.ConnectedMtws.IndexOf(mtwIdStr);

                if (index < 0)
                {
                    log(string.Format("Obsolete data received of an MTw {0} that's no longer in the list.", mtwIdStr));
                    return;
                }
                if (docalibrate)
                {
                    if (e.Packet.containsCalibratedData())
                    {
                        var res = e.Packet.calibratedData();
                        var accres = res.m_acc.value(0);
                        res.m_acc = new XsVector3(0, 0, 0);
                        res.m_mag = new XsVector3(0, 0, 0);
                        res.m_gyr = new XsVector3(0, 0, 0);
                        e.Packet.setCalibratedData(res);

                        accres = res.m_acc.value(0);
                        docalibrate = false;
                    }

                }

                if (!e.Packet.containsSdiData())
                {
                    log(string.Format("Packet received of an MTw {0} not containing data.", mtwIdStr));
                    return;
                }

                // Getting SDI data.
                XsSdiData sdiData = e.Packet.sdiData();
                uint deviceId = e.Device.deviceId().legacyDeviceId();
                _connectedMtwData[deviceId].XsQuaternion = sdiData.orientationIncrement(); //xsQuaternion;

                _connectedMtwData[deviceId]._rssi = e.Packet.rssi();

                if (e.Packet.containsUtcTime())
                {
                    var time = e.Packet.utcTime();
                    _connectedMtwData[deviceId].XsTime = time;
                }
                else
                {
                    XsTimeInfo timeInfo = new XsTimeInfo();
                    DateTime now = DateTime.UtcNow;
                    timeInfo.m_hour = (byte)now.Hour;
                    timeInfo.m_minute = (byte)now.Minute;
                    timeInfo.m_second = (byte)now.Second;
                    timeInfo.m_nano = (uint)now.Nanosecond;
                    e.Packet.setUtcTime(timeInfo);
                }
                if (e.Packet.containsOrientation())
                {
                    //Getting Euler angles.
                    XsEuler oriEuler = e.Packet.orientationEuler();

                    // Just for fun: pitch to select.
                    // (you only want to select this in the GUI after the XKF-3w filters stabilized though)
                    //if (checkBoxPitchToSelect.Checked == true && Math.Abs(oriEuler.y()) > 30)
                    //{
                    //ConnectedMtw[SelectedMtw] = mtwIdStr;
                    //}
                    _connectedMtwData[deviceId]._orientation = oriEuler;

                }
                // -- Determine effective update rate percentage --

                // Determine the number of frames over which the SDI data in this
                // packet was determined.
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

                _connectedMtwData[deviceId]._frameSkipsList.Add(frameSkips);
                _connectedMtwData[deviceId]._sumFrameSkips = _connectedMtwData[deviceId]._sumFrameSkips + (uint)frameSkips;
                _connectedMtwData[deviceId]._effectiveUpdateRate = (int)(100 * (1 - (float)_connectedMtwData[deviceId]._sumFrameSkips / (float)(_connectedMtwData[deviceId]._frameSkipsList.Count() + _connectedMtwData[deviceId]._sumFrameSkips)));

                while (_connectedMtwData[deviceId]._frameSkipsList.Count() + _connectedMtwData[deviceId]._sumFrameSkips > 99 && _connectedMtwData[deviceId]._frameSkipsList.Count() > 0)
                {
                    _connectedMtwData[deviceId]._sumFrameSkips = _connectedMtwData[deviceId]._sumFrameSkips - (uint)_connectedMtwData[deviceId]._frameSkipsList[0];
                    _connectedMtwData[deviceId]._frameSkipsList.RemoveAt(0);
                }

                ConnectedMTwData mtwData = _connectedMtwData[deviceId];
                var ax = 0d;
                var ay = 0d;
                var az = 0d;

                //var temo = e.Packet.accelerationHR();

                var ac = e.Packet.freeAcceleration();

                var temo = e.Packet.calibratedAcceleration();
                var aa = temo.size();
                ax = temo.value(0);
                ay = temo.value(1);
                az = temo.value(2);

                var X = mtwData._orientation.x();
                var Y = mtwData._orientation.y();
                var Z = mtwData._orientation.z();

                _content.DeviceModels[index].XsTime = mtwData.XsTime?.ToString().PadRight(12, '0');
                _content.DeviceModels[index].X = X.RoundTwo();
                _content.DeviceModels[index].Y = Y.RoundTwo();
                _content.DeviceModels[index].Z = Z.RoundTwo();
                _content.DeviceModels[index].packetId = e.Packet.packetId().ToString();

                RecoredData recoredData = new RecoredData()
                {
                    PackageId = e.Packet.packetId().ToString(),
                    Querternion = new Querternion()
                    {
                        x = mtwData.XsQuaternion.x(),
                        y = mtwData.XsQuaternion.y(),
                        z = mtwData.XsQuaternion.z(),
                        w = mtwData.XsQuaternion.w(),
                    },
                    Accelerate = new Vector3()
                    {
                        X = (float)ax,
                        Y = (float)ay,
                        Z = (float)az
                    },
                    Orientation = new Vector3()
                    {
                        X = (float)X,
                        Y = (float)Y,
                        Z = (float)Z
                    }

                };
                qty++;
                if (qty % 10 == 0)
                    Application.Current.Dispatcher.BeginInvoke(() =>
                    {
                        var q = new System.Windows.Media.Media3D.Quaternion(recoredData.Querternion.x, recoredData.Querternion.y, recoredData.Querternion.z, recoredData.Querternion.w);
                        var rot = new QuaternionRotation3D(q);
                        _content.Transform3D = new RotateTransform3D(rot);
                    });
                double[] qua = new double[] { recoredData.Querternion.x, recoredData.Querternion.y, recoredData.Querternion.z, recoredData.Querternion.w };
                double[] anu = new double[] { recoredData.Orientation.X, recoredData.Orientation.Y, recoredData.Orientation.Z };
                QuaternionHelper quaternionHelper = new QuaternionHelper();
                _content.DeviceModels[index].Angle = quaternionHelper.GetYawAngle(qua, anu);
                _content.DeviceModels[index].AngleXZ = quaternionHelper.GetPitchAngle(qua, anu);
                _content.DeviceModels[index].AngleYZ = quaternionHelper.GetRollAngle(qua, anu);

                //var item = recoredData;
                //var madgwick = new MadgwickAHRS(0.01f);
                //madgwick.Update(item.Orientation.X, item.Orientation.Y, item.Orientation.Z, item.Accelerate.X, item.Accelerate.Y, item.Accelerate.Z);
                //var quaternion = new Quaternion( madgwick.Quaternion[1], madgwick.Quaternion[2], madgwick.Quaternion[3], madgwick.Quaternion[0]);
                //var mad = new AngleCalculater().QuaternionToEuler(quaternion);
                //_content.DeviceModels[index].Angle = mad.X.ConvertRadiansToDegrees();
                //_content.DeviceModels[index].AngleXZ = mad.Y.ConvertRadiansToDegrees();
                //_content.DeviceModels[index].AngleYZ = mad.Z.ConvertRadiansToDegrees();


                switch (mtwIdStr)
                {
                    case "00B43CC0":
                        _content.DataReceived(recoredData, 0);
                        break;
                    case "00B43CBF":
                        _content.DataReceived(recoredData, 1);
                        break;
                    case "00B43D12":
                        _content.DataReceived(recoredData, 2);
                        break;
                    case "00B43CAB":
                        _content.DataReceived(recoredData, 3);
                        break;
                    case "00B43D0B":
                        _content.DataReceived(recoredData, 4);
                        break;
                    case "00B43B3F":
                        _content.DataReceived(recoredData, 5);
                        break;
                    case "10B41904":
                        _content.DataReceived(recoredData, 6);
                        break;
                    case "10B41913":
                        _content.DataReceived(recoredData, 7);
                        break;
                    default:
                        break;
                }

                ImuDataQueue.Enqueue(recoredData);
                if (_content.ConnectedMtws[_content.SelectedMtw] == mtwIdStr)
                {
                    _content.XsTime = $"{mtwIdStr}.{mtwData.XsTime?.ToString().PadRight(12, '0')}{Environment.NewLine}{mtwData._orientation.x().RoundTwo()},{mtwData._orientation.y().RoundTwo()},{mtwData._orientation.z().RoundTwo()}";

                    string key = e.Packet.packetId().ToString();// $"{mtwData.XsTime?.ToString().PadRight(12, '0')}";

                    if (_content.RotationByDegree)
                        actionQueue.Enqueue([_connectedMtwData[deviceId]._orientation.x().DegreesToRadians(), _connectedMtwData[deviceId]._orientation.y().DegreesToRadians(),
                            _connectedMtwData[deviceId]._orientation.z().DegreesToRadians(), 1]);
                    else
                        actionQueue.Enqueue([_connectedMtwData[deviceId].XsQuaternion.x(), _connectedMtwData[deviceId].XsQuaternion.y(), _connectedMtwData[deviceId].XsQuaternion.z(), _connectedMtwData[deviceId].XsQuaternion.w()]);

                }
            });
        }
        void _callbackHandler_BatteryLevelChanged(object? sender, BatteryLevelChangedArgs e)
        {
            this.Dispatcher.BeginInvoke(() =>
            {
                string mtwIdStr = e.DeviceId.toXsString().toString();
                Int32 index = _content.ConnectedMtws.IndexOf(mtwIdStr);
                if (index < 0)
                {
                    log(string.Format("Obsolete data received of an MTw {0} that's no longer in the list.", mtwIdStr));
                    return;
                }

                _connectedMtwData[e.DeviceId.legacyDeviceId()]._batteryLevel = e.Level;

            });
        }
        #endregion
        private void log(string log)
        {
            this.Dispatcher.BeginInvoke(() =>
            {
                _content.LogList += (log);
                _content.LogList += (Environment.NewLine);

                if (_content.IsScollerToEnd)
                {
                    LogBox.CaretIndex = LogBox.Text.Length;
                    LogBox.ScrollToEnd();
                }
            });
        }
        private void SetRadioChannel(int channel)
        {
            if (_content.DeviceState != States.CONNECTED)
            {
                log($"Not Connected, Device State is {_content.DeviceState}");
                return;
            }
            if (_MyWirelessMasterDevice?.enableRadio(channel) == true)
            {
                if (channel != -1)
                {
                    log(string.Format("Master Enabled. ID: {0}, Channel: {1}", _MyWirelessMasterDevice?.deviceId().toXsString().toString(), channel));

                    // Supported update rates and maximum available from xda
                    XsIntArray supportedRates = _MyWirelessMasterDevice?.supportedUpdateRates();
                    int maxUpdateRate = _MyWirelessMasterDevice?.maximumUpdateRate() ?? 0;

                    //--Put the allowed update rates in the combobox for the user to choose from--
                    _content.UpdateRates.Clear();
                    for (uint i = 0; i < supportedRates.size() && supportedRates.at(i) <= maxUpdateRate; ++i)
                    {
                        // This is an allowed update rate, so add it to the list.
                        _content.UpdateRates.Add(supportedRates.at(i).ToString());
                    }

                    // Select the current update rate of the station.
                    int updateRateIndex = _content.UpdateRates.IndexOf(Convert.ToString(_MyWirelessMasterDevice?.updateRate()));//.FindString(Convert.ToString(_MyWirelessMasterDevice?.updateRate()));
                    _content.SelectedRate = updateRateIndex;

                    _content.DeviceState = States.ENABLED;

                    // Set a default update rate of 75 (if available) when we set the radio channel
                    updateRateIndex = _content.UpdateRates.IndexOf(Convert.ToString(75));

                    if (updateRateIndex != -1)
                    {
                        _content.SelectedRate = updateRateIndex;
                    }
                }
                else
                {
                    log(string.Format("Master Disabled. ID: {0}", _MyWirelessMasterDevice?.deviceId().toXsString().toString()));
                    _content.UpdateRates.Clear();
                    _content.DeviceState = States.CONNECTED;
                }
                setWidgetsStates();
            }
            else
            {
                if (channel != -1)
                {
                    log(string.Format("Failed to enable wireless master. ID: {0}, Channel: {1}", _MyWirelessMasterDevice?.deviceId().toXsString().toString(), channel));
                }
                else
                {
                    log(string.Format("Failed to disable wireless master. ID: {0}", _MyWirelessMasterDevice?.deviceId().toXsString().toString()));
                }
            }
        }
        private void setWidgetsStates()
        {
            switch (_content.DeviceState)
            {
                case States.DETECTING:
                    {

                        //pictureBoxStateDiagram.Image = global::awindamonitor.Properties.Resources.connected;
                    }
                    break;

                case States.CONNECTING:
                    {
                        //btnEnable.Enabled = false;
                        //pictureBoxStateDiagram.Image = global::awindamonitor.Properties.Resources.connecting;
                        log("Scanning for station.");
                        //btnRecord.Enabled = false;
                    }
                    break;

                case States.CONNECTED:
                    {
                        //btnMeasure.Enabled = false;
                        //labelChannel.Enabled = true;
                        //comboBoxChannel.Enabled = true;
                        //btnEnable.Enabled = true;
                        //btnEnable.Text = "Enable";
                        //labelUpdateRate.Enabled = false;
                        //comboBoxUpdateRate.Enabled = false;
                        //pictureBoxStateDiagram.Image = global::awindamonitor.Properties.Resources.connected;
                        //btnRecord.Enabled = false;
                    }
                    break;

                case States.ENABLED:
                    {
                        //btnMeasure.Enabled = connectedMtwList.Items.Count > 0;
                        //btnMeasure.Text = "Start Measurement";
                        //btnEnable.Text = "Disable";
                        //btnEnable.Enabled = true;
                        //labelChannel.Enabled = false;
                        //comboBoxChannel.Enabled = false;
                        //labelUpdateRate.Enabled = true;
                        //comboBoxUpdateRate.Enabled = true;
                        //pictureBoxStateDiagram.Image = global::awindamonitor.Properties.Resources.enabled;
                        //btnRecord.Enabled = false;
                    }
                    break;

                case States.OPERATIONAL:
                    {
                        //btnMeasure.Enabled = connectedMtwList.Items.Count > 0;
                        //btnMeasure.Text = "Start Measurement";
                        //btnEnable.Text = "Disable";
                        //btnEnable.Enabled = true;
                        //labelChannel.Enabled = false;
                        //comboBoxChannel.Enabled = false;
                        //labelUpdateRate.Enabled = true;
                        //comboBoxUpdateRate.Enabled = true;
                        //pictureBoxStateDiagram.Image = global::awindamonitor.Properties.Resources.operational;
                        //btnRecord.Enabled = false;
                    }
                    break;

                case States.AWAIT_MEASUREMENT_START:
                case States.AWAIT_RECORDING_START:
                    {
                        //btnEnable.Enabled = false;
                        //btnMeasure.Text = "Waiting for start...";
                        //btnMeasure.Enabled = false;
                        //labelUpdateRate.Enabled = false;
                        //comboBoxUpdateRate.Enabled = false;
                        //pictureBoxStateDiagram.Image = global::awindamonitor.Properties.Resources.await_measurement_start;
                        //btnRecord.Enabled = false;
                    }
                    break;

                case States.MEASURING:
                    {
                        //btnEnable.Enabled = false;
                        //btnMeasure.Text = "Stop measuring";
                        //btnMeasure.Enabled = true;
                        //labelUpdateRate.Enabled = false;
                        //comboBoxUpdateRate.Enabled = false;
                        //btnRecord.Text = "Start recording";
                        //btnRecord.Enabled = true;
                        //labelFilename.Enabled = true;
                        //textBoxFilename.Enabled = true;
                        //labelFlushing.Enabled = false;
                        //progressBarFlushing.Enabled = false;
                        //progressBarFlushing.Value = 0;
                        //pictureBoxStateDiagram.Image = global::awindamonitor.Properties.Resources.measuring;
                    }
                    break;

                case States.RECORDING:
                    {
                        //btnRecord.Enabled = true;
                        //btnRecord.Text = "Stop recording";
                        //labelFilename.Enabled = false;
                        //textBoxFilename.Enabled = false;
                        //btnMeasure.Enabled = false;
                        //pictureBoxStateDiagram.Image = global::awindamonitor.Properties.Resources.recording;
                    }
                    break;

                case States.FLUSHING:
                    {
                        //btnRecord.Enabled = false;
                        //btnRecord.Text = "Flushing";
                        //labelFlushing.Enabled = true;
                        //progressBarFlushing.Enabled = true;
                        //pictureBoxStateDiagram.Image = global::awindamonitor.Properties.Resources.flushing;
                    }
                    break;

                default:
                    break;
            }
        }

        private void Button_Click(object sender, RoutedEventArgs e)
        {
            switch (_content.DeviceState)
            {
                case States.ENABLED:
                case States.OPERATIONAL:
                    {
                        // First set the update rate
                        int desiredUpdateRate = Convert.ToInt32(_content.UpdateRates[_content.SelectedRate]);
                        if (desiredUpdateRate != -1 && desiredUpdateRate != _MyWirelessMasterDevice?.updateRate())
                        {

                            if (_MyWirelessMasterDevice?.setUpdateRate(desiredUpdateRate) == true)
                            {
                                log(string.Format("Update rate set. ID: {0}, Rate: {1}", _MyWirelessMasterDevice?.deviceId().toXsString().toString(), desiredUpdateRate));
                            }
                            else
                            {
                                log(string.Format("Failed to set update rate. ID: {0}, Rate: {1}", _MyWirelessMasterDevice?.deviceId().toXsString().toString(), desiredUpdateRate));
                            }
                        }

                        if (_content.SelectedRate == 0)
                        {
                            log("Note: at the highest update rate\nrecording will be at effective update rate.");
                        }

                        States bkpState = _content.DeviceState;
                        // Set the state to AWAIT_MEASUREMENT_START and go to measurement
                        _content.DeviceState = States.AWAIT_MEASUREMENT_START;



                        if (_MyWirelessMasterDevice?.gotoMeasurement() == true)
                        {
                            log(string.Format("Waiting for measurement start. ID: {0}", _MyWirelessMasterDevice.deviceId().toXsString().toString()));
                        }
                        else
                        {
                            // If gotoMeasurement fails revert the state
                            _content.DeviceState = bkpState;
                        }

                    }
                    break;

                case States.MEASURING:
                    {
                        if (_MyWirelessMasterDevice?.gotoConfig() == true)
                        {
                            //log(string.Format("Stopping measurement. ID: {0}", _MyWirelessMasterDevice.deviceId().toXsString().toString()));
                        }
                        else
                        {
                            log(string.Format("Failed to stop measurement. ID: {0}", _MyWirelessMasterDevice?.deviceId().toXsString().toString()));
                        }
                    }
                    break;
                default:
                    break;

            }
            setWidgetsStates();
        }
    }
}