using GaitTraining.Gait;
using GaitTraining.Imu;
using IMUMoCap.AHRS;
using IMUMoCap.Methods;
using IMUMoCap.Vide;
using System.Collections.Concurrent;
using System.Data;
using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using System.Net.WebSockets;
using System.Numerics;
using System.Reflection.Metadata;
using System.Text;
using System.Text.Json;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Animation;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Windows.Threading;
using XDA;
using static System.Net.Mime.MediaTypeNames;
using Application = System.Windows.Application;
using MeshGeometry3D = System.Windows.Media.Media3D.MeshGeometry3D;
using Quaternion = System.Windows.Media.Media3D.Quaternion;

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
        public WebSocketBroadcastServer? _wsServer;
        ConcurrentQueue<double[]> actionQueue = new ConcurrentQueue<double[]>();
        BlockingCollection<RecoredData> ImuDataQueue = new BlockingCollection<RecoredData>(new ConcurrentQueue<RecoredData>(), 2000);
        string imuPelvis = "imuPelvis", imuL = "imuL", imuR = "imuR";
        private readonly bool _useConjugateForHeading = true; // 你想用哪套就统一哪套
        ImuFrameAggregator aggregator;
        GaitPipeline pipeline;

        private volatile bool _stanceL;
        private volatile bool _stanceR;
        private DispatcherTimer _uiTimer;
        public MainWindow()
        {
            InitializeComponent();
            this.DataContext = _content;

            _wsServer = new WebSocketBroadcastServer();
            _wsServer.Start(new[] { "http://+:8765/ws/" });
            log("WebSocket server started: ws://192.168.137.1:8765/ws/");
           
            _wsServer.OnTextMessage += (clientId, text) =>
            {
                // 注意：这里可能在后台线程
                HandleWsMessage(clientId, text);
            };

            _content.StatusLabel = "Ready to calibration";
            _imuRotTf = new RotateTransform3D(_imuRot);

            BuildAxes(AxesVisual);
            BuildImuBox(ImuVisual);

            BuildAxes(AxesVisual1);
            BuildImuBox(ImuVisual1);


            BuildAxes(AxesVisual12);
            BuildImuBox(ImuVisual12);
            //Imus.Add(new ImuViewModel(imuPelvis) { IMUDodel = ImuVisual, DeviceId = 0x00B43D12, Role = ImuRole.Pelvis });
            Imus.Add(new ImuViewModel(imuPelvis) { IMUDodel = ImuVisual, DeviceId = 0x00B43CAB, Role = ImuRole.Pelvis });
            Imus.Add(new ImuViewModel(imuL) { IMUDodel = ImuVisual1, DeviceId = 0x10B41904, Role = ImuRole.Left });
            Imus.Add(new ImuViewModel(imuR) { IMUDodel = ImuVisual12, DeviceId = 0x10b41913, Role = ImuRole.Right });



            // 你已有的 stance 判定（用 packetId 同步）
            stanceLeftDet = new FootStanceDetector(fs: 100, windowMs: 100, minEnterMs: 80, minExitMs: 40);
            stanceRightDet = new FootStanceDetector(fs: 100, windowMs: 100, minEnterMs: 80, minExitMs: 40);


            StanceDetector detectorL = new StanceDetector();
            StanceDetector detectorR = new StanceDetector();
            QualityGate qualityGate = new QualityGate(new QualityGateConfig() { EnableAll = false });

            aggregator = new ImuFrameAggregator();
            StandingCalibrator calibrator = new StandingCalibrator();


            // 初始化
            pipeline = new GaitPipeline();

            // 步态输出：每步一次，直接打日志
            pipeline.OnStepFpa += result =>
                Dispatcher.Invoke(() =>
                {
                    _content.ProgDirDeg = result.ProgDirDeg;
                    if (result.Foot == ImuRole.Left)
                    {
                        _content.LeftFpaDeg = result.FpaDeg;
                    }
                    else if (result.Foot == ImuRole.Right)
                    {
                        _content.RightFpaDeg = result.FpaDeg;
                    }
                    PushToClient(result);
                    log("Step: " + result.ToString());
                });

            // 标定事件
            pipeline.OnCalibrationStateChanged += state =>
                Dispatcher.Invoke(() =>
                {
                    _content.StatusLabel = state switch
                    {
                        CalibrationState.Waiting => "Stand up straight, ready to begin...",
                        CalibrationState.Collecting => "Data collection in progress. Please remain still.",
                        CalibrationState.Done => "Calibration completed",
                        CalibrationState.Failed => "Calibration failed. Please try again.",
                        _ => ""
                    };
                });

            pipeline.OnCalibrationFailed += (_, msg) =>
                Dispatcher.Invoke(() => log("Calibration failed：" + msg));

            pipeline.OnSanityWarning += (_, msg) =>
                Dispatcher.Invoke(() => log("Calibration Warning：" + msg));

            pipeline.OnCalibrationDone += result =>
                Dispatcher.Invoke(() =>
                {
                    _content.CalibrationState = "Calibrated";
                    log("Calibration completed: " + result.ToString());
                });

            pipeline.OnStanceStatusChanged += status =>
            {
                // Update fields only in the callback thread; do not touch the UI.
                _stanceL = status.LeftInStance;
                _stanceR = status.RightInStance;
            };

            _uiTimer = new DispatcherTimer
            {
                Interval = TimeSpan.FromMilliseconds(100)  // 10Hz
            };
            _uiTimer.Tick += (_, _) =>
            {
                _content.StanceSummary = $"L:{_stanceL} R:{_stanceR}";
            };
            _uiTimer.Start();



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
        protected override void OnClosed(EventArgs e)
        {
            _imuLoopCts?.Cancel();
            ImuDataQueue?.CompleteAdding();
            // stop websocket server
            if (_wsServer != null)
            {
                try { _wsServer.StopAsync().GetAwaiter().GetResult(); } catch { }
            }

            base.OnClosed(e);
        }

        private void HandleWsMessage(Guid clientId, string text)
        {
            try
            {
                using var doc = JsonDocument.Parse(text);
                var root = doc.RootElement;

                // 约定：客户端发送 {"cmd":"...", ...}
                if (!root.TryGetProperty("cmd", out var cmdEl)) return;
                var cmd = cmdEl.GetString() ?? "";

                switch (cmd)
                {
                    case "ping":
                        _ = _wsServer?.BroadcastJsonAsync(new { type = "pong", t = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() });
                        break;

                    case "setTargetFpa":
                        // {"cmd":"setTargetFpa","foot":"Left","target":10.0}
                        var foot = root.TryGetProperty("foot", out var fEl) ? fEl.GetString() : "Both";
                        var target = root.TryGetProperty("target", out var tEl) ? tEl.GetDouble() : 0.0;

                        Dispatcher.Invoke(() =>
                        {
                            // TODO：这里改成你自己的目标值变量/绑定字段
                            // _content.TargetLeftFpa = target; ...
                            log($"WS: setTargetFpa foot={foot} target={target}");
                        });
                        break;

                    case "start":
                        Dispatcher.Invoke(() =>
                        {
                            // TODO：调用你现有的开始训练/开始采集
                            log("WS: start");
                        });
                        break;

                    case "stop":
                        Dispatcher.Invoke(() =>
                        {
                            // TODO：停止
                            log("WS: stop");
                        });
                        break;

                    case "calibrate":
                        Dispatcher.Invoke(() =>
                        {
                            // TODO：触发你的校准逻辑，例如 pipeline.BeginCalibration() / StartCalibration()
                            log("WS: calibrate");
                        });
                        break;

                    default:
                        Dispatcher.Invoke(() => log($"WS: unknown cmd={cmd}, raw={text}"));
                        break;
                }
            }
            catch (Exception ex)
            {
                Dispatcher.Invoke(() => log($"WS parse error: {ex.Message}, raw={text}"));
            }
        }

        private void PushToClient(GaitTraining.Gait.StepFpaResult result)
        {
            bool isStance = true;

            // status 你要的是一个字符串：这里我用当前 StatusLabel（你也可换成 CalibrationState 等）
            string status = Math.Abs(result.FpaDeg) > 15 ? "Red" : "Green";

            // 发送给 AR 眼镜的 payload（只推“结果”，不推原始IMU）
            var payload = new
            {
                Foot = result.Foot.ToString(),     // "Left"/"Right"/...
                FpaDegree = result.FpaDeg,         // double
                IsStance = isStance,               // bool
                status = status                    // string
            };

            // 广播（后台线程，不阻塞 UI）
            _ = _wsServer?.BroadcastJsonAsync(payload);

        }

        QuaternionHelper quaternionHelper = new QuaternionHelper();
        private CancellationTokenSource? _imuLoopCts;
        private void InitDevice()
        {
            Task.Run(() =>
            {
                if (_content.DeviceState != States.MEASURING && _content.DeviceState != States.AWAIT_MEASUREMENT_START && _content.DeviceState != States.RECORDING && _content.DeviceState != States.FLUSHING)
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

            _imuLoopCts = new CancellationTokenSource();
            var token = _imuLoopCts.Token;

            Task.Run(() =>
            {
                try
                {
                    while (true)
                    {
                        var info = ImuDataQueue.Take(token);


                    }
                }
                catch (OperationCanceledException)
                {
                    // Expected on shutdown.
                }
            }, token);
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

                            // --- Station 专用：确保进入 operational state（文档：XsDevice::makeOperational） ---
                            var did = e.PortInfo.deviceId();

                            // 这两个方法在你的 SDK 文档里都有：isAwindaXStation / isAwinda2Station
                            bool isStation = did.isAwindaXStation() || did.isAwinda2Station();

                            if (isStation)
                            {
                                // 建议：先回到 config 再 makeOperational（更稳）
                                // 如果 device 已在测量，直接 enableRadio 可能会失败或无效
                                if (_MyWirelessMasterDevice.deviceState() != XsDeviceState.XDS_Config)
                                    _MyWirelessMasterDevice.gotoConfig();

                                // 关键：Station 置 operational
                                bool ok = _MyWirelessMasterDevice.makeOperational();
                                log($"makeOperational() => {ok}");
                            }

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
                if (index >= 0)
                {
                    // Found --> delete
                    _content.ConnectedMtws.Remove(mtwIdStr);
                    _connectedMtwData.Remove(e.DeviceId.legacyDeviceId());
                    _content.SelectedMtw = _content.ConnectedMtws.Count - 1;
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
        //e.Packet.calibratedData：当前时刻的瞬时测量值
        //SdiData：在上一个采样间隔内已经积分好的增量
        //calibratedAcceleration，calibratedGyroscopeData，calibratedMagneticField 分开取的数据
        //correctedMagneticField通过了ICC(In-Run Compass Calibration)/representative motion的数据。
        /* ICC方法
         1. gotoConfig()
         2. setDeviceOptionFlags(XDOF_EnableInrunCompassCalibration, XDOF_None)
         3. 配置输出：至少包含 XDI_MagneticFieldCorrected（你也可以同时要 Acc/Gyro）
         4. gotoMeasurement()
         5. 用户开始做一段“代表性运动”前：startRepresentativeMotion()
         6. 做完后：result = stopRepresentativeMotion()
         7. 如果希望设备记住这次校正：storeIccResults()
         8. 之后在实时 XsDataPacket 里读 correctedMagneticField()（或 COM 对应的 XsDataPacket_correctedMagneticField）
         */
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
                currentPacketId = e.Packet.packetId();

                if (!e.Packet.containsSdiData())
                {
                    log(string.Format("Packet received of an MTw {0} not containing data.", mtwIdStr));
                    return;
                }
                if (e.Packet.containsCalibratedAcceleration())
                {
                    var acc = e.Packet.calibratedAcceleration;
                }
                if (e.Packet.containsCalibratedGyroscopeData())
                {
                    var gyro = e.Packet.calibratedGyroscopeData;
                }
                if (e.Packet.containsCalibratedMagneticField())
                {
                    var mag = e.Packet.calibratedMagneticField;
                }
                if (e.Packet.containsCorrectedMagneticField())
                {
                    var mag = e.Packet.correctedMagneticField;
                }
                // Getting SDI data.
                XsSdiData sdiData = e.Packet.sdiData();

                uint deviceId = e.Device.deviceId().legacyDeviceId();
                ImuViewModel imuViewModel = GetOrAssignSlot(deviceId);

                string devices = imuViewModel.SlotName;

                _connectedMtwData[deviceId].XsQuaternion = sdiData.orientationIncrement(); //xsQuaternion;

                _connectedMtwData[deviceId]._rssi = e.Packet.rssi();

                OnXsensData(imuViewModel.Role, e.Packet);
                if (e.Packet.containsOrientation())
                {
                    var quat = e.Packet.orientationQuaternion();

                    OnNewImuQuaternion(new Quaternion(quat.x(), quat.y(), quat.z(), quat.w()), imuViewModel);
                    //Getting Euler angles.
                    XsEuler oriEuler = e.Packet.orientationEuler();


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
                _connectedMtwData[deviceId]._effectiveUpdateRate = (int)(100 * (1 - (float)_connectedMtwData[deviceId]._sumFrameSkips / (float)(_connectedMtwData[deviceId]._frameSkipsList.Count() +
                _connectedMtwData[deviceId]._sumFrameSkips)));

                while (_connectedMtwData[deviceId]._frameSkipsList.Count() + _connectedMtwData[deviceId]._sumFrameSkips > 99 && _connectedMtwData[deviceId]._frameSkipsList.Count() > 0)
                {
                    _connectedMtwData[deviceId]._sumFrameSkips = _connectedMtwData[deviceId]._sumFrameSkips - (uint)_connectedMtwData[deviceId]._frameSkipsList[0];
                    _connectedMtwData[deviceId]._frameSkipsList.RemoveAt(0);
                }

                ConnectedMTwData mtwData = _connectedMtwData[deviceId];


                if (_content.SelectedMtw >= 0 && _content.SelectedMtw < _content.ConnectedMtws.Count && _content.ConnectedMtws[_content.SelectedMtw] == mtwIdStr)
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
        uint pelvisId = 0x00B43D12, leftFootId = 0x10B41904, rightFootId = 0x10B41913;
        public ImuViewModel GetOrAssignSlot(uint deviceId)
        {
            if (imuDevicesMap.TryGetValue(deviceId, out var vm))
                return vm;

            ImuViewModel imuVM = Imus.FirstOrDefault(a => a.DeviceId == deviceId);
            if (imuVM == null)
                throw new Exception($"Unrecognized deviceId {deviceId:X}. Please check your device IDs and update the code accordingly.");

            imuVM.BindDevice(deviceId);
            imuDevicesMap[deviceId] = imuVM;
            return imuVM;
        }

        List<ImuViewModel> Imus = new List<ImuViewModel>();
        Dictionary<uint, ImuViewModel> imuDevicesMap = new Dictionary<uint, ImuViewModel>();

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
        private readonly QuaternionRotation3D _imuRot = new QuaternionRotation3D(System.Windows.Media.Media3D.Quaternion.Identity);
        private readonly RotateTransform3D _imuRotTf;

        #region 3D Rendering
        /// <summary>
        /// 你在 IMU 回调里，把最新的 orientationQuats 传进来调用这个方法即可
        /// </summary>
        public void OnNewImuQuaternion(Quaternion qImu, ImuViewModel Imu3D)
        {
            // 重要：WPF Quaternion 构造/存储顺序是 (X,Y,Z,W)
            // 如果你的 orientationQuats 是 (w,x,y,z)，你要自己调换成：
            // qImu = new Quaternion(x, y, z, w);

            qImu.Normalize();

            // 如果你发现方向整体反了/像镜像，常用修正是取共轭（相当于 inverse）
            // qImu = qImu.Conjugate();

            // UI线程更新
            Dispatcher.BeginInvoke(() =>
            {
                //Imu3D.Transform = new IMUUIUpdater().CreateTransform(qImu, applyConjugate: true); // 你之前验证 Conjugate 会更接近正确，所以先保持 true
                // 1) 原本的姿态 transform（你已有）
                var baseTf = new IMUUIUpdater().CreateTransform(qImu, applyConjugate: true);

                // 2) 选择对应的标定 yaw 修正角（弧度->度）
                float deltaRad = Imu3D.Role switch
                {
                    ImuRole.Left => _deltaLeftRad,
                    ImuRole.Right => _deltaRightRad,
                    _ => 0f
                };
                double deltaDeg = deltaRad * 180.0 / Math.PI;

                // 3) 绕 UI 的 Up 轴做一个 yaw 修正旋转
                //    假设 UI: Y up
                var yawFix = new RotateTransform3D(
                    new AxisAngleRotation3D(new Vector3D(0, 1, 0), deltaDeg));

                // 4) 合成 transform：base + yawFix
                var group = new Transform3DGroup();
                group.Children.Add(baseTf);
                group.Children.Add(yawFix);

                Imu3D.IMUDodel.Transform = group;
            });
        }



        private void BuildImuBox(ModelVisual3D IMUmodelVisual3D)
        {
            // 盒子尺寸（随便设个比例：X前、Y上、Z侧）
            double lx = 0.30;
            double ly = 0.08;
            double lz = 0.18;

            var mesh = CreateBoxMesh(lx, ly, lz);

            var mat = new DiffuseMaterial(new SolidColorBrush(Color.FromRgb(210, 210, 210)));
            var backMat = new DiffuseMaterial(new SolidColorBrush(Color.FromRgb(190, 190, 190)));

            var model = new GeometryModel3D
            {
                Geometry = mesh,
                Material = mat,
                BackMaterial = backMat
            };

            // 给盒子加一个“前向标记”（小三角/小杆），方便你判断X轴朝向
            var forward = new GeometryModel3D
            {
                Geometry = CreateArrowMesh(), // 一个小箭头
                Material = new DiffuseMaterial(Brushes.Orange),
                BackMaterial = new DiffuseMaterial(Brushes.Orange)
            };

            var group = new Model3DGroup();
            group.Children.Add(model);
            group.Children.Add(forward);

            var mv = new ModelVisual3D
            {
                Content = group,
                Transform = _imuRotTf
            };

            IMUmodelVisual3D.Children.Add(mv);
        }

        private void BuildAxes(ModelVisual3D modelVisual3D)
        {
            double len = 1.0;        // 轴长度
            double t = 0.0035;       // 轴粗细（改这个！越小越细）

            var gx = new GeometryModel3D
            {
                Geometry = CreateBoxMesh(len, t, t, new Point3D(len / 2, 0, 0)),
                Material = new DiffuseMaterial(Brushes.Red),
                BackMaterial = new DiffuseMaterial(Brushes.Red)
            };

            var gy = new GeometryModel3D
            {
                Geometry = CreateBoxMesh(t, len, t, new Point3D(0, len / 2, 0)),
                Material = new DiffuseMaterial(Brushes.LimeGreen),
                BackMaterial = new DiffuseMaterial(Brushes.LimeGreen)
            };

            var gz = new GeometryModel3D
            {
                Geometry = CreateBoxMesh(t, t, len, new Point3D(0, 0, len / 2)),
                Material = new DiffuseMaterial(Brushes.DodgerBlue),
                BackMaterial = new DiffuseMaterial(Brushes.DodgerBlue)
            };

            var group = new Model3DGroup();
            group.Children.Add(gx);
            group.Children.Add(gy);
            group.Children.Add(gz);

            modelVisual3D.Children.Add(new ModelVisual3D { Content = group });
        }

        // ------------------------------
        // Mesh helpers
        // ------------------------------

        private System.Windows.Media.Media3D.MeshGeometry3D CreateBoxMesh(double lx, double ly, double lz, Point3D? center = null)
        {
            var c = center ?? new Point3D(0, 0, 0);
            double x0 = c.X - lx / 2, x1 = c.X + lx / 2;
            double y0 = c.Y - ly / 2, y1 = c.Y + ly / 2;
            double z0 = c.Z - lz / 2, z1 = c.Z + lz / 2;

            var mesh = new MeshGeometry3D();

            // 8 vertices
            var p000 = new Point3D(x0, y0, z0);
            var p001 = new Point3D(x0, y0, z1);
            var p010 = new Point3D(x0, y1, z0);
            var p011 = new Point3D(x0, y1, z1);
            var p100 = new Point3D(x1, y0, z0);
            var p101 = new Point3D(x1, y0, z1);
            var p110 = new Point3D(x1, y1, z0);
            var p111 = new Point3D(x1, y1, z1);

            // Add 6 faces (each face: 2 triangles). We duplicate vertices per face for correct normals.
            AddFace(mesh, p101, p100, p110, p111); // +X
            AddFace(mesh, p000, p001, p011, p010); // -X
            AddFace(mesh, p010, p011, p111, p110); // +Y
            AddFace(mesh, p100, p101, p001, p000); // -Y
            AddFace(mesh, p001, p101, p111, p011); // +Z
            AddFace(mesh, p100, p000, p010, p110); // -Z

            return mesh;
        }

        private void AddFace(MeshGeometry3D mesh, Point3D p0, Point3D p1, Point3D p2, Point3D p3)
        {
            int i0 = mesh.Positions.Count;
            mesh.Positions.Add(p0);
            mesh.Positions.Add(p1);
            mesh.Positions.Add(p2);
            mesh.Positions.Add(p3);

            // two triangles
            mesh.TriangleIndices.Add(i0);
            mesh.TriangleIndices.Add(i0 + 1);
            mesh.TriangleIndices.Add(i0 + 2);

            mesh.TriangleIndices.Add(i0);
            mesh.TriangleIndices.Add(i0 + 2);
            mesh.TriangleIndices.Add(i0 + 3);

            // simple normal (face normal)
            Vector3D n = Vector3D.CrossProduct(p1 - p0, p2 - p0);
            n.Normalize();
            mesh.Normals.Add(n);
            mesh.Normals.Add(n);
            mesh.Normals.Add(n);
            mesh.Normals.Add(n);
        }

        private MeshGeometry3D CreateArrowMesh()
        {
            // 一个很简单的小“前向箭头”：沿 +X 方向放一个小三角楔子
            // 放在盒子前端附近：x≈+0.18，y=0，z=0
            var mesh = new MeshGeometry3D();

            var p0 = new Point3D(0.18, 0.00, 0.00); // tip
            var p1 = new Point3D(0.10, 0.03, 0.03);
            var p2 = new Point3D(0.10, -0.03, 0.03);
            var p3 = new Point3D(0.10, -0.03, -0.03);
            var p4 = new Point3D(0.10, 0.03, -0.03);

            // 4 side faces around tip (triangles)
            AddTri(mesh, p0, p1, p2);
            AddTri(mesh, p0, p2, p3);
            AddTri(mesh, p0, p3, p4);
            AddTri(mesh, p0, p4, p1);

            // base (two triangles)
            AddTri(mesh, p1, p4, p3);
            AddTri(mesh, p1, p3, p2);

            return mesh;
        }

        private void AddTri(MeshGeometry3D mesh, Point3D a, Point3D b, Point3D c)
        {
            int i0 = mesh.Positions.Count;
            mesh.Positions.Add(a);
            mesh.Positions.Add(b);
            mesh.Positions.Add(c);

            mesh.TriangleIndices.Add(i0);
            mesh.TriangleIndices.Add(i0 + 1);
            mesh.TriangleIndices.Add(i0 + 2);

            Vector3D n = Vector3D.CrossProduct(b - a, c - a);
            if (n.Length > 1e-9) n.Normalize();
            mesh.Normals.Add(n);
            mesh.Normals.Add(n);
            mesh.Normals.Add(n);
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
                            //ICC
                            //_MyWirelessMasterDevice.startRepresentativeMotion();
                            //var result = _MyWirelessMasterDevice.stopRepresentativeMotion();
                            //_MyWirelessMasterDevice.storeIccResults();
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
                            //ICC
                            //_MyWirelessMasterDevice.setDeviceOptionFlags(XsDeviceOptionFlag.XDOF_EnableInrunCompassCalibration, XsDeviceOptionFlag.XDOF_None);
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
        private void btnClear(object sender, EventArgs e)
        {
            _content.LogList = "";
        }
        float _deltaLeftRad = 0, _deltaRightRad = 0;
        private void Button_Click_1(object sender, RoutedEventArgs e)
        {
            Task.Run(() =>
            {
                for (int i = 0; i < 1000; i++)
                {
                    float fpa = new Random().Next(-45, 45);
                    GaitTraining.Gait.StepFpaResult result = new GaitTraining.Gait.StepFpaResult() { Foot = i % 2 == 0 ? ImuRole.Left : ImuRole.Right, FpaDeg = fpa, };
                    PushToClient(result);

                    Thread.Sleep(500);
                }

            });

            //new CvsUntil().WriteCVS("D:\\SourceCode\\IMUData\\", "IMUDataNewR.csv", datas);
        }
        public void Test()
        {

        }
        FootStanceDetector stanceLeftDet, stanceRightDet;



        private long _lastLogTicks = 0;
        private static readonly long OneSecondTicks = TimeSpan.FromSeconds(1).Ticks;



        public void TestStance()
        {
            var det = new ImuGaitEventDetector
            {
                Fs = 100.0,
                MinStepIntervalSec = 0.35,

                // 你的新数据有 FreeAcc_*，HS 阈值建议先用 1.0~2.0 m/s^2 之间试
                HSThreshold = 1.2,

                TO_SearchStartSec = 0.10,
                PitchRateThreshold = 1.5
            };

            var samples = det.LoadCsv("D:\\SourceCode\\IMUData\\IMUData.csv");
            var evs = det.Detect(samples);

            log($"Samples: {samples.Count}");
            log($"HeelStrikes (HS): {evs.HeelStrikes.Count}  => StepCount≈{evs.StepCount}");
            log($"ToeOffs (TO): {evs.ToeOffs.Count}");

            for (int i = 0; i < evs.HeelStrikes.Count; i++)
            {
                int hs = evs.HeelStrikes[i];
                log($"HS[{i}_{hs}] t={samples[hs].T:F3}s dynAccMag={samples[hs].DynAccMag:F3}");

                if (i < evs.ToeOffs.Count)
                {
                    int to = evs.ToeOffs[i];
                    log($"  TO[{i}_{hs}] t={samples[to].T:F3}s pitchRate={samples[to].PitchRate:F3} rad/s");
                }
            }
        }


        long currentPacketId = 0;
        private void btnCalibration_click(object sender, RoutedEventArgs e)
        {
            _content.CalibrationState = "Calibrating";
            pipeline.BeginCalibration();
        }

        List<RecoredData> datas = new List<RecoredData>();
        // 在 Xsens 的 OnDataAvailable 回调中
        void OnXsensData(ImuRole sensor, XsDataPacket packet)
        {
            var rawData = new RecoredData()
            {
                PackageId = packet.packetId().ToString(),
                quaternion = Utils.ToNumericsQuaternion(packet.orientationQuaternion()),
                Accelerate = Utils.ToNumericsVector3(packet.calibratedAcceleration()),
                Orientation = Utils.ToNumericsVector3(packet.calibratedGyroscopeData()),
                MadgwickAHRS = Utils.ToNumericsVector3(packet.calibratedMagneticField())
            };
            datas.Add(rawData);

            var raw = new ImuRawData(
                Utils.ToNumericsQuaternion(packet.orientationQuaternion()),
                Utils.ToNumericsVector3(packet.calibratedAcceleration()),
                Utils.ToNumericsVector3(packet.calibratedGyroscopeData()),
                Utils.ToNumericsVector3(packet.calibratedMagneticField()));
            pipeline.OnImuData(sensor, packet.packetId(), raw);

            //aggregator.OnImuData(sensor, packet.packetId(), raw);
        }


    }
}
