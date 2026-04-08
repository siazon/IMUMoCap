using IMUMoCap.AHRS;
using IMUMoCap.Methods;
using IMUMoCap.Model;
using IMUMoCap.Services;
using System.Collections.Concurrent;
using System.Data;
using System.Diagnostics;
using System.Net;
using System.Net.NetworkInformation;
using System.Net.Sockets;
using System.Net.WebSockets;
using System.Numerics;
using System.Reflection.Metadata;
using System.Runtime.Intrinsics.X86;
using System.Text;
using System.Text.Json;
using System.Windows;
using System.Windows.Controls;
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
using System.IO;

namespace IMUMoCap
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private MainPageVM _content = new MainPageVM();
        private ImuDeviceManager _deviceManager = null!;
        private ImuSlotRegistry _slotRegistry = null!;
        public WebSocketBroadcastServer? _wsServer;
        ConcurrentQueue<double[]> actionQueue = new ConcurrentQueue<double[]>();
        BlockingCollection<RecoredData> ImuDataQueue = new BlockingCollection<RecoredData>(new ConcurrentQueue<RecoredData>(), 2000);
        string imuPelvis = "imuPelvis", imuL = "imuL", imuR = "imuR";
        private readonly bool _useConjugateForHeading = true; // 你想用哪套就统一哪套
        private const double TrainingStatusThresholdDeg = 10.0;
        private TestState _sessionState = TestState.Launching;

        private volatile bool _stanceL;
        private volatile bool _stanceR;
        private DispatcherTimer _uiTimer;

        // Data recording configuration
        private int _sampleRateHz = 100; // Default 10 Hz
        private readonly List<ImuSampleFrame> _imuSamples = new();
        private readonly ImuFrameCollector _imuFrameCollector = new();

        public MainWindow()
        {
            InitializeComponent();
            this.DataContext = _content;

            _wsServer = new WebSocketBroadcastServer();
            _wsServer.Start(new[] { "http://+:8765/ws/" });
            log("WebSocket server started: ws://192.168.137.1:8765/ws/");
            _wsServer.OnTextMessage += (clientId, text) => HandleWsMessage(clientId, text);
            _wsServer.OnClientConnected += (id, remote) => { };

            var imuList = new List<ImuViewModel>
            {
                new ImuViewModel(imuPelvis) { IMUDodel = ImuVisual,   DeviceId = 0x00B43CAB, Role = ImuRole.Pelvis },
                new ImuViewModel(imuL)      { IMUDodel = ImuVisual1,  DeviceId = 0x10B41904, Role = ImuRole.Left   },
                new ImuViewModel(imuR)      { IMUDodel = ImuVisual12, DeviceId = 0x10b41913, Role = ImuRole.Right  },
            };
            _slotRegistry = new ImuSlotRegistry(imuList);

            _deviceManager = new ImuDeviceManager(_slotRegistry);
            _deviceManager.Log                  += msg => Dispatcher.BeginInvoke(() => log(msg));
            _deviceManager.StateChanged         += s   => Dispatcher.BeginInvoke(() => OnDeviceStateChanged(s));
            _deviceManager.UpdateRatesAvailable += ev  => Dispatcher.BeginInvoke(() => OnUpdateRatesAvailable(ev));
            _deviceManager.MtwConnected         += ev  => Dispatcher.BeginInvoke(() => OnMtwConnected(ev));
            _deviceManager.MtwDisconnected      += ev  => Dispatcher.BeginInvoke(() => OnMtwDisconnected(ev));
            _deviceManager.DataPacketReceived   += ev  => Dispatcher.BeginInvoke(() => OnDataPacket(ev));
            _deviceManager.BatteryLevelChanged  += ev  => Dispatcher.BeginInvoke(() => OnBattery(ev));

            _content.StatusLabel = "Ready to calibration";
            _imuRotTf = new RotateTransform3D(_imuRot);
            _imuFrameCollector.SampleRateHz = _sampleRateHz;
            UpdateImuStatusIndicators();

            StartScanAsync();
        }

        private void StartScanAsync()
        {
            _imuLoopCts = new CancellationTokenSource();
            var token = _imuLoopCts.Token;
            Task.Run(() => _deviceManager.ScanPorts());
            Task.Run(async () =>
            {
                try
                {
                    while (!token.IsCancellationRequested)
                    {
                        var info = await Task.Run(() => ImuDataQueue.Take(token), token);
                    }
                }
                catch (OperationCanceledException) { }
            }, token);
        }

        private void OnDeviceStateChanged(States s)
        {
            _content.DeviceState = s;
            setWidgetsStates();
        }

        private void OnUpdateRatesAvailable(UpdateRatesEvent ev)
        {
            _content.UpdateRates.Clear();
            foreach (var r in ev.Rates) _content.UpdateRates.Add(r);
            _content.SelectedRate = ev.SelectedIndex;
        }

        private void OnMtwConnected(MtwConnectedEvent ev)
        {
            if (_content.ConnectedMtws.IndexOf(ev.DeviceIdStr) >= 0) return;
            _content.ConnectedMtws.Add(ev.DeviceIdStr);
            _content.DeviceModels.Add(new DeviceModel { DeviceName = ev.DeviceIdStr });
            _content.SelectedMtw = _content.ConnectedMtws.Count - 1;

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

            if (ev.Packet.containsOrientation())
            {
                var quat = ev.Packet.orientationQuaternion();
                OnNewImuQuaternion(new Quaternion(quat.x(), quat.y(), quat.z(), quat.w()), ev.Slot);
            }

            string mtwIdStr = ev.Slot.SlotName;
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

            OnXsensData(ev.Slot.Role, ev.DeviceId, ev.Packet);
        }

        private void OnBattery(BatteryEvent ev) { }

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

                    case "Start":
                        Dispatcher.Invoke(() =>
                        {
                            log("WS: start");
                        });
                        break;

                    case "stop":
                    case "Stop":
                        Dispatcher.Invoke(() =>
                        {
                            log("WS: stop");
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

        private CancellationTokenSource? _imuLoopCts;

        /// <summary>
        /// Update IMU status indicators in the 3D viewports
        /// </summary>
        private void UpdateImuStatusIndicators()
        {
            Dispatcher.Invoke(() =>
            {
                foreach (var imu in _slotRegistry.Imus)
                {
                    string statusText;
                    string dotColor;

                    if (imu.IsConnected)
                    {
                        statusText = $"{imu.Role}: Connected";
                        dotColor = "Green";
                    }
                    else
                    {
                        statusText = $"{imu.Role}: Disconnected";
                        dotColor = "Gray";
                    }

                    // Update the corresponding UI elements
                    switch (imu.Role)
                    {
                        case ImuRole.Pelvis:
                            PelvisStatusText.Text = statusText;
                            PelvisStatusDot.Fill = new SolidColorBrush((Color)ColorConverter.ConvertFromString(dotColor));
                            break;
                        case ImuRole.Left:
                            LeftStatusText.Text = statusText;
                            LeftStatusDot.Fill = new SolidColorBrush((Color)ColorConverter.ConvertFromString(dotColor));
                            break;
                        case ImuRole.Right:
                            RightStatusText.Text = statusText;
                            RightStatusDot.Fill = new SolidColorBrush((Color)ColorConverter.ConvertFromString(dotColor));
                            break;
                    }
                }
            });
        }

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

        private void btnClear(object sender, EventArgs e)
        {
            _content.LogList = "";
        }
        float _deltaLeftRad = 0, _deltaRightRad = 0;


        private long _lastLogTicks = 0;
        private static readonly long OneSecondTicks = TimeSpan.FromSeconds(1).Ticks;


        private string getMsgStr(TestState sessionState)
        {
            string msg = "Initializing... Please wait.";
            switch (sessionState)
            {
                case TestState.Launching:
                    break;
                case TestState.Connected:
                    msg = "Please stand upright. Stomp your left foot to start";
                    break;
                case TestState.Calibrating:
                    msg = "1. Static Calibration in progress... Do not move.";
                    break;
                case TestState.Calibrated:
                    msg = "Calibration Successful. Initializing Baseline Assessment.";
                    break;
                case TestState.Baseline:
                    msg = "2. Recording Baseline Gait Data... Walk Naturally.";
                    break;
                case TestState.Step:
                    break;
                default:
                    break;
            }
            return msg;
        }

        long currentPacketId = 0;

        private void BtnMeasure_Click(object sender, RoutedEventArgs e)
        {
            int desiredRate = _content.UpdateRates.Count > 0
                ? Convert.ToInt32(_content.UpdateRates[_content.SelectedRate])
                : -1;
            _deviceManager.StartMeasurement(desiredRate);
        }

        private void BtnTest_Click(object sender, RoutedEventArgs e)
        {

        }

        private void btnCalibration_click(object sender, RoutedEventArgs e)
        {

        }

        private void Button_Restart(object sender, RoutedEventArgs e)
        {

        }

        private void Button_SaveData(object sender, RoutedEventArgs e)
        {
            try
            {
                // Update sample rate from UI
                if (int.TryParse(this.txtSampleRate.Text, out int newSampleRate) && newSampleRate > 0)
                {
                    _sampleRateHz = newSampleRate;
                    _imuFrameCollector.SampleRateHz = _sampleRateHz;
                    log($"Sample rate updated to {_sampleRateHz} Hz");
                }
                else
                {
                    log("Invalid sample rate. Using current rate.");
                }

                if (_imuSamples.Count == 0)
                {
                    log("No data to save.");
                    return;
                }
                var saveFileDialog = new Microsoft.Win32.SaveFileDialog
                {
                    Filter = "CSV files (*.csv)|*.csv|All files (*.*)|*.*",
                    DefaultExt = ".csv",
                    FileName = $"ImuSamples_{DateTime.Now:yyyyMMdd_HHmmss}.csv"
                };

                if (saveFileDialog.ShowDialog() != true)
                    return;

                var csvUtil = new CsvUtil();
                csvUtil.WriteImuSamplesCsv(saveFileDialog.FileName, _imuSamples);
            }
            catch (Exception ex)
            {
                log($"Error saving data: {ex.Message}");
            }
        }

        void OnXsensData(ImuRole sensor, uint deviceId, XsDataPacket packet)
        {
            // One callback packet becomes a single-IMU sample, then a synchronized 3-IMU frame.
            var (sample, _) = _imuFrameCollector.Process(sensor, deviceId, packet);
            _imuSamples.Add(sample);
        }

    }
}
