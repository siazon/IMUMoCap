using IMUMoCap.AHRS;
using IMUMoCap.Methods;
using IMUMoCap.Model;
using IMUMoCap.Pipeline;
using IMUMoCap.Pipeline.Models;
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
        private readonly GaitPipeline _pipeline = new();
        public WebSocketBroadcastServer? _wsServer;
        ConcurrentQueue<double[]> actionQueue = new ConcurrentQueue<double[]>();
        BlockingCollection<RecordedData> ImuDataQueue = new BlockingCollection<RecordedData>(new ConcurrentQueue<RecordedData>(), 2000);
        string imuPelvis = "imuPelvis", imuL = "imuL", imuR = "imuR";
        private readonly bool _useConjugateForHeading = true; // 你想用哪套就统一哪套
        private const double TrainingStatusThresholdDeg = 10.0;
        private TestState _sessionState = TestState.Launching;

        private volatile bool _stanceL;
        private volatile bool _stanceR;
        private DispatcherTimer _uiTimer;
        private volatile bool _isReplaying = false;

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
                new ImuViewModel(imuPelvis) { ImuVisual = ImuVisual,   DeviceId = 0x00B43D0B, Role = ImuRole.Pelvis },
                new ImuViewModel(imuL)      { ImuVisual = ImuVisual1,  DeviceId = 0x10B41904, Role = ImuRole.Left   },
                new ImuViewModel(imuR)      { ImuVisual = ImuVisual12, DeviceId = 0x10b41913, Role = ImuRole.Right  },
            };
            _slotRegistry = new ImuSlotRegistry(imuList);

            _deviceManager = new ImuDeviceManager(_slotRegistry);
            _deviceManager.Log += msg => Dispatcher.BeginInvoke(() => log(msg));
            _deviceManager.StateChanged += s => Dispatcher.BeginInvoke(() => OnDeviceStateChanged(s));
            _deviceManager.UpdateRatesAvailable += ev => Dispatcher.BeginInvoke(() => OnUpdateRatesAvailable(ev));
            _deviceManager.MtwConnected += ev => Dispatcher.BeginInvoke(() => OnMtwConnected(ev));
            _deviceManager.MtwDisconnected += ev => Dispatcher.BeginInvoke(() => OnMtwDisconnected(ev));
            _deviceManager.DataPacketReceived += ev => Dispatcher.BeginInvoke(() => OnDataPacket(ev));
            _deviceManager.BatteryLevelChanged += ev => Dispatcher.BeginInvoke(() => OnBattery(ev));

            _content.StatusLabel = "Ready to calibration";
            _imuRotTf = new RotateTransform3D(_imuRot);
            _imuFrameCollector.SampleRateHz = _sampleRateHz;
            UpdateImuStatusIndicators();

            // 流水线事件订阅
            _pipeline.OnLog += msg => Dispatcher.BeginInvoke(() => log(msg));
            _pipeline.OnCalibrationStateChanged += s =>
            {
                if (!_isReplaying) Dispatcher.BeginInvoke(() => OnCalibrationStateChanged(s));
            };
            _pipeline.OnBaselineProgress += (l, r) => Dispatcher.BeginInvoke(() =>
                log($"Baseline: L={l} steps, R={r} steps"));
            _pipeline.OnBaselineCompleted += profile =>
            {
                if (!_isReplaying) Dispatcher.BeginInvoke(() => HandleBaselineCompleted(profile));
            };
            _pipeline.OnFpaResult += (result, isTraining) => Dispatcher.BeginInvoke(() => OnFpaResult(result, isTraining));

            // 参数同步：VM 属性变化时推入 pipeline
            _content.PropertyChanged += (_, e) => SyncParamToPipeline(e.PropertyName);

            StartScanAsync();
        }

        private void StartScanAsync()
        {
            Task.Run(() => _deviceManager.ScanPorts());
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
            foreach (var item in _slotRegistry.Imus)
            {
                if (item.DeviceId == ev.DeviceId)
                {
                    item.IsConnected = true;
                }
            }
            var vm = _slotRegistry.Imus.FirstOrDefault(a => a.DeviceId == ev.DeviceId);
            if (vm != null) vm.IsConnected = true;


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
            var snap = _deviceManager.GetMtwDataSnapshot(ev.DeviceId);
            if (snap == null) return;

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
                _content.XsTime = $"{mtwIdStr} | {snap.Orientation.x().RoundTwo()}, " +
                                  $"{snap.Orientation.y().RoundTwo()}, {snap.Orientation.z().RoundTwo()}";

                if (_content.RotationByDegree)
                    actionQueue.Enqueue([
                        snap.Orientation.x().DegreesToRadians(),
                        snap.Orientation.y().DegreesToRadians(),
                        snap.Orientation.z().DegreesToRadians(), 1]);
                else
                    actionQueue.Enqueue([
                        snap.Quaternion.x(), snap.Quaternion.y(),
                        snap.Quaternion.z(), snap.Quaternion.w()]);
            }

            OnXsensData(ev.Slot.Role, ev.DeviceId, ev.Packet);
        }

        private void OnBattery(BatteryEvent ev)
        {
            // Battery UI display not implemented; extend here to show battery level per slot.
        }

        protected override void OnClosed(EventArgs e)
        {
            _imuLoopCts?.Cancel();
            _baselineTimer?.Stop();
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
                        //_ = _wsServer?.BroadcastJsonAsync(new { type = "pong", t = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() });
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

                if (_content.IsScrollerToEnd)
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
            _pipeline.ProcessPacket(sensor, deviceId, packet);
        }

        // ── 流水线状态机 ──────────────────────────────────────────────────────

        private DispatcherTimer? _baselineTimer;

        private void BroadcastArState(string state)
        {
            if (_wsServer == null) return;
            _ = _wsServer.BroadcastJsonAsync(new { type = "state", state });
        }

        private void OnCalibrationStateChanged(CalibrationState state)
        {
            switch (state)
            {
                case CalibrationState.WaitingForStart:
                    BroadcastArState("waiting");
                    break;

                case CalibrationState.CollectingStaticPose:
                    _content.StatusLabel = "Calibrating... Stand still.";
                    _sessionState = TestState.Calibrating;
                    BroadcastArState("calibrating");
                    break;

                case CalibrationState.Completed:
                    _content.StatusLabel = "Calibration done. Starting baseline walk.";
                    _content.CalibrationState = "Calibrated";
                    _sessionState = TestState.Calibrated;
                    _pipeline.StartBaseline();
                    BroadcastArState("baseline");
                    _sessionState = TestState.Baseline;
                    _content.StatusLabel = "Baseline: Walk until both feet reach 20 steps.";
                    break;

                case CalibrationState.Failed:
                    _content.StatusLabel = "Calibration failed. Please restart the application.";
                    _sessionState = TestState.Launching;
                    BroadcastArState("error");
                    break;
            }
        }

        private void HandleBaselineCompleted(BaselineProfile? profile)
        {
            _baselineTimer?.Stop();
            if (profile == null || !profile.IsValid)
            {
                string detail = profile == null
                    ? "insufficient valid steps"
                    : $"L={profile.ValidSteps_L} R={profile.ValidSteps_R} steps (need ≥{profile.MinRequiredSteps} each)";
                _content.StatusLabel = $"Baseline insufficient ({detail}). Please restart.";
                BroadcastArState("error");
                return;
            }
            _sessionState = TestState.Step;
            _content.StatusLabel =
                $"Training started. Target L={profile.Target_L:F1}° ({profile.Direction_L})  " +
                $"R={profile.Target_R:F1}° ({profile.Direction_R})";
            _content.LeftFpaTarget = $"Target {profile.Target_L:F1}° ±{profile.Tolerance_L:F1}° ({profile.Direction_L})";
            _content.RightFpaTarget = $"Target {profile.Target_R:F1}° ±{profile.Tolerance_R:F1}° ({profile.Direction_R})";
            _pipeline.StartTraining();
            BroadcastArState("training");
        }

        // ── 参数同步 ──────────────────────────────────────────────────────────

        private void SyncParamToPipeline(string? propName)
        {
            switch (propName)
            {
                case nameof(MainPageVM.StompThreshold):
                    _pipeline.Params.StompThreshold_ms2 = _content.StompThreshold; break;
                case nameof(MainPageVM.StaticGyroThreshold):
                    _pipeline.Params.StaticGyroThreshold = _content.StaticGyroThreshold; break;
                case nameof(MainPageVM.StanceFreeAccThreshold):
                    _pipeline.Params.StanceFreeAccThreshold = _content.StanceFreeAccThreshold; break;
                case nameof(MainPageVM.StanceGyroThreshold):
                    _pipeline.Params.StanceGyroThreshold = _content.StanceGyroThreshold; break;
                case nameof(MainPageVM.PdConfidenceThreshold):
                    _pipeline.Params.PdConfidenceThreshold = _content.PdConfidenceThreshold; break;
                case nameof(MainPageVM.PdStabilityThreshold):
                    _pipeline.Params.PdStabilityThreshold = _content.PdStabilityThreshold; break;
                case nameof(MainPageVM.MinBaselineSteps):
                    _pipeline.Params.MinBaselineSteps = _content.MinBaselineSteps; break;
            }
        }

        // ── 新增按钮处理器 ────────────────────────────────────────────────────

        private void BtnRestartCal_Click(object sender, RoutedEventArgs e)
        {
            _baselineTimer?.Stop();
            _baselineTimer = null;
            _pipeline.Reset();
            _content.StatusLabel = "Calibration reset. Stomp left foot to begin.";
            _content.LeftFpaTarget = "";
            _content.RightFpaTarget = "";
            _content.LeftFpaNote = "";
            _content.RightFpaNote = "";
            _content.LeftFpaBackground = MainPageVM.DefaultCardBg;
            _content.RightFpaBackground = MainPageVM.DefaultCardBg;
            _sessionState = TestState.Launching;
            BroadcastArState("waiting");
            log("Calibration restarted by user.");
        }

        private void BtnSaveDiagnostics_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                var rows = _pipeline.GetDiagnosticsSnapshot();
                if (rows.Count == 0)
                {
                    log("No diagnostics data to save.");
                    return;
                }

                var dlg = new Microsoft.Win32.SaveFileDialog
                {
                    Filter = "CSV files (*.csv)|*.csv|All files (*.*)|*.*",
                    DefaultExt = ".csv",
                    FileName = $"Diagnostics_{DateTime.Now:yyyyMMdd_HHmmss}.csv",
                };
                if (dlg.ShowDialog() != true) return;

                using var sw = new System.IO.StreamWriter(dlg.FileName, append: false,
                    encoding: System.Text.Encoding.UTF8);
                sw.WriteLine(Pipeline.Models.DiagnosticsRow.CsvHeader);
                foreach (var row in rows)
                    sw.WriteLine(row.ToCsvRow());

                log($"Diagnostics saved: {rows.Count} rows → {dlg.FileName}");
            }
            catch (Exception ex)
            {
                log($"Error saving diagnostics: {ex.Message}");
            }
        }

        private async void BtnLoadData_Click(object sender, RoutedEventArgs e)
        {

            Console.WriteLine("BtnLoadData_Click: ");

            var dlg = new Microsoft.Win32.OpenFileDialog
            {
                Title = "Load IMU Samples CSV",
                Filter = "IMU Samples CSV|ImuSamples_*.csv;*.csv",
                DefaultExt = ".csv",
            };
            if (dlg.ShowDialog() != true) return;

            BtnLoadData.IsEnabled = false;
            log("Loading...");

            // Parse CSV on background thread to avoid blocking the UI.
            var (samples, bundles) = await Task.Run(() =>
            {
                var s = new Methods.CsvUtil().ReadImuSamplesCsv(dlg.FileName);
                var b = Methods.CsvUtil.GroupIntoBundles(s);
                return (s, b);
            });

            if (samples.Count == 0)
            {
                log("Load failed: no samples found.");
                BtnLoadData.IsEnabled = true;
                return;
            }

            int completeBundles = bundles.Count(b => b.IsComplete);
            log($"Loaded {samples.Count} samples → {bundles.Count} bundles ({completeBundles} complete). Replaying...");

            _isReplaying = true;
            _pipeline.Reset();
            _sessionState = TestState.Launching;

            void onCalChanged(CalibrationState s)
            {
                if (s != CalibrationState.Completed) return;
                _pipeline.StartBaseline();
                Dispatcher.BeginInvoke(() =>
                {
                    _sessionState = TestState.Baseline;
                    log("Replay: calibration done — baseline started.");
                });
            }
            void onBaselineDone(BaselineProfile? profile)
            {
                if (profile?.IsValid != true) return;
                _pipeline.StartTraining();
                Dispatcher.BeginInvoke(() =>
                {
                    _sessionState = TestState.Step;
                    _content.LeftFpaTarget = $"Target {profile.Target_L:F1}° ±{profile.Tolerance_L:F1}° ({profile.Direction_L})";
                    _content.RightFpaTarget = $"Target {profile.Target_R:F1}° ±{profile.Tolerance_R:F1}° ({profile.Direction_R})";
                    _content.StatusLabel =
                        $"Training started. Target L={profile.Target_L:F1}° ({profile.Direction_L})  " +
                        $"R={profile.Target_R:F1}° ({profile.Direction_R})";
                    log("Replay: baseline done — training started.");
                });
            }

            _pipeline.OnCalibrationStateChanged += onCalChanged;
            _pipeline.OnBaselineCompleted += onBaselineDone;

            try
            {
                await Task.Run(async () =>
                {
                    foreach (var bundle in bundles)
                    {
                        _pipeline.Process(bundle);
                        await Task.Delay(10);
                    }
                });
            }
            finally
            {
                _pipeline.OnCalibrationStateChanged -= onCalChanged;
                _pipeline.OnBaselineCompleted -= onBaselineDone;
                _isReplaying = false;
                BtnLoadData.IsEnabled = true;
            }

            if (_pipeline.InBaseline)
            {
                var profile = _pipeline.FinalizeBaseline();
                if (profile?.IsValid == true)
                {
                    _pipeline.StartTraining();
                    _sessionState = TestState.Step;
                }
            }

            log(BuildReplayTimeline(
                bundles,
                _pipeline.GetDiagnosticsSnapshot(),
                _pipeline.Params.StompThreshold_ms2,
                _pipeline.Params.MinBaselineSteps));
            log("Replay done. Save diagnostics to inspect results.");
        }

        private string BuildReplayTimeline(
            List<ImuFrameBundle> bundles,
            List<DiagnosticsRow> diag,
            float stompThreshold,
            int minBaselineSteps)
        {
            const uint XSF_ORIENT = 0x02;
            var events = new List<(string pid, string desc)>();

            // ── Pre-calibration: scan bundles ─────────────────────────────────
            long firstPid = -1, stompPid = -1;
            float stompAz = 0f;
            long invStart = -1, invEnd = -1, validResumed = -1;

            foreach (var b in bundles)
            {
                if (!b.IsComplete) continue;
                if (firstPid < 0) firstPid = b.PacketId;

                var lf = b.LeftFoot!;
                if (stompPid < 0 && lf.HasAcceleration)
                {
                    float diff = MathF.Abs(lf.Acceleration.Z - 9.81f);
                    if (diff > stompThreshold) { stompPid = b.PacketId; stompAz = lf.Acceleration.Z; }
                }

                if (stompPid >= 0 && b.PacketId > stompPid && validResumed < 0)
                {
                    var lsw = b.LeftFoot!.StatusWord;
                    var psw = b.Pelvis!.StatusWord;
                    var rsw = b.RightFoot!.StatusWord;
                    bool allOk = lsw.HasValue && (lsw.Value & XSF_ORIENT) != 0
                              && psw.HasValue && (psw.Value & XSF_ORIENT) != 0
                              && rsw.HasValue && (rsw.Value & XSF_ORIENT) != 0;
                    if (!allOk) { if (invStart < 0) invStart = b.PacketId; invEnd = b.PacketId; }
                    else if (invStart >= 0) validResumed = b.PacketId;
                }
            }

            if (firstPid >= 0 && stompPid > firstPid + 5)
                events.Add(($"{firstPid}–{stompPid - 1}",
                    $"传感器初始化 + 静止等待（约 {(stompPid - firstPid) / 100f:F1} 秒）"));

            if (stompPid >= 0)
                events.Add(($"{stompPid}",
                    $"跺脚触发标定（Az={stompAz:F1}, diff={MathF.Abs(stompAz - 9.81f):F1} > 阈值{stompThreshold:F0} ✓）"));
            else
                events.Add(("—", $"未检测到跺脚（所有帧 diff < 阈值{stompThreshold:F0}）"));

            if (invStart >= 0)
                events.Add(($"{invStart}–{invEnd}",
                    $"Left 足 SW=0（跺脚后 AHRS 方向短暂失效，约 {(invEnd - invStart + 1) / 100f:F1}s）"));

            if (validResumed >= 0)
                events.Add(($"{validResumed}", "有效帧恢复，仍在静止"));

            // ── Post-calibration: scan DiagnosticsRows ────────────────────────
            if (diag.Count == 0)
            {
                events.Add(("—", "标定未完成（无诊断数据）"));
                return FormatTimelineTable(events);
            }

            long calPid = diag[0].PacketId;
            events.Add(($"≈{calPid}", "标定完成（静止确认10帧 + 采集50帧）"));

            long standEndPid = calPid;
            long walkMotionPid = -1, isWalkTruePid = -1;
            bool prevWalking = false;
            int fpaL = 0, fpaR = 0;
            var turningEps = new List<(long s, long e)>();
            var reacqEps = new List<(long s, long e)>();
            var prevState = ContextState.Straight;
            long prevSegStart = -1;

            foreach (var row in diag)
            {
                if (!row.IsWalking && row.LeftGyr.Length() < 0.3f) standEndPid = row.PacketId;
                if (walkMotionPid < 0 && row.LeftGyr.Length() > 0.8f) walkMotionPid = row.PacketId;
                if (!prevWalking && row.IsWalking && isWalkTruePid < 0) isWalkTruePid = row.PacketId;
                prevWalking = row.IsWalking;
                if (!float.IsNaN(row.FpaLeft_Deg)) fpaL++;
                if (!float.IsNaN(row.FpaRight_Deg)) fpaR++;

                if (row.MotionState != prevState)
                {
                    if (prevState == ContextState.Turning) turningEps.Add((prevSegStart, row.PacketId));
                    if (prevState == ContextState.ReacquiringPd) reacqEps.Add((prevSegStart, row.PacketId));
                    prevSegStart = row.MotionState != ContextState.Straight ? row.PacketId : -1;
                }
                prevState = row.MotionState;
            }
            if (prevState == ContextState.Turning && prevSegStart >= 0) turningEps.Add((prevSegStart, diag[^1].PacketId));
            if (prevState == ContextState.ReacquiringPd && prevSegStart >= 0) reacqEps.Add((prevSegStart, diag[^1].PacketId));

            long turnBarrier = turningEps.Count > 0 ? turningEps[0].s : long.MaxValue;
            long reacqBarrier = reacqEps.Count > 0 ? reacqEps[^1].e : long.MaxValue;
            int preL = 0, preR = 0, postL = 0, postR = 0;
            foreach (var row in diag)
            {
                if (!float.IsNaN(row.FpaLeft_Deg))
                { if (row.PacketId < turnBarrier) preL++; else if (row.PacketId > reacqBarrier) postL++; }
                if (!float.IsNaN(row.FpaRight_Deg))
                { if (row.PacketId < turnBarrier) preR++; else if (row.PacketId > reacqBarrier) postR++; }
            }

            if (standEndPid > calPid + 20)
                events.Add(($"{calPid}–{standEndPid}",
                    $"标定后继续站立 ~{(standEndPid - calPid) / 100f:F1}s → IsWalking=False → FPA门控失败"));

            if (walkMotionPid > 0) events.Add(($"≈{walkMotionPid}", "走路开始"));
            if (isWalkTruePid > 0) events.Add(($"≈{isWalkTruePid}", "IsWalking 第一次变 True（100帧窗口填满）"));

            long fpaWindowStart = isWalkTruePid > 0 ? isWalkTruePid
                                : walkMotionPid > 0 ? walkMotionPid : calPid;

            if (turningEps.Count > 0)
            {
                if (preL > 0 || preR > 0)
                    events.Add(($"{fpaWindowStart}–{turningEps[0].s}",
                        $"有效 FPA 计算窗口（约 {preL}步L / {preR}步R）"));
                foreach (var ep in turningEps)
                    events.Add(($"{ep.s}–{ep.e}", "转弯检测（yaw rate > 0.70 rad/s），进入 Turning"));
                foreach (var ep in reacqEps)
                    events.Add(($"{ep.s}–{ep.e}", "ReacquiringPd：PD 历史清空，等待 2 步重建"));
                if (postL > 0 || postR > 0)
                {
                    long postStart = reacqEps.Count > 0 ? reacqEps[^1].e : turningEps[^1].e;
                    events.Add(($"{postStart}–{diag[^1].PacketId}",
                        $"转弯后有效 FPA 计算（约 {postL}步L / {postR}步R）"));
                }
            }
            else if (fpaL > 0 || fpaR > 0)
            {
                long fpaFirst = -1, fpaLast = -1;
                foreach (var row in diag)
                    if (!float.IsNaN(row.FpaLeft_Deg) || !float.IsNaN(row.FpaRight_Deg))
                    { if (fpaFirst < 0) fpaFirst = row.PacketId; fpaLast = row.PacketId; }
                events.Add(($"{fpaFirst}–{fpaLast}", $"有效 FPA 计算（{fpaL}步L / {fpaR}步R）"));
            }
            else
            {
                events.Add(("—", "无 FPA 输出（门控持续阻断）"));
            }

            string conclusion = fpaL >= minBaselineSteps && fpaR >= minBaselineSteps
                ? "Baseline 完成"
                : "步数不足 → Baseline 未完成 → Training 未启动";
            events.Add(("合计", $"FPA: L={fpaL}步 / R={fpaR}步，MinBaselineSteps={minBaselineSteps} → {conclusion}"));

            return FormatTimelineTable(events);
        }

        private static string FormatTimelineTable(List<(string pid, string desc)> events)
        {
            const int PidW = 14;
            int descW = events.Count > 0 ? events.Max(e => DisplayWidth(e.desc)) : 20;
            descW = Math.Max(descW, 8);

            var sb = new StringBuilder();
            sb.AppendLine($"┌─{new string('─', PidW)}─┬─{new string('─', descW)}─┐");
            sb.AppendLine($"│ {PadD("PacketId", PidW)} │ {PadD("事件", descW)} │");
            foreach (var (pid, desc) in events)
            {
                sb.AppendLine($"├─{new string('─', PidW)}─┼─{new string('─', descW)}─┤");
                sb.AppendLine($"│ {PadD(pid, PidW)} │ {PadD(desc, descW)} │");
            }
            sb.AppendLine($"└─{new string('─', PidW)}─┴─{new string('─', descW)}─┘");
            return sb.ToString();
        }

        private static int DisplayWidth(string s)
        {
            int w = 0;
            foreach (char c in s)
                w += (c >= 0x4E00 && c <= 0x9FFF)
                  || (c >= 0x3000 && c <= 0x303F)
                  || (c >= 0xFF00 && c <= 0xFF60) ? 2 : 1;
            return w;
        }

        private static string PadD(string s, int width)
        {
            int dw = DisplayWidth(s);
            return dw >= width ? s : s + new string(' ', width - dw);
        }

        private void BtnToggleLog_Click(object sender, RoutedEventArgs e)
        {
            _content.LogPanelVisible = !_content.LogPanelVisible;
            bool visible = _content.LogPanelVisible;
            if (BtnToggleLog != null)
                BtnToggleLog.Content = visible ? "Hide Log" : "Show Log";
            // Collapse/restore the log row height so the splitter takes no space when hidden
            LogRow.Height = visible ? new GridLength(160, GridUnitType.Pixel) : new GridLength(0);
            SplitterRow.Height = visible ? new GridLength(5, GridUnitType.Pixel) : new GridLength(0);
        }

        private void BtnToggleParams_Click(object sender, RoutedEventArgs e)
        {
            _content.ParamsPanelVisible = !_content.ParamsPanelVisible;
            if (BtnToggleParams != null)
                BtnToggleParams.Content = _content.ParamsPanelVisible ? "Hide Params" : "Show Params";
        }

        private void OnFpaResult(FpaResult result, bool isTraining)
        {
            bool inTraining = _sessionState == TestState.Step;

            if (!float.IsNaN(result.Fpa_L))
            {
                _content.LeftFpaDeg = result.Fpa_L;
                if (inTraining)
                {
                    _content.LeftFpaBackground = result.OnTarget_L ? MainPageVM.OnTargetBg : MainPageVM.OffTargetBg;
                    if (!float.IsNaN(result.Error_L) && !float.IsNaN(result.Tolerance_L))
                        _content.LeftFpaNote = $"err: {result.Error_L:+0.0;-0.0}° / ±{result.Tolerance_L:F1}°";
                }
            }

            if (!float.IsNaN(result.Fpa_R))
            {
                _content.RightFpaDeg = result.Fpa_R;
                if (inTraining)
                {
                    _content.RightFpaBackground = result.OnTarget_R ? MainPageVM.OnTargetBg : MainPageVM.OffTargetBg;
                    if (!float.IsNaN(result.Error_R) && !float.IsNaN(result.Tolerance_R))
                        _content.RightFpaNote = $"err: {result.Error_R:+0.0;-0.0}° / ±{result.Tolerance_R:F1}°";
                }
            }

            _content.StanceSummary =
                $"L: {result.Fpa_L:F1}° {(result.OnTarget_L ? "✓" : $"err={result.Error_L:+0.0;-0.0}°")}  " +
                $"R: {result.Fpa_R:F1}° {(result.OnTarget_R ? "✓" : $"err={result.Error_R:+0.0;-0.0}°")}";

            if (_wsServer != null)
            {
                _ = _wsServer.BroadcastJsonAsync(new
                {
                    stage = isTraining ? "training" : "baseline",
                    type = "fpa",
                    packetId = result.PacketId,
                    fpaL = result.Fpa_L,
                    fpaR = result.Fpa_R,
                    onTargetL = result.OnTarget_L,
                    onTargetR = result.OnTarget_R,
                    errorL = result.Error_L,
                    errorR = result.Error_R,
                });
            }
        }

    }
}
