using IMUMoCap.AHRS;
using IMUMoCap.Methods;
using IMUMoCap.Model;
using IMUMoCap.Pipeline;
using IMUMoCap.Pipeline.Models;
using IMUMoCap.Services;
using System.Collections.Concurrent;
using System.IO;
using System.Text;
using System.Text.Json;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using System.Windows.Threading;
using XDA;
using Quaternion = System.Windows.Media.Media3D.Quaternion;

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

        // Replay controls
        private volatile bool _replayPaused = false;
        private volatile int  _replayDelayMs = 10;
        private static readonly int[]    ReplaySpeedStepsMs  = { 50, 10, 2, 0 }; // slow → fast
        private static readonly string[] ReplaySpeedLabels   = { "0.2×", "1×", "5×", "Max" };
        private int _replaySpeedIndex = 1; // default: 1× real-time

        // Timeline panel
        private const int TimelineCapacity = 600;
        private readonly Queue<DiagnosticsRow> _timelineBuffer = new(TimelineCapacity + 1);
        private DispatcherTimer? _timelineTimer;
        private static readonly SolidColorBrush s_darkGray;
        private static readonly SolidColorBrush s_darkRed;
        private static readonly SolidColorBrush s_darkOrange;
        private static readonly SolidColorBrush s_darkGreen;
        private static readonly Pen              s_cyanPen;
        private static readonly Pen              s_purplePen;
        static MainWindow()
        {
            s_darkGray   = Freeze(new SolidColorBrush(Color.FromRgb(0x44, 0x44, 0x44)));
            s_darkRed    = Freeze(new SolidColorBrush(Color.FromRgb(0x80, 0x10, 0x10)));
            s_darkOrange = Freeze(new SolidColorBrush(Color.FromRgb(0x80, 0x40, 0x00)));
            s_darkGreen  = Freeze(new SolidColorBrush(Color.FromRgb(0x10, 0x50, 0x10)));
            var cyanBrush   = Freeze(new SolidColorBrush(Colors.Cyan));
            var violetBrush = Freeze(new SolidColorBrush(Colors.Violet));
            s_cyanPen   = Freeze(new Pen(cyanBrush,   1.5));
            s_purplePen = Freeze(new Pen(violetBrush, 1.5));
        }
        private static T Freeze<T>(T f) where T : Freezable { f.Freeze(); return f; }

        // Data recording configuration
        private int _sampleRateHz = 100; // Default 100 Hz
        private readonly List<ImuSampleFrame> _imuSamples = new();
        private readonly ImuFrameCollector _imuFrameCollector = new();

        // Experiment session recording (participant/condition/stage → file)
        private static readonly string[] ConditionStages = { "Baseline", "Training1", "Training2", "Training3", "Retention" };
        private readonly Methods.ExperimentRecorder _recorder = new();
        private string _currentStage = "";
        private bool _isRetention = false; // true once _currentStage == "Retention": AR feedback removed
        private int _retentionStepsL, _retentionStepsR; // valid steps collected during Retention (auto-end at 100/foot)
        private bool _inRest = false; // true during the 120s rest between Training2 and Training3 (_currentStage stays "Training2")

        // Last broadcast AR state, resent as a snapshot when a client (re)connects.
        private string _lastState = "waiting";
        private string _lastCondition = "";
        private int _lastBlock = 0;
        private int _lastRestDurationSec = 0;
        private string _lastReason = "";
        private readonly Dictionary<string, int> _stageAttempt = new();
        private static readonly TimeSpan TrainingBlockMaxDuration = TimeSpan.FromMinutes(5);
        // Baseline has no step-count fallback of its own (BaselineProcessor's 2x-steps rule can still
        // stall indefinitely if one foot never gets enough valid steps, e.g. a short corridor with heavy
        // turning exclusion) — force-finalize after this long so the flow never hangs. Matches the
        // timeout BaselineProcessor.cs's doc comment already assumed but that was never wired up.
        private static readonly TimeSpan BaselineMaxDuration = TimeSpan.FromMinutes(3);
        private const int RetentionTargetSteps = 100; // Retention ends at 100 valid steps/foot or the 5-min cap (Research Overview §2.6)
        private const int RestDurationSec = 120; // Rest between Training2 and Training3 (Research Overview §2.5)
        private readonly System.Diagnostics.Stopwatch _stageStopwatch = new();

        public MainWindow()
        {
            InitializeComponent();
            this.DataContext = _content;

            _wsServer = new WebSocketBroadcastServer();
            _wsServer.Start(new[] { "http://+:8765/ws/" });
            log("WebSocket server started: ws://192.168.137.1:8765/ws/");
            _wsServer.OnTextMessage += (clientId, text) => HandleWsMessage(clientId, text);
            _wsServer.OnClientConnected += (id, remote) => Dispatcher.BeginInvoke(() =>
            {
                log($"WS client connected: {remote}");
                _content.ConnectedWsClients.Add(remote ?? "unknown");
                SendStateSnapshot(id); // (re)connect: restore the current state on the client
            });
            _wsServer.OnClientDisconnected += (id, remote) => Dispatcher.BeginInvoke(() =>
            {
                log($"WS client disconnected: {remote}");
                _content.ConnectedWsClients.Remove(remote ?? "unknown");
            });

            var imuList = new List<ImuViewModel>
            {
                new ImuViewModel(imuPelvis) { DeviceId = 0x00B43CAB, Role = ImuRole.Pelvis },
                new ImuViewModel(imuL)      { DeviceId = 0x10b41913, Role = ImuRole.Left   },
                new ImuViewModel(imuR)      { DeviceId = 0x10B41904, Role = ImuRole.Right  },
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
            _imuFrameCollector.SampleRateHz = _sampleRateHz;
            UpdateImuStatusIndicators();

            // 流水线事件订阅
            _pipeline.OnLog += msg => Dispatcher.BeginInvoke(() => log(msg));
            _pipeline.OnCalibrationStateChanged += s =>
            {
                if (!_isReplaying) Dispatcher.BeginInvoke(() => OnCalibrationStateChanged(s));
            };
            _pipeline.OnBaselineProgress += (l, r) => Dispatcher.BeginInvoke(() =>
            {
                log($"Baseline: L={l} steps, R={r} steps");
                if (!_isReplaying)
                    BroadcastStepProgress("baseline", l, r, _pipeline.Params.MinBaselineSteps);
            });
            _pipeline.OnBaselineCompleted += profile =>
            {
                if (!_isReplaying) Dispatcher.BeginInvoke(() => HandleBaselineCompleted(profile));
            };
            _pipeline.OnFpaResult += (result, isTraining) => Dispatcher.BeginInvoke(() => OnFpaResult(result, isTraining));
            _pipeline.OnDiagnosticsFrame += row => Dispatcher.BeginInvoke(() =>
            {
                _timelineBuffer.Enqueue(row);
                if (_timelineBuffer.Count > TimelineCapacity) _timelineBuffer.Dequeue();

                if (_recorder.IsRecording && !_content.IsPaused)
                    _recorder.WriteRow(row, _currentStage, _stageAttempt.GetValueOrDefault(_currentStage, 1));
            });
            _pipeline.OnStepOutcome += (foot, emitted, reason) => Dispatcher.BeginInvoke(() =>
            {
                if (!_recorder.IsRecording || _content.IsPaused) return;
                _recorder.NoteStepOutcome(_currentStage, emitted, reason);

                var stats = _recorder.GetStageStats(_currentStage);
                if (stats != null && stats.TotalAttempted > 0)
                    _content.TurningExclusionDisplay =
                        $"Turning excluded: {stats.ExcludedTurning + stats.ExcludedReacquiring}/{stats.TotalAttempted} ({stats.TurningExclusionRate:P0})";
            });
            _pipeline.OnTrainingBlockComplete += () => Dispatcher.BeginInvoke(() =>
            {
                if (!_isReplaying && !_inRest && IsTrainingBlockStage(_currentStage))
                    EndTrainingBlock();
            });

            _timelineTimer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(60) };
            _timelineTimer.Tick += (_, _) =>
            {
                DrawTimeline();
                _content.StageElapsedDisplay = _stageStopwatch.Elapsed.ToString(@"mm\:ss");

                // Training block max-duration fallback: ends the block at 5 min even if 150 steps/foot wasn't reached.
                if (!_isReplaying && !_inRest && IsTrainingBlockStage(_currentStage) && _stageStopwatch.Elapsed >= TrainingBlockMaxDuration)
                    EndTrainingBlock();

                // Retention max-duration fallback: ends the whole flow at 5 min even if 100 steps/foot wasn't reached.
                if (!_isReplaying && _isRetention && _stageStopwatch.Elapsed >= TrainingBlockMaxDuration)
                    EndFlow("retention 5-min timeout");
            };
            _timelineTimer.Start();

            // 参数同步：VM 属性变化时推入 pipeline
            _content.PropertyChanged += (_, e) => SyncParamToPipeline(e.PropertyName);
            LoadParamsIfExists();

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
                _content.StatusLabel = "All IMUs connected. Starting measurement...";
                log("All IMUs connected. Auto-starting measurement...");
                int desiredRate = _content.UpdateRates.Count > 0
                    ? Convert.ToInt32(_content.UpdateRates[_content.SelectedRate])
                    : -1;
                _deviceManager.StartMeasurement(desiredRate);
            }
            else
            {
                _content.StatusLabel = ConnectedCountLabel();
            }
        }

        private void OnMtwDisconnected(MtwDisconnectedEvent ev)
        {
            _content.ConnectedMtws.Remove(ev.DeviceIdStr);
            _content.SelectedMtw = _content.ConnectedMtws.Count - 1;

            var vm = _slotRegistry.Imus.FirstOrDefault(a => a.DeviceId == ev.DeviceId);
            if (vm != null) vm.IsConnected = false;

            UpdateImuStatusIndicators();
            _content.StatusLabel = ConnectedCountLabel();
        }

        private string ConnectedCountLabel() =>
            $"{_slotRegistry.Imus.Count(i => i.IsConnected)}/{_slotRegistry.Imus.Count} IMUs connected.";

        private void OnDataPacket(DataPacketEvent ev)
        {
            var snap = _deviceManager.GetMtwDataSnapshot(ev.DeviceId);
            if (snap == null) return;

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
            _timelineTimer?.Stop();
            _recorder.Close();
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
                        _ = _wsServer?.SendJsonAsync(clientId, new { type = "pong", ts = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() });
                        break;

                    case "Start":
                        Dispatcher.Invoke(() =>
                        {
                            log("WS: start");
                        });
                        break;

                    case "ReadyForCalibration":
                        Dispatcher.Invoke(() =>
                        {
                            if (_pipeline.TriggerCalibration())
                                log("WS: participant signaled ready — calibration triggered.");
                            else
                                log("WS: participant signaled ready, but calibration isn't armed yet (Start Condition not open).");
                        });
                        break;

                    case "continueTraining":
                    {
                        int fromBlock = root.TryGetProperty("fromBlock", out var fb) && fb.TryGetInt32(out var v) ? v : -1;
                        Dispatcher.Invoke(() => HandleContinueTraining(fromBlock));
                        break;
                    }

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
                    string dotColor = imu.IsConnected ? "Green" : "Gray";

                    // Dot color conveys connection state; label stays a static role name.
                    switch (imu.Role)
                    {
                        case ImuRole.Pelvis:
                            PelvisStatusDot.Fill = new SolidColorBrush((Color)ColorConverter.ConvertFromString(dotColor));
                            break;
                        case ImuRole.Left:
                            LeftStatusDot.Fill = new SolidColorBrush((Color)ColorConverter.ConvertFromString(dotColor));
                            break;
                        case ImuRole.Right:
                            RightStatusDot.Fill = new SolidColorBrush((Color)ColorConverter.ConvertFromString(dotColor));
                            break;
                    }
                }
            });
        }

        private void log(string log)
        {

            this.Dispatcher.BeginInvoke(() =>
            {
                _content.LogList += $"[{DateTime.Now:HH:mm:ss.fff}] {log}";
                _content.LogList += (Environment.NewLine);

                if (_content.IsScrollerToEnd)
                {
                    LogBox.CaretIndex = LogBox.Text.Length;
                    LogBox.ScrollToEnd();
                }
            });
        }

        // ── AR 反馈事件日志：记录每条广播给 AR 端的 cue（内容+时间戳），事后核对算法输出与实际呈现是否一致/有无掉帧 ──
        private long _arEventSeq = 0;
        private const int ArEventLogMaxChars = 200_000;

        private void LogArEvent(string content)
        {
            long seq = ++_arEventSeq;
            _content.ArEventLog += $"[{seq:000000}] {DateTime.Now:HH:mm:ss.fff}  {content}{Environment.NewLine}";
            if (_content.ArEventLog.Length > ArEventLogMaxChars)
                _content.ArEventLog = _content.ArEventLog.Substring(_content.ArEventLog.Length - ArEventLogMaxChars);

            ArEventLogBox.CaretIndex = ArEventLogBox.Text.Length;
            ArEventLogBox.ScrollToEnd();
        }

        private void BtnClearArLog_Click(object sender, RoutedEventArgs e)
        {
            _content.ArEventLog = "";
        }

        private void setWidgetsStates()
        {
            switch (_content.DeviceState)
            {
                case States.MEASURING:
                    BtnMeasureLabel.Text = "⏹ Stop";
                    break;
                default:
                    BtnMeasureLabel.Text = "▶ Measure";
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

        private void Button_SaveData(object sender, RoutedEventArgs e)
        {
            try
            {
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
                _imuSamples.Clear();
            }
            catch (Exception ex)
            {
                log($"Error saving data: {ex.Message}");
            }
        }

        private bool _loggedLiveDuringReplay = false;

        void OnXsensData(ImuRole sensor, uint deviceId, XsDataPacket packet)
        {
            if (_isReplaying)
            {
                if (!_loggedLiveDuringReplay)
                {
                    _loggedLiveDuringReplay = true;
                    log("Live IMU data ignored — a replay is currently running.");
                }
                return;
            }

            // Operator emergency pause freezes the whole pipeline at the data source: step counters,
            // fpa broadcasts, and CSV rows must all stop advancing while paused.
            if (_content.IsPaused) return;

            // One callback packet becomes a single-IMU sample, then a synchronized 3-IMU frame.
            var (sample, _) = _imuFrameCollector.Process(sensor, deviceId, packet);
            _imuSamples.Add(sample);
            _pipeline.ProcessPacket(sensor, deviceId, packet);
        }

        // ── 流水线状态机 ──────────────────────────────────────────────────────

        private DispatcherTimer? _baselineTimer;

        // Force-finalizes baseline (insufficient steps → HandleBaselineCompleted reports the error) if
        // BaselineMaxDuration elapses before the step thresholds are met.
        private void StartBaselineTimeoutTimer()
        {
            _baselineTimer?.Stop();
            _baselineTimer = new DispatcherTimer { Interval = BaselineMaxDuration };
            _baselineTimer.Tick += (_, _) =>
            {
                _baselineTimer?.Stop();
                log($"Baseline timed out after {BaselineMaxDuration.TotalMinutes:F0} min — force-finalizing.");
                _pipeline.FinalizeBaseline();
            };
            _baselineTimer.Start();
        }

        private void BroadcastArState(string state, string reason = "", int restDurationSec = 0, int? blockOverride = null)
        {
            if (_wsServer == null) return;
            string condition = ConditionForAr();
            int block = blockOverride ?? BlockForStage(_currentStage);
            var (targetL, directionL, targetR, directionR) = TargetForAr();
            _lastState = state;
            _lastCondition = condition;
            _lastBlock = block;
            _lastRestDurationSec = restDurationSec;
            _lastReason = reason;
            _ = _wsServer.BroadcastJsonAsync(new
            {
                type = "state",
                state,
                condition,
                block,
                restDurationSec,
                reason,
                targetL,
                directionL,
                targetR,
                directionR,
                ts = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds(),
            });
            LogArEvent($"type=state  state={state}  condition={condition}  block={block}");
        }

        // Sends the current state to a single (re)connecting client so it restores its display.
        private void SendStateSnapshot(Guid id)
        {
            if (_wsServer == null) return;
            var (targetL, directionL, targetR, directionR) = TargetForAr();
            _ = _wsServer.SendJsonAsync(id, new
            {
                type = "state",
                state = _lastState,
                condition = _lastCondition,
                block = _lastBlock,
                restDurationSec = _lastRestDurationSec,
                reason = _lastReason,
                targetL,
                directionL,
                targetR,
                directionR,
                ts = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds(),
            });
        }

        // EF stepping-stone rotation needs the participant's fixed personalised target angle/direction
        // (Research Overview §2.3/2.4); it's constant for the whole condition once baseline completes,
        // so it's read live from BaselineProfile rather than latched like the rest-only fields above.
        // Sentinel defaults (0f/"") before baseline completes, per the no-nullable-fields design constraint.
        private (float targetL, string directionL, float targetR, string directionR) TargetForAr()
        {
            var bp = _pipeline.BaselineProfile;
            if (bp == null) return (0f, "", 0f, "");
            return (bp.Target_L, DirectionForAr(bp.Direction_L), bp.Target_R, DirectionForAr(bp.Direction_R));
        }

        private static string DirectionForAr(TrainingDirection d) => d == TrainingDirection.ToeIn ? "toe-in" : "toe-out";

        // WS `condition` field: "" before Start Condition is clicked.
        private string ConditionForAr() => _recorder.Condition is "EF" or "IF" ? _recorder.Condition : "";

        private static int BlockForStage(string stage) => stage switch
        {
            "Training1" => 1,
            "Training2" => 2,
            "Training3" => 3,
            _ => 0,
        };

        // Tells the AR client the operator redid the current stage, so it resets locally-tracked counters.
        private void BroadcastRedo(string stage, int attempt, string reason)
        {
            if (_wsServer == null) return;
            _ = _wsServer.BroadcastJsonAsync(new
            {
                type = "redo",
                stage,
                attempt,
                reason,
                ts = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds(),
            });
        }

        // Drives the AR text-group step-count progress bar during baseline and retention.
        private void BroadcastStepProgress(string stage, int stepsL, int stepsR, int required)
        {
            if (_wsServer == null) return;
            _ = _wsServer.BroadcastJsonAsync(new
            {
                type = "stepProgress",
                stage,
                stepsL,
                stepsR,
                requiredL = required,
                requiredR = required,
                ts = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds(),
            });
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
                    StartBaselineTimeoutTimer();
                    break;

                case CalibrationState.Failed:
                    _content.StatusLabel = "Calibration failed. Please restart the application.";
                    _sessionState = TestState.Launching;
                    BroadcastArState("error", "Calibration failed");
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
                BroadcastArState("error", $"Baseline insufficient ({detail})");
                return;
            }
            _sessionState = TestState.Step;
            _pipeline.StartTraining(); // resets tolerance α to the Block 1 default before we display it below
            _content.StatusLabel =
                $"Training started. Target L={profile.Target_L:F1}° ({profile.Direction_L})  " +
                $"R={profile.Target_R:F1}° ({profile.Direction_R})";

            _recorder.SaveBaseline(profile);

            // Advance the experiment stage bookkeeping (Baseline → Training1) so tolerance α, CSV stage
            // tagging, and the 150-step/5-min auto-termination all pick up correctly. AdvanceStage also
            // broadcasts state=training with block=1. Only meaningful when a real condition recording
            // is open (Start Condition was clicked first).
            if (_recorder.IsRecording && _currentStage == "Baseline")
            {
                AdvanceStage();
            }
            else
            {
                // No recording open (e.g. calibration-only test run) — still show target/tolerance locally.
                BroadcastArState("training", blockOverride: 1);
                _content.LeftFpaTarget = $"Target {profile.Target_L:F1}° ±{profile.ToleranceL(_pipeline.CurrentToleranceAlpha):F1}° ({profile.Direction_L})";
                _content.RightFpaTarget = $"Target {profile.Target_R:F1}° ±{profile.ToleranceR(_pipeline.CurrentToleranceAlpha):F1}° ({profile.Direction_R})";
            }
        }

        // ── 参数同步 ──────────────────────────────────────────────────────────

        private void SyncParamToPipeline(string? propName)
        {
            switch (propName)
            {
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
                case nameof(MainPageVM.BaselineImbalanceRatioThreshold):
                    _pipeline.Params.BaselineImbalanceRatioThreshold = _content.BaselineImbalanceRatioThreshold; break;
            }
        }

        private static readonly string ParamsFilePath =
            Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "pipeline_params.json");

        private void LoadParamsIfExists()
        {
            if (!File.Exists(ParamsFilePath)) return;
            var json = File.ReadAllText(ParamsFilePath, Encoding.UTF8);
            var p = JsonSerializer.Deserialize<PipelineParams>(json);
            if (p == null) return;

            _content.StaticGyroThreshold = p.StaticGyroThreshold;
            _content.StanceFreeAccThreshold = p.StanceFreeAccThreshold;
            _content.StanceGyroThreshold = p.StanceGyroThreshold;
            _content.PdConfidenceThreshold = p.PdConfidenceThreshold;
            _content.PdStabilityThreshold = p.PdStabilityThreshold;
            _content.MinBaselineSteps = p.MinBaselineSteps;
            _content.BaselineImbalanceRatioThreshold = p.BaselineImbalanceRatioThreshold;
            _pipeline.Params.StanceFootPitchThreshold = p.StanceFootPitchThreshold;
        }

        private void BtnSaveParams_Click(object sender, RoutedEventArgs e)
        {
            var json = JsonSerializer.Serialize(_pipeline.Params, new JsonSerializerOptions { WriteIndented = true });
            File.WriteAllText(ParamsFilePath, json, Encoding.UTF8);
            log("Parameters saved.");
        }

        private ParamsDialog? _paramsDialog;

        private void BtnOpenParams_Click(object sender, RoutedEventArgs e)
        {
            if (_paramsDialog != null) { _paramsDialog.Activate(); return; }

            _paramsDialog = new ParamsDialog { Owner = this, DataContext = _content };
            _paramsDialog.SaveParamsClicked += BtnSaveParams_Click;
            _paramsDialog.Closed += (_, _) => _paramsDialog = null;
            _paramsDialog.Show();
        }

        // ── 新增按钮处理器 ────────────────────────────────────────────────────

        private void BtnStartCal_Click(object sender, RoutedEventArgs e)
        {
            _baselineTimer?.Stop();
            _baselineTimer = null;
            _pipeline.Reset();
            _timelineBuffer.Clear();
            _imuSamples.Clear();
            _imuFrameCollector.Reset();
            _content.LeftFpaTarget = "";
            _content.RightFpaTarget = "";
            _content.LeftFpaNote = "";
            _content.RightFpaNote = "";
            _content.LeftFpaBackground = MainPageVM.DefaultCardBg;
            _content.RightFpaBackground = MainPageVM.DefaultCardBg;
            _sessionState = TestState.Launching;

            if (_pipeline.TriggerCalibration())
            {
                log("Calibration reset and started — stand still.");
            }
            else
            {
                _content.StatusLabel = "Reset. Click Start Condition first, then Start Calibration.";
                BroadcastArState("waiting");
                log("Calibration reset — not started (click Start Condition first).");
            }
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

        // ── 实验会话：Participant/Condition/Stage → 文件 ─────────────────────────

        private void BtnStartCondition_Click(object sender, RoutedEventArgs e)
        {
            string participantId = txtParticipantId.Text.Trim();
            if (string.IsNullOrEmpty(participantId)) { log("Enter Participant ID first."); return; }
            if (_content.DeviceState != States.MEASURING) { log("Start Condition blocked: IMUs are not measuring. Click Measure first."); return; }

            string condition = (cmbCondition.SelectedItem as ComboBoxItem)?.Content?.ToString() ?? "EF";
            string orderGroup = (cmbOrderGroup.SelectedItem as ComboBoxItem)?.Content?.ToString() ?? "A";

            if (Methods.ExperimentRecorder.SessionFileExists(participantId, condition))
            {
                var overwrite = MessageBox.Show(
                    $"P{participantId}_{condition}_Session.csv already exists and will be overwritten. Continue?",
                    "Participant/Condition already exists", MessageBoxButton.YesNo, MessageBoxImage.Warning);
                if (overwrite != MessageBoxResult.Yes)
                {
                    log($"Start Condition cancelled — {condition} data for P{participantId} already exists.");
                    return;
                }
            }

            _recorder.StartCondition(participantId, condition, orderGroup);
            _pipeline.CalibrationArmed = true;
            _stageAttempt.Clear();
            _currentStage = ConditionStages[0];
            _isRetention = false;
            _stageStopwatch.Restart();

            _content.CurrentStageLabel = _currentStage;
            _content.CurrentSessionFileName = _recorder.CurrentSessionFilePath ?? "";
            _content.CurrentMetaFileName = _recorder.CurrentMetaFilePath ?? "";
            _content.CurrentQoeFileName = _recorder.CurrentQoeFilePath ?? "";
            _content.TurningExclusionDisplay = "";
            BroadcastArState("armed"); // tells the AR client it can show its "ready to calibrate" button
            log($"Started condition {condition} (order {orderGroup}) → {_recorder.CurrentSessionFilePath}");
        }

        private void BtnNextStage_Click(object sender, RoutedEventArgs e)
        {
            if (!_recorder.IsRecording) { log("No active condition recording."); return; }

            int idx = Array.IndexOf(ConditionStages, _currentStage);
            if (idx < 0 || idx >= ConditionStages.Length - 1) { log("Already at final stage (Retention)."); return; }

            AdvanceStage();
        }

        private static bool IsTrainingBlockStage(string stage) =>
            stage is "Training1" or "Training2" or "Training3";

        // A training block finished (150 steps/foot or 5-min). Training2 goes to the 120s rest first;
        // every other block advances straight to the next stage.
        // Guards against a duplicate block-completion event for the same block: the 150-steps/foot
        // pipeline event is queued via Dispatcher.BeginInvoke (background thread), while the 5-min
        // timeout fires synchronously on the UI thread's DispatcherTimer.Tick — if both conditions are
        // met near-simultaneously, the queued event can still run after the timer has already advanced
        // the stage, which would otherwise end the *next* block early. Reset whenever a new training
        // block is entered (AdvanceStage, in the trainingBlock > 0 branch).
        private bool _trainingBlockEnding = false;

        private void EndTrainingBlock()
        {
            if (_trainingBlockEnding) return; // stale duplicate completion event for this block — ignore
            _trainingBlockEnding = true;

            if (_currentStage == "Training2")
                EnterRest();
            else
                AdvanceStage();
        }

        // Enters the 120s rest between Training2 and Training3. Stays in the Training2 recording stage
        // (rest is not a data stage). Advance to Training3 happens on AR `continueTraining` or the
        // operator Next Stage button. _stageStopwatch is restarted so the server can enforce the 120s min.
        private void EnterRest()
        {
            _inRest = true;
            _stageStopwatch.Restart();
            BroadcastArState("rest", restDurationSec: RestDurationSec, blockOverride: 2);
            log($"Rest started ({RestDurationSec}s) after Training2 — waiting for participant Continue.");
        }

        // AR `continueTraining` after the rest countdown. Advances to Training3 only if we are actually
        // in rest, the just-finished block matches (2), and the 120s minimum has really elapsed — the PC
        // never trusts the client's own timer for protocol-timing integrity.
        private void HandleContinueTraining(int fromBlock)
        {
            if (!_inRest)
            {
                log($"WS: continueTraining ignored — not in rest (fromBlock={fromBlock}).");
                return;
            }
            if (fromBlock != 2)
            {
                log($"WS: continueTraining ignored — fromBlock={fromBlock} != 2 (stale/duplicate).");
                return;
            }
            if (_stageStopwatch.Elapsed < TimeSpan.FromSeconds(RestDurationSec))
            {
                log($"WS: continueTraining rejected — only {_stageStopwatch.Elapsed.TotalSeconds:F0}s of {RestDurationSec}s rest elapsed.");
                return;
            }
            log("WS: continueTraining accepted → Training3.");
            AdvanceStage(); // Training2 → Training3 (clears _inRest, broadcasts state=training block=3)
        }

        /// <summary>
        /// Moves _currentStage to the next entry in ConditionStages. Called both manually (Next Stage button)
        /// and automatically — training block auto-completion (150 valid steps/foot) and the 5-minute block
        /// timeout both call this too (Research Overview §2.6).
        /// </summary>
        private void AdvanceStage()
        {
            _inRest = false; // any stage advance ends a pending rest
            int idx = Array.IndexOf(ConditionStages, _currentStage);
            if (idx < 0 || idx >= ConditionStages.Length - 1) return;

            _recorder.FlushMeta();
            _currentStage = ConditionStages[idx + 1];
            _stageStopwatch.Restart();
            _content.CurrentStageLabel = _currentStage;
            _content.TurningExclusionDisplay = "";

            // Progressive-difficulty tolerance narrowing across the 3 training blocks (Research Overview §2.4).
            int trainingBlock = _currentStage switch
            {
                "Training1" => 1,
                "Training2" => 2,
                "Training3" => 3,
                _ => 0,
            };
            if (trainingBlock > 0)
            {
                _trainingBlockEnding = false; // fresh block instance — re-arm the completion guard
                _pipeline.SetTrainingBlock(trainingBlock);
                BroadcastArState("training"); // block field reflects the just-entered training block (spec #8)
                if (_pipeline.BaselineProfile is { } bp)
                {
                    _content.LeftFpaTarget = $"Target {bp.Target_L:F1}° ±{bp.ToleranceL(_pipeline.CurrentToleranceAlpha):F1}° ({bp.Direction_L})";
                    _content.RightFpaTarget = $"Target {bp.Target_R:F1}° ±{bp.ToleranceR(_pipeline.CurrentToleranceAlpha):F1}° ({bp.Direction_R})";
                }
            }
            else if (_currentStage == "Retention")
            {
                // Immediate Retention Test: AR feedback removed, FPA data collection continues (Research Overview §2.6).
                _isRetention = true;
                _retentionStepsL = _retentionStepsR = 0;
                BroadcastArState("retention");
            }

            log($"Stage → {_currentStage}");
        }

        private void BtnEndCondition_Click(object sender, RoutedEventArgs e)
        {
            EndFlow("operator ended");
        }

        /// <summary>
        /// Ends the whole condition flow and saves the session file. Called both by the operator End
        /// button and automatically when Retention hits 100 valid steps/foot or the 5-min cap.
        /// Idempotent (no-op once the recording is closed).
        /// </summary>
        private void EndFlow(string reason)
        {
            if (!_recorder.IsRecording) return;
            _isRetention = false;
            _inRest = false;
            _baselineTimer?.Stop();
            _recorder.FlushMeta();
            _recorder.Close();
            _pipeline.CalibrationArmed = false;
            _stageStopwatch.Stop();
            BroadcastArState("ended");
            log($"Recording closed ({reason}): {_content.CurrentSessionFileName}");
        }

        // Maps the current stage to the AR state string, used to restore the AR display after a resume.
        private static string ArStateForStage(string stage) => stage switch
        {
            "Baseline" => "baseline",
            "Training1" or "Training2" or "Training3" => "training",
            "Retention" => "retention",
            _ => "armed",
        };

        private void BtnPauseResume_Click(object sender, RoutedEventArgs e)
        {
            if (!_recorder.IsRecording) { log("No active recording to pause."); return; }

            if (!_content.IsPaused)
            {
                _content.IsPaused = true;
                _stageStopwatch.Stop();
                _recorder.BeginPause(_currentStage);
                BtnPauseResume.Content = "Resume";
                BroadcastArState("paused"); // operator emergency pause → AR shows its paused display
                log($"Paused at {_currentStage}");
            }
            else
            {
                var dlg = new PauseResumeDialog(_recorder.PauseElapsed) { Owner = this };
                bool? ok = dlg.ShowDialog();
                if (ok != true) return; // 关闭对话框但没填写/没选择 → 保持暂停状态

                if (dlg.Redo)
                {
                    int nextAttempt = _stageAttempt.GetValueOrDefault(_currentStage, 1) + 1;
                    _stageAttempt[_currentStage] = nextAttempt;
                    _recorder.EndPause("Redo", dlg.Reason);
                    _recorder.MarkRedo(_currentStage, nextAttempt);
                    _stageStopwatch.Restart();
                    BroadcastRedo(_currentStage, nextAttempt, dlg.Reason);
                    log($"Resumed with Redo — {_currentStage} attempt #{nextAttempt}: {dlg.Reason}");
                }
                else
                {
                    _recorder.EndPause("Continue", dlg.Reason);
                    _stageStopwatch.Start();
                    log($"Resumed, continuing {_currentStage}: {dlg.Reason}");
                }

                // Tell AR the participant is resuming: it shows a "keep walking" text prompt for the
                // current stage and only switches back to the graphic group when the next fpa arrives.
                BroadcastArState(ArStateForStage(_currentStage));

                _content.IsPaused = false;
                BtnPauseResume.Content = "Pause";
            }
        }

        private async void BtnLoadData_Click(object sender, RoutedEventArgs e)
        {
            if (_recorder.IsRecording)
            {
                log("Load IMU Samples blocked: a condition is currently recording — " +
                    "replaying now would write mislabeled data into that session's CSV. End or pause it first.");
                return;
            }
            if (_content.DeviceState == States.MEASURING)
            {
                log("Load IMU Samples blocked: IMUs are currently measuring — replaying now would mix live and replayed data. Stop measurement first.");
                return;
            }

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
            _content.StatusLabel = "Loading IMU samples...";

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
                _content.StatusLabel = "Load failed: no samples found.";
                BtnLoadData.IsEnabled = true;
                return;
            }

            int completeBundles = bundles.Count(b => b.IsComplete);
            log($"Loaded {samples.Count} samples → {bundles.Count} bundles ({completeBundles} complete). Replaying...");
            _content.StatusLabel = $"Replaying {samples.Count} samples ({completeBundles} complete bundles)...";

            _isReplaying = true;
            _loggedLiveDuringReplay = false;
            BtnMeasure.IsEnabled = false;
            _pipeline.Reset();
            _sessionState = TestState.Launching;

            // Replay drives the pipeline directly (no operator click), so arm + trigger
            // calibration ourselves — otherwise it stays in WaitingForStart and Process()
            // never gets past the calibration gate (no gait/FPA/diagnostics/timeline output).
            bool prevCalibrationArmed = _pipeline.CalibrationArmed;
            _pipeline.CalibrationArmed = true;
            _pipeline.TriggerCalibration();

            void onCalChanged(CalibrationState s)
            {
                if (s != CalibrationState.Completed) return;
                _pipeline.StartBaseline();
                Dispatcher.BeginInvoke(() =>
                {
                    _sessionState = TestState.Baseline;
                    _content.StatusLabel = "Replay: calibration done. Baseline started.";
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
                    _content.LeftFpaTarget = $"Target {profile.Target_L:F1}° ±{profile.ToleranceL(_pipeline.CurrentToleranceAlpha):F1}° ({profile.Direction_L})";
                    _content.RightFpaTarget = $"Target {profile.Target_R:F1}° ±{profile.ToleranceR(_pipeline.CurrentToleranceAlpha):F1}° ({profile.Direction_R})";
                    _content.StatusLabel =
                        $"Training started. Target L={profile.Target_L:F1}° ({profile.Direction_L})  " +
                        $"R={profile.Target_R:F1}° ({profile.Direction_R})";
                    log("Replay: baseline done — training started.");
                });
            }

            _pipeline.OnCalibrationStateChanged += onCalChanged;
            _pipeline.OnBaselineCompleted += onBaselineDone;

            _replayPaused = false;
            _replaySpeedIndex = 1;
            _replayDelayMs = ReplaySpeedStepsMs[_replaySpeedIndex];
            ReplayControls.Visibility = Visibility.Visible;
            BtnReplayPause.Content = "⏸";

            try
            {
                await Task.Run(async () =>
                {
                    foreach (var bundle in bundles)
                    {
                        while (_replayPaused) await Task.Delay(50);
                        _pipeline.Process(bundle);
                        if (_replayDelayMs > 0) await Task.Delay(_replayDelayMs);
                    }
                });
            }
            finally
            {
                _pipeline.OnCalibrationStateChanged -= onCalChanged;
                _pipeline.OnBaselineCompleted -= onBaselineDone;
                _pipeline.CalibrationArmed = prevCalibrationArmed;
                _isReplaying = false;
                _replayPaused = false;
                ReplayControls.Visibility = Visibility.Collapsed;
                BtnLoadData.IsEnabled = true;
                BtnMeasure.IsEnabled = true;
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
                LegacyStompThreshold_ms2, // live calibration no longer uses stomp; this is only for interpreting old recordings
                _pipeline.Params.MinBaselineSteps));
            log("Replay done. Save diagnostics to inspect results.");
            _content.StatusLabel = "Replay done. Save diagnostics to inspect results.";
        }

        private const float LegacyStompThreshold_ms2 = 15f;

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
                    $"Sensor init + static wait (~{(stompPid - firstPid) / 100f:F1} s)"));

            if (stompPid >= 0)
                events.Add(($"{stompPid}",
                    $"Stomp triggered calibration (Az={stompAz:F1}, diff={MathF.Abs(stompAz - 9.81f):F1} > threshold {stompThreshold:F0} ✓)"));
            else
                events.Add(("—", $"No stomp detected (all frames diff < threshold {stompThreshold:F0})"));

            if (invStart >= 0)
                events.Add(($"{invStart}–{invEnd}",
                    $"Left foot SW=0 (AHRS orientation briefly invalid after stomp, ~{(invEnd - invStart + 1) / 100f:F1}s)"));

            if (validResumed >= 0)
                events.Add(($"{validResumed}", "Valid frames resumed, still static"));

            // ── Post-calibration: scan DiagnosticsRows ────────────────────────
            if (diag.Count == 0)
            {
                events.Add(("—", "Calibration incomplete (no diagnostics data)"));
                return FormatTimelineTable(events);
            }

            long calPid = diag[0].PacketId;
            events.Add(($"≈{calPid}", "Calibration done (10 static confirm + 50 collect frames)"));

            long standEndPid = calPid;
            long walkMotionPid = -1, isWalkTruePid = -1;
            bool prevWalking = false;
            int fpaL = 0, fpaR = 0;
            int trainOkL = 0, trainMissL = 0, trainOkR = 0, trainMissR = 0;
            long trainFirstPid = -1, trainLastPid = -1;
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

                // Training 步：Error 不为 NaN 说明 Baseline 已设置（Training 阶段）
                if (!float.IsNaN(row.FpaLeft_Error))
                {
                    if (trainFirstPid < 0) trainFirstPid = row.PacketId;
                    trainLastPid = row.PacketId;
                    if (row.FpaLeft_OnTarget) trainOkL++; else trainMissL++;
                }
                if (!float.IsNaN(row.FpaRight_Error))
                {
                    if (trainFirstPid < 0) trainFirstPid = row.PacketId;
                    trainLastPid = row.PacketId;
                    if (row.FpaRight_OnTarget) trainOkR++; else trainMissR++;
                }

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
                    $"Standing still after calibration ~{(standEndPid - calPid) / 100f:F1}s → IsWalking=False → FPA gating failed"));

            if (walkMotionPid > 0) events.Add(($"≈{walkMotionPid}", "Walking started"));
            if (isWalkTruePid > 0) events.Add(($"≈{isWalkTruePid}", "IsWalking first turned True (100-frame window full)"));

            long fpaWindowStart = isWalkTruePid > 0 ? isWalkTruePid
                                : walkMotionPid > 0 ? walkMotionPid : calPid;

            if (turningEps.Count > 0)
            {
                if (preL > 0 || preR > 0)
                    events.Add(($"{fpaWindowStart}–{turningEps[0].s}",
                        $"Valid FPA window (~{preL} steps L / {preR} steps R)"));
                foreach (var ep in turningEps)
                    events.Add(($"{ep.s}–{ep.e}", "Turn detected (yaw rate > 0.70 rad/s), entering Turning"));
                foreach (var ep in reacqEps)
                    events.Add(($"{ep.s}–{ep.e}", "ReacquiringPd: PD history cleared, waiting 2 steps to rebuild"));
                if (postL > 0 || postR > 0)
                {
                    long postStart = reacqEps.Count > 0 ? reacqEps[^1].e : turningEps[^1].e;
                    events.Add(($"{postStart}–{diag[^1].PacketId}",
                        $"Valid FPA after turn (~{postL} steps L / {postR} steps R)"));
                }
            }
            else if (fpaL > 0 || fpaR > 0)
            {
                long fpaFirst = -1, fpaLast = -1;
                foreach (var row in diag)
                    if (!float.IsNaN(row.FpaLeft_Deg) || !float.IsNaN(row.FpaRight_Deg))
                    { if (fpaFirst < 0) fpaFirst = row.PacketId; fpaLast = row.PacketId; }
                events.Add(($"{fpaFirst}–{fpaLast}", $"Valid FPA ({fpaL} steps L / {fpaR} steps R)"));
            }
            else
            {
                events.Add(("—", "No FPA output (gating blocked continuously)"));
            }

            string conclusion = fpaL >= minBaselineSteps && fpaR >= minBaselineSteps
                ? "Baseline done"
                : "Insufficient steps → Baseline incomplete → Training not started";
            events.Add(("Total", $"FPA: L={fpaL} steps / R={fpaR} steps, MinBaselineSteps={minBaselineSteps} → {conclusion}"));

            if (trainOkL + trainMissL + trainOkR + trainMissR > 0)
                events.Add(($"{trainFirstPid}–{trainLastPid}",
                    $"Training: L={trainOkL} steps OK {trainMissL} steps Miss / R={trainOkR} steps OK {trainMissR} steps Miss"));
            else if (fpaL >= minBaselineSteps && fpaR >= minBaselineSteps)
                events.Add(("—", "Training started, no FPA step data yet"));

            return FormatTimelineTable(events);
        }

        private static string FormatTimelineTable(List<(string pid, string desc)> events)
        {
            const int PidW = 14;
            int descW = events.Count > 0 ? events.Max(e => DisplayWidth(e.desc)) : 20;
            descW = Math.Max(descW, 8);

            var sb = new StringBuilder();
            sb.AppendLine($"┌─{new string('─', PidW)}─┬─{new string('─', descW)}─┐");
            sb.AppendLine($"│ {PadD("PacketId", PidW)} │ {PadD("Event", descW)} │");
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

        private void BtnReplayPause_Click(object sender, RoutedEventArgs e)
        {
            _replayPaused = !_replayPaused;
            if (_replayPaused)
            {
                BtnReplayPause.Content = "▶";
                if (_timelineBuffer.Count > 0)
                    log($"Paused. Timeline first frame PacketId={_timelineBuffer.Peek().PacketId}");
            }
            else
            {
                _replaySpeedIndex = 1;
                _replayDelayMs = ReplaySpeedStepsMs[_replaySpeedIndex];
                BtnReplayPause.Content = "⏸";
                log($"Resumed at {ReplaySpeedLabels[_replaySpeedIndex]}");
            }
        }

        private void BtnReplaySlower_Click(object sender, RoutedEventArgs e)
        {
            if (_replaySpeedIndex <= 0) return;
            _replaySpeedIndex--;
            _replayDelayMs = ReplaySpeedStepsMs[_replaySpeedIndex];
            log($"Replay speed: {ReplaySpeedLabels[_replaySpeedIndex]}");
        }

        private void BtnReplayFaster_Click(object sender, RoutedEventArgs e)
        {
            if (_replaySpeedIndex >= ReplaySpeedStepsMs.Length - 1) return;
            _replaySpeedIndex++;
            _replayDelayMs = ReplaySpeedStepsMs[_replaySpeedIndex];
            log($"Replay speed: {ReplaySpeedLabels[_replaySpeedIndex]}");
        }

        private void TimelineBorder_SizeChanged(object sender, SizeChangedEventArgs e) => DrawTimeline();

        private void DrawTimeline()
        {
            var frames = _timelineBuffer.ToArray();
            int w = Math.Max(0, (int)TimelineBorder.ActualWidth - 64);
            if (w < 2) return;

            const int laneH = 16, laneCount = 6, h = laneH * laneCount;
            double dx = Math.Max(1.0, (double)w / TimelineCapacity);
            bool showText = dx >= 12;
            float confThr = (float)_pipeline.Params.PdConfidenceThreshold;
            float stabThr = (float)_pipeline.Params.PdStabilityThreshold;

            var typeface = new Typeface("Consolas");

            var dv = new DrawingVisual();
            using (var dc = dv.RenderOpen())
            {
                dc.DrawRectangle(Brushes.Black, null, new Rect(0, 0, w, h));

                for (int i = 0; i < frames.Length; i++)
                {
                    double x = (double)i / TimelineCapacity * w;
                    var r = frames[i];

                    Rect LaneRect(int lane) => new Rect(x, lane * laneH, dx, laneH - 1);

                    // Lane 0: PdIsValid
                    dc.DrawRectangle(r.PdIsValid ? Brushes.DodgerBlue : s_darkGray, null, LaneRect(0));
                    // Lane 1: L.Stance
                    dc.DrawRectangle(r.LeftStance ? Brushes.LimeGreen : s_darkGray, null, LaneRect(1));
                    // Lane 2: R.Stance
                    dc.DrawRectangle(r.RightStance ? Brushes.LimeGreen : s_darkGray, null, LaneRect(2));
                    // Lane 3: IsWalking
                    dc.DrawRectangle(r.IsWalking ? Brushes.DodgerBlue : s_darkGray, null, LaneRect(3));
                    // Lane 4: MotionContext
                    var ctxBrush = r.MotionState switch
                    {
                        ContextState.Turning       => Brushes.Red,
                        ContextState.ReacquiringPd => Brushes.Orange,
                        _                          => Brushes.LimeGreen,
                    };
                    dc.DrawRectangle(ctxBrush, null, LaneRect(4));
                    // Lane 5: FPA gate background (why blocked) + tick on actual output
                    Brush fpaGateBrush;
                    if (r.MotionState != ContextState.Straight || r.MotionConfidence < confThr)
                        fpaGateBrush = s_darkRed;
                    else if (!r.PdIsValid || r.PdStability < stabThr)
                        fpaGateBrush = s_darkOrange;
                    else if (!r.IsWalking)
                        fpaGateBrush = s_darkGray;
                    else
                        fpaGateBrush = s_darkGreen;
                    dc.DrawRectangle(fpaGateBrush, null, LaneRect(5));
                    bool hasL = !float.IsNaN(r.FpaLeft_Deg);
                    bool hasR = !float.IsNaN(r.FpaRight_Deg);
                    if (hasL)
                    {
                        dc.DrawLine(s_cyanPen, new Point(x, 5 * laneH), new Point(x, 6 * laneH));
                        if (showText)
                        {
                            var ft = new FormattedText($"{r.FpaLeft_Deg:F0}",
                                System.Globalization.CultureInfo.InvariantCulture,
                                FlowDirection.LeftToRight, typeface, 9, Brushes.Cyan,
                                VisualTreeHelper.GetDpi(this).PixelsPerDip);
                            dc.DrawText(ft, new Point(x + 1, 5 * laneH));
                        }
                    }
                    if (hasR)
                    {
                        dc.DrawLine(s_purplePen, new Point(x, 5 * laneH), new Point(x, 6 * laneH));
                        if (showText && !hasL)
                        {
                            var ft = new FormattedText($"{r.FpaRight_Deg:F0}",
                                System.Globalization.CultureInfo.InvariantCulture,
                                FlowDirection.LeftToRight, typeface, 9, Brushes.Violet,
                                VisualTreeHelper.GetDpi(this).PixelsPerDip);
                            dc.DrawText(ft, new Point(x + 1, 5 * laneH));
                        }
                    }
                }
            }

            var rtb = new RenderTargetBitmap(w, h, 96, 96, PixelFormats.Pbgra32);
            rtb.Render(dv);
            rtb.Freeze();
            TimelineImage.Source = rtb;
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

            // Immediate Retention Test: count valid steps and end the whole flow at 100/foot (the 5-min cap
            // is handled by the timeline timer). AR feedback is removed, so no fpa is broadcast below.
            if (_isRetention && _recorder.IsRecording)
            {
                if (!float.IsNaN(result.Fpa_L)) _retentionStepsL++;
                if (!float.IsNaN(result.Fpa_R)) _retentionStepsR++;
                BroadcastStepProgress("retention", _retentionStepsL, _retentionStepsR, RetentionTargetSteps);
                if (_retentionStepsL >= RetentionTargetSteps && _retentionStepsR >= RetentionTargetSteps)
                {
                    EndFlow($"retention target reached ({_retentionStepsL}/{_retentionStepsR} valid steps)");
                    return;
                }
            }

            // fpa feedback is sent only during a training block — not during baseline or retention
            // (both show the text group + progress bar; the AR client renders no target graphics).
            if (_wsServer != null && isTraining && !_isRetention && !_inRest)
            {
                string stageTag = "training";

                var latencyMs = _pipeline.TakeFrameLatencyMs(result.PacketId); // TEMP：延迟测量，用完删除
                if (latencyMs.HasValue)
                    log($"[延迟-temp] 帧→AR广播: {latencyMs.Value} ms (packetId={result.PacketId})");

                _ = _wsServer.BroadcastJsonAsync(new
                {
                    stage = stageTag,
                    type = "fpa",
                    block = BlockForStage(_currentStage),
                    packetId = result.PacketId,
                    fpaL = result.Fpa_L,
                    fpaR = result.Fpa_R,
                    onTargetL = result.OnTarget_L,
                    onTargetR = result.OnTarget_R,
                    errorL = result.Error_L,
                    errorR = result.Error_R,
                    confidence = result.ContextConfidence,
                    stability = result.PdStability,
                    quality = result.Quality,
                    ts = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds(),
                });

                LogArEvent($"type=fpa  stage={stageTag}  packetId={result.PacketId}  " +
                    $"L={result.Fpa_L:F1}°({(result.OnTarget_L ? "OK" : "ERR")})  R={result.Fpa_R:F1}°({(result.OnTarget_R ? "OK" : "ERR")})  quality={result.Quality}");
            }
        }

    }
}
