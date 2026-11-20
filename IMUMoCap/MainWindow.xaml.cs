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
        private readonly MagMonitor _magMonitor = new();
        private readonly WarmupMonitor _warmup = new(); // MTw filter warm-up gate for Start Condition
        private DispatcherTimer? _magTimer;

        // Experiment session recording (participant/condition/stage → file)
        private static readonly string[] ConditionStages = { "Baseline", "Training1", "Training2", "Training3", "Retention" };

        // Row order for the at-a-glance flow checklist (MainWindow "Flow" panel) — mirrors the full
        // protocol sequence including the Rest+MC/questionnaire windows that aren't in ConditionStages.
        // MarkerStage non-null → the row's label becomes clickable while its window is open (see FlowItem_MouseLeftButtonUp).
        private static readonly (string Label, string? MarkerStage)[] ChecklistDefs =
        {
            ("Baseline", null), ("Training1", null), ("MC (Block1)", "ManipulationCheck_AfterBlock1"),
            ("Training2", null), ("Training3", null), ("MC (Block3)", "ManipulationCheck_AfterBlock3"),
            ("NASA-TLX #1", "NASA-TLX_Admin1"), ("IMI-PC", "IMI-PC"),
            ("Retention", null), ("NASA-TLX #2", "NASA-TLX_Admin2"),
        };
        private readonly Methods.ExperimentRecorder _recorder = new();
        // The experiment protocol state lives in ProtocolFlow (T8). Every transition goes through _shadow.Fire(...),
        // which feeds the event to the flow and logs the ones it refuses (Data/shadow_flow.log + the operator log).
        // The fields below (_currentStage, _inRest, ...) are read-only views of the flow, not state of their own.
        private readonly Protocol.ProtocolFlow _flow = new();
        private readonly Protocol.ShadowMonitor _shadow;
        private string _currentStage => _flow.CurrentStage;
        private bool _isRetention => _flow.IsRetention; // Retention (or waiting to redo it): AR feedback removed
        // True once Retention's 100/foot target (or the stall-prevention timeout) is reached. Data collection
        // and step counting stop, but the recording stays open — the operator still has to mark NASA-TLX
        // Admin 2 and click End Recording manually (Retention -> [NASA-TLX(2) marker] -> Ended).
        private bool _retentionComplete => _flow.RetentionComplete;
        private int _retentionStepsL, _retentionStepsR; // valid steps collected during Retention (auto-end at 100/foot)
        private bool _inRest => _flow.InRest; // resting after Training1/Training3 (_currentStage stays that training stage)
        private string? _mountingRejection; // set when the calibration that is about to report Failed was rejected for IMU mounting

        // Last broadcast AR state, resent as a snapshot when a client (re)connects.
        private string _lastState = "waiting";
        private string _lastCondition = "";
        private int _lastBlock = 0;
        private int _lastRestDurationSec = 0;
        private string _lastReason = "";
        // What the AR client was last shown before an operator pause (the latest non-"paused" broadcast). Resume
        // re-sends exactly this: recomputing a state from _currentStage is wrong while waiting for Ready or resting
        // (_currentStage is still the previous stage then), and it left the AR with no Ready button and the
        // operator's Ready (AR) button disabled — a deadlock.
        private (string State, string Reason, int RestSec, int Block) _arStateBeforePause = ("waiting", "", 0, 0);

        // IF render-mode toggle (ParamsDialog checkbox bound to MainPageVM.IsIfLastResultMode): tells the AR client
        // which rendering mode to use for IF — "lastResult" (default: the step's fpa result stays shown until the next
        // one) or "placeholder" (idle pose + terminal flash that reverts). Both are driven only by the per-step `fpa`
        // messages, i.e. the stride-direction FPA. The former "live" mode (continuous angle = foot yaw − current pelvis
        // yaw, which drifted with the pelvis heading) and its `live` stream were removed on 2026-11-17.
        private string _ifRenderMode = "lastResult";
        // Pure stall-prevention safety net, not a co-equal exit criterion — every step-count-gated stage's
        // real exit criterion is a fixed 100 valid steps/foot. Widened from 5min (which was tuned for the
        // old 150-steps/foot target) to give a 100-step block enough headroom.
        // ASSUMPTION: exact fallback duration isn't specified by the protocol doc — flagged for researcher confirmation.
        private static readonly TimeSpan TrainingBlockMaxDuration = TimeSpan.FromMinutes(10);
        // Baseline has no step-count fallback of its own (BaselineProcessor's 2x-steps rule can still
        // stall indefinitely if one foot never gets enough valid steps, e.g. a short corridor with heavy
        // turning exclusion) — force-finalize after this long so the flow never hangs. Widened alongside
        // TrainingBlockMaxDuration for the same 100-step-target reason; same assumption applies.
        private static readonly TimeSpan BaselineMaxDuration = TimeSpan.FromMinutes(10);
        // Temporarily skips the between-stage recalibration (Baseline→Training1, Rest/Training2→next, →Retention): the
        // next stage starts on the previous calibration once the participant taps a plain Ready (no recalibration).
        // Redo still recalibrates, and the first calibration is unaffected. Recorded in RunConfig.Constants; yaw drift is no longer bounded between stages.
        private static readonly bool SkipInterStageRecalibration = true;
        // Stride-direction FPA (docs/stride_fpa_design.md): true = feedback uses each foot's own stride direction instead
        // of the pelvis-based PD (which drifted 6–10°/min under VRU, 2026-11-16 Pl89). Both FPAs are recorded either way.
        // Recorded in RunConfig.Constants.
        private static readonly bool UseStrideFpa = true;
        private const int RestDurationSec = 120; // Rest+MC window after Training1 (Research Overview §2.5)
        // Rest+MC+NASA-TLX Admin1(~5min)+IMI-PC window after Training3, before Retention.
        // ASSUMPTION: exact floor not specified by the protocol doc (only the ~5min instrument portion is) —
        // flagged for researcher confirmation.
        private const int PostBlock3RestDurationSec = 300;
        private readonly System.Diagnostics.Stopwatch _stageStopwatch = new();

        // Operator-driven visual screening (Screening button): every stone/footprint state a participant
        // will meet — angle × neutral/blue/orange. Angles use the wire convention "positive = toe-out" for
        // both feet; the AR client negates the right foot when rotating. Colors: neutral | on (blue) | off (orange).
        // Hint is operator-only (shown on the PC panel, never sent to AR): what the page shows + what to ask,
        // so the participant answers from what they see and every operator asks the same thing.
        private sealed record ScreeningPage(string Kind, float AngleL, float AngleR, string ColorL, string ColorR, string Hint);
        private static readonly ScreeningPage[] ScreeningPages =
        {
            new("EF", -10, -10, "neutral", "neutral", "EF stones · toe-in 10° · neutral color\nAsk: Do the toes of both stones point in or out?"),
            new("EF",  10,  10, "neutral", "neutral", "EF stones · toe-out 10° · neutral color\nAsk: Do the toes of both stones point in or out?"),
            new("EF", -15, -15, "on",  "off",         "EF stones · 15° · left blue / right orange\nAsk: What color is each stone (left, right)? Which way does each point?"),
            new("EF", -25, -25, "off", "on",          "EF stones · 25° · left orange / right blue\nAsk: What color is each stone (left, right)? Which way does each point? "),
            new("IF",   0,   0, "neutral", "neutral", "IF footprints · 0° · neutral color\nAsk: Are the footprints pointing straight ahead?"),
            new("IF",  15,  15, "on",  "on",          "IF footprints · +15° toe-out · blue/blue\nAsk: Pointing in or out? What color?"),
            new("IF", -15, -15, "off", "off",         "IF footprints · -15° toe-in · orange/orange\nAsk: Pointing in or out? What color?"),
            new("IF", -10,  10, "on",  "off",         "IF footprints · left -10° blue / right +10° orange\nAsk: Which way does each footprint point, and what color?"),
        };
        private bool _screeningActive = false;
        private int _screeningPage = 0;
        private string _screeningParticipantId = "";

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

            var imuList = LoadOrCreateImuConfig()
                .Select(e => new ImuViewModel(e.SlotName) { DeviceId = ParseHexDeviceId(e.DeviceId), Role = e.Role })
                .ToList();
            _slotRegistry = new ImuSlotRegistry(imuList);
            _magTimer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(350) };
            _magTimer.Tick += (_, _) => UpdateMagStatusText();
            _magTimer.Start();
            _content.ImuSlots = _slotRegistry.Imus;
            _content.ImuRoleOptions = (ImuRole[])Enum.GetValues(typeof(ImuRole));

            _deviceManager = new ImuDeviceManager(_slotRegistry);
            _deviceManager.Log += msg => Dispatcher.BeginInvoke(() => log(msg));
            _deviceManager.StateChanged += s => Dispatcher.BeginInvoke(() => OnDeviceStateChanged(s));
            _deviceManager.UpdateRatesAvailable += ev => Dispatcher.BeginInvoke(() => OnUpdateRatesAvailable(ev));
            _deviceManager.MtwConnected += ev => Dispatcher.BeginInvoke(() => OnMtwConnected(ev));
            _deviceManager.MtwDisconnected += ev => Dispatcher.BeginInvoke(() => OnMtwDisconnected(ev));
            _deviceManager.DataPacketReceived += ev => Dispatcher.BeginInvoke(() => OnDataPacket(ev));
            _deviceManager.BatteryLevelChanged += ev => Dispatcher.BeginInvoke(() => OnBattery(ev));

            _content.StatusLabel = "Ready to calibration";
            _shadow = new Protocol.ShadowMonitor(_flow, WriteShadowLine);
            _paramChangeTimer.Tick += (_, _) => FlushParamChanges();
            _imuFrameCollector.SampleRateHz = _sampleRateHz;
            UpdateImuStatusIndicators();

            foreach (var (label, markerStage) in ChecklistDefs)
                _content.FlowChecklist.Add(new Model.ChecklistItem(label, markerStage));
            for (int i = 0; i < _content.FlowChecklist.Count; i++)
            {
                _content.FlowChecklist[i].HasNext = i < _content.FlowChecklist.Count - 1;
            }

            // 流水线事件订阅
            _pipeline.OnLog += msg => Dispatcher.BeginInvoke(() => log(msg));
            _pipeline.OnCalibrationStateChanged += s =>
            {
                if (!_isReplaying) Dispatcher.BeginInvoke(() => OnCalibrationStateChanged(s));
            };
            _pipeline.OnBaselineProgress += (l, r) => Dispatcher.BeginInvoke(() =>
            {
                log($"Baseline: L={l} steps, R={r} steps");
                _content.BlockStepsDisplay = $"L {l} · R {r}";
                if (!_isReplaying)
                    BroadcastStepProgress("baseline", l, r, _pipeline.Params.BaselineStepsPerFoot);
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
                    _recorder.WriteRow(row, _currentStage, _flow.AttemptOf(_currentStage));
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
            _pipeline.UseStrideDirection = UseStrideFpa;
            _pipeline.OnStrideZeroEstimated += (foot, offsetDeg, sdDeg) => Dispatcher.BeginInvoke(() =>
            {
                string detail = $"{foot} phi={offsetDeg:F1}deg sd={sdDeg:F1}deg";
                log(sdDeg > StrideZeroEstimator.WarnSdDeg
                    ? $"WARNING: stride FPA zero {detail} — spread > {StrideZeroEstimator.WarnSdDeg:F0}° in the first strides (not walking straight, or outliers)."
                    : $"Stride FPA zero estimated: {detail}");
                if (_recorder.IsRecording) _recorder.RecordEvent("StrideZero", detail);
            });
            // Every MTw filter restart goes into the Events (the warm-up monitor only logs it), so the affected seconds can
            // be excluded afterwards. Restarts seem to come with near-saturated rotation (~2000°/s) while handling the IMUs.
            _pipeline.OnFilterRestart += (role, packetId) => Dispatcher.BeginInvoke(() =>
            {
                if (_recorder.IsRecording) _recorder.RecordEvent("FilterRestart", $"{role} at packet {packetId}");
            });
            _pipeline.OnStrideZeroInvalidated += (foot, why) => Dispatcher.BeginInvoke(() =>
            {
                log($"WARNING: stride FPA zero for {foot} invalidated: {why}. That foot gets no FPA until it is recalibrated — use Restart Condition (or a redo with recalibration).");
                if (_recorder.IsRecording) _recorder.RecordEvent("StrideZeroInvalidated", $"{foot}: {why}");
            });
            _pipeline.OnStrideZeroCrossCheck += (foot, diffDeg, usedB) => Dispatcher.BeginInvoke(() =>
            {
                string detail = float.IsNaN(diffDeg)
                    ? $"{foot} not cross-checked (pelvis-referenced zero unavailable after {FpaEngine.ZeroCrossCheckTimeoutStrides} strides), using B (stand vs walking path)"
                    : $"{foot} pelvisZero-standZero={diffDeg:F1}deg, using {(usedB ? "B (stand vs walking path)" : "A (pelvis-referenced, fallback)")}";
                log(usedB
                    ? $"Stride FPA zero cross-check: {detail}"
                    : $"WARNING: stride FPA zero cross-check {detail} — over {FpaEngine.ZeroCrossCheckMaxDeg:F0}°: the participant probably wasn't facing the walking path at calibration. FPA stays usable (plan A); for plan B, Restart Condition and calibrate facing the path.");
                if (_recorder.IsRecording) _recorder.RecordEvent("StrideZeroCheck", detail);
            });
            // IMU mounting (MountingMonitor): wrong way up at calibration → the calibration fails (the Failed case below
            // shows why); tilt shifted while walking → warning, and a big shift pauses so the operator decides
            // (Continue / Redo with recalibration / End Recording).
            _pipeline.OnMountingRejected += problem => Dispatcher.BeginInvoke(() =>
            {
                if (_isReplaying) return;
                _mountingRejection = problem;
                if (_recorder.IsRecording) _recorder.RecordEvent("MountingRejected", problem);
            });
            _pipeline.OnMountingAlert += alert => Dispatcher.BeginInvoke(() =>
            {
                if (_isReplaying) return;
                string detail = $"{alert.Role} tilt shifted {alert.ShiftDeg:F0}° from calibration";
                if (_recorder.IsRecording) _recorder.RecordEvent("MountingShift", $"{alert.Level}: {detail}");
                if (alert.Level == MountingLevel.Warn)
                {
                    log($"WARNING: {detail} (> {MountingMonitor.WarnShiftDeg:F0}°) — check that the {alert.Role} IMU hasn't slipped.");
                    return;
                }
                log($"WARNING: {detail} (> {MountingMonitor.PauseShiftDeg:F0}°) — the {alert.Role} IMU has probably slipped.");
                bool walkingStage = _recorder.IsRecording && !_content.IsPaused && !_inRest && !_retentionComplete
                                    && !_pipeline.AwaitingRecalibration && _sessionState is TestState.Baseline or TestState.Step;
                if (!walkingStage) return;
                PauseRecording();
                MessageBox.Show(
                    $"The {alert.Role} IMU tilt has shifted {alert.ShiftDeg:F0}° since calibration — it has probably slipped. " +
                    "The recording is paused.\n\n" +
                    "Check the IMU, then Resume and choose Redo (re-mount and recalibrate) or Continue, or End Recording to stop this condition.",
                    "IMU slipped", MessageBoxButton.OK, MessageBoxImage.Warning);
            });
            _pipeline.OnTrainingBlockComplete += () => Dispatcher.BeginInvoke(() =>
            {
                if (!_isReplaying && !_inRest && IsTrainingBlockStage(_currentStage))
                    EndTrainingBlock("target reached");
            });

            _timelineTimer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(60) };
            _timelineTimer.Tick += (_, _) =>
            {
                DrawTimeline();
                _content.StageElapsedDisplay = _stageStopwatch.Elapsed.ToString(@"mm\:ss");
                BtnScreening.IsEnabled = !_recorder.IsRecording; // screening is only for between conditions
                BtnStartCondition.IsEnabled = !_recorder.IsRecording; // End Recording first; Restart Condition restarts the current one

                var frameQuality = _imuFrameCollector.GenerateReport();
                _content.PelvisGapCount = frameQuality.PelvisGapFrames;
                _content.LeftGapCount = frameQuality.LeftFootGapFrames;
                _content.RightGapCount = frameQuality.RightFootGapFrames;

                // Rest window auto-advance: once the minimum duration has elapsed, move on by itself —
                // no participant/operator "Continue" tap needed, since EnterRecalPrompt() immediately arms
                // the pre-stage recalibration prompt ("Tap Ready"), which already serves as the resume
                // signal. The PC's own stopwatch is the only timer trusted for protocol-timing integrity.
                if (!_isReplaying && _inRest)
                {
                    int requiredRestSec = _restBlockJustFinished == 3 ? PostBlock3RestDurationSec : RestDurationSec;
                    if (_stageStopwatch.Elapsed >= TimeSpan.FromSeconds(requiredRestSec))
                    {
                        log($"Rest complete ({requiredRestSec}s elapsed) — advancing to recalibration.");
                        if (_shadow.Fire(Protocol.FlowEvent.RestElapsed).Accepted)
                            EnterRecalPrompt(restEndReason: "elapsed");
                    }
                }

                // Training block stall-prevention fallback: ends the block at the timeout even if 100 steps/foot wasn't reached.
                if (!_isReplaying && !_inRest && IsTrainingBlockStage(_currentStage) && _stageStopwatch.Elapsed >= TrainingBlockMaxDuration)
                    EndTrainingBlock("stall-prevention timeout");

                // Retention stall-prevention fallback: stops data collection at the timeout even if 100 steps/foot
                // wasn't reached; the operator still finishes the flow manually via End Recording after NASA-TLX Admin 2.
                if (!_isReplaying && _isRetention && !_retentionComplete && _stageStopwatch.Elapsed >= TrainingBlockMaxDuration)
                    CompleteRetention("retention stall-prevention timeout");

                RefreshChecklist();
            };
            _timelineTimer.Start();

            // 参数同步：VM 属性变化时推入 pipeline
            _content.PropertyChanged += (_, e) => SyncParamToPipeline(e.PropertyName);
            _content.PropertyChanged += (_, e) => SyncIfRenderModeFromVm(e.PropertyName);
            LoadParamsIfExists();

            StartScanAsync();
        }

        private void StartScanAsync()
        {
            Task.Run(() => _deviceManager.ScanPorts());
        }

        private void OnDeviceStateChanged(States s)
        {
            var previous = _content.DeviceState;
            _content.DeviceState = s;
            // XKF3-hm restarts with every measurement, so the warm-up starts over too.
            if (s != States.MEASURING || previous != States.MEASURING)
            {
                if (_warmup.IsComplete || _warmupSkipped) log($"Warm-up reset: device state {previous} → {s}");
                _warmup.Reset();
                _warmupSkipped = false;
            }
            if (s != previous) log($"Device state {previous} → {s}");
            _recorder.RecordMeasurementTransition(previous, s); // MeasureStart / MeasureStop while a condition is recording
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

            // Only the sensors actually bound to a role (Pelvis/Left/Right) are required to
            // start — the rest of imu_sensors.json's 8 entries are spares that may never connect.
            var requiredImus = _slotRegistry.Imus.Where(i => i.Role != ImuRole.None).ToList();
            bool allConnected = requiredImus.Count > 0 && requiredImus.All(i => i.IsConnected);
            if (allConnected && _content.DeviceState == States.MEASURING)
            {
                // StartMeasurement toggles: while MEASURING it would stop the measurement, so leave it running.
                _content.StatusLabel = ConnectedCountLabel();
                log($"IMU {ev.DeviceIdStr} reconnected; measurement already running, not toggling it.");
                if (_recorder.IsRecording) _recorder.RecordEvent("ImuReconnected", ev.DeviceIdStr);
            }
            else if (allConnected)
            {
                _content.StatusLabel = "All required IMUs connected. Starting measurement...";
                log("All required IMUs connected. Auto-starting measurement...");
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

            // A required IMU dropping mid-condition silences the pipeline (frame bundles never complete),
            // so pause and let the operator decide Continue/Redo on resume.
            if (vm != null && vm.Role != ImuRole.None && _recorder.IsRecording)
            {
                _recorder.RecordEvent("ImuDisconnected", $"{vm.Role} {ev.DeviceIdStr}");
                log($"WARNING: required IMU {vm.Role} ({ev.DeviceIdStr}) disconnected during recording.");
                if (!_content.IsPaused) PauseRecording();
            }
        }

        private string ConnectedCountLabel()
        {
            var required = _slotRegistry.Imus.Where(i => i.Role != ImuRole.None).ToList();
            return $"{required.Count(i => i.IsConnected)}/{required.Count} IMUs connected.";
        }

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
            // Closing the window mid-condition (2026-11-17 13:07 P3d) used to lose the in-memory ImuSamples: end the
            // recording the same way End Recording does, including the auto-save of the three CSVs.
            if (_recorder.IsRecording) EndFlow("application closed while recording", autoSaveCsvs: true);
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

                    case "ReadyForCalibration":
                        Dispatcher.Invoke(() => HandleReadyForCalibration("WS: participant"));
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

        // The operator log panel is also written to logs/app_<start time>.log, so a session can be analysed afterwards.
        private readonly string _appLogPath = Path.Combine(AppContext.BaseDirectory, "logs", $"app_{DateTime.Now:yyyyMMdd_HHmmss}.log");

        private void log(string log)
        {

            this.Dispatcher.BeginInvoke(() =>
            {
                string line = $"[{DateTime.Now:HH:mm:ss.fff}] {log}";
                _content.LogList += line;
                _content.LogList += (Environment.NewLine);
                try
                {
                    Directory.CreateDirectory(Path.GetDirectoryName(_appLogPath)!);
                    File.AppendAllText(_appLogPath, line + Environment.NewLine);
                }
                catch { /* logging must never disturb the experiment */ }

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
            // Event log aligns to the samples by PacketId. Taken from the raw packet path (not from diagnostics rows,
            // which don't exist yet during the very first calibration), after the pause gate above.
            _recorder.LatestPacketId = sample.PacketId;
            _imuSamples.Add(sample);
            _warmup.Update(sample);
            _magMonitor.Update(sample);
            _pipeline.ProcessPacket(sensor, deviceId, packet);
        }

        // ── 流水线状态机 ──────────────────────────────────────────────────────

        private DispatcherTimer? _baselineTimer;

        // Force-finalizes baseline (insufficient steps → HandleBaselineCompleted reports the error) if
        // BaselineMaxDuration elapses before the step thresholds are met.
        // Tracks how much of BaselineMaxDuration is left so the timeout stops counting while the operator has paused
        // (data packets are dropped during a pause, so the participant can't accumulate steps then — flow-review §2-19).
        private readonly Protocol.PausableCountdown _baselineCountdown = new();

        private void PauseBaselineTimer()
        {
            if (_baselineTimer is not { IsEnabled: true } timer) return;
            timer.Stop();
            _baselineCountdown.Pause();
        }

        private void ResumeBaselineTimer()
        {
            if (_baselineTimer == null || !_baselineCountdown.IsPaused) return;
            _baselineCountdown.Resume();
            _baselineTimer.Interval = _baselineCountdown.Remaining > TimeSpan.Zero
                ? _baselineCountdown.Remaining : TimeSpan.FromMilliseconds(1);
            _baselineTimer.Start();
        }

        private void StartBaselineTimeoutTimer()
        {
            _baselineTimer?.Stop();
            _baselineCountdown.Start(BaselineMaxDuration);
            _baselineTimer = new DispatcherTimer { Interval = BaselineMaxDuration };
            _baselineTimer.Tick += (_, _) =>
            {
                _baselineTimer?.Stop();
                log($"Baseline timed out after {BaselineMaxDuration.TotalMinutes:F0} min — force-finalizing.");
                _pipeline.FinalizeBaseline();
            };
            _baselineTimer.Start();
        }

        // The participant's "Ready" tap on the AR glasses (cmd "ReadyForCalibration") — also reachable from the
        // operator's Ready (AR) button, for when the glasses' button is hard to press. Routes to the pre-stage
        // recalibration when a stage transition is pending, otherwise to the session's first calibration.
        private void HandleReadyForCalibration(string source)
        {
            // While paused no packets reach the pipeline, so a calibration started now would just stall (T10).
            if (_content.IsPaused) { log($"{source} signaled ready while paused — ignored; resume first."); return; }
            if (_pendingStageEntry is { } readyStage && !_redoPending && SkipInterStageRecalibration)
            {
                // Plain Ready gate before Block1/Block3 (see EnterRecalPrompt): no calibration, just start the stage.
                _recorder.RecordEvent("ReadyTapped", $"before {readyStage}");
                _shadow.Fire(Protocol.FlowEvent.CalibrationStarted);
                if (_shadow.Fire(Protocol.FlowEvent.CalibrationCompleted).Accepted) EnterStage(readyStage);
                log($"{source} signaled ready — starting {readyStage}.");
            }
            else if (_pendingStageEntry != null)
            {
                if (_pipeline.TriggerRecalibration())
                    log($"{source} signaled ready — recalibration triggered before {_pendingStageEntry}.");
                else
                    log($"{source} signaled ready for recalibration, but pipeline rejected it.");
            }
            else if (_pipeline.TriggerCalibration())
                log($"{source} signaled ready — calibration triggered.");
            else
                log($"{source} signaled ready, but calibration isn't armed yet (Start Condition not open).");
        }

        private void BtnReady_Click(object sender, RoutedEventArgs e) => HandleReadyForCalibration("Operator");

        private void BroadcastArState(string state, string reason = "", int restDurationSec = 0, int? blockOverride = null)
        {
            if (_wsServer == null) return;
            string condition = ConditionForAr();
            int block = blockOverride ?? BlockForStage(_currentStage);
            var (targetL, directionL, targetR, directionR) = TargetForAr();
            _lastState = state;
            // The AR client only shows its Ready button in the "armed" state — mirror that on the operator's button.
            Dispatcher.BeginInvoke(() => BtnReady.IsEnabled = _lastState == "armed");
            _lastCondition = condition;
            _lastBlock = block;
            _lastRestDurationSec = restDurationSec;
            _lastReason = reason;
            if (state != "paused") _arStateBeforePause = (state, reason, restDurationSec, block);
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
                ifRenderMode = _ifRenderMode,
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
                ifRenderMode = _ifRenderMode,
                ts = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds(),
            });
            // Mid-screening reconnect: the snapshot above would end screening on the client — put the page back.
            if (_screeningActive) _ = _wsServer.SendJsonAsync(id, ScreeningPayload());
        }

        // EF stepping-stone rotation needs the participant's fixed personalised target angle/direction
        // (Research Overview §2.3/2.4); it's constant for the whole condition once baseline completes,
        // so it's read live from BaselineProfile rather than latched like the rest-only fields above.
        // Sentinel defaults (0f/"") before baseline completes, per the no-nullable-fields design constraint.
        private (float targetL, string directionL, float targetR, string directionR) TargetForAr()
        {
            var bp = _pipeline.BaselineProfile;
            if (bp == null) return (0f, "", 0f, "");
            // AR rotates the stone by (side == "toe-in" ? -|target| : |target|), i.e. it expects a magnitude plus a side.
            // Target_L/R are signed FPA (negative = toe-in), so send |target| and derive the side from its sign
            // (not from the training direction: a toe-out target for a very toed-in person can still be negative).
            return (MathF.Abs(bp.Target_L), SideForAr(bp.Target_L), MathF.Abs(bp.Target_R), SideForAr(bp.Target_R));
        }

        private static string SideForAr(float signedTarget) => signedTarget < 0f ? "toe-in" : "toe-out";

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

        private bool _warmupSkipped;          // operator pressed Skip Warm-up for this measurement
        private bool _warmupWasComplete;      // for logging the done transition
        private DateTime _warmupProgressLoggedAt;

        // First status-bar line: per-sensor warm-up progress, plus log lines for done / voided and every 5 s while waiting.
        private void UpdateWarmupStatus(TextBlock tb)
        {
            var w = _warmup.GetSnapshot();
            static string Sec(double? s) => s is double v ? $"{System.Math.Min(v, WarmupMonitor.RequiredSeconds):F0}" : "–";
            string perSensor = $"P {Sec(w.PelvisSec)} · L {Sec(w.LeftSec)} · R {Sec(w.RightSec)} /{WarmupMonitor.RequiredSeconds:F0} s";

            string text; Brush color;
            if (_content.DeviceState != States.MEASURING) { text = $"Warm-up: not measuring ({_content.DeviceState})"; color = Brushes.Gray; }
            else if (w.IsComplete) { text = "Warm-up done"; color = Brushes.ForestGreen; }
            else if (_warmupSkipped) { text = $"Warm-up SKIPPED by operator ({perSensor})"; color = Brushes.Crimson; }
            else { text = $"Warm-up {perSensor} — keep IMUs still (– = no valid filter data)"; color = Brushes.DarkOrange; }
            tb.Inlines.Add(new System.Windows.Documents.Run(text) { Foreground = color });
            tb.Inlines.Add(new System.Windows.Documents.LineBreak());

            if (w.IsComplete && !_warmupWasComplete) log($"Warm-up done (packet {w.CompletedPacketId}).");
            foreach (var note in _warmup.TakeRestartNotes()) log($"Warm-up reset: {note}");
            _warmupWasComplete = w.IsComplete;
            if (_content.DeviceState == States.MEASURING && !w.IsComplete && DateTime.Now - _warmupProgressLoggedAt > TimeSpan.FromSeconds(5))
            {
                _warmupProgressLoggedAt = DateTime.Now;
                log($"Warm-up progress: {perSensor}");
            }
        }

        private void BtnSkipWarmup_Click(object sender, RoutedEventArgs e)
        {
            if (_content.DeviceState != States.MEASURING) { log("Skip Warm-up: IMUs are not measuring."); return; }
            if (_warmup.IsComplete) { log("Skip Warm-up: warm-up is already done."); return; }
            var ok = MessageBox.Show(
                "Skip the IMU filter warm-up?\n\nThe calibration may then use a filter that hasn't settled yet. " +
                "This is recorded in the Events (WarmupSkipped) and RunConfig.",
                "Skip warm-up", MessageBoxButton.YesNo, MessageBoxImage.Warning);
            if (ok != MessageBoxResult.Yes) return;
            _warmupSkipped = true;
            log($"Warm-up skipped by operator ({_warmup.GetSnapshot().StillSeconds:F1} s still of {WarmupMonitor.RequiredSeconds:F0} s).");
        }

        // Live mag / heading-consistency text in the status bar (display only).
        private void UpdateMagStatusText()
        {
            var snap = _magMonitor.GetSnapshot();
            var tb = MagStatusText;
            tb.Inlines.Clear();

            UpdateWarmupStatus(tb);

            static Brush Pick(MagLevel l) => l switch
            {
                MagLevel.Ok => Brushes.ForestGreen,
                MagLevel.Warn => Brushes.DarkOrange,
                MagLevel.Bad => Brushes.Crimson,
                _ => Brushes.Gray,
            };
            void Line(string name, MagSensorSnapshot s)
            {
                string text = !s.HasData ? $"{name}: no mag data"
                    : !s.HasBaseline ? $"{name}: |B| {s.Norm:F2}  dip {s.DipDeg:F0}° (no baseline)"
                    : $"{name}: |B| {s.Norm:F2} ({s.NormDevPct:+0;-0;0}%)  dip {s.DipDeg:F0}° ({s.DipDevDeg:+0;-0;0}°)";
                tb.Inlines.Add(new System.Windows.Documents.Run(text) { Foreground = Pick(s.Level) });
                tb.Inlines.Add(new System.Windows.Documents.LineBreak());
            }
            Line("Pel", snap.Pelvis);
            Line("L", snap.Left);
            Line("R", snap.Right);
            string heading = snap.HasHeading ? $"Pel-Feet: {snap.PelvisMinusFeetDeg:+0.0;-0.0;0.0}°" : "Pel-Feet: n/a";
            tb.Inlines.Add(new System.Windows.Documents.Run(heading) { Foreground = Pick(snap.HeadingLevel) });
        }

        private void OnCalibrationStateChanged(CalibrationState state)
        {
            if (state == CalibrationState.CollectingStaticPose) _magMonitor.Reset();
            else if (state == CalibrationState.Completed && _pipeline.CalibrationProfile is { } cp)
                _magMonitor.StartBaseline(cp.PelvisRef, cp.LeftFootRef, cp.RightFootRef);

            // Read what we were waiting for BEFORE feeding the event: the state machine moves on the moment it is fired.
            string? pending = _pendingStageEntry;
            bool redo = _redoPending;
            string calDetail = pending == null ? "initial"
                : redo ? $"recalibration for redo of {pending}"
                : $"recalibration before {pending}";
            Protocol.FlowEvent? flowEvent = state switch
            {
                CalibrationState.CollectingStaticPose => Protocol.FlowEvent.CalibrationStarted,
                CalibrationState.Completed => Protocol.FlowEvent.CalibrationCompleted,
                CalibrationState.Failed => Protocol.FlowEvent.CalibrationFailed,
                _ => null,
            };
            if (flowEvent is { } fe)
            {
                // A calibration callback that doesn't fit the current phase (e.g. one that was queued before a
                // restart) must not act on the experiment.
                if (!_shadow.Fire(fe).Accepted) return;
                _recorder.RecordEvent(state switch
                {
                    CalibrationState.CollectingStaticPose => "CalibrationStart",
                    CalibrationState.Completed => "CalibrationDone",
                    _ => "CalibrationFailed",
                }, calDetail);
                // Normally impossible (Start Condition waits for warm-up), but a measurement restarted mid-condition
                // resets it; don't block the experiment, just mark the calibration for later review.
                if (state == CalibrationState.CollectingStaticPose && !_warmup.IsComplete)
                    _recorder.RecordEvent("WarmupIncomplete", $"{_warmup.GetSnapshot().StillSeconds:F1}s still of {WarmupMonitor.RequiredSeconds:F0}s");
            }

            switch (state)
            {
                case CalibrationState.WaitingForStart:
                    BroadcastArState("waiting");
                    break;

                case CalibrationState.CollectingStaticPose:
                    _sessionState = TestState.Calibrating;
                    if (pending != null)
                    {
                        _content.StatusLabel = $"Recalibrating before {pending}... Stand still.";
                        BroadcastArState("calibrating", reason: "Quick recheck — please stand still for a moment before continuing.");
                    }
                    else
                    {
                        _content.StatusLabel = "Calibrating... Stand still.";
                        BroadcastArState("calibrating");
                    }
                    break;

                case CalibrationState.Completed:
                    if (pending is { } nextStage)
                    {
                        EnterStage(nextStage, redo);
                    }
                    else
                    {
                        _content.StatusLabel = "Calibration done. Starting baseline walk.";
                        _content.CalibrationState = "Calibrated";
                        _sessionState = TestState.Calibrated;
                        _pipeline.StartBaseline();
                        BroadcastArState("baseline");
                        _sessionState = TestState.Baseline;
                        _content.StatusLabel = $"Baseline: Walk until both feet reach {_pipeline.Params.BaselineStepsPerFoot} steps.";
                        StartBaselineTimeoutTimer();
                    }
                    break;

                case CalibrationState.Failed when _mountingRejection is { } problem:
                    // Rejected because an IMU is mounted the wrong way up (not because the participant moved).
                    _mountingRejection = null;
                    if (pending == null) _sessionState = TestState.Launching;
                    _content.StatusLabel = $"Calibration rejected — IMU mounted wrong: {problem}. Fix it, then tap Ready to retry.";
                    BroadcastArState("armed", reason: "Please wait — the operator is adjusting a sensor.\nTap Ready when asked.");
                    MessageBox.Show($"Calibration rejected — IMU mounted the wrong way:\n\n{problem.Replace("; ", "\n")}\n\n" +
                                    "Pelvis IMU: X axis up. Foot IMUs: Z axis up. Fix the IMU, then tap Ready to calibrate again.",
                                    "IMU mounting", MessageBoxButton.OK, MessageBoxImage.Warning);
                    break;

                case CalibrationState.Failed:
                    if (pending != null)
                    {
                        _content.StatusLabel = $"Recalibration before {pending} failed — please stand still and tap Ready to retry.";
                        BroadcastArState("armed", reason: "Calibration didn't complete.\nStand still and tap Ready to try again.");
                    }
                    else
                    {
                        // Retry is unlimited (flow-review O5); if it keeps failing the operator decides whether to restart.
                        _content.StatusLabel = "Calibration failed — please stand still and tap Ready to retry.";
                        _sessionState = TestState.Launching;
                        BroadcastArState("armed", reason: "Calibration didn't complete.\nStand still and tap Ready to try again.");
                    }
                    break;
            }
        }

        private void HandleBaselineCompleted(BaselineProfile? profile)
        {
            _baselineTimer?.Stop();
            // Only meaningful while the baseline is actually being collected (not for a stale/duplicate completion).
            if (!_shadow.Fire(Protocol.FlowEvent.BaselineCompleted(profile is { IsValid: true })).Accepted) return;
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
            _recorder.SaveBaseline(profile);

            // The state machine has already moved Baseline -> (recalibrating before) Training1; this performs the side
            // effects: close the Baseline stage and ask the participant for Ready. StatusLabel is NOT set to
            // "Training started..." here - that message belongs in EnterStage() once training actually begins, not
            // before the participant has even stood still for recalibration.
            EnterRecalPrompt();
        }

        // ── 参数同步 ──────────────────────────────────────────────────────────

        // Every Parameters change while a condition is recording is written to the event log (flow-review Q6), so the
        // data can be explained afterwards. Sliders fire PropertyChanged for every tick of a drag, so changes are
        // coalesced: ~0.8 s after the last change one ParamChange per parameter is written (value before the burst ->
        // value after it).
        private static readonly HashSet<string> PipelineParamNames =
            typeof(PipelineParams).GetProperties().Select(p => p.Name).ToHashSet();
        private readonly DispatcherTimer _paramChangeTimer = new() { Interval = TimeSpan.FromMilliseconds(800) };
        private PipelineParams? _paramsAtLastRecord;

        private void FlushParamChanges()
        {
            _paramChangeTimer.Stop();
            if (!_recorder.IsRecording || _paramsAtLastRecord == null) return;
            foreach (var change in PipelineParams.Diff(_paramsAtLastRecord, _pipeline.Params))
                _recorder.RecordEvent("ParamChange", change.Describe());
            _paramsAtLastRecord = _pipeline.Params.Clone();
        }

        private void SyncParamToPipeline(string? propName)
        {
            if (_recorder.IsRecording && propName != null && PipelineParamNames.Contains(propName))
            {
                _paramChangeTimer.Stop();   // restart the quiet-period countdown
                _paramChangeTimer.Start();
            }
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
                case nameof(MainPageVM.TargetStepsPerFoot):
                    _pipeline.Params.TargetStepsPerFoot = _content.TargetStepsPerFoot; break;
                case nameof(MainPageVM.BaselineStepsPerFoot):
                    _pipeline.Params.BaselineStepsPerFoot = _content.BaselineStepsPerFoot; break;
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
            _content.TargetStepsPerFoot = p.TargetStepsPerFoot;
            _content.BaselineStepsPerFoot = p.BaselineStepsPerFoot;
            _content.BaselineImbalanceRatioThreshold = p.BaselineImbalanceRatioThreshold;
            _pipeline.Params.StanceFootPitchThreshold = p.StanceFootPitchThreshold;
        }

        private void BtnSaveParams_Click(object sender, RoutedEventArgs e)
        {
            var json = JsonSerializer.Serialize(_pipeline.Params, new JsonSerializerOptions { WriteIndented = true });
            File.WriteAllText(ParamsFilePath, json, Encoding.UTF8);
            SaveImuConfig();
            log("Parameters saved.");
        }

        private static readonly string ImuConfigFilePath =
            Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "imu_sensors.json");
        private static readonly JsonSerializerOptions ImuConfigJsonOptions = new() { WriteIndented = true };

        private static uint ParseHexDeviceId(string hex)
            => Convert.ToUInt32(hex.StartsWith("0x", StringComparison.OrdinalIgnoreCase) ? hex[2..] : hex, 16);

        /// <summary>
        /// Loads imu_sensors.json. If missing, seeds it with the 3 sensors that used to be
        /// hardcoded here, so existing setups keep working until the user edits in the other 5.
        /// </summary>
        private List<ImuSensorConfigEntry> LoadOrCreateImuConfig()
        {
            if (!File.Exists(ImuConfigFilePath))
            {
                var seed = new List<ImuSensorConfigEntry>
                {
                    new() { SlotName = "IMU-1", DeviceId = "0x00B43CAB", Role = ImuRole.Pelvis },
                    new() { SlotName = "IMU-2", DeviceId = "0x10B41913", Role = ImuRole.Left },
                    new() { SlotName = "IMU-3", DeviceId = "0x10B41904", Role = ImuRole.Right },
                };
                File.WriteAllText(ImuConfigFilePath, JsonSerializer.Serialize(seed, ImuConfigJsonOptions), Encoding.UTF8);
                return seed;
            }

            var json = File.ReadAllText(ImuConfigFilePath, Encoding.UTF8);
            return JsonSerializer.Deserialize<List<ImuSensorConfigEntry>>(json) ?? new List<ImuSensorConfigEntry>();
        }

        private void SaveImuConfig()
        {
            var entries = _slotRegistry.Imus.Select(imu => new ImuSensorConfigEntry
            {
                SlotName = imu.SlotName,
                DeviceId = "0x" + imu.DeviceId.ToString("X8"),
                Role = imu.Role,
            }).ToList();
            File.WriteAllText(ImuConfigFilePath, JsonSerializer.Serialize(entries, ImuConfigJsonOptions), Encoding.UTF8);
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

        // "Restart Condition" (formerly "Start Calibration"): something went wrong, start this condition over
        // (flow-review Q3 / §7.8). The data so far is archived, never deleted, and the operator is asked first.
        private void RestartCondition()
        {
            string participantId = _recorder.ParticipantId;
            string condition = _recorder.Condition;
            var confirm = MessageBox.Show(
                $"Restart P{participantId} {condition} from the beginning?\n\n" +
                "The data recorded so far is archived (not deleted) in the participant's Aborted folder, " +
                "and the experiment starts over at calibration.",
                "Restart condition", MessageBoxButton.YesNo, MessageBoxImage.Warning);
            if (confirm != MessageBoxResult.Yes) { log("Restart cancelled."); return; }

            _recorder.RecordEvent("Abort", "operator restarted the condition");
            EndFlow("restarted by operator");
            string? archived = _recorder.ArchiveCurrentFiles();
            log($"Previous data archived → {archived}");

            // Clear the leftovers the old Start Calibration button always cleared.
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

            BeginCondition(participantId, condition);
        }

        private void BtnStartCal_Click(object sender, RoutedEventArgs e)
        {
            if (_recorder.IsRecording) { RestartCondition(); return; }
            log("Restart Condition: no condition is recording — use Start Condition.");
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

                // Pause / recalibration / marker / stage events of the current condition, next to the diagnostics CSV;
                // aligned to its (and the Session CSV's) PacketId column.
                if (_recorder.Events.Count > 0)
                {
                    var eventsPath = Path.Combine(Path.GetDirectoryName(dlg.FileName) ?? "",
                        Path.GetFileNameWithoutExtension(dlg.FileName) + "_Events.csv");
                    _recorder.WriteEventsCsv(eventsPath);
                    log($"Events saved: {_recorder.Events.Count} events → {eventsPath}");
                }
            }
            catch (Exception ex)
            {
                log($"Error saving diagnostics: {ex.Message}");
            }
        }

        private void BtnOpenDataFolder_Click(object sender, RoutedEventArgs e)
        {
            var sessionPath = _recorder.CurrentSessionFilePath;
            if (string.IsNullOrEmpty(sessionPath))
            {
                log("Open Data Folder: no condition started yet.");
                return;
            }
            var dir = Path.GetDirectoryName(sessionPath);
            if (dir == null || !Directory.Exists(dir))
            {
                log($"Open Data Folder: folder not found ({dir}).");
                return;
            }
            System.Diagnostics.Process.Start("explorer.exe", dir);
        }

        // Reacts to the ParamsDialog checkbox (bound to MainPageVM.IsIfLastResultMode) via _content.PropertyChanged —
        // switches the AR client's IF rendering between last-result-sticky and placeholder. Takes effect immediately,
        // no pause/reconnect needed: updates _ifRenderMode and re-broadcasts the current state right away so a connected
        // client picks it up without waiting for the next natural state transition.
        private void SyncIfRenderModeFromVm(string? propName)
        {
            if (propName != nameof(MainPageVM.IsIfLastResultMode)) return;

            var previousIfRenderMode = _ifRenderMode;
            _ifRenderMode = _content.IsIfLastResultMode ? "lastResult" : "placeholder";
            log($"[IF] Render mode switched to '{_ifRenderMode}'.");
            if (_ifRenderMode != previousIfRenderMode)
                _recorder.RecordEvent("ParamChange", new ParamChange("IfRenderMode", previousIfRenderMode, _ifRenderMode).Describe());
            BroadcastArState(_lastState, _lastReason, _lastRestDurationSec, _lastBlock);
        }

        // ── 实验会话：Participant/Condition/Stage → 文件 ─────────────────────────

        // ── Visual screening ─────────────────────────────────────────────────────────
        // Operator shows the AR client every stone/footprint state, asks the participant what they see, then
        // marks Pass/Fail (saved per participant; Start Condition refuses unscreened or failed participants).
        // Works at any time: finishing/cancelling re-broadcasts the pre-screening state, so the flow resumes.

        private void BtnScreening_Click(object sender, RoutedEventArgs e)
        {
            if (_screeningActive) { EndScreening(); log("Visual screening cancelled (no result saved)."); return; }
            if (_recorder.IsRecording) { log("Screening is only available between conditions — End Recording first."); return; }

            string participantId = txtParticipantId.Text.Trim();
            if (string.IsNullOrEmpty(participantId)) { log("Enter Participant ID first."); return; }

            _screeningActive = true;
            _screeningParticipantId = participantId;
            _screeningPage = 0;
            ScreeningPanel.Visibility = Visibility.Visible;
            TxtScreeningHint.Visibility = Visibility.Visible;
            log($"Visual screening started for P{participantId}.");
            ShowScreeningPage();
        }

        private void BtnScreeningPrev_Click(object sender, RoutedEventArgs e) => MoveScreeningPage(-1);
        private void BtnScreeningNext_Click(object sender, RoutedEventArgs e) => MoveScreeningPage(1);
        private void BtnScreeningPass_Click(object sender, RoutedEventArgs e) => FinishScreening(true);
        private void BtnScreeningFail_Click(object sender, RoutedEventArgs e) => FinishScreening(false);

        private void MoveScreeningPage(int delta)
        {
            if (!_screeningActive) return;
            _screeningPage = Math.Clamp(_screeningPage + delta, 0, ScreeningPages.Length - 1);
            ShowScreeningPage();
        }

        private void ShowScreeningPage()
        {
            TxtScreeningPage.Text = $"{_screeningPage + 1} / {ScreeningPages.Length}";
            TxtScreeningHint.Text = ScreeningPages[_screeningPage].Hint;
            if (_wsServer == null) return;
            _ = _wsServer.BroadcastJsonAsync(ScreeningPayload());
            LogArEvent($"type=screening  page={_screeningPage + 1}/{ScreeningPages.Length}  kind={ScreeningPages[_screeningPage].Kind}");
        }

        private object ScreeningPayload()
        {
            var p = ScreeningPages[_screeningPage];
            return new
            {
                type = "screening",
                screenPage = _screeningPage + 1,
                screenTotal = ScreeningPages.Length,
                screenKind = p.Kind,
                screenAngleL = p.AngleL,
                screenAngleR = p.AngleR,
                screenColorL = p.ColorL,
                screenColorR = p.ColorR,
                ts = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds(),
            };
        }

        private void FinishScreening(bool passed)
        {
            if (!_screeningActive) return;
            Methods.ExperimentRecorder.SaveScreeningResult(_screeningParticipantId, passed);
            log($"Visual screening for P{_screeningParticipantId}: {(passed ? "PASS" : "FAIL")}.");
            EndScreening();
        }

        private void EndScreening()
        {
            _screeningActive = false;
            ScreeningPanel.Visibility = Visibility.Collapsed;
            TxtScreeningHint.Visibility = Visibility.Collapsed;
            // The AR client leaves screening on any state message — restore whatever it was showing before.
            BroadcastArState(_lastState, _lastReason, _lastRestDurationSec, _lastBlock);
        }

        private static readonly string ShadowLogPath =
            Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Data", "shadow_flow.log");

        private void WriteShadowLine(string line)
        {
            try
            {
                Directory.CreateDirectory(Path.GetDirectoryName(ShadowLogPath)!);
                File.AppendAllText(ShadowLogPath, $"{DateTime.UtcNow:O} {line}{Environment.NewLine}");
            }
            catch { /* diagnostics must never disturb the experiment */ }
            log(line);
        }

        // Snapshot of everything that shapes the collected data but isn't visible in the CSVs: software version
        // (includes the git commit), all PipelineParams, and the protocol constants hard-coded in this file/pipeline.
        // Params are copied so later live edits don't rewrite the snapshot.
        private RunConfig BuildRunConfig() => new()
        {
            AppVersion = System.Reflection.CustomAttributeExtensions
                .GetCustomAttribute<System.Reflection.AssemblyInformationalVersionAttribute>(typeof(MainWindow).Assembly)
                ?.InformationalVersion ?? "",
            Params = JsonSerializer.Deserialize<PipelineParams>(JsonSerializer.Serialize(_pipeline.Params))!,
            Constants = new Dictionary<string, object>
            {
                ["RestDurationSec"] = RestDurationSec,
                ["PostBlock3RestDurationSec"] = PostBlock3RestDurationSec,
                ["TrainingBlockMaxDurationMin"] = TrainingBlockMaxDuration.TotalMinutes,
                ["BaselineMaxDurationMin"] = BaselineMaxDuration.TotalMinutes,
                ["SkipInterStageRecalibration"] = SkipInterStageRecalibration,
                ["UseVruFilterProfile"] = IMUMoCap.Services.ImuDeviceManager.UseVruFilterProfile,
                ["ToleranceDeg"] = IMUMoCap.Pipeline.Models.BaselineProfile.ToleranceDeg,
                ["TargetDeltaDeg"] = IMUMoCap.Pipeline.Models.BaselineProfile.TargetDeltaDeg,
                ["TargetMinDeg"] = IMUMoCap.Pipeline.Models.BaselineProfile.TargetMinDeg,
                ["IfRenderMode"] = _ifRenderMode,
                // The pipeline's frame-count thresholds are written for AssumedSampleRateHz; if the IMUs actually run at
                // a different UpdateRateHz, every "N frames" threshold lasts a different amount of time (flow-review §2-15).
                ["UpdateRateHz"] = CurrentUpdateRateHz(),
                ["SupportedUpdateRatesHz"] = string.Join(",", _content.UpdateRates),
                ["AssumedSampleRateHz"] = _sampleRateHz,
                ["StanceDebounceMs"] = GaitPipeline.StanceDebounceMs,
                ["SettleMinStanceMs"] = GaitPipeline.SettleMinStanceMs,
                ["UseStrideFpa"] = UseStrideFpa,
                ["StrideZeroMethod"] = "B (stand heading - first straight strides) if within ZeroCrossCheckMaxDeg of A, else A",
                ["StrideZeroCrossCheckMaxDeg"] = FpaEngine.ZeroCrossCheckMaxDeg,
                ["StrideZeroCrossCheckTimeoutStrides"] = FpaEngine.ZeroCrossCheckTimeoutStrides,
                ["StrideMaxMissingPackets"] = StrideGate.MaxMissingPackets,
                ["WarmupRequiredSec"] = WarmupMonitor.RequiredSeconds,
                ["WarmupGyroThresholdDegS"] = WarmupMonitor.GyroThresholdRadS * 180f / MathF.PI,
                ["WarmupAccThreshold"] = WarmupMonitor.AccThreshold,
                ["MountingPelvisMaxTiltDeg"] = MountingMonitor.PelvisMaxTiltDeg,
                ["MountingFootMaxTiltDeg"] = MountingMonitor.FootMaxTiltDeg,
                ["MountingWarnShiftDeg"] = MountingMonitor.WarnShiftDeg,
                ["MountingPauseShiftDeg"] = MountingMonitor.PauseShiftDeg,
                // Seconds from measurement start to calibration = (CalibrationStart PacketId in Events − this) / rate.
                ["MeasurementStartPacketId"] = _warmup.GetSnapshot().MeasurementStartPacketId ?? -1,
                ["WarmupDonePacketId"] = _warmup.GetSnapshot().CompletedPacketId ?? -1,
                ["WarmupSkipped"] = _warmupSkipped && !_warmup.IsComplete,
            },
        };

        // Rate the IMUs were told to measure at (the app chooses it; there is no UI for it). -1 if unknown.
        private int CurrentUpdateRateHz() =>
            _content.SelectedRate >= 0 && _content.SelectedRate < _content.UpdateRates.Count
            && int.TryParse(_content.UpdateRates[_content.SelectedRate], out var hz) ? hz : -1;

        private void BtnStartCondition_Click(object sender, RoutedEventArgs e)
        {
            // A new condition while one is recording would mix the two conditions in one recording (an IF file held EF data).
            if (_recorder.IsRecording) { log("Start Condition blocked: a condition is still recording. End Recording first (or Restart Condition)."); return; }
            string participantId = txtParticipantId.Text.Trim();
            if (string.IsNullOrEmpty(participantId)) { log("Enter Participant ID first."); return; }
            if (_content.DeviceState != States.MEASURING) { log("Start Condition blocked: IMUs are not measuring. Click Measure first."); return; }
            if (!_warmup.IsComplete && !_warmupSkipped)
            {
                var w = _warmup.GetSnapshot();
                log($"Start Condition blocked: IMU filter warm-up {w.StillSeconds:F0}/{WarmupMonitor.RequiredSeconds:F0} s. " +
                    "Leave all IMUs still on the table until the status bar shows \"Warm-up done\" (or use Skip Warm-up).");
                return;
            }

            if (_screeningActive) { log("Finish the visual screening (Pass/Fail) before Start Condition."); return; }
            string? screening = Methods.ExperimentRecorder.GetScreeningResult(participantId);
            if (screening != "Pass")
            {
                string why = screening == "Fail"
                    ? $"P{participantId} failed the visual screening and cannot take part."
                    : $"P{participantId} has no visual screening result yet. Click Screening and mark Pass/Fail first.";
                log($"Start Condition blocked: {why}");
                MessageBox.Show(why, "Visual screening required", MessageBoxButton.OK, MessageBoxImage.Warning);
                return;
            }

            string condition = (cmbCondition.SelectedItem as ComboBoxItem)?.Content?.ToString() ?? "EF";

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

            BeginCondition(participantId, condition);
        }

        // Everything that starts a condition once BtnStartCondition_Click's checks have passed. Shared with
        // RestartCondition so there is a single reset list instead of two that can drift apart.
        private void BeginCondition(string participantId, string condition)
        {
            // A pause must not carry over into the new recording: while IsPaused, no data rows are written.
            _content.IsPaused = false;
            BtnPauseResume.Content = "Pause";
            _autoSaveStamp = null; // this condition's auto-saved CSVs get a fresh stamp
            _recorder.StartCondition(participantId, condition);
            if (_warmupSkipped && !_warmup.IsComplete)
                _recorder.RecordEvent("WarmupSkipped", $"{_warmup.GetSnapshot().StillSeconds:F1}s still of {WarmupMonitor.RequiredSeconds:F0}s");
            _shadow.Fire(Protocol.FlowEvent.StartCondition);
            _recorder.SetRunConfig(BuildRunConfig());
            _paramsAtLastRecord = _pipeline.Params.Clone(); // later changes are reported relative to this snapshot
            int updateRateHz = CurrentUpdateRateHz();
            _pipeline.SampleRateHz = updateRateHz > 0 ? updateRateHz : _sampleRateHz; // ms-defined stance thresholds follow the real rate
            if (updateRateHz > 0 && updateRateHz != _sampleRateHz)
                log($"WARNING: IMUs are measuring at {updateRateHz} Hz, but the pipeline's frame-count thresholds assume {_sampleRateHz} Hz.");
            // Clears BaselineProfile and all per-module internal state (gait/motion/PD/FPA) left over
            // from whatever ran before — without this, RefreshChecklist() reads the PREVIOUS
            // participant/condition's BaselineProfile and shows Baseline as already done.
            _pipeline.Reset();
            _pipeline.CalibrationArmed = true;
            _stageStopwatch.Restart();
            _recorder.BeginStage(_currentStage);
            RefreshChecklist();

            _content.CurrentStageLabel = _currentStage;
            _content.CurrentSessionFileName = _recorder.CurrentSessionFilePath ?? "";
            _content.TurningExclusionDisplay = "";
            _content.BlockStepsDisplay = "L 0 · R 0";
            BroadcastArState("armed"); // tells the AR client it can show its "ready to calibrate" button
            log($"Started condition {condition} → {_recorder.CurrentSessionFilePath}");
        }

        private void BtnNextStage_Click(object sender, RoutedEventArgs e)
        {
            if (!_recorder.IsRecording) { log("No active condition recording."); return; }

            bool wasResting = _inRest; // read before firing: the state machine leaves Rest the moment it is fired
            var result = _shadow.Fire(Protocol.FlowEvent.NextStage);
            if (!result.Accepted) { log($"Next Stage ignored — {result.Reason}."); return; }

            EnterRecalPrompt(restEndReason: wasResting ? "skipped by Next Stage" : null);
        }

        private static bool IsTrainingBlockStage(string stage) =>
            stage is "Training1" or "Training2" or "Training3";

        // A training block finished (100 steps/foot or the stall-prevention timeout). Training1 and Training3
        // each go to a Rest+MC window first (Research Overview §2.5); Training2 advances straight to Training3
        // (no rest/MC between Block2 and Block3 by design).
        // Guards against a duplicate block-completion event for the same block: the 100-steps/foot
        // pipeline event is queued via Dispatcher.BeginInvoke (background thread), while the timeout
        // fires synchronously on the UI thread's DispatcherTimer.Tick — if both conditions are
        // met near-simultaneously, the queued event can still run after the timer has already advanced
        // the stage, which would otherwise end the *next* block early. Reset whenever a new training
        // block is entered (EnterStage, in the trainingBlock > 0 branch).
        private bool _trainingBlockEnding = false;

        private void EndTrainingBlock(string reason)
        {
            if (_trainingBlockEnding) return; // stale duplicate completion event for this block — ignore
            if (!_shadow.Fire(Protocol.FlowEvent.BlockComplete).Accepted) return; // no training block is running
            _trainingBlockEnding = true;
            // The only marker of where the block's data ends: from here on rows keep the training stage's tag
            // (Rest is not a stage), so without this event Rest-time samples can't be told apart from training.
            string follows = _currentStage switch
            {
                "Training1" => $"rest {RestDurationSec}s follows",
                "Training3" => $"rest {PostBlock3RestDurationSec}s follows",
                _ => "no rest, recalibration follows",
            };
            _recorder.RecordEvent("BlockComplete",
                $"{reason}; steps L/R={_pipeline.TrainingStepsL}/{_pipeline.TrainingStepsR}; {follows}");

            switch (_currentStage)
            {
                case "Training1":
                    EnterRest(blockJustFinished: 1, durationSec: RestDurationSec);
                    break;
                case "Training3":
                    EnterRest(blockJustFinished: 3, durationSec: PostBlock3RestDurationSec);
                    break;
                default: // Training2 → Training3: no rest/MC window by design
                    EnterRecalPrompt();
                    break;
            }
        }

        // Which training block the current rest window follows (1 or 3) — decides the required rest
        // duration (_timelineTimer's auto-advance check below) and which block number is broadcast
        // to the AR client.
        private int _restBlockJustFinished => _flow.RestBlockJustFinished;

        // Enters the Rest+MC window after Training1 or Training3. Stays in the just-finished training stage
        // (rest is not a data stage). _stageStopwatch is restarted so the server can enforce the minimum
        // duration; _timelineTimer's tick auto-advances (via EnterRecalPrompt(), which also arms the
        // pre-stage recalibration prompt) once that duration has elapsed — no separate participant/operator
        // "Continue" tap is needed, since the recalibration "Tap Ready" prompt that follows already serves
        // as the resume signal.
        // MC (Manipulation Check) is a real RA-administered questionnaire, same as NASA-TLX/IMI-PC — it is
        // NOT auto-logged; the operator marks it by clicking its Flow label once the RA actually
        // administers it (see FlowItem_MouseLeftButtonUp).
        private void EnterRest(int blockJustFinished, int durationSec)
        {
            _content.CurrentStageLabel = $"{_currentStage} (Resting)";
            _stageStopwatch.Restart();
            BroadcastArState("rest", restDurationSec: durationSec, blockOverride: blockJustFinished);
            log($"Rest started ({durationSec}s) after Training{blockJustFinished} — auto-advances to recalibration when elapsed. Manipulation Check due.");
        }

        // Pure audit-trail markers for RA-administered instruments (paper/RA-administered, not rendered by
        // this app) — just timestamps the moment into the Meta JSON so it doesn't build questionnaire UI.
        // The five "was it clicked" flags below back the End Recording completeness check (see BtnEndCondition_Click)
        // — they only ever get cleared in BtnStartCondition_Click, never on stage advance/pause/redo, since a
        // marker represents a real paper-instrument event that stays true regardless of later data-collection redos.

        // Single handler for every markable (underlined) label in the Flow checklist (see MainWindow.xaml's
        // ItemsControl.ItemTemplate) — which marker it logs comes from the clicked row's own MarkerStage,
        // read off the TextBlock's DataContext (the bound ChecklistItem), not from separate named handlers.
        // Ignored unless the row's marker window is open (CanMark), matching what the underline advertises.
        private void FlowItem_MouseLeftButtonUp(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            if ((sender as FrameworkElement)?.DataContext is not Model.ChecklistItem { MarkerStage: { } stage, CanMark: true }) return;

            bool late = _flow.IsMarkLate(stage); // past its window (e.g. the Rest was skipped): flagged, not refused
            if (!_shadow.Fire(Protocol.FlowEvent.Mark(stage)).Accepted) return; // not due yet, or already added late
            _recorder.MarkStageEvent(stage, late);
            log(late ? $"Marker logged (late): {stage}" : $"Marker logged: {stage}");
            RefreshChecklist(); // instant feedback instead of waiting for the next 60ms timer tick
        }

        // The stage we are about to enter once the recalibration in progress completes (null otherwise), and whether
        // that recalibration is for redoing the stage being collected: both are views of ProtocolFlow.
        private string? _pendingStageEntry => _flow.PendingStageEntry;
        private bool _redoPending => _flow.IsRedoRecalibration;

        /// <summary>
        /// Redo (flow-review T9): restart the stage that is currently being collected. It goes through the same
        /// "recalibrate, then enter the stage" gate as a normal stage change — EnterStage then clears that stage's
        /// counters and timers, so steps from the abandoned attempt can no longer count towards its target
        /// (before this, a Baseline redo finished after 3 s with the 8+7 steps of the discarded attempt).
        /// </summary>
        private void BeginRedoRecalibration()
        {
            _pipeline.AwaitingRecalibration = true; // freeze fpa/step output until the participant taps Ready
            // The stage stopwatch and (for Baseline) the timeout stay stopped while we wait; EnterStage restarts them.
            _content.CurrentStageLabel = $"{_currentStage} (Redo — recalibrating)";
            _content.StatusLabel = $"Redo {_currentStage} — waiting for participant to tap Ready.";
            BroadcastArState("armed", reason: "Redo — quick recheck.\nTap Ready when you are.");
            log($"Redo {_currentStage} — recalibrating first, waiting for participant ready signal.");
        }

        /// <summary>
        /// The side effects of leaving the current stage for the next one: close the stage, freeze fpa/step output
        /// and ask the participant for Ready (the recalibration gate - IMU yaw drift bounding, see
        /// docs/task19_calibration_duration_tuning.py). The transition itself has ALREADY happened: every caller
        /// fires its event (Next Stage, block complete, rest elapsed, baseline completed) first and only calls this
        /// when ProtocolFlow accepted it, so the flow is now waiting for Ready and names the stage we are heading to.
        /// EnterStage() runs once the participant has stood still and the calibration completes.
        /// </summary>
        /// <param name="restEndReason">Set when this ends a Rest ("elapsed" / "skipped by Next Stage"), to log RestEnd.</param>
        private void EnterRecalPrompt(string? restEndReason = null)
        {
            if (restEndReason != null) _recorder.RecordEvent("RestEnd", restEndReason);
            string nextStage = _pendingStageEntry ?? "";
            _recorder.EndStage(_currentStage);
            if (SkipInterStageRecalibration)
            {
                // Every stage change waits for a plain Ready tap (no recalibration) so the participant notices it —
                // see HandleReadyForCalibration.
                _pipeline.AwaitingRecalibration = true; // freeze fpa/step output until Ready
                _recorder.RecordEvent("ReadyWait", $"before {nextStage}");
                _content.CurrentStageLabel = $"{_currentStage} → {nextStage} (Waiting for Ready)";
                _content.StatusLabel = $"Waiting for participant to tap Ready before {nextStage}.";
                BroadcastArState("armed", reason: $"Next: {nextStage}.\nTap Ready when you are.");
                log($"Waiting for Ready before {nextStage}.");
                return;
            }
            _pipeline.AwaitingRecalibration = true; // freeze fpa/training output until the participant taps Ready (see GaitPipeline.AwaitingRecalibration)
            _content.CurrentStageLabel = $"{_currentStage} → {nextStage} (Recalibrating)";
            _content.StatusLabel = $"Recalibrating before {nextStage} — waiting for participant to tap Ready.";
            _recorder.MarkStageEvent($"Recalibrate_Before{nextStage}");
            BroadcastArState("armed", reason: "Quick recheck before continuing.\nTap Ready when you are.");
            log($"Recalibrating before {nextStage} — waiting for participant ready signal.");
        }

        /// <summary>
        /// Actually begins `stage` — the old tail of the stage change, run once the pre-stage recalibration
        /// (armed above) completes. Unchanged from before the recalibration gate was added.
        /// </summary>
        private void EnterStage(string stage, bool isRedo = false)
        {
            _pipeline.AwaitingRecalibration = false;
            // OnCalibrationStateChanged's CollectingStaticPose case set _sessionState = Calibrating for
            // the recalibration we just finished; restore it to Step so inTraining
            // checks (BlockStepsDisplay, on-target backgrounds) resume —
            // otherwise they stay stuck off for the rest of the condition after the very first
            // recalibration gate (Baseline→Training1 included).
            _sessionState = TestState.Step;
            if (!isRedo)
                _recorder.BeginStage(_currentStage);
            else // the timing restarts with the new attempt (ActiveSeconds = last attempt only); earlier attempts stay in the events
            {
                _recorder.RestartStageTiming(stage);
                _recorder.RecordEvent("StageStart", $"redo attempt={_flow.AttemptOf(stage)}", stage);
            }
            _recorder.FlushMeta();
            _stageStopwatch.Restart();
            _content.CurrentStageLabel = _currentStage;
            _content.TurningExclusionDisplay = "";
            _content.BlockStepsDisplay = "L 0 · R 0";

            // Progressive-difficulty tolerance narrowing across the training blocks (Research Overview §2.4).
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
                    _content.StatusLabel =
                        $"Training started. Target L={bp.Target_L:F1}° ({bp.Direction_L})  " +
                        $"R={bp.Target_R:F1}° ({bp.Direction_R})";
                    _content.LeftFpaTarget = $"Target {bp.Target_L:F1}° ±{bp.ToleranceL:F1}° ({bp.Direction_L})";
                    _content.RightFpaTarget = $"Target {bp.Target_R:F1}° ±{bp.ToleranceR:F1}° ({bp.Direction_R})";
                }
            }
            else if (_currentStage == "Retention")
            {
                // Immediate Retention Test: AR feedback removed, FPA data collection continues (Research Overview §2.6).
                _retentionStepsL = _retentionStepsR = 0;
                _content.StatusLabel = "Retention: AR feedback removed, walk naturally.";
                BroadcastArState("retention");
            }
            else if (_currentStage == "Baseline")
            {
                // Only reached when the baseline is redone: start collecting it from scratch (StartBaseline resets
                // the collected steps), with a fresh timeout.
                _pipeline.StartBaseline();
                _sessionState = TestState.Baseline;
                _content.StatusLabel = $"Baseline: Walk until both feet reach {_pipeline.Params.BaselineStepsPerFoot} steps.";
                BroadcastArState("baseline");
                StartBaselineTimeoutTimer();
            }

            log(isRedo ? $"Stage → {_currentStage} (redo)" : $"Stage → {_currentStage}");
        }

        private void BtnEndCondition_Click(object sender, RoutedEventArgs e)
        {
            var missing = MissingExpectedMarkers();
            if (missing.Count > 0)
            {
                var proceed = MessageBox.Show(
                    $"The following audit markers haven't been logged yet:\n\n{string.Join("\n", missing)}\n\n" +
                    "If the RA already administered these on paper, go back and click the matching Mark button first. End Recording anyway?",
                    "Audit markers missing", MessageBoxButton.YesNo, MessageBoxImage.Warning);
                if (proceed != MessageBoxResult.Yes)
                {
                    log($"End Recording cancelled — missing markers: {string.Join(", ", missing)}");
                    return;
                }
                log($"End Recording confirmed despite missing markers: {string.Join(", ", missing)}");
            }
            EndFlow("operator ended", autoSaveCsvs: true);
        }

        // Stamp shared by the Retention-done and End Recording saves of one condition, so the second save overwrites the
        // first instead of leaving two sets of files. Cleared when a condition starts.
        private string? _autoSaveStamp;

        /// <summary>
        /// Saves ImuSamples / Diagnostics / Events CSVs into the condition's data folder (the "Open Data Folder" one),
        /// same format as the manual Save buttons. Must run before EndFlow's _pipeline.Reset(), which empties the
        /// diagnostics buffer.
        /// </summary>
        private void AutoSaveRecordingCsvs(bool clearSamples)
        {
            try
            {
                string? dir = Path.GetDirectoryName(_recorder.CurrentSessionFilePath ?? "");
                if (string.IsNullOrEmpty(dir)) { log("Auto-save skipped: no condition data folder."); return; }
                _autoSaveStamp ??= DateTime.Now.ToString("yyyyMMdd_HHmmss");
                var paths = RecordingCsvExporter.Save(dir, _autoSaveStamp, _imuSamples, _pipeline.GetDiagnosticsSnapshot(), _recorder);
                log($"Auto-saved {_imuSamples.Count} IMU samples, diagnostics and events → {dir} (*_{_autoSaveStamp}*.csv)");
                if (clearSamples) _imuSamples.Clear(); // same as the manual Save IMU Data button
            }
            catch (Exception ex)
            {
                log($"Auto-save failed: {ex.Message} — use the manual Save buttons.");
            }
        }

        // Recomputes the whole flow checklist every call (called from the 60ms timeline timer) rather than being
        // pushed from each individual mutation site — cheap for 10 rows, and guarantees the panel can never drift
        // out of sync with the state it's reflecting. Since T7 the source is ProtocolFlow (see _flow), not the
        // scattered legacy flags.
        private void RefreshChecklist()
        {
            if (_content.FlowChecklist.Count == 0) return; // not yet initialized

            foreach (var (label, status) in _flow.Checklist())
                SetChecklistStatus(label, status);
            foreach (var item in _content.FlowChecklist)
                if (item.MarkerStage != null)
                    item.CanMark = _flow.CanMark(item.MarkerStage);
        }

        private void SetChecklistStatus(string label, Model.ChecklistStatus status)
        {
            foreach (var item in _content.FlowChecklist)
            {
                if (item.Label == label) { item.Status = status; return; }
            }
        }

        // Only flags a marker as missing once the stage that's supposed to produce it has actually been
        // reached — an operator ending early (dropout, equipment failure) before Block3's rest window or
        // before Retention finished should NOT be nagged about instruments that were never due yet.
        private List<string> MissingExpectedMarkers() => _flow.MissingExpectedMarkers().ToList();

        /// <summary>
        /// Retention's 100/foot target (or the stall-prevention timeout) was reached. Stops step counting
        /// but does NOT close the recording — Retention -> [NASA-TLX(2) marker] -> Ended still needs the
        /// operator to mark NASA-TLX Admin 2 and click End Recording. Idempotent.
        /// </summary>
        private void CompleteRetention(string reason)
        {
            if (!_shadow.Fire(Protocol.FlowEvent.RetentionTargetReached).Accepted) return; // not in Retention, or already complete
            // Marks where Retention's counted data ends; rows written afterwards (waiting for NASA-TLX #2 / End) keep the Retention tag.
            _recorder.RecordEvent("RetentionDone", $"{reason}; steps L/R={_retentionStepsL}/{_retentionStepsR}");
            AutoSaveRecordingCsvs(clearSamples: false); // End Recording saves again (same names) with everything up to End
            _content.StatusLabel = "Retention complete. Administer NASA-TLX (Admin 2), then click End Recording.";
            BroadcastArState("ended"); // tell the participant the experiment is over ("Session complete. Thank you!")
            log($"Retention complete ({reason}) — awaiting NASA-TLX Admin 2 before ending. Click End Recording when done.");
        }

        /// <summary>
        /// Ends the whole condition flow and saves the session file. Called both by the operator End
        /// button and automatically when Retention hits 100 valid steps/foot or the stall-prevention timeout.
        /// Idempotent (no-op once the recording is closed).
        /// </summary>
        private void EndFlow(string reason, bool autoSaveCsvs = false)
        {
            if (!_recorder.IsRecording) return;
            FlushParamChanges(); // don't lose a change made in the last moments
            _shadow.Fire(Protocol.FlowEvent.EndRecording);
            _shadow.ReportSummary();
            _pipeline.AwaitingRecalibration = false;
            _baselineTimer?.Stop();
            if (_content.IsPaused)
            {
                // Ending while paused: close the open pause so it is logged, and don't leave the next condition paused.
                _recorder.EndPause("Ended", "recording ended while paused");
                _content.IsPaused = false;
                BtnPauseResume.Content = "Pause";
            }
            _recorder.EndStage(_currentStage);
            _recorder.RecordEvent("End", reason);
            _recorder.FlushMeta();
            _recorder.Close();
            if (autoSaveCsvs) AutoSaveRecordingCsvs(clearSamples: true); // before _pipeline.Reset() empties the diagnostics buffer
            _pipeline.CalibrationArmed = false;
            // Belt-and-suspenders alongside BtnStartCondition_Click's own _pipeline.Reset() — leaves
            // no stale BaselineProfile/module state lingering in the UI between End Recording and
            // whenever the operator gets around to starting the next condition.
            _pipeline.Reset();
            _stageStopwatch.Stop();
            BroadcastArState("ended");
            log($"Recording closed ({reason}): {_content.CurrentSessionFileName}");
        }

        private void PauseRecording()
        {
            _content.IsPaused = true;
            _shadow.Fire(Protocol.FlowEvent.Pause);
            _stageStopwatch.Stop();
            PauseBaselineTimer();
            _recorder.BeginPause(_currentStage);
            BtnPauseResume.Content = "Resume";
            BroadcastArState("paused"); // operator emergency pause → AR shows its paused display
            log($"Paused at {_currentStage}");
        }

        private void BtnPauseResume_Click(object sender, RoutedEventArgs e)
        {
            if (!_recorder.IsRecording) { log("No active recording to pause."); return; }

            if (!_content.IsPaused)
            {
                PauseRecording();
            }
            else
            {
                var dlg = new PauseResumeDialog(_recorder.PauseElapsed, redoAllowed: _flow.CanRedo) { Owner = this };
                bool? ok = dlg.ShowDialog();
                if (ok != true) return; // 关闭对话框但没填写/没选择 → 保持暂停状态

                if (dlg.Redo)
                {
                    // Redo throws away the steps counted in this stage and needs a new calibration, so ask first
                    // (typical reason: an IMU came loose or shifted).
                    var sure = MessageBox.Show(
                        $"Redo {_currentStage}?\n\n" +
                        "The steps counted so far in this stage are discarded and the participant must calibrate again " +
                        "before the stage starts over. Use this when something went wrong, e.g. an IMU came loose or shifted.",
                        "Redo stage", MessageBoxButton.YesNo, MessageBoxImage.Warning);
                    if (sure != MessageBoxResult.Yes) { log("Redo cancelled — still paused."); return; }

                    var redoResult = _shadow.Fire(Protocol.FlowEvent.ResumeRedo);
                    if (!redoResult.Accepted) { log($"Redo refused — {redoResult.Reason}. Still paused."); return; }
                    int nextAttempt = _flow.AttemptOf(_currentStage); // the flow has just advanced the attempt number
                    _recorder.EndPause("Redo", dlg.Reason);
                    _recorder.MarkRedo(_currentStage, nextAttempt);
                    BroadcastRedo(_currentStage, nextAttempt, dlg.Reason);
                    log($"Resumed with Redo — {_currentStage} attempt #{nextAttempt}: {dlg.Reason}");
                    BeginRedoRecalibration();
                    _content.IsPaused = false;
                    BtnPauseResume.Content = "Pause";
                    return; // the AR is now asked for Ready; don't restore the pre-pause state or restart timers
                }
                else
                {
                    // Continuing without every required IMU would leave the pipeline silent (frame bundles never complete).
                    var missing = _slotRegistry.Imus.Where(i => i.Role != ImuRole.None && !i.IsConnected).Select(i => i.Role).ToList();
                    if (missing.Count > 0)
                    {
                        log($"Continue refused — IMU not connected: {string.Join(", ", missing)}. Still paused.");
                        MessageBox.Show($"Cannot continue: IMU not connected ({string.Join(", ", missing)}).\n\nWait for it to reconnect, or choose Redo.",
                            "IMU missing", MessageBoxButton.OK, MessageBoxImage.Warning);
                        return;
                    }
                    _recorder.EndPause("Continue", dlg.Reason);
                    _shadow.Fire(Protocol.FlowEvent.ResumeContinue);
                    _stageStopwatch.Start();
                    log($"Resumed, continuing {_currentStage}: {dlg.Reason}");
                }

                // Put the AR back exactly where it was before the pause (state + reason + rest duration + block).
                // For a training/baseline/retention state it shows the "keep walking" text prompt and only switches
                // back to the graphic group when the next fpa arrives; for "armed" it brings the Ready button back.
                var (resumeState, resumeReason, resumeRestSec, resumeBlock) = _arStateBeforePause;
                BroadcastArState(resumeState, resumeReason, resumeRestSec, resumeBlock);

                ResumeBaselineTimer();
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
                    _content.LeftFpaTarget = $"Target {profile.Target_L:F1}° ±{profile.ToleranceL:F1}° ({profile.Direction_L})";
                    _content.RightFpaTarget = $"Target {profile.Target_R:F1}° ±{profile.ToleranceR:F1}° ({profile.Direction_R})";
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
                _pipeline.Params.BaselineStepsPerFoot));
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
            events.Add(("Total", $"FPA: L={fpaL} steps / R={fpaR} steps, BaselineStepsPerFoot={minBaselineSteps} → {conclusion}"));

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

            if (inTraining && !_isRetention)
                _content.BlockStepsDisplay = $"L {_pipeline.TrainingStepsL} · R {_pipeline.TrainingStepsR}";

            // Immediate Retention Test: count valid steps until 100/foot (the stall-prevention timeout is
            // handled by the timeline timer). AR feedback is removed, so no fpa is broadcast below. Completion
            // stops counting but does not close the recording — see CompleteRetention().
            if (_isRetention && _recorder.IsRecording && !_retentionComplete)
            {
                if (!float.IsNaN(result.Fpa_L)) _retentionStepsL++;
                if (!float.IsNaN(result.Fpa_R)) _retentionStepsR++;
                _content.BlockStepsDisplay = $"L {_retentionStepsL} · R {_retentionStepsR}";
                BroadcastStepProgress("retention", _retentionStepsL, _retentionStepsR, _pipeline.Params.TargetStepsPerFoot);
                if (_retentionStepsL >= _pipeline.Params.TargetStepsPerFoot && _retentionStepsR >= _pipeline.Params.TargetStepsPerFoot)
                {
                    CompleteRetention($"retention target reached ({_retentionStepsL}/{_retentionStepsR} valid steps)");
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
                    log($"[latency-temp] frame -> AR broadcast: {latencyMs.Value} ms (packetId={result.PacketId})");

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
