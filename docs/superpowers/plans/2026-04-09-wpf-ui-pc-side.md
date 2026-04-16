# WPF UI + PC-Side Pipeline Changes Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a live-tunable parameter panel, diagnostics CSV export, restart-calibration button, collapsible log, and WebSocket state broadcast to the IMUMoCap WPF desktop app.

**Architecture:** Three layers of change — (1) pipeline backend gains `PipelineParams`, `DiagnosticsRow`, and state-broadcast support; (2) `MainPageVM` gains new bindable properties; (3) `MainWindow.xaml` is redesigned as a left-right split with collapsible panels. No new assemblies — everything stays in the existing `IMUMoCap` project.

**Tech Stack:** C# 12 / .NET 8 / WPF / MaterialDesignInXAML (already referenced), `System.Text.Json` (already referenced), `System.IO` for CSV writing.

---

## File Structure

| File | Action | Responsibility |
|---|---|---|
| `IMUMoCap/Pipeline/Models/PipelineParams.cs` | Create | The 7 tunable parameters as a plain class |
| `IMUMoCap/Pipeline/Models/DiagnosticsRow.cs` | Create | Per-frame diagnostics struct for CSV export |
| `IMUMoCap/Pipeline/GaitPipeline.cs` | Modify | Add `Params` property, `SyncParams()`, diagnostics buffer, `GetDiagnosticsSnapshot()` |
| `IMUMoCap/VM/MainPageVM.cs` | Modify | Add 7 param properties + `LeftFpaNote`/`RightFpaNote` |
| `IMUMoCap/MainWindow.xaml` | Modify | New left-right split layout with params panel, log toggle |
| `IMUMoCap/MainWindow.xaml.cs` | Modify | Param sync, restart calibration, save diagnostics, WS state broadcast |

---

### Task 1: Create PipelineParams model

**Files:**
- Create: `IMUMoCap/Pipeline/Models/PipelineParams.cs`

- [ ] **Step 1: Create the file**

```csharp
// IMUMoCap/Pipeline/Models/PipelineParams.cs
namespace IMUMoCap.Pipeline.Models
{
    /// <summary>
    /// Live-tunable pipeline parameters. GaitPipeline reads these on every Process() call
    /// and pushes them into sub-processors, so changes take effect immediately.
    /// </summary>
    public sealed class PipelineParams
    {
        // CalibrationProcessor
        public float StompThreshold_ms2      { get; set; } = 25f;   // 10–50
        public float StaticGyroThreshold     { get; set; } = 0.3f;  // 0.05–1.0

        // GaitEventDetector
        public float StanceFreeAccThreshold  { get; set; } = 2.5f;  // 0.5–5.0
        public float StanceGyroThreshold     { get; set; } = 1.0f;  // 0.1–3.0

        // FpaEngine gate thresholds
        public float PdConfidenceThreshold   { get; set; } = 0.7f;  // 0.3–1.0
        public float PdStabilityThreshold    { get; set; } = 0.7f;  // 0.3–1.0

        // BaselineProcessor
        public int   MinBaselineSteps        { get; set; } = 20;    // 10–50
    }
}
```

- [ ] **Step 2: Build to verify no errors**

```
cd D:\SourceCode\IMUMoCap
dotnet build IMUMoCap/IMUMoCap.csproj
```
Expected: `Build succeeded. 0 Error(s)`

- [ ] **Step 3: Commit**

```bash
git add IMUMoCap/Pipeline/Models/PipelineParams.cs
git commit -m "feat: add PipelineParams for live-tunable pipeline configuration"
```

---

### Task 2: Create DiagnosticsRow model

**Files:**
- Create: `IMUMoCap/Pipeline/Models/DiagnosticsRow.cs`

This row represents one complete synchronized 3-IMU frame plus all pipeline intermediate results. One row per valid frame output from `GaitPipeline.Process()`. This format is designed for feeding to AI for debug analysis.

- [ ] **Step 1: Create the file**

```csharp
// IMUMoCap/Pipeline/Models/DiagnosticsRow.cs
using System;
using System.Numerics;

namespace IMUMoCap.Pipeline.Models
{
    public sealed class DiagnosticsRow
    {
        // ── Timing ────────────────────────────────────────────────────────────
        public long   Timestamp  { get; init; }  // DateTimeOffset.UtcNow.ToUnixTimeMilliseconds()
        public long   PacketId   { get; init; }

        // ── Pelvis IMU ────────────────────────────────────────────────────────
        public Vector3    PelvisAcc  { get; init; }
        public Vector3    PelvisGyr  { get; init; }
        public Quaternion PelvisQuat { get; init; }

        // ── Left Foot IMU ─────────────────────────────────────────────────────
        public Vector3    LeftAcc    { get; init; }
        public Vector3    LeftGyr    { get; init; }
        public Quaternion LeftQuat   { get; init; }

        // ── Right Foot IMU ────────────────────────────────────────────────────
        public Vector3    RightAcc   { get; init; }
        public Vector3    RightGyr   { get; init; }
        public Quaternion RightQuat  { get; init; }

        // ── GaitEvent ─────────────────────────────────────────────────────────
        public bool LeftStance  { get; init; }
        public bool RightStance { get; init; }
        public bool IsWalking   { get; init; }

        // ── MotionContext ─────────────────────────────────────────────────────
        public ContextState MotionState      { get; init; }
        public float        MotionConfidence { get; init; }

        // ── PdEstimate ────────────────────────────────────────────────────────
        public float PdDirectionDeg { get; init; }
        public float PdStability    { get; init; }
        public bool  PdIsValid      { get; init; }

        // ── FpaResult (NaN when not available this frame) ─────────────────────
        public float FpaLeft_Deg      { get; init; }  // float.NaN if no result this frame
        public float FpaLeft_Error    { get; init; }
        public bool  FpaLeft_OnTarget { get; init; }
        public float FpaRight_Deg     { get; init; }
        public float FpaRight_Error   { get; init; }
        public bool  FpaRight_OnTarget { get; init; }

        // ── CSV header (matches property order above) ─────────────────────────
        public static string CsvHeader =>
            "Timestamp,PacketId," +
            "Pelvis_AccX,Pelvis_AccY,Pelvis_AccZ," +
            "Pelvis_GyrX,Pelvis_GyrY,Pelvis_GyrZ," +
            "Pelvis_QuatW,Pelvis_QuatX,Pelvis_QuatY,Pelvis_QuatZ," +
            "Left_AccX,Left_AccY,Left_AccZ," +
            "Left_GyrX,Left_GyrY,Left_GyrZ," +
            "Left_QuatW,Left_QuatX,Left_QuatY,Left_QuatZ," +
            "Right_AccX,Right_AccY,Right_AccZ," +
            "Right_GyrX,Right_GyrY,Right_GyrZ," +
            "Right_QuatW,Right_QuatX,Right_QuatY,Right_QuatZ," +
            "GaitEvent_LeftStance,GaitEvent_RightStance,GaitEvent_IsWalking," +
            "MotionContext_State,MotionContext_Confidence," +
            "PD_DirectionDeg,PD_Stability,PD_IsValid," +
            "FPA_Left_Deg,FPA_Left_Error,FPA_Left_OnTarget," +
            "FPA_Right_Deg,FPA_Right_Error,FPA_Right_OnTarget";

        public string ToCsvRow() =>
            $"{Timestamp},{PacketId}," +
            $"{PelvisAcc.X:F4},{PelvisAcc.Y:F4},{PelvisAcc.Z:F4}," +
            $"{PelvisGyr.X:F4},{PelvisGyr.Y:F4},{PelvisGyr.Z:F4}," +
            $"{PelvisQuat.W:F6},{PelvisQuat.X:F6},{PelvisQuat.Y:F6},{PelvisQuat.Z:F6}," +
            $"{LeftAcc.X:F4},{LeftAcc.Y:F4},{LeftAcc.Z:F4}," +
            $"{LeftGyr.X:F4},{LeftGyr.Y:F4},{LeftGyr.Z:F4}," +
            $"{LeftQuat.W:F6},{LeftQuat.X:F6},{LeftQuat.Y:F6},{LeftQuat.Z:F6}," +
            $"{RightAcc.X:F4},{RightAcc.Y:F4},{RightAcc.Z:F4}," +
            $"{RightGyr.X:F4},{RightGyr.Y:F4},{RightGyr.Z:F4}," +
            $"{RightQuat.W:F6},{RightQuat.X:F6},{RightQuat.Y:F6},{RightQuat.Z:F6}," +
            $"{LeftStance},{RightStance},{IsWalking}," +
            $"{MotionState},{MotionConfidence:F3}," +
            $"{PdDirectionDeg:F2},{PdStability:F3},{PdIsValid}," +
            $"{(float.IsNaN(FpaLeft_Deg) ? "" : FpaLeft_Deg.ToString("F2"))}," +
            $"{(float.IsNaN(FpaLeft_Error) ? "" : FpaLeft_Error.ToString("F2"))}," +
            $"{FpaLeft_OnTarget}," +
            $"{(float.IsNaN(FpaRight_Deg) ? "" : FpaRight_Deg.ToString("F2"))}," +
            $"{(float.IsNaN(FpaRight_Error) ? "" : FpaRight_Error.ToString("F2"))}," +
            $"{FpaRight_OnTarget}";
    }
}
```

- [ ] **Step 2: Build to verify no errors**

```
dotnet build IMUMoCap/IMUMoCap.csproj
```
Expected: `Build succeeded. 0 Error(s)`

- [ ] **Step 3: Commit**

```bash
git add IMUMoCap/Pipeline/Models/DiagnosticsRow.cs
git commit -m "feat: add DiagnosticsRow for AI-friendly diagnostics CSV export"
```

---

### Task 3: Integrate PipelineParams and diagnostics buffer into GaitPipeline

**Files:**
- Modify: `IMUMoCap/Pipeline/GaitPipeline.cs`

The key changes:
1. Add `public PipelineParams Params { get; } = new();`
2. Add `private void SyncParams()` — copies Params values to sub-processor properties
3. Call `SyncParams()` at the top of `Process(ImuFrameBundle)`
4. Add `_diagnosticsBuffer` list
5. Append a `DiagnosticsRow` at the end of `Process()` (always, regardless of FPA result)
6. Add `public List<DiagnosticsRow> GetDiagnosticsSnapshot()` — returns a copy
7. Clear buffer in `Reset()`

Context: `CalibrationProcessor` has `StompAccThreshold_ms2` and `StaticGyroThreshold`. `GaitEventDetector` has `FreeAccStanceThreshold` and `GyroThreshold`. `FpaEngine` has `ContextConfidenceThreshold` and `PdStabilityThreshold`. `BaselineProcessor` has `MinValidSteps`.

- [ ] **Step 1: Open `IMUMoCap/Pipeline/GaitPipeline.cs` and add the Params property and diagnostics buffer field after the existing `_fpa` field declaration**

After line:
```csharp
        private readonly FpaEngine               _fpa            = new();
```

Add:
```csharp

        // ── 可调参数（UI 实时更新）──────────────────────────────────────────────
        public PipelineParams Params { get; } = new();

        // ── 诊断数据缓冲区 ────────────────────────────────────────────────────
        private readonly List<DiagnosticsRow> _diagnosticsBuffer = new();
```

- [ ] **Step 2: Add the `SyncParams()` private method just before the `private void Process(ImuFrameBundle bundle)` method**

```csharp
        private void SyncParams()
        {
            // CalibrationProcessor
            _calibration.StompAccThreshold_ms2 = Params.StompThreshold_ms2;
            _calibration.StaticGyroThreshold   = Params.StaticGyroThreshold;

            // GaitEventDetector
            _gait.FreeAccStanceThreshold = Params.StanceFreeAccThreshold;
            _gait.GyroThreshold          = Params.StanceGyroThreshold;

            // FpaEngine
            _fpa.ContextConfidenceThreshold = Params.PdConfidenceThreshold;
            _fpa.PdStabilityThreshold       = Params.PdStabilityThreshold;

            // BaselineProcessor
            _baseline.MinValidSteps = Params.MinBaselineSteps;
        }
```

- [ ] **Step 3: Add `SyncParams()` call and diagnostics row append inside `Process(ImuFrameBundle bundle)`**

The method currently starts with `var validFrame = _gate.Evaluate(bundle);`. Change `Process` to:

```csharp
        private void Process(ImuFrameBundle bundle)
        {
            SyncParams();

            // Step 1: DataQualityGate
            var validFrame = _gate.Evaluate(bundle);
            if (validFrame == null) return;

            // Step 2: CalibrationProcessor（仅在未完成校准时运行）
            if (_calibration.State != CalibrationState.Completed &&
                _calibration.State != CalibrationState.Failed)
            {
                bool stateChanged = _calibration.Process(validFrame);
                if (stateChanged)
                {
                    OnCalibrationStateChanged?.Invoke(_calibration.State);
                    if (_calibration.State == CalibrationState.Completed)
                        OnLog?.Invoke("Calibration completed.");
                    else if (_calibration.State == CalibrationState.Failed)
                        OnLog?.Invoke("Calibration failed — please restart.");
                }
                if (_calibration.State != CalibrationState.Completed) return;
            }

            var profile = _calibration.Profile!;

            // Step 3: GaitEventDetector
            var gaitEvent = _gait.Detect(validFrame, profile);

            // Step 4: MotionContextDetector
            var motionCtx = _motion.Detect(validFrame, gaitEvent);

            // Step 5: ProgressionDirEstimator
            var pdEstimate = _pd.Update(validFrame, gaitEvent, motionCtx, _motion);

            // Step 6: FpaEngine
            var fpaResult = _fpa.Process(validFrame, gaitEvent, motionCtx, pdEstimate);

            // ── 诊断数据采集（每帧一行，FPA 可为 null）────────────────────────
            _diagnosticsBuffer.Add(new DiagnosticsRow
            {
                Timestamp        = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds(),
                PacketId         = validFrame.PacketId,
                PelvisAcc        = validFrame.Pelvis.HasAcceleration    ? validFrame.Pelvis.Acceleration    : System.Numerics.Vector3.Zero,
                PelvisGyr        = validFrame.Pelvis.HasRateOfTurn       ? validFrame.Pelvis.RateOfTurn       : System.Numerics.Vector3.Zero,
                PelvisQuat       = validFrame.Pelvis.HasQuaternion       ? validFrame.Pelvis.Quaternion       : System.Numerics.Quaternion.Identity,
                LeftAcc          = validFrame.LeftFoot.HasAcceleration   ? validFrame.LeftFoot.Acceleration   : System.Numerics.Vector3.Zero,
                LeftGyr          = validFrame.LeftFoot.HasRateOfTurn      ? validFrame.LeftFoot.RateOfTurn      : System.Numerics.Vector3.Zero,
                LeftQuat         = validFrame.LeftFoot.HasQuaternion      ? validFrame.LeftFoot.Quaternion      : System.Numerics.Quaternion.Identity,
                RightAcc         = validFrame.RightFoot.HasAcceleration  ? validFrame.RightFoot.Acceleration  : System.Numerics.Vector3.Zero,
                RightGyr         = validFrame.RightFoot.HasRateOfTurn     ? validFrame.RightFoot.RateOfTurn     : System.Numerics.Vector3.Zero,
                RightQuat        = validFrame.RightFoot.HasQuaternion     ? validFrame.RightFoot.Quaternion     : System.Numerics.Quaternion.Identity,
                LeftStance       = gaitEvent.LeftStance,
                RightStance      = gaitEvent.RightStance,
                IsWalking        = gaitEvent.IsWalking,
                MotionState      = motionCtx.State,
                MotionConfidence = motionCtx.Confidence,
                PdDirectionDeg   = pdEstimate.DirectionRad * (180f / MathF.PI),
                PdStability      = pdEstimate.Stability,
                PdIsValid        = pdEstimate.IsValid,
                FpaLeft_Deg      = fpaResult?.Fpa_L      ?? float.NaN,
                FpaLeft_Error    = fpaResult?.Error_L    ?? float.NaN,
                FpaLeft_OnTarget = fpaResult?.OnTarget_L ?? false,
                FpaRight_Deg     = fpaResult?.Fpa_R      ?? float.NaN,
                FpaRight_Error   = fpaResult?.Error_R    ?? float.NaN,
                FpaRight_OnTarget = fpaResult?.OnTarget_R ?? false,
            });

            if (fpaResult == null) return;

            if (InBaseline)
            {
                _baseline.AddStep(fpaResult);
                OnBaselineProgress?.Invoke(_baseline.CollectedSteps_L, _baseline.CollectedSteps_R);
                OnFpaResult?.Invoke(fpaResult);
                return;
            }

            if (InTraining)
                OnFpaResult?.Invoke(fpaResult);
        }
```

- [ ] **Step 4: Add `GetDiagnosticsSnapshot()` public method and update `Reset()` to clear the buffer**

Add method after `Reset()`:
```csharp
        /// <summary>
        /// 返回诊断数据快照（副本），用于 CSV 导出。不清空缓冲区。
        /// </summary>
        public List<DiagnosticsRow> GetDiagnosticsSnapshot()
            => new List<DiagnosticsRow>(_diagnosticsBuffer);
```

In the existing `Reset()` method, add `_diagnosticsBuffer.Clear();` after `InBaseline = InTraining = false;`:
```csharp
        public void Reset()
        {
            _gate.Reset();
            _calibration.Reset();
            _gait.Reset();
            _motion.Reset();
            _pd.Reset();
            _baseline.Reset();
            _fpa.Reset();
            BaselineProfile = null;
            InBaseline = InTraining = false;
            _diagnosticsBuffer.Clear();
        }
```

- [ ] **Step 5: Add missing using directives at the top of `GaitPipeline.cs`**

The file already has `using System;`. Add:
```csharp
using System.Collections.Generic;
```
(Check if already present — add only if missing.)

- [ ] **Step 6: Build to verify**

```
dotnet build IMUMoCap/IMUMoCap.csproj
```
Expected: `Build succeeded. 0 Error(s)`

- [ ] **Step 7: Commit**

```bash
git add IMUMoCap/Pipeline/GaitPipeline.cs
git commit -m "feat: add PipelineParams live sync and DiagnosticsRow buffer to GaitPipeline"
```

---

### Task 4: Add `type:"state"` WebSocket broadcast

**Files:**
- Modify: `IMUMoCap/MainWindow.xaml.cs`

The AR glasses need to know which phase the system is in. Broadcast a `{"type":"state","state":"..."}` JSON message whenever the pipeline phase changes. The mapping:
- CalibrationState.WaitingForStart → `"waiting"`
- CalibrationState.CollectingStaticPose → `"calibrating"`
- Right after `_pipeline.StartBaseline()` → `"baseline"`
- Right after `_pipeline.StartTraining()` → `"training"`
- CalibrationState.Failed → `"error"`

- [ ] **Step 1: Add `BroadcastArState(string)` helper method to `MainWindow.xaml.cs`**

Add this private method anywhere in the class (e.g., just before `OnCalibrationStateChanged`):

```csharp
        private void BroadcastArState(string state)
        {
            if (_wsServer == null) return;
            _ = _wsServer.BroadcastJsonAsync(new { type = "state", state });
        }
```

- [ ] **Step 2: Update `OnCalibrationStateChanged` to broadcast state on each transition**

Replace the existing `OnCalibrationStateChanged` method:

```csharp
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
                    _baselineTimer = new DispatcherTimer { Interval = TimeSpan.FromMinutes(3) };
                    _baselineTimer.Tick += (_, _) => { _baselineTimer.Stop(); FinalizeBaseline(); };
                    _baselineTimer.Start();
                    _sessionState = TestState.Baseline;
                    _content.StatusLabel = "Baseline: Walk naturally for 3 minutes.";
                    break;

                case CalibrationState.Failed:
                    _content.StatusLabel = "Calibration failed. Please restart the application.";
                    _sessionState = TestState.Launching;
                    BroadcastArState("error");
                    break;
            }
        }
```

- [ ] **Step 3: Update `FinalizeBaseline()` to broadcast "training" after successful start**

Replace the existing `FinalizeBaseline()` method:

```csharp
        private void FinalizeBaseline()
        {
            var profile = _pipeline.FinalizeBaseline();
            if (profile == null)
            {
                _content.StatusLabel = "Baseline insufficient (< 20 valid steps). Please restart.";
                BroadcastArState("error");
                return;
            }
            _sessionState = TestState.Step;
            _content.StatusLabel =
                $"Training started. Target L={profile.Target_L:F1}° ({profile.Direction_L})  " +
                $"R={profile.Target_R:F1}° ({profile.Direction_R})";
            _pipeline.StartTraining();
            BroadcastArState("training");
        }
```

- [ ] **Step 4: Build to verify**

```
dotnet build IMUMoCap/IMUMoCap.csproj
```
Expected: `Build succeeded. 0 Error(s)`

- [ ] **Step 5: Commit**

```bash
git add IMUMoCap/MainWindow.xaml.cs
git commit -m "feat: broadcast type:state WebSocket messages for AR HUD state transitions"
```

---

### Task 5: Update MainPageVM with new bindable properties

**Files:**
- Modify: `IMUMoCap/VM/MainPageVM.cs`

`MainPageVM` is the `DataContext` for `MainWindow`. WPF binds to its properties via `INotifyPropertyChanged` (already implemented with `OnPropertyChanged`). The XAML already references `LeftFpaNote` and `RightFpaNote` which don't exist yet — add them. Also add 7 param properties for the slider panel.

- [ ] **Step 1: Add `LeftFpaNote` and `RightFpaNote` to `MainPageVM.cs`**

Add after the `RightFpaDeg` property block (which ends around line 268 in the current file):

```csharp
        private string _LeftFpaNote = "";
        public string LeftFpaNote
        {
            get { return _LeftFpaNote; }
            set { _LeftFpaNote = value; OnPropertyChanged(nameof(LeftFpaNote)); }
        }

        private string _RightFpaNote = "";
        public string RightFpaNote
        {
            get { return _RightFpaNote; }
            set { _RightFpaNote = value; OnPropertyChanged(nameof(RightFpaNote)); }
        }
```

- [ ] **Step 2: Add 7 parameter properties**

These properties are two-way bound to sliders. MainWindow.xaml.cs will push their values into `_pipeline.Params` whenever they change.

Add after the `RightFpaNote` block:

```csharp
        // ── Pipeline tunable parameters (bound to right-panel sliders) ────────

        private float _stompThreshold = 25f;
        public float StompThreshold
        {
            get { return _stompThreshold; }
            set { _stompThreshold = value; OnPropertyChanged(nameof(StompThreshold)); }
        }

        private float _staticGyroThreshold = 0.3f;
        public float StaticGyroThreshold
        {
            get { return _staticGyroThreshold; }
            set { _staticGyroThreshold = value; OnPropertyChanged(nameof(StaticGyroThreshold)); }
        }

        private float _stanceFreeAccThreshold = 2.5f;
        public float StanceFreeAccThreshold
        {
            get { return _stanceFreeAccThreshold; }
            set { _stanceFreeAccThreshold = value; OnPropertyChanged(nameof(StanceFreeAccThreshold)); }
        }

        private float _stanceGyroThreshold = 1.0f;
        public float StanceGyroThreshold
        {
            get { return _stanceGyroThreshold; }
            set { _stanceGyroThreshold = value; OnPropertyChanged(nameof(StanceGyroThreshold)); }
        }

        private float _pdConfidenceThreshold = 0.7f;
        public float PdConfidenceThreshold
        {
            get { return _pdConfidenceThreshold; }
            set { _pdConfidenceThreshold = value; OnPropertyChanged(nameof(PdConfidenceThreshold)); }
        }

        private float _pdStabilityThreshold = 0.7f;
        public float PdStabilityThreshold
        {
            get { return _pdStabilityThreshold; }
            set { _pdStabilityThreshold = value; OnPropertyChanged(nameof(PdStabilityThreshold)); }
        }

        private int _minBaselineSteps = 20;
        public int MinBaselineSteps
        {
            get { return _minBaselineSteps; }
            set { _minBaselineSteps = value; OnPropertyChanged(nameof(MinBaselineSteps)); }
        }

        // ── Log panel visibility ──────────────────────────────────────────────
        private bool _logPanelVisible = true;
        public bool LogPanelVisible
        {
            get { return _logPanelVisible; }
            set { _logPanelVisible = value; OnPropertyChanged(nameof(LogPanelVisible)); }
        }

        // ── Params panel visibility ───────────────────────────────────────────
        private bool _paramsPanelVisible = true;
        public bool ParamsPanelVisible
        {
            get { return _paramsPanelVisible; }
            set { _paramsPanelVisible = value; OnPropertyChanged(nameof(ParamsPanelVisible)); }
        }
```

- [ ] **Step 3: Build to verify**

```
dotnet build IMUMoCap/IMUMoCap.csproj
```
Expected: `Build succeeded. 0 Error(s)`

- [ ] **Step 4: Commit**

```bash
git add IMUMoCap/VM/MainPageVM.cs
git commit -m "feat: add param, note, and panel visibility properties to MainPageVM"
```

---

### Task 6: Redesign MainWindow.xaml layout

**Files:**
- Modify: `IMUMoCap/MainWindow.xaml`

Replace the entire `<Grid>` content with a new left-right `DockPanel` split layout. The 3D viewports are preserved (they're used for sensor visualization). The right panel is fixed at 270px and contains the params sliders and action buttons.

Key WPF notes for this layout:
- `BooleanToVisibilityConverter` is built into WPF — add as resource to convert `LogPanelVisible` → `Visibility`.
- Sliders use `Minimum`, `Maximum`, `Value` (two-way binding), `SmallChange`, `LargeChange`.
- The collapsible panels use `Visibility="{Binding LogPanelVisible, Converter={StaticResource BoolToVis}}"`.

- [ ] **Step 1: Replace the entire contents of `MainWindow.xaml`**

```xml
<Window
    x:Class="IMUMoCap.MainWindow"
    xmlns="http://schemas.microsoft.com/winfx/2006/xaml/presentation"
    xmlns:x="http://schemas.microsoft.com/winfx/2006/xaml"
    xmlns:d="http://schemas.microsoft.com/expression/blend/2008"
    xmlns:local="clr-namespace:IMUMoCap"
    xmlns:mc="http://schemas.openxmlformats.org/markup-compatibility/2006"
    Title="IMUMoCap"
    Width="1280"
    Height="800"
    Style="{StaticResource MaterialDesignWindow}"
    WindowState="Maximized"
    mc:Ignorable="d">

    <Window.Resources>
        <BooleanToVisibilityConverter x:Key="BoolToVis"/>
    </Window.Resources>

    <!-- Root: left panel (flex) | right panel (fixed 270px) -->
    <DockPanel LastChildFill="True">

        <!-- ── RIGHT PANEL ──────────────────────────────────────────────────── -->
        <Border DockPanel.Dock="Right" Width="270"
                BorderBrush="#22000000" BorderThickness="1,0,0,0"
                Background="#F5F5F5">
            <ScrollViewer VerticalScrollBarVisibility="Auto">
                <StackPanel Margin="12">

                    <!-- Toggle params -->
                    <Button Click="BtnToggleParams_Click" Margin="0,0,0,6"
                            HorizontalAlignment="Stretch">
                        <TextBlock Text="{Binding ParamsPanelVisible,
                            Converter={x:Static local:BoolToToggleTextConverter.ParamsInstance}}"/>
                    </Button>

                    <!-- Params section (collapsible) -->
                    <StackPanel Visibility="{Binding ParamsPanelVisible, Converter={StaticResource BoolToVis}}">

                        <TextBlock Text="参数调节" FontWeight="Bold" Margin="0,0,0,8"/>

                        <!-- 1. Stomp threshold -->
                        <TextBlock Text="{Binding StompThreshold, StringFormat='跺脚阈值: {0:F0} m/s²'}" FontSize="11"/>
                        <Slider Minimum="10" Maximum="50" SmallChange="1" LargeChange="5"
                                Value="{Binding StompThreshold, Mode=TwoWay}" Margin="0,0,0,8"/>

                        <!-- 2. Static gyro threshold -->
                        <TextBlock Text="{Binding StaticGyroThreshold, StringFormat='静止陀螺仪阈值: {0:F2} rad/s'}" FontSize="11"/>
                        <Slider Minimum="0.05" Maximum="1.0" SmallChange="0.01" LargeChange="0.1"
                                Value="{Binding StaticGyroThreshold, Mode=TwoWay}" Margin="0,0,0,8"/>

                        <!-- 3. Stance free-acc threshold -->
                        <TextBlock Text="{Binding StanceFreeAccThreshold, StringFormat='支撑相自由加速度阈值: {0:F1} m/s²'}" FontSize="11"/>
                        <Slider Minimum="0.5" Maximum="5.0" SmallChange="0.1" LargeChange="0.5"
                                Value="{Binding StanceFreeAccThreshold, Mode=TwoWay}" Margin="0,0,0,8"/>

                        <!-- 4. Stance gyro threshold -->
                        <TextBlock Text="{Binding StanceGyroThreshold, StringFormat='支撑相陀螺仪阈值: {0:F1} rad/s'}" FontSize="11"/>
                        <Slider Minimum="0.1" Maximum="3.0" SmallChange="0.05" LargeChange="0.2"
                                Value="{Binding StanceGyroThreshold, Mode=TwoWay}" Margin="0,0,0,8"/>

                        <!-- 5. PD confidence threshold -->
                        <TextBlock Text="{Binding PdConfidenceThreshold, StringFormat='PD置信度阈值: {0:F2}'}" FontSize="11"/>
                        <Slider Minimum="0.3" Maximum="1.0" SmallChange="0.01" LargeChange="0.1"
                                Value="{Binding PdConfidenceThreshold, Mode=TwoWay}" Margin="0,0,0,8"/>

                        <!-- 6. PD stability threshold -->
                        <TextBlock Text="{Binding PdStabilityThreshold, StringFormat='PD稳定度阈值: {0:F2}'}" FontSize="11"/>
                        <Slider Minimum="0.3" Maximum="1.0" SmallChange="0.01" LargeChange="0.1"
                                Value="{Binding PdStabilityThreshold, Mode=TwoWay}" Margin="0,0,0,8"/>

                        <!-- 7. Min baseline steps -->
                        <TextBlock Text="{Binding MinBaselineSteps, StringFormat='最少Baseline步数: {0}'}" FontSize="11"/>
                        <Slider Minimum="10" Maximum="50" SmallChange="1" LargeChange="5"
                                IsSnapToTickEnabled="True" TickFrequency="1"
                                Value="{Binding MinBaselineSteps, Mode=TwoWay}" Margin="0,0,0,12"/>

                    </StackPanel>

                    <Separator Margin="0,4,0,8"/>

                    <!-- Action buttons -->
                    <Button x:Name="BtnRestartCal" Click="BtnRestartCal_Click"
                            HorizontalAlignment="Stretch" Margin="0,0,0,8"
                            ToolTip="重置流水线并重新等待跺脚触发校准">
                        重新开始校准
                    </Button>

                    <Button x:Name="BtnSaveDiagnostics" Click="BtnSaveDiagnostics_Click"
                            HorizontalAlignment="Stretch" Margin="0,0,0,8"
                            ToolTip="将当前诊断数据导出为 CSV，供 AI 分析">
                        保存诊断数据 (CSV)
                    </Button>

                    <Separator Margin="0,4,0,8"/>

                    <!-- Legacy buttons -->
                    <Button Click="BtnMeasure_Click" HorizontalAlignment="Stretch" Margin="0,0,0,4">Measure</Button>
                    <Button Click="BtnTest_Click" HorizontalAlignment="Stretch" Margin="0,0,0,4">Test</Button>
                    <Button Click="btnClear" HorizontalAlignment="Stretch" Margin="0,0,0,4">Clear Log</Button>
                    <StackPanel Orientation="Horizontal" Margin="0,4,0,0">
                        <TextBlock Text="Sample Hz:" VerticalAlignment="Center" Margin="0,0,4,0" FontSize="11"/>
                        <TextBox x:Name="txtSampleRate" Width="50" Height="24" Text="10"/>
                    </StackPanel>
                    <Button Click="Button_SaveData" HorizontalAlignment="Stretch" Margin="0,4,0,0">Save IMU Samples</Button>

                </StackPanel>
            </ScrollViewer>
        </Border>

        <!-- ── LEFT PANEL ───────────────────────────────────────────────────── -->
        <DockPanel LastChildFill="True">

            <!-- Status grid (top, always visible) -->
            <Grid DockPanel.Dock="Top" Margin="8,6,8,4">
                <Grid.ColumnDefinitions>
                    <ColumnDefinition Width="*"/>
                    <ColumnDefinition Width="*"/>
                    <ColumnDefinition Width="*"/>
                </Grid.ColumnDefinitions>

                <!-- Col 0: System status -->
                <Border Grid.Column="0" Padding="10" CornerRadius="6" Background="#11000000" Margin="0,0,4,0">
                    <StackPanel>
                        <TextBlock Text="{Binding DeviceState}" FontSize="16" FontWeight="SemiBold"/>
                        <TextBlock Text="{Binding StatusLabel}" FontSize="18" FontWeight="Bold" Margin="0,4,0,8" TextWrapping="Wrap"/>
                        <UniformGrid Columns="2">
                            <TextBlock Text="校准状态:" Opacity="0.7" FontSize="11"/>
                            <TextBlock Text="{Binding CalibrationState}" FontSize="11"/>
                            <TextBlock Text="行进方向:" Opacity="0.7" FontSize="11"/>
                            <TextBlock Text="{Binding ProgDirDeg, StringFormat={}{0:F1}°}" FontSize="11"/>
                            <TextBlock Text="帧拒绝:" Opacity="0.7" FontSize="11"/>
                            <TextBlock Text="{Binding PelvisRejectStreak}" FontSize="11"/>
                            <TextBlock Text="步态:" Opacity="0.7" FontSize="11"/>
                            <TextBlock Text="{Binding StanceSummary}" FontSize="11"/>
                        </UniformGrid>

                        <!-- IMU connection indicators -->
                        <StackPanel Margin="0,8,0,0">
                            <StackPanel Orientation="Horizontal">
                                <Ellipse x:Name="PelvisStatusDot" Width="10" Height="10" Fill="Gray" Margin="0,0,6,0" VerticalAlignment="Center"/>
                                <TextBlock x:Name="PelvisStatusText" Text="Pelvis: --" FontSize="11"/>
                            </StackPanel>
                            <StackPanel Orientation="Horizontal" Margin="0,2,0,0">
                                <Ellipse x:Name="LeftStatusDot" Width="10" Height="10" Fill="Gray" Margin="0,0,6,0" VerticalAlignment="Center"/>
                                <TextBlock x:Name="LeftStatusText" Text="Left: --" FontSize="11"/>
                            </StackPanel>
                            <StackPanel Orientation="Horizontal" Margin="0,2,0,0">
                                <Ellipse x:Name="RightStatusDot" Width="10" Height="10" Fill="Gray" Margin="0,0,6,0" VerticalAlignment="Center"/>
                                <TextBlock x:Name="RightStatusText" Text="Right: --" FontSize="11"/>
                            </StackPanel>
                        </StackPanel>
                    </StackPanel>
                </Border>

                <!-- Col 1: Left FPA -->
                <Border Grid.Column="1" Padding="10" CornerRadius="6" Background="#11000000" Margin="4,0">
                    <StackPanel HorizontalAlignment="Center" VerticalAlignment="Center">
                        <TextBlock Text="Left FPA" FontSize="14" Opacity="0.75" HorizontalAlignment="Center"/>
                        <TextBlock Text="{Binding LeftFpaDeg, StringFormat={}{0:F1}°}"
                                   FontSize="48" FontWeight="Bold" HorizontalAlignment="Center"/>
                        <TextBlock Text="{Binding LeftFpaNote}" Opacity="0.7" HorizontalAlignment="Center"/>
                    </StackPanel>
                </Border>

                <!-- Col 2: Right FPA -->
                <Border Grid.Column="2" Padding="10" CornerRadius="6" Background="#11000000" Margin="4,0,0,0">
                    <StackPanel HorizontalAlignment="Center" VerticalAlignment="Center">
                        <TextBlock Text="Right FPA" FontSize="14" Opacity="0.75" HorizontalAlignment="Center"/>
                        <TextBlock Text="{Binding RightFpaDeg, StringFormat={}{0:F1}°}"
                                   FontSize="48" FontWeight="Bold" HorizontalAlignment="Center"/>
                        <TextBlock Text="{Binding RightFpaNote}" Opacity="0.7" HorizontalAlignment="Center"/>
                    </StackPanel>
                </Border>
            </Grid>

            <!-- Log toggle button -->
            <Button DockPanel.Dock="Bottom" Click="BtnToggleLog_Click"
                    HorizontalAlignment="Left" Margin="8,2,8,2" Padding="8,2">
                <TextBlock Text="{Binding LogPanelVisible,
                    Converter={x:Static local:BoolToToggleTextConverter.LogInstance}}"/>
            </Button>

            <!-- Log panel (collapsible, bottom) -->
            <TextBox
                DockPanel.Dock="Bottom"
                x:Name="LogBox"
                Height="160"
                AcceptsReturn="True"
                IsReadOnly="True"
                Text="{Binding LogList}"
                TextWrapping="Wrap"
                VerticalScrollBarVisibility="Auto"
                Margin="8,0,8,2"
                Visibility="{Binding LogPanelVisible, Converter={StaticResource BoolToVis}}"/>

            <!-- 3D viewports (fill remaining space) -->
            <UniformGrid Rows="1" Columns="3">

                <!-- Pelvis viewport -->
                <Grid>
                    <Viewport3D>
                        <Viewport3D.Camera>
                            <PerspectiveCamera Position="2,1.2,2" LookDirection="-2,-1.2,-2" UpDirection="0,1,0" FieldOfView="45"/>
                        </Viewport3D.Camera>
                        <ModelVisual3D>
                            <ModelVisual3D.Content>
                                <Model3DGroup>
                                    <AmbientLight Color="#666666"/>
                                    <DirectionalLight Color="#FFFFFF" Direction="-1,-1,-1"/>
                                    <DirectionalLight Color="#FFFFFF" Direction="1,-0.5,0.2"/>
                                </Model3DGroup>
                            </ModelVisual3D.Content>
                        </ModelVisual3D>
                        <ModelVisual3D x:Name="AxesVisual"/>
                        <ModelVisual3D x:Name="ImuVisual"/>
                    </Viewport3D>
                </Grid>

                <!-- Left foot viewport -->
                <Grid>
                    <Viewport3D>
                        <Viewport3D.Camera>
                            <PerspectiveCamera Position="2,1.2,2" LookDirection="-2,-1.2,-2" UpDirection="0,1,0" FieldOfView="45"/>
                        </Viewport3D.Camera>
                        <ModelVisual3D>
                            <ModelVisual3D.Content>
                                <Model3DGroup>
                                    <AmbientLight Color="#666666"/>
                                    <DirectionalLight Color="#FFFFFF" Direction="-1,-1,-1"/>
                                    <DirectionalLight Color="#FFFFFF" Direction="1,-0.5,0.2"/>
                                </Model3DGroup>
                            </ModelVisual3D.Content>
                        </ModelVisual3D>
                        <ModelVisual3D x:Name="AxesVisual1"/>
                        <ModelVisual3D x:Name="ImuVisual1"/>
                    </Viewport3D>
                </Grid>

                <!-- Right foot viewport -->
                <Grid>
                    <Viewport3D>
                        <Viewport3D.Camera>
                            <PerspectiveCamera Position="2,1.2,2" LookDirection="-2,-1.2,-2" UpDirection="0,1,0" FieldOfView="45"/>
                        </Viewport3D.Camera>
                        <ModelVisual3D>
                            <ModelVisual3D.Content>
                                <Model3DGroup>
                                    <AmbientLight Color="#FFFFFF" Direction="-1,-1,-1"/>
                                    <DirectionalLight Color="#FFFFFF" Direction="1,-0.5,0.2"/>
                                </Model3DGroup>
                            </ModelVisual3D.Content>
                        </ModelVisual3D>
                        <ModelVisual3D x:Name="AxesVisual12"/>
                        <ModelVisual3D x:Name="ImuVisual12"/>
                    </Viewport3D>
                </Grid>

            </UniformGrid>

        </DockPanel>
    </DockPanel>
</Window>
```

> **Note:** The XAML references `local:BoolToToggleTextConverter` — this must be created in Task 7.

- [ ] **Step 2: Build (expect a compile error about missing converter — Task 7 fixes it)**

```
dotnet build IMUMoCap/IMUMoCap.csproj
```

This will fail with "BoolToToggleTextConverter not found". That's expected — continue to Task 7.

---

### Task 7: Wire up MainWindow.xaml.cs — converters, param sync, restart, save diagnostics

**Files:**
- Modify: `IMUMoCap/MainWindow.xaml.cs`
- Create: `IMUMoCap/Converters/BoolToToggleTextConverter.cs`

- [ ] **Step 1: Create `IMUMoCap/Converters/BoolToToggleTextConverter.cs`**

This converter turns `true`/`false` into toggle button text (e.g. "隐藏日志" / "显示日志").

```csharp
// IMUMoCap/Converters/BoolToToggleTextConverter.cs
using System;
using System.Globalization;
using System.Windows.Data;

namespace IMUMoCap
{
    /// <summary>
    /// Converts bool visibility state to a toggle button label.
    /// Usage: x:Static local:BoolToToggleTextConverter.LogInstance
    /// </summary>
    public sealed class BoolToToggleTextConverter : IValueConverter
    {
        public string TrueText  { get; set; } = "隐藏";
        public string FalseText { get; set; } = "显示";

        public static readonly BoolToToggleTextConverter LogInstance = new()
        {
            TrueText  = "隐藏日志",
            FalseText = "显示日志",
        };

        public static readonly BoolToToggleTextConverter ParamsInstance = new()
        {
            TrueText  = "隐藏参数",
            FalseText = "显示参数",
        };

        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
            => value is bool b && b ? TrueText : FalseText;

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
            => throw new NotSupportedException();
    }
}
```

- [ ] **Step 2: Add param-sync subscription and new button handlers to `MainWindow.xaml.cs`**

In the `MainWindow()` constructor, after the pipeline event subscriptions block (after `_pipeline.OnFpaResult += ...`), add:

```csharp
            // Param sync: push VM param values into pipeline on every property change
            _content.PropertyChanged += (_, e) => SyncParamToPipeline(e.PropertyName);
```

- [ ] **Step 3: Add `SyncParamToPlugin()` private method**

Add this method near the bottom of the class:

```csharp
        private void SyncParamToPlugin(string? propName)
        {
            switch (propName)
            {
                case nameof(MainPageVM.StompThreshold):
                    _pipeline.Params.StompThreshold_ms2 = _content.StompThreshold;
                    break;
                case nameof(MainPageVM.StaticGyroThreshold):
                    _pipeline.Params.StaticGyroThreshold = _content.StaticGyroThreshold;
                    break;
                case nameof(MainPageVM.StanceFreeAccThreshold):
                    _pipeline.Params.StanceFreeAccThreshold = _content.StanceFreeAccThreshold;
                    break;
                case nameof(MainPageVM.StanceGyroThreshold):
                    _pipeline.Params.StanceGyroThreshold = _content.StanceGyroThreshold;
                    break;
                case nameof(MainPageVM.PdConfidenceThreshold):
                    _pipeline.Params.PdConfidenceThreshold = _content.PdConfidenceThreshold;
                    break;
                case nameof(MainPageVM.PdStabilityThreshold):
                    _pipeline.Params.PdStabilityThreshold = _content.PdStabilityThreshold;
                    break;
                case nameof(MainPageVM.MinBaselineSteps):
                    _pipeline.Params.MinBaselineSteps = _content.MinBaselineSteps;
                    break;
            }
        }
```

- [ ] **Step 4: Add the button click handlers**

Add these methods to `MainWindow.xaml.cs`:

```csharp
        private void BtnRestartCal_Click(object sender, RoutedEventArgs e)
        {
            _baselineTimer?.Stop();
            _baselineTimer = null;
            _pipeline.Reset();
            _content.StatusLabel = "Calibration reset. Stomp left foot to begin.";
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
                    Filter      = "CSV files (*.csv)|*.csv|All files (*.*)|*.*",
                    DefaultExt  = ".csv",
                    FileName    = $"Diagnostics_{DateTime.Now:yyyyMMdd_HHmmss}.csv",
                };
                if (dlg.ShowDialog() != true) return;

                using var sw = new System.IO.StreamWriter(dlg.FileName, append: false,
                    encoding: System.Text.Encoding.UTF8);
                sw.WriteLine(DiagnosticsRow.CsvHeader);
                foreach (var row in rows)
                    sw.WriteLine(row.ToCsvRow());

                log($"Diagnostics saved: {rows.Count} rows → {dlg.FileName}");
            }
            catch (Exception ex)
            {
                log($"Error saving diagnostics: {ex.Message}");
            }
        }

        private void BtnToggleLog_Click(object sender, RoutedEventArgs e)
        {
            _content.LogPanelVisible = !_content.LogPanelVisible;
        }

        private void BtnToggleParams_Click(object sender, RoutedEventArgs e)
        {
            _content.ParamsPanelVisible = !_content.ParamsPanelVisible;
        }
```

- [ ] **Step 5: Add missing using directive for `DiagnosticsRow` in `MainWindow.xaml.cs`**

At the top of `MainWindow.xaml.cs`, the `using IMUMoCap.Pipeline.Models;` is already present. Verify it's there; if not, add it alongside the other pipeline usings.

- [ ] **Step 6: Build to verify**

```
dotnet build IMUMoCap/IMUMoCap.csproj
```
Expected: `Build succeeded. 0 Error(s)`

If there are XAML `x:Static` errors for `BoolToToggleTextConverter`, ensure the converter file is in the `IMUMoCap` namespace (not a sub-namespace like `IMUMoCap.Converters`) OR update `xmlns:local` in the XAML to include the correct namespace. The converter class is declared `namespace IMUMoCap` so `xmlns:local="clr-namespace:IMUMoCap"` (already in the XAML) will find it.

- [ ] **Step 7: Manual smoke test**

1. Launch the app. Verify: right panel shows 7 sliders with labels.
2. Move the "跺脚阈值" slider — the label number updates in real time.
3. Click "隐藏参数" — sliders disappear. Click again — reappear.
4. Click "隐藏日志" — log TextBox disappears. Click again — reappears.
5. Click "重新开始校准" — log shows "Calibration restarted by user." and status label resets.
6. Click "保存诊断数据 (CSV)" — if no data, log shows "No diagnostics data to save."

- [ ] **Step 8: Commit**

```bash
git add IMUMoCap/Converters/BoolToToggleTextConverter.cs IMUMoCap/MainWindow.xaml IMUMoCap/MainWindow.xaml.cs IMUMoCap/VM/MainPageVM.cs
git commit -m "feat: redesign WPF layout — params panel, log toggle, restart cal, save diagnostics CSV"
```

---

## Self-Review

**Spec coverage check:**
- [x] Left-right split layout → Task 6
- [x] Status grid (always visible) → Task 6
- [x] Log panel collapsible → Tasks 5, 6, 7
- [x] 7 tunable parameters with sliders → Tasks 1, 5, 6, 7
- [x] Live parameter update (no restart needed) → Tasks 3, 7
- [x] 重新开始校准 button → Task 7
- [x] 保存诊断数据 CSV button → Tasks 2, 7
- [x] DiagnosticsRow CSV format (matches spec columns) → Task 2
- [x] `type:"state"` WebSocket broadcast → Task 4
- [x] CalibrationState → AR state mapping → Task 4
- [x] PipelineParams record → Task 1
- [x] Diagnostics buffer cleared on Reset() → Task 3

**Type consistency check:**
- `PipelineParams.StompThreshold_ms2` used in Task 1 and Task 3 (SyncParams) — matches.
- `DiagnosticsRow.CsvHeader` and `ToCsvRow()` used in Task 7 — match Task 2 definition.
- `_pipeline.GetDiagnosticsSnapshot()` returns `List<DiagnosticsRow>` — matches Task 3 return type.
- `MainPageVM.StompThreshold` (float) maps to `PipelineParams.StompThreshold_ms2` (float) — both float, compatible.
- `BoolToToggleTextConverter.LogInstance` / `ParamsInstance` referenced in XAML as `x:Static local:BoolToToggleTextConverter.LogInstance` — `local` namespace is `clr-namespace:IMUMoCap`, class is in `namespace IMUMoCap` — correct.

**Placeholder scan:** No TBD/TODO/placeholder patterns found in plan steps.
