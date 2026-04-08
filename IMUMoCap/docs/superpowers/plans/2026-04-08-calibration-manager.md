# Calibration Manager Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add installation-error calibration driven by a left-foot stomp, then apply the solved offsets to all downstream calibrated output.

**Architecture:** Keep calibration intentionally compact: one state/result model plus one manager class. The manager owns state transitions, static-pose sample collection, robust-center solving, and profile updates; `MainWindow` only feeds frames in and reads state out.

**Tech Stack:** C#, .NET/WPF, `System.Numerics`, existing IMU pipeline models.

---

### Task 1: Add calibration state model

**Files:**
- Modify: `d:\SourceCode\IMUMoCap\IMUMoCap\Model\CalibratedFrameModels.cs`

- [ ] Add a `CalibrationStatus` enum for `Idle`, `WaitingForStart`, `CollectingStaticPose`, `Completed`, and `Failed`.
- [ ] Add a `CalibrationStateModel` class carrying status, message, current profile, collected sample count, and success/failure metadata.
- [ ] Extend `CalibrationProfile` only as needed to keep solved offsets and helper accessors in one place.

### Task 2: Implement calibration manager

**Files:**
- Create: `d:\SourceCode\IMUMoCap\IMUMoCap\Methods\CalibrationManager.cs`
- Modify: `d:\SourceCode\IMUMoCap\IMUMoCap\Methods\CalibrationModule.cs`

- [ ] Add a compact `CalibrationManager` that:
  - waits for all IMUs to be connected,
  - arms in `WaitingForStart`,
  - starts collection on left-foot stomp,
  - buffers candidate static-pose frames,
  - solves a robust-center profile,
  - exposes a current `CalibrationStateModel`.
- [ ] Keep the solve heuristic simple and robust:
  - use low-motion frames as candidates,
  - tolerate brief disturbances,
  - use the dominant static orientation cluster,
  - derive foot forward from horizontal heading,
  - derive pelvis vertical from gravity and pelvis forward from average foot forward.
- [ ] Keep `CalibrationModule` as the profile applier, but make sure it can consume the manager’s solved profile immediately after completion.

### Task 3: Wire calibration into MainWindow

**Files:**
- Modify: `d:\SourceCode\IMUMoCap\IMUMoCap\MainWindow.xaml.cs`

- [ ] Instantiate the calibration manager and seed it with `WaitingForStart` when all three IMUs are connected.
- [ ] Feed completed quality frames and gait stomp detections into the manager.
- [ ] Update the existing session/UI state from the manager’s status so the frontend can show:
  - waiting to start,
  - collecting static pose,
  - calibration completed,
  - calibration failed.
- [ ] Switch the active calibration profile in `CalibrationModule` once the manager completes.

### Task 4: Keep exports and logging aligned

**Files:**
- Modify: `d:\SourceCode\IMUMoCap\IMUMoCap\Methods\CsvUtil.cs`
- Modify: `d:\SourceCode\IMUMoCap\IMUMoCap\MainWindow.xaml.cs`

- [ ] Add calibration status/profile summary fields to pipeline export only if they are cheap and unambiguous.
- [ ] Keep runtime logging lightweight: state transitions only, no per-frame calibration spam.

### Task 5: Verify by reasoning and constrained replay

**Files:**
- No code changes required unless verification reveals a clear issue.

- [ ] Sanity-check these paths:
  - before stomp: state remains `WaitingForStart`
  - after left stomp: state enters `CollectingStaticPose`
  - once enough stable-consensus samples exist: state becomes `Completed`
  - downstream calibrated output reads the new profile
- [ ] Document any remaining limitations, especially the current `obj\Debug\net8.0-windows` build lock that prevents full build verification.
