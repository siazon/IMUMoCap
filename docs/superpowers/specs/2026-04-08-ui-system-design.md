# IMUMoCap UI System Design

Date: 2026-04-08

## Goal

Design a two-end UI system for IMUMoCap:

- A WPF desktop console as the only control surface
- An AR glasses display as a passive WebSocket client that only renders feedback and never sends control messages back

The design should support the full gait-training flow from device preparation to calibration, baseline collection, live testing, and experiment completion.

## Product Direction

### Control Model

The system uses a single-master interaction model:

- WPF is the source of truth for workflow state, device state, experiment state, and target parameters
- AR glasses only receive pushed display state over WebSocket
- All state transitions are initiated on the WPF side

This keeps orchestration, logging, and safety on the desktop side and minimizes AR-side complexity.

### UI Split

The UI is intentionally divided into two roles:

- WPF: control console plus monitoring dashboard
- AR: real-time participant-facing feedback

## WPF UI Design

### Overall Layout

The WPF main window should move from the current flat layout to a three-zone, flow-driven console:

1. Top workflow strip
2. Main content area with control cards on the left and monitoring on the right
3. Bottom log/debug drawer

This makes the current experiment phase, available next action, and device health easy to understand at a glance.

### Top Workflow Strip

Purpose:

- Show where the session currently is
- Reduce operator confusion
- Make the next step obvious

Suggested stages:

- Device Connection
- Stomp Prompt
- Calibration
- Baseline
- Testing
- Finished

Suggested visual states:

- Gray: not started
- Blue: active
- Green: completed
- Red: blocked or error

Primary operator actions should also live in this strip or directly below it:

- Start Calibration
- Start Baseline
- Start Test
- Pause/Stop
- Save Session
- Restart Session

### Left Control Column

This area should be organized as workflow cards rather than loose buttons.

Suggested cards:

#### Device Card

Shows:

- Pelvis / left / right IMU connection state
- Battery or signal quality if available
- AR WebSocket connection state
- Current sample rate

#### Calibration Card

Shows:

- Current instruction such as "Please stomp"
- Current calibration status
- Calibration complete / failed / needs retry

Actions:

- Start calibration
- Recalibrate

#### Baseline Card

Shows:

- Baseline active/inactive
- Time elapsed or remaining
- Data quality readiness

Actions:

- Start baseline
- Cancel baseline

#### Test Card

Shows:

- Test running / paused / finished
- Personalized left/right FPA target
- Whether current gait is on target

Actions:

- Start test
- Pause/stop
- End and save

### Right Monitoring Column

This area should answer: "Is the participant currently moving correctly?"

Suggested sections:

#### Real-Time FPA Cards

Large, high-priority cards for:

- Left FPA
- Right FPA

Each card should show:

- Current measured value
- Target value
- On-target/off-target state
- Optional short note such as "too toe-out" or "good"

#### Sensor / Pose Visualization

Keep the existing 3D views, but visually subordinate them beneath the main performance metrics.

Use them for:

- Pelvis orientation status
- Left IMU orientation status
- Right IMU orientation status

These are diagnostic aids, not the primary operator focus.

#### Training Summary

A compact text summary should explain the current session state in plain language, for example:

- Left foot near target
- Right foot exceeds target
- Waiting for calibration
- Baseline in progress

### Bottom Log Drawer

Detailed logs remain useful for development and troubleshooting but should not dominate the main UI.

Behavior:

- Collapsed or compact by default
- Expandable when debugging
- Includes clear button and auto-scroll option

## AR UI Design

### Overall Principle

The AR glasses UI should be minimal, glanceable, and single-purpose.

The participant is walking, so the UI must avoid dense text, unstable layouts, or competing signals.

The AR display has two modes only:

1. Prompt mode
2. Footprint feedback mode

### Prompt Mode

Used before the live gait test starts or whenever the system is paused or blocked.

Supported prompts:

- Please stomp
- Calibrating
- Calibration complete
- Baseline in progress
- Test starting
- Test paused
- Test complete
- Connection error

Visual behavior:

- Large centered text
- High contrast
- Optional subtle translucent backing plate
- Neutral palette, avoiding green/red performance signaling during setup phases

### Footprint Feedback Mode

Used only during the formal testing phase.

Display:

- Left footprint
- Right footprint

Meaning:

- Green footprint: that foot currently meets the personalized FPA target
- Red footprint: that foot does not currently meet the target

Design constraints:

- Fixed screen placement
- Symmetric left/right arrangement
- No extra dashboard widgets
- Optional short hint text only if it improves behavior quickly

The footprint graphic should be a clear footprint silhouette or outline rather than a complex anatomical rendering.

## State Model

The same shared session state should drive both WPF and AR.

### Canonical Phase Field

Recommended phase values:

- `idle`
- `stomp_prompt`
- `calibrating`
- `calibrated`
- `baseline`
- `testing`
- `paused`
- `finished`
- `error`

### WPF Mapping

`phase` drives:

- Workflow strip highlighting
- Main status headline
- Which control actions are enabled
- Summary text shown to the operator

### AR Mapping

`phase` drives:

- Prompt mode content for all non-testing phases
- Footprint feedback mode when `phase = testing`

AR should not infer phase from raw sensor values. It should only render the explicit pushed state.

## WebSocket Payload Design

WPF should broadcast a single self-contained JSON payload for AR rendering.

Recommended payload:

```json
{
  "type": "gait_feedback",
  "phase": "testing",
  "statusText": "测试中",
  "leftOnTarget": true,
  "rightOnTarget": false,
  "leftFpa": 6.2,
  "rightFpa": 11.4,
  "leftTargetFpa": 6.0,
  "rightTargetFpa": 7.0,
  "hintText": "右脚再收一点",
  "timestamp": 1775635200000
}
```

### Payload Rules

- Send complete state snapshots, not partial diffs
- Keep field names stable
- Allow AR to redraw from the latest message only
- Include `statusText` for direct prompt rendering
- Include target and measured values so desktop and AR can share the same logic and diagnostics if needed

## Component Responsibilities

### WPF

- Own experiment workflow transitions
- Collect and process IMU state
- Determine personalized targets
- Evaluate left/right on-target status
- Broadcast display state over WebSocket
- Show logs and diagnostics

### AR

- Maintain a WebSocket connection
- Parse the latest broadcast payload
- Render prompt mode or footprint mode
- Recover cleanly from disconnects by showing a connection-related prompt

## Error Handling

### WPF Errors

Handle:

- Missing IMU
- Calibration failure
- Baseline cancellation
- WebSocket server unavailable
- Test interrupted

Operator-facing behavior:

- Show explicit status in the workflow strip
- Disable invalid next actions
- Keep error text visible until acknowledged or recovered

### AR Errors

Handle:

- No connection yet
- Connection lost mid-session
- Invalid or stale payload

Participant-facing behavior:

- Fall back to a simple prompt such as "Connection error" or "Waiting for system"
- Avoid showing outdated footprint success/failure after signal loss

## Testing Strategy

### WPF UI

Verify:

- Phase transitions enable the correct controls
- Status strip stays consistent with internal session state
- FPA cards update correctly from live data
- Log drawer does not interfere with normal workflow operation

### AR UI

Verify:

- Prompt mode shows the correct text for every non-testing phase
- Testing mode switches reliably to footprint feedback
- Green/red footprint state matches `leftOnTarget` and `rightOnTarget`
- Disconnects clear the old feedback and show a safe fallback prompt

### Integration

Verify:

- WPF and AR stay in sync from one shared payload schema
- Phase changes are reflected on AR without extra client logic
- Restarting a session resets both desktop and AR presentation correctly

## Recommended Implementation Order

1. Introduce a canonical session phase model in WPF
2. Refactor the WPF layout into workflow strip, control cards, monitoring area, and log drawer
3. Centralize the payload builder for WebSocket broadcast
4. Build the AR prompt mode
5. Build the AR footprint mode
6. Add integration checks for each workflow phase

## Scope Boundaries

This design intentionally does not include:

- AR-side control inputs
- AR-to-WPF messages
- Detailed biomechanics algorithms
- Rich AR scene anchoring or world-locked 3D spatial UI

Those can be considered later if needed, but they are not required for the first stable version.
