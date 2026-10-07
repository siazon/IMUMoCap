# AR Frontend — IMUARFPA

Unity AR client running on XREAL smart glasses. Displays real-time foot-progression-angle
(FPA) gait training feedback, driven entirely by a WebSocket connection to a PC-side backend.
This doc exists to orient an agent doing further research/dev on this codebase — it does not
duplicate content already derivable by reading the code (which it links to).

## Stack

- Unity 2022.3.62f3
- XREAL XR SDK (`com.xreal.xr`, local tarball dependency — not on a registry)
- `com.unity.xr.interaction.toolkit` 2.6.5, `com.unity.xr.hands` 1.8.0
- UI: uGUI (`Canvas` + `TextMeshPro`), no custom render pipeline work involved in this feature
- No third-party networking/JSON libs — raw `System.Net.WebSockets.ClientWebSocket` +
  Unity's built-in `JsonUtility`

## Key files

| File | Role |
|---|---|
| `Assets/FootWebSocketClient.cs` | WebSocket transport: connect/reconnect/heartbeat, JSON parse, dispatches typed messages to the HUD on the main thread |
| `Assets/FootHudController.cs` | Consumes parsed messages, drives UI state machine and foot-graphic rendering |
| `Assets/FeedbackColors.cs` | Shared on-target/off-target color palette (see Colors below) — single source of truth for both EF and IF |
| `Assets/Scenes/FPATraining.unity` | Main/only training scene; build index 0, sole enabled scene in `EditorBuildSettings.asset` |
| `Assets/XR/Settings/XREALSettings.asset` | XREAL SDK config (tracking mode, supported devices) |

There is no local copy of the WS protocol spec; code comments reference
`docs/superpowers/specs/2026-07-29-ws-protocol.md`, which lives outside this repo (likely
alongside the backend). Treat `ServerMessage` in `FootWebSocketClient.cs` as the current source
of truth for the wire format from the Unity side — it has since diverged from that spec (the
`live` message type was retired, then reinstated as an IF-only stream alongside the new
`ifRenderMode` field, see below) and the spec doc has not been re-synced.

## Backend communication

**Transport**: single persistent WebSocket, `ws://192.168.137.1:8765/ws/` (Inspector-editable
`serverUrl` field on `FootWebSocketClient`). PC backend is the WS *server*; the glasses are the
*client*. No auth/handshake beyond the WS upgrade — assumes trusted local network (glasses and
PC on the same hotspot/LAN, hence the `192.168.137.x` ICS-style address).

**Lifecycle** (`FootWebSocketClient.cs`):
- `Start()` spawns a background `ConnectionLoop` task (does not block the Unity main thread)
- Auto-reconnect with exponential backoff: `reconnectInitialDelaySec` (1s) →
  `reconnectMaxDelaySec` (10s), doubling each failed attempt
- Heartbeat: `{"cmd":"ping"}` every `heartbeatIntervalSec` (10s); server replies `{"type":"pong"}`
- All socket I/O happens off the main thread; results are marshalled to Unity via a
  `ConcurrentQueue<Action>` drained in `Update()` — required because Unity API calls
  (transform/UI mutation) are not thread-safe
- Server-side sends are now serialized per-client with a `SemaphoreSlim` (`WebSocketBroadcastServer.cs`,
  PC side) — earlier, concurrent `BroadcastJsonAsync` calls from different message types
  (live/fpa/state/redo/stepProgress) could race, throw on the underlying `WebSocket.SendAsync`,
  and get the client silently dropped from the server's client list even though this socket was
  still open. Worth knowing if the glasses ever appear to "stop receiving updates" without an
  explicit disconnect — check the PC-side client count rather than assuming it's this client's bug.

**Message envelope**: every server→client message is JSON with a `type` discriminator,
deserialized into one shared `ServerMessage` class (not polymorphic — all fields present,
irrelevant ones left at default). See the class definition at the bottom of
`FootWebSocketClient.cs` for the authoritative field list and which `type` populates which
fields.

**Server → Client message types**:

| `type` | Purpose | Consumed by |
|---|---|---|
| `state` | Session state machine transition | `FootHudController.OnStateChange` |
| `fpa` | Per-step angle feedback during a training block (has on-target signal) — the terminal-pulse trigger in both conditions | `OnFpaUpdate` |
| `live` | IF-only real-time angle stream, 10Hz, no on-target signal — see "IF rendering modes" below | `OnLiveUpdate` |
| `stepProgress` | Step-count progress during baseline/retention | `OnStepProgress` |
| `redo` | Operator forced a redo of current stage | `OnRedo` |
| `pong` | Heartbeat ack | (no-op) |

`live` was retired, then reinstated: the PC backend broadcasts it unconditionally whenever
IF + training + not retention/not rest (10Hz, throttled server-side), independent of which IF
render mode is active — see "IF rendering modes" below. `state` additionally carries an
`ifRenderMode` field (`"placeholder"` | `"live"` | `"lastResult"`) that tells the AR client which
of the three IF render paths to use; it is read in `OnStateChange` and falls back to
`"placeholder"` when absent
(old/un-upgraded PC server, or `JsonUtility`'s default null for an unset string field) — EF is
entirely unaffected by this field.

**Client → Server commands** (`FootWebSocketClient` public methods, plain-text JSON, no envelope):
- `SendReadyForCalibrationAsync()` → `{"cmd":"ReadyForCalibration"}` — fired from the HUD's `Start`
  confirm-button click, never automatically. Routed by the PC to `TriggerCalibration()` (session's
  first calibration) or `TriggerRecalibration()` (pre-stage recalibration, see below), depending on
  whether a stage transition is currently pending.

`continueTraining`/`SendContinueTrainingAsync` was removed: the PC now auto-advances out of `rest`
on its own stopwatch once the minimum duration elapses, straight into the pre-stage recalibration
prompt (`armed`) below — that prompt's own `Start` tap already serves as the resume signal, so a
separate `Continue` tap is redundant.

**State machine** (`state` field values, handled in `FootHudController.OnStateChange`):
`waiting → armed → calibrating → baseline → training ⇄ rest → armed → calibrating → training ⇄ ... → retention → ended`,
with `paused` and `error` reachable from most states. Every stage transition into Training1/2/3 or
Retention is gated by a pre-stage recalibration (re-bounds IMU yaw drift — see
docs/task19_calibration_duration_tuning.py): `AdvanceStage()` broadcasts `armed` with a
recalibration-specific `reason` ("Quick recheck before continuing...") instead of entering the
stage directly; the participant taps `Start`, `calibrating` plays out again (same "please stand
still" mechanism as the session's first calibration, also text-overridden via `reason`), and only
once that completes does the stage actually begin. `rest` no longer needs a `Continue` tap — it
auto-advances into this same `armed` gate once its minimum duration elapses.

**Terminal-pulse rendering** (`condition` field: `""` | `"EF"` | `"IF"` selects which graphic
group is shown during `training`): both conditions are visible for the **entire** training
block (dwell-time matched — see "Dwell-time symmetry" below) and only pulse on an `fpa` message
(i.e. only on an actual foot-contact event). Both now share the same "slightly enlarged" scale
pulse (`PulseScale`, 1.05x) — EF scales+colors in place (never rotated), IF additionally rotates
to the measured error angle. The scale always reverts unconditionally after `FootOutDisplaySec`
(0.35s, shared constant in `FootHudController.cs`) via `Invoke`/`CancelInvoke`; whether
rotation/color also revert, or persist, differs by condition/render mode (see below).

- **EF (stepping stones)**: `stoneLeft`/`stoneRight` show a fixed target angle/position, set
  once per block from `state.targetL/R` + `directionL/R` (toe-in/toe-out) via
  `ApplyStoneTargets()` — never updated per-step. On each `fpa` message,
  `FootHudController.PulseStone()` scales the stone itself up to `PulseScale` (1.05x) in
  place — never rotated or hidden — and tints it `FeedbackColors.OnTarget`/`OffTarget`
  (blue/orange, matching IF's palette) by `msg.onTargetL`/`msg.onTargetR`;
  `ResetStoneLeft()`/`ResetStoneRight()` revert scale and color (back to the stone's normal
  texture color, cached from the `Image`'s color at `Start()`) after `FootOutDisplaySec`. This
  replaced an earlier `footOutL`/`footOutR` overlay
  that rotated to the actual measured angle and flashed on top of the stone — removed so
  attention stays on the stone's fixed target rather than a rotating overlay; those fields/
  GameObjects no longer exist in `FootHudController.cs` (see Scene structure below).
- **IF (footprint), placeholder mode** (`state.ifRenderMode == "placeholder"` — see "IF
  rendering modes" below for the full set, and which one actually ships as default):
  `leftSolid`/`rightSolid` are visible for the whole block, set to an idle pose by
  `ResetLeftSolid()`/`ResetRightSolid()` in `ShowGraphicGroup()` — zero rotation, scale 1, the
  `Image`'s own cached texture color. This idle pose is a dwell-time placeholder only; it does
  **not** represent any real-time angle measurement (see "Dwell-time symmetry" below). On each
  `fpa` message, `FootHudController.FlashFoot()` rotates to `msg.errorL`/`msg.errorR`
  (= FPA − target), scales up to `PulseScale`, and colors by `msg.onTargetL`/`msg.onTargetR`,
  then reverts *everything* (rotation, scale, color) to the idle pose (not hidden) after the same
  `FootOutDisplaySec` window as EF's pulse.

**IF rendering modes** (`state.ifRenderMode`, `FootHudController._ifRenderMode`): a runtime
toggle (PC-side `ParamsDialog` checkboxes, bound to `MainPageVM.IsIfLiveMode`/
`IsIfLastResultMode`, mutually exclusive — checking one
unchecks the other) lets a researcher switch IF between three render paths live, mid-pilot,
without pausing/reconnecting — EF is never affected by this field. All three share the same
`FlashFoot()` rotate+scale+color pulse on `fpa`, and in all three the **scale always reverts**
unconditionally after `FootOutDisplaySec` (`ResetLeftSolid/RightSolidScale`, scale-only, called
alongside whichever rotation/color revert the mode picks — see `PulseIfSolid()`); only
rotation/color persistence differs per mode:

- **`"placeholder"`**: idle pose + fpa-triggered rotate/scale/color pulse that fully reverts
  (rotation included) after `FootOutDisplaySec` — see above.
- **`"live"`**: `leftSolid`/`rightSolid` still start the block at the same zero-rotation idle
  pose, but now also consume the `live` message (`OnLiveUpdate`) — IF-only, broadcast at 10Hz
  by the PC backend whenever IF + training + not retention/not rest, independent of
  `ifRenderMode` (the toggle only selects how the AR client renders, not whether the PC side
  sends). `OnLiveUpdate` rotates straight to `msg.angleL`/`msg.angleR` without waiting for `fpa`
  and without touching color — `live` carries no on-target signal, so color is left at whatever
  it currently is. `fpa` messages still trigger `FlashFoot()`'s rotate+scale+color pulse exactly
  as in placeholder mode, but the post-pulse rotation/color revert
  (`RevertLeftSolidColor()`/`RevertRightSolidColor()`) only fades the color back — rotation is
  left alone so it keeps tracking the live stream instead of snapping back to 0 (the scale still
  reverts, same as every mode). If neither a `live` nor an `fpa` update lands for a foot within
  `liveStaleThresholdSec` (0.5s, Inspector-tunable), `Update()` reverts that foot to the full
  idle pose (rotation included) so it never gets stuck showing a stale angle from a foot that's
  gone inactive. [Task 15's pilot consistency check](task15_live_vs_fpa_consistency_report.md)
  found `live` doesn't track the confirmed `fpa` result closely enough to recommend it outside
  pilot comparison — see that report before using `"live"` for real data collection.
- **`"lastResult"`** (**default** — `MainPageVM.IsIfLastResultMode` defaults `true` (its
  `ParamsDialog` checkbox ships checked), and `OnStateChange`
  falls back to `"lastResult"` when `state.ifRenderMode` is absent, e.g. an un-upgraded PC
  server): never consumes `live` at all (`OnLiveUpdate` returns immediately whenever
  `_ifRenderMode != "live"`, `"lastResult"` included). `fpa` still triggers `FlashFoot()`'s
  rotate+scale+color pulse, but the post-pulse rotation/color revert target is `NoRevert()` — a
  no-op — so rotation+color are never reverted: the icon just keeps showing the previous step's
  confirmed result (angle + on/off-target color) indefinitely, until the next `fpa` message
  overwrites it with a new pulse (the scale still reverts after `FootOutDisplaySec`, same as
  every mode — only the icon's size pulse is transient, its angle/color are sticky). No staleness
  fallback (unlike `"live"`) — persisting the last result indefinitely is the intended behavior,
  not a bug to guard against.

**Dwell-time symmetry (EF/IF)**: EF's stone is visible for the entire block by design (fixed
target cue). IF's footprint previously was hidden by default and only shown during its 0.35s
terminal flash — an asymmetry in on-screen dwell time between conditions, independent of the
EF/IF (external/internal focus) manipulation the study is meant to isolate, and documented as a
confound risk for H2a (guidance-hypothesis literature: a continuously available visual reference
changes motor-learning dynamics regardless of focus instructions — Salmoni, Schmidt, & Walter,
1984; Winstein & Schmidt, 1990). IF's icon is now also visible for the whole block in all three
render modes, at an idle pose, to match EF's dwell time. In placeholder mode that idle pose is
deliberately **not** a low-fidelity real-time angle preview — rotation is fixed at
0° and color is the icon's own cached texture color, never `FeedbackColors.OnTarget`/
`OffTarget` — a continuously-updated live-angle stream was initially considered and rejected for
the *default* behavior on these grounds: swing-phase angle estimates are less accurate than the
settled stance-phase value `fpa` already reports, and continuous rotation would both create a
"preview vs confirmed value" mismatch and reintroduce a motion-onset visual-capture confound
(motion automatically captures attention independent of task relevance — Abrams & Christ, 2003,
*Psychological Science*; Jonides & Yantis, 1988, *Perception & Psychophysics*). The `"live"`
render mode above reinstates that stream, but only as an explicit, researcher-toggled opt-in for
pilot comparison — not the shipped default — so these objections still apply to it and should be
weighed before using `"live"` outside pilot testing.

**Colors** (`Assets/FeedbackColors.cs`): on-target/off-target used to be `Color.green`/
`Color.red`; both conditions pull from this one shared static class instead —
`OnTarget` = blue `RGB(0,114,178)`, `OffTarget` = orange `RGB(230,159,0)` (Wong 2011,
*Nature Methods*, colorblind-safe palette). Change the palette in exactly one place if it
ever needs to change again. EF's stone pulse now uses the full `OnTarget`/`OffTarget` pair too
(matches IF's flash colors) — it reverts to the stone's own cached texture color only after the
pulse ends, not on-target vs off-target.

**Focus reminder** (`focusReminderPanel`/`focusReminderText`, new Inspector fields on
`FootHudController`): at the start of every training block (not on resume-from-pause),
`ShowGraphicGroup()` runs immediately (footprint/stone become visible right away), and
`ShowFocusReminderThenHide()` overlays a condition-specific instruction text below the graphic
for `focusReminderDurationSec` (3s), then hides just the text — the graphic stays. Text content
is word-count-matched between conditions (`FocusReminderTextEF`/`FocusReminderTextIF`, 10 words
each) so neither condition's pre-block instruction reads as measurably longer/more complex.
**The `focusReminderPanel` GameObject is not created by any script** — it was added by hand in
the Unity Editor and must be wired into the two new Inspector fields on `FootHudController`
manually if the scene is ever rebuilt from scratch.

**NaN/no-data handling**: the backend sends `NaN` (bare or as string) for a foot's angle
fields when that foot has no data this packet. `JsonUtility` can't deserialize `NaN`/`null`
into a `float`, so `HandleMessage` string-replaces those tokens with `0` *before* parsing,
but first scans the raw JSON to set `fpaLIsNaN`/`fpaRIsNaN` flags (from `fpaL`/`fpaR`/`errorL`/
`errorR`) — and, for `live`, `angleLIsNaN`/`angleRIsNaN` flags (from `angleL`/`angleR`) — so
downstream code can distinguish "genuinely zero" from "no data — skip rendering this foot this
frame."

## Scene structure (`FPATraining.unity`)

Single scene, one `Canvas`. Relevant hierarchy under it (names match the public fields wired
in the Inspector on `FootHudController`):

- `FootprintRoot` (IF condition) → `FootL`, `FootR`: visible for the whole block at an idle pose
  (dwell-time matched to EF's always-visible stone, see "Dwell-time symmetry" above), pulsing
  per-step via the terminal-pulse mechanism described above — no longer hidden-by-default.
  `FootOutlineL`/`FootOutlineR` still exist in the scene but are dead: no C# field references
  them anymore (the old outline-vs-solid rendering split was removed), and their `m_IsActive`
  was set to `0` in the scene file so they don't show up as a stray leftover overlay.
- `StoneRoot` (EF condition) → `StoneL`, `StoneR`: each has a `RectTransform` + `Image` wired to
  `stoneLeft`/`stoneRight` + `stoneLeftImage`/`stoneRightImage` on `FootHudController`. Now
  textured (a sprite assigned on the `Image`'s Source Image) — the earlier "plain semi-
  transparent rectangle, not a placeholder awaiting art" note is stale. `FootOutL`/`FootOutR`
  still exist in the scene but are now dead the same way `FootOutlineL`/`FootOutlineR` are: no
  C# field references them anymore (the footprint-overlay-on-stone flash was replaced by the
  stone's own scale/color pulse, see Terminal-pulse rendering above) — leave them alone or
  delete them, they're inert.
- `CalibOverlay` → text + `Button` (the `Start` confirm — reused verbatim for both the session's
  first calibration and every pre-stage recalibration, only the overlay text differs via `reason`)
  + `Slider` (step-count progress bar)
- Focus-reminder text object (hand-added, see above) — not part of the original prefab/scene
  authoring, wired manually into `FootHudController`'s Inspector.

`FootHudController` toggles `FootprintRoot`/`StoneRoot`/`CalibOverlay` active state as mutually
exclusive groups depending on `state`/`condition` — never assumes more than one is visible at once.

## Known issues / open threads for research

- **WS protocol spec location**: the spec this client's wire format must match,
  `docs/superpowers/specs/2026-07-29-ws-protocol.md`, lives in the PC-backend side of this same
  monorepo (`IMUMoCap/../docs/superpowers/specs/`), not inside `IMUARFPA/`, and **has not been
  updated to reflect the `live`/`ifRenderMode` churn** described above (retired, then
  reinstated with different semantics) — treat `ServerMessage` in `FootWebSocketClient.cs` as
  the current source of truth over that spec doc until it's re-synced.
- **`live.angleL`/`angleR` and `fpa.errorL`/`errorR` are NOT the same reference frame**:
  `live` is foot yaw relative to the *live* pelvis heading (calibration-offset removed, no
  gait-event gating — see `RawFootYawDeg` on the PC side); `fpa.errorL`/`errorR` is relative to
  the PD (progression-direction) estimate, only emitted once a foot has settled in stance. The
  two approximately coincide while walking a straight line, but are not exactly equivalent —
  most visibly right after a turn, before PD has reacquired. In IF live mode this means the
  continuously-rotating icon is a **swing-phase visual reference only, not a precise
  measurement**, and can visibly "jump" when an `fpa` pulse lands (switching briefly to the
  PD-frame error angle) before reverting to tracking the pelvis-frame `live` angle again. This
  needs to be disclosed in the preregistration wherever IF live mode's angle display is
  described — it is not interchangeable with the `fpa` value IF placeholder mode and EF both
  render exclusively.
- **`serverUrl` is hardcoded** to a specific ICS IP (`192.168.137.1`) with no environment/build
  config switch — fine for a fixed lab rig, fragile if the PC's hotspot IP or network topology
  changes.
- **`XREALSettings.asset`** changed `InitialTrackingType` (1→2) and `SupportDevices` (bitmask
  changed) as part of an earlier working set — not yet investigated whether intentional or a side
  effect of editing settings in the Unity Editor while working on the scene.
- **`FootOutDisplaySec` (0.35s), `focusReminderDurationSec` (3s), `PulseScale` (1.05x, shared by
  EF and IF), and `liveStaleThresholdSec` (0.5s, IF live mode only) are hand-tuned constants**,
  not derived from
  a fresh data-driven analysis the way the original 0.2s EF flash duration was
  (`docs/task8_footprint_display_duration.py`) — worth re-running that kind of check if the
  flash ever seems to linger into the next same-foot step, feels too fast/slow to read
  on-glasses, the pulse scale feels too subtle/strong, or a foot's live-mode icon freezes/resets
  more or less eagerly than intended.
