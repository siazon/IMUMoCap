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
| `Assets/Scenes/FPATraining.unity` | Main/only training scene (new; not yet wired into Build Settings — see Known Issues) |
| `Assets/XR/Settings/XREALSettings.asset` | XREAL SDK config (tracking mode, supported devices) |

There is no local copy of the WS protocol spec; code comments reference
`docs/superpowers/specs/2026-07-29-ws-protocol.md`, which lives outside this repo (likely
alongside the backend). Treat `ServerMessage` in `FootWebSocketClient.cs` as the
current source of truth for the wire format from the Unity side.

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

**Message envelope**: every server→client message is JSON with a `type` discriminator,
deserialized into one shared `ServerMessage` class (not polymorphic — all fields present,
irrelevant ones left at default). See the class definition at the bottom of
`FootWebSocketClient.cs` for the authoritative field list and which `type` populates which
fields.

**Server → Client message types**:

| `type` | Purpose | Consumed by |
|---|---|---|
| `state` | Session state machine transition | `FootHudController.OnStateChange` |
| `fpa` | Per-step angle feedback during a training block (has on-target signal) | `OnFpaUpdate` |
| `live` | Continuous real-time angle stream (no on-target signal) | `OnLiveUpdate` |
| `stepProgress` | Step-count progress during baseline/retention | `OnStepProgress` |
| `redo` | Operator forced a redo of current stage | `OnRedo` |
| `pong` | Heartbeat ack | (no-op) |

**Client → Server commands** (`FootWebSocketClient` public methods, plain-text JSON, no envelope):
- `SendReadyForCalibrationAsync()` → `{"cmd":"ReadyForCalibration"}`
- `SendContinueTrainingAsync(fromBlock)` → `{"cmd":"continueTraining","fromBlock":N}`

Both are fired from HUD confirm-button clicks, never automatically.

**State machine** (`state` field values, handled in `FootHudController.OnStateChange`):
`waiting → armed → calibrating → baseline → training ⇄ rest ⇄ retention → ended`, with
`paused` and `error` reachable from most states. `armed` and post-`rest` require an explicit
operator/user confirm tap (`Start` / `Continue` buttons) before the client tells the server to
proceed — the server does not auto-advance past those gates.

**Training condition** (`condition` field: `""` | `"EF"` | `"IF"`) selects which graphic group
is shown during `training`:
- `EF` (stepping stones): fixed target angle cue, set once from `state.targetL/R` +
  `directionL/R` (toe-in/toe-out), not updated per-step
- `IF` (footprint): rotates and colors (green/red) per-step from `fpa.errorL/R` +
  `fpa.onTargetL/R`

**NaN/no-data handling**: the backend sends `NaN` (bare or as string) for a foot's angle
fields when that foot has no data this packet. `JsonUtility` can't deserialize `NaN`/`null`
into a `float`, so `HandleMessage` string-replaces those tokens with `0` *before* parsing,
but first scans the raw JSON to set `fpaLIsNaN`/`fpaRIsNaN` flags so downstream code can
distinguish "genuinely zero" from "no data — skip rendering this foot this frame."

## Scene structure (`FPATraining.unity`)

Single scene, one `Canvas`. Relevant hierarchy under it (names match the public fields wired
in the Inspector on `FootHudController`):

- `FootprintRoot` (IF condition) → `FootL`/`FootOutlineL`, `FootR`/`FootOutlineR`
- `StoneRoot` (EF condition) → `StoneL`, `StoneR`
- `CalibOverlay` → text + `Button` (repurposed for both "Start" and "Continue" prompts,
  label/handler swapped at runtime) + `Slider` (step-count progress bar)

`FootHudController` toggles `FootprintRoot`/`StoneRoot`/`CalibOverlay` active state as mutually
exclusive groups depending on `state`/`condition` — never assumes more than one is visible at once.

## Known issues / open threads for research

- **Build Settings inconsistency**: `ProjectSettings/EditorBuildSettings.asset` currently
  enables the now-deleted `SampleScene.unity` and disables `HelloMR.unity`; the new
  `FPATraining.unity` isn't in the scene list at all. A build right now would reference a
  missing scene and omit the actual training scene. Needs fixing before this branch ships.
- **No local WS protocol spec**: `docs/superpowers/specs/2026-07-29-ws-protocol.md` is
  referenced in comments but not present in this repo — worth locating (backend repo? shared
  docs system?) before making further protocol changes, to avoid drifting from the
  server-side contract.
- **`serverUrl` is hardcoded** to a specific ICS IP (`192.168.137.1`) with no environment/build
  config switch — fine for a fixed lab rig, fragile if the PC's hotspot IP or network topology
  changes.
- **`XREALSettings.asset`** changed `InitialTrackingType` (1→2) and `SupportDevices` (bitmask
  changed) as part of this working set — not yet investigated whether intentional or a side
  effect of editing settings in the Unity Editor while working on the scene.
