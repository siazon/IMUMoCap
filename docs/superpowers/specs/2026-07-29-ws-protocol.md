# WebSocket Protocol Spec — PC ↔ AR HUD

Date: 2026-07-29

Supersedes the WS message shapes in `2026-04-09-ui-design.md` / `2026-04-09-ar-unity-xreal.md`
(those assumed a receive-only AR client and 5 flat states; this spec adds EF/IF condition,
training block number, the Training2→Training3 rest period, and AR→PC commands).

## Design constraint: Unity `JsonUtility` has no nullable support

The AR app parses messages with `JsonUtility.FromJson<T>`, which does not support `int?`/nullable
value types. Every message uses fixed non-nullable fields with sentinel defaults instead of
omitting/nulling fields:

- `condition`: `string` — `""` before Start Condition, else `"EF"` / `"IF"`
- `block`: `int` — `0` when not in a training phase, else `1`/`2`/`3`
- `restDurationSec`: `int` — `0` unless `state == "rest"`
- `reason`: `string` — `""` when not applicable

---

## Interaction model

Stage transitions are **automatic**: each training block ends on 150 valid steps/foot or the 5-min
timeout, and the pipeline advances the stage itself. The **AR client** initiates the two
participant-driven actions — start (`ReadyForCalibration`) and post-rest/post-pause continue
(`continueTraining`). The **operator** buttons (Next Stage, Pause/Resume) are an emergency/manual
fallback only, not the normal path.

---

## Notification coverage by flow stage

| # | Flow point | Trigger | AR display group | Current status |
|---|---|---|---|---|
| 1 | Start Condition | Operator click | Text group: `state=armed` | Done — `state=armed` carries `condition` (EF/IF) |
| 2 | Calibration triggered | Button or AR `ReadyForCalibration` | Text group: `state=calibrating` | OK |
| 3 | Calibration complete | Static pose collected | Text + progress bar: `state=baseline` | OK |
| 4 | Baseline step progress | Each collected step | Progress bar | Done — `stepProgress` (stage=baseline) |
| 5 | Calibration/Baseline failed | Timeout or insufficient steps | Text group: `state=error` | OK |
| 6 | Baseline done → Training1 | Steps threshold met | EF stepping-stones / IF footprints (by condition) | Done — `state=training`, `block=1`, `condition` set |
| 7 | Per-step feedback | Each valid step | Same graphic group | Done — `fpa` carries `stage`/`block`/`ts` |
| 8 | Training1 → Training2 | 150 steps/foot or 5-min timeout | Same graphic group, `block` changes | Done — `AdvanceStage` broadcasts `state=training` with the new `block` |
| 9 | Training2 → Rest | Same trigger as #8 | Switch to text group | Done — `state=rest`, `block=2`, `restDurationSec=120` |
| 10 | Rest countdown ends → Continue button shown | 120s elapsed | Text group button sub-state | Timer runs client-side; no extra PC message needed |
| 11 | User taps Continue → Training3 starts | AR→PC command | Switch to graphic group, `block=3` | Done — `continueTraining` handler (in-rest + `fromBlock==2` + 120s checks) → Training3 |
| 12 | Training3 → Retention | Same trigger as #8 | Text group + step-progress bar (no feedback graphics) | Done — `state=retention`, `fpa` stops, `stepProgress` (stage=retention) drives the bar |
| 12b | Retention ends → flow closes | 100 steps/foot **or** 5-min cap | Text group: `state=ended` | Done — auto-ends flow, saves session file, broadcasts `state=ended` |
| 13 | Pause (operator, emergency) | Button click | Text group: `state=paused` | Done — broadcasts `state=paused` |
| 14 | Resume — Continue | Dialog choice | Text "keep walking" for current stage, → graphics on first `fpa` | Done — re-broadcasts current stage `state` |
| 15 | Resume — Redo | Dialog choice | Same as #14, plus reset locally-tracked counters | Done — broadcasts `redo` (stage/attempt/reason) then re-broadcasts stage `state` |
| 16 | End Condition (operator) | Recording closed | Switch to text group | Done — operator End and auto-end (#12b) both broadcast `state=ended` |
| 17 | ping heartbeat | AR sends `ping` | N/A | Done — replies `pong` to the sender only (`SendJsonAsync`) |
| 18 | Client reconnect | New/re- connection | Apply current snapshot | Done — server sends the current `state` to the connecting client only (`SendJsonAsync`) |

---

## PC → AR (broadcast)

Every PC → AR message carries `ts` (server broadcast time, unix ms) so the AR app can detect and
discard stale messages after a reconnect.

### `state` — all phase/status transitions

One shape reused for every transition (armed/waiting/calibrating/baseline/training/rest/
retention/paused/ended/error) — new phases are just new `state` string values, not new message
types.

```json
{
  "type": "state",
  "state": "training",
  "condition": "EF",
  "block": 1,
  "restDurationSec": 0,
  "reason": "",
  "targetL": 8.5, "directionL": "toe-out",
  "targetR": 7.2, "directionR": "toe-out",
  "ts": 1737955200000
}
```

Field values by phase:

| state | condition | block | restDurationSec | reason |
|---|---|---|---|---|
| `waiting` | `""` | 0 | 0 | `""` |
| `armed` | `"EF"`/`"IF"` | 0 | 0 | `""` |
| `calibrating` | same | 0 | 0 | `""` |
| `baseline` | same | 0 | 0 | `""` |
| `training` | same | 1/2/3 | 0 | `""` |
| `rest` | same | 2 (block just finished) | 120 | `""` |
| `retention` | same | 0 | 0 | `""` |
| `paused` | same | block at time of pause | 0 | `""` |
| `ended` | same | 0 | 0 | `""` |
| `error` | same | 0 | 0 | e.g. `"Calibration timeout"` |

**`targetL`/`directionL`/`targetR`/`directionR`** — the participant's personalised FPA target angle
(degrees) and direction (`"toe-in"`/`"toe-out"`), from the baseline-driven target `Tᵢ = μᵢ ± k·SDᵢ`
(Research Overview §2.4). Constant for the whole condition once baseline completes — sentinel
`0`/`""` before that. **This is the only thing EF's stepping stone renders** — see the EF/IF
consumption note under `fpa` below.

**Washout no longer exists** — the study moved to a two-day design (one condition per day), so each
day is just a normal EF or IF condition; there is no separate washout recording or state.

### `fpa` — per-step feedback

Sent **only during a training block**, for **both** EF and IF — not sent during `baseline` or
`retention` (both show the text group + `stepProgress` bar, no target graphics — Research Overview §2.6).

**EF/IF consume this message differently:**
- **IF** uses the whole thing: `errorL`/`errorR` drive the footprint's live rotation, `onTargetL`/
  `onTargetR` drive its green/red color, updated every step.
- **EF ignores `errorL`/`errorR`/`onTargetL`/`onTargetR`/`quality` entirely.** The stepping stone is a
  pure external target cue — its rotation comes only from `state.targetL`/`directionL`/`targetR`/
  `directionR` (fixed for the whole condition) and never changes based on whether the participant
  actually lands on it. No color/success feedback is rendered for EF by design (external-focus
  encoding must not reintroduce an internal evaluative signal — see Research Overview §1.3).
- PC still computes and broadcasts full `fpa` during EF regardless — the per-step measurement is
  required for the research outcomes (FPA target achievement rate, adaptation trend, RQ2) and is
  written to the session CSV independently of what the AR client displays.

On resume after a pause (states #14/#15), the AR client shows a "keep walking" text prompt for the
current stage and only switches back to the graphic group when the **next `fpa` arrives** — so the
participant is walking before feedback graphics reappear. (For EF this just means: stone reappears
once walking resumes: there's no color/onTarget state to restore.)

```json
{
  "type": "fpa",
  "stage": "training",
  "block": 2,
  "packetId": 12345,
  "fpaL": -3.2, "fpaR": 4.1,
  "onTargetL": true, "onTargetR": false,
  "errorL": 0.2, "errorR": -2.1,
  "confidence": 0.92, "stability": 0.88,
  "quality": "good",
  "ts": 1737955200000
}
```

### `stepProgress` — new

Drives the text-group step-count progress bar during `baseline` and `retention` (both show a
progress bar, no feedback graphics). `requiredL/R` mirror the PC-side step targets: `MinBaselineSteps`
(default 20) for baseline, 100 for retention.

Retention auto-ends the whole condition flow — closes and saves the session file and broadcasts
`state=ended` — as soon as both feet reach 100 valid steps or the 5-min cap elapses (whichever first).

```json
{
  "type": "stepProgress",
  "stage": "baseline",
  "stepsL": 8, "stepsR": 6,
  "requiredL": 20, "requiredR": 20,
  "ts": 1737955200000
}
```

### `redo` — new

Sent when the operator redoes the current stage (Pause dialog → Redo), so the AR app resets any
locally-tracked counters/progress for that stage.

```json
{
  "type": "redo",
  "stage": "Training2",
  "attempt": 2,
  "reason": "Participant tripped",
  "ts": 1737955200000
}
```

### `pong` — heartbeat reply

Sent back to the pinging client only (not broadcast).

```json
{ "type": "pong", "ts": 1737955200000 }
```

---

## AR → PC (commands, `{"cmd": "..."}`)

### `ping` (existing)

```json
{ "cmd": "ping" }
```

### `ReadyForCalibration` (existing)

```json
{ "cmd": "ReadyForCalibration" }
```

### `continueTraining` — new

Sent when the participant taps the Continue button after the Training2→3 rest period.

```json
{ "cmd": "continueTraining", "fromBlock": 2 }
```

`fromBlock` is an idempotency/staleness guard: PC only advances to Training3 if it is currently
in `rest` state AND its own record of "block just finished" matches `fromBlock`. A mismatched or
duplicate command is ignored and logged, not acted on.

**Server-side minimum-duration check**: PC records the timestamp it entered `rest` and rejects
`continueTraining` if fewer than 120s have elapsed, even though the AR button is only supposed to
appear after its own local countdown — the PC must not trust the client's timer alone for
protocol-timing integrity.

---

## Naming consistency note

Existing `cmd` values are inconsistently cased (`ping` lowercase, `ReadyForCalibration`
PascalCase, `Start`/`stop`/`Stop` mixed) — this is pre-existing and out of scope to fix here.
`continueTraining` follows the lowerCamelCase style already used for JSON *fields* (`fpaL`,
`onTargetL`) rather than the existing `cmd` capitalization. **Decided and implemented as
`continueTraining`** (lowerCamelCase); the pre-existing mixed-case `cmd` values are left as-is.

---

## Design decisions

1. **`condition` is repeated on every `state` message** (not sent once at `armed`). The AR client is
   fully stateless: it just applies the latest snapshot, so a mid-session reconnect never loses the
   EF/IF assignment. Cost is a few bytes per message — negligible.
2. **The "rest started at" timestamp lives in `MainWindow.xaml.cs`, reusing the existing
   `_stageStopwatch`.** It already `Restart()`s on every stage entry, so on `continueTraining` the PC
   checks `_stageStopwatch.Elapsed >= 120s` before advancing to Training3. `GaitPipeline.cs` stays
   IMU/FPA-only; protocol timing is not pushed into it.

## Implementation status

Done in `MainWindow.xaml.cs`: retention auto-end (100 steps/5-min → save + `state=ended`),
baseline/retention `fpa` suppression, pause/resume `state` broadcasts, `stepProgress`
(baseline + retention), and the full `state` shape — `condition`/`block`/`restDurationSec`/`reason`/`ts`
on every broadcast, with `state=training` re-sent on each block entry so `block` tracks 1→2→3.
The rest flow is done too: Training2 completion enters `state=rest` (block 2, 120s) instead of
auto-advancing, and `continueTraining` advances to Training3 only after the in-rest + `fromBlock==2`
+ real-120s-elapsed checks pass (operator Next Stage is the manual fallback). Operator End now also
broadcasts `state=ended`, and a (re)connecting client is sent the current `state` snapshot (targeted
via `SendJsonAsync`, so existing clients aren't disturbed). The `redo` message (Pause→Redo) and the
`block`/`ts` fields on `fpa` are done too.

**All 18 flow points in the coverage table are now implemented.** Remaining follow-ups are non-blocking:
the mixed-case `cmd` values (`ping`/`ReadyForCalibration`/`Start`/`Stop`) are left as-is by decision.

**Note — rest CSV tagging:** during the 120s rest, `_currentStage` stays `"Training2"` (rest is not a
`ConditionStages` entry), so the standing-rest diagnostics rows are written under the Training2 stage
tag. Impact is negligible (FPA is per valid step; standing produces essentially none), so this is left
as-is; pause recording during rest if full data isolation is ever required.
