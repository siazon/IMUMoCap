# AR HUD Unity App (XREAL) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Create a Unity AR app for XREAL glasses that receives WebSocket messages from the PC and displays a 2D HUD: state-driven instruction text and real-time green/red footprint feedback during training.

**Architecture:** Unity 2022 LTS Camera-space Canvas. A single `WsClient` MonoBehaviour manages the WebSocket connection and fires C# events. `HudController` owns the Canvas and drives the 5 display states. `FootprintDisplay` manages the two footprint Image widgets. The PC broadcasts `type:"fpa"` (footprint update) and `type:"state"` (phase transition) JSON messages; the AR app only receives — it never sends.

**Tech Stack:** Unity 2022 LTS, XREAL NRSDK (6DoF tracking + display), NativeWebSocket (WebSocket client), `Newtonsoft.Json` or `UnityEngine.JsonUtility` (JSON parsing), C# 9.

---

## Prerequisites (Manual Setup — do before Task 1)

1. Install Unity 2022 LTS (2022.3.x).
2. Create a new 3D project named `IMUMoCapAR`.
3. Import **XREAL NRSDK** via Package Manager (add `https://nreal-public.github.io/NRSDKForUnityAndroid/` scoped registry or import `.unitypackage` from XREAL developer portal).
4. Import **NativeWebSocket** via UPM: `https://github.com/endel/NativeWebSocket.git#upm`
   - In Unity: `Window → Package Manager → + → Add package from git URL`
5. Set Build Target: `File → Build Settings → Android`.
6. In `Player Settings`: set minimum API level to 26, enable `ARCore` and `Internet` permissions.
7. In `NRProjectConfig`, enable `SixDof` and `RGBCamera` as needed.

---

## File Structure

| File | Action | Responsibility |
|---|---|---|
| `Assets/Scripts/WsClient.cs` | Create | WebSocket connection, reconnect, raw message events |
| `Assets/Scripts/Messages.cs` | Create | JSON message DTOs (`FpaMessage`, `StateMessage`) |
| `Assets/Scripts/HudController.cs` | Create | Canvas state machine, subscribes to WsClient events |
| `Assets/Scripts/FootprintDisplay.cs` | Create | Tints left/right footprint Image widgets green/red |
| `Assets/Scenes/HUD.unity` | Create (manual) | Unity Scene with Camera-space Canvas wired to scripts |

---

### Task 1: WsClient — WebSocket connection and message dispatch

**Files:**
- Create: `Assets/Scripts/WsClient.cs`

This MonoBehaviour owns the WebSocket connection. It reconnects automatically 3s after disconnect. It fires `OnMessage(string)` on the main thread so UI code can safely update Unity components.

- [ ] **Step 1: Create `Assets/Scripts/WsClient.cs`**

```csharp
// Assets/Scripts/WsClient.cs
using System;
using System.Threading.Tasks;
using NativeWebSocket;
using UnityEngine;

/// <summary>
/// Manages the WebSocket connection to the IMUMoCap PC server.
/// Reconnects automatically every ReconnectDelaySec seconds after disconnect.
/// Fires OnRawMessage on the Unity main thread.
/// </summary>
public class WsClient : MonoBehaviour
{
    [Header("Connection")]
    [Tooltip("Full WebSocket URL, e.g. ws://192.168.137.1:8765/ws/")]
    public string ServerUrl = "ws://192.168.137.1:8765/ws/";

    [Tooltip("Seconds to wait before reconnecting after disconnect")]
    public float ReconnectDelaySec = 3f;

    /// <summary>Fired on the main thread when a text message arrives.</summary>
    public event Action<string> OnRawMessage;

    private WebSocket _ws;
    private bool _reconnecting;

    private void Start() => _ = Connect();

    private void Update()
    {
        // NativeWebSocket requires DispatchMessageQueue() each frame to route
        // incoming messages to the main thread.
#if !UNITY_WEBGL || UNITY_EDITOR
        _ws?.DispatchMessageQueue();
#endif
    }

    private async Task Connect()
    {
        _reconnecting = false;

        _ws = new WebSocket(ServerUrl);

        _ws.OnOpen    += () => Debug.Log($"[WsClient] Connected to {ServerUrl}");
        _ws.OnClose   += code => { Debug.Log($"[WsClient] Closed: {code}"); ScheduleReconnect(); };
        _ws.OnError   += err  => { Debug.LogWarning($"[WsClient] Error: {err}"); ScheduleReconnect(); };
        _ws.OnMessage += bytes =>
        {
            string text = System.Text.Encoding.UTF8.GetString(bytes);
            OnRawMessage?.Invoke(text);
        };

        try
        {
            await _ws.Connect();
        }
        catch (Exception ex)
        {
            Debug.LogWarning($"[WsClient] Connect failed: {ex.Message}");
            ScheduleReconnect();
        }
    }

    private void ScheduleReconnect()
    {
        if (_reconnecting) return;
        _reconnecting = true;
        Invoke(nameof(Reconnect), ReconnectDelaySec);
    }

    private void Reconnect()
    {
        _ = Connect();
    }

    private async void OnApplicationQuit()
    {
        if (_ws != null && _ws.State == WebSocketState.Open)
            await _ws.Close();
    }
}
```

- [ ] **Step 2: Verify compile (Unity Editor)**

In Unity Editor, open `Window → Console`. The project should show 0 compile errors. If NativeWebSocket is not installed you'll see `namespace NativeWebSocket not found` — install the package first (see Prerequisites).

- [ ] **Step 3: Commit**

```bash
git add Assets/Scripts/WsClient.cs
git commit -m "feat: add WsClient MonoBehaviour with auto-reconnect WebSocket"
```

---

### Task 2: Message DTOs

**Files:**
- Create: `Assets/Scripts/Messages.cs`

Two message types arrive from the PC:
- `type:"state"` → `StateMessage`
- `type:"fpa"` → `FpaMessage`

Both are parsed from JSON using `JsonUtility` (Unity built-in, no extra packages).

- [ ] **Step 1: Create `Assets/Scripts/Messages.cs`**

```csharp
// Assets/Scripts/Messages.cs
using System;
using UnityEngine;

[Serializable]
public class BaseMessage
{
    public string type;
}

/// <summary>
/// {"type":"state","state":"waiting"|"calibrating"|"baseline"|"training"|"error"}
/// </summary>
[Serializable]
public class StateMessage
{
    public string type;
    public string state;
}

/// <summary>
/// {"type":"fpa","fpaL":-3.2,"fpaR":4.1,"onTargetL":true,"onTargetR":false,"errorL":...,"errorR":...}
/// Note: the PC sends packetId, fpaL, fpaR, onTargetL, onTargetR, errorL, errorR.
/// </summary>
[Serializable]
public class FpaMessage
{
    public string type;
    public long   packetId;
    public float  fpaL;
    public float  fpaR;
    public bool   onTargetL;
    public bool   onTargetR;
    public float  errorL;
    public float  errorR;
}

/// <summary>
/// Utility to parse incoming WebSocket messages into typed objects.
/// </summary>
public static class MessageParser
{
    public static (FpaMessage? fpa, StateMessage? state) Parse(string json)
    {
        try
        {
            var baseMsg = JsonUtility.FromJson<BaseMessage>(json);
            if (baseMsg == null) return (null, null);

            switch (baseMsg.type)
            {
                case "fpa":   return (JsonUtility.FromJson<FpaMessage>(json), null);
                case "state": return (null, JsonUtility.FromJson<StateMessage>(json));
                default:      return (null, null);
            }
        }
        catch (Exception ex)
        {
            Debug.LogWarning($"[MessageParser] Failed to parse: {json}\n{ex.Message}");
            return (null, null);
        }
    }
}
```

- [ ] **Step 2: Verify compile in Unity Editor**

`Window → Console` — 0 errors expected.

- [ ] **Step 3: Commit**

```bash
git add Assets/Scripts/Messages.cs
git commit -m "feat: add FpaMessage and StateMessage DTOs with MessageParser"
```

---

### Task 3: FootprintDisplay — green/red footprint icons

**Files:**
- Create: `Assets/Scripts/FootprintDisplay.cs`

Controls two `UnityEngine.UI.Image` components (left foot, right foot). Each image is tinted green (`#00CC44`) when `onTarget == true` and red (`#FF3333`) when `onTarget == false`. The entire display is hidden when not in training state.

- [ ] **Step 1: Create `Assets/Scripts/FootprintDisplay.cs`**

```csharp
// Assets/Scripts/FootprintDisplay.cs
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// Controls the two footprint Image widgets.
/// Call SetVisible(true) in training state, SetVisible(false) in all other states.
/// Call UpdateFootprints(onTargetL, onTargetR) on each FPA message.
/// </summary>
public class FootprintDisplay : MonoBehaviour
{
    [Header("Footprint Images")]
    public Image LeftFootImage;
    public Image RightFootImage;

    private static readonly Color OnTargetColor  = new Color(0f,    0.80f, 0.267f, 1f); // #00CC44
    private static readonly Color OffTargetColor = new Color(1f,    0.20f, 0.20f,  1f); // #FF3333
    private static readonly Color DefaultColor   = new Color(0.75f, 0.75f, 0.75f,  1f); // grey when no data

    private void Awake()
    {
        // Start hidden; HudController enables us in training state
        gameObject.SetActive(false);
    }

    /// <summary>Show or hide the entire footprint widget.</summary>
    public void SetVisible(bool visible) => gameObject.SetActive(visible);

    /// <summary>Update footprint colors based on FPA result.</summary>
    public void UpdateFootprints(bool onTargetL, bool onTargetR)
    {
        if (LeftFootImage  != null) LeftFootImage.color  = onTargetL ? OnTargetColor : OffTargetColor;
        if (RightFootImage != null) RightFootImage.color = onTargetR ? OnTargetColor : OffTargetColor;
    }

    /// <summary>Reset to grey (e.g. on entering training state before first FPA arrives).</summary>
    public void ResetColors()
    {
        if (LeftFootImage  != null) LeftFootImage.color  = DefaultColor;
        if (RightFootImage != null) RightFootImage.color = DefaultColor;
    }
}
```

- [ ] **Step 2: Verify compile in Unity Editor — 0 errors.**

- [ ] **Step 3: Commit**

```bash
git add Assets/Scripts/FootprintDisplay.cs
git commit -m "feat: add FootprintDisplay for green/red footprint HUD icons"
```

---

### Task 4: HudController — state machine

**Files:**
- Create: `Assets/Scripts/HudController.cs`

Subscribes to `WsClient` events and drives the HUD. There are 5 states:

| State string | Instruction text shown |
|---|---|
| `waiting` | 请用左脚跺地开始校准 |
| `calibrating` | 校准中，请保持静止站立 |
| `baseline` | Baseline 采集中，请自然行走 |
| `training` | (hide text, show footprints) |
| `error` | 校准失败，请等待重新开始 |

- [ ] **Step 1: Create `Assets/Scripts/HudController.cs`**

```csharp
// Assets/Scripts/HudController.cs
using UnityEngine;
using TMPro;  // TextMeshPro — use UnityEngine.UI.Text if TMP not installed

/// <summary>
/// Drives the 2D HUD state machine.
/// Assign WsClient, InstructionText, and FootprintDisplay in Inspector.
/// </summary>
public class HudController : MonoBehaviour
{
    [Header("References")]
    public WsClient        WsClient;
    public TMP_Text        InstructionText;   // Center-screen text
    public FootprintDisplay FootprintDisplay;

    private void OnEnable()
    {
        if (WsClient != null)
            WsClient.OnRawMessage += HandleMessage;
    }

    private void OnDisable()
    {
        if (WsClient != null)
            WsClient.OnRawMessage -= HandleMessage;
    }

    private void Start()
    {
        // Show waiting state by default until a state message arrives
        ApplyState("waiting");
    }

    private void HandleMessage(string json)
    {
        var (fpa, state) = MessageParser.Parse(json);

        if (state != null)
            ApplyState(state.state);

        if (fpa != null)
            HandleFpa(fpa);
    }

    private void ApplyState(string state)
    {
        switch (state)
        {
            case "waiting":
                ShowText("请用左脚跺地开始校准");
                FootprintDisplay.SetVisible(false);
                break;

            case "calibrating":
                ShowText("校准中，请保持静止站立");
                FootprintDisplay.SetVisible(false);
                break;

            case "baseline":
                ShowText("Baseline 采集中，请自然行走");
                FootprintDisplay.SetVisible(false);
                break;

            case "training":
                HideText();
                FootprintDisplay.SetVisible(true);
                FootprintDisplay.ResetColors();
                break;

            case "error":
                ShowText("校准失败，请等待重新开始");
                FootprintDisplay.SetVisible(false);
                break;

            default:
                Debug.LogWarning($"[HudController] Unknown state: {state}");
                break;
        }
    }

    private void HandleFpa(FpaMessage fpa)
    {
        // Only update footprints when in training state (footprints visible)
        if (FootprintDisplay.gameObject.activeSelf)
            FootprintDisplay.UpdateFootprints(fpa.onTargetL, fpa.onTargetR);
    }

    private void ShowText(string text)
    {
        if (InstructionText == null) return;
        InstructionText.gameObject.SetActive(true);
        InstructionText.text = text;
    }

    private void HideText()
    {
        if (InstructionText == null) return;
        InstructionText.gameObject.SetActive(false);
    }
}
```

- [ ] **Step 2: Verify compile — 0 errors**

If TMP is not installed: `Window → Package Manager → TextMeshPro → Install`. If you prefer not to use TMP, replace `TMP_Text` with `UnityEngine.UI.Text` and remove the `using TMPro;` line.

- [ ] **Step 3: Commit**

```bash
git add Assets/Scripts/HudController.cs
git commit -m "feat: add HudController state machine driving 5 AR HUD states"
```

---

### Task 5: Unity Scene setup (manual, in Unity Editor)

**Files:**
- Create: `Assets/Scenes/HUD.unity` (save from Unity Editor)

This task is entirely manual in the Unity Editor.

- [ ] **Step 1: Create a new Scene named `HUD`**

`File → New Scene → Basic (Built-in)` → Save as `Assets/Scenes/HUD.unity`.

- [ ] **Step 2: Set up NRSDK Camera Rig**

Delete the default `Main Camera`. In `Project` window, locate `NRCameraRig` prefab (from NRSDK package) in `Assets/NRSDK/Prefabs/` and drag it into the Hierarchy.

- [ ] **Step 3: Add a Camera-space Canvas**

In Hierarchy: `Right-click → UI → Canvas`.
- Set `Render Mode` = `Screen Space - Camera`
- Set `Render Camera` = the `NRCameraRig`'s `CenterCamera` child (or `LeftCamera` — use `CenterCamera` for monoscopic HUD)
- Set `Plane Distance` = `2` (2 meters in front of camera)

- [ ] **Step 4: Add InstructionText (center of canvas)**

In the Canvas: `Right-click → UI → TextMeshPro → Text (UI)`.
- Name it `InstructionText`
- `Rect Transform`: Anchors = center-middle, Pos X=0, Pos Y=50, Width=800, Height=200
- Font Size = 48, Alignment = Center/Middle
- Set text to `请用左脚跺地开始校准` (preview)

- [ ] **Step 5: Add FootprintPanel (bottom third of canvas)**

In the Canvas: `Right-click → Create Empty`. Name it `FootprintPanel`.
- `Rect Transform`: Anchors = bottom-center, Pos X=0, Pos Y=100, Width=500, Height=200

Inside `FootprintPanel`, add two Image objects:
- `Right-click → UI → Image` → Name `LeftFootImage`
  - Anchors = center-left, Pos X=-100, Pos Y=0, Width=160, Height=180
  - Source Image: assign a footprint sprite (see Step 6)
- `Right-click → UI → Image` → Name `RightFootImage`
  - Anchors = center-right, Pos X=100, Pos Y=0, Width=160, Height=180
  - Source Image: assign a footprint sprite (mirror of left)

- [ ] **Step 6: Add footprint sprites**

Create two simple sprites (or use placeholder `UI/Default`):
- In `Project`: `Right-click → Create → 2D → Sprites → Square` — this gives a white square as placeholder
- Import proper footprint PNG images: place `foot_left.png` and `foot_right.png` in `Assets/Sprites/`
- In `Texture Importer`: set `Texture Type = Sprite (2D and UI)`
- Assign them to `LeftFootImage.Source Image` and `RightFootImage.Source Image`

If no artwork is available yet, leave as the default white square (the green/red tinting will still be visible).

- [ ] **Step 7: Create an empty GameObject `WsClientHost` and attach `WsClient`**

In Hierarchy: `Right-click → Create Empty`. Name it `WsClientHost`.
- Add Component → `WsClient`
- Set `Server Url` to `ws://192.168.137.1:8765/ws/` (edit to match your PC's LAN IP)

- [ ] **Step 8: Create an empty GameObject `HudHost` and attach `HudController` + `FootprintDisplay`**

In Hierarchy: `Right-click → Create Empty`. Name it `HudHost`.

Add Component → `FootprintDisplay`:
- Assign `LeftFootImage` field → drag the `LeftFootImage` Image object from hierarchy
- Assign `RightFootImage` field → drag the `RightFootImage` Image object

Add Component → `HudController`:
- Assign `WsClient` field → drag `WsClientHost`
- Assign `InstructionText` field → drag `InstructionText` TMP object
- Assign `FootprintDisplay` field → drag `HudHost` (itself, since FootprintDisplay is on the same GO)

- [ ] **Step 9: Save scene. Verify in Play mode (PC with WS server running)**

Press Play in Unity Editor. Watch Console:
- Expected: `[WsClient] Connected to ws://...` (if PC WS server is running)
- If PC is not running: `[WsClient] Error: ...` then 3s later attempts reconnect

Test state transitions by sending test messages from the PC:
```json
{"type":"state","state":"training"}
{"type":"fpa","fpaL":3.2,"fpaR":-1.1,"onTargetL":true,"onTargetR":false,"errorL":0.2,"errorR":-2.1,"packetId":1}
```

Expected: text hides, footprint panel appears, left = green, right = red.

- [ ] **Step 10: Commit**

```bash
git add Assets/Scenes/HUD.unity Assets/Scripts/
git commit -m "feat: complete AR HUD scene with NRSDK camera-space Canvas, WS client, and footprint display"
```

---

### Task 6: Android build and device deploy

- [ ] **Step 1: Build Android APK**

`File → Build Settings → Android`:
- Add `Assets/Scenes/HUD.unity` to build
- Click `Build`
- Output: `Build/IMUMoCapAR.apk`

- [ ] **Step 2: Install on XREAL glasses**

Connect glasses via USB. Run:
```bash
adb install -r Build/IMUMoCapAR.apk
```
Expected: `Success`

- [ ] **Step 3: Launch and verify end-to-end**

1. Launch `IMUMoCapAR` on glasses.
2. Start WPF app on PC. Ensure both are on the same LAN.
3. Connect IMUs — verify "请用左脚跺地开始校准" text appears on AR glasses.
4. Stomp left foot — verify text changes to "校准中，请保持静止站立" on AR glasses.
5. After calibration completes — text changes to "Baseline 采集中，请自然行走".
6. After 3-minute baseline — footprints appear. Walk → verify green/red feedback.

- [ ] **Step 4: Final commit**

```bash
git add .
git commit -m "feat: AR HUD complete — XREAL Unity app with state machine and footprint feedback"
```

---

## Self-Review

**Spec coverage check:**
- [x] Camera-space Canvas 2D HUD → Task 5
- [x] WebSocket receive-only client → Task 1
- [x] Auto-reconnect (3s) → Task 1 (`ReconnectDelaySec = 3`)
- [x] 5 display states (waiting/calibrating/baseline/training/error) → Task 4
- [x] State driven by `type:"state"` messages → Tasks 2, 4
- [x] Footprints in training state only → Task 4 (`SetVisible` calls)
- [x] Latest step only (no history) → Task 3 (single left/right Image, overwritten each FPA message)
- [x] Green = onTarget, Red = not onTarget → Task 3
- [x] Footprints centered bottom-third → Task 5 step 5
- [x] Instruction text centered → Task 5 step 4
- [x] `type:"fpa"` message fields (fpaL, fpaR, onTargetL, onTargetR) → Task 2 `FpaMessage` fields match the PC's broadcast in `MainWindow.xaml.cs`

**Type consistency check:**
- `MessageParser.Parse()` returns `(FpaMessage?, StateMessage?)` — consumed in `HudController.HandleMessage()` — types match.
- `FootprintDisplay.UpdateFootprints(bool, bool)` called with `fpa.onTargetL, fpa.onTargetR` — bool fields in `FpaMessage` — correct.
- `HudController.WsClient.OnRawMessage` is `event Action<string>` — subscribed with `HandleMessage(string json)` — signature matches.

**Placeholder scan:** No TBD/TODO/placeholder patterns found.
