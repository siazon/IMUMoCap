using System;
using System.Collections.Concurrent;
using System.Net.Http;
using System.Net.WebSockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;

public class FootWebSocketClient : MonoBehaviour
{
    [Header("Server")]
    public string serverUrl = "ws://192.168.137.1:8765/ws/";
    public FootHudController hudController;

    [Header("Reconnect")]
    public bool autoReconnect = true;
    public float reconnectInitialDelaySec = 1f;
    public float reconnectMaxDelaySec = 10f;
    public float connectTimeoutSec = 5f;

    [Header("Heartbeat")]
    public bool enableHeartbeat = true;
    public float heartbeatIntervalSec = 10f;

    private ClientWebSocket ws;
    private CancellationTokenSource cts;
    private readonly ConcurrentQueue<Action> mainThreadActions = new();
    private volatile bool stopping;
    private volatile int _sessionGen = 0;

    private async void Start()
    {
        stopping = false;
        _ = Task.Run(ConnectionLoop);
        await Task.Yield();
    }

    private void Update()
    {
        while (mainThreadActions.TryDequeue(out var action))
            action();
    }

    // ── connection ────────────────────────────────────────────────────────────

    private async Task ConnectionLoop()
    {
        float delay = reconnectInitialDelaySec;
        int attempt = 0;

        while (!stopping)
        {
            try
            {
                if (ws != null && ws.State == WebSocketState.Open)
                {
                    await Task.Delay(200);
                    continue;
                }

                attempt++;
                await ConnectOnceAsync();
                delay = reconnectInitialDelaySec;
                attempt = 0;

                int gen = _sessionGen;
                _ = Task.Run(() => ReceiveLoop(gen));
                if (enableHeartbeat)
                    _ = Task.Run(HeartbeatLoop);
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"[WS] Connect error (#{attempt}): {ex.Message}");
                if (!autoReconnect) break;
            }

                Debug.Log($"[WS] Connected");
            if (stopping) break;
            await Task.Delay(TimeSpan.FromSeconds(delay));
            delay = Mathf.Min(delay * 2f, reconnectMaxDelaySec);
        }
    }

    private async Task ConnectOnceAsync()
    {
        CleanupSocket();
        Interlocked.Increment(ref _sessionGen);
        cts = new CancellationTokenSource();
        ws = new ClientWebSocket();
        ws.Options.Proxy = null;
        ws.Options.UseDefaultCredentials = false;
        ws.Options.KeepAliveInterval = TimeSpan.FromSeconds(5);
  Debug.Log($"[WS] ConnectOnceAsync ");
        using var timeoutCts = new CancellationTokenSource(TimeSpan.FromSeconds(connectTimeoutSec));
        using var linked = CancellationTokenSource.CreateLinkedTokenSource(cts.Token, timeoutCts.Token);

        Debug.Log($"[WS] Connecting to {serverUrl}");
        try
        {
            await ws.ConnectAsync(new Uri(serverUrl), linked.Token);
            Debug.Log("[WS] Connected");
        }
        catch (OperationCanceledException)
        {
            Debug.LogError($"[WS] Connection timeout after {connectTimeoutSec}s");
            throw;
        }
        catch (HttpRequestException ex)
        {
            Debug.LogError($"[WS] HTTP error: {ex.Message}");
            throw;
        }
    }

    // ── receive ───────────────────────────────────────────────────────────────

    private async Task ReceiveLoop(int gen)
    {
        var buffer = new byte[4096];
        var sb = new StringBuilder();

        try
        {
            while (!stopping && ws?.State == WebSocketState.Open && cts != null && !cts.IsCancellationRequested)
            {
                sb.Length = 0;
                WebSocketReceiveResult result;
                do
                {
                    result = await ws.ReceiveAsync(new ArraySegment<byte>(buffer), cts.Token);
                    if (result.MessageType == WebSocketMessageType.Close)
                    {
                        Debug.LogWarning($"[WS] Server closed: {result.CloseStatusDescription}");
                        await SafeCloseAsync("server closed");
                        return;
                    }
                    if (result.MessageType == WebSocketMessageType.Text)
                        sb.Append(Encoding.UTF8.GetString(buffer, 0, result.Count));
                }
                while (!result.EndOfMessage);

                if (sb.Length == 0) continue;

                string json = sb.ToString();
                HandleMessage(json);
            }
        }
        catch (OperationCanceledException) { }
        catch (Exception ex)
        {
            Debug.LogError($"[WS] ReceiveLoop: {ex.GetType().Name} - {ex.Message}");
        }
        finally
        {
            if (_sessionGen == gen) CleanupSocket();
        }
    }

    private static bool ContainsNaN(string json, string field) =>
        json.Contains($"\"{field}\":\"NaN\"") || json.Contains($"\"{field}\": \"NaN\"") ||
        json.Contains($"\"{field}\":NaN")     || json.Contains($"\"{field}\": NaN");

    private void HandleMessage(string json)
    {
        // Detect NaN fields before any replacement (server may send "NaN" string or bare NaN).
        // A NaN on either fpa*/error* for a foot means "no data" for that foot this packet.
        bool fpaLNaN = ContainsNaN(json, "fpaL") || ContainsNaN(json, "errorL") || ContainsNaN(json, "angleL");
        bool fpaRNaN = ContainsNaN(json, "fpaR") || ContainsNaN(json, "errorR") || ContainsNaN(json, "angleR");

        // JSON null/NaN → 0: JsonUtility cannot deserialize these into float fields.
        string jsonAfterReplace = json.Replace(": null", ": 0").Replace(":null", ":0")
                                      .Replace(": \"NaN\"", ": 0").Replace(":\"NaN\"", ":0")
                                      .Replace(": NaN", ": 0").Replace(":NaN", ":0");
        if (jsonAfterReplace != json)
            Debug.Log($"[WS] HandleMessage: null/NaN replaced  before={json}  after={jsonAfterReplace}");
        json = jsonAfterReplace;

        ServerMessage msg;
        try
        {
            msg = JsonUtility.FromJson<ServerMessage>(json);
        }
        catch (Exception ex)
        {
            Debug.LogError($"[WS] Parse failed: {ex.Message} | {json}");
            return;
        }

        if (msg == null || string.IsNullOrEmpty(msg.type))
        {
            Debug.LogWarning($"[WS] HandleMessage: msg.type is null/empty after parse | json={json}");
            return;
        }

        if (msg.type == "live")
        {
            Debug.Log($"[WS] Recv: {json}");
            Debug.Log($"[WS] HandleMessage: type={msg.type} ts={msg.ts}" +
                      (msg.type == "state" ? $" state={msg.state} condition={msg.condition} block={msg.block}" : "") +
                      (msg.type == "fpa"    ? $" block={msg.block} fpaL={msg.fpaL} fpaR={msg.fpaR} errorL={msg.errorL} errorR={msg.errorR} onTargetL={msg.onTargetL} onTargetR={msg.onTargetR}" : "") +
                      (msg.type == "live"   ? $" packetId={msg.packetId} angleL={msg.angleL} angleR={msg.angleR} confidence={msg.confidence} stability={msg.stability}" : "") +
                      (msg.type == "stepProgress" ? $" stage={msg.stage} stepsL={msg.stepsL}/{msg.requiredL} stepsR={msg.stepsR}/{msg.requiredR}" : "") +
                      (msg.type == "redo" ? $" stage={msg.stage} attempt={msg.attempt} reason={msg.reason}" : ""));
        }

        switch (msg.type)
        {
            case "state":
                if (!string.IsNullOrEmpty(msg.state))
                {
                    var captured = msg;
                    mainThreadActions.Enqueue(() => hudController?.OnStateChange(captured));
                }
                else
                {
                    Debug.LogWarning($"[WS] HandleMessage: state message has empty state field | json={json}");
                }
                break;

            case "fpa":
                msg.fpaLIsNaN = fpaLNaN;
                msg.fpaRIsNaN = fpaRNaN;
                var capturedFpa = msg;
                mainThreadActions.Enqueue(() => hudController?.OnFpaUpdate(capturedFpa));
                break;

            case "live":
                msg.fpaLIsNaN = fpaLNaN;
                msg.fpaRIsNaN = fpaRNaN;
                var capturedLive = msg;
                mainThreadActions.Enqueue(() => hudController?.OnLiveUpdate(capturedLive));
                break;

            case "stepProgress":
                var capturedProgress = msg;
                mainThreadActions.Enqueue(() => hudController?.OnStepProgress(capturedProgress));
                break;

            case "redo":
                var capturedRedo = msg;
                mainThreadActions.Enqueue(() => hudController?.OnRedo(capturedRedo));
                break;

            case "pong":
                break;
        }
    }

    // ── heartbeat ─────────────────────────────────────────────────────────────

    private async Task HeartbeatLoop()
    {
        try
        {
            while (!stopping && ws?.State == WebSocketState.Open && cts != null && !cts.IsCancellationRequested)
            {
                await Task.Delay(TimeSpan.FromSeconds(heartbeatIntervalSec), cts.Token);
                await SendTextAsync("{\"cmd\":\"ping\"}");
            }
        }
        catch (OperationCanceledException) { }
        catch (Exception ex)
        {
            Debug.LogWarning($"[WS] Heartbeat: {ex.Message}");
        }
    }

    // ── public send ───────────────────────────────────────────────────────────

    public async Task SendTextAsync(string text)
    {
        try
        {
            if (ws == null || ws.State != WebSocketState.Open)
            {
                Debug.LogWarning($"[WS] SendTextAsync skipped — socket not open (state={ws?.State.ToString() ?? "null"}): {text}");
                return;
            }
            Debug.Log($"[WS] Send: {text}");
            var bytes = Encoding.UTF8.GetBytes(text);
            await ws.SendAsync(new ArraySegment<byte>(bytes), WebSocketMessageType.Text, true, cts.Token);
        }
        catch (Exception ex)
        {
            Debug.LogWarning($"[WS] Send error: {ex.Message}");
        }
    }

    // AR→PC commands (spec 2026-07-29-ws-protocol.md)
    public Task SendReadyForCalibrationAsync() => SendTextAsync("{\"cmd\":\"ReadyForCalibration\"}");

    public Task SendContinueTrainingAsync(int fromBlock) =>
        SendTextAsync($"{{\"cmd\":\"continueTraining\",\"fromBlock\":{fromBlock}}}");

    // ── cleanup ───────────────────────────────────────────────────────────────

    private async Task SafeCloseAsync(string reason)
    {
        try
        {
            if (ws != null && (ws.State == WebSocketState.Open || ws.State == WebSocketState.CloseReceived))
                await ws.CloseAsync(WebSocketCloseStatus.NormalClosure, reason, CancellationToken.None);
        }
        catch { }
        finally { CleanupSocket(); }
    }

    private void CleanupSocket()
    {
        try { cts?.Cancel(); } catch { }
        try { cts?.Dispose(); } catch { }
        cts = null;
        try { ws?.Dispose(); } catch { }
        ws = null;

        mainThreadActions.Enqueue(() => hudController?.OnStateChange("disconnected"));
    }

    private async void OnDestroy()
    {
        stopping = true;
        try { await SafeCloseAsync("destroy"); } catch { }
    }

    private async void OnApplicationQuit()
    {
        stopping = true;
        try { await SafeCloseAsync("quit"); } catch { }
    }
}

// Mirrors the server JSON protocol exactly (docs/superpowers/specs/2026-07-29-ws-protocol.md).
[Serializable]
public class ServerMessage
{
    public string type;      // "pong" | "state" | "fpa" | "stepProgress" | "redo" | "live"
    public long   ts;        // server broadcast time, unix ms (all PC→AR messages)

    // state — armed/waiting/calibrating/baseline/training/rest/retention/paused/ended/error
    public string state;
    public string condition;      // "" | "EF" | "IF"
    public int    block;          // 0 | 1/2/3 (state, fpa)
    public int    restDurationSec;// state: rest, seconds
    public string reason;         // state: error; redo
    public float  targetL;        // state: EF stone target angle, left (deg)
    public string directionL;     // state: "toe-in" | "toe-out"
    public float  targetR;        // state: EF stone target angle, right (deg)
    public string directionR;     // state: "toe-in" | "toe-out"

    // fpa — per-step feedback, training only
    public string stage;      // fpa/stepProgress/redo: baseline | training | retention | Training2 ...
    public long   packetId;   // fpa
    public float  fpaL;       // fpa: left foot angle (degrees)
    public float  fpaR;       // fpa: right foot angle (degrees)
    public bool   fpaLIsNaN;  // set by client when server sends NaN for fpaL/errorL
    public bool   fpaRIsNaN;  // set by client when server sends NaN for fpaR/errorR
    public bool   onTargetL;  // fpa: drives IF footprint color
    public bool   onTargetR;  // fpa: drives IF footprint color
    public float  errorL;     // fpa: fpaL - targetL, drives IF footprint rotation
    public float  errorR;     // fpa: fpaR - targetR, drives IF footprint rotation

    // live — real-time footprint angle stream (no on-target color signal)
    public float  angleL;      // live: left foot angle (degrees), drives footprint rotation
    public float  angleR;      // live: right foot angle (degrees), drives footprint rotation
    public float  confidence;  // live
    public float  stability;   // live

    // stepProgress — baseline/retention step-count bar
    public int stepsL;
    public int stepsR;
    public int requiredL;
    public int requiredR;

    // redo
    public int attempt;
}
