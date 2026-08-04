using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.WebSockets;
using System.Text;
using System.Text.Json;
using System.Threading.Tasks;

namespace IMUMoCap.Methods
{
    // ======================= WebSocket Server (Broadcast) =======================


    // 一个非常轻量的 WS 广播服务：WPF/Console 都能用
    public sealed class WebSocketBroadcastServer
    {
        private readonly HttpListener _listener = new HttpListener();
        private readonly CancellationTokenSource _cts = new CancellationTokenSource();
        private readonly object _gate = new object();
        private readonly Dictionary<Guid, WebSocket> _clients = new Dictionary<Guid, WebSocket>();
        public event Action<Guid, string>? OnTextMessage;
        public event Action<Guid, string?>? OnClientConnected;
        public event Action<Guid, string?>? OnClientDisconnected;
        public bool IsRunning { get; private set; }
        public int ClientCount
        {
            get { lock (_gate) return _clients.Count; }
        }

        // 例如：prefix="http://+:8765/ws/"  (注意：HttpListener 这里是 http 前缀，但客户端连的是 ws://)
        public void Start(string[] prefix)
        {
            if (IsRunning) return;
            try
            {


                _listener.Prefixes.Clear();
                foreach (var item in prefix)
                {
                    _listener.Prefixes.Add(item);
                }
                _listener.Start();
                IsRunning = true;
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.Message);
            }
            _ = Task.Run(AcceptLoopAsync);
        }

        public async Task StopAsync()
        {
            if (!IsRunning) return;
            IsRunning = false;

            try { _cts.Cancel(); } catch { }
            try { _listener.Stop(); } catch { }
            try { _listener.Close(); } catch { }

            List<WebSocket> sockets;
            lock (_gate) sockets = _clients.Values.ToList();
            lock (_gate) _clients.Clear();

            foreach (var ws in sockets)
            {
                try
                {
                    if (ws.State == WebSocketState.Open || ws.State == WebSocketState.CloseReceived)
                        await ws.CloseAsync(WebSocketCloseStatus.NormalClosure, "Server stopping", CancellationToken.None);
                }
                catch { /* ignore */ }
                try { ws.Dispose(); } catch { /* ignore */ }
            }
        }

        private static readonly JsonSerializerOptions _jsonOpts = new JsonSerializerOptions
        {
            NumberHandling = System.Text.Json.Serialization.JsonNumberHandling.AllowNamedFloatingPointLiterals,
        };

        public async Task BroadcastJsonAsync(object payload)
        {
            if (!IsRunning) return;

            string json = JsonSerializer.Serialize(payload, _jsonOpts);
            var bytes = Encoding.UTF8.GetBytes(json);
            var seg = new ArraySegment<byte>(bytes);

            List<(Guid id, WebSocket ws)> targets;
            lock (_gate) targets = _clients.Select(kv => (kv.Key, kv.Value)).ToList();

            foreach (var (id, ws) in targets)
            {
                if (ws.State != WebSocketState.Open)
                {
                    RemoveClient(id);
                    continue;
                }

                try
                {
                    await ws.SendAsync(seg, WebSocketMessageType.Text, endOfMessage: true, cancellationToken: _cts.Token);
                }
                catch
                {
                    RemoveClient(id);
                }
            }
        }
        // Sends one JSON message to a single client (used for the on-connect state snapshot).
        public async Task SendJsonAsync(Guid id, object payload)
        {
            if (!IsRunning) return;

            WebSocket? ws;
            lock (_gate) { if (!_clients.TryGetValue(id, out ws)) return; }
            if (ws.State != WebSocketState.Open) { RemoveClient(id); return; }

            string json = JsonSerializer.Serialize(payload, _jsonOpts);
            var seg = new ArraySegment<byte>(Encoding.UTF8.GetBytes(json));
            try { await ws.SendAsync(seg, WebSocketMessageType.Text, endOfMessage: true, cancellationToken: _cts.Token); }
            catch { RemoveClient(id); }
        }

        private async Task ReceiveLoopAsync(Guid id, WebSocket ws, string? remote)
        {
            var buffer = new byte[4096];
            var ms = new MemoryStream();

            try
            {
                while (!_cts.IsCancellationRequested && ws.State == WebSocketState.Open)
                {
                    ms.SetLength(0);

                    WebSocketReceiveResult result;
                    do
                    {
                        result = await ws.ReceiveAsync(new ArraySegment<byte>(buffer), _cts.Token);

                        if (result.MessageType == WebSocketMessageType.Close)
                            break;

                        ms.Write(buffer, 0, result.Count);
                    }
                    while (!result.EndOfMessage);

                    if (result.MessageType == WebSocketMessageType.Close)
                        break;

                    if (result.MessageType == WebSocketMessageType.Text)
                    {
                        var text = System.Text.Encoding.UTF8.GetString(ms.ToArray());
                        if (!text.Contains("ping"))
                        { 
                        
                        }
                        OnTextMessage?.Invoke(id, text);
                    }
                    // 二进制消息也可以支持：result.MessageType == WebSocketMessageType.Binary
                }
            }
            catch
            {
                // ignore; treat as disconnect
            }
            finally
            {
                RemoveClient(id);
                OnClientDisconnected?.Invoke(id, remote);
                try
                {
                    if (ws.State == WebSocketState.Open || ws.State == WebSocketState.CloseReceived)
                        await ws.CloseAsync(WebSocketCloseStatus.NormalClosure, "bye", CancellationToken.None);
                }
                catch { }
                try { ws.Dispose(); } catch { }
            }
        }

        private void RemoveClient(Guid id)
        {
            WebSocket? ws = null;
            lock (_gate)
            {
                if (_clients.TryGetValue(id, out ws))
                    _clients.Remove(id);
            }
        }

        private async Task AcceptLoopAsync()
        {
            while (!_cts.IsCancellationRequested)
            {
                HttpListenerContext? ctx = null;
                try
                {
                    ctx = await _listener.GetContextAsync();
                }
                catch
                {
                    if (_cts.IsCancellationRequested) break;
                    continue;
                }

                // 只接 WebSocket
                if (!ctx.Request.IsWebSocketRequest)
                {
                    ctx.Response.StatusCode = 400;
                    ctx.Response.Close();
                    continue;
                }

                HttpListenerWebSocketContext? wsCtx = null;
                try
                {
                    wsCtx = await ctx.AcceptWebSocketAsync(subProtocol: null);
                }
                catch
                {
                    try { ctx.Response.StatusCode = 500; ctx.Response.Close(); } catch { }
                    continue;
                }

                var ws = wsCtx.WebSocket;
                var id = Guid.NewGuid();
                lock (_gate) _clients[id] = ws;

                // 记录客户端来源（可能为 null）
                string? remote = ctx.Request.RemoteEndPoint?.ToString();

                // 触发连接事件
                OnClientConnected?.Invoke(id, remote);

                // 可选：启动一个接收循环（用于感知断线/客户端发消息），不处理内容也行
                _ = Task.Run(() => ReceiveLoopAsync(id, ws, remote));
            }
        }

    }
}
