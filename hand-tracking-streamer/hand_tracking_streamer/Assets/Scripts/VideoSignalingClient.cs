using System;
using System.Net.WebSockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Newtonsoft.Json.Linq;
using UnityEngine;

public sealed class VideoSignalingClient : IDisposable
{
    public sealed class Envelope
    {
        public string Type;
        public string SessionId;
        public string PayloadJson;
    }

    public event Action<Envelope> OnEnvelope;
    public event Action<string> OnError;
    public event Action OnConnected;
    public event Action OnDisconnected;

    private ClientWebSocket _socket;
    private CancellationTokenSource _cts;
    private Task _receiveLoopTask;
    private readonly SemaphoreSlim _sendLock = new SemaphoreSlim(1, 1);
    private bool _disposed;

    public bool IsConnected => _socket != null && _socket.State == WebSocketState.Open;

    public async Task<bool> ConnectAsync(string host, int port, int timeoutMs = 4000)
    {
        try
        {
            if (_disposed) return false;
            _cts = new CancellationTokenSource();
            _socket = new ClientWebSocket();
            using var connectCts = CancellationTokenSource.CreateLinkedTokenSource(_cts.Token);
            connectCts.CancelAfter(timeoutMs);
            await _socket.ConnectAsync(new Uri($"ws://{host}:{port}"), connectCts.Token);
            _receiveLoopTask = ReceiveLoopAsync(_cts.Token);
            OnConnected?.Invoke();
            return true;
        }
        catch (OperationCanceledException)
        {
            OnError?.Invoke($"Signaling connect timed out: ws://{host}:{port}");
            return false;
        }
        catch (Exception ex)
        {
            OnError?.Invoke($"Signaling connect failed: ws://{host}:{port} {ex.Message}");
            return false;
        }
    }

    public async Task SendAsync(string type, string sessionId, string payloadJson)
    {
        await _sendLock.WaitAsync();
        try
        {
            if (!IsConnected || _disposed) return;
            byte[] bytes = Encoding.UTF8.GetBytes(BuildEnvelopeJson(type, sessionId, payloadJson));
            using var timeout = CancellationTokenSource.CreateLinkedTokenSource(_cts.Token);
            timeout.CancelAfter(2500);
            await _socket.SendAsync(new ArraySegment<byte>(bytes), WebSocketMessageType.Text, true, timeout.Token);
        }
        finally { _sendLock.Release(); }
    }

    public Task DisconnectAsync()
    {
        Dispose();
        return Task.CompletedTask;
    }

    public void Dispose()
    {
        if (_disposed) return;
        _disposed = true;
        // Abort instead of awaiting a close handshake from an offline server.
        _cts?.Cancel();
        _socket?.Abort();
        _socket?.Dispose();
        _cts?.Dispose();
        OnDisconnected?.Invoke();
    }

    private async Task ReceiveLoopAsync(CancellationToken token)
    {
        byte[] buffer = new byte[16 * 1024];
        using var message = new System.IO.MemoryStream();

        while (!token.IsCancellationRequested && _socket != null)
        {
            WebSocketReceiveResult result;
            try
            {
                result = await _socket.ReceiveAsync(new ArraySegment<byte>(buffer), token);
            }
            catch (OperationCanceledException)
            {
                break;
            }
            catch (Exception ex)
            {
                OnError?.Invoke($"Signaling receive failed: {ex.Message}");
                break;
            }

            if (result.MessageType == WebSocketMessageType.Close)
            {
                break;
            }

            if (result.MessageType != WebSocketMessageType.Text) continue;
            message.Write(buffer, 0, result.Count);
            if (message.Length > 1024 * 1024)
            {
                OnError?.Invoke("Signaling message exceeds 1 MB.");
                break;
            }
            if (!result.EndOfMessage) continue;

            string raw = Encoding.UTF8.GetString(message.ToArray());
            message.SetLength(0);
            TryDispatchEnvelope(raw);
        }
        if (!_disposed) OnDisconnected?.Invoke();
    }

    private void TryDispatchEnvelope(string raw)
    {
        try
        {
            JObject root = JObject.Parse(raw);
            string type = root.Value<string>("type") ?? string.Empty;
            string sessionId = root.Value<string>("session_id") ?? string.Empty;
            string payload = root["payload"] != null ? root["payload"].ToString() : "{}";
            OnEnvelope?.Invoke(new Envelope
            {
                Type = type,
                SessionId = sessionId,
                PayloadJson = payload,
            });
        }
        catch (Exception ex)
        {
            OnError?.Invoke($"Invalid signaling payload: {ex.Message}");
        }
    }

    private static string BuildEnvelopeJson(string type, string sessionId, string payloadJson)
    {
        var payload = ParsePayloadOrEmpty(payloadJson);
        var envelope = new JObject
        {
            ["type"] = type ?? string.Empty,
            ["session_id"] = sessionId ?? string.Empty,
            ["payload"] = payload,
        };
        return envelope.ToString(Newtonsoft.Json.Formatting.None);
    }

    private static JToken ParsePayloadOrEmpty(string payloadJson)
    {
        if (string.IsNullOrWhiteSpace(payloadJson))
        {
            return new JObject();
        }
        try
        {
            return JToken.Parse(payloadJson);
        }
        catch
        {
            return new JObject();
        }
    }
}
