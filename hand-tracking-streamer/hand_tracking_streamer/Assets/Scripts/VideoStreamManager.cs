using System;
using System.Collections.Concurrent;
using System.Threading.Tasks;
using Newtonsoft.Json.Linq;
using UnityEngine;

public class VideoStreamManager : MonoBehaviour
{
    public enum SessionState { Idle, SignalingConnecting, OfferSent, Connected, Playing, Stopping }
    [SerializeField] private QuestVideoReceiver videoReceiver;
    [SerializeField] private VideoPanelRenderer panelRenderer;
    [SerializeField] private VideoStatsOverlay statsOverlay;
    [SerializeField] private string logSource = "Left";
    [Header("Automatic robot vision")]
    [SerializeField] private bool automaticRobotVision = true;
    [Tooltip("Empty: follow the existing tracking host. For Quest using USB/broadcast tracking, enter the computer LAN IP here.")]
    [SerializeField] private string videoHost = "";
    [SerializeField] private int videoPort = 8765;
    [SerializeField, Min(1)] private float retrySeconds = 3;
    [SerializeField, Min(5)] private float firstFrameTimeoutSeconds = 15;
    [SerializeField, Min(1)] private float stalledFrameTimeoutSeconds = 3;

    private VideoSignalingClient client;
    private SessionState state;
    private string sessionId = "";
    private string connectedHost = "";
    private int generation;
    private bool destroyed;
    private double retryAt;
    private Texture remoteTexture;
    private readonly RobotVideoTiming timing = new RobotVideoTiming();
    private readonly ConcurrentQueue<Action> mainThreadActions = new ConcurrentQueue<Action>();
    public SessionState CurrentState => state;

    private void Awake() => SetVideoUiVisible(false);

    // Compatibility with AppManager: never reveal video before actual decoded frames.
    public void SetVideoUiVisible(bool visible)
    {
        panelRenderer?.SetRobotVisionActive(visible && state == SessionState.Playing);
        if (!visible) statsOverlay?.SetVisible(false);
    }

    private string ResolveHost(AppManager app)
    {
        string host = string.IsNullOrWhiteSpace(videoHost) ? app?.ServerIP : videoHost;
        if (string.IsNullOrWhiteSpace(host) || host == "255.255.255.255")
            return Application.isEditor ? "127.0.0.1" : null;
        // Editor's loopback is this computer; Quest's loopback is the headset.
        return host.Trim();
    }

    private void Update()
    {
        while (mainThreadActions.TryDequeue(out var action))
        {
            try { action(); } catch (Exception ex) { FailVideo(ex.Message); }
        }
        var app = AppManager.Instance;
        bool wanted = !destroyed && app != null && app.isStreaming &&
                      (automaticRobotVision || app.ShowVideoStream);
        string host = ResolveHost(app);
        if (!wanted || string.IsNullOrWhiteSpace(host))
        {
            if (state != SessionState.Idle) _ = StopVideoSession("tracking_stopped");
            return;
        }
        if (state != SessionState.Idle && host != connectedHost)
        {
            _ = StopVideoSession("host_changed");
            return;
        }
        double now = Time.realtimeSinceStartupAsDouble;
        if (state != SessionState.Idle && timing.TimedOut(now, firstFrameTimeoutSeconds, stalledFrameTimeoutSeconds))
        {
            FailVideo(timing.HasFrame ? "Robot video stalled; original view restored." : "Waiting for robot video service.");
            return;
        }
        if (state == SessionState.Idle && now >= retryAt)
            _ = StartVideoSession(host, videoPort, "720p30", false);
    }

    private void QueueForSession(int version, Action action)
    {
        mainThreadActions.Enqueue(() => { if (!destroyed && version == generation) action(); });
    }

    public async Task<bool> StartVideoSession(string host, int port, string preset, bool showDebugStats)
    {
        if (destroyed || !isActiveAndEnabled || state != SessionState.Idle) return false;
        if (videoReceiver == null || panelRenderer == null)
        {
            retryAt = Time.realtimeSinceStartupAsDouble + retrySeconds;
            return false;
        }
        int version = ++generation;
        connectedHost = host;
        sessionId = Guid.NewGuid().ToString("N");
        state = SessionState.SignalingConnecting;
        timing.Begin(Time.realtimeSinceStartupAsDouble);
        remoteTexture = null;
        SetVideoUiVisible(false);
        statsOverlay?.SetPreset(preset);
        var connection = new VideoSignalingClient();
        client = connection;
        connection.OnEnvelope += e => QueueForSession(version, () => HandleEnvelope(e));
        connection.OnError += e => QueueForSession(version, () => FailVideo(e));
        connection.OnDisconnected += () => QueueForSession(version, () => FailVideo("Robot video disconnected."));
        try
        {
            bool connected = await connection.ConnectAsync(host, port, 2500);
            if (destroyed || version != generation) return false;
            if (!connected) { FailVideo("Robot video service unavailable."); return false; }
            state = SessionState.Connected;
            videoReceiver.InitializePeer();
            videoReceiver.OnLocalOfferReady += OnLocalOfferReady;
            videoReceiver.OnLocalIceCandidate += OnLocalIceCandidate;
            videoReceiver.OnRemoteTexture += OnRemoteTexture;
            videoReceiver.OnFrameDecoded += OnFrameDecoded;
            videoReceiver.OnPeerStateChanged += OnPeerStateChanged;
            videoReceiver.OnError += FailVideo;
            await connection.SendAsync("hello", sessionId,
                new JObject { ["app_version"] = Application.version, ["video_preset"] = preset, ["client_name"] = "quest" }.ToString());
            if (version != generation) return false;
            await connection.SendAsync("start_video", sessionId, "{}");
            if (version != generation) return false;
            state = SessionState.OfferSent;
            bool ok = await videoReceiver.CreateAndSendOfferAsync();
            if (version != generation) return false;
            if (!ok) FailVideo("Could not create video offer.");
            return ok;
        }
        catch (Exception ex)
        {
            if (version == generation && !destroyed) FailVideo(ex.Message);
            return false;
        }
    }

    public Task StopVideoSession(string reason)
    {
        // Invalidate queued callbacks BEFORE closing the old socket/peer.
        ++generation;
        state = SessionState.Stopping;
        SetVideoUiVisible(false); // Restore passthrough immediately, even if the network is dead.
        if (videoReceiver != null)
        {
            videoReceiver.OnLocalOfferReady -= OnLocalOfferReady;
            videoReceiver.OnLocalIceCandidate -= OnLocalIceCandidate;
            videoReceiver.OnRemoteTexture -= OnRemoteTexture;
            videoReceiver.OnFrameDecoded -= OnFrameDecoded;
            videoReceiver.OnPeerStateChanged -= OnPeerStateChanged;
            videoReceiver.OnError -= FailVideo;
            videoReceiver.ClosePeer();
        }
        client?.Dispose();
        client = null;
        remoteTexture = null;
        panelRenderer?.Clear();
        sessionId = "";
        state = SessionState.Idle;
        retryAt = Time.realtimeSinceStartupAsDouble + retrySeconds;
        return Task.CompletedTask;
    }

    private async Task Send(string type, JObject payload)
    {
        var connection = client;
        int version = generation;
        if (connection == null) return;
        try { await connection.SendAsync(type, sessionId, payload.ToString()); }
        catch (Exception ex) { if (version == generation && !destroyed) FailVideo(ex.Message); }
    }

    private async void OnLocalOfferReady(string sdp)
    {
        if (string.IsNullOrWhiteSpace(sdp) || !sdp.Contains("m=video"))
        { FailVideo("Video offer has no video track."); return; }
        await Send("offer", new JObject { ["sdp"] = sdp });
    }

    private async void OnLocalIceCandidate(string candidate, string mid, int? index)
    {
        await Send("ice_candidate", new JObject {
            ["candidate"] = candidate, ["sdpMid"] = mid,
            ["sdpMLineIndex"] = index.HasValue ? new JValue(index.Value) : JValue.CreateNull() });
    }

    private void OnRemoteTexture(Texture texture)
    {
        remoteTexture = texture;
        panelRenderer?.SetTexture(texture);
        if (timing.HasFrame) ShowRobotVision();
    }

    private void OnFrameDecoded()
    {
        timing.FrameDecoded(Time.realtimeSinceStartupAsDouble);
        if (remoteTexture != null) ShowRobotVision();
    }

    private void ShowRobotVision()
    {
        if (state == SessionState.Playing) return;
        state = SessionState.Playing;
        panelRenderer?.SetStatus("");
        SetVideoUiVisible(true);
        Log("Robot vision active.");
    }

    private void OnPeerStateChanged(string value)
    {
        statsOverlay?.SetPeerState(value);
        if (value == "Failed" || value == "Closed" || value == "Disconnected" ||
            value == "ICE Failed" || value == "ICE Disconnected") FailVideo(value);
    }

    private async void HandleEnvelope(VideoSignalingClient.Envelope envelope)
    {
        if (envelope.SessionId != sessionId) return;
        int version = generation;
        try
        {
            var p = JObject.Parse(envelope.PayloadJson);
            switch (envelope.Type)
            {
                case "answer":
                    bool ok = await videoReceiver.SetRemoteAnswerAsync(p.Value<string>("sdp"));
                    if (!ok && version == generation) FailVideo("Invalid video answer.");
                    break;
                case "ice_candidate":
                    videoReceiver.AddRemoteIceCandidate(p.Value<string>("candidate"), p.Value<string>("sdpMid"), p.Value<int?>("sdpMLineIndex"));
                    break;
                case "error": FailVideo(p.Value<string>("message") ?? "Robot video error."); break;
                case "video_state":
                    if (p.Value<string>("state") == "error" || p.Value<string>("state") == "stopped")
                        FailVideo(p.Value<string>("reason") ?? "Robot video stopped.");
                    break;
                case "stats":
                    statsOverlay?.SetStats(p.Value<float?>("fps") ?? 0, p.Value<float?>("bitrate_kbps") ?? 0,
                        p.Value<int?>("frame_drops") ?? 0, p.Value<float?>("rtt_ms") ?? -1);
                    break;
            }
        }
        catch (Exception ex) { if (version == generation && !destroyed) FailVideo(ex.Message); }
    }

    private void FailVideo(string reason)
    {
        if (destroyed || state == SessionState.Idle || state == SessionState.Stopping) return;
        bool wasPlaying = state == SessionState.Playing;
        _ = StopVideoSession(reason);
        // An unavailable optional server is normal; do not flood the existing HUD.
        if (wasPlaying) Log(reason + " Original view restored; tracking continues.");
    }

    private void Log(string message)
    {
        if (LogManager.Instance != null) LogManager.Instance.Log(logSource, "[Video] " + message);
    }

    private void OnDisable() { _ = StopVideoSession("disabled"); }
    private void OnDestroy() { destroyed = true; _ = StopVideoSession("destroyed"); }
}
