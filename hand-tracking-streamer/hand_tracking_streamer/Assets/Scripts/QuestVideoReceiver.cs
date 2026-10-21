using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using Unity.WebRTC;
using UnityEngine;

public class QuestVideoReceiver : MonoBehaviour
{
    public event Action<string> OnLocalOfferReady;
    public event Action<string, string, int?> OnLocalIceCandidate;
    public event Action<Texture> OnRemoteTexture;
    public event Action OnFrameDecoded;
    public event Action<string> OnPeerStateChanged;
    public event Action<string> OnError;

    private RTCPeerConnection _peer;
    private VideoStreamTrack _remoteTrack;
    private Coroutine _updateCoroutine;
    private bool _updateRunning;
    private Coroutine _statsCoroutine;
    private readonly List<TaskCompletionSource<bool>> _pending = new List<TaskCompletionSource<bool>>();

    public void InitializePeer()
    {
        if (!_updateRunning)
        {
            _updateCoroutine = StartCoroutine(WebRTC.Update());
            _updateRunning = true;
        }

        ClosePeer();
        _peer = new RTCPeerConnection();
        _peer.AddTransceiver(TrackKind.Video, new RTCRtpTransceiverInit
        { direction = RTCRtpTransceiverDirection.RecvOnly });
        var peer = _peer;
        _statsCoroutine = StartCoroutine(WatchDecodedFrames(peer));
        _peer.OnConnectionStateChange = state => { if (_peer == peer) OnPeerStateChanged?.Invoke(state.ToString()); };
        _peer.OnIceConnectionChange = state => { if (_peer == peer) OnPeerStateChanged?.Invoke($"ICE {state}"); };
        _peer.OnIceCandidate = candidate =>
        {
            if (_peer != peer || candidate == null) return;
            OnLocalIceCandidate?.Invoke(candidate.Candidate, candidate.SdpMid, candidate.SdpMLineIndex);
        };
        _peer.OnTrack = e =>
        {
            if (_peer == peer && e.Track is VideoStreamTrack vt)
            {
                _remoteTrack = vt;
                _remoteTrack.OnVideoReceived += texture => { if (_peer == peer) OnRemoteTexture?.Invoke(texture); };
            }
        };
    }

    public Task<bool> CreateAndSendOfferAsync()
    {
        var tcs = new TaskCompletionSource<bool>();
        _pending.Add(tcs);
        StartCoroutine(CreateOfferRoutine(tcs));
        return tcs.Task;
    }

    public Task<bool> SetRemoteAnswerAsync(string sdp)
    {
        var tcs = new TaskCompletionSource<bool>();
        _pending.Add(tcs);
        StartCoroutine(SetRemoteAnswerRoutine(sdp, tcs));
        return tcs.Task;
    }

    public void AddRemoteIceCandidate(string candidate, string sdpMid, int? sdpMLineIndex)
    {
        if (_peer == null || string.IsNullOrWhiteSpace(candidate)) return;
        var init = new RTCIceCandidateInit
        {
            candidate = candidate,
            sdpMid = sdpMid,
            sdpMLineIndex = sdpMLineIndex ?? 0,
        };
        _peer.AddIceCandidate(new RTCIceCandidate(init));
    }

    public void ClosePeer()
    {
        if (_statsCoroutine != null) { StopCoroutine(_statsCoroutine); _statsCoroutine = null; }
        var pending = _pending.ToArray();
        _pending.Clear();
        foreach (var completion in pending) completion.TrySetResult(false);
        if (_remoteTrack != null)
        {
            _remoteTrack.Dispose();
            _remoteTrack = null;
        }
        if (_peer != null)
        {
            _peer.Close();
            _peer.Dispose();
            _peer = null;
        }
    }

    private IEnumerator CreateOfferRoutine(TaskCompletionSource<bool> tcs)
    {
        if (_peer == null)
        {
            tcs.TrySetResult(false);
            yield break;
        }

        var peer = _peer;
        var op = peer.CreateOffer();
        yield return op;
        if (peer != _peer) { tcs.TrySetResult(false); yield break; }
        if (op.IsError)
        {
            OnError?.Invoke($"CreateOffer failed: {op.Error.message}");
            tcs.TrySetResult(false);
            yield break;
        }

        var desc = op.Desc;
        var setLocal = peer.SetLocalDescription(ref desc);
        yield return setLocal;
        if (peer != _peer) { tcs.TrySetResult(false); yield break; }
        if (setLocal.IsError)
        {
            OnError?.Invoke($"SetLocalDescription failed: {setLocal.Error.message}");
            tcs.TrySetResult(false);
            yield break;
        }

        OnLocalOfferReady?.Invoke(desc.sdp);
        tcs.TrySetResult(true);
    }

    private IEnumerator SetRemoteAnswerRoutine(string sdp, TaskCompletionSource<bool> tcs)
    {
        if (_peer == null)
        {
            tcs.TrySetResult(false);
            yield break;
        }

        var desc = new RTCSessionDescription
        {
            type = RTCSdpType.Answer,
            sdp = sdp,
        };
        var peer = _peer;
        var op = peer.SetRemoteDescription(ref desc);
        yield return op;
        if (peer != _peer) { tcs.TrySetResult(false); yield break; }
        if (op.IsError)
        {
            OnError?.Invoke($"SetRemoteDescription failed: {op.Error.message}");
            tcs.TrySetResult(false);
            yield break;
        }

        tcs.TrySetResult(true);
    }

    private void OnDestroy()
    {
        ClosePeer();
        if (_updateRunning)
        {
            if (_updateCoroutine != null)
            {
                StopCoroutine(_updateCoroutine);
                _updateCoroutine = null;
            }
            _updateRunning = false;
        }
    }

    private IEnumerator WatchDecodedFrames(RTCPeerConnection peer)
    {
        ulong previous = 0;
        while (_peer == peer)
        {
            yield return new WaitForSecondsRealtime(0.5f);
            if (_peer != peer) yield break;
            var operation = peer.GetStats();
            yield return operation;
            if (_peer != peer) yield break;
            if (operation.IsError) continue;
            using var report = operation.Value;
            ulong decoded = 0;
            foreach (var stat in report.Stats.Values)
                if (stat is RTCInboundRTPStreamStats inbound && inbound.kind == "video")
                    decoded += inbound.framesDecoded;
            if (decoded > previous) OnFrameDecoded?.Invoke();
            previous = decoded;
        }
    }
}
