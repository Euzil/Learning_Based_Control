using UnityEngine;
using Oculus.Interaction.Body.Input;
using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using Stopwatch = System.Diagnostics.Stopwatch;

public class BodyLandmarkStreamer : MonoBehaviour
{
    [Header("Logging")]
    [SerializeField] private bool _logToHUD = true;

    [Tooltip("The Log Source name to push text to")]
    [SerializeField] private string _hudLogSource = "Right";

    [SerializeField] private float _frequency = 0.01f;

    [Header("Streaming")]
    [SerializeField] private bool _streamRotations = true;

    private IBody _body;
    private float _timer;
    private float _diagnosticTimer;
    private string _lastDiagnostic = "Waiting for body tracking data.";
    private bool _hasLoggedRuntimeException;
    private bool _hasReceivedBodyUpdate;
    private bool _isRefreshingBodyData;
    private JoystickLocomotionProvider _locomotionProvider;

    private UdpClient _udpClient;
    private TcpClient _tcpClient;
    private NetworkStream _tcpStream;
    private IPEndPoint _remoteEndPoint;
    private bool _isInitialized;
    private int _currentProtocol = -1;

    private readonly StringBuilder _sbPacket = new StringBuilder(8192);

    private uint _frameId;
    private static readonly double _ticksToNs = 1_000_000_000.0 / Stopwatch.Frequency;

    private static readonly BodyJointId[] _upperBodyJoints =
    {
        BodyJointId.Body_Root,
        BodyJointId.Body_Hips,
        BodyJointId.Body_SpineLower,
        BodyJointId.Body_SpineMiddle,
        BodyJointId.Body_SpineUpper,
        BodyJointId.Body_Chest,
        BodyJointId.Body_Neck,
        BodyJointId.Body_Head,
        BodyJointId.Body_LeftShoulder,
        BodyJointId.Body_LeftScapula,
        BodyJointId.Body_LeftArmUpper,
        BodyJointId.Body_LeftArmLower,
        BodyJointId.Body_LeftHandWristTwist,
        BodyJointId.Body_RightShoulder,
        BodyJointId.Body_RightScapula,
        BodyJointId.Body_RightArmUpper,
        BodyJointId.Body_RightArmLower,
        BodyJointId.Body_RightHandWristTwist,
        BodyJointId.Body_LeftHandPalm,
        BodyJointId.Body_LeftHandWrist,
        BodyJointId.Body_LeftHandThumbMetacarpal,
        BodyJointId.Body_LeftHandThumbProximal,
        BodyJointId.Body_LeftHandThumbDistal,
        BodyJointId.Body_LeftHandThumbTip,
        BodyJointId.Body_LeftHandIndexMetacarpal,
        BodyJointId.Body_LeftHandIndexProximal,
        BodyJointId.Body_LeftHandIndexIntermediate,
        BodyJointId.Body_LeftHandIndexDistal,
        BodyJointId.Body_LeftHandIndexTip,
        BodyJointId.Body_LeftHandMiddleMetacarpal,
        BodyJointId.Body_LeftHandMiddleProximal,
        BodyJointId.Body_LeftHandMiddleIntermediate,
        BodyJointId.Body_LeftHandMiddleDistal,
        BodyJointId.Body_LeftHandMiddleTip,
        BodyJointId.Body_LeftHandRingMetacarpal,
        BodyJointId.Body_LeftHandRingProximal,
        BodyJointId.Body_LeftHandRingIntermediate,
        BodyJointId.Body_LeftHandRingDistal,
        BodyJointId.Body_LeftHandRingTip,
        BodyJointId.Body_LeftHandLittleMetacarpal,
        BodyJointId.Body_LeftHandLittleProximal,
        BodyJointId.Body_LeftHandLittleIntermediate,
        BodyJointId.Body_LeftHandLittleDistal,
        BodyJointId.Body_LeftHandLittleTip,
        BodyJointId.Body_RightHandPalm,
        BodyJointId.Body_RightHandWrist,
        BodyJointId.Body_RightHandThumbMetacarpal,
        BodyJointId.Body_RightHandThumbProximal,
        BodyJointId.Body_RightHandThumbDistal,
        BodyJointId.Body_RightHandThumbTip,
        BodyJointId.Body_RightHandIndexMetacarpal,
        BodyJointId.Body_RightHandIndexProximal,
        BodyJointId.Body_RightHandIndexIntermediate,
        BodyJointId.Body_RightHandIndexDistal,
        BodyJointId.Body_RightHandIndexTip,
        BodyJointId.Body_RightHandMiddleMetacarpal,
        BodyJointId.Body_RightHandMiddleProximal,
        BodyJointId.Body_RightHandMiddleIntermediate,
        BodyJointId.Body_RightHandMiddleDistal,
        BodyJointId.Body_RightHandMiddleTip,
        BodyJointId.Body_RightHandRingMetacarpal,
        BodyJointId.Body_RightHandRingProximal,
        BodyJointId.Body_RightHandRingIntermediate,
        BodyJointId.Body_RightHandRingDistal,
        BodyJointId.Body_RightHandRingTip,
        BodyJointId.Body_RightHandLittleMetacarpal,
        BodyJointId.Body_RightHandLittleProximal,
        BodyJointId.Body_RightHandLittleIntermediate,
        BodyJointId.Body_RightHandLittleDistal,
        BodyJointId.Body_RightHandLittleTip
    };

    private void Start()
    {
        _body = GetComponent<IBody>();
        if (_body == null)
        {
            _body = GetComponentInChildren<IBody>();
        }
        if (_body == null)
        {
            LogHUD("Error: No IBody component found!");
            enabled = false;
            return;
        }

        _body.WhenBodyUpdated += OnBodyUpdated;
        _locomotionProvider = FindFirstObjectByType<JoystickLocomotionProvider>();
        LogHUD($"Body streamer found body source: {_body.GetType().Name}");
    }

    private void OnDestroy()
    {
        if (_body != null) _body.WhenBodyUpdated -= OnBodyUpdated;
        Disconnect();
    }

    private void OnBodyUpdated()
    {
        _hasReceivedBodyUpdate = true;
    }

    private void Update()
    {
        if (AppManager.Instance == null || !AppManager.Instance.isStreaming)
        {
            if (_isInitialized) Disconnect();
            return;
        }

        if (!_isInitialized) InitializeNetwork();

        _timer += Time.deltaTime;
        if (_timer >= _frequency)
        {
            _timer = 0f;
            try
            {
                ProcessBodyData();
            }
            catch (Exception ex)
            {
                _lastDiagnostic = $"{ex.GetType().Name}: {ex.Message}";
                if (!_hasLoggedRuntimeException)
                {
                    _hasLoggedRuntimeException = true;
                    UnityEngine.Debug.LogException(ex);
                    LogHUD($"Body streamer runtime error: {_lastDiagnostic}");
                }
            }
        }

        _diagnosticTimer += Time.deltaTime;
        if (_diagnosticTimer >= 1f)
        {
            _diagnosticTimer = 0f;
            LogTrackingDiagnostic();
        }
    }

    private void ProcessBodyData()
    {
        RefreshBodyData();

        if (!_body.IsTrackedDataValid)
        {
            _lastDiagnostic = "Body data invalid. Check Body Tracking permission, support, and calibration.";
            return;
        }

        if (_body.SkeletonMapping == null)
        {
            _lastDiagnostic = "Body skeleton mapping is missing.";
            return;
        }

        bool addDebugHeaderMeta = AppManager.Instance != null && AppManager.Instance.ShowDebugInfo;
        uint frameId = 0;
        ulong sendTimestampNs = 0;
        if (addDebugHeaderMeta)
        {
            _frameId++;
            frameId = _frameId;
            sendTimestampNs = GetMonotonicTimestampNs();
        }

        _sbPacket.Clear();

        if (_body.GetRootPose(out Pose rootPose))
        {
            if (addDebugHeaderMeta)
            {
                AppendHeaderWithMeta(_sbPacket, "root", frameId, sendTimestampNs);
                _sbPacket.Append(", ");
            }
            else
            {
                _sbPacket.Append("Body root:, ");
            }
            AppendPose(_sbPacket, ApplyLocomotion(rootPose));
            _sbPacket.Append("\n");
        }

        if (addDebugHeaderMeta)
        {
            AppendHeaderWithMeta(_sbPacket, _streamRotations ? "joints_pose" : "joints", frameId, sendTimestampNs);
        }
        else
        {
            _sbPacket.Append(_streamRotations ? "Body joints_pose:" : "Body joints:");
        }

        int validJointCount = 0;
        foreach (BodyJointId jointId in _upperBodyJoints)
        {
            _sbPacket.Append(", ");
            if (_body.SkeletonMapping.Joints.Contains(jointId) && _body.GetJointPose(jointId, out Pose jointPose))
            {
                validJointCount++;
                Pose streamedJointPose = ApplyLocomotion(jointPose);
                if (_streamRotations)
                {
                    AppendPose(_sbPacket, streamedJointPose);
                }
                else
                {
                    AppendVector3(_sbPacket, streamedJointPose.position);
                }
            }
            else
            {
                _sbPacket.Append(_streamRotations ? "0, 0, 0, 0, 0, 0, 1" : "0, 0, 0");
            }
        }

        _lastDiagnostic = $"Streaming upper body. Valid joints: {validJointCount}/{_upperBodyJoints.Length}. High confidence: {_body.IsHighConfidence}";
        SendData(_sbPacket.ToString());
    }

    private Pose ApplyLocomotion(Pose source)
    {
        if (_locomotionProvider == null)
        {
            _locomotionProvider = FindFirstObjectByType<JoystickLocomotionProvider>();
        }

        return _locomotionProvider != null
            ? _locomotionProvider.ApplyToPose(source)
            : source;
    }

    private void LogTrackingDiagnostic()
    {
        RefreshBodyData();
        bool hasMapping = _body.SkeletonMapping != null;
        LogHUD(
            $"UpperBody Tracking | Connected={_body.IsConnected} | Valid={_body.IsTrackedDataValid} | " +
            $"HighConf={_body.IsHighConfidence} | UpdatedEvent={_hasReceivedBodyUpdate} | " +
            $"Mapping={hasMapping} | {_lastDiagnostic}"
        );
    }

    private void RefreshBodyData()
    {
        if (_isRefreshingBodyData) return;

        if (_body is Body bodyComponent)
        {
            _isRefreshingBodyData = true;
            try
            {
                bodyComponent.ModifyDataFromSource.MarkInputDataRequiresUpdate();
                bodyComponent.MarkInputDataRequiresUpdate();
            }
            finally
            {
                _isRefreshingBodyData = false;
            }
        }
    }

    private void InitializeNetwork()
    {
        string ip = AppManager.Instance.ServerIP;
        int port = AppManager.Instance.ServerPort;
        _currentProtocol = AppManager.Instance.SelectedProtocol;

        try
        {
            if (_currentProtocol == 0)
            {
                _udpClient = new UdpClient();
                _udpClient.Client.SendBufferSize = 0;
                _remoteEndPoint = new IPEndPoint(IPAddress.Parse(ip), port);
                _udpClient.BeginReceive(new AsyncCallback(OnUdpReceive), null);
                LogHUD($"UDP Ready: {ip}:{port}");
            }
            else
            {
                _tcpClient = new TcpClient(AddressFamily.InterNetwork);
                _tcpClient.NoDelay = true;
                _tcpClient.SendTimeout = 1000;
                _tcpClient.ReceiveTimeout = 1000;
                _tcpClient.Connect(ip, port);
                _tcpStream = _tcpClient.GetStream();

                string type = _currentProtocol == 1 ? "Wired" : "WiFi";
                LogHUD($"TCP({type}) Connected: {ip}:{port}");
            }
            _isInitialized = true;
        }
        catch (Exception ex)
        {
            LogHUD($"Conn Error: {ex.Message}");
            AppManager.Instance.StopStreaming();
        }
    }

    private void OnUdpReceive(IAsyncResult res)
    {
        try
        {
            IPEndPoint remote = new IPEndPoint(IPAddress.Any, 0);
            _udpClient.EndReceive(res, ref remote);
            _udpClient.BeginReceive(new AsyncCallback(OnUdpReceive), null);
        }
        catch { }
    }

    private void SendData(string message)
    {
        if (AppManager.Instance != null && !AppManager.Instance.isStreaming) return;

        try
        {
            if (_currentProtocol == 0 && _udpClient != null)
            {
                byte[] data = Encoding.UTF8.GetBytes(message);
                _udpClient.Send(data, data.Length, _remoteEndPoint);
            }
            else if ((_currentProtocol == 1 || _currentProtocol == 2) && _tcpStream != null && _tcpStream.CanWrite)
            {
                byte[] data = Encoding.UTF8.GetBytes(message + "\n");
                _tcpStream.Write(data, 0, data.Length);
            }
        }
        catch (Exception ex)
        {
            Disconnect();
            if (AppManager.Instance != null)
            {
                AppManager.Instance.HandleDisconnection("Host Closed Connection:" + ex.Message);
            }
        }
    }

    private void Disconnect()
    {
        try
        {
            if (_udpClient != null) { _udpClient.Close(); _udpClient = null; }
            if (_tcpStream != null) { _tcpStream.Close(); _tcpStream = null; }
            if (_tcpClient != null) { _tcpClient.Close(); _tcpClient = null; }
        }
        catch { }
        _isInitialized = false;
    }

    private static ulong GetMonotonicTimestampNs()
    {
        return (ulong)(Stopwatch.GetTimestamp() * _ticksToNs);
    }

    private static void AppendHeaderWithMeta(StringBuilder sb, string section, uint frameId, ulong sendTimestampNs)
    {
        sb.Append("Body ")
          .Append(section)
          .Append(" | f = ")
          .Append(frameId)
          .Append(" | t = ")
          .Append(sendTimestampNs)
          .Append(":");
    }

    private static void AppendPose(StringBuilder sb, Pose pose)
    {
        AppendVector3(sb, pose.position);
        sb.Append(", ");
        AppendQuaternion(sb, pose.rotation);
    }

    private static void AppendVector3(StringBuilder sb, Vector3 vec)
    {
        sb.Append(vec.x.ToString("F4")).Append(", ")
          .Append(vec.y.ToString("F4")).Append(", ")
          .Append(vec.z.ToString("F4"));
    }

    private static void AppendQuaternion(StringBuilder sb, Quaternion q)
    {
        sb.Append(q.x.ToString("F3")).Append(", ")
          .Append(q.y.ToString("F3")).Append(", ")
          .Append(q.z.ToString("F3")).Append(", ")
          .Append(q.w.ToString("F3"));
    }

    private void LogHUD(string msg)
    {
        UnityEngine.Debug.Log($"[BodyLandmarkStreamer] {msg}");

        if (_logToHUD && LogManager.Instance != null)
        {
            LogManager.Instance.Log(_hudLogSource, msg);
        }
    }
}
