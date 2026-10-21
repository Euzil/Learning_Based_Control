using UnityEngine;
using Oculus.Interaction.Body.Input;
using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using Stopwatch = System.Diagnostics.Stopwatch;

public class FootLandmarkStreamer : MonoBehaviour
{
    public enum FootSide { Left, Right }

    [Header("Configuration")]
    [Tooltip("Which foot data to track and stream")]
    [SerializeField] private FootSide _footSide;

    [Header("Logging")]
    [SerializeField] private bool _logToHUD = true;

    [Tooltip("The Log Source name to push text to")]
    [SerializeField] private string _hudLogSource = "Right";

    [SerializeField] private float _frequency = 0.01f;
    [Tooltip("Disabled by default because the production robot generates its own lower-body pose.")]
    [SerializeField] private bool _streamFootData = false;

    private IBody _body;
    private float _timer;

    private UdpClient _udpClient;
    private TcpClient _tcpClient;
    private NetworkStream _tcpStream;
    private IPEndPoint _remoteEndPoint;
    private bool _isInitialized;
    private int _currentProtocol = -1;
    private bool _hasReceivedBodyUpdate;
    private float _diagnosticTimer;
    private string _lastDiagnostic = "Waiting for body tracking data.";
    private bool _hasLoggedRuntimeException;
    private JoystickLocomotionProvider _locomotionProvider;
    private bool _hasLastAnklePosition;
    private Vector3 _lastAnklePosition;
    private float _lastAnkleDelta;
    private float _diagnosticWindowDelta;
    private int _streamedFrames;

    private readonly StringBuilder _sbPacket = new StringBuilder(2048);
    private readonly StringBuilder _sbLog = new StringBuilder(2048);

    private uint _frameId;
    private static readonly double _ticksToNs = 1_000_000_000.0 / Stopwatch.Frequency;

    private BodyJointId[] FootJoints => _footSide == FootSide.Left
        ? _leftFootJoints
        : _rightFootJoints;

    private BodyJointId AnkleJoint => _footSide == FootSide.Left
        ? BodyJointId.Body_LeftFootAnkle
        : BodyJointId.Body_RightFootAnkle;

    private static readonly BodyJointId[] _leftFootJoints =
    {
        BodyJointId.Body_LeftLegUpper,
        BodyJointId.Body_LeftLegLower,
        BodyJointId.Body_LeftFootAnkleTwist,
        BodyJointId.Body_LeftFootAnkle,
        BodyJointId.Body_LeftFootSubtalar,
        BodyJointId.Body_LeftFootTransverse,
        BodyJointId.Body_LeftFootBall
    };

    private static readonly BodyJointId[] _rightFootJoints =
    {
        BodyJointId.Body_RightLegUpper,
        BodyJointId.Body_RightLegLower,
        BodyJointId.Body_RightFootAnkleTwist,
        BodyJointId.Body_RightFootAnkle,
        BodyJointId.Body_RightFootSubtalar,
        BodyJointId.Body_RightFootTransverse,
        BodyJointId.Body_RightFootBall
    };

    public IBody Body => _body;
    public FootSide Side => _footSide;

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
        LogHUD($"Foot streamer [{_footSide}] found body source: {_body.GetType().Name}");
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
        if (!_streamFootData || AppManager.Instance == null || !AppManager.Instance.isStreaming || !AppManager.Instance.UseHandTracking)
        {
            if (_isInitialized) Disconnect();
            return;
        }

        int mode = AppManager.Instance.SelectedHandMode;
        if (mode == 1 && _footSide == FootSide.Right) return;
        if (mode == 2 && _footSide == FootSide.Left) return;

        if (!_isInitialized) InitializeNetwork();

        _timer += Time.deltaTime;
        if (_timer >= _frequency)
        {
            _timer = 0f;
            try
            {
                ProcessFootData();
            }
            catch (Exception ex)
            {
                _lastDiagnostic = $"{ex.GetType().Name}: {ex.Message}";
                if (!_hasLoggedRuntimeException)
                {
                    _hasLoggedRuntimeException = true;
                    UnityEngine.Debug.LogException(ex);
                    LogHUD($"Foot streamer runtime error: {_lastDiagnostic}");
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

    private void ProcessFootData()
    {
        RefreshBodyData();

        if (!_body.IsTrackedDataValid)
        {
            _lastDiagnostic = "Body data invalid. Check Body Tracking permission, device support, and user calibration.";
            return;
        }

        if (_body.SkeletonMapping == null)
        {
            _lastDiagnostic = "Body skeleton mapping is missing.";
            return;
        }

        if (!_body.SkeletonMapping.Joints.Contains(AnkleJoint))
        {
            _lastDiagnostic = "Foot joints unavailable. Body tracking is not using the FullBody joint set.";
            return;
        }

        if (!_body.GetJointPose(AnkleJoint, out Pose anklePose))
        {
            _lastDiagnostic = $"Ankle pose unavailable: {AnkleJoint}.";
            return;
        }

        _lastAnkleDelta = _hasLastAnklePosition
            ? Vector3.Distance(_lastAnklePosition, anklePose.position)
            : 0f;
        _diagnosticWindowDelta += _lastAnkleDelta;
        _lastAnklePosition = anklePose.position;
        _hasLastAnklePosition = true;
        _streamedFrames++;

        _lastDiagnostic = $"Streaming {_footSide} foot. High confidence: {_body.IsHighConfidence}";

        _sbPacket.Clear();
        _sbLog.Clear();

        bool addDebugHeaderMeta = AppManager.Instance != null && AppManager.Instance.ShowDebugInfo;
        uint frameId = 0;
        ulong sendTimestampNs = 0;

        if (addDebugHeaderMeta)
        {
            _frameId++;
            frameId = _frameId;
            sendTimestampNs = GetMonotonicTimestampNs();
        }

        if (addDebugHeaderMeta)
        {
            AppendHeaderWithMeta(_sbPacket, "foot", frameId, sendTimestampNs);
            _sbPacket.Append(", ");
        }
        else
        {
            _sbPacket.Append(_footSide).Append(" foot:, ");
        }

        AppendVector3(_sbPacket, ApplyLocomotion(anklePose.position));
        _sbPacket.Append(", ");
        AppendQuaternion(_sbPacket, anklePose.rotation);

        _sbPacket.Append("\n");
        if (addDebugHeaderMeta)
        {
            AppendHeaderWithMeta(_sbPacket, "foot_landmarks", frameId, sendTimestampNs);
        }
        else
        {
            _sbPacket.Append(_footSide).Append(" foot_landmarks:");
        }

        Quaternion inverseAnkleRotation = Quaternion.Inverse(anklePose.rotation);
        foreach (BodyJointId jointId in FootJoints)
        {
            _sbPacket.Append(", ");
            if (_body.GetJointPose(jointId, out Pose jointPose))
            {
                Vector3 ankleRelativePosition = inverseAnkleRotation * (jointPose.position - anklePose.position);
                AppendVector3(_sbPacket, ankleRelativePosition);
            }
            else
            {
                _sbPacket.Append("0, 0, 0");
            }
        }

        if (_logToHUD)
        {
            _sbLog.AppendLine($"=== [{_footSide}] Foot ===");
            _sbLog.AppendLine($"Ankle: {anklePose.position.ToString("F3")}");
            foreach (BodyJointId jointId in FootJoints)
            {
                if (_body.GetJointPose(jointId, out Pose jointPose))
                {
                    _sbLog.AppendLine($"{GetJointLabel(jointId)}: {jointPose.position.ToString("F3")}");
                }
            }
            LogHUD(_sbLog.ToString());
        }

        SendData(_sbPacket.ToString());
    }

    private Vector3 ApplyLocomotion(Vector3 source)
    {
        if (_locomotionProvider == null)
        {
            _locomotionProvider = FindFirstObjectByType<JoystickLocomotionProvider>();
        }

        return _locomotionProvider != null
            ? _locomotionProvider.ApplyToPosition(source)
            : source;
    }

    private void LogTrackingDiagnostic()
    {
        if (!_logToHUD) return;

        RefreshBodyData();

        bool hasMapping = _body.SkeletonMapping != null;
        bool hasAnkleJoint = hasMapping && _body.SkeletonMapping.Joints.Contains(AnkleJoint);
        LogHUD(
            $"=== [{_footSide}] Body Tracking ===\n" +
            $"Connected: {_body.IsConnected}\n" +
            $"Valid: {_body.IsTrackedDataValid}\n" +
            $"HighConf: {_body.IsHighConfidence}\n" +
            $"UpdatedEvent: {_hasReceivedBodyUpdate}\n" +
            $"FullBodyFoot: {hasAnkleJoint}\n" +
            $"Ankle: {(_hasLastAnklePosition ? _lastAnklePosition.ToString("F3") : "n/a")}\n" +
            $"LastDelta: {_lastAnkleDelta:F5}\n" +
            $"WindowDelta: {_diagnosticWindowDelta:F5}\n" +
            $"Frames: {_streamedFrames}\n" +
            $"{_lastDiagnostic}"
        );

        _diagnosticWindowDelta = 0f;
        _streamedFrames = 0;
    }

    private void RefreshBodyData()
    {
        if (_body is Body bodyComponent)
        {
            bodyComponent.ModifyDataFromSource.MarkInputDataRequiresUpdate();
            bodyComponent.MarkInputDataRequiresUpdate();
            _hasReceivedBodyUpdate = true;
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

    private void AppendHeaderWithMeta(StringBuilder sb, string section, uint frameId, ulong sendTimestampNs)
    {
        sb.Append(_footSide)
          .Append(" ")
          .Append(section)
          .Append(" | f = ")
          .Append(frameId)
          .Append(" | t = ")
          .Append(sendTimestampNs)
          .Append(":");
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

    private static string GetJointLabel(BodyJointId jointId)
    {
        string name = jointId.ToString();
        return name.StartsWith("Body_", StringComparison.Ordinal) ? name.Substring(5) : name;
    }

    private void LogHUD(string msg)
    {
        UnityEngine.Debug.Log($"[FootLandmarkStreamer:{_footSide}] {msg}");

        if (_logToHUD && LogManager.Instance != null)
        {
            LogManager.Instance.Log(_hudLogSource, msg);
        }
    }
}
