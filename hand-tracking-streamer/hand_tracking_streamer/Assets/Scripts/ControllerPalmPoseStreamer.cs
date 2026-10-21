using System;
using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using Debug = UnityEngine.Debug;

public class ControllerPalmPoseStreamer : MonoBehaviour
{
    public enum ControllerSide { Left, Right }

    [Header("Configuration")]
    [Tooltip("Which Quest Touch controller should act as the robot palm target.")]
    [SerializeField] private ControllerSide _controllerSide = ControllerSide.Left;

    [Tooltip("Optional tracking space. If empty, the script uses OVRCameraRig.trackingSpace when available.")]
    [SerializeField] private Transform _trackingSpace;

    [SerializeField] private float _frequency = 0.01f;

    [Header("Logging")]
    [SerializeField] private bool _logToHUD = true;
    [SerializeField] private string _hudLogSource = "Right";

    private UdpClient _udpClient;
    private TcpClient _tcpClient;
    private NetworkStream _tcpStream;
    private IPEndPoint _remoteEndPoint;
    private bool _isInitialized;
    private int _currentProtocol = -1;
    private float _timer;
    private uint _frameId;
    private JoystickLocomotionProvider _locomotionProvider;

    private readonly StringBuilder _sbPacket = new StringBuilder(256);
    private readonly StringBuilder _sbLog = new StringBuilder(256);

    private static readonly double TicksToNs = 1_000_000_000.0 / Stopwatch.Frequency;

    private OVRInput.Controller OvrController => _controllerSide == ControllerSide.Left
        ? OVRInput.Controller.LTouch
        : OVRInput.Controller.RTouch;

    public ControllerSide Side => _controllerSide;

    public void Configure(ControllerSide side)
    {
        _controllerSide = side;
    }

    private void Update()
    {
        if (AppManager.Instance == null || !AppManager.Instance.isStreaming || !AppManager.Instance.UseControllerPalm)
        {
            if (_isInitialized) Disconnect();
            return;
        }

        int mode = AppManager.Instance.SelectedHandMode;
        if (mode == 1 && _controllerSide == ControllerSide.Right) return;
        if (mode == 2 && _controllerSide == ControllerSide.Left) return;

        if (!IsControllerConnected())
        {
            if (_isInitialized) Disconnect();
            return;
        }

        if (!_isInitialized) InitializeNetwork();

        _timer += Time.deltaTime;
        if (_timer < _frequency)
        {
            return;
        }
        _timer = 0f;

        SendPalmPose();
    }

    private void OnDestroy()
    {
        Disconnect();
    }

    private void SendPalmPose()
    {
        Pose pose = ApplyLocomotion(ReadControllerPose());

        _sbPacket.Clear();
        _sbLog.Clear();

        bool addDebugHeaderMeta = AppManager.Instance != null && AppManager.Instance.ShowDebugInfo;
        string wristSection = AppManager.Instance != null
            && AppManager.Instance.SelectedInputMode == AppManager.ControlInputMode.FullHandAndController
                ? "controller_wrist"
                : "wrist";

        if (addDebugHeaderMeta)
        {
            _frameId++;
            AppendHeaderWithMeta(_sbPacket, wristSection, _frameId, GetMonotonicTimestampNs());
            _sbPacket.Append(", ");
        }
        else
        {
            _sbPacket.Append(_controllerSide).Append(" ").Append(wristSection).Append(":, ");
        }

        AppendPose(_sbPacket, pose);

        if (_logToHUD)
        {
            _sbLog.AppendLine($"=== [{_controllerSide} Controller Palm] ===");
            _sbLog.AppendLine($"Pos: {pose.position.ToString("F3")}");
            LogHUD(_sbLog.ToString());
        }

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

    private bool IsControllerConnected()
    {
        OVRInput.Controller connected = OVRInput.GetConnectedControllers();
        return (connected & OvrController) != 0;
    }

    private Pose ReadControllerPose()
    {
        Vector3 localPosition = OVRInput.GetLocalControllerPosition(OvrController);
        Quaternion localRotation = OVRInput.GetLocalControllerRotation(OvrController);

        Transform space = ResolveTrackingSpace();
        if (space == null)
        {
            return new Pose(localPosition, localRotation);
        }

        return new Pose(
            space.TransformPoint(localPosition),
            space.rotation * localRotation
        );
    }

    private Transform ResolveTrackingSpace()
    {
        if (_trackingSpace != null)
        {
            return _trackingSpace;
        }

        OVRCameraRig rig = FindFirstObjectByType<OVRCameraRig>();
        if (rig != null)
        {
            _trackingSpace = rig.trackingSpace;
        }

        return _trackingSpace;
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
                LogHUD($"Controller palm UDP Ready: {ip}:{port}");
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
                LogHUD($"Controller palm TCP({type}) Connected: {ip}:{port}");
            }
            _isInitialized = true;
        }
        catch (Exception ex)
        {
            LogHUD($"Controller palm Conn Error: {ex.Message}");
            AppManager.Instance.StopStreaming();
        }
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
                AppManager.Instance.HandleDisconnection("Controller palm send failed: " + ex.Message);
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
        return (ulong)(Stopwatch.GetTimestamp() * TicksToNs);
    }

    private void AppendHeaderWithMeta(StringBuilder sb, string section, uint frameId, ulong sendTimestampNs)
    {
        sb.Append(_controllerSide)
          .Append(" ")
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
        Debug.Log($"[ControllerPalmPoseStreamer:{_controllerSide}] {msg}");

        if (_logToHUD && LogManager.Instance != null)
        {
            LogManager.Instance.Log(_hudLogSource, msg);
        }
    }
}
