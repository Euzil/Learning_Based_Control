using UnityEngine;
using System.Globalization;

public class JoystickLocomotionProvider : MonoBehaviour
{
    [Header("Joystick Locomotion")]
    [SerializeField] private bool _enabled = true;
    [SerializeField] private float _moveSpeed = 1.0f;
    [SerializeField] private float _verticalSpeed = 0.6f;
    [SerializeField] private float _minVerticalOffset = -0.6f;
    [SerializeField] private float _maxVerticalOffset = 0.4f;
    [SerializeField] private float _turnSpeedDegrees = 90.0f;
    [SerializeField] private float _deadZone = 0.15f;
    [SerializeField] private Transform _directionReference;

    public Vector3 PositionOffset { get; private set; }
    public Vector3 HorizontalOffset => new Vector3(PositionOffset.x, 0f, PositionOffset.z);
    public float VerticalOffset => PositionOffset.y;
    public float YawOffsetDegrees { get; private set; }
    public Vector2 MoveInput { get; private set; }
    public float TurnInput { get; private set; }
    public float VerticalInput { get; private set; }

    public bool HasMotionInput => MoveInput.sqrMagnitude > 0.0001f || Mathf.Abs(TurnInput) > 0.0001f || Mathf.Abs(VerticalInput) > 0.0001f;

    private string _session = System.Guid.NewGuid().ToString("N");
    private uint _sequence;
    private float _initialHeading;

    // Tracking orientation stays physical. Joystick yaw is sent only in ReferencePacket.
    // Metadata describes only the translation actually applied to this tracking frame.
    public string AnnotateTracking(string packet)
    {
        string offset = string.Format(CultureInfo.InvariantCulture,
            " | locomotion = {0:R} {1:R} {2:R} {3:R}",
            PositionOffset.x, PositionOffset.y, PositionOffset.z, 0f);
        return packet.Replace(":", offset + ":");
    }

    public string ReferencePacket()
    {
        return string.Format(CultureInfo.InvariantCulture,
            "Locomotion reference:, {0}, {1}, {2:R}, {3:R}, {4:R}, {5:R}, {6:R}, {7:R}, {8:R}",
            _session, ++_sequence, PositionOffset.x, PositionOffset.y, PositionOffset.z,
            YawOffsetDegrees * Mathf.Deg2Rad, _initialHeading, MoveInput.magnitude, TurnInput);
    }

    private void Update()
    {
        if (!_enabled || AppManager.Instance == null || !AppManager.Instance.isStreaming)
        {
            MoveInput = Vector2.zero;
            TurnInput = 0f;
            VerticalInput = 0f;
            return;
        }

        // Raw axes avoid the virtual Primary/Secondary remapping of individual Touch devices.
        Vector2 leftStick = OVRInput.Get(OVRInput.RawAxis2D.LThumbstick, OVRInput.Controller.Touch);
        Vector2 rightStick = OVRInput.Get(OVRInput.RawAxis2D.RThumbstick, OVRInput.Controller.Touch);
        float forwardTrigger = Trigger(OVRInput.RawAxis1D.RIndexTrigger);
        float backwardTrigger = Trigger(OVRInput.RawAxis1D.LIndexTrigger);
        float upTrigger = Trigger(OVRInput.RawAxis1D.RHandTrigger);
        float downTrigger = Trigger(OVRInput.RawAxis1D.LHandTrigger);

        MoveInput = new Vector2(0f, forwardTrigger - backwardTrigger);
        float turn = Mathf.Abs(leftStick.x) > Mathf.Abs(rightStick.x) ? leftStick.x : rightStick.x;
        TurnInput = Mathf.Abs(turn) >= _deadZone ? turn : 0f;
        VerticalInput = upTrigger - downTrigger;

        Vector3 forward = ResolveForward();
        Vector3 right = Vector3.Cross(Vector3.up, forward).normalized;
        Vector3 planarMove = forward * MoveInput.y + right * MoveInput.x;
        if (planarMove.sqrMagnitude > 1f)
        {
            planarMove.Normalize();
        }

        Vector3 nextOffset = PositionOffset + planarMove * _moveSpeed * Time.deltaTime;
        nextOffset.y = Mathf.Clamp(
            nextOffset.y + VerticalInput * _verticalSpeed * Time.deltaTime,
            _minVerticalOffset,
            _maxVerticalOffset);
        PositionOffset = nextOffset;
        YawOffsetDegrees += TurnInput * _turnSpeedDegrees * Time.deltaTime;
    }

    public Pose ApplyToPose(Pose source)
    {
        return new Pose(source.position + PositionOffset, source.rotation);
    }

    public Vector3 ApplyToPosition(Vector3 source)
    {
        return source + PositionOffset;
    }

    public Pose ApplyHorizontalToPose(Pose source)
    {
        return new Pose(source.position + HorizontalOffset, source.rotation);
    }

    public Vector3 ApplyHorizontalToPosition(Vector3 source)
    {
        return source + HorizontalOffset;
    }

    public void ResetLocomotion()
    {
        _session = System.Guid.NewGuid().ToString("N");
        _sequence = 0;
        Vector3 initialForward = ResolveForward();
        _initialHeading = Mathf.Atan2(initialForward.x, initialForward.z);
        PositionOffset = Vector3.zero;
        YawOffsetDegrees = 0f;
        MoveInput = Vector2.zero;
        TurnInput = 0f;
        VerticalInput = 0f;
    }

    private Vector3 ResolveForward()
    {
        Transform reference = _directionReference;
        if (reference == null && Camera.main != null)
        {
            reference = Camera.main.transform;
        }

        Vector3 forward = reference != null ? reference.forward : transform.forward;
        forward.y = 0f;
        if (forward.sqrMagnitude < 0.0001f)
        {
            return Vector3.forward;
        }

        return forward.normalized;
    }

    private float Trigger(OVRInput.RawAxis1D axis)
    {
        float value = OVRInput.Get(axis, OVRInput.Controller.Touch);
        return value <= _deadZone ? 0f : Mathf.Clamp01((value - _deadZone) / (1f - _deadZone));
    }
}
