using UnityEngine;

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

    private void Update()
    {
        if (!_enabled || AppManager.Instance == null || !AppManager.Instance.isStreaming)
        {
            MoveInput = Vector2.zero;
            TurnInput = 0f;
            VerticalInput = 0f;
            return;
        }

        Vector2 move = OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick, OVRInput.Controller.LTouch);
        Vector2 turnAndHeight = OVRInput.Get(OVRInput.Axis2D.SecondaryThumbstick, OVRInput.Controller.RTouch);

        MoveInput = ApplyDeadZone(move);
        TurnInput = Mathf.Abs(turnAndHeight.x) >= _deadZone ? turnAndHeight.x : 0f;
        VerticalInput = Mathf.Abs(turnAndHeight.y) >= _deadZone ? turnAndHeight.y : 0f;

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
        Quaternion yawRotation = Quaternion.Euler(0f, YawOffsetDegrees, 0f);
        return new Pose(source.position + PositionOffset, yawRotation * source.rotation);
    }

    public Vector3 ApplyToPosition(Vector3 source)
    {
        return source + PositionOffset;
    }

    public Pose ApplyHorizontalToPose(Pose source)
    {
        Quaternion yawRotation = Quaternion.Euler(0f, YawOffsetDegrees, 0f);
        return new Pose(source.position + HorizontalOffset, yawRotation * source.rotation);
    }

    public Vector3 ApplyHorizontalToPosition(Vector3 source)
    {
        return source + HorizontalOffset;
    }

    public void ResetLocomotion()
    {
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

    private Vector2 ApplyDeadZone(Vector2 input)
    {
        if (input.magnitude < _deadZone)
        {
            return Vector2.zero;
        }

        return Vector2.ClampMagnitude(input, 1f);
    }
}
