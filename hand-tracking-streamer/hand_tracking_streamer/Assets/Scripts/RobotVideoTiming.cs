// Pure timing policy: independent of ROS, WebRTC and Unity's scaled game time.
public sealed class RobotVideoTiming
{
    public bool HasFrame { get; private set; }
    private double startedAt;
    private double lastFrameAt;

    public void Begin(double now)
    {
        HasFrame = false;
        startedAt = lastFrameAt = now;
    }

    public void FrameDecoded(double now)
    {
        HasFrame = true;
        lastFrameAt = now;
    }

    public bool TimedOut(double now, double firstFrameTimeout, double stalledFrameTimeout)
    {
        return HasFrame ? now - lastFrameAt >= stalledFrameTimeout
                        : now - startedAt >= firstFrameTimeout;
    }
}
