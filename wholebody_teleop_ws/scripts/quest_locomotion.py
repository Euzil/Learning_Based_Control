"""Quest joystick world goals and a bounded position-to-velocity outer loop.

No ROS or simulator state writes: the learned policy executes all movement.
"""
import math
import re
import numpy as np


def parse_reference(line):
    label, sep, payload = line.partition(':')
    if not sep or label.strip().lower() != 'locomotion reference':
        return None
    fields = [v.strip() for v in payload.strip().lstrip(',').split(',')]
    if len(fields) != 9 or not re.fullmatch(r'[0-9a-f]{32}', fields[0]):
        return None
    try:
        seq = int(fields[1])
        values = np.array([float(v) for v in fields[2:]])
    except ValueError:
        return None
    if seq < 1 or not np.isfinite(values).all() or not 0 <= values[5] <= 1.001 or abs(values[6]) > 1.001:
        return None
    return fields[0], seq, values


def remove_locomotion(label, poses):
    """Undo the exact translation and left quaternion multiplication of this frame."""
    parts = [p.strip() for p in label.split('|') if p.strip().startswith('locomotion')]
    if not parts:
        return poses  # Compatible with old APKs (no independent locomotion control).
    if len(parts) != 1:
        return None
    try:
        offset = np.array([float(v) for v in parts[0].split('=', 1)[1].split()])
    except (ValueError, IndexError):
        return None
    if offset.shape != (4,) or not np.isfinite(offset).all():
        return None
    result = poses.copy().reshape(-1, 7)
    result[:, :3] -= offset[:3]
    x, y, z, w = result[:, 3:].T.copy()
    s, c = math.sin(-offset[3]/2), math.cos(-offset[3]/2)
    result[:, 3:] = np.column_stack((c*x+s*z, c*y+s*w, c*z-s*x, c*w-s*y))
    return result.reshape(poses.shape)


def yaw_from_xyzw(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < .1:
        raise ValueError('Invalid odometry quaternion')
    x, y, z, w = q/np.linalg.norm(q)
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def rotation(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


def wrap(angle):
    return (angle+math.pi) % (2*math.pi)-math.pi


class WorldReferenceController:
    def __init__(self, default_height, height_range):
        self.default_height = float(default_height)
        self.height_range = np.asarray(height_range, dtype=float)
        if (self.height_range.shape != (2,) or not np.isfinite(self.height_range).all()
                or self.height_range[0] > self.default_height or self.height_range[1] < self.default_height):
            raise ValueError('Invalid model height range')
        self.reset()

    def reset(self):
        self.session = None
        self.sequence = 0
        self.received = -math.inf
        self.goal = None
        self.height = self.default_height
        self.active = False
        self.previous = None
        self.previous_height_offset = None
        self.previous_translation = np.zeros(2)
        self.previous_turn = 0.
        self.move = self.turn = False
        self.retired_sessions = set()

    def accept(self, reference, position, yaw, now):
        session, sequence, values = reference
        if session in self.retired_sessions:
            return False
        if session == self.session and sequence <= self.sequence:
            return False
        # Unity right/up/forward -> robot forward/left/up, then initial heading alignment.
        offset = np.array([values[2], -values[0]])
        new_session = session != self.session
        reconnect = now-self.received > .3
        if new_session or reconnect or self.goal is None:
            if new_session and self.session is not None:
                self.retired_sessions.add(self.session)
            self.alignment = yaw + values[4]
            self.goal = np.array([position[0], position[1], yaw], dtype=float)
            self.previous = np.r_[offset, -values[3]]
            self.move = self.turn = False
        delta = np.r_[offset, -values[3]]-self.previous
        translation = rotation(self.alignment) @ delta[:2]

        moving, turning = values[5] > 1e-4, abs(values[6]) > 1e-4
        if moving and self.move:
            if np.dot(translation, self.previous_translation) < -1e-10:
                self.goal[:2] = position[:2]
            self.goal[:2] += translation
        else:
            self.goal[:2] = position[:2]
        if turning and self.turn:
            if delta[2]*self.previous_turn < -1e-10:
                self.goal[2] = yaw
            self.goal[2] += delta[2]
        else:
            self.goal[2] = yaw
        self.previous = np.r_[offset, -values[3]]
        self.move, self.turn = moving, turning
        if np.linalg.norm(translation) > 1e-8:
            self.previous_translation = translation
        if abs(delta[2]) > 1e-8:
            self.previous_turn = delta[2]
        # Accumulate only achievable height changes: reversing at a limit responds immediately.
        if new_session or self.previous_height_offset is None:
            self.height = float(np.clip(self.default_height+values[1], *self.height_range))
        elif not reconnect:
            self.height = float(np.clip(self.height+values[1]-self.previous_height_offset,
                                        *self.height_range))
        self.previous_height_offset = values[1]
        self.session, self.sequence, self.received = session, sequence, now
        self.active = True
        return True

    def command(self, position, yaw, now, enabled=True):
        if not enabled or now-self.received > .3:
            self.active = False
            self.move = self.turn = False
        if not self.active:
            if self.goal is not None:
                self.goal[:] = [position[0], position[1], yaw]
            return np.zeros(3), self.height
        error = self.goal[:2]-position[:2]
        # Prevent accumulated unreachable goals from producing a long catch-up motion.
        length = np.linalg.norm(error)
        if length > .25:
            error *= .25/length
            self.goal[:2] = position[:2]+error
        cmd = np.zeros(3)
        if self.move:
            cmd[:2] = rotation(-yaw) @ (1.5*error)
        else:
            self.goal[:2] = position[:2]
        if self.turn:
            # Allow the WBC's full yaw-rate range; a 0.6 rad/s cap stalled in-place stepping.
            # QuestPolicy still clips this command to the selected model's own limits.
            error_yaw = np.clip(wrap(self.goal[2]-yaw), -.8, .8)
            self.goal[2] = yaw+error_yaw
            cmd[2] = 1.5*error_yaw
        else:
            self.goal[2] = yaw
        return cmd, self.height
