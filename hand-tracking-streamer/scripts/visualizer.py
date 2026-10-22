"""Matplotlib visualizer for HTS hands, controllers, and upper-body landmarks.

Usage:
    python visualizer.py --protocol udp --host 0.0.0.0 --port 9000
    python visualizer.py --protocol tcp --host localhost --port 8000
"""

from __future__ import annotations

import argparse
import logging
import socket
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, Iterable, Optional, Tuple

try:
    import matplotlib.pyplot as plt
    import numpy as np
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "Missing dependency. Please install numpy and matplotlib."
    ) from exc


def _quat_normalize(quat: np.ndarray) -> np.ndarray:
    """Return a normalized quaternion."""
    norm = np.linalg.norm(quat)
    if norm <= 0.0:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return quat / norm


def _quat_rotate(points: np.ndarray, quat: np.ndarray) -> np.ndarray:
    """Rotate Nx3 points by quaternion (x, y, z, w)."""
    quat = _quat_normalize(quat)
    q_xyz = quat[:3]
    q_w = quat[3]
    t = 2.0 * np.cross(q_xyz, points)
    return points + q_w * t + np.cross(q_xyz, t)


def _quat_to_matrix(quat: np.ndarray) -> np.ndarray:
    """Convert a quaternion (x, y, z, w) to a 3x3 rotation matrix."""
    quat = _quat_normalize(quat)
    x, y, z, w = quat
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
        ],
        dtype=float,
    )


def _matrix_to_quat(mat: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix to a quaternion (x, y, z, w)."""
    trace = mat[0, 0] + mat[1, 1] + mat[2, 2]
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (mat[2, 1] - mat[1, 2]) * s
        y = (mat[0, 2] - mat[2, 0]) * s
        z = (mat[1, 0] - mat[0, 1]) * s
    elif mat[0, 0] > mat[1, 1] and mat[0, 0] > mat[2, 2]:
        s = 2.0 * np.sqrt(1.0 + mat[0, 0] - mat[1, 1] - mat[2, 2])
        w = (mat[2, 1] - mat[1, 2]) / s
        x = 0.25 * s
        y = (mat[0, 1] + mat[1, 0]) / s
        z = (mat[0, 2] + mat[2, 0]) / s
    elif mat[1, 1] > mat[2, 2]:
        s = 2.0 * np.sqrt(1.0 + mat[1, 1] - mat[0, 0] - mat[2, 2])
        w = (mat[0, 2] - mat[2, 0]) / s
        x = (mat[0, 1] + mat[1, 0]) / s
        y = 0.25 * s
        z = (mat[1, 2] + mat[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + mat[2, 2] - mat[0, 0] - mat[1, 1])
        w = (mat[1, 0] - mat[0, 1]) / s
        x = (mat[0, 2] + mat[2, 0]) / s
        y = (mat[1, 2] + mat[2, 1]) / s
        z = 0.25 * s
    return _quat_normalize(np.array([x, y, z, w], dtype=float))


# Unity LH (x right, y up, z forward) -> RH (x front, y left, z up)
_UNITY_TO_RH = np.array(
    [
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ],
    dtype=float,
)


def _convert_vec(vec: np.ndarray) -> np.ndarray:
    """Convert a vector from Unity LH to RH coordinates."""
    return _UNITY_TO_RH @ vec


def _convert_quat(quat: np.ndarray) -> np.ndarray:
    """Convert a quaternion from Unity LH to RH coordinates."""
    r_unity = _quat_to_matrix(quat)
    r_rh = _UNITY_TO_RH @ r_unity @ _UNITY_TO_RH.T
    return _matrix_to_quat(r_rh)


@dataclass
class HandState:
    """State for a single hand."""

    side: str
    wrist_position: Optional[np.ndarray] = None
    wrist_quat: Optional[np.ndarray] = None
    landmarks_local: Optional[np.ndarray] = None
    last_update: float = field(default_factory=time.monotonic)

    def update_wrist(self, data: Iterable[float]) -> None:
        """Update wrist pose from a 7-float sequence."""
        values = np.array(list(data), dtype=float)
        if values.size < 7:
            return
        self.wrist_position = _convert_vec(values[:3])
        self.wrist_quat = _convert_quat(values[3:7])
        self.last_update = time.monotonic()

    def update_landmarks(self, data: Iterable[float]) -> None:
        """Update local landmarks from a flat xyz array."""
        values = np.array(list(data), dtype=float)
        if values.size < 3:
            return
        if values.size % 3 != 0:
            values = values[: values.size - (values.size % 3)]
        reshaped = values.reshape((-1, 3))
        self.landmarks_local = (_UNITY_TO_RH @ reshaped.T).T
        self.last_update = time.monotonic()

    def world_points(self) -> Optional[np.ndarray]:
        """Return landmarks transformed to world space."""
        if self.landmarks_local is None:
            return None
        if self.wrist_position is None or self.wrist_quat is None:
            return self.landmarks_local
        return _quat_rotate(self.landmarks_local, self.wrist_quat) + self.wrist_position

    def wrist_point(self) -> Optional[np.ndarray]:
        """Return the wrist position if available."""
        if self.wrist_position is None:
            return None
        return self.wrist_position


@dataclass
class BodyState:
    """State for the upper-body stream."""

    root_position: Optional[np.ndarray] = None
    root_quat: Optional[np.ndarray] = None
    joints_world: Optional[np.ndarray] = None
    joint_quats: Optional[np.ndarray] = None
    last_update: float = field(default_factory=time.monotonic)

    def update_root(self, data: Iterable[float]) -> None:
        values = np.array(list(data), dtype=float)
        if values.size < 7:
            return
        self.root_position = _convert_vec(values[:3])
        self.root_quat = _convert_quat(values[3:7])
        self.last_update = time.monotonic()

    def update_joints(self, data: Iterable[float], with_rotations: bool) -> None:
        values = np.array(list(data), dtype=float)
        stride = 7 if with_rotations else 3
        if values.size < stride:
            return
        values = values[: values.size - (values.size % stride)]
        reshaped = values.reshape((-1, stride))
        self.joints_world = (_UNITY_TO_RH @ reshaped[:, :3].T).T
        if with_rotations:
            self.joint_quats = np.array(
                [_convert_quat(quat) for quat in reshaped[:, 3:7]],
                dtype=float,
            )
        self.last_update = time.monotonic()


def _parse_line(line: str) -> Optional[Tuple[str, str, Tuple[float, ...]]]:
    """Parse a CSV line into (side, kind, floats)."""
    parts = [part.strip() for part in line.split(",")]
    if not parts:
        return None
    label = parts[0].lower()
    if "foot" in label:
        return None

    if "body root" in label or "body joints" in label:
        if "joints_pose" in label:
            kind = "body_joints_pose"
        elif "joints" in label:
            kind = "body_joints"
        else:
            kind = "body_root"
        floats = []
        for part in parts[1:]:
            if not part:
                continue
            try:
                floats.append(float(part))
            except ValueError:
                continue
        return ("body", kind, tuple(floats))

    if (
        "wrist" not in label
        and "controller" not in label
        and "landmarks" not in label
    ):
        return None
    side = "right" if "right" in label else "left" if "left" in label else ""
    if not side:
        return None
    if "controller_wrist" in label or ("controller" in label and "wrist" in label):
        kind = "controller_wrist"
    elif "wrist" in label:
        kind = "wrist"
    else:
        kind = "landmarks"
    floats = []
    for part in parts[1:]:
        if not part:
            continue
        try:
            floats.append(float(part))
        except ValueError:
            continue
    return (side, kind, tuple(floats))


class StreamReceiver:
    """Background receiver for UDP/TCP hand data."""

    def __init__(self, protocol: str, host: str, port: int) -> None:
        import os
        forward_port = os.environ.get("HTS_SIM_FORWARD_PORT")
        self._forward_target = ("127.0.0.1", int(forward_port)) if forward_port else None
        self._forward_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if forward_port else None
        if self._forward_socket is not None:
            self._forward_socket.setblocking(False)
        self.protocol = protocol
        self.host = host
        self.port = port
        self.hands: Dict[str, HandState] = {
            "right": HandState("right"),
            "left": HandState("left"),
        }
        self.controllers: Dict[str, HandState] = {
            "right": HandState("right"),
            "left": HandState("left"),
        }
        self.body = BodyState()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._conn_threads: list[threading.Thread] = []

    def start(self) -> None:
        """Start the receiver thread."""
        self._thread.start()

    def stop(self) -> None:
        """Stop the receiver thread and active connections."""
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        for thread in list(self._conn_threads):
            if thread.is_alive():
                thread.join(timeout=0.5)

    def _handle_line(self, line: str) -> None:
        if self._forward_socket is not None and line.strip().lower().startswith("body joints_pose:"):
            try:
                self._forward_socket.sendto(line.encode("utf-8"), self._forward_target)
            except OSError:
                pass  # A slow/offline simulator must not interrupt live capture.
        parsed = _parse_line(line)
        if not parsed:
            return
        side, kind, floats = parsed
        if kind == "wrist":
            hand = self.hands[side]
            hand.update_wrist(floats)
        elif kind == "landmarks":
            hand = self.hands[side]
            hand.update_landmarks(floats)
        elif kind == "controller_wrist":
            controller = self.controllers[side]
            controller.update_wrist(floats)
        elif kind == "body_root":
            self.body.update_root(floats)
        elif kind == "body_joints_pose":
            self.body.update_joints(floats, with_rotations=True)
        elif kind == "body_joints":
            self.body.update_joints(floats, with_rotations=False)

    def _run_udp(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.host, self.port))
        sock.settimeout(0.5)
        logging.info("UDP listening on %s:%d", self.host, self.port)
        try:
            while not self._stop.is_set():
                try:
                    data, _addr = sock.recvfrom(65536)
                except socket.timeout:
                    continue
                except OSError:
                    break
                try:
                    message = data.decode("utf-8")
                except UnicodeDecodeError:
                    continue
                for line in message.splitlines():
                    if line:
                        self._handle_line(line)
        finally:
            sock.close()

    def _handle_tcp_conn(self, conn: socket.socket, addr) -> None:
        with conn:
            logging.info("Accepted connection from %s", addr)
            conn.settimeout(0.5)
            buffer = ""
            while not self._stop.is_set():
                try:
                    data = conn.recv(4096)
                except socket.timeout:
                    continue
                if not data:
                    break
                try:
                    buffer += data.decode("utf-8")
                except UnicodeDecodeError:
                    continue
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    if line:
                        self._handle_line(line)

    def _run_tcp(self) -> None:
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind((self.host, self.port))
        server_sock.listen(16)
        server_sock.settimeout(0.5)
        logging.info("TCP server listening on %s:%d", self.host, self.port)
        try:
            while not self._stop.is_set():
                try:
                    conn, addr = server_sock.accept()
                except socket.timeout:
                    continue
                thread = threading.Thread(
                    target=self._handle_tcp_conn, args=(conn, addr), daemon=True
                )
                self._conn_threads.append(thread)
                thread.start()
        finally:
            server_sock.close()

    def _run(self) -> None:
        if self.protocol == "udp":
            self._run_udp()
        else:
            self._run_tcp()


def _set_axes_from_bounds(ax: plt.Axes, center: np.ndarray, limit: float) -> None:
    """Set axes limits and aspect from center and half-extent."""
    ax.set_xlim(center[0] - limit, center[0] + limit)
    ax.set_ylim(center[1] - limit, center[1] + limit)
    ax.set_zlim(center[2] - limit, center[2] + limit)
    try:
        ax.set_box_aspect([1.0, 1.0, 1.0])
    except Exception:
        pass


def _finger_segments(
    wrist: np.ndarray, landmarks: np.ndarray
) -> Tuple[Tuple[np.ndarray, np.ndarray], ...]:
    """Return line segments for finger trees based on streamed indices."""
    if landmarks.shape[0] >= 21:
        # Landmarks include wrist at index 0 (as seen in sample data).
        idx = landmarks
        thumb = (1, 2, 3, 4)
        index = (5, 6, 7, 8)
        middle = (9, 10, 11, 12)
        ring = (13, 14, 15, 16)
        little = (17, 18, 19, 20)
    else:
        # Landmarks exclude wrist; indices start at ThumbMetacarpal.
        idx = landmarks
        thumb = (0, 1, 2, 3)
        index = (4, 5, 6, 7)
        middle = (8, 9, 10, 11)
        ring = (12, 13, 14, 15)
        little = (16, 17, 18, 19)
    segments = []
    # Thumb: wrist -> 1 -> 2 -> 3 -> 4
    segments.append((wrist, idx[thumb[0]]))
    segments.append((idx[thumb[0]], idx[thumb[1]]))
    segments.append((idx[thumb[1]], idx[thumb[2]]))
    segments.append((idx[thumb[2]], idx[thumb[3]]))
    # Index: wrist -> 5 -> 6 -> 7 -> 8
    segments.append((wrist, idx[index[0]]))
    segments.append((idx[index[0]], idx[index[1]]))
    segments.append((idx[index[1]], idx[index[2]]))
    segments.append((idx[index[2]], idx[index[3]]))
    # Middle: wrist -> 9 -> 10 -> 11 -> 12
    segments.append((wrist, idx[middle[0]]))
    segments.append((idx[middle[0]], idx[middle[1]]))
    segments.append((idx[middle[1]], idx[middle[2]]))
    segments.append((idx[middle[2]], idx[middle[3]]))
    # Ring: wrist -> 13 -> 14 -> 15 -> 16
    segments.append((wrist, idx[ring[0]]))
    segments.append((idx[ring[0]], idx[ring[1]]))
    segments.append((idx[ring[1]], idx[ring[2]]))
    segments.append((idx[ring[2]], idx[ring[3]]))
    # Little: wrist -> 17 -> 18 -> 19 -> 20
    segments.append((wrist, idx[little[0]]))
    segments.append((idx[little[0]], idx[little[1]]))
    segments.append((idx[little[1]], idx[little[2]]))
    segments.append((idx[little[2]], idx[little[3]]))
    return tuple(segments)


def _init_finger_lines(ax: plt.Axes, color: str) -> list:
    lines = []
    for _ in range(20):
        (line,) = ax.plot([], [], [], color=color, linewidth=2)
        lines.append(line)
    return lines


def _update_finger_lines(lines: list, segments) -> None:
    for line, (start, end) in zip(lines, segments):
        line.set_data([start[0], end[0]], [start[1], end[1]])
        line.set_3d_properties([start[2], end[2]])


_BODY_SEGMENTS = (
    ((1, 2), "#444444", 2.5),  # hips -> spine lower
    ((2, 3), "#444444", 2.5),
    ((3, 4), "#444444", 2.5),
    ((4, 5), "#444444", 2.5),
    ((5, 6), "#444444", 2.5),  # chest -> neck
    ((6, 7), "#D62728", 2.2),  # neck -> head
    ((6, 8), "#4C78A8", 2.3),
    ((8, 9), "#4C78A8", 2.3),
    ((9, 10), "#4C78A8", 2.8),
    ((10, 11), "#4C78A8", 3.0),
    ((11, 12), "#4C78A8", 2.8),
    ((12, 19), "#4C78A8", 2.8),  # forearm twist -> wrist
    ((19, 18), "#4C78A8", 3.2),  # wrist -> palm
    ((6, 13), "#E45756", 2.3),
    ((13, 14), "#E45756", 2.3),
    ((14, 15), "#E45756", 2.8),
    ((15, 16), "#E45756", 3.0),
    ((16, 17), "#E45756", 2.8),
    ((17, 45), "#E45756", 2.8),  # forearm twist -> wrist
    ((45, 44), "#E45756", 3.2),  # wrist -> palm
    ((8, 13), "#444444", 1.8),  # shoulder line
)

_BODY_HAND_FINGER_JOINTS = set(range(20, 44)) | set(range(46, 70))
_BODY_HIDDEN_JOINTS = {0} | _BODY_HAND_FINGER_JOINTS
_BODY_HEAD_INDEX = 7


def _init_body_lines(ax: plt.Axes) -> list:
    lines = []
    for _, color, linewidth in _BODY_SEGMENTS:
        (line,) = ax.plot([], [], [], color=color, linewidth=linewidth, alpha=0.85)
        lines.append(line)
    return lines


def _update_body_lines(lines: list, points: np.ndarray) -> None:
    for line, ((start_idx, end_idx), _, _) in zip(lines, _BODY_SEGMENTS):
        if points.shape[0] <= max(start_idx, end_idx):
            line.set_data([], [])
            line.set_3d_properties([])
            continue
        start = points[start_idx]
        end = points[end_idx]
        line.set_data([start[0], end[0]], [start[1], end[1]])
        line.set_3d_properties([start[2], end[2]])


def _init_head_axes(ax: plt.Axes) -> list:
    colors = ("#D62728", "#2CA02C", "#1F77B4")
    lines = []
    for color in colors:
        (line,) = ax.plot([], [], [], color=color, linewidth=3)
        lines.append(line)
    return lines


def _update_head_axes(lines: list, head_point: np.ndarray, head_quat: np.ndarray) -> None:
    axis_len = 0.22
    axes = np.eye(3, dtype=float) * axis_len
    rotated_axes = _quat_rotate(axes, head_quat)
    for line, axis in zip(lines, rotated_axes):
        end = head_point + axis
        line.set_data([head_point[0], end[0]], [head_point[1], end[1]])
        line.set_3d_properties([head_point[2], end[2]])


def _init_head_shape(ax: plt.Axes) -> list:
    lines = []
    for _ in range(12):
        (line,) = ax.plot([], [], [], color="#D62728", linewidth=2.5, alpha=0.9)
        lines.append(line)
    return lines


def _update_head_shape(lines: list, head_point: np.ndarray, head_quat: np.ndarray) -> None:
    half_size = 0.07
    vertical_gap = 0.035
    center_offset = np.array([0.0, 0.0, half_size + vertical_gap], dtype=float)
    corners = np.array(
        [
            [-half_size, -half_size, -half_size],
            [half_size, -half_size, -half_size],
            [half_size, half_size, -half_size],
            [-half_size, half_size, -half_size],
            [-half_size, -half_size, half_size],
            [half_size, -half_size, half_size],
            [half_size, half_size, half_size],
            [-half_size, half_size, half_size],
        ],
        dtype=float,
    )
    edges = (
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 4),
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
    )
    rotated_corners = _quat_rotate(corners + center_offset, head_quat) + head_point
    for line, (start_idx, end_idx) in zip(lines, edges):
        start = rotated_corners[start_idx]
        end = rotated_corners[end_idx]
        line.set_data([start[0], end[0]], [start[1], end[1]])
        line.set_3d_properties([start[2], end[2]])


def run_visualizer(
    protocol: str,
    host: str,
    port: int,
    show_left: bool,
    show_right: bool,
    axis_limit: float,
    alpha: float,
    show_fingers: bool,
) -> None:
    """Run the matplotlib visualizer."""
    receiver = StreamReceiver(protocol=protocol, host=host, port=port)
    receiver.start()

    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlabel("")
    ax.set_ylabel("")
    ax.set_zlabel("")
    ax.grid(False)
    try:
        ax.view_init(elev=10, azim=-170, roll=0)
    except TypeError:
        ax.view_init(elev=10, azim=-170)

    right_scatter = ax.scatter([], [], [], c="#E45756", s=14, alpha=0.35)
    left_scatter = ax.scatter([], [], [], c="#4C78A8", s=14, alpha=0.35)
    right_controller = ax.scatter([], [], [], c="#8C1D18", s=70, marker="s", label="R controller")
    left_controller = ax.scatter([], [], [], c="#1E3F66", s=70, marker="s", label="L controller")
    body_head_scatter = ax.scatter([], [], [], c="#D62728", s=36, marker="o", label="Head")
    right_wrist = ax.scatter([], [], [], c="#B2333C", s=52, marker="x", label="R hand")
    left_wrist = ax.scatter([], [], [], c="#2D5E8D", s=52, marker="x", label="L hand")
    ax.scatter([0.0], [0.0], [0.0], c="#666666", s=24, marker="+")
    ax.legend(loc="upper right", frameon=False, fontsize=8)

    right_lines = _init_finger_lines(ax, color="#FFE692") if show_fingers else []
    left_lines = _init_finger_lines(ax, color="#94FFDF") if show_fingers else []
    body_lines = _init_body_lines(ax)
    head_axes = _init_head_axes(ax)
    head_shape = _init_head_shape(ax)

    plt.show(block=False)

    cached_right_points = None
    cached_left_points = None
    cached_right_controller_point = None
    cached_left_controller_point = None
    cached_body_points = None
    ema_center = None
    ema_limit = axis_limit

    try:
        while plt.fignum_exists(fig.number):
            if show_right:
                right_points = receiver.hands["right"].world_points()
                if right_points is not None:
                    if show_fingers:
                        right_scatter._offsets3d = (
                            right_points[:, 0],
                            right_points[:, 1],
                            right_points[:, 2],
                        )
                    cached_right_points = right_points
                right_wrist_point = receiver.hands["right"].wrist_point()
                if right_wrist_point is not None:
                    right_wrist._offsets3d = (
                        [right_wrist_point[0]],
                        [right_wrist_point[1]],
                        [right_wrist_point[2]],
                    )
                if (
                    show_fingers
                    and right_points is not None
                    and right_wrist_point is not None
                ):
                    segments = _finger_segments(right_wrist_point, right_points)
                    _update_finger_lines(right_lines, segments)

                right_controller_point = receiver.controllers["right"].wrist_point()
                if right_controller_point is not None:
                    right_controller._offsets3d = (
                        [right_controller_point[0]],
                        [right_controller_point[1]],
                        [right_controller_point[2]],
                    )
                    cached_right_controller_point = right_controller_point

            if show_left:
                left_points = receiver.hands["left"].world_points()
                if left_points is not None:
                    if show_fingers:
                        left_scatter._offsets3d = (
                            left_points[:, 0],
                            left_points[:, 1],
                            left_points[:, 2],
                        )
                    cached_left_points = left_points
                left_wrist_point = receiver.hands["left"].wrist_point()
                if left_wrist_point is not None:
                    left_wrist._offsets3d = (
                        [left_wrist_point[0]],
                        [left_wrist_point[1]],
                        [left_wrist_point[2]],
                    )
                if (
                    show_fingers
                    and left_points is not None
                    and left_wrist_point is not None
                ):
                    segments = _finger_segments(left_wrist_point, left_points)
                    _update_finger_lines(left_lines, segments)

                left_controller_point = receiver.controllers["left"].wrist_point()
                if left_controller_point is not None:
                    left_controller._offsets3d = (
                        [left_controller_point[0]],
                        [left_controller_point[1]],
                        [left_controller_point[2]],
                    )
                    cached_left_controller_point = left_controller_point

            body_points = receiver.body.joints_world
            if body_points is not None:
                if body_points.shape[0] > _BODY_HEAD_INDEX:
                    head_point = body_points[_BODY_HEAD_INDEX]
                    body_head_scatter._offsets3d = (
                        [head_point[0]],
                        [head_point[1]],
                        [head_point[2]],
                    )
                    if (
                        receiver.body.joint_quats is not None
                        and receiver.body.joint_quats.shape[0] > _BODY_HEAD_INDEX
                    ):
                        head_quat = receiver.body.joint_quats[_BODY_HEAD_INDEX]
                        _update_head_axes(
                            head_axes,
                            head_point,
                            head_quat,
                        )
                        _update_head_shape(head_shape, head_point, head_quat)
                cached_body_points = body_points
                _update_body_lines(body_lines, body_points)

            points_for_bounds = []
            if cached_right_points is not None:
                points_for_bounds.append(cached_right_points)
            if cached_left_points is not None:
                points_for_bounds.append(cached_left_points)
            if cached_right_controller_point is not None:
                points_for_bounds.append(cached_right_controller_point.reshape((1, 3)))
            if cached_left_controller_point is not None:
                points_for_bounds.append(cached_left_controller_point.reshape((1, 3)))
            if cached_body_points is not None:
                points_for_bounds.append(cached_body_points)

            if points_for_bounds:
                all_points = np.vstack(points_for_bounds)
                mins = all_points.min(axis=0)
                maxs = all_points.max(axis=0)
                center = (mins + maxs) * 0.5
                extent = (maxs - mins).max() * 0.5
                padding = max(extent * 0.3, 0.02)
                target_limit = max(extent + padding, 0.05)
                if ema_center is None:
                    ema_center = center
                    ema_limit = target_limit
                else:
                    ema_center = (1.0 - alpha) * ema_center + alpha * center
                    ema_limit = (1.0 - alpha) * ema_limit + alpha * target_limit
                _set_axes_from_bounds(ax, ema_center, float(ema_limit))

            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            plt.pause(0.001)
    finally:
        receiver.stop()


def main() -> None:
    """Parse CLI args and start the visualizer."""
    parser = argparse.ArgumentParser(
        prog="visualizer",
        description="Matplotlib visualizer for HTS streams (stable scaling).",
    )
    parser.add_argument(
        "--protocol",
        choices=("udp", "tcp"),
        default="udp",
        help="Transport protocol to listen on (default: udp).",
    )
    parser.add_argument(
        "--host",
        default="0.0.0.0",
        help="Host/IP to bind to (default: 0.0.0.0 for UDP).",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=9000,
        help="Port to listen on (default: 9000 for UDP).",
    )
    parser.add_argument(
        "--left-only",
        action="store_true",
        help="Only visualize the left hand.",
    )
    parser.add_argument(
        "--right-only",
        action="store_true",
        help="Only visualize the right hand.",
    )
    parser.add_argument(
        "--axis-limit",
        type=float,
        default=0.4,
        help="Initial axis limit for X/Y/Z (default: 0.4).",
    )
    parser.add_argument(
        "--alpha",
        type=float,
        default=0.1,
        help="EMA smoothing factor for dynamic scaling (default: 0.1).",
    )
    parser.add_argument(
        "--show-fingers",
        action="store_true",
        help="Render finger bone lines.",
    )
    args = parser.parse_args()

    if args.left_only and args.right_only:
        raise SystemExit("Choose only one of --left-only or --right-only.")

    show_left = not args.right_only
    show_right = not args.left_only

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )

    run_visualizer(
        protocol=args.protocol,
        host=args.host,
        port=args.port,
        show_left=show_left,
        show_right=show_right,
        axis_limit=args.axis_limit,
        alpha=args.alpha,
        show_fingers=args.show_fingers,
    )


if __name__ == "__main__":
    main()
