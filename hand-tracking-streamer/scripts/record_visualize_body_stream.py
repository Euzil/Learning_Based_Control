"""Record and visualize Unity/Quest full-body streams at the same time.

This combines the live Matplotlib view from visualizer.py with the NPZ recorder
from record_body_stream.py.

Examples:
    python scripts/record_visualize_body_stream.py --protocol tcp --host localhost --port 8000
    python scripts/record_visualize_body_stream.py --protocol tcp --duration 10 -o recordings/test.npz
"""

from __future__ import annotations

import argparse
import logging
import time
from pathlib import Path
from typing import Optional

try:
    import matplotlib.pyplot as plt
    import numpy as np
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "Missing dependency. Please install numpy and matplotlib."
    ) from exc

from record_body_stream import (
    BODY_JOINT_COUNT,
    BodyRecorder,
    _default_host,
    _default_port,
    _timestamped_output_path,
    parse_body_line,
)
from visualizer import (
    StreamReceiver,
    _BODY_HEAD_INDEX,
    _BODY_HIDDEN_JOINTS,
    _foot_segments,
    _finger_segments,
    _init_body_lines,
    _init_finger_lines,
    _init_foot_lines,
    _init_head_axes,
    _init_head_shape,
    _set_axes_from_bounds,
    _update_body_lines,
    _update_finger_lines,
    _update_head_axes,
    _update_head_shape,
)


class RecordingStreamReceiver(StreamReceiver):
    """Stream receiver that also records raw body frames."""

    def __init__(
        self,
        protocol: str,
        host: str,
        port: int,
        recorder: BodyRecorder,
    ) -> None:
        super().__init__(protocol=protocol, host=host, port=port)
        self.recorder = recorder

    def _handle_line(self, line: str) -> None:
        parsed = parse_body_line(line)
        if parsed is not None:
            kind, values = parsed
            self.recorder.update(kind, values)
        super()._handle_line(line)


def run_recording_visualizer(
    protocol: str,
    host: str,
    port: int,
    output_path: Path,
    duration: Optional[float],
    expected_joints: int,
    show_left: bool,
    show_right: bool,
    axis_limit: float,
    alpha: float,
    show_fingers: bool,
) -> None:
    """Run live visualization and save recorded body frames on exit."""
    recorder = BodyRecorder(expected_joints=expected_joints)
    receiver = RecordingStreamReceiver(
        protocol=protocol,
        host=host,
        port=port,
        recorder=recorder,
    )
    receiver.start()

    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    try:
        ax.view_init(elev=10, azim=-170, roll=0)
    except TypeError:
        ax.view_init(elev=10, azim=-170)

    right_scatter = ax.scatter([], [], [], c="#E45756", s=20, label="Right")
    left_scatter = ax.scatter([], [], [], c="#4C78A8", s=20, label="Left")
    right_foot_scatter = ax.scatter([], [], [], c="#F58518", s=28, label="Right foot")
    left_foot_scatter = ax.scatter([], [], [], c="#54A24B", s=28, label="Left foot")
    body_scatter = ax.scatter([], [], [], c="#222222", s=10, label="Body")
    body_head_scatter = ax.scatter([], [], [], c="#D62728", s=30, marker="o", label="Head")
    right_wrist = ax.scatter([], [], [], c="#B2333C", s=60, marker="x")
    left_wrist = ax.scatter([], [], [], c="#2D5E8D", s=60, marker="x")
    right_ankle = ax.scatter([], [], [], c="#B86100", s=70, marker="^")
    left_ankle = ax.scatter([], [], [], c="#2E7D32", s=70, marker="^")
    ax.scatter([0.0], [0.0], [0.0], c="#222222", s=40, marker="o")
    ax.legend(loc="upper right")

    right_lines = _init_finger_lines(ax, color="#FFE692") if show_fingers else []
    left_lines = _init_finger_lines(ax, color="#94FFDF") if show_fingers else []
    right_foot_lines = _init_foot_lines(ax, color="#F58518")
    left_foot_lines = _init_foot_lines(ax, color="#54A24B")
    body_lines = _init_body_lines(ax)
    head_axes = _init_head_axes(ax)
    head_shape = _init_head_shape(ax)

    plt.show(block=False)

    cached_right_points = None
    cached_left_points = None
    cached_right_foot_points = None
    cached_left_foot_points = None
    cached_body_points = None
    ema_center = None
    ema_limit = axis_limit
    start_time = time.monotonic()
    last_logged_frame_count = 0
    next_report = start_time + 1.0

    logging.info("Recording to %s", output_path)
    if protocol == "tcp":
        logging.info("If using USB, run: adb reverse tcp:%d tcp:%d", port, port)

    try:
        while plt.fignum_exists(fig.number):
            now = time.monotonic()
            if duration is not None and now - start_time >= duration:
                break

            if show_right:
                right_points = receiver.hands["right"].world_points()
                if right_points is not None:
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
                    _update_finger_lines(
                        right_lines,
                        _finger_segments(right_wrist_point, right_points),
                    )

                right_foot_points = receiver.feet["right"].world_points()
                if right_foot_points is not None:
                    right_foot_scatter._offsets3d = (
                        right_foot_points[:, 0],
                        right_foot_points[:, 1],
                        right_foot_points[:, 2],
                    )
                    cached_right_foot_points = right_foot_points
                    _update_finger_lines(right_foot_lines, _foot_segments(right_foot_points))
                right_ankle_point = receiver.feet["right"].wrist_point()
                if right_ankle_point is not None:
                    right_ankle._offsets3d = (
                        [right_ankle_point[0]],
                        [right_ankle_point[1]],
                        [right_ankle_point[2]],
                    )

            if show_left:
                left_points = receiver.hands["left"].world_points()
                if left_points is not None:
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
                    _update_finger_lines(
                        left_lines,
                        _finger_segments(left_wrist_point, left_points),
                    )

                left_foot_points = receiver.feet["left"].world_points()
                if left_foot_points is not None:
                    left_foot_scatter._offsets3d = (
                        left_foot_points[:, 0],
                        left_foot_points[:, 1],
                        left_foot_points[:, 2],
                    )
                    cached_left_foot_points = left_foot_points
                    _update_finger_lines(left_foot_lines, _foot_segments(left_foot_points))
                left_ankle_point = receiver.feet["left"].wrist_point()
                if left_ankle_point is not None:
                    left_ankle._offsets3d = (
                        [left_ankle_point[0]],
                        [left_ankle_point[1]],
                        [left_ankle_point[2]],
                    )

            body_points = receiver.body.joints_world
            if body_points is not None:
                visible_body_points = np.delete(
                    body_points,
                    [idx for idx in _BODY_HIDDEN_JOINTS if idx < body_points.shape[0]],
                    axis=0,
                )
                body_scatter._offsets3d = (
                    visible_body_points[:, 0],
                    visible_body_points[:, 1],
                    visible_body_points[:, 2],
                )
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
                        _update_head_axes(head_axes, head_point, head_quat)
                        _update_head_shape(head_shape, head_point, head_quat)
                cached_body_points = body_points
                _update_body_lines(body_lines, body_points)

            points_for_bounds = []
            if cached_right_points is not None:
                points_for_bounds.append(cached_right_points)
            if cached_left_points is not None:
                points_for_bounds.append(cached_left_points)
            if cached_right_foot_points is not None:
                points_for_bounds.append(cached_right_foot_points)
            if cached_left_foot_points is not None:
                points_for_bounds.append(cached_left_foot_points)
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

            if now >= next_report:
                new_frames = recorder.frame_count - last_logged_frame_count
                last_logged_frame_count = recorder.frame_count
                logging.info(
                    "recorded frames: %d total, %d in last second",
                    recorder.frame_count,
                    new_frames,
                )
                next_report += 1.0

            ax.set_title(f"Recording frames: {recorder.frame_count}")
            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            plt.pause(0.001)
    finally:
        receiver.stop()
        if recorder.frame_count == 0:
            logging.warning("No complete body frames recorded; output file was not written.")
        else:
            metadata = {
                "protocol": protocol,
                "host": host,
                "port": port,
                "expected_joints": expected_joints,
                "created_unix_time": time.time(),
                "format": "timestamps[N], root_poses[N,7], joint_poses[N,84,7]",
                "visualized_live": True,
            }
            recorder.save(output_path, metadata)
            logging.info("Saved %d frames to %s", recorder.frame_count, output_path)


def main() -> None:
    parser = argparse.ArgumentParser(
        prog="record_visualize_body_stream",
        description="Record Unity/Quest body data while showing the live Matplotlib view.",
    )
    parser.add_argument(
        "--protocol",
        choices=("udp", "tcp"),
        default="tcp",
        help="Transport protocol to listen on (default: tcp).",
    )
    parser.add_argument(
        "--host",
        default=None,
        help="Host/IP to bind to (default: localhost for TCP, 0.0.0.0 for UDP).",
    )
    parser.add_argument(
        "-p",
        "--port",
        type=int,
        default=None,
        help="Port to listen on (default: 8000 for TCP, 9000 for UDP).",
    )
    parser.add_argument(
        "-o",
        "--output",
        default=None,
        help="Output .npz path (default: recordings/body_motion_YYYYmmdd_HHMMSS.npz).",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Recording duration in seconds. If omitted, close the figure or press Ctrl+C.",
    )
    parser.add_argument(
        "--expected-joints",
        type=int,
        default=BODY_JOINT_COUNT,
        help="Expected full-body joint count (default: 84).",
    )
    parser.add_argument(
        "--left-only",
        action="store_true",
        help="Only visualize the left hand/foot streams.",
    )
    parser.add_argument(
        "--right-only",
        action="store_true",
        help="Only visualize the right hand/foot streams.",
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
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=("DEBUG", "INFO", "WARNING", "ERROR"),
        help="Logging verbosity (default: INFO).",
    )
    args = parser.parse_args()

    if args.left_only and args.right_only:
        raise SystemExit("Choose only one of --left-only or --right-only.")

    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)s %(message)s",
    )

    protocol = args.protocol
    host = args.host or _default_host(protocol)
    port = args.port or _default_port(protocol)

    run_recording_visualizer(
        protocol=protocol,
        host=host,
        port=port,
        output_path=_timestamped_output_path(args.output),
        duration=args.duration,
        expected_joints=args.expected_joints,
        show_left=not args.right_only,
        show_right=not args.left_only,
        axis_limit=args.axis_limit,
        alpha=args.alpha,
        show_fingers=args.show_fingers,
    )


if __name__ == "__main__":
    main()
