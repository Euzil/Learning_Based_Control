"""Replay recorded Unity/Quest upper-body NPZ files for visual inspection.

The input file should be produced by record_body_stream.py or
record_visualize_body_stream.py and contain:

    timestamps[N]
    joint_poses[N, 70, 7]

Examples:
    python scripts/replay_body_recording.py recordings/body_motion_20260812_120000.npz
    python scripts/replay_body_recording.py recordings/test_01.npz --speed 0.5 --loop
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

try:
    import matplotlib.pyplot as plt
    import numpy as np
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "Missing dependency. Please install numpy and matplotlib."
    ) from exc

from visualizer import (
    _BODY_HEAD_INDEX,
    _BODY_HIDDEN_JOINTS,
    _convert_quat,
    _convert_vec,
    _init_body_lines,
    _init_head_axes,
    _init_head_shape,
    _set_axes_from_bounds,
    _update_body_lines,
    _update_head_axes,
    _update_head_shape,
)


def _load_recording(path: Path) -> tuple[np.ndarray, np.ndarray]:
    if not path.exists():
        raise SystemExit(f"Recording not found: {path}")

    data = np.load(path, allow_pickle=False)
    if "joint_poses" not in data:
        raise SystemExit(f"{path} does not contain 'joint_poses'.")

    joint_poses = np.asarray(data["joint_poses"], dtype=float)
    if joint_poses.ndim != 3 or joint_poses.shape[2] < 7:
        raise SystemExit(
            "Expected joint_poses with shape [frames, joints, 7]. "
            f"Got {joint_poses.shape}."
        )
    if joint_poses.shape[0] == 0:
        raise SystemExit("Recording contains no frames.")

    if "timestamps" in data:
        timestamps = np.asarray(data["timestamps"], dtype=float)
    else:
        timestamps = np.arange(joint_poses.shape[0], dtype=float) / 30.0

    if timestamps.shape[0] != joint_poses.shape[0]:
        timestamps = np.arange(joint_poses.shape[0], dtype=float) / 30.0

    return timestamps, joint_poses


def _convert_joint_frame(joint_frame: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    points = np.asarray([_convert_vec(vec) for vec in joint_frame[:, :3]], dtype=float)
    quats = np.asarray([_convert_quat(quat) for quat in joint_frame[:, 3:7]], dtype=float)
    return points, quats


def _compute_bounds(all_points: np.ndarray, axis_limit: float) -> tuple[np.ndarray, float]:
    mins = all_points.min(axis=0)
    maxs = all_points.max(axis=0)
    center = (mins + maxs) * 0.5
    extent = (maxs - mins).max() * 0.5
    padding = max(extent * 0.3, 0.02)
    limit = max(extent + padding, axis_limit)
    return center, float(limit)


def replay_recording(
    path: Path,
    speed: float,
    loop: bool,
    axis_limit: float,
    frame_step: int,
) -> None:
    timestamps, joint_poses = _load_recording(path)
    frame_step = max(frame_step, 1)
    speed = max(speed, 1e-6)

    converted_points = []
    converted_quats = []
    for frame in joint_poses:
        points, quats = _convert_joint_frame(frame)
        converted_points.append(points)
        converted_quats.append(quats)
    body_points = np.asarray(converted_points, dtype=float)
    body_quats = np.asarray(converted_quats, dtype=float)

    visible_points = np.delete(
        body_points.reshape((-1, 3)),
        [],
        axis=0,
    )
    center, limit = _compute_bounds(visible_points, axis_limit)

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

    body_scatter = ax.scatter([], [], [], c="#222222", s=10, label="Body")
    head_scatter = ax.scatter([], [], [], c="#D62728", s=30, marker="o", label="Head")
    ax.scatter([0.0], [0.0], [0.0], c="#222222", s=40, marker="o")
    ax.legend(loc="upper right")

    body_lines = _init_body_lines(ax)
    head_axes = _init_head_axes(ax)
    head_shape = _init_head_shape(ax)
    _set_axes_from_bounds(ax, center, limit)
    plt.show(block=False)

    frame_count = body_points.shape[0]
    frame_index = 0
    previous_timestamp = timestamps[0]

    while plt.fignum_exists(fig.number):
        points = body_points[frame_index]
        quats = body_quats[frame_index]
        visible_frame_points = np.delete(
            points,
            [idx for idx in _BODY_HIDDEN_JOINTS if idx < points.shape[0]],
            axis=0,
        )

        body_scatter._offsets3d = (
            visible_frame_points[:, 0],
            visible_frame_points[:, 1],
            visible_frame_points[:, 2],
        )
        _update_body_lines(body_lines, points)

        if points.shape[0] > _BODY_HEAD_INDEX:
            head_point = points[_BODY_HEAD_INDEX]
            head_quat = quats[_BODY_HEAD_INDEX]
            head_scatter._offsets3d = (
                [head_point[0]],
                [head_point[1]],
                [head_point[2]],
            )
            _update_head_axes(head_axes, head_point, head_quat)
            _update_head_shape(head_shape, head_point, head_quat)

        ax.set_title(
            f"{path.name} | frame {frame_index + 1}/{frame_count} | "
            f"t={timestamps[frame_index]:.3f}s"
        )
        fig.canvas.draw_idle()
        fig.canvas.flush_events()
        plt.pause(0.001)

        next_index = frame_index + frame_step
        if next_index >= frame_count:
            if not loop:
                break
            next_index = 0

        if next_index > frame_index:
            dt = timestamps[next_index] - previous_timestamp
        else:
            dt = 1.0 / 30.0
        previous_timestamp = timestamps[next_index]
        time.sleep(max(dt / speed, 0.0))
        frame_index = next_index

    plt.ioff()
    plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(
        prog="replay_body_recording",
        description="Replay recorded BodyLandmarkStreamer NPZ files.",
    )
    parser.add_argument("recording", help="Path to a .npz recording.")
    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="Playback speed multiplier (default: 1.0).",
    )
    parser.add_argument(
        "--loop",
        action="store_true",
        help="Loop playback until the figure is closed.",
    )
    parser.add_argument(
        "--axis-limit",
        type=float,
        default=0.4,
        help="Minimum axis half-extent (default: 0.4).",
    )
    parser.add_argument(
        "--frame-step",
        type=int,
        default=1,
        help="Advance by this many frames per update (default: 1).",
    )
    args = parser.parse_args()

    replay_recording(
        path=Path(args.recording).expanduser(),
        speed=args.speed,
        loop=args.loop,
        axis_limit=args.axis_limit,
        frame_step=args.frame_step,
    )


if __name__ == "__main__":
    main()
