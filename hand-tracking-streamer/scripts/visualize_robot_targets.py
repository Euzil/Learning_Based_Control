#!/usr/bin/env python3
"""Replay compact robot-target NPZ files produced by extract_robot_targets.py.

Expected input fields:
    timestamps[N]
    target_positions[N, K, 3]
    target_names[K]

Example:
    python scripts/visualize_robot_targets.py recordings/test_01_robot_targets_minimal_rh.npz
    python scripts/visualize_robot_targets.py recordings/test_01_robot_targets_ik_rh.npz --loop --labels
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
        "Missing dependency. Install it with: pip install numpy matplotlib"
    ) from exc


TARGET_COLORS = {
    "pelvis": "#222222",
    "chest": "#555555",
    "neck": "#555555",
    "head": "#D62728",
    "left_hand": "#1F77B4",
    "right_hand": "#FF7F0E",
    "left_foot": "#2CA02C",
    "right_foot": "#9467BD",
    "left_ankle": "#2CA02C",
    "right_ankle": "#9467BD",
    "left_toe": "#2CA02C",
    "right_toe": "#9467BD",
    "left_elbow": "#1F77B4",
    "right_elbow": "#FF7F0E",
    "left_knee": "#2CA02C",
    "right_knee": "#9467BD",
}


def load_targets(path: Path) -> tuple[np.ndarray, np.ndarray, list[str], str]:
    if not path.exists():
        raise SystemExit(f"File not found: {path}")

    data = np.load(path, allow_pickle=False)
    required = {"timestamps", "target_positions", "target_names"}
    missing = required.difference(data.files)
    if missing:
        raise SystemExit(f"{path} is missing required arrays: {sorted(missing)}")

    timestamps = np.asarray(data["timestamps"], dtype=float)
    positions = np.asarray(data["target_positions"], dtype=float)
    names = [str(name) for name in data["target_names"]]

    if positions.ndim != 3 or positions.shape[-1] != 3:
        raise SystemExit(f"Expected target_positions shape [N, K, 3], got {positions.shape}")
    if timestamps.shape[0] != positions.shape[0]:
        raise SystemExit(
            f"timestamps length {timestamps.shape[0]} does not match frames {positions.shape[0]}"
        )
    if len(names) != positions.shape[1]:
        raise SystemExit(f"target_names length {len(names)} does not match K={positions.shape[1]}")

    coordinate_frame = "unknown"
    if "coordinate_frame" in data.files:
        coordinate_frame = str(data["coordinate_frame"][0])

    return timestamps, positions, names, coordinate_frame


def compute_bounds(positions: np.ndarray, axis_limit: float) -> tuple[np.ndarray, float]:
    flat = positions.reshape((-1, 3))
    mins = np.nanmin(flat, axis=0)
    maxs = np.nanmax(flat, axis=0)
    center = (mins + maxs) * 0.5
    half_extent = float(np.max(maxs - mins) * 0.5)
    return center, max(axis_limit, half_extent * 1.2, 0.1)


def connection_pairs(names: list[str]) -> list[tuple[int, int]]:
    idx = {name: i for i, name in enumerate(names)}

    candidate_pairs = [
        ("pelvis", "chest"),
        ("chest", "neck"),
        ("neck", "head"),
        ("chest", "head"),
        ("chest", "left_shoulder"),
        ("left_shoulder", "left_upper_arm"),
        ("left_upper_arm", "left_lower_arm"),
        ("left_lower_arm", "left_hand"),
        ("chest", "left_elbow"),
        ("left_elbow", "left_hand"),
        ("chest", "left_hand"),
        ("chest", "right_shoulder"),
        ("right_shoulder", "right_upper_arm"),
        ("right_upper_arm", "right_lower_arm"),
        ("right_lower_arm", "right_hand"),
        ("chest", "right_elbow"),
        ("right_elbow", "right_hand"),
        ("chest", "right_hand"),
        ("pelvis", "left_upper_leg"),
        ("left_upper_leg", "left_lower_leg"),
        ("left_lower_leg", "left_ankle"),
        ("left_ankle", "left_toe"),
        ("pelvis", "left_knee"),
        ("left_knee", "left_ankle"),
        ("pelvis", "left_foot"),
        ("pelvis", "right_upper_leg"),
        ("right_upper_leg", "right_lower_leg"),
        ("right_lower_leg", "right_ankle"),
        ("right_ankle", "right_toe"),
        ("pelvis", "right_knee"),
        ("right_knee", "right_ankle"),
        ("pelvis", "right_foot"),
    ]

    pairs = []
    seen = set()
    for a, b in candidate_pairs:
        if a in idx and b in idx:
            pair = (idx[a], idx[b])
            if pair not in seen:
                pairs.append(pair)
                seen.add(pair)
    return pairs


def set_axes(ax, center: np.ndarray, limit: float) -> None:
    ax.set_xlim(center[0] - limit, center[0] + limit)
    ax.set_ylim(center[1] - limit, center[1] + limit)
    ax.set_zlim(center[2] - limit, center[2] + limit)
    try:
        ax.set_box_aspect([1.0, 1.0, 1.0])
    except Exception:
        pass


def replay(
    path: Path,
    speed: float,
    loop: bool,
    labels: bool,
    axis_limit: float,
    frame_step: int,
) -> None:
    timestamps, positions, names, coordinate_frame = load_targets(path)
    frame_step = max(1, frame_step)
    speed = max(speed, 1e-6)

    center, limit = compute_bounds(positions, axis_limit)
    pairs = connection_pairs(names)
    colors = [TARGET_COLORS.get(name, "#222222") for name in names]

    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    set_axes(ax, center, limit)

    first_pts = positions[0]
    scatter = ax.scatter(
        first_pts[:, 0],
        first_pts[:, 1],
        first_pts[:, 2],
        s=45,
        c=colors,
        depthshade=True,
    )
    lines = [ax.plot([], [], [], color="#222222", linewidth=2.0, alpha=0.75)[0] for _ in pairs]
    text_labels = [ax.text(0.0, 0.0, 0.0, name, fontsize=8) for name in names] if labels else []

    try:
        ax.view_init(elev=12, azim=-150, roll=0)
    except TypeError:
        ax.view_init(elev=12, azim=-150)

    plt.show(block=False)

    frame = 0
    previous_timestamp = timestamps[0] if timestamps.size else 0.0
    while plt.fignum_exists(fig.number):
        pts = positions[frame]
        scatter._offsets3d = (pts[:, 0], pts[:, 1], pts[:, 2])

        for line, (a, b) in zip(lines, pairs):
            pa = pts[a]
            pb = pts[b]
            line.set_data([pa[0], pb[0]], [pa[1], pb[1]])
            line.set_3d_properties([pa[2], pb[2]])

        for label, point in zip(text_labels, pts):
            label.set_position((point[0], point[1]))
            label.set_3d_properties(point[2])

        ax.set_title(
            f"{path.name} | frame {frame + 1}/{positions.shape[0]} | "
            f"t={timestamps[frame]:.3f}s | {coordinate_frame}"
        )
        fig.canvas.draw_idle()
        fig.canvas.flush_events()
        plt.pause(0.001)

        next_frame = frame + frame_step
        if next_frame >= positions.shape[0]:
            if not loop:
                break
            next_frame = 0

        if next_frame > frame:
            dt = timestamps[next_frame] - previous_timestamp
        else:
            dt = 1.0 / 30.0
        previous_timestamp = timestamps[next_frame]
        time.sleep(max(dt / speed, 0.0))
        frame = next_frame

    plt.ioff()
    plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Visualize compact robot-target NPZ files."
    )
    parser.add_argument("recording", type=Path, help="Path to *_robot_targets_*.npz")
    parser.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier.")
    parser.add_argument("--loop", action="store_true", help="Loop until the window is closed.")
    parser.add_argument("--labels", action="store_true", help="Show target names next to points.")
    parser.add_argument("--axis-limit", type=float, default=0.6, help="Minimum axis half-extent.")
    parser.add_argument("--frame-step", type=int, default=1, help="Advance by this many frames.")
    args = parser.parse_args()

    replay(
        path=args.recording.expanduser(),
        speed=args.speed,
        loop=args.loop,
        labels=args.labels,
        axis_limit=args.axis_limit,
        frame_step=args.frame_step,
    )


if __name__ == "__main__":
    main()
