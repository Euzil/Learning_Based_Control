#!/usr/bin/env python3
"""Create HTS-to-xrobot orientation calibration for TWIST2 offline replay."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from hts_offline_xrobot_streamer import (
    G1_ROT_OFFSET_BY_HUMAN_BODY,
    HTSOfflineXRobotStreamer,
    XROBOT_BODY_TO_HTS84,
    quat_inv_wxyz,
    quat_mul_wxyz,
)


def aligned_mean_quat(quats: list[np.ndarray]) -> np.ndarray:
    if not quats:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    reference = np.asarray(quats[0], dtype=np.float64)
    accum = np.zeros(4, dtype=np.float64)
    for quat in quats:
        quat = np.asarray(quat, dtype=np.float64)
        if float(np.dot(reference, quat)) < 0.0:
            quat = -quat
        accum += quat
    norm = float(np.linalg.norm(accum))
    if norm < 1e-8:
        return reference
    return accum / norm


def stable_window(recording: Path, transform: str, window_frames: int) -> tuple[int, int]:
    data = np.load(recording, allow_pickle=False)
    poses = np.asarray(data["joint_poses"], dtype=np.float32)
    if poses.shape[0] <= window_frames:
        return 0, poses.shape[0] - 1

    if transform == "hts_rh":
        matrix = np.array([[0, 0, 1], [-1, 0, 0], [0, 1, 0]], dtype=np.float64)
    elif transform == "xrobot_compat":
        matrix = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=np.float64)
    else:
        raise ValueError(f"Unknown transform: {transform}")

    body_indices = list(XROBOT_BODY_TO_HTS84.values())
    positions = poses[:, body_indices, :3] @ matrix.T
    best_score = float("inf")
    best_start = 0
    for start in range(0, poses.shape[0] - window_frames + 1):
        segment = positions[start : start + window_frames]
        root = segment[:, 0:1]
        relative = segment - root
        relative_std = float(np.mean(np.std(relative, axis=0)))
        root_motion = float(np.linalg.norm(segment[-1, 0] - segment[0, 0]))
        score = relative_std + 0.25 * root_motion
        if score < best_score:
            best_score = score
            best_start = start
    return best_start, best_start + window_frames - 1


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate HTS orientation calibration JSON.")
    parser.add_argument("recording", type=Path)
    parser.add_argument("-o", "--output", type=Path, required=True)
    parser.add_argument("--transform", choices=["hts_rh", "xrobot_compat"], default="hts_rh")
    parser.add_argument("--target-height", type=float, default=1.8)
    parser.add_argument("--start-frame", type=int)
    parser.add_argument("--end-frame", type=int)
    parser.add_argument("--window-frames", type=int, default=60)
    args = parser.parse_args()

    recording = args.recording.expanduser()
    streamer = HTSOfflineXRobotStreamer(
        recording,
        coordinate_transform=args.transform,
        orientation_mode="source",
        target_height=args.target_height,
    )

    if args.start_frame is None or args.end_frame is None:
        start, end = stable_window(recording, args.transform, args.window_frames)
    else:
        start, end = args.start_frame, args.end_frame
    start = int(np.clip(start, 0, len(streamer.timestamps) - 1))
    end = int(np.clip(end, start, len(streamer.timestamps) - 1))

    corrections = {}
    for body_name in XROBOT_BODY_TO_HTS84:
        source_quats = []
        for frame_idx in range(start, end + 1):
            body, _left, _right, _ctrl, _head = streamer.get_frame_at_index(frame_idx)
            source_quats.append(np.asarray(body[body_name][1], dtype=np.float64))
        source_neutral = aligned_mean_quat(source_quats)
        target_neutral = quat_inv_wxyz(
            np.asarray(G1_ROT_OFFSET_BY_HUMAN_BODY.get(body_name, [1.0, 0.0, 0.0, 0.0]), dtype=np.float64)
        )
        correction = quat_mul_wxyz(quat_inv_wxyz(source_neutral), target_neutral)
        corrections[body_name] = correction.tolist()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "schema": "hts_xrobot_orientation_calibration_v1",
        "recording": str(recording),
        "transform": args.transform,
        "target_height": args.target_height,
        "start_frame": start,
        "end_frame": end,
        "start_time": float(streamer.timestamps[start]),
        "end_time": float(streamer.timestamps[end]),
        "body_orientation_corrections": corrections,
    }
    args.output.write_text(json.dumps(payload, indent=2))
    print(f"saved: {args.output}")
    print(f"window: frames {start}-{end}, time {payload['start_time']:.3f}-{payload['end_time']:.3f}s")


if __name__ == "__main__":
    main()
