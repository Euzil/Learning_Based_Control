#!/usr/bin/env python3
"""Inspect HTS 84-joint recordings through TWIST2/GMR without running policy."""

from __future__ import annotations

import argparse
import json
import tempfile
from pathlib import Path

import mujoco as mj
import numpy as np
import general_motion_retargeting.motion_retarget as motion_retarget
from general_motion_retargeting import GeneralMotionRetargeting as GMR

from hts_offline_xrobot_streamer import HTSOfflineXRobotStreamer


def position_distances(retarget: GMR) -> dict[str, float]:
    distances = {}
    mj.mj_forward(retarget.model, retarget.configuration.data)
    for frame_name, entry in retarget.ik_match_table2.items():
        human_body, pos_weight, _rot_weight, _pos_offset, _rot_offset = entry
        if pos_weight == 0 or human_body not in retarget.scaled_human_data:
            continue
        body_id = retarget.robot_body_names.get(frame_name)
        if body_id is None:
            continue
        robot_pos = retarget.configuration.data.xpos[body_id]
        target_pos = retarget.scaled_human_data[human_body][0]
        distances[frame_name] = float(np.linalg.norm(robot_pos - target_pos))
    return distances


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run HTS84 -> XRobot-format -> GMR retargeting and print qpos stats."
    )
    parser.add_argument("recording", type=Path)
    parser.add_argument("--robot", default="unitree_g1")
    parser.add_argument("--actual-human-height", type=float, default=1.6)
    parser.add_argument("--frame-step", type=int, default=10)
    parser.add_argument("--max-frames", type=int, default=80)
    parser.add_argument("--transform", choices=["hts_rh", "xrobot_compat"], default="hts_rh")
    parser.add_argument(
        "--orientation",
        choices=[
            "source",
            "identity",
            "raw_wxyz",
            "premultiply",
            "calibrated",
            "segments",
            "segments_x",
            "segments_y",
            "segments_z",
            "segments_robot",
            "segments_robot_x",
            "segments_robot_y",
            "segments_robot_z",
        ],
        default="source",
    )
    parser.add_argument("--target-height", type=float, default=0.0)
    parser.add_argument("--calibration", type=Path)
    parser.add_argument(
        "--position-only",
        action="store_true",
        help="Use a temporary xrobot IK config that replaces orientation tasks with position tasks.",
    )
    parser.add_argument("-o", "--output", type=Path)
    args = parser.parse_args()

    streamer = HTSOfflineXRobotStreamer(
        args.recording.expanduser(),
        coordinate_transform=args.transform,
        orientation_mode=args.orientation,
        target_height=args.target_height if args.target_height > 0.0 else None,
        calibration_path=args.calibration,
    )
    src_human = "xrobot_position_only" if args.position_only else "xrobot"
    temp_config = None
    if args.position_only:
        original_config = Path(motion_retarget.IK_CONFIG_DICT["xrobot"][args.robot])
        config = json.loads(original_config.read_text())
        for table_name in ("ik_match_table1", "ik_match_table2"):
            for entry in config[table_name].values():
                entry[1] = 10
                entry[2] = 0
        temp_config = tempfile.NamedTemporaryFile("w", suffix=".json", delete=False)
        json.dump(config, temp_config)
        temp_config.close()
        motion_retarget.IK_CONFIG_DICT[src_human] = {args.robot: Path(temp_config.name)}

    retarget = GMR(
        src_human=src_human,
        tgt_robot=args.robot,
        actual_human_height=args.actual_human_height,
        verbose=False,
    )

    frame_indices = np.arange(0, len(streamer.timestamps), max(1, args.frame_step), dtype=np.int32)
    if args.max_frames:
        frame_indices = frame_indices[: args.max_frames]

    qpos_seq = []
    errors1 = []
    errors2 = []
    pos_errors = []
    last_pos_dist = {}
    for out_i, idx in enumerate(frame_indices):
        body, _left, _right, _ctrl, _head = streamer.get_frame_at_index(int(idx))
        qpos = retarget.retarget(body, offset_to_ground=True)
        qpos_seq.append(qpos.copy())
        errors1.append(retarget.error1())
        errors2.append(retarget.error2())
        last_pos_dist = position_distances(retarget)
        pos_errors.append(np.mean(list(last_pos_dist.values())) if last_pos_dist else np.nan)
        if out_i % 10 == 0 or out_i == len(frame_indices) - 1:
            print(
                f"frame {out_i + 1:3d}/{len(frame_indices)} src={idx:4d} "
                f"root_z={qpos[2]:.3f} err1={errors1[-1]:.3f} "
                f"err2={errors2[-1]:.3f} pos_m={pos_errors[-1]:.3f}"
            )

    qpos_seq = np.asarray(qpos_seq, dtype=np.float32)
    dof = qpos_seq[:, 7:]
    print("\nSummary")
    print(f"recording: {args.recording}")
    print(f"transform: {args.transform}")
    print(f"orientation: {args.orientation}")
    print(f"position_scale: {streamer.position_scale:.3f}")
    print(f"calibration: {args.calibration}")
    print(f"position_only: {args.position_only}")
    print(f"frames inspected: {len(qpos_seq)} / {len(streamer.timestamps)}")
    print(f"qpos shape: {qpos_seq.shape}")
    print(f"root xyz min: {qpos_seq[:, :3].min(axis=0)}")
    print(f"root xyz max: {qpos_seq[:, :3].max(axis=0)}")
    print(f"dof min: {dof.min():.3f} max: {dof.max():.3f} absmax: {np.abs(dof).max():.3f}")
    print(f"err1 mean/max: {np.mean(errors1):.3f} / {np.max(errors1):.3f}")
    print(f"err2 mean/max: {np.mean(errors2):.3f} / {np.max(errors2):.3f}")
    print(f"pos_m mean/max: {np.nanmean(pos_errors):.3f} / {np.nanmax(pos_errors):.3f}")
    if last_pos_dist:
        worst = sorted(last_pos_dist.items(), key=lambda item: item[1], reverse=True)[:5]
        print("last frame worst pos distances:")
        for name, dist in worst:
            print(f"  {name}: {dist:.3f} m")

    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            args.output,
            timestamps=streamer.timestamps[frame_indices],
            frame_indices=frame_indices,
            qpos=qpos_seq,
            error1=np.asarray(errors1, dtype=np.float32),
            error2=np.asarray(errors2, dtype=np.float32),
            pos_error=np.asarray(pos_errors, dtype=np.float32),
            transform=np.asarray([args.transform]),
            orientation=np.asarray([args.orientation]),
        )
        print(f"saved: {args.output}")


if __name__ == "__main__":
    main()
