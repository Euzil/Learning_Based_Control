#!/usr/bin/env python3
"""GMR-like retargeting from 84-joint Quest/Unity recordings to G1 references.

This is a lightweight local substitute for TWIST2's external GMR package.  The
default behavior is deliberately conservative: keep the G1 root and legs in a
stable standing reference, and retarget upper-body motion as displacement around
the robot's neutral pose.  That removes the largest human-vs-robot morphology
mismatch while preserving gestures such as waving.

Output:
    qpos_ref[N, 36]
    dof_pos_ref[N, 29]
    mimic_obs[N, 35] compatible with TWIST2 deploy_real/server_low_level_g1_sim.py
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np
from scipy.optimize import least_squares

from convert_84_to_g1_reference import (
    DEFAULT_DOF_POS,
    DEFAULT_XML,
    build_mimic_obs,
    joint_bounds,
    load_raw_recording,
    lowpass,
    make_qpos,
)
from extract_robot_targets import convert_poses_unity_to_rh


G1_DEFAULT_ROOT_POS = np.array([0.0, 0.0, 0.793], dtype=np.float64)
SCRIPT_DIR = Path(__file__).resolve().parent
HAND_TRACKING_DIR = SCRIPT_DIR.parent
WORKSPACE_DIR = HAND_TRACKING_DIR.parent

SOURCE_JOINTS = {
    "pelvis": 1,
    "chest": 5,
    "head": 7,
    "left_shoulder": 8,
    "left_elbow": 11,
    "left_hand": 19,
    "right_shoulder": 13,
    "right_elbow": 16,
    "right_hand": 45,
    "left_knee": 71,
    "left_ankle": 73,
    "right_knee": 78,
    "right_ankle": 80,
}

IK_TARGET_TO_BODY = {
    "chest": "torso_link",
    "head": "head_mocap",
    "left_elbow": "left_elbow_link",
    "left_hand": "left_rubber_hand",
    "right_elbow": "right_elbow_link",
    "right_hand": "right_rubber_hand",
}

IK_WEIGHTS = {
    "chest": 0.35,
    "head": 0.25,
    "left_elbow": 1.0,
    "left_hand": 4.0,
    "right_elbow": 1.0,
    "right_hand": 4.0,
}

UPPER_DOF_INDICES = np.arange(12, 29, dtype=np.int32)
FULL_DOF_INDICES = np.arange(0, 29, dtype=np.int32)


def yaw_matrix(angle: float) -> np.ndarray:
    c = float(np.cos(angle))
    s = float(np.sin(angle))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def model_body_positions(model: mujoco.MjModel, qpos: np.ndarray, body_names: list[str]) -> dict[str, np.ndarray]:
    data = mujoco.MjData(model)
    data.qpos[:] = qpos
    mujoco.mj_forward(model, data)
    out = {}
    for name in body_names:
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if body_id < 0:
            raise SystemExit(f"G1 body not found in XML: {name}")
        out[name] = data.xpos[body_id].copy()
    return out


def load_source_positions(joint_poses: np.ndarray) -> tuple[np.ndarray, list[str], np.ndarray]:
    names = list(SOURCE_JOINTS)
    indices = np.asarray([SOURCE_JOINTS[name] for name in names], dtype=np.int32)
    poses_rh = convert_poses_unity_to_rh(joint_poses[:, indices, :])
    return poses_rh[..., :3].astype(np.float64), names, indices


def estimate_initial_yaw(source_positions: np.ndarray, names: list[str]) -> float:
    name_to_idx = {name: i for i, name in enumerate(names)}
    left = source_positions[0, name_to_idx["left_shoulder"]]
    right = source_positions[0, name_to_idx["right_shoulder"]]
    shoulder_axis = left - right
    shoulder_axis[2] = 0.0
    norm = np.linalg.norm(shoulder_axis[:2])
    if norm < 1e-6:
        return 0.0
    current = np.arctan2(shoulder_axis[1], shoulder_axis[0])
    return (np.pi / 2.0) - current


def build_motion_targets(
    model: mujoco.MjModel,
    source_positions: np.ndarray,
    names: list[str],
    motion_scale: float,
    torso_motion_scale: float,
    max_hand_displacement: float,
    smooth_alpha: float,
) -> tuple[np.ndarray, list[str], dict[str, str]]:
    """Transfer human motion deltas onto the G1 neutral target positions."""
    if smooth_alpha > 0.0:
        source_positions = lowpass(source_positions, smooth_alpha)

    name_to_idx = {name: i for i, name in enumerate(names)}
    pelvis = source_positions[:, name_to_idx["pelvis"]]
    pelvis0 = pelvis[0]
    rot = yaw_matrix(estimate_initial_yaw(source_positions, names))

    default_qpos = make_qpos(G1_DEFAULT_ROOT_POS, DEFAULT_DOF_POS.copy())
    body_names = list(IK_TARGET_TO_BODY.values())
    defaults = model_body_positions(model, default_qpos, body_names)

    target_names = list(IK_TARGET_TO_BODY)
    targets = np.zeros((source_positions.shape[0], len(target_names), 3), dtype=np.float64)

    for target_i, target_name in enumerate(target_names):
        src_i = name_to_idx[target_name]
        rel0 = source_positions[0, src_i] - pelvis0
        body_default = defaults[IK_TARGET_TO_BODY[target_name]]
        scale = torso_motion_scale if target_name in {"chest", "head"} else motion_scale

        for frame_i in range(source_positions.shape[0]):
            rel = source_positions[frame_i, src_i] - pelvis[frame_i]
            delta = rot @ (rel - rel0)
            if target_name.endswith("hand"):
                delta_norm = np.linalg.norm(delta)
                if delta_norm > max_hand_displacement:
                    delta *= max_hand_displacement / delta_norm
            targets[frame_i, target_i] = body_default + scale * delta

    return targets, target_names, IK_TARGET_TO_BODY.copy()


def resolve_ik_mapping(
    model: mujoco.MjModel,
    target_names: list[str],
    target_to_body: dict[str, str],
) -> list[tuple[str, int, int, float]]:
    mapping = []
    for i, target_name in enumerate(target_names):
        body_name = target_to_body[target_name]
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if body_id < 0:
            raise SystemExit(f"G1 body not found in XML: {body_name}")
        mapping.append((target_name, i, body_id, IK_WEIGHTS[target_name]))
    return mapping


def solve_retargeted_ik(
    model: mujoco.MjModel,
    timestamps: np.ndarray,
    targets: np.ndarray,
    target_names: list[str],
    target_to_body: dict[str, str],
    solve_set: str,
    frame_step: int,
    max_frames: int | None,
    reg_weight: float,
    temporal_weight: float,
    max_dof_offset: float,
) -> dict[str, np.ndarray]:
    frame_indices = np.arange(0, len(timestamps), max(1, frame_step), dtype=np.int32)
    if max_frames is not None:
        frame_indices = frame_indices[:max_frames]

    data = mujoco.MjData(model)
    lower, upper = joint_bounds(model)
    default_dof = DEFAULT_DOF_POS.astype(np.float64).copy()
    lower = np.maximum(lower, default_dof - max_dof_offset)
    upper = np.minimum(upper, default_dof + max_dof_offset)

    if solve_set == "upper":
        solve_ids = UPPER_DOF_INDICES
    elif solve_set == "full":
        solve_ids = FULL_DOF_INDICES
    else:
        raise SystemExit(f"Unknown solve set: {solve_set}")

    x_lower = lower[solve_ids]
    x_upper = upper[solve_ids]
    prev_dof = np.clip(default_dof, lower, upper)
    mapping = resolve_ik_mapping(model, target_names, target_to_body)

    qpos_ref = []
    dof_ref = []
    mean_errors = []
    max_errors = []

    print("GMR-like IK target mapping:")
    for target_name, _target_idx, body_id, weight in mapping:
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
        print(f"  {target_name:>12s} -> {body_name:<22s} weight={weight:g}")
    print(f"solving dofs: {solve_set} ({len(solve_ids)} of 29)")

    for out_i, frame_idx in enumerate(frame_indices):
        frame_targets = targets[frame_idx]

        def unpack(x: np.ndarray) -> np.ndarray:
            dof = prev_dof.copy()
            dof[solve_ids] = x
            return np.clip(dof, lower, upper)

        def residual(x: np.ndarray) -> np.ndarray:
            dof = unpack(x)
            data.qpos[:] = make_qpos(G1_DEFAULT_ROOT_POS, dof)
            mujoco.mj_forward(model, data)
            res = []
            for _name, target_idx, body_id, weight in mapping:
                res.extend((data.xpos[body_id] - frame_targets[target_idx]) * weight)
            res.extend((dof[solve_ids] - default_dof[solve_ids]) * reg_weight)
            res.extend((dof[solve_ids] - prev_dof[solve_ids]) * temporal_weight)
            return np.asarray(res, dtype=np.float64)

        result = least_squares(
            residual,
            prev_dof[solve_ids],
            bounds=(x_lower, x_upper),
            max_nfev=90,
            ftol=1e-4,
            xtol=1e-4,
            gtol=1e-4,
        )
        dof = unpack(result.x)
        qpos = make_qpos(G1_DEFAULT_ROOT_POS, dof)

        data.qpos[:] = qpos
        mujoco.mj_forward(model, data)
        errors = []
        for _name, target_idx, body_id, _weight in mapping:
            errors.append(np.linalg.norm(data.xpos[body_id] - frame_targets[target_idx]))
        errors = np.asarray(errors, dtype=np.float64)

        qpos_ref.append(qpos.copy())
        dof_ref.append(dof.copy())
        mean_errors.append(float(errors.mean()))
        max_errors.append(float(errors.max()))
        prev_dof = dof

        if out_i % 25 == 0 or out_i == len(frame_indices) - 1:
            print(
                f"frame {out_i + 1:4d}/{len(frame_indices)} src={frame_idx:4d} "
                f"mean_err={mean_errors[-1]:.4f} max_err={max_errors[-1]:.4f}"
            )

    qpos_ref_array = np.asarray(qpos_ref, dtype=np.float32)
    timestamps_out = timestamps[frame_indices]
    return {
        "timestamps": timestamps_out,
        "source_frame_indices": frame_indices,
        "qpos_ref": qpos_ref_array,
        "dof_pos_ref": np.asarray(dof_ref, dtype=np.float32),
        "mimic_obs": build_mimic_obs(timestamps_out, qpos_ref_array),
        "ik_mean_error": np.asarray(mean_errors, dtype=np.float32),
        "ik_max_error": np.asarray(max_errors, dtype=np.float32),
        "target_names": np.asarray(target_names),
        "target_positions_rh": targets[frame_indices].astype(np.float32),
        "target_to_g1_body_json": np.asarray([json.dumps(target_to_body)]),
        "retarget_type": np.asarray(["gmr_like_delta_upper_body"]),
        "solve_set": np.asarray([solve_set]),
    }


def default_output_path(source: Path) -> Path:
    return source.with_name(f"{source.stem}_g1_gmr_like.npz")


def resolve_xml_path(path: Path) -> Path:
    path = path.expanduser()
    if path.exists():
        return path
    script_relative = (SCRIPT_DIR / path).resolve()
    if script_relative.exists():
        return script_relative
    hand_tracking_relative = (HAND_TRACKING_DIR / path).resolve()
    if hand_tracking_relative.exists():
        return hand_tracking_relative
    workspace_relative = (WORKSPACE_DIR / path).resolve()
    if workspace_relative.exists():
        return workspace_relative
    raise SystemExit(f"G1 MuJoCo XML not found: {path}")


def main() -> None:
    parser = argparse.ArgumentParser(description="GMR-like retarget 84-joint recordings to G1 references.")
    parser.add_argument("recording", type=Path, help="Input raw 84-joint .npz recording.")
    parser.add_argument("--xml", type=Path, default=DEFAULT_XML, help="G1 MuJoCo XML path.")
    parser.add_argument("-o", "--output", type=Path, help="Output G1 reference .npz path.")
    parser.add_argument("--solve-set", choices=["upper", "full"], default="upper")
    parser.add_argument("--frame-step", type=int, default=1)
    parser.add_argument("--max-frames", type=int)
    parser.add_argument("--motion-scale", type=float, default=1.0)
    parser.add_argument("--torso-motion-scale", type=float, default=0.25)
    parser.add_argument("--max-hand-displacement", type=float, default=0.45)
    parser.add_argument("--smooth-alpha", type=float, default=0.80)
    parser.add_argument("--reg-weight", type=float, default=0.08)
    parser.add_argument("--temporal-weight", type=float, default=0.25)
    parser.add_argument("--max-dof-offset", type=float, default=1.15)
    args = parser.parse_args()

    recording = args.recording.expanduser()
    output = args.output.expanduser() if args.output else default_output_path(recording)
    timestamps, _root_poses, joint_poses = load_raw_recording(recording)
    source_positions, source_names, source_indices = load_source_positions(joint_poses)
    model = mujoco.MjModel.from_xml_path(str(resolve_xml_path(args.xml)))

    targets, target_names, target_to_body = build_motion_targets(
        model=model,
        source_positions=source_positions,
        names=source_names,
        motion_scale=args.motion_scale,
        torso_motion_scale=args.torso_motion_scale,
        max_hand_displacement=args.max_hand_displacement,
        smooth_alpha=args.smooth_alpha,
    )
    result = solve_retargeted_ik(
        model=model,
        timestamps=timestamps,
        targets=targets,
        target_names=target_names,
        target_to_body=target_to_body,
        solve_set=args.solve_set,
        frame_step=args.frame_step,
        max_frames=args.max_frames,
        reg_weight=args.reg_weight,
        temporal_weight=args.temporal_weight,
        max_dof_offset=args.max_dof_offset,
    )
    result["source_joint_names"] = np.asarray(source_names)
    result["source_joint_indices"] = source_indices
    result["source_recording"] = np.asarray([str(recording)])

    output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(output, **result)
    print(f"saved: {output}")
    print(f"qpos_ref: {result['qpos_ref'].shape}")
    print(f"mimic_obs: {result['mimic_obs'].shape}")
    print(
        f"IK error mean={result['ik_mean_error'].mean():.4f}m "
        f"max={result['ik_max_error'].max():.4f}m"
    )


if __name__ == "__main__":
    main()
