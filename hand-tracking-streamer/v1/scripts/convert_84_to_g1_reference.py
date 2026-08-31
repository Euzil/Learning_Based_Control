#!/usr/bin/env python3
"""Convert 84-joint Quest/Unity body recordings to G1 reference poses.

Pipeline:
    raw recording npz
        joint_poses[N, 84, 7] in Unity coordinates
    -> compact G1 task targets
        target_positions[N, K, 3] in RH robotics coordinates
    -> MuJoCo position IK
        qpos_ref[N, 36] = root xyz + root quat(wxyz) + 29 G1 joint angles
    -> TWIST2-style reference observation
        mimic_obs[N, 35] = root_vel_xy + root_z + roll_pitch + yaw_vel + dof_pos

This script only generates reference poses. It does not run the low-level policy.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np
from scipy.optimize import least_squares

from extract_robot_targets import TARGET_PRESETS, convert_poses_unity_to_rh


DEFAULT_XML = Path("../TWIST2/assets/g1/g1_mocap_29dof.xml")

TARGET_TO_G1_BODY = {
    "pelvis": "pelvis",
    "chest": "torso_link",
    "head": "head_mocap",
    "left_elbow": "left_elbow_link",
    "left_hand": "left_rubber_hand",
    "right_elbow": "right_elbow_link",
    "right_hand": "right_rubber_hand",
    "left_knee": "left_knee_link",
    "left_ankle": "left_ankle_roll_link",
    "left_toe": "left_toe_link",
    "right_knee": "right_knee_link",
    "right_ankle": "right_ankle_roll_link",
    "right_toe": "right_toe_link",
}

TARGET_WEIGHTS = {
    "pelvis": 2.0,
    "chest": 1.5,
    "head": 0.5,
    "left_elbow": 0.8,
    "left_hand": 3.0,
    "right_elbow": 0.8,
    "right_hand": 3.0,
    "left_knee": 0.6,
    "left_ankle": 2.0,
    "left_toe": 0.7,
    "right_knee": 0.6,
    "right_ankle": 2.0,
    "right_toe": 0.7,
}

DEFAULT_DOF_POS = np.array(
    [
        -0.2, 0.0, 0.0, 0.4, -0.2, 0.0,
        -0.2, 0.0, 0.0, 0.4, -0.2, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.4, 0.0, 1.2, 0.0, 0.0, 0.0,
        0.0, -0.4, 0.0, 1.2, 0.0, 0.0, 0.0,
    ],
    dtype=np.float64,
)


def load_raw_recording(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    data = np.load(path, allow_pickle=False)
    required = {"timestamps", "root_poses", "joint_poses"}
    missing = required.difference(data.files)
    if missing:
        raise SystemExit(f"{path} is missing required arrays: {sorted(missing)}")

    timestamps = np.asarray(data["timestamps"], dtype=np.float64)
    root_poses = np.asarray(data["root_poses"], dtype=np.float32)
    joint_poses = np.asarray(data["joint_poses"], dtype=np.float32)
    if joint_poses.ndim != 3 or joint_poses.shape[1:] != (84, 7):
        raise SystemExit(f"Expected joint_poses shape [N, 84, 7], got {joint_poses.shape}")
    if root_poses.shape != (joint_poses.shape[0], 7):
        raise SystemExit(f"Expected root_poses shape [{joint_poses.shape[0]}, 7], got {root_poses.shape}")
    if timestamps.shape != (joint_poses.shape[0],):
        raise SystemExit(f"Expected timestamps shape [{joint_poses.shape[0]}], got {timestamps.shape}")
    return timestamps, root_poses, joint_poses


def lowpass(values: np.ndarray, alpha: float) -> np.ndarray:
    if alpha <= 0.0:
        return values
    alpha = min(alpha, 0.99)
    out = values.copy()
    for i in range(1, len(out)):
        out[i] = alpha * out[i - 1] + (1.0 - alpha) * out[i]
    return out


def quat_wxyz_to_matrix(quat: np.ndarray) -> np.ndarray:
    quat = np.asarray(quat, dtype=np.float64)
    norm = np.linalg.norm(quat)
    if norm <= 0.0:
        quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    else:
        quat = quat / norm
    w, x, y, z = quat
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def quat_wxyz_to_rpy(quat: np.ndarray) -> np.ndarray:
    rot = quat_wxyz_to_matrix(quat)
    roll = np.arctan2(rot[2, 1], rot[2, 2])
    pitch = np.arctan2(-rot[2, 0], np.sqrt(rot[2, 1] ** 2 + rot[2, 2] ** 2))
    yaw = np.arctan2(rot[1, 0], rot[0, 0])
    return np.array([roll, pitch, yaw], dtype=np.float64)


def extract_targets(
    joint_poses: np.ndarray,
    preset: str,
) -> tuple[np.ndarray, list[str], np.ndarray]:
    if preset not in TARGET_PRESETS:
        raise SystemExit(f"Unknown preset {preset!r}. Available: {sorted(TARGET_PRESETS)}")
    target_map = TARGET_PRESETS[preset]
    names = list(target_map.keys())
    indices = np.asarray(list(target_map.values()), dtype=np.int32)
    poses_rh = convert_poses_unity_to_rh(joint_poses[:, indices, :])
    return poses_rh[..., :3].astype(np.float64), names, indices


def resolve_target_body_mapping(
    model: mujoco.MjModel,
    target_names: list[str],
) -> list[tuple[str, int, int, float]]:
    name_to_target_idx = {name: i for i, name in enumerate(target_names)}
    mapping = []
    for target_name, body_name in TARGET_TO_G1_BODY.items():
        if target_name not in name_to_target_idx:
            continue
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if body_id < 0:
            print(f"Skipping target {target_name}: G1 body not found: {body_name}")
            continue
        mapping.append(
            (
                target_name,
                name_to_target_idx[target_name],
                body_id,
                TARGET_WEIGHTS.get(target_name, 1.0),
            )
        )
    if not mapping:
        raise SystemExit("No extracted target can be mapped to the G1 model.")
    return mapping


def joint_bounds(model: mujoco.MjModel) -> tuple[np.ndarray, np.ndarray]:
    lows = []
    highs = []
    for joint_id in range(1, model.njnt):
        lows.append(model.jnt_range[joint_id, 0])
        highs.append(model.jnt_range[joint_id, 1])
    return np.asarray(lows, dtype=np.float64), np.asarray(highs, dtype=np.float64)


def make_qpos(root_pos: np.ndarray, dof_pos: np.ndarray) -> np.ndarray:
    qpos = np.zeros(7 + len(dof_pos), dtype=np.float64)
    qpos[:3] = root_pos
    qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    qpos[7:] = dof_pos
    return qpos


def align_targets_to_robot(
    target_positions: np.ndarray,
    target_names: list[str],
    robot_pelvis_default: np.ndarray,
    scale: float,
    keep_root_xy: bool,
    keep_root_height: bool,
) -> tuple[np.ndarray, np.ndarray]:
    name_to_idx = {name: i for i, name in enumerate(target_names)}
    if "pelvis" not in name_to_idx:
        raise SystemExit("Target preset must include 'pelvis'.")

    pelvis_idx = name_to_idx["pelvis"]
    pelvis = target_positions[:, pelvis_idx]
    pelvis_delta = (pelvis - pelvis[0]) * scale
    root_pos = robot_pelvis_default[None, :] + pelvis_delta
    if keep_root_xy:
        root_pos[:, :2] = robot_pelvis_default[:2]
    if keep_root_height:
        root_pos[:, 2] = robot_pelvis_default[2]

    relative_targets = (target_positions - pelvis[:, None, :]) * scale
    aligned_targets = root_pos[:, None, :] + relative_targets
    return root_pos, aligned_targets


def build_mimic_obs(timestamps: np.ndarray, qpos_ref: np.ndarray) -> np.ndarray:
    n = len(qpos_ref)
    root_pos = qpos_ref[:, :3]
    root_quat = qpos_ref[:, 3:7]
    dof_pos = qpos_ref[:, 7:]

    roll_pitch = np.zeros((n, 2), dtype=np.float64)
    yaw = np.zeros(n, dtype=np.float64)
    for i in range(n):
        rpy = quat_wxyz_to_rpy(root_quat[i])
        roll_pitch[i] = rpy[:2]
        yaw[i] = rpy[2]

    dt = np.diff(timestamps)
    dt = np.where(dt > 1e-6, dt, 1.0 / 50.0)
    root_vel_world = np.zeros((n, 3), dtype=np.float64)
    root_vel_world[1:] = (root_pos[1:] - root_pos[:-1]) / dt[:, None]

    root_vel_xy = np.zeros((n, 2), dtype=np.float64)
    for i in range(n):
        root_vel_xy[i] = (quat_wxyz_to_matrix(root_quat[i]).T @ root_vel_world[i])[:2]

    yaw_vel = np.zeros(n, dtype=np.float64)
    yaw_vel[1:] = np.diff(np.unwrap(yaw)) / dt

    mimic_obs = np.concatenate(
        [root_vel_xy, root_pos[:, 2:3], roll_pitch, yaw_vel[:, None], dof_pos],
        axis=1,
    )
    if mimic_obs.shape[1] != 35:
        raise RuntimeError(f"Expected mimic_obs width 35, got {mimic_obs.shape}")
    return mimic_obs.astype(np.float32)


def solve_g1_ik(
    model: mujoco.MjModel,
    timestamps: np.ndarray,
    target_positions: np.ndarray,
    target_names: list[str],
    scale: float,
    frame_step: int,
    max_frames: int | None,
    keep_root_xy: bool,
    keep_root_height: bool,
    smooth_targets_alpha: float,
    reg_weight: float,
    temporal_weight: float,
) -> dict[str, np.ndarray]:
    frame_indices = np.arange(0, len(timestamps), max(1, frame_step), dtype=np.int32)
    if max_frames is not None:
        frame_indices = frame_indices[:max_frames]

    data = mujoco.MjData(model)
    default_dof = DEFAULT_DOF_POS.copy()
    if len(default_dof) != model.nq - 7:
        default_dof = np.zeros(model.nq - 7, dtype=np.float64)
    default_qpos = make_qpos(np.array([0.0, 0.0, 0.793]), default_dof)
    data.qpos[:] = default_qpos
    mujoco.mj_forward(model, data)
    pelvis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    robot_pelvis_default = data.xpos[pelvis_id].copy()

    if smooth_targets_alpha > 0.0:
        target_positions = lowpass(target_positions, smooth_targets_alpha)

    root_pos, aligned_targets = align_targets_to_robot(
        target_positions,
        target_names,
        robot_pelvis_default,
        scale,
        keep_root_xy,
        keep_root_height,
    )
    mapping = resolve_target_body_mapping(model, target_names)
    lower, upper = joint_bounds(model)
    prev_dof = np.clip(default_dof, lower, upper)

    qpos_ref = []
    dof_ref = []
    mean_errors = []
    max_errors = []

    print("G1 IK target mapping:")
    for target_name, _target_idx, body_id, weight in mapping:
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
        print(f"  {target_name:>12s} -> {body_name:<22s} weight={weight:g}")

    for out_i, frame_idx in enumerate(frame_indices):
        frame_root = root_pos[frame_idx]
        frame_targets = aligned_targets[frame_idx]

        def residual(dof: np.ndarray) -> np.ndarray:
            data.qpos[:] = make_qpos(frame_root, dof)
            mujoco.mj_forward(model, data)
            res = []
            for _name, target_idx, body_id, weight in mapping:
                res.extend((data.xpos[body_id] - frame_targets[target_idx]) * weight)
            res.extend((dof - default_dof) * reg_weight)
            res.extend((dof - prev_dof) * temporal_weight)
            return np.asarray(res, dtype=np.float64)

        result = least_squares(
            residual,
            prev_dof,
            bounds=(lower, upper),
            max_nfev=80,
            ftol=1e-4,
            xtol=1e-4,
            gtol=1e-4,
        )
        dof = np.clip(result.x, lower, upper)
        qpos = make_qpos(frame_root, dof)
        data.qpos[:] = qpos
        mujoco.mj_forward(model, data)

        errors = []
        for _name, target_idx, body_id, _weight in mapping:
            errors.append(np.linalg.norm(data.xpos[body_id] - frame_targets[target_idx]))
        errors = np.asarray(errors)
        mean_errors.append(float(errors.mean()))
        max_errors.append(float(errors.max()))

        qpos_ref.append(qpos.copy())
        dof_ref.append(dof.copy())
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
        "target_positions_rh": target_positions[frame_indices].astype(np.float32),
        "target_to_g1_body_json": np.asarray([json.dumps(TARGET_TO_G1_BODY)]),
        "preset": np.asarray(["g1_reference"]),
        "scale": np.asarray([scale], dtype=np.float32),
    }


def default_output_path(source: Path, preset: str) -> Path:
    return source.with_name(f"{source.stem}_g1_reference_{preset}.npz")


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert raw 84-joint recordings to G1 references.")
    parser.add_argument("recording", type=Path, help="Input raw 84-joint .npz recording.")
    parser.add_argument("--xml", type=Path, default=DEFAULT_XML, help="G1 MuJoCo XML path.")
    parser.add_argument("-o", "--output", type=Path, help="Output G1 reference .npz path.")
    parser.add_argument(
        "--preset",
        choices=sorted(TARGET_PRESETS),
        default="g1",
        help="Source target preset to use before IK.",
    )
    parser.add_argument("--scale", type=float, default=1.0, help="Scale human targets around pelvis.")
    parser.add_argument("--frame-step", type=int, default=1, help="Use every Nth frame.")
    parser.add_argument("--max-frames", type=int, help="Limit frames for fast debug conversion.")
    parser.add_argument("--move-root-xy", action="store_true", help="Use recorded pelvis XY motion.")
    parser.add_argument("--move-root-height", action="store_true", help="Use recorded pelvis Z motion.")
    parser.add_argument("--smooth-targets-alpha", type=float, default=0.75, help="Low-pass target positions.")
    parser.add_argument("--reg-weight", type=float, default=0.04, help="Keep joints near default pose.")
    parser.add_argument("--temporal-weight", type=float, default=0.20, help="Keep consecutive frames smooth.")
    args = parser.parse_args()

    recording = args.recording.expanduser()
    output = args.output.expanduser() if args.output else default_output_path(recording, args.preset)
    timestamps, _root_poses, joint_poses = load_raw_recording(recording)
    target_positions, target_names, target_indices = extract_targets(joint_poses, args.preset)

    model = mujoco.MjModel.from_xml_path(str(args.xml.expanduser()))
    result = solve_g1_ik(
        model=model,
        timestamps=timestamps,
        target_positions=target_positions,
        target_names=target_names,
        scale=args.scale,
        frame_step=args.frame_step,
        max_frames=args.max_frames,
        keep_root_xy=not args.move_root_xy,
        keep_root_height=not args.move_root_height,
        smooth_targets_alpha=args.smooth_targets_alpha,
        reg_weight=args.reg_weight,
        temporal_weight=args.temporal_weight,
    )
    result["source_target_indices"] = target_indices
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
