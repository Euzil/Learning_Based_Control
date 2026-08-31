#!/usr/bin/env python3
"""Extract compact robot-control targets from 84-joint body recordings.

Input recordings are expected to contain:
    timestamps[N]
    root_poses[N, 7]
    joint_poses[N, 84, 7]

Each pose is [x, y, z, qx, qy, qz, qw] in Unity coordinates by default.
The output keeps a small set of named end-effector/body targets suitable for
retargeting, IK, or high-level robot control.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np


BODY_JOINT_NAMES = [
    "Body_Root",
    "Body_Hips",
    "Body_SpineLower",
    "Body_SpineMiddle",
    "Body_SpineUpper",
    "Body_Chest",
    "Body_Neck",
    "Body_Head",
    "Body_LeftShoulder",
    "Body_LeftScapula",
    "Body_LeftArmUpper",
    "Body_LeftArmLower",
    "Body_LeftHandWristTwist",
    "Body_RightShoulder",
    "Body_RightScapula",
    "Body_RightArmUpper",
    "Body_RightArmLower",
    "Body_RightHandWristTwist",
    "Body_LeftHandPalm",
    "Body_LeftHandWrist",
    "Body_LeftHandThumbMetacarpal",
    "Body_LeftHandThumbProximal",
    "Body_LeftHandThumbDistal",
    "Body_LeftHandThumbTip",
    "Body_LeftHandIndexMetacarpal",
    "Body_LeftHandIndexProximal",
    "Body_LeftHandIndexIntermediate",
    "Body_LeftHandIndexDistal",
    "Body_LeftHandIndexTip",
    "Body_LeftHandMiddleMetacarpal",
    "Body_LeftHandMiddleProximal",
    "Body_LeftHandMiddleIntermediate",
    "Body_LeftHandMiddleDistal",
    "Body_LeftHandMiddleTip",
    "Body_LeftHandRingMetacarpal",
    "Body_LeftHandRingProximal",
    "Body_LeftHandRingIntermediate",
    "Body_LeftHandRingDistal",
    "Body_LeftHandRingTip",
    "Body_LeftHandLittleMetacarpal",
    "Body_LeftHandLittleProximal",
    "Body_LeftHandLittleIntermediate",
    "Body_LeftHandLittleDistal",
    "Body_LeftHandLittleTip",
    "Body_RightHandPalm",
    "Body_RightHandWrist",
    "Body_RightHandThumbMetacarpal",
    "Body_RightHandThumbProximal",
    "Body_RightHandThumbDistal",
    "Body_RightHandThumbTip",
    "Body_RightHandIndexMetacarpal",
    "Body_RightHandIndexProximal",
    "Body_RightHandIndexIntermediate",
    "Body_RightHandIndexDistal",
    "Body_RightHandIndexTip",
    "Body_RightHandMiddleMetacarpal",
    "Body_RightHandMiddleProximal",
    "Body_RightHandMiddleIntermediate",
    "Body_RightHandMiddleDistal",
    "Body_RightHandMiddleTip",
    "Body_RightHandRingMetacarpal",
    "Body_RightHandRingProximal",
    "Body_RightHandRingIntermediate",
    "Body_RightHandRingDistal",
    "Body_RightHandRingTip",
    "Body_RightHandLittleMetacarpal",
    "Body_RightHandLittleProximal",
    "Body_RightHandLittleIntermediate",
    "Body_RightHandLittleDistal",
    "Body_RightHandLittleTip",
    "Body_LeftLegUpper",
    "Body_LeftLegLower",
    "Body_LeftFootAnkleTwist",
    "Body_LeftFootAnkle",
    "Body_LeftFootSubtalar",
    "Body_LeftFootTransverse",
    "Body_LeftFootBall",
    "Body_RightLegUpper",
    "Body_RightLegLower",
    "Body_RightFootAnkleTwist",
    "Body_RightFootAnkle",
    "Body_RightFootSubtalar",
    "Body_RightFootTransverse",
    "Body_RightFootBall",
]


TARGET_PRESETS = {
    # Main high-level control targets for a humanoid robot.
    "minimal": {
        "pelvis": 1,
        "chest": 5,
        "head": 7,
        "left_hand": 19,
        "right_hand": 45,
        "left_foot": 73,
        "right_foot": 80,
    },
    # Adds knees, elbows, and toe/ball points. Useful for IK debugging.
    "ik": {
        "pelvis": 1,
        "chest": 5,
        "head": 7,
        "left_elbow": 11,
        "left_hand": 19,
        "right_elbow": 16,
        "right_hand": 45,
        "left_knee": 71,
        "left_ankle": 73,
        "left_toe": 76,
        "right_knee": 78,
        "right_ankle": 80,
        "right_toe": 83,
    },
    # Command targets tuned for G1 policy tracking. This keeps the dominant
    # motion intent while avoiding head/toe constraints that often overfit the
    # human skeleton proportions during IK.
    "g1": {
        "pelvis": 1,
        "chest": 5,
        "left_elbow": 11,
        "left_hand": 19,
        "right_elbow": 16,
        "right_hand": 45,
        "left_knee": 71,
        "left_ankle": 73,
        "right_knee": 78,
        "right_ankle": 80,
    },
    # A compact full-body skeleton, still excluding fingers and foot internals.
    "body": {
        "pelvis": 1,
        "spine_lower": 2,
        "spine_middle": 3,
        "chest": 5,
        "neck": 6,
        "head": 7,
        "left_shoulder": 8,
        "left_upper_arm": 10,
        "left_lower_arm": 11,
        "left_hand": 19,
        "right_shoulder": 13,
        "right_upper_arm": 15,
        "right_lower_arm": 16,
        "right_hand": 45,
        "left_upper_leg": 70,
        "left_lower_leg": 71,
        "left_ankle": 73,
        "left_toe": 76,
        "right_upper_leg": 77,
        "right_lower_leg": 78,
        "right_ankle": 80,
        "right_toe": 83,
    },
    # Finger tips only, for later hand/gripper intent extraction.
    "hands": {
        "left_wrist": 19,
        "left_thumb_tip": 23,
        "left_index_tip": 28,
        "left_middle_tip": 33,
        "left_ring_tip": 38,
        "left_little_tip": 43,
        "right_wrist": 45,
        "right_thumb_tip": 49,
        "right_index_tip": 54,
        "right_middle_tip": 59,
        "right_ring_tip": 64,
        "right_little_tip": 69,
    },
}


# Unity left-handed (x right, y up, z forward) to common robotics right-handed
# coordinates (x forward, y left, z up).
UNITY_TO_RH = np.array(
    [
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ],
    dtype=np.float32,
)


def quat_normalize(quat: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(quat, axis=-1, keepdims=True)
    return np.divide(quat, norm, out=np.zeros_like(quat), where=norm > 0.0)


def quat_to_matrix(quat: np.ndarray) -> np.ndarray:
    quat = quat_normalize(quat)
    x, y, z, w = np.moveaxis(quat, -1, 0)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z

    mat = np.empty(quat.shape[:-1] + (3, 3), dtype=quat.dtype)
    mat[..., 0, 0] = 1.0 - 2.0 * (yy + zz)
    mat[..., 0, 1] = 2.0 * (xy - wz)
    mat[..., 0, 2] = 2.0 * (xz + wy)
    mat[..., 1, 0] = 2.0 * (xy + wz)
    mat[..., 1, 1] = 1.0 - 2.0 * (xx + zz)
    mat[..., 1, 2] = 2.0 * (yz - wx)
    mat[..., 2, 0] = 2.0 * (xz - wy)
    mat[..., 2, 1] = 2.0 * (yz + wx)
    mat[..., 2, 2] = 1.0 - 2.0 * (xx + yy)
    return mat


def matrix_to_quat(mat: np.ndarray) -> np.ndarray:
    flat = mat.reshape((-1, 3, 3))
    quats = np.empty((flat.shape[0], 4), dtype=mat.dtype)

    for i, m in enumerate(flat):
        trace = m[0, 0] + m[1, 1] + m[2, 2]
        if trace > 0.0:
            s = 0.5 / np.sqrt(trace + 1.0)
            qw = 0.25 / s
            qx = (m[2, 1] - m[1, 2]) * s
            qy = (m[0, 2] - m[2, 0]) * s
            qz = (m[1, 0] - m[0, 1]) * s
        elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
            s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
            qw = (m[2, 1] - m[1, 2]) / s
            qx = 0.25 * s
            qy = (m[0, 1] + m[1, 0]) / s
            qz = (m[0, 2] + m[2, 0]) / s
        elif m[1, 1] > m[2, 2]:
            s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
            qw = (m[0, 2] - m[2, 0]) / s
            qx = (m[0, 1] + m[1, 0]) / s
            qy = 0.25 * s
            qz = (m[1, 2] + m[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
            qw = (m[1, 0] - m[0, 1]) / s
            qx = (m[0, 2] + m[2, 0]) / s
            qy = (m[1, 2] + m[2, 1]) / s
            qz = 0.25 * s
        quats[i] = [qx, qy, qz, qw]

    return quat_normalize(quats).reshape(mat.shape[:-2] + (4,))


def convert_poses_unity_to_rh(poses: np.ndarray) -> np.ndarray:
    converted = poses.copy()
    converted[..., :3] = poses[..., :3] @ UNITY_TO_RH.T

    rot_unity = quat_to_matrix(poses[..., 3:7])
    rot_rh = UNITY_TO_RH @ rot_unity @ UNITY_TO_RH.T
    converted[..., 3:7] = matrix_to_quat(rot_rh)
    return converted


def derive_robot_features(target_poses: np.ndarray, target_names: list[str]) -> dict[str, np.ndarray]:
    positions = target_poses[..., :3]
    name_to_idx = {name: i for i, name in enumerate(target_names)}
    features: dict[str, np.ndarray] = {}

    def vec(a: str, b: str) -> np.ndarray | None:
        if a not in name_to_idx or b not in name_to_idx:
            return None
        return positions[:, name_to_idx[b]] - positions[:, name_to_idx[a]]

    forward = vec("pelvis", "chest")
    if forward is not None:
        features["pelvis_to_chest"] = forward.astype(np.float32)

    for side in ("left", "right"):
        hand = vec("pelvis", f"{side}_hand")
        foot = vec("pelvis", f"{side}_foot")
        if hand is not None:
            features[f"{side}_hand_relative_to_pelvis"] = hand.astype(np.float32)
        if foot is not None:
            features[f"{side}_foot_relative_to_pelvis"] = foot.astype(np.float32)

    return features


def load_recording(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    recording = np.load(path, allow_pickle=False)
    required = {"timestamps", "root_poses", "joint_poses"}
    missing = required.difference(recording.files)
    if missing:
        raise SystemExit(f"{path} is missing required arrays: {sorted(missing)}")

    timestamps = np.asarray(recording["timestamps"], dtype=np.float64)
    root_poses = np.asarray(recording["root_poses"], dtype=np.float32)
    joint_poses = np.asarray(recording["joint_poses"], dtype=np.float32)

    if joint_poses.ndim != 3 or joint_poses.shape[1:] != (84, 7):
        raise SystemExit(f"Expected joint_poses shape [N, 84, 7], got {joint_poses.shape}")
    if root_poses.shape != (joint_poses.shape[0], 7):
        raise SystemExit(f"Expected root_poses shape [{joint_poses.shape[0]}, 7], got {root_poses.shape}")
    if timestamps.shape != (joint_poses.shape[0],):
        raise SystemExit(f"Expected timestamps shape [{joint_poses.shape[0]}], got {timestamps.shape}")

    return timestamps, root_poses, joint_poses


def extract_targets(
    source: Path,
    output: Path,
    preset: str,
    keep_coordinates: str,
) -> None:
    timestamps, root_poses, joint_poses = load_recording(source)

    target_map = TARGET_PRESETS[preset]
    target_names = list(target_map.keys())
    source_indices = np.asarray(list(target_map.values()), dtype=np.int32)
    source_joint_names = np.asarray([BODY_JOINT_NAMES[i] for i in source_indices])

    target_poses = joint_poses[:, source_indices, :]
    root_out = root_poses
    coordinate_frame = "unity_lh"

    if keep_coordinates == "rh":
        target_poses = convert_poses_unity_to_rh(target_poses)
        root_out = convert_poses_unity_to_rh(root_poses[:, None, :])[:, 0, :]
        coordinate_frame = "right_handed_x_forward_y_left_z_up"

    features = derive_robot_features(target_poses, target_names)
    output.parent.mkdir(parents=True, exist_ok=True)

    np.savez_compressed(
        output,
        timestamps=timestamps,
        root_poses=root_out.astype(np.float32),
        target_poses=target_poses.astype(np.float32),
        target_positions=target_poses[..., :3].astype(np.float32),
        target_quaternions_xyzw=target_poses[..., 3:7].astype(np.float32),
        target_names=np.asarray(target_names),
        source_indices=source_indices,
        source_joint_names=source_joint_names,
        coordinate_frame=np.asarray([coordinate_frame]),
        preset=np.asarray([preset]),
        **features,
    )

    duration = timestamps[-1] - timestamps[0] if timestamps.size > 1 else 0.0
    mean_fps = (len(timestamps) - 1) / duration if duration > 0.0 else 0.0
    print(f"source: {source}")
    print(f"output: {output}")
    print(f"preset: {preset}")
    print(f"coordinate_frame: {coordinate_frame}")
    print(f"frames: {len(timestamps)}")
    print(f"mean_fps: {mean_fps:.2f}")
    print(f"target_poses: {target_poses.shape}")
    print("targets:")
    for name, idx, src_name in zip(target_names, source_indices, source_joint_names):
        print(f"  {name:>12s} <- {idx:02d} {src_name}")


def default_output_path(source: Path, preset: str, keep_coordinates: str) -> Path:
    suffix = "rh" if keep_coordinates == "rh" else "unity"
    return source.with_name(f"{source.stem}_robot_targets_{preset}_{suffix}.npz")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Extract compact robot-control targets from 84-joint NPZ recordings."
    )
    parser.add_argument("source", type=Path, help="Input .npz recording.")
    parser.add_argument("-o", "--output", type=Path, help="Output .npz path.")
    parser.add_argument(
        "--preset",
        choices=sorted(TARGET_PRESETS),
        default="minimal",
        help="Target set to extract. Use 'ik' or 'body' for richer retargeting.",
    )
    parser.add_argument(
        "--coordinates",
        choices=("rh", "unity"),
        default="rh",
        help="Output coordinate frame. 'rh' converts from Unity to robotics RH coordinates.",
    )
    args = parser.parse_args()

    source = args.source.expanduser()
    output = args.output.expanduser() if args.output else default_output_path(
        source, args.preset, args.coordinates
    )
    extract_targets(source, output, args.preset, args.coordinates)


if __name__ == "__main__":
    main()
