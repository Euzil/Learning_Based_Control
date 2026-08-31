#!/usr/bin/env python3
"""Offline HTS 84-joint recording adapter for TWIST2's XRobot teleop path.

The purpose of this adapter is narrow: make an HTS/Pico 84-joint NPZ recording
look like the live XRobotStreamer output expected by GMR/TWIST2. It does not
retarget to G1 itself and does not generate robot actions.
"""

from __future__ import annotations

import time
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R


XROBOT_BODY_TO_HTS84 = {
    "Pelvis": 1,
    "Left_Hip": 70,
    "Right_Hip": 77,
    "Spine1": 2,
    "Left_Knee": 71,
    "Right_Knee": 78,
    "Spine2": 3,
    "Left_Ankle": 73,
    "Right_Ankle": 80,
    "Spine3": 5,
    "Left_Foot": 76,
    "Right_Foot": 83,
    "Neck": 6,
    "Left_Collar": 9,
    "Right_Collar": 14,
    "Head": 7,
    "Left_Shoulder": 10,
    "Right_Shoulder": 15,
    "Left_Elbow": 11,
    "Right_Elbow": 16,
    "Left_Wrist": 12,
    "Right_Wrist": 17,
    "Left_Hand": 19,
    "Right_Hand": 45,
}

LEFT_HAND_TO_HTS84 = {
    "Wrist": 19,
    "Palm": 18,
    "ThumbMetacarpal": 20,
    "ThumbProximal": 21,
    "ThumbDistal": 22,
    "ThumbTip": 23,
    "IndexMetacarpal": 24,
    "IndexProximal": 25,
    "IndexIntermediate": 26,
    "IndexDistal": 27,
    "IndexTip": 28,
    "MiddleMetacarpal": 29,
    "MiddleProximal": 30,
    "MiddleIntermediate": 31,
    "MiddleDistal": 32,
    "MiddleTip": 33,
    "RingMetacarpal": 34,
    "RingProximal": 35,
    "RingIntermediate": 36,
    "RingDistal": 37,
    "RingTip": 38,
    "LittleMetacarpal": 39,
    "LittleProximal": 40,
    "LittleIntermediate": 41,
    "LittleDistal": 42,
    "LittleTip": 43,
}

RIGHT_HAND_TO_HTS84 = {
    "Wrist": 45,
    "Palm": 44,
    "ThumbMetacarpal": 46,
    "ThumbProximal": 47,
    "ThumbDistal": 48,
    "ThumbTip": 49,
    "IndexMetacarpal": 50,
    "IndexProximal": 51,
    "IndexIntermediate": 52,
    "IndexDistal": 53,
    "IndexTip": 54,
    "MiddleMetacarpal": 55,
    "MiddleProximal": 56,
    "MiddleIntermediate": 57,
    "MiddleDistal": 58,
    "MiddleTip": 59,
    "RingMetacarpal": 60,
    "RingProximal": 61,
    "RingIntermediate": 62,
    "RingDistal": 63,
    "RingTip": 64,
    "LittleMetacarpal": 65,
    "LittleProximal": 66,
    "LittleIntermediate": 67,
    "LittleDistal": 68,
    "LittleTip": 69,
}

G1_ROT_OFFSET_BY_HUMAN_BODY = {
    "Pelvis": [-0.5, 0.5, -0.5, -0.5],
    "Left_Hip": [-0.5, 0.5, -0.5, -0.5],
    "Left_Knee": [-0.5, 0.5, -0.5, -0.5],
    "Left_Foot": [-0.5, 0.5, -0.5, -0.5],
    "Right_Hip": [-0.5, 0.5, -0.5, -0.5],
    "Right_Knee": [-0.5, 0.5, -0.5, -0.5],
    "Right_Foot": [-0.5, 0.5, -0.5, -0.5],
    "Spine3": [-0.5, 0.5, -0.5, -0.5],
    "Left_Shoulder": [0.7071067811865475, 0.0, 0.7071067811865475, 0.0],
    "Left_Elbow": [0.0, 0.0, 1.0, 0.0],
    "Left_Wrist": [0.0, 0.0, 1.0, 0.0],
    "Right_Shoulder": [0.0, 0.7071067811865475, 0.0, -0.7071067811865475],
    "Right_Elbow": [0.0, 1.0, 0.0, 0.0],
    "Right_Wrist": [0.0, 1.0, 0.0, 0.0],
}


def quat_mul_wxyz(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=np.float64,
    )


def quat_inv_wxyz(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64)
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64) / max(float(np.dot(q, q)), 1e-8)


def normalize(vec: np.ndarray, fallback: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vec))
    if norm < 1e-8:
        return fallback.astype(np.float64)
    return vec.astype(np.float64) / norm


def quat_from_primary_axis(
    primary: np.ndarray,
    secondary: np.ndarray,
    primary_axis: str,
) -> np.ndarray:
    primary = normalize(primary, np.array([1.0, 0.0, 0.0]))
    secondary = secondary - primary * float(np.dot(secondary, primary))
    secondary = normalize(secondary, np.array([0.0, 0.0, 1.0]))

    if primary_axis == "x":
        x_axis = primary
        z_axis = secondary
        y_axis = normalize(np.cross(z_axis, x_axis), np.array([0.0, 1.0, 0.0]))
        z_axis = normalize(np.cross(x_axis, y_axis), z_axis)
    elif primary_axis == "y":
        y_axis = primary
        z_axis = secondary
        x_axis = normalize(np.cross(y_axis, z_axis), np.array([1.0, 0.0, 0.0]))
        z_axis = normalize(np.cross(x_axis, y_axis), z_axis)
    elif primary_axis == "z":
        z_axis = primary
        y_axis = secondary
        x_axis = normalize(np.cross(y_axis, z_axis), np.array([1.0, 0.0, 0.0]))
        y_axis = normalize(np.cross(z_axis, x_axis), y_axis)
    else:
        raise ValueError(f"Unknown primary axis: {primary_axis}")

    matrix = np.column_stack([x_axis, y_axis, z_axis])
    if np.linalg.det(matrix) < 0:
        matrix[:, 1] *= -1.0
    return R.from_matrix(matrix).as_quat(scalar_first=True)


class HTSOfflineXRobotStreamer:
    def __init__(
        self,
        recording_path: str | Path,
        speed: float = 1.0,
        loop: bool = False,
        coordinate_transform: str = "hts_rh",
        orientation_mode: str = "source",
        target_height: float | None = None,
        calibration_path: str | Path | None = None,
    ):
        self.recording_path = Path(recording_path).expanduser()
        data = np.load(self.recording_path, allow_pickle=False)
        if "timestamps" not in data.files or "joint_poses" not in data.files:
            raise ValueError(f"{self.recording_path} must contain timestamps and joint_poses")

        self.timestamps = np.asarray(data["timestamps"], dtype=np.float64)
        self.joint_poses = np.asarray(data["joint_poses"], dtype=np.float32)
        if self.joint_poses.ndim != 3 or self.joint_poses.shape[1:] != (84, 7):
            raise ValueError(f"Expected joint_poses shape [N,84,7], got {self.joint_poses.shape}")
        if self.timestamps.shape != (self.joint_poses.shape[0],):
            raise ValueError(f"Expected timestamps shape [{self.joint_poses.shape[0]}], got {self.timestamps.shape}")

        self.speed = max(float(speed), 1e-6)
        self.loop = loop
        self.frame_index = 0
        self.last_frame_index = -1
        self.reached_end = False
        self.start_wall_time = time.time()
        self.sent_start_signal = False
        self.coordinate_transform = coordinate_transform
        self.orientation_mode = orientation_mode
        self.orientation_corrections = {}
        self.position_scale = 1.0
        if coordinate_transform == "hts_rh":
            # HTS Unity LH (x right, y up, z forward) -> robotics RH
            # (x forward, y left, z up). This matches hand-tracking-streamer's
            # visualizer/converter utilities.
            self.rotation_matrix = np.array(
                [[0, 0, 1], [-1, 0, 0], [0, 1, 0]],
                dtype=np.float64,
            )
            self.rotation_quat = None
        elif coordinate_transform == "xrobot_compat":
            # Same transform used by GMR's live XRobotStreamer. This is useful
            # only if the recording is already in XRoboToolkit's raw convention.
            self.rotation_matrix = np.array(
                [[1, 0, 0], [0, 0, -1], [0, 1, 0]],
                dtype=np.float64,
            )
            self.rotation_quat = R.from_matrix(self.rotation_matrix).as_quat(scalar_first=True)
        else:
            raise ValueError(f"Unknown coordinate transform: {coordinate_transform}")

        if target_height is not None and target_height > 0.0:
            self.position_scale = float(target_height) / max(self._estimate_height(), 1e-6)
        if calibration_path:
            self._load_calibration(calibration_path)

    def _frame_index_for_now(self) -> int:
        elapsed = (time.time() - self.start_wall_time) * self.speed
        if self.loop and self.timestamps[-1] > 0.0:
            elapsed = elapsed % self.timestamps[-1]
        idx = int(np.searchsorted(self.timestamps, elapsed, side="right") - 1)
        idx = int(np.clip(idx, 0, len(self.timestamps) - 1))
        self.last_frame_index = idx
        if not self.loop and idx == len(self.timestamps) - 1:
            self.reached_end = True
        return idx

    def _load_calibration(self, calibration_path: str | Path) -> None:
        path = Path(calibration_path).expanduser()
        with path.open() as f:
            data = json.load(f)
        self.orientation_corrections = {
            name: np.asarray(quat, dtype=np.float64)
            for name, quat in data.get("body_orientation_corrections", {}).items()
        }

    def _convert_pose(self, pose_xyzw: np.ndarray, body_name: str | None = None) -> list[list[float]]:
        x, y, z, qx, qy, qz, qw = [float(v) for v in pose_xyzw]
        position = (np.array([x, y, z], dtype=np.float64) @ self.rotation_matrix.T) * self.position_scale
        if self.orientation_mode == "identity":
            orientation = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        elif self.orientation_mode == "calibrated":
            base_orientation = self._source_orientation(qx, qy, qz, qw)
            correction = self.orientation_corrections.get(body_name)
            if correction is None:
                orientation = base_orientation
            else:
                orientation = quat_mul_wxyz(base_orientation, correction)
        elif self.coordinate_transform == "hts_rh":
            if self.orientation_mode == "raw_wxyz":
                orientation = np.array([qw, qx, qy, qz], dtype=np.float64)
            elif self.orientation_mode == "premultiply":
                orientation = quat_mul_wxyz(
                    R.from_euler("x", 90, degrees=True).as_quat(scalar_first=True),
                    np.array([qw, qx, qy, qz], dtype=np.float64),
                )
            else:
                rot_unity = R.from_quat([qx, qy, qz, qw]).as_matrix()
                rot_rh = self.rotation_matrix @ rot_unity @ self.rotation_matrix.T
                orientation = R.from_matrix(rot_rh).as_quat(scalar_first=True)
        elif self.coordinate_transform == "xrobot_compat":
            orientation = quat_mul_wxyz(
                self.rotation_quat,
                np.array([qw, qx, qy, qz], dtype=np.float64),
            )
        else:
            raise ValueError(f"Unknown coordinate transform: {self.coordinate_transform}")
        return [position.tolist(), orientation.tolist()]

    def _source_orientation(self, qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
        if self.coordinate_transform == "hts_rh":
            rot_unity = R.from_quat([qx, qy, qz, qw]).as_matrix()
            rot_rh = self.rotation_matrix @ rot_unity @ self.rotation_matrix.T
            return R.from_matrix(rot_rh).as_quat(scalar_first=True)
        if self.coordinate_transform == "xrobot_compat":
            return quat_mul_wxyz(
                self.rotation_quat,
                np.array([qw, qx, qy, qz], dtype=np.float64),
            )
        raise ValueError(f"Unknown coordinate transform: {self.coordinate_transform}")

    def _convert_position(self, pose_xyzw: np.ndarray) -> np.ndarray:
        return (pose_xyzw[:3].astype(np.float64) @ self.rotation_matrix.T) * self.position_scale

    def _estimate_height(self) -> float:
        frame = self.joint_poses[0]
        head = frame[7, :3] @ self.rotation_matrix.T
        left_foot = frame[76, :3] @ self.rotation_matrix.T
        right_foot = frame[83, :3] @ self.rotation_matrix.T
        foot = 0.5 * (left_foot + right_foot)
        return float(abs(head[2] - foot[2]))

    def _body_dict(self, frame: np.ndarray) -> dict[str, list[list[float]]]:
        if not self.orientation_mode.startswith("segments"):
            return {name: self._convert_pose(frame[idx], name) for name, idx in XROBOT_BODY_TO_HTS84.items()}

        positions = {
            name: self._convert_position(frame[idx])
            for name, idx in XROBOT_BODY_TO_HTS84.items()
        }
        body = {
            name: [pos.tolist(), [1.0, 0.0, 0.0, 0.0]]
            for name, pos in positions.items()
        }
        use_gmr_inverse_offset = self.orientation_mode.startswith("segments_robot")
        primary_axis = self.orientation_mode.rsplit("_", maxsplit=1)[-1]
        if primary_axis in ("segments", "robot"):
            primary_axis = "y"

        def set_orientation(name: str, desired_quat: np.ndarray) -> None:
            quat = desired_quat
            if use_gmr_inverse_offset and name in G1_ROT_OFFSET_BY_HUMAN_BODY:
                quat = quat_mul_wxyz(
                    desired_quat,
                    quat_inv_wxyz(np.asarray(G1_ROT_OFFSET_BY_HUMAN_BODY[name], dtype=np.float64)),
                )
            body[name][1] = quat.tolist()

        pelvis = positions["Pelvis"]
        left_hip = positions["Left_Hip"]
        right_hip = positions["Right_Hip"]
        spine = positions["Spine3"]
        left_shoulder = positions["Left_Shoulder"]
        right_shoulder = positions["Right_Shoulder"]
        lateral = normalize(left_hip - right_hip, np.array([0.0, 1.0, 0.0]))
        shoulder_lateral = normalize(left_shoulder - right_shoulder, lateral)
        up = normalize(spine - pelvis, np.array([0.0, 0.0, 1.0]))
        forward = normalize(np.cross(lateral, up), np.array([1.0, 0.0, 0.0]))

        set_orientation("Pelvis", quat_from_primary_axis(forward, up, primary_axis))
        for name in ("Spine1", "Spine2", "Spine3", "Neck", "Head", "Left_Collar", "Right_Collar"):
            set_orientation(name, quat_from_primary_axis(up, shoulder_lateral, primary_axis))

        limb_chains = (
            ("Left_Hip", "Left_Knee", -lateral),
            ("Left_Knee", "Left_Ankle", -lateral),
            ("Left_Ankle", "Left_Foot", forward),
            ("Left_Foot", "Left_Foot", forward),
            ("Right_Hip", "Right_Knee", lateral),
            ("Right_Knee", "Right_Ankle", lateral),
            ("Right_Ankle", "Right_Foot", forward),
            ("Right_Foot", "Right_Foot", forward),
            ("Left_Shoulder", "Left_Elbow", up),
            ("Left_Elbow", "Left_Wrist", up),
            ("Left_Wrist", "Left_Hand", up),
            ("Left_Hand", "Left_Hand", up),
            ("Right_Shoulder", "Right_Elbow", up),
            ("Right_Elbow", "Right_Wrist", up),
            ("Right_Wrist", "Right_Hand", up),
            ("Right_Hand", "Right_Hand", up),
        )
        for proximal, distal, secondary in limb_chains:
            direction = positions[distal] - positions[proximal]
            if proximal == distal:
                continue
            set_orientation(proximal, quat_from_primary_axis(direction, secondary, primary_axis))

        body["Left_Hand"][1] = body["Left_Wrist"][1]
        body["Right_Hand"][1] = body["Right_Wrist"][1]
        body["Left_Foot"][1] = body["Left_Ankle"][1]
        body["Right_Foot"][1] = body["Right_Ankle"][1]
        return body

    def _hand_dict(
        self,
        frame: np.ndarray,
        side_prefix: str,
        mapping: dict[str, int],
    ) -> dict[str, list[list[float]]]:
        return {
            f"{side_prefix}Hand{name}": self._convert_pose(frame[idx], f"{side_prefix}Hand{name}")
            for name, idx in mapping.items()
        }

    def _controller_data(self, frame_index: int) -> dict:
        # Press A once at startup so TWIST2's state machine enters teleop.
        start_pressed = not self.sent_start_signal
        self.sent_start_signal = True
        timestamp_ns = int(self.timestamps[frame_index] * 1e9)
        return {
            "LeftController": {
                "index_trig": 0.0,
                "grip": 0.0,
                "key_one": False,
                "key_two": False,
                "axis": [0.0, 0.0],
                "axis_click": False,
            },
            "RightController": {
                "index_trig": 0.0,
                "grip": 0.0,
                "key_one": start_pressed,
                "key_two": False,
                "axis": [0.0, 0.0],
                "axis_click": False,
            },
            "timestamp": timestamp_ns,
        }

    def get_current_frame(self):
        idx = self._frame_index_for_now()
        return self.get_frame_at_index(idx, auto_start=True)

    def get_frame_at_index(self, idx: int, auto_start: bool = False):
        idx = int(np.clip(idx, 0, len(self.timestamps) - 1))
        frame = self.joint_poses[idx]
        body = self._body_dict(frame)
        left_hand = True, self._hand_dict(frame, "Left", LEFT_HAND_TO_HTS84)
        right_hand = True, self._hand_dict(frame, "Right", RIGHT_HAND_TO_HTS84)
        if auto_start:
            controller = self._controller_data(idx)
        else:
            timestamp_ns = int(self.timestamps[idx] * 1e9)
            controller = {
                "LeftController": {
                    "index_trig": 0.0,
                    "grip": 0.0,
                    "key_one": False,
                    "key_two": False,
                    "axis": [0.0, 0.0],
                    "axis_click": False,
                },
                "RightController": {
                    "index_trig": 0.0,
                    "grip": 0.0,
                    "key_one": False,
                    "key_two": False,
                    "axis": [0.0, 0.0],
                    "axis_click": False,
                },
                "timestamp": timestamp_ns,
            }
        headset = None
        return body, left_hand, right_hand, controller, headset
