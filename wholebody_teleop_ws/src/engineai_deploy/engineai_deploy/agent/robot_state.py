from dataclasses import dataclass, field
from typing import Dict, Any
import numpy as np


@dataclass
class SensorState:
    """All raw sensor data in robot/sensor frame. Owned and populated by PM01Agent.

    Adding a new sensor means adding a field here and populating it in
    PM01Agent.read() — nothing else in the pipeline needs to change.
    """
    # IMU
    root_quat_w:    np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0, 1.0]))
    root_ang_vel_b: np.ndarray = field(default_factory=lambda: np.zeros(3))
    root_lin_acc_b: np.ndarray = field(default_factory=lambda: np.zeros(3))
    # Joints (robot order)
    dof_pos: np.ndarray = field(default_factory=lambda: np.zeros(0))
    dof_vel: np.ndarray = field(default_factory=lambda: np.zeros(0))
    # Skin
    skin_infos:    Dict[str, Any] = field(default_factory=dict)
    skin_features: Dict[str, Any] = field(default_factory=dict)
    # FT sensors
    left_ft:  np.ndarray = field(default_factory=lambda: np.zeros(6))
    right_ft: np.ndarray = field(default_factory=lambda: np.zeros(6))
    # Odometry
    odom_pose:  np.ndarray = field(default_factory=lambda: np.zeros(7))
    odom_twist: np.ndarray = field(default_factory=lambda: np.zeros(6))
    # height scan
    height_scan: np.ndarray = field(default_factory=lambda: np.zeros(0))


@dataclass
class RobotState:
    """Policy-facing state. Joints are in policy order (remapped by PolicySkill).
    All sensor data is accessed via the shared SensorState reference.
    """
    # Joints (policy order)
    dof_pos: np.ndarray = field(default_factory=lambda: np.zeros(0))
    dof_vel: np.ndarray = field(default_factory=lambda: np.zeros(0))
    # raw sensor values
    sensors: SensorState = field(default_factory=SensorState)
