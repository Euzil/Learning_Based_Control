import numpy as np

from .commands_base import CommandBase

from interface_protocol.msg import GamepadKeys
from engineai_deploy.agent.control_input import ControlInput

IDX_VX = 0
IDX_VY = 1
IDX_OMEGA = 2
IDX_HEIGHT = 3
IDX_YAW = 4
IDX_PITCH = 5
IDX_ROLL = 6
LOCO_DIM = 7


class ULCCommand(CommandBase):
    def __init__(self, metadata: dict):

        policy_joint_names = list(metadata["joint_names"])
        n_joints = len(policy_joint_names)

        # loco command + limits
        self._loco_cmd = np.zeros(LOCO_DIM)
        self._loco_cmd_lb = np.zeros(LOCO_DIM)
        self._loco_cmd_ub = np.zeros(LOCO_DIM)
        cmd_keys = ["range_lin_vel_x", "range_lin_vel_y", "range_ang_vel_z", "range_height", "range_yaw", "range_pitch", "range_roll"]
        for k, key in enumerate(cmd_keys):
            if key in metadata:
                ranges = metadata[key]
                self._loco_cmd_lb[k] = ranges[0]
                self._loco_cmd_ub[k] = ranges[1]
        self._loco_cmd[IDX_HEIGHT] = metadata["default_height"]

        # mapping arm joints into full joint vector (must be computed before arm ranges)
        arm_joint_names = list(metadata["arm_joint_names"])
        self._upper_ids = np.array([policy_joint_names.index(name) for name in arm_joint_names])

        # joint targets + limits (full joint vector; arm joints get per-joint bounds)
        self._q_target_default = metadata["default_joint_pos"].copy()
        self._q_target = self._q_target_default.copy()
        self._q_target_lb = -100.0 * np.ones(n_joints)
        self._q_target_ub =  100.0 * np.ones(n_joints)
        for k, arm_ranges in enumerate(metadata["arm_joint_ranges"].values()):
            self._q_target_lb[self._upper_ids[k]] = arm_ranges[0]
            self._q_target_ub[self._upper_ids[k]] = arm_ranges[1]

        # output buffer
        self._cmd = np.zeros(LOCO_DIM + n_joints)

    def set_arm_joint_command(self, arm_joint_command: np.ndarray):
        self._q_target[self._upper_ids] = arm_joint_command
        np.clip(self._q_target, self._q_target_lb, self._q_target_ub, out=self._q_target)

    @property
    def loco_command(self) -> np.ndarray:
        """[7] (vx, vy, ωz, height, yaw, pitch, roll)"""
        return self._loco_cmd

    @property
    def arm_targets(self) -> np.ndarray:
        """[N_dof] absolute joint position targets; non-arm joints hold defaults."""
        return self._q_target

    @property
    def arm_delta(self) -> np.ndarray:
        """[N_dof] arm_targets − default_joint_pos; zero for non-arm joints."""
        return self._q_target - self._q_target_default

    def compute(self, input: ControlInput) -> np.ndarray:
        analog = input.analog
        digital = input.digital

        # planar velocity
        joy_raw = np.array([
            analog[GamepadKeys.LEFT_STICK_X],
            analog[GamepadKeys.LEFT_STICK_Y],
            analog[GamepadKeys.RIGHT_STICK_Y],
        ])
        sign = np.sign(joy_raw)
        limit = np.where(sign >= 0, abs(self._loco_cmd_ub[:3]), abs(self._loco_cmd_lb[:3]))
        self._loco_cmd[:3] = joy_raw * limit

        # height / tilt (incremental)
        dt = 1.0 / 50.0
        self._loco_cmd[IDX_HEIGHT] += 0.4 * digital[GamepadKeys.LB] * dt
        self._loco_cmd[IDX_HEIGHT] -= 0.4 * digital[GamepadKeys.RB] * dt
        self._loco_cmd[IDX_YAW] += 0.4 * digital[GamepadKeys.CROSS_Y_LEFT] * dt
        self._loco_cmd[IDX_YAW] -= 0.4 * digital[GamepadKeys.CROSS_Y_RIGHT] * dt
        self._loco_cmd[IDX_PITCH] += 0.4 * digital[GamepadKeys.CROSS_X_UP] * dt
        self._loco_cmd[IDX_PITCH] -= 0.4 * digital[GamepadKeys.CROSS_X_DOWN] * dt
        # self._loco_cmd[IDX_ROLL] += 0.4 * digital[GamepadKeys.CROSS_Y_LEFT] * dt
        # self._loco_cmd[IDX_ROLL] -= 0.4 * digital[GamepadKeys.CROSS_Y_RIGHT] * dt
        
        # np.clip(self._loco_cmd, self._loco_cmd_lb, self._loco_cmd_ub, out=self._loco_cmd)
        
        # print(f"loco={self._loco_cmd}")

        self._cmd[:LOCO_DIM] = self._loco_cmd
        self._cmd[LOCO_DIM:] = self._q_target
        return self._cmd
