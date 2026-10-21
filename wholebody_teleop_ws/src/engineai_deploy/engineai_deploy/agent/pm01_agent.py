import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from engineai_deploy.agent.config_loader import ConfigLoader
from engineai_deploy.agent.pm01_connection import PM01Connection
from engineai_deploy.agent.control_input import ControlInput
from engineai_deploy.agent.robot_state import SensorState
from engineai_deploy.utils.math import quat_rotate

from interface_protocol.msg import JointCommand, ParallelParserType


class PM01Agent:
    """Hardware interface to the robot. Always speaks in robot-joint order.

    Policy-specific concerns (joint mapping, gains) are handled by
    PolicySkill. Call set_gains() when activating a new skill.
    """
    def __init__(self,
                 node: Node,
                 config: ConfigLoader,
                 is_dry_run: bool = False,
                 verbose: bool = False):
        self._node = node
        self._config = config
        self._is_dry_run = is_dry_run
        self._verbose = verbose
        self._is_stable = True
        self._is_started = False

        self._connection = PM01Connection(node, config, is_dry_run)

        num_dof = len(self._config.joint_names)
        self._sensor_state = SensorState(
            dof_pos=self._config.joint_default_position.copy(),
            dof_vel=np.zeros(num_dof),
        )

        self._input = ControlInput()

        # Command message — initialized with config defaults, gains updated via set_gains()
        self._prev_pos_command = None
        self._command = JointCommand()
        self._command.position = list(self._config.joint_default_position)
        self._command.stiffness = list(self._config.joint_default_stiffness)
        self._command.damping = list(self._config.joint_default_damping)
        self._command.velocity = [0.0] * num_dof
        self._command.feed_forward_torque = [0.0] * num_dof
        self._command.torque = [0.0] * num_dof
        self._command.parallel_parser_type = ParallelParserType.RL_PARSER

    def set_gains(self, stiffness: np.ndarray, damping: np.ndarray):
        """Update joint gains (robot order). Called when a new skill becomes active."""
        self._command.stiffness = [float(v) for v in stiffness]
        self._command.damping = [float(v) for v in damping]

    def is_okay(self) -> bool:
        return self._connection.is_connected and self._is_stable

    @property
    def is_started(self) -> bool:
        return self._is_started

    def connect(self, timeout: Duration = Duration(seconds=-1)) -> bool:
        timeout_sec = timeout.nanoseconds / 1e9
        is_infinite_timeout = timeout_sec < 0.0
        self._node.get_logger().info(
            f"[PM01Agent]: Starting connection routine "
            f"(Timeout: {'Infinite' if is_infinite_timeout else timeout_sec})..."
        )

        start_time = time.monotonic()
        while not self._connection.is_connected:
            elapsed_sec = time.monotonic() - start_time
            if not is_infinite_timeout and elapsed_sec >= timeout_sec:
                raise TimeoutError(
                    f"[PM01Agent]: Connection timeout reached after {elapsed_sec:.2f} seconds."
                )
            rclpy.spin_once(self._node, timeout_sec=0.01)
            missing = self._connection.missing_topics
            self._node.get_logger().warn(
                f"[PM01Agent]: Waiting for connection... Missing topics: {missing}",
                throttle_duration_sec=1.0,
            )

        self._node.get_logger().warn("[PM01Agent]: Successfully connected!")

    def read(self, dt: float):
        """Read all sensor data into SensorState."""
        self._safety_check()
        self._start_stop_check()

        imu_msg = self._connection.imu_info
        if imu_msg:
            self._sensor_state.root_quat_w[:] = [
                imu_msg.quaternion.x,
                imu_msg.quaternion.y,
                imu_msg.quaternion.z,
                imu_msg.quaternion.w,
            ]
            self._sensor_state.root_ang_vel_b[:] = [
                imu_msg.angular_velocity.x,
                imu_msg.angular_velocity.y,
                imu_msg.angular_velocity.z,
            ]
            self._sensor_state.root_lin_acc_b[:] = [
                imu_msg.linear_acceleration.x,
                imu_msg.linear_acceleration.y,
                imu_msg.linear_acceleration.z,
            ]

        joint_state_msg = self._connection.joint_state
        if joint_state_msg:
            self._sensor_state.dof_pos[:] = joint_state_msg.position
            self._sensor_state.dof_vel[:] = joint_state_msg.velocity

        gamepad_msg = self._connection.gamepad_keys
        if gamepad_msg:
            self._input.update(gamepad_msg, dt)

        ft_sensors = self._connection.ft_sensors
        if len(ft_sensors) == 2:
            l = ft_sensors["left_ankle_ft"]   # TODO: don't hardcode the name here
            self._sensor_state.left_ft[:] = [
                l.force.x, l.force.y, l.force.z,
                l.torque.x, l.torque.y, l.torque.z,
            ]
            r = ft_sensors["right_ankle_ft"]
            self._sensor_state.right_ft[:] = [
                r.force.x, r.force.y, r.force.z,
                r.torque.x, r.torque.y, r.torque.z,
            ]

        odom_sensors = self._connection.odom_sensors
        if len(odom_sensors) == 1:
            o = odom_sensors["odom"]
            self._sensor_state.odom_pose[:] = [
                o.pose.pose.position.x,
                o.pose.pose.position.y,
                o.pose.pose.position.z,
                o.pose.pose.orientation.x,
                o.pose.pose.orientation.y,
                o.pose.pose.orientation.z,
                o.pose.pose.orientation.w,
            ]
            self._sensor_state.odom_twist[:] = [
                o.twist.twist.linear.x,
                o.twist.twist.linear.y,
                o.twist.twist.linear.z,
                o.twist.twist.angular.x,
                o.twist.twist.angular.y,
                o.twist.twist.angular.z,
            ]
        
        ray_sensors = self._connection.ray_caster_sensors
        if len(ray_sensors) == 1:
            r = ray_sensors["base_height_scan"]
            self._sensor_state.height_scan = np.ndarray(
                r.distance
            )

        self._sensor_state.skin_infos    = self._connection.skin_infos
        self._sensor_state.skin_features = self._connection.skin_features

    def write(self, pos_robot_order: np.ndarray, vel_robot_order: np.ndarray):
        """Write joint targets in robot order."""
        # check for delta in joint commands
        if not self._command_check(pos_robot_order, vel_robot_order):
            return
        
        # copy to msg
        self._command.position = [float(v) for v in pos_robot_order]
        self._command.velocity = [float(v) for v in vel_robot_order]

        # send
        if self._is_started and not self._is_dry_run:
            self._connection.send_command(self._command)
        elif self._verbose:
            print("[PM01Agent]: --- Commands ---")
            print("[PM01Agent]: position= ", [f"{x:.2f}" for x in self._command.position])
            print("[PM01Agent]: velocity= ", [f"{x:.2f}" for x in self._command.velocity])
            print("[PM01Agent]: torque=   ", [f"{x:.2f}" for x in self._command.torque])
            print("[PM01Agent]: stiffness=", [f"{x:.2f}" for x in self._command.stiffness])
            print("[PM01Agent]: damping=  ", [f"{x:.2f}" for x in self._command.damping])

    @property
    def config(self) -> ConfigLoader:
        return self._config

    @property
    def sensor_state(self) -> SensorState:
        return self._sensor_state

    @property
    def num_dof(self) -> int:
        return len(self._config.joint_names)

    @property
    def joint_names(self) -> list:
        return self._config.joint_names

    @property
    def joint_default_stiffness(self) -> np.ndarray:
        return self._config.joint_default_stiffness.copy()

    @property
    def joint_default_damping(self) -> np.ndarray:
        return self._config.joint_default_damping.copy()

    @property
    def joint_default_pos(self) -> np.ndarray:
        return self._config.joint_default_position.copy()

    @property
    def input(self) -> ControlInput:
        return self._input

    def _safety_check(self):
        # check orientation
        g_b = quat_rotate(self._sensor_state.root_quat_w, np.array([0.0, 0.0, -1.0]))
        tilt = np.linalg.norm(g_b[:2])
        if tilt > self._config.max_imu_tilt:
            self._is_stable = False
            self._node.get_logger().error(
                f"[PM01Agent]: Orientation safety violation imu_tilt={tilt} > {self._config.max_imu_tilt}"
            )
        # check joint velocity
        dof_vel_abs = np.abs(self._sensor_state.dof_vel)
        max_dof_vel = np.max(dof_vel_abs)
        if max_dof_vel > self._config.max_joint_vel:
            self._is_stable = False
            worst_idx = int(np.argmax(dof_vel_abs))
            self._node.get_logger().error(
                f"[PM01Agent]: Joint safety violation — joint '{self._config.joint_names[worst_idx]}' "
                f"vel {max_dof_vel:.4f} rad/s exceeds threshold {self._config.max_joint_delta:.4f} rad/s."
            )
            
    def _command_check(self, pos_robot_order: np.ndarray, vel_robot_order: np.ndarray):
        # check joint commands
        if self._prev_pos_command is None:
            self._prev_pos_command = pos_robot_order.copy()
        
        delta = np.abs(pos_robot_order - self._prev_pos_command)
        max_delta = np.max(delta)
        if max_delta > self._config.max_joint_delta:
            self._is_stable = False
            worst_idx = int(np.argmax(delta))
            self._node.get_logger().error(
                f"[PM01Agent]: Joint safety violation — joint '{self._config.joint_names[worst_idx]}' "
                f"delta {delta[worst_idx]:.4f} rad exceeds threshold {self._config.max_joint_delta:.4f} rad."
            )
            self._node.get_logger().error(f"[PM01Agent] prev_pos_command={self._prev_pos_command}")
            self._node.get_logger().error(f"[PM01Agent] pos_robot_order={pos_robot_order}")
            
        self._prev_pos_command = pos_robot_order.copy()
        return self._is_stable

    def _start_stop_check(self):
        # wait for start
        if not self._is_started:
            self._node.get_logger().warn(
                f"[PM01Agent]: Waiting for start signal",
                throttle_duration_sec=1.0,
            )
        
        # stopping (set is_stable to false to stop completly)
        if self._is_started and self._input.stop:
            self._node.get_logger().error(f"[PM01Agent] User shutdown request")
            self._is_started = False
            self._is_stable = False
        
        # starting (starts sending commands)
        if not self._is_started and self._input.start:
            self._node.get_logger().warn(f"[PM01Agent] User start request")
            self._is_started = True