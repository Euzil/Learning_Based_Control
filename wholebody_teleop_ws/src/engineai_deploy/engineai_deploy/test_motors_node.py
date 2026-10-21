"""
Test script to check motor functionallity.
"""

import sys
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from interface_protocol.msg import JointCommand, ParallelParserType

from engineai_deploy.agent.config_loader import ConfigLoader
from engineai_deploy.agent.pm01_connection import PM01Connection


class PM01Robot:
    def __init__(self, node: Node, config: ConfigLoader, dry_run: bool):
        self.node = node
        self.config = config
        self._dry_run = dry_run
        
        # Robot command message
        self._command = JointCommand()
        self._command.position = list(self.config.joint_default_position)
        self._command.stiffness = list(self.config.joint_default_stiffness)
        self._command.damping = list(self.config.joint_default_damping)
        
        self._num_dof_robot = len(self._command.position)
        self._command.velocity = [0.0] * self._num_dof_robot
        self._command.feed_forward_torque = [0.0] * self._num_dof_robot
        self._command.torque = [0.0] * self._num_dof_robot
        self._command.parallel_parser_type = ParallelParserType.RL_PARSER
        
        # init the connection
        self.connection = PM01Connection()
        self.connection.initialize(node, self.config)

    def is_okay(self) -> bool:
        if self.connection is None:
            return False
        return self.connection.is_connected
    
    def connect(self, node: Node, timeout: Duration = Duration(seconds=-1)) -> bool:
        logger = node.get_logger()

        timeout_sec = timeout.nanoseconds / 1e9
        is_infinite_timeout = timeout_sec < 0.0
        
        logger.info(f"Starting connection routine (Timeout: {'Infinite' if is_infinite_timeout else timeout_sec})...")

        # wait for connection
        start_time = time.monotonic()
        last_log_time = start_time
        while not self.connection.is_connected:
            now = time.monotonic()
            elapsed_sec = now - start_time
            
            if not is_infinite_timeout and elapsed_sec >= timeout_sec:
                err_msg = f"Connection timeout reached after {elapsed_sec:.2f} seconds."
                raise TimeoutError(err_msg)

            rclpy.spin_once(node, timeout_sec=0.01)
            
            if now - last_log_time > 1.0:
                logger.info("Waiting for connection...")
                last_log_time = now
                
        # override command with current states
        self._command.position = self.connection.joint_state.position

        logger.warn("Successfully connected!")
    
    def update(self, target_dof_pos: np.ndarray, target_dof_vel: np.ndarray) -> bool:
        
        if self._num_dof_robot != len(target_dof_pos) or self._num_dof_robot != len(target_dof_vel):
            raise ValueError("Wrong number of target_dof_pos or target_dof_vel")
        
        dof_pos_diff = target_dof_pos - np.array(self._command.position)
        if (np.abs(dof_pos_diff) > 0.1).any():
            raise ValueError("Jump in target_dof_pos detected")
        
        self._command.position = target_dof_pos.tolist()
        self._command.velocity = target_dof_vel.tolist()

        if not self._dry_run:
            self.connection.send_command(self._command)

    @property
    def dof_pos(self) -> np.ndarray:
        return np.array(self.connection.joint_state.position)
        
    @property
    def dof_vel(self) -> np.ndarray:
        return np.array(self.connection.joint_state.velocity)
    
    @property
    def dof_names(self) -> list:
        return self.config.joint_names


class PM01MotorTest(Node):
    def __init__(self, name: str = "motor_test"):
        super().__init__(name)
        
    def initalize(self, config_file: str, dry_run: bool = False):
        
        # Load robot configuration
        try:
            config = ConfigLoader(config_file)
        except Exception:
            msg = f"Failed to load Config from {config_file}"
            self.get_logger().error(msg)
            raise RuntimeError(msg)

        # Initalize the robot
        self.robot = PM01Robot(self, config, dry_run)
        
        # prepare motor test
        self._dt = 1. / 50.
        self._freq = 1.0
        self._ampl = np.pi / 4
        self._joint_names = ["hip_yaw_l_joint", "hip_yaw_r_joint"]
        self._joint_ids = [self.robot.dof_names.index(name) for name in self._joint_names]
        
        # Connect
        try:
            self.robot.connect(self, timeout=Duration(seconds=60))
        except TimeoutError as e:
            self.get_logger().error(f"Failed to connect: {e}")
            raise
        
    def start(self) -> bool:
        
        self._target_pos = self.robot.dof_pos.copy()
        self._target_vel = np.zeros_like(self.robot.dof_pos)
        self._inital_pos = self.robot.dof_pos.copy()
        self._t = 0.0
        
        # Start Control Timer (dt_control)
        self.control_timer = self.create_timer(
            self._dt, 
            self._control_callback
        )
        
    def _control_callback(self):
        for jid in self._joint_ids:
            self._target_pos[jid] = self._inital_pos[jid] + self._ampl * np.sin(2 * np.pi * self._freq * self._t)
        self._t += self._dt
        
        self.robot.update(self._target_pos, self._target_vel)


def main(args=None):
    rclpy.init()
    
    motor_tester = PM01MotorTest("motor_tester_node")
    
    motor_tester.declare_parameter('config', 'config.yaml')
    motor_tester.declare_parameter('dry_run', False)
    
    config_file = motor_tester.get_parameter('config').get_parameter_value().string_value
    dry_run = motor_tester.get_parameter('dry_run').get_parameter_value().bool_value

    try:
        motor_tester.initalize(config_file, dry_run)
    except Exception as e:
        motor_tester.get_logger().error(f"Failed with: {e}")
        motor_tester.destroy_node()
        rclpy.shutdown()
        return 1
        
    motor_tester.start()
    rclpy.spin(motor_tester)
    
    motor_tester.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())