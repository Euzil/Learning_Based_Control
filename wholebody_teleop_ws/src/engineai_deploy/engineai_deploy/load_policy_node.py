#!/usr/bin/env python3

import os
import sys
import argparse
import logging
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from engineai_deploy.model.actor_critic_mlp import Policy

def main(args=None):
    rclpy.init()
    
    node = Node("load_policy_node")
    
    # read settings
    node.declare_parameter('task', 'walk')
    node.declare_parameter('model', 'policy.pt')
    task_file = node.get_parameter('task').get_parameter_value().string_value
    model_file = node.get_parameter('model').get_parameter_value().string_value

    # print states
    current_dir = Path.cwd()
    model_path = os.path.join(current_dir, "models", task_file, model_file)
    node.get_logger().warn(f"Loading model: {model_file}")
    node.get_logger().warn(f"Full path: {model_path}")

    # setup    
    try:
        # load model
        policy = Policy(model_path, verbose=True)
        
        # Get Metadata
        metadata = policy.metadata
        policy_default_joint_pos = metadata["default_joint_pos"]
        policy_joint_stiffness = metadata["joint_stiffness"]
        policy_joint_damping = metadata["joint_damping"]
        policy_observations = metadata["observation_names"]
        
        node.get_logger().warn(f"Joint pose: {policy_default_joint_pos}")
        node.get_logger().warn(f"Joint stiffness: {policy_joint_stiffness}")
        node.get_logger().warn(f"Joint damping: {policy_joint_damping}")
        node.get_logger().warn(f"Observation names: {policy_observations}")
        
    except Exception as e:
        node.get_logger().error(f"Exception: {e}")
        return 1
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
            
    return 0

if __name__ == "__main__":
    sys.exit(main())