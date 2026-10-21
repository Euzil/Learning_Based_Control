#!/usr/bin/env python3
import os
import sys
import traceback
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from engineai_deploy.runners.multi_policy_runner import MultiPolicyRunner

def main(args=None):
    rclpy.init(args=args)
    node = Node("multi_policy_runner_node")

    # Read settings
    node.declare_parameter('config', 'config.yaml')
    node.declare_parameter('tasks', ['walk', 'run'])
    node.declare_parameter('models', ['walk_policy.onnx', 'run_policy.onnx'])
    node.declare_parameter('default_task', 'walk')
    node.declare_parameter('switch_duration', 1.0)
    node.declare_parameter('warmup_duration', 0.0)
    node.declare_parameter('dry_run', False)
    node.declare_parameter('verbose', False)

    config_file = node.get_parameter('config').get_parameter_value().string_value
    tasks = node.get_parameter('tasks').get_parameter_value().string_array_value
    models = node.get_parameter('models').get_parameter_value().string_array_value
    default_task = node.get_parameter('default_task').get_parameter_value().string_value
    switch_duration = node.get_parameter('switch_duration').get_parameter_value().double_value
    warmup_duration = node.get_parameter('warmup_duration').get_parameter_value().double_value
    is_dry_run = node.get_parameter('dry_run').get_parameter_value().bool_value
    is_verbose = node.get_parameter('verbose').get_parameter_value().bool_value

    # Validate inputs
    if len(tasks) != len(models):
        node.get_logger().error(
            f"Parameter mismatch: Length of tasks ({len(tasks)}) "
            f"must match length of models ({len(models)})."
        )
        node.destroy_node()
        rclpy.shutdown()
        return 1

    # Construct the dictionary of models
    current_dir = Path.cwd()
    model_paths = {}

    for task, model in zip(tasks, models):
        path = os.path.join(current_dir, "models", task, model)
        model_paths[task] = path
        node.get_logger().info(f"Registered skill '{task}' -> {path}")
    node.get_logger().warn(f"Default skill set to: {default_task}")

    # Initialize Runner
    try:
        runner = MultiPolicyRunner(
            node,
            config_file=config_file,
            skill_dict=model_paths,
            default_skill=default_task,
            switch_duration=switch_duration,
            warmup_duration=warmup_duration,
            dry_run=is_dry_run,
            verbose=is_verbose,
        )
    except Exception:
        node.get_logger().error(f"Runner initialization failed:\n{traceback.format_exc()}")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    # start
    try:
        runner.start()
    except Exception:
        node.get_logger().error(f"Runner start failed:\n{traceback.format_exc()}")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    # execute
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down gracefully...")
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    return 0

if __name__ == "__main__":
    sys.exit(main())
