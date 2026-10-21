#!/usr/bin/python3

import logging
import sys
from skin_real.skin_manager import SkinManager

import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init()
    
    node = Node("skin_real_node")    
    
    node.declare_parameter('config', 'config.yaml')
    cfg_file = node.get_parameter('config').get_parameter_value().string_value
    node.get_logger().info(f"Loading config from: {cfg_file}")
    
    manager = SkinManager(node, cfg_file)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Initiating shutdown...")
    finally:
        manager.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
        
if __name__ == "__main__":
    sys.exit(main())