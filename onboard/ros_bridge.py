#!/usr/bin/env python3
"""
File B: ros_bridge.py
Role: ROS Node Hub. 
1. Subscribes to /cmd_vel_bridge -> Forwards via UDP to robot driver.
2. Subscribes to /utlidar/robot_odom -> Relays to /odom_bridge.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry  # Added for Odom handling
import socket
import struct

# Target Address (Must match unitree_driver.py)
TARGET_IP = "127.0.0.1"
TARGET_PORT = 8899

class RosBridge(Node):
    def __init__(self):
        super().__init__('unitree_ros_bridge')
        
        # === Part 1: Downstream (ROS -> UDP) ===
        # Create UDP Socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Subscribe to velocity command
        self.create_subscription(Twist, '/cmd_vel_bridge', self.cmd_callback, 10)
        
        # === Part 2: Upstream (ROS -> ROS Relay) ===
        # Publisher for the new topic
        self.odom_pub = self.create_publisher(Odometry, '/odom_bridge', 10)
        
        # Subscribe to the original LiDAR odom topic
        # Note: 'qos_profile=10' is standard, but some lidar drivers use 'best_effort'.
        # If you don't receive msgs, try changing QOS settings.
        self.create_subscription(Odometry, '/utlidar/robot_odom', self.odom_callback, 10)
        
        self.get_logger().info(f"ROS Bridge Started.")
        self.get_logger().info(f" -> Forwarding /cmd_vel_bridge to UDP {TARGET_IP}:{TARGET_PORT}")
        self.get_logger().info(f" -> Relaying /utlidar/robot_odom to /odom_bridge")

    def cmd_callback(self, msg):
        """
        Handles velocity commands from ROS and sends to Unitree Driver via UDP.
        """
        vx = float(msg.linear.x)
        vyaw = float(msg.angular.z) 
        
        try:
            # Pack data into binary (2 floats)
            data = struct.pack('ff', vx, vyaw)
            self.sock.sendto(data, (TARGET_IP, TARGET_PORT))
        except Exception as e:
            self.get_logger().error(f"Failed to send UDP command: {e}")

    def odom_callback(self, msg):
        """
        Handles Odometry messages and republishes them to the new topic.
        """
        # We can modify the message here if needed (e.g., change frame_id)
        # For now, we just pass it through directly.
        self.odom_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RosBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()