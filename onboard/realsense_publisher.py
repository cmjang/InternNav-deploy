#!/usr/bin/env python3
"""
RealSense Camera ROS2 Publisher Node with Auto-Reconnect
Use pyrealsense2 to open camera and publish depth and RGB image topics
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
import time

class RealSensePublisher(Node):
    def __init__(self):
        super().__init__('realsense_publisher')
        
        # Declare parameters
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30) 
        self.declare_parameter('enable_depth', True)
        self.declare_parameter('enable_color', True)
        
        # Get parameters
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.enable_depth = self.get_parameter('enable_depth').value
        self.enable_color = self.get_parameter('enable_color').value
        
        # Create publishers
        self.rgb_publisher = self.create_publisher(Image, '/camera/camera/color/image_raw', 10)
        self.depth_publisher = self.create_publisher(Image, '/camera/camera/aligned_depth_to_color/image_raw', 10)
        self.rgb_info_publisher = self.create_publisher(CameraInfo, '/camera/camera/color/camera_info', 10)
        self.depth_info_publisher = self.create_publisher(CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info', 10)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # RealSense objects (Initialize as None)
        self.pipeline = None
        self.config = None
        self.align = None
        self.profile = None
        
        # Intrinsics
        self.color_intrinsics = None
        self.depth_intrinsics = None

        # State flags
        self.is_connected = False
        self.last_reconnect_attempt = 0
        self.reconnect_interval = 2.0 # Try to reconnect every 2 seconds

        # Create timer (Main Loop)
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('RealSense publisher node initialized. Waiting for camera...')

    def start_realsense(self):
        """Attempt to start the RealSense pipeline"""
        try:
            self.get_logger().info('Attempting to connect to RealSense camera...')
            
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            
            # Configure streams
            if self.enable_color:
                self.config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
            if self.enable_depth:
                self.config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
            
            # Start pipeline
            self.profile = self.pipeline.start(self.config)
            
            # Get camera intrinsics
            if self.enable_color:
                color_stream = self.profile.get_stream(rs.stream.color)
                self.color_intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
            if self.enable_depth:
                depth_stream = self.profile.get_stream(rs.stream.depth)
                self.depth_intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()
            
            # Align depth to color
            self.align = rs.align(rs.stream.color)
            
            self.is_connected = True
            self.get_logger().info(f'RealSense connected successfully: {self.width}x{self.height}@{self.fps}fps')
            return True
            
        except Exception as e:
            self.get_logger().warn(f'RealSense connection failed: {e}')
            self.is_connected = False
            if self.pipeline:
                try:
                    self.pipeline.stop()
                except:
                    pass
                self.pipeline = None
            return False

    def stop_realsense(self):
        """Safely stop the pipeline"""
        self.is_connected = False
        if self.pipeline:
            try:
                self.pipeline.stop()
            except Exception as e:
                pass # Ignore errors during stop
            self.pipeline = None
        self.get_logger().warn('RealSense disconnected/stopped.')

    def create_camera_info(self, intrinsics, frame_id):
        """Create CameraInfo message from RealSense intrinsics"""
        info = CameraInfo()
        info.header.frame_id = frame_id
        info.width = intrinsics.width
        info.height = intrinsics.height
        
        # Intrinsic matrix K
        info.k = [
            intrinsics.fx, 0.0, intrinsics.ppx,
            0.0, intrinsics.fy, intrinsics.ppy,
            0.0, 0.0, 1.0
        ]
        
        # Distortion coefficients D
        info.d = list(intrinsics.coeffs)
        
        # Projection matrix P
        info.p = [
            intrinsics.fx, 0.0, intrinsics.ppx, 0.0,
            0.0, intrinsics.fy, intrinsics.ppy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        
        # Rectification matrix R (identity matrix)
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        
        return info

    def timer_callback(self):
        """
        Main loop: 
        1. If disconnected, try to connect.
        2. If connected, try to get frames.
        3. If error occurs, disconnect and wait for next loop.
        """
        
        # --- CASE 1: Reconnection Logic ---
        if not self.is_connected:
            current_time = time.time()
            if current_time - self.last_reconnect_attempt > self.reconnect_interval:
                self.last_reconnect_attempt = current_time
                self.start_realsense()
            return # Skip the rest if not connected

        # --- CASE 2: Normal Operation ---
        try:
            # Wait for frames (timeout is short so we don't block ROS loop too long if camera hangs)
            # 1000ms is standard, but if device is gone, it throws RuntimeError
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            
            # Align depth frame to color frame
            aligned_frames = self.align.process(frames)
            
            # Get current timestamp
            current_time_msg = self.get_clock().now().to_msg()
            
            # Publish color image
            if self.enable_color:
                color_frame = aligned_frames.get_color_frame()
                if color_frame:
                    color_image = np.asanyarray(color_frame.get_data())
                    rgb_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
                    rgb_msg.header.stamp = current_time_msg
                    rgb_msg.header.frame_id = 'camera_color_optical_frame'
                    self.rgb_publisher.publish(rgb_msg)
                    
                    if self.color_intrinsics:
                        color_info = self.create_camera_info(self.color_intrinsics, 'camera_color_optical_frame')
                        color_info.header.stamp = current_time_msg
                        self.rgb_info_publisher.publish(color_info)
            
            # Publish depth image
            if self.enable_depth:
                depth_frame = aligned_frames.get_depth_frame()
                if depth_frame:
                    depth_image = np.asanyarray(depth_frame.get_data())
                    depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding='16UC1')
                    depth_msg.header.stamp = current_time_msg
                    depth_msg.header.frame_id = 'camera_depth_optical_frame'
                    self.depth_publisher.publish(depth_msg)
                    
                    if self.depth_intrinsics:
                        depth_info = self.create_camera_info(self.depth_intrinsics, 'camera_depth_optical_frame')
                        depth_info.header.stamp = current_time_msg
                        self.depth_info_publisher.publish(depth_info)

        except Exception as e:
            # --- CASE 3: Error Handling ---
            self.get_logger().error(f'Error during frame retrieval: {e}')
            self.get_logger().warn('Resetting camera connection...')
            self.stop_realsense()

    def destroy_node(self):
        self.stop_realsense()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = RealSensePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Critical Error: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()