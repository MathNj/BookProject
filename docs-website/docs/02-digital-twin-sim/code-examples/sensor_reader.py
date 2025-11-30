#!/usr/bin/env python3
"""
sensor_reader.py
================

ROS 2 node to subscribe to all three sensors and display their data.

This node demonstrates:
1. Subscribing to LiDAR (LaserScan)
2. Subscribing to depth camera (Image)
3. Subscribing to IMU (Imu)
4. Processing and displaying sensor data
5. Calculating basic metrics (distances, orientation)

Usage:
    1. Launch Gazebo: ros2 launch gazebo_ros gazebo.launch.py world:=simple_world.sdf
    2. Spawn robot: ros2 launch gazebo_ros spawn_model.launch.py model:=robot_sim.urdf
    3. Run this node: python3 sensor_reader.py

Expected output:
    [sensor_reader] LiDAR: 1024 rays, nearest obstacle: 2.5m at 90°
    [sensor_reader] Camera: RGB image 640x480 received
    [sensor_reader] IMU: accel (0.00, 0.00, 9.81) m/s², gyro (0.00, 0.00, 0.00) rad/s
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, Image, Imu
from cv_bridge import CvBridge
import numpy as np
import math

# Color codes for terminal output
GREEN = '\033[92m'
BLUE = '\033[94m'
YELLOW = '\033[93m'
RED = '\033[91m'
RESET = '\033[0m'


class SensorReader(Node):
    """
    Subscribes to all three sensors and displays their data in real-time.
    """

    def __init__(self):
        """Initialize the sensor reader node."""
        super().__init__('sensor_reader')

        self.get_logger().info('Starting sensor reader...')

        # OpenCV bridge for image conversion (ROS 2 → OpenCV)
        self.bridge = CvBridge()

        # Create QoS profile for sensor data (best-effort, lower latency)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10  # Queue size: keep last 10 messages
        )

        # ========================================
        # SENSOR 1: LiDAR Subscription
        # ========================================
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/robot/lidar/scan',
            self.lidar_callback,
            qos_profile
        )
        self.lidar_subscription  # Prevent unused variable warning

        # ========================================
        # SENSOR 2: Camera Subscription
        # ========================================
        self.camera_subscription = self.create_subscription(
            Image,
            '/robot/depth_camera/image_raw',
            self.camera_callback,
            qos_profile
        )
        self.camera_subscription

        # ========================================
        # SENSOR 3: IMU Subscription
        # ========================================
        self.imu_subscription = self.create_subscription(
            Imu,
            '/robot/imu/data',
            self.imu_callback,
            qos_profile
        )
        self.imu_subscription

        # Timer: print summary every 1 second
        self.timer = self.create_timer(1.0, self.print_summary)

        # State tracking
        self.last_lidar = None
        self.last_camera = None
        self.last_imu = None

        self.get_logger().info(f'{GREEN}✅ Sensor reader ready. Listening to topics...{RESET}')

    # ========================================
    # LIDAR CALLBACK
    # ========================================
    def lidar_callback(self, msg: LaserScan):
        """
        Process LaserScan message from LiDAR.

        Args:
            msg: LaserScan message containing:
                - ranges: array of distances (meters)
                - angle_min, angle_max: field of view
                - angle_increment: degrees between rays
        """
        self.last_lidar = msg

        # Filter out invalid readings (inf or nan values)
        valid_ranges = [r for r in msg.ranges if r > 0 and not math.isinf(r) and not math.isnan(r)]

        if not valid_ranges:
            self.get_logger().warn('LiDAR: No valid readings')
            return

        # Calculate statistics
        min_distance = min(valid_ranges)
        max_distance = max(valid_ranges)
        mean_distance = np.mean(valid_ranges)

        # Find angle of nearest obstacle
        min_idx = msg.ranges.index(min_distance) if min_distance in msg.ranges else 0
        angle_rad = msg.angle_min + (min_idx * msg.angle_increment)
        angle_deg = math.degrees(angle_rad)

        # Log in compact format
        self.get_logger().debug(
            f'{BLUE}LiDAR:{RESET} {len(valid_ranges)} rays, '
            f'min={min_distance:.2f}m, max={max_distance:.2f}m, '
            f'mean={mean_distance:.2f}m, nearest at {angle_deg:.1f}°'
        )

    # ========================================
    # CAMERA CALLBACK
    # ========================================
    def camera_callback(self, msg: Image):
        """
        Process Image message from depth camera.

        Args:
            msg: Image message containing:
                - width, height: image dimensions
                - encoding: pixel format (e.g., 'rgb8')
                - data: raw image bytes
        """
        self.last_camera = msg

        self.get_logger().debug(
            f'{YELLOW}Camera:{RESET} {msg.width}x{msg.height}, '
            f'encoding={msg.encoding}, timestamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec}'
        )

        # Note: Full image processing (color detection, etc.) would be done here
        # For now, just track that we received data

    # ========================================
    # IMU CALLBACK
    # ========================================
    def imu_callback(self, msg: Imu):
        """
        Process Imu message from IMU sensor.

        Args:
            msg: Imu message containing:
                - linear_acceleration: (x, y, z) in m/s²
                - angular_velocity: (x, y, z) in rad/s
                - orientation: quaternion (if available)
        """
        self.last_imu = msg

        # Extract linear acceleration
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        # Extract angular velocity (gyroscope)
        wx = msg.angular_velocity.x
        wy = msg.angular_velocity.y
        wz = msg.angular_velocity.z

        # Calculate total acceleration (magnitude)
        total_accel = math.sqrt(ax**2 + ay**2 + az**2)

        # Estimate pitch from gyroscope (simplified: integrate omega_y)
        pitch_estimate = math.degrees(wy * 0.01)  # Rough estimate over 0.01s

        self.get_logger().debug(
            f'{RED}IMU:{RESET} accel=({ax:.2f}, {ay:.2f}, {az:.2f}) m/s², '
            f'gyro=({wx:.4f}, {wy:.4f}, {wz:.4f}) rad/s, '
            f'total_accel={total_accel:.2f}m/s², pitch_est={pitch_estimate:.1f}°'
        )

    # ========================================
    # PERIODIC SUMMARY
    # ========================================
    def print_summary(self):
        """Print sensor status summary every second."""
        lidar_status = '✅' if self.last_lidar else '❌'
        camera_status = '✅' if self.last_camera else '❌'
        imu_status = '✅' if self.last_imu else '❌'

        self.get_logger().info(
            f'Sensor Status: LiDAR {lidar_status} | Camera {camera_status} | IMU {imu_status}'
        )


def main():
    """Entry point for the ROS 2 node."""
    rclpy.init()

    sensor_reader = SensorReader()

    try:
        rclpy.spin(sensor_reader)
    except KeyboardInterrupt:
        print(f'\n{YELLOW}Shutting down sensor reader...{RESET}')
    finally:
        sensor_reader.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
