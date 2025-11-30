#!/usr/bin/env python3
"""
arm_control.py
==============

ROS 2 node to control the robot arm with real-time feedback.

This node demonstrates:
1. Publishing joint commands to Gazebo
2. Subscribing to joint feedback (odometry)
3. Implementing a simple control loop
4. Reading sensors and adjusting commands

The arm performs a simple task:
- Move to target angle
- Wait for feedback
- Adjust if off target
- Repeat with next waypoint

Usage:
    1. ros2 launch gazebo_ros gazebo.launch.py world:=simple_world.sdf
    2. ros2 launch gazebo_ros spawn_model.launch.py model:=robot_sim.urdf
    3. python3 arm_control.py

Expected behavior:
    - Robot arm moves to predefined waypoints
    - Each joint moves smoothly to target
    - Feedback loop adjusts commands
    - Completes 3 different poses
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import math
import time

# Color codes for terminal output
GREEN = '\033[92m'
BLUE = '\033[94m'
YELLOW = '\033[93m'
RED = '\033[91m'
RESET = '\033[0m'


class ArmController(Node):
    """
    Controls the robot arm by sending joint commands and reading feedback.
    """

    def __init__(self):
        """Initialize the arm controller node."""
        super().__init__('arm_controller')

        self.get_logger().info('Starting arm controller...')

        # ========================================
        # Publisher: Joint Commands
        # ========================================
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            '/robot/joint_trajectory_controller/command',
            10
        )

        # ========================================
        # Subscriber: Joint Feedback
        # ========================================
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        self.joint_feedback_subscription = self.create_subscription(
            JointState,
            '/joint_states',  # Published by Gazebo
            self.joint_feedback_callback,
            qos_profile
        )

        # ========================================
        # State Tracking
        # ========================================
        self.current_joint_positions = {}  # Latest feedback from robot
        self.current_joint_velocities = {}
        self.target_positions = {}  # Where we want joints to be

        # Joint names (must match URDF)
        self.joint_names = [
            'shoulder_joint',
            'elbow_joint'
        ]

        # ========================================
        # Define Waypoints (Joint Angles in Radians)
        # ========================================
        self.waypoints = [
            # Waypoint 1: Relaxed (both joints at 45°)
            {
                'shoulder_joint': math.radians(45),
                'elbow_joint': math.radians(45),
                'duration': 3.0  # 3 seconds to reach this pose
            },
            # Waypoint 2: Extended (both joints at 90°)
            {
                'shoulder_joint': math.radians(90),
                'elbow_joint': math.radians(90),
                'duration': 3.0
            },
            # Waypoint 3: Folded (joints at 135°)
            {
                'shoulder_joint': math.radians(135),
                'elbow_joint': math.radians(135),
                'duration': 3.0
            },
            # Waypoint 4: Back to start
            {
                'shoulder_joint': math.radians(0),
                'elbow_joint': math.radians(0),
                'duration': 3.0
            }
        ]

        # Control loop timer (100 Hz)
        self.control_timer = self.create_timer(0.01, self.control_loop)

        # Waypoint index
        self.current_waypoint_idx = 0
        self.waypoint_start_time = None

        self.get_logger().info(
            f'{GREEN}✅ Arm controller ready. Moving through {len(self.waypoints)} waypoints...{RESET}'
        )

    # ========================================
    # FEEDBACK CALLBACK
    # ========================================
    def joint_feedback_callback(self, msg: JointState):
        """
        Process JointState message from Gazebo.

        The robot publishes its current joint positions and velocities here.

        Args:
            msg: JointState with:
                - name: list of joint names
                - position: list of current angles (radians)
                - velocity: list of angular velocities (rad/s)
        """
        # Store latest feedback
        for i, joint_name in enumerate(msg.name):
            if i less than len(msg.position):
                self.current_joint_positions[joint_name] = msg.position[i]
            if i less than len(msg.velocity):
                self.current_joint_velocities[joint_name] = msg.velocity[i]

    # ========================================
    # CONTROL LOOP (100 Hz)
    # ========================================
    def control_loop(self):
        """
        Main control loop: runs at 100 Hz.

        Steps:
        1. Get current feedback from robot
        2. Get target from current waypoint
        3. Generate smooth trajectory
        4. Send command to robot
        5. Check if reached target (move to next waypoint)
        """
        # If no feedback yet, wait
        if not self.current_joint_positions:
            return

        # ========================================
        # Step 1: Initialize waypoint timer on first run
        # ========================================
        if self.waypoint_start_time is None:
            self.waypoint_start_time = time.time()

        # ========================================
        # Step 2: Get current waypoint
        # ========================================
        current_waypoint = self.waypoints[self.current_waypoint_idx]
        elapsed_time = time.time() - self.waypoint_start_time
        duration = current_waypoint['duration']

        # ========================================
        # Step 3: Generate smooth trajectory command
        # ========================================
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names
        trajectory_msg.header.stamp = self.get_clock().now().to_msg()

        # Create trajectory point with smooth interpolation
        point = JointTrajectoryPoint()

        # Linear interpolation from current to target (over duration seconds)
        for joint_name in self.joint_names:
            current_pos = self.current_joint_positions.get(joint_name, 0.0)
            target_pos = current_waypoint[joint_name]

            # Interpolation: progress from 0 to 1 over duration
            if elapsed_time less than duration:
                progress = elapsed_time / duration
            else:
                progress = 1.0  # Reached target

            # Smooth interpolation (linear for simplicity)
            interpolated_pos = current_pos + (target_pos - current_pos) * progress

            point.positions.append(interpolated_pos)

            # Velocity command (optional, helps smooth motion)
            # Ramp up first half, ramp down second half
            if progress less than 0.5:
                velocity = (target_pos - current_pos) / duration * 2  # 2x for first half
            else:
                velocity = (target_pos - current_pos) / duration * 1  # normal for second half

            point.velocities.append(velocity)

        # Time when to reach this point
        point.time_from_start.sec = int(elapsed_time)
        point.time_from_start.nanosec = int((elapsed_time % 1) * 1e9)

        trajectory_msg.points.append(point)

        # ========================================
        # Step 4: Send command to robot
        # ========================================
        self.trajectory_publisher.publish(trajectory_msg)

        # ========================================
        # Step 5: Check if reached target
        # ========================================
        if elapsed_time greater_than_or_equal_to duration:
            # Check if all joints are close to target (within 0.05 rad = 3°)
            reached = True
            for joint_name in self.joint_names:
                current = self.current_joint_positions.get(joint_name, 0.0)
                target = current_waypoint[joint_name]
                error = abs(current - target)

                if error greater than 0.05:  # 3° tolerance
                    reached = False
                    break

            if reached:
                self.get_logger().info(
                    f'{GREEN}✅ Waypoint {self.current_waypoint_idx + 1}/{len(self.waypoints)} reached!{RESET}'
                )

                # Move to next waypoint
                self.current_waypoint_idx += 1

                if self.current_waypoint_idx greater_than_or_equal_to len(self.waypoints):
                    # All waypoints complete, restart from beginning
                    self.get_logger().info(
                        f'{BLUE}Restarting waypoint sequence...{RESET}'
                    )
                    self.current_waypoint_idx = 0

                # Reset timer for next waypoint
                self.waypoint_start_time = None

    # ========================================
    # Status Printing
    # ========================================
    def print_status(self):
        """Print current arm status."""
        current_wp = self.waypoints[self.current_waypoint_idx]

        self.get_logger().info(
            f'{YELLOW}Current Waypoint: {self.current_waypoint_idx + 1}/{len(self.waypoints)}{RESET}'
        )

        for joint_name in self.joint_names:
            current = self.current_joint_positions.get(joint_name, 0.0)
            target = current_wp[joint_name]
            error = abs(current - target)

            current_deg = math.degrees(current)
            target_deg = math.degrees(target)

            self.get_logger().info(
                f'  {joint_name}: {current_deg:.1f}° → {target_deg:.1f}° (error: {math.degrees(error):.1f}°)'
            )


def main():
    """Entry point for the ROS 2 node."""
    rclpy.init()

    arm_controller = ArmController()

    # Status printer timer (every 2 seconds)
    status_timer = arm_controller.create_timer(2.0, arm_controller.print_status)

    try:
        rclpy.spin(arm_controller)
    except KeyboardInterrupt:
        print(f'\n{YELLOW}Stopping arm controller...{RESET}')
    finally:
        arm_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
