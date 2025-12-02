# Navigation (Nav2) for Humanoids

## Learning Objectives

By the end of this chapter, you will:
- Understand the Nav2 navigation stack (Planner, Controller, Behavior Tree)
- Configure Nav2 parameters for humanoid locomotion (balance, stride)
- Build and test a basic Behavior Tree for autonomous navigation
- Send goal poses to a humanoid robot from RViz
- Test dynamic obstacle avoidance and recovery behaviors

## Prerequisites

- Completed Chapter 1: Isaac Sim Bridge (ROS 2 topics flowing)
- Completed Chapter 3: Visual SLAM (map generation working)
- Nav2 Humble stack installed (`ros2 install nav2-humble`)
- Jetson Orin or RTX 4070 Ti with Docker
- Basic understanding of ROS 2 services and actions

## Concept: Point A to Point B

Navigation is the capstone of robot autonomy. Given:
1. A map (from VSLAM)
2. A current location (odometry)
3. A goal location

The navigation stack must answer: **"What is the safest, fastest path?"**

**Nav2** (Navigation 2) is the ROS 2 standard for autonomous navigation. It combines:

- **Global Planner**: Finds the overall path (Dijkstra, RRT*, etc.)
- **Local Planner**: Adjusts the path to avoid dynamic obstacles (DWA, TEB, etc.)
- **Controller**: Translates planned motion into motor commands
- **Behavior Tree**: Orchestrates recovery when stuck

### Nav2 Architecture

```
Goal Pose (RViz)
    ↓
┌─────────────────────────────────────────────┐
│         Behavior Tree Executor              │
│  ┌─────────────────────────────────────┐   │
│  │ Goal → Plan Path → Follow Path      │   │
│  │         ↓           ↓               │   │
│  │      Global Planner Local Planner   │   │
│  │         (Dijkstra)    (DWA)         │   │
│  └─────────────────────────────────────┘   │
└─────────────────────────────────────────────┘
    ↓
 Motor Commands
    ↓
┌──────────────┐
│   Robot      │
│  (Humanoid)  │
└──────────────┘
```

### Key Concepts

1. **Costmap**: A grid where each cell represents navigable space
   - Free space: 0 (white)
   - Obstacles: 100 (black)
   - Inflation: 50-99 (safety margin around obstacles)

2. **Planner**: Finds a path through free space
   - Global planner: Dijkstra, A*, RRT*
   - Local planner: DWA, TEB, regulated pure pursuit

3. **Controller**: Tracks the planned path
   - Outputs linear and angular velocities
   - Accounts for robot dynamics (max acceleration, turning radius)

4. **Behavior Tree**: Decision logic
   - If stuck, try recovery behaviors (spin, backup, clear costmap)
   - If path clear, follow planned route
   - If goal reached, succeed

5. **Humanoid-Specific**: Bipedal locomotion constraints
   - Max stride length (walking vs running)
   - Balance constraints (COM over support polygon)
   - Step frequency limits (5–10 steps/sec for humanoids)

## Nav2 Configuration for Humanoids

Humanoid robots have unique constraints compared to wheeled robots:

| Parameter | Wheeled Robot | Humanoid Robot |
|-----------|--------------|---|
| **Robot Radius** | 0.2 m | 0.3 m (shoulder width) |
| **Max Velocity** | 1.0 m/s | 0.5 m/s (walking speed) |
| **Max Acceleration** | 2.0 m/s² | 0.5 m/s² (balance-limited) |
| **Min Turning Radius** | 0.5 m | 1.0 m (avoid toppling) |
| **Footstep Frequency** | N/A | 2 Hz (max) |
| **Recovery Behaviors** | Spin, backup | Side shuffle, step back carefully |

### Step 1: Create nav2_params.yaml

Save as `docs/03-robot-brain/configs/nav2_params.yaml`:

```yaml
# Nav2 Configuration for Humanoid Robots (Jetson Orin)

amcl:
  ros__parameters:
    use_sim_time: false
    alpha1: 0.2                # Odometry rotation noise
    alpha2: 0.2                # Odometry translation noise
    alpha3: 0.2                # Odometry strafe noise
    alpha4: 0.2                # Odometry drift noise
    alpha5: 0.2                # Odometry drift noise
    base_frame_id: "base_link"
    global_frame_id: "map"
    odom_frame_id: "odom"
    z_max: 2.0
    z_min: 0.05
    z_hit: 0.95
    z_short: 0.1
    z_max_prob: 0.05
    sigma_hit: 0.2
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    particle_count: 400
    max_beams: 60
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.99
    recovery_alpha_fast_factor: 0.0
    recovery_alpha_slow_factor: 0.001
    initial_pose_x: 0.0
    initial_pose_y: 0.0
    initial_pose_a: 0.0
    always_transform_to_odom: false
    do_beamskip: false
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3

bt_navigator:
  ros__parameters:
    use_sim_time: false
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: true
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    blackboard:
      initial_queue_size: 1000
    plugins:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_compute_path_through_poses_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_assisted_teleop_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_drive_on_heading_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_is_stuck_condition_bt_node
      - nav2_goal_reached_condition_bt_node
      - nav2_goal_updated_condition_bt_node
      - nav2_initial_pose_received_condition_bt_node
      - nav2_reinitialize_global_localization_service_bt_node
      - nav2_rate_controller_bt_node
      - nav2_distance_controller_bt_node
      - nav2_speed_controller_bt_node
      - nav2_truncate_path_action_bt_node
      - nav2_truncate_path_local_action_bt_node
      - nav2_goal_updater_node_bt_node
      - nav2_recovery_node_bt_node
      - nav2_pipeline_sequence_bt_node
      - nav2_round_robin_node_bt_node
      - nav2_sequence_bt_node
      - nav2_parallel_bt_node
      - nav2_selector_bt_node
      - nav2_condition_check_bool_bt_node

controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 20.0      # 20 Hz control rate (important for humanoids)
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_core::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    general_goal_checker:
      stateful: true
      plugin: "nav2_core::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25

    FollowPath:
      plugin: "teb_local_planner::TebLocalPlannerROS"  # TEB for humanoids (balance-aware)

      # Humanoid-specific constraints
      max_vel_x: 0.5              # Max walking speed (m/s)
      max_vel_x_backwards: 0.2    # Max backstepping (m/s)
      max_vel_theta: 1.0          # Max angular velocity (rad/s)
      acc_lim_x: 0.5              # Max acceleration (m/s²) - lower for balance
      acc_lim_theta: 1.0          # Max angular acceleration (rad/s²)

      # Humanoid footstep parameters
      dt_ref: 0.1                 # Time horizon (seconds)
      dt_hysteresis: 0.1
      min_turning_radius: 1.0     # Avoid tight turns
      wheelbase: 0.6              # Distance between "feet" (meters)
      cmd_angle_instead_rotvel: false

      # Trajectory optimization
      weight_max_vel_x: 2.0
      weight_max_vel_theta: 1.0
      weight_acc_lim_x: 1.0
      weight_acc_lim_theta: 1.0
      weight_kinematics_nh: 1000.0
      weight_kinematics_forward_drive: 1.0
      weight_kinematics_turning_radius: 50.0
      weight_optimaltime: 1.0
      weight_shortest_path: 0.0
      weight_obstacle: 50.0
      weight_inflation: 0.2
      weight_dynamic_obstacle: 0.0
      weight_dynamic_obstacle_inflation: 0.2
      weight_vmaximization: 0.4

      # Humanoid-specific tweaks
      include_costmap_obstacles: true
      legacy_obstacle_association: false
      obstacle_poses_affected: 30
      costmap_obstacles_behind_robot_dist: 1.5
      inflation_dist: 0.3         # 30 cm safety margin
      max_global_plan_lookahead_dist: 0.5
      force_reinit_new_goal: false
      free_goal_vel: false
      complete_global_plan: true

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: false
    planner_plugins: ["GridBased"]

    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      use_rviz: true
      tolerance: 0.5
      use_astar: false             # Use Dijkstra (more reliable for tight spaces)
      allow_unknown: false
      default_queue_size: 0        # 0 = unlimited

smoother_server:
  ros__parameters:
    use_sim_time: false
    smoother_plugins: ["simple_smoother"]

    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 0.3
      max_its: 1000

behavior_server:
  ros__parameters:
    use_sim_time: false
    behavior_plugins:
      - nav2_spin_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_assisted_teleop_action_bt_node
      - nav2_drive_on_heading_bt_node
      - nav2_wait_action_bt_node

    spin:
      plugin: "nav2_behaviors::Spin"
      max_rotational_vel: 1.0
      min_rotational_vel: 0.4
      rotational_acc_lim: 3.2

    back_up:
      plugin: "nav2_behaviors::BackUp"
      max_backups: 1
      backup_dist: 0.5
      max_backup_vel: 0.2

    assisted_teleop:
      plugin: "nav2_behaviors::AssistedTeleop"
      max_vel_x: 0.5
      max_vel_y: 0.0

    drive_on_heading:
      plugin: "nav2_behaviors::DriveOnHeading"
      max_vel_x: 0.5
      max_vel_y: 0.0

    wait:
      plugin: "nav2_behaviors::Wait"
      wait_duration: 2000

costmap_filter_info_server:
  ros__parameters:
    use_sim_time: false
    type: "binary"
    filter_info_topic: "/filter_mask"
    mask_topic: "/mask"

map_server:
  ros__parameters:
    use_sim_time: false
    yaml_filename: "map.yaml"      # VSLAM-generated map
    topic_name: "map"
    frame_id: "map"
    publish_map_intra_process: false

recoveries_server:
  ros__parameters:
    use_sim_time: false
    recovery_plugins: ["spin", "backup"]
    global_frame: "map"
    robot_base_frame: "base_link"
    transform_timeout: 0.1
    tf_publish_period: 0.01
    costmap_min_size: 2
    padding_scale: 1.1
    footprint_padding: 0.01

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 5.0       # 5 Hz updates (humanoid doesn't need faster)
      publish_frequency: 2.0
      global_frame: "map"
      robot_base_frame: "base_link"
      use_sim_time: false
      robot_radius: 0.3           # Humanoid shoulder width (~30 cm)
      resolution: 0.05            # 5 cm grid cells
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: depth
        depth:
          topic: "/camera/depth/image_rect_raw"
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 2.0
        inflation_radius: 0.3      # 30 cm inflation radius

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 10.0      # 10 Hz local updates
      publish_frequency: 2.0
      global_frame: "odom"
      robot_base_frame: "base_link"
      use_sim_time: false
      rolling_window: true
      width: 10                   # 10 m × 10 m local window
      height: 10
      resolution: 0.05            # 5 cm cells
      robot_radius: 0.3
      plugins: ["voxel_layer", "inflation_layer"]

      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: true
        publish_voxel_map: false
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: depth
        depth:
          topic: "/camera/depth/image_rect_raw"
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 2.0
        inflation_radius: 0.3

# Frame/TF parameters
frame_prefix: ""
map_frame: "map"
odom_frame: "odom"
base_frame: "base_link"
scan_topic: "scan"
```

### Step 2: Create a Behavior Tree for Navigation

A **Behavior Tree** is a decision flowchart that controls robot behavior. Create this XML file:

**File**: `docs/03-robot-brain/configs/behavior_tree.xml`

```xml
<?xml version="1.0"?>
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="Root">
      <!-- Initialize -->
      <Condition ID="InitialPoseReceived" />

      <!-- Main Navigation Loop -->
      <Parallel failure_policy="on_one_failure">
        <!-- Check if new goal has arrived -->
        <CheckNewGoal />

        <!-- Navigation sequence -->
        <Sequence>
          <!-- Compute global path -->
          <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased" />

          <!-- Follow path with local planning -->
          <FollowPath path="{path}" controller_id="FollowPath" />

          <!-- Check if goal reached -->
          <IsGoalReached goal="{goal}" />
        </Sequence>
      </Parallel>

      <!-- Success -->
      <Success />
    </Sequence>
  </BehaviorTree>

  <!-- Recovery Behaviors (when stuck) -->
  <BehaviorTree ID="RecoveryTree">
    <Fallback name="Recovery">
      <!-- Try spinning in place -->
      <Spin spin_dist="1.57" max_rotational_vel="1.0" />

      <!-- Try backing up -->
      <BackUp backup_dist="0.5" backup_vel="0.2" />

      <!-- Clear costmap and retry -->
      <ClearCostmap />
    </Fallback>
  </BehaviorTree>

  <!-- Node Definitions -->
  <TreeNodesModel>
    <!-- Conditions -->
    <Condition ID="InitialPoseReceived" />
    <Condition ID="CheckNewGoal" />
    <Condition ID="IsGoalReached" goal="{goal}" />

    <!-- Actions -->
    <Action ID="ComputePathToPose" goal="{goal}" path="{path}" planner_id="GridBased" />
    <Action ID="FollowPath" path="{path}" controller_id="FollowPath" />
    <Action ID="Spin" spin_dist="1.57" max_rotational_vel="1.0" />
    <Action ID="BackUp" backup_dist="0.5" backup_vel="0.2" />
    <Action ID="ClearCostmap" />

    <!-- Flow Control -->
    <Control ID="Sequence" />
    <Control ID="Fallback" />
    <Control ID="Parallel" failure_policy="on_one_failure" />
  </TreeNodesModel>
</root>
```

### Step 3: Launch Nav2 Stack

**File**: `docs/03-robot-brain/code-examples/nav2_launch.py`

```python
#!/usr/bin/env python3
"""
nav2_launch.py - Launch Nav2 navigation stack
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """Generate Nav2 launch description."""

    # Declare arguments
    params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            os.path.dirname(__file__), 'nav2_params.yaml'
        ),
        description='Nav2 parameters file'
    )

    bt_xml_file = DeclareLaunchArgument(
        'bt_xml_file',
        default_value=os.path.join(
            os.path.dirname(__file__), 'behavior_tree.xml'
        ),
        description='Behavior Tree XML file'
    )

    return LaunchDescription([
        params_file,
        bt_xml_file,

        # Include main Nav2 bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'bringup_launch.py'
                ])
            ),
            launch_arguments=[
                ('params_file', LaunchConfiguration('params_file')),
                ('bt_xml_file', LaunchConfiguration('bt_xml_file')),
                ('use_sim_time', 'false'),
            ]
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()
```

Launch it:
```bash
ros2 launch nav2_launch.py params_file:=/path/to/nav2_params.yaml
```

### Step 4: Send Goals from RViz

1. Open RViz:
   ```bash
   rviz2 -d nav2_rviz_config.rviz
   ```

2. Load the VSLAM map (if not already loaded):
   - **Map topic**: `/map`

3. Use the "Nav2 Goal" button in RViz:
   - Click the "Nav2 Goal" tool (top toolbar)
   - Click on the map where you want the robot to go
   - The planner computes a path and the robot follows it

4. Watch the robot move autonomously!

## Code Snippet: Sending Goals Programmatically

**File**: `docs/03-robot-brain/code-examples/send_goal.py`

```python
#!/usr/bin/env python3
"""
send_goal.py - Send navigation goal to Nav2
"""
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from tf2_geometry_msgs import do_transform_pose
import tf2_ros

class GoalSender(Node):
    def __init__(self):
        super().__init__('goal_sender')
        self.nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x: float, y: float, theta: float = 0.0):
        """
        Send a goal pose to Nav2.

        Args:
            x: X position (meters)
            y: Y position (meters)
            theta: Yaw angle (radians)
        """
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        import math
        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)
        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        # Send goal asynchronously
        self.nav2_client.wait_for_server()
        future = self.nav2_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

        self.get_logger().info(f"Sent goal: ({x}, {y})")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Navigation finished!')

def main(args=None):
    rclpy.init(args=args)
    sender = GoalSender()

    # Send goal: (x=2.0, y=3.0, theta=0)
    sender.send_goal(2.0, 3.0, 0.0)

    rclpy.spin(sender)
    sender.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Run it:
```bash
python3 send_goal.py
```

## Performance Tuning for Humanoids

### Critical Parameters

**Max Velocity**: Balance vs speed
- Too fast (>0.8 m/s): Robot topples, takes wide turns
- Too slow (&lt;0.3 m/s): Inefficient movement
- **Recommended for humanoids**: 0.4–0.6 m/s

**Inflation Radius**: Safety margin around obstacles
- Too small (&lt;0.2 m): Collisions
- Too large (>0.5 m): Paths infeasible in tight spaces
- **Recommended for humanoids**: 0.3 m (30 cm)

**Controller Frequency**: Update rate for motor commands
- Too low (&lt;10 Hz): Jerky, unstable motion
- Too high (>50 Hz): Unnecessary CPU load
- **Recommended for humanoids**: 20 Hz

## Troubleshooting

### Problem: Robot doesn't move (stuck planning)

**Cause**: Planner can't find a path (start/goal in obstacles).

**Solution**:
```bash
# Check costmap
ros2 topic echo /global_costmap/costmap  # Should show map with obstacles

# Clear costmaps
ros2 service call /clear_entire_global_costmap nav2_msgs/srv/ClearEntireCostmap {}

# Verify odometry
ros2 topic echo /odom  # Should show position updates
```

### Problem: Robot takes inefficient paths

**Cause**: Costmap inflation too large or local planner conservative.

**Solutions**:
1. Reduce `inflation_radius` in nav2_params.yaml
2. Increase `weight_shortest_path` in TEB controller
3. Reduce `min_turning_radius` if safe

### Problem: Robot oscillates or spins in circles

**Cause**: TEB controller parameters unbalanced.

**Solution**: Lower `weight_kinematics_forward_drive` or increase `max_vel_x`.

## Key Takeaways

✅ **Nav2** provides production-grade autonomous navigation
✅ **Behavior Trees** enable flexible, recoverable navigation logic
✅ **Humanoid tuning** requires balance-aware parameter choices
✅ **TEB planner** optimizes trajectories for humanoid dynamics
✅ **Real-time RViz feedback** accelerates debugging and tuning

## Estimated Completion Time

- **Reading**: 45–50 minutes
- **Configuration**: 20 minutes
- **Hands-on lab** (launch + send goals + test): 40–50 minutes
- **Troubleshooting**: 20 minutes
- **Total**: 125–150 minutes

## Hardware Requirements

| Platform | Status | Notes |
|----------|--------|-------|
| **Jetson Orin + RealSense D435** | ✅ Primary | Full Nav2 stack, 20 Hz updates |
| **RTX 4070 Ti (sim)** | ✅ Development | Fast iteration for params tuning |
| **Jetson Orin NX** | ⚠️ Limited | Reduced costmap resolution, 10 Hz updates |
| **Cloud (Omniverse)** | ✅ Alternative | &lt;$5/hour, full performance |

---

## Module 3 Complete

You have now mastered:
1. ✅ **Isaac Sim Bridge** — Connect simulation to ROS 2
2. ✅ **Synthetic Data** — Generate labeled training datasets in minutes
3. ✅ **Visual SLAM** — Map environments with cameras
4. ✅ **Nav2 Navigation** — Move robots autonomously

**Next steps**:
- Deploy to physical Unitree H1 humanoid robot
- Collect real-world SLAM data and fine-tune model
- Implement multi-robot navigation (fleet management)
- Add manipulation (arm planning) for pick-and-place tasks
