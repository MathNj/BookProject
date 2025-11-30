# Module 2 Code Examples - Quick Start Guide

This directory contains complete, working code examples for Module 2: The Digital Twin. All examples are designed to run on a system with ROS 2 Humble, Gazebo Fortress, and an NVIDIA RTX GPU.

:::info Prerequisites
Before running these examples, complete **Chapter 1: Getting Started** to verify all prerequisites are met.
:::

## Quick Start (5 Steps to First Simulation)

Follow these steps to get your first simulation running in under 10 minutes.

### Step 1: Verify Prerequisites

Open a terminal and run:

```bash
ros2 --version          # Should show ROS 2 Humble
gazebo --version        # Should show Gazebo 7+
nvidia-smi              # Should show your NVIDIA GPU
```

If any command fails, return to **Chapter 1** to install missing software.

### Step 2: Source ROS 2 Environment

```bash
source /opt/ros/humble/setup.bash
```

Add to `~/.bashrc` to make permanent:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Step 3: Navigate to Code Examples

```bash
cd docs-website/docs/02-digital-twin-sim/code-examples
```

### Step 4: Launch Gazebo with Sample World

**Terminal 1** - Launch the world:
```bash
gazebo simple_world.sdf
```

You should see Gazebo window with ground plane and obstacles. This confirms physics simulation is working.

### Step 5: Spawn Robot and Read Sensors

**Terminal 2** - Spawn the robot and start sensor reading:
```bash
# Spawn robot model
ros2 launch gazebo_ros spawn_model.launch.py -x 0 -y 0 -z 0.1 model:=robot_sim.urdf

# In same terminal, after robot spawns, run sensor reader
python3 sensor_reader.py
```

**Expected output**:
```
[INFO] LiDAR detected: 1024 points, mean range: 2.45 m
[INFO] Camera ready: 640x480 RGB + Depth
[INFO] IMU data flowing: accel=[0.0, 0.0, -9.81]
```

✅ **Congratulations!** Your first digital twin is running.

---

## Code Files Overview

### 1. `robot_sim.urdf` (2 KB)

**What it is**: Robot description file in URDF format

**What it contains**:
- 2-joint robot arm (shoulder, elbow)
- 3 links (base, upper arm, forearm) with mass/inertia
- Depth camera at end-effector (640x480, 30 FPS)
- LiDAR at base (1024 rays, 40 m range)
- IMU on main body (accelerometer + gyroscope)
- Full XML comments explaining each section

**What you'll learn**:
- URDF joint and link definitions
- Sensor plugin configuration
- Coordinate frames and transformations

**When it's used**:
- Loaded by Gazebo to define robot structure
- Used with `ros2 launch gazebo_ros spawn_model.launch.py`

**Key sections**:
```xml
<robot name="robot_arm">
  <link name="base"> ... </link>
  <link name="upper_arm"> ... </link>
  <link name="forearm"> ... </link>
  <joint name="shoulder"> ... </joint>
  <joint name="elbow"> ... joint>
  <sensor type="lidar"> ... </sensor>
  <sensor type="rgbd_camera"> ... </sensor>
  <sensor type="imu"> ... </sensor>
</robot>
```

---

### 2. `simple_world.sdf` (1.5 KB)

**What it is**: Gazebo world description in SDF format

**What it contains**:
- Ground plane with physics enabled
- 2 obstacle boxes (table-like structures)
- Lighting (directional light + ambient)
- Physics parameters (gravity = 9.81 m/s², ODE solver)

**What you'll learn**:
- SDF world structure
- Physics engine configuration
- Collision shapes and friction

**When it's used**:
- Launched with `gazebo simple_world.sdf`
- Defines simulation environment before robot spawns

**Key physics parameters**:
```xml
<gravity>0 0 -9.81</gravity>
<magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
<physics name="default_physics" default="0" type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1</real_time_factor>
</physics>
```

---

### 3. `sensor_reader.py` (1.5 KB)

**What it is**: Python ROS 2 node that reads sensor data

**What it contains**:
- Subscriptions to 3 sensor topics:
  - `/robot/lidar/points` (PointCloud2 message)
  - `/robot/camera/rgb/image_raw` (Image message)
  - `/robot/imu/data` (Imu message)
- Callback functions for each sensor
- Data processing (statistics, shape checks)

**What you'll learn**:
- ROS 2 Python client library (rclpy)
- Message types and callbacks
- Data processing in real-time

**How to run**:
```bash
python3 sensor_reader.py
```

**Expected output** (every 1 second):
```
[INFO] LiDAR: 1024 points | mean range: 3.45 m | min/max: 0.1/40.0 m
[INFO] Camera: 640x480 RGB | 640x480 Depth | 30 FPS
[INFO] IMU: accel=[0.0, 0.0, -9.81] m/s² | gyro=[0.0, 0.0, 0.0] rad/s
```

**Key code structure**:
```python
import rclpy
from sensor_msgs.msg import PointCloud2, Image, Imu

class SensorReader(rclpy.Node):
    def __init__(self):
        super().__init__('sensor_reader')
        self.lidar_sub = self.create_subscription(PointCloud2, '/robot/lidar/points', self.lidar_callback, 10)
        self.camera_sub = self.create_subscription(Image, '/robot/camera/rgb/image_raw', self.camera_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/robot/imu/data', self.imu_callback, 10)

    def lidar_callback(self, msg):
        # Process point cloud data
        pass
```

---

### 4. `arm_control.py` (1 KB)

**What it is**: Python ROS 2 control script for robot arm

**What it contains**:
- Joint publisher to send commands to robot arm
- Odometry subscriber to read robot feedback
- Control loop that moves arm in circular pattern
- Performance metrics (loop frequency, latency)

**What you'll learn**:
- Robot control loops
- Command-feedback patterns
- Real-time performance monitoring

**How to run**:
```bash
python3 arm_control.py
```

**Expected output** (every 0.1 seconds):
```
[INFO] Control loop: 100 Hz
[INFO] Shoulder joint: 0.52 rad (29.8°)
[INFO] Elbow joint: 1.05 rad (60.2°)
[INFO] Feedback latency: 5.2 ms
```

**Key control parameters** (editable in file):
```python
SPEED = 1.0              # Rotation speed (rad/s)
CIRCLE_RADIUS = 0.5      # Circle pattern radius (m)
LOOP_FREQ = 100          # Control loop frequency (Hz)
```

---

## Modification Exercises

### Exercise 1: Change World Obstacles

**Goal**: Make obstacles taller and narrower

**Steps**:
1. Open `simple_world.sdf` in text editor
2. Find the obstacle box definitions (`<model name="box"`)
3. Change `<size>1 1 1</size>` to `<size>0.5 0.5 2</size>`
4. Save and relaunch Gazebo
5. Observe the taller obstacles

**Expected**: Obstacles are now half the width but twice as tall

**Learning**: How SDF physics parameters affect simulation

---

### Exercise 2: Add IMU Noise to Robot

**Goal**: Add realistic noise to IMU readings

**Steps**:
1. Open `robot_sim.urdf`
2. Find the `<sensor type="imu">` section
3. Add noise model (from Gazebo docs):
   ```xml
   <noise>
     <type>gaussian</type>
     <mean>0</mean>
     <stddev>0.01</stddev>
   </noise>
   ```
4. Respawn robot and run `sensor_reader.py`
5. Observe IMU values with noise

**Expected**: IMU accelerations fluctuate realistically (±0.01 m/s²)

**Learning**: How to add sensor realism to simulation

---

### Exercise 3: Speed Up Arm Control Loop

**Goal**: Make robot arm move faster

**Steps**:
1. Open `arm_control.py` in text editor
2. Find line: `SPEED = 1.0`
3. Change to: `SPEED = 2.0`
4. Run the script: `python3 arm_control.py`
5. Observe arm completes circle twice as fast

**Expected**: Arm rotates twice per second instead of once per second

**Learning**: How to tune robot control parameters

---

### Exercise 4: Add a Third Obstacle

**Goal**: Add another obstacle to the world

**Steps**:
1. Open `simple_world.sdf`
2. Copy an entire `<model>` block (including opening and closing tags)
3. Paste it at the end, before `</world>`
4. Change name to something unique: `<model name="obstacle_3">`
5. Change position: `<pose>2 2 0.5 0 0 0</pose>` (x, y, z position)
6. Save and relaunch Gazebo
7. Verify new obstacle appears

**Expected**: Third obstacle appears at coordinates (2, 2)

**Learning**: How to modify SDF world descriptions

---

## Troubleshooting

### Issue: Gazebo Launches But No Physics

**Symptom**: Objects don't fall, gravity disabled

**Diagnosis**:
```bash
gazebo --verbose simple_world.sdf | grep -i gravity
```

**Solution**:
1. Check GPU: `nvidia-smi` (if no process, physics using CPU)
2. Try explicit GPU mode:
   ```bash
   gazebo --render-engine ogre2 simple_world.sdf
   ```
3. Check Gazebo performance in GUI (should show 1000 Hz or higher)

---

### Issue: "No sensor data on topic"

**Symptom**: `sensor_reader.py` waits forever for messages

**Diagnosis**:
```bash
ros2 topic list | grep -i robot
ros2 topic list | grep -i sensor
```

**Solution**:
1. Verify robot spawned:
   ```bash
   ros2 launch gazebo_ros spawn_model.launch.py -x 0 -y 0 -z 0.1 model:=robot_sim.urdf
   ```
2. List available topics:
   ```bash
   ros2 topic list
   ```
3. Check sensor plugin names match topic subscriptions in `sensor_reader.py`

---

### Issue: "Gazebo too slow (20 Hz instead of 1000 Hz)"

**Symptom**: Simulation runs but very slowly

**Diagnosis**:
```bash
# Check if GPU is being used
nvidia-smi

# Watch Gazebo Hz counter in GUI
# Should show 1000+ Hz with GPU
```

**Solution**:
- Option 1: Check GPU memory (should be free):
  ```bash
  nvidia-smi -a | grep "Free"
  ```
- Option 2: CPU mode is acceptable but slower:
  ```bash
  # Expected: 50 Hz on modern CPU, 10 Hz on older CPU
  gazebo --render-engine ogre --verbose simple_world.sdf
  ```

---

### Issue: Robot Arm Not Moving

**Symptom**: `arm_control.py` runs but robot doesn't move

**Diagnosis**:
```bash
# Check if command topic exists
ros2 topic list | grep -i "command\|joint"

# Monitor the command topic
ros2 topic echo /robot/arm_commands
```

**Solution**:
1. Verify robot is fully spawned (wait 5 seconds after spawn command)
2. Check topic names in `arm_control.py` match actual topics
3. Verify ROS 2 environment is sourced in all terminals

---

## Performance Baseline

### Expected Performance on NVIDIA RTX 4070 Ti

```
Gazebo physics:       1000 Hz (real-time factor = 1.0)
Sensor rendering:     30+ FPS (smooth visualization)
ROS 2 messaging:      less than 5 ms latency
Control loop:         100 Hz (stable)
GPU memory used:      ~6 GB / 12 GB
Temperature:          65-75°C
```

### Expected Performance on CPU (Intel i7 / AMD Ryzen 7)

```
Gazebo physics:       50 Hz (real-time factor = 0.05)
Sensor rendering:     10 FPS (slower visualization)
ROS 2 messaging:      20-50 ms latency
Control loop:         20 Hz (slower control)
CPU usage:            80-100% (all cores)
Temperature:          70-85°C
```

---

## File Structure

```
code-examples/
├── README.md                    (this file - quick start guide)
├── robot_sim.urdf              (robot definition with sensors)
├── simple_world.sdf            (Gazebo world definition)
├── sensor_reader.py            (Python ROS 2 sensor reader)
├── arm_control.py              (Python ROS 2 control script)
└── rviz_sensor_config.rviz     (RViz visualization config - coming Phase 5)
```

---

## Next Steps

✅ **First simulation working?** Great! Now proceed to:
1. **Chapter 2**: Learn why simulation matters (conceptual)
2. **Chapter 3**: Learn Gazebo in depth (hands-on)
3. **Chapter 4**: Add more sensors (advanced)

❓ **Want to dive deeper?**
- Review ROS 2 documentation: https://docs.ros.org/en/humble/
- Learn Gazebo: https://gazebosim.org/docs
- Explore sensor plugins: https://github.com/gazebosim/gz-sim/tree/main/src/systems/sensors

🚀 **Ready to modify these examples?** Start with Exercise 1 above!
