# Simulating Sensors in Gazebo

Welcome to hands-on sensor simulation! In this chapter, you'll add realistic sensors to your robot, read sensor data in ROS 2, and visualize it in real-time.

## Why Sensors Matter in Simulation

### The Reality Check

In real robotics, the robot **doesn't just actuate**—it **senses**. A robot arm without sensors can't:
- Detect obstacles (LiDAR)
- See what it's doing (camera)
- Know its orientation (IMU)
- Measure joint angles (encoders)

**Simulation advantage**: Test sensor algorithms before expensive hardware deployment.

| Sensor | Real Hardware Cost | Simulation Cost |
|--------|------------------|-----------------|
| **LiDAR** | $500–$5,000 | Free (GPU ray-tracing) |
| **Depth Camera** | $200–$1,000 | Free (OpenGL rendering) |
| **IMU** | $50–$500 | Free (physics integration) |
| **Encoder** | Built-in, $0 | Free (joint kinematics) |

### Three Types of Sensors in Gazebo

1. **Simulated Sensors** (with plugins): LiDAR, depth camera, camera, IMU, magnetometer
2. **Perfect Sensors** (ideal data): Joint encoders, odometry, ground truth pose
3. **Noisy Sensors** (realistic): Gaussian noise, dropout, lag

**In this chapter**, we'll add all three types to a robot arm.

---

## Understanding Gazebo Sensor Plugins

### URDF vs SDF for Sensors

**URDF** (structural only):
```xml
<sensor name="lidar" type="ray"/>  <!-- Just says "this robot has a LiDAR" -->
```

**SDF** (with simulation physics):
```xml
<sensor type="lidar" name="lidar">
  <lidar>
    <scan>
      <horizontal>
        <samples>1024</samples>  <!-- 1024 rays per scan -->
        <min_angle>-3.14</min_angle>  <!-- -180° -->
        <max_angle>3.14</max_angle>   <!-- +180° -->
      </horizontal>
    </scan>
  </lidar>
  <!-- Gazebo plugin: actually simulates ray-tracing -->
  <plugin name="gazebo_ros_lidar_sensor" filename="libgazebo_ros_lidar_sensor.so">
    <ros>
      <remapping>~/out:=lidar/scan</remapping>  <!-- Publish to /robot/lidar/scan -->
    </ros>
  </plugin>
</sensor>
```

### Sensor Plugin Architecture

When Gazebo runs with a sensor:

```
┌─────────────────────────────────────┐
│  Gazebo Simulation Loop (1000 Hz)   │
├─────────────────────────────────────┤
│ 1. Update physics (gravity, forces) │
│ 2. Run sensor plugins               │
│    ├─ LiDAR: ray-cast from GPU      │
│    ├─ Camera: render to texture     │
│    └─ IMU: read linear accel        │
│ 3. Publish sensor data to ROS 2     │
│    ├─ /robot/lidar/scan             │
│    ├─ /robot/camera/image           │
│    └─ /robot/imu/data               │
└─────────────────────────────────────┘
          ↓ (1000 Hz)
    ┌──────────────┐
    │  ROS 2 Node  │
    │ (your code)  │
    │  Subscribe   │
    │  to topics   │
    └──────────────┘
```

---

## Sensor 1: LiDAR (2D Laser Scanner)

### What is LiDAR?

**LiDAR = Light Detection and Ranging**

- Spins 360° horizontally
- Sends 1024 laser rays per rotation
- Measures distance to obstacles
- Output: PointCloud2 or LaserScan message

### Adding LiDAR to Robot URDF

```xml
<!-- In robot_sim.urdf, at the end of base_link -->
<sensor name="lidar" type="ray">
  <!-- Position: 0.1m above base -->
  <pose>0 0 0.1 0 0 0</pose>

  <!-- Ray scanning parameters -->
  <ray>
    <scan>
      <!-- Horizontal scan: 360° -->
      <horizontal>
        <samples>1024</samples>  <!-- 1024 rays per rotation -->
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>  <!-- -180° -->
        <max_angle>3.14159</max_angle>   <!-- +180° -->
      </horizontal>
    </scan>

    <!-- Range: 0.1m to 40m -->
    <range>
      <min>0.1</min>
      <max>40.0</max>
      <resolution>0.01</resolution>  <!-- 1cm resolution -->
    </range>

    <!-- Gaussian noise: ±1cm std dev -->
    <noise>
      <type>gaussian</type>
      <mean>0</mean>
      <stddev>0.01</stddev>  <!-- 1cm noise -->
    </noise>
  </ray>
</sensor>
```

### Reading LiDAR Data in ROS 2

The sensor publishes `LaserScan` messages to `/robot/lidar/scan`:

```python
import rclpy
from sensor_msgs.msg import LaserScan

class LidarReader:
    def __init__(self):
        self.node = rclpy.create_node('lidar_reader')
        self.subscription = self.node.create_subscription(
            LaserScan,
            '/robot/lidar/scan',
            self.lidar_callback,
            10  # Queue size
        )

    def lidar_callback(self, msg):
        """
        Called when new LaserScan arrives (at 10 Hz typically)
        """
        print(f"LiDAR received {len(msg.ranges)} rays")
        print(f"  Min distance: {min(msg.ranges):.2f}m")
        print(f"  Max distance: {max(msg.ranges):.2f}m")
        print(f"  Field of view: {msg.angle_min:.2f} to {msg.angle_max:.2f} rad")

        # Find nearest obstacle
        nearest_distance = min(msg.ranges)
        nearest_angle = msg.ranges.index(min(msg.ranges))
        print(f"  Nearest obstacle: {nearest_distance:.2f}m at {nearest_angle}°")
```

### LiDAR Parameters Explained

| Parameter | Value | Meaning |
|-----------|-------|---------|
| **samples** | 1024 | Number of rays per full rotation |
| **min_angle** | -3.14159 | Left side (-180°) |
| **max_angle** | 3.14159 | Right side (+180°) |
| **min range** | 0.1m | Don't detect closer than 10cm |
| **max range** | 40m | Can't detect farther than 40m |
| **noise stddev** | 0.01m | ±1cm Gaussian noise |

**Understanding LaserScan message**:
```python
msg.ranges          # Array of 1024 distances (meters)
msg.angle_min       # -3.14159 (radians)
msg.angle_max       # 3.14159 (radians)
msg.angle_increment # 0.00611 (360° / 1024 rays)
msg.time_increment  # Time between rays
msg.scan_time       # Time for full rotation (0.1 sec at 10 Hz)
```

---

## Sensor 2: Depth Camera (RGB-D)

### What is a Depth Camera?

**Depth Camera = RGB image + Depth map**

- Regular camera on top (RGB)
- Infrared depth sensor below (Depth)
- Output: Two separate images (color + distance)

### Adding Depth Camera to Robot URDF

```xml
<!-- In robot_sim.urdf, at end of forearm link -->
<sensor name="depth_camera" type="camera">
  <!-- Position: at end of arm -->
  <pose>0 0 0.25 0 0 0</pose>

  <!-- Camera properties -->
  <camera>
    <!-- Image resolution: 640x480 -->
    <horizontal_fov>1.047</horizontal_fov>  <!-- 60° field of view -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>  <!-- RGB colors -->
    </image>

    <!-- Depth sensing parameters -->
    <depth_camera>
      <output>depths</output>  <!-- Also output depth map -->
    </depth_camera>

    <!-- Clipping planes: don't render outside this range -->
    <clip>
      <near>0.01</near>  <!-- 1cm minimum -->
      <far>10.0</far>    <!-- 10m maximum -->
    </clip>

    <!-- Lens distortion (realistic) -->
    <distortion>
      <k1>-0.25</k1>  <!-- Barrel distortion coefficient -->
    </distortion>
  </camera>

  <!-- Sensor noise: Gaussian -->
  <noise>
    <type>gaussian</type>
    <mean>0</mean>
    <stddev>0.007</stddev>  <!-- 0.7% depth noise -->
  </noise>
</sensor>
```

### Reading Depth Camera Data in ROS 2

Two topics published:
- `/robot/depth_camera/image_raw` — RGB image
- `/robot/depth_camera/depth/image_raw` — Depth map

```python
from sensor_msgs.msg import Image, CameraInfo
import cv2
from cv_bridge import CvBridge

class CameraReader:
    def __init__(self):
        self.node = rclpy.create_node('camera_reader')
        self.bridge = CvBridge()

        # Subscribe to RGB image
        self.node.create_subscription(
            Image,
            '/robot/depth_camera/image_raw',
            self.image_callback,
            10
        )

        # Subscribe to depth map
        self.node.create_subscription(
            Image,
            '/robot/depth_camera/depth/image_raw',
            self.depth_callback,
            10
        )

    def image_callback(self, msg):
        """Convert ROS Image to OpenCV, detect colors"""
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        print(f"RGB image received: {cv_image.shape}")  # (480, 640, 3)

        # Example: detect blue objects
        # (covered in Module 3: Robot Brain)

    def depth_callback(self, msg):
        """Convert depth map, find nearest object"""
        depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")

        # Find minimum depth (nearest obstacle)
        min_depth = depth_image[depth_image greater than 0].min()
        print(f"Nearest obstacle: {min_depth:.2f}m")
```

### Depth Camera Parameters

| Parameter | Value | Meaning |
|-----------|-------|---------|
| **FOV** | 60° | Field of view (horizontal) |
| **Resolution** | 640x480 | Image dimensions (VGA) |
| **Min depth** | 0.01m | Don't measure closer than 1cm |
| **Max depth** | 10m | Can't measure farther than 10m |
| **Depth noise** | 0.7% | Realistic noise model |
| **Distortion** | -0.25 | Barrel distortion (wide lens) |

---

## Sensor 3: IMU (Inertial Measurement Unit)

### What is an IMU?

**IMU = Accelerometer + Gyroscope**

- **Accelerometer**: Measures linear acceleration (m/s²) in 3D
- **Gyroscope**: Measures angular velocity (rad/s) in 3D
- **Output**: 3-axis acceleration + 3-axis rotation rate

### Adding IMU to Robot URDF

```xml
<!-- In robot_sim.urdf, at base_link -->
<sensor name="imu" type="imu">
  <!-- Position: at robot center of mass -->
  <pose>0 0 0.05 0 0 0</pose>

  <!-- IMU properties -->
  <imu>
    <!-- Linear accelerometer noise -->
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0</mean>
          <stddev>0.002</stddev>  <!-- ±0.002 m/s² noise -->
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0</mean>
          <stddev>0.002</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0</mean>
          <stddev>0.002</stddev>
        </noise>
      </z>
    </linear_acceleration>

    <!-- Angular velocity (gyroscope) noise -->
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0</mean>
          <stddev>0.0001</stddev>  <!-- ±0.0001 rad/s noise -->
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0</mean>
          <stddev>0.0001</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0</mean>
          <stddev>0.0001</stddev>
        </noise>
      </z>
    </angular_velocity>
  </imu>
</sensor>
```

### Reading IMU Data in ROS 2

IMU publishes to `/robot/imu/data`:

```python
from sensor_msgs.msg import Imu

class IMUReader:
    def __init__(self):
        self.node = rclpy.create_node('imu_reader')
        self.node.create_subscription(
            Imu,
            '/robot/imu/data',
            self.imu_callback,
            10
        )

    def imu_callback(self, msg):
        """
        Called at 100 Hz (typical for IMU)
        """
        # Linear acceleration (m/s²)
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        # Angular velocity (rad/s)
        wx = msg.angular_velocity.x
        wy = msg.angular_velocity.y
        wz = msg.angular_velocity.z

        print(f"Acceleration: ({ax:.2f}, {ay:.2f}, {az:.2f}) m/s²")
        print(f"Angular vel: ({wx:.4f}, {wy:.4f}, {wz:.4f}) rad/s")

        # Calculate pitch (rotation around Y-axis)
        pitch = wx * 0.01  # Simplified (integrate over 0.01s)
        print(f"Estimated pitch: {pitch:.3f} rad")
```

### IMU Message Structure

```python
msg.linear_acceleration     # (x, y, z) in m/s²
msg.angular_velocity        # (x, y, z) in rad/s
msg.linear_acceleration_covariance    # Uncertainty (3x3 matrix)
msg.angular_velocity_covariance       # Uncertainty (3x3 matrix)
msg.orientation             # Optional: quaternion (from integration)
msg.header.stamp            # Timestamp (for synchronization)
```

---

## Visualizing Sensor Data in RViz

### Launching RViz with Gazebo

**Terminal 1** (Gazebo simulation):
```bash
source /opt/ros/humble/setup.bash
ros2 launch gazebo_ros gazebo.launch.py world:=simple_world.sdf
```

**Terminal 2** (Spawn robot):
```bash
ros2 launch gazebo_ros spawn_model.launch.py model:=robot_sim.urdf
```

**Terminal 3** (RViz visualization):
```bash
rviz2
```

### RViz Configuration for Sensors

In RViz, add displays:

1. **PointCloud2** (for LiDAR):
   - Topic: `/robot/lidar/scan`
   - Color Transformer: Intensity
   - Point Style: Spheres (easier to see)
   - Size: 0.05m

2. **Image** (for camera):
   - Topic: `/robot/depth_camera/image_raw`
   - Window size: 640x480
   - Zoom: 1.0

3. **Image** (for depth):
   - Topic: `/robot/depth_camera/depth/image_raw`
   - Color scheme: grayscale (white = close, black = far)

4. **Imu** (for IMU):
   - Topic: `/robot/imu/data`
   - Accel. Color: Red
   - Gyro. Color: Green

### Saving RViz Configuration

Once configured:
1. In RViz: **File → Save Config**
2. Save as: `code-examples/rviz_sensor_config.rviz`
3. Next time, load: `rviz2 -d code-examples/rviz_sensor_config.rviz`

---

## Complete Example: Sensor Fusion

Combining all three sensors:

```python
import rclpy
from sensor_msgs.msg import LaserScan, Image, Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy

class SensorFusion:
    def __init__(self):
        self.node = rclpy.create_node('sensor_fusion')

        # Use high-reliability QoS for sensor data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Subscribe to all three sensors
        self.node.create_subscription(
            LaserScan, '/robot/lidar/scan',
            self.lidar_callback, qos_profile
        )
        self.node.create_subscription(
            Image, '/robot/depth_camera/image_raw',
            self.camera_callback, qos_profile
        )
        self.node.create_subscription(
            Imu, '/robot/imu/data',
            self.imu_callback, qos_profile
        )

        # State variables
        self.latest_lidar = None
        self.latest_imu = None
        self.fusion_rate = self.node.create_timer(0.1, self.fuse_sensors)

    def lidar_callback(self, msg):
        self.latest_lidar = msg

    def camera_callback(self, msg):
        # Image processing (skipped for brevity)
        pass

    def imu_callback(self, msg):
        self.latest_imu = msg

    def fuse_sensors(self):
        """Called at 10 Hz to fuse sensor data"""
        if self.latest_lidar is None or self.latest_imu is None:
            return

        # Example: Use LiDAR + IMU to estimate position
        # (Covered in Module 3: Sensor Fusion)
        print("Fusing LiDAR + IMU...")
```

---

## Performance Tuning: Sensor Settings

### LiDAR Performance Trade-offs

| Samples | Performance | Accuracy | Use Case |
|---------|-------------|----------|----------|
| **256** | 1000 Hz | Low | Obstacle detection only |
| **512** | 500 Hz | Medium | Safe navigation |
| **1024** | 200 Hz | High | SLAM (mapping) |
| **2048** | 50 Hz | Very high | Research/offline |

### Camera Frame Rate

| FPS | Bandwidth | Latency | Use Case |
|-----|-----------|---------|----------|
| **5** | 5 MB/s | 200ms | Slow monitoring |
| **15** | 15 MB/s | 67ms | Robot grasping |
| **30** | 30 MB/s | 33ms | Real-time navigation |
| **60** | 60 MB/s | 17ms | High-speed control |

### Memory Usage on GPU

```
LiDAR (1024 rays):     ~10 MB memory
Depth camera (640x480): ~50 MB memory
RGB camera (640x480):   ~50 MB memory
────────────────────────────────
Total sensors:          ~110 MB (out of 8-12 GB available)
```

**Recommendation**: All three sensors fit comfortably on NVIDIA RTX GPUs.

---

## Troubleshooting Sensors

### Issue 1: "No messages received on /robot/lidar/scan"

**Diagnosis**:
```bash
# Check if topic exists
ros2 topic list | grep lidar

# Check if plugin loaded
gazebo-server --verbose 2>&1 | grep gazebo_ros_lidar
```

**Solutions**:
1. Verify sensor in URDF exists
2. Ensure plugin name is correct: `libgazebo_ros_lidar_sensor.so`
3. Check Gazebo sim is running: `gazebo simple_world.sdf`

### Issue 2: "Sensor data publishing but very slow (1 Hz instead of 10 Hz)"

**Diagnosis**:
```bash
# Check sensor update rate
ros2 topic hz /robot/lidar/scan

# Check GPU usage
nvidia-smi
```

**Solutions**:
1. **Reduce LiDAR samples**: Change 1024 to 512 rays
2. **Reduce camera resolution**: Change 640x480 to 320x240
3. **Check GPU memory**: `nvidia-smi` should show less than 50% usage

### Issue 3: "Camera image is all black or has artifacts"

**Diagnosis**:
- Check lighting in SDF world file

**Solutions**:
1. Increase ambient light: `<ambient>0.5 0.5 0.5 1</ambient>`
2. Add spotlight near camera: `<light name="work_light" type="spot"/>`
3. Check clipping planes: near=0.01m, far=10m (reasonable?)

### Issue 4: "IMU readings are all zeros"

**Diagnosis**:
```bash
# Check if IMU is static (not moving)
ros2 echo /robot/imu/data
```

**Solutions**:
1. IMU only publishes when robot accelerates
2. If at rest on ground: should show gravity (0, 0, 9.81 m/s²)
3. Check plugin loaded: grep "gazebo_ros_imu" gazebo.log

---

## Next Steps

✅ **Sensors installed and working?**

👉 **Next chapters**:
1. **Chapter 4** (soon): Control the robot from ROS 2
2. **Chapter 5** (optional): Visualize in Unity
3. **Module 3**: Use sensor data for navigation and learning

### Hands-On Exercises

**Exercise 1: Modify LiDAR resolution**
1. Open `code-examples/robot_sim.urdf`
2. Find `<samples>1024</samples>`
3. Change to 256 samples
4. Relaunch and measure: Does 10 Hz improve to 30plus Hz?
5. Restore to 1024 when done

**Exercise 2: Add camera noise**
1. In `robot_sim.urdf`, find depth camera
2. Increase `<stddev>` from 0.007 to 0.05 (5% noise)
3. Observe: Depth readings become noisier
4. Restore original

**Exercise 3: Read all three sensors**
1. Create `code-examples/sensor_fusion.py` (starter code provided)
2. Subscribe to `/robot/lidar/scan`, `/robot/depth_camera/image_raw`, `/robot/imu/data`
3. Print one value from each sensor every 0.5 seconds
4. Verify all three publishing at expected rates

---

## Chapter Summary

| Concept | Key Takeaway |
|---------|------|
| **LiDAR Sensor** | 1024 rays, 360°, distance to obstacles, ray-casting GPU accelerated |
| **Depth Camera** | RGB + depth map, 640x480, realistic lens distortion, edge detection |
| **IMU Sensor** | Accelerometer (m/s²) + gyroscope (rad/s), 100 Hz, orientation estimation |
| **Sensor Plugins** | Gazebo plugins simulate physics and publish ROS 2 messages |
| **RViz Visualization** | Display sensors in real-time, debug algorithm performance |
| **Performance** | LiDAR samples and camera resolution are main GPU/CPU trade-offs |

✅ **Ready to control the robot?** Proceed to **Chapter 4: Robot Control**!
