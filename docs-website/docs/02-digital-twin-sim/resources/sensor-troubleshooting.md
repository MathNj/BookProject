# Sensor Troubleshooting Guide

This guide helps diagnose and fix common sensor issues in Gazebo simulation.

## Quick Diagnostic Flowchart

```
Sensor not working?
    ↓
[Step 1] Check if Gazebo is running
    ↓ Yes / No
    │
    ├─→ No: Launch Gazebo
    │      ros2 launch gazebo_ros gazebo.launch.py world:=simple_world.sdf
    │
    └─→ Yes: Proceed to Step 2
            [Step 2] Check if topic exists
            ↓
            ros2 topic list | grep lidar
            ↓ Found / Not Found
            │
            ├─→ Not Found: Check URDF for sensor definition
            │              Verify plugin is installed
            │
            └─→ Found: Proceed to Step 3
                      [Step 3] Check message frequency
                      ↓
                      ros2 topic hz /robot/lidar/scan
                      ↓ High Hz / Low Hz
                      │
                      ├─→ Low Hz: GPU bottleneck, reduce samples
                      │
                      └─→ High Hz: Data quality issue, check noise
```

---

## LIDAR Issues

### Issue 1: No LiDAR Topic (nothing on /robot/lidar/scan)

#### Symptoms
```bash
ros2 topic list
# Output: (no /robot/lidar/scan)
```

#### Diagnosis Steps

**Step 1**: Verify LiDAR is defined in URDF
```bash
grep -A 20 "sensor name=\"lidar\"" code-examples/robot_sim.urdf
# Should output 20 lines of LiDAR configuration
```

**Step 2**: Check Gazebo is actually running
```bash
ps aux | grep gazebo
# Should show "gazebo" process
```

**Step 3**: Check Gazebo logs for errors
```bash
cat /tmp/gazebo.log
# Look for "error" or "lidar" mentions
```

#### Solutions

| Cause | Fix |
|-------|-----|
| **URDF not loaded** | Re-run spawn command with correct path |
| **Gazebo crashed** | Restart: `pkill gazebo` then relaunch |
| **Plugin missing** | Install: `sudo apt install ros-humble-gazebo-ros-pkgs` |
| **Wrong sensor type** | Check: `<sensor type="ray">` (not "lidar") |

#### Fix Steps

1. **Restart Gazebo cleanly**:
   ```bash
   # Terminal 1
   pkill gazebo
   sleep 2
   ros2 launch gazebo_ros gazebo.launch.py world:=simple_world.sdf

   # Terminal 2 (wait 3 seconds, then spawn)
   sleep 3
   ros2 launch gazebo_ros spawn_model.launch.py model:=robot_sim.urdf
   ```

2. **Verify topic appears**:
   ```bash
   # Terminal 3
   ros2 topic list | grep lidar
   # Should show: /robot/lidar/scan
   ```

---

### Issue 2: LiDAR Publishes but Very Slow (1 Hz instead of 10 Hz)

#### Symptoms
```bash
ros2 topic hz /robot/lidar/scan
# Output: frequency: 1.0Hz (should be greater than 10 Hz)
```

#### Root Cause
GPU is bottlenecked by ray-casting computations. Likely causes:
- Too many rays (1024 or greater)
- Other processes consuming GPU
- System overheating (thermal throttling)

#### Diagnosis

**Check GPU usage**:
```bash
nvidia-smi
# Look for "Gazebo" process using GPU
# Should see greater than 50% GPU utilization
# Temperature should be less than 80°C
```

**Check how many rays**:
```bash
grep "samples" code-examples/robot_sim.urdf
# Output: <samples>1024</samples>
```

#### Solutions

| Problem | Solution |
|---------|----------|
| **1024 rays too slow** | Reduce to 512 rays (2x faster) |
| **GPU memory full** | Kill other GPU apps |
| **GPU overheating** | Let cool for 5 minutes, check fans |
| **CPU bottleneck** | Close other applications |

#### Fix Steps

1. **Reduce LiDAR rays**:
   ```bash
   # Edit robot_sim.urdf
   # Find: <samples>1024</samples>
   # Change to: <samples>512</samples>
   # Restart robot spawn
   ```

2. **Verify improved performance**:
   ```bash
   ros2 topic hz /robot/lidar/scan
   # Should now be greater than 20 Hz
   ```

---

### Issue 3: LiDAR Data All Zeros or Infinity

#### Symptoms
```python
import rclpy
from sensor_msgs.msg import LaserScan

# Subscribe and print first 5 rays
msg.ranges[:5]
# Output: [inf, inf, inf, inf, inf]
```

#### Root Cause
- Robot is too close to obstacles (closer than min_range 0.1m)
- Sensor range is misconfigured
- Physics isn't updating (simulation paused)

#### Diagnosis

**Check if Gazebo is paused**:
```bash
# In Gazebo GUI, look for "Paused" indicator
# Or check ROS 2 clock:
ros2 run rosgraph_msgs _clock.msg
# Should see increasing timestamps
```

**Check robot position**:
```bash
# Look in Gazebo GUI where robot spawned
# If robot inside obstacle: move it in GUI
```

#### Solutions

1. **Unpause simulation** (if paused):
   - In Gazebo GUI: Click play button (||)

2. **Move robot away from walls**:
   ```bash
   # Respawn at better position
   ros2 launch gazebo_ros spawn_model.launch.py -x 0.5 -y 0.5 -z 0.1 model:=robot_sim.urdf
   ```

3. **Verify data is valid**:
   ```python
   valid_ranges = [r for r in msg.ranges if r > 0 and not math.isinf(r)]
   print(f"Valid readings: {len(valid_ranges)} out of {len(msg.ranges)}")
   ```

---

## CAMERA Issues

### Issue 1: Camera Topic Not Publishing (/robot/depth_camera/image_raw not found)

#### Symptoms
```bash
ros2 topic list | grep camera
# Output: (nothing, no camera topics)
```

#### Diagnosis Steps

1. **Check camera is in URDF**:
   ```bash
   grep -A 10 "sensor name=\"depth_camera\"" code-examples/robot_sim.urdf
   ```

2. **Check Gazebo loaded sensors**:
   ```bash
   gazebo --verbose simple_world.sdf 2>&1 | grep -i camera
   ```

#### Solutions

| Cause | Fix |
|-------|-----|
| **Sensor not in URDF** | Add camera to forearm link |
| **Camera in wrong frame** | Ensure it's attached to a robot link |
| **Plugin not loaded** | Reinstall gazebo_ros_pkgs |

---

### Issue 2: Camera Image Is Completely Black

#### Symptoms
- RViz shows pitch-black image
- No visible features

#### Root Cause
- Insufficient lighting in world
- Camera clipping planes wrong
- Object too close/far from camera

#### Diagnosis

**Check world has lighting**:
```bash
grep "light" code-examples/simple_world.sdf
# Should have: <light name="sun" type="directional">
```

**Check camera settings**:
```bash
grep -A 5 "clip" code-examples/robot_sim.urdf
# Look for: <near>0.01</near> <far>10.0</far>
```

#### Solutions

1. **Add more light to world**:
   ```xml
   <!-- In simple_world.sdf, after physics -->
   <light name="sun" type="directional">
     <direction>0 0 -1</direction>
     <intensity>1.5</intensity>  <!-- Increase from 1.0 to 1.5 -->
   </light>
   ```

2. **Adjust camera clipping planes**:
   ```xml
   <clip>
     <near>0.001</near>  <!-- Make closer range visible -->
     <far>20.0</far>     <!-- Extend far range -->
   </clip>
   ```

3. **Move robot closer to objects**:
   ```bash
   # Respawn with new position
   ros2 launch gazebo_ros spawn_model.launch.py -x 0 -y 0 -z 0.5 model:=robot_sim.urdf
   ```

---

### Issue 3: Camera Data Laggy (5-10 second delay)

#### Symptoms
- RViz shows old images (delayed by seconds)
- Movement in Gazebo doesn't immediately appear in camera

#### Root Cause
- Camera resolution too high
- Network bandwidth saturated
- CPU can't keep up with image processing

#### Diagnosis

**Check camera frame rate**:
```bash
ros2 topic hz /robot/depth_camera/image_raw
# Should be greater than 15 Hz (not 0.1 Hz)
```

**Check resolution**:
```bash
grep -B 2 -A 2 "width\|height" code-examples/robot_sim.urdf
# Output: <width>640</width> <height>480</height>
```

#### Solutions

1. **Reduce camera resolution**:
   ```xml
   <!-- In robot_sim.urdf -->
   <image>
     <width>320</width>    <!-- down from 640 -->
     <height>240</height>  <!-- down from 480 -->
   </image>
   ```

2. **Restart robot**:
   ```bash
   pkill -f spawn_model
   ros2 launch gazebo_ros spawn_model.launch.py model:=robot_sim.urdf
   ```

3. **Verify improved latency**:
   ```bash
   ros2 topic hz /robot/depth_camera/image_raw
   # Should now be greater than 20 Hz
   ```

---

## IMU Issues

### Issue 1: IMU Topic Exists But No Data (/robot/imu/data shows old timestamps)

#### Symptoms
```bash
ros2 topic hz /robot/imu/data
# Output: no new messages
# Or: old timestamps (several seconds old)
```

#### Root Cause
- IMU only publishes when robot moves/accelerates
- Simulation paused
- IMU not properly initialized

#### Diagnosis

**Check if robot is moving**:
- Look at Gazebo GUI
- Robot should be affected by gravity (falling if not on ground)

**Check simulation time**:
```bash
ros2 topic echo /clock
# Timestamps should be increasing
```

#### Solutions

1. **If simulation is paused**: Click play button in Gazebo GUI

2. **Robot not falling?** Make sure physics is enabled:
   ```xml
   <!-- In simple_world.sdf -->
   <physics type="ode">
     <max_step_size>0.001</max_step_size>
     <real_time_update_rate>1000</real_time_update_rate>
   </physics>
   ```

3. **Force robot motion** (to generate accelerometer readings):
   ```bash
   # Apply a force in RViz
   # Or respawn robot at different position (causes initial acceleration)
   ```

---

### Issue 2: IMU Values All Zero

#### Symptoms
```python
msg.linear_acceleration  # (0, 0, 0)
msg.angular_velocity     # (0, 0, 0)
# Even when robot is falling!
```

#### Root Cause
- IMU plugin not loaded
- Wrong coordinate frame
- Numerical precision issue

#### Diagnosis

**Check if gravity is being applied**:
```xml
<!-- In simple_world.sdf -->
<gravity>0 0 -9.81</gravity>  <!-- Should be non-zero -->
```

**Check IMU noise (might be too high)**:
```bash
grep -A 5 "imu" code-examples/robot_sim.urdf | grep stddev
```

#### Solutions

1. **Verify gravity exists**:
   ```bash
   grep "gravity" code-examples/simple_world.sdf
   # Should show: <gravity>0 0 -9.81</gravity>
   ```

2. **Check IMU has reasonable noise**:
   ```xml
   <stddev>0.002</stddev>  <!-- Should be less than 0.1 -->
   ```

3. **Restart Gazebo**:
   ```bash
   pkill gazebo
   ros2 launch gazebo_ros gazebo.launch.py world:=simple_world.sdf
   ```

---

### Issue 3: IMU Readings Too Noisy

#### Symptoms
```python
# Readings jump randomly even when robot is stationary
ax, ay, az = 0.05, -0.03, 9.85  # Should be 0, 0, 9.81
```

#### Root Cause
- IMU noise is too high
- Simulation timestep too large (unstable physics)

#### Solutions

1. **Reduce IMU noise**:
   ```xml
   <!-- In robot_sim.urdf -->
   <stddev>0.0005</stddev>  <!-- Down from 0.002 -->
   ```

2. **Increase physics update rate**:
   ```xml
   <!-- In simple_world.sdf -->
   <max_step_size>0.0001</max_step_size>  <!-- Down from 0.001 -->
   <real_time_update_rate>10000</real_time_update_rate>  <!-- Up from 1000 -->
   ```

---

## All-Sensors Issues

### Issue 1: GPU Memory Full (CUDA out of memory error)

#### Symptoms
```
RuntimeError: CUDA out of memory. Tried to allocate 256.00 MiB (GPU 0; 11.00 GiB total capacity)
```

#### Root Cause
- Too many sensors
- Sensor resolutions too high
- Too many rays in LiDAR

#### Solutions

1. **Reduce sensor resolutions**:
   ```xml
   <!-- LiDAR: reduce rays -->
   <samples>256</samples>  <!-- down from 1024 -->

   <!-- Camera: reduce resolution -->
   <image>
     <width>160</width>    <!-- down from 640 -->
     <height>120</height>
   </image>
   ```

2. **Check GPU memory**:
   ```bash
   nvidia-smi
   # Should show "Free: XXX MiB" greater than 2000 MiB
   ```

3. **Kill other GPU processes**:
   ```bash
   pkill python3  # Kill other Python scripts
   ```

---

### Issue 2: Sensors Work But Simulation Crashes After 5 Minutes

#### Symptoms
- Sensors publish fine for 5+ minutes
- Then Gazebo crashes
- Error: "Segmentation fault"

#### Root Cause
- Memory leak in sensor plugins
- Unbounded message queue
- GPU memory leaking

#### Solutions

1. **Limit message queue sizes** (in sensor_reader.py):
   ```python
   qos_profile = QoSProfile(depth=5)  # Only keep 5 messages
   ```

2. **Restart Gazebo periodically**:
   ```bash
   # Create a script to restart every 30 minutes
   while true; do
     timeout 1800 gazebo simple_world.sdf
     sleep 5
   done
   ```

---

## Recovery Checklist

If everything breaks, try this in order:

1. **Hard restart Gazebo**:
   ```bash
   pkill -9 gazebo
   pkill -9 gzserver
   pkill -9 gzclient
   sleep 5
   ```

2. **Rebuild URDF (might have syntax errors)**:
   ```bash
   ros2 run urdf_parser urdf_to_graphviz code-examples/robot_sim.urdf
   # Should produce visual diagram, not errors
   ```

3. **Rebuild SDF**:
   ```bash
   gazebo -u code-examples/simple_world.sdf > /tmp/test.sdf 2>&1
   # Check for errors in /tmp/test.sdf
   ```

4. **Minimal startup** (no sensors):
   ```bash
   # Create minimal_robot.urdf with NO sensors
   # Test basic spawning works
   ```

5. **Add sensors back one at a time**:
   - Test LiDAR only
   - Test camera only
   - Test IMU only
   - Then all three

---

## Getting Help

If you're still stuck:

1. **Check ROS 2 logs**:
   ```bash
   cat ~/.ros/log/latest/*/stdout.log
   ```

2. **Check Gazebo logs**:
   ```bash
   gazebo --verbose 2>&1 | head -100
   ```

3. **Ask on ROS Answers**:
   - Include error message
   - Include sensor configuration (relevant XML)
   - Include output of `nvidia-smi` and `ros2 topic hz /topic/name`

4. **Report issue on Gazebo GitHub**:
   - gazebosim/gz-sim (modern Gazebo)
   - osrf/gazebo (classic Gazebo)

