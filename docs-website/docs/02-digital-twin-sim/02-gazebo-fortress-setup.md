# Setting Up Gazebo Fortress

Welcome to hands-on simulation! In this chapter, you'll install Gazebo Fortress, understand its architecture, and build your first simulated world.

## What is Gazebo Fortress?

**Gazebo Fortress** (also called **Ignition Gazebo**) is a modern, open-source physics simulator designed for robotics:

| Aspect | Details |
|--------|---------|
| **Physics Engine** | Ignition Physics (deterministic, reproducible) |
| **ROS 2 Integration** | Native via `ros-humble-ros-gz` bridge |
| **Performance** | 1000+ Hz on modern GPU (real-time) |
| **Sensors** | Ray-tracing, OpenGL, plugin-based |
| **License** | Open-source (Apache 2.0) |
| **Platform** | Ubuntu 22.04 LTS (primary) |

### Gazebo Fortress vs Gazebo Classic

| Feature | Fortress | Classic |
|---------|----------|---------|
| **Architecture** | Modern, modular | Legacy, monolithic |
| **ROS 2** | Native support | Bridged |
| **Physics Accuracy** | Better | Good |
| **Plugin System** | Flexible | Limited |
| **Performance** | GPU-optimized | CPU-heavy |
| **Community** | Active development | Mature but stable |
| **Recommendation** | ✅ Use this | Use if needed |

**For this course, we use Gazebo Fortress** because it's the modern standard for ROS 2 robotics.

---

## Installing Gazebo Fortress

### Step 1: Update Package Manager

```bash
sudo apt update
sudo apt upgrade -y
```

### Step 2: Install ROS 2 Humble

If you haven't already, install ROS 2 Humble:

```bash
# Follow official ROS 2 Humble installation
# https://docs.ros.org/en/humble/Installation.html

# Or quick install for Ubuntu 22.04:
sudo apt install ros-humble-desktop
source /opt/ros/humble/setup.bash
```

### Step 3: Install Gazebo Bridge

This is the **official ROS 2 ↔ Gazebo integration**:

```bash
sudo apt install ros-humble-ros-gz
```

**What this installs**:
- Gazebo Fortress simulation engine
- ROS 2 bridge (gazebo_ros package)
- Gazebo plugins for sensors
- Command-line tools (gz sim, etc.)

### Step 4: Verify Installation

```bash
# Check Gazebo version
gazebo --version

# Expected output:
# Gazebo version 7.x.x
```

If you see version 7.x or higher, installation succeeded ✅

### Step 5: Test ROS 2 + Gazebo Integration

Open two terminals:

**Terminal 1** - Launch empty Gazebo world:
```bash
source /opt/ros/humble/setup.bash
ros2 launch gazebo_ros gazebo.launch.py
```

**Terminal 2** - Verify ROS 2 topics are publishing:
```bash
source /opt/ros/humble/setup.bash
ros2 topic list | grep gazebo
```

**Expected output** (you should see topics like):
```
/gazebo/model_list
/gazebo/model_states
/gazebo/physics_properties
```

If topics appear, Gazebo ↔ ROS 2 integration is working ✅

---

## Understanding SDF: Gazebo's World Format

### What is SDF?

**SDF = Simulation Description Format**

SDF is XML-based format for describing Gazebo worlds:
- Physics parameters (gravity, friction, solver)
- Robot models (links, joints, sensors)
- Environment (ground, obstacles, lighting)
- Sensors and actuators

### SDF vs URDF: Key Differences

| Aspect | URDF | SDF |
|--------|------|-----|
| **Purpose** | Robot description for ROS | Complete world description for Gazebo |
| **Scope** | Single robot | Robot + environment + physics |
| **Physics** | No physics data | Full physics parameters |
| **Sensors** | Limited | Full sensor plugin support |
| **Used By** | ROS tools, planners | Gazebo simulator |
| **Relationship** | Defines structure | Defines structure + physics + environment |

### Example Comparison

**URDF** (robot definition):
```xml
<robot name="arm">
  <link name="base"/>
  <link name="shoulder"/>
  <link name="forearm"/>
  <joint name="shoulder_joint" type="revolute">
    <parent link="base"/>
    <child link="shoulder"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
```
✅ What is the robot? (structure)

**SDF** (world definition):
```xml
<sdf version="1.9">
  <world name="my_world">
    <gravity>0 0 -9.81</gravity>
    <physics>
      <max_step_size>0.001</max_step_size>
    </physics>
    <model name="ground_plane">
      <!-- Ground -->
    </model>
    <model name="arm">
      <!-- Robot definition -->
    </model>
  </world>
</sdf>
```
✅ What is the robot? (structure)
✅ Where is it? (world)
✅ What are physics parameters? (simulation)

### Converting URDF to SDF

Gazebo automatically converts URDF → SDF when loading:

```bash
# Gazebo loads URDF and converts internally
ros2 launch gazebo_ros spawn_model.launch.py model:=robot.urdf
```

But for **world definition**, you write SDF directly.

---

## Building Your First Gazebo World

### SDF World Structure

Every Gazebo world has this structure:

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="my_world">
    <!-- Physics engine configuration -->
    <physics>...</physics>

    <!-- Lighting -->
    <light>...</light>

    <!-- Ground plane -->
    <model name="ground_plane">...</model>

    <!-- Obstacles -->
    <model name="obstacle_1">...</model>

    <!-- Robot (spawned separately) -->
  </world>
</sdf>
```

### Key SDF Elements

#### 1. Physics Configuration

Controls how simulation behaves:

```xml
<physics default="0" type="ode">
  <!-- Step size: smaller = more accurate, slower -->
  <max_step_size>0.001</max_step_size>

  <!-- Real-time update rate (Hz) -->
  <real_time_update_rate>1000</real_time_update_rate>

  <!-- Real-time factor (1.0 = real-time, >1.0 = faster) -->
  <real_time_factor>1.0</real_time_factor>
</physics>
```

**What each parameter means**:
- `max_step_size=0.001`: Each physics step = 1ms. With 1000 Hz, achieves real-time
- `real_time_update_rate=1000`: Target 1000 steps per second
- `real_time_factor=1.0`: Simulation time = Wall clock time (1 sec simulation = 1 sec real)

#### 2. Gravity

Sets gravitational acceleration:

```xml
<gravity>0 0 -9.81</gravity>
```

Means: 0 m/s² in X, 0 in Y, -9.81 m/s² in Z (downward)

#### 3. Light

Illuminates the world:

```xml
<light name="sun" type="directional">
  <direction>0.5 0.5 -1</direction>
  <intensity>1</intensity>
  <ambient>0.4 0.4 0.4 1</ambient>
  <diffuse>0.8 0.8 0.8 1</diffuse>
</light>
```

#### 4. Ground Plane

A flat, physics-enabled surface:

```xml
<model name="ground_plane">
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <plane>
          <normal>0 0 1</normal>
          <size>100 100</size>
        </plane>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <plane>
          <normal>0 0 1</normal>
          <size>100 100</size>
        </plane>
      </geometry>
      <material>
        <ambient>0.8 0.8 0.8 1</ambient>
        <diffuse>0.8 0.8 0.8 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

#### 5. Obstacles (Models)

Physical objects in the world:

```xml
<model name="table">
  <pose>1 0 0.4 0 0 0</pose>  <!-- position and orientation -->
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <box>
          <size>1 1 0.8</size>  <!-- width, depth, height -->
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>1 1 0.8</size>
        </box>
      </geometry>
      <material>
        <diffuse>0.5 0.3 0.1 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

---

## Launching Your First World

### Basic Launch (Empty World)

```bash
source /opt/ros/humble/setup.bash
gazebo empty.sdf
```

You should see:
- ✅ Gazebo window opens
- ✅ Grid on ground plane
- ✅ Sky gradient (blue to white)

### Launch Custom World

```bash
gazebo simple_world.sdf
```

You should see:
- ✅ Ground plane
- ✅ Obstacles (table, boxes)
- ✅ Lighting (sun + ambient)
- ✅ Physics enabled (gravity visible)

### Monitor Simulation Performance

In the Gazebo GUI:
1. Look at bottom-right corner
2. Watch **Simulation Hz counter** (should show 1000+ Hz on GPU)
3. If shows **50 Hz**: Running on CPU (acceptable but slow)

**Performance indicators**:
- **1000+ Hz**: GPU physics (excellent, real-time)
- **100-500 Hz**: Mixed load (good, acceptable)
- **50 Hz**: CPU physics (acceptable for learning)
- **less than 20 Hz**: Performance issue (check GPU/CPU usage)

---

## Physics Parameters Explained

### Parameter Tuning

Different robots need different physics settings:

```xml
<!-- Fast robots (e.g., quadrupeds running) -->
<physics type="ode">
  <max_step_size>0.0005</max_step_size>  <!-- Smaller steps for accuracy -->
  <real_time_update_rate>2000</real_time_update_rate>  <!-- 2000 Hz -->
</physics>

<!-- Slow robots (e.g., robotic arms) -->
<physics type="ode">
  <max_step_size>0.002</max_step_size>  <!-- Larger steps, still stable -->
  <real_time_update_rate>500</real_time_update_rate>  <!-- 500 Hz is OK -->
</physics>

<!-- Learning/debugging (acceptable to be slower) -->
<physics type="ode">
  <max_step_size>0.01</max_step_size>  <!-- Large steps -->
  <real_time_update_rate>100</real_time_update_rate>  <!-- 100 Hz (slow but OK) -->
</physics>
```

### Surface Properties: Friction

Objects in world have material properties:

```xml
<collision>
  <surface>
    <friction>
      <ode>
        <mu>0.5</mu>        <!-- Friction coefficient -->
        <mu2>0.5</mu2>
      </ode>
    </friction>
    <contact>
      <ode>
        <soft_cfm>0.0001</soft_cfm>  <!-- Damping -->
      </ode>
    </contact>
  </surface>
</collision>
```

**Friction values**:
- **0.0**: Frictionless (ice)
- **0.3-0.5**: Normal (carpet, concrete)
- **1.0**: High friction (rubber)

---

## Spawning Your Robot

Once Gazebo world is running, spawn the robot:

**Terminal 1** (already running Gazebo):
```bash
ros2 launch gazebo_ros gazebo.launch.py world:=simple_world.sdf
```

**Terminal 2** (spawn robot):
```bash
source /opt/ros/humble/setup.bash
ros2 launch gazebo_ros spawn_model.launch.py -x 0 -y 0 -z 0.1 model:=robot_sim.urdf
```

**What happens**:
1. Gazebo loads world with obstacles
2. Robot spawns at position (0, 0, 0.1)
3. Gravity pulls robot down
4. Robot collides with ground plane
5. Robot is now part of simulation

---

## Troubleshooting Gazebo Launch

### Issue: "gazebo: command not found"

**Solution**: Install Gazebo
```bash
sudo apt install gazebo ros-humble-ros-gz
```

### Issue: Gazebo Window Doesn't Appear

**Solution**: Check graphics drivers
```bash
# Verify GPU is available
nvidia-smi

# Run Gazebo verbose to see errors
gazebo --verbose empty.sdf
```

### Issue: Physics Too Slow (20 Hz instead of 1000 Hz)

**Solution**: Check if GPU is being used
```bash
# Monitor GPU usage
watch -n 1 nvidia-smi

# If Gazebo not using GPU:
# 1. Check CUDA installation: nvcc --version
# 2. Try explicit GPU rendering:
gazebo --render-engine ogre2 simple_world.sdf
```

### Issue: Robot Jittery or Unstable

**Solution**: Adjust physics parameters
```xml
<!-- Make physics more stable (slower) -->
<max_step_size>0.0005</max_step_size>  <!-- Reduce from 0.001 -->
<real_time_update_rate>2000</real_time_update_rate>  <!-- Increase from 1000 -->
```

---

## Next Steps

✅ Gazebo launched and working?

👉 **Next**:
1. Complete the exercises below
2. Proceed to Chapter 4: Adding Sensors to Simulation

### Hands-On Exercise: Modify World

**Goal**: Edit `simple_world.sdf` and change obstacle properties

**Exercise 1: Change obstacle size**
1. Open `code-examples/simple_world.sdf`
2. Find `<size>1 1 1</size>` (obstacle box)
3. Change to `<size>2 1 0.5</size>` (taller, narrower)
4. Save and relaunch: `gazebo simple_world.sdf`
5. Observe new obstacle shape

**Exercise 2: Change ground friction**
1. Find friction value: `<mu>0.5</mu>`
2. Change to `<mu>0.1</mu>` (icy, slippery)
3. Spawn robot and observe: slides more easily
4. Change to `<mu>1.0</mu>` (sticky)
5. Observe: robot grips better

**Exercise 3: Add a third obstacle**
1. Copy entire `<model name="obstacle_1">` block
2. Paste before `</world>`
3. Change name to `obstacle_3`
4. Change position: `<pose>3 0 0.5 0 0 0</pose>`
5. Relaunch and verify new obstacle appears

---

## Chapter Summary

| Concept | Key Takeaway |
|---------|---|
| **Gazebo Fortress** | Modern ROS 2-native physics simulator |
| **SDF Format** | XML-based world + physics description |
| **Physics Configuration** | 0.001s steps @ 1000 Hz = real-time |
| **World Structure** | Physics + lighting + ground + obstacles |
| **GPU Performance** | 1000+ Hz on RTX (real-time) |
| **Troubleshooting** | Check GPU, adjust physics, verify ROS 2 |

✅ Ready to add sensors? Proceed to **Chapter 4**!
