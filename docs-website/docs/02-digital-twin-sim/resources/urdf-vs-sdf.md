# URDF vs SDF: Understanding Robot Description Formats

This guide explains the differences between URDF (Unified Robot Description Format) and SDF (Simulation Description Format), when to use each, and how they work together.

## Quick Comparison

| Aspect | URDF | SDF |
|--------|------|-----|
| **Full Name** | Unified Robot Description Format | Simulation Description Format |
| **Primary Use** | Robot kinematics & ROS tools | Gazebo simulation |
| **Scope** | Single robot structure | Robot + environment + physics |
| **Physics Data** | ❌ No | ✅ Yes |
| **Sensor Plugins** | ⚠️ Limited | ✅ Full support |
| **File Extension** | .urdf | .sdf |
| **XML Format** | Yes | Yes |
| **Version** | 1.0 (stable) | 1.9 (current) |
| **ROS 2 Tools** | urdf-launch, urdf-parser | -- |
| **Gazebo** | Auto-converts to SDF | Native format |

---

## URDF: Robot Description Format

### What is URDF?

URDF is an **XML format for describing robot kinematics and visual properties**. It defines:
- **Links**: Rigid bodies in the robot (base, arm, wheel)
- **Joints**: Connections between links (revolute, prismatic, fixed)
- **Visual**: How robot looks (meshes, colors)
- **Collision**: How robot collides with environment (collision shapes)
- **Inertial**: Mass distribution (for kinematics calculations)

### Why URDF?

URDF is the **ROS standard** for robot descriptions because:
1. ✅ Lightweight (no physics overhead)
2. ✅ Compatible with planning, visualization, kinematics
3. ✅ Used by all ROS tools (MoveIt, RViz, navigation stack)
4. ✅ Human-readable XML format
5. ✅ Focus on robot structure, not simulation

### URDF Example: 2-Joint Robot Arm

```xml
<?xml version="1.0"?>
<robot name="robot_arm">

  <!-- Link: Base (fixed to ground) -->
  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0"
               iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
    </collision>
  </link>

  <!-- Link: Upper arm -->
  <link name="upper_arm">
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.005" ixy="0" ixz="0"
               iyy="0.005" iyz="0" izz="0.005"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.5" radius="0.03"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.5" radius="0.03"/>
      </geometry>
    </collision>
  </link>

  <!-- Joint: Shoulder (connects base to upper arm) -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>  <!-- Rotate around Z axis -->
    <limit lower="0" upper="3.14159" effort="10" velocity="1.57"/>
  </joint>

  <!-- Link: Forearm -->
  <link name="forearm">
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.003" ixy="0" ixz="0"
               iyy="0.003" iyz="0" izz="0.003"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.5" radius="0.02"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.5" radius="0.02"/>
      </geometry>
    </collision>
  </link>

  <!-- Joint: Elbow (connects upper arm to forearm) -->
  <joint name="elbow_joint" type="revolute">
    <parent link="upper_arm"/>
    <child link="forearm"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="0" upper="3.14159" effort="5" velocity="1.57"/>
  </joint>

</robot>
```

### URDF Key Elements

```xml
<!-- Link: Rigid body -->
<link name="name">
  <inertial>
    <mass value="1.0"/>
    <inertia ixx="..." ixy="..." ixz="..."
             iyy="..." iyz="..." izz="..."/>
  </inertial>
  <visual>
    <geometry>
      <box size="1 1 1"/>  <!-- or sphere, cylinder, mesh -->
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="1 1 1"/>
    </geometry>
  </collision>
</link>

<!-- Joint: Connection between links -->
<joint name="name" type="revolute|prismatic|fixed|...">
  <parent link="parent_name"/>
  <child link="child_name"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="0" upper="1.57" effort="10" velocity="1"/>
</joint>
```

---

## SDF: Simulation Description Format

### What is SDF?

SDF is an **XML format for describing complete simulation worlds**. It defines:
- **Physics**: Gravity, solver parameters, step size
- **Models**: Robots, obstacles, environment
- **Lights**: Illumination (sun, spotlights)
- **Sensors**: Cameras, LiDAR, IMU with plugins
- **Properties**: Friction, damping, restitution

### Why SDF?

SDF is **Gazebo's native format** because:
1. ✅ Complete world description (not just robot)
2. ✅ Built-in physics parameters
3. ✅ Sensor plugin support (ray-tracing, cameras)
4. ✅ Environment configuration
5. ✅ Designed specifically for simulation

### SDF Example: Complete World

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="my_world">

    <!-- Physics engine -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Gravity -->
    <gravity>0 0 -9.81</gravity>

    <!-- Lighting -->
    <light name="sun" type="directional">
      <direction>-0.5 -0.5 -1</direction>
      <intensity>1</intensity>
    </light>

    <!-- Ground plane -->
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
      </link>
    </model>

    <!-- Robot model (includes sensor plugins) -->
    <model name="robot_arm">
      <pose>0 0 0.1 0 0 0</pose>
      <!-- Links, joints from URDF converted to SDF -->
      <link name="base_link">
        <!-- ... -->
      </link>
      <!-- ... -->

      <!-- Sensor plugins (LiDAR, Camera) -->
      <sensor type="lidar" name="lidar">
        <pose>0 0 0.1 0 0 0</pose>
        <lidar>
          <scan>
            <horizontal>
              <samples>1024</samples>
              <resolution>1</resolution>
              <min_angle>-3.14159</min_angle>
              <max_angle>3.14159</max_angle>
            </horizontal>
          </scan>
        </lidar>
      </sensor>

    </model>

  </world>
</sdf>
```

### SDF Key Elements

```xml
<!-- World container -->
<world name="name">
  <physics type="ode|bullet|dart">
    <max_step_size>0.001</max_step_size>
  </physics>
  <gravity>0 0 -9.81</gravity>

  <!-- Model: Robot or object -->
  <model name="name">
    <pose>x y z roll pitch yaw</pose>
    <static>false</static>  <!-- Can move? -->
    <link name="name">
      <!-- Visual, collision, inertial -->
    </link>
    <joint name="name" type="revolute">
      <!-- Joint definition -->
    </joint>
    <!-- Sensors with plugins -->
    <sensor type="lidar" name="name">
      <!-- Sensor parameters -->
    </sensor>
  </model>

</world>
```

---

## How URDF and SDF Work Together

### Workflow: URDF → SDF

When you load a robot in Gazebo:

```
URDF File (robot.urdf)
      ↓
Gazebo loads URDF
      ↓
Internally converts to SDF
      ↓
Adds physics, sensors, plugins
      ↓
SDF simulation in Gazebo
```

### Example Conversion

**URDF Link**:
```xml
<link name="upper_arm">
  <inertial>
    <mass value="0.5"/>
    <inertia ixx="0.005" ixy="0" ixz="0"
             iyy="0.005" iyz="0" izz="0.005"/>
  </inertial>
  <visual>
    <geometry>
      <cylinder length="0.5" radius="0.03"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <cylinder length="0.5" radius="0.03"/>
    </geometry>
  </collision>
</link>
```

**Converted to SDF**:
```xml
<model name="upper_arm">
  <link name="upper_arm">
    <inertial>
      <mass>0.5</mass>
      <inertia>
        <ixx>0.005</ixx>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyy>0.005</iyy>
        <iyz>0</iyz>
        <izz>0.005</izz>
      </inertia>
    </inertial>
    <visual name="visual">
      <geometry>
        <cylinder>
          <length>0.5</length>
          <radius>0.03</radius>
        </cylinder>
      </geometry>
    </visual>
    <collision name="collision">
      <geometry>
        <cylinder>
          <length>0.5</length>
          <radius>0.03</radius>
        </cylinder>
      </geometry>
    </collision>
  </link>
</model>
```

---

## Practical Differences

### When to Use URDF

Use URDF when:
- ✅ Describing robot kinematics only
- ✅ Planning motion (MoveIt)
- ✅ Visualizing in RViz
- ✅ Sharing robot models in community
- ✅ Not simulating (just planning/visualization)

**URDF File Structure**:
```
robot_description/
├── urdf/
│   ├── robot.urdf          (Main description)
│   ├── materials.xacro     (Color definitions)
│   └── meshes/
│       ├── base.stl
│       ├── arm.stl
│       └── gripper.stl
└── launch/
    └── view_robot.launch
```

### When to Use SDF

Use SDF when:
- ✅ Simulating in Gazebo
- ✅ Defining environment (obstacles, lighting)
- ✅ Configuring physics parameters
- ✅ Adding sensor plugins
- ✅ Testing algorithms before hardware

**SDF File Structure**:
```
gazebo_simulations/
├── worlds/
│   ├── simple_world.sdf
│   ├── complex_world.sdf
│   └── objects/
│       ├── table.sdf
│       └── obstacle.sdf
├── models/
│   ├── robot_arm/
│   │   ├── robot_arm.urdf
│   │   └── meshes/
│   └── gripper/
│       ├── gripper.urdf
│       └── meshes/
└── launch/
    └── sim.launch.py
```

---

## Advanced: Sensor Plugins (SDF Only)

### Why Sensors Need SDF

URDF describes sensors structurally (where attached), but **SDF plugins simulate sensor physics**.

**Example: LiDAR Sensor**

URDF (structural only):
```xml
<sensor name="lidar" type="ray"/>
```

SDF (with simulation physics):
```xml
<sensor type="lidar" name="lidar">
  <lidar>
    <scan>
      <horizontal>
        <samples>1024</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>40</max>
      <resolution>0.01</resolution>
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0</mean>
      <stddev>0.01</stddev>
    </noise>
  </lidar>
  <!-- Gazebo plugin for ray-tracing simulation -->
  <plugin name="gazebo_ros_ray_sensor"
          filename="libgazebo_ros_ray_sensor.so">
    <ros>
      <remapping>~/out:=lidar/points</remapping>
    </ros>
  </plugin>
</sensor>
```

**What the plugin does**:
- Simulates ray-tracing (GPU-accelerated)
- Generates PointCloud2 messages
- Publishes to ROS 2 topics
- Applies realistic noise model

---

## Decision Tree: URDF or SDF?

```
Do you need to SIMULATE in Gazebo?
    ├─ YES → Use SDF (world file)
    │         └─ Include URDF robot models in SDF
    │
    └─ NO → Do you need PLANNING / VISUALIZATION?
             ├─ YES → Use URDF only
             │         └─ Use with MoveIt, RViz
             │
             └─ NO → Use neither (not describing robot)
```

---

## Conversion and Compatibility

### Converting URDF to SDF

Gazebo can auto-convert URDF → SDF:

```bash
# Load URDF robot in Gazebo (auto-converts)
ros2 launch gazebo_ros spawn_model.launch.py model:=robot.urdf

# Manually convert URDF to SDF (if needed)
gz sdf print robot.urdf > robot.sdf
```

### Using URDF in SDF

Reference URDF robots within SDF worlds:

```xml
<sdf version="1.9">
  <world name="my_world">
    <!-- Gazebo loads and converts URDF -->
    <model name="robot_arm">
      <urdf uri="file:///path/to/robot.urdf"/>
    </model>
  </world>
</sdf>
```

---

## Summary

| Use Case | Format | Tools | Output |
|----------|--------|-------|--------|
| **Motion Planning** | URDF | MoveIt | Trajectories |
| **Visualization** | URDF | RViz | 3D display |
| **Simulation** | SDF | Gazebo | Physics + sensor data |
| **Hybrid** | URDF + SDF | MoveIt + Gazebo | Planning + simulation |

### Key Takeaway

- **URDF**: "What is the robot?" (structure only)
- **SDF**: "What is the robot + where is it + how do physics work?" (full simulation)
- **Both together**: Leverage ROS 2 tools (URDF) + Gazebo simulation (SDF)

---

## Resources

- **URDF Documentation**: http://wiki.ros.org/urdf
- **SDF Documentation**: http://sdformat.org
- **Gazebo + ROS 2**: https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html
- **Example URDFs**: https://github.com/ros-simulation/urdf_tutorials
