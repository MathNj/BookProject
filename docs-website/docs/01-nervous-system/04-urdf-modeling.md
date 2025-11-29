---
title: Understanding Robot Structure with URDF
sidebar_label: URDF Modeling
---

# Understanding Robot Structure with URDF

## Learning Outcomes

By the end of this chapter, you will be able to:
- Understand what URDF (Unified Robot Description Format) is and why it matters
- Explain the difference between links (bones) and joints (motors)
- Read and interpret a URDF file
- Understand coordinate frames and transformations
- Create a simple URDF describing a multi-link robot
- Validate URDF syntax and troubleshoot common errors

---

## What is URDF?

**URDF** stands for **Unified Robot Description Format**. It's an XML-based language for describing robot structure.

Think of URDF as a **blueprint for your robot**. Just as a house blueprint describes walls, doors, and rooms, a URDF describes:
- **Links**: The rigid body parts of the robot (bones, chassis, wheels)
- **Joints**: How those parts connect and move (hinges, motors)
- **Geometry**: What the robot looks like (shapes for visualization)
- **Physics**: How heavy each part is, how it moves

### Why URDF Matters

URDF enables:
- **Visualization**: Tools like RViz show what your robot looks like
- **Simulation**: Physics engines calculate motion and collisions
- **Planning**: Algorithms plan arm movements without hitting obstacles
- **Documentation**: Clear specification of robot structure for the team

---

## Core Concepts: Links and Joints

### Links: The Skeleton (Bones)

A **link** is a rigid body part—something that doesn't bend or flex. In real robots:

| Link Type | Example | Function |
|---|---|---|
| **Base** | Robot chassis | Everything attaches here |
| **Limb** | Robot arm segment | Moves to reach targets |
| **Wheel** | Drive wheel | Provides locomotion |
| **Gripper** | Robot hand | Grasps objects |
| **Sensor Mount** | Camera bracket | Holds sensors at correct angle |

#### What Defines a Link?

Each link in a URDF has three key properties:

**1. Inertia (Mass and Weight Distribution)**
```xml
<inertial>
  <mass value="1.5"/>  <!-- kg -->
  <inertia ixx="0.01" iyy="0.01" izz="0.01"
           ixy="0.0" ixz="0.0" iyz="0.0"/>
</inertial>
```
- **mass**: Weight in kilograms
- **inertia**: How mass is distributed (affects rotation dynamics)
- Used for **physics simulation** to calculate motion realistically

**2. Visual Geometry (What It Looks Like)**
```xml
<visual>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <geometry>
    <cylinder radius="0.05" length="0.3"/>
  </geometry>
  <material name="red"/>
</visual>
```
- **geometry**: Shape (box, cylinder, sphere, mesh)
- **origin**: Position and rotation
- **material**: Color and appearance
- Used for **visualization** in RViz

**3. Collision Geometry (Physical Boundaries)**
```xml
<collision>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <geometry>
    <cylinder radius="0.05" length="0.3"/>
  </geometry>
</collision>
```
- Defines what can collide with obstacles
- Can be simpler (cheaper computationally) or identical to visual
- Used for **collision detection** and planning

### Joints: The Motors (Connectors)

A **joint** connects two links and defines how they move relative to each other. In real robots:

| Joint Type | Movement | Example | Degrees of Freedom |
|---|---|---|---|
| **Revolute** | Rotates around an axis | Elbow, shoulder, knee | 1 (angle) |
| **Prismatic** | Slides along an axis | Elevator, piston | 1 (distance) |
| **Fixed** | Doesn't move | Welded parts | 0 (rigid connection) |
| **Continuous** | Rotates without limits | Wheel, spinning base | 1 (unbounded rotation) |
| **Planar** | Moves in 2D plane | Sliding puck | 2 (x, y) |
| **Floating** | Moves freely in space | Flying drone | 6 (x, y, z + 3 rotations) |

#### What Defines a Joint?

Each joint connects exactly two links:

```xml
<joint name="elbow" type="revolute">
  <parent link="upper_arm"/>   <!-- The "before" link -->
  <child link="forearm"/>       <!-- The "after" link -->

  <origin xyz="0 0 0.3" rpy="0 0 0"/>  <!-- Where the joint is -->

  <axis xyz="0 0 1"/>           <!-- Rotation axis (Z = vertical) -->

  <limit lower="-1.57" upper="1.57"    <!-- Min/max angles (-90° to +90°) -->
         effort="10.0" velocity="1.0"/> <!-- Force and speed limits -->
</joint>
```

**Key Properties**:
- **parent/child**: Which links to connect
- **origin**: Position in 3D space (x, y, z) and rotation (roll, pitch, yaw)
- **axis**: Direction of movement (x=1, y=0, z=0 means slide/rotate along X)
- **limits**: Physical constraints on movement

### The Kinematic Chain

Joints create a **chain** of connected links:

```
base_link (the ground)
    ↓
  [joint_1] (shoulder)
    ↓
  link_1 (upper arm)
    ↓
  [joint_2] (elbow)
    ↓
  link_2 (forearm)
    ↓
  [joint_3] (wrist)
    ↓
  end_effector (gripper)
```

Each joint allows the downstream links to move relative to upstream links.

---

## Coordinate Frames and Transforms

### The Reference Frame

Every link has its own **coordinate frame**—an origin point with X, Y, Z axes:

```
     Z (up)
     ↑
     │
     ├──→ X (forward)
    /
   / Y (right)
```

### Origins and Rotations

When you specify a joint's **origin**, you're saying where it is relative to the parent link:

```xml
<origin xyz="0 0 0.3" rpy="0 0 0"/>
         x  y  z       roll pitch yaw
```

- **xyz**: Position in meters (x forward, y right, z up)
- **rpy**: Rotation in radians (roll around X, pitch around Y, yaw around Z)

**Example**:
```xml
<!-- Joint is 30cm above parent, rotated 90° around Z -->
<origin xyz="0 0 0.3" rpy="0 0 1.57"/>
```

### Visualizing Transforms

```
Parent Link Frame:
       Z↑
       │
   ──┼──→X
      /
     Y

Joint at origin xyz="0 0 0.3" rpy="0 0 0"
       ↓
    (moved up 0.3m, not rotated)

Child Link Frame:
       Z↑
       │
   ──┼──→X
      /
     Y (shifted up in parent space)
```

---

## Building a Simple 2-Joint Robot Arm

Let's examine a real URDF and understand every part.

### File Structure

```xml
<?xml version="1.0"?>
<robot name="simple_arm">
  <!-- Define all links here -->
  <!-- Define all joints here -->
</robot>
```

### Link Definitions

#### Base Link: The Foundation

```xml
<link name="base_link">
  <inertial>
    <mass value="1.0"/>
    <inertia ixx="0.01" ixy="0.0" ixz="0.0"
             iyy="0.01" iyz="0.0" izz="0.01"/>
  </inertial>

  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.2 0.2 0.1"/>  <!-- 20cm × 20cm × 10cm box -->
    </geometry>
    <material name="gray"/>
  </visual>

  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.2 0.2 0.1"/>
    </geometry>
  </collision>
</link>
```

**Reading This**:
- Base link is 1 kg (heavy, stable foundation)
- Visually: a gray box
- Inertia values approximate a box's mass distribution

#### Link 1: Upper Arm

```xml
<link name="link_1">
  <inertial>
    <mass value="0.5"/>
    <inertia ixx="0.002" ixy="0.0" ixz="0.0"
             iyy="0.002" iyz="0.0" izz="0.001"/>
  </inertial>

  <visual>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="0.05" length="0.3"/>
    </geometry>
    <material name="red"/>
  </visual>

  <collision>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="0.05" length="0.3"/>
    </geometry>
  </collision>
</link>
```

**Reading This**:
- Upper arm is 0.5 kg (lighter, movable)
- Visually: a red cylinder 30cm long, 5cm radius
- Positioned at z=0.15 (centered vertically in the cylinder)

#### Link 2: Forearm

```xml
<link name="link_2">
  <inertial>
    <mass value="0.3"/>
    <inertia ixx="0.001" ixy="0.0" ixz="0.0"
             iyy="0.001" iyz="0.0" izz="0.0005"/>
  </inertial>

  <visual>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="0.03" length="0.2"/>
    </geometry>
    <material name="blue"/>
  </visual>

  <collision>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="0.03" length="0.2"/>
    </geometry>
  </collision>
</link>
```

**Reading This**:
- Forearm is 0.3 kg (lighter than upper arm)
- Visually: a blue cylinder 20cm long, 3cm radius (thinner than upper arm)
- Positioned at z=0.1 (centered in its cylinder)

### Joint Definitions

#### Joint 1: Shoulder

```xml
<joint name="joint_1" type="revolute">
  <parent link="base_link"/>
  <child link="link_1"/>

  <origin xyz="0 0 0.05" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>

  <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  <dynamics damping="0.1" friction="0.1"/>
</joint>
```

**Breaking This Down**:
- **type="revolute"**: Rotates around one axis (like a shoulder)
- **parent/child**: Connects base to upper arm
- **origin xyz="0 0 0.05"**: Joint is 5cm above the base
- **axis xyz="0 0 1"**: Rotates around Z-axis (vertical, like swinging arm left-right)
- **lower="-1.57" upper="1.57"**: Range is -90° to +90° (in radians: -π/2 to +π/2)
- **effort="10.0"**: Max torque is 10 N⋅m
- **velocity="1.0"**: Max rotation speed is 1 rad/s
- **damping/friction**: Energy loss due to bearing friction

#### Joint 2: Elbow

```xml
<joint name="joint_2" type="revolute">
  <parent link="link_1"/>
  <child link="link_2"/>

  <origin xyz="0 0 0.3" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>

  <limit lower="-1.57" upper="1.57" effort="5.0" velocity="1.0"/>
  <dynamics damping="0.05" friction="0.05"/>
</joint>
```

**Breaking This Down**:
- **parent/child**: Connects upper arm to forearm
- **origin xyz="0 0 0.3"**: Joint is at the END of the upper arm (30cm from shoulder)
- **axis xyz="0 0 1"**: Also rotates around Z-axis
- **lower/upper**: Same range as shoulder (-90° to +90°)
- **effort="5.0"**: Weaker than shoulder (5 N⋅m vs 10)—realistic since forearm is lighter

---

## Visualizing the URDF

### Step 1: Save the URDF

Copy this to a file called `simple_arm.urdf`:
```bash
cat > simple_arm.urdf << 'EOF'
<?xml version="1.0"?>
<robot name="simple_arm">
  <!-- [all the links and joints from above] -->
</robot>
EOF
```

### Step 2: Use RViz to Visualize

(Note: RViz is covered in depth in later modules. For now, here's the overview.)

```bash
# Start RViz
rviz2

# In RViz:
# 1. Set Fixed Frame to "base_link"
# 2. Add Display: RobotModel
# 3. Set Model Description to the URDF file
# 4. See the arm visualized!
```

### Expected Visualization

```
     [Link 2] (blue forearm)
        |
        | (joint_2)
        |
     [Link 1] (red upper arm)
        |
        | (joint_1)
        |
     [Base] (gray box)
```

---

## URDF Validation and Common Errors

### How to Check URDF Syntax

```bash
# Method 1: Use check_urdf tool
check_urdf simple_arm.urdf

# Output if valid:
# robot name is: simple_arm
# ----------- Successfully Parsed XML ---------------
# root Link: base_link has 1 child(ren)
#     child(1):  link_1
#         child(1):  link_2

# Output if invalid:
# Error: [XML parsing error message]
```

### Common Error 1: Missing Mass

**Error**:
```
Error: [Collision geometry specified for link 'link_1'] with no inertia
```

**Cause**: Every link must have `<inertial>` section

**Fix**: Add mass and inertia to every link

---

### Common Error 2: Circular References

**Error**:
```
Error: joint 'joint_1' parent 'link_1' appears before child in tree
```

**Cause**: Joint tries to connect links in wrong order (child already has parents)

**Fix**: Ensure parent-child relationships form a tree (no cycles)

---

### Common Error 3: Undefined Link Reference

**Error**:
```
Error: [joint 'joint_1'] Child link [link_3] not found
```

**Cause**: Joint references a link that doesn't exist

**Fix**: Check spelling; define missing links

---

### Common Error 4: Invalid Axis

**Error**:
```
Warning: Link 'link_1' has no axis specified
```

**Cause**: Revolute joint without axis

**Fix**: Add `<axis xyz="..." />`

---

## Modification Exercise

Now it's your turn to modify the arm!

### Task: Change Arm Dimensions

**Goal**: Make the arm longer and with different proportions

**Current Dimensions**:
- Upper arm: 30cm long, 5cm radius
- Forearm: 20cm long, 3cm radius

**Try This**:

1. **Make the upper arm longer**:
   ```xml
   <!-- Current -->
   <cylinder radius="0.05" length="0.3"/>

   <!-- New: 40cm instead of 30cm -->
   <cylinder radius="0.05" length="0.4"/>

   <!-- Also update the joint origin to match the new length -->
   <!-- Old: -->
   <origin xyz="0 0 0.3" rpy="0 0 0"/>
   <!-- New: -->
   <origin xyz="0 0 0.4" rpy="0 0 0"/>
   ```

2. **Make the forearm thicker**:
   ```xml
   <!-- Current -->
   <cylinder radius="0.03" length="0.2"/>

   <!-- New: thicker and longer -->
   <cylinder radius="0.05" length="0.3"/>
   ```

3. **Increase joint limits** to allow more rotation:
   ```xml
   <!-- Current: -90° to +90° -->
   <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>

   <!-- New: -180° to +180° (full rotation) -->
   <limit lower="-3.14" upper="3.14" effort="10.0" velocity="1.0"/>
   ```

### Validation After Changes

```bash
# Check syntax
check_urdf simple_arm.urdf

# Visualize to see the changes
rviz2
```

---

## Real-World URDF Examples

### Example 1: Mobile Robot Base

```xml
<robot name="mobile_robot">
  <link name="base_link">
    <!-- Chassis: 50cm × 30cm × 20cm box -->
    <visual>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </visual>
  </link>

  <!-- Left wheel -->
  <link name="left_wheel"/>
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <axis xyz="0 1 0"/>  <!-- Spins around Y-axis -->
  </joint>

  <!-- Right wheel -->
  <link name="right_wheel"/>
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <axis xyz="0 1 0"/>
  </joint>
</robot>
```

### Example 2: Sensor Mount

```xml
<!-- Camera on a pan-tilt base -->
<link name="pan_tilt_base"/>

<!-- Pan joint: left-right rotation -->
<joint name="pan_joint" type="revolute">
  <parent link="base_link"/>
  <child link="pan_tilt_base"/>
  <axis xyz="0 0 1"/>  <!-- Rotate around Z -->
  <limit lower="-3.14" upper="3.14"/>
</joint>

<!-- Tilt joint: up-down rotation -->
<link name="tilt_mount"/>
<joint name="tilt_joint" type="revolute">
  <parent link="pan_tilt_base"/>
  <child link="tilt_mount"/>
  <axis xyz="1 0 0"/>  <!-- Rotate around X -->
  <limit lower="-1.57" upper="1.57"/>
</joint>

<!-- Camera on the tilt mount -->
<link name="camera"/>
<joint name="camera_joint" type="fixed">
  <parent link="tilt_mount"/>
  <child link="camera"/>
  <origin xyz="0 0 0.1"/>  <!-- 10cm above tilt mount -->
</joint>
```

---

## Key Concepts Summary

| Concept | Definition | Real-World Analogy |
|---|---|---|
| **Link** | Rigid body part | Bone in skeleton |
| **Joint** | Connection between links | Muscle connecting bones |
| **Revolute Joint** | Rotation around one axis | Elbow, knee |
| **Prismatic Joint** | Sliding along axis | Elevator, hydraulic piston |
| **Fixed Joint** | No movement | Welded steel joint |
| **Inertia** | Mass distribution | Weight and how it's spread |
| **Origin (xyz)** | 3D position | Location in space |
| **Origin (rpy)** | 3D rotation | Angle in three directions |
| **Axis** | Direction of movement | Which way it spins/slides |

---

## Next Steps

Now that you understand URDF:

1. **Load your URDF into RViz** (next module) to visualize it
2. **Add sensors** (cameras, lidar) as additional links
3. **Add a gripper** (more joints and links) to the arm
4. **Simulate motion** using Gazebo to test physics

---

## Exercises

1. **Read Existing URDF**: Download a real robot URDF from [URDF Examples](https://github.com/ros/urdf_tutorials) and identify all links and joints.

2. **Modify Joint Limits**: Change the elbow joint's angular limits and visualize the change.

3. **Add a Gripper**: Create a fixed joint with a gripper link at the end of link_2.

4. **Change Materials**: Modify the visual materials to use different colors.

5. **Calculate Reach**: The arm's "reach" is the total length when fully extended. Calculate it for:
   - Current arm (30 + 20 = 50cm)
   - Your modified arm

---

**Ready to see your robot in action?** Next, we'll visualize and simulate this URDF using ROS 2 tools.
