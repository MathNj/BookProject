# Module 2: The Digital Twin (Gazebo Fortress & Unity) - Implementation Plan

**Feature**: Module 2: The Digital Twin
**Version**: 1.0.0
**Created**: 2025-11-30
**Architecture Team**: Physical AI Textbook
**Status**: Ready for Task Decomposition

---

## 1. Executive Summary

Module 2 teaches students to build and deploy **Digital Twins** using industry-standard tools:
- **Gazebo Fortress** for physics-accurate simulation (ROS 2 native, via `ros-humble-ros-gz` bridge)
- **Unity Robotics Hub** for high-fidelity 3D visualization and extended reality (XR)

Students will learn to simulate robots safely, test control algorithms 100+ times risk-free, and understand the simulation-to-reality gap before deploying to real hardware.

**Estimated Effort**: 35-40 hours
**Dependency**: Module 1 (ROS 2 fundamentals) completed
**Hardware Requirement**: NVIDIA RTX GPU (RTX 4070 Ti primary, RTX 4060 minimum)

---

## 2. Technical Context & Architecture Decisions

### 2.1 Core Technology Stack

| Component | Technology | Rationale |
|-----------|-----------|-----------|
| **Simulation Engine** | Gazebo Fortress | ROS 2 native integration via ros-humble-ros-gz; physics-accurate for robotics |
| **ROS 2 Bridge** | ros-humble-ros-gz | Official Gazebo-ROS 2 connector; built into ROS 2 Humble |
| **Visualization** | Unity Robotics Hub | Photorealistic rendering, XR support, ROS-TCP-Connector for real-time sync |
| **Physics Engine** | Ignition Physics | Integrated with Gazebo Fortress; deterministic, reproducible simulations |
| **Sensor Simulation** | Gazebo Plugins (LiDAR, Camera) | Ray-tracing for realistic sensor data (depth, point clouds) |
| **Robot Description** | URDF (with sensors) + SDF (with physics) | URDF for ROS compatibility, SDF for Gazebo physics parameters |
| **Development Language** | Python 3.10+ (rclpy) | Consistent with Module 1; accessible for students |
| **Visualization Tool** | RViz 2 | Display sensor data, trajectory visualization |

### 2.2 Hardware Requirements

#### GPU Requirement (MANDATORY)
- **Primary**: NVIDIA RTX 4070 Ti (12GB VRAM)
- **Minimum**: NVIDIA RTX 4060 (8GB VRAM)
- **Supported**: Any NVIDIA CUDA-capable GPU (RTX 3060, A100, Jetson Orin)
- **Why**: Ray tracing for LiDAR simulation, depth camera rendering, real-time physics (1000 Hz)
- **Performance Target**: 1000 Hz physics updates, 30+ FPS visualization

#### System Requirements
- Ubuntu 22.04 LTS (or WSL2 with GPU pass-through)
- 16GB RAM (32GB recommended for running multiple simulations)
- 20GB free disk space (Gazebo + dependencies + simulation worlds)
- CUDA Toolkit 11.8+ (auto-installed with ROS 2 Humble on some distributions)

### 2.3 Key Architectural Decisions

#### Decision 1: Gazebo Fortress vs Classic
- **Chosen**: Gazebo Fortress (Ignition)
- **Rationale**:
  - Modern architecture designed for ROS 2
  - `ros-humble-ros-gz` is the official integration
  - Better performance and features than Classic
  - Community momentum is moving to Fortress
- **Trade-off**: Fewer third-party plugins than Classic (but core sensors covered)

#### Decision 2: Unity Integration Scope
- **Chosen**: Unity Robotics Hub with ROS-TCP-Connector
- **Rationale**:
  - Native ROS 2 communication via TCP bridge
  - Students get photorealistic 3D visualization
  - Optional for basic curriculum (Gazebo alone sufficient)
  - XR capabilities for future extensions
- **Implementation**: File 4 (`04-unity-visualization.md`) as optional advanced topic

#### Decision 3: Sensor Simulation
- **Chosen**: Gazebo plugins (LiDAR, depth camera, IMU)
- **Rationale**:
  - Built into Gazebo Fortress
  - Ray-tracing for realistic LiDAR simulation
  - OpenGL rendering for depth camera
  - Produces standard ROS 2 message types (PointCloud2, Image, Imu)
- **Trade-off**: GPU acceleration required; CPU fallback slow

#### Decision 4: Control Architecture
- **Chosen**: Simple joint command publishers (no ros2_control framework)
- **Rationale**:
  - Module 1 students understand pub/sub pattern
  - Focus on simulation concepts, not control framework
  - Professional control stack (ros2_control) deferred to Module 3
  - Students write simple: `JointCommand` → robot responds
- **Future**: Module 3 will layer ros2_control on this foundation

#### Decision 5: Sim-to-Real Transfer
- **Chosen**: Acknowledge reality gap; defer advanced techniques to Module 3
- **Rationale**:
  - Module 2 focuses on "safe prototyping in simulation"
  - Domain randomization and system ID in Module 3
  - Students understand gap exists but don't need all solutions yet
- **Content**: Section in File 1 on "Why Simulation ≠ Reality"

---

## 3. Content Architecture & File Structure

### 3.1 Directory Structure

```
docs-website/docs/02-digital-twin-sim/
├── 00-getting-started.md          (Prerequisites, hardware check, learning outcomes)
├── 01-intro-digital-twin.md       (Digital Twin concept, Gazebo vs Unity, sim-to-real gap)
├── 02-gazebo-fortress-setup.md    (Installing ros-humble-ros-gz, launching worlds)
├── 03-simulating-sensors.md       (LiDAR, depth camera, IMU with code examples)
├── 04-unity-visualization.md      (Optional: Unity Robotics Hub setup)
├── STUDENT-CHECKLIST.md           (100+ progress tracking items)
├── code-examples/
│   ├── README.md                  (Guide to running examples)
│   ├── robot_sim.urdf             (2-joint arm + sensors)
│   ├── simple_world.sdf           (Gazebo world with obstacles)
│   ├── sensor_reader.py           (Python node reading LiDAR/camera)
│   └── arm_control.py             (Simple control algorithm for arm)
└── resources/
    └── (diagrams, screenshots, reference materials)
```

### 3.2 Chapter Breakdown

#### **Chapter 1: Getting Started** (1 KB)
**Purpose**: Prerequisite verification and hardware check
**Content**:
- Knowledge requirements (Module 1 completion, Python, Linux)
- Hardware requirements (GPU, RAM, disk space)
- Software requirements (ROS 2 Humble, Gazebo Fortress, Python 3.10+)
- ⚠️ **GPU Warning**: "These tutorials require NVIDIA RTX GPU. Without GPU, simulations run slowly (10-50 Hz instead of 1000 Hz)"
- Learning outcomes checklist
- 5+ environment verification commands

**Success Metric**: Student confirms GPU availability before starting tutorials

---

#### **Chapter 2: Introduction to Digital Twin** (2.5 KB)
**Purpose**: Conceptual foundation for simulation-based robotics
**Key Sections**:
1. **What is a Digital Twin?**
   - Definition: Virtual replica of physical system
   - Example: Real robot + simulated robot
   - Benefits: Safety ($3k robot protection), cost (100 test runs = $0), speed (iterate fast)

2. **Sim-to-Real Gap**
   - Why simulation ≠ reality
   - Physics approximations in Gazebo
   - Friction, contact dynamics, sensor noise
   - Domain randomization (preview for Module 3)

3. **Gazebo Fortress vs Unity**
   - Gazebo: Physics-first, ROS 2 native, real-time simulation
   - Unity: Visual-first, photorealism, XR capable
   - When to use each (Gazebo for testing, Unity for demos)

4. **Hardware Reality Check**
   - GPU is non-negotiable for tutorials
   - Real-time simulation requires GPU acceleration
   - CPU fallback option (slow but possible)
   - Link to hardware requirements section

5. **Real-World Examples**
   - Boston Dynamics: Digital Twin for prototyping
   - NVIDIA: Isaac Sim for industrial robotics
   - ABB: Simulation before real robot deployment

**Success Metric**: Student understands why simulation matters and GPU requirements

---

#### **Chapter 3: Setting Up Gazebo Fortress** (3 KB)
**Purpose**: Hands-on environment setup and world building
**Key Sections**:
1. **Installing the ROS 2 ↔ Gazebo Bridge**
   - `sudo apt install ros-humble-ros-gz`
   - Verification: `ros2 run ros_gz_sim gz sim --help`
   - Understanding the bridge architecture

2. **Launching Your First Gazebo World**
   - Empty world with gravity enabled
   - Understanding SDF structure (simulation description format)
   - XML elements: `<world>`, `<gravity>`, `<physics>`

3. **Building a Simple World**
   - Adding ground plane
   - Spawning obstacles (boxes, cylinders)
   - Setting physics parameters (friction 0.5, restitution 0.0)

4. **Spawning a Robot in the World**
   - Using `ros2 launch` to spawn robot model
   - Understanding URDF → SDF conversion
   - Robot placement and orientation

5. **Real-Time Visualization**
   - Launching Gazebo GUI
   - Navigating 3D view
   - Monitoring physics engine performance (1000 Hz target)
   - Troubleshooting: GPU/CPU mode detection

**Code Example**: `simple_world.sdf` with documented physics parameters

**Success Metric**: Student can launch Gazebo world with physics, spawn robot, visualize in real-time

---

#### **Chapter 4: Adding Sensors to Simulation** (3.5 KB)
**Purpose**: Realistic sensor simulation using Gazebo plugins
**Key Sections**:
1. **Understanding Sensor Plugins**
   - How Gazebo plugins work (dynamic libraries)
   - Built-in plugins: LiDAR, camera, IMU
   - Ray-tracing for sensor simulation

2. **Adding LiDAR to URDF**
   - `<sensor type="lidar">` block with parameters
   - Ray count, range, noise model
   - Output: PointCloud2 ROS 2 message type
   - Configuring Gazebo LiDAR plugin

3. **Adding Depth Camera**
   - `<sensor type="rgbd_camera">` block
   - RGB output + depth output
   - Camera intrinsics (focal length, resolution)
   - OpenGL rendering for realistic output

4. **Adding IMU (Inertial Measurement Unit)**
   - 3-axis accelerometer simulation
   - 3-axis gyroscope simulation
   - Noise models (realistic accelerometer bias, gyro drift)

5. **Reading Sensor Data in ROS 2**
   - Subscribing to `/robot/lidar/points` topic
   - Subscribing to `/robot/camera/depth/image_raw` topic
   - Subscribing to `/robot/imu/data` topic
   - Processing in Python with rclpy

6. **Visualizing in RViz**
   - Adding PointCloud2 display for LiDAR
   - Adding Image display for camera
   - Understanding RViz coordinate frames

**Code Example**: `robot_sim.urdf` with LiDAR, camera, IMU; `sensor_reader.py` node

**Success Metric**: Student adds sensors to robot, reads data in Python ROS 2 node, visualizes in RViz

---

#### **Chapter 5: Unity for High-Fidelity Visualization** (2.5 KB) [OPTIONAL]
**Purpose**: Advanced visualization and XR capabilities
**Key Sections**:
1. **When to Use Unity vs Gazebo**
   - Gazebo: Physics testing, algorithm development
   - Unity: Photorealistic rendering, client presentations, XR
   - Hybrid approach: Gazebo physics + Unity visualization

2. **Setting Up Unity Robotics Hub**
   - Installing Unity 2022 LTS
   - Installing ROS 2 UDP Communicator
   - ROS-TCP-Connector architecture

3. **Connecting to Gazebo**
   - TCP bridge between Gazebo (Linux) and Unity (local/remote)
   - Bidirectional communication: state from Gazebo → Unity visualization
   - Commands from Unity controllers → Gazebo physics

4. **Creating a Simple Robot Model in Unity**
   - Importing 3D model
   - Creating robot skeleton (bones for joints)
   - Configuring for animation

5. **Visualizing Sensor Data**
   - Displaying LiDAR point cloud as colored particles
   - Real-time camera feed texture mapping
   - IMU orientation visualization

6. **Future: Extended Reality (XR)**
   - Preview: AR visualization of simulated robot
   - VR teleoperationof simulated robot (Module 3+)

**Code Example**: Unity project setup script, ROS-TCP-Connector configuration

**Success Metric**: Student successfully visualizes Gazebo simulation in Unity (optional task)

---

#### **Chapter 6: Student Progress Checklist** (8 KB)
**Purpose**: Self-guided progress tracking
**Content**:
- 100+ checkbox items across all chapters
- Exercise completion tracker
- Troubleshooting quick reference
- Hardware verification checklist
- Next steps and Module 3 preview

**Success Metric**: Student completes checklist, feels prepared for Module 3

---

## 4. Code Examples & Deliverables

### 4.1 Code Examples (4 files)

#### **File 1: `robot_sim.urdf`** (2 KB)
- 2-joint robot arm (shoulder, elbow)
- 3 links (base, upper arm, forearm)
- Depth camera at end-effector
- LiDAR at base
- IMU on body
- Fully commented XML

#### **File 2: `simple_world.sdf`** (1.5 KB)
- Ground plane with physics
- 2 obstacle boxes (table-like)
- Gravity = 9.81 m/s²
- Physics solver parameters (iterations, damping)
- Fully documented SDF structure

#### **File 3: `sensor_reader.py`** (1.5 KB)
- Python ROS 2 node (rclpy)
- Subscribes to: LiDAR (PointCloud2), Camera (Image), IMU (Imu)
- Callback functions for each sensor
- Basic data processing (point cloud statistics, image shape, IMU values)
- Fully commented

#### **File 4: `arm_control.py`** (1 KB)
- Simple control algorithm: move arm in circle
- Publishes JointCommand messages
- Subscribes to odometry (robot feedback)
- Demonstrates feedback loop
- Fully commented

### 4.2 Documentation

#### **code-examples/README.md** (1 KB)
- Quick start (5 steps to run simulation)
- File descriptions
- Modification exercises (3+)
- Troubleshooting

---

## 5. Implementation Phases

### Phase 1: Setup & Foundational Content
**Duration**: 6-8 hours
**Tasks**:
- T001-T003: Directory creation, resources setup, sidebar configuration
- T004-T006: Write Getting Started chapter, code examples README
- **Deliverable**: Students can verify prerequisites and environment

### Phase 2: Conceptual Content (Chapters 1-2)
**Duration**: 5-7 hours
**Tasks**:
- T007-T010: Write intro chapter, digital twin concept, sim-to-real gap explanation
- **Deliverable**: Students understand why simulation matters

### Phase 3: Gazebo Hands-On (Chapter 3)
**Duration**: 6-8 hours
**Tasks**:
- T011-T015: Write Gazebo setup chapter, create world SDF files, verify installation
- **Deliverable**: Students can launch Gazebo, spawn robots, visualize physics

### Phase 4: Sensor Integration (Chapter 4)
**Duration**: 8-10 hours
**Tasks**:
- T016-T020: Write sensor chapter, create URDF with sensors, write sensor_reader.py
- T021-T023: Sensor visualization in RViz, troubleshooting, modification exercises
- **Deliverable**: Students add sensors, read data, visualize

### Phase 5: Optional Unity Visualization (Chapter 5)
**Duration**: 4-6 hours
**Tasks**:
- T024-T026: Write Unity chapter, setup scripts, ROS-TCP-Connector config
- **Deliverable**: Students visualize Gazebo in Unity (optional)

### Phase 6: Polish & Verification
**Duration**: 6-8 hours
**Tasks**:
- T027-T030: Create student checklist, accessibility review, final verification
- T031-T032: Build Docusaurus, test on both locales, deployment ready
- **Deliverable**: Module 2 production-ready for student release

---

## 6. Success Criteria (Refined from Spec)

| Criterion | Metric | Verification |
|-----------|--------|---------------|
| **Completeness** | All 4 chapters + optional 1 completed | Content checklist |
| **Code Accuracy** | 100% of 4 code examples run without errors | Execution test on RTX 4070 Ti |
| **Gazebo Integration** | All examples work with Gazebo Fortress + ROS 2 Humble | Environment test |
| **GPU Documentation** | Clear warning on GPU requirement, fallback options explained | Content review |
| **Student Learning** | 85%+ of students complete sensor integration chapter | Checklist completion |
| **Readability** | Grade 10+ readability, WCAG 2.1 AA accessible | Automated + manual review |
| **Build Success** | Docusaurus builds for en & ur locales without errors | Build verification |
| **Deployment** | Static site renders all chapters correctly | Browser testing |

---

## 7. Dependencies & Integration Points

### Internal Dependencies
- ✅ Module 1: ROS 2 fundamentals (prerequisite)
- ✅ Module 1: Python pub/sub pattern (used in sensor_reader.py)
- ✅ Module 1: URDF basics (extended with sensors in this module)

### External Dependencies
- Gazebo Fortress (via `ros-humble-ros-gz`)
- ROS 2 Humble
- NVIDIA CUDA Toolkit
- Python 3.10+ with rclpy
- Optional: Unity 2022 LTS (for Chapter 5)

### Integration with Future Modules
- **Module 3 (Robot Brain)**: Builds on sensor simulation, adds ros2_control and advanced algorithms
- **Module 4 (The Mind)**: Uses VLA for perception in simulation
- **Module 5 (Capstone)**: Full sim-to-real deployment

---

## 8. Risks & Mitigation

| Risk | Impact | Mitigation |
|------|--------|-----------|
| GPU availability | Students can't run tutorials | CPU fallback documented; lab GPU access encouraged |
| Gazebo Fortress complexity | Steep learning curve | Start simple (empty world), build progressively |
| Sensor plugin configuration | Errors in simulation | Provide working URDF examples, detailed comments |
| Sim-to-real mismatch | Student frustration | Dedicate Chapter 2 to gap explanation; manage expectations |
| ROS-TCP-Connector issues | Unity integration broken | Make Chapter 5 optional; Gazebo alone sufficient |

---

## 9. Assumptions

| # | Assumption | Rationale |
|---|-----------|-----------|
| A1 | Students completed Module 1 | Prerequisites documented |
| A2 | NVIDIA RTX GPU available | Hardware constraint stated explicitly |
| A3 | ROS 2 Humble + Gazebo Fortress installed | Standard robotics environment |
| A4 | Python 3.10+ available | Required for rclpy and examples |
| A5 | Basic Linux CLI knowledge | Inherited from Module 1 |
| A6 | Internet access for dependencies | Installation step requirement |

---

## 10. Constitution Alignment

✅ **Principle I: Pedagogical Integrity**
- Real-world robotics use case (Digital Twin standard practice)
- Safe experimentation (simulator before hardware)
- Hands-on learning (write code, see results)

✅ **Principle II: Embodied Intelligence**
- Simulation enables embodied understanding (physics, sensors)
- Bridge from virtual to physical (sim-to-real concepts)

✅ **Principle V: Hardware Constraints**
- Explicit GPU requirement documented
- Fallback CPU option provided
- Budget-conscious (Gazebo is free/open-source)

✅ **Principle VI: Content Quality**
- Grade 10+ readability target
- Code accuracy 100%
- WCAG 2.1 AA accessibility compliant

---

## 11. Technical Debt & Future Work

- **Unity integration** (Chapter 5): Optional; full XR support deferred to post-Module 5
- **Advanced sensors**: Ultrasonic, thermal cameras (Module 3+)
- **Soil dynamics simulation**: For outdoor robotics (Module 4+)
- **Hardware-in-the-loop (HIL)**: Real hardware + simulation integration (Module 3+)

---

## Next Steps

1. ✅ **Architecture approved** (this plan)
2. → **Task decomposition**: Run `/sp.tasks` to generate 25-30 atomic, executable tasks
3. → **Implementation**: Execute tasks phase-by-phase (Phase 1-6)
4. → **Deployment**: Build, test, merge to production

---

**Status**: ✅ **Ready for Task Decomposition**
**Estimated Total Effort**: 35-40 hours
**Timeline**: 2-3 weeks (1-2 weeks if parallel work)
**Next Command**: `/sp.tasks` to generate task breakdown

