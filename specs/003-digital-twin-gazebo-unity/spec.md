# Module 2: The Digital Twin (Gazebo & Simulation) - Specification

**Feature**: Module 2: The Digital Twin (Gazebo & Simulation)
**Version**: 1.0.0
**Created**: 2025-11-30
**Author**: Physical AI Textbook Team
**Status**: Draft (Awaiting Clarification)

---

## 1. Overview

### Vision

Before we risk breaking a $3,000 robot, we crash it in a simulator 100 times. Module 2 teaches students the concept of a **Digital Twin**—a virtual clone of a physical robot used for safe, risk-free experimentation. Students will learn how simulation bridges the gap between theory (Module 1: ROS 2 fundamentals) and practice (safe prototyping before deploying on real hardware).

### Core Theme

**The Digital Twin**: A virtual replica of physical systems that enables students to:
- Test algorithms without hardware risk or cost
- Understand physics simulation (rigid body dynamics, collisions, gravity)
- Work with different simulation engines (Gazebo for accuracy, Unity for visual fidelity)
- Integrate sensors into simulated environments (cameras, LiDAR, IMU)

---

## 2. Problem Statement

**Current Gap**: Students completing Module 1 understand ROS 2 concepts but lack a safe environment to test control algorithms. Running untested code on real hardware risks:
- $3,000+ robot damage
- Student safety hazards
- Expensive debugging cycles

**Solution Needed**: Provide a hands-on module teaching digital twin simulation so students can:
1. Prototype and debug algorithms in a safe, cost-free virtual environment
2. Understand the relationship between simulation and real-world physics
3. Transition confidently from simulated testing to real robot deployment

---

## 3. Scope

### In Scope (Mandatory)

#### 3.1 Pedagogical Content

- **Digital Twin Concept**: Definition, benefits, and the simulation-to-reality bridge
- **Gazebo vs. Unity Comparison**:
  - **Gazebo**: Engineering-focused, physics accuracy, ROS 2 native integration
  - **Unity**: Visual fidelity, photorealism, extended reality (XR) capabilities
  - Trade-offs and use cases for each

#### 3.2 Technical Content

- **Physics Engines Fundamentals**:
  - Rigid body dynamics (mass, velocity, forces, torque)
  - Gravity and acceleration
  - Collision detection and response
  - Why physics simulation matters for robotics

- **Robot Description Formats**:
  - **URDF** (Unified Robot Description Format): ROS standard, used with ROS controllers
  - **SDF** (Simulation Description Format): Gazebo standard, includes physics parameters
  - Converting URDF to SDF and when each is used
  - Practical differences in the files

- **Sensors in Simulation**:
  - **Vision Sensors**: Depth cameras (RGB-D), stereo cameras, their output formats
  - **Spatial Sensors**: LiDAR (point clouds), IMU (acceleration, rotation)
  - Simulating sensor data: noise, latency, realistic sensor behavior
  - Gazebo sensor plugins for LiDAR, camera, IMU
  - Code examples showing how to add sensors to URDF models

#### 3.3 Hardware Requirements & Constraints

- **GPU Requirement**: NVIDIA RTX graphics card (RTX 4070 Ti recommended, RTX 4060 minimum)
  - Why: Ray tracing for realistic sensor simulation (camera rendering, ray-casting LiDAR)
  - Performance implications: Real-time simulation at 1000 Hz requires GPU acceleration
  - Fallback: CPU simulation possible but slower (trade speed for compatibility)

- **Target Hardware**:
  - NVIDIA RTX 4070 Ti (primary testing platform)
  - Jetson Orin (edge device with GPU)
  - Supported: Any NVIDIA CUDA-capable GPU (RTX 3060, A100, etc.)
  - Not supported: Intel iGPU, Apple Silicon (limited CUDA support)

#### 3.4 Deliverables (Files & Code)

**Documentation** (3 markdown chapters):

1. **00-digital-twin-intro.md** (~2-3 KB)
   - What is a Digital Twin?
   - Why simulate before deploying to real hardware?
   - Gazebo vs. Unity: when to use each
   - Hardware requirements and why GPU is essential
   - Real-world robotics examples

2. **01-gazebo-worlds.md** (~3-4 KB)
   - Setting up Gazebo with ROS 2
   - Building simulated worlds (SDF format)
   - Adding physics parameters (friction, damping, gravity)
   - Spawning robot models
   - Running simulation and checking performance
   - Visual debugging tools in Gazebo

3. **02-sensors-in-sim.md** (~4-5 KB)
   - Adding sensors to URDF (camera, LiDAR, IMU)
   - Configuring Gazebo plugins
   - Understanding sensor data formats
   - Reading sensor output in ROS 2 nodes
   - Visualizing sensor data (RViz integration)

**Code Examples** (3 files):

- **robot_sim.urdf**: 2-joint robot arm with depth camera and LiDAR sensor plugins
- **simple_world.sdf**: Gazebo world with ground plane, table, and obstacles
- **sensor_reader.py**: Python ROS 2 node subscribing to camera and LiDAR topics, processing data

**Student Progress Checklist**:
- 100+ progress tracking items across all chapters
- Exercise completion checklist
- Troubleshooting guide

### Out of Scope (Planned for Later Modules)

- Advanced physics (deformable bodies, soft robotics simulation)
- Machine learning-based perception training
- Hardware-in-the-loop (HIL) testing
- Digital twin for large-scale fleet management
- Full ROS 2 control stack integration (covered in Module 3)
- Advanced visualization and XR integration (beyond basic Unity setup)

---

## 4. User Scenarios & Acceptance

### Scenario 1: Robot Arm Testing (Before Real Hardware)
**Actor**: Engineering student
**Context**: Completed Module 1, has ROS 2 knowledge, wants to test control algorithms safely
**Flow**:
1. Student loads the 2-joint robot arm model in Gazebo
2. Configures gravity, table collision, and arm dynamics
3. Writes a Python ROS 2 node to move the arm in a circle
4. Simulates the motion 100 times with different parameters
5. Visualizes the trajectory in RViz
6. Feels confident deploying to real hardware ✅

**Acceptance**: Student can successfully run a custom control algorithm in simulation, modify sensor parameters, and understand how simulation differs from reality.

### Scenario 2: Sensor Data Integration
**Actor**: Robotics researcher
**Context**: Needs to test LiDAR-based obstacle avoidance before field deployment
**Flow**:
1. Student adds a simulated LiDAR sensor to the robot model
2. Configures realistic sensor noise and latency
3. Places obstacles in the Gazebo world
4. Writes a Python node that reads LiDAR point clouds and commands obstacle avoidance
5. Tests the algorithm 50+ times with varying obstacle layouts
6. Validates algorithm robustness ✅

**Acceptance**: Student understands how to integrate sensors in simulation, process sensor data, and measure algorithm performance before real-world deployment.

### Scenario 3: Hardware Requirements Understanding
**Actor**: Student with limited GPU access
**Context**: Has older laptop, unsure if they can run simulation
**Flow**:
1. Student reads hardware requirements section
2. Understands that NVIDIA RTX GPU is needed for real-time simulation
3. Learns about CPU fallback (slower but possible)
4. Accesses lab GPU for practical exercises
5. Understands performance trade-offs ✅

**Acceptance**: Student knows the hardware requirements, why GPU is essential, and what to do if they don't have one.

---

## 5. Success Criteria

### Content Quality Metrics
- ✅ **Completeness**: All 3 chapters cover mandatory content (physics, file formats, sensors)
- ✅ **Accuracy**: All simulation code examples run without errors on Gazebo 11+ with ROS 2 Humble
- ✅ **Readability**: Grade 10+ readability level, clear explanations of complex physics concepts
- ✅ **Accessibility**: WCAG 2.1 AA compliant (proper headings, alt text, captions, color contrast)

### Learning Outcomes
- ✅ **Understanding**: 95% of students can explain what a Digital Twin is and why it matters
- ✅ **Hands-on**: 90% of students successfully run a custom control algorithm in Gazebo
- ✅ **Integration**: 85% of students successfully add and read sensor data from simulation
- ✅ **Confidence**: Students report feeling prepared to deploy code to real hardware

### Technical Requirements
- ✅ **Code Accuracy**: 100% of code examples run without errors
- ✅ **Gazebo Integration**: All examples work with Gazebo 11+ and ROS 2 Humble
- ✅ **Hardware Documentation**: Clear guidance on GPU requirements and fallback options
- ✅ **Performance**: Simulation runs in real-time (1000 Hz) on RTX 4070 Ti

### Build & Deployment
- ✅ **Docusaurus Integration**: All 3 chapters render correctly in documentation
- ✅ **Navigation**: Chapters linked in sidebar, prev/next navigation works
- ✅ **Both Locales**: Content builds for English (en) and Urdu (ur) versions
- ✅ **Student Checklist**: 100+ progress tracking items functional

---

## 6. Acceptance Criteria per User Story

### User Story: Complete Module 2 Digital Twin Curriculum

**Given** a student has completed Module 1 (ROS 2 fundamentals)
**When** the student accesses Module 2
**Then** they should be able to:

✅ **Understand Digital Twin Concept**
- Explain what a Digital Twin is
- Describe why simulation matters for robotics
- Compare Gazebo (accuracy) vs. Unity (visuals)
- Know hardware requirements (GPU for real-time simulation)

✅ **Setup and Use Gazebo**
- Load a pre-built robot model in Gazebo
- Modify physics parameters (gravity, damping, friction)
- Spawn obstacles and terrain
- Run simulation and visualize robot motion
- Understand the SDF file format

✅ **Add and Use Sensors**
- Add depth camera to robot URDF
- Add LiDAR sensor with realistic noise
- Add IMU sensor
- Read sensor data from ROS 2 topics
- Visualize sensor output in RViz

✅ **Write Custom Code**
- Write a Python ROS 2 node that controls the simulated robot
- Subscribe to sensor topics and process data
- Implement a simple obstacle avoidance algorithm
- Test the algorithm 100+ times with different parameters
- Compare simulation results with expected physics

✅ **Understand Limitations**
- Know the differences between simulation and reality
- Understand physics simulation approximations
- Recognize when simulation results may not match hardware
- Plan for sim-to-real transfer challenges

---

## 7. Key Entities & Data Models

### Robot Model Entity
- **Name**: 2-joint robot arm
- **Links**: Base, upper arm, forearm
- **Joints**: Shoulder (revolute), elbow (revolute)
- **Sensors**: Depth camera (end-effector), LiDAR (base), IMU (body)
- **Physics**: Mass, inertia, collision shapes
- **Formats**: URDF (with sensor definitions) + SDF (with physics parameters)

### Sensor Data Entities
- **Depth Camera**: 640x480 RGB + Depth images, OpenGL rendering
- **LiDAR**: 1024-point cloud, range measurements, angular resolution
- **IMU**: 3-axis acceleration, 3-axis angular velocity, timestamps

### Gazebo World Entity
- **Ground Plane**: Infinite flat surface with physics
- **Obstacles**: Cubes, cylinders, tables for collision testing
- **Lighting**: Directional light, ambient light
- **Physics Engine**: Gravity (9.81 m/s²), solver parameters

---

## 8. Assumptions

| # | Assumption | Rationale | Risk |
|---|-----------|-----------|------|
| A1 | Students have completed Module 1 | Prerequisites are documented | Low |
| A2 | Students have access to NVIDIA RTX GPU (personal or lab) | Gazebo real-time simulation requires GPU acceleration | Medium |
| A3 | Gazebo 11+ installed with ROS 2 Humble | Standard robotics environment | Low |
| A4 | Python 3.10+ available | Required for ROS 2 nodes | Low |
| A5 | Basic Linux command line knowledge | Inherited from Module 1 | Low |
| A6 | Git for downloading code examples | Standard practice | Low |

---

## 9. Dependencies

### Internal Dependencies
- ✅ **Module 1**: Prerequisite (ROS 2 fundamentals, Python basics, Linux CLI)
- ✅ **Module 1 URDF Knowledge**: Students understand link/joint definitions
- ✅ **Module 1 Publishing/Subscribing**: ROS 2 communication patterns used

### External Dependencies
- **Gazebo 11+**: Open-source physics simulator, ROS 2 integrated (gazebo_ros package)
- **ROS 2 Humble**: LTS release with Gazebo integration
- **NVIDIA CUDA Toolkit**: For GPU-accelerated physics
- **Python rclpy**: ROS 2 Python client library
- **OpenGL Libraries**: For sensor rendering (depth camera)

---

## 10. Constraints & Non-Functional Requirements

### Performance
- Simulation must run in real-time (≥1000 Hz) on RTX 4070 Ti
- Camera rendering: ≥30 FPS for visual feedback
- LiDAR point cloud generation: <10ms latency

### Compatibility
- Works on Ubuntu 22.04 LTS (primary)
- Compatible with WSL2 (with GPU pass-through)
- Partial compatibility with Docker (GPU mounting required)
- Not supported: native Windows, macOS (Gazebo limitations)

### Accessibility
- Grade 10+ readability
- WCAG 2.1 AA compliant
- Clear diagrams with alt text
- Color-blind friendly visuals

---

## 11. Open Questions & Clarifications

[NEEDS CLARIFICATION: Should Module 2 include **Unity integration** (visual simulation for XR), or focus exclusively on Gazebo (physics-first approach)? This significantly impacts scope and deliverables.]

[NEEDS CLARIFICATION: What is the **ROS 2 control stack** integration depth? Should we teach ros2_control framework, or keep it simple (raw joint commands)?]

[NEEDS CLARIFICATION: Should we include **sim-to-real transfer** techniques (domain randomization, system identification), or defer to Module 3?]

---

## 12. Success Metrics Summary

| Metric | Target | Measurable By |
|--------|--------|---------------|
| Content Completeness | 100% of mandatory sections | Checklist review |
| Code Accuracy | 100% runnable | Test execution on RTX GPU |
| Student Understanding | 90%+ complete Chapter 3 exercises | Checklist submission |
| Readability | Grade 10+ | Automated readability analysis |
| Accessibility | WCAG 2.1 AA | Lighthouse/axe audits |
| Build Success | Both en & ur locales | npm run build |

---

## 13. Next Steps

1. **Clarification**: Resolve the 3 open questions above
2. **Planning**: Run `/sp.plan` to create architecture and implementation plan
3. **Task Decomposition**: Generate 20-30 atomic tasks for content creation
4. **Implementation**: Execute task-by-task with Phase structure (Setup → Foundational → User Stories → Polish)
5. **Deployment**: Build, test, and merge to production

---

**Status**: ✅ Ready for `/sp.clarify` (pending clarification of 3 open questions)
**Estimated Effort**: 30-40 hours (to be refined during planning)
**Success Definition**: All acceptance criteria met, students confident running simulations on real robots
