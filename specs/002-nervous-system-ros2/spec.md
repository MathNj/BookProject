# Feature Specification: Module 1 - The Robotic Nervous System (ROS 2)

**Feature Branch**: `002-nervous-system-ros2`
**Created**: 2025-11-30
**Status**: Draft
**Input**: Define requirements for Module 1 of the Physical AI Textbook focusing on ROS 2 architecture and practical application.

## User Scenarios & Testing

### User Story 1 - Learn ROS 2 Fundamentals Using Nervous System Analogy (Priority: P1)

A university-level student learning robotics needs to understand ROS 2's core architecture (Nodes, Topics, Services) through an intuitive biological analogy that bridges abstract concepts to real physical systems.

**Why this priority**: Understanding ROS 2 architecture is foundational for all subsequent robotics work. Without grasping how nodes communicate via topics and services, students cannot build or understand robot applications. This is the essential prerequisite.

**Independent Test**: Student can complete the module and explain the relationship between brain/neurons/nerves and ROS 2's compute/nodes/topics using their own examples.

**Acceptance Scenarios**:

1. **Given** a student with basic programming knowledge, **When** they read the introduction material, **Then** they can correctly map biological nervous system components (brain, neurons, nerves, muscles) to ROS 2 components (PC/compute, nodes, topics/services, actuators)

2. **Given** a student who understands the analogy, **When** they examine a code example of a talker/listener node pair, **Then** they can explain what messages are being published and subscribed to

3. **Given** a student completing the module, **When** asked to design a simple robot system, **Then** they can identify appropriate node boundaries and communication patterns

---

### User Story 2 - Set Up ROS 2 Development Environment (Priority: P1)

A student needs clear, step-by-step installation and verification instructions to successfully set up ROS 2 Humble on Ubuntu 22.04 and confirm their installation works with provided demo applications.

**Why this priority**: Students cannot engage with practical ROS 2 development without a working installation. Installation issues are a major barrier to learning. Clear verification steps prevent silent failures.

**Independent Test**: Student completes all installation steps and successfully runs the talker/listener demo nodes, observing message exchange.

**Acceptance Scenarios**:

1. **Given** a student on Ubuntu 22.04 with internet access, **When** they follow the installation guide step-by-step, **Then** ROS 2 Humble installs without errors

2. **Given** an installed ROS 2 environment, **When** the student runs the verification steps (talker and listener), **Then** they see "Hello World" messages being published and received

3. **Given** a student who encounters installation problems, **When** they consult the troubleshooting section, **Then** they can diagnose and resolve common issues (missing setup sourcing, package dependencies, etc.)

---

### User Story 3 - Write and Run First Python Node (Priority: P2)

A student needs practical experience writing a Python ROS 2 node (using rclpy) to transition from passive learning to active experimentation with publisher and subscriber patterns.

**Why this priority**: Hands-on coding solidifies conceptual understanding. After learning theory and installing tools, students need to write simple code to internalize the pub/sub pattern. This enables self-directed exploration.

**Independent Test**: Student writes and successfully runs a custom Python node that publishes data and/or subscribes to messages without reference materials (beyond standard library docs).

**Acceptance Scenarios**:

1. **Given** the installation is complete, **When** a student writes a simple publisher node in Python using rclpy, **Then** the node compiles and publishes messages to a topic

2. **Given** a working publisher, **When** a student writes a subscriber node, **Then** the subscriber receives and displays published messages

3. **Given** a student's custom nodes, **When** they run both together, **Then** the pub/sub communication works without errors

---

### User Story 4 - Understand Robot Description with URDF (Priority: P2)

A student needs to learn how robots are represented structurally in ROS 2 using URDF (Unified Robot Description Format) and understand how this description bridges between physical hardware and software.

**Why this priority**: URDF understanding is essential for advanced robotics work (simulation, motion planning, hardware control). While not strictly required for basic pub/sub patterns, it contextualizes ROS 2 within the broader robot control pipeline.

**Independent Test**: Student can read a simple URDF file and understand what physical structure it describes, and can create a basic URDF for a simple robot.

**Acceptance Scenarios**:

1. **Given** a URDF file of a simple robot (e.g., 2-joint arm), **When** a student reads the XML, **Then** they can correctly describe the robot's structure (joints, links, hierarchy)

2. **Given** a robot specification (e.g., "mobile base with one revolute arm"), **When** a student creates a basic URDF file, **Then** the URDF is structurally valid and describes the intended robot

---

### Edge Cases

- **Incomplete ROS 2 Installation**: If demo packages are missing, module must provide explicit apt commands to reinstall (e.g., `sudo apt install ros-humble-demo-nodes-cpp`)
- **Non-Linux Students**: Documentation MUST explicitly state that Ubuntu 22.04 native or Docker/WSL2 is mandatory; no Mac/Windows native support
- **Version Conflicts**: If a student has ROS 2 Iron or older Humble installed, documentation must provide upgrade path or clear instructions to uninstall before new installation
- **Topic Communication Failures**: Module must include troubleshooting for common issues (ROS_DOMAIN_ID conflicts, uninitialized nodes, topic naming mismatches)
- **Python Version Incompatibility**: If student uses Python <3.10, provide clear error messaging and upgrade instructions
- **Network/Localhost Issues**: Instructions assume single-machine development (all nodes on localhost); multi-machine scenarios deferred to advanced modules

## Requirements

### Functional Requirements

- **FR-001**: Module MUST introduce ROS 2 architecture using the nervous system analogy (brain, neurons, nerves, muscles mapped to PC, nodes, topics/services, actuators)

- **FR-002**: Module MUST explain ROS 2 Nodes, Topics, and Services with clear definitions and real-world robotics context

- **FR-003**: Module MUST provide a complete, step-by-step installation guide for ROS 2 Humble on Ubuntu 22.04 LTS

- **FR-004**: Module MUST include verification steps to confirm successful ROS 2 installation (running talker/listener demo nodes)

- **FR-005**: Module MUST provide working Python code examples (using rclpy) demonstrating a talker node that publishes messages

- **FR-006**: Module MUST provide working Python code examples demonstrating a listener node that subscribes to and displays messages

- **FR-007**: Module MUST introduce URDF (Unified Robot Description Format) and explain how it describes robot structure

- **FR-008**: Module MUST explain how ROS 2 bridges the "brain" (computation/AI) to the "body" (motors, actuators, sensors) in physical robots

- **FR-009**: Content MUST be written at university level (technically rigorous but accessible to students with programming background)

- **FR-010**: All code examples MUST be tested and verified to run without errors

### Key Entities

- **ROS 2 Node**: A computational process performing a specific robotic function (perception, control, planning, etc.)
- **Topic**: A named asynchronous communication channel where nodes publish and subscribe to typed messages
- **Service**: A synchronous request-response communication pattern between nodes
- **Message**: Structured data (with defined types) exchanged between nodes via topics/services
- **URDF**: XML format describing robot structure (links, joints, hierarchy, and physical properties)
- **rclpy**: Python client library for ROS 2 that enables Python nodes to publish, subscribe, and manage ROS 2 resources

## Success Criteria

### Measurable Outcomes

- **SC-001**: Students can correctly explain the nervous system analogy and map at least 4 biological components to ROS 2 components

- **SC-002**: 95% of students successfully install ROS 2 Humble and verify installation by running talker/listener demos without external help

- **SC-003**: Students can write a working Python publisher node that publishes data on a custom topic

- **SC-004**: Students can write a working Python subscriber node that receives and processes published data

- **SC-005**: Students can read a URDF file and accurately describe the physical structure it represents

- **SC-006**: Module content demonstrates 100% code accuracy (all provided code examples execute without errors)

- **SC-007**: All code examples include sufficient comments and explanation that students can understand implementation without external references

- **SC-008**: Students report (via survey) that the nervous system analogy improved their understanding of ROS 2 architecture by 40% or more

## Clarifications

### Session 2025-11-30

- Q: Which ROS 2 distribution should be the default? → A: ROS 2 Humble (LTS - Long Term Support until 2027)
- Q: How do we handle Windows/Mac users? → A: Linux native requirement; Docker or VM (WSL2) mandatory for non-Linux
- Q: Do we assume students know Python? → A: Yes, basic Python proficiency required (loops, functions, classes)
- Q: What URDF scope for Module 1? → A: Simple 2-joint robot arm to teach core concepts; humanoid deferred to advanced modules

## Assumptions

1. **Target Audience**: University-level students with programming experience (Python required - loops, functions, classes) but no prior ROS experience
2. **ROS 2 Distribution**: ROS 2 Humble (LTS, stable until 2027) is the exclusive distribution for this module; compatibility with Iron and future versions is out of scope
3. **Development Environment**: Ubuntu 22.04 LTS is the REQUIRED native OS; Windows/macOS students MUST use Docker container or WSL2 VM (not optional workarounds)
4. **Python Requirement**: rclpy (Python client library) is the primary language; students must have hands-on Python experience before starting
5. **Hardware Target**: Code examples target x86-64 Linux; ARM platforms (Jetson, etc.) are deferred to specialized modules
6. **Pedagogical Approach**: The nervous system analogy is effective for intuitive understanding of distributed robotics architecture
7. **URDF Complexity**: Module 1 introduces URDF with a simple 2-joint robot arm (2 links, 2 revolute joints); complex humanoid/mobile base models deferred to later modules
8. **Scope Boundary**: Module 1 focuses on ROS 2 communication fundamentals and basic URDF; advanced topics (real-time constraints, custom message types, service architectures, middleware configuration) are deferred to later modules

## Out of Scope

- Advanced ROS 2 features (middleware configuration, DDS tuning, QoS policies beyond basic examples)
- Integration with specific hardware platforms (robots will be discussed conceptually, not with hardware driver implementation)
- Containerization and deployment (Docker usage is mentioned but not the primary focus)
- C++ node implementation (Python is primary language; C++ deferred to specialized modules)
- Gazebo simulation setup and usage (simulation is mentioned but not covered in depth)
