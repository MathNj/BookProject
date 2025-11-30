---
title: Student Progress Checklist - Module 1
sidebar_label: Progress Checklist
---

# Module 1: Student Progress Checklist

Use this checklist to track your progress through **Module 1: The Robotic Nervous System (ROS 2)**. Check off each item as you complete it!

---

## Chapter 1: Getting Started

**File**: 00-getting-started.md

### Knowledge Requirements
- [ ] Review your Python basics (variables, functions, loops, classes)
- [ ] Understand basic Linux command line (cd, ls, mkdir, chmod)
- [ ] Know how to open a terminal on your system

### System Requirements
- [ ] Install Ubuntu 22.04 LTS (native, Docker, or WSL2)
- [ ] Verify you have at least 5GB of free disk space
- [ ] Confirm Python 3.10+ is installed (`python3 --version`)
- [ ] Have Git installed for code examples

### Learning Outcomes for Chapter 1
- [ ] I can identify the 5 modules in this textbook
- [ ] I understand what prerequisites I need
- [ ] I know which platform I'll be using (Ubuntu native, WSL2, Docker, etc.)
- [ ] I'm ready to install ROS 2 Humble

---

## Chapter 2: Introduction to ROS 2 - The Robotic Nervous System

**File**: 01-intro-to-ros2.md

### Nervous System Analogy
- [ ] I understand the Brain ↔ Computer mapping
- [ ] I can explain how Neurons ↔ Nodes work
- [ ] I understand Nerves ↔ Topics as communication pathways
- [ ] I can map Muscles ↔ Actuators

### ROS 2 Concepts
- [ ] I know what a **Node** is and can give 2 examples
- [ ] I understand what a **Topic** is and how pub/sub works
- [ ] I can explain what **Services** are (request/response pattern)
- [ ] I can read a **Node Graph** and identify publishers and subscribers

### Real-World Applications
- [ ] I can describe a mobile robot's node architecture
- [ ] I can describe a robotic arm's node architecture
- [ ] I understand distributed robotics with multiple nodes

### Exercise
- [ ] I completed the "Map the Nodes" exercise
- [ ] I drew a node graph for a custom robot scenario

---

## Chapter 3: Installation - Setting Up ROS 2 Humble

**File**: 02-installation.md

### Prerequisites
- [ ] My system meets all hardware requirements (5GB disk, Ubuntu 22.04)
- [ ] I understand which platform I'm using (Native/WSL2/Docker)
- [ ] I have administrator access to install packages

### Installation (Select Your Platform)

#### Option A: Ubuntu 22.04 Native
- [ ] I followed the main installation steps
- [ ] `ros2 --version` shows ROS 2 Humble
- [ ] Setup script is in my `.bashrc`

#### Option B: Windows with WSL2
- [ ] WSL2 is installed with Ubuntu 22.04
- [ ] I ran the installation inside WSL2 terminal
- [ ] X11 forwarding is configured (if using RViz)
- [ ] `ros2 --version` works in WSL2

#### Option C: macOS with Docker
- [ ] Docker Desktop for macOS is installed
- [ ] I can run ROS 2 in a Docker container
- [ ] I can access files from my Mac via volume mounting

#### Option D: Windows with Docker Desktop
- [ ] Docker Desktop for Windows is installed
- [ ] I can run ROS 2 in a Docker container
- [ ] Container has sufficient RAM allocated (8GB+)

### First Node Test
- [ ] **Terminal 1**: I ran `ros2 run demo_nodes_cpp talker`
- [ ] **Terminal 2**: I ran `ros2 run demo_nodes_cpp listener`
- [ ] I saw the talker publishing "Hello World" messages
- [ ] I saw the listener receiving the messages
- [ ] The demo proved pub/sub communication works! ✅

### Troubleshooting (If Needed)
- [ ] If I got "Command ros2 not found", I sourced the setup script
- [ ] If I got "Unable to locate package", I re-added the ROS 2 repository
- [ ] If I had DOMAIN_ID conflicts, I set `ROS_DOMAIN_ID=42`
- [ ] If I had rclpy import errors, I sourced the setup script in Python terminal
- [ ] I found the solution in the Troubleshooting section

---

## Chapter 4: Nodes and Topics - Write Your First Python Nodes

**File**: 03-nodes-and-topics.md

### Understanding the Code
- [ ] I read through `minimal_publisher.py` line by line
- [ ] I understand the `MinimalPublisher` class structure
- [ ] I know how `create_publisher()` and `create_timer()` work
- [ ] I read through `minimal_subscriber.py` line by line
- [ ] I understand callback functions and `create_subscription()`

### Running the Examples
- [ ] I downloaded `minimal_publisher.py` and `minimal_subscriber.py`
- [ ] **Terminal 1**: I ran the publisher node: `python3 minimal_publisher.py`
- [ ] **Terminal 2**: I ran the subscriber node: `python3 minimal_subscriber.py`
- [ ] The publisher sent messages every 0.5 seconds
- [ ] The subscriber received and displayed all messages
- [ ] Both nodes worked together without errors ✅

### Debugging
- [ ] I used `ros2 node list` to see active nodes
- [ ] I used `ros2 topic list` to see all topics
- [ ] I used `ros2 topic echo /chatter` to inspect messages

### Modification Exercises (Choose at least 2)
- [ ] **Exercise 1**: Changed the message frequency from 0.5s to 1.0s
  - [ ] I verified the publisher sends slower messages
- [ ] **Exercise 2**: Changed "Hello World" to a custom message
  - [ ] I verified the subscriber displays my custom message
- [ ] **Exercise 3**: Modified the topic name from `/chatter` to `/my_topic`
  - [ ] I updated both publisher and subscriber
  - [ ] I verified they communicate on the new topic
- [ ] **Exercise 4**: Created a second publisher on the same topic
  - [ ] Both publishers sent messages
  - [ ] The subscriber received from both

---

## Chapter 5: URDF - Robot Description Format

**File**: 04-urdf-modeling.md

### URDF Concepts
- [ ] I understand what URDF is (XML format for robot structure)
- [ ] I know the 2 main URDF components: **Links** and **Joints**
- [ ] I understand link attributes: mass, inertia, visual, collision
- [ ] I understand joint types: revolute (rotates), prismatic (slides), fixed

### Reading URDF
- [ ] I read through `simple_arm.urdf` line by line
- [ ] I can explain the base link (ground reference)
- [ ] I can explain link1 and link2 (arm segments)
- [ ] I understand joint definitions with axes and limits
- [ ] I can calculate arm reach: sum of link lengths

### Visualization (With RViz)
- [ ] I loaded `simple_arm.urdf` in RViz
- [ ] I could see the 3D robot structure
- [ ] I could rotate each joint and see the arm move
- [ ] I verified joint limits prevent over-rotation

### Modification Exercises (Choose at least 1)
- [ ] **Exercise 1**: Changed link lengths
  - [ ] Modified `<origin xyz="0 0 0.15">` to a new value
  - [ ] Reloaded in RViz and saw the new dimensions
- [ ] **Exercise 2**: Changed joint limits
  - [ ] Modified joint angle limits from -90° to +45°
  - [ ] Verified the joint won't rotate beyond limits in RViz
- [ ] **Exercise 3**: Added a 3rd link (hand/gripper)
  - [ ] Extended the arm with a new link and joint
  - [ ] Verified it attached properly
- [ ] **Exercise 4**: Changed visual colors
  - [ ] Modified `<color rgba="...">` values
  - [ ] Saw the new colors in RViz

---

## Module Completion

### Final Verification ✅
- [ ] I completed all 5 chapters
- [ ] I ran all code examples successfully
- [ ] I understand the nervous system analogy
- [ ] I can create simple ROS 2 nodes in Python
- [ ] I can read and modify URDF files
- [ ] I'm ready for Module 2: Digital Twin (Simulation)

### Reflection
- [ ] My biggest learning from Module 1 was: _________________
- [ ] The hardest concept was: _________________
- [ ] The most useful skill was: _________________

### Next Steps
1. **Module 2** - Learn about simulation with Isaac Sim
2. **Practice** - Create your own robot nodes and URDF
3. **Share** - Show your code to instructors or peers
4. **Experiment** - Combine multiple nodes for complex behaviors

---

## Support Resources

### If You Get Stuck
- **Installation help**: See Chapter 3's Troubleshooting section (8 common issues)
- **Code help**: All code examples have detailed comments
- **Concept help**: Re-read the nervous system analogy in Chapter 2
- **ROS 2 docs**: https://docs.ros.org/en/humble/

### Learning Tips
- ✅ **Type out code** - Don't copy/paste. Typing builds muscle memory.
- ✅ **Modify examples** - Change values and see what happens
- ✅ **Debug systematically** - Use `ros2 node list`, `ros2 topic list`, `ros2 topic echo`
- ✅ **Draw diagrams** - Sketch node graphs to understand architecture
- ✅ **Explain to someone** - Teaching others cements your learning

---

**Last Updated**: November 30, 2025
**Module**: Module 1 - The Robotic Nervous System (ROS 2)
**Status**: Ready for Students ✅
