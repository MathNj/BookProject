# Module 2: Digital Twin - Student Progress Checklist

Track your progress through Module 2. This checklist has **150+ items** across all chapters.

**Navigation**:
- [Chapter 0: Getting Started](#chapter-0-getting-started) (16 items)
- [Chapter 1: Digital Twin Concepts](#chapter-1-introduction-to-digital-twin-simulation) (19 items)
- [Chapter 2: Gazebo Setup](#chapter-2-setting-up-gazebo-fortress) (30 items)
- [Chapter 3: URDF vs SDF](#chapter-3-urdf-vs-sdf-reference) (20 items)
- [Chapter 4: Sensors](#chapter-4-simulating-sensors-in-gazebo) (50 items)
- [Chapter 5: Control](#chapter-5-robot-control-integrated-lesson) (20 items)
- [Chapter 6: Unity (Optional)](#chapter-6-optional-unity-for-visualization) (10 items)
- [Reflection & Assessment](#reflection--assessment) (15+ items)

---

## Chapter 0: Getting Started

### Prerequisites Verification
- [ ] **0.1** Python 3.8+ installed (`python3 --version`)
- [ ] **0.2** ROS 2 Humble installed (`ros2 --version`)
- [ ] **0.3** Can launch RViz (`rviz2` opens without errors)
- [ ] **0.4** Basic Linux CLI skills (understand `cd`, `ls`, `grep`, `cat`)
- [ ] **0.5** Completed Module 1: The Nervous System

### Environment Setup
- [ ] **0.6** Clone textbook repository to ~/textbook
- [ ] **0.7** Created ~/ros2_ws directory for working files
- [ ] **0.8** Updated ~/.bashrc to source ROS 2 setup.bash
- [ ] **0.9** Run `which ros2` - confirms ROS 2 in PATH
- [ ] **0.10** Install required packages: `sudo apt install ros-humble-gazebo-ros-pkgs`

### Hardware Verification
- [ ] **0.11** GPU available: `nvidia-smi` shows GPU with greater than 4GB VRAM
- [ ] **0.12** CUDA toolkit version matches GPU: `nvcc --version`
- [ ] **0.13** System RAM minimum 16GB: `free -h`
- [ ] **0.14** SSD disk space greater than 50GB available
- [ ] **0.15** CPU has greater than 4 cores: `nproc`
- [ ] **0.16** Ran gazebo-install-verify.sh - all 5 tests pass ✅

**Checkpoint**: All 16 items checked ✅

---

## Chapter 1: Introduction to Digital Twin Simulation

### Conceptual Understanding
- [ ] **1.1** Define "Digital Twin" in own words (5+ sentences)
- [ ] **1.2** List 3 reasons why simulation is essential for robotics
- [ ] **1.3** Compare safety, cost, and speed benefits (sim vs hardware)
- [ ] **1.4** Understand real-time simulation target: 1000 Hz on GPU
- [ ] **1.5** Explain GPU requirement (1000 Hz vs 50 Hz on CPU)

### Sim-to-Real Gap
- [ ] **1.6** Identify 4 categories of sim-to-real gap
- [ ] **1.7** List 2 physics approximations in Gazebo
- [ ] **1.8** Explain sensor idealization (perfect rays vs real noise)
- [ ] **1.9** Describe latency differences (simulation vs real hardware)
- [ ] **1.10** Define domain randomization (preview for Module 3)

### Real-World Applications
- [ ] **1.11** Summarize Boston Dynamics Atlas example (time saved)
- [ ] **1.12** Explain NVIDIA Isaac Sim manufacturing use case
- [ ] **1.13** Describe ABB flexible manufacturing example
- [ ] **1.14** Identify similar robotics problems in own research/work
- [ ] **1.15** Create your own "why simulation?" motivation statement

### Visualization & Diagrams
- [ ] **1.16** Study all 6 ASCII diagrams in resources
- [ ] **1.17** Explain Digital Twin workflow diagram
- [ ] **1.18** Understand performance comparison GPU vs CPU
- [ ] **1.19** Review Gazebo vs Unity comparison matrix
- [ ] **1.20** (Optional) Sketch Digital Twin concept on paper

**Checkpoint**: Digital Twin concepts mastered ✅

---

## Chapter 2: Setting Up Gazebo Fortress

### Installation & Verification
- [ ] **2.1** Update package manager: `sudo apt update && sudo apt upgrade`
- [ ] **2.2** Install ROS 2 Humble: `sudo apt install ros-humble-desktop`
- [ ] **2.3** Install Gazebo bridge: `sudo apt install ros-humble-ros-gz`
- [ ] **2.4** Verify Gazebo version: `gazebo --version` (shows 7.x+)
- [ ] **2.5** Test ROS 2 + Gazebo: `ros2 launch gazebo_ros gazebo.launch.py`

### Understanding SDF Format
- [ ] **2.6** Explain "SDF" acronym and full name
- [ ] **2.7** Identify 3 differences between URDF and SDF
- [ ] **2.8** Read URDF example in chapter and understand each tag
- [ ] **2.9** Read SDF example in chapter and understand structure
- [ ] **2.10** Explain physics tag: max_step_size, real_time_update_rate

### Building Your First World
- [ ] **2.11** Understand gravity: `<gravity>0 0 -9.81</gravity>` means 9.81 m/s² downward
- [ ] **2.12** Understand lighting setup (directional sun, intensity, direction)
- [ ] **2.13** Create ground plane with static=true, collision geometry
- [ ] **2.14** Add static box obstacle at position (1, 0, 0.4)
- [ ] **2.15** Launch simple_world.sdf without errors: `gazebo simple_world.sdf`

### Hands-On: World Modification
- [ ] **2.16** **Exercise 1**: Change obstacle size to `<size>2 1 0.5</size>`
- [ ] **2.17** Relaunch and verify taller, narrower obstacles appear
- [ ] **2.18** **Exercise 2**: Add third obstacle at position (3, 0, 0.5)
- [ ] **2.19** Verify 3 obstacles visible in Gazebo GUI
- [ ] **2.20** **Exercise 3**: Change friction from 0.5 to 0.1 (slippery)
- [ ] **2.21** Spawn robot and observe sliding on low-friction ground
- [ ] **2.22** Change friction back to 1.0, observe increased grip

### Physics Tuning & Performance
- [ ] **2.23** Understand ODE physics solver configuration
- [ ] **2.24** Explain step size impact (smaller = more accurate, slower)
- [ ] **2.25** Tune for fast robots: `<max_step_size>0.0005</max_step_size>`
- [ ] **2.26** Tune for slow robots: `<max_step_size>0.002</max_step_size>`
- [ ] **2.27** Measure physics performance: check Gazebo GUI Hz counter
- [ ] **2.28** Achieve greater than 50 Hz physics on your hardware
- [ ] **2.29** Monitor GPU usage while running Gazebo (nvidia-smi)
- [ ] **2.30** Log performance metrics: FPS, Hz, GPU temp, memory

### Troubleshooting
- [ ] **2.31** Fix: "gazebo: command not found" → reinstall gazebo
- [ ] **2.32** Fix: "Gazebo window doesn't appear" → check GPU drivers
- [ ] **2.33** Diagnose: "Physics too slow (less than 20 Hz)" → check GPU usage
- [ ] **2.34** Fix: "Robot jittery" → reduce max_step_size

**Checkpoint**: Gazebo and SDF working on your machine ✅

---

## Chapter 3: URDF vs SDF Reference

### URDF Fundamentals
- [ ] **3.1** Explain URDF: "Unified Robot Description Format"
- [ ] **3.2** List 5 components: link, joint, visual, collision, inertial
- [ ] **3.3** Write simple link with mass and inertia matrix
- [ ] **3.4** Write revolute joint with limits and axis
- [ ] **3.5** Create minimal 2-link robot URDF from scratch
- [ ] **3.6** Explain why joint limits matter (safety, realism)
- [ ] **3.7** Understand inertial matrix components (ixx, iyy, izz)

### SDF Fundamentals
- [ ] **3.8** Explain SDF: "Simulation Description Format"
- [ ] **3.9** Compare SDF vs URDF in 7-criteria table
- [ ] **3.10** Understand world-level configuration (physics, gravity, lights)
- [ ] **3.11** Write SDF model definition for box obstacle
- [ ] **3.12** Include robot model inside SDF world file

### Conversion & Compatibility
- [ ] **3.13** Explain auto-conversion: URDF → SDF (Gazebo does it)
- [ ] **3.14** Manually convert URDF to SDF: `gz sdf print robot.urdf`
- [ ] **3.15** Reference URDF inside SDF: `<urdf uri="file:///path/robot.urdf"/>`
- [ ] **3.16** Load URDF in Gazebo and verify joint motion
- [ ] **3.17** Check URDF syntax errors with urdf_parser tool
- [ ] **3.18** Debug URDF issues: joint limits, frame transforms
- [ ] **3.19** Create matching URDF → SDF comparison document
- [ ] **3.20** Understand when to use URDF (planning) vs SDF (simulation)

**Checkpoint**: URDF and SDF completely understood ✅

---

## Chapter 4: Simulating Sensors in Gazebo

### LiDAR Configuration (10 items)
- [ ] **4.1** Understand LiDAR: 1024 rays, 360°, distance measurement
- [ ] **4.2** Configure ray samples: balance 256 (fast) vs 2048 (accurate)
- [ ] **4.3** Set range: min 0.1m, max 40m, resolution 0.01m
- [ ] **4.4** Add Gaussian noise: `<stddev>0.01</stddev>` (1cm)
- [ ] **4.5** Subscribe to LaserScan in Python: read msg.ranges
- [ ] **4.6** **Exercise 1**: Modify samples from 1024 to 256
- [ ] **4.7** Measure performance improvement (2-3x faster)
- [ ] **4.8** Calculate min, max, mean distances from LiDAR
- [ ] **4.9** Find nearest obstacle angle from LiDAR data
- [ ] **4.10** Visualize LiDAR as PointCloud2 in RViz

### Depth Camera Configuration (10 items)
- [ ] **4.11** Understand depth camera: RGB image + depth map
- [ ] **4.12** Configure resolution: 640x480 standard (balance speed/quality)
- [ ] **4.13** Set field of view: 60° horizontal (1.047 rad)
- [ ] **4.14** Add lens distortion: barrel effect (k1=-0.25)
- [ ] **4.15** Configure clipping planes: near 0.01m, far 10m
- [ ] **4.16** **Exercise 2**: Reduce resolution to 320x240
- [ ] **4.17** Measure FPS improvement (2-3x increase)
- [ ] **4.18** Subscribe to RGB image in Python
- [ ] **4.19** Convert ROS Image to OpenCV using cv_bridge
- [ ] **4.20** Subscribe to depth map and find nearest pixel

### IMU Configuration (10 items)
- [ ] **4.21** Understand IMU: 3-axis accelerometer + 3-axis gyroscope
- [ ] **4.22** Configure accelerometer noise: ±0.002 m/s² (realistic)
- [ ] **4.23** Configure gyroscope noise: ±0.0001 rad/s
- [ ] **4.24** Understand gravity effect: stationary = (0, 0, 9.81)
- [ ] **4.25** Subscribe to Imu message in Python
- [ ] **4.26** Read accelerometer and calculate total magnitude
- [ ] **4.27** Read gyroscope and estimate pitch angle
- [ ] **4.28** Verify IMU publishes at 100+ Hz
- [ ] **4.29** **Exercise 3**: Increase noise to 5% for robustness testing
- [ ] **4.30** Observe and record noise effects on readings

### RViz Visualization (10 items)
- [ ] **4.31** Load provided rviz_sensor_config.rviz file
- [ ] **4.32** Configure PointCloud2 display for LiDAR
- [ ] **4.33** Add Image display for RGB camera
- [ ] **4.34** Add Image display for depth map (grayscale)
- [ ] **4.35** Add Imu display showing accel + gyro vectors
- [ ] **4.36** Add TF (coordinate frame) visualization
- [ ] **4.37** Add RobotModel display showing URDF
- [ ] **4.38** Save custom RViz configuration to file
- [ ] **4.39** Load RViz config on every startup
- [ ] **4.40** Screenshot final RViz window with all displays

### Sensor Integration (10 items)
- [ ] **4.41** Run sensor_reader.py alongside Gazebo
- [ ] **4.42** Verify all 3 sensors publishing data simultaneously
- [ ] **4.43** Implement sensor fusion (combine LiDAR + IMU)
- [ ] **4.44** Calculate obstacle distance + robot orientation together
- [ ] **4.45** Implement control loop: adjust commands based on sensors
- [ ] **4.46** Subscribe to multiple topics with QoS profile
- [ ] **4.47** Handle sensor timestamp synchronization
- [ ] **4.48** Test sensor data reliability (greater than 99% arrival)
- [ ] **4.49** Measure sensor data latency (time from physics to topic)
- [ ] **4.50** Document sensor configurations in notebook

### Performance Tuning (10 items)
- [ ] **4.51** Understand LiDAR samples trade-off (rays vs Hz)
- [ ] **4.52** Understand camera resolution trade-off (quality vs FPS)
- [ ] **4.53** Profile GPU memory usage with all sensors
- [ ] **4.54** Benchmark: greater than 50 Hz physics + all sensors
- [ ] **4.55** Read sensor-tuning-guide.md completely
- [ ] **4.56** Pick optimal sensor settings for your hardware
- [ ] **4.57** Test all 3 configurations: max speed, balanced, high accuracy
- [ ] **4.58** Document performance: Hz, FPS, memory, temperature
- [ ] **4.59** Compare sensor settings vs physics performance
- [ ] **4.60** Explain trade-offs in your own words

### Troubleshooting & Advanced (10 items)
- [ ] **4.61** Fix: "No LiDAR topic" → check URDF, restart
- [ ] **4.62** Fix: "LiDAR too slow" → reduce samples from 1024 to 512
- [ ] **4.63** Fix: "Camera image black" → increase world lighting
- [ ] **4.64** Fix: "IMU all zeros" → check gravity, robot moving
- [ ] **4.65** Read sensor-troubleshooting.md thoroughly
- [ ] **4.66** Create sensor debugging checklist
- [ ] **4.67** Test sensor robustness with added noise
- [ ] **4.68** Stress test: run sensors for greater than 10 minutes
- [ ] **4.69** Document any sensor issues found
- [ ] **4.70** Ask for help on ROS Discourse if stuck

**Checkpoint**: All 3 sensors working perfectly ✅

---

## Chapter 5: Robot Control (Integrated Lesson)

### Arm Control Implementation (10 items)
- [ ] **5.1** Understand trajectory-based control (JointTrajectory messages)
- [ ] **5.2** Study arm_control.py: 4 waypoints, smooth interpolation
- [ ] **5.3** Run arm_control.py and observe robot moving through poses
- [ ] **5.4** Verify control loop runs at 100 Hz
- [ ] **5.5** Understand feedback loop: subscribe to /joint_states
- [ ] **5.6** **Exercise 1**: Add 5th waypoint to control sequence
- [ ] **5.7** Change waypoint durations (speed up or slow down)
- [ ] **5.8** Implement position tolerance check (when reached?)
- [ ] **5.9** Add velocity ramping (smooth acceleration/deceleration)
- [ ] **5.10** Visualize joint trajectory in RViz with plot markers

### Sensor-Based Control (10 items)
- [ ] **5.11** Modify arm_control.py to read LiDAR feedback
- [ ] **5.12** Stop arm if obstacle closer than 0.5m
- [ ] **5.13** Use camera to detect target object (color-based blob)
- [ ] **5.14** Adjust arm movement based on IMU readings
- [ ] **5.15** Implement safety check: stop if accel greater than threshold
- [ ] **5.16** Log all sensor readings during control task
- [ ] **5.17** Analyze logs: Which sensor fired first? Timing?
- [ ] **5.18** Test control robustness (does it work with noise?)
- [ ] **5.19** Compare performance: with/without sensor feedback
- [ ] **5.20** Document control architecture in design document

**Checkpoint**: Full integrated system working ✅

---

## Chapter 6 (Optional): Unity for Visualization

### Decision Making
- [ ] **6.1** Read "when-to-use-unity" section carefully
- [ ] **6.2** Decide: Does my project need Unity?
- [ ] **6.3** Understand: Physics in Gazebo, visuals in Unity

### Setup (if choosing Unity)
- [ ] **6.4** Download Unity 2022.3 LTS
- [ ] **6.5** Clone ROS-TCP-Connector repository
- [ ] **6.6** Build ros_tcp_endpoint on Linux
- [ ] **6.7** Launch Gazebo + ROS-TCP bridge
- [ ] **6.8** Create Unity project, import ROS2 plugin
- [ ] **6.9** Import robot_sim.urdf into Unity
- [ ] **6.10** Verify robot syncs with Gazebo

**Checkpoint (Optional)**: Unity visualization connected (if using) ✅

---

## Reflection & Assessment

### Learning Outcomes
- [ ] **R.1** Explain why digital twins are essential for robotics
- [ ] **R.2** Understand real-time simulation requirements (1000 Hz)
- [ ] **R.3** Build functional Gazebo world from scratch
- [ ] **R.4** Configure LiDAR, camera, and IMU sensors
- [ ] **R.5** Implement control loop with feedback
- [ ] **R.6** Diagnose and fix common sensor/simulation issues

### Skills Acquired
- [ ] **S.1** URDF and SDF file editing (XML fluency)
- [ ] **S.2** ROS 2 publish/subscribe pattern (Python)
- [ ] **S.3** Physics simulation configuration and tuning
- [ ] **S.4** Sensor simulation and noise modeling
- [ ] **S.5** Performance profiling and optimization
- [ ] **S.6** Gazebo GUI navigation and debugging
- [ ] **S.7** Troubleshooting complex systems
- [ ] **S.8** Reading and understanding technical documentation

### Reflection Questions
**Write 2-3 paragraph answers to each:**

1. **What was the most challenging part of Module 2?** (Gazebo setup? Sensors? Control?)
2. **How do sensors in simulation differ from real hardware?** (Based on sim-to-real gap)
3. **Design your own robot task:** What would your arm grasp? Where in the world?
4. **Performance vs Accuracy:** If GPU was slow, what would you trade-off?
5. **Unity decision:** Would you use Unity for your project? Why/why not?
6. **Next steps:** Which Module 3 topics interest you most?

**Reflection Checklist**:
- [ ] **R.7** Written answer to Q1: Challenge faced
- [ ] **R.8** Written answer to Q2: Sim-to-real gap
- [ ] **R.9** Written answer to Q3: Robot task design
- [ ] **R.10** Written answer to Q4: Performance decision
- [ ] **R.11** Written answer to Q5: Unity choice
- [ ] **R.12** Written answer to Q6: Module 3 goals

---

## Module 2 Completion

### Minimum Requirements (to proceed to Module 3)
- [ ] **M.1** All 5 Gazebo chapters (0-4) read and understood
- [ ] **M.2** All 3 sensors (LiDAR, camera, IMU) working in simulation
- [ ] **M.3** Able to modify URDF and SDF files without errors
- [ ] **M.4** Control loop implemented and tested
- [ ] **M.5** Gazebo physics greater than 50 Hz on your hardware
- [ ] **M.6** No crashes during extended simulation (greater than 10 mins)

### Self-Assessment
**Rate yourself 1-5** (1=beginner, 5=expert):

| Topic | Rating | Notes |
|-------|--------|-------|
| Gazebo setup & usage | ___ | ___________ |
| URDF file editing | ___ | ___________ |
| SDF world configuration | ___ | ___________ |
| Sensor simulation | ___ | ___________ |
| ROS 2 control loops | ___ | ___________ |
| Performance optimization | ___ | ___________ |
| Troubleshooting | ___ | ___________ |
| **Average** | **___** | (target: greater than 3) |

### Final Checklist
- [ ] **F.1** All 150+ checklist items completed or intentionally skipped
- [ ] **F.2** All 6+ reflection questions answered in writing
- [ ] **F.3** Self-assessment average rating greater than 3.0
- [ ] **F.4** No outstanding errors or crashes in simulation
- [ ] **F.5** Successfully integrated all 3 sensors + control loop
- [ ] **F.6** Documented learnings in personal notebook/wiki
- [ ] **F.7** Ready to proceed to Module 3: Robot Brain

---

## Resources

- **Gazebo Documentation**: http://gazebosim.org/docs
- **ROS 2 Humble Tutorials**: https://docs.ros.org/en/humble/
- **URDF Tutorials**: http://wiki.ros.org/urdf/Tutorials
- **Troubleshooting Guides**: See sensor-troubleshooting.md and sensor-tuning-guide.md in resources/

---

**Total Progress Items**: 150+ checkboxes
**Estimated Time**: 20-30 hours
**Difficulty**: Intermediate (assumes Module 1 completion)

✨ **Good luck! You've got this!** ✨
- [ ] Launched empty Gazebo world
- [ ] Understand SDF file format
- [ ] Understand URDF vs SDF differences

### World Building
- [ ] Created simple world with ground plane
- [ ] Added obstacle boxes to world
- [ ] Configured physics parameters (friction, gravity)
- [ ] Spawned robot model in world
- [ ] Monitored simulation performance

### Visualization
- [ ] Launched Gazebo GUI
- [ ] Navigated 3D visualization
- [ ] Verified physics running (1000 Hz target)
- [ ] Checked GPU mode detection

**Checkpoint 3**: Gazebo working, worlds created ✅

---

## Chapter 4: Simulating Sensors

### Understanding Plugins
- [ ] Know how Gazebo plugins work
- [ ] Understand ray-tracing for LiDAR
- [ ] Understand OpenGL for depth cameras
- [ ] Know realistic sensor noise modeling

### Adding Sensors
- [ ] Added LiDAR sensor to robot URDF
- [ ] Added depth camera to robot URDF
- [ ] Added IMU sensor to robot URDF
- [ ] Configured Gazebo plugins
- [ ] Verified sensors output ROS 2 messages

### Reading Sensor Data
- [ ] Subscribed to LiDAR topic in Python
- [ ] Subscribed to camera topic in Python
- [ ] Subscribed to IMU topic in Python
- [ ] Processed sensor data with callbacks
- [ ] Printed sensor statistics

### Visualization
- [ ] Launched RViz
- [ ] Added PointCloud2 display for LiDAR
- [ ] Added Image display for camera
- [ ] Displayed IMU orientation
- [ ] Understood coordinate frames

**Checkpoint 4**: Sensors working, data pipeline complete ✅

---

## Chapter 5: Unity Visualization (Optional)

### Setup (Optional)
- [ ] Installed Unity 2022 LTS
- [ ] Installed ROS 2 UDP Communicator
- [ ] Configured ROS-TCP-Connector

### Visualization (Optional)
- [ ] Connected Gazebo to Unity
- [ ] Displayed robot in Unity viewport
- [ ] Visualized LiDAR point clouds
- [ ] Displayed camera feed
- [ ] Understood real-time synchronization

**Note**: This chapter is optional. Focus on Chapters 1-4 first.

**Checkpoint 5** (Optional): Unity visualization working ✅

---

## Final Validation

### Overall Understanding
- [ ] Understand Digital Twin concept
- [ ] Know why simulation matters
- [ ] Confident with Gazebo Fortress
- [ ] Can work with sensors in simulation
- [ ] Feel ready for Module 3

### Hardware Confidence
- [ ] Know GPU requirements
- [ ] Can verify your hardware
- [ ] Know fallback options if no GPU

### Next Steps
- [ ] Reviewed Module 3 preview
- [ ] Know what's coming (robot control)
- [ ] Ready to proceed to Robot Brain

---

## Reflection Prompts

Take a moment to reflect on your learning:

**What surprised you most about simulation?**
```
[Your answer here]
```

**What was the hardest part of this module?**
```
[Your answer here]
```

**What questions do you have for Module 3?**
```
[Your answer here]
```

---

## Support & Resources

**Troubleshooting**:
- Having GPU issues? See Chapter 1 troubleshooting
- Gazebo not launching? See Chapter 3 troubleshooting
- Sensors not working? See Chapter 4 troubleshooting

**Next Module**:
- Module 3: Robot Brain - Advanced control algorithms
- Module 4: The Mind - Vision language models
- Module 5: Capstone - Full sim-to-real deployment

---

**Content being added in Phase 6 implementation...**
