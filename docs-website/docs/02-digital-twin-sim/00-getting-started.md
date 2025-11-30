# Getting Started with Digital Twin Simulation

Welcome to Module 2! Before diving into simulating robots, let's make sure your system is ready. This chapter will help you verify all prerequisites and understand the hardware requirements for simulation.

## Prerequisites Checklist

### Knowledge Prerequisites

Before starting this module, you should have completed:

- ✅ **Module 1: The Nervous System** - You understand ROS 2 fundamentals, nodes, topics, and services
- ✅ **Python 3.8+** - You can write basic Python scripts and understand classes/functions
- ✅ **Linux CLI basics** - You're comfortable with `bash`, `cd`, `ls`, `mkdir`, and `apt` commands
- ✅ **URDF basics** - You've written or read robot description files from Module 1

If any of these are unfamiliar, review Module 1 before proceeding.

### Software Requirements

Your system must have these tools installed:

| Software | Version | Purpose | Check Command |
|----------|---------|---------|---|
| **ROS 2** | Humble | Robot Operating System | `ros2 --version` |
| **Gazebo** | Fortress (Ignition) | Physics simulator | `gazebo --version` |
| **Python** | 3.10+ | Programming language | `python3 --version` |
| **rclpy** | Latest | ROS 2 Python library | `python3 -c "import rclpy; print(rclpy.__version__)"` |
| **Colcon** | Latest | ROS 2 build tool | `colcon --version` |
| **Git** | Latest | Version control | `git --version` |

**Ubuntu 22.04 LTS** is the recommended operating system for this module.

### Hardware Requirements

#### GPU Requirement (MANDATORY)

:::danger NVIDIA RTX 4070 Ti Requirement
**These tutorials require an NVIDIA RTX GPU for real-time simulation.**

**Primary (recommended)**: NVIDIA RTX 4070 Ti (12GB VRAM)
**Minimum**: NVIDIA RTX 4060 (8GB VRAM)
**Supported**: Any NVIDIA CUDA-capable GPU (RTX 3060, RTX 4090, A100, Jetson Orin)
**NOT Supported**: Intel iGPU, Apple Silicon (limited CUDA support)

**Why GPU is essential**:
- Real-time physics simulation requires 1000 Hz update rate
- Sensor simulation uses GPU ray-tracing (LiDAR, depth cameras)
- OpenGL rendering for 3D visualization
- Without GPU, fallback CPU simulation runs at 10-50 Hz (too slow for robotics)

Check if you have a GPU with the command below.
:::

#### System Requirements

| Resource | Minimum | Recommended |
|----------|---------|-------------|
| **GPU VRAM** | 8 GB | 12 GB |
| **System RAM** | 16 GB | 32 GB |
| **Disk Space** | 20 GB | 50 GB |
| **CPU Cores** | 4 cores | 8+ cores |

## Environment Verification Commands

Run these commands to verify your system is ready. **All should succeed before proceeding.**

### 1. Check ROS 2 Installation

```bash
ros2 --version
```

**Expected output**:
```
ROS 2 Humble Hawksbill (0.20.x)
```

If this fails, install ROS 2 Humble from [official docs](https://docs.ros.org/en/humble/Installation.html).

### 2. Check Gazebo Installation

```bash
gazebo --version
```

**Expected output**:
```
Gazebo 7.x.x or higher
```

If this fails, install with:
```bash
sudo apt install gazebo
```

### 3. Check Python Version

```bash
python3 --version
```

**Expected output**:
```
Python 3.10 or higher
```

### 4. Check rclpy Installation

```bash
python3 -c "import rclpy; print(f'rclpy version: {rclpy.__version__}')"
```

**Expected output**:
```
rclpy version: 0.20.x (or higher)
```

If this fails, install with:
```bash
sudo apt install python3-rclpy
```

### 5. Check GPU Availability (CRITICAL)

```bash
nvidia-smi
```

**Expected output** (if you have an NVIDIA GPU):
```
+-----------------------+
| NVIDIA-SMI x.xx       |
+-----------------------+
| GPU  Name        Persistence-M | Bus-Id  Disp.A | Volatile Uncorr. ECC |
|   0  NVIDIA RTX 4070   On        |   00:00.0 Off |                    0 |
|  0%   53C    P0   33W / 320W    |
+-----------------------+
```

**If nvidia-smi shows your GPU**: ✅ You have GPU support
**If nvidia-smi fails or shows no GPU**: ⚠️ You'll need to use CPU mode (slower)

### 6. Check CUDA Installation

```bash
nvcc --version
```

**Expected output**:
```
nvcc: NVIDIA (R) Cuda compiler driver
Cuda compilation tools, release 11.8, ...
```

If CUDA is not installed:
```bash
sudo apt install nvidia-cuda-toolkit
```

### 7. Check Disk Space

```bash
df -h | grep "/$"
```

**Expected output**: At least 20 GB free on `/` partition

Example output (needs 20+ GB):
```
Filesystem      Size  Used Avail Use% Mounted on
/dev/sda1       200G  150G   50G  75% /
```

### 8. Check ROS 2 Humble + Gazebo Bridge

```bash
ros2 pkg list | grep -i "ros_gz\|gazebo"
```

**Expected output**: Should list several `ros_gz_*` packages

If not found, install the bridge:
```bash
sudo apt install ros-humble-ros-gz
```

### 9. Test ROS 2 + Gazebo Integration

Open a terminal and run:

```bash
ros2 launch gazebo_ros gazebo.launch.py
```

**Expected**: Gazebo window opens with empty world

Press `Ctrl+C` to close.

### 10. Verify Colcon Build Tool

```bash
colcon --version
```

**Expected output**:
```
colcon 0.12.x or higher
```

---

## Learning Outcomes

By the end of this chapter, you will be able to:

- [ ] ✅ Explain what a Digital Twin is and why simulation matters
- [ ] ✅ List hardware requirements for real-time robot simulation
- [ ] ✅ Verify all software prerequisites on your system
- [ ] ✅ Understand GPU vs CPU trade-offs for simulation
- [ ] ✅ Locate relevant documentation when setup issues arise
- [ ] ✅ Explain why NVIDIA RTX GPUs are required for these tutorials
- [ ] ✅ Know fallback options if GPU is unavailable
- [ ] ✅ Set up your environment for Module 2 exercises

---

## Quick Summary: Are You Ready?

✅ **If you answered YES to all:**
1. ROS 2 Humble installed and working
2. Gazebo Fortress installed and working
3. Python 3.10+ available
4. NVIDIA RTX GPU detected (or acceptable CPU fallback)
5. 20+ GB free disk space
6. Completed Module 1

**👉 Proceed to Chapter 2: Introduction to Digital Twin**

⚠️ **If you answered NO to any:**
1. Install missing software using commands above
2. Troubleshoot with the guide below
3. Come back when all checks pass

---

## Troubleshooting Common Issues

### Issue: "nvidia-smi not found"

**Cause**: NVIDIA drivers not installed

**Solution**:
```bash
# Check GPU hardware
lspci | grep -i nvidia

# Install drivers
sudo ubuntu-drivers autoinstall
```

### Issue: "ROS 2 commands not found"

**Cause**: ROS 2 not sourced in terminal

**Solution**:
```bash
source /opt/ros/humble/setup.bash
# Add to ~/.bashrc to make permanent:
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Issue: "Gazebo fails to launch"

**Cause**: GPU memory issues or missing CUDA

**Solution**:
```bash
# Run in verbose mode to see errors
gazebo --verbose

# Check GPU memory
nvidia-smi

# Run CPU-only mode (slower)
gazebo --render-engine ogre2 --verbose
```

### Issue: "Module 'rclpy' not found"

**Cause**: Python environment mismatch

**Solution**:
```bash
# Use ROS 2 Python environment
source /opt/ros/humble/setup.bash

# Install via apt
sudo apt install python3-rclpy
```

### Issue: "Gazebo too slow (low Hz)"

**Cause**: Running on CPU instead of GPU

**Solution**:
1. Verify GPU is being used: `nvidia-smi` (should show Gazebo process)
2. If not: Check CUDA installation
3. If acceptable: Accept 50 Hz CPU mode and adjust expectations

---

## Next Steps

✅ **Prerequisites verified?** Great! Move on to **Chapter 2: Introduction to Digital Twin** to learn why simulation matters and understand the sim-to-real gap.

❓ **Questions?** Check the troubleshooting section or review Module 1 prerequisites.

🚀 **Ready to simulate?** Let's go!
