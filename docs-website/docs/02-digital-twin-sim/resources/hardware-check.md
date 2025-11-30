# Hardware Verification & Troubleshooting Guide

This guide helps you verify your system is ready for Module 2 and troubleshoot common hardware issues.

## Quick Hardware Check Script

Copy and paste this script into a terminal to verify all prerequisites:

```bash
#!/bin/bash
# Module 2 Hardware Verification Script
# Run: bash hardware-check.sh

echo "========================================="
echo "Module 2 Hardware Verification"
echo "========================================="
echo ""

# Check ROS 2
echo "[1/8] Checking ROS 2..."
if command -v ros2 &> /dev/null; then
    ros2 --version
    echo "✅ ROS 2 found"
else
    echo "❌ ROS 2 not found - Install from https://docs.ros.org/en/humble/"
fi
echo ""

# Check Gazebo
echo "[2/8] Checking Gazebo..."
if command -v gazebo &> /dev/null; then
    gazebo --version
    echo "✅ Gazebo found"
else
    echo "❌ Gazebo not found - Run: sudo apt install gazebo"
fi
echo ""

# Check Python
echo "[3/8] Checking Python..."
if command -v python3 &> /dev/null; then
    python3 --version
    PYTHON_VERSION=$(python3 -c 'import sys; print(".".join(map(str, sys.version_info[:2])))')
    if [[ $PYTHON_VERSION > "3.10" ]]; then
        echo "✅ Python 3.10+ found"
    else
        echo "⚠️  Python version is $PYTHON_VERSION (3.10+ recommended)"
    fi
else
    echo "❌ Python not found"
fi
echo ""

# Check rclpy
echo "[4/8] Checking rclpy..."
if python3 -c "import rclpy" 2>/dev/null; then
    python3 -c "import rclpy; print(f'✅ rclpy {rclpy.__version__} found')"
else
    echo "❌ rclpy not found - Run: sudo apt install python3-rclpy"
fi
echo ""

# Check NVIDIA GPU
echo "[5/8] Checking NVIDIA GPU..."
if command -v nvidia-smi &> /dev/null; then
    echo "✅ NVIDIA drivers found"
    nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader

    # Check CUDA capability
    CUDA_CAPABILITY=$(nvidia-smi --query-gpu=compute_cap --format=csv,noheader | head -1)
    echo "CUDA Capability: $CUDA_CAPABILITY"

    if [[ $CUDA_CAPABILITY > "3.0" ]]; then
        echo "✅ GPU supports CUDA (compute capability $CUDA_CAPABILITY)"
    else
        echo "⚠️  GPU has older compute capability - May be slow"
    fi
else
    echo "⚠️  NVIDIA drivers not found"
    echo "   Check GPU with: lspci | grep -i nvidia"
    echo "   Install drivers with: sudo ubuntu-drivers autoinstall"
fi
echo ""

# Check CUDA toolkit
echo "[6/8] Checking CUDA Toolkit..."
if command -v nvcc &> /dev/null; then
    nvcc --version | head -3
    echo "✅ CUDA toolkit found"
else
    echo "⚠️  CUDA toolkit not found (GPU simulation may be slow)"
    echo "   Install with: sudo apt install nvidia-cuda-toolkit"
fi
echo ""

# Check Disk Space
echo "[7/8] Checking Disk Space..."
DISK_AVAILABLE=$(df / | awk 'NR==2 {print int($4/1024/1024)}')
echo "Free disk space: ${DISK_AVAILABLE} GB"
if [[ $DISK_AVAILABLE -gt 20 ]]; then
    echo "✅ Sufficient disk space (need 20+ GB)"
else
    echo "❌ Insufficient disk space (need 20+ GB, have ${DISK_AVAILABLE} GB)"
fi
echo ""

# Check System RAM
echo "[8/8] Checking System RAM..."
RAM_TOTAL=$(free -h | awk 'NR==2 {print $2}')
RAM_AVAILABLE=$(free -h | awk 'NR==2 {print $7}')
echo "Total RAM: $RAM_TOTAL, Available: $RAM_AVAILABLE"
if [[ $(echo "$RAM_AVAILABLE" | grep -oE '[0-9]+') -gt 8 ]]; then
    echo "✅ Sufficient RAM available"
else
    echo "⚠️  Low RAM available (16+ GB recommended)"
fi
echo ""

echo "========================================="
echo "Hardware Check Complete!"
echo "========================================="
```

**To run this script:**
```bash
# Save as hardware-check.sh
bash hardware-check.sh
```

---

## NVIDIA GPU Verification

### Check if GPU is Detected

```bash
lspci | grep -i nvidia
```

**Good output** (GPU found):
```
01:00.0 VGA compatible controller: NVIDIA Corporation GA102 [GeForce RTX 3070 Ti] (rev a1)
```

**Bad output** (no GPU):
```
(no output)
```

### Verify NVIDIA Drivers

```bash
nvidia-smi
```

**Good output** (drivers installed):
```
+-----------------------+
| NVIDIA-SMI 535.104.05 |
+-----------------------+
| GPU  Name        Persistence-M | Bus-Id  Disp.A | Volatile Uncorr. ECC |
|   0  NVIDIA RTX 4070   On        |   00:00.0 Off |                    0 |
| 0%   40C    P0   45W / 320W    |
+-----------------------+
```

**Bad output** (drivers not installed):
```
Command not found
```

### Check GPU Memory

```bash
nvidia-smi --query-gpu=memory.total,memory.used,memory.free --format=csv
```

**Expected output** (for RTX 4070 Ti):
```
memory.total [MB], memory.used [MB], memory.free [MB]
12288, 100, 12188
```

**What it means**:
- **memory.total**: Total GPU VRAM (12 GB for RTX 4070 Ti)
- **memory.used**: Currently allocated (should be low before simulation)
- **memory.free**: Available for simulation (should be 8+ GB for real-time simulation)

### Check GPU Temperature

```bash
nvidia-smi --query-gpu=temperature.gpu,power.draw,power.limit --format=csv
```

**Expected output** (at idle):
```
temperature.gpu, power.draw [W], power.limit [W]
40, 5, 320
```

**What it means**:
- **temperature.gpu**: 40-45°C at idle (65-75°C under load is normal)
- **power.draw**: Should be 5-10W idle, 100-200W during simulation
- **power.limit**: Maximum power (320W for RTX 4070 Ti)

### Monitor GPU During Simulation

```bash
# Run in one terminal, simulation in another
nvidia-smi dmon
```

**Output** (updates every second):
```
gpu   pwr gtemp mtemp    sm   mem   enc   dec  mclk  pclk
  0   200   68   65    95    60     0     0  7505  2505
  0   205   70   67    96    62     0     0  7505  2505
```

**What it means**:
- **pwr**: Power draw (200W = heavy simulation load)
- **gtemp**: GPU temperature (65-75°C is normal under load)
- **sm**: Multiprocessor utilization (95-100% = GPU fully utilized)
- **mem**: Memory utilization (60% = good, simulation using 7GB of 12GB)

---

## CUDA Capability Check

### Verify CUDA is Installed

```bash
nvcc --version
```

**Good output**:
```
nvcc: NVIDIA (R) Cuda compiler driver
Cuda compilation tools, release 11.8, V11.8.89
```

**Bad output**:
```
Command not found
```

### Check GPU Compute Capability

```bash
nvidia-smi --query-gpu=compute_cap --format=csv,noheader
```

**Expected output** (for RTX 4070 Ti):
```
8.9
```

**What compute capability means**:
- **3.0-3.5** (very old, slow)
- **5.0-6.2** (good, RTX 2000 series)
- **7.0-7.5** (very good, RTX 2080 Ti)
- **8.0-8.9** (excellent, RTX 3000/4000 series)

---

## System Resources Check

### Check CPU Cores

```bash
nproc
```

**Good output** (8 cores):
```
8
```

**What it means**:
- ROS 2 and Python can parallelize across cores
- 4+ cores recommended, 8+ cores preferred

### Check CPU Model

```bash
lscpu | grep "Model name"
```

**Good output** (modern CPU):
```
Model name: Intel(R) Core(TM) i7-13700K
```

### Monitor CPU During Simulation

```bash
# Run in one terminal, simulation in another
htop
```

**What to look for**:
- CPU usage should not exceed 80% (allows OS to respond)
- If simulation uses 100% CPU: Switch to GPU mode
- If GPU available but not used: Check Gazebo plugin configuration

### Check Available System RAM

```bash
free -h
```

**Good output** (16GB system):
```
              total        used        free      shared  buff/cache   available
Mem:           15Gi       3.2Gi       8.2Gi       256Mi       4.0Gi      11.5Gi
```

**What it means**:
- **total**: Total system RAM (16GB recommended minimum)
- **available**: Free RAM (should be 8+ GB during simulation)
- **used**: Currently allocated RAM

---

## Disk Space Check

### Check Disk Space

```bash
df -h
```

**Good output**:
```
Filesystem      Size  Used Avail Use% Mounted on
/dev/sda1       500G  250G  250G  50% /
```

**What it means**:
- You need 20+ GB free for Gazebo + dependencies + simulation worlds
- 50+ GB free is recommended for multiple large simulations

### Check Installation Size

```bash
du -sh /opt/ros/humble
```

**Expected output**:
```
8.2G    /opt/ros/humble
```

Gazebo and ROS 2 together use ~15-20 GB.

---

## Temperature Monitoring

### Safe Operating Temperatures

| Component | Idle | Load | Critical |
|-----------|------|------|----------|
| **GPU** | 30-40°C | 65-75°C | > 85°C |
| **CPU** | 30-45°C | 60-80°C | > 95°C |
| **RAM** | 30-50°C | 40-60°C | > 80°C |

### Monitor All Temperatures

```bash
watch -n 1 'nvidia-smi; echo ""; sensors'
```

**If temperatures are too high:**
1. Check ventilation (fans spinning?)
2. Reduce simulation complexity
3. Switch to CPU mode (slower but cooler)
4. Check for thermal throttling: `nvidia-smi -i 0 -q | grep -i throttle`

---

## Fallback Options If GPU is Unavailable

### Option 1: CPU-Only Simulation

**Performance**: 50 Hz physics (slow but functional)

```bash
# Launch Gazebo in CPU mode
gazebo --render-engine ogre --verbose simple_world.sdf

# Or use CPU physics engine
gazebo --physics-engine dart simple_world.sdf
```

**Limitations:**
- 10-50 Hz instead of 1000 Hz
- Single-threaded (won't use multiple CPU cores effectively)
- Sensor simulation slower (depth camera rendering slow)

### Option 2: Cloud-Based GPU (AWS, GCP, Azure)

Rent GPU instances for simulation:

```bash
# AWS EC2 with GPU
aws ec2 run-instances --image-id ami-xxxxx --instance-type g4dn.xlarge

# GCP Compute Engine with GPU
gcloud compute instances create gazebo-sim --accelerator=type=nvidia-tesla-t4 --machine-type=n1-standard-8
```

**Cost**: $0.50-2.00/hour for GPU instances

### Option 3: Docker with GPU Support

Run simulation in Docker container:

```bash
# Run ROS 2 + Gazebo in Docker with GPU
docker run --gpus all -it ros:humble gazebo
```

---

## Troubleshooting Common Hardware Issues

### Issue: "nvidia-smi: command not found"

**Cause**: NVIDIA drivers not installed

**Solution**:
```bash
# Check GPU hardware
lspci | grep -i nvidia

# Install NVIDIA drivers
sudo ubuntu-drivers autoinstall

# Reboot
sudo reboot

# Verify installation
nvidia-smi
```

---

### Issue: "CUDA Capability Too Old"

**Cause**: GPU compute capability less than 3.0 (very old GPU)

**Symptoms**:
```bash
nvidia-smi --query-gpu=compute_cap --format=csv,noheader
# Output: 2.0 or lower
```

**Solution**:
- Use CPU simulation: `gazebo --render-engine ogre`
- Upgrade GPU if possible
- Use cloud GPU services

---

### Issue: "Out of GPU Memory"

**Cause**: Not enough VRAM for simulation

**Symptoms**:
```bash
nvidia-smi
# Shows: memory.free: 1024 MB (should be 8+ GB)
```

**Solution**:
1. Close other GPU applications (video players, browsers with GPU rendering)
2. Reduce simulation complexity (fewer sensors, smaller worlds)
3. Use CPU mode (slower but uses system RAM)

```bash
# Monitor memory during simulation
watch -n 1 nvidia-smi
```

---

### Issue: "GPU Temperature Too High (> 85°C)"

**Cause**: Thermal throttling, poor ventilation

**Symptoms**:
```bash
nvidia-smi | grep Temp
# Output: 87C (should be less than 75C)
```

**Solution**:
1. Check GPU cooling:
   ```bash
   # Monitor fan speed
   nvidia-smi -i 0 -q | grep -A 3 "Fan Speed"
   ```
2. Improve ventilation (clean dust filters, improve airflow)
3. Reduce simulation complexity
4. Enable max fan mode:
   ```bash
   sudo nvidia-smi -pm 1
   ```

---

### Issue: "GPU Not Being Used (always CPU mode)"

**Cause**: Gazebo not configured for GPU, or GPU disabled

**Symptoms**:
```bash
nvidia-smi
# Output: No gazebo process listed
```

**Solution**:
1. Check Gazebo render engine:
   ```bash
   gazebo --verbose simple_world.sdf 2>&1 | grep -i "render\|gpu\|ogre"
   ```
2. Force GPU rendering:
   ```bash
   gazebo --render-engine ogre2 simple_world.sdf
   ```
3. Check for GPU process:
   ```bash
   watch -n 1 'nvidia-smi | grep gazebo'
   ```

---

## Performance Tuning

### Optimize GPU Usage

```bash
# Enable persistence mode (faster startup)
sudo nvidia-smi -pm 1

# Set GPU to performance mode
sudo nvidia-smi -pm 1
sudo nvidia-smi -pmc 0

# Monitor clocks
nvidia-smi -q -l
```

### Optimize Gazebo Physics

In `simple_world.sdf`:
```xml
<physics default="0" type="ode">
  <max_step_size>0.001</max_step_size>        <!-- Smaller = more accurate, slower -->
  <real_time_update_rate>1000</real_time_update_rate>  <!-- 1000 Hz target -->
  <real_time_factor>1.0</real_time_factor>    <!-- 1.0 = real-time, > 1.0 = faster -->
</physics>
```

**Tuning tips:**
- Increase `max_step_size` if too slow (less accurate but faster)
- Decrease `real_time_factor` if stability issues

---

## Next Steps

✅ **All hardware checks pass?** Great! Proceed to Module 2 content.

⚠️ **Issues?** Check the troubleshooting section above or ask for help.

📊 **Want baseline performance?** Run and monitor:
```bash
gazebo simple_world.sdf  # Terminal 1
python3 sensor_reader.py # Terminal 2
watch -n 1 nvidia-smi    # Terminal 3 (monitor GPU)
```

Expected: Gazebo runs at 1000 Hz, GPU utilization 60-80%, temperature 65-75°C.
