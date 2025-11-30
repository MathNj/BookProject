# Sensor Performance Tuning Guide

This guide explains how to tune sensor parameters for optimal performance, balancing accuracy vs speed.

## LiDAR Tuning

### Parameter: Number of Rays (samples)

**Location in `robot_sim.urdf`**:
```xml
<horizontal>
  <samples>1024</samples>  <!-- Change this value -->
</horizontal>
```

### Trade-offs

| Samples | Physics Rate | Accuracy | Use Case | Memory |
|---------|-------------|----------|----------|--------|
| **128** | 2000+ Hz | Very low | Debugging only | less than 1 MB |
| **256** | 1500 Hz | Low | Obstacle detection | 2 MB |
| **512** | 800 Hz | Medium | Safe navigation | 5 MB |
| **1024** | 200 Hz | High | SLAM/mapping | 10 MB |
| **2048** | 50 Hz | Very high | Research only | 20 MB |

### How to Adjust

1. **Want faster simulation?** Reduce to 256 or 512 rays
   ```xml
   <samples>256</samples>  <!-- 6x faster physics -->
   ```

2. **Want more accurate?** Increase to 2048 rays
   ```xml
   <samples>2048</samples>  <!-- Much more detail but slower -->
   ```

3. **Benchmark your setup**:
   ```bash
   # Run sensor_reader.py and watch terminal
   # Look for "Physics: XXX Hz" in RViz bottom-right corner
   python3 sensor_reader.py
   ```

### Understanding Impact

- **More rays = better resolution** (1° vs 3° per ray)
- **More rays = slower physics** (GPU ray-casting is the bottleneck)
- **On RTX GPU**: 1024 rays achieves real-time (200+ Hz)
- **On CPU**: 256 rays is practical maximum (50 Hz)

### Recommended Starting Values

| Your Hardware | Recommended | Why |
|---------------|-------------|-----|
| **RTX 4070 Ti** | 1024 rays | Real-time capable |
| **RTX 3080** | 512 rays | Safe margin (100+ Hz) |
| **GTX 1080** | 256 rays | Conservative (50 Hz) |
| **CPU only** | 128 rays | Very slow (loss of detail) |

---

## Depth Camera Tuning

### Parameter 1: Image Resolution

**Location in `robot_sim.urdf`**:
```xml
<image>
  <width>640</width>   <!-- Change for resolution -->
  <height>480</height>
</image>
```

### Resolution Impact

| Resolution | FPS on GPU | Bandwidth | Use Case |
|-----------|-----------|-----------|----------|
| **160x120** | 60+ FPS | 5 MB/s | Low-res testing |
| **320x240** | 30 FPS | 15 MB/s | Real-time control |
| **640x480** | 15 FPS | 30 MB/s | Detailed vision |
| **1280x960** | 5 FPS | 120 MB/s | Research only |

**Rule of thumb**: Halving resolution = 4x faster

### Parameter 2: Frame Rate (Exposure)

Gazebo internally renders at 30 FPS by default. To change:

```xml
<clip>
  <near>0.01</near>   <!-- Minimum distance (1cm) -->
  <far>10.0</far>     <!-- Maximum distance (10m) -->
</clip>
```

- **Smaller `far` value** = faster rendering (less to process)
- **Larger `far` value** = can see distant objects but slower

### Parameter 3: Noise

**Location in `robot_sim.urdf`**:
```xml
<noise>
  <type>gaussian</type>
  <mean>0</mean>
  <stddev>0.007</stddev>  <!-- Noise level (0.7%) -->
</noise>
```

### Noise Impact

| Noise | Simulation Effect | Realism |
|-------|------------------|---------|
| **0.0** (no noise) | Perfect measurements | Unrealistic |
| **0.005** (0.5%) | Slight uncertainty | Ideal for learning |
| **0.01** (1.0%) | Visible noise | Realistic |
| **0.05** (5.0%) | Very noisy | Overestimated |

**Recommendation**: Keep at 0.007 (0.7%) for balanced realism + practicality.

### Recommended Camera Settings

```xml
<!-- For real-time navigation (30 FPS) -->
<image>
  <width>320</width>
  <height>240</height>
</image>

<!-- For high-accuracy vision (10 FPS) -->
<image>
  <width>640</width>
  <height>480</height>
</image>

<!-- For quick testing (60+ FPS) -->
<image>
  <width>160</width>
  <height>120</height>
</image>
```

---

## IMU Tuning

### Parameter: Noise Levels

**Location in `robot_sim.urdf`**:
```xml
<!-- Accelerometer noise (m/s²) -->
<linear_acceleration>
  <x>
    <noise type="gaussian">
      <stddev>0.002</stddev>  <!-- ±0.002 m/s² -->
    </noise>
  </x>
  <!-- ... y, z similar -->
</linear_acceleration>

<!-- Gyroscope noise (rad/s) -->
<angular_velocity>
  <x>
    <noise type="gaussian">
      <stddev>0.0001</stddev>  <!-- ±0.0001 rad/s -->
    </noise>
  </x>
  <!-- ... y, z similar -->
</angular_velocity>
```

### Noise Comparison

| Sensor | Default Noise | Reality | Comments |
|--------|--------------|---------|----------|
| **Accelerometer** | ±0.002 m/s² | ±0.05 m/s² | Our sim is idealized |
| **Gyroscope** | ±0.0001 rad/s | ±0.001 rad/s | Our sim is very clean |

### When to Increase Noise

**Increase noise to test robustness**:
```xml
<!-- For robustness testing: 5x normal noise -->
<stddev>0.01</stddev>   <!-- ±0.01 m/s² (5x increase) -->
```

**Why?** Algorithms trained on clean data often fail on real hardware.

---

## Balanced Configuration for Learning

For Module 2 exercises, we recommend:

```xml
<!-- In robot_sim.urdf -->

<!-- LiDAR: good balance of speed and detail -->
<horizontal>
  <samples>512</samples>  <!-- 512 rays, 50+ Hz physics -->
</horizontal>

<!-- Camera: real-time capable -->
<image>
  <width>320</width>
  <height>240</height>
</image>
<noise>
  <stddev>0.007</stddev>  <!-- Realistic noise level -->
</noise>

<!-- IMU: realistic noise for robustness -->
<linear_acceleration>
  <stddev>0.002</stddev>
</linear_acceleration>
<angular_velocity>
  <stddev>0.0001</stddev>
</angular_velocity>
```

**Performance**: 50+ Hz physics, 15 FPS camera, 100 Hz IMU on RTX GPU.

---

## Measuring Performance

### 1. Monitor Simulation Speed (in RViz)

Look at bottom-right corner of RViz window:
- **1000+ Hz**: Real-time, all good
- **100-500 Hz**: Good, acceptable
- **50-100 Hz**: Slow but functional
- **less than 20 Hz**: Too slow, increase sensor samples

### 2. Monitor GPU Usage

Open terminal:
```bash
watch -n 1 nvidia-smi
```

Look for:
- **GPU Memory**: Should be less than 8 GB total
- **GPU Utilization**: Should be greater than 50% (else CPU bottleneck)
- **Temperature**: Should be less than 80°C

### 3. Monitor Topics Publishing Rate

```bash
# Check LiDAR publishing rate (should be 10+ Hz)
ros2 topic hz /robot/lidar/scan

# Check camera publishing rate (should be 15+ FPS)
ros2 topic hz /robot/depth_camera/image_raw

# Check IMU publishing rate (should be 100+ Hz)
ros2 topic hz /robot/imu/data
```

### 4. Measure End-to-End Latency

Sensor latency = time from robot action to sensor data received.

```python
import time
from rclpy.time import Time

# In your sensor callback
latency_ms = (time.time() - msg.header.stamp.sec) * 1000
if latency_ms greater than 100:  # more than 100ms
    print(f"WARNING: High sensor latency: {latency_ms:.1f}ms")
```

---

## Optimization Checklist

### For Maximum Speed (CPU/old GPU)

```xml
<!-- Minimal accuracy, maximum speed -->
<samples>128</samples>        <!-- LiDAR: very sparse -->
<image>
  <width>160</width>
  <height>120</height>        <!-- Camera: very small -->
</image>
<far>5.0</far>                <!-- Camera: limited range -->
```

**Result**: 100+ Hz physics, 30+ FPS camera, but low accuracy.

### For Balanced Learning (Standard GPU)

```xml
<!-- Good speed, good accuracy -->
<samples>512</samples>        <!-- LiDAR: moderate -->
<image>
  <width>320</width>
  <height>240</height>        <!-- Camera: VGA -->
</image>
<far>10.0</far>               <!-- Camera: normal range -->
```

**Result**: 50+ Hz physics, 15 FPS camera, good for learning.

### For Accuracy (High-end GPU only)

```xml
<!-- Maximum accuracy, slower speed -->
<samples>1024</samples>       <!-- LiDAR: very dense -->
<image>
  <width>640</width>
  <height>480</height>        <!-- Camera: high res -->
</image>
<far>40.0</far>               <!-- Camera: extended range -->
<stddev>0.01</stddev>         <!-- More realistic noise -->
```

**Result**: 200 Hz physics, 10 FPS camera, high fidelity.

---

## Common Performance Problems

### Problem 1: Physics Drops to less than 20 Hz

**Diagnosis**:
1. Check GPU usage: `nvidia-smi` (should be greater than 50%)
2. Check sensor settings: Are samples too high?

**Solution**:
```xml
<!-- Reduce LiDAR resolution -->
<samples>256</samples>  <!-- down from 512 -->
```

### Problem 2: Camera Image Laggy (less than 5 FPS)

**Diagnosis**:
```bash
ros2 topic hz /robot/depth_camera/image_raw  # Should be greater than 15
```

**Solution**:
```xml
<!-- Reduce camera resolution -->
<image>
  <width>160</width>   <!-- down from 320 -->
  <height>120</height>
</image>
```

### Problem 3: GPU Out of Memory

**Error**: CUDA out of memory or "GPU allocation failed"

**Solution**:
- Reduce all sensor resolutions
- Reduce LiDAR samples
- Use CPU-only mode as fallback

### Problem 4: IMU Data Not Publishing

**Diagnosis**:
```bash
ros2 topic list | grep imu  # Should show /robot/imu/data
```

**Solution**: Restart Gazebo. IMU needs proper initialization.

---

## Summary Table: Quick Reference

| Goal | LiDAR | Camera | IMU | Physics |
|------|-------|--------|-----|---------|
| **Max Speed** | 128 rays | 160x120 | No noise | 2000+ Hz |
| **Balanced** | 512 rays | 320x240 | Normal noise | 50 Hz |
| **High Accuracy** | 1024 rays | 640x480 | Realistic | 200 Hz |

Pick the row that matches your hardware and goals!

