# Visual SLAM: Mapping with Eyes

## Learning Objectives

By the end of this chapter, you will:
- Understand Visual SLAM fundamentals (feature detection, bundle adjustment, loop closure)
- Compare Visual SLAM (vision-based) vs Lidar SLAM (laser-based) approaches
- Configure and run isaac_ros_visual_slam on Jetson Orin
- Calibrate a camera using OpenCV checkerboard patterns
- Visualize live 3D maps in RViz

## Prerequisites

- Completed Chapter 1: Isaac Sim Bridge (ROS 2 topics flowing)
- Completed Chapter 2: Synthetic Data Generation (understanding Isaac Sim scenes)
- Jetson Orin Nano/NX or RTX 4070 Ti (with Docker container)
- Intel RealSense D435 camera (or USB webcam)
- ROS 2 Humble with Nav2 stack installed

## Concept: Mapping Without Lasers

In the physical world, robots need to know where they are. Traditionally, roboticists use expensive laser scanners (Lidar) to measure distances to walls and objects, building a 2D "occupancy grid" map.

But **eyes are cheaper than lasers**. A $50 RGB-D camera (Intel RealSense) can do the same job by analyzing **pixel patterns** over time.

### How Visual SLAM Works

**VSLAM = Visual Simultaneous Localization and Mapping**

Imagine you're walking through a new building with your eyes closed, then open them and try to remember where you are:

1. **Feature Detection**: You look for distinctive visual markers (corners, edges, textures)
2. **Feature Tracking**: You follow those features across video frames
3. **Pose Estimation**: You triangulate your position and orientation from the feature locations
4. **Loop Closure**: You recognize a place you've seen before and correct your map

This is VSLAM in a nutshell:

```
Frame N        Frame N+1       Frame N+2
+---------+    +---------+    +---------+
| [O] [O] |    | [O]     |    |     [O] |
|  O   O  | -> |  O   O  | -> |  O   O  |
|  [O]    |    |  [O]    |    |  [O]    |
+---------+    +---------+    +---------+

Features     Tracked         New position
detected     across frames   estimated
```

### Why Vision-Based SLAM?

| Aspect | Visual SLAM | Lidar SLAM |
|--------|-------------|-----------|
| **Cost** | $50 (RealSense) | $1,000+ (Velodyne/Livox) |
| **Works Indoors** | ✅ Excellent | ⚠️ Depends on texture |
| **Sensitivity** | ⚠️ Fails in dark/featureless | ✅ Works in dark |
| **Outdoor Sunlight** | ⚠️ Washed out | ✅ Robust |
| **Accuracy** | ±5 cm (good camera) | ±1 cm (high-end) |
| **Latency** | &lt;2 sec (Jetson) | &lt;1 sec (GPU lidar) |
| **Power** | 1–2W (camera) | 8–10W (lidar) |

**Verdict**: Vision-first for indoor mobile robots; Lidar as fallback for outdoor/featureless environments.

## Visual SLAM Fundamentals

### Step 1: Feature Detection

The VSLAM algorithm detects and tracks **visual features**—distinctive points in images that are easy to recognize:

**Common features**:
- **Corners** (ORB, FAST, Harris)
- **Edges** (SIFT, SURF)
- **Textures** (deep learning-based)

Example: A checkerboard has strong corner features; a plain white wall has none.

### Step 2: Feature Matching

Across consecutive frames, the algorithm matches features:

```
Frame 1                Frame 2
Feature A              Feature A
at pixel (100, 50)     at pixel (105, 48)
         ↓
Displacement: (5, -2) pixels
         ↓
Calculate 3D position using camera intrinsics
```

### Step 3: Pose Estimation (Bundle Adjustment)

Using matched features across multiple frames, the algorithm estimates:
- **Robot's position** (x, y, z)
- **Robot's orientation** (yaw, pitch, roll)
- **3D coordinates of features**

This is solved via **bundle adjustment**—a least-squares optimization that refines all estimates simultaneously.

### Step 4: Loop Closure

As the robot explores, it may re-encounter a previously visited location. VSLAM detects this (via place recognition) and corrects accumulated drift:

```
Exploration path:

Start ──→ ──→ ──→ ──→
         │         │
         └─→ ──→ ──┘
            Loop closure!
            (Correct drift)
```

## Comparison: Visual SLAM vs Lidar SLAM

### Visual SLAM (isaac_ros_visual_slam)

**Pros**:
- Cheap (RGB-D camera ~$50)
- Robust indoors (textured walls)
- No mechanical moving parts
- Works in GPS-denied environments

**Cons**:
- Fails in dark or featureless environments
- Requires good camera calibration
- Slower than GPU-accelerated lidar
- Affected by dynamic lighting changes

### Lidar SLAM (SLAM Toolbox, Fast-SLAM)

**Pros**:
- Works in darkness
- High absolute accuracy (±1 cm)
- Robust to lighting changes
- Fast (< 100 ms per scan)

**Cons**:
- Expensive ($1,000–$10,000)
- Large power consumption (8–10W)
- Overkill for small indoor robots
- Requires clear line-of-sight

### When to Use Each

| Environment | Recommendation | Reason |
|-------------|-----------------|--------|
| **Indoor office/warehouse** | Visual SLAM | Textured walls, good lighting, low cost |
| **Dark mine/underground** | Lidar SLAM | No visual features in darkness |
| **Outdoor urban** | Lidar SLAM | Sunlight washes out vision; robust distance |
| **Robot arm (stationary)** | Visual SLAM | High camera resolution; perfect for object detection |
| **Mobile robot (budget)** | Visual SLAM | RTX 4070 Ti or Jetson Orin can run it |
| **Autonomous vehicle** | Lidar SLAM | Speed and safety critical; cost not a factor |

## Setting Up isaac_ros_visual_slam

### Step 1: Hardware Setup

**Option A: USB RGB-D Camera (Intel RealSense)**

1. Connect Intel RealSense D435 via USB 3.0
2. Install RealSense SDK:
   ```bash
   sudo apt-get install librealsense2-dev
   ros2 run realsense_ros2_camera realsense_ros2_camera_node
   ```
3. Verify topics:
   ```bash
   ros2 topic list | grep -E "camera|depth"
   ```

**Option B: Jetson Onboard Camera**

1. Jetson Orin has CSI camera connectors
2. Install camera driver:
   ```bash
   sudo apt-get install nvidia-jetson-multimedia-api
   ```
3. Launch camera node:
   ```bash
   ros2 launch jetson_camera jetson_csi_camera.launch.py
   ```

### Step 2: Camera Calibration

Before VSLAM, the camera must be calibrated. Use OpenCV's checkerboard method:

**File**: `docs/03-robot-brain/code-examples/camera_calibration.py`

```python
#!/usr/bin/env python3
"""
camera_calibration.py - Calibrate camera using OpenCV checkerboard
"""
import cv2
import numpy as np
from pathlib import Path

def calibrate_camera(
    checkerboard_size=(9, 6),
    num_images=20,
    output_file="camera_calibration.json"
):
    """
    Capture checkerboard images and compute camera intrinsics.

    Args:
        checkerboard_size: (columns, rows) of checkerboard corners
        num_images: number of images to capture
        output_file: JSON file to save calibration
    """

    # Termination criteria for OpenCV
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Prepare object points (0,0,0), (1,0,0), (2,0,0), ...
    objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2)
    objp *= 0.025  # Scale to 2.5 cm squares

    objpoints = []  # 3D points in real world space
    imgpoints = []  # 2D points in image plane

    cap = cv2.VideoCapture(0)
    print(f"📷 Capture {num_images} images of checkerboard...")
    print("Press 'c' to capture, 'q' to quit.")

    image_count = 0
    while image_count < num_images:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read frame")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Find checkerboard corners
        ret, corners = cv2.findChessboardCorners(gray, checkerboard_size, None)

        if ret:
            # Refine corner positions
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            # Draw checkerboard on display
            frame_with_corners = cv2.drawChessboardCorners(
                frame.copy(), checkerboard_size, corners2, ret
            )
            cv2.putText(
                frame_with_corners,
                f"Found! Press 'c' to capture ({image_count}/{num_images})",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )
        else:
            frame_with_corners = frame.copy()
            cv2.putText(
                frame_with_corners,
                "Move checkerboard into view...",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
            )

        cv2.imshow("Calibration", cv2.resize(frame_with_corners, (800, 600)))

        key = cv2.waitKey(1) & 0xFF
        if key == ord("c") and ret:
            objpoints.append(objp)
            imgpoints.append(corners2)
            image_count += 1
            print(f"  ✓ Image {image_count}/{num_images} captured")
        elif key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()

    print(f"\n🔧 Calibrating camera with {len(objpoints)} images...")

    # Calibrate
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )

    if ret:
        print(f"✅ Calibration successful!")
        print(f"Camera Matrix:\n{camera_matrix}")
        print(f"Distortion Coefficients:\n{dist_coeffs}")

        # Save to JSON
        calib_data = {
            "camera_matrix": camera_matrix.tolist(),
            "distortion_coefficients": dist_coeffs.flatten().tolist(),
            "image_width": gray.shape[1],
            "image_height": gray.shape[0],
            "num_images_used": len(objpoints),
        }

        import json
        with open(output_file, "w") as f:
            json.dump(calib_data, f, indent=2)

        print(f"📁 Saved to: {output_file}")
        return True
    else:
        print(f"❌ Calibration failed")
        return False

if __name__ == "__main__":
    calibrate_camera(num_images=20, output_file="camera_calibration.json")
```

Run it:
```bash
python3 camera_calibration.py
# Move checkerboard in front of camera, press 'c' to capture each image
```

### Step 3: Run isaac_ros_visual_slam Node

Create a launch file:

**File**: `docs/03-robot-brain/code-examples/vslam_launch.py`

```python
#!/usr/bin/env python3
"""
vslam_launch.py - Launch isaac_ros_visual_slam node
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import json
from pathlib import Path

def generate_launch_description():
    """
    Generate launch description for Visual SLAM node.
    """

    # Load camera calibration
    calib_file = Path("camera_calibration.json")
    if calib_file.exists():
        with open(calib_file) as f:
            calib = json.load(f)
        camera_matrix = calib["camera_matrix"]
        distortion = calib["distortion_coefficients"]
    else:
        # Default Intel RealSense D435 intrinsics
        camera_matrix = [
            [614.0, 0.0, 320.0],
            [0.0, 614.0, 240.0],
            [0.0, 0.0, 1.0]
        ]
        distortion = [0.0, 0.0, 0.0, 0.0, 0.0]

    return LaunchDescription([
        # VSLAM node
        Node(
            package='isaac_ros_visual_slam',
            executable='visual_slam_node',
            name='visual_slam',
            namespace='',
            parameters=[
                {
                    # Subscriptions
                    'input_img_topic': '/camera/color/image_raw',
                    'input_depth_topic': '/camera/depth/image_rect_raw',
                    'input_camera_info_topic': '/camera/color/camera_info',

                    # Outputs
                    'output_map_topic': '/map',
                    'output_odom_topic': '/odom',

                    # VSLAM parameters
                    'keyframe_threshold': 0.3,           # Keyframe distance (meters)
                    'map_update_frequency_hz': 20.0,     # 20 Hz map updates
                    'confidence_threshold': 0.85,        # Minimum confidence for map points

                    # Camera intrinsics
                    'camera_matrix': camera_matrix,
                    'distortion_coefficients': distortion,

                    # Frame IDs
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'base_frame': 'camera',
                }
            ],
            remappings=[
                # Remap topics if using different camera names
                ('/camera/color/image_raw', '/camera/color/image_raw'),
                ('/camera/depth/image_rect_raw', '/camera/depth/image_rect_raw'),
            ]
        ),

        # RViz visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/path/to/vslam_rviz_config.rviz'],
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()
```

Launch it:
```bash
ros2 launch vslam_launch.py
```

**Expected output**:
```
[visual_slam-1] [INFO] Visual SLAM initialized
[visual_slam-1] [INFO] Camera calibration loaded
[visual_slam-1] [INFO] Keyframe 0 initialized
[visual_slam-1] [INFO] Map contains 142 points
[visual_slam-1] [INFO] Map contains 284 points
```

### Step 4: Visualize the Map in RViz

1. Open RViz:
   ```bash
   rviz2
   ```

2. Add displays:
   - **Add → MarkerArray** → `/map` (the 3D point cloud)
   - **Add → OccupancyGrid** → `/map` (2D cost map for navigation)
   - **Add → PoseArray** → `/camera_poses` (camera trajectory)

3. Move the camera around while watching the map grow in real-time.

## Configuration: vslam_params.yaml

Save as `docs/03-robot-brain/configs/vslam_params.yaml`:

```yaml
# Visual SLAM Parameters for isaac_ros_visual_slam

vision_slam_node:
  ros__parameters:
    # Input/Output topics
    input_img_topic: "/camera/color/image_raw"
    input_depth_topic: "/camera/depth/image_rect_raw"
    input_camera_info_topic: "/camera/color/camera_info"

    output_map_topic: "/map"
    output_odom_topic: "/odom"
    output_poses_topic: "/camera_poses"

    # Camera intrinsics (Intel RealSense D435)
    fx: 614.0      # Focal length X
    fy: 614.0      # Focal length Y
    cx: 320.0      # Principal point X
    cy: 240.0      # Principal point Y

    # Feature detector
    feature_detector:
      type: "ORB"  # ORB (fast) or SIFT (slow, accurate)
      num_features: 500
      scale_factor: 1.2
      num_levels: 8

    # Keyframe selection
    keyframe_threshold: 0.3          # Movement threshold (meters)
    min_tracked_features: 100         # Min features to track before new keyframe
    min_match_ratio: 0.7              # Min matching ratio to previous frame

    # Map optimization
    map_update_frequency_hz: 20.0
    local_map_size: 500               # Max keyframes in local map
    loop_closure_min_score: 0.8

    # Confidence threshold (0-1)
    # Higher = only high-confidence map points
    confidence_threshold: 0.85

    # Frame IDs (for TF tree)
    map_frame: "map"
    odom_frame: "odom"
    base_frame: "camera"

    # Performance tuning
    use_gpu: true                     # Use NVIDIA GPU (Jetson)
    max_workers: 2                    # Num parallel threads
    queue_size: 10                    # ROS 2 subscription queue
```

## Performance Benchmarks

| Hardware | Image Res | FPS | Map Points | Latency |
|----------|-----------|-----|-----------|---------|
| **Jetson Orin** | 640×480 | 30 | 200-400 | &lt;2 sec |
| **Jetson Orin NX** | 640×480 | 15 | 100-200 | &lt;3 sec |
| **RTX 4070 Ti** | 1920×1080 | 60 | 1000+ | &lt;0.5 sec |
| **Cloud (Omniverse)** | 1280×720 | 30 | 500 | &lt;1 sec |

## Troubleshooting

### Problem: "No features detected" / Map not growing

**Cause**: Camera pointed at featureless surface (blank wall).

**Solution**:
1. Point camera at textured environment (bookshelves, patterned objects)
2. Increase brightness: check camera exposure settings
3. Verify camera is in focus

### Problem: Map drifts or "jumps"

**Cause**: Accumulated odometry error or loop closure correction.

**Solution**:
1. Move camera more slowly
2. Ensure textured environment for stable feature tracking
3. Increase `confidence_threshold` to 0.9 (stricter)

### Problem: Latency > 2 seconds

**Cause**: GPU bottleneck or CPU throttling.

**Solution**:
1. Reduce image resolution (e.g., 640×480)
2. Check Jetson power mode:
   ```bash
   sudo /usr/bin/jetson_clocks  # Max performance
   ```
3. Monitor GPU/CPU usage:
   ```bash
   nvidia-smi  # Check GPU
   top         # Check CPU
   ```

## Key Takeaways

✅ **Visual SLAM** enables cheap, robust indoor mapping using RGB-D cameras
✅ **Feature detection and matching** form the core of VSLAM algorithms
✅ **Camera calibration** is essential for accurate 3D reconstruction
✅ **isaac_ros_visual_slam** on Jetson Orin achieves &lt;2 sec latency for real-time mapping
✅ **Compare with Lidar SLAM** to choose the right sensor for your environment

## Estimated Completion Time

- **Reading**: 45–50 minutes
- **Hands-on lab** (calibration + VSLAM + RViz viz): 45–60 minutes
- **Troubleshooting**: 15–20 minutes
- **Total**: 100–130 minutes

## Hardware Requirements

| Platform | Status | Notes |
|----------|--------|-------|
| **RTX 4070 Ti + RealSense D435** | ✅ Primary | Full resolution, 60 FPS |
| **Jetson Orin + CSI Camera** | ✅ Recommended | 30 FPS, &lt;2 sec latency |
| **Jetson Orin NX + USB Camera** | ⚠️ Limited | 15 FPS, may need resolution reduction |
| **Cloud (Omniverse)** | ✅ Alternative | 30 FPS, &lt;$5/hour |

---

**Next chapter**: [Navigation (Nav2)](./04-nav2-integration.md) — Turn the map into autonomous robot movement using the Nav2 navigation stack.
