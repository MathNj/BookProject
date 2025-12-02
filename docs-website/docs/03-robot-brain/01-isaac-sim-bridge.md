# Bridging Isaac Sim to ROS 2

## Learning Objectives

By the end of this chapter, you will:
- Understand how Isaac Sim connects to ROS 2 through the Bridge Extension
- Configure and enable the ROS 2 Bridge in Isaac Sim
- Load a USD environment and verify sensor topics are publishing
- Monitor camera and depth sensor data streams using ROS 2 tools

## Prerequisites

- Completed Module 1: Nervous System (ROS 2 fundamentals)
- Completed Module 2: Digital Twin (Isaac Sim installation)
- RTX 4070 Ti GPU (or equivalent cloud setup via Omniverse Cloud)
- Docker container with Isaac ROS environment (optional but recommended)

## Concept: The Bridge to Reality

In Module 2, we learned how to build simulated robots in Isaac Sim—a physics-accurate, photorealistic digital environment. But simulation is only half the story. The real magic happens when we connect that simulation to **ROS 2**, the same middleware that controls physical robots.

### Why Bridge Isaac Sim to ROS 2?

1. **Seamless Hardware Transition**: Code developed and tested in Isaac Sim works unchanged on real robots
2. **Sensor Realism**: Isaac Sim generates pixel-perfect camera images and accurate depth measurements
3. **Scaled Training**: Generate thousands of labeled images per hour for machine learning models

### The Evolution: `ros_gz_bridge` → ROS 2 Bridge Extension

In the Gazebo era, we used **`ros_gz_bridge`**—a separate ROS 2 node that translated between Gazebo topics and ROS 2 topics. It worked, but required manual configuration and often created latency.

**Isaac Sim takes a different approach**: The **ROS 2 Bridge Extension** is built directly into Isaac Sim using **Omnigraph**, a visual node-based programming system. This means:

- ✅ Lower latency (native integration, no inter-process overhead)
- ✅ Visual configuration (you can see the data flow in the UI)
- ✅ Bi-directional communication (read sensors AND send commands)
- ✅ No extra Docker containers needed

## Step-by-Step: Enabling the ROS 2 Bridge

### Step 1: Launch Isaac Sim

Open Isaac Sim (version 2024.1 or later). You should see the main interface with:
- **Viewport** (center): The 3D scene
- **Stage panel** (right): Scene hierarchy
- **Content browser** (left): Asset library

### Step 2: Enable the ROS 2 Bridge Extension

1. Go to **Windows → Extensions** (top menu)
2. Search for `"ROS2 Bridge"` in the search box
3. Click the toggle to **Enable** the extension
   - You'll see a console message: `[INFO] ROS 2 Bridge initialized`
   - The extension loads in the background

```
Expected console output:
[INFO] Connected to ROS 2 domain ID: 0
[INFO] ROS 2 Bridge ready for Omnigraph nodes
```

### Step 3: Load a Warehouse USD Environment

1. Go to **File → Open** or press `Ctrl+O`
2. Navigate to: `nvidia/Assets/Isaac/2024.1/Environments/Simple_Warehouse/warehouse.usd`
3. Click **Open**
   - The warehouse scene loads with shelves, lighting, and textures
   - Simulation automatically plays (you'll see physics updates)

Alternatively, if the warehouse asset is not available locally, you can load a simple scene:
```
/home/user/.local/share/ov/pkg/isaac-sim-4.0.0/standalone_examples/resources/scenes/simple_room.usd
```

### Step 4: Add a Camera Sensor to the Scene

The warehouse may already have cameras, but let's verify or add one:

1. In the **Stage panel** (right), find the robot or create a new camera:
   - Right-click on the **World** or robot prim
   - Select **Create → Camera**
   - Or use the search bar: **Windows → Searchable Omnigraph**

2. Configure the camera:
   - Position: `(0, 0, 1.5)` (eye height)
   - Rotation: `(0, 0, 0)` (facing forward)
   - Camera type: `Perspective`

### Step 5: Configure ROS 2 Topic Publishers via Omnigraph

Now we'll connect the camera to ROS 2 topics using Omnigraph:

1. Go to **Windows → Omnigraph → Action Graph Editor**
2. In the new window, click **File → New Graph**
3. You'll see a blank canvas with a **Create Node** button

Add these nodes:

**Node 1: On Tick** (event trigger)
- **Name**: `OnTick`
- **Type**: `omni/graph/action/OnTick`
- **Purpose**: Fire an event every frame

**Node 2: Get Camera Image** (reads RGB from camera)
- **Type**: `omni/isaac/sensor/IsaacReadCamera`
- **Inputs**:
  - `execIn`: connect from `OnTick.execOut`
  - `cameraPrim`: select your camera prim (`/World/Camera` or similar)
  - `colorOutput`: RGB8
- **Outputs**:
  - `data`: RGB image buffer

**Node 3: ROS 2 Publish** (sends to ROS 2 topic)
- **Type**: `omni/isaac/ros2/PublishImage`
- **Inputs**:
  - `execIn`: connect from `IsaacReadCamera.execOut`
  - `data`: connect from `IsaacReadCamera.data`
  - `topicName`: `/rgb_left/image_raw`
  - `frameId`: `camera`
- **Outputs**: `execOut`

**Optional Node 4: Depth Sensor Publisher**
- **Type**: `omni/isaac/ros2/PublishImage`
- **Inputs**:
  - `topicName`: `/depth/image_raw`
  - `frameId`: `camera_depth`

4. Click **Evaluate Graph** to start publishing
5. You should see console output:
```
[INFO] Publishing /rgb_left/image_raw at 30 Hz
[INFO] Publishing /depth/image_raw at 30 Hz
```

### Step 6: Verify Topics Are Publishing

Open a **terminal window** (ROS 2 sourced environment) and run:

```bash
ros2 topic list
```

You should see:
```
/rgb_left/image_raw
/depth/image_raw
/camera_info
/clock
```

To inspect the image data stream:

```bash
ros2 topic echo /rgb_left/image_raw --once
```

Example output:
```
header:
  seq: 1234
  stamp:
    sec: 1701432000
    nsec: 500000000
  frame_id: camera
height: 1080
width: 1920
encoding: rgb8
is_bigendian: false
step: 5760
data: [255, 128, 64, ...]  # Raw RGB pixel values
```

To visualize the image in RViz:

```bash
rviz2 -d config.rviz
```

Then add an **Image** display with topic `/rgb_left/image_raw`. You'll see a live video feed from the Isaac Sim camera in RViz!

## Code Snippet: Python Script to Monitor Topics

Create a script `docs/03-robot-brain/code-examples/verify_topics.py`:

```python
#!/usr/bin/env python3
"""
verify_topics.py - Monitor ROS 2 topics from Isaac Sim Bridge
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np

class TopicMonitor(Node):
    def __init__(self):
        super().__init__('topic_monitor')

        # Subscribe to camera topics
        self.rgb_sub = self.create_subscription(
            Image, '/rgb_left/image_raw', self.rgb_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, '/depth/image_raw', self.depth_callback, 10
        )

        self.rgb_frame_count = 0
        self.depth_frame_count = 0
        self.get_logger().info('Monitoring ROS 2 topics from Isaac Sim...')

    def rgb_callback(self, msg):
        self.rgb_frame_count += 1
        if self.rgb_frame_count % 30 == 0:
            self.get_logger().info(
                f"RGB frames: {self.rgb_frame_count}, "
                f"Resolution: {msg.width}x{msg.height}, "
                f"Encoding: {msg.encoding}"
            )

    def depth_callback(self, msg):
        self.depth_frame_count += 1
        if self.depth_frame_count % 30 == 0:
            self.get_logger().info(
                f"Depth frames: {self.depth_frame_count}, "
                f"Resolution: {msg.width}x{msg.height}"
            )

def main(args=None):
    rclpy.init(args=args)
    monitor = TopicMonitor()
    rclpy.spin(monitor)
    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Run it:
```bash
python3 verify_topics.py
```

Expected output:
```
[INFO] Monitoring ROS 2 topics from Isaac Sim...
[INFO] RGB frames: 30, Resolution: 1920x1080, Encoding: rgb8
[INFO] Depth frames: 30, Resolution: 1920x1080
[INFO] RGB frames: 60, Resolution: 1920x1080, Encoding: rgb8
```

## Troubleshooting

### Problem: "ROS 2 Bridge Extension not found"

**Cause**: Extension not installed or Isaac Sim version too old.

**Solution**:
```bash
# Update Isaac Sim to latest version
omniverse-launcher --update isaac-sim

# Or manually enable via terminal
omni.isaac.ros2_bridge
```

### Problem: Topics not appearing in `ros2 topic list`

**Cause**: ROS 2 domain ID mismatch or Omnigraph not evaluating.

**Solutions**:
1. Check domain ID (default 0):
   ```bash
   echo $ROS_DOMAIN_ID  # Should output 0 (or your configured ID)
   ```

2. Verify Omnigraph is running:
   - In Isaac Sim, go to **Windows → Omnigraph → Action Graph Editor**
   - Check that your graph has **green checkmarks** on all nodes
   - Click **Play** (►) button to start evaluation

3. Check Isaac Sim logs:
   ```bash
   tail -f ~/.local/share/isaac-sim/logs/isaac.log | grep ROS2
   ```

### Problem: Image data is black or corrupted

**Cause**: Camera is not positioned in the scene or has no lighting.

**Solutions**:
1. Verify camera position in Stage panel:
   - Select the camera prim
   - Check **Translate** values (should be non-zero, e.g., `(0, 0, 1.5)`)

2. Add lighting to the scene:
   - Right-click **World** → **Create → Light → Distant Light**
   - Position it above the scene to illuminate objects

3. Verify texture rendering:
   - In Isaac Sim viewport, switch to **Rendered** mode (not wireframe)
   - Check that objects have visible materials

## Configuration Example: `warehouse.usda`

If building a custom scene, here's a minimal USD configuration:

```usda
#usda 1.0

def Xform "World"
{
    def Xform "Camera"
    {
        matrix4d xformOp:transform = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 1.5, 1) )

        def Camera "Camera"
        {
            float focalLength = 24
            float focusDistance = 400
            float f_number = 2.0
            float horizontalAperture = 20.955
            float verticalAperture = 11.6
        }
    }

    def Xform "Warehouse"
    {
        def Sphere "Cube" (
            prepend references = @./warehouse.usd@</Xform>
        )
        {
            double size = 1.0
            matrix4d xformOp:transform = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )
        }
    }

    def Xform "Lights"
    {
        def DomeLight "DomeLight"
        {
            asset inputs:texture:file = @./hdri_background.hdr@
            float intensity = 1.0
        }
    }
}
```

## Key Takeaways

✅ **Isaac Sim's ROS 2 Bridge** provides native, low-latency connection to ROS 2 ecosystems
✅ **Omnigraph** visual programming makes sensor configuration accessible without code
✅ **Camera and depth data** flow seamlessly into ROS 2 topics for perception pipelines
✅ **Verification tools** (`ros2 topic list`, RViz) confirm data is flowing correctly
✅ **The bridge is the gateway** to synthetic data generation and AI training (covered in next chapter)

## Estimated Completion Time

- **Reading**: 30-40 minutes
- **Hands-on lab**: 20-30 minutes
- **Total**: 50-70 minutes

## Hardware Requirements

| Platform | Status | Notes |
|----------|--------|-------|
| **RTX 4070 Ti (Development)** | ✅ Primary | Recommended for full performance |
| **Jetson Orin (Edge)** | ⚠️ Limited | Can run Isaac Sim lite version |
| **Cloud (AWS/Azure Omniverse)** | ✅ Alternative | &lt;$5/hour cost via Omniverse Cloud |

---

**Next chapter**: [Synthetic Data Generation](./02-synthetic-data-generation.md) — Learn how to generate thousands of labeled images in minutes using Isaac Replicator.
