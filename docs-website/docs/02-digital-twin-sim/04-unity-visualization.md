# Unity for High-Fidelity Visualization

**Status**: Optional Advanced Topic

:::info Optional Chapter
This chapter is optional. Complete Chapters 1-4 first. Unity provides photorealistic visualization; Gazebo alone is sufficient for algorithm development. Use Unity when you need:
- Client demonstrations (photorealistic rendering)
- Remote visualization (web-based dashboards)
- Extended Reality (AR/VR) interfaces
- Marketing materials and investor demos
:::

## Why Unity After Gazebo?

### The Gazebo + Unity Workflow

Professional robotics teams use **both** tools strategically:

```
Your Code
    ↓
Gazebo (Physics Simulation)
    ↓
    ├─→ Raw sensor data (for algorithm development)
    │
    └─→ Robot state (position, orientation)
            ↓
        TCP/UDP Bridge
            ↓
        Unity (Visualization)
            ↓
            ├─→ 3D Display (high quality)
            ├─→ Real-time dashboards
            ├─→ AR overlay for field deployment
            └─→ Client presentations
```

### Decision Matrix: Gazebo vs Unity

| Aspect | Gazebo | Unity |
|--------|--------|-------|
| **Primary Purpose** | Physics simulation | Visualization |
| **Visual Fidelity** | Engineering quality | Photorealistic |
| **Rendering Speed** | 30-60 FPS | 120+ FPS possible |
| **ROS 2 Integration** | Native (ros-humble-ros-gz) | Via ROS-TCP-Connector |
| **Hardware Simulation** | Accurate physics | Visual only |
| **Real-time Performance** | less than 5ms latency | 10-20ms latency |
| **Best for Algorithms** | ✅ Yes | ❌ No (visualization only) |
| **Best for Demos** | ⚠️ Acceptable | ✅ Yes (beautiful) |
| **Cost** | Free, open-source | Free (with license) |
| **Learning Curve** | Moderate (robotics-focused) | Steep (game engine) |

### When to Use ONLY Gazebo

**Skip Unity if you're**:
- Developing and testing algorithms
- Focused on physics accuracy
- Limited by time/budget
- Running on Linux servers

### When to ADD Unity

**Add Unity when you're**:
- Presenting to non-technical stakeholders
- Deploying to field (need AR/VR guidance)
- Building client dashboards
- Marketing your robotics system
- Testing human-robot interaction

---

## Architecture: ROS-TCP-Connector

### How Gazebo and Unity Talk

```
Machine 1 (Linux)          Machine 2 (Windows/Mac/Linux)
┌──────────────────┐       ┌──────────────────────────┐
│  Gazebo Server   │       │  Unity Editor/Runtime    │
│  (Physics 1000Hz)├──TCP──┤  (Rendering 60+ FPS)     │
│                  │       │                          │
│ Publishes:       │       │ Receives:                │
│ - /clock         │       │ - Joint states           │
│ - Robot pose     │       │ - Sensor data            │
│ - Sensor data    │       │ - Physics updates        │
│                  │       │                          │
│ Subscribes:      │       │ Sends:                   │
│ - Joint commands ├─TCP─→ │ - User inputs            │
│ - Visualization  │       │ - Camera controls        │
│   requests       │       │ - Debug messages         │
└──────────────────┘       └──────────────────────────┘
     │                            │
     └────────────────────────────┘
        (Same LAN or VPN)
```

### Key Components

**ROS-TCP-Connector** (Open-source):
- Converts ROS 2 messages to Unity serializable format
- Handles clock synchronization
- Runs on both Gazebo (ROS 2) and Unity (C#)
- Latency: typically 10-20ms over local network

**Message Bridge**:
- Gazebo publishes sensor data (LaserScan, Image, Imu)
- Unity receives and renders (PointCloud, texture, orientation)
- Bidirectional: Unity sends user input → Gazebo reads commands

---

## Setup: Gazebo + Unity Connection

### Prerequisites

**On Gazebo Machine (Linux)**:
- Ubuntu 22.04 LTS
- ROS 2 Humble (already installed)
- Gazebo Fortress (already installed)

**On Unity Machine**:
- Unity 2022.3 LTS (free, download from unity.com)
- ROS-TCP-Connector package (free, from GitHub)
- Minimum specs: 4GB RAM, 8GB disk space

### Step 1: Install ROS-TCP-Connector (on Gazebo Linux)

```bash
# Clone the ROS-TCP-Connector repository
cd ~/ros2_ws/src
git clone https://github.com/Unity-Technologies/ros2_tcp_connector.git

# Build the package
cd ~/ros2_ws
colcon build --packages-select ros_tcp_endpoint

# Source it
source ~/ros2_ws/install/setup.bash
```

### Step 2: Launch Gazebo with TCP Bridge

**Terminal 1** - Start Gazebo world:
```bash
source /opt/ros/humble/setup.bash
ros2 launch gazebo_ros gazebo.launch.py world:=simple_world.sdf
```

**Terminal 2** - Start ROS-TCP bridge (listens on port 10000):
```bash
source ~/ros2_ws/install/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_DOMAIN_ID:=0
```

### Step 3: Import URDF into Unity

In Unity 2022.3:
1. Window → ROS2 → URDF Importer
2. Select `robot_sim.urdf`
3. Click "Import URDF"
4. Configure joint controllers (shoulder, elbow)

### Step 4: Configure Communication

In Unity:
1. Create empty GameObject: "ROS2Bridge"
2. Add component: "ROS TCP Connector"
3. Set IP: `192.168.1.X` (Gazebo machine IP)
4. Set Port: `10000`
5. Press Play

**You should see**:
- Robot appears in Unity viewport
- Moves in real-time with Gazebo simulation
- Sensor data streams in (if subscribed)

---

## Visualizing Sensor Data in Unity

### LiDAR Visualization

Display Gazebo LiDAR as PointCloud in Unity:

```csharp
using RosMessageTypes.Sensor;
using UnityEngine;

public class LidarVisualizer : MonoBehaviour
{
    // Subscribe to /robot/lidar/scan from Gazebo
    void OnLaserScan(LaserScanMsg laserScan)
    {
        // Convert laser rays to world positions
        Vector3[] pointCloud = new Vector3[laserScan.ranges.Length];

        for (int i = 0; i less than laserScan.ranges.Length; i++)
        {
            float distance = laserScan.ranges[i];
            float angle = laserScan.angle_min + (i * laserScan.angle_increment);

            // Convert polar (angle, distance) to cartesian (x, y)
            pointCloud[i] = new Vector3(
                Mathf.Cos(angle) * distance,
                0.1f,  // Height above ground
                Mathf.Sin(angle) * distance
            );
        }

        // Render points as small spheres or particles
        RenderPointCloud(pointCloud);
    }

    void RenderPointCloud(Vector3[] points)
    {
        // Draw points in viewport
        // (Detailed implementation: create mesh with point vertices)
    }
}
```

### Camera Feed Visualization

Display Gazebo depth camera in Unity:

```csharp
using RosMessageTypes.Sensor;
using UnityEngine;

public class CameraVisualizer : MonoBehaviour
{
    Texture2D cameraTexture;

    void OnCameraImage(ImageMsg image)
    {
        // Convert ROS Image to Unity Texture2D
        byte[] imageData = image.data;
        cameraTexture = new Texture2D(
            (int)image.width,
            (int)image.height,
            TextureFormat.RGB24,
            false
        );
        cameraTexture.LoadRawTextureData(imageData);
        cameraTexture.Apply();

        // Display on UI canvas or 3D quad
        GetComponent less than RawImage greater than().texture = cameraTexture;
    }
}
```

### IMU Orientation Visualization

Show robot orientation from Gazebo IMU:

```csharp
using RosMessageTypes.Sensor;
using UnityEngine;

public class OrientationVisualizer : MonoBehaviour
{
    void OnIMUData(ImuMsg imuData)
    {
        // Extract quaternion orientation
        Quaternion rotation = new Quaternion(
            imuData.orientation.x,
            imuData.orientation.y,
            imuData.orientation.z,
            imuData.orientation.w
        );

        // Apply to robot GameObject
        GetComponent less than Transform greater than().rotation = rotation;

        // Show acceleration as arrow
        Vector3 accel = new Vector3(
            imuData.linear_acceleration.x,
            imuData.linear_acceleration.y,
            imuData.linear_acceleration.z
        );
        DrawAccelerationArrow(accel);
    }

    void DrawAccelerationArrow(Vector3 acceleration)
    {
        // Draw red arrow showing acceleration direction
        Debug.DrawLine(transform.position, transform.position + acceleration.normalized, Color.red);
    }
}
```

---

## Remote Visualization (Web Dashboard)

### Streaming Data to Web Browser

Using ros2-web-bridge (optional, for web dashboards):

```javascript
// In a web page (HTML + JavaScript)

const ros = new ROSLIB.Ros({
  url: 'ws://gazebo-machine-ip:9090'  // WebSocket bridge
});

ros.on('connection', function() {
  console.log('Connected to Gazebo');

  // Subscribe to LiDAR
  const lidarTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/robot/lidar/scan',
    messageType: 'sensor_msgs/LaserScan'
  });

  lidarTopic.subscribe(function(message) {
    // Render point cloud in web browser using THREE.js
    renderPointCloud(message.ranges);
  });
});
```

**Result**: View live robot data in any web browser!

---

## Advanced: Extended Reality (AR/VR)

### AR for Field Deployment

Overlay robot in real-world using phone camera:

```csharp
// Using ARKit (iPhone) or ARCore (Android)
using UnityEngine.XR.ARFoundation;

public class ARRobotOverlay : MonoBehaviour
{
    void Update()
    {
        // Get phone camera position
        Camera mainCamera = Camera.main;

        // Sync robot virtual position with real-world view
        // Show sensor overlays in AR
        DisplayARPointCloud();
        DisplayARTarget();
    }

    void DisplayARPointCloud()
    {
        // Render LiDAR scan as AR objects in scene
        // Shows obstacles in real-world coordinates
    }

    void DisplayARTarget()
    {
        // Show goal location as AR marker
        // Help operators navigate robot
    }
}
```

**Use Case**: Remote robot operation - Operator sees robot's view + sensor data overlaid on phone.

---

## Performance Optimization

### Frame Rate Considerations

| Setting | FPS | Latency | Quality |
|---------|-----|---------|---------|
| **Low (Mobile AR)** | 30 FPS | 33ms | Standard |
| **Medium (Desktop)** | 60 FPS | 17ms | High |
| **High (VR)** | 90+ FPS | less than 12ms | Very High |

### Network Optimization

**Reduce bandwidth** if streaming over slow connection:

```csharp
// Compress point cloud before sending
public class PointCloudCompressor
{
    public static byte[] Compress(Vector3[] points)
    {
        // Send every 4th point instead of all
        // Reduces bandwidth by 75%
        List less than Vector3 greater than compressed = new List less than Vector3 greater than();
        for (int i = 0; i less than points.Length; i += 4)
        {
            compressed.Add(points[i]);
        }
        return SerializeToBytes(compressed.ToArray());
    }
}
```

---

## Troubleshooting Connection

### Problem 1: Unity Can't Connect to Gazebo

**Error**: "Connection refused" on port 10000

**Solution**:
1. Verify ROS-TCP bridge is running: `ps aux | grep ros_tcp`
2. Check firewall: `sudo ufw allow 10000`
3. Verify IP address: `hostname -I` (on Gazebo machine)
4. Update Unity with correct IP

### Problem 2: Robot Not Moving in Unity

**Symptoms**: Robot appears but doesn't move

**Solution**:
1. Check Gazebo is publishing joint states: `ros2 topic hz /joint_states`
2. Verify URDF imported correctly in Unity
3. Check topic names match in bridge configuration

### Problem 3: High Latency (greater than 100ms)

**Symptoms**: Robot movements delayed by 1+ seconds

**Solution**:
1. Check network: `ping gazebo-machine-ip`
2. Reduce point cloud size (fewer rays)
3. Lower camera resolution
4. Run on same LAN (not over internet)

---

## When NOT to Use Unity

Unity is **not recommended** for:
- ❌ Physics simulation (use Gazebo)
- ❌ Sensor noise modeling (use Gazebo)
- ❌ Algorithm development (too slow, too complex)
- ❌ Real-time control loops (latency too high)

**Use Gazebo for algorithm development, then add Unity for presentation.**

---

## Next Steps

✅ **Want photorealistic visualization?**

👉 **Optional Workflow**:
1. Develop algorithm in Gazebo (Chapters 1-4)
2. Test in Gazebo (real-time, accurate)
3. Export to Unity for presentation (once algorithm works)
4. Demo to stakeholders/clients

---

## Chapter Summary

| Concept | Key Takeaway |
|---------|------|
| **Gazebo vs Unity** | Physics-first (Gazebo) vs Visual-first (Unity) |
| **ROS-TCP-Connector** | TCP bridge: 10-20ms latency, real-time sync |
| **Sensor Visualization** | LiDAR (PointCloud), Camera (texture), IMU (orientation) |
| **Remote Operation** | Web dashboards and AR/VR field deployment |
| **Integration** | Run Gazebo on server, Unity on local machine |

✅ **Recommended**: Master Chapters 1-4 (Gazebo) before adding Unity visualization.

:::success Ready to Deploy?
After completing Module 2, students can:
1. Run algorithms in real Gazebo simulation
2. Verify with all sensors (LiDAR, camera, IMU)
3. Visualize results in Unity
4. Proceed to Module 3: Robot Brain (AI/ML integration)
:::
