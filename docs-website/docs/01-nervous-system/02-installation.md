# Setup & Installation

## Prerequisites

Before installing ROS 2 Humble, ensure you have:

- **Ubuntu 22.04 LTS** (recommended for best compatibility)
- **Administrator access** to your system
- **At least 2GB of free disk space**
- **Active internet connection**

## Installing ROS 2 Humble

ROS 2 Humble is the long-term support (LTS) release recommended for production and learning. Follow these steps:

### Step 1: Set up the Repository

First, add the ROS 2 GPG key and repository to your system:

```bash
# Update your system first
sudo apt update

# Install curl if not already installed
sudo apt install curl gnupg lsb-release ubuntu-keyring

# Add the ROS 2 GPG key
sudo curl -sSL https://repo.ros2.org/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the ROS 2 repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://repo.ros2.org/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Step 2: Install ROS 2 Humble Desktop

Update your package list and install ROS 2 Humble with the desktop development tools:

```bash
# Update package list
sudo apt update

# Install ROS 2 Humble Desktop (includes RViz visualization tool)
sudo apt install ros-humble-desktop

# Install additional development tools
sudo apt install ros-humble-dev-tools
```

**Note**: This installation includes:
- ROS 2 core libraries
- RViz (3D visualization tool)
- Colcon (build system)
- Demo nodes for testing

### Step 3: Source the Setup Script

Before using ROS 2, you need to source the setup script:

```bash
source /opt/ros/humble/setup.bash
```

To source this automatically every time you open a terminal, add it to your `.bashrc`:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 4: Verify the Installation

Check that ROS 2 is properly installed:

```bash
ros2 --version
```

You should see output like: `ROS 2 Humble (released in May 2023)`

## Running Your First Node

ROS 2 comes with demo nodes that you can run immediately to verify everything is working:

### Terminal 1: Run the Talker Node

```bash
ros2 run demo_nodes_cpp talker
```

You should see output like:
```
[INFO] [talker]: Publishing: 'Hello World: 1'
[INFO] [talker]: Publishing: 'Hello World: 2'
[INFO] [talker]: Publishing: 'Hello World: 3'
...
```

### Terminal 2: Run the Listener Node

Open a new terminal and run:

```bash
ros2 run demo_nodes_cpp listener
```

You should see output like:
```
[INFO] [listener]: I heard: [Hello World: 1]
[INFO] [listener]: I heard: [Hello World: 2]
[INFO] [listener]: I heard: [Hello World: 3]
...
```

**Congratulations!** The talker node is publishing "Hello World" messages, and the listener node is receiving them. This demonstrates ROS 2's publish-subscribe communication model in action.

## Verification Checklist

- [ ] Ubuntu 22.04 LTS is installed
- [ ] ROS 2 Humble is installed without errors
- [ ] `ros2 --version` shows ROS 2 Humble
- [ ] Talker node runs successfully
- [ ] Listener node receives messages from talker
- [ ] Setup script is sourced in your `.bashrc`

## Troubleshooting

### Problem: "Command ros2 not found"
**Solution**: Make sure you've sourced the setup script:
```bash
source /opt/ros/humble/setup.bash
```

### Problem: Demo nodes don't run
**Solution**: Reinstall the demo nodes:
```bash
sudo apt install ros-humble-demo-nodes-cpp
```

### Problem: Permission denied during installation
**Solution**: Make sure you're using `sudo` for system-wide installation:
```bash
sudo apt install ros-humble-desktop
```

## Next Steps

Now that ROS 2 is installed, you're ready to:
1. Learn how to create your own ROS 2 package
2. Write publisher and subscriber nodes
3. Explore the ROS 2 communication ecosystem

Continue to the next module to start building your first robot application!
