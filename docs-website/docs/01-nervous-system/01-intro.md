---
id: intro-to-ros-2
sidebar_position: 1
---

# The Nervous System (ROS 2)

## Overview

Just like a human nervous system connects the brain to the body's muscles and sensors, **ROS 2 (Robot Operating System 2)** connects your robot's computational brain to its actuators (motors), sensors, and other hardware components.

In this module, we'll explore the fundamental concepts of ROS 2 by drawing parallels to biological systems:

- **Brain** = Your computer running ROS 2
- **Nerves** = ROS 2 Topics and Services
- **Muscles & Sensors** = Robot actuators and sensors
- **Signals** = Messages passed between nodes

## ROS 2 Nodes and Topics

### What are Nodes?

A **Node** is an independent process that performs a specific task in your robotic system. Just like your brain has different regions for vision, motor control, and decision-making, ROS 2 nodes specialize in specific functions:

- A **camera node** might capture images
- A **motor controller node** might drive wheel actuators
- A **decision-making node** might process sensor data and command actions

### What are Topics?

**Topics** are communication channels through which nodes exchange messages. Think of them as pathways in your nervous system:

- Nodes **publish** messages to topics (sending signals)
- Nodes **subscribe** to topics (receiving signals)
- Multiple nodes can publish and subscribe to the same topic

### Example: A Simple Talker Node

Here's a Python example of a ROS 2 node that publishes "Hello" messages every second:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker')
        # Create a publisher that sends String messages to 'chatter' topic
        self.publisher = self.create_publisher(String, 'chatter', 10)

        # Create a timer to call callback every 1 second
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.count}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    talker = TalkerNode()
    rclpy.spin(talker)
    talker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### How It Works

1. **Node Creation**: The `TalkerNode` inherits from `Node` and initializes itself with the name `'talker'`
2. **Publisher**: We create a publisher that sends `String` messages to the `'chatter'` topic
3. **Timer**: A timer triggers every 1 second, calling `timer_callback()`
4. **Publishing**: Each callback publishes a message with a counter value

## Key Concepts

| Concept | Explanation |
|---------|-------------|
| **Node** | An independent ROS 2 process performing a specific function |
| **Topic** | A named communication channel for asynchronous message passing |
| **Publisher** | A node component that sends messages to a topic |
| **Subscriber** | A node component that receives messages from a topic |
| **Message** | Structured data exchanged between nodes (e.g., String, Int32, Pose) |

## What's Next?

In the next section, we'll walk through installing ROS 2 Humble and setting up your development environment so you can run your first nodes.
