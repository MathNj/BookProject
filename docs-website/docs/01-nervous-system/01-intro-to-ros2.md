---
title: Introduction to ROS 2 - The Robotic Nervous System
sidebar_label: Introduction to ROS 2
---

# Introduction to ROS 2: The Robotic Nervous System

## Learning Outcomes

By the end of this chapter, you will be able to:
- Map biological nervous system components to ROS 2 architecture
- Explain the role of nodes, topics, and services in distributed robotics
- Understand how the publish-subscribe pattern enables robot coordination
- Visualize communication between multiple ROS 2 nodes
- Apply the nervous system analogy to real-world robotics problems

---

## Overview: The Nervous System Analogy

Think about how your body works. When you touch something hot, your sensory neurons instantly transmit a signal to your brain. Your brain processes that information and sends a command to your muscles to pull your hand away. All of this happens through a network of specialized components working together—your *nervous system*.

ROS 2 (Robot Operating System 2) is like the nervous system of a robot. Instead of biological neurons and nerves, robots have **nodes** and **topics** that communicate in real time. Just as your body coordinates complex behaviors through neural networks, a robot coordinates its sensors, processors, and actuators through ROS 2.

### The Biological ↔ ROS 2 Mapping

| Biological Component | Function | ROS 2 Equivalent | Role |
|---|---|---|---|
| **Brain** | Central processing, decision-making | **Computer/Main Node** | Runs all ROS 2 software |
| **Neurons** | Processing units, signal generation | **Nodes** | Independent programs that perform tasks |
| **Sensory Nerves** | Transmit data from body to brain | **Subscriber Nodes** | Listen to sensor data from topics |
| **Motor Nerves** | Transmit commands from brain to muscles | **Publisher Nodes** | Send commands to actuators via topics |
| **Nerves** | Communication pathways | **Topics** | Named communication channels |
| **Muscles** | Actuators that perform actions | **Actuators** | Motors, grippers, speakers, etc. |
| **Synapse** | Connection point between neurons | **Topic Subscription** | Where nodes connect to data streams |

---

## ROS 2 Nodes: The Brain's Components

### What is a Node?

A **node** is an independent program that performs a single, well-defined task. Nodes are the building blocks of any ROS 2 system. Each node:

- Runs as a separate process (can run on the same computer or across a network)
- Has its own identity and lifecycle
- Communicates with other nodes through topics and services
- Can be started, stopped, or restarted independently without affecting other nodes

### Real-World Examples

**Example 1: A Mobile Robot**
- **Camera Node**: Captures images from the robot's camera and publishes them
- **Navigation Node**: Reads camera images and publishes movement commands
- **Motor Driver Node**: Listens to movement commands and controls the wheels

**Example 2: A Robotic Arm**
- **Sensor Node**: Reads force/torque sensors and publishes pressure data
- **Control Node**: Analyzes pressure and decides arm movements
- **Joint Driver Node**: Receives movement commands and actuates motors

### Node Graph: Visualization

A **node graph** is a visual representation of how nodes connect and communicate. Here's a simple example:

```
┌─────────────────┐
│   Camera Node   │
│  (Publisher)    │
└────────┬────────┘
         │ Publishes: /camera/rgb_image
         ▼
    ┌─────────┐
    │  Topic  │
    │/camera/ │
    │rgb_image│
    └────┬────┘
         │ Subscribes
         ▼
┌─────────────────────────┐
│  Image Processing Node  │
│   (Subscriber)          │
│   (Publisher)           │
└────────┬────────────────┘
         │ Publishes: /control/motor_cmd
         ▼
    ┌────────────┐
    │   Topic    │
    │/control/   │
    │motor_cmd   │
    └────┬───────┘
         │ Subscribes
         ▼
┌──────────────────────┐
│   Motor Driver Node  │
│   (Subscriber)       │
└──────────────────────┘
```

---

## ROS 2 Topics: The Communication Highways

### What is a Topic?

A **topic** is a named communication channel through which nodes exchange messages. Think of it like a radio frequency:

- **Nodes publish** messages to a topic (like broadcasting on a radio station)
- **Nodes subscribe** to a topic (like tuning into a radio station)
- Multiple nodes can publish to the same topic
- Multiple nodes can subscribe to the same topic
- Topics are decoupled—publishers don't need to know about subscribers and vice versa

### The Publish-Subscribe Pattern

ROS 2 uses the **publish-subscribe (pub-sub)** pattern for communication:

1. **Publisher Node**: Generates data and sends it to a topic
   - Example: A camera node publishing image frames 30 times per second
   - The publisher doesn't care who reads the data

2. **Topic**: Acts as a mailbox for messages
   - Stores the most recent message
   - Delivers that message to all current subscribers
   - When a new message arrives, it replaces the old one

3. **Subscriber Node**: Listens to a topic and processes incoming messages
   - Example: A vision processing node subscribing to camera images
   - The subscriber doesn't care who publishes the data

### Why Pub-Sub Matters

The pub-sub pattern is powerful because:

- **Decoupling**: Publishers and subscribers don't need to know about each other
- **Scalability**: Easy to add new subscribers without modifying publishers
- **Flexibility**: Nodes can be restarted, moved, or replaced independently
- **Simplicity**: Clean separation of concerns—each node does one thing well

### Message Types

Each topic has an associated **message type** that defines the structure of data it carries:

```
Topic: /robot/position
Message Type: geometry_msgs/Pose

Contains:
├── position (x, y, z coordinates)
├── orientation (rotation in 3D space)
└── timestamp
```

Common ROS 2 message types include:
- `std_msgs/Float32` — A single floating-point number
- `sensor_msgs/Image` — Camera image data
- `geometry_msgs/Twist` — Linear and angular velocity
- `std_msgs/String` — Text messages

---

## ROS 2 Services: Request-Response Communication

While topics are perfect for streaming data (one-way communication), sometimes nodes need to ask questions and get immediate responses. That's where **services** come in.

### What is a Service?

A **service** is a request-response communication pattern:
- A **client node** sends a request and waits for a reply
- A **service node** (server) receives the request, processes it, and sends a response
- The client is blocked until it gets the response

### When to Use Services vs Topics

| Use Case | Topics | Services |
|---|---|---|
| Continuous data stream | ✓ | ✗ |
| One-time queries | ✗ | ✓ |
| Sensor data | ✓ | ✗ |
| RPC-style calls | ✗ | ✓ |
| Decoupled systems | ✓ | ✗ |
| Request confirmation | ✗ | ✓ |

**Example**: A robot might use topics to stream arm position every 100ms, but use services to request "Can you reach point X?" and wait for a yes/no response.

Services are introduced in User Story 2 with practical examples.

---

## The ROS 2 Middleware: How It All Works

### The Distributed Graph

ROS 2 maintains a **graph** of all running nodes, topics, and services:

- When you start a node, it registers itself with the ROS 2 system
- When a node publishes to a topic, the middleware records this
- When a node subscribes to a topic, the middleware connects it to existing publishers
- The graph is dynamic—nodes can join and leave at any time

### Message Transportation

Here's what happens when a publisher sends a message:

1. **Publication**: Node calls `publisher.publish(message)`
2. **Serialization**: Message is converted to bytes
3. **Distribution**: Middleware delivers to all current subscribers
4. **Deserialization**: Each subscriber converts bytes back to message object
5. **Callback**: Subscriber's callback function processes the message

This entire process happens in milliseconds.

### Domain ID: Separating Networks

ROS 2 allows multiple independent networks on the same physical network using **Domain IDs**:

```bash
# Start node in Domain 0 (default)
ros2 run my_package my_node

# Start node in Domain 1 (different network)
ROS_DOMAIN_ID=1 ros2 run my_package my_node
```

Nodes with different Domain IDs cannot communicate—useful for running multiple independent robot systems in the same room.

---

## Real-World Robotics Context

### Mobile Robot Example

A warehouse delivery robot uses ROS 2 nodes like:

```
┌──────────────────┐
│  GPS Node        │──► /gps/position
├──────────────────┤
│  LIDAR Node      │──► /lidar/scan
├──────────────────┤
│  Navigation Node │◄── /gps/position
│                  │◄── /lidar/scan
│                  │──► /cmd_vel (velocity commands)
├──────────────────┤
│  Motor Driver    │◄── /cmd_vel
│  Node            │──► /wheel/speed (feedback)
└──────────────────┘
```

The **Navigation Node** is the "brain"—it reads multiple sensor topics and makes decisions about where to move. It publishes motion commands without caring about the motors; the Motor Driver Node handles the low-level control.

### Humanoid Robot Example

A humanoid robot coordinating complex behaviors:

```
Topics in Use:
- /camera/rgb_image (Vision)
- /microphone/audio (Speech Input)
- /arm/position (Arm Feedback)
- /gripper/force (Gripper Sensors)

Nodes in Use:
- Vision Processing Node
- Speech Recognition Node
- Task Planning Node (the decision maker)
- Arm Control Node
- Gripper Control Node
```

Each subsystem (vision, speech, arm, gripper) can be developed and tested independently, then integrated through topics.

---

## Key Concepts Summary

| Concept | Definition | Analogy |
|---|---|---|
| **Node** | Independent program performing a task | Neuron in the brain |
| **Topic** | Named communication channel | Nerve pathway |
| **Message** | Data structure transmitted on a topic | Signal traveling along a nerve |
| **Publisher** | Node sending data to a topic | Sensory neuron |
| **Subscriber** | Node receiving data from a topic | Motor neuron |
| **Graph** | Visualization of nodes and topics | Neural network diagram |
| **Middleware** | Infrastructure managing communication | Nervous system's connectivity |

---

## Next Steps

Now that you understand ROS 2's architecture, you're ready to:

1. **Install ROS 2 Humble** (next chapter) and verify it works
2. **Write your own nodes** (User Story 2) and see the pub-sub pattern in action
3. **Explore URDF** (User Story 3) to describe robot structures
4. **Build real robots** that use these concepts

---

## Exercises

Try answering these questions:

1. **Analogy Check**: What biological component would a temperature sensor node map to? (Answer: A sensory neuron)

2. **Architecture Design**: A robot arm has:
   - Joint position sensors
   - A planning algorithm
   - Motor drivers

   Draw a node graph showing how these components might communicate using topics.

3. **Topic Naming**: Suggest appropriate topic names for:
   - A robot's battery voltage
   - A robot's target destination
   - A robot's current movement command

4. **Service or Topic?**: For each scenario, decide if you'd use a service or topic:
   - Streaming camera images every 30ms
   - Asking "What is the current battery percentage?"
   - Reporting arm position feedback
   - Requesting a path to a destination

---

**Ready to proceed?** In the next chapter, you'll install ROS 2 and run your first nodes to see these concepts in action.
