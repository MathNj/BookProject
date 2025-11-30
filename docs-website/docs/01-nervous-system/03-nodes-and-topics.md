---
title: Writing Your First ROS 2 Nodes
sidebar_label: Nodes and Topics (Python)
---

# Writing Your First ROS 2 Nodes

## Learning Outcomes

By the end of this chapter, you will be able to:
- Understand the structure of a ROS 2 node in Python
- Write a publisher node that sends messages
- Write a subscriber node that receives messages
- Run both nodes together and see them communicate
- Debug common issues in node communication
- Modify code examples to experiment with different behaviors

---

## Quick Review: Nodes and Topics

Before writing code, let's recap the concepts from Chapter 1:

- **Node**: An independent program that does one specific task
- **Topic**: A named communication channel where messages flow
- **Publisher**: A node that sends messages to a topic (like broadcasting)
- **Subscriber**: A node that listens to a topic (like tuning into a radio station)
- **Message**: The data being transmitted (e.g., a string, a number, sensor data)

In this chapter, we'll write:
1. A **Publisher Node** that sends "Hello World" messages
2. A **Subscriber Node** that receives and prints those messages

---

## Tutorial 1: Writing a Publisher Node

### What Does a Publisher Do?

A publisher node:
- Sends data out to a topic at regular intervals
- Doesn't care if anyone is listening (fire-and-forget pattern)
- Continues publishing even if no subscribers are present
- Real-world example: A temperature sensor constantly publishing readings

### The Publisher Code: `minimal_publisher.py`

```python
#!/usr/bin/env python3
"""
Minimal ROS 2 Publisher Example

This node publishes a simple string message to the 'chatter' topic every 0.5 seconds.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    """A ROS 2 node that publishes messages to the 'chatter' topic."""

    def __init__(self):
        """Initialize the MinimalPublisher node."""
        # Call the parent Node class constructor with the node name
        super().__init__('minimal_publisher')

        # Create a publisher for String messages on the 'chatter' topic
        # queue_size=10 means buffer up to 10 messages if no subscribers
        self.publisher_ = self.create_publisher(String, 'chatter', 10)

        # Create a timer that calls publish_message() every 0.5 seconds
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_message)

        # Counter for message sequence
        self.i = 0

    def publish_message(self):
        """Timer callback function that publishes a message."""
        # Create a new String message object
        msg = String()

        # Set the message data
        msg.data = f'Hello World: {self.i}'

        # Publish the message to subscribers on the 'chatter' topic
        self.publisher_.publish(msg)

        # Log the published message (for debugging)
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment the counter
        self.i += 1


def main(args=None):
    """Main function that starts the ROS 2 publisher node."""
    # Initialize the ROS 2 system
    rclpy.init(args=args)

    # Create an instance of our MinimalPublisher node
    minimal_publisher = MinimalPublisher()

    try:
        # Start spinning (running) the node
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        print("\nShutting down publisher node...")
    finally:
        # Clean up
        minimal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Breaking Down the Publisher Code

#### Line-by-Line Explanation

**Imports** (lines 1-4):
```python
import rclpy                      # ROS 2 Python library
from rclpy.node import Node       # Base class for all nodes
from std_msgs.msg import String   # Message type for text strings
```

**Class Definition** (line 7):
```python
class MinimalPublisher(Node):
```
- Inherits from `Node` to become a ROS 2 node
- This is the pattern for all ROS 2 nodes—inherit from `Node`

**Constructor** (lines 10-16):
```python
def __init__(self):
    super().__init__('minimal_publisher')  # Give the node a name
    self.publisher_ = self.create_publisher(String, 'chatter', 10)
    self.timer = self.create_timer(0.5, self.publish_message)
```
- `super().__init__('minimal_publisher')` — Registers this node with ROS 2
- `create_publisher(String, 'chatter', 10)` — Creates a publisher that:
  - Sends `String` messages (imported from `std_msgs.msg`)
  - On the topic called `'chatter'`
  - With a queue size of 10 (buffer for subscribers that lag)
- `create_timer(0.5, self.publish_message)` — Sets up a timer that calls `publish_message()` every 0.5 seconds

**Publish Method** (lines 18-28):
```python
def publish_message(self):
    msg = String()                  # Create a new message object
    msg.data = f'Hello World: {self.i}'  # Set the message content
    self.publisher_.publish(msg)    # Send it out
    self.get_logger().info(...)     # Print to console
    self.i += 1                     # Increment counter
```
- Called automatically by the timer every 0.5 seconds
- Creates a message, fills it with data, and publishes it

**Main Function** (lines 31-49):
```python
def main(args=None):
    rclpy.init(args=args)              # Start ROS 2
    minimal_publisher = MinimalPublisher()  # Create the node
    rclpy.spin(minimal_publisher)      # Run forever (until Ctrl+C)
```
- `rclpy.init()` — Initializes the ROS 2 system
- `rclpy.spin()` — Blocks and processes callbacks (timers, subscriptions)
- `rclpy.shutdown()` — Cleanup when done

### How to Run the Publisher

1. **Open a terminal** in your ROS 2 environment

2. **Source the ROS 2 setup file**:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

3. **Run the publisher**:
   ```bash
   python minimal_publisher.py
   ```

4. **Expected output**:
   ```
   [minimal_publisher]: Publishing: "Hello World: 0"
   [minimal_publisher]: Publishing: "Hello World: 1"
   [minimal_publisher]: Publishing: "Hello World: 2"
   ...
   ```

The publisher will continue printing messages every 0.5 seconds until you press **Ctrl+C** to stop it.

---

## Tutorial 2: Writing a Subscriber Node

### What Does a Subscriber Do?

A subscriber node:
- Listens to a topic for incoming messages
- Receives messages through a callback function
- Processes each message as it arrives
- Real-world example: A robot's brain listening to sensor readings

### The Subscriber Code: `minimal_subscriber.py`

```python
#!/usr/bin/env python3
"""
Minimal ROS 2 Subscriber Example

This node subscribes to the 'chatter' topic and prints each received message.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):
    """A ROS 2 node that subscribes to messages from the 'chatter' topic."""

    def __init__(self):
        """Initialize the MinimalSubscriber node."""
        # Call the parent Node class constructor with the node name
        super().__init__('minimal_subscriber')

        # Create a subscription to String messages on the 'chatter' topic
        # When a message arrives, listener_callback() is called automatically
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)

        # Prevent unused variable warning
        self.subscription  # noqa: F841

    def listener_callback(self, msg):
        """Callback function that processes received messages."""
        # Extract the string data from the message
        message_text = msg.data

        # Log the received message to the console
        self.get_logger().info(f'I heard: "{message_text}"')


def main(args=None):
    """Main function that starts the ROS 2 subscriber node."""
    # Initialize the ROS 2 system
    rclpy.init(args=args)

    # Create an instance of our MinimalSubscriber node
    minimal_subscriber = MinimalSubscriber()

    try:
        # Start spinning (running) the node
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        print("\nShutting down subscriber node...")
    finally:
        # Clean up
        minimal_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Breaking Down the Subscriber Code

**Key Differences from the Publisher:**

| Aspect | Publisher | Subscriber |
|---|---|---|
| Method | `create_publisher()` | `create_subscription()` |
| Parameters | `(MessageType, topic_name, queue_size)` | `(MessageType, topic_name, callback, queue_size)` |
| Callback | Timer (periodic) | Message arrival (event-driven) |
| Pattern | Push (active sending) | Pull/Wait (passive listening) |

**The Callback** (lines 20-24):
```python
def listener_callback(self, msg):
    """This is called automatically when a message arrives."""
    message_text = msg.data
    self.get_logger().info(f'I heard: "{message_text}"')
```
- Defined in `create_subscription()` as the third argument
- Called automatically by ROS 2 whenever a message arrives
- Receives the message as a parameter
- Can be named anything (we called it `listener_callback`)

### How to Run the Subscriber

1. **Open a second terminal** (keep the publisher running in the first)

2. **Source ROS 2**:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

3. **Run the subscriber**:
   ```bash
   python minimal_subscriber.py
   ```

4. **Expected output**:
   ```
   [minimal_subscriber]: I heard: "Hello World: 5"
   [minimal_subscriber]: I heard: "Hello World: 6"
   [minimal_subscriber]: I heard: "Hello World: 7"
   ...
   ```

**Notice**: The subscriber won't see message 0-4 if it starts late—that's because topics only keep the most recent message (in this configuration).

---

## Running Publisher and Subscriber Together

Now let's see the pub-sub pattern in action:

### Setup (3 Terminals)

**Terminal 1 - Publisher:**
```bash
cd ~/ros2_ws/src/examples/minimal_publisher
source /opt/ros/humble/setup.bash
python minimal_publisher.py
```
Output:
```
[minimal_publisher]: Publishing: "Hello World: 0"
[minimal_publisher]: Publishing: "Hello World: 1"
...
```

**Terminal 2 - Subscriber:**
```bash
cd ~/ros2_ws/src/examples/minimal_subscriber
source /opt/ros/humble/setup.bash
python minimal_subscriber.py
```
Output:
```
[minimal_subscriber]: I heard: "Hello World: 5"
[minimal_subscriber]: I heard: "Hello World: 6"
...
```

**Terminal 3 - Monitoring (Optional):**
```bash
source /opt/ros/humble/setup.bash
ros2 node list           # See all running nodes
ros2 topic list          # See all active topics
ros2 topic echo /chatter # See all messages on a topic
```

### What's Happening

1. **Publisher** publishes to `/chatter` every 0.5 seconds
2. **Subscriber** listens to `/chatter` and prints each message
3. **ROS 2 Middleware** handles routing messages from publisher to subscriber
4. Both nodes run **independently**—stop either one and the other keeps running

---

## Debugging Common Issues

### Issue 1: Subscriber Doesn't Receive Messages

**Symptom**: Subscriber node runs but no output

**Cause**: Subscriber started after publisher; topics only store the latest message

**Solution**:
```bash
# Start subscriber BEFORE publisher
# OR use ros2 topic echo to verify messages exist
ros2 topic echo /chatter
```

### Issue 2: "Node not found" or "Topic not found"

**Symptom**: Error when trying to run the node

**Cause**: Python can't find ROS 2 libraries

**Solution**:
```bash
# Make sure you sourced ROS 2
source /opt/ros/humble/setup.bash

# Verify ROS 2 is installed
ros2 --version

# Check Python version (must be 3.10+)
python3 --version
```

### Issue 3: ROS_DOMAIN_ID Conflicts

**Symptom**: Multiple subscriber instances on same topic; nodes don't see each other

**Cause**: Different Domain IDs isolate ROS 2 networks

**Solution**:
```bash
# All nodes must use the same Domain ID
export ROS_DOMAIN_ID=0
python minimal_publisher.py

# In another terminal, same Domain ID:
export ROS_DOMAIN_ID=0
python minimal_subscriber.py
```

### Issue 4: "ModuleNotFoundError: No module named 'rclpy'"

**Symptom**: Python can't import rclpy

**Cause**: ROS 2 packages not properly sourced

**Solution**:
```bash
# Ensure ROS 2 environment is set up
source /opt/ros/humble/setup.bash

# Verify rclpy is installed
python3 -c "import rclpy; print(rclpy.__version__)"
```

---

## Modification Exercises

Now that you understand the basics, let's experiment:

### Exercise 1: Change the Message Content

**Objective**: Modify the publisher to send different information

**Current Code**:
```python
msg.data = f'Hello World: {self.i}'
```

**Try This**:
```python
# Send the current time
import time
msg.data = f'Time: {time.time()}'

# Send a countdown
msg.data = f'Count down: {100 - self.i}'

# Send the message alternating between two strings
if self.i % 2 == 0:
    msg.data = "ON"
else:
    msg.data = "OFF"
```

**Verification**: Run the modified publisher and see different messages appear

---

### Exercise 2: Change the Publishing Rate

**Objective**: Publish faster or slower

**Current Code**:
```python
timer_period = 0.5  # 0.5 seconds = 2 messages per second
```

**Try This**:
```python
# Publish faster (every 0.1 seconds = 10 messages/sec)
timer_period = 0.1

# Publish slower (every 2 seconds)
timer_period = 2.0

# Publish every 5 seconds
timer_period = 5.0
```

**Verification**: Compare output speed between different rates

---

### Exercise 3: Process Sensor Data in Subscriber

**Objective**: Do something more complex in the callback

**Current Code**:
```python
def listener_callback(self, msg):
    self.get_logger().info(f'I heard: "{msg.data}"')
```

**Try This**:
```python
# Count received messages
def __init__(self):
    super().__init__('minimal_subscriber')
    self.subscription = self.create_subscription(
        String, 'chatter', self.listener_callback, 10)
    self.message_count = 0

def listener_callback(self, msg):
    self.message_count += 1
    self.get_logger().info(
        f'Message #{self.message_count}: "{msg.data}"'
    )

    # Alert after every 10th message
    if self.message_count % 10 == 0:
        self.get_logger().warn(
            f'Received {self.message_count} messages!'
        )
```

**Verification**: Run the modified subscriber and see the count increment

---

### Exercise 4: Create a Custom Response Publisher

**Objective**: Subscriber responds to messages by publishing its own messages

**Challenge**: Modify the subscriber to:
1. Subscribe to `/chatter`
2. Create a publisher to `/response`
3. When it receives a message, publish a response

**Hint**:
```python
class ResponsiveSubscriber(Node):
    def __init__(self):
        super().__init__('responsive_subscriber')

        # Subscribe to chatter
        self.subscription = self.create_subscription(
            String, 'chatter', self.listener_callback, 10)

        # Also publish responses
        self.publisher_ = self.create_publisher(String, 'response', 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

        # Send a response
        response = String()
        response.data = f'Echo: {msg.data}'
        self.publisher_.publish(response)
```

**Verification**: Run this responsive subscriber alongside the publisher, then create another subscriber to `/response` to see the echo messages

---

## Summary

You've learned:

- **Node Structure**: All ROS 2 nodes inherit from `Node`
- **Publishers**: Use `create_publisher()` with a timer for periodic messages
- **Subscribers**: Use `create_subscription()` with a callback for event-driven processing
- **The Pub-Sub Pattern**: Decoupled communication where publishers and subscribers don't need to know about each other
- **Debugging**: Tools like `ros2 topic list` and `ros2 topic echo` to inspect the system
- **Experimentation**: Modifying code to send different data, change rates, and add processing logic

---

## Next Steps

Now that you can write basic publishers and subscribers, you're ready to:

1. **Learn about other message types** (sensor_msgs, geometry_msgs) for real sensor data
2. **Explore services** for request-response communication
3. **Build more complex node architectures** with multiple publishers/subscribers
4. **Work with real robots** and connect to actual sensors and motors

---

**Ready?** Move to the next chapter to learn about **URDF: Describing Your Robot's Structure**.
