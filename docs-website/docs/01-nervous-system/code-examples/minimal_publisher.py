#!/usr/bin/env python3
"""
Minimal ROS 2 Publisher Example

This node demonstrates the basic publisher pattern in ROS 2 using rclpy.
It publishes a simple string message to the 'chatter' topic every second.

Usage:
    python minimal_publisher.py

The node will:
1. Initialize the ROS 2 system
2. Create a publisher on the 'chatter' topic
3. Publish "Hello World: N" messages every 1 second
4. Continue until interrupted (Ctrl+C)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    """
    A ROS 2 node that publishes messages to the 'chatter' topic.

    This node demonstrates:
    - Creating a ROS 2 node by inheriting from Node
    - Creating a publisher that sends messages
    - Using a timer to publish messages at regular intervals
    """

    def __init__(self):
        """
        Initialize the MinimalPublisher node.

        This constructor:
        1. Calls the parent Node.__init__() with the node name
        2. Creates a publisher for String messages on 'chatter' topic
        3. Sets up a timer that calls publish_message() every 0.5 seconds
        """
        # Call the parent Node class constructor with the node name
        super().__init__('minimal_publisher')

        # Create a publisher that sends String messages to the 'chatter' topic
        # queue_size=10 means the publisher will queue up to 10 messages if no subscribers
        self.publisher_ = self.create_publisher(String, 'chatter', 10)

        # Create a timer that calls publish_message() every 0.5 seconds
        # This ensures messages are published at regular intervals
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_message)

        # Counter for message sequence
        self.i = 0

    def publish_message(self):
        """
        Timer callback function that publishes a message.

        This method is called automatically by the timer every 0.5 seconds.
        It creates a message, publishes it, and logs the action.
        """
        # Create a new String message object
        msg = String()

        # Set the message data to a string with the current counter
        msg.data = f'Hello World: {self.i}'

        # Publish the message to subscribers listening on the 'chatter' topic
        self.publisher_.publish(msg)

        # Log the published message to the console (for debugging)
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment the counter for the next message
        self.i += 1


def main(args=None):
    """
    Main function that starts the ROS 2 node.

    This function:
    1. Initializes the ROS 2 system
    2. Creates an instance of MinimalPublisher
    3. Runs the node until Ctrl+C is pressed
    4. Cleans up when the node shuts down

    Args:
        args: Command-line arguments (typically None in simple cases)
    """
    # Initialize the ROS 2 system
    # This must be called before any ROS 2 operations
    rclpy.init(args=args)

    # Create an instance of our MinimalPublisher node
    minimal_publisher = MinimalPublisher()

    try:
        # Start spinning (running) the node
        # spin() blocks until the node is interrupted (Ctrl+C)
        # It processes callbacks and keeps the node alive
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        # Handle user interruption (Ctrl+C)
        print("\nShutting down publisher node...")
    finally:
        # Clean up and shut down the ROS 2 system
        minimal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
