#!/usr/bin/env python3
"""
Minimal ROS 2 Subscriber Example

This node demonstrates the basic subscriber pattern in ROS 2 using rclpy.
It subscribes to the 'chatter' topic and prints each received message.

Usage:
    python minimal_subscriber.py

The node will:
1. Initialize the ROS 2 system
2. Create a subscription to the 'chatter' topic
3. Wait for and process incoming messages
4. Print each message to the console

To see both publisher and subscriber in action:
    Terminal 1: python minimal_publisher.py
    Terminal 2: python minimal_subscriber.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):
    """
    A ROS 2 node that subscribes to messages from the 'chatter' topic.

    This node demonstrates:
    - Creating a ROS 2 node by inheriting from Node
    - Subscribing to a topic to receive messages
    - Processing messages through a callback function
    """

    def __init__(self):
        """
        Initialize the MinimalSubscriber node.

        This constructor:
        1. Calls the parent Node.__init__() with the node name
        2. Creates a subscription to the 'chatter' topic
        3. Sets up a callback function to handle received messages
        """
        # Call the parent Node class constructor with the node name
        super().__init__('minimal_subscriber')

        # Create a subscription to String messages on the 'chatter' topic
        # When a message arrives, the listener_callback() function is called
        # queue_size=10 means the node will queue up to 10 messages if the callback
        # is slow to process them
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)

        # Prevent unused variable warning (the subscription is used implicitly)
        self.subscription  # noqa: F841

    def listener_callback(self, msg):
        """
        Callback function that processes received messages.

        This method is called automatically whenever a new message arrives
        on the 'chatter' topic. It receives the message and can perform
        any processing needed.

        Args:
            msg: The received message object (type: String)
                 Contains one field: msg.data (the string content)
        """
        # Extract the string data from the message
        message_text = msg.data

        # Log the received message to the console
        self.get_logger().info(f'I heard: "{message_text}"')

        # Note: In this simple example, we just print the message.
        # In more complex applications, you might:
        # - Process the data (e.g., calculate statistics)
        # - Publish responses to other topics
        # - Update internal state
        # - Trigger other actions


def main(args=None):
    """
    Main function that starts the ROS 2 subscriber node.

    This function:
    1. Initializes the ROS 2 system
    2. Creates an instance of MinimalSubscriber
    3. Runs the node until Ctrl+C is pressed
    4. Cleans up when the node shuts down

    Args:
        args: Command-line arguments (typically None in simple cases)
    """
    # Initialize the ROS 2 system
    # This must be called before any ROS 2 operations
    rclpy.init(args=args)

    # Create an instance of our MinimalSubscriber node
    minimal_subscriber = MinimalSubscriber()

    try:
        # Start spinning (running) the node
        # spin() blocks until the node is interrupted (Ctrl+C)
        # It continuously listens for messages and calls the callback when they arrive
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        # Handle user interruption (Ctrl+C)
        print("\nShutting down subscriber node...")
    finally:
        # Clean up and shut down the ROS 2 system
        minimal_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
