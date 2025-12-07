---
sidebar_position: 5
---

# Basic ROS 2 Tutorial Steps

## Objective

Follow these step-by-step instructions to run your first ROS 2 publisher-subscriber example and understand the basic concepts.

## Prerequisites

Before starting this tutorial, ensure you have:

1. ROS 2 Humble installed on Ubuntu 22.04
2. Basic understanding of Linux command line
3. Python 3.8 or higher

## Step 1: Create a ROS 2 Workspace

First, create a workspace directory for your ROS 2 packages:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

## Step 2: Create a Package

Create a new package for your tutorials:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_robot_tutorials
```

## Step 3: Create the Publisher-Subscriber Code

Create the Python file with the publisher-subscriber code. Create the file:

```bash
mkdir -p ~/ros2_ws/src/my_robot_tutorials/my_robot_tutorials
```

Then copy the following code into `~/ros2_ws/src/my_robot_tutorials/my_robot_tutorials/simple_publisher_subscriber.py`:

```python
#!/usr/bin/env python3
# Simple Publisher-Subscriber Example for ROS 2 Humble

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)

    # Create both publisher and subscriber nodes
    publisher = SimplePublisher()
    subscriber = SimpleSubscriber()

    # Run both nodes in the same process
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass

    # Cleanup
    publisher.destroy_node()
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Step 4: Update the Package Configuration

Update the `setup.py` file in your package to include the executable. Edit `~/ros2_ws/src/my_robot_tutorials/setup.py` and add to the `entry_points` section:

```python
'console_scripts': [
    'simple_publisher_subscriber = my_robot_tutorials.simple_publisher_subscriber:main',
],
```

## Step 5: Build the Workspace

Build your ROS 2 workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_tutorials
```

## Step 6: Source the Workspace

Source the built workspace:

```bash
source ~/ros2_ws/install/setup.bash
```

## Step 7: Run the Publisher-Subscriber Example

Run the publisher-subscriber example:

```bash
ros2 run my_robot_tutorials simple_publisher_subscriber
```

You should see output like:

```
[INFO] [1620000000.000000000] [simple_publisher]: Publishing: "Hello World: 0"
[INFO] [1620000000.500000000] [simple_subscriber]: I heard: "Hello World: 0"
[INFO] [1620000001.000000000] [simple_publisher]: Publishing: "Hello World: 1"
[INFO] [1620000001.500000000] [simple_subscriber]: I heard: "Hello World: 1"
```

## Step 8: Understanding the Output

- The publisher node sends messages to the 'topic' at 0.5-second intervals
- The subscriber node receives and displays these messages
- This demonstrates the publisher-subscriber communication pattern in ROS 2

## Expected Outcome

By completing this tutorial, you should have:

1. Created a ROS 2 workspace and package
2. Written a simple publisher-subscriber program
3. Built and run the program successfully
4. Observed the communication between nodes

## Next Steps

- Experiment with different message types
- Create separate publisher and subscriber nodes
- Try creating a service client and server

## Summary

This tutorial introduced you to the basic concepts of ROS 2 nodes, topics, and message passing. You learned how to create a simple publisher-subscriber pair and run them in a ROS 2 environment.