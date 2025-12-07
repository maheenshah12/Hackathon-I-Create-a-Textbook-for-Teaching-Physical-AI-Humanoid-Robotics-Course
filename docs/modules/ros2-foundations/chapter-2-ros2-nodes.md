---
sidebar_position: 3
---

# Chapter 2: ROS 2 Nodes

## Objective

By the end of this chapter, you will understand what ROS 2 nodes are, how to create them, and how they communicate with each other using topics, services, and parameters.

## What is a Node?

A node is a fundamental building block of a ROS 2 system. It's an executable that uses ROS 2 to communicate with other nodes. Nodes can publish or subscribe to topics, provide or use services, and manage parameters. A single executable can contain multiple nodes, and multiple executables can run the same node.

## Node Architecture

Nodes in ROS 2 follow a client library architecture. The most common client libraries are:

- **rclcpp**: C++ client library
- **rclpy**: Python client library

Each node has:

- A unique name within the ROS graph
- One or more publishers, subscribers, services, or actions
- Access to parameters
- A node handle or node interface

## Creating a Node in Python

Let's create a simple ROS 2 node using Python. First, we'll need to create a package:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_robot_tutorials
```

Now create the Python file for our node:

```python
# my_robot_tutorials/my_robot_tutorials/simple_node.py
import rclpy
from rclpy.node import Node

class SimpleNode(Node):
    def __init__(self):
        super().__init__('simple_node')
        self.get_logger().info('Hello from simple node!')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating a Publisher Node

A publisher node sends messages to a topic. Here's an example:

```python
# my_robot_tutorials/my_robot_tutorials/publisher_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating a Subscriber Node

A subscriber node receives messages from a topic. Here's an example:

```python
# my_robot_tutorials/my_robot_tutorials/subscriber_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
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
    node = SubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Node Parameters

Nodes can have parameters that can be configured at runtime:

```python
# my_robot_tutorials/my_robot_tutorials/parameter_node.py
import rclpy
from rclpy.node import Node

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare a parameter
        self.declare_parameter('my_parameter', 'default_value')

        # Get the parameter value
        param_value = self.get_parameter('my_parameter').value
        self.get_logger().info(f'Parameter value: {param_value}')

def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Running Nodes

To run your nodes:

1. Build your workspace:
```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_tutorials
source install/setup.bash
```

2. Run the publisher:
```bash
ros2 run my_robot_tutorials publisher_node
```

3. In another terminal, run the subscriber:
```bash
ros2 run my_robot_tutorials subscriber_node
```

## Node Lifecycle

ROS 2 nodes have a lifecycle that includes:

- **Unconfigured**: Initial state
- **Inactive**: Configured but not active
- **Active**: Running and processing
- **Finalized**: Cleaned up and ready for destruction

## Summary

In this chapter, you've learned how to create ROS 2 nodes in Python, how to implement publishers and subscribers, and how to work with parameters. You now understand the node lifecycle and how to run nodes in a ROS 2 system.

## Common Errors and Troubleshooting

- **Node name conflicts**: Use unique node names to avoid conflicts
- **Import errors**: Make sure your package is properly built and sourced
- **Topic connection issues**: Check that topic names match exactly between publishers and subscribers
- **Node not responding**: Check that rclpy.spin() is called to process callbacks

## References

- [ROS 2 Node Documentation](https://docs.ros.org/en/humble/Concepts/About-ROS-2-Client-Libraries.html)
- [Python Node Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)