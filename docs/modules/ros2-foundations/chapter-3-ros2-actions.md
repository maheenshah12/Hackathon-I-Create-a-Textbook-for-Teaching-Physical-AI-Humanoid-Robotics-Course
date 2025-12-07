---
sidebar_position: 4
---

# Chapter 3: ROS 2 Actions

## Objective

By the end of this chapter, you will understand what ROS 2 actions are, how they differ from topics and services, and how to implement them in your robotic applications.

## What are Actions?

Actions are a communication pattern in ROS 2 that is designed for long-running tasks. They combine the features of topics (feedback) and services (goals and results) to provide a robust way to handle tasks that take time to complete and may be preempted or provide ongoing feedback.

Actions are particularly useful for:
- Navigation tasks (moving a robot to a specific location)
- Manipulation tasks (grasping an object)
- Calibration procedures
- Any long-running task that requires feedback

## Action Architecture

An action consists of three parts:

1. **Goal**: The request to start the action
2. **Feedback**: Messages sent during the action execution
3. **Result**: The final outcome of the action

## Action States

Actions go through various states during their lifecycle:
- **PENDING**: The goal has been accepted but not yet started
- **ACTIVE**: The goal is being processed
- **PREEMPTING**: The goal is being canceled but is still active
- **SUCCEEDED**: The goal completed successfully
- **ABORTED**: The goal failed during execution
- **CANCELED**: The goal was canceled after completion

## Creating an Action Server in Python

First, let's define the action file. Create `Fibonacci.action` in your package:

```
#goal definition
int32 order

#result definition
int32[] sequence

#feedback
int32[] sequence
```

Now let's create an action server:

```python
# my_robot_tutorials/my_robot_tutorials/action_server.py
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from my_robot_tutorials.action import Fibonacci


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])

            self.get_logger().info(f'Feedback: {feedback_msg.sequence}')
            goal_handle.publish_feedback(feedback_msg)

            time.sleep(1)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info(f'Result: {result.sequence}')
        return result


def main(args=None):
    rclpy.init(args=args)
    fibonacci_action_server = FibonacciActionServer()
    rclpy.spin(fibonacci_action_server)
    fibonacci_action_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Creating an Action Client in Python

Now let's create a client that sends goals to our action server:

```python
# my_robot_tutorials/my_robot_tutorials/action_client.py
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from my_robot_tutorials.action import Fibonacci


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.sequence}')


def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()
    time.sleep(2)  # Give the client time to connect
    action_client.send_goal(10)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
```

## Running Actions

To run your action server and client:

1. Make sure your action definition is properly configured in your package's CMakeLists.txt or setup.py
2. Build your workspace:
```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_tutorials
source install/setup.bash
```

3. Run the action server:
```bash
ros2 run my_robot_tutorials action_server
```

4. In another terminal, run the action client:
```bash
ros2 run my_robot_tutorials action_client
```

## When to Use Actions vs Topics vs Services

- **Use Topics** when you need continuous, asynchronous communication (sensors, robot state)
- **Use Services** when you need synchronous request-response communication for short tasks
- **Use Actions** when you need to handle long-running tasks that may provide feedback or be preempted

## Summary

In this chapter, you've learned about ROS 2 actions, which are designed for long-running tasks. You now understand the difference between actions, topics, and services, and how to implement both action servers and clients. You've seen how actions can provide feedback during execution and handle goal preemption.

## Common Errors and Troubleshooting

- **Action server not found**: Ensure the action server is running before starting the client
- **Action definition mismatch**: Make sure the action definition matches between client and server
- **Goal not accepted**: Check that your action server is properly initialized and connected
- **Feedback not received**: Verify that the feedback callback is properly defined and called

## References

- [ROS 2 Actions Documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Actions-In-Python.html)
- [Action Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)