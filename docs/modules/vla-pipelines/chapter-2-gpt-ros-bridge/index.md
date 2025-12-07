---
sidebar_position: 2
---

# Chapter 2: GPT-ROS Bridge

## Overview

This chapter covers the integration of OpenAI's GPT models with ROS 2 for advanced natural language processing in robotic systems. You'll learn how to create a bridge between GPT's language understanding capabilities and ROS 2's robotic control systems, enabling complex voice-to-action translation.

## Learning Objectives

By the end of this chapter, you will be able to:
- Set up GPT integration with ROS 2 for natural language processing
- Create a GPT-ROS bridge for translating natural language to robotic actions
- Implement context-aware command interpretation
- Handle complex multi-step instructions with GPT
- Integrate GPT with existing robotic perception and manipulation systems

## Introduction to GPT for Robotics

GPT (Generative Pre-trained Transformer) models provide advanced natural language understanding and generation capabilities that are highly valuable for robotics applications. In robotics, GPT can interpret complex, natural language commands and translate them into specific robotic actions, making human-robot interaction more intuitive.

### Key Benefits for Robotics:
- Natural language command interpretation
- Context-aware instruction processing
- Complex task decomposition
- Multi-modal command handling
- Conversational interfaces for robots

## Setting Up GPT with ROS 2

### Prerequisites

Before implementing GPT integration, ensure you have:
- ROS 2 Humble installed
- Python 3.8 or higher
- OpenAI API key (or local LLM setup)
- OpenAI Python library installed

### Installation

First, install the required dependencies:

```bash
pip install openai
pip install transformers  # For local models
pip install torch  # For local models
```

### Basic GPT-ROS Bridge Implementation

Here's a basic implementation of a GPT-ROS bridge that processes natural language commands:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from openai import OpenAI
import json
import os

class GPTBridgeNode(Node):
    def __init__(self):
        super().__init__('gpt_bridge_node')

        # Initialize OpenAI client
        # You can use environment variable for API key
        api_key = os.getenv('OPENAI_API_KEY')
        if not api_key:
            self.get_logger().error('OPENAI_API_KEY environment variable not set')
            raise ValueError('OpenAI API key not found')

        self.client = OpenAI(api_key=api_key)

        # Create subscriber for natural language commands
        self.command_subscriber = self.create_subscription(
            String,
            'natural_language_commands',
            self.command_callback,
            10
        )

        # Create publisher for robot movement
        self.movement_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create publisher for action responses
        self.response_publisher = self.create_publisher(String, 'gpt_responses', 10)

        # Define system prompt for robot behavior
        self.system_prompt = """
        You are a robot command interpreter. Your role is to convert natural language commands into specific robot actions.
        Respond with a JSON object containing the action type and parameters.

        Action types:
        - move_linear: {direction: 'forward/backward', distance: float in meters}
        - move_angular: {direction: 'left/right', angle: float in radians}
        - stop: {}
        - speak: {text: string}

        Example:
        Input: "Go forward 2 meters"
        Output: {"action": "move_linear", "direction": "forward", "distance": 2.0}
        """

        self.get_logger().info('GPT bridge node initialized')

    def command_callback(self, msg):
        """Process incoming natural language commands"""
        command = msg.data
        self.get_logger().info(f'Received natural language command: {command}')

        try:
            # Call GPT to interpret the command
            response = self.client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": command}
                ],
                temperature=0.1,  # Low temperature for more deterministic output
                max_tokens=200
            )

            # Extract the response
            gpt_response = response.choices[0].message.content.strip()

            # Try to parse as JSON
            try:
                action_data = json.loads(gpt_response)
                self.execute_action(action_data)
            except json.JSONDecodeError:
                # If not JSON, try to extract JSON from the response
                import re
                json_match = re.search(r'\{.*\}', gpt_response, re.DOTALL)
                if json_match:
                    action_data = json.loads(json_match.group())
                    self.execute_action(action_data)
                else:
                    self.get_logger().error(f'Could not parse GPT response as JSON: {gpt_response}')
                    return

        except Exception as e:
            self.get_logger().error(f'Error processing command with GPT: {e}')

    def execute_action(self, action_data):
        """Execute the action specified by GPT"""
        action_type = action_data.get('action')

        if action_type == 'move_linear':
            direction = action_data.get('direction', 'forward')
            distance = action_data.get('distance', 1.0)

            # Create movement command based on direction and distance
            twist = Twist()
            if direction == 'forward':
                twist.linear.x = 0.5  # 0.5 m/s
            else:
                twist.linear.x = -0.5  # -0.5 m/s

            # Publish command for duration based on distance
            duration = abs(distance) / 0.5  # Assuming 0.5 m/s speed
            self.execute_timed_movement(twist, duration)

            self.get_logger().info(f'Moving {direction} {distance} meters')

        elif action_type == 'move_angular':
            direction = action_data.get('direction', 'left')
            angle = action_data.get('angle', 1.57)  # 90 degrees in radians

            # Create rotation command
            twist = Twist()
            if direction == 'left':
                twist.angular.z = 0.5  # 0.5 rad/s
            else:
                twist.angular.z = -0.5  # -0.5 rad/s

            # Publish command for duration based on angle
            duration = abs(angle) / 0.5  # Assuming 0.5 rad/s speed
            self.execute_timed_movement(twist, duration)

            self.get_logger().info(f'Turning {direction} {angle} radians')

        elif action_type == 'stop':
            # Stop the robot
            twist = Twist()
            self.movement_publisher.publish(twist)
            self.get_logger().info('Stopping robot')

        elif action_type == 'speak':
            text = action_data.get('text', '')
            response_msg = String()
            response_msg.data = text
            self.response_publisher.publish(response_msg)
            self.get_logger().info(f'Speaking: {text}')

        else:
            self.get_logger().warning(f'Unknown action type: {action_type}')

    def execute_timed_movement(self, twist, duration):
        """Execute movement for a specified duration"""
        import time

        # Publish the movement command
        start_time = time.time()
        while time.time() - start_time < duration and rclpy.ok():
            self.movement_publisher.publish(twist)
            time.sleep(0.1)  # 10Hz update rate

        # Stop the robot after movement
        stop_twist = Twist()
        self.movement_publisher.publish(stop_twist)

def main(args=None):
    rclpy.init(args=args)
    node = GPTBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Context-Aware Command Processing

For more sophisticated command processing, you can maintain context and state information:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from openai import OpenAI
import json
import os
import threading
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class ContextAwareGPTNode(Node):
    def __init__(self):
        super().__init__('context_aware_gpt_node')

        # Initialize OpenAI client
        api_key = os.getenv('OPENAI_API_KEY')
        if not api_key:
            self.get_logger().error('OPENAI_API_KEY environment variable not set')
            raise ValueError('OpenAI API key not found')

        self.client = OpenAI(api_key=api_key)

        # Create subscriber for natural language commands
        self.command_subscriber = self.create_subscription(
            String,
            'natural_language_commands',
            self.command_callback,
            10
        )

        # Create publisher for robot movement
        self.movement_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create subscriber for robot state (odometry, laser scan, etc.)
        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        self.laser_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10
        )

        # Robot state
        self.robot_pose = None
        self.laser_data = None
        self.state_lock = threading.Lock()

        # Define system prompt with context awareness
        self.system_prompt = """
        You are a context-aware robot command interpreter. Your role is to convert natural language commands into specific robot actions.
        Consider the robot's current state, position, and environment when interpreting commands.
        Respond with a JSON object containing the action type and parameters.

        Action types:
        - move_linear: {direction: 'forward/backward', distance: float in meters}
        - move_angular: {direction: 'left/right', angle: float in radians}
        - stop: {}
        - navigate_to: {x: float, y: float, frame: string}
        - check_obstacle: {direction: 'forward/backward/left/right'}
        - speak: {text: string}

        Current robot state will be provided. Use this context to interpret commands appropriately.
        """

        self.get_logger().info('Context-aware GPT node initialized')

    def odom_callback(self, msg):
        """Update robot pose from odometry"""
        with self.state_lock:
            self.robot_pose = {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'theta': self.quaternion_to_yaw(msg.pose.pose.orientation)
            }

    def laser_callback(self, msg):
        """Update laser scan data"""
        with self.state_lock:
            self.laser_data = {
                'ranges': list(msg.ranges),
                'angle_min': msg.angle_min,
                'angle_max': msg.angle_max,
                'angle_increment': msg.angle_increment
            }

    def quaternion_to_yaw(self, orientation):
        """Convert quaternion to yaw angle"""
        import math
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def command_callback(self, msg):
        """Process incoming natural language commands with context"""
        command = msg.data
        self.get_logger().info(f'Received natural language command: {command}')

        try:
            # Get current robot state
            with self.state_lock:
                robot_state = {
                    'pose': self.robot_pose,
                    'laser_data': self.laser_data
                }

            # Create context-aware prompt
            context_prompt = f"""
            Current robot state:
            - Position: {robot_state['pose']}
            - Laser scan: {len(robot_state['laser_data']['ranges']) if robot_state['laser_data'] else 0} range readings

            Command to interpret: {command}
            """

            # Call GPT to interpret the command with context
            response = self.client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": context_prompt}
                ],
                temperature=0.1,
                max_tokens=300
            )

            # Extract the response
            gpt_response = response.choices[0].message.content.strip()

            # Parse and execute action
            try:
                action_data = json.loads(gpt_response)
                self.execute_action(action_data)
            except json.JSONDecodeError:
                import re
                json_match = re.search(r'\{.*\}', gpt_response, re.DOTALL)
                if json_match:
                    action_data = json.loads(json_match.group())
                    self.execute_action(action_data)
                else:
                    self.get_logger().error(f'Could not parse GPT response as JSON: {gpt_response}')

        except Exception as e:
            self.get_logger().error(f'Error processing command with GPT: {e}')

    def execute_action(self, action_data):
        """Execute the action specified by GPT"""
        action_type = action_data.get('action')

        if action_type == 'move_linear':
            direction = action_data.get('direction', 'forward')
            distance = action_data.get('distance', 1.0)

            twist = Twist()
            if direction == 'forward':
                twist.linear.x = 0.5
            else:
                twist.linear.x = -0.5

            duration = abs(distance) / 0.5
            self.execute_timed_movement(twist, duration)

            self.get_logger().info(f'Moving {direction} {distance} meters')

        elif action_type == 'navigate_to':
            x = action_data.get('x', 0.0)
            y = action_data.get('y', 0.0)

            # This would typically call a navigation action server
            self.get_logger().info(f'Navigating to position ({x}, {y})')
            # TODO: Implement navigation to goal

        elif action_type == 'check_obstacle':
            direction = action_data.get('direction', 'forward')
            self.check_obstacle(direction)

        elif action_type == 'stop':
            twist = Twist()
            self.movement_publisher.publish(twist)
            self.get_logger().info('Stopping robot')

        elif action_type == 'speak':
            text = action_data.get('text', '')
            response_msg = String()
            response_msg.data = text
            self.response_publisher.publish(response_msg)
            self.get_logger().info(f'Speaking: {text}')

        else:
            self.get_logger().warning(f'Unknown action type: {action_type}')

    def check_obstacle(self, direction):
        """Check for obstacles in the specified direction"""
        if not self.laser_data:
            self.get_logger().warning('No laser data available for obstacle detection')
            return

        ranges = self.laser_data['ranges']
        angle_min = self.laser_data['angle_min']
        angle_increment = self.laser_data['angle_increment']

        # Define angle ranges for different directions
        if direction == 'forward':
            # Front: -30 to 30 degrees
            start_idx = int((angle_min + 30 * 3.14159 / 180) / angle_increment)
            end_idx = int((angle_min + 150 * 3.14159 / 180) / angle_increment)
        elif direction == 'backward':
            # Back: 150 to 210 degrees
            start_idx = int((angle_min + 150 * 3.14159 / 180) / angle_increment)
            end_idx = int((angle_min + 210 * 3.14159 / 180) / angle_increment)
        elif direction == 'left':
            # Left: 60 to 120 degrees
            start_idx = int((angle_min + 60 * 3.14159 / 180) / angle_increment)
            end_idx = int((angle_min + 120 * 3.14159 / 180) / angle_increment)
        elif direction == 'right':
            # Right: -120 to -60 degrees
            start_idx = int((angle_min + 240 * 3.14159 / 180) / angle_increment)
            end_idx = int((angle_min + 300 * 3.14159 / 180) / angle_increment)
        else:
            self.get_logger().warning(f'Unknown direction: {direction}')
            return

        # Check for obstacles in the specified range
        if start_idx >= 0 and end_idx < len(ranges):
            min_distance = min(ranges[start_idx:end_idx])
            if min_distance < 1.0:  # Obstacle within 1 meter
                self.get_logger().info(f'Obstacle detected {min_distance:.2f}m in {direction} direction')
            else:
                self.get_logger().info(f'No obstacles detected in {direction} direction')
        else:
            self.get_logger().warning('Invalid angle range for obstacle detection')

    def execute_timed_movement(self, twist, duration):
        """Execute movement for a specified duration"""
        import time

        start_time = time.time()
        while time.time() - start_time < duration and rclpy.ok():
            self.movement_publisher.publish(twist)
            time.sleep(0.1)

        stop_twist = Twist()
        self.movement_publisher.publish(stop_twist)

def main(args=None):
    rclpy.init(args=args)
    node = ContextAwareGPTNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Multi-Step Instruction Processing

For handling complex, multi-step instructions, you can implement a task planner:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from openai import OpenAI
import json
import os
import threading
import time

class GPTTaskPlannerNode(Node):
    def __init__(self):
        super().__init__('gpt_task_planner_node')

        # Initialize OpenAI client
        api_key = os.getenv('OPENAI_API_KEY')
        if not api_key:
            self.get_logger().error('OPENAI_API_KEY environment variable not set')
            raise ValueError('OpenAI API key not found')

        self.client = OpenAI(api_key=api_key)

        # Create subscriber for natural language commands
        self.command_subscriber = self.create_subscription(
            String,
            'natural_language_commands',
            self.command_callback,
            10
        )

        # Create publisher for robot movement
        self.movement_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create publisher for task status
        self.status_publisher = self.create_publisher(String, 'task_status', 10)

        # Task execution state
        self.is_executing_task = False
        self.task_queue = []
        self.task_lock = threading.Lock()

        # Define system prompt for task planning
        self.system_prompt = """
        You are a robot task planner. Your role is to break down complex natural language commands into a sequence of simple actions.
        Respond with a JSON array containing the sequence of actions to execute.

        Action types:
        - move_linear: {direction: 'forward/backward', distance: float in meters}
        - move_angular: {direction: 'left/right', angle: float in radians}
        - wait: {duration: float in seconds}
        - check_condition: {condition: string}
        - speak: {text: string}

        Example:
        Input: "Go forward 2 meters, turn right, and go forward 1 meter"
        Output: [
          {"action": "move_linear", "direction": "forward", "distance": 2.0},
          {"action": "move_angular", "direction": "right", "angle": 1.57},
          {"action": "move_linear", "direction": "forward", "distance": 1.0}
        ]
        """

        self.get_logger().info('GPT task planner node initialized')

    def command_callback(self, msg):
        """Process incoming natural language commands"""
        command = msg.data
        self.get_logger().info(f'Received complex command: {command}')

        # Only process if not already executing a task
        with self.task_lock:
            if self.is_executing_task:
                self.get_logger().warning('Already executing a task, queuing new command')
                self.task_queue.append(command)
                return

        try:
            # Call GPT to plan the task
            response = self.client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": command}
                ],
                temperature=0.1,
                max_tokens=500
            )

            # Extract the response
            gpt_response = response.choices[0].message.content.strip()

            # Parse the task sequence
            try:
                task_sequence = json.loads(gpt_response)

                # Start executing the task sequence
                self.execute_task_sequence(task_sequence)

            except json.JSONDecodeError:
                import re
                json_match = re.search(r'\[.*\]', gpt_response, re.DOTALL)
                if json_match:
                    task_sequence = json.loads(json_match.group())
                    self.execute_task_sequence(task_sequence)
                else:
                    self.get_logger().error(f'Could not parse GPT response as JSON array: {gpt_response}')

        except Exception as e:
            self.get_logger().error(f'Error planning task with GPT: {e}')

    def execute_task_sequence(self, task_sequence):
        """Execute a sequence of tasks"""
        with self.task_lock:
            self.is_executing_task = True

        self.get_logger().info(f'Executing task sequence with {len(task_sequence)} steps')

        for i, task in enumerate(task_sequence):
            if not rclpy.ok():
                break

            self.get_logger().info(f'Executing task {i+1}/{len(task_sequence)}: {task["action"]}')

            # Publish task status
            status_msg = String()
            status_msg.data = f'Executing task {i+1}/{len(task_sequence)}: {task["action"]}'
            self.status_publisher.publish(status_msg)

            # Execute the task
            self.execute_single_task(task)

        with self.task_lock:
            self.is_executing_task = False
            self.get_logger().info('Task sequence completed')

            # Process any queued tasks
            if self.task_queue:
                next_command = self.task_queue.pop(0)
                # Schedule the next command to be processed
                self.get_logger().info('Processing queued command')
                # Note: In a real implementation, you'd want to schedule this properly
                # For now, we'll just log it - you'd typically use a timer or similar mechanism

    def execute_single_task(self, task):
        """Execute a single task"""
        action_type = task.get('action')

        if action_type == 'move_linear':
            direction = task.get('direction', 'forward')
            distance = task.get('distance', 1.0)

            twist = Twist()
            if direction == 'forward':
                twist.linear.x = 0.5
            else:
                twist.linear.x = -0.5

            duration = abs(distance) / 0.5
            self.execute_timed_movement(twist, duration)

        elif action_type == 'move_angular':
            direction = task.get('direction', 'left')
            angle = task.get('angle', 1.57)

            twist = Twist()
            if direction == 'left':
                twist.angular.z = 0.5
            else:
                twist.angular.z = -0.5

            duration = abs(angle) / 0.5
            self.execute_timed_movement(twist, duration)

        elif action_type == 'wait':
            duration = task.get('duration', 1.0)
            time.sleep(duration)

        elif action_type == 'speak':
            text = task.get('text', '')
            self.get_logger().info(f'Robot says: {text}')

        elif action_type == 'check_condition':
            condition = task.get('condition', '')
            self.get_logger().info(f'Checking condition: {condition}')
            # In a real implementation, this would check robot sensors or state

    def execute_timed_movement(self, twist, duration):
        """Execute movement for a specified duration"""
        start_time = time.time()
        while time.time() - start_time < duration and rclpy.ok():
            self.movement_publisher.publish(twist)
            time.sleep(0.1)

        stop_twist = Twist()
        self.movement_publisher.publish(stop_twist)

def main(args=None):
    rclpy.init(args=args)
    node = GPTTaskPlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Local LLM Integration

For privacy or offline use, you can also integrate local LLMs using Hugging Face transformers:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from transformers import pipeline, AutoTokenizer, AutoModelForCausalLM
import torch
import json

class LocalGPTNode(Node):
    def __init__(self):
        super().__init__('local_gpt_node')

        # Initialize local LLM (using a smaller model for efficiency)
        self.tokenizer = AutoTokenizer.from_pretrained("microsoft/DialoGPT-small")
        self.model = AutoModelForCausalLM.from_pretrained("microsoft/DialoGPT-small")

        # Create subscriber for natural language commands
        self.command_subscriber = self.create_subscription(
            String,
            'natural_language_commands',
            self.command_callback,
            10
        )

        # Create publisher for robot movement
        self.movement_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Conversation history for context
        self.chat_history_ids = None

        self.get_logger().info('Local GPT node initialized')

    def command_callback(self, msg):
        """Process incoming natural language commands with local model"""
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        try:
            # Encode the command
            new_user_input_ids = self.tokenizer.encode(command + self.tokenizer.eos_token, return_tensors='pt')

            # Append to chat history
            bot_input_ids = torch.cat([self.chat_history_ids, new_user_input_ids], dim=-1) if self.chat_history_ids is not None else new_user_input_ids

            # Generate response
            self.chat_history_ids = self.model.generate(
                bot_input_ids,
                max_length=1000,
                num_beams=5,
                early_stopping=True,
                pad_token_id=self.tokenizer.eos_token_id
            )

            # Decode response
            response = self.tokenizer.decode(self.chat_history_ids[:, bot_input_ids.shape[-1]:][0], skip_special_tokens=True)

            self.get_logger().info(f'Generated response: {response}')

            # In a real implementation, you'd parse the response to extract robot commands
            # This is a simplified example
            self.process_response(response)

        except Exception as e:
            self.get_logger().error(f'Error processing command with local model: {e}')

    def process_response(self, response):
        """Process the GPT response and extract robot commands"""
        # This would contain logic to parse the response and extract commands
        # For now, just log the response
        self.get_logger().info(f'Processing response: {response}')

def main(args=None):
    rclpy.init(args=args)
    node = LocalGPTNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Testing the GPT-ROS Bridge

To test the GPT-ROS bridge:

1. Set your OpenAI API key:
```bash
export OPENAI_API_KEY="your-api-key-here"
```

2. Launch the GPT bridge node:
```bash
ros2 run your_package gpt_bridge_node
```

3. Send test commands:
```bash
ros2 topic pub /natural_language_commands std_msgs/String "data: 'Go forward 2 meters'"
```

## Best Practices

- Use appropriate model size based on your computational resources and latency requirements
- Implement proper error handling for API failures
- Add rate limiting to avoid exceeding API quotas
- Use context to maintain conversation state across multiple commands
- Implement safety checks to validate GPT-generated actions before execution
- Consider privacy implications when using cloud-based models

## Summary

This chapter covered the integration of GPT with ROS 2 for advanced natural language processing in robotics. You learned how to create a bridge between GPT's language understanding capabilities and ROS 2's robotic control systems, enabling complex voice-to-action translation. The next chapter will cover the complete VLA workflows that combine Whisper, GPT, and robotic action execution.