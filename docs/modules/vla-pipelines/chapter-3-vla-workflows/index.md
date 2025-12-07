---
sidebar_position: 3
---

# Chapter 3: VLA Workflows

## Overview

This chapter covers the integration of Vision-Language-Action (VLA) pipelines that combine computer vision, natural language processing, and robotic action execution. You'll learn how to create complete workflows that process natural language commands, analyze visual input, and execute corresponding robotic actions in a cohesive system.

## Learning Objectives

By the end of this chapter, you will be able to:
- Design and implement complete VLA pipelines combining vision, language, and action
- Create end-to-end systems that process natural language commands with visual context
- Integrate perception, reasoning, and action execution in robotic systems
- Implement multimodal fusion for enhanced robotic decision-making
- Build robust VLA systems that handle real-world complexity

## Introduction to Vision-Language-Action Pipelines

Vision-Language-Action (VLA) pipelines represent the integration of three critical components in modern robotics:
- **Vision**: Processing visual input from cameras, depth sensors, and other perception systems
- **Language**: Understanding natural language commands and providing human-readable feedback
- **Action**: Executing physical actions in the environment through robotic systems

VLA systems enable robots to understand complex, natural language commands in the context of their visual environment, making human-robot interaction more intuitive and effective.

### Key Components of VLA Systems:
- Visual perception and scene understanding
- Natural language processing and command interpretation
- Action planning and execution
- Multimodal fusion and decision-making
- Feedback and communication systems

## Complete VLA Pipeline Architecture

Here's a comprehensive implementation of a VLA pipeline that combines all components:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist, PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
from openai import OpenAI
import json
import os
import time

class VLAPipelineNode(Node):
    def __init__(self):
        super().__init__('vla_pipeline_node')

        # Initialize OpenAI client
        api_key = os.getenv('OPENAI_API_KEY')
        if not api_key:
            self.get_logger().error('OPENAI_API_KEY environment variable not set')
            raise ValueError('OpenAI API key not found')

        self.client = OpenAI(api_key=api_key)

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Create subscribers
        self.command_subscriber = self.create_subscription(
            String,
            'natural_language_commands',
            self.command_callback,
            10
        )

        self.image_subscriber = self.create_subscription(
            Image,
            'camera/rgb/image_raw',
            self.image_callback,
            10
        )

        # Create publishers
        self.movement_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.action_status_publisher = self.create_publisher(String, 'action_status', 10)
        self.object_detection_publisher = self.create_publisher(String, 'detected_objects', 10)

        # VLA pipeline state
        self.current_image = None
        self.image_lock = threading.Lock()
        self.is_processing_command = False

        # Define system prompt for VLA reasoning
        self.system_prompt = """
        You are a Vision-Language-Action (VLA) reasoning system for robotics.
        Your role is to interpret natural language commands in the context of visual input,
        identify relevant objects in the scene, and generate appropriate robotic actions.

        The visual context includes information about objects in the environment.
        Respond with a JSON object containing the action type, target object, and parameters.

        Action types:
        - move_to_object: {object_name: string, approach_distance: float}
        - pick_object: {object_name: string, grasp_position: {x: float, y: float, z: float}}
        - place_object: {object_name: string, target_position: {x: float, y: float, z: float}}
        - navigate_to: {x: float, y: float}
        - identify_object: {object_name: string}
        - speak: {text: string}

        Example:
        Input: "Pick up the red cup on the table"
        Output: {
          "action": "pick_object",
          "object_name": "red cup",
          "grasp_position": {"x": 1.2, "y": 0.5, "z": 0.1}
        }
        """

        self.get_logger().info('VLA pipeline node initialized')

    def command_callback(self, msg):
        """Process incoming natural language commands with visual context"""
        command = msg.data
        self.get_logger().info(f'Received VLA command: {command}')

        # Only process if not already processing
        if self.is_processing_command:
            self.get_logger().warning('Already processing a command, skipping')
            return

        # Get current visual context
        with self.image_lock:
            if self.current_image is None:
                self.get_logger().warning('No image available for VLA processing')
                return

            # Process image to extract visual context
            visual_context = self.extract_visual_context(self.current_image)

        # Set processing flag
        self.is_processing_command = True

        try:
            # Create VLA prompt with visual context
            vla_prompt = f"""
            Natural language command: {command}

            Visual context: {visual_context}

            Please interpret the command in the context of the visual information
            and provide an appropriate robotic action.
            """

            # Call GPT for VLA reasoning
            response = self.client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": vla_prompt}
                ],
                temperature=0.1,
                max_tokens=400
            )

            # Extract and execute action
            gpt_response = response.choices[0].message.content.strip()

            try:
                action_data = json.loads(gpt_response)
                self.execute_vla_action(action_data)
            except json.JSONDecodeError:
                import re
                json_match = re.search(r'\{.*\}', gpt_response, re.DOTALL)
                if json_match:
                    action_data = json.loads(json_match.group())
                    self.execute_vla_action(action_data)
                else:
                    self.get_logger().error(f'Could not parse GPT response as JSON: {gpt_response}')

        except Exception as e:
            self.get_logger().error(f'Error in VLA pipeline: {e}')

        finally:
            self.is_processing_command = False

    def image_callback(self, msg):
        """Process incoming image data"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Store the image for VLA processing
            with self.image_lock:
                self.current_image = cv_image.copy()

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def extract_visual_context(self, image):
        """Extract visual context from the image"""
        # Simple object detection using color-based segmentation
        # In a real implementation, you'd use more sophisticated vision models
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define color ranges for common objects
        color_ranges = {
            'red': ([0, 50, 50], [10, 255, 255]),
            'blue': ([100, 50, 50], [130, 255, 255]),
            'green': ([40, 50, 50], [80, 255, 255]),
            'yellow': ([20, 50, 50], [40, 255, 255])
        }

        objects = []

        for color_name, (lower, upper) in color_ranges.items():
            lower = np.array(lower, dtype="uint8")
            upper = np.array(upper, dtype="uint8")

            mask = cv2.inRange(hsv, lower, upper)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 500:  # Filter out small contours
                    # Get bounding box
                    x, y, w, h = cv2.boundingRect(contour)
                    center_x = x + w // 2
                    center_y = y + h // 2

                    # Convert to relative coordinates (0-1)
                    rel_x = center_x / image.shape[1]
                    rel_y = center_y / image.shape[0]

                    objects.append({
                        'name': f'{color_name} object',
                        'color': color_name,
                        'position': {'x': rel_x, 'y': rel_y},
                        'size': area
                    })

        # Also detect common shapes (simple approach)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edged = cv2.Canny(blurred, 50, 150)

        contours, _ = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 1000:  # Filter out small contours
                # Approximate contour to get shape
                peri = cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, 0.04 * peri, True)

                shape = "unknown"
                if len(approx) == 3:
                    shape = "triangle"
                elif len(approx) == 4:
                    # Check if it's square or rectangle
                    (x, y, w, h) = cv2.boundingRect(approx)
                    ar = w / float(h)
                    shape = "square" if 0.95 <= ar <= 1.05 else "rectangle"
                elif len(approx) > 4:
                    shape = "circle"

                if shape != "unknown":
                    x, y, w, h = cv2.boundingRect(contour)
                    center_x = x + w // 2
                    center_y = y + h // 2

                    rel_x = center_x / image.shape[1]
                    rel_y = center_y / image.shape[0]

                    objects.append({
                        'name': f'{shape}',
                        'shape': shape,
                        'position': {'x': rel_x, 'y': rel_y},
                        'size': area
                    })

        # Create visual context description
        if objects:
            context = f"Detected {len(objects)} objects: "
            for obj in objects[:5]:  # Limit to first 5 objects
                context += f"{obj['name']} at position ({obj['position']['x']:.2f}, {obj['position']['y']:.2f}), "
        else:
            context = "No significant objects detected in the scene."

        return context

    def execute_vla_action(self, action_data):
        """Execute the VLA action specified by the reasoning system"""
        action_type = action_data.get('action')
        self.get_logger().info(f'Executing VLA action: {action_type}')

        # Publish action status
        status_msg = String()
        status_msg.data = f'Executing: {action_type}'
        self.action_status_publisher.publish(status_msg)

        if action_type == 'move_to_object':
            object_name = action_data.get('object_name', '')
            approach_distance = action_data.get('approach_distance', 0.5)

            self.get_logger().info(f'Moving towards {object_name}, distance: {approach_distance}')
            # In a real implementation, this would navigate to the object
            # For now, we'll just move forward
            self.move_forward(approach_distance)

        elif action_type == 'pick_object':
            object_name = action_data.get('object_name', '')
            grasp_position = action_data.get('grasp_position', {})

            self.get_logger().info(f'Attempting to pick {object_name} at {grasp_position}')
            # In a real implementation, this would control a manipulator arm
            # For now, we'll just log the action
            self.get_logger().info(f'Pick action for {object_name} would execute here')

        elif action_type == 'place_object':
            object_name = action_data.get('object_name', '')
            target_position = action_data.get('target_position', {})

            self.get_logger().info(f'Placing {object_name} at {target_position}')
            # In a real implementation, this would control a manipulator arm
            # For now, we'll just log the action
            self.get_logger().info(f'Place action for {object_name} would execute here')

        elif action_type == 'navigate_to':
            x = action_data.get('x', 0.0)
            y = action_data.get('y', 0.0)

            self.get_logger().info(f'Navigating to position ({x}, {y})')
            # This would typically call a navigation action server
            # For now, we'll just log the action
            self.get_logger().info(f'Navigation to ({x}, {y}) would execute here')

        elif action_type == 'identify_object':
            object_name = action_data.get('object_name', '')

            self.get_logger().info(f'Identifying object: {object_name}')
            # In a real implementation, this might highlight or point to the object
            # For now, we'll just log the action
            self.get_logger().info(f'Object identification for {object_name} would execute here')

        elif action_type == 'speak':
            text = action_data.get('text', '')
            self.get_logger().info(f'Speaking: {text}')
            # In a real implementation, this would use text-to-speech

        else:
            self.get_logger().warning(f'Unknown action type: {action_type}')

    def move_forward(self, distance):
        """Move the robot forward by the specified distance"""
        # Calculate duration based on speed (assuming 0.5 m/s)
        duration = abs(distance) / 0.5
        twist = Twist()
        twist.linear.x = 0.5 if distance > 0 else -0.5

        start_time = time.time()
        while time.time() - start_time < duration and rclpy.ok():
            self.movement_publisher.publish(twist)
            time.sleep(0.1)

        # Stop the robot
        stop_twist = Twist()
        self.movement_publisher.publish(stop_twist)

def main(args=None):
    rclpy.init(args=args)
    node = VLAPipelineNode()

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

## Advanced VLA with Object Detection

For more sophisticated object detection, you can integrate with popular computer vision models:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
from openai import OpenAI
import json
import os

class AdvancedVLANode(Node):
    def __init__(self):
        super().__init__('advanced_vla_node')

        # Initialize OpenAI client
        api_key = os.getenv('OPENAI_API_KEY')
        if not api_key:
            self.get_logger().error('OPENAI_API_KEY environment variable not set')
            raise ValueError('OpenAI API key not found')

        self.client = OpenAI(api_key=api_key)

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Create subscribers and publishers
        self.command_subscriber = self.create_subscription(
            String,
            'natural_language_commands',
            self.command_callback,
            10
        )

        self.image_subscriber = self.create_subscription(
            Image,
            'camera/rgb/image_raw',
            self.image_callback,
            10
        )

        self.movement_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.object_detection_publisher = self.create_publisher(String, 'detected_objects', 10)

        # VLA state
        self.current_image = None
        self.image_lock = threading.Lock()

        # Simple class names for detection (in a real system, you'd use a proper model)
        self.class_names = [
            'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train',
            'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign',
            'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep',
            'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella',
            'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard',
            'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard',
            'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork',
            'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
            'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair',
            'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv',
            'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave',
            'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase',
            'scissors', 'teddy bear', 'hair drier', 'toothbrush'
        ]

        # Define system prompt for advanced VLA reasoning
        self.system_prompt = """
        You are an advanced Vision-Language-Action (VLA) reasoning system for robotics.
        Your role is to interpret complex natural language commands in the context of
        detailed visual input, perform spatial reasoning, and generate appropriate robotic actions.

        The visual context includes detected objects with their positions, sizes, and relationships.
        Use spatial reasoning to understand object relationships and execute appropriate actions.

        Action types:
        - approach_object: {object_name: string, distance: float}
        - grasp_object: {object_name: string, position: {x: float, y: float, z: float}}
        - manipulate_object: {object_name: string, action: string}
        - navigate_to_object: {object_name: string}
        - describe_scene: {object_name: string}
        - speak: {text: string}

        Example:
        Input: "Pick up the red cup on the left side of the table"
        Output: {
          "action": "grasp_object",
          "object_name": "cup",
          "position": {"x": 0.8, "y": 0.3, "z": 0.1},
          "spatial_context": "left side of table"
        }
        """

        self.get_logger().info('Advanced VLA node initialized')

    def command_callback(self, msg):
        """Process incoming natural language commands with advanced visual context"""
        command = msg.data
        self.get_logger().info(f'Received advanced VLA command: {command}')

        with self.image_lock:
            if self.current_image is None:
                self.get_logger().warning('No image available for VLA processing')
                return

            # Extract detailed visual context
            visual_context = self.extract_detailed_visual_context(self.current_image)

        try:
            # Create advanced VLA prompt
            vla_prompt = f"""
            Natural language command: {command}

            Detailed visual context:
            {visual_context}

            Please interpret the command in the context of the visual information,
            perform spatial reasoning, and provide an appropriate robotic action.
            """

            # Call GPT for advanced VLA reasoning
            response = self.client.chat.completions.create(
                model="gpt-4",  # Using GPT-4 for more complex reasoning
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": vla_prompt}
                ],
                temperature=0.1,
                max_tokens=500
            )

            # Extract and execute action
            gpt_response = response.choices[0].message.content.strip()

            try:
                action_data = json.loads(gpt_response)
                self.execute_advanced_vla_action(action_data)
            except json.JSONDecodeError:
                import re
                json_match = re.search(r'\{.*\}', gpt_response, re.DOTALL)
                if json_match:
                    action_data = json.loads(json_match.group())
                    self.execute_advanced_vla_action(action_data)
                else:
                    self.get_logger().error(f'Could not parse GPT response as JSON: {gpt_response}')

        except Exception as e:
            self.get_logger().error(f'Error in advanced VLA pipeline: {e}')

    def image_callback(self, msg):
        """Process incoming image data"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            with self.image_lock:
                self.current_image = cv_image.copy()

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def extract_detailed_visual_context(self, image):
        """Extract detailed visual context with object detection and spatial relationships"""
        # This is a simplified implementation
        # In a real system, you'd use YOLO, Detectron2, or similar models

        height, width = image.shape[:2]

        # Simple color-based detection for demonstration
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define regions of interest (left, center, right)
        left_region = image[:, :width//3]
        center_region = image[:, width//3:2*width//3]
        right_region = image[:, 2*width//3:]

        objects = []

        # Detect objects in different regions
        regions = [('left', left_region), ('center', center_region), ('right', right_region)]

        for region_name, region in regions:
            # Detect colored objects
            color_ranges = {
                'red': ([0, 50, 50], [10, 255, 255]),
                'blue': ([100, 50, 50], [130, 255, 255]),
                'green': ([40, 50, 50], [80, 255, 255]),
                'yellow': ([20, 50, 50], [40, 255, 255])
            }

            for color_name, (lower, upper) in color_ranges.items():
                lower = np.array(lower, dtype="uint8")
                upper = np.array(upper, dtype="uint8")

                mask = cv2.inRange(cv2.cvtColor(region, cv2.COLOR_BGR2HSV), lower, upper)
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                for contour in contours:
                    area = cv2.contourArea(contour)
                    if area > 500:  # Filter out small contours
                        # Get bounding box
                        x, y, w, h = cv2.boundingRect(contour)
                        center_x = x + w // 2
                        center_y = y + h // 2

                        # Convert to relative coordinates within the full image
                        abs_x = (center_x + (0 if region_name == 'left' else width//3 if region_name == 'center' else 2*width//3)) / width
                        abs_y = (center_y + y) / height

                        objects.append({
                            'name': f'{color_name} object',
                            'region': region_name,
                            'position': {'x': abs_x, 'y': abs_y},
                            'size': area
                        })

        # Create detailed context description
        if objects:
            context = f"Detected {len(objects)} objects:\n"
            for obj in objects:
                context += f"- {obj['name']} in {obj['region']} region at ({obj['position']['x']:.2f}, {obj['position']['y']:.2f}), size: {obj['size']:.0f}\n"
        else:
            context = "No significant objects detected in the scene."

        # Add spatial relationships
        if len(objects) > 1:
            context += "\nSpatial relationships:\n"
            for i, obj1 in enumerate(objects):
                for j, obj2 in enumerate(objects):
                    if i != j:
                        dx = obj1['position']['x'] - obj2['position']['x']
                        dy = obj1['position']['y'] - obj2['position']['y']

                        if abs(dx) > 0.1:  # Significant horizontal difference
                            direction = "right" if dx > 0 else "left"
                            context += f"- {obj1['name']} is to the {direction} of {obj2['name']}\n"
                        if abs(dy) > 0.1:  # Significant vertical difference
                            direction = "below" if dy > 0 else "above"
                            context += f"- {obj1['name']} is {direction} {obj2['name']}\n"

        return context

    def execute_advanced_vla_action(self, action_data):
        """Execute the advanced VLA action"""
        action_type = action_data.get('action')
        self.get_logger().info(f'Executing advanced VLA action: {action_type}')

        if action_type == 'approach_object':
            object_name = action_data.get('object_name', '')
            distance = action_data.get('distance', 0.5)

            self.get_logger().info(f'Approaching {object_name}, distance: {distance}')
            # Implementation would navigate to the object

        elif action_type == 'grasp_object':
            object_name = action_data.get('object_name', '')
            position = action_data.get('position', {})
            spatial_context = action_data.get('spatial_context', '')

            self.get_logger().info(f'Grasping {object_name} at {position}, context: {spatial_context}')
            # Implementation would control manipulator arm

        elif action_type == 'navigate_to_object':
            object_name = action_data.get('object_name', '')

            self.get_logger().info(f'Navigating to {object_name}')
            # Implementation would use navigation system

        elif action_type == 'describe_scene':
            object_name = action_data.get('object_name', '')

            self.get_logger().info(f'Describing scene around {object_name}')
            # Implementation would provide detailed description

        elif action_type == 'speak':
            text = action_data.get('text', '')
            self.get_logger().info(f'Speaking: {text}')

        else:
            self.get_logger().warning(f'Unknown action type: {action_type}')

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedVLANode()

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

## VLA Pipeline with Memory and Context

For more sophisticated VLA systems, you can implement memory and context tracking:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
from openai import OpenAI
import json
import os
import time
from collections import deque

class MemoryEnhancedVLANode(Node):
    def __init__(self):
        super().__init__('memory_enhanced_vla_node')

        # Initialize OpenAI client
        api_key = os.getenv('OPENAI_API_KEY')
        if not api_key:
            self.get_logger().error('OPENAI_API_KEY environment variable not set')
            raise ValueError('OpenAI API key not found')

        self.client = OpenAI(api_key=api_key)

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Create subscribers and publishers
        self.command_subscriber = self.create_subscription(
            String,
            'natural_language_commands',
            self.command_callback,
            10
        )

        self.image_subscriber = self.create_subscription(
            Image,
            'camera/rgb/image_raw',
            self.image_callback,
            10
        )

        self.movement_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.response_publisher = self.create_publisher(String, 'vla_responses', 10)

        # VLA state with memory
        self.current_image = None
        self.image_lock = threading.Lock()
        self.command_history = deque(maxlen=10)  # Keep last 10 commands
        self.action_history = deque(maxlen=10)   # Keep last 10 actions
        self.object_memory = {}  # Track objects across time

        # Define system prompt with memory context
        self.system_prompt = """
        You are a memory-enhanced Vision-Language-Action (VLA) system for robotics.
        You have access to:
        1. Current visual input from the robot's camera
        2. Historical commands and actions
        3. Tracked objects in the environment

        Use this memory to:
        - Understand references to previously mentioned objects ("it", "that", "the red one")
        - Maintain context across multiple interactions
        - Track object locations and states over time
        - Resolve ambiguous references based on context

        Respond with a JSON object containing the action type and parameters.

        Action types:
        - move_to: {object_name: string, distance: float}
        - manipulate: {object_name: string, action: string}
        - remember: {object_name: string, location: {x: float, y: float, z: float}}
        - recall: {object_name: string}
        - speak: {text: string}

        Example:
        Previous: "Move to the red cup" -> Action: move_to red cup
        Current: "Pick it up" -> Action: manipulate red cup
        """

        self.get_logger().info('Memory-enhanced VLA node initialized')

    def command_callback(self, msg):
        """Process incoming natural language commands with memory"""
        command = msg.data
        self.get_logger().info(f'Received command with memory: {command}')

        # Add to command history
        self.command_history.append({
            'timestamp': time.time(),
            'command': command
        })

        with self.image_lock:
            if self.current_image is None:
                self.get_logger().warning('No image available for VLA processing')
                return

            visual_context = self.extract_visual_context(self.current_image)

        try:
            # Create memory-enhanced prompt
            memory_context = self.build_memory_context()

            vla_prompt = f"""
            Natural language command: {command}

            Visual context: {visual_context}

            Memory context:
            - Previous commands: {list(self.command_history)}
            - Previous actions: {list(self.action_history)}
            - Known objects: {self.object_memory}

            Please interpret the command using all available context and provide an appropriate action.
            """

            # Call GPT for memory-enhanced VLA reasoning
            response = self.client.chat.completions.create(
                model="gpt-4",
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": vla_prompt}
                ],
                temperature=0.1,
                max_tokens=500
            )

            gpt_response = response.choices[0].message.content.strip()

            try:
                action_data = json.loads(gpt_response)

                # Add to action history
                self.action_history.append({
                    'timestamp': time.time(),
                    'action': action_data
                })

                self.execute_memory_enhanced_action(action_data)
            except json.JSONDecodeError:
                import re
                json_match = re.search(r'\{.*\}', gpt_response, re.DOTALL)
                if json_match:
                    action_data = json.loads(json_match.group())

                    # Add to action history
                    self.action_history.append({
                        'timestamp': time.time(),
                        'action': action_data
                    })

                    self.execute_memory_enhanced_action(action_data)
                else:
                    self.get_logger().error(f'Could not parse GPT response as JSON: {gpt_response}')

        except Exception as e:
            self.get_logger().error(f'Error in memory-enhanced VLA pipeline: {e}')

    def image_callback(self, msg):
        """Process incoming image data"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            with self.image_lock:
                self.current_image = cv_image.copy()

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def extract_visual_context(self, image):
        """Extract visual context from image"""
        # Simplified implementation - in real system use proper object detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        color_ranges = {
            'red': ([0, 50, 50], [10, 255, 255]),
            'blue': ([100, 50, 50], [130, 255, 255]),
            'green': ([40, 50, 50], [80, 255, 255]),
            'yellow': ([20, 50, 50], [40, 255, 255])
        }

        objects = []
        height, width = image.shape[:2]

        for color_name, (lower, upper) in color_ranges.items():
            lower = np.array(lower, dtype="uint8")
            upper = np.array(upper, dtype="uint8")

            mask = cv2.inRange(hsv, lower, upper)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for i, contour in enumerate(contours):
                area = cv2.contourArea(contour)
                if area > 500:
                    x, y, w, h = cv2.boundingRect(contour)
                    center_x = x + w // 2
                    center_y = y + h // 2

                    obj_name = f'{color_name}_{i}'
                    objects.append({
                        'name': obj_name,
                        'color': color_name,
                        'position': {'x': center_x / width, 'y': center_y / height},
                        'size': area
                    })

                    # Update object memory with location
                    self.object_memory[obj_name] = {
                        'last_seen': time.time(),
                        'position': {'x': center_x / width, 'y': center_y / height},
                        'type': 'colored_object',
                        'color': color_name
                    }

        if objects:
            context = f"Detected {len(objects)} objects: "
            for obj in objects:
                context += f"{obj['name']} ({obj['color']}) at ({obj['position']['x']:.2f}, {obj['position']['y']:.2f}), "
        else:
            context = "No significant objects detected."

        return context

    def build_memory_context(self):
        """Build memory context from history"""
        context = {
            'recent_commands': list(self.command_history),
            'recent_actions': list(self.action_history),
            'tracked_objects': self.object_memory,
            'current_time': time.time()
        }
        return context

    def execute_memory_enhanced_action(self, action_data):
        """Execute action with memory considerations"""
        action_type = action_data.get('action')
        self.get_logger().info(f'Executing memory-enhanced action: {action_type}')

        if action_type == 'move_to':
            object_name = action_data.get('object_name', '')

            # Check if object is in memory
            if object_name in self.object_memory:
                obj_info = self.object_memory[object_name]
                self.get_logger().info(f'Moving to {object_name} at {obj_info["position"]}')
                # Implementation would navigate to object location
            else:
                self.get_logger().info(f'Moving to {object_name} (location unknown, will search)')

        elif action_type == 'manipulate':
            object_name = action_data.get('object_name', '')
            action = action_data.get('action', 'grasp')

            self.get_logger().info(f'{action.capitalize()}ing {object_name}')
            # Implementation would control manipulator

        elif action_type == 'remember':
            object_name = action_data.get('object_name', '')
            location = action_data.get('location', {})

            self.object_memory[object_name] = {
                'last_seen': time.time(),
                'position': location,
                'type': 'remembered_object'
            }
            self.get_logger().info(f'Remembering {object_name} at {location}')

        elif action_type == 'recall':
            object_name = action_data.get('object_name', '')

            if object_name in self.object_memory:
                obj_info = self.object_memory[object_name]
                response_msg = String()
                response_msg.data = f'{object_name} was last seen at {obj_info["position"]}'
                self.response_publisher.publish(response_msg)
                self.get_logger().info(f'Recalled {object_name} location: {obj_info["position"]}')
            else:
                response_msg = String()
                response_msg.data = f'I don\'t remember seeing {object_name}'
                self.response_publisher.publish(response_msg)
                self.get_logger().info(f'Don\'t remember {object_name}')

        elif action_type == 'speak':
            text = action_data.get('text', '')
            response_msg = String()
            response_msg.data = text
            self.response_publisher.publish(response_msg)
            self.get_logger().info(f'Speaking: {text}')

        else:
            self.get_logger().warning(f'Unknown action type: {action_type}')

def main(args=None):
    rclpy.init(args=args)
    node = MemoryEnhancedVLANode()

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

## Testing the Complete VLA Pipeline

To test the complete VLA pipeline:

1. Launch the VLA node:
```bash
ros2 run your_package vla_pipeline_node
```

2. Publish camera images (if not already available):
```bash
# Using a camera driver or image publisher
ros2 run image_publisher image_publisher_node --ros-args -p file_name:=/path/to/test/image.jpg
```

3. Send VLA commands:
```bash
ros2 topic pub /natural_language_commands std_msgs/String "data: 'Move to the red object'"
```

## Best Practices for VLA Systems

- Use appropriate models for your computational constraints (local models for privacy, cloud models for capability)
- Implement proper error handling and fallback behaviors
- Design for safety with validation of GPT-generated actions
- Maintain context and memory for coherent multi-turn interactions
- Consider real-time constraints and optimize for latency
- Implement multimodal fusion for enhanced decision-making
- Test extensively in simulation before real-world deployment

## Summary

This chapter covered the implementation of complete Vision-Language-Action (VLA) pipelines that integrate computer vision, natural language processing, and robotic action execution. You learned how to create systems that process natural language commands in the context of visual input and execute corresponding robotic actions. The chapter included implementations for basic VLA, advanced object detection, and memory-enhanced systems that maintain context across interactions.

The VLA approach enables robots to understand complex, natural language commands in the context of their visual environment, making human-robot interaction more intuitive and effective. This completes the VLA Pipelines module, providing developers with the tools to build sophisticated natural language interfaces for robotic systems.