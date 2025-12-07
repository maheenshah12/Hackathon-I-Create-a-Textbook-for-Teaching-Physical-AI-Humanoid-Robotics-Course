#!/usr/bin/env python3
"""
Complete Vision-Language-Action (VLA) Pipeline Example

This script demonstrates a complete VLA pipeline that integrates:
1. Whisper for speech recognition
2. GPT for natural language understanding
3. Computer vision for object detection
4. ROS 2 for robotic control

The pipeline processes natural language commands, analyzes visual input,
and executes corresponding robotic actions.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, AudioData
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import whisper
import pyaudio
import numpy as np
import threading
from openai import OpenAI
import json
import os
import time
import queue
from collections import deque

class CompleteVLAPipelineNode(Node):
    def __init__(self):
        super().__init__('complete_vla_pipeline_node')

        # Initialize OpenAI client
        api_key = os.getenv('OPENAI_API_KEY')
        if not api_key:
            self.get_logger().error('OPENAI_API_KEY environment variable not set')
            raise ValueError('OpenAI API key not found')

        self.client = OpenAI(api_key=api_key)

        # Initialize Whisper model for speech recognition
        try:
            self.whisper_model = whisper.load_model("base")
        except Exception as e:
            self.get_logger().warning(f'Could not load Whisper model: {e}')
            self.whisper_model = None

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Create subscribers
        self.audio_subscriber = self.create_subscription(
            AudioData,
            'audio_input',
            self.audio_callback,
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
        self.voice_command_publisher = self.create_publisher(String, 'voice_commands', 10)
        self.natural_language_publisher = self.create_publisher(String, 'natural_language_commands', 10)
        self.status_publisher = self.create_publisher(String, 'vla_status', 10)

        # VLA pipeline state
        self.current_image = None
        self.audio_buffer = queue.Queue()
        self.image_lock = threading.Lock()
        self.is_processing = False
        self.command_history = deque(maxlen=20)
        self.object_memory = {}

        # Audio configuration for real-time processing
        self.rate = 16000
        self.chunk = 1024
        self.audio_format = pyaudio.paInt16
        self.channels = 1

        # Initialize PyAudio for audio capture
        self.audio = pyaudio.PyAudio()
        self.audio_stream = self.audio.open(
            format=self.audio_format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )

        # Start audio processing thread
        self.audio_thread = threading.Thread(target=self.process_audio, daemon=True)
        self.audio_thread.start()

        # Define system prompts for different components
        self.vla_system_prompt = """
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

        self.get_logger().info('Complete VLA pipeline node initialized')

    def audio_callback(self, msg):
        """Handle incoming audio data"""
        try:
            audio_array = np.frombuffer(msg.data, dtype=np.int16)
            self.audio_buffer.put(audio_array)
        except Exception as e:
            self.get_logger().error(f'Error processing audio callback: {e}')

    def process_audio(self):
        """Process audio data in a separate thread"""
        while rclpy.ok():
            try:
                audio_data = self.audio_buffer.get(timeout=1.0)

                if self.whisper_model:
                    # Process audio with Whisper
                    result = self.whisper_model.transcribe(
                        audio_data.astype(np.float32) / 32768.0
                    )

                    if result['text'].strip():
                        # Publish recognized text
                        text_msg = String()
                        text_msg.data = result['text']
                        self.voice_command_publisher.publish(text_msg)

                        # Process the command through VLA pipeline
                        self.process_vla_command(result['text'])

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Error in audio processing thread: {e}')

    def image_callback(self, msg):
        """Process incoming image data"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            with self.image_lock:
                self.current_image = cv_image.copy()

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def process_vla_command(self, command):
        """Process a VLA command through the complete pipeline"""
        self.get_logger().info(f'Processing VLA command: {command}')

        # Add to command history
        self.command_history.append({
            'timestamp': time.time(),
            'command': command
        })

        # Check if we're already processing
        if self.is_processing:
            self.get_logger().warning('Already processing a command, skipping')
            return

        self.is_processing = True

        try:
            # Get visual context
            with self.image_lock:
                if self.current_image is None:
                    self.get_logger().warning('No image available for VLA processing')
                    self.is_processing = False
                    return

                visual_context = self.extract_visual_context(self.current_image)

            # Create VLA prompt
            vla_prompt = f"""
            Natural language command: {command}

            Visual context: {visual_context}

            Memory context:
            - Known objects: {self.object_memory}

            Please interpret the command in the context of the visual information
            and provide an appropriate robotic action.
            """

            # Call GPT for VLA reasoning
            response = self.client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": self.vla_system_prompt},
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
            self.is_processing = False

    def extract_visual_context(self, image):
        """Extract visual context from image with simple object detection"""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define color ranges for common objects
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
                if area > 500:  # Filter out small contours
                    x, y, w, h = cv2.boundingRect(contour)
                    center_x = x + w // 2
                    center_y = y + h // 2

                    obj_name = f'{color_name}_{i}'
                    obj_info = {
                        'name': obj_name,
                        'color': color_name,
                        'position': {'x': center_x / width, 'y': center_y / height},
                        'size': area
                    }

                    objects.append(obj_info)

                    # Update object memory
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

    def execute_vla_action(self, action_data):
        """Execute the VLA action specified by the reasoning system"""
        action_type = action_data.get('action')
        self.get_logger().info(f'Executing VLA action: {action_type}')

        # Publish status
        status_msg = String()
        status_msg.data = f'Executing: {action_type}'
        self.status_publisher.publish(status_msg)

        if action_type == 'move_to_object':
            object_name = action_data.get('object_name', '')
            approach_distance = action_data.get('approach_distance', 0.5)

            self.get_logger().info(f'Moving towards {object_name}, distance: {approach_distance}')
            self.move_to_object(object_name, approach_distance)

        elif action_type == 'navigate_to':
            x = action_data.get('x', 0.0)
            y = action_data.get('y', 0.0)

            self.get_logger().info(f'Navigating to position ({x}, {y})')
            self.navigate_to(x, y)

        elif action_type == 'identify_object':
            object_name = action_data.get('object_name', '')

            self.get_logger().info(f'Identifying object: {object_name}')
            self.identify_object(object_name)

        elif action_type == 'speak':
            text = action_data.get('text', '')
            self.get_logger().info(f'Speaking: {text}')
            # In a real implementation, this would use text-to-speech

        else:
            self.get_logger().warning(f'Unknown action type: {action_type}')

    def move_to_object(self, object_name, approach_distance):
        """Move towards a detected object"""
        if object_name in self.object_memory:
            obj_info = self.object_memory[object_name]
            pos = obj_info['position']

            self.get_logger().info(f'Moving towards {object_name} at relative position ({pos["x"]:.2f}, {pos["y"]:.2f})')

            # Simple movement based on object position
            twist = Twist()

            # Move forward if object is in front
            if pos['y'] < 0.6:  # Object is in upper half (in front)
                twist.linear.x = 0.3  # Move forward

            # Adjust angular velocity based on horizontal position
            horizontal_offset = pos['x'] - 0.5  # Center is at 0.5
            twist.angular.z = -horizontal_offset * 0.5  # Proportional control

            # Move for a short duration
            duration = 2.0
            start_time = time.time()
            while time.time() - start_time < duration and rclpy.ok():
                self.movement_publisher.publish(twist)
                time.sleep(0.1)

            # Stop
            stop_twist = Twist()
            self.movement_publisher.publish(stop_twist)
        else:
            self.get_logger().warning(f'Object {object_name} not in memory')

    def navigate_to(self, x, y):
        """Navigate to a specific coordinate (simplified implementation)"""
        self.get_logger().info(f'Navigating to ({x}, {y})')

        # In a real implementation, this would use navigation2
        # For this example, we'll just move forward for a bit
        twist = Twist()
        twist.linear.x = 0.3

        duration = 3.0
        start_time = time.time()
        while time.time() - start_time < duration and rclpy.ok():
            self.movement_publisher.publish(twist)
            time.sleep(0.1)

        # Stop
        stop_twist = Twist()
        self.movement_publisher.publish(stop_twist)

    def identify_object(self, object_name):
        """Identify and provide information about an object"""
        if object_name in self.object_memory:
            obj_info = self.object_memory[object_name]
            self.get_logger().info(f'Object {object_name}: {obj_info}')

            # Publish description
            desc_msg = String()
            desc_msg.data = f'I see a {obj_info["color"]} object at position {obj_info["position"]}'
            self.status_publisher.publish(desc_msg)
        else:
            self.get_logger().warning(f'Object {object_name} not in memory')

    def destroy_node(self):
        """Clean up resources"""
        if hasattr(self, 'audio_stream'):
            self.audio_stream.stop_stream()
            self.audio_stream.close()
        if hasattr(self, 'audio'):
            self.audio.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CompleteVLAPipelineNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()