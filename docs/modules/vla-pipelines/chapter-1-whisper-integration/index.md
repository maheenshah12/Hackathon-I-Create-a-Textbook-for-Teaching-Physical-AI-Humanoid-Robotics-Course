---
sidebar_position: 1
---

# Chapter 1: Whisper Integration

## Overview

This chapter covers the integration of OpenAI's Whisper speech recognition model with ROS 2 for voice command processing in robotic systems. You'll learn how to set up Whisper for real-time speech-to-text conversion, process audio input from microphones, and integrate voice commands with robotic control systems.

## Learning Objectives

By the end of this chapter, you will be able to:
- Set up Whisper speech recognition in a ROS 2 environment
- Process audio input streams and convert speech to text
- Integrate voice commands with robotic action execution
- Handle real-time audio processing with ROS 2 nodes
- Implement error handling and confidence scoring for voice recognition

## Introduction to Whisper for Robotics

Whisper is a state-of-the-art automatic speech recognition (ASR) system developed by OpenAI. It's particularly well-suited for robotics applications due to its robustness across different accents, background noise, and audio quality. In robotics, Whisper enables natural human-robot interaction through voice commands.

### Key Benefits for Robotics:
- Multilingual support for international applications
- Robust performance in noisy environments
- Real-time processing capabilities
- Integration with existing ROS 2 infrastructure

## Setting Up Whisper with ROS 2

### Prerequisites

Before implementing Whisper integration, ensure you have:
- ROS 2 Humble installed
- Python 3.8 or higher
- Audio input device (microphone)
- OpenAI Whisper library installed

### Installation

First, install the required dependencies:

```bash
pip install openai-whisper
pip install pyaudio
pip install sounddevice
```

### Basic Whisper Node Implementation

Here's a basic implementation of a Whisper node that processes audio input:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import whisper
import pyaudio
import numpy as np
import threading
import queue
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData

class WhisperNode(Node):
    def __init__(self):
        super().__init__('whisper_node')

        # Initialize Whisper model
        self.model = whisper.load_model("base")  # Options: tiny, base, small, medium, large

        # Audio configuration
        self.rate = 16000  # Sample rate
        self.chunk = 1024  # Buffer size
        self.format = pyaudio.paInt16
        self.channels = 1

        # Initialize PyAudio
        self.audio = pyaudio.PyAudio()

        # Create publisher for recognized text
        self.text_publisher = self.create_publisher(String, 'voice_commands', 10)

        # Create subscriber for audio data
        self.audio_subscriber = self.create_subscription(
            AudioData,
            'audio_input',
            self.audio_callback,
            10
        )

        # Audio buffer for processing
        self.audio_buffer = queue.Queue()

        # Start audio processing thread
        self.processing_thread = threading.Thread(target=self.process_audio, daemon=True)
        self.processing_thread.start()

        self.get_logger().info('Whisper node initialized')

    def audio_callback(self, msg):
        """Callback to handle incoming audio data"""
        # Convert audio data to numpy array
        audio_array = np.frombuffer(msg.data, dtype=np.int16)
        self.audio_buffer.put(audio_array)

    def process_audio(self):
        """Process audio data in a separate thread"""
        while rclpy.ok():
            try:
                # Get audio data from buffer
                audio_data = self.audio_buffer.get(timeout=1.0)

                # Process audio with Whisper
                result = self.model.transcribe(audio_data.astype(np.float32) / 32768.0)

                # Publish recognized text
                if result['text'].strip():
                    text_msg = String()
                    text_msg.data = result['text']
                    self.text_publisher.publish(text_msg)
                    self.get_logger().info(f'Recognized: {result["text"]}')

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Error processing audio: {e}')

    def destroy_node(self):
        """Clean up resources"""
        self.audio.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WhisperNode()

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

## Real-Time Audio Capture

For real-time audio capture, you can also implement a separate node that captures audio from a microphone:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import pyaudio
import threading
from audio_common_msgs.msg import AudioData

class AudioCaptureNode(Node):
    def __init__(self):
        super().__init__('audio_capture_node')

        # Audio configuration
        self.rate = 16000
        self.chunk = 1024
        self.format = pyaudio.paInt16
        self.channels = 1

        # Initialize PyAudio
        self.audio = pyaudio.PyAudio()

        # Create publisher for audio data
        self.audio_publisher = self.create_publisher(AudioData, 'audio_input', 10)

        # Start audio stream
        self.stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )

        # Start audio capture thread
        self.capture_thread = threading.Thread(target=self.capture_audio, daemon=True)
        self.capture_thread.start()

        self.get_logger().info('Audio capture node initialized')

    def capture_audio(self):
        """Capture audio in a separate thread"""
        while rclpy.ok():
            try:
                # Read audio data
                data = self.stream.read(self.chunk)

                # Create and publish audio message
                audio_msg = AudioData()
                audio_msg.data = data
                self.audio_publisher.publish(audio_msg)

            except Exception as e:
                self.get_logger().error(f'Error capturing audio: {e}')
                break

    def destroy_node(self):
        """Clean up resources"""
        self.stream.stop_stream()
        self.stream.close()
        self.audio.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AudioCaptureNode()

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

## Voice Command Processing

To process voice commands and convert them to robotic actions, you can create a command processing node:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import re

class VoiceCommandProcessor(Node):
    def __init__(self):
        super().__init__('voice_command_processor')

        # Create subscriber for voice commands
        self.command_subscriber = self.create_subscription(
            String,
            'voice_commands',
            self.command_callback,
            10
        )

        # Create publisher for robot movement
        self.movement_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Define command patterns
        self.command_patterns = {
            'move_forward': r'go forward|move forward|forward',
            'move_backward': r'go backward|move backward|backward|back',
            'turn_left': r'turn left|left',
            'turn_right': r'turn right|right',
            'stop': r'stop|halt|pause'
        }

        self.get_logger().info('Voice command processor initialized')

    def command_callback(self, msg):
        """Process incoming voice commands"""
        command = msg.data.lower().strip()
        self.get_logger().info(f'Received command: {command}')

        # Process command based on patterns
        for action, pattern in self.command_patterns.items():
            if re.search(pattern, command):
                self.execute_command(action)
                break

    def execute_command(self, action):
        """Execute the specified command"""
        twist = Twist()

        if action == 'move_forward':
            twist.linear.x = 0.5  # Move forward at 0.5 m/s
            self.get_logger().info('Moving forward')
        elif action == 'move_backward':
            twist.linear.x = -0.5  # Move backward at 0.5 m/s
            self.get_logger().info('Moving backward')
        elif action == 'turn_left':
            twist.angular.z = 0.5  # Turn left at 0.5 rad/s
            self.get_logger().info('Turning left')
        elif action == 'turn_right':
            twist.angular.z = -0.5  # Turn right at 0.5 rad/s
            self.get_logger().info('Turning right')
        elif action == 'stop':
            # Stop the robot (all velocities zero)
            self.get_logger().info('Stopping robot')

        # Publish the movement command
        self.movement_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandProcessor()

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

## Configuration and Parameters

For flexibility, you can configure the Whisper node using ROS 2 parameters:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import whisper
import pyaudio
import numpy as np
import threading
import queue
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData

class ConfigurableWhisperNode(Node):
    def __init__(self):
        super().__init__('configurable_whisper_node')

        # Declare parameters
        self.declare_parameter('model_size', 'base')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('chunk_size', 1024)
        self.declare_parameter('language', 'en')

        # Get parameter values
        model_size = self.get_parameter('model_size').get_parameter_value().string_value
        self.rate = self.get_parameter('sample_rate').get_parameter_value().integer_value
        self.chunk = self.get_parameter('chunk_size').get_parameter_value().integer_value
        self.language = self.get_parameter('language').get_parameter_value().string_value

        # Initialize Whisper model
        self.model = whisper.load_model(model_size)

        # Initialize PyAudio
        self.audio = pyaudio.PyAudio()

        # Create publisher for recognized text
        self.text_publisher = self.create_publisher(String, 'voice_commands', 10)

        # Audio buffer for processing
        self.audio_buffer = queue.Queue()

        # Start audio processing thread
        self.processing_thread = threading.Thread(target=self.process_audio, daemon=True)
        self.processing_thread.start()

        self.get_logger().info(f'Configurable Whisper node initialized with model: {model_size}')

    def process_audio(self):
        """Process audio data in a separate thread"""
        # Implementation similar to basic node
        pass

    def destroy_node(self):
        """Clean up resources"""
        self.audio.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ConfigurableWhisperNode()

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

## Testing the Whisper Integration

To test the Whisper integration:

1. Launch the audio capture node:
```bash
ros2 run your_package audio_capture_node
```

2. Launch the Whisper node:
```bash
ros2 run your_package whisper_node
```

3. Launch the command processor:
```bash
ros2 run your_package voice_command_processor
```

4. Speak voice commands to your microphone and observe the output.

## Best Practices

- Use appropriate model size based on your computational resources (tiny/small for real-time, large for accuracy)
- Implement confidence scoring to filter out low-quality transcriptions
- Add noise reduction preprocessing for better performance in noisy environments
- Use appropriate buffer sizes to balance latency and performance
- Implement proper error handling for audio device access

## Summary

This chapter introduced Whisper integration with ROS 2 for voice command processing. You learned how to set up audio capture, process speech with Whisper, and convert voice commands to robotic actions. The next chapter will cover the integration with GPT for more advanced natural language processing capabilities.