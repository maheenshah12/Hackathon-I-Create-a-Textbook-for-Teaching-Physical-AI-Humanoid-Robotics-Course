#!/usr/bin/env python3
"""
Audio Capture Node for VLA Pipeline

This node captures audio from a microphone and publishes it to the VLA pipeline.
"""

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