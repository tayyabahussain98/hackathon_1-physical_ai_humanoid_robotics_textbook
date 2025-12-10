---
title: "Voice-to-Action with OpenAI Whisper"
sidebar_position: 1
description: "Learn how to convert voice commands to robot actions using OpenAI Whisper for speech recognition"
---

# Voice-to-Action with OpenAI Whisper

## Overview

The Vision-Language-Action (VLA) paradigm enables robots to understand natural language commands and execute corresponding physical actions. This section focuses on the voice processing component, specifically using OpenAI Whisper for speech recognition and command interpretation.

## Architecture of Voice-to-Action System

The voice-to-action system consists of several components working together:

1. **Audio Input**: Capturing voice commands from the environment
2. **Speech Recognition**: Converting speech to text using Whisper
3. **Intent Parsing**: Extracting actionable intents from recognized text
4. **Action Mapping**: Converting intents to robot actions
5. **Execution**: Executing actions through the robot's control system

## OpenAI Whisper Integration

OpenAI Whisper is an excellent choice for robotic applications due to its robustness in various acoustic conditions:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
import openai
import threading
import pyaudio
import wave
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import json

class WhisperVoiceProcessor(Node):
    """
    Voice processor using OpenAI Whisper for speech recognition in robotics
    """

    def __init__(self):
        super().__init__('whisper_voice_processor')

        # Publishers and subscribers
        self.voice_cmd_pub = self.create_publisher(String, '/voice_commands', 10)
        self.status_pub = self.create_publisher(String, '/voice_processor_status', 10)

        # Configuration
        self.whisper_api_key = self.declare_parameter('whisper_api_key', '').value
        self.wake_word = self.declare_parameter('wake_word', 'robot').value
        self.active_listening = False

        # Initialize OpenAI client
        if self.whisper_api_key:
            openai.api_key = self.whisper_api_key
        else:
            self.get_logger().warn("No Whisper API key provided, using simulation mode")

        # Audio processing parameters
        self.audio_buffer = []
        self.buffer_size = 44100 * 3  # 3 seconds of audio at 44.1kHz
        self.silence_threshold = 500  # Threshold for voice activity detection

        # Initialize audio input
        self.audio_input_active = False
        self.audio_thread = threading.Thread(target=self.audio_input_loop)
        self.audio_thread.daemon = True
        self.audio_thread.start()

        # Timer for periodic processing
        self.process_timer = self.create_timer(0.1, self.process_audio_buffer)

        self.get_logger().info("Whisper Voice Processor initialized")

    def audio_input_loop(self):
        """
        Continuous audio input loop
        """
        # This is a simplified version - in practice, you'd use a library like PyAudio
        # to capture audio from the robot's microphones
        pass

    def process_audio_buffer(self):
        """
        Process the audio buffer for voice activity and wake word detection
        """
        if not self.audio_buffer:
            return

        # Perform voice activity detection
        if self.is_voice_activity_present(self.audio_buffer):
            if not self.active_listening:
                # Check for wake word
                if self.detect_wake_word(self.audio_buffer):
                    self.active_listening = True
                    self.get_logger().info("Wake word detected, starting voice command processing")
                    self.process_voice_command()

    def is_voice_activity_present(self, audio_data):
        """
        Simple voice activity detection based on amplitude
        """
        if len(audio_data) == 0:
            return False

        # Calculate average amplitude
        amplitude = np.mean(np.abs(audio_data))
        return amplitude > self.silence_threshold

    def detect_wake_word(self, audio_data):
        """
        Detect wake word in audio data (simplified for this example)
        """
        # In practice, this would involve audio pattern matching
        # or a dedicated wake word detection system
        return self.wake_word.lower() in self.wake_word.lower()  # Simplified

    def process_voice_command(self):
        """
        Process voice command using Whisper API
        """
        try:
            # In a real implementation, we would send the audio data to Whisper
            # For this example, we'll simulate the transcription
            recognized_text = self.simulate_whisper_transcription()

            if recognized_text:
                self.get_logger().info(f"Recognized: {recognized_text}")
                self.parse_and_execute_command(recognized_text)

            self.active_listening = False

        except Exception as e:
            self.get_logger().error(f"Error processing voice command: {e}")
            self.active_listening = False

    def simulate_whisper_transcription(self):
        """
        Simulate Whisper transcription (in real implementation, this would call the API)
        """
        # This is a placeholder - in reality, you would send audio data to Whisper API
        # and receive transcribed text
        return "move to the kitchen and bring me a red cup"

    def parse_and_execute_command(self, text):
        """
        Parse recognized text and execute corresponding action
        """
        # Update status
        status_msg = String()
        status_msg.data = f"Processing: {text}"
        self.status_pub.publish(status_msg)

        # Parse the command
        command_intent = self.parse_command_intent(text)

        if command_intent:
            # Publish command for execution
            cmd_msg = String()
            cmd_msg.data = json.dumps(command_intent)
            self.voice_cmd_pub.publish(cmd_msg)

            self.get_logger().info(f"Command published: {command_intent}")
        else:
            self.get_logger().warn(f"Could not parse command from: {text}")

    def parse_command_intent(self, text):
        """
        Parse intent from natural language text
        """
        text_lower = text.lower()

        # Define command patterns
        if any(word in text_lower for word in ['move to', 'go to', 'navigate to', 'walk to']):
            # Extract location
            locations = ['kitchen', 'living room', 'bedroom', 'office', 'dining room', 'bathroom']
            for location in locations:
                if location in text_lower:
                    return {
                        'action': 'navigate',
                        'target': location,
                        'original_text': text
                    }

        elif any(word in text_lower for word in ['bring', 'fetch', 'get', 'pick up']):
            # Extract object to fetch
            objects = ['cup', 'bottle', 'book', 'phone', 'keys', 'water', 'coffee']
            colors = ['red', 'blue', 'green', 'yellow', 'black', 'white']

            for obj in objects:
                if obj in text_lower:
                    # Extract color if mentioned
                    for color in colors:
                        if color in text_lower:
                            return {
                                'action': 'fetch_object',
                                'object': obj,
                                'color': color,
                                'original_text': text
                            }
                    return {
                        'action': 'fetch_object',
                        'object': obj,
                        'original_text': text
                    }

        elif any(word in text_lower for word in ['stop', 'halt', 'pause']):
            return {
                'action': 'stop',
                'original_text': text
            }

        elif any(word in text_lower for word in ['wave', 'hello', 'greet']):
            return {
                'action': 'greet',
                'original_text': text
            }

        # If no pattern matched, return None
        return None


def main(args=None):
    rclpy.init(args=args)
    processor = WhisperVoiceProcessor()

    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        processor.get_logger().info("Whisper voice processor stopped by user")
    finally:
        processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Whisper API Configuration

For production use, configure Whisper with appropriate parameters:

```yaml
# Example Whisper configuration
whisper_voice_processor:
  ros__parameters:
    whisper_api_key: "your-api-key-here"
    wake_word: "robot"
    model: "whisper-1"  # or local model name
    language: "en"
    response_format: "json"
    temperature: 0.0
    vad_enabled: true  # Voice activity detection
    wake_word_timeout: 5.0  # Seconds to wait for command after wake word
    silence_threshold: 500
```

## Performance Optimization

When using Whisper for robotics applications:

1. **Local Models**: Consider using local Whisper models for reduced latency
2. **Audio Preprocessing**: Apply noise reduction for better recognition
3. **Contextual Prompts**: Use contextual prompts to improve accuracy
4. **Edge Computing**: Deploy on edge devices for real-time performance

## Privacy and Security

When processing voice data:

1. **Data Encryption**: Encrypt voice data in transit
2. **Local Processing**: Prefer local processing when possible
3. **Consent**: Ensure user consent for voice data processing
4. **Data Retention**: Implement appropriate data retention policies

## Integration with Robot Systems

The voice processing system integrates with the robot's:

1. **Navigation System**: For movement commands
2. **Manipulation System**: For object interaction commands
3. **Perception System**: For object recognition and location commands
4. **Behavior System**: For high-level task execution

## Error Handling

The system should handle:

1. **Recognition Errors**: Unclear or misrecognized commands
2. **Network Issues**: Connectivity problems with Whisper API
3. **Ambient Noise**: Background noise affecting recognition
4. **Wake Word Confusion**: False wake word detections

## Summary

OpenAI Whisper provides a robust foundation for voice command processing in humanoid robotics. By properly integrating Whisper with robot control systems, we can create intuitive human-robot interaction experiences that allow natural language commands to be converted into robot actions.

## Next Steps

In the next section, we'll explore cognitive planning to translate language commands into sequences of ROS 2 actions.