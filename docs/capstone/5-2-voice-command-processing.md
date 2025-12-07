---
title: Voice Command Processing
sidebar_position: 2
description: Implementing voice command processing for the autonomous humanoid robot
---

# Voice Command Processing

## Overview

Voice command processing is the primary interface for human-robot interaction in our autonomous humanoid system. This component converts natural language spoken commands into actionable instructions that can be processed by the robot's task planning system.

## Architecture

The voice command processing system consists of several components working together:

1. **Audio Input**: Capturing voice commands from the environment
2. **Speech Recognition**: Converting speech to text using OpenAI Whisper
3. **Intent Parsing**: Extracting actionable intents from natural language
4. **Command Validation**: Ensuring commands are safe and executable
5. **Task Translation**: Converting high-level commands to executable actions

## Implementation

Here's the implementation of the voice command processing system:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
import openai
import json
import threading
import pyaudio
import wave
import time

class VoiceCommandProcessor(Node):
    """
    Voice command processor for the autonomous humanoid robot
    """

    def __init__(self):
        super().__init__('voice_command_processor')

        # Publishers
        self.command_pub = self.create_publisher(String, '/robot_commands', 10)
        self.status_pub = self.create_publisher(String, '/voice_status', 10)

        # Subscribers
        self.audio_sub = self.create_subscription(
            AudioData,
            '/audio_input',
            self.audio_callback,
            10
        )

        # Configuration
        self.whisper_api_key = self.declare_parameter('whisper_api_key', '').value
        self.active_listening = False
        self.command_history = []

        # Initialize Whisper client
        if self.whisper_api_key:
            openai.api_key = self.whisper_api_key
        else:
            self.get_logger().warn("No Whisper API key provided, using simulation mode")

        # Start listening thread
        self.listening_thread = threading.Thread(target=self.listen_for_wake_word)
        self.listening_thread.daemon = True
        self.listening_thread.start()

        self.get_logger().info("Voice Command Processor initialized")

    def audio_callback(self, msg):
        """
        Handle incoming audio data
        """
        if self.active_listening:
            # Process audio for speech recognition
            try:
                # In a real implementation, we would process the audio data
                # For this example, we'll simulate recognition
                recognized_text = self.simulate_speech_recognition(msg)
                if recognized_text:
                    self.process_command(recognized_text)
            except Exception as e:
                self.get_logger().error(f"Error processing audio: {e}")

    def simulate_speech_recognition(self, audio_msg):
        """
        Simulate speech recognition (in real implementation, this would use Whisper API)
        """
        # This is a placeholder - in reality, we would convert audio_msg to text
        # using the Whisper API or a local speech recognition model
        return "move to the kitchen and bring me a red cup"

    def process_command(self, text):
        """
        Process the recognized text command
        """
        self.get_logger().info(f"Recognized command: {text}")

        # Update status
        status_msg = String()
        status_msg.data = f"Processing: {text}"
        self.status_pub.publish(status_msg)

        # Parse intent from text
        intent = self.parse_intent(text)

        if intent:
            # Validate command
            if self.validate_command(intent):
                # Publish command to robot
                cmd_msg = String()
                cmd_msg.data = json.dumps(intent)
                self.command_pub.publish(cmd_msg)

                self.get_logger().info(f"Command published: {intent}")
                self.command_history.append({
                    'text': text,
                    'intent': intent,
                    'timestamp': time.time()
                })
            else:
                self.get_logger().warn(f"Invalid command: {intent}")
        else:
            self.get_logger().warn(f"Could not parse intent from: {text}")

    def parse_intent(self, text):
        """
        Parse intent from natural language text
        """
        # Simple keyword-based parsing (in practice, use NLP models)
        text_lower = text.lower()

        # Define command patterns
        if 'move to' in text_lower or 'go to' in text_lower:
            # Extract location
            for location in ['kitchen', 'living room', 'bedroom', 'office', 'dining room']:
                if location in text_lower:
                    return {
                        'action': 'navigate',
                        'target': location,
                        'original_text': text
                    }

        elif 'bring' in text_lower or 'fetch' in text_lower or 'get' in text_lower:
            # Extract object to fetch
            object_keywords = ['cup', 'bottle', 'book', 'phone', 'keys', 'water']
            for obj in object_keywords:
                if obj in text_lower:
                    # Extract color if mentioned
                    color_keywords = ['red', 'blue', 'green', 'yellow', 'black', 'white']
                    color = None
                    for color_word in color_keywords:
                        if color_word in text_lower:
                            color = color_word
                            break

                    return {
                        'action': 'fetch_object',
                        'object': obj,
                        'color': color,
                        'original_text': text
                    }

        elif 'stop' in text_lower or 'halt' in text_lower:
            return {
                'action': 'stop',
                'original_text': text
            }

        elif 'wave' in text_lower or 'hello' in text_lower:
            return {
                'action': 'greet',
                'original_text': text
            }

        # If no pattern matched, return None
        return None

    def validate_command(self, intent):
        """
        Validate that the command is safe and executable
        """
        # Check if action is in allowed list
        allowed_actions = ['navigate', 'fetch_object', 'stop', 'greet']
        if intent['action'] not in allowed_actions:
            return False

        # Additional validation can be added here
        # For example, check if target location is in map
        # Or verify object is recognizable

        return True

    def listen_for_wake_word(self):
        """
        Listen for wake word to activate command processing
        """
        # This would implement wake word detection
        # For now, we'll just set active listening to True after a delay
        time.sleep(2)  # Simulate initialization
        self.active_listening = True
        self.get_logger().info("Voice command processor activated - listening for commands")


def main(args=None):
    rclpy.init(args=args)
    processor = VoiceCommandProcessor()

    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        processor.get_logger().info("Voice command processor stopped by user")
    finally:
        processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Integration with Task Planning

The voice command processor integrates with the task planning system by publishing parsed commands to the `/robot_commands` topic. The task planner subscribes to this topic and converts high-level commands into executable action sequences.

## Safety Considerations

When implementing voice command processing, several safety considerations must be addressed:

1. **Command Validation**: All commands must be validated to ensure they're safe and appropriate
2. **Authorization**: Implement voice recognition to ensure only authorized users can control the robot
3. **Fail-Safe Mechanisms**: Include emergency stop functionality
4. **Privacy**: Protect user privacy when processing voice data

## Performance Optimization

To ensure responsive voice command processing:

1. **Wake Word Detection**: Use efficient algorithms to detect when the robot should listen
2. **Latency Reduction**: Minimize processing time between command and execution
3. **Error Handling**: Gracefully handle recognition errors and ambiguous commands
4. **Context Awareness**: Use context to improve command interpretation accuracy

## Testing Voice Commands

The voice command system should be tested with various scenarios:

- Different accents and speaking patterns
- Noisy environments
- Complex multi-step commands
- Ambiguous or unclear commands
- Emergency stop commands

## Next Steps

In the next section, we'll implement navigation and obstacle handling to execute the movement commands generated by the voice processing system.