---
title: "Multi-modal Interaction: Speech, Vision, Gesture"
sidebar_position: 3
description: "Learn how to integrate speech, vision, and gesture for natural human-robot interaction in humanoid robotics"
---

# Multi-modal Interaction: Speech, Vision, Gesture

## Overview

Multi-modal interaction combines multiple communication channels to create more natural and intuitive human-robot interaction. In humanoid robotics, this involves integrating speech recognition, computer vision, and gesture recognition to understand user intent and respond appropriately. This approach mimics human communication patterns, making robots more accessible and effective in human environments.

## Multi-modal Architecture

The multi-modal interaction system consists of several integrated components:

1. **Speech Processing**: Natural language understanding and speech synthesis
2. **Visual Processing**: Object recognition, face detection, and gesture interpretation
3. **Gesture Recognition**: Hand and body movement interpretation
4. **Fusion Engine**: Combines multiple modalities for coherent understanding
5. **Response Generation**: Generates appropriate robot responses

## Multi-modal Fusion System

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, PoseStamped
from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose
from builtin_interfaces.msg import Time
import numpy as np
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Any
import json

@dataclass
class ModalityInput:
    """
    Data structure for multi-modal input
    """
    timestamp: Time
    modality_type: str  # 'speech', 'vision', 'gesture'
    confidence: float
    data: Any
    source_location: Optional[PointStamped] = None

class MultiModalFusion(Node):
    """
    Multi-modal fusion system combining speech, vision, and gesture
    """

    def __init__(self):
        super().__init__('multi_modal_fusion')

        # Publishers and subscribers
        self.intent_pub = self.create_publisher(String, '/fused_intent', 10)
        self.status_pub = self.create_publisher(String, '/multi_modal_status', 10)

        # Speech input
        self.speech_sub = self.create_subscription(
            String, '/voice_commands', self.speech_callback, 10
        )

        # Vision input
        self.vision_sub = self.create_subscription(
            Detection2DArray, '/object_detections', self.vision_callback, 10
        )

        # Gesture input
        self.gesture_sub = self.create_subscription(
            String, '/gesture_recognition', self.gesture_callback, 10
        )

        # Face detection input
        self.face_sub = self.create_subscription(
            Detection2DArray, '/face_detections', self.face_callback, 10
        )

        # Configuration
        self.fusion_window = self.declare_parameter('fusion_window', 2.0).value  # seconds
        self.confidence_threshold = self.declare_parameter('confidence_threshold', 0.7).value
        self.gaze_tracking_enabled = self.declare_parameter('gaze_tracking_enabled', True).value

        # Input buffers for each modality
        self.speech_buffer = []
        self.vision_buffer = []
        self.gesture_buffer = []
        self.face_buffer = []

        # Timer for periodic fusion
        self.fusion_timer = self.create_timer(0.1, self.fusion_callback)

        # Context tracking
        self.current_context = {
            'user_attention': None,
            'recent_objects': [],
            'conversation_history': [],
            'interaction_mode': 'normal'
        }

        self.get_logger().info("Multi-modal Fusion System initialized")

    def speech_callback(self, msg):
        """
        Handle speech input from voice processing system
        """
        try:
            speech_data = json.loads(msg.data)
            timestamp = self.get_clock().now().to_msg()

            modality_input = ModalityInput(
                timestamp=timestamp,
                modality_type='speech',
                confidence=speech_data.get('confidence', 1.0),
                data=speech_data
            )

            self.speech_buffer.append(modality_input)
            self.cleanup_old_inputs(self.speech_buffer)

            self.get_logger().info(f"Speech input received: {speech_data}")

        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid JSON in speech message: {msg.data}")

    def vision_callback(self, msg):
        """
        Handle vision input from object detection system
        """
        timestamp = self.get_clock().now().to_msg()

        # Convert detections to modality input
        for detection in msg.detections:
            confidence = max([hypothesis.score for hypothesis in detection.results], default=0.0)

            if confidence >= self.confidence_threshold:
                modality_input = ModalityInput(
                    timestamp=timestamp,
                    modality_type='vision',
                    confidence=confidence,
                    data=detection
                )

                self.vision_buffer.append(modality_input)

        self.cleanup_old_inputs(self.vision_buffer)
        self.get_logger().info(f"Vision input received: {len(msg.detections)} detections")

    def gesture_callback(self, msg):
        """
        Handle gesture input from gesture recognition system
        """
        try:
            gesture_data = json.loads(msg.data)
            timestamp = self.get_clock().now().to_msg()

            modality_input = ModalityInput(
                timestamp=timestamp,
                modality_type='gesture',
                confidence=gesture_data.get('confidence', 1.0),
                data=gesture_data
            )

            self.gesture_buffer.append(modality_input)
            self.cleanup_old_inputs(self.gesture_buffer)

            self.get_logger().info(f"Gesture input received: {gesture_data}")

        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid JSON in gesture message: {msg.data}")

    def face_callback(self, msg):
        """
        Handle face detection input for attention tracking
        """
        timestamp = self.get_clock().now().to_msg()

        for detection in msg.detections:
            confidence = max([hypothesis.score for hypothesis in detection.results], default=0.0)

            if confidence >= self.confidence_threshold:
                # Extract face position for attention tracking
                bbox = detection.bbox
                center_x = bbox.center.position.x
                center_y = bbox.center.position.y

                face_point = PointStamped()
                face_point.header = detection.header
                face_point.point.x = center_x
                face_point.point.y = center_y
                face_point.point.z = 0.0  # Approximate depth

                modality_input = ModalityInput(
                    timestamp=timestamp,
                    modality_type='face',
                    confidence=confidence,
                    data=detection,
                    source_location=face_point
                )

                self.face_buffer.append(modality_input)

        self.cleanup_old_inputs(self.face_buffer)
        self.get_logger().info(f"Face input received: {len(msg.detections)} faces")

    def cleanup_old_inputs(self, buffer_list):
        """
        Remove inputs older than the fusion window
        """
        current_time = self.get_clock().now().nanoseconds
        window_ns = int(self.fusion_window * 1e9)

        # Remove old inputs
        buffer_list[:] = [
            item for item in buffer_list
            if (current_time - rclpy.time.Time.from_msg(item.timestamp).nanoseconds) < window_ns
        ]

    def fusion_callback(self):
        """
        Perform multi-modal fusion to generate coherent intent
        """
        # Get current inputs from all modalities
        current_speech = self.speech_buffer.copy()
        current_vision = self.vision_buffer.copy()
        current_gesture = self.gesture_buffer.copy()
        current_face = self.face_buffer.copy()

        if not any([current_speech, current_vision, current_gesture, current_face]):
            return

        # Perform fusion to generate intent
        fused_intent = self.perform_fusion(
            current_speech, current_vision, current_gesture, current_face
        )

        if fused_intent:
            # Publish the fused intent
            intent_msg = String()
            intent_msg.data = json.dumps(fused_intent)
            self.intent_pub.publish(intent_msg)

            self.get_logger().info(f"Fused intent published: {fused_intent}")

            # Update context
            self.update_context(fused_intent)

    def perform_fusion(self, speech_inputs, vision_inputs, gesture_inputs, face_inputs):
        """
        Perform multi-modal fusion to generate coherent understanding
        """
        # Initialize intent structure
        intent = {
            'timestamp': self.get_clock().now().to_msg(),
            'confidence': 0.0,
            'action': None,
            'targets': [],
            'context': {},
            'modality_weights': {}
        }

        # Analyze speech inputs
        speech_intent = self.analyze_speech(speech_inputs)

        # Analyze vision inputs
        vision_targets = self.analyze_vision(vision_inputs)

        # Analyze gesture inputs
        gesture_intent = self.analyze_gesture(gesture_inputs)

        # Analyze face inputs for attention
        attention_target = self.analyze_face(face_inputs)

        # Combine modalities based on confidence and context
        combined_confidence = 0.0
        weights = {}

        if speech_intent:
            weights['speech'] = speech_intent.get('confidence', 0.8)
            combined_confidence += weights['speech'] * 0.4  # 40% weight for speech
            intent['action'] = speech_intent.get('action')
            intent['targets'].extend(speech_intent.get('targets', []))

        if vision_targets:
            weights['vision'] = max([t.get('confidence', 0.0) for t in vision_targets], default=0.6)
            combined_confidence += weights['vision'] * 0.3  # 30% weight for vision
            intent['targets'].extend(vision_targets)

        if gesture_intent:
            weights['gesture'] = gesture_intent.get('confidence', 0.7)
            combined_confidence += weights['gesture'] * 0.2  # 20% weight for gesture
            if not intent['action']:  # Only use gesture action if speech didn't provide one
                intent['action'] = gesture_intent.get('action')

        if attention_target:
            weights['attention'] = attention_target.get('confidence', 0.9)
            combined_confidence += weights['attention'] * 0.1  # 10% weight for attention
            intent['context']['attention_target'] = attention_target

        # Normalize confidence
        intent['confidence'] = min(combined_confidence, 1.0)
        intent['modality_weights'] = weights

        # Apply context to refine intent
        refined_intent = self.apply_context(intent)

        return refined_intent

    def analyze_speech(self, speech_inputs):
        """
        Analyze speech inputs to extract intent
        """
        if not speech_inputs:
            return None

        # Get the most recent high-confidence speech input
        high_conf_inputs = [inp for inp in speech_inputs if inp.confidence >= self.confidence_threshold]

        if not high_conf_inputs:
            return None

        # Use the most recent input
        latest_input = max(high_conf_inputs, key=lambda x:
            rclpy.time.Time.from_msg(x.timestamp).nanoseconds)

        # Extract action and targets from speech data
        speech_data = latest_input.data
        action = speech_data.get('action')
        original_text = speech_data.get('original_text', '')

        # Extract targets from text if not provided in structured data
        targets = speech_data.get('targets', [])
        if not targets and original_text:
            targets = self.extract_targets_from_text(original_text)

        return {
            'action': action,
            'targets': targets,
            'confidence': latest_input.confidence,
            'original_text': original_text
        }

    def analyze_vision(self, vision_inputs):
        """
        Analyze vision inputs to identify relevant targets
        """
        if not vision_inputs:
            return []

        targets = []
        for vision_input in vision_inputs:
            if vision_input.confidence >= self.confidence_threshold:
                detection = vision_input.data
                # Extract object information
                for result in detection.results:
                    class_id = result.hypothesis.class_id
                    confidence = result.hypothesis.score

                    target = {
                        'type': class_id,
                        'confidence': confidence,
                        'position': detection.bbox.center.position,
                        'modality': 'vision'
                    }
                    targets.append(target)

        return targets

    def analyze_gesture(self, gesture_inputs):
        """
        Analyze gesture inputs to extract intent
        """
        if not gesture_inputs:
            return None

        # Get the most recent high-confidence gesture
        high_conf_inputs = [inp for inp in gesture_inputs if inp.confidence >= self.confidence_threshold]

        if not high_conf_inputs:
            return None

        latest_input = max(high_conf_inputs, key=lambda x:
            rclpy.time.Time.from_msg(x.timestamp).nanoseconds)

        gesture_data = latest_input.data
        gesture_type = gesture_data.get('gesture_type', '')

        # Map gestures to actions
        gesture_to_action = {
            'pointing': 'navigate_to_location',
            'beckoning': 'come_here',
            'waving': 'greet',
            'thumbs_up': 'confirm',
            'thumbs_down': 'reject'
        }

        action = gesture_to_action.get(gesture_type)

        return {
            'action': action,
            'gesture_type': gesture_type,
            'confidence': latest_input.confidence
        }

    def analyze_face(self, face_inputs):
        """
        Analyze face inputs to determine attention focus
        """
        if not face_inputs:
            return None

        # Get the most confident face detection
        high_conf_inputs = [inp for inp in face_inputs if inp.confidence >= self.confidence_threshold]

        if not high_conf_inputs:
            return None

        # Return the most confident face as attention target
        best_face = max(high_conf_inputs, key=lambda x: x.confidence)

        return {
            'type': 'person',
            'confidence': best_face.confidence,
            'position': best_face.source_location.point if best_face.source_location else None,
            'modality': 'face'
        }

    def extract_targets_from_text(self, text):
        """
        Extract potential targets from natural language text
        """
        text_lower = text.lower()
        targets = []

        # Define common object categories
        objects = ['cup', 'bottle', 'book', 'phone', 'keys', 'water', 'coffee', 'apple', 'banana']
        locations = ['kitchen', 'living room', 'bedroom', 'office', 'dining room', 'bathroom']

        # Extract objects
        for obj in objects:
            if obj in text_lower:
                targets.append({
                    'type': 'object',
                    'name': obj,
                    'modality': 'speech',
                    'confidence': 0.8
                })

        # Extract locations
        for loc in locations:
            if loc in text_lower:
                targets.append({
                    'type': 'location',
                    'name': loc,
                    'modality': 'speech',
                    'confidence': 0.8
                })

        return targets

    def apply_context(self, intent):
        """
        Apply contextual information to refine the intent
        """
        # Use attention target to disambiguate references
        attention_target = intent['context'].get('attention_target')
        if attention_target and intent['targets']:
            # If user is looking at something, prioritize that target
            for target in intent['targets']:
                if target.get('type') == 'person':
                    target['confidence'] = max(target['confidence'], attention_target['confidence'])

        # Apply conversation history to understand references
        recent_objects = self.current_context.get('recent_objects', [])
        if recent_objects and 'that' in intent.get('original_text', '').lower():
            # 'that' likely refers to the most recently mentioned object
            if recent_objects:
                recent_obj = recent_objects[-1]
                intent['targets'].append(recent_obj)

        return intent

    def update_context(self, intent):
        """
        Update the interaction context based on the intent
        """
        # Update attention target
        if 'attention_target' in intent['context']:
            self.current_context['user_attention'] = intent['context']['attention_target']

        # Update recent objects
        for target in intent.get('targets', []):
            if target.get('type') == 'object':
                self.current_context['recent_objects'].append(target)
                # Keep only recent objects (limit to 5)
                if len(self.current_context['recent_objects']) > 5:
                    self.current_context['recent_objects'] = self.current_context['recent_objects'][-5:]

        # Update conversation history
        self.current_context['conversation_history'].append(intent)
        # Keep only recent history (limit to 10)
        if len(self.current_context['conversation_history']) > 10:
            self.current_context['conversation_history'] = self.current_context['conversation_history'][-10:]

    def publish_status(self, status: str):
        """
        Publish status message
        """
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)


class MultiModalInteractionManager(Node):
    """
    Manager for coordinating multi-modal interaction components
    """

    def __init__(self):
        super().__init__('multi_modal_interaction_manager')

        # Create the fusion system
        self.fusion_system = MultiModalFusion()

        # Publishers for robot responses
        self.speech_pub = self.create_publisher(String, '/robot_speech', 10)
        self.gesture_pub = self.create_publisher(String, '/robot_gestures', 10)
        self.animation_pub = self.create_publisher(String, '/robot_animations', 10)

        # Subscriber for fused intents
        self.intent_sub = self.create_subscription(
            String, '/fused_intent', self.intent_callback, 10
        )

        self.get_logger().info("Multi-modal Interaction Manager initialized")

    def intent_callback(self, msg):
        """
        Handle fused intent and generate appropriate robot response
        """
        try:
            intent_data = json.loads(msg.data)

            # Generate response based on intent
            response = self.generate_response(intent_data)

            if response:
                self.execute_response(response)

        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid JSON in intent message: {msg.data}")

    def generate_response(self, intent_data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Generate robot response based on fused intent
        """
        action = intent_data.get('action')
        confidence = intent_data.get('confidence', 0.0)

        if confidence < 0.5:
            # Confidence too low, ask for clarification
            return {
                'speech': "I'm not sure I understood correctly. Could you please repeat?",
                'gesture': 'confused',
                'animation': 'thinking'
            }

        if action == 'navigate':
            target = intent_data.get('targets', [{}])[0] if intent_data.get('targets') else {}
            location = target.get('name', 'unknown location')
            return {
                'speech': f"Okay, I'll go to the {location}.",
                'gesture': 'acknowledging',
                'animation': 'walking',
                'action': 'navigate',
                'target': location
            }

        elif action == 'fetch_object':
            target = intent_data.get('targets', [{}])[0] if intent_data.get('targets') else {}
            obj_name = target.get('name', 'unknown object')
            return {
                'speech': f"Okay, I'll get the {obj_name} for you.",
                'gesture': 'reaching',
                'animation': 'fetching',
                'action': 'fetch',
                'target': obj_name
            }

        elif action == 'greet':
            return {
                'speech': "Hello! How can I help you today?",
                'gesture': 'waving',
                'animation': 'greeting'
            }

        elif action == 'come_here':
            return {
                'speech': "I'm coming over now.",
                'gesture': 'walking',
                'animation': 'approaching'
            }

        elif action == 'confirm':
            return {
                'speech': "Got it, I understand.",
                'gesture': 'nodding',
                'animation': 'confirming'
            }

        elif action == 'reject':
            return {
                'speech': "I understand you disagree.",
                'gesture': 'shrugging',
                'animation': 'acknowledging'
            }

        else:
            # Default response for unknown actions
            return {
                'speech': f"I can help with that. What specifically would you like me to do?",
                'gesture': 'inquiring',
                'animation': 'listening'
            }

    def execute_response(self, response: Dict[str, Any]):
        """
        Execute the generated response
        """
        # Publish speech response
        if 'speech' in response:
            speech_msg = String()
            speech_msg.data = response['speech']
            self.speech_pub.publish(speech_msg)

        # Publish gesture response
        if 'gesture' in response:
            gesture_msg = String()
            gesture_msg.data = json.dumps({
                'gesture_type': response['gesture'],
                'timestamp': self.get_clock().now().to_msg()
            })
            self.gesture_pub.publish(gesture_msg)

        # Publish animation response
        if 'animation' in response:
            animation_msg = String()
            animation_msg.data = json.dumps({
                'animation_type': response['animation'],
                'timestamp': self.get_clock().now().to_msg()
            })
            self.animation_pub.publish(animation_msg)

        self.get_logger().info(f"Response executed: {response}")


def main(args=None):
    rclpy.init(args=args)

    # Create both nodes
    fusion_node = MultiModalFusion()
    manager_node = MultiModalInteractionManager()

    # Create executor and add nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(fusion_node)
    executor.add_node(manager_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        fusion_node.get_logger().info("Multi-modal system stopped by user")
        manager_node.get_logger().info("Interaction manager stopped by user")
    finally:
        fusion_node.destroy_node()
        manager_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Attention and Gaze Tracking

For natural interaction, the system implements attention and gaze tracking:

1. **Face Detection**: Identifies when users are looking at the robot
2. **Gaze Estimation**: Determines where users are looking in the environment
3. **Joint Attention**: Coordinates robot attention with user attention
4. **Social Gaze**: Implements appropriate gaze behavior during interaction

## Contextual Understanding

The system maintains contextual information:

1. **Conversation History**: Tracks the flow of interaction
2. **Object References**: Understands pronouns and references to previously mentioned objects
3. **Spatial Context**: Maintains awareness of object locations and relationships
4. **Temporal Context**: Understands timing and sequence of events

## Integration with Robot Systems

Multi-modal interaction integrates with:

1. **Navigation System**: For movement based on gesture or speech commands
2. **Manipulation System**: For object interaction based on visual and speech input
3. **Perception System**: For continuous environment awareness
4. **Speech System**: For natural language interaction

## Performance Considerations

When implementing multi-modal interaction:

1. **Latency**: Minimize processing delays for natural interaction
2. **Synchronization**: Align inputs from different modalities in time
3. **Resource Usage**: Balance computational requirements with real-time performance
4. **Robustness**: Handle missing or noisy input from individual modalities

## Privacy and Social Considerations

Multi-modal systems must consider:

1. **Privacy**: Protect user data captured through cameras and microphones
2. **Social Norms**: Implement culturally appropriate interaction patterns
3. **Accessibility**: Support users with different abilities and communication styles
4. **Trust**: Build user confidence through consistent and predictable behavior

## Summary

Multi-modal interaction creates more natural and intuitive human-robot communication by combining speech, vision, and gesture. Through effective fusion of multiple modalities and contextual understanding, humanoid robots can engage in more human-like interactions that feel natural and responsive.

## Next Steps

This completes Module 4 on Vision-Language-Action systems. In the next module, we'll integrate all these capabilities in a comprehensive autonomous humanoid capstone project.