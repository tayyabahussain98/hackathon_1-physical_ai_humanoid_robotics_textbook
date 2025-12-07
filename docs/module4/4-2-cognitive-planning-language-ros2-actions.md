---
title: "Cognitive Planning: Language to ROS 2 Actions"
sidebar_position: 2
description: "Learn how to translate natural language commands into sequences of ROS 2 actions for humanoid robots"
---

# Cognitive Planning: Language to ROS 2 Actions

## Overview

Cognitive planning bridges the gap between natural language understanding and robot action execution. This section explores how to translate high-level language commands into concrete sequences of ROS 2 actions that can be executed by humanoid robots. The cognitive planning system acts as an intelligent intermediary that interprets user intent and generates executable action plans.

## Architecture of Cognitive Planning System

The cognitive planning system consists of several interconnected components:

1. **Language Parser**: Interprets natural language commands and extracts semantic meaning
2. **Action Planner**: Generates sequences of atomic actions based on parsed intent
3. **Knowledge Base**: Stores world models, object properties, and action constraints
4. **Action Executor**: Coordinates ROS 2 action execution with robot systems
5. **Feedback Handler**: Monitors execution and handles plan failures or updates

## Natural Language Understanding

The cognitive planning system begins with understanding natural language commands. For humanoid robotics, this involves:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from move_base_msgs.action import MoveBase
from manipulation_msgs.action import PickObject, PlaceObject
import json
import re
from typing import List, Dict, Any
import openai

class CognitivePlanner(Node):
    """
    Cognitive planner that translates natural language to ROS 2 action sequences
    """

    def __init__(self):
        super().__init__('cognitive_planner')

        # Publishers and subscribers
        self.plan_status_pub = self.create_publisher(String, '/cognitive_plan_status', 10)
        self.command_sub = self.create_subscription(
            String, '/voice_commands', self.command_callback, 10
        )

        # Action clients for different robot capabilities
        self.nav_client = ActionClient(self, MoveBase, 'move_base')
        self.manipulation_client = ActionClient(self, PickObject, 'pick_object')
        self.place_client = ActionClient(self, PlaceObject, 'place_object')

        # Configuration
        self.openai_api_key = self.declare_parameter('openai_api_key', '').value
        if self.openai_api_key:
            openai.api_key = self.openai_api_key

        # Knowledge base for object properties and locations
        self.knowledge_base = self.initialize_knowledge_base()

        # Current plan tracking
        self.current_plan = []
        self.plan_index = 0

        self.get_logger().info("Cognitive Planner initialized")

    def initialize_knowledge_base(self) -> Dict[str, Any]:
        """
        Initialize the knowledge base with known objects, locations, and capabilities
        """
        return {
            'locations': {
                'kitchen': {'x': 1.0, 'y': 2.0, 'theta': 0.0},
                'living_room': {'x': -2.0, 'y': 1.5, 'theta': 1.57},
                'bedroom': {'x': 3.0, 'y': -1.0, 'theta': 3.14},
                'office': {'x': -1.0, 'y': -2.0, 'theta': -1.57},
                'dining_room': {'x': 0.0, 'y': 3.0, 'theta': 0.0},
                'bathroom': {'x': 2.5, 'y': 0.5, 'theta': 0.0}
            },
            'objects': {
                'cup': {'graspable': True, 'movable': True, 'color': ['red', 'blue', 'green', 'white']},
                'bottle': {'graspable': True, 'movable': True, 'color': ['clear', 'green', 'brown']},
                'book': {'graspable': True, 'movable': True, 'color': ['red', 'blue', 'black', 'brown']},
                'phone': {'graspable': True, 'movable': True, 'color': ['black', 'white', 'silver']},
                'keys': {'graspable': True, 'movable': True, 'color': ['metallic']},
                'water': {'movable': True, 'container': 'bottle', 'color': ['clear']},
                'coffee': {'movable': True, 'container': 'cup', 'color': ['brown', 'black']}
            },
            'capabilities': {
                'navigation': True,
                'manipulation': True,
                'grasping': True,
                'object_recognition': True,
                'speech': True
            }
        }

    def command_callback(self, msg):
        """
        Handle incoming voice commands and generate action plans
        """
        try:
            command_data = json.loads(msg.data)
            self.get_logger().info(f"Processing command: {command_data}")

            # Generate plan based on command intent
            plan = self.generate_action_plan(command_data)

            if plan:
                self.execute_plan(plan)
            else:
                self.get_logger().warn(f"Could not generate plan for command: {command_data}")
                self.publish_status(f"Failed to generate plan for: {command_data.get('original_text', '')}")

        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid JSON in command: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Error processing command: {e}")

    def generate_action_plan(self, command_data: Dict[str, Any]) -> List[Dict[str, Any]]:
        """
        Generate a sequence of actions based on the command intent
        """
        action = command_data.get('action', '')
        original_text = command_data.get('original_text', '')

        if action == 'navigate':
            target_location = command_data.get('target', '')
            return self.plan_navigation(target_location)

        elif action == 'fetch_object':
            obj = command_data.get('object', '')
            color = command_data.get('color', '')
            return self.plan_fetch_object(obj, color)

        elif action == 'stop':
            return [{'action_type': 'stop', 'description': 'Stop current execution'}]

        elif action == 'greet':
            return [{'action_type': 'greet', 'description': 'Perform greeting action'}]

        else:
            self.get_logger().warn(f"Unknown action type: {action}")
            return []

    def plan_navigation(self, location: str) -> List[Dict[str, Any]]:
        """
        Plan navigation actions to reach a specified location
        """
        if location not in self.knowledge_base['locations']:
            self.get_logger().warn(f"Unknown location: {location}")
            return []

        location_data = self.knowledge_base['locations'][location]

        plan = [
            {
                'action_type': 'navigate',
                'target_pose': {
                    'x': location_data['x'],
                    'y': location_data['y'],
                    'theta': location_data['theta']
                },
                'description': f'Navigate to {location}',
                'timeout': 60.0
            }
        ]

        return plan

    def plan_fetch_object(self, obj: str, color: str = '') -> List[Dict[str, Any]]:
        """
        Plan actions to fetch an object (navigate, recognize, grasp, return)
        """
        if obj not in self.knowledge_base['objects']:
            self.get_logger().warn(f"Unknown object: {obj}")
            return []

        obj_data = self.knowledge_base['objects'][obj]
        if not obj_data.get('graspable', False):
            self.get_logger().warn(f"Object {obj} is not graspable")
            return []

        plan = [
            {
                'action_type': 'find_object',
                'object_type': obj,
                'color': color,
                'description': f'Locate {color} {obj}' if color else f'Locate {obj}',
                'timeout': 30.0
            },
            {
                'action_type': 'navigate_to_object',
                'description': f'Navigate to {color} {obj}' if color else f'Navigate to {obj}',
                'timeout': 60.0
            },
            {
                'action_type': 'grasp_object',
                'object_type': obj,
                'color': color,
                'description': f'Grasp {color} {obj}' if color else f'Grasp {obj}',
                'timeout': 30.0
            },
            {
                'action_type': 'return_to_user',
                'description': 'Return to user location',
                'timeout': 60.0
            }
        ]

        return plan

    def execute_plan(self, plan: List[Dict[str, Any]]):
        """
        Execute a sequence of actions
        """
        self.current_plan = plan
        self.plan_index = 0

        if plan:
            self.publish_status(f"Starting plan execution with {len(plan)} actions")
            self.execute_next_action()
        else:
            self.publish_status("No plan to execute")

    def execute_next_action(self):
        """
        Execute the next action in the current plan
        """
        if self.plan_index >= len(self.current_plan):
            self.publish_status("Plan execution completed")
            return

        action = self.current_plan[self.plan_index]
        self.get_logger().info(f"Executing action {self.plan_index + 1}/{len(self.current_plan)}: {action['description']}")

        # Execute the specific action based on type
        action_type = action['action_type']

        if action_type == 'navigate':
            self.execute_navigation_action(action)
        elif action_type == 'find_object':
            self.execute_find_object_action(action)
        elif action_type == 'navigate_to_object':
            self.execute_navigation_action(action)  # Could be specialized
        elif action_type == 'grasp_object':
            self.execute_grasp_action(action)
        elif action_type == 'return_to_user':
            self.execute_return_to_user_action(action)
        elif action_type == 'stop':
            self.execute_stop_action(action)
        elif action_type == 'greet':
            self.execute_greet_action(action)
        else:
            self.get_logger().error(f"Unknown action type: {action_type}")
            self.plan_index += 1
            self.execute_next_action()

    def execute_navigation_action(self, action: Dict[str, Any]):
        """
        Execute navigation action using MoveBase action
        """
        # Create goal for navigation
        goal_msg = MoveBase.Goal()
        goal_msg.target_pose.header.frame_id = 'map'
        goal_msg.target_pose.header.stamp = self.get_clock().now().to_msg()

        target_pose = action['target_pose']
        goal_msg.target_pose.pose.position.x = target_pose['x']
        goal_msg.target_pose.pose.position.y = target_pose['y']

        # Convert theta to quaternion
        from tf_transformations import quaternion_from_euler
        quat = quaternion_from_euler(0, 0, target_pose['theta'])
        goal_msg.target_pose.pose.orientation.x = quat[0]
        goal_msg.target_pose.pose.orientation.y = quat[1]
        goal_msg.target_pose.pose.orientation.z = quat[2]
        goal_msg.target_pose.pose.orientation.w = quat[3]

        # Send goal
        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(lambda future: self.navigation_goal_callback(future, action))

    def navigation_goal_callback(self, future, action):
        """
        Callback for navigation goal completion
        """
        try:
            goal_handle = future.result()
            if goal_handle.accepted:
                self.get_logger().info(f"Navigation goal accepted: {action['description']}")

                # Wait for result
                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(lambda result_future: self.navigation_result_callback(result_future, action))
            else:
                self.get_logger().error(f"Navigation goal rejected: {action['description']}")
                self.handle_action_failure(action)
        except Exception as e:
            self.get_logger().error(f"Exception in navigation goal callback: {e}")
            self.handle_action_failure(action)

    def navigation_result_callback(self, result_future, action):
        """
        Callback for navigation result
        """
        try:
            result = result_future.result().result
            status = result_future.result().status

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f"Navigation succeeded: {action['description']}")
                self.plan_index += 1
                self.execute_next_action()
            else:
                self.get_logger().error(f"Navigation failed with status {status}: {action['description']}")
                self.handle_action_failure(action)
        except Exception as e:
            self.get_logger().error(f"Exception in navigation result callback: {e}")
            self.handle_action_failure(action)

    def execute_find_object_action(self, action: Dict[str, Any]):
        """
        Execute object finding action (simplified - would integrate with perception system)
        """
        # This would typically call an object detection service
        # For now, simulate finding the object
        self.get_logger().info(f"Looking for {action.get('color', '')} {action['object_type']}")

        # Simulate object detection
        import time
        time.sleep(2.0)  # Simulate processing time

        # For this example, assume object is found
        self.get_logger().info("Object found")
        self.plan_index += 1
        self.execute_next_action()

    def execute_grasp_action(self, action: Dict[str, Any]):
        """
        Execute grasping action
        """
        # This would call a grasping action server
        # For now, simulate grasping
        self.get_logger().info(f"Attempting to grasp {action.get('color', '')} {action['object_type']}")

        # Simulate grasping
        import time
        time.sleep(3.0)  # Simulate grasping time

        # For this example, assume grasp succeeds
        self.get_logger().info("Object grasped successfully")
        self.plan_index += 1
        self.execute_next_action()

    def execute_return_to_user_action(self, action: Dict[str, Any]):
        """
        Execute return to user action
        """
        # This would navigate back to user position
        # For now, simulate return
        self.get_logger().info("Returning to user position")

        # Simulate return
        import time
        time.sleep(2.0)  # Simulate return time

        self.get_logger().info("Returned to user")
        self.plan_index += 1
        self.execute_next_action()

    def execute_stop_action(self, action: Dict[str, Any]):
        """
        Execute stop action
        """
        self.get_logger().info("Stopping current execution")
        self.current_plan = []
        self.plan_index = len(self.current_plan)  # Mark as complete
        self.publish_status("Execution stopped")

    def execute_greet_action(self, action: Dict[str, Any]):
        """
        Execute greeting action
        """
        self.get_logger().info("Performing greeting action")
        # This would trigger robot to wave, speak, etc.
        self.plan_index += 1
        self.execute_next_action()

    def handle_action_failure(self, action: Dict[str, Any]):
        """
        Handle action failure and determine next steps
        """
        self.get_logger().error(f"Action failed: {action['description']}")
        self.publish_status(f"Action failed: {action['description']}")

        # For now, continue with next action, but in a real system you might have
        # more sophisticated failure handling
        self.plan_index += 1
        if self.plan_index < len(self.current_plan):
            self.execute_next_action()
        else:
            self.publish_status("Plan execution completed with failures")

    def publish_status(self, status: str):
        """
        Publish status message
        """
        status_msg = String()
        status_msg.data = status
        self.plan_status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    planner = CognitivePlanner()

    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        planner.get_logger().info("Cognitive planner stopped by user")
    finally:
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Knowledge Representation

For effective cognitive planning, the system maintains a knowledge base about:

1. **Object Properties**: Information about graspable objects, their locations, and characteristics
2. **Spatial Relationships**: Maps of known locations and navigation paths
3. **Action Preconditions**: Conditions required for successful action execution
4. **Robot Capabilities**: What the robot can and cannot do

## Plan Execution Monitoring

The cognitive planning system continuously monitors plan execution and handles:

1. **Action Success/Failure**: Tracking whether each action succeeds or fails
2. **Plan Adaptation**: Modifying plans when unexpected situations arise
3. **Recovery Strategies**: Implementing fallback behaviors when actions fail
4. **User Feedback**: Incorporating user corrections or new commands during execution

## Integration with ROS 2 Action Architecture

The cognitive planning system integrates with ROS 2's action architecture:

- **Action Clients**: For sending goals to navigation and manipulation systems
- **Action Servers**: For providing planning services to other nodes
- **Goal Status Monitoring**: Tracking the status of ongoing actions
- **Result Handling**: Processing action results and updating plans accordingly

## Error Handling and Recovery

The system implements robust error handling:

1. **Timeout Management**: Actions that exceed expected time limits
2. **Sensor Validation**: Verifying sensor data before proceeding
3. **Constraint Checking**: Ensuring actions don't violate physical constraints
4. **Fallback Planning**: Alternative plans when primary plans fail

## Performance Considerations

When implementing cognitive planning:

1. **Planning Efficiency**: Minimize computation time for real-time response
2. **Memory Management**: Efficiently store and update knowledge base
3. **Communication Overhead**: Optimize ROS 2 message passing
4. **Concurrency**: Handle multiple simultaneous planning requests

## Summary

Cognitive planning serves as the intelligent bridge between natural language commands and robot action execution. By maintaining a rich knowledge base and implementing robust planning algorithms, humanoid robots can effectively interpret user intent and execute complex sequences of actions to accomplish tasks.

## Next Steps

In the next section, we'll explore multi-modal interaction systems that combine speech, vision, and gesture for more natural human-robot interaction.