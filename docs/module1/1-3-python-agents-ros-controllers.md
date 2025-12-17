---
title: Python Agents Bridging to ROS Controllers
sidebar_position: 3
description: Learn how to connect Python-based decision-making agents to ROS 2 controllers in humanoid robotics
---

# Python Agents Bridging to ROS Controllers

## Overview

In humanoid robotics, there's often a need to connect high-level decision-making systems (agents) with low-level controllers that manage the physical hardware. Python agents provide an excellent platform for implementing complex decision-making algorithms, and ROS 2 offers the communication infrastructure to connect these agents to the robot's control systems.

This section covers how to design and implement bridges between Python-based agents and ROS 2 controllers, enabling sophisticated behaviors in humanoid robots.

## The Agent-Controller Architecture

In humanoid robotics, the system is typically organized in layers:

- **High-level agents**: Python-based decision-making systems that plan behaviors and goals
- **Mid-level controllers**: ROS 2 nodes that coordinate between high-level goals and low-level hardware
- **Low-level controllers**: Firmware running on robot hardware that directly control motors and sensors

The bridge between Python agents and ROS controllers facilitates communication across these layers.

## Simple Python Agent Example

Here's a basic example of a Python agent that interfaces with ROS 2:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
import threading

class BehaviorAgent(Node):
    def __init__(self):
        super().__init__('behavior_agent')

        # Publishers for sending commands to controllers
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Subscribers for receiving sensor data
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Store current joint states
        self.current_joint_states = JointState()

        # Timer for behavior execution
        self.behavior_timer = self.create_timer(0.5, self.execute_behavior)

        self.get_logger().info('Behavior Agent initialized')

    def joint_state_callback(self, msg):
        """Callback for receiving joint state updates"""
        self.current_joint_states = msg
        self.get_logger().debug(f'Received joint states: {len(msg.name)} joints')

    def execute_behavior(self):
        """Main behavior execution loop"""
        # Example: Simple periodic movement behavior
        if len(self.current_joint_states.name) > 0:
            self.get_logger().info('Executing periodic movement behavior')
            self.execute_periodic_movement()

    def execute_periodic_movement(self):
        """Execute a simple periodic movement"""
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = ['left_hip', 'right_hip', 'left_knee', 'right_knee']

        point = JointTrajectoryPoint()
        point.positions = [0.1, -0.1, 0.05, -0.05]  # Target positions
        point.velocities = [0.0, 0.0, 0.0, 0.0]      # Target velocities
        point.time_from_start.sec = 1                # Execution time
        point.time_from_start.nanosec = 0

        trajectory_msg.points = [point]
        self.trajectory_pub.publish(trajectory_msg)

def main(args=None):
    rclpy.init(args=args)

    # Create the behavior agent
    agent = BehaviorAgent()

    try:
        # Run the agent
        rclpy.spin(agent)
    except KeyboardInterrupt:
        agent.get_logger().info('Behavior agent interrupted')
    finally:
        agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Agent with State Management

For more complex humanoid behaviors, agents need to maintain state and make decisions based on that state:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
from enum import Enum

class RobotState(Enum):
    IDLE = 1
    WALKING = 2
    STANDING = 3
    BALANCING = 4
    EMERGENCY_STOP = 5

class AdvancedBehaviorAgent(Node):
    def __init__(self):
        super().__init__('advanced_behavior_agent')

        # State management
        self.current_state = RobotState.IDLE
        self.previous_state = RobotState.IDLE
        self.state_start_time = self.get_clock().now()

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/agent_status', 10)

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10
        )

        # State transition timer
        self.state_timer = self.create_timer(0.1, self.state_machine)

        # Robot state variables
        self.joint_positions = {}
        self.imu_data = None
        self.balance_threshold = 0.1  # Radians

        self.get_logger().info('Advanced Behavior Agent initialized')

    def joint_state_callback(self, msg):
        """Update joint position data"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]

    def imu_callback(self, msg):
        """Update IMU data for balance control"""
        self.imu_data = msg

    def state_machine(self):
        """Main state machine for behavior control"""
        # Determine next state based on current conditions
        next_state = self.determine_next_state()

        # Handle state transitions
        if next_state != self.current_state:
            self.handle_state_transition(self.current_state, next_state)
            self.current_state = next_state
            self.state_start_time = self.get_clock().now()

        # Execute current state behavior
        self.execute_current_state()

    def determine_next_state(self):
        """Determine the next state based on sensor data"""
        if self.imu_data is None:
            return RobotState.EMERGENCY_STOP

        # Check balance (simplified)
        roll = self.imu_data.orientation.x
        pitch = self.imu_data.orientation.y

        if abs(roll) > self.balance_threshold or abs(pitch) > self.balance_threshold:
            return RobotState.BALANCING

        # Default to standing if in neutral position
        return RobotState.STANDING

    def handle_state_transition(self, old_state, new_state):
        """Handle actions during state transitions"""
        self.get_logger().info(f'State transition: {old_state.name} -> {new_state.name}')

        # Publish status update
        status_msg = String()
        status_msg.data = f'State changed to {new_state.name}'
        self.status_pub.publish(status_msg)

    def execute_current_state(self):
        """Execute behavior for current state"""
        if self.current_state == RobotState.STANDING:
            self.execute_standing_behavior()
        elif self.current_state == RobotState.BALANCING:
            self.execute_balancing_behavior()
        elif self.current_state == RobotState.WALKING:
            self.execute_walking_behavior()
        elif self.current_state == RobotState.EMERGENCY_STOP:
            self.execute_emergency_stop()

    def execute_standing_behavior(self):
        """Maintain standing position"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd)

    def execute_balancing_behavior(self):
        """Attempt to balance the robot"""
        if self.imu_data is None:
            return

        # Simple balance correction based on IMU data
        pitch_error = self.imu_data.orientation.y
        roll_error = self.imu_data.orientation.x

        # Generate corrective movement (simplified)
        cmd = Twist()
        cmd.linear.x = -pitch_error * 0.5  # Move forward/back to correct pitch
        cmd.angular.z = -roll_error * 0.5  # Rotate to correct roll

        self.cmd_vel_pub.publish(cmd)

    def execute_walking_behavior(self):
        """Execute walking gait"""
        # Walking implementation would go here
        pass

    def execute_emergency_stop(self):
        """Stop all movement"""
        cmd = Twist()
        # Stop all movement
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    agent = AdvancedBehaviorAgent()

    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        agent.get_logger().info('Advanced agent stopped by user')
    finally:
        agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Bridge Pattern Implementation

For more sophisticated applications, you might want to implement a dedicated bridge node that translates between your Python agent and ROS controllers:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState
import queue
import threading

class AgentBridge(Node):
    def __init__(self):
        super().__init__('agent_bridge')

        # ROS interfaces
        self.sensor_sub = self.create_subscription(
            JointState, '/joint_states', self.sensor_callback, 10
        )
        self.command_pub = self.create_publisher(
            Float64MultiArray, '/agent_commands', 10
        )

        # Agent communication queue
        self.agent_queue = queue.Queue()
        self.sensor_queue = queue.Queue()

        # Start agent in separate thread
        self.agent_thread = threading.Thread(target=self.run_agent)
        self.agent_thread.daemon = True
        self.agent_thread.start()

        # Timer to process agent commands
        self.process_timer = self.create_timer(0.05, self.process_agent_commands)

    def sensor_callback(self, msg):
        """Receive sensor data and forward to agent"""
        sensor_data = {
            'positions': list(msg.position),
            'velocities': list(msg.velocity),
            'efforts': list(msg.effort),
            'names': list(msg.name)
        }
        self.sensor_queue.put(sensor_data)

    def run_agent(self):
        """Run the Python agent in a separate thread"""
        # This would be your actual agent implementation
        while rclpy.ok():
            try:
                # Get sensor data if available
                if not self.sensor_queue.empty():
                    sensor_data = self.sensor_queue.get_nowait()
                    # Process with agent logic
                    command = self.agent_logic(sensor_data)
                    if command is not None:
                        self.agent_queue.put(command)
            except queue.Empty:
                pass
            except Exception as e:
                self.get_logger().error(f'Agent error: {e}')

            # Small delay to prevent busy waiting
            time.sleep(0.01)

    def agent_logic(self, sensor_data):
        """Placeholder for agent decision-making logic"""
        # Your agent implementation goes here
        # Return command based on sensor data
        return [0.1, 0.2, 0.3]  # Example command

    def process_agent_commands(self):
        """Process commands from the agent and publish to ROS"""
        try:
            while not self.agent_queue.empty():
                command = self.agent_queue.get_nowait()
                cmd_msg = Float64MultiArray()
                cmd_msg.data = command
                self.command_pub.publish(cmd_msg)
        except queue.Empty:
            pass

def main(args=None):
    rclpy.init(args=args)
    bridge = AgentBridge()

    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integration with Humanoid Robot Control

In a humanoid robot system, the Python agent typically handles:

1. **High-level planning**: Determining what actions to take
2. **State management**: Tracking the robot's current behavior state
3. **Goal setting**: Defining targets for lower-level controllers
4. **Monitoring**: Watching for error conditions and safety issues

The ROS controllers then handle:

1. **Trajectory generation**: Creating smooth paths to reach goals
2. **Low-level control**: Direct motor control and feedback
3. **Safety monitoring**: Hardware-level safety checks
4. **Sensor fusion**: Combining multiple sensor inputs

## Best Practices for Agent-Controller Integration

1. **Clear Interfaces**: Define clear APIs between your Python agents and ROS controllers
2. **Error Handling**: Implement robust error handling for communication failures
3. **Timing Considerations**: Ensure your agent runs at appropriate frequencies
4. **Safety First**: Always implement safety mechanisms that can override agent decisions
5. **Modularity**: Design your system so different agents can be swapped in and out
6. **Logging**: Implement comprehensive logging for debugging complex behaviors

## Summary

Python agents provide a powerful platform for implementing sophisticated decision-making in humanoid robots. By properly bridging these agents to ROS controllers, you can create complex behaviors that leverage both high-level reasoning and low-level control precision. The key is to establish clear communication patterns and maintain appropriate separation of concerns between different system layers.

## Next Steps

In the next section, we'll explore Understanding URDF for Humanoids, which is essential for describing the physical structure of your robot in ROS 2.