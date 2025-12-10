---
title: ROS 2 Nodes, Topics, and Services
sidebar_position: 2
description: Understanding the fundamental communication patterns in ROS 2 for humanoid robotics
---

# ROS 2 Nodes, Topics, and Services

## Overview

In this section, we'll explore the three fundamental communication patterns in ROS 2: nodes, topics, and services. These form the backbone of how different components of a humanoid robot communicate and coordinate with each other.

## Nodes: The Building Blocks

Nodes are the fundamental executable units of a ROS 2 program. Each node runs independently and can communicate with other nodes through topics, services, or actions. In a humanoid robot, you might have nodes for:

- Joint controllers
- Sensor data processing
- Perception systems
- Path planning
- Behavior management

### Creating a Simple Node

Here's an example of a basic ROS 2 node written in Python:

```python
import rclpy
from rclpy.node import Node

class JointControllerNode(Node):
    def __init__(self):
        super().__init__('joint_controller_node')
        self.get_logger().info('Joint Controller Node has been started')

        # Initialize joint control parameters
        self.target_positions = {}
        self.current_positions = {}

def main(args=None):
    rclpy.init(args=args)
    joint_controller_node = JointControllerNode()

    try:
        rclpy.spin(joint_controller_node)
    except KeyboardInterrupt:
        pass
    finally:
        joint_controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This example shows the basic structure of a ROS 2 node. The `JointControllerNode` inherits from the `Node` class and implements the necessary initialization. The `rclpy.spin()` function keeps the node running and processing callbacks.

## Topics: Publish-Subscribe Communication

Topics enable asynchronous, one-to-many communication between nodes using a publish-subscribe pattern. This is ideal for sensor data, where multiple nodes might need to process the same information simultaneously.

### Publisher Example

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = JointState()
        msg.name = ['left_hip', 'left_knee', 'left_ankle', 'right_hip', 'right_knee', 'right_ankle']
        msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Current joint positions
        msg.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Current joint velocities
        msg.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]    # Current joint efforts

        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing joint states: {msg.position}')

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()

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

### Subscriber Example

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received joint states: {msg.position}')

def main(args=None):
    rclpy.init(args=args)
    node = JointStateSubscriber()

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

## Services: Request-Response Communication

Services provide synchronous, one-to-one communication with a request-response pattern. This is useful for operations that require a specific response, such as requesting the current robot state or commanding a specific action.

### Service Server Example

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool

class ResetService(Node):
    def __init__(self):
        super().__init__('reset_service')
        self.srv = self.create_service(SetBool, 'reset_robot', self.reset_callback)

    def reset_callback(self, request, response):
        if request.data:  # If request is True
            self.get_logger().info('Resetting robot to home position...')
            # Perform reset operations here
            response.success = True
            response.message = 'Robot reset successfully'
        else:
            response.success = False
            response.message = 'Reset request denied'

        return response

def main(args=None):
    rclpy.init(args=args)
    reset_service = ResetService()

    try:
        rclpy.spin(reset_service)
    except KeyboardInterrupt:
        pass
    finally:
        reset_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client Example

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool

class ResetClient(Node):
    def __init__(self):
        super().__init__('reset_client')
        self.cli = self.create_client(SetBool, 'reset_robot')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = SetBool.Request()

    def send_request(self, request_data):
        self.req.data = request_data
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    reset_client = ResetClient()

    response = reset_client.send_request(True)
    reset_client.get_logger().info(f'Result: {response.success}, {response.message}')

    reset_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Communication Pattern Diagram

<img src="/img/module1/ros2-communication-patterns.svg" alt="ROS 2 Communication Patterns" style={{ width: "100%", height: "auto" }} />

The diagram above illustrates how nodes communicate through topics (publish-subscribe) and services (request-response) in a humanoid robot system.

## Practical Applications in Humanoid Robotics

### Sensor Data Distribution
- IMU sensors publish orientation data to `/imu/data` topic
- Multiple nodes (balance controller, state estimator) subscribe to this topic
- Camera nodes publish images to `/camera/image_raw` for perception systems

### Control Commands
- High-level planner sends movement commands via service calls to the motion controller
- Joint controllers publish desired positions to actuator nodes
- Feedback from encoders published to monitor actual positions

### Coordination
- Behavior state published to `/behavior_state` for coordination between different subsystems
- Emergency stop service available for immediate robot stopping across all nodes

## Best Practices

1. **Topic Naming**: Use descriptive, hierarchical names (e.g., `/arm/left/joint_states` instead of `/joints`)

2. **Message Types**: Use standard message types when possible to promote interoperability

3. **QoS Settings**: Configure Quality of Service settings appropriately for your application (reliability, durability, etc.)

4. **Error Handling**: Implement proper error handling in both publishers and subscribers

5. **Resource Management**: Properly clean up resources (timers, subscriptions, publishers) when nodes are destroyed

## Summary

Nodes, topics, and services form the core communication infrastructure of ROS 2. Understanding how to properly design and implement these communication patterns is crucial for creating robust humanoid robot systems. The publish-subscribe model works well for sensor data and state updates, while services are appropriate for operations requiring a specific response.

## Quality of Service (QoS) Settings

When working with humanoid robots, communication reliability is crucial. ROS 2 provides Quality of Service (QoS) settings to fine-tune communication behavior:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Example: Reliable communication for critical control data
reliable_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE
)

# Example: Best-effort for high-frequency sensor data
best_effort_qos = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE
)

# Using QoS in publisher/subscriber
publisher = self.create_publisher(JointState, 'joint_states', reliable_qos)
subscription = self.create_subscription(
    JointState, 'joint_states', self.callback, best_effort_qos
)
```

## Launch Files for Complex Systems

Humanoid robots typically involve many nodes that need to be launched together. Launch files allow you to define and launch multiple nodes:

```python
# launch/humanoid_system.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='humanoid_control',
            executable='joint_controller',
            name='joint_controller_node',
            parameters=[
                {'kp': 10.0},
                {'ki': 0.1},
                {'kd': 0.05}
            ]
        ),
        Node(
            package='perception',
            executable='camera_driver',
            name='camera_node',
            parameters=[
                {'frame_rate': 30},
                {'resolution': [640, 480]}
            ]
        )
    ])
```

## Parameter Management

Nodes in humanoid systems often need to be configurable. ROS 2 provides a parameter system:

```python
import rclpy
from rclpy.node import Node

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')

        # Declare parameters with default values
        self.declare_parameter('control_frequency', 100)
        self.declare_parameter('max_velocity', 1.0)
        self.declare_parameter('safety_margin', 0.1)

        # Get parameter values
        self.frequency = self.get_parameter('control_frequency').value
        self.max_vel = self.get_parameter('max_velocity').value
        self.margin = self.get_parameter('safety_margin').value

        self.get_logger().info(f'Control frequency: {self.frequency}Hz')

def main(args=None):
    rclpy.init(args=args)
    controller = HumanoidController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Testing and Debugging

ROS 2 provides tools for testing and debugging your humanoid robot systems:

### Command Line Tools
- `ros2 topic info /topic_name`: Get information about a topic
- `ros2 topic hz /topic_name`: Measure message frequency
- `ros2 bag record -a`: Record all topics for later analysis
- `ros2 run rqt_graph rqt_graph`: Visualize the node graph
- `ros2 lifecycle`: Manage lifecycle nodes (for state management)

### Testing with rosbag
```bash
# Record data during robot operation
ros2 bag record /joint_states /imu/data /tf

# Play back data for testing
ros2 bag play recorded_session
```

## Safety Considerations

When implementing communication patterns for humanoid robots, safety is paramount:

1. **Timeouts**: Always implement timeouts for service calls
2. **Watchdogs**: Use watchdog timers to detect communication failures
3. **Graceful Degradation**: Design systems to operate safely when components fail
4. **Emergency Stops**: Implement emergency stop mechanisms that can interrupt all operations

## Performance Optimization

For real-time humanoid control, consider these performance tips:

1. **Efficient Message Types**: Use compact message types
2. **Appropriate Frequencies**: Match publication rates to control requirements
3. **Threading**: Use appropriate threading models for your application
4. **Memory Management**: Minimize memory allocations in tight loops

## Summary

Nodes, topics, and services form the core communication infrastructure of ROS 2. Understanding how to properly design and implement these communication patterns is crucial for creating robust humanoid robot systems. The publish-subscribe model works well for sensor data and state updates, while services are appropriate for operations requiring a specific response. Proper QoS settings, parameter management, and safety considerations ensure reliable operation in humanoid robotics applications.

## Next Steps

In the next section, we'll explore how Python agents can bridge to ROS controllers, enabling higher-level decision-making to control the physical robot systems.