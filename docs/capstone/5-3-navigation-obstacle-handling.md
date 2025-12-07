---
title: Navigation & Obstacle Handling
sidebar_position: 3
description: Implementing navigation and obstacle handling for the autonomous humanoid robot
---

# Navigation & Obstacle Handling

## Overview

Navigation and obstacle handling form the core of the humanoid robot's mobility system. This component enables the robot to move safely through complex environments, avoiding obstacles while reaching designated destinations. The system integrates concepts from Module 3 (NVIDIA Isaac) with ROS 2 navigation capabilities.

## Architecture

The navigation and obstacle handling system consists of:

1. **Localization**: Determining the robot's position in the environment
2. **Mapping**: Creating and maintaining a representation of the environment
3. **Path Planning**: Computing optimal paths to destinations
4. **Obstacle Detection**: Identifying and tracking obstacles in real-time
5. **Motion Control**: Executing planned trajectories while avoiding obstacles
6. **Recovery Behaviors**: Handling navigation failures and replanning

## Navigation Stack Integration

The system builds upon the ROS 2 Navigation2 stack with customizations for humanoid robots:

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from sensor_msgs.msg import LaserScan, PointCloud2
from visualization_msgs.msg import MarkerArray
from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient
import numpy as np
import math
from tf2_ros import Buffer, TransformListener
import tf_transformations as tft

class HumanoidNavigationNode(Node):
    """
    Navigation node optimized for humanoid robots with bipedal locomotion
    """

    def __init__(self):
        super().__init__('humanoid_navigation_node')

        # Action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # TF buffer for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers for visualization
        self.obstacle_pub = self.create_publisher(MarkerArray, '/detected_obstacles', 10)
        self.path_pub = self.create_publisher(MarkerArray, '/planned_path', 10)

        # Subscribers for sensor data
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        # Obstacle detection parameters
        self.obstacle_threshold = 0.5  # meters
        self.robot_radius = 0.4  # meters (approximate humanoid width)
        self.is_navigating = False
        self.current_goal = None

        # Timer for periodic navigation updates
        self.nav_timer = self.create_timer(0.1, self.navigation_update)

        self.get_logger().info("Humanoid Navigation Node initialized")

    def navigate_to_location(self, location_name):
        """
        Navigate to a predefined location by name
        """
        # Convert location name to coordinates (this would use a map)
        location_coords = self.get_location_coordinates(location_name)
        if location_coords:
            return self.navigate_to_pose(location_coords)
        else:
            self.get_logger().error(f"Unknown location: {location_name}")
            return False

    def navigate_to_pose(self, pose):
        """
        Navigate to a specific pose
        """
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Navigation action server not available")
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = ''  # Use default behavior tree

        self.current_goal = pose
        self.is_navigating = True

        self.get_logger().info(f"Sending navigation goal: {pose.pose.position}")

        # Send goal asynchronously
        future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback_callback
        )

        future.add_done_callback(self.navigation_result_callback)
        return True

    def get_location_coordinates(self, location_name):
        """
        Get coordinates for a named location
        """
        # Predefined locations in the environment
        locations = {
            'kitchen': self.create_pose(2.0, 1.0, 0.0),
            'living room': self.create_pose(-1.0, 0.0, 1.57),  # 90 degrees
            'bedroom': self.create_pose(3.0, -2.0, 0.0),
            'office': self.create_pose(-2.0, 2.0, 3.14),  # 180 degrees
            'dining room': self.create_pose(0.0, -3.0, -1.57)  # -90 degrees
        }

        if location_name in locations:
            return locations[location_name]
        else:
            return None

    def create_pose(self, x, y, theta):
        """
        Create a PoseStamped message from coordinates
        """
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        # Convert theta (rotation) to quaternion
        quat = tft.quaternion_from_euler(0, 0, theta)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        return pose

    def lidar_callback(self, msg):
        """
        Process LiDAR data for obstacle detection
        """
        if not self.is_navigating:
            return

        # Detect obstacles in the scan
        obstacles = self.detect_obstacles_from_scan(msg)

        # Publish detected obstacles for visualization
        self.publish_obstacles(obstacles)

        # Check if path is blocked
        if self.is_path_blocked(obstacles):
            self.get_logger().warn("Path is blocked by obstacles, pausing navigation")
            # In a real system, this would trigger replanning
            # For now, we'll just log the issue

    def detect_obstacles_from_scan(self, scan_msg):
        """
        Detect obstacles from LiDAR scan data
        """
        obstacles = []
        angle_increment = scan_msg.angle_increment

        for i, range_val in enumerate(scan_msg.ranges):
            if not math.isfinite(range_val) or range_val > self.obstacle_threshold:
                continue

            # Calculate angle
            angle = scan_msg.angle_min + i * angle_increment

            # Convert to Cartesian coordinates
            x = range_val * math.cos(angle)
            y = range_val * math.sin(angle)

            # Only consider obstacles in front of robot (forward hemisphere)
            if x > 0:  # In robot's coordinate frame
                obstacles.append((x, y, range_val))

        return obstacles

    def is_path_blocked(self, obstacles):
        """
        Check if the current path to goal is blocked
        """
        if not self.current_goal or not obstacles:
            return False

        # Get robot position
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
        except Exception:
            return False

        goal_x = self.current_goal.pose.position.x
        goal_y = self.current_goal.pose.position.y

        # Check if obstacles are on the path between robot and goal
        for obs_x, obs_y, _ in obstacles:
            # Simple line-of-sight check (in robot's frame after transformation)
            # This is simplified - a real system would use more sophisticated path checking
            distance_to_path = self.distance_to_line(
                (robot_x, robot_y),
                (goal_x, goal_y),
                (obs_x, obs_y)
            )

            if distance_to_path < self.robot_radius:
                return True

        return False

    def distance_to_line(self, p1, p2, p3):
        """
        Calculate distance from point p3 to line defined by p1 and p2
        """
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3

        # Calculate distance using cross product
        numerator = abs((y2 - y1) * x3 - (x2 - x1) * y3 + x2 * y1 - y2 * x1)
        denominator = math.sqrt((y2 - y1)**2 + (x2 - x1)**2)

        if denominator == 0:
            return math.sqrt((x3 - x1)**2 + (y3 - y1)**2)

        return numerator / denominator

    def publish_obstacles(self, obstacles):
        """
        Publish detected obstacles for visualization
        """
        marker_array = MarkerArray()

        for i, (x, y, range_val) in enumerate(obstacles):
            marker = Marker()
            marker.header.frame_id = 'base_link'  # Robot's frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'obstacles'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.0

            marker.scale.x = 0.2  # 20cm diameter
            marker.scale.y = 0.2
            marker.scale.z = 0.2

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8  # 80% alpha

            marker_array.markers.append(marker)

        self.obstacle_pub.publish(marker_array)

    def navigation_feedback_callback(self, feedback_msg):
        """
        Handle navigation feedback
        """
        self.get_logger().debug(f"Navigation feedback: {feedback_msg}")

    def navigation_result_callback(self, future):
        """
        Handle navigation result
        """
        goal_result = future.result()
        if goal_result.status == 4:  # SUCCESS
            self.get_logger().info("Navigation completed successfully")
        else:
            self.get_logger().warn(f"Navigation failed with status: {goal_result.status}")

        self.is_navigating = False
        self.current_goal = None

    def navigation_update(self):
        """
        Periodic navigation update function
        """
        # This function runs periodically to update navigation state
        # In a real system, this would handle dynamic replanning, etc.
        pass

    def stop_navigation(self):
        """
        Stop current navigation
        """
        self.is_navigating = False
        # In a real system, this would cancel the navigation action
        self.get_logger().info("Navigation stopped")


def main(args=None):
    rclpy.init(args=args)
    navigator = HumanoidNavigationNode()

    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info("Navigation node stopped by user")
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Bipedal Locomotion Considerations

Humanoid robots present unique challenges for navigation compared to wheeled robots:

1. **Dynamic Balance**: The robot must maintain balance while moving
2. **Footstep Planning**: Path planning must consider foot placement
3. **Stability**: Movement must be stable on two legs
4. **Terrain Adaptation**: Ability to handle uneven surfaces

## Obstacle Avoidance Strategies

The system implements multiple obstacle avoidance strategies:

1. **Static Obstacle Avoidance**: Avoid fixed obstacles using map data
2. **Dynamic Obstacle Avoidance**: Detect and avoid moving obstacles
3. **Predictive Avoidance**: Anticipate obstacle movements
4. **Recovery Behaviors**: Handle navigation failures gracefully

## Integration with Voice Commands

The navigation system integrates with voice command processing by:

1. Receiving destination coordinates from the command processor
2. Converting location names to coordinates using the location mapping
3. Executing navigation to the specified location
4. Reporting navigation status back to the command system

## Performance Optimization

To optimize navigation performance for humanoid robots:

1. **Efficient Path Planning**: Use algorithms optimized for bipedal locomotion
2. **Real-time Obstacle Detection**: Process sensor data in real-time
3. **Predictive Path Adjustment**: Anticipate and adjust paths before obstacles
4. **Energy Efficiency**: Minimize energy consumption during navigation

## Testing Navigation

The navigation system should be tested with various scenarios:

- Static and dynamic obstacle avoidance
- Different terrain types (carpet, hardwood, uneven surfaces)
- Narrow passages and doorways
- Staircases (if supported)
- Recovery from navigation failures

## Next Steps

In the next section, we'll implement object recognition and manipulation capabilities to handle the "fetch" commands from the voice processing system.