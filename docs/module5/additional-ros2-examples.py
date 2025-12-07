#!/usr/bin/env python3
"""
Additional ROS 2 Examples for Humanoid Robotics
These examples demonstrate advanced ROS 2 concepts for humanoid robot control
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration

import math
import time
import threading
from enum import Enum
import numpy as np


class WalkingPattern(Enum):
    """Different walking patterns for humanoid locomotion"""
    STANDING = 0
    WALKING_FORWARD = 1
    WALKING_BACKWARD = 2
    TURNING_LEFT = 3
    TURNING_RIGHT = 4
    STOPPING = 5


class AdvancedJointController(Node):
    """
    Advanced joint controller with trajectory planning and safety features
    """

    def __init__(self):
        super().__init__('advanced_joint_controller')

        # Publishers
        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Action client for more sophisticated trajectory control
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

        # Internal state
        self.current_joint_states = JointState()
        self.target_positions = {}
        self.is_safe = True

        # Safety timer
        self.safety_timer = self.create_timer(0.1, self.safety_check)

        self.get_logger().info("Advanced Joint Controller initialized")

    def joint_state_callback(self, msg):
        """Update current joint state"""
        self.current_joint_states = msg

    def safety_check(self):
        """Perform safety checks on joint positions and velocities"""
        # Check for dangerous joint positions
        for i, name in enumerate(self.current_joint_states.name):
            if i < len(self.current_joint_states.position):
                pos = self.current_joint_states.position[i]

                # Example safety limits (these would be robot-specific)
                if abs(pos) > 3.14:  # 180 degrees
                    self.get_logger().warn(f"Dangerous joint position detected for {name}: {pos}")
                    self.is_safe = False
                    return

        self.is_safe = True

    def generate_walk_trajectory(self, pattern=WalkingPattern.WALKING_FORWARD, duration=2.0):
        """Generate a trajectory for walking patterns"""
        # Define joint names for a humanoid robot
        joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint',
            'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint'
        ]

        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = joint_names

        # Create trajectory points
        num_points = 20
        time_step = duration / num_points

        for i in range(num_points + 1):
            point = JointTrajectoryPoint()

            # Calculate progress (0 to 1)
            progress = i / num_points
            t = progress * duration

            # Generate joint positions based on walking pattern
            positions = []
            for j, joint_name in enumerate(joint_names):
                # Base position (standing)
                pos = 0.0

                if 'hip' in joint_name:
                    # Hip joints move for walking
                    if pattern == WalkingPattern.WALKING_FORWARD:
                        pos = 0.1 * math.sin(2 * math.pi * t / duration)
                    elif pattern == WalkingPattern.WALKING_BACKWARD:
                        pos = -0.1 * math.sin(2 * math.pi * t / duration)
                    elif pattern == WalkingPattern.TURNING_LEFT:
                        pos = 0.1 * math.sin(2 * math.pi * t / duration) if 'left' in joint_name else -0.1 * math.sin(2 * math.pi * t / duration)
                    elif pattern == WalkingPattern.TURNING_RIGHT:
                        pos = -0.1 * math.sin(2 * math.pi * t / duration) if 'left' in joint_name else 0.1 * math.sin(2 * math.pi * t / duration)

                elif 'knee' in joint_name:
                    # Knee joints bend for walking
                    if pattern == WalkingPattern.WALKING_FORWARD:
                        pos = 0.05 * math.sin(2 * math.pi * t / duration + math.pi)

                elif 'ankle' in joint_name:
                    # Ankle joints adjust for balance
                    if pattern == WalkingPattern.WALKING_FORWARD:
                        pos = 0.02 * math.sin(2 * math.pi * t / duration)

                positions.append(pos)

            point.positions = positions
            point.velocities = [0.0] * len(positions)  # Calculate velocities
            point.accelerations = [0.0] * len(positions)  # Calculate accelerations

            # Set time from start
            point.time_from_start = Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))
            trajectory_msg.points.append(point)

        return trajectory_msg

    def execute_trajectory(self, trajectory_msg):
        """Execute a joint trajectory"""
        if not self.is_safe:
            self.get_logger().error("Safety check failed - not executing trajectory")
            return False

        self.joint_trajectory_pub.publish(trajectory_msg)
        self.get_logger().info(f"Published trajectory with {len(trajectory_msg.points)} points")
        return True

    def send_trajectory_action(self, trajectory_msg):
        """Send trajectory using action interface for more sophisticated control"""
        if not self.trajectory_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Trajectory action server not available")
            return False

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory_msg

        # Send goal
        future = self.trajectory_client.send_goal_async(goal_msg)
        return future


class BalanceController(Node):
    """
    Balance controller using IMU feedback for humanoid stability
    """

    def __init__(self):
        super().__init__('balance_controller')

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Publishers
        self.ankle_command_pub = self.create_publisher(
            Float64MultiArray,
            '/ankle_controller/commands',
            10
        )

        # Internal state
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # Balance control timer
        self.balance_timer = self.create_timer(0.05, self.balance_control_loop)  # 20 Hz

        self.get_logger().info("Balance Controller initialized")

    def imu_callback(self, msg):
        """Process IMU data to extract orientation"""
        # Convert quaternion to Euler angles
        import tf_transformations as tft

        orientation_q = msg.orientation
        quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        euler = tft.euler_from_quaternion(quaternion)

        self.roll, self.pitch, self.yaw = euler

    def balance_control_loop(self):
        """Main balance control loop using PID control"""
        # Balance thresholds (in radians)
        max_roll_error = 0.1  # ~5.7 degrees
        max_pitch_error = 0.1  # ~5.7 degrees

        # PID gains
        kp = 2.0  # Proportional gain
        ki = 0.1  # Integral gain
        kd = 0.5  # Derivative gain

        # Calculate errors
        roll_error = -self.roll  # Negative because we want to correct the tilt
        pitch_error = -self.pitch

        # Simple PID implementation
        roll_correction = kp * roll_error
        pitch_correction = kp * pitch_error

        # Apply corrections (limit to safe ranges)
        roll_correction = max(min(roll_correction, 0.1), -0.1)
        pitch_correction = max(min(pitch_correction, 0.1), -0.1)

        # Create command message
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [roll_correction, pitch_correction]  # [left_ankle, right_ankle]

        # Publish commands if balance error is significant
        if abs(roll_error) > 0.01 or abs(pitch_error) > 0.01:
            self.ankle_command_pub.publish(cmd_msg)
            if abs(roll_error) > max_roll_error or abs(pitch_error) > max_pitch_error:
                self.get_logger().warn(f"Large balance error detected: roll={self.roll:.3f}, pitch={self.pitch:.3f}")


class BehaviorManager(Node):
    """
    High-level behavior manager that coordinates different robot behaviors
    """

    def __init__(self):
        super().__init__('behavior_manager')

        # Publishers
        self.state_pub = self.create_publisher(String, '/robot_state', 10)
        self.command_pub = self.create_publisher(String, '/high_level_commands', 10)

        # Internal state
        self.current_behavior = WalkingPattern.STANDING
        self.is_active = True

        # Behavior timer
        self.behavior_timer = self.create_timer(1.0, self.behavior_update)

        self.get_logger().info("Behavior Manager initialized")

    def behavior_update(self):
        """Update robot behavior based on state"""
        state_msg = String()

        if self.current_behavior == WalkingPattern.STANDING:
            state_msg.data = "standing"
        elif self.current_behavior == WalkingPattern.WALKING_FORWARD:
            state_msg.data = "walking_forward"
        elif self.current_behavior == WalkingPattern.WALKING_BACKWARD:
            state_msg.data = "walking_backward"
        elif self.current_behavior == WalkingPattern.TURNING_LEFT:
            state_msg.data = "turning_left"
        elif self.current_behavior == WalkingPattern.TURNING_RIGHT:
            state_msg.data = "turning_right"
        elif self.current_behavior == WalkingPattern.STOPPING:
            state_msg.data = "stopping"

        self.state_pub.publish(state_msg)

    def set_behavior(self, behavior):
        """Change robot behavior"""
        self.current_behavior = behavior
        self.get_logger().info(f"Changed behavior to {behavior.name}")

    def start_walking_forward(self):
        """Start walking forward behavior"""
        self.set_behavior(WalkingPattern.WALKING_FORWARD)

    def stop_walking(self):
        """Stop walking and return to standing"""
        self.set_behavior(WalkingPattern.STOPPING)


def main():
    """Main function to demonstrate advanced ROS 2 controllers"""
    rclpy.init()

    # Create nodes
    joint_controller = AdvancedJointController()
    balance_controller = BalanceController()
    behavior_manager = BehaviorManager()

    # Create multi-threaded executor to run all nodes
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(joint_controller)
    executor.add_node(balance_controller)
    executor.add_node(behavior_manager)

    try:
        # Example: Start walking forward after 2 seconds
        def start_walking():
            time.sleep(2)
            behavior_manager.start_walking_forward()

            # Generate and execute a walking trajectory
            trajectory = joint_controller.generate_walk_trajectory(
                pattern=WalkingPattern.WALKING_FORWARD,
                duration=5.0
            )
            joint_controller.execute_trajectory(trajectory)

        # Start walking thread
        walking_thread = threading.Thread(target=start_walking)
        walking_thread.start()

        # Spin all nodes
        executor.spin()

    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        executor.shutdown()
        joint_controller.destroy_node()
        balance_controller.destroy_node()
        behavior_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()