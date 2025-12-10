#!/usr/bin/env python3
"""
Gazebo Simulation Examples for Humanoid Robotics
These examples demonstrate how to interface with Gazebo for humanoid robot simulation
"""

import rospy
import numpy as np
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import JointState, LaserScan, Imu
from gazebo_msgs.srv import SpawnModel, DeleteModel, GetModelState
from std_msgs.msg import Float64
import tf.transformations as tft
import math


class GazeboHumanoidSimulator:
    """
    Class to handle Gazebo simulation for humanoid robots
    """

    def __init__(self):
        # Initialize ROS node
        rospy.init_node('gazebo_humanoid_simulator', anonymous=True)

        # Publishers for joint control
        self.joint_publishers = {}
        self.setup_joint_controllers()

        # Subscribers for sensor data
        self.setup_sensor_subscribers()

        # Service proxies for Gazebo interaction
        self.setup_gazebo_services()

        # Robot state variables
        self.current_joint_states = JointState()
        self.robot_pose = Pose()
        self.robot_twist = Twist()

    def setup_joint_controllers(self):
        """
        Set up publishers for controlling humanoid robot joints
        """
        # Example joint names for a humanoid robot
        joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint',
            'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint'
        ]

        for joint_name in joint_names:
            pub = rospy.Publisher(f'/humanoid_robot/{joint_name}/command', Float64, queue_size=10)
            self.joint_publishers[joint_name] = pub

    def setup_sensor_subscribers(self):
        """
        Set up subscribers for various sensors
        """
        # Joint states
        rospy.Subscriber('/humanoid_robot/joint_states', JointState, self.joint_states_callback)

        # IMU data
        rospy.Subscriber('/humanoid_robot/imu/data', Imu, self.imu_callback)

        # LiDAR data
        rospy.Subscriber('/humanoid_robot/laser_scan', LaserScan, self.lidar_callback)

    def setup_gazebo_services(self):
        """
        Set up service clients for Gazebo interaction
        """
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        self.spawn_model_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

        rospy.wait_for_service('/gazebo/delete_model')
        self.delete_model_srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

        rospy.wait_for_service('/gazebo/get_model_state')
        self.get_model_state_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    def joint_states_callback(self, msg):
        """
        Callback for joint state updates
        """
        self.current_joint_states = msg

    def imu_callback(self, msg):
        """
        Callback for IMU sensor updates
        """
        # Convert quaternion to Euler angles
        orientation_q = msg.orientation
        roll, pitch, yaw = tft.euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])

        # Check for balance (simplified)
        balance_threshold = math.radians(15)  # 15 degrees
        if abs(roll) > balance_threshold or abs(pitch) > balance_threshold:
            rospy.logwarn('Potential balance issue detected!')

    def lidar_callback(self, msg):
        """
        Callback for LiDAR sensor updates
        """
        # Process LiDAR ranges for obstacle detection
        ranges = np.array(msg.ranges)
        valid_ranges = ranges[np.isfinite(ranges)]

        # Simple obstacle detection
        obstacle_threshold = 1.0  # meters
        obstacles = np.where(valid_ranges < obstacle_threshold)[0]

        if len(obstacles) > 0:
            rospy.loginfo(f'Detected {len(obstacles)} potential obstacles')

    def move_to_pose(self, joint_angles, duration=5.0):
        """
        Move humanoid robot to a specific pose over time

        Args:
            joint_angles (dict): Dictionary mapping joint names to target angles
            duration (float): Duration to complete the movement in seconds
        """
        rate = rospy.Rate(100)  # 100 Hz
        start_time = rospy.Time.now()
        start_angles = {name: self.get_joint_position(name) for name in joint_angles.keys()}

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()

            if elapsed >= duration:
                # Send final positions
                for joint_name, target_angle in joint_angles.items():
                    cmd = Float64()
                    cmd.data = target_angle
                    self.joint_publishers[joint_name].publish(cmd)
                break

            # Interpolate between start and target positions
            t = elapsed / duration
            for joint_name, target_angle in joint_angles.items():
                start_angle = start_angles[joint_name]
                current_angle = start_angle + (target_angle - start_angle) * t
                cmd = Float64()
                cmd.data = current_angle
                self.joint_publishers[joint_name].publish(cmd)

            rate.sleep()

    def get_joint_position(self, joint_name):
        """
        Get current position of a specific joint

        Args:
            joint_name (str): Name of the joint

        Returns:
            float: Current joint position in radians
        """
        try:
            idx = self.current_joint_states.name.index(joint_name)
            return self.current_joint_states.position[idx]
        except ValueError:
            rospy.logwarn(f'Joint {joint_name} not found in joint states')
            return 0.0

    def balance_control_loop(self):
        """
        Main control loop for maintaining humanoid robot balance
        """
        rate = rospy.Rate(100)  # 100 Hz control loop

        while not rospy.is_shutdown():
            # Get current IMU readings
            # Note: In a real implementation, you'd get these from the IMU callback

            # Simple PD controller for balance (simplified example)
            # This would typically involve more complex control algorithms like LQR or MPC

            # Example: Adjust ankle joints based on pitch angle
            pitch_correction = 0.0  # This would come from IMU data
            ankle_adjustment = 0.1 * pitch_correction  # Proportional control

            # Apply corrections to ankle joints
            ankle_joints = ['left_ankle_joint', 'right_ankle_joint']
            for joint in ankle_joints:
                current_pos = self.get_joint_position(joint)
                target_pos = current_pos + ankle_adjustment

                cmd = Float64()
                cmd.data = target_pos
                self.joint_publishers[joint].publish(cmd)

            rate.sleep()

    def spawn_object_in_environment(self, model_name, model_xml, pose, reference_frame="world"):
        """
        Spawn an object in the Gazebo environment

        Args:
            model_name (str): Name of the model to spawn
            model_xml (str): XML description of the model
            pose (Pose): Initial pose of the model
            reference_frame (str): Reference frame for the pose

        Returns:
            bool: True if spawn was successful
        """
        try:
            resp = self.spawn_model_srv(
                model_name=model_name,
                model_xml=model_xml,
                robot_namespace="",
                initial_pose=pose,
                reference_frame=reference_frame
            )
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Spawn model service call failed: {e}")
            return False

    def get_robot_state(self, model_name="humanoid_robot", relative_entity_name="world"):
        """
        Get the current state of the robot in the simulation

        Args:
            model_name (str): Name of the model to query
            relative_entity_name (str): Reference entity name

        Returns:
            tuple: (pose, twist) of the robot
        """
        try:
            resp = self.get_model_state_srv(
                model_name=model_name,
                relative_entity_name=relative_entity_name
            )
            self.robot_pose = resp.pose
            self.robot_twist = resp.twist
            return resp.pose, resp.twist
        except rospy.ServiceException as e:
            rospy.logerr(f"Get model state service call failed: {e}")
            return None, None


def main():
    """
    Main function to demonstrate Gazebo simulation capabilities
    """
    simulator = GazeboHumanoidSimulator()

    # Example: Move to a standing pose
    standing_pose = {
        'left_hip_joint': 0.0,
        'left_knee_joint': 0.0,
        'left_ankle_joint': 0.0,
        'right_hip_joint': 0.0,
        'right_knee_joint': 0.0,
        'right_ankle_joint': 0.0,
        'left_shoulder_joint': 0.2,
        'left_elbow_joint': -0.5,
        'right_shoulder_joint': 0.2,
        'right_elbow_joint': -0.5
    }

    rospy.loginfo("Moving humanoid robot to standing pose...")
    simulator.move_to_pose(standing_pose, duration=3.0)

    # Start balance control loop
    rospy.loginfo("Starting balance control loop...")
    simulator.balance_control_loop()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass