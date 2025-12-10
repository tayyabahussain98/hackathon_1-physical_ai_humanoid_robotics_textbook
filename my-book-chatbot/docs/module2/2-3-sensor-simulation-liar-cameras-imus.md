---
title: "Sensor Simulation: LiDAR, Depth Cameras, IMUs"
sidebar_position: 3
description: "Learn how to simulate various sensors for humanoid robots in Gazebo and Unity environments"
---

# Sensor Simulation: LiDAR, Depth Cameras, IMUs

## Overview

Sensor simulation is a critical aspect of humanoid robotics development, enabling the testing of perception and navigation algorithms without requiring physical hardware. In this section, we'll explore how to simulate various sensors including LiDAR, depth cameras, and IMUs in both Gazebo and Unity environments. These sensors are essential for humanoid robots to perceive their environment and maintain balance and navigation capabilities.

## Sensor Simulation in Gazebo

Gazebo provides realistic simulation of various sensor types through its sensor plugins. The sensors are defined in the robot's URDF file and simulated with physics-based accuracy.

### LiDAR Sensors

LiDAR sensors are crucial for humanoid robots for mapping, localization, and obstacle detection. Here's how to configure a LiDAR sensor in Gazebo:

```xml
<gazebo reference="lidar_link">
  <sensor type="ray" name="lidar_sensor">
    <always_on>true</always_on>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.10</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/humanoid_robot</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_frame</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### Depth Cameras

Depth cameras provide both color and depth information, essential for humanoid robots for object recognition and navigation:

```xml
<gazebo reference="camera_depth_optical_frame">
  <sensor type="depth" name="depth_camera">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>10.0</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
      <ros>
        <namespace>/humanoid_robot</namespace>
        <remapping>image_raw:=camera/image_raw</remapping>
        <remapping>depth/image_raw:=camera/depth/image_raw</remapping>
        <remapping>points:=camera/depth/points</remapping>
      </ros>
      <output_type>sensor_msgs/Image</output_type>
      <baseline>0.2</baseline>
      <distortion_k1>0.0</distortion_k1>
      <distortion_k2>0.0</distortion_k2>
      <distortion_k3>0.0</distortion_k3>
      <distortion_t1>0.0</distortion_t1>
      <distortion_t2>0.0</distortion_t2>
      <point_cloud_cutoff>0.5</point_cloud_cutoff>
      <frame_name>camera_depth_optical_frame</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### IMU Sensors

IMUs (Inertial Measurement Units) are critical for humanoid robot balance and orientation sensing:

```xml
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0017</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0017</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0017</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
      <ros>
        <namespace>/humanoid_robot</namespace>
        <remapping>~/out:=imu/data</remapping>
      </ros>
      <frame_name>imu_link</frame_name>
      <initial_orientation_as_reference>false</initial_orientation_as_reference>
    </plugin>
  </sensor>
</gazebo>
```

## Processing Sensor Data in ROS 2

Once the sensors are simulated, we need to process the data in ROS 2. Here's an example of how to subscribe to and process different sensor streams:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu, PointCloud2
from cv_bridge import CvBridge
import numpy as np
import math

class SensorProcessor(Node):
    def __init__(self):
        super().__init__('sensor_processor')

        # Initialize CV bridge for image processing
        self.bridge = CvBridge()

        # Create subscribers for different sensors
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/humanoid_robot/scan',
            self.lidar_callback,
            10
        )

        self.camera_sub = self.create_subscription(
            Image,
            '/humanoid_robot/camera/image_raw',
            self.camera_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/humanoid_robot/camera/depth/image_raw',
            self.depth_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/humanoid_robot/imu/data',
            self.imu_callback,
            10
        )

        # Create publishers for processed data
        self.obstacle_pub = self.create_publisher(
            PointCloud2,
            '/processed_obstacles',
            10
        )

        self.get_logger().info('Sensor processor initialized')

    def lidar_callback(self, msg):
        """Process LiDAR data for obstacle detection"""
        ranges = np.array(msg.ranges)

        # Filter out invalid ranges (inf or nan)
        valid_ranges = ranges[np.isfinite(ranges)]

        # Simple obstacle detection: find ranges below threshold
        obstacle_threshold = 1.0  # meters
        obstacles = np.where(valid_ranges < obstacle_threshold)[0]

        if len(obstacles) > 0:
            self.get_logger().info(f'Detected {len(obstacles)} potential obstacles')

        # Process angles and distances
        angle_increment = msg.angle_increment
        angle_min = msg.angle_min

        for i in obstacles[:5]:  # Process first 5 obstacles
            angle = angle_min + i * angle_increment
            distance = ranges[i]
            # Convert polar to Cartesian coordinates
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            self.get_logger().debug(f'Obstacle at ({x:.2f}, {y:.2f}), distance: {distance:.2f}m')

    def camera_callback(self, msg):
        """Process RGB camera data"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Perform basic processing - example: edge detection
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150)

            # In real application, you might run object detection, etc.
            self.get_logger().debug(f'Processed camera image: {cv_image.shape}')

        except Exception as e:
            self.get_logger().error(f'Error processing camera image: {str(e)}')

    def depth_callback(self, msg):
        """Process depth camera data"""
        try:
            # Convert depth image to OpenCV format
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

            # Process depth information
            height, width = depth_image.shape

            # Example: Calculate average depth in center region
            center_h, center_w = height // 2, width // 2
            crop_size = 50
            center_region = depth_image[
                center_h - crop_size:center_h + crop_size,
                center_w - crop_size:center_w + crop_size
            ]

            avg_depth = np.nanmean(center_region[np.isfinite(center_region)])

            if not np.isnan(avg_depth):
                self.get_logger().debug(f'Average depth in center: {avg_depth:.2f}m')

        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')

    def imu_callback(self, msg):
        """Process IMU data for orientation and acceleration"""
        # Extract orientation (quaternion)
        orientation = msg.orientation
        quat = [orientation.x, orientation.y, orientation.z, orientation.w]

        # Convert quaternion to Euler angles
        euler = self.quaternion_to_euler(quat)
        roll, pitch, yaw = euler

        # Extract angular velocity
        ang_vel = msg.angular_velocity
        # Extract linear acceleration
        lin_acc = msg.linear_acceleration

        # Log orientation data
        self.get_logger().debug(
            f'Orientation - Roll: {math.degrees(roll):.2f}°, '
            f'Pitch: {math.degrees(pitch):.2f}°, '
            f'Yaw: {math.degrees(yaw):.2f}°'
        )

        # Check for balance issues (simplified)
        balance_threshold = math.radians(15)  # 15 degrees
        if abs(roll) > balance_threshold or abs(pitch) > balance_threshold:
            self.get_logger().warn('Potential balance issue detected!')

    def quaternion_to_euler(self, quat):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        x, y, z, w = quat

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    sensor_processor = SensorProcessor()

    try:
        rclpy.spin(sensor_processor)
    except KeyboardInterrupt:
        sensor_processor.get_logger().info('Shutting down sensor processor')
    finally:
        sensor_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Sensor Fusion for Humanoid Robotics

Humanoid robots typically use sensor fusion to combine data from multiple sensors for improved perception and decision making:

```python
import numpy as np
from scipy.spatial.transform import Rotation as R

class SensorFusion:
    def __init__(self):
        # Initialize sensor fusion filters
        self.imu_orientation = np.array([0, 0, 0])  # roll, pitch, yaw
        self.lidar_position = np.array([0, 0, 0])   # x, y, z
        self.odometry_position = np.array([0, 0, 0])  # x, y, theta

        # Covariance matrices for each sensor
        self.imu_cov = np.eye(3) * 0.01
        self.lidar_cov = np.eye(3) * 0.1
        self.odom_cov = np.eye(3) * 0.05

    def fuse_imu_lidar_odom(self, imu_data, lidar_data, odom_data):
        """Fuse IMU, LiDAR, and odometry data using weighted averaging"""

        # Get measurements and uncertainties
        imu_orient = np.array([imu_data.roll, imu_data.pitch, imu_data.yaw])
        lidar_pos = np.array([lidar_data.x, lidar_data.y, lidar_data.z])
        odom_pos = np.array([odom_data.x, odom_data.y, odom_data.theta])

        # Compute weights based on inverse covariance (lower uncertainty = higher weight)
        imu_weight = np.linalg.inv(self.imu_cov)
        lidar_weight = np.linalg.inv(self.lidar_cov)
        odom_weight = np.linalg.inv(self.odom_cov)

        # Weighted fusion
        fused_orientation = np.linalg.solve(
            imu_weight,
            imu_weight @ imu_orient
        )

        # For position, we might use a more complex fusion scheme
        # Simple weighted average for demonstration
        total_weight = np.sum([imu_weight[0,0], lidar_weight[0,0], odom_weight[0,0]])
        fused_position = (
            lidar_weight[0,0] * lidar_pos +
            odom_weight[0,0] * odom_pos[:2]  # Only x,y from odometry
        ) / (lidar_weight[0,0] + odom_weight[0,0])

        return fused_orientation, np.append(fused_position, odom_pos[2])  # Add theta back

    def kalman_filter_predict(self, state, covariance, dt):
        """Kalman filter prediction step"""
        # State transition matrix (simplified)
        F = np.eye(len(state))
        # Process noise (simplified)
        Q = np.eye(len(state)) * 0.01

        # Predict next state
        predicted_state = F @ state
        predicted_covariance = F @ covariance @ F.T + Q

        return predicted_state, predicted_covariance

    def kalman_filter_update(self, predicted_state, predicted_covariance, measurement, measurement_matrix, measurement_noise):
        """Kalman filter update step"""
        # Innovation
        innovation = measurement - measurement_matrix @ predicted_state
        # Innovation covariance
        innovation_cov = measurement_matrix @ predicted_covariance @ measurement_matrix.T + measurement_noise
        # Kalman gain
        kalman_gain = predicted_covariance @ measurement_matrix.T @ np.linalg.inv(innovation_cov)

        # Updated state
        updated_state = predicted_state + kalman_gain @ innovation
        # Updated covariance
        updated_covariance = predicted_covariance - kalman_gain @ measurement_matrix @ predicted_covariance

        return updated_state, updated_covariance
```

## Unity Sensor Simulation

Unity also provides sensor simulation capabilities, particularly for generating synthetic data:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor;

public class UnitySensorSimulation : MonoBehaviour
{
    public Camera rgbCamera;
    public Camera depthCamera;
    public GameObject lidarRig;
    public int lidarResolution = 720;
    public float lidarRange = 30.0f;
    public float lidarFov = 360.0f;

    private RosTopicPublisher<LaserScan> lidarPublisher;
    private RosTopicPublisher<Image> rgbPublisher;
    private RosTopicPublisher<Image> depthPublisher;
    private RosTopicPublisher<IMU> imuPublisher;

    void Start()
    {
        var ros = RosTopicConnector.instance;

        // Initialize publishers
        lidarPublisher = ros.Advertise<LaserScan>("/unity/lidar_scan");
        rgbPublisher = ros.Advertise<Image>("/unity/rgb_image");
        depthPublisher = ros.Advertise<Image>("/unity/depth_image");
        imuPublisher = ros.Advertise<IMU>("/unity/imu_data");
    }

    void Update()
    {
        // Publish sensor data at appropriate rates
        if (Time.frameCount % 10 == 0) // 10 Hz for LiDAR
        {
            PublishLidarData();
        }

        if (Time.frameCount % 3 == 0) // 30 Hz for cameras
        {
            PublishRgbImage();
            PublishDepthImage();
        }

        if (Time.frameCount % 1 == 0) // 60 Hz for IMU
        {
            PublishImuData();
        }
    }

    void PublishLidarData()
    {
        var scanMsg = new LaserScan();

        // Simulate LiDAR rays
        float[] ranges = new float[lidarResolution];
        float angleStep = lidarFov / lidarResolution;

        for (int i = 0; i < lidarResolution; i++)
        {
            float angle = Mathf.Deg2Rad * (i * angleStep - lidarFov / 2);

            // Raycast to simulate LiDAR measurement
            Vector3 direction = new Vector3(Mathf.Cos(angle), 0, Mathf.Sin(angle));
            RaycastHit hit;

            if (Physics.Raycast(lidarRig.transform.position,
                               lidarRig.transform.TransformDirection(direction),
                               out hit, lidarRange))
            {
                ranges[i] = hit.distance;
            }
            else
            {
                ranges[i] = lidarRange; // No obstacle detected
            }
        }

        scanMsg.ranges = ranges;
        scanMsg.angle_min = Mathf.Deg2Rad * (-lidarFov / 2);
        scanMsg.angle_max = Mathf.Deg2Rad * (lidarFov / 2);
        scanMsg.angle_increment = Mathf.Deg2Rad * angleStep;
        scanMsg.range_min = 0.1f;
        scanMsg.range_max = lidarRange;
        scanMsg.header.stamp = new TimeStamp(Time.time);

        lidarPublisher.Publish(scanMsg);
    }

    void PublishRgbImage()
    {
        // Capture RGB image from camera
        RenderTexture currentRT = RenderTexture.active;
        RenderTexture.active = rgbCamera.targetTexture;

        Texture2D imageTex = new Texture2D(rgbCamera.pixelWidth, rgbCamera.pixelHeight, TextureFormat.RGB24, false);
        imageTex.ReadPixels(new Rect(0, 0, rgbCamera.pixelWidth, rgbCamera.pixelHeight), 0, 0);
        imageTex.Apply();

        RenderTexture.active = currentRT;

        // Convert to ROS Image message
        var imageMsg = new Image();
        imageMsg.width = (uint)rgbCamera.pixelWidth;
        imageMsg.height = (uint)rgbCamera.pixelHeight;
        imageMsg.encoding = "rgb8";
        imageMsg.is_bigendian = 0;
        imageMsg.step = (uint)(rgbCamera.pixelWidth * 3);
        imageMsg.data = imageTex.EncodeToPNG();
        imageMsg.header.stamp = new TimeStamp(Time.time);

        rgbPublisher.Publish(imageMsg);

        // Clean up texture
        Destroy(imageTex);
    }

    void PublishDepthImage()
    {
        // Similar to RGB but for depth data
        RenderTexture currentRT = RenderTexture.active;
        RenderTexture.active = depthCamera.targetTexture;

        Texture2D depthTex = new Texture2D(depthCamera.pixelWidth, depthCamera.pixelHeight, TextureFormat.RFloat, false);
        depthTex.ReadPixels(new Rect(0, 0, depthCamera.pixelWidth, depthCamera.pixelHeight), 0, 0);
        depthTex.Apply();

        RenderTexture.active = currentRT;

        // Process depth data (simplified)
        Color[] pixels = depthTex.GetPixels();
        float[] depthValues = new float[pixels.Length];

        for (int i = 0; i < pixels.Length; i++)
        {
            depthValues[i] = pixels[i].r; // Assuming depth is stored in red channel
        }

        var depthMsg = new Image();
        depthMsg.width = (uint)depthCamera.pixelWidth;
        depthMsg.height = (uint)depthCamera.pixelHeight;
        depthMsg.encoding = "32FC1"; // 32-bit float per channel
        depthMsg.is_bigendian = 0;
        depthMsg.step = (uint)(depthCamera.pixelWidth * sizeof(float));
        depthMsg.data = FloatArrayToByteArray(depthValues);
        depthMsg.header.stamp = new TimeStamp(Time.time);

        depthPublisher.Publish(depthMsg);

        Destroy(depthTex);
    }

    void PublishImuData()
    {
        var imuMsg = new IMU();

        // Get orientation from Unity (convert from left-handed to right-handed)
        Quaternion unityRot = transform.rotation;
        Quaternion rosRot = new Quaternion(unityRot.x, unityRot.y, unityRot.z, unityRot.w);

        imuMsg.orientation.x = rosRot.x;
        imuMsg.orientation.y = rosRot.y;
        imuMsg.orientation.z = rosRot.z;
        imuMsg.orientation.w = rosRot.w;

        // Angular velocity (simplified)
        imuMsg.angular_velocity.x = 0.0f; // Would come from Rigidbody component
        imuMsg.angular_velocity.y = 0.0f;
        imuMsg.angular_velocity.z = 0.0f;

        // Linear acceleration (simplified)
        imuMsg.linear_acceleration.x = Physics.gravity.x;
        imuMsg.linear_acceleration.y = Physics.gravity.y;
        imuMsg.linear_acceleration.z = Physics.gravity.z;

        imuMsg.header.stamp = new TimeStamp(Time.time);

        imuPublisher.Publish(imuMsg);
    }

    byte[] FloatArrayToByteArray(float[] floatArray)
    {
        byte[] byteArray = new byte[floatArray.Length * sizeof(float)];
        for (int i = 0; i < floatArray.Length; i++)
        {
            byte[] floatBytes = BitConverter.GetBytes(floatArray[i]);
            Array.Copy(floatBytes, 0, byteArray, i * sizeof(float), sizeof(float));
        }
        return byteArray;
    }
}
```

## Sensor Calibration and Validation

For accurate simulation, sensors need to be calibrated to match their real-world counterparts:

1. **Intrinsic calibration**: Camera focal length, distortion coefficients
2. **Extrinsic calibration**: Position and orientation of sensors relative to robot
3. **Temporal synchronization**: Ensuring sensor data is properly time-stamped
4. **Noise modeling**: Accurately representing sensor noise characteristics

## Performance Considerations

Simulating multiple sensors can be computationally expensive:

- **Update rates**: Match simulation update rates to real sensor frequencies
- **Resolution**: Balance sensor resolution with performance requirements
- **Filtering**: Pre-filter sensor data to reduce processing load
- **Parallel processing**: Use multi-threading where possible

## Summary

Sensor simulation is fundamental to humanoid robotics development, providing safe and reproducible testing environments. By accurately simulating LiDAR, cameras, and IMUs in both Gazebo and Unity, we can develop and validate perception and control algorithms before deploying to physical robots. Proper sensor fusion techniques combine multiple sensor inputs to improve the robot's understanding of its environment and state.

## Next Steps

In the next module, we'll explore the AI-Robot Brain using NVIDIA Isaac, where we'll see how these sensor inputs are processed for perception, navigation, and decision-making in humanoid robots.