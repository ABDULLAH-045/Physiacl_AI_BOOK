# Data Model: Module 2: The Digital Twin (Gazebo & Unity)

This document describes the key data entities, structures, and communication contracts for the Digital Twin module, focusing on data flow between Gazebo, Unity, and ROS 2.

## 1. Key Data Entities

-   **Robot State**: Joint positions, velocities, efforts; link poses (position and orientation).
-   **Sensor Data**:
    -   LiDAR: `sensor_msgs/LaserScan`
    -   Depth Camera: `sensor_msgs/Image`, `sensor_msgs/CameraInfo`, `sensor_msgs/PointCloud2`
    -   IMU: `sensor_msgs/Imu`
-   **Control Commands**:
    -   Joint commands: `trajectory_msgs/JointTrajectory`, `std_msgs/Float64MultiArray`
    -   Velocity commands: `geometry_msgs/Twist`
-   **Simulation State**: Model poses, world properties (gravity).
-   **Environment Data**: Static and dynamic object poses, properties.

## 2. ROS 2 Message Types & Data Structures

Define the specific ROS 2 message types used for communication and any custom message definitions.

### Robot Control
-   **`geometry_msgs/Twist`**: For base velocity commands.
    -   `linear.x`, `linear.y`, `linear.z` (m/s)
    -   `angular.x`, `angular.y`, `angular.z` (rad/s)
-   **`trajectory_msgs/JointTrajectory`**: For commanding sequences of joint states.
    -   `joint_names` (string array)
    -   `points` (JointTrajectoryPoint array, with positions, velocities, accelerations, effort, time_from_start)

### Sensor Data
-   **`sensor_msgs/LaserScan`**: For LiDAR data.
    -   `header` (`std_msgs/Header`)
    -   `angle_min`, `angle_max`, `angle_increment` (rad)
    -   `time_increment`, `scan_time` (s)
    -   `range_min`, `range_max` (m)
    -   `ranges` (float array, m)
    -   `intensities` (float array)
-   **`sensor_msgs/Image`**: For camera image data.
    -   `header` (`std_msgs/Header`)
    -   `height`, `width` (pixels)
    -   `encoding` (string, e.g., "bgr8", "mono8")
    -   `is_bigendian` (uint8)
    -   `step` (uint32, full row length in bytes)
    -   `data` (uint8 array)
-   **`sensor_msgs/PointCloud2`**: For 3D point cloud data from depth cameras.
    -   `header` (`std_msgs/Header`)
    -   `height`, `width` (pixels)
    -   `fields` (PointField array)
    -   `is_bigendian`, `point_step`, `row_step` (uint8, uint32, uint32)
    -   `data` (uint8 array)
    -   `is_dense` (bool)
-   **`sensor_msgs/Imu`**: For IMU data.
    -   `header` (`std_msgs/Header`)
    -   `orientation` (`geometry_msgs/Quaternion`)
    -   `angular_velocity` (`geometry_msgs/Vector3`)
    -   `linear_acceleration` (`geometry_msgs/Vector3`)

## 3. Communication Flows

Describe the primary communication paths and topics/services/actions.

### Gazebo to ROS 2
-   **Joint States**: `/joint_states` (`sensor_msgs/JointState`)
-   **Sensor Data**: `/scan`, `/camera/depth/image_raw`, `/imu/data`
-   **Model States**: `/gazebo/model_states` (`gazebo_msgs/ModelStates`)

### ROS 2 to Gazebo
-   **Control Commands**: `/cmd_vel`, `/joint_command`
-   **Simulation Commands**: `/gazebo/set_model_state`, `/gazebo/reset_world` (services)

### Unity to ROS 2
-   **Robot State (Feedback)**: `/unity_robot/joint_states`, `/unity_robot/pose`
-   **Sensor Data (e.g., from virtual Unity sensors)**: Custom topics if Unity-specific sensors are implemented.
-   **User Input/HMI events**: Custom topics/services for button presses, slider values, etc.

### ROS 2 to Unity
-   **Control Commands**: `/cmd_vel`, `/joint_command`
-   **Simulation Commands**: `/unity_sim/reset` (service)
-   **Visualization Data**: `/global_path`, `/local_costmap`

## 4. Database Schema (if applicable)

If any persistent data storage is involved (e.g., for simulation logs, recorded trajectories), describe its schema here.

## 5. Data Flow Diagram (Conceptual)

*(Placeholder for a diagram illustrating the data flow between Gazebo, Unity, ROS 2, and any external components. This can be a text-based description for now, or a reference to an external diagram.)*
