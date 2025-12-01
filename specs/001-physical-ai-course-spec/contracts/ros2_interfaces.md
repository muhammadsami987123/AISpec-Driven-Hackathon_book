# ROS 2 Interfaces (Conceptual Contracts)

This document outlines the conceptual ROS 2 interfaces that define communication within the Physical AI & Humanoid Robotics system. These are not formal IDL definitions but rather descriptions of the data flowing between key components.

## 1. Sensor Data Topics

### `/camera/rgb`
- **Type**: `sensor_msgs/msg/Image`
- **Description**: Raw RGB image data from the Intel RealSense camera.
- **Publishing Node**: `realsense_node`
- **Subscribing Nodes**: `perception_node` (for object detection), `vslam_node` (for visual SLAM)

### `/camera/depth`
- **Type**: `sensor_msgs/msg/Image` (typically 16-bit or 32-bit float)
- **Description**: Depth image data from the Intel RealSense camera.
- **Publishing Node**: `realsense_node`
- **Subscribing Nodes**: `perception_node` (for 3D object localization), `vslam_node` (for point cloud generation)

### `/lidar/points` (Optional)
- **Type**: `sensor_msgs/msg/PointCloud2`
- **Description**: 3D point cloud data from a LiDAR sensor.
- **Publishing Node**: `lidar_node`
- **Subscribing Nodes**: `navigation_node` (for obstacle avoidance), `mapping_node`

### `/imu/data`
- **Type**: `sensor_msgs/msg/Imu`
- **Description**: Inertial Measurement Unit data (acceleration, angular velocity, orientation).
- **Publishing Node**: `realsense_node` (integrated IMU)
- **Subscribing Nodes**: `control_node` (for balance/gait stability), `vslam_node` (for dead reckoning)

### `/force_torque_sensors/wrist_left` / `/wrist_right`
- **Type**: `geometry_msgs/msg/WrenchStamped`
- **Description**: Force and torque readings from wrist-mounted sensors.
- **Publishing Node**: `robot_hardware_interface` / `simulation_interface`
- **Subscribing Nodes**: `manipulation_node` (for grasping feedback), `control_node` (for collision detection)

## 2. Cognition & Planning Topics/Services/Actions

### `/robot/voice_command`
- **Type**: `std_msgs/msg/String` (or custom message for structured command)
- **Description**: Transcribed voice command text from the natural language interface.
- **Publishing Node**: `whisper_transcription_node`
- **Subscribing Nodes**: `llm_intent_node`

### `/robot/llm_intent`
- **Type**: Custom message (e.g., `physical_ai_msgs/msg/RobotIntent`)
    - `command_id`: string
    - `action_type`: string (e.g., "navigate", "manipulate", "perceive")
    - `target_object_id`: string (optional)
    - `target_pose`: `geometry_msgs/msg/PoseStamped` (optional)
    - `parameters`: string (JSON string for complex arguments)
- **Description**: Parsed intent and high-level action plan from the LLM.
- **Publishing Node**: `llm_intent_node`
- **Subscribing Nodes**: `planning_node`, `task_executive_node`

### `/navigation/goal_pose` (Action Client/Server)
- **Type**: `nav2_msgs/action/NavigateToPose`
- **Description**: Action to send a navigation goal to the robot.
- **Client**: `planning_node`
- **Server**: `nav2_controller_server`

### `/manipulation/grasp_object` (Service)
- **Type**: Custom service (e.g., `physical_ai_srvs/srv/GraspObject`)
    - **Request**:
        - `object_id`: string
        - `grasp_pose`: `geometry_msgs/msg/PoseStamped`
        - `grasp_force`: float
    - **Response**:
        - `success`: bool
        - `message`: string
- **Description**: Service to command the robot to grasp a detected object.
- **Client**: `task_executive_node`
- **Server**: `manipulation_control_node`

## 3. Control & Actuation Topics

### `/joint_trajectory_controller/joint_trajectory`
- **Type**: `trajectory_msgs/msg/JointTrajectory`
- **Description**: Desired joint trajectories for the robot's actuators.
- **Publishing Node**: `control_node` (e.g., `biped_gait_node`, `manipulation_control_node`)
- **Subscribing Nodes**: `hardware_interface` (for real robot), `simulation_interface` (for Gazebo/Isaac Sim)

### `/robot_state_publisher/joint_states`
- **Type**: `sensor_msgs/msg/JointState`
- **Description**: Current joint positions, velocities, and efforts from the robot hardware/simulator.
- **Publishing Node**: `hardware_interface` / `simulation_interface`
- **Subscribing Nodes**: `robot_state_publisher`, `control_node` (for feedback)
