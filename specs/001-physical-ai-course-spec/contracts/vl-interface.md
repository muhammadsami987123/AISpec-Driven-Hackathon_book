# Vision-Language-Action (VLA) Interface Contract

This document outlines the high-level conceptual contract for the Vision-Language-Action (VLA) pipeline, focusing on the integration of natural language understanding with robot perception and action.

## 1. Voice-to-Intent (V2I) Module

### Input
- **Source**: Microphone audio stream
- **Format**: Raw audio (e.g., WAV, FLAC)
- **Interface**: Internal stream/topic (e.g., `/audio/raw`)

### Processing
- **Component**: Speech-to-Text (e.g., OpenAI Whisper)
- **Output**: Transcribed text
- **Interface**: ROS 2 Topic: `/robot/voice_command` (`std_msgs/msg/String`)

### Output
- **Component**: Large Language Model (LLM) for Intent Parsing (e.g., GPT-4)
- **Output**: Structured Robot Intent
- **Interface**: ROS 2 Topic: `/robot/llm_intent` (Custom Message: `RobotIntent`)
    - `command_id`: string
    - `action_type`: string (e.g., "navigate", "manipulate", "perceive", "report")
    - `target_object_id`: string (optional, UUID or semantic label from perception system)
    - `target_location`: `geometry_msgs/msg/PoseStamped` or semantic label (optional)
    - `parameters`: JSON string for additional action-specific parameters (e.g., "grasp_force": 0.5)
    - `confidence`: float (0.0 - 1.0)

## 2. Perception-to-Action (P2A) Module

### Input
- **Source**: Robot Sensors (Camera, Depth, LiDAR, IMU)
- **Format**: Raw sensor data (e.g., `sensor_msgs/msg/Image`, `sensor_msgs/msg/PointCloud2`, `sensor_msgs/msg/Imu`)
- **Interface**: ROS 2 Topics (`/camera/rgb`, `/camera/depth`, `/lidar/points`, `/imu/data`)

### Processing
- **Component**: Perception System (Object Detection, SLAM, 6D Pose Estimation)
- **Output**: Environmental model, localized objects, robot pose
- **Interface**: Internal ROS 2 Topics/Services (e.g., `/perception/objects`, `/map_server/map`, `/robot/pose`)

### Input
- **Source**: Structured Robot Intent from V2I Module (`/robot/llm_intent`)
- **Format**: `RobotIntent` custom message

### Processing
- **Component**: Task Planner / Executive (e.g., Nav2, MoveIt! for ROS 2)
- **Output**: Sequence of low-level robot actions/trajectories
- **Interface**: ROS 2 Actions/Services (e.g., `/navigation/navigate_to_pose` (Action), `/manipulation/grasp_object` (Service))

### Output
- **Component**: Robot Control Systems (Joint Controllers, Gait Generators)
- **Output**: Motor commands
- **Interface**: ROS 2 Topics (e.g., `/joint_trajectory_controller/joint_trajectory`)

## 3. Feedback Loop

### Input
- **Source**: Robot Joint States, Force/Torque Sensors, Vision Feedback
- **Format**: `sensor_msgs/msg/JointState`, `geometry_msgs/msg/WrenchStamped`, `sensor_msgs/msg/Image`
- **Interface**: ROS 2 Topics (e.g., `/robot_state_publisher/joint_states`, `/force_torque_sensors/wrist_left`, `/camera/feedback`)

### Processing
- **Component**: Action Monitor, Error Detection, LLM for Verbal Feedback
- **Output**: Updated action status, error reports, natural language response to user
- **Interface**: ROS 2 Topic (`/robot/action_status`), ROS 2 Service (`/robot/speak` (string))