# Data Model for Physical AI & Humanoid Robotics Course

## Entities and Relationships

### 1. HumanoidRobot
- **Description**: The central autonomous agent in the physical environment or simulation.
- **Fields**:
    - `id`: Unique identifier (string)
    - `name`: Human-readable name (string)
    - `kinematic_model`: URDF/XACRO description of the robot's links and joints (string, path to file)
    - `current_pose`: 3D position and orientation (x, y, z, roll, pitch, yaw)
    - `joint_states`: Current angles and velocities for all joints (map of joint_name to value)
    - `health_status`: Operational status (enum: `online`, `offline`, `error`, `low_power`)
- **Relationships**:
    - Has many `Sensor`s
    - Executes `Action`s
    - Interacts with `Environment` `Object`s

### 2. Sensor
- **Description**: Devices used by the `HumanoidRobot` to perceive its environment.
- **Fields**:
    - `id`: Unique identifier (string)
    - `type`: (enum: `microphone`, `camera_rgb`, `camera_depth`, `lidar`, `imu`, `force_torque`)
    - `mount_point`: Location on the robot (string, e.g., "head", "left_wrist")
    - `data_stream_topic`: ROS 2 topic for raw sensor data (string)
    - `calibration_status`: (enum: `calibrated`, `uncalibrated`, `error`)
- **Relationships**:
    - Belongs to one `HumanoidRobot`

### 3. Environment
- **Description**: The physical or simulated space in which the robot operates.
- **Fields**:
    - `id`: Unique identifier (string)
    - `name`: Human-readable name (string)
    - `map_data`: 3D occupancy grid or point cloud data (path to file/data structure)
    - `gravity_vector`: (x, y, z)
    - `friction_coefficient`: (float)
- **Relationships**:
    - Contains many `Object`s

### 4. Object
- **Description**: Manipulable or interactable items within the `Environment`.
- **Fields**:
    - `id`: Unique identifier (string)
    - `name`: Human-readable name (string, e.g., "cup", "table")
    - `mesh_model`: 3D model data (string, path to file)
    - `current_pose`: 3D position and orientation (x, y, z, roll, pitch, yaw)
    - `material_properties`: (e.g., `slippery`, `fragile`, `heavy`)
- **Relationships**:
    - Belongs to one `Environment`

### 5. VoiceCommand
- **Description**: Natural language input from a human user.
- **Fields**:
    - `id`: Unique identifier (string)
    - `raw_audio_path`: Path to recorded audio file (string)
    - `transcribed_text`: Text from speech-to-text (string, e.g., "pick up the cup")
    - `timestamp`: Time of command (datetime)
    - `intent`: Parsed high-level intent (string, e.g., "manipulate", "navigate")
    - `target_object_id`: (Optional) ID of the `Object` referred to in the command (string)
    - `target_location`: (Optional) 3D coordinate or named location (x, y, z / string)

### 6. Action (RobotAction)
- **Description**: A sequence of robot behaviors executed by the `HumanoidRobot`.
- **Fields**:
    - `id`: Unique identifier (string)
    - `type`: (enum: `navigate`, `manipulate`, `perceive`, `communicate`)
    - `status`: (enum: `pending`, `in_progress`, `completed`, `failed`, `cancelled`)
    - `start_time`: (datetime)
    - `end_time`: (datetime, if completed)
    - `target_pose`: (Optional) Desired end-effector or base pose
    - `grasp_configuration`: (Optional) Parameters for gripper control
    - `feedback`: Textual feedback/logs from execution (string)
- **Relationships**:
    - Triggered by a `VoiceCommand` (or internal cognition)
    - Executed by a `HumanoidRobot`

## Data Flow Overview

1.  `VoiceCommand` (audio) -> Speech-to-Text (Whisper) -> `VoiceCommand` (text)
2.  `VoiceCommand` (text) -> LLM (GPT-4) -> Parsed `Intent` & `Action` plan
3.  `Sensor` data (raw) -> ROS 2 (sensor fusion) -> `Environment` map, `Object` localization, `HumanoidRobot` pose
4.  `Action` plan (high-level) -> ROS 2 (Nav2, MoveIt!) -> Low-level `Control` commands
5.  `Control` commands -> `HumanoidRobot` actuators -> Physical motion
6.  `HumanoidRobot` motion -> `Sensor` data (feedback) -> Closed-loop adjustment
