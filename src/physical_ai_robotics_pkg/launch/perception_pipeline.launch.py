from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
  """Launch all perception-related nodes."""
  return LaunchDescription(
    [
      Node(
        package="physical_ai_robotics_pkg",
        executable="realsense_node.py",
        name="realsense_node",
        output="screen",
      ),
      Node(
        package="physical_ai_robotics_pkg",
        executable="lidar_node.py",
        name="lidar_node",
        output="screen",
      ),
      Node(
        package="physical_ai_robotics_pkg",
        executable="perception_node.py",
        name="perception_node",
        output="screen",
      ),
      Node(
        package="physical_ai_robotics_pkg",
        executable="vslam_node.py",
        name="vslam_node",
        output="screen",
      ),
    ]
  )


