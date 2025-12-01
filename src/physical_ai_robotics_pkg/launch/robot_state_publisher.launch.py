from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
  """Launch file for the basic robot_state_publisher scaffolding."""
  return LaunchDescription(
    [
      Node(
        package="physical_ai_robotics_pkg",
        executable="robot_state_publisher_node.py",
        name="robot_state_publisher_node",
        output="screen",
      )
    ]
  )


