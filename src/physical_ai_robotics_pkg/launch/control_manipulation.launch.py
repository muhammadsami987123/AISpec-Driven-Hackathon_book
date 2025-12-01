from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
  """Launch control and manipulation nodes."""
  return LaunchDescription(
    [
      Node(
        package="physical_ai_robotics_pkg",
        executable="biped_gait_node.py",
        name="biped_gait_node",
        output="screen",
      ),
      Node(
        package="physical_ai_robotics_pkg",
        executable="manipulation_control_node.py",
        name="manipulation_control_node",
        output="screen",
      ),
      Node(
        package="physical_ai_robotics_pkg",
        executable="task_executive_node.py",
        name="task_executive_node",
        output="screen",
      ),
    ]
  )


