from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
  """Launch cognition and planning nodes."""
  return LaunchDescription(
    [
      Node(
        package="physical_ai_robotics_pkg",
        executable="whisper_node.py",
        name="whisper_transcription_node",
        output="screen",
      ),
      Node(
        package="physical_ai_robotics_pkg",
        executable="llm_intent_node.py",
        name="llm_intent_node",
        output="screen",
      ),
      Node(
        package="physical_ai_robotics_pkg",
        executable="planning_node.py",
        name="planning_node",
        output="screen",
      ),
    ]
  )


