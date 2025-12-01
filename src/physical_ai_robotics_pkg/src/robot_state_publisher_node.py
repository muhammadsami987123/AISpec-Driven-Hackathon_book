"""
Basic robot_state_publisher-style node.

This placeholder assumes that a separate joint state source exists
and focuses on providing a concrete file for course scaffolding.
"""

import rclpy
from rclpy.node import Node


class RobotStatePublisherNode(Node):
  """Stub node for publishing robot state or wiring robot_state_publisher."""

  def __init__(self) -> None:
    super().__init__("robot_state_publisher_node")
    self.get_logger().info("RobotStatePublisherNode initialized (stub).")


def main(args=None) -> None:
  rclpy.init(args=args)
  node = RobotStatePublisherNode()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
  main()


