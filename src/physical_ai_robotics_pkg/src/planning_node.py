"""
Planning node stub.

Consumes `RobotIntent` messages and would normally create navigation
goals and call Nav2 actions. Here we log received intents only.
"""

import rclpy
from rclpy.node import Node
from physical_ai_robotics_pkg.msg import RobotIntent  # type: ignore[import]


class PlanningNode(Node):
  def __init__(self) -> None:
    super().__init__("planning_node")
    self.create_subscription(RobotIntent, "/robot/llm_intent", self._on_intent, 10)
    self.get_logger().info("PlanningNode initialized (stub).")

  def _on_intent(self, msg: RobotIntent) -> None:  # noqa: ARG002
    self.get_logger().info("Received RobotIntent; planning path (stub).")


def main(args=None) -> None:
  rclpy.init(args=args)
  node = PlanningNode()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
  main()


