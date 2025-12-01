"""
Task executive node stub.

Sequences high-level actions such as navigation and manipulation in
response to `RobotIntent` messages. Here we only log that an intent
was received.
"""

import rclpy
from rclpy.node import Node
from physical_ai_robotics_pkg.msg import RobotIntent  # type: ignore[import]


class TaskExecutiveNode(Node):
  def __init__(self) -> None:
    super().__init__("task_executive_node")
    self.create_subscription(RobotIntent, "/robot/llm_intent", self._on_intent, 10)
    self.get_logger().info("TaskExecutiveNode initialized (stub).")

  def _on_intent(self, msg: RobotIntent) -> None:  # noqa: ARG002
    self.get_logger().info("TaskExecutive received intent (stub sequencing).")


def main(args=None) -> None:
  rclpy.init(args=args)
  node = TaskExecutiveNode()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
  main()


