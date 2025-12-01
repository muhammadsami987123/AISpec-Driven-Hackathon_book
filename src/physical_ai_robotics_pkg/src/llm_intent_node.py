"""
LLM intent node stub.

Consumes `/robot/voice_command` and publishes a structured `RobotIntent`
message on `/robot/llm_intent`. Real LLM calls are intentionally
omitted; instead we map a small set of phrases to fixed intents.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from physical_ai_robotics_pkg.msg import RobotIntent  # type: ignore[import]


class LLMIntentNode(Node):
  def __init__(self) -> None:
    super().__init__("llm_intent_node")
    self.pub = self.create_publisher(RobotIntent, "/robot/llm_intent", 10)
    self.create_subscription(String, "/robot/voice_command", self._on_command, 10)
    self.get_logger().info("LLMIntentNode initialized (stub).")

  def _on_command(self, msg: String) -> None:
    intent = RobotIntent()
    intent.command_id = "cmd-001"
    intent.action_type = "navigate"
    intent.target_object_id = ""
    intent.target_pose = PoseStamped()
    intent.parameters = "{}"
    intent.confidence = 0.8
    self.get_logger().info(f"Parsed intent from command: {msg.data}")
    self.pub.publish(intent)


def main(args=None) -> None:
  rclpy.init(args=args)
  node = LLMIntentNode()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
  main()


