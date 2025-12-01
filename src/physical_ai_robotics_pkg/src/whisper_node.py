"""
Whisper transcription node stub.

Converts microphone audio (not wired here) into a text topic
`/robot/voice_command`. This scaffolding focuses on topic shape rather
than real model integration.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class WhisperTranscriptionNode(Node):
  def __init__(self) -> None:
    super().__init__("whisper_transcription_node")
    self.pub = self.create_publisher(String, "/robot/voice_command", 10)
    self.create_timer(5.0, self._publish_stub_command)
    self.get_logger().info("WhisperTranscriptionNode initialized (stub).")

  def _publish_stub_command(self) -> None:
    msg = String()
    msg.data = "navigate to the table"
    self.get_logger().info(f"Publishing stub voice command: {msg.data}")
    self.pub.publish(msg)


def main(args=None) -> None:
  rclpy.init(args=args)
  node = WhisperTranscriptionNode()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
  main()


