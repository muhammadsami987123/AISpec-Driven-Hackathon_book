"""
Perception node stub for object detection and 3D localization.

Consumes camera RGB/depth streams and would normally produce detections
and localized objects. Here we just log that messages are received so
the course focus can be on wiring the broader pipeline first.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class PerceptionNode(Node):
  def __init__(self) -> None:
    super().__init__("perception_node")
    self.create_subscription(Image, "/camera/rgb", self._on_rgb, 10)
    self.create_subscription(Image, "/camera/depth", self._on_depth, 10)
    self.get_logger().info("PerceptionNode initialized (stub).")

  def _on_rgb(self, msg: Image) -> None:  # noqa: ARG002
    self.get_logger().debug("Received RGB frame.")

  def _on_depth(self, msg: Image) -> None:  # noqa: ARG002
    self.get_logger().debug("Received depth frame.")


def main(args=None) -> None:
  rclpy.init(args=args)
  node = PerceptionNode()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
  main()


