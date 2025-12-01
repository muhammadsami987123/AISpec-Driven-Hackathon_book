"""
VSLAM node stub.

In a full implementation this node would fuse visual and IMU data to
estimate robot pose and build a map. Here we limit ourselves to logging
callbacks to keep the focus on system wiring.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu


class VSLAMNode(Node):
  def __init__(self) -> None:
    super().__init__("vslam_node")
    self.create_subscription(Image, "/camera/rgb", self._on_rgb, 10)
    self.create_subscription(Imu, "/imu/data", self._on_imu, 10)
    self.get_logger().info("VSLAMNode initialized (stub).")

  def _on_rgb(self, msg: Image) -> None:  # noqa: ARG002
    self.get_logger().debug("VSLAM received RGB frame.")

  def _on_imu(self, msg: Imu) -> None:  # noqa: ARG002
    self.get_logger().debug("VSLAM received IMU sample.")


def main(args=None) -> None:
  rclpy.init(args=args)
  node = VSLAMNode()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
  main()


