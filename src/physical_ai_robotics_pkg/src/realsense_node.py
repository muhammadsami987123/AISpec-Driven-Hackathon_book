"""
Stub implementation of a RealSense sensor node.

Publishes RGB, depth, and IMU topics as described in the contracts.
In the course, students will replace the placeholder timers with
real sensor integration (hardware or simulation bridge).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu


class RealSenseNode(Node):
  def __init__(self) -> None:
    super().__init__("realsense_node")
    self.rgb_pub = self.create_publisher(Image, "/camera/rgb", 10)
    self.depth_pub = self.create_publisher(Image, "/camera/depth", 10)
    self.imu_pub = self.create_publisher(Imu, "/imu/data", 10)

    # Timers publish dummy messages as placeholders.
    self.create_timer(1.0, self._publish_stub_data)
    self.get_logger().info("RealSenseNode initialized (stub).")

  def _publish_stub_data(self) -> None:
    """Publish empty messages to keep the pipeline wired."""
    self.rgb_pub.publish(Image())
    self.depth_pub.publish(Image())
    self.imu_pub.publish(Imu())


def main(args=None) -> None:
  rclpy.init(args=args)
  node = RealSenseNode()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
  main()


