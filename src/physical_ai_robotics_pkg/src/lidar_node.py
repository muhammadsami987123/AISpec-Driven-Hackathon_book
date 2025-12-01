"""
Optional LiDAR node scaffolding.

Publishes a stub PointCloud2 message on `/lidar/points`. This lets
students focus on wiring the pipeline before integrating a real LiDAR
or simulated sensor stream.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class LidarNode(Node):
  def __init__(self) -> None:
    super().__init__("lidar_node")
    self.points_pub = self.create_publisher(PointCloud2, "/lidar/points", 10)
    self.create_timer(1.0, self._publish_stub_cloud)
    self.get_logger().info("LidarNode initialized (stub).")

  def _publish_stub_cloud(self) -> None:
    self.points_pub.publish(PointCloud2())


def main(args=None) -> None:
  rclpy.init(args=args)
  node = LidarNode()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
  main()


