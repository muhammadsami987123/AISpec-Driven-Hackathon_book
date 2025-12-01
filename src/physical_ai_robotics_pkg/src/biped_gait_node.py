"""
Biped gait node stub.

Publishes placeholder joint trajectories. Real gait generation is out
of scope for this scaffolding and will be developed iteratively.
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory


class BipedGaitNode(Node):
  def __init__(self) -> None:
    super().__init__("biped_gait_node")
    self.pub = self.create_publisher(
      JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10
    )
    self.create_timer(2.0, self._publish_stub_trajectory)
    self.get_logger().info("BipedGaitNode initialized (stub).")

  def _publish_stub_trajectory(self) -> None:
    self.pub.publish(JointTrajectory())


def main(args=None) -> None:
  rclpy.init(args=args)
  node = BipedGaitNode()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
  main()


