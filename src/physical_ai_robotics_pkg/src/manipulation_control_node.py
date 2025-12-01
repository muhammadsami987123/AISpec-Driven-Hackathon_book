"""
Manipulation control node stub.

Would normally compute and publish joint trajectories for reaching and
grasping while listening to force/torque feedback. Here we just
publish empty trajectories to demonstrate topic wiring.
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory


class ManipulationControlNode(Node):
  def __init__(self) -> None:
    super().__init__("manipulation_control_node")
    self.pub = self.create_publisher(
      JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10
    )
    self.create_timer(3.0, self._publish_stub_trajectory)
    self.get_logger().info("ManipulationControlNode initialized (stub).")

  def _publish_stub_trajectory(self) -> None:
    self.pub.publish(JointTrajectory())


def main(args=None) -> None:
  rclpy.init(args=args)
  node = ManipulationControlNode()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
  main()


