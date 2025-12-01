"""
Inverse kinematics solver scaffolding.

For the course, this module will host IK algorithms for the humanoid's
arms and hands. We provide a minimal API surface and documentation so
that unit tests and higher-level nodes can depend on it.
"""

from typing import List, Sequence


class KinematicsSolver:
  """Placeholder IK solver with a simple method signature."""

  def __init__(self, joint_names: Sequence[str]) -> None:
    self.joint_names = list(joint_names)

  def solve(self, target_pose) -> List[float]:  # noqa: ANN001
    """
    Compute joint positions that achieve the given end-effector pose.

    Parameters
    ----------
    target_pose:
      Arbitrary pose representation (e.g., geometry_msgs/Pose).

    Returns
    -------
    List[float]
      One position per joint in `self.joint_names`.
    """

    # Stub implementation: return zeros of appropriate length.
    return [0.0 for _ in self.joint_names]


__all__ = ["KinematicsSolver"]


