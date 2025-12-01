from physical_ai_robotics_pkg.src.kinematics_solver import KinematicsSolver


def test_kinematics_solver_returns_correct_length() -> None:
  joint_names = ["joint1", "joint2", "joint3"]
  solver = KinematicsSolver(joint_names)
  result = solver.solve(target_pose=None)
  assert len(result) == len(joint_names)


