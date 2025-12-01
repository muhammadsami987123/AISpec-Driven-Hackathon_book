"""
Smoke test placeholder for the perception pipeline.

These tests are meant to be run inside a ROS 2 environment; for now
we simply assert that the module imports succeed.
"""

import importlib


def test_import_perception_nodes() -> None:
  for module in [
    "physical_ai_robotics_pkg.src.realsense_node",
    "physical_ai_robotics_pkg.src.lidar_node",
    "physical_ai_robotics_pkg.src.perception_node",
    "physical_ai_robotics_pkg.src.vslam_node",
  ]:
    importlib.import_module(module)


