"""
Simulation-level test scaffolding for autonomous navigation.

Real tests will launch Nav2 and the robot in simulation; here we
limit ourselves to import checks so the project structure is valid.
"""

import importlib


def test_import_planning_and_nav_launch_files() -> None:
  importlib.import_module("physical_ai_robotics_pkg.launch.perception_pipeline")
  importlib.import_module("physical_ai_robotics_pkg.launch.cognition_planning")
  importlib.import_module("physical_ai_robotics_pkg.launch.nav2_bringup")


