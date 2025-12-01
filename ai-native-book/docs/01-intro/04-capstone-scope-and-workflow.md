---
id: capstone-scope-and-workflow
title: Capstone Scope and Hybrid Workflow
sidebar_label: Capstone Scope & Workflow
---

Chapter&nbsp;1 Topic&nbsp;4 connects the research decisions to the concrete
scope and workflow of your **Autonomous Humanoid** capstone.

## 1. What the Capstone Actually Builds

Based on the research, the capstone is a **hybrid robotics/AI system** that:

- Runs a humanoid (or humanoid‑like) robot in high‑fidelity simulation and,
  where available, on real hardware.
- Processes **voice commands** with Whisper and LLMs, turning natural
  language into structured robot intents.
- Performs **perception** using RGB‑D and IMU data (and optionally lidar) to
  detect, localize, and track objects and obstacles.
- Uses **VSLAM and mapping** to answer “Where am I?” and “What does my world
  look like?”.
- Employs **Nav2‑based planning and control** to move safely through complex
  environments.
- Executes **manipulation behaviors**—reaching, grasping, placing—with
  feedback from force/torque and joint sensors.

The end result is not a single demo script, but a *reusable ROS 2 package
and node graph* you can extend and adapt.

## 2. Why “Hybrid: Simulation and Physical Deployment” Matters

The project is explicitly framed as:

> Robotics/AI System (Hybrid: Simulation & Physical Deployment)

That means:

- **Simulation‑only is not enough**: You must design with an eye toward real
  sensors, friction, latency, and failures—even if you only deploy to
  physical hardware in final stages or shared lab time.
- **Hardware‑only is unrealistic**: Iterating purely on robots is too slow,
  risky, and expensive. Digital twins are the default environment for
  development and most testing.
- **Interfaces must bridge both worlds**: Node boundaries, topic contracts,
  and configuration files should support swapping between simulated and
  physical devices with minimal code changes.

Throughout the course you will ask, for each subsystem: *How does this run
in Gazebo/Isaac? How does it change when I move to the Edge Kit or a real
humanoid?*

## 3. End‑to‑End Workflow: From Research to Robot

The research phase implies a repeatable workflow you will follow at smaller
scales for each feature:

1. **Design and justification**  
   Clarify requirements and constraints, and choose tools (e.g., ROS 2
   packages, Isaac extensions, message definitions) with explicit rationale.

2. **Simulation‑first implementation**  
   Implement nodes and launch files against simulated sensors and robots.
   Use Gazebo/Isaac to iterate quickly on perception, planning, and control.

3. **Testing and validation**  
   Use `pytest`, ROS 2 test tools, and scenario‑based simulation tests to
   validate behavior and performance against clear success criteria.

4. **Edge deployment and tuning**  
   Move components that need to run on the **Jetson Orin Nano** or similar
   edge hardware, respecting its compute and memory limits.

5. **Optional physical robot trials**  
   For labs with access to Unitree or similar platforms, run carefully
   staged trials, logging everything for later analysis.

This loop repeats as you add capabilities (new sensors, behaviors, or VLA
skills), always grounded in the original research constraints.

## 4. How Scope Stays Ambitious but Achievable

The research defines a scope that is both **comprehensive and bounded**:

- Comprehensive because it spans the full stack:
  perception → mapping → planning → control → VLA.
- Bounded because:
  - You focus on one primary robot archetype (a humanoid).
  - You rely on existing libraries and stacks (ROS 2, Nav2, Isaac) instead
    of re‑inventing them from scratch.
  - Data storage and large‑scale infrastructure are intentionally kept out of
    the critical path.

In practice, this means you will:

- Own the *integration and glue code* that turns individual tools into a
  coherent humanoid system.
- Make deliberate trade‑offs when time or compute is limited, while staying
  within the original performance and ethical constraints.

## 5. How This Chapter Prepares You

By the end of Chapter&nbsp;1, you should have:

- A clear picture of **what** you are building (the Autonomous Humanoid
  system).
- An understanding of **where** it runs (workstation, Edge Kit, optional
  physical robots).
- A sense of **how** research‑driven constraints (performance, ethics,
  resource limits) shape every design decision.

The remaining chapters will dive into each pillar—perception, control,
simulation, and VLA—with this capstone scope and workflow as the guiding
through‑line.


