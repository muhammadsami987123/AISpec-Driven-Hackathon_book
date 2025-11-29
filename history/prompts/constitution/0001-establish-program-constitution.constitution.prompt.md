---
id: 0001
title: Establish Program Constitution
stage: constitution
date: 2025-11-29
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: user
command: /sp.constitution
labels: ["docs", "constitution"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - N/A
---

## Prompt

# Physical AI & Humanoid Robotics
## Program Constitution

**Preamble**

We, the faculty and students of the Physical AI & Humanoid Robotics capstone program, establish this constitution to guide our shared mission of bridging the gap between digital intelligence and embodied systems. We recognize that the future of AI extends beyond screens and into the physical world—a transition that demands rigorous technical discipline, ethical responsibility, and collaborative excellence.

This constitution serves as our social contract: a binding agreement on principles, responsibilities, and expectations that govern how we work together to advance the science of embodied intelligence.

---

## I. Foundational Principles

### 1.1 Mission
To educate the next generation of Physical AI engineers who can design, simulate, and deploy humanoid robots capable of operating autonomously in human-centered environments.

### 1.2 Core Values

**Embodied Excellence** — We believe that understanding AI requires touching the physical world. Simulation without deployment, and theory without practice, are incomplete.

**Collaborative Intelligence** — This program is built on the principle that complex systems require diverse expertise. We value peer learning, code reviews, and knowledge-sharing across teams.

**Responsible Deployment** — We acknowledge that robots operating in human spaces carry ethical weight. All designs prioritize safety, transparency, and beneficial outcomes.

**Open Standards** — We commit to industry-standard tools (ROS 2, NVIDIA Isaac, Gazebo) and reproducible practices, ensuring our work contributes to the broader robotics ecosystem.

---

## II. Learning Objectives & Competencies

Upon completion of this program, students will demonstrate mastery in:

- **Physical AI Fundamentals**: Understanding embodied intelligence, sensor integration, and the translation of digital algorithms into physical action.
- **ROS 2 Architecture**: Designing modular, distributed systems using nodes, topics, services, and actions.
- **Simulation & Digital Twins**: Creating photorealistic simulations that enable safe training before real-world deployment.
- **Perception & Navigation**: Implementing VSLAM, computer vision, and path planning for autonomous operation.
- **Language-Guided Reasoning**: Integrating large language models to convert natural language commands into executable robot behaviors.
- **Sim-to-Real Transfer**: Bridging the gap between simulation and physical deployment with robust, production-ready code.

---

## III. Hardware Access & Lab Governance

### 3.1 Resource Allocation

The program operates under a **resource-constrained reality**. High-performance compute, sensors, and robots are shared infrastructure. Access is governed by:

- **First-Come, First-Served Reservation System**: Students book lab resources 48 hours in advance.

- **Equitable Distribution**: No single team monopolizes GPU time, sensors, or robots during the quarter.

- **Peak vs. Off-Peak Usage**: Students are encouraged to run long training jobs during off-peak hours (nights/ weekends).

### 3.2 Hardware Responsibility

Students working with lab hardware agree to:

- **Careful Handling**: Robots, sensors, and workstations are expensive and irreplaceable mid-quarter. Damage due to negligence incurs responsibility.

- **Proper Documentation**: All hardware issues (non-responsive GPUs, USB connection failures, sensor drift) must be logged in the shared lab wiki within 24 hours.

- **Clean Shutdown**: All processes must be cleanly terminated. No force-kills of running simulations or arbitrary unplugging of devices.

- **Calibration & Maintenance**: Students performing experiments with sensors (IMUs, cameras) own the calibration process and document deviations.

### 3.3 Cloud vs. Local Execution

- **Simulation Training**: Students may use AWS cloud instances (within institutional budget: $205/quarter per cohort) for long-running physics simulations.

- **Model Deployment

## Response snapshot

The program constitution has been successfully updated to version 1.0.0. This is an initial full version based on the comprehensive content provided.

Please review the following template files to ensure they align with the new constitution:
- `.specify/templates/plan-template.md`
- `.specify/templates/spec-template.md`
- `.specify/templates/tasks-template.md`
- All command files in `.specify/templates/commands/`

Suggested commit message: `docs: establish program constitution v1.0.0 (initial full version)`

## Outcome

- ✅ Impact: Program constitution updated to v1.0.0, establishing foundational principles, learning objectives, and governance for the Physical AI & Humanoid Robotics capstone program.
- 🧪 Tests: N/A (documentation update)
- 📁 Files: .specify/memory/constitution.md updated
- 🔁 Next prompts: Review linked templates for consistency with new constitution.
- 🧠 Reflection: Successfully updated the constitution and provided detailed instructions for follow-up consistency checks. The bash script for PHR creation failed, prompting fallback to agent-native file writing.

## Evaluation notes (flywheel)

- Failure modes observed: Bash script for `create-phr.sh` not found.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Investigate the `create-phr.sh` script path or ensure it's generated/available.
