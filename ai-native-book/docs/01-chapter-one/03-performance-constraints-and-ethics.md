---
id: performance-constraints-and-ethics
title: Performance, Constraints, and Ethics in Physical AI
sidebar_label: Performance & Ethics
---

Chapter&nbsp;1 Topic&nbsp;3 focuses on *how* the course’s technical choices
must perform in the real world, and *why* constraints and ethics are built
into the design from day one.

## 1. Performance Targets for Real Robots

Physical AI systems don’t just need to be “correct” in theory; they must be
**fast and stable enough** to control hardware safely:

- **Real‑time control loops (p95 latency)**  
  Controllers that stabilize a humanoid’s balance or regulate joint torques
  must run at high frequency with tight latency bounds. Occasional spikes
  can mean trips, falls, or damaged hardware.

- **Real‑time sensor processing**  
  Camera frames, depth images, point clouds, and IMU streams arrive
  continuously. Your perception stack must keep up so that downstream
  planners never act on stale world models.

- **60 FPS simulation**  
  High‑frame‑rate simulation in Gazebo or Isaac Sim gives you more accurate
  physics and smoother motion, and produces better training and validation
  data. If your simulation chugs, your controllers and perception models see
  an unrealistic world.

- **Fast natural language understanding**  
  Whisper and LLM‑based intent parsing should respond quickly enough for
  interactive use. Multi‑second delays are acceptable for offline tools,
  but conversational robots need tight, predictable response times.

In this course, you’ll learn to profile these components and reason about
where latency matters most (e.g., control loops and perception) versus where
you can tolerate slower, batched workloads.

## 2. Storage and Data: Why “N/A” Is a Choice

The research explicitly sets **“Storage: N/A”** for the core humanoid stack:

- The robot primarily manipulates *transient* data: sensor frames, control
  commands, and short‑lived internal states.
- Long‑term artifacts—logs, datasets for training, experiment results—can be
  stored in files or external systems, decoupled from the real‑time control
  loop.

This design has several implications:

- The ROS 2 graph stays focused on low‑latency messaging, not heavy
  transactional workloads.
- You avoid coupling robot safety to database availability.
- When you do log data, you do it intentionally: recording topics during
  experiments, exporting traces for offline analysis, and using them in
  your learning pipelines.

Throughout the capstone, you’ll be encouraged to ask: *Does this really need
to be persisted?* If yes, *can I keep it out of the real‑time path?*

## 3. Real‑World Constraints You Must Design For

The course treats constraints as **first‑class design inputs**, not
after‑the‑fact annoyances:

- **Shared, limited hardware**  
  GPUs, edge kits, and robots are shared resources. Your solutions must run
  efficiently and degrade gracefully when compute or bandwidth is tight.

- **Physical wear, risk, and downtime**  
  Every experiment has a non‑zero chance of breaking hardware. That is why
  simulation, staged deployment, and conservative safety limits are part of
  the normal workflow.

- **Timing and variability**  
  Real environments introduce jitter, network delays, mechanical backlash,
  and sensor noise. Designs that are only stable in “perfect lab conditions”
  are considered incomplete.

You’ll see these constraints concretely in lab rubrics and project
requirements: it’s not enough to “make it work on my machine”—it has to work
reliably on shared lab infrastructure and physical robots.

## 4. Ethics and Responsibility in Humanoid Systems

Humanoid robots operate close to humans, often in safety‑critical contexts.
The research phase emphasizes **ethical responsibility** alongside technical
goals:

- **Safety first**  
  Fail‑safe defaults, conservative force limits, and clear emergency stop
  behaviors are non‑negotiable. “It usually doesn’t crash” is not an
  acceptable standard around people.

- **Transparency and explainability**  
  When a humanoid takes an action—especially one driven by an LLM‑based
  planner—users and operators should be able to understand *why*. Logging,
  visualization tools, and clear internal interfaces all support this.

- **Beneficial outcomes**  
  Capstone projects are framed around assisting tasks—logistics, inspection,
  caregiving support—not replacing human agency or creating opaque
  surveillance systems.

You will be asked to justify design decisions in light of these principles:
*How does this choice affect safety? What happens on failure? Who is in
control?*

## 5. Hybrid by Design: Simulation and Deployment Together

Finally, the research describes the project as a **hybrid system**: always
thinking about both simulation and physical deployment:

- Simulation (Gazebo, Isaac Sim) is your **primary sandbox**.
- The Physical AI Edge Kit and optional humanoid hardware are your **reality
  check**.

The core workflow is:

1. Design and test behaviors in high‑fidelity digital twins.
2. Validate performance, robustness, and safety conditions in simulation.
3. Migrate to edge hardware with clear expectations about what may change
   (sensors, latency, friction, unmodeled contacts).

By the end of this chapter, you should see performance, constraints, and
ethics not as extra checkboxes, but as *the context in which all technical
work lives*. The rest of the book will build specific tools—perception,
planning, control, and VLA—within these boundaries.


