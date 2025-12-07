---
id: 1
title: Physical AI Curriculum Plan
stage: plan
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5-20250929
feature: master
branch: master
user: system
command: /sp.plan
labels: ["curriculum", "robotics", "physical-ai", "ros2", "planning"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - specs/master/plan.md
  - specs/master/research.md
  - specs/master/data-model.md
  - specs/master/quickstart.md
  - specs/master/contracts/module1-ros2.md
  - specs/master/contracts/module2-digital-twin.md
  - specs/master/contracts/module3-isaac.md
  - specs/master/contracts/module4-vla.md
  - CLAUDE.md
tests:
  - N/A (planning phase)
---

## Prompt

```text
Quarter Overview
The future of AI extends beyond digital spaces into the physical world. This capstone quarter introduces Physical AI—AI systems that function in reality and comprehend physical laws. Students learn to design, simulate, and deploy humanoid robots capable of natural human interactions using ROS 2, Gazebo, and NVIDIA Isaac.
● Module 1: The Robotic Nervous System (ROS 2)
○ Focus: Middleware for robot control.
○ ROS 2 Nodes, Topics, and Services.
○ Bridging Python Agents to ROS controllers using rclpy.
○ Understanding URDF (Unified Robot Description Format) for humanoids.
● Module 2: The Digital Twin (Gazebo & Unity)
○ Focus: Physics simulation and environment building.
○ Simulating physics, gravity, and collisions in Gazebo.
○ High-fidelity rendering and human-robot interaction in Unity.
○ Simulating sensors: LiDAR, Depth Cameras, and IMUs.
● Module 3: The AI-Robot Brain (NVIDIA Isaac™)
○ Focus: Advanced perception and training.
○ NVIDIA Isaac Sim: Photorealistic sim
```

## Response snapshot

Successfully created comprehensive implementation plan for Physical AI & Humanoid Robotics curriculum spanning 4 modules (ROS 2, Gazebo/Unity, NVIDIA Isaac, Vision-Language-Action).

**Key Deliverables**:

1. **plan.md**: Complete implementation plan with:
   - Technical context (Python 3.10+, ROS 2, Gazebo, Unity, Isaac Sim)
   - Constitution check (all principles validated)
   - Project structure (curriculum repository layout)
   - Phase 0-2 workflow defined

2. **research.md**: Resolved all NEEDS CLARIFICATION items:
   - Module 4: VLA architecture using CLIP + LangChain + ROS 2
   - Capstone: Household assistant robot (3 of 5 tasks)
   - Prerequisites: AI bootcamp graduates, no prior robotics
   - Assessment: 80% hands-on labs, 20% capstone
   - Hardware tiers: Cloud/budget/optimal ($0-2000+)
   - Docker strategy for reproducible environments

3. **data-model.md**: Defined curriculum entities:
   - 10 core entities (Curriculum, Module, Lesson, Lab, Assessment, Student, Project, Technology, CodeExample, Rubric)
   - Entity relationships and validation rules
   - State machines (student progress, lab submissions, capstone)
   - Storage formats (YAML for curriculum, DB for students)

4. **contracts/*.md**: Learning objectives and deliverables for each module:
   - Module 1 (ROS 2): 3 labs, URDF humanoid modeling
   - Module 2 (Gazebo/Unity): Sensor simulation, digital twin
   - Module 3 (Isaac Sim): Photorealistic sim, synthetic data, perception
   - Module 4 (VLA): CLIP grounding, LLM task planning

5. **quickstart.md**: Environment setup guide:
   - 3 installation methods (Docker, native, cloud)
   - Hardware requirements (3 tiers)
   - Docker images for ROS 2, Isaac, VLA
   - Troubleshooting and validation checklist

**Architecture Decisions**:
- Dual-track simulation: Gazebo (required), Isaac Sim (optional GPU)
- VLA principles-based teaching (not proprietary RT-1/RT-2)
- Multi-tier hardware strategy (accessibility for all students)
- Docker-first for reproducibility

**Constitution Alignment**: All principles validated (technical accuracy, real-world alignment, clarity, embodied intelligence)

## Outcome

- ✅ Impact: Complete planning artifacts for Physical AI curriculum (4 modules + capstone)
- 🧪 Tests: N/A (planning phase, implementation in /sp.tasks)
- 📁 Files: 9 files created (plan, research, data-model, quickstart, 4 contracts, CLAUDE.md updated)
- 🔁 Next prompts: `/sp.tasks` to generate implementation tasks, `/sp.clarify` if spec adjustments needed
- 🧠 Reflection: Successfully resolved incomplete Module 4 and Capstone details through comprehensive research. Multi-tier hardware strategy ensures accessibility while maintaining technical rigor.

## Evaluation notes (flywheel)

- Failure modes observed: None. Input was incomplete (Module 4 cut off) but was successfully inferred and expanded based on curriculum context.
- Graders run and results (PASS/FAIL): PASS - All constitutional principles validated, all NEEDS CLARIFICATION items resolved
- Prompt variant (if applicable): N/A (first planning iteration)
- Next experiment (smallest change to try): Consider adding Module 5 for real hardware deployment if curriculum extends beyond 10 weeks
