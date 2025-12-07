---
id: 2
title: Generate Curriculum Implementation Tasks
stage: tasks
date: 2025-12-07
surface: agent
model: claude-sonnet-4-5-20250929
feature: master
branch: main
user: user
command: /sp.tasks
labels: ["curriculum", "robotics", "ros2", "task-generation", "milestones"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - specs/master/tasks.md
tests:
  - none
---

## Prompt

/sp.tasks Tasks, milestones, and actionable commands


Milestone 0 — Setup (Week 0):
- Repo: `git clone <course-template>`
- Create team folder: `teams/<team-name>/`
- Install ROS 2 toolchain, rclpy, colcon, Gazebo, and Isaac Sim. (Provide distro-agnostic instructions.)
- Verify: `ros2 topic list` (should run without error) and `gazebo --version`.


Milestone 1 — ROS 2 Foundation (Weeks 1-3):
- Deliver: ROS 2 package `humanoid_base` containing URDF/xacro, joint controllers, and minimal rclpy teleop node.
- Required files: `package.xml`, `CMakeLists.txt`, `urdf/humanoid.xacro`, `launch/bringup.launch.py`.
- Example rclpy node outline (publish/subscribe):
- `cmd_vel` style topic for walking commands
- `joint_states` publisher


Commands and checks:
- Build: `colcon build --packages-select humanoid_base`.
- Run bringup: `ros2 launch humanoid_base bringup.launch.py`.
- Check topics: `ros2 topic echo /joint_states`.


Milestone 2 — Digital Twin (Weeks 3-6):
- Deliver: Gazebo world `home_world.world`, a simulated humanoid that uses `ros2_control` controllers, and Unity scene for HRI demo.
- Simulated sensors: depth camera (RGB-D), LiDAR (2D/3D), IMU.
- Tasks:
- Connect URDF to Gazebo via `ros2_control` plugins
- Spawn robot: `ros2 run gazebo_ros spawn_entity.py -file urdf/humanoid.urdf -entity humanoid`
- Validate physics: test falling, balance, and simple walk-traj following via joint trajectory controllers.


Milestone 3 — Isaac Sim and Perception (Weeks 6-9):
- Deliver: Isaac Sim scene for photorealistic testing and synthetic dataset generation. Provide scripts to generate labeled images for object detection and segmentation.
- Tasks:
- Use Isaac ROS nodes for camera input and VSLAM pipeline (where available).
- Export synthetic datasets and provide annotation manifests.


Milestone 4 — VLA and Integration (Weeks 9-11):
- Deliver: LLM-based planner that converts natural language into a ROS 2 action sequence.
- Example fast path: "Pick up the red cup on the table" -> perception (detect cup) -> navigation (path plan to table) -> manipulation (pick via pregrasp/trajectory).
- Integration tests:
- `ros2 action send_goal /navigate_to Nav2Action "{pose: ...}"`
- `ros2 service call /manipulator/pick std_srvs/Trigger "{}"`


Capstone Demo (Week 12):
- Deliverables: recorded demo (video), live simulation demo, final report (PDF) with embedded citations, and the repository.
- Acceptance test: Autonomous run where a voice command triggers a full pipeline end-to-end in simulation.

## Response snapshot

Generated comprehensive tasks.md file for Physical AI & Humanoid Robotics Curriculum with 184 tasks organized across 6 major milestones:

**Task Structure:**
- Milestone 0 — Setup (11 tasks): Repository initialization, ROS 2/Gazebo/Isaac Sim installation, Docker images, validation
- Milestone 1 — Module 1 ROS 2 Foundation (31 tasks): Lessons, 3 labs (custom node, service, URDF humanoid), examples, grading rubrics
- Milestone 2 — Module 2 Digital Twin (34 tasks): Gazebo fundamentals, Unity integration, sensor simulation, 3 labs with solutions
- Milestone 3 — Module 3 Isaac Sim (35 tasks): Photorealistic simulation, synthetic data generation, sensor fusion, 3 labs
- Milestone 4 — Module 4 VLA (30 tasks): Vision-language grounding, LLM task planning, CLIP/LangChain integration, 2 labs
- Capstone Demo (25 tasks): Starter template, 5 task scenarios, evaluation rubrics, acceptance testing
- Final Phase (18 tasks): Documentation, validation, repository finalization

**Key Features:**
- 46 tasks marked [P] for parallel execution
- Clear file paths for all deliverables (lessons, labs, examples, solutions)
- Sequential dependencies: Setup → Module 1 → Module 2 → Module 3 → Module 4 → Capstone
- Each module includes: lesson materials, lab specifications, starter code, grading rubrics, sample solutions
- Capstone includes: ROS 2 workspace template, Gazebo/Isaac environments, 5 task scenarios, evaluation framework

**Implementation Strategy:**
- MVP: Setup + Module 1 (test with pilot cohort first)
- Incremental delivery: Deploy one module at a time, gather feedback
- Parallel team strategy: Multiple developers can work on different modules after Setup complete

All tasks follow strict checklist format with IDs (T001-T184), parallel markers [P], exact file paths, and clear acceptance criteria.

## Outcome

- ✅ Impact: Created comprehensive implementation plan for Physical AI curriculum with 184 actionable tasks across 6 milestones
- 🧪 Tests: No tests - this is a planning/organization task
- 📁 Files: Created specs/master/tasks.md (18KB, 840 lines)
- 🔁 Next prompts: Use this tasks.md to implement curriculum materials sequentially (start with Milestone 0 and Milestone 1)
- 🧠 Reflection: Successfully generated tasks based on existing planning artifacts (plan.md, module contracts, data-model.md, research.md) since spec.md with user stories was not available. Organized by milestones/modules rather than user stories, which fits the educational curriculum structure better.

## Evaluation notes (flywheel)

- Failure modes observed: Initial prerequisite check failed because branch was 'main' but feature directory was 'master'. Adapted by directly reading available design documents.
- Graders run and results (PASS/FAIL): PASS - All tasks follow required checklist format: `- [ ] TID [P?] Description with file path`
- Prompt variant (if applicable): N/A - Standard /sp.tasks workflow
- Next experiment (smallest change to try): Consider creating spec.md with user stories for capstone project to enable story-driven task organization for student projects
