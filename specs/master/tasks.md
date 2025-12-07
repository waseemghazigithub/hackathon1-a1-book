# Tasks: Physical AI & Humanoid Robotics Curriculum

**Branch**: `master` | **Date**: 2025-12-07
**Input**: Design documents from `/specs/master/` (plan.md, contracts/, data-model.md, research.md, quickstart.md)
**Prerequisites**: ROS 2, Gazebo, Isaac Sim, Unity, LLM tools (see quickstart.md)

**Organization**: Tasks are organized by milestones and modules to enable incremental curriculum development and deployment.

---

## Format: `[ID] [P?] Description with file path`

- **[P]**: Can run in parallel (different files, no dependencies)
- Include exact file paths in descriptions
- Path conventions: `curriculum/`, `shared/`, `capstone/`, `docs/`

---

## Milestone 0 — Setup (Week 0)

**Purpose**: Repository initialization and development environment setup

- [ ] T001 Clone course template repository and create team folder at `teams/<team-name>/`
- [ ] T002 [P] Install ROS 2 toolchain (Humble or Iron) and verify with `ros2 topic list`
- [ ] T003 [P] Install rclpy, colcon, and build tools
- [ ] T004 [P] Install Gazebo (Classic 11 or Sim) and verify with `gazebo --version`
- [ ] T005 [P] Install NVIDIA Isaac Sim 2023.1+ (if GPU available) and verify launch
- [ ] T006 [P] Create Docker images for consistent environments (base ROS 2, Isaac, full stack)
- [ ] T007 [P] Write distro-agnostic installation instructions in `docs/setup/installation-guide.md`
- [ ] T008 Create curriculum metadata file at `curriculum/curriculum.yaml`
- [ ] T009 Set up shared resources structure: `shared/urdf-library/`, `shared/ros2-utils/`, `shared/docker/`
- [ ] T010 Configure Git for submissions: user name and email
- [ ] T011 Create validation script to verify all prerequisites in `scripts/validate-setup.sh`

**Checkpoint**: Development environment ready, `ros2 topic list` succeeds, Gazebo launches

---

## Milestone 1 — ROS 2 Foundation (Weeks 1-3)

**Purpose**: Module 1 curriculum materials and lab infrastructure

### Module 1 Structure Setup

- [ ] T012 Create module directory structure at `curriculum/module1-ros2/` with subdirectories: `lessons/`, `examples/`, `labs/`, `resources/`
- [ ] T013 Create module specification file at `curriculum/module1-ros2/module.yaml`
- [ ] T014 Write module README at `curriculum/module1-ros2/README.md` with learning objectives

### Week 1: ROS 2 Fundamentals

**Lesson Materials**:
- [ ] T015 [P] Write lesson: "ROS 2 Architecture Overview" in `curriculum/module1-ros2/lessons/week1-lesson1-architecture.md`
- [ ] T016 [P] Write lesson: "Communication Patterns: Topics and Messages" in `curriculum/module1-ros2/lessons/week1-lesson2-topics.md`
- [ ] T017 [P] Create example: Simple talker/listener in `curriculum/module1-ros2/examples/talker_listener.py`

**Lab 1: Custom ROS 2 Node (5 points)**:
- [ ] T018 Create Lab 1 specification at `curriculum/module1-ros2/labs/lab1.yaml`
- [ ] T019 Create Lab 1 starter package at `curriculum/module1-ros2/labs/lab1-starter/` with `package.xml`, `setup.py`, `README.md`
- [ ] T020 Write Lab 1 instructions in `curriculum/module1-ros2/labs/lab1-instructions.md`
- [ ] T021 Create Lab 1 grading rubric in `curriculum/module1-ros2/labs/lab1-rubric.md`
- [ ] T022 Create sample solution for Lab 1 (for TAs) in `curriculum/module1-ros2/labs/lab1-solution/`

### Week 2: Services and Communication

**Lesson Materials**:
- [ ] T023 [P] Write lesson: "Service Communication Pattern" in `curriculum/module1-ros2/lessons/week2-lesson1-services.md`
- [ ] T024 [P] Write lesson: "Launch Files and Parameters" in `curriculum/module1-ros2/lessons/week2-lesson2-launch.md`
- [ ] T025 [P] Create example: Service client/server in `curriculum/module1-ros2/examples/service_example.py`

**Lab 2: Service Client/Server (5 points)**:
- [ ] T026 Create Lab 2 specification at `curriculum/module1-ros2/labs/lab2.yaml`
- [ ] T027 Create Lab 2 starter package with custom `.srv` definition at `curriculum/module1-ros2/labs/lab2-starter/`
- [ ] T028 Write Lab 2 instructions in `curriculum/module1-ros2/labs/lab2-instructions.md`
- [ ] T029 Create Lab 2 grading rubric in `curriculum/module1-ros2/labs/lab2-rubric.md`
- [ ] T030 Create sample solution for Lab 2 (for TAs) in `curriculum/module1-ros2/labs/lab2-solution/`

### Week 3: Robot Description and Visualization

**Lesson Materials**:
- [ ] T031 [P] Write lesson: "URDF Basics" in `curriculum/module1-ros2/lessons/week3-lesson1-urdf.md`
- [ ] T032 [P] Write lesson: "XACRO for Modular Descriptions" in `curriculum/module1-ros2/lessons/week3-lesson2-xacro.md`
- [ ] T033 [P] Create example: Simple humanoid URDF in `shared/urdf-library/simple-humanoid.urdf`

**Lab 3: Build URDF Humanoid Model (10 points)**:
- [ ] T034 Create Lab 3 specification at `curriculum/module1-ros2/labs/lab3.yaml`
- [ ] T035 Create Lab 3 starter template with basic humanoid structure at `curriculum/module1-ros2/labs/lab3-starter/urdf/humanoid.xacro`
- [ ] T036 Write Lab 3 instructions in `curriculum/module1-ros2/labs/lab3-instructions.md`
- [ ] T037 Create Lab 3 grading rubric in `curriculum/module1-ros2/labs/lab3-rubric.md`
- [ ] T038 Create sample solution: Complete humanoid URDF (10+ joints) at `curriculum/module1-ros2/labs/lab3-solution/`
- [ ] T039 Create RViz config for humanoid visualization at `curriculum/module1-ros2/labs/lab3-starter/config/humanoid.rviz`

### Module 1 Integration

- [ ] T040 Create Module 1 bringup launch file at `curriculum/module1-ros2/launch/bringup.launch.py`
- [ ] T041 Test all Module 1 labs: verify build with `colcon build --packages-select humanoid_base`
- [ ] T042 Create Module 1 assessment checklist in `curriculum/module1-ros2/assessment/checklist.md`

**Checkpoint**: Module 1 complete, all labs tested, ROS 2 package builds successfully, URDF visualizes in RViz

---

## Milestone 2 — Digital Twin (Weeks 3-6)

**Purpose**: Module 2 curriculum materials, Gazebo and Unity integration

### Module 2 Structure Setup

- [ ] T043 Create module directory structure at `curriculum/module2-digital-twin/` with subdirectories: `lessons/`, `gazebo-worlds/`, `unity-scenes/`, `labs/`
- [ ] T044 Create module specification file at `curriculum/module2-digital-twin/module.yaml`
- [ ] T045 Write module README at `curriculum/module2-digital-twin/README.md`

### Week 1: Gazebo Fundamentals

**Lesson Materials**:
- [ ] T046 [P] Write lesson: "Gazebo Architecture and Plugins" in `curriculum/module2-digital-twin/lessons/week1-lesson1-gazebo-architecture.md`
- [ ] T047 [P] Write lesson: "Creating Worlds with SDF" in `curriculum/module2-digital-twin/lessons/week1-lesson2-sdf.md`
- [ ] T048 [P] Create example: Basic Gazebo world in `curriculum/module2-digital-twin/gazebo-worlds/sample_world.world`

**Lab 4: Create Gazebo World with Physics (5 points)**:
- [ ] T049 Create Lab 4 specification at `curriculum/module2-digital-twin/labs/lab4.yaml`
- [ ] T050 Create Lab 4 starter world template at `curriculum/module2-digital-twin/labs/lab4-starter/worlds/home_world.world`
- [ ] T051 Write Lab 4 instructions in `curriculum/module2-digital-twin/labs/lab4-instructions.md`
- [ ] T052 Create Lab 4 grading rubric in `curriculum/module2-digital-twin/labs/lab4-rubric.md`
- [ ] T053 Create launch file to spawn humanoid in Gazebo at `curriculum/module2-digital-twin/labs/lab4-starter/launch/spawn_humanoid.launch.py`
- [ ] T054 Create sample solution for Lab 4 at `curriculum/module2-digital-twin/labs/lab4-solution/`

### Week 2: Sensor Simulation

**Lesson Materials**:
- [ ] T055 [P] Write lesson: "LiDAR and Depth Sensors" in `curriculum/module2-digital-twin/lessons/week2-lesson1-sensors.md`
- [ ] T056 [P] Write lesson: "Sensor Noise Models" in `curriculum/module2-digital-twin/lessons/week2-lesson2-sensor-noise.md`
- [ ] T057 [P] Create example: Sensor configuration in URDF at `curriculum/module2-digital-twin/examples/sensor_config.xacro`

**Lab 5: Unity-ROS 2 Integration (5 points)**:
- [ ] T058 Create Lab 5 specification at `curriculum/module2-digital-twin/labs/lab5.yaml`
- [ ] T059 Create Unity starter project with humanoid scene at `curriculum/module2-digital-twin/labs/lab5-starter/UnityProject/`
- [ ] T060 Configure ROS-TCP-Connector in Unity project
- [ ] T061 Write Lab 5 instructions in `curriculum/module2-digital-twin/labs/lab5-instructions.md`
- [ ] T062 Create Lab 5 grading rubric in `curriculum/module2-digital-twin/labs/lab5-rubric.md`
- [ ] T063 Create demo video script showing Unity-ROS 2 synchronization

### Week 3: Digital Twin Integration

**Lesson Materials**:
- [ ] T064 [P] Write lesson: "Real-Time Control Loops" in `curriculum/module2-digital-twin/lessons/week3-lesson1-realtime.md`
- [ ] T065 [P] Write lesson: "Multi-Robot Simulations" in `curriculum/module2-digital-twin/lessons/week3-lesson2-multi-robot.md`

**Lab 6: Simulate Humanoid with Sensor Data (10 points)**:
- [ ] T066 Create Lab 6 specification at `curriculum/module2-digital-twin/labs/lab6.yaml`
- [ ] T067 Create Gazebo world with sensors at `curriculum/module2-digital-twin/labs/lab6-starter/worlds/sensor_world.world`
- [ ] T068 Add LiDAR, depth camera, and IMU to humanoid URDF at `curriculum/module2-digital-twin/labs/lab6-starter/urdf/humanoid_sensors.xacro`
- [ ] T069 Create ROS 2 node template for sensor processing at `curriculum/module2-digital-twin/labs/lab6-starter/src/sensor_processor.py`
- [ ] T070 Write Lab 6 instructions in `curriculum/module2-digital-twin/labs/lab6-instructions.md`
- [ ] T071 Create RViz config showing all sensor visualizations at `curriculum/module2-digital-twin/labs/lab6-starter/config/sensors.rviz`
- [ ] T072 Create Lab 6 grading rubric in `curriculum/module2-digital-twin/labs/lab6-rubric.md`
- [ ] T073 Create sample solution with sensor processing logic at `curriculum/module2-digital-twin/labs/lab6-solution/`

### Module 2 Integration

- [ ] T074 Create ros2_control configuration for humanoid at `shared/ros2-utils/ros2_control_config.yaml`
- [ ] T075 Test all Module 2 labs: verify Gazebo simulation and Unity integration
- [ ] T076 Create Module 2 assessment checklist in `curriculum/module2-digital-twin/assessment/checklist.md`

**Checkpoint**: Module 2 complete, Gazebo simulation works, Unity visualizes robot, sensors publish data to ROS 2

---

## Milestone 3 — Isaac Sim and Perception (Weeks 6-9)

**Purpose**: Module 3 curriculum materials, Isaac Sim integration, synthetic data generation

### Module 3 Structure Setup

- [ ] T077 Create module directory structure at `curriculum/module3-isaac/` with subdirectories: `lessons/`, `isaac-scenes/`, `perception-demos/`, `labs/`
- [ ] T078 Create module specification file at `curriculum/module3-isaac/module.yaml`
- [ ] T079 Write module README at `curriculum/module3-isaac/README.md`

### Week 1: Isaac Sim Fundamentals

**Lesson Materials**:
- [ ] T080 [P] Write lesson: "Isaac Sim Architecture and Omniverse" in `curriculum/module3-isaac/lessons/week1-lesson1-isaac-architecture.md`
- [ ] T081 [P] Write lesson: "USD Format and Scene Description" in `curriculum/module3-isaac/lessons/week1-lesson2-usd.md`
- [ ] T082 [P] Create example: URDF to USD conversion script at `curriculum/module3-isaac/examples/urdf_to_usd.py`

**Lab 7: Isaac Sim Environment Setup (5 points)**:
- [ ] T083 Create Lab 7 specification at `curriculum/module3-isaac/labs/lab7.yaml`
- [ ] T084 Create Isaac Sim scene template at `curriculum/module3-isaac/labs/lab7-starter/scenes/home_scene.usd`
- [ ] T085 Write Lab 7 instructions in `curriculum/module3-isaac/labs/lab7-instructions.md`
- [ ] T086 Create Lab 7 grading rubric in `curriculum/module3-isaac/labs/lab7-rubric.md`
- [ ] T087 Create ROS 2 bridge script for Isaac Sim at `curriculum/module3-isaac/labs/lab7-starter/scripts/ros2_bridge.py`
- [ ] T088 Create sample solution for Lab 7 at `curriculum/module3-isaac/labs/lab7-solution/`

### Week 2: Perception with Isaac

**Lesson Materials**:
- [ ] T089 [P] Write lesson: "Pre-trained Isaac Models (DOPE, FoundationPose)" in `curriculum/module3-isaac/lessons/week2-lesson1-pretrained-models.md`
- [ ] T090 [P] Write lesson: "Domain Randomization for Robustness" in `curriculum/module3-isaac/lessons/week2-lesson2-domain-randomization.md`
- [ ] T091 [P] Create example: Domain randomization script at `curriculum/module3-isaac/examples/randomize_scene.py`

**Lab 8: Train Perception Model on Synthetic Data (5 points)**:
- [ ] T092 Create Lab 8 specification at `curriculum/module3-isaac/labs/lab8.yaml`
- [ ] T093 Create data generation script using Replicator at `curriculum/module3-isaac/labs/lab8-starter/scripts/generate_dataset.py`
- [ ] T094 Create training script template at `curriculum/module3-isaac/labs/lab8-starter/scripts/train_detector.py`
- [ ] T095 Write Lab 8 instructions in `curriculum/module3-isaac/labs/lab8-instructions.md`
- [ ] T096 Create Lab 8 grading rubric in `curriculum/module3-isaac/labs/lab8-rubric.md`
- [ ] T097 Create sample dataset (100+ images) and annotation manifest at `curriculum/module3-isaac/labs/lab8-starter/data/`
- [ ] T098 Create sample solution with trained model at `curriculum/module3-isaac/labs/lab8-solution/`

### Week 3: Sensor Fusion and Integration

**Lesson Materials**:
- [ ] T099 [P] Write lesson: "Sensor Fusion Techniques" in `curriculum/module3-isaac/lessons/week3-lesson1-sensor-fusion.md`
- [ ] T100 [P] Write lesson: "Isaac ROS GEMs" in `curriculum/module3-isaac/lessons/week3-lesson2-isaac-ros-gems.md`

**Lab 9: Implement Sensor Fusion Pipeline (10 points)**:
- [ ] T101 Create Lab 9 specification at `curriculum/module3-isaac/labs/lab9.yaml`
- [ ] T102 Create Isaac Sim scene with RGB camera + depth sensor at `curriculum/module3-isaac/labs/lab9-starter/scenes/fusion_scene.usd`
- [ ] T103 Create fusion algorithm template at `curriculum/module3-isaac/labs/lab9-starter/src/sensor_fusion.py`
- [ ] T104 Write Lab 9 instructions in `curriculum/module3-isaac/labs/lab9-instructions.md`
- [ ] T105 Create RViz config for 3D bounding boxes at `curriculum/module3-isaac/labs/lab9-starter/config/fusion.rviz`
- [ ] T106 Create performance benchmarking script at `curriculum/module3-isaac/labs/lab9-starter/scripts/benchmark.py`
- [ ] T107 Create Lab 9 grading rubric in `curriculum/module3-isaac/labs/lab9-rubric.md`
- [ ] T108 Create sample solution with fusion implementation at `curriculum/module3-isaac/labs/lab9-solution/`

### Module 3 Integration

- [ ] T109 Use Isaac ROS nodes for camera input and VSLAM pipeline integration
- [ ] T110 Test all Module 3 labs: verify Isaac Sim launches, synthetic data generates, fusion works
- [ ] T111 Create Module 3 assessment checklist in `curriculum/module3-isaac/assessment/checklist.md`

**Checkpoint**: Module 3 complete, Isaac Sim works, synthetic datasets generated, perception pipeline functions

---

## Milestone 4 — VLA and Integration (Weeks 9-11)

**Purpose**: Module 4 curriculum materials, LLM-robot integration, VLA pipeline

### Module 4 Structure Setup

- [ ] T112 Create module directory structure at `curriculum/module4-vla/` with subdirectories: `lessons/`, `vla-agents/`, `labs/`
- [ ] T113 Create module specification file at `curriculum/module4-vla/module.yaml`
- [ ] T114 Write module README at `curriculum/module4-vla/README.md`

### Week 1: Vision-Language Foundation

**Lesson Materials**:
- [ ] T115 [P] Write lesson: "Vision-Language Models (CLIP, BLIP)" in `curriculum/module4-vla/lessons/week1-lesson1-vlm.md`
- [ ] T116 [P] Write lesson: "Spatial Reasoning with Depth" in `curriculum/module4-vla/lessons/week1-lesson2-spatial-reasoning.md`
- [ ] T117 [P] Create example: CLIP zero-shot detection at `curriculum/module4-vla/examples/clip_detection.py`

**Lab 10: Vision-Language Grounding with CLIP (10 points)**:
- [ ] T118 Create Lab 10 specification at `curriculum/module4-vla/labs/lab10.yaml`
- [ ] T119 Create ROS 2 node template for CLIP integration at `curriculum/module4-vla/labs/lab10-starter/src/clip_node.py`
- [ ] T120 Write Lab 10 instructions in `curriculum/module4-vla/labs/lab10-instructions.md`
- [ ] T121 Create test queries dataset (10+ objects) at `curriculum/module4-vla/labs/lab10-starter/data/test_queries.yaml`
- [ ] T122 Create RViz config for detection visualization at `curriculum/module4-vla/labs/lab10-starter/config/detections.rviz`
- [ ] T123 Create Lab 10 grading rubric in `curriculum/module4-vla/labs/lab10-rubric.md`
- [ ] T124 Create sample solution with 3D localization at `curriculum/module4-vla/labs/lab10-solution/`

### Week 2: LLM-Robot Communication

**Lesson Materials**:
- [ ] T125 [P] Write lesson: "LLM-Based Task Planning" in `curriculum/module4-vla/lessons/week2-lesson1-llm-planning.md`
- [ ] T126 [P] Write lesson: "Prompt Engineering for Robotics" in `curriculum/module4-vla/lessons/week2-lesson2-prompting.md`
- [ ] T127 [P] Create example: LangChain agent structure at `curriculum/module4-vla/examples/langchain_agent.py`

**Lab 11: LLM-Based Task Planning Agent (10 points)**:
- [ ] T128 Create Lab 11 specification at `curriculum/module4-vla/labs/lab11.yaml`
- [ ] T129 Create LangChain agent template at `curriculum/module4-vla/labs/lab11-starter/src/task_planner.py`
- [ ] T130 Create robot control tools library at `curriculum/module4-vla/labs/lab11-starter/src/robot_tools.py`
- [ ] T131 Write Lab 11 instructions in `curriculum/module4-vla/labs/lab11-instructions.md`
- [ ] T132 Create test commands dataset (5+ variations) at `curriculum/module4-vla/labs/lab11-starter/data/test_commands.yaml`
- [ ] T133 Create prompt templates at `curriculum/module4-vla/labs/lab11-starter/prompts/`
- [ ] T134 Create Lab 11 grading rubric in `curriculum/module4-vla/labs/lab11-rubric.md`
- [ ] T135 Create sample solution with action generation at `curriculum/module4-vla/labs/lab11-solution/`

### Week 3: End-to-End VLA Integration

**Lesson Materials**:
- [ ] T136 [P] Write lesson: "Closed-Loop VLA Pipeline" in `curriculum/module4-vla/lessons/week3-lesson1-vla-pipeline.md`
- [ ] T137 [P] Write lesson: "Error Handling and Recovery" in `curriculum/module4-vla/lessons/week3-lesson2-error-handling.md`
- [ ] T138 [P] Create example: Full VLA integration at `curriculum/module4-vla/examples/vla_integration.py`

### Module 4 Integration

- [ ] T139 Create integration example combining Labs 10 & 11 at `curriculum/module4-vla/examples/household_assistant.py`
- [ ] T140 Test all Module 4 labs: verify CLIP detection, LLM planning, action generation
- [ ] T141 Create Module 4 assessment checklist in `curriculum/module4-vla/assessment/checklist.md`

**Checkpoint**: Module 4 complete, VLA pipeline functional, natural language commands execute in simulation

---

## Capstone Demo (Week 12)

**Purpose**: Capstone project infrastructure, templates, and evaluation rubrics

### Capstone Structure

- [ ] T142 Create capstone directory at `capstone/` with subdirectories: `requirements/`, `starter-code/`, `evaluation/`
- [ ] T143 Write capstone requirements document at `capstone/requirements.md`
- [ ] T144 Create evaluation rubric at `capstone/evaluation/rubric.md`

### Capstone Starter Template

- [ ] T145 Create starter ROS 2 workspace at `capstone/starter-code/ros2_ws/`
- [ ] T146 Create base humanoid package at `capstone/starter-code/ros2_ws/src/humanoid_base/`
- [ ] T147 Create perception package template at `capstone/starter-code/ros2_ws/src/perception_pipeline/`
- [ ] T148 Create planning package template at `capstone/starter-code/ros2_ws/src/task_planner/`
- [ ] T149 Create navigation package template at `capstone/starter-code/ros2_ws/src/navigation/`
- [ ] T150 Create manipulation package template at `capstone/starter-code/ros2_ws/src/manipulation/`
- [ ] T151 Create launch files for full system at `capstone/starter-code/ros2_ws/src/humanoid_base/launch/`

### Capstone Environment

- [ ] T152 Create Gazebo world for capstone (furnished home) at `capstone/starter-code/worlds/capstone_home.world`
- [ ] T153 Create Isaac Sim scene for capstone (photorealistic home) at `capstone/starter-code/scenes/capstone_home.usd`
- [ ] T154 Create object library (10+ household items) at `capstone/starter-code/models/`

### Capstone Tasks Definition

- [ ] T155 Define Task 1: Object Retrieval scenario in `capstone/requirements/task1-object-retrieval.md`
- [ ] T156 Define Task 2: Delivery scenario in `capstone/requirements/task2-delivery.md`
- [ ] T157 Define Task 3: Placement scenario in `capstone/requirements/task3-placement.md`
- [ ] T158 Define Task 4: Search scenario in `capstone/requirements/task4-search.md`
- [ ] T159 Define Task 5: Multi-step scenario in `capstone/requirements/task5-multi-step.md`

### Capstone Evaluation

- [ ] T160 Create acceptance test script for voice command → action execution at `capstone/evaluation/acceptance_test.sh`
- [ ] T161 Create grading checklist for TAs at `capstone/evaluation/grading-checklist.md`
- [ ] T162 Create demo video requirements at `capstone/evaluation/demo-video-guide.md`
- [ ] T163 Create technical report template at `capstone/evaluation/report-template.md`

### Capstone Integration Testing

- [ ] T164 Test capstone starter code: verify all packages build with `colcon build`
- [ ] T165 Test autonomous run: voice command triggers full pipeline end-to-end
- [ ] T166 Validate deliverables: recorded demo, live simulation, final report structure

**Checkpoint**: Capstone infrastructure complete, starter template works, evaluation rubrics defined

---

## Final Phase: Documentation and Validation

**Purpose**: Cross-cutting documentation, validation, and deployment preparation

### Documentation

- [ ] T167 [P] Create instructor guide at `docs/instructors/teaching-guide.md`
- [ ] T168 [P] Create TA guide at `docs/instructors/ta-guide.md`
- [ ] T169 [P] Write grading guidelines at `docs/instructors/grading-guidelines.md`
- [ ] T170 [P] Create course syllabus template at `docs/syllabus.md`
- [ ] T171 [P] Write FAQ document at `docs/faq.md`
- [ ] T172 [P] Create troubleshooting guide at `docs/troubleshooting.md`

### Validation and Testing

- [ ] T173 Run quickstart.md validation: verify all setup steps work
- [ ] T174 Test all Docker images: base ROS 2, Isaac Sim, full stack
- [ ] T175 Verify all lab starter code builds and runs
- [ ] T176 Test all module examples execute without errors
- [ ] T177 Validate all launch files work in Gazebo and Isaac Sim
- [ ] T178 Run end-to-end curriculum flow: Module 1 → 2 → 3 → 4 → Capstone

### Repository Finalization

- [ ] T179 Create repository README at `README.md` with overview and quick links
- [ ] T180 Create LICENSE file (choose appropriate license for education)
- [ ] T181 Create CONTRIBUTING guide at `CONTRIBUTING.md`
- [ ] T182 Set up GitHub issues templates for bug reports and feature requests
- [ ] T183 Create CI/CD pipeline for automated testing (optional)
- [ ] T184 Tag version 1.0.0 release

**Checkpoint**: Curriculum ready for deployment, all documentation complete, validation passed

---

## Dependencies & Execution Order

### Phase Dependencies

- **Milestone 0 (Setup)**: No dependencies - start immediately
- **Milestone 1 (Module 1)**: Depends on Setup completion
- **Milestone 2 (Module 2)**: Depends on Module 1 completion (uses URDF from Module 1)
- **Milestone 3 (Module 3)**: Depends on Module 2 completion (uses simulation environment)
- **Milestone 4 (Module 4)**: Depends on Modules 1-3 completion (integrates all prior work)
- **Capstone**: Depends on all 4 modules completion
- **Final Phase**: Depends on all milestones completion

### Parallel Opportunities

- **Within Setup**: All Docker/installation tasks marked [P] can run in parallel
- **Within Each Module**: Lesson materials marked [P] can be written in parallel
- **Across Weeks**: Labs for different weeks within a module can be developed in parallel once module structure is set
- **Documentation**: All Final Phase documentation tasks marked [P] can run in parallel

### Sequential Dependencies Within Milestones

- Module structure → Lesson materials → Lab specifications → Lab starter code → Lab solutions
- Lab N must have grading rubric before Lab N+1 begins
- Module N must be complete before Module N+1 starts (curriculum flow dependency)

---

## Implementation Strategy

### MVP First (Minimum Viable Curriculum)

1. Complete Milestone 0: Setup
2. Complete Milestone 1: Module 1 only (ROS 2 Foundation)
3. **STOP and VALIDATE**: Test Module 1 with pilot cohort
4. Deploy Module 1, gather feedback
5. Iterate based on student feedback before building Module 2

### Incremental Delivery

1. Setup → Module 1 → Test with students → Deploy
2. Add Module 2 → Test → Deploy (now have Modules 1-2)
3. Add Module 3 → Test → Deploy (now have Modules 1-3)
4. Add Module 4 → Test → Deploy (full curriculum)
5. Add Capstone → Test → Deploy (complete program)

### Parallel Team Strategy

With multiple curriculum developers:

1. Team completes Setup together
2. Once Setup done:
   - Developer A: Module 1 (Weeks 1-3)
   - Developer B: Module 2 (Weeks 4-6)
   - Developer C: Module 3 (Weeks 7-9)
   - Developer D: Module 4 (Weeks 10-11) + Capstone
3. Each developer creates lesson materials, labs, and solutions for their module
4. Team reviews and integrates modules sequentially

---

## Key Integration Points

### Cross-Module Dependencies

- **Module 1 → Module 2**: Humanoid URDF from Lab 3 is used in Module 2 Gazebo simulations
- **Module 2 → Module 3**: Gazebo sensor configurations inform Isaac Sim sensor setup
- **Module 3 → Module 4**: Perception models trained in Module 3 are used for VLA in Module 4
- **All Modules → Capstone**: Capstone integrates all module learnings into one system

### Shared Resources

- `shared/urdf-library/`: Humanoid models reused across modules
- `shared/ros2-utils/`: Common ROS 2 utilities (ros2_control configs, launch templates)
- `shared/docker/`: Docker images used by all students for consistent environments

### Testing Strategy

- Each lab has acceptance criteria in its rubric
- Each module has integration tests to verify all labs work together
- Capstone has end-to-end acceptance test for autonomous task execution
- Final validation runs entire curriculum from setup to capstone

---

## Notes

- **[P] tasks**: Different files, no dependencies on other tasks
- **File paths**: All paths relative to repository root
- **Lab structure**: Each lab has specification (YAML), instructions (MD), starter code, grading rubric, and sample solution
- **Commit strategy**: Commit after completing each milestone or logical task group
- **Validation**: Stop at checkpoints to validate milestone completion before proceeding
- **Iteration**: Gather feedback after each module deployment to improve subsequent modules

---

## Summary

**Total Tasks**: 184
**Milestones**: 6 (Setup, Module 1-4, Capstone, Final)
**Parallel Opportunities**: ~60 tasks marked [P] can run in parallel within their phases
**Suggested MVP**: Milestone 0 + Milestone 1 (Setup + Module 1 ROS 2 Foundation)
**Incremental Delivery**: Add one module at a time, validate with students between modules

**Critical Path**: Setup → Module 1 → Module 2 → Module 3 → Module 4 → Capstone → Documentation
