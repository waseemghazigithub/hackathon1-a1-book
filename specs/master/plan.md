# Implementation Plan: Physical AI & Humanoid Robotics Curriculum

**Branch**: `master` | **Date**: 2025-12-06 | **Spec**: N/A (Planning from curriculum description)
**Input**: Curriculum overview for Physical AI capstone quarter

## Summary

This capstone quarter introduces students to Physical AI—AI systems that operate in the physical world and understand physical laws. Students will learn to design, simulate, and deploy humanoid robots with natural interaction capabilities through a four-module curriculum covering ROS 2 (robotic middleware), digital twin simulation (Gazebo & Unity), NVIDIA Isaac (advanced perception), and Vision-Language-Action robotics.

The curriculum bridges digital AI knowledge with physical embodied systems, progressing from foundational robot control to advanced AI-robot integration.

## Technical Context

**Language/Version**: Python 3.10+ (ROS 2 Humble/Iron), C++ 17 (ROS 2 nodes), C# (Unity integration)
**Primary Dependencies**:
- ROS 2 Humble/Iron (rclpy, rclcpp, std_msgs, sensor_msgs, control_msgs)
- Gazebo Classic 11 or Gazebo Sim (Fortress/Garden)
- Unity 2021.3 LTS+ with ROS-TCP-Connector
- NVIDIA Isaac Sim 2023.1+ or Isaac ROS
- Additional: urdf, xacro, robot_state_publisher, joint_state_publisher

**Storage**:
- Robot configurations: URDF/XACRO files
- Simulation worlds: SDF (Gazebo), Unity scenes
- Training data: Isaac Sim synthetic datasets (USD format)
- Student projects: Git repositories

**Testing**:
- Unit: pytest (Python nodes), gtest (C++ nodes)
- Integration: ROS 2 launch tests, Gazebo integration tests
- Simulation validation: Isaac Sim test scenarios

**Target Platform**:
- Development: Ubuntu 22.04 (ROS 2 Humble) or Ubuntu 24.04 (ROS 2 Iron)
- Simulation: NVIDIA GPU-enabled workstations (RTX 3060+ recommended for Isaac Sim)
- Optional: Physical humanoid platforms (e.g., unitree H1, simulated equivalents)

**Project Type**: Educational curriculum with simulation-based robotics projects

**Performance Goals**:
- Real-time robot control: 100Hz+ joint control loop
- Simulation: 30+ FPS in Gazebo, 60+ FPS in Unity visualization
- Isaac Sim: Real-time ray tracing at 1920x1080
- Sensor processing: 10Hz+ for LiDAR/depth, 30Hz+ for cameras

**Constraints**:
- Hardware: Students need GPU-capable machines (NVIDIA preferred)
- Time: Quarter-based curriculum (10-12 weeks)
- Prerequisites: Python programming, basic linear algebra, AI/ML fundamentals
- Licensing: NVIDIA Isaac requires academic license

**Scale/Scope**:
- 4 modules with progressive complexity
- ~20-40 students per cohort
- Capstone project: Autonomous humanoid with VLA pipeline
- ~30-40 hours of instruction + 60-80 hours student project work

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### ✅ Technical Accuracy
- All robotics frameworks (ROS 2, Gazebo, Unity, Isaac) are industry-standard
- URDF, kinematics, controllers, sensors are standard robotics terminology
- ROS 2 Humble/Iron are current LTS releases
- Gazebo and Isaac Sim are established simulation platforms

### ✅ Real-world Alignment
- Curriculum follows realistic robotics workflow: middleware → simulation → perception → high-level AI
- ROS 2 is the industry standard for modern robotics
- Digital twin approach (Gazebo/Unity) matches commercial practice
- NVIDIA Isaac aligns with current AI-robotics research

### ✅ Clarity for Practitioners
- Progressive structure: basics (ROS 2) → simulation → advanced AI
- Hands-on: Each module includes practical implementation
- Clear technology stack with specific versions
- Capstone provides end-to-end integration experience

### ✅ Embodied Intelligence Perspective
- Explicit focus on bridging digital AI with physical systems
- VLA (Vision-Language-Action) integration in curriculum
- Emphasis on physical laws and real-world constraints
- Humanoid robotics as test bed for embodied intelligence

### Content Structure Alignment
- ✅ Module 1: ROS 2 (robotic nervous system) - PRESENT
- ✅ Module 2: Digital Twin simulation (Gazebo & Unity) - PRESENT
- ✅ Module 3: NVIDIA Isaac & advanced AI-robot perception - PRESENT
- ⚠️ Module 4: Vision-Language-Action robotics - NEEDS CLARIFICATION (mentioned but incomplete in input)
- ⚠️ Capstone description - NEEDS CLARIFICATION (needs full definition)

**Gate Status**: CONDITIONAL PASS - Proceed to Phase 0 research to clarify Module 4 and Capstone details

## Project Structure

### Documentation (this feature)

```text
specs/master/
├── plan.md              # This file
├── research.md          # Phase 0: Technology research and clarifications
├── data-model.md        # Phase 1: Curriculum entities and relationships
├── quickstart.md        # Phase 1: Setup guide for students/instructors
├── contracts/           # Phase 1: Module interfaces and learning objectives
│   ├── module1-ros2.md
│   ├── module2-digital-twin.md
│   ├── module3-isaac.md
│   └── module4-vla.md
└── tasks.md             # Phase 2: Implementation tasks (via /sp.tasks)
```

### Source Code (repository root)

```text
# Curriculum repository structure
curriculum/
├── module1-ros2/
│   ├── lessons/          # Lesson materials
│   ├── examples/         # ROS 2 example nodes
│   ├── labs/             # Hands-on lab assignments
│   └── resources/        # URDF files, launch files
│
├── module2-digital-twin/
│   ├── lessons/
│   ├── gazebo-worlds/    # Gazebo simulation environments
│   ├── unity-scenes/     # Unity humanoid scenes
│   └── labs/
│
├── module3-isaac/
│   ├── lessons/
│   ├── isaac-scenes/     # Isaac Sim USD files
│   ├── perception-demos/ # Sensor processing examples
│   └── labs/
│
├── module4-vla/
│   ├── lessons/
│   ├── vla-agents/       # LLM-robot integration examples
│   └── labs/
│
├── capstone/
│   ├── requirements.md   # Capstone project specifications
│   ├── starter-code/     # Base humanoid project template
│   └── evaluation/       # Grading rubrics
│
├── shared/
│   ├── urdf-library/     # Reusable robot descriptions
│   ├── ros2-utils/       # Common ROS 2 utilities
│   └── docker/           # Development environment containers
│
└── tests/
    ├── integration/      # Cross-module integration tests
    └── simulation/       # Simulation validation tests
```

**Structure Decision**: Educational curriculum repository with modular structure. Each module is self-contained with lessons, examples, and labs. Shared resources (URDF, utilities, Docker) centralized for reuse. Capstone project provided as separate directory with starter template.

## Complexity Tracking

> No constitutional violations detected. Curriculum aligns with all core principles and standards.

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| N/A | N/A | N/A |

## Phase 0: Research & Clarifications

**Unknowns to Resolve**:

1. **Module 4 Content** (NEEDS CLARIFICATION)
   - Full curriculum for Vision-Language-Action robotics
   - LLM integration patterns for embodied agents
   - Specific VLA frameworks (e.g., RT-1, RT-2, PaLM-E)

2. **Capstone Project Definition** (NEEDS CLARIFICATION)
   - Complete project requirements
   - Autonomous humanoid capabilities expected
   - VLA pipeline components and integration points

3. **Student Prerequisites** (NEEDS CLARIFICATION)
   - Required prior AI/ML knowledge depth
   - Expected programming skill level
   - Hardware access requirements

4. **Assessment Strategy** (NEEDS CLARIFICATION)
   - Module-level evaluation criteria
   - Capstone grading rubric
   - Hands-on vs. theoretical balance

**Research Tasks**:
- Best practices for ROS 2 education (curriculum structure, progression)
- Gazebo vs. Isaac Sim trade-offs for humanoid simulation
- Unity integration patterns with ROS 2
- Current VLA research and teachable frameworks
- GPU hardware recommendations for student cohorts
- Docker/container strategies for consistent student environments

## Phase 1: Design & Contracts

*Deferred until Phase 0 research complete*

**Planned Deliverables**:
- `data-model.md`: Curriculum entities (modules, lessons, labs, assessments, student projects)
- `contracts/module*.md`: Learning objectives, prerequisites, deliverables for each module
- `quickstart.md`: Environment setup guide (Ubuntu, ROS 2, Gazebo, Unity, Isaac)

## Phase 2: Tasks

*Created via `/sp.tasks` command after Phase 1 complete*

## Notes

**Key Decisions Pending**:
1. Module 4 and Capstone details must be clarified before curriculum implementation
2. Hardware requirements impact student accessibility—need Docker/cloud simulation fallback strategy
3. NVIDIA Isaac licensing for academic use must be confirmed
4. Unity vs. pure Gazebo for visualization—trade-offs in fidelity vs. complexity

**Next Steps**:
1. Run Phase 0 research to resolve NEEDS CLARIFICATION items
2. Complete Module 4 and Capstone specifications
3. Generate Phase 1 design artifacts (data model, contracts, quickstart)
4. Validate constitution alignment post-design
