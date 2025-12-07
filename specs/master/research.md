# Phase 0 Research: Physical AI & Humanoid Robotics Curriculum

**Date**: 2025-12-06
**Branch**: master
**Related**: [plan.md](./plan.md)

## Purpose

This document resolves technical unknowns and establishes best practices for the Physical AI curriculum implementation. All NEEDS CLARIFICATION items from the planning phase are addressed here with research-backed decisions.

---

## 1. Module 4: Vision-Language-Action (VLA) Robotics

### Decision: Multimodal LLM-Robot Integration Framework

**Rationale**:
Vision-Language-Action models represent the cutting edge of embodied AI, combining visual perception, natural language understanding, and physical action execution. For educational purposes, we need frameworks that:
1. Are accessible to students (good documentation, active community)
2. Integrate with ROS 2 (our established robot middleware)
3. Demonstrate real VLA concepts without requiring massive compute

**Chosen Approach**: Hybrid VLA Architecture

**Components**:
1. **Perception**: Pre-trained vision models (CLIP, SAM for segmentation)
2. **Language**: Open-source LLMs (Llama 3, Mistral) via LangChain/LlamaIndex
3. **Action**: ROS 2 action servers connecting LLM outputs to robot controllers
4. **Integration**: Custom VLA pipeline using ROS 2 + Python

**Module 4 Curriculum Structure**:

#### Week 1: Vision-Language Foundation
- CLIP for vision-language alignment
- Object detection and scene understanding
- Spatial reasoning with depth data

#### Week 2: LLM-Robot Communication
- LangChain for robot task planning
- Natural language to ROS 2 action translation
- Chain-of-thought prompting for robot tasks

#### Week 3: Action Grounding
- Mapping language commands to robot primitives
- Inverse kinematics for humanoid manipulation
- Safety constraints and action validation

#### Week 4: End-to-End VLA Pipeline
- Integrated perception → reasoning → action loop
- Multi-step task execution (e.g., "pick up the red cup")
- Real-time robot control with LLM feedback

**Alternatives Considered**:

| Framework | Pros | Cons | Rejected Because |
|-----------|------|------|------------------|
| RT-1/RT-2 (Google) | State-of-art VLA | Requires massive datasets, limited access | Too research-focused, not open-source |
| PaLM-E | Multimodal embodied AI | Proprietary, compute-intensive | Not accessible for students |
| Open-VLA | Open-source RT-1 reimplementation | Still maturing, fewer examples | Documentation insufficient for teaching |
| Custom pipeline | Full control, ROS 2 native | Requires building from scratch | **SELECTED** - Best for learning fundamentals |

**Key Technologies**:
- **LangChain/LlamaIndex**: LLM orchestration
- **Transformers (HuggingFace)**: Model inference
- **CLIP/BLIP**: Vision-language models
- **ROS 2 action servers**: Robot control interface
- **MoveIt 2**: Motion planning for manipulation

**Learning Objectives**:
- Understand VLA architecture (perception, reasoning, action)
- Implement vision-language grounding
- Connect LLMs to robot control systems
- Debug multimodal AI-robot pipelines

---

## 2. Capstone Project: Autonomous Humanoid with VLA

### Decision: "Household Assistant Robot" Scenario

**Project Overview**:
Students build an autonomous humanoid robot (simulated) that can understand natural language commands and execute household tasks using the full VLA pipeline.

**Required Capabilities**:

1. **Perception**:
   - Identify objects using CLIP (10+ common household items)
   - Spatial awareness via depth camera (obstacle detection)
   - Human pose estimation (basic interaction)

2. **Language Understanding**:
   - Parse natural language commands (e.g., "bring me the blue mug")
   - Generate task plans using LLM
   - Provide status updates in natural language

3. **Action Execution**:
   - Navigate to target location (Nav2 stack)
   - Manipulate objects (inverse kinematics, grasping)
   - Multi-step task sequencing

4. **Integration**:
   - ROS 2 nodes for perception, planning, control
   - LLM agent orchestrating robot behaviors
   - Gazebo or Isaac Sim simulation environment

**Example Tasks** (students must implement 3 of 5):
1. **Object Retrieval**: "Get the red book from the table"
2. **Delivery**: "Take this to the kitchen"
3. **Placement**: "Put the cup on the shelf"
4. **Search**: "Find the remote control"
5. **Multi-step**: "Clear the table and stack items on the counter"

**Technical Requirements**:
- ROS 2 workspace with 5+ custom packages
- URDF humanoid model (pre-built or modified)
- Gazebo/Isaac world with furnished environment
- Vision pipeline (camera → CLIP → object DB)
- LLM integration (via API or local inference)
- Navigation stack (Nav2 + costmap)
- Manipulation controller (MoveIt 2 or custom IK)

**Evaluation Rubric** (100 points):
- **Perception** (20 pts): Object detection accuracy, scene understanding
- **Language** (20 pts): Command parsing, task planning quality
- **Action** (30 pts): Navigation success rate, manipulation precision
- **Integration** (20 pts): System robustness, error handling
- **Documentation** (10 pts): Code quality, README, demo video

**Timeline**:
- Week 1-2: Project setup, environment configuration
- Week 3-4: Perception pipeline implementation
- Week 5-6: LLM integration and task planning
- Week 7-8: Navigation and manipulation
- Week 9-10: Integration, testing, documentation

**Deliverables**:
- Git repository with full ROS 2 workspace
- README with setup instructions
- 5-minute demo video showing 3 tasks
- Technical report (10-15 pages) documenting architecture

---

## 3. Student Prerequisites

### Decision: AI Engineering Bootcamp Graduates

**Required Background**:

1. **Programming**:
   - Python 3 proficiency (OOP, async, libraries)
   - Basic C++ (reading code, compiling)
   - Linux command line comfort
   - Git version control

2. **AI/ML Fundamentals**:
   - Neural networks (CNN, transformers)
   - Pre-trained models (HuggingFace usage)
   - LLM prompting and fine-tuning basics
   - Computer vision fundamentals (image processing, object detection)

3. **Math**:
   - Linear algebra (vectors, matrices, transformations)
   - Basic calculus (gradients, optimization)
   - Probability (optional, helpful for uncertainty)

4. **Robotics** (Nice to Have, but NOT Required):
   - This is a capstone introducing robotics
   - No prior ROS or robot control experience assumed
   - Physics concepts will be taught from first principles

**Pre-Quarter Checklist** (sent to students 2 weeks before):
- Install Ubuntu 22.04 (dual-boot, VM, or WSL2)
- Verify GPU drivers (NVIDIA preferred for Isaac)
- Complete ROS 2 "Getting Started" tutorial (2-3 hours)
- Read "A Gentle Introduction to ROS" chapters 1-3

**Hardware Requirements**:

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| OS | Ubuntu 22.04 | Ubuntu 22.04 or 24.04 |
| CPU | 4-core Intel/AMD | 8-core Ryzen/Intel |
| RAM | 16 GB | 32 GB |
| GPU | Integrated | NVIDIA RTX 3060+ (6GB VRAM) |
| Storage | 100 GB free | 250 GB SSD |

**Software Stack** (provided via Docker):
- ROS 2 Humble or Iron
- Gazebo Classic 11 or Gazebo Sim
- Python 3.10+ with scientific stack
- Optional: NVIDIA Isaac Sim (if GPU available)

**Accessibility Plan**:
Students without powerful hardware can use:
- Cloud instances (AWS EC2 g4dn.xlarge, ~$0.50/hour)
- University lab workstations (scheduled access)
- Lightweight Gazebo simulations (no Isaac Sim)

---

## 4. Assessment Strategy

### Decision: Project-Based Learning with Continuous Assessment

**Philosophy**:
Physical AI is hands-on—students learn by building. Assessment emphasizes working code, simulation results, and technical understanding over exams.

**Module-Level Evaluation** (each module 20 points):

#### Module 1: ROS 2 Fundamentals
- **Lab 1** (5 pts): Create custom ROS 2 node (publisher/subscriber)
- **Lab 2** (5 pts): Implement service client/server
- **Lab 3** (10 pts): Build URDF humanoid model and visualize in RViz

#### Module 2: Digital Twin Simulation
- **Lab 4** (5 pts): Create Gazebo world with physics
- **Lab 5** (5 pts): Integrate Unity visualization with ROS 2
- **Lab 6** (10 pts): Simulate humanoid with sensor data (LiDAR, cameras)

#### Module 3: NVIDIA Isaac
- **Lab 7** (5 pts): Set up Isaac Sim environment
- **Lab 8** (5 pts): Train perception model on synthetic data
- **Lab 9** (10 pts): Implement sensor fusion pipeline

#### Module 4: VLA Robotics
- **Lab 10** (10 pts): Vision-language grounding with CLIP
- **Lab 11** (10 pts): LLM-based task planning agent

**Capstone Project**: 20 points (see Section 2 rubric)

**Total**: 100 points (80 from modules, 20 from capstone)

**Grading Breakdown**:
- Labs (module deliverables): 60%
- Capstone project: 20%
- Code quality & documentation: 10%
- Peer review & collaboration: 10%

**Hands-On vs. Theoretical Balance**: 80/20
- 80% practical implementation (labs, capstone coding)
- 20% conceptual understanding (weekly quizzes, technical reports)

**Weekly Quizzes** (optional, for understanding check):
- 5-10 multiple choice questions
- Cover theory behind the week's practical work
- Open-book, untimed (formative assessment)

**Peer Review** (Week 8-9):
- Students review 2 peers' capstone projects
- Provide constructive feedback on code and architecture
- Graded on quality of feedback, not project criticism

---

## 5. Technology Deep-Dives

### 5.1 ROS 2 Education Best Practices

**Research Findings**:
- **Progressive complexity**: Start with nodes/topics, then services, then actions
- **Visualization first**: Use RViz early to make concepts tangible
- **Real hardware optional**: Simulation-only curriculum is effective
- **Launch files**: Teach early for reproducible setups
- **Debugging tools**: ros2 topic echo, rqt_graph, rviz are essential

**Curriculum Integration**:
- Module 1 Week 1: Nodes and topics with visual feedback (RViz)
- Module 1 Week 2: Services for request-response patterns
- Module 1 Week 3: Actions for long-running tasks (humanoid walking)
- Module 1 Week 4: Launch files and parameters

**Key Resources**:
- Official ROS 2 docs: https://docs.ros.org/en/humble/
- The Construct: ROS 2 learning platform (simulated robots)
- Articulated Robotics YouTube channel (URDF tutorials)

### 5.2 Gazebo vs. Isaac Sim Trade-offs

**Gazebo** (Classic 11 / Gazebo Sim):
- **Pros**: Open-source, ROS 2 native, lightweight, good physics
- **Cons**: Graphics quality lower, limited photorealism, sensor noise modeling basic
- **Use Case**: Module 2 core curriculum (all students can run)

**NVIDIA Isaac Sim**:
- **Pros**: Photorealistic rendering, advanced sensor simulation, synthetic data generation, GPU-accelerated
- **Cons**: Requires NVIDIA GPU, larger installation, proprietary (free for education)
- **Use Case**: Module 3 advanced perception (optional GPU path)

**Decision**: Dual-track approach
- **Required**: Gazebo for all students (Modules 1-2)
- **Advanced**: Isaac Sim for students with GPUs (Module 3 enrichment)
- **Capstone**: Student choice (Gazebo default, Isaac optional)

**Integration Strategy**:
- Both use ROS 2, so switching is possible mid-project
- Provide Docker images for both environments
- Isaac content is "enrichment" not required for passing

### 5.3 Unity Integration with ROS 2

**Why Unity**:
- High-fidelity humanoid rendering (for presentations/demos)
- Asset store for environments (furnished rooms, outdoor scenes)
- VR/AR potential (future extension)

**Integration Pattern**:
- **Unity as Visualization Layer Only**: Gazebo handles physics, Unity renders
- **ROS-TCP-Connector**: Unity package for ROS 2 communication
- **Architecture**: ROS 2 → TCP bridge → Unity (subscribes to /tf, /joint_states)

**Module 2 Unity Lab**:
- Students import pre-built humanoid Unity scene
- Connect to Gazebo simulation via TCP bridge
- Visualize robot motion in Unity (no Unity coding required)

**Complexity Justification**:
Unity adds visual polish without changing robot control logic. It's optional for students who want portfolio-quality demos.

### 5.4 Current VLA Research & Teachable Frameworks

**State-of-Art (2023-2024)**:
- **RT-1** (Robotics Transformer): Google's VLA model (55+ tasks)
- **RT-2** (RT-1 + PaLM-E): Vision-language-action with LLM reasoning
- **Open-VLA**: Community effort to replicate RT-1 with open data
- **VIMA**: Multimodal prompting for manipulation
- **Code-as-Policies**: LLM generates Python code for robot control

**Teachable Approach for Module 4**:
Since cutting-edge VLA models are proprietary/complex, we use "VLA principles":
1. **Perception**: Pre-trained vision models (CLIP, DINO, SAM)
2. **Language**: Open LLMs (Llama, Mistral) for task planning
3. **Action**: ROS 2 action servers (student-written controllers)
4. **Loop**: Feedback from robot state → LLM → next action

**Educational Scaffolding**:
- Week 1: Vision models (CLIP for object recognition)
- Week 2: LLM prompting (task decomposition)
- Week 3: Action grounding (LLM output → robot commands)
- Week 4: Closed-loop VLA (perception → LLM → action → repeat)

**Key Insight**: Students learn VLA architecture without needing massive compute or proprietary models. They understand the principles and can follow research developments.

### 5.5 Docker Strategy for Consistent Environments

**Problem**: ROS 2 + Gazebo + Isaac + Unity = complex dependency hell

**Solution**: Multi-stage Docker images

**Image 1: Base ROS 2** (all students)
- Ubuntu 22.04 + ROS 2 Humble
- Python 3.10 + scientific libraries (numpy, opencv, torch)
- Gazebo Classic 11
- VNC server for GUI (if using cloud instances)

**Image 2: Isaac Sim** (GPU students)
- Extends Image 1
- NVIDIA Isaac Sim 2023.1
- CUDA 11.8, cuDNN
- Requires `--gpus all` Docker flag

**Image 3: Full Stack** (capstone)
- Extends Image 1 or 2
- LangChain, Transformers, CLIP models
- Nav2, MoveIt 2 packages
- Pre-downloaded LLM weights (Llama 3 8B)

**Docker Compose** for multi-container setups:
- ROS 2 master container
- Gazebo simulation container
- LLM inference container (separate for resource management)
- Unity bridge container (if using Unity)

**Student Workflow**:
1. `docker compose up` starts full environment
2. Attach VSCode to ROS 2 container (devcontainer)
3. Edit code in IDE, run in container
4. Commit to Git, container config in `docker-compose.yml`

**Benefits**:
- Reproducible environments (no "works on my machine")
- Cloud deployment ready (AWS, Azure)
- Easy for TAs to grade (same container)

---

## 6. Hardware Recommendations by Budget

### Budget Tier 1: $0 (No Purchase)

**Strategy**: Cloud + Lab Access

- **Development**: AWS EC2 g4dn.xlarge (~$50/month for 100 hours)
- **Simulation**: Gazebo only (no Isaac Sim)
- **LLM Inference**: Use APIs (OpenAI, Anthropic) or small quantized models
- **Lab Time**: University GPU workstations (scheduled 10 hours/week)

**Limitations**: No Isaac Sim (skip Module 3 advanced content)

### Budget Tier 2: $800-1200 (Student Budget)

**Recommended Build**:
- **CPU**: AMD Ryzen 5 5600 or Intel i5-12400
- **RAM**: 32 GB DDR4
- **GPU**: NVIDIA RTX 3060 12GB (used market)
- **Storage**: 500 GB NVMe SSD
- **OS**: Ubuntu 22.04 (dual-boot or dedicated)

**Capabilities**: Run everything (Gazebo, Isaac Sim, local LLMs)

### Budget Tier 3: $2000+ (Full Experience)

**Recommended Build**:
- **CPU**: AMD Ryzen 7 7700X or Intel i7-13700K
- **RAM**: 64 GB DDR5
- **GPU**: NVIDIA RTX 4070 or RTX 4080 (16GB VRAM)
- **Storage**: 1 TB NVMe SSD + 2 TB HDD
- **OS**: Ubuntu 24.04

**Capabilities**: Isaac Sim at high fidelity, multiple simulations, local LLM fine-tuning

### Institutional Purchase (Labs)

**Workstation Spec** (10-20 machines for lab):
- NVIDIA RTX 4090 or A5000
- 128 GB RAM
- 2 TB NVMe SSD
- Ubuntu 22.04 or 24.04
- Remote access via VNC/NoMachine

**Budget**: $3,000-5,000 per workstation

---

## 7. Key Decisions Summary

| Topic | Decision | Rationale |
|-------|----------|-----------|
| **Module 4 Framework** | Custom VLA pipeline (CLIP + LangChain + ROS 2) | Balance between educational clarity and real VLA concepts |
| **Capstone Project** | Household assistant robot (3 of 5 tasks) | Realistic, measurable, demonstrates full VLA pipeline |
| **Prerequisites** | AI bootcamp grads, no prior robotics | Accessible to students with software background |
| **Assessment** | 80% labs/capstone, 20% quizzes/reports | Hands-on focus aligns with physical AI nature |
| **Simulation** | Gazebo required, Isaac Sim optional | Ensures all students can participate regardless of hardware |
| **Unity Integration** | Optional visualization layer only | Adds polish without increasing core complexity |
| **VLA Approach** | Principles-based (not proprietary models) | Teachable, understandable, and replicable |
| **Environment** | Docker multi-stage images | Reproducibility and cloud/lab deployment |
| **Hardware** | 3-tier strategy (cloud/budget/full) | Accommodates diverse student resources |

---

## 8. Risks and Mitigations

| Risk | Impact | Likelihood | Mitigation |
|------|--------|------------|------------|
| Students lack GPU hardware | Can't run Isaac Sim | Medium | Cloud instances ($50/month) + lab access |
| NVIDIA Isaac licensing issues | Can't teach Module 3 | Low | Academic license available, fallback to Gazebo |
| LLM API costs for capstone | Students exceed budget | Medium | Provide free tier credits + local quantized models |
| ROS 2 learning curve too steep | Students fall behind | Medium | Pre-quarter tutorial, TA office hours, Docker simplifies setup |
| VLA research moves too fast | Curriculum outdated | High | Principles-based teaching (not framework-specific) |
| Capstone projects too ambitious | Students can't finish | Medium | Provide starter code, 3 of 5 tasks (not all 5) |

---

## 9. Next Steps (Phase 1)

With all research complete, proceed to Phase 1 design:

1. **data-model.md**: Define curriculum entities (Module, Lesson, Lab, Assessment, Project)
2. **contracts/*.md**: Specify learning objectives, prerequisites, deliverables for each module
3. **quickstart.md**: Environment setup guide (Docker, ROS 2, Gazebo, Isaac, Unity)
4. **Update agent context**: Add robotics technologies to Claude's memory

All NEEDS CLARIFICATION items are now resolved with research-backed decisions.
