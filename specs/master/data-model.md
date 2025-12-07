# Data Model: Physical AI Curriculum

**Date**: 2025-12-06
**Branch**: master
**Related**: [plan.md](./plan.md), [research.md](./research.md)

## Purpose

This document defines the core entities, relationships, and data structures for the Physical AI & Humanoid Robotics curriculum. These entities represent the curriculum's organizational structure, content units, assessment mechanisms, and student interactions.

---

## Entity Relationship Diagram (ASCII)

```
┌─────────────┐
│ Curriculum  │
│             │ 1
└──────┬──────┘
       │ has
       │
       │ 1..*
┌──────▼──────┐
│   Module    │
│             │ 1
└──────┬──────┘
       │ contains
       │
       │ 1..*
┌──────▼──────┐       ┌─────────────┐
│   Lesson    │       │     Lab     │
│             │       │             │
└──────┬──────┘       └──────┬──────┘
       │                     │
       │ requires            │ produces
       │                     │
       │ 0..*         0..1   │
┌──────▼──────────────▼──────▼──────┐
│         Assessment                 │
│                                    │
└──────┬─────────────────────────────┘
       │
       │ evaluates
       │
       │ 1..*
┌──────▼──────┐       ┌─────────────┐
│   Student   │──────▶│   Project   │
│             │ works │             │
└─────────────┘  on   └─────────────┘
                       (capstone)
```

---

## Core Entities

### 1. Curriculum

The top-level container for the entire Physical AI program.

**Attributes**:
- `title`: string - "Physical AI & Humanoid Robotics Capstone"
- `version`: string - Semantic version (e.g., "1.0.0")
- `duration_weeks`: integer - Total quarter length (10-12 weeks)
- `total_credits`: integer - Academic credit units
- `prerequisites`: list[string] - Required background (see research.md section 3)
- `learning_outcomes`: list[string] - High-level program objectives
- `modules`: list[Module] - Ordered list of 4 modules

**Learning Outcomes** (from constitution):
1. Design and implement ROS 2-based robot control systems
2. Build digital twin simulations in Gazebo and Unity
3. Apply NVIDIA Isaac for advanced perception and training
4. Integrate Vision-Language-Action pipelines for autonomous robots
5. Deploy end-to-end humanoid robot systems in simulation

**Validation Rules**:
- Must have exactly 4 modules (constitution requirement)
- Modules must be ordered sequentially (1 → 2 → 3 → 4)
- Total duration must fit academic quarter (10-12 weeks)

---

### 2. Module

A major curriculum division focusing on a specific technology or concept area.

**Attributes**:
- `module_number`: integer (1-4)
- `title`: string
  - Module 1: "The Robotic Nervous System (ROS 2)"
  - Module 2: "The Digital Twin (Gazebo & Unity)"
  - Module 3: "The AI-Robot Brain (NVIDIA Isaac)"
  - Module 4: "Vision-Language-Action Robotics"
- `focus`: string - Core concept (e.g., "Middleware for robot control")
- `duration_weeks`: integer (2-3 weeks per module)
- `prerequisites`: list[Module] - Required prior modules
- `learning_objectives`: list[string] - Specific skills/knowledge
- `technologies`: list[Technology] - Tools and frameworks used
- `lessons`: list[Lesson] - Instructional content units
- `labs`: list[Lab] - Hands-on assignments
- `assessment`: Assessment - Evaluation mechanism
- `points`: integer - Contribution to total grade (20 points each)

**Example (Module 1)**:
```json
{
  "module_number": 1,
  "title": "The Robotic Nervous System (ROS 2)",
  "focus": "Middleware for robot control",
  "duration_weeks": 3,
  "prerequisites": [],
  "learning_objectives": [
    "Understand ROS 2 architecture (nodes, topics, services, actions)",
    "Create custom ROS 2 packages with Python and C++",
    "Build URDF models for humanoid robots",
    "Visualize robot state in RViz",
    "Debug ROS 2 systems using CLI tools"
  ],
  "technologies": [
    {"name": "ROS 2 Humble/Iron", "type": "middleware"},
    {"name": "rclpy", "type": "library"},
    {"name": "URDF/XACRO", "type": "format"},
    {"name": "RViz", "type": "visualization"}
  ],
  "lessons": [...],  // See Lesson entity
  "labs": [...],     // See Lab entity
  "assessment": {...},  // See Assessment entity
  "points": 20
}
```

**State Transitions**:
```
NOT_STARTED → IN_PROGRESS → COMPLETED
```
(Applies to student's progress through module)

---

### 3. Lesson

An instructional content unit (lecture, reading, video) within a module.

**Attributes**:
- `lesson_id`: string - Unique identifier (e.g., "M1L2")
- `module`: Module - Parent module
- `title`: string - Lesson name
- `sequence`: integer - Order within module
- `duration_minutes`: integer - Expected study time
- `format`: enum[lecture, reading, video, demo, workshop]
- `content_url`: string - Link to materials (slides, docs, video)
- `learning_objectives`: list[string] - What students will learn
- `key_concepts`: list[string] - Technical terms introduced
- `examples`: list[CodeExample] - Demonstrations or code snippets
- `prerequisites`: list[Lesson] - Required prior lessons

**Example (Module 1, Week 1, Lesson 2)**:
```json
{
  "lesson_id": "M1L2",
  "module": "Module 1",
  "title": "ROS 2 Communication Patterns: Topics and Messages",
  "sequence": 2,
  "duration_minutes": 90,
  "format": "lecture",
  "content_url": "/curriculum/module1-ros2/lessons/week1-lesson2-topics.md",
  "learning_objectives": [
    "Explain publish-subscribe pattern in ROS 2",
    "Create publisher and subscriber nodes",
    "Understand message types (std_msgs, sensor_msgs)"
  ],
  "key_concepts": [
    "Publisher",
    "Subscriber",
    "Topic",
    "Message",
    "QoS (Quality of Service)"
  ],
  "examples": [
    {
      "title": "Simple Talker/Listener",
      "language": "python",
      "file_path": "/curriculum/module1-ros2/examples/talker_listener.py"
    }
  ],
  "prerequisites": ["M1L1"]
}
```

---

### 4. Lab

A hands-on assignment where students implement robotics concepts.

**Attributes**:
- `lab_id`: string - Unique identifier (e.g., "Lab1")
- `module`: Module - Parent module
- `title`: string - Lab name
- `sequence`: integer - Order within module (1-3)
- `points`: integer - Grade contribution (5 or 10 points)
- `difficulty`: enum[beginner, intermediate, advanced]
- `estimated_hours`: integer - Expected completion time
- `objectives`: list[string] - What students will build
- `requirements`: list[string] - Technical deliverables
- `starter_code_url`: string - Link to template repository
- `submission_format`: string - How to submit (Git repo, ROS bag, video)
- `rubric`: Rubric - Grading criteria
- `due_date_offset`: integer - Days from module start

**Example (Module 1, Lab 1)**:
```json
{
  "lab_id": "Lab1",
  "module": "Module 1",
  "title": "Create Custom ROS 2 Node",
  "sequence": 1,
  "points": 5,
  "difficulty": "beginner",
  "estimated_hours": 3,
  "objectives": [
    "Create ROS 2 package with Python node",
    "Implement publisher for custom message type",
    "Implement subscriber to process data",
    "Visualize data in RViz"
  ],
  "requirements": [
    "ROS 2 package named 'my_robot_control'",
    "Publisher node publishing to /robot_state topic",
    "Subscriber node logging received messages",
    "README with build and run instructions"
  ],
  "starter_code_url": "/curriculum/module1-ros2/labs/lab1-starter",
  "submission_format": "Git repository URL",
  "rubric": {
    "criteria": [
      {"name": "Package builds successfully", "points": 1},
      {"name": "Publisher publishes at 10 Hz", "points": 1},
      {"name": "Subscriber correctly processes messages", "points": 2},
      {"name": "Code quality and documentation", "points": 1}
    ]
  },
  "due_date_offset": 7
}
```

---

### 5. Assessment

Evaluation mechanism for measuring student learning.

**Attributes**:
- `assessment_id`: string - Unique identifier
- `type`: enum[lab, quiz, project, peer_review]
- `title`: string
- `module`: Module - Parent module (null for capstone)
- `points`: integer - Grade contribution
- `rubric`: Rubric - Evaluation criteria
- `submission_deadline`: datetime
- `auto_graded`: boolean - Whether automated grading is used

**Types**:

#### Lab Assessment (most common)
See Lab entity above—labs are the primary assessment mechanism.

#### Quiz Assessment (formative, optional)
```json
{
  "assessment_id": "M1Q1",
  "type": "quiz",
  "title": "Module 1 Week 1 Check",
  "module": "Module 1",
  "points": 0,  // Formative only
  "format": "multiple_choice",
  "num_questions": 10,
  "time_limit_minutes": 30,
  "open_book": true,
  "auto_graded": true
}
```

#### Capstone Project Assessment
```json
{
  "assessment_id": "CAPSTONE",
  "type": "project",
  "title": "Autonomous Humanoid with VLA Pipeline",
  "module": null,  // Spans all modules
  "points": 20,
  "rubric": {
    "criteria": [
      {"name": "Perception pipeline", "points": 4},
      {"name": "Language understanding", "points": 4},
      {"name": "Action execution", "points": 6},
      {"name": "System integration", "points": 4},
      {"name": "Documentation", "points": 2}
    ]
  },
  "deliverables": [
    "Git repository with ROS 2 workspace",
    "5-minute demo video",
    "Technical report (10-15 pages)"
  ],
  "submission_deadline": "Week 10, Friday 11:59 PM",
  "auto_graded": false
}
```

#### Peer Review Assessment
```json
{
  "assessment_id": "PEER_REVIEW",
  "type": "peer_review",
  "title": "Capstone Code Review",
  "module": null,
  "points": 10,  // Part of collaboration grade
  "num_reviews_required": 2,
  "review_criteria": [
    "Code quality and organization",
    "ROS 2 best practices",
    "Documentation clarity",
    "Suggested improvements"
  ]
}
```

---

### 6. Student

An individual enrolled in the curriculum.

**Attributes**:
- `student_id`: string - Unique identifier
- `name`: string
- `email`: string
- `cohort`: string - Quarter/year (e.g., "Fall 2024")
- `prerequisites_met`: boolean - Passed background check
- `hardware_tier`: enum[cloud, budget, full] - Student's compute resources
- `progress`: dict[Module, ModuleProgress] - Per-module status
- `lab_submissions`: list[LabSubmission]
- `capstone_project`: Project - Final project
- `total_points`: integer - Cumulative grade (0-100)

**ModuleProgress** (nested object):
```json
{
  "module": "Module 1",
  "status": "completed",
  "lessons_completed": 12,
  "lessons_total": 12,
  "labs_submitted": 3,
  "labs_total": 3,
  "points_earned": 20,
  "points_possible": 20
}
```

**LabSubmission** (nested object):
```json
{
  "lab": "Lab1",
  "submitted_at": "2024-09-15T18:30:00Z",
  "repository_url": "https://github.com/student/lab1-ros2",
  "status": "graded",
  "points_earned": 5,
  "points_possible": 5,
  "feedback": "Excellent work! Clean code and thorough README."
}
```

---

### 7. Project (Capstone)

The culminating project where students integrate all module learnings.

**Attributes**:
- `project_id`: string - Unique identifier
- `student`: Student - Project owner
- `title`: string - Project name (student-chosen or assigned)
- `scenario`: string - "Household Assistant Robot" (see research.md)
- `tasks_selected`: list[string] - 3 of 5 tasks (Object Retrieval, Delivery, etc.)
- `repository_url`: string - Git repository
- `simulation_platform`: enum[gazebo, isaac_sim] - Student's choice
- `technologies_used`: list[Technology]
- `milestones`: list[Milestone] - Timeline checkpoints
- `final_grade`: integer - Points earned (0-20)
- `demo_video_url`: string - YouTube/Vimeo link
- `technical_report_url`: string - PDF link

**Milestone** (nested object):
```json
{
  "milestone_number": 1,
  "title": "Project Setup & Environment Configuration",
  "due_week": 1,
  "status": "completed",
  "deliverable": "ROS 2 workspace with starter URDF and launch files",
  "verified_by_ta": true
}
```

**Required Milestones** (from research.md):
1. Project setup & environment (Week 1-2)
2. Perception pipeline (Week 3-4)
3. LLM integration & task planning (Week 5-6)
4. Navigation & manipulation (Week 7-8)
5. Integration, testing, documentation (Week 9-10)

---

### 8. Technology

A tool, framework, or library used in the curriculum.

**Attributes**:
- `name`: string - Technology name
- `type`: enum[middleware, simulator, library, tool, language]
- `version`: string - Specific version required
- `modules_used_in`: list[Module] - Where it appears
- `installation_url`: string - Setup documentation
- `license`: string - Software license (e.g., "Apache 2.0", "NVIDIA Academic")
- `hardware_requirements`: dict - GPU, RAM, etc.

**Example Technologies**:
```json
[
  {
    "name": "ROS 2 Humble",
    "type": "middleware",
    "version": "22.04",
    "modules_used_in": ["Module 1", "Module 2", "Module 3", "Module 4"],
    "installation_url": "https://docs.ros.org/en/humble/Installation.html",
    "license": "Apache 2.0",
    "hardware_requirements": {"ram_gb": 8, "storage_gb": 20}
  },
  {
    "name": "NVIDIA Isaac Sim",
    "type": "simulator",
    "version": "2023.1.0",
    "modules_used_in": ["Module 3"],
    "installation_url": "https://developer.nvidia.com/isaac-sim",
    "license": "NVIDIA Academic (Free)",
    "hardware_requirements": {"gpu": "NVIDIA RTX 3060+", "vram_gb": 6, "ram_gb": 32}
  },
  {
    "name": "LangChain",
    "type": "library",
    "version": "0.1.0+",
    "modules_used_in": ["Module 4"],
    "installation_url": "https://python.langchain.com/docs/get_started/installation",
    "license": "MIT",
    "hardware_requirements": {}
  }
]
```

---

### 9. CodeExample

A reusable code snippet or demo provided to students.

**Attributes**:
- `example_id`: string
- `title`: string
- `description`: string - What the code demonstrates
- `language`: enum[python, cpp, bash, yaml]
- `file_path`: string - Location in curriculum repository
- `lesson`: Lesson - Associated lesson (optional)
- `lines_of_code`: integer
- `concepts_demonstrated`: list[string]

**Example**:
```json
{
  "example_id": "EX_M1_TALKER",
  "title": "ROS 2 Publisher (Talker)",
  "description": "Simple publisher node that sends string messages at 1 Hz",
  "language": "python",
  "file_path": "/curriculum/module1-ros2/examples/talker.py",
  "lesson": "M1L2",
  "lines_of_code": 30,
  "concepts_demonstrated": [
    "ROS 2 node initialization",
    "Creating a publisher",
    "Timer callbacks",
    "Message publishing"
  ]
}
```

---

### 10. Rubric

A structured grading criteria for assessments.

**Attributes**:
- `rubric_id`: string
- `assessment`: Assessment - Parent assessment
- `criteria`: list[Criterion] - Individual grading components
- `total_points`: integer - Sum of all criteria points

**Criterion** (nested object):
```json
{
  "name": "Code Quality",
  "description": "Clean, readable, follows PEP 8 / ROS 2 conventions",
  "points": 2,
  "levels": [
    {"level": "Excellent", "points": 2, "description": "All standards met"},
    {"level": "Good", "points": 1.5, "description": "Minor issues"},
    {"level": "Needs Improvement", "points": 1, "description": "Multiple violations"},
    {"level": "Insufficient", "points": 0, "description": "Major quality issues"}
  ]
}
```

---

## Relationships

### Curriculum → Module (1:4)
- Curriculum contains exactly 4 modules
- Modules are ordered sequentially

### Module → Lesson (1:many)
- Each module has 8-12 lessons (2-3 weeks × 4 lessons/week)
- Lessons are ordered within module

### Module → Lab (1:3)
- Each module has 3 labs
- Labs are assessed for points (5, 5, 10 per module)

### Module → Assessment (1:1 or 1:many)
- Each module has labs (multiple) + optional quizzes
- Capstone project is separate (not tied to single module)

### Student → Module (many:many via ModuleProgress)
- Students progress through modules sequentially
- Track completion status per module

### Student → Lab (many:many via LabSubmission)
- Students submit labs for grading
- Can resubmit for improved grade (instructor policy)

### Student → Project (1:1)
- Each student has one capstone project
- Project spans weeks 1-10

### Lesson → CodeExample (1:many)
- Lessons reference multiple code examples
- Examples can be reused across lessons

---

## Validation Rules

### Curriculum-Level
1. Must have exactly 4 modules (constitution requirement)
2. Total points must sum to 100 (80 from modules, 20 from capstone)
3. Duration must fit 10-12 week quarter

### Module-Level
1. Each module worth 20 points (labs only)
2. Lab points per module: 5 + 5 + 10 = 20
3. Module sequence: 1 → 2 → 3 → 4 (no skipping)

### Lab-Level
1. Lab points must be 5 or 10
2. Must have clear rubric with criteria summing to lab points
3. Starter code must exist at specified URL

### Assessment-Level
1. All rubric criteria must sum to total assessment points
2. Submission deadline must be within quarter dates
3. Auto-graded assessments must have digital submission format

### Student-Level
1. Cannot start module N+1 without completing module N labs
2. Total points cannot exceed 100
3. Prerequisites must be met before enrollment

### Project-Level
1. Must select exactly 3 of 5 available tasks
2. All 5 milestones must be defined
3. Repository must be public or accessible to instructors

---

## State Machines

### Student Module Progress
```
[NOT_STARTED]
    ↓ (student views first lesson)
[IN_PROGRESS]
    ↓ (all lessons completed AND all labs submitted)
[COMPLETED]
```

### Lab Submission Status
```
[NOT_SUBMITTED]
    ↓ (student submits repository URL)
[SUBMITTED]
    ↓ (TA assigns grade)
[GRADED] ──→ (optional resubmission) → [RESUBMITTED] → [GRADED]
```

### Capstone Project Status
```
[NOT_STARTED]
    ↓ (project proposal submitted)
[SETUP] (Weeks 1-2)
    ↓ (milestone 1 verified)
[DEVELOPMENT] (Weeks 3-8)
    ↓ (milestone 4 verified)
[TESTING] (Weeks 9-10)
    ↓ (final deliverables submitted)
[SUBMITTED]
    ↓ (graded by instructor)
[COMPLETED]
```

---

## Data Storage

### File Formats

**Curriculum Metadata**: `curriculum.yaml`
```yaml
title: Physical AI & Humanoid Robotics Capstone
version: 1.0.0
duration_weeks: 10
total_credits: 4
prerequisites:
  - Python 3 proficiency
  - AI/ML fundamentals
  - Linear algebra
modules:
  - module1
  - module2
  - module3
  - module4
```

**Module Specification**: `module1/module.yaml`
```yaml
module_number: 1
title: The Robotic Nervous System (ROS 2)
focus: Middleware for robot control
duration_weeks: 3
points: 20
learning_objectives:
  - Understand ROS 2 architecture
  - Create custom ROS 2 packages
  - Build URDF models
lessons:
  - lessons/week1-lesson1.md
  - lessons/week1-lesson2.md
  # ...
labs:
  - labs/lab1.yaml
  - labs/lab2.yaml
  - labs/lab3.yaml
```

**Lab Specification**: `module1/labs/lab1.yaml`
```yaml
lab_id: Lab1
title: Create Custom ROS 2 Node
sequence: 1
points: 5
difficulty: beginner
estimated_hours: 3
objectives:
  - Create ROS 2 package
  - Implement publisher/subscriber
requirements:
  - Package named 'my_robot_control'
  - Publisher to /robot_state
starter_code: /curriculum/module1-ros2/labs/lab1-starter
submission_format: git_repository
due_date_offset: 7
rubric:
  criteria:
    - name: Package builds
      points: 1
    - name: Publisher works
      points: 1
    - name: Subscriber works
      points: 2
    - name: Documentation
      points: 1
```

**Student Record**: Database (PostgreSQL, MongoDB)
```json
{
  "student_id": "STU_2024_001",
  "name": "Jane Doe",
  "email": "jane@example.com",
  "cohort": "Fall 2024",
  "prerequisites_met": true,
  "hardware_tier": "budget",
  "progress": {
    "module1": {"status": "completed", "points_earned": 20},
    "module2": {"status": "in_progress", "points_earned": 10},
    "module3": {"status": "not_started", "points_earned": 0},
    "module4": {"status": "not_started", "points_earned": 0}
  },
  "total_points": 30
}
```

---

## Summary

This data model provides:
1. **Clear entity definitions** for curriculum, modules, lessons, labs, assessments, students, projects
2. **Relationships** showing how entities connect
3. **Validation rules** ensuring data integrity
4. **State machines** tracking student and project progress
5. **Storage formats** (YAML for curriculum, database for student data)

All entities align with the Physical AI curriculum goals and support the learning outcomes defined in the constitution.
