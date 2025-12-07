<!--
Sync Impact Report:
Version change: 0.0.0 -> 0.1.0 (MINOR: New principles/sections added and guidance expanded)
Modified principles:
  - PRINCIPLE_1_NAME -> Technical Accuracy
  - PRINCIPLE_2_NAME -> Real-world Alignment
  - PRINCIPLE_3_NAME -> Clarity for Practitioners
  - PRINCIPLE_4_NAME -> Embodied Intelligence Perspective
  - PRINCIPLE_5_NAME -> (Removed as separate principle, now part of Key Standards)
  - PRINCIPLE_6_NAME -> (Removed as separate principle, now part of Content Structure Requirements)
Added sections:
  - Key Standards
  - Content Structure Requirements
Removed sections: None
Templates requiring updates:
  - .specify/templates/plan-template.md: ⚠ pending
  - .specify/templates/spec-template.md: ⚠ pending
  - .specify/templates/tasks-template.md: ⚠ pending
  - .specify/templates/commands/*.md: ⚠ pending
Follow-up TODOs:
  - TODO(RATIFICATION_DATE): Original adoption date unknown.
-->
# Physical AI & Humanoid Robotics — Capstone Framework and Documentation Constitution

## Core Principles

### Technical Accuracy
Technical accuracy across robotics, ROS 2, simulation, and AI systems.

### Real-world Alignment
Alignment with real-world robotics workflow (ROS 2 → Gazebo/Unity → Isaac → VLA).

### Clarity for Practitioners
Clarity for engineering students and robotics practitioners.

### Embodied Intelligence Perspective
Embodied intelligence perspective: bridging digital AI with physical systems.

## Key Standards

All descriptions of robotics frameworks (ROS 2, Gazebo, Unity, Isaac ROS, Nav2) MUST be technically correct.
Include precise terminology for humanoid robotics (URDF, kinematics, joints, controllers, sensors).
Provide actionable, implementation-level guidance (commands, file structures, workflows).
Include examples where relevant: ROS 2 nodes, topics, controllers, SLAM pipelines, VLA action chains.
Maintain educational clarity: concepts MUST be teachable and progressively structured.

## Content Structure Requirements

Module 1: ROS 2 (robotic nervous system)
Module 2: Digital Twin simulation (Gazebo & Unity)
Module 3: NVIDIA Isaac & advanced AI-robot perception
Module 4: Vision-Language-Action robotics (LLMs + embodied agents)
Capstone description: Autonomous Humanoid Robot executing VLA pipeline

## Governance

### Constraints
Tone: authoritative, engineering-focused, and implementation-ready.
No fictional robotics claims — all components MUST be feasible within current ROS 2/Isaac tooling.
Use diagrams (ASCII or described) if helpful.
Optional: include Python + ROS 2 snippet examples.
No hallucinated ROS packages or Isaac features.

### Success Criteria
Students CAN follow all modules to build a working humanoid simulation pipeline.
Output IS technically valid and CAN integrate with real ROS 2 / Isaac Sim workflows.
Each module CLEARLY links AI (LLMs, perception, planning) to robotic control.
Documentation IS complete enough for a capstone evaluator to approve.

**Version**: 0.1.0 | **Ratified**: TODO(RATIFICATION_DATE): Original adoption date unknown. | **Last Amended**: 2025-12-05
