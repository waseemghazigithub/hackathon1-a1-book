# Module 1: The Robotic Nervous System (ROS 2)

**Duration**: 3 weeks | **Points**: 20 | **Focus**: Middleware for robot control

---

## Learning Objectives

By the end of this module, students will be able to:

1. **Explain ROS 2 Architecture**
   - Describe the publish-subscribe, service, and action communication patterns
   - Understand DDS (Data Distribution Service) middleware layer
   - Differentiate between ROS 1 and ROS 2 design philosophies

2. **Create Custom ROS 2 Packages**
   - Initialize packages using `ros2 pkg create`
   - Write nodes in Python (rclpy) and C++ (rclcpp)
   - Configure package.xml and CMakeLists.txt / setup.py

3. **Implement Communication Patterns**
   - Build publisher/subscriber pairs for topics
   - Create service clients and servers for request-response
   - Use action servers for long-running tasks (e.g., robot navigation)

4. **Build Robot Descriptions**
   - Write URDF (Unified Robot Description Format) files for humanoid robots
   - Use XACRO macros to reduce repetition
   - Define joints, links, and collision geometry

5. **Visualize and Debug**
   - Use RViz to visualize robot models and sensor data
   - Debug with ROS 2 CLI tools (`ros2 topic`, `ros2 node`, `rqt_graph`)
   - Interpret TF (transform) tree for robot kinematics

---

## Prerequisites

### Technical Prerequisites
- **Python 3.8+**: OOP, functions, classes, modules
- **Linux Command Line**: Basic navigation, file operations, text editing
- **Git**: Clone, commit, push (for lab submissions)
- **Optional C++**: Helpful but not required for this module

### Pre-Module Preparation (2-3 hours)
- Complete ROS 2 "Getting Started" tutorial (official docs)
- Install ROS 2 Humble via Docker (see quickstart.md)
- Verify installation: `ros2 run demo_nodes_cpp talker`

---

## Module Outline

### Week 1: ROS 2 Fundamentals
**Topics**:
- ROS 2 architecture overview (nodes, topics, messages)
- Installing and configuring ROS 2 workspace
- Creating your first node (publisher/subscriber)
- Understanding message types (std_msgs, sensor_msgs)

**Lab 1** (5 points): Create Custom ROS 2 Node
- Build a package with publisher and subscriber
- Publish robot state messages at 10 Hz
- Log received data
- Due: End of Week 1

### Week 2: Services and Communication
**Topics**:
- Service communication pattern (request-response)
- Parameters and dynamic reconfiguration
- Launch files for multi-node systems
- Quality of Service (QoS) settings

**Lab 2** (5 points): Implement Service Client/Server
- Create service for robot command requests
- Implement server to process commands
- Write client to send requests
- Due: End of Week 2

### Week 3: Robot Description and Visualization
**Topics**:
- URDF basics: links, joints, geometry
- XACRO for modular robot descriptions
- RViz visualization and interactive markers
- TF (Transform) tree and coordinate frames

**Lab 3** (10 points): Build URDF Humanoid Model
- Create URDF for simple humanoid (torso, head, arms, legs)
- Define joint hierarchy and kinematics
- Visualize in RViz with joint_state_publisher
- Animate joints with sliders
- Due: End of Week 3

---

## Technologies Used

| Technology | Purpose | Resources |
|------------|---------|-----------|
| ROS 2 Humble/Iron | Robot middleware | [Official Docs](https://docs.ros.org/en/humble/) |
| rclpy | Python client library | [API Reference](https://docs.ros2.org/foxy/api/rclpy/) |
| URDF | Robot description format | [URDF Tutorial](http://wiki.ros.org/urdf/Tutorials) |
| XACRO | XML macros for URDF | [XACRO Docs](http://wiki.ros.org/xacro) |
| RViz | 3D visualization tool | [RViz User Guide](http://wiki.ros.org/rviz/UserGuide) |
| Colcon | Build system | [Colcon Tutorial](https://docs.ros.org/en/humble/Tutorials/Colcon-Tutorial.html) |

---

## Deliverables

### Lab 1: Custom ROS 2 Node (5 points)
**Due**: Week 1, Friday 11:59 PM

**Requirements**:
- ROS 2 package named `my_robot_control`
- Publisher node: publishes to `/robot_state` at 10 Hz
- Subscriber node: logs received messages
- Launch file to start both nodes
- README with build and run instructions

**Submission**: Git repository URL

**Rubric**:
- Package builds successfully (1 pt)
- Publisher publishes at 10 Hz (1 pt)
- Subscriber correctly processes messages (2 pts)
- Code quality and documentation (1 pt)

### Lab 2: Service Client/Server (5 points)
**Due**: Week 2, Friday 11:59 PM

**Requirements**:
- Service definition for robot command (custom .srv file)
- Service server: processes commands and returns status
- Service client: sends requests and handles responses
- Launch file for server
- README with usage examples

**Submission**: Git repository URL

**Rubric**:
- Service definition is correct (1 pt)
- Server implements logic correctly (2 pts)
- Client sends requests and handles responses (1 pt)
- Documentation and code quality (1 pt)

### Lab 3: URDF Humanoid Model (10 points)
**Due**: Week 3, Friday 11:59 PM

**Requirements**:
- URDF file for humanoid (minimum 10 joints: head, shoulders, elbows, hips, knees)
- XACRO macros for repeated structures (arms, legs)
- Visual and collision geometry for all links
- Correct joint hierarchy (kinematic tree)
- RViz visualization with joint_state_publisher
- README with model description and screenshots

**Submission**: Git repository URL

**Rubric**:
- URDF structure and joint hierarchy (3 pts)
- XACRO usage for modularity (2 pts)
- Visual/collision geometry (2 pts)
- RViz visualization works correctly (2 pts)
- Documentation and screenshots (1 pt)

---

## Assessment

**Total Module Points**: 20

| Assessment | Points | Type |
|------------|--------|------|
| Lab 1 | 5 | Hands-on implementation |
| Lab 2 | 5 | Hands-on implementation |
| Lab 3 | 10 | Hands-on implementation |
| Weekly Quizzes (optional) | 0 | Formative (not graded) |

**Late Policy**: 10% deduction per day, max 3 days late

---

## Resources

### Official Documentation
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)

### Video Tutorials
- [ROS 2 for Beginners (Articulated Robotics)](https://www.youtube.com/playlist?list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT)
- [ROS 2 Control Tutorial](https://www.youtube.com/watch?v=4VVrTCnxvSw)

### Books
- "A Gentle Introduction to ROS" by Jason M. O'Kane (Chapters 1-5)
- "Programming Robots with ROS" (relevant chapters)

### Community
- [ROS Discourse](https://discourse.ros.org/)
- [ROS Answers](https://answers.ros.org/)

---

## Success Criteria

Students successfully complete this module when they can:

1. ✅ Build and run multi-node ROS 2 systems
2. ✅ Debug communication issues using ROS 2 CLI tools
3. ✅ Create robot models from scratch using URDF
4. ✅ Visualize robot state in RViz
5. ✅ Understand when to use topics vs. services vs. actions

**Next Module**: Module 2 (Digital Twin Simulation) builds on these foundations by simulating the robots you've described in Gazebo and Unity.
