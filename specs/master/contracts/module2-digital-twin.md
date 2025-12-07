# Module 2: The Digital Twin (Gazebo & Unity)

**Duration**: 2-3 weeks | **Points**: 20 | **Focus**: Physics simulation and environment building

---

## Learning Objectives

By the end of this module, students will be able to:

1. **Simulate Physics in Gazebo**
   - Create Gazebo worlds with custom environments
   - Configure physics engines (gravity, friction, damping)
   - Simulate collisions and contact dynamics
   - Understand simulation vs. real-world trade-offs

2. **Integrate Robots with Simulation**
   - Spawn URDF robots in Gazebo
   - Configure Gazebo plugins for sensors and controllers
   - Bridge ROS 2 topics with simulation state
   - Control simulated robots via ROS 2 commands

3. **Simulate Sensors**
   - Add LiDAR, depth cameras, and IMUs to robots
   - Process sensor data in ROS 2 nodes
   - Visualize sensor data in RViz
   - Understand sensor noise models

4. **Build High-Fidelity Visualizations in Unity**
   - Set up Unity with ROS-TCP-Connector
   - Import humanoid models into Unity scenes
   - Subscribe to ROS 2 topics from Unity
   - Create photorealistic environments for demos

5. **Design Digital Twins**
   - Understand digital twin concept (virtual replica of physical system)
   - Synchronize simulation state with robot control
   - Use simulation for testing before real-world deployment

---

## Prerequisites

### Technical Prerequisites
- **Module 1 Complete**: ROS 2 nodes, URDF, RViz
- **Basic 3D Concepts**: Coordinate systems, rotations, transforms
- **Optional Unity**: C# helpful but not required (templates provided)

### Software Setup
- Gazebo Classic 11 or Gazebo Sim (Fortress/Garden)
- Unity 2021.3 LTS+ (free personal license)
- ROS-TCP-Connector Unity package

---

## Module Outline

### Week 1: Gazebo Fundamentals
**Topics**:
- Gazebo architecture and plugins
- Creating worlds with SDF (Simulation Description Format)
- Physics engines: ODE, Bullet, Simbody
- Spawning robots and objects dynamically

**Lab 4** (5 points): Create Gazebo World with Physics
- Build custom world (floor, walls, objects)
- Configure physics parameters (gravity, friction)
- Spawn Module 1 URDF humanoid
- Test collision detection
- Due: End of Week 1

### Week 2: Sensor Simulation
**Topics**:
- LiDAR sensors: 2D and 3D scanning
- Depth cameras: RGB-D data and point clouds
- IMUs: acceleration and orientation
- Sensor noise and accuracy modeling

**Lab 5** (5 points): Integrate Unity Visualization with ROS 2
- Install Unity and ROS-TCP-Connector
- Import humanoid model from Module 1
- Subscribe to `/joint_states` and `/tf` topics
- Animate robot in Unity based on ROS 2 data
- Due: End of Week 2

### Week 3: Digital Twin Integration
**Topics**:
- Human-robot interaction in simulation
- Multi-robot simulations
- Real-time control loops (Gazebo ↔ ROS 2)
- Debugging simulation issues

**Lab 6** (10 points): Simulate Humanoid with Sensor Data
- Add LiDAR, depth camera, and IMU to humanoid
- Publish sensor data to ROS 2 topics
- Write node to process and visualize sensor data
- Create RViz dashboard showing all sensors
- Document sensor specifications
- Due: End of Week 3

---

## Technologies Used

| Technology | Purpose | Resources |
|------------|---------|-----------|
| Gazebo Classic 11 | Physics simulation | [Gazebo Tutorials](http://gazebosim.org/tutorials) |
| Gazebo Sim (Ignition) | Next-gen simulator | [Gazebo Docs](https://gazebosim.org/docs) |
| SDF | Simulation world format | [SDF Specification](http://sdformat.org/) |
| Unity 2021.3 LTS | High-fidelity rendering | [Unity Learn](https://learn.unity.com/) |
| ROS-TCP-Connector | Unity-ROS 2 bridge | [GitHub Repo](https://github.com/Unity-Technologies/ROS-TCP-Connector) |
| gazebo_ros_pkgs | ROS 2 Gazebo plugins | [Documentation](http://gazebosim.org/tutorials?tut=ros2_overview) |

---

## Deliverables

### Lab 4: Gazebo World with Physics (5 points)
**Due**: Week 1, Friday 11:59 PM

**Requirements**:
- Gazebo world file (.world) with custom environment
- At least 3 objects with different physics properties
- Humanoid from Module 1 spawned in world
- Launch file to start Gazebo with world
- README documenting physics parameters

**Submission**: Git repository URL

**Rubric**:
- World file structure (1 pt)
- Physics parameters configured (1 pt)
- Humanoid spawns correctly (1 pt)
- Collision detection works (1 pt)
- Documentation (1 pt)

### Lab 5: Unity-ROS 2 Integration (5 points)
**Due**: Week 2, Friday 11:59 PM

**Requirements**:
- Unity project with humanoid scene
- ROS-TCP-Connector configured
- Subscribes to `/joint_states` topic
- Robot animates in Unity based on ROS 2 data
- README with setup instructions and screenshots

**Submission**: Git repository URL (Unity project) + demo video (1-2 min)

**Rubric**:
- Unity project builds successfully (1 pt)
- ROS-TCP-Connector configured (1 pt)
- Robot animation syncs with ROS 2 (2 pts)
- Documentation and screenshots (1 pt)

### Lab 6: Humanoid with Sensors (10 points)
**Due**: Week 3, Friday 11:59 PM

**Requirements**:
- Gazebo world with humanoid and environment
- LiDAR sensor (2D or 3D) publishing to `/scan` or `/point_cloud`
- Depth camera publishing to `/camera/depth/image_raw`
- IMU publishing to `/imu/data`
- ROS 2 node processing sensor data (e.g., obstacle detection)
- RViz config showing all sensor visualizations
- README with sensor specifications and examples

**Submission**: Git repository URL + ROS bag file (30 seconds of sensor data)

**Rubric**:
- All three sensors configured correctly (3 pts)
- Sensor data publishes to correct topics (2 pts)
- Processing node implements basic logic (2 pts)
- RViz visualization configured (2 pts)
- Documentation and ROS bag (1 pt)

---

## Assessment

**Total Module Points**: 20

| Assessment | Points | Type |
|------------|--------|------|
| Lab 4 | 5 | Hands-on simulation |
| Lab 5 | 5 | Hands-on Unity integration |
| Lab 6 | 10 | Hands-on sensor simulation |
| Weekly Quizzes (optional) | 0 | Formative (not graded) |

**Late Policy**: 10% deduction per day, max 3 days late

---

## Resources

### Official Documentation
- [Gazebo Classic Tutorials](http://gazebosim.org/tutorials)
- [Gazebo ROS 2 Integration](http://gazebosim.org/tutorials?tut=ros2_overview)
- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)

### Video Tutorials
- [Gazebo Simulation for Robotics (The Construct)](https://www.theconstructsim.com/robotigniteacademy_learnros/ros-courses-library/gazebo-simulation/)
- [Unity for Robotics Tutorials](https://www.youtube.com/playlist?list=PLz3qVfbCh3G5N6u-5JUJDYIhV_VhXJZnl)

### Community
- [Gazebo Community](https://community.gazebosim.org/)
- [Unity Robotics Forum](https://forum.unity.com/forums/robotics.623/)

---

## Advanced Topics (Optional Enrichment)

For students with extra time or interest:

1. **Multi-Robot Simulation**
   - Spawn multiple humanoids in Gazebo
   - Implement collision avoidance
   - Coordinate motion using ROS 2 topics

2. **Custom Gazebo Plugins**
   - Write C++ plugin for custom sensor
   - Extend Gazebo functionality for specific needs

3. **VR Integration with Unity**
   - Use Unity XR toolkit for VR visualization
   - Interact with simulated robot in VR

4. **Real-Time Performance**
   - Profile simulation performance
   - Optimize physics settings for faster-than-realtime

---

## Gazebo vs. Unity: When to Use Each

| Aspect | Gazebo | Unity |
|--------|--------|-------|
| **Physics** | Accurate, ROS-native | Basic, game-engine physics |
| **Graphics** | Functional | Photorealistic |
| **Sensors** | Robotics-specific (LiDAR, IMU) | Camera, depth, basic |
| **Use Case** | Algorithm development, testing | Demos, visualization, VR |
| **Learning Curve** | Moderate (ROS integration) | Steeper (C#, Unity Editor) |

**Recommendation**: Use Gazebo for control/perception development, Unity for final presentations and demos.

---

## Success Criteria

Students successfully complete this module when they can:

1. ✅ Build Gazebo worlds with realistic physics
2. ✅ Simulate robots with multiple sensors (LiDAR, cameras, IMU)
3. ✅ Process sensor data in ROS 2 for basic perception tasks
4. ✅ Integrate Unity visualization with ROS 2 systems
5. ✅ Understand digital twin concept and its applications

**Next Module**: Module 3 (NVIDIA Isaac) uses these simulation skills for advanced perception and AI training with photorealistic rendering.
