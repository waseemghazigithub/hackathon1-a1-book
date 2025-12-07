# Physical AI Curriculum Quickstart Guide

**Last Updated**: 2025-12-06
**Target Audience**: Students and instructors setting up the curriculum environment

---

## Overview

This guide walks you through setting up the complete development environment for the Physical AI & Humanoid Robotics curriculum. By the end, you'll have:

- ✅ ROS 2 Humble or Iron installed
- ✅ Gazebo Classic 11 or Gazebo Sim configured
- ✅ Unity 2021.3 LTS with ROS-TCP-Connector (optional)
- ✅ NVIDIA Isaac Sim 2023.1+ (if GPU available)
- ✅ Python environment with AI/ML libraries
- ✅ Docker containers for reproducible environments

**Estimated Setup Time**: 2-4 hours (depending on hardware and internet speed)

---

## Prerequisites

### Hardware Requirements

Choose your tier based on available hardware:

#### Tier 1: Minimum (Cloud/Lab Access)
- **CPU**: 4-core Intel/AMD
- **RAM**: 16 GB
- **GPU**: Integrated (no dedicated GPU)
- **Storage**: 100 GB free
- **Note**: Can run ROS 2 + Gazebo. No Isaac Sim.

#### Tier 2: Recommended (Budget Build)
- **CPU**: 6-8 core (Ryzen 5/7, Intel i5/i7)
- **RAM**: 32 GB
- **GPU**: NVIDIA RTX 3060 12GB
- **Storage**: 250 GB SSD
- **Note**: Can run everything including Isaac Sim at 1080p

#### Tier 3: Optimal (Full Experience)
- **CPU**: 8+ core (Ryzen 7/9, Intel i7/i9)
- **RAM**: 64 GB
- **GPU**: NVIDIA RTX 4070/4080 (16GB+ VRAM)
- **Storage**: 500 GB+ NVMe SSD
- **Note**: Isaac Sim at 4K, multiple simulations

### Software Prerequisites

- **OS**: Ubuntu 22.04 LTS (recommended) or Ubuntu 24.04 LTS
  - **Windows Users**: Use WSL2 or dual-boot (WSL2 has ROS 2 support)
  - **macOS Users**: Use VM (Parallels, VMware) or cloud instance
- **Internet**: Broadband (will download ~20-50 GB of packages)
- **Accounts**:
  - GitHub account (for lab submissions)
  - NVIDIA Developer account (for Isaac Sim, free)
  - Optional: OpenAI/Anthropic account (for Module 4)

---

## Installation Methods

We provide **three installation methods**. Choose one:

1. **Docker (Recommended)**: Fastest, most reproducible
2. **Native Installation**: Best performance, more complex
3. **Cloud Instance**: No local setup, pay-per-use

---

## Method 1: Docker Installation (Recommended)

Docker provides pre-configured environments with all dependencies.

### Step 1: Install Docker

```bash
# Update package list
sudo apt update

# Install Docker
sudo apt install -y docker.io docker-compose

# Add your user to docker group (no sudo needed)
sudo usermod -aG docker $USER

# Log out and back in for group changes to take effect
# Verify installation
docker --version
```

### Step 2: Pull Curriculum Docker Images

We provide three images (choose based on your hardware):

```bash
# Image 1: Base ROS 2 + Gazebo (all students, ~5 GB)
docker pull physicalai/ros2-gazebo:humble

# Image 2: ROS 2 + Isaac Sim (GPU students, ~15 GB)
docker pull physicalai/ros2-isaac:2023.1

# Image 3: Full stack with LLMs (capstone, ~20 GB)
docker pull physicalai/ros2-vla:latest
```

### Step 3: Run Docker Container

#### For Base ROS 2 + Gazebo (Modules 1-2):

```bash
docker run -it \
  --name physicalai-ros2 \
  --network host \
  -v ~/physicalai-workspace:/workspace \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  physicalai/ros2-gazebo:humble
```

#### For Isaac Sim (Module 3, requires NVIDIA GPU):

```bash
docker run -it \
  --name physicalai-isaac \
  --gpus all \
  --network host \
  -v ~/physicalai-workspace:/workspace \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  physicalai/ros2-isaac:2023.1
```

### Step 4: Verify Installation

Inside the Docker container:

```bash
# Test ROS 2
ros2 run demo_nodes_cpp talker
# Open new terminal: ros2 run demo_nodes_cpp listener

# Test Gazebo
gazebo --verbose

# Test Isaac Sim (if using isaac image)
~/.local/share/ov/pkg/isaac_sim-2023.1.0/isaac-sim.sh
```

### Step 5: Set Up Workspace

```bash
# Inside Docker container
cd /workspace
mkdir -p ros2_ws/src
cd ros2_ws
colcon build
source install/setup.bash

# Add to bashrc for convenience
echo "source /workspace/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## Method 2: Native Installation

For best performance, install directly on Ubuntu 22.04.

### Step 1: Install ROS 2 Humble

```bash
# Set locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 apt repository
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl gnupg lsb-release

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Desktop
sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools

# Source ROS 2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 2: Install Gazebo Classic 11

```bash
sudo apt install -y gazebo11 libgazebo11-dev
sudo apt install -y ros-humble-gazebo-ros-pkgs
```

### Step 3: Install Python Dependencies

```bash
# System Python packages
sudo apt install -y python3-pip python3-venv

# Create virtual environment
python3 -m venv ~/physicalai-venv
source ~/physicalai-venv/bin/activate

# Install AI/ML libraries
pip install --upgrade pip
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
pip install transformers langchain openai anthropic
pip install opencv-python numpy matplotlib scipy
pip install open3d  # For point cloud processing
```

### Step 4: Install Unity (Optional, for Module 2)

```bash
# Download Unity Hub
wget -O UnityHub.AppImage https://public-cdn.cloud.unity3d.com/hub/prod/UnityHub.AppImage
chmod +x UnityHub.AppImage
./UnityHub.AppImage

# Install Unity 2021.3 LTS via Unity Hub
# Install ROS-TCP-Connector via Unity Package Manager:
# Window > Package Manager > Add package from git URL:
# https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
```

### Step 5: Install NVIDIA Isaac Sim (Optional, for Module 3)

**Requirements**: NVIDIA GPU (RTX 3060+), NVIDIA drivers 535+, CUDA 11.8+

```bash
# Install NVIDIA drivers
sudo apt install -y nvidia-driver-535

# Install CUDA 11.8
wget https://developer.download.nvidia.com/compute/cuda/11.8.0/local_installers/cuda_11.8.0_520.61.05_linux.run
sudo sh cuda_11.8.0_520.61.05_linux.run

# Download Isaac Sim (requires NVIDIA Developer account)
# Go to: https://developer.nvidia.com/isaac-sim
# Download Isaac Sim 2023.1.0 for Linux
# Extract and run:
cd ~/Downloads
tar -xzf isaac_sim-2023.1.0-linux.tar.gz
cd isaac_sim-2023.1.0
./isaac-sim.sh
```

### Step 6: Verify Installation

```bash
# Test ROS 2
ros2 run demo_nodes_cpp talker

# Test Gazebo
gazebo

# Test Isaac Sim (if installed)
cd ~/isaac_sim-2023.1.0
./isaac-sim.sh
```

---

## Method 3: Cloud Instance Setup

For students without powerful local hardware.

### AWS EC2 Setup

**Recommended Instance**: `g4dn.xlarge` (1x NVIDIA T4, 16GB VRAM, ~$0.50/hour)

#### Step 1: Launch EC2 Instance

```bash
# Launch via AWS Console or CLI
aws ec2 run-instances \
  --image-id ami-0c55b159cbfafe1f0 \  # Ubuntu 22.04
  --instance-type g4dn.xlarge \
  --key-name your-key \
  --security-group-ids sg-xxxxx
```

#### Step 2: SSH and Install

```bash
# SSH into instance
ssh -i your-key.pem ubuntu@your-instance-ip

# Follow Method 2 (Native Installation) steps
# Or use Docker (Method 1)
```

#### Step 3: Set Up VNC for GUI

```bash
# Install VNC server
sudo apt install -y tightvncserver xfce4 xfce4-goodies

# Start VNC server
vncserver :1

# Connect from local machine:
ssh -L 5901:localhost:5901 -i your-key.pem ubuntu@your-instance-ip
# Use VNC client to connect to localhost:5901
```

### Cost Estimation

| Usage | Instance | Hours/Week | Cost/Month |
|-------|----------|------------|------------|
| Light (Modules 1-2) | t3.xlarge | 10 | ~$15 |
| Moderate (+ Isaac) | g4dn.xlarge | 20 | ~$40 |
| Heavy (full-time) | g4dn.xlarge | 40 | ~$80 |

**Tip**: Stop instances when not in use to save costs.

---

## Post-Installation Setup

### 1. Create ROS 2 Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 2. Clone Curriculum Repository

```bash
cd ~/ros2_ws/src
git clone https://github.com/your-org/physicalai-curriculum.git
cd ~/ros2_ws
colcon build --packages-select physicalai-curriculum
source install/setup.bash
```

### 3. Configure Git for Lab Submissions

```bash
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
```

### 4. Test Sample Lab

```bash
# Run Module 1 example
ros2 run physicalai_curriculum talker_example

# Launch Gazebo with sample world
ros2 launch physicalai_curriculum sample_world.launch.py
```

---

## Docker Compose (Advanced)

For multi-container setups (e.g., ROS 2 + LLM inference server):

Create `docker-compose.yml`:

```yaml
version: '3.8'

services:
  ros2:
    image: physicalai/ros2-gazebo:humble
    container_name: ros2-master
    network_mode: host
    environment:
      - DISPLAY=${DISPLAY}
    volumes:
      - ~/physicalai-workspace:/workspace
      - /tmp/.X11-unix:/tmp/.X11-unix
    command: /bin/bash

  llm-server:
    image: ollama/ollama:latest
    container_name: llm-inference
    ports:
      - "11434:11434"
    volumes:
      - ollama-models:/root/.ollama
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: 1
              capabilities: [gpu]

volumes:
  ollama-models:
```

Run with:
```bash
docker-compose up -d
docker exec -it ros2-master /bin/bash
```

---

## Troubleshooting

### Issue: Gazebo won't start

**Solution**:
```bash
# Check OpenGL support
glxinfo | grep "OpenGL"

# If missing, install:
sudo apt install -y mesa-utils libgl1-mesa-glx
```

### Issue: ROS 2 topics not visible

**Solution**:
```bash
# Check ROS_DOMAIN_ID (should be same across all terminals)
echo $ROS_DOMAIN_ID

# Set if unset:
export ROS_DOMAIN_ID=0
```

### Issue: Isaac Sim crashes on startup

**Solution**:
```bash
# Check NVIDIA drivers
nvidia-smi

# Verify CUDA
nvcc --version

# Update drivers if needed:
sudo apt install --reinstall nvidia-driver-535
```

### Issue: Docker permission denied

**Solution**:
```bash
sudo usermod -aG docker $USER
# Log out and back in
```

### Issue: Unity ROS-TCP-Connector not connecting

**Solution**:
```bash
# Start ROS-TCP-Endpoint on ROS 2 side
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0

# In Unity, set ROSConnection IP to your machine's IP
```

---

## Validation Checklist

Before starting Module 1, verify:

- [ ] ROS 2 installed: `ros2 --version` shows Humble or Iron
- [ ] Gazebo works: `gazebo` launches without errors
- [ ] ROS 2 workspace builds: `colcon build` succeeds
- [ ] Sample node runs: `ros2 run demo_nodes_cpp talker` works
- [ ] RViz launches: `rviz2` opens
- [ ] Git configured: `git config --list` shows name and email
- [ ] (Optional) Isaac Sim launches: `./isaac-sim.sh` opens
- [ ] (Optional) Unity installed: Unity Hub shows 2021.3 LTS

---

## Next Steps

1. **Complete Pre-Module Preparation** (see Module 1 contract):
   - ROS 2 "Getting Started" tutorial (2-3 hours)
   - Read "A Gentle Introduction to ROS" chapters 1-3

2. **Join Course Resources**:
   - Course Slack/Discord (link from instructor)
   - GitHub organization (accept invitation)

3. **Start Module 1**:
   - Read `contracts/module1-ros2.md`
   - Watch Week 1 lecture videos
   - Begin Lab 1

---

## Additional Resources

### Documentation
- [ROS 2 Humble Docs](https://docs.ros.org/en/humble/)
- [Gazebo Tutorials](http://gazebosim.org/tutorials)
- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html)

### Community
- [ROS Discourse](https://discourse.ros.org/)
- [NVIDIA Isaac Forum](https://forums.developer.nvidia.com/c/agx-autonomous-machines/isaac/251)
- Course-specific Discord/Slack

### Learning Resources
- [ROS 2 Tutorials (Articulated Robotics)](https://www.youtube.com/playlist?list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT)
- [The Construct ROS 2 Courses](https://www.theconstructsim.com/robotigniteacademy_learnros/)

---

## Getting Help

If you encounter issues during setup:

1. **Search Documentation**: Check ROS 2 docs, Gazebo wiki, Isaac Sim docs
2. **Community Forums**: Post on ROS Discourse or course Discord
3. **Office Hours**: Attend TA office hours (schedule on course page)
4. **Email Instructor**: For account or course-specific issues

**Important**: Include the following in help requests:
- Your OS and version (`lsb_release -a`)
- Installation method (Docker, native, cloud)
- Hardware tier
- Full error messages (copy-paste, not screenshots)
- Steps to reproduce

---

**You're now ready to begin Module 1!** Good luck and enjoy building physical AI systems.
