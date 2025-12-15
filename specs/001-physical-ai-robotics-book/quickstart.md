# Quickstart Guide: Physical AI & Humanoid Robotics Book/Course

## Overview
This guide provides a rapid setup path for the "Physical AI & Humanoid Robotics" book/course. It covers the essential steps to get started with the core technologies: ROS 2, simulation environments, NVIDIA Isaac, and Vision-Language-Action (VLA) models.

## Prerequisites

### System Requirements
- **Operating System**: Ubuntu 22.04 LTS or Windows 11
- **CPU**: 8+ cores recommended
- **RAM**: 16GB minimum, 32GB recommended
- **GPU**: NVIDIA RTX 3060 or higher (for Isaac Sim and Unity)
- **Storage**: 50GB+ free space
- **Internet**: Stable connection for downloads

### Software Dependencies
- Python 3.10+
- Node.js 18+
- Git
- Docker (recommended)

## Installation Steps

### 1. ROS 2 Humble Hawksbill Setup
```bash
# Ubuntu 22.04
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-colcon-common-extensions
source /opt/ros/humble/setup.bash
```

### 2. Docusaurus Documentation Framework
```bash
npx create-docusaurus@latest docs classic
cd docs
npm install
```

### 3. Simulation Environment Setup

#### Gazebo Garden
```bash
sudo apt install gazebo
# Or follow installation guide at: https://gazebosim.org/docs/garden/install
```

#### Unity Hub (for Unity 2023.2+)
1. Download Unity Hub from https://unity.com/download
2. Install Unity Hub
3. Install Unity 2023.2+ LTS version through Unity Hub
4. Install packages: XR, Robotics, Visual Scripting

#### NVIDIA Isaac Sim (Prerequisites)
1. Ensure NVIDIA GPU with RTX 3060+ and current drivers
2. Install NVIDIA Omniverse Launcher
3. Download Isaac Sim through Omniverse

### 4. VLA Model Dependencies
```bash
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
pip install transformers datasets accelerate
pip install openai clip
```

## Course Structure Overview

### Modules
1. **ROS 2 Fundamentals** - Core concepts of Robot Operating System 2
2. **Simulation Environments** - Gazebo and Unity for robot simulation
3. **NVIDIA Isaac Robotics** - Advanced robotics simulation and AI
4. **Vision-Language-Action (VLA)** - Embodied intelligence models

### Content Organization
```
docs/
├── modules/
│   ├── ros2-fundamentals/
│   │   ├── 01-introduction-to-ros2.md
│   │   ├── 02-nodes-topics-services.md
│   │   └── 03-ros2-practical-labs.md
│   ├── simulation/
│   │   ├── 01-gazebo-basics.md
│   │   ├── 02-unity-robotics.md
│   │   └── 03-simulation-labs.md
│   ├── nvidia-isaac/
│   │   ├── 01-isaac-sim-overview.md
│   │   ├── 02-ai-integration.md
│   │   └── 03-isaac-labs.md
│   └── vla-integration/
│       ├── 01-vla-models-overview.md
│       ├── 02-embodied-intelligence.md
│       └── 03-vla-labs.md
├── labs/
├── capstone/
├── assets/
└── src/
```

## Running the Documentation

### Local Development
```bash
cd docs
npm start
```
This command starts a local development server and opens the documentation in your browser. Most changes are reflected live without having to restart the server.

### Building for Production
```bash
npm run build
```
This command generates static content in the `build` directory and can be served using any static hosting service.

## First Steps in the Course

1. Complete the ROS 2 fundamentals module
2. Set up your simulation environment (Gazebo or Unity)
3. Run the basic robot simulation examples
4. Progress through the weekly content in sequence
5. Complete the capstone project integrating all concepts

## Troubleshooting Common Issues

### ROS 2 Installation Issues
- Ensure your locale is set to UTF-8: `locale`
- Check that your network allows the package repositories

### Simulation Performance
- If experiencing slow performance, try reducing simulation quality settings
- Close other applications to free up system resources
- Consider using cloud-based simulation options

### Isaac Sim Compatibility
- Verify GPU compatibility with Isaac Sim requirements
- Update graphics drivers to the latest version
- Check NVIDIA Omniverse connection

## Getting Help
- Check the FAQ section in the main documentation
- Join the course community forum
- Review the troubleshooting guides in each module
- Contact course support for technical issues