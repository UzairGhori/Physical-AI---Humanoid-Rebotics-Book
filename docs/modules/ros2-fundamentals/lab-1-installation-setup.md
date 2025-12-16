---
sidebar_position: 6
---

# Lab 1: ROS 2 Installation and Environment Setup

## Objective

In this lab, you will install ROS 2 (Jazzy Jalisco) on your development machine and set up a basic workspace. This foundational setup is essential for all subsequent ROS 2 development. While we recommend Jazzy Jalisco for its latest features and long-term support, you may also use Humble Hawksbill for maximum stability.

## Prerequisites

- Ubuntu 22.04 LTS (recommended) or Windows 10/11 with WSL2
- Basic command-line knowledge
- Administrative privileges on your machine

## Learning Outcomes

After completing this lab, you will be able to:
- Install ROS 2 Jazzy Jalisco on your system (or Humble Hawksbill for stability)
- Configure your ROS 2 environment
- Create and build a basic ROS 2 workspace
- Verify the installation with basic ROS 2 commands

## Step-by-Step Instructions

### Step 1: System Requirements Check

Before installing ROS 2, ensure your system meets the requirements:

1. Check your Ubuntu version (if using Ubuntu):
   ```bash
   lsb_release -a
   ```
   You should see Ubuntu 22.04 or later.

2. Ensure your system is up to date:
   ```bash
   sudo apt update && sudo apt upgrade -y
   ```

### Step 2: Setup Locale

Make sure your locale is set to UTF-8:

```bash
locale  # Check for LANG=en_US.UTF-8
```

If not set, configure it:
```bash
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### Step 3: Add ROS 2 Repository

1. Add the ROS 2 GPG key:
   ```bash
   sudo apt update && sudo apt install -y curl gnupg lsb-release
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   ```

2. Add the repository to your sources list:
   ```bash
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

### Step 4: Install ROS 2

1. Update your apt repository:
   ```bash
   sudo apt update
   ```

2. Install ROS 2 Jazzy Desktop package (includes Gazebo):
   ```bash
   sudo apt install -y ros-jazzy-desktop
   ```

   Alternatively, if you prefer the stable Humble Hawksbill distribution:
   ```bash
   sudo apt install -y ros-humble-desktop
   ```

3. Install additional dependencies:
   ```bash
   sudo apt install -y ros-dev-tools
   ```

### Step 5: Environment Setup

1. Source the ROS 2 setup script for Jazzy Jalisco:
   ```bash
   source /opt/ros/jazzy/setup.bash
   ```

   Or for Humble Hawksbill:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. Add this to your bashrc to automatically source the appropriate ROS 2 distribution:
   ```bash
   echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
   ```

   Or for Humble Hawksbill:
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   ```

### Step 6: Create a ROS 2 Workspace

1. Create a workspace directory:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   ```

2. Build the workspace:
   ```bash
   colcon build
   ```

3. Source the workspace:
   ```bash
   source install/setup.bash
   ```

4. Add workspace sourcing to your bashrc:
   ```bash
   echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
   ```

### Step 7: Verify Installation

1. Test with the turtlesim package:
   ```bash
   ros2 run turtlesim turtlesim_node
   ```

2. In a new terminal, run:
   ```bash
   ros2 run turtlesim turtle_teleop_key
   ```

3. You should see a turtle in a window that you can control with arrow keys.

### Step 8: Check ROS 2 Environment

1. List available nodes:
   ```bash
   ros2 node list
   ```

2. List available topics:
   ```bash
   ros2 topic list
   ```

## Troubleshooting

### Common Issues and Solutions

1. **Permission errors**: Make sure you're not running ROS 2 commands with `sudo`

2. **Package not found**: Ensure you've sourced the ROS 2 setup script:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

3. **Display issues on WSL2**: Configure X11 forwarding or use VcXsrv/X410

4. **Python import errors**: Ensure Python 3 is properly configured:
   ```bash
   python3 --version
   pip3 --version
   ```

## Assessment Questions

1. What is the difference between sourcing `/opt/ros/humble/setup.bash` and `~/ros2_ws/install/setup.bash`?

2. What command would you use to list all available ROS 2 packages?

3. Explain the purpose of the `src` directory in a ROS 2 workspace.

## Summary

In this lab, you successfully installed ROS 2 Humble Hawksbill, set up your development environment, created a workspace, and verified the installation. This foundation is essential for all subsequent ROS 2 development work.

The next lab will focus on implementing publisher/subscriber communication patterns.