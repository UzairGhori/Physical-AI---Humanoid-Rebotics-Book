---
sidebar_position: 2
---

# Chapter 1: Introduction to ROS 2 Concepts

## Overview

This chapter introduces the fundamental concepts of Robot Operating System 2 (ROS 2), the next-generation middleware framework for robotics development. We'll explore what ROS 2 is, why it was developed, and how it differs from its predecessor ROS 1.

## What is ROS 2?

Robot Operating System 2 (ROS 2) is not an actual operating system but rather a flexible framework for writing robot software. It provides a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

### Key Characteristics

- **Distributed**: ROS 2 allows processes to run on different machines and communicate seamlessly
- **Modular**: Components can be developed and tested independently
- **Language Agnostic**: Supports multiple programming languages (C++, Python, Rust, etc.)
- **Open Source**: Community-driven with extensive documentation and packages
- **Production-Ready**: Designed with industrial and commercial applications in mind

## Why ROS 2?

ROS 2 was developed to address limitations in the original ROS (ROS 1) and provide a more robust foundation for modern robotics:

- **Real-time support**: Better real-time capabilities for time-critical applications
- **Multi-robot systems**: Improved support for multiple robots working together
- **Security**: Built-in security features for production environments
- **Quality of Service (QoS)**: Configurable communication policies for different requirements
- **Cross-platform**: Better support for different operating systems (Linux, Windows, macOS, embedded systems)
- **Professional use**: Production-ready for industrial applications with improved stability
- **Modern tooling**: Enhanced development tools, debugging capabilities, and system introspection

## Current ROS 2 Distributions

ROS 2 follows a time-based release schedule with Long-Term Support (LTS) distributions. As of 2025, the actively supported distributions include:

- **Rolling Ridley**: The development distribution that tracks the latest features
- **Jazzy Jalisco** (2025): The latest stable release with 5-year support
- **Iron Irwini** (2023): 5-year support ending in 2028
- **Humble Hawksbill** (2022): 5-year support ending in 2027 (LTS)

For this course, we recommend using **Jazzy Jalisco** as it provides the latest features with long-term support, or **Humble Hawksbill** for maximum stability and compatibility.

## Core Architecture

ROS 2 is built on Data Distribution Service (DDS) which provides a middleware layer that enables communication between different components of a robotic system.

### Key Components

1. **Nodes**: Processes that perform computation
2. **Topics**: Named buses over which nodes exchange messages
3. **Messages**: Data packets sent between nodes
4. **Services**: Synchronous request/response communication
5. **Actions**: Asynchronous goal-oriented communication
6. **Parameters**: Configuration values that can be changed at runtime

## Installation

For detailed installation instructions, refer to the [official ROS 2 documentation](https://docs.ros.org/en/humble/Installation.html).

### Supported Distributions

- **Humble Hawksbill** (LTS): Recommended for long-term support
- **Iron Irwini**: Latest stable release
- **Jazzy Jalisco**: Development release

## Development Environment

ROS 2 uses a workspace-based development model:

```bash
# Create a workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

## Summary

This chapter introduced the fundamental concepts of ROS 2, including its purpose, advantages over ROS 1, and core architectural components. The next chapter will dive deeper into nodes, topics, services, and actions - the building blocks of ROS 2 applications.