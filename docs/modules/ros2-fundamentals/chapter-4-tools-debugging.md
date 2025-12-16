---
sidebar_position: 5
---

# Chapter 4: ROS 2 Tools and Debugging

## Overview

This chapter covers the essential ROS 2 tools for debugging, monitoring, and managing robotic systems. These tools are crucial for developing and maintaining complex robotic applications.

## Command Line Tools

### ros2 run

Execute a specific executable from a package:

```bash
# Run a node from a package
ros2 run turtlesim turtlesim_node

# With arguments
ros2 run my_package my_node --ros-args -p param_name:=value

# With multiple parameters
ros2 run my_package my_node --ros-args -p param1:=value1 -p param2:=value2
```

### ros2 doctor

Introduced in newer ROS 2 distributions, ros2 doctor provides system health checks:

```bash
# Basic system check
ros2 doctor

# Verbose output
ros2 doctor -v

# Check specific aspects
ros2 doctor --reporter network
ros2 doctor --reporter rosdistro
```

### ros2 node

List and manage nodes:

```bash
# List all running nodes
ros2 node list

# Get information about a specific node
ros2 node info /turtlesim

# List node parameters
ros2 param list /turtlesim
```

### ros2 topic

Inspect and interact with topics:

```bash
# List all topics
ros2 topic list

# Get information about a specific topic
ros2 topic info /turtle1/cmd_vel

# Echo messages on a topic
ros2 topic echo /turtle1/pose

# Publish a message to a topic
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 2.0}, angular: {z: 1.8}}'

# Show topic statistics
ros2 topic hz /turtle1/pose
```

### ros2 service

Work with services:

```bash
# List all services
ros2 service list

# Get information about a specific service
ros2 service info /clear

# Call a service
ros2 service call /clear std_srvs/srv/Empty
```

### ros2 action

Work with actions:

```bash
# List all actions
ros2 action list

# Get information about a specific action
ros2 action info /fibonacci_as

# Send a goal to an action
ros2 action send_goal /fibonacci example_interfaces/action/Fibonacci '{order: 5}'
```

### ros2 param

Manage parameters:

```bash
# List parameters for a node
ros2 param list /turtlesim

# Get a parameter value
ros2 param get /turtlesim background_r

# Set a parameter value
ros2 param set /turtlesim background_r 255
```

## Visualization Tools

### RViz2

RViz2 is the 3D visualization tool for ROS 2:

```bash
# Launch RViz2
ros2 run rviz2 rviz2
```

Key features:
- Visualize sensor data (point clouds, laser scans, images)
- Display robot models and transforms
- Interactive markers for controlling robots
- Plot data in real-time

### rqt

rqt is a Qt-based framework for GUI tools:

```bash
# Launch rqt
ros2 run rqt_gui rqt_gui

# Launch specific rqt plugins
ros2 run rqt_plot rqt_plot
ros2 run rqt_graph rqt_graph
ros2 run rqt_console rqt_console
```

## System Monitoring Tools

### ros2 doctor

Check the health of your ROS 2 system:

```bash
# Basic system check
ros2 doctor

# Verbose output
ros2 doctor -v

# Check specific aspects
ros2 doctor --reporter network
```

### ros2 bag

Record and replay ROS 2 messages:

```bash
# Record all topics
ros2 bag record -a

# Record specific topics
ros2 bag record /topic1 /topic2

# Record with custom options
ros2 bag record -o my_recording /turtle1/pose

# Play back a recording
ros2 bag play my_recording

# List contents of a bag file
ros2 bag info my_recording
```

### ros2 launch

Run launch files:

```bash
# Run a launch file
ros2 launch my_package my_launch_file.py

# Run with arguments
ros2 launch my_package my_launch_file.py robot_name:=my_robot

# Dry run to see what would be launched
ros2 launch --dry-run my_package my_launch_file.py
```

## Debugging Techniques

### Logging

ROS 2 provides multiple logging levels:

```cpp
// C++ logging
RCLCPP_DEBUG(this->get_logger(), "Debug message");
RCLCPP_INFO(this->get_logger(), "Info message");
RCLCPP_WARN(this->get_logger(), "Warning message");
RCLCPP_ERROR(this->get_logger(), "Error message");
RCLCPP_FATAL(this->get_logger(), "Fatal message");
```

```python
# Python logging
self.get_logger().debug('Debug message')
self.get_logger().info('Info message')
self.get_logger().warn('Warning message')
self.get_logger().error('Error message')
self.get_logger().fatal('Fatal message')
```

### Setting Log Levels

```bash
# Set log level for all nodes
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=INFO

# Set log level for a specific node
ros2 run my_package my_node --ros-args --log-level INFO

# Set log level for specific components
ros2 run my_package my_node --ros-args --log-level my_package:=DEBUG
```

### Debugging with GDB

```bash
# Run a node with GDB
gdb --args ros2 run my_package my_node
(gdb) run

# Or use ros2 debug
ros2 debug my_package my_node
```

## Performance Analysis

### Memory and CPU Monitoring

```bash
# Monitor system resources
htop
iotop

# Monitor specific ROS 2 processes
ros2 run my_package my_node &
pid=$(pgrep -f my_node)
top -p $pid
```

### Network Analysis

```bash
# Monitor network traffic
netstat -i
iftop

# Check DDS communication
ros2 doctor --reporter network
```

## Troubleshooting Common Issues

### Topic Connection Issues

```bash
# Check if nodes are connected to topics
ros2 topic info /topic_name

# Verify QoS profiles match
ros2 topic info /topic_name --verbose
```

### Node Communication Issues

```bash
# Check network connectivity
ros2 doctor

# Verify ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID

# Check for firewall issues
sudo ufw status
```

### Resource Leaks

Monitor for common resource leaks:

```bash
# Check for memory leaks
valgrind --tool=memcheck ros2 run my_package my_node

# Monitor file descriptors
lsof -p $(pgrep -f my_node)
```

## Best Practices

1. **Use meaningful names**: Choose descriptive names for nodes, topics, and services
2. **Validate inputs**: Always validate incoming messages and parameters
3. **Log appropriately**: Use appropriate log levels for different types of information
4. **Handle errors gracefully**: Implement proper error handling and recovery
5. **Monitor performance**: Regularly check CPU, memory, and network usage
6. **Document your system**: Keep documentation updated with system architecture

## Summary

This chapter covered essential ROS 2 tools for development, debugging, and system management. These tools are crucial for building robust and maintainable robotic systems. With this foundation in ROS 2, we can now move on to simulation environments in the next module.