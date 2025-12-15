---
sidebar_position: 4
---

# Chapter 3: Custom Messages and Launch Files

## Overview

This chapter covers how to create custom message types in ROS 2 and how to use launch files to manage complex robotic systems with multiple nodes.

## Custom Messages

While ROS 2 provides many standard message types, custom messages are often needed for specific applications.

### Message Definition Structure

Messages are defined using the Interface Definition Language (IDL):

```
# Custom message example: RobotStatus.msg
string robot_name
int32 battery_level
float64[] position  # x, y, z coordinates
bool is_moving
time last_update
```

### Creating Custom Messages

1. **Create a msg directory** in your package:
```
my_robot_msgs/
├── CMakeLists.txt
├── package.xml
└── msg/
    └── RobotStatus.msg
```

2. **Define the message** in a `.msg` file:
```
# msg/RobotStatus.msg
string robot_name
int32 battery_level
float64[] position
bool is_moving
builtin_interfaces/Time last_update
```

3. **Update package.xml**:
```xml
<buildtool_depend>ament_cmake</buildtool_depend>
<build_depend>builtin_interfaces</build_depend>
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>builtin_interfaces</exec_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

4. **Update CMakeLists.txt**:
```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  DEPENDENCIES builtin_interfaces
)
```

### Using Custom Messages in Code

**Publisher with Custom Message (C++):**
```cpp
#include "my_robot_msgs/msg/robot_status.hpp"
#include "rclcpp/rclcpp.hpp"

class RobotStatusPublisher : public rclcpp::Node
{
public:
  RobotStatusPublisher() : Node("robot_status_publisher")
  {
    publisher_ = this->create_publisher<my_robot_msgs::msg::RobotStatus>(
      "robot_status", 10);
    timer_ = this->create_wall_timer(
      1000ms, std::bind(&RobotStatusPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = my_robot_msgs::msg::RobotStatus();
    message.robot_name = "MyRobot01";
    message.battery_level = 85;
    message.position = {1.0, 2.0, 0.0};
    message.is_moving = false;
    message.last_update = this->get_clock()->now();

    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<my_robot_msgs::msg::RobotStatus>::SharedPtr publisher_;
};
```

## Launch Files

Launch files allow you to start multiple nodes with a single command and manage complex robotic systems.

### Basic Launch File (Python)

```python
# launch/my_launch_file.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='turtle_teleop_key',
            name='teleop',
            remappings=[
                ('/turtle1/cmd_vel', '/my_turtle/cmd_vel'),
            ]
        )
    ])
```

### Launch File with Parameters

```python
# launch/robot_with_params.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    robot_name_launch_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='default_robot',
        description='Name of the robot'
    )

    return LaunchDescription([
        robot_name_launch_arg,
        Node(
            package='my_robot_driver',
            executable='robot_driver',
            name='robot_driver',
            parameters=[
                {'robot_name': LaunchConfiguration('robot_name')},
                {'max_velocity': 1.0},
                {'wheel_radius': 0.05}
            ]
        )
    ])
```

### Launch File with Conditions

```python
# launch/conditional_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        Node(
            package='my_robot_driver',
            executable='robot_driver',
            name='robot_driver',
            parameters=[
                {'use_sim_time': use_sim_time}
            ],
            condition=IfCondition(use_sim_time)
        )
    ])
```

## Advanced Launch Features

### Including Other Launch Files

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('my_robot_description'),
                    'launch',
                    'robot_description.launch.py'
                ])
            ])
        )
    ])
```

### Setting Environment Variables

```python
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable(name='RCUTILS_LOGGING_SEVERITY_THRESHOLD', value='INFO'),
        Node(
            package='my_robot_driver',
            executable='robot_driver',
            name='robot_driver'
        )
    ])
```

## Parameters

Parameters provide a way to configure nodes at runtime:

```python
# In a node
class MyRobotNode(Node):
    def __init__(self):
        super().__init__('my_robot_node')

        # Declare parameters with default values
        self.declare_parameter('robot_name', 'default_robot')
        self.declare_parameter('max_velocity', 1.0)
        self.declare_parameter('wheel_radius', 0.05)

        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
```

### YAML Parameter Files

```yaml
# config/robot_params.yaml
my_robot_node:
  ros__parameters:
    robot_name: "my_robot"
    max_velocity: 2.0
    wheel_radius: 0.05
    sensors:
      - "lidar_front"
      - "camera_left"
      - "camera_right"
```

## Summary

This chapter covered custom message creation and launch file configuration, essential tools for building complex robotic systems. The next chapter will explore ROS 2 tools and debugging techniques.