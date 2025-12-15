---
sidebar_position: 9
---

# Lab 4: Custom Messages and Launch Files

## Objective

In this lab, you will create custom message types for your ROS 2 applications and use launch files to manage complex robotic systems with multiple nodes. This will allow you to define your own data structures and automate system startup.

## Prerequisites

- Completed all previous ROS 2 labs
- Understanding of ROS 2 nodes, topics, services, and actions
- Basic knowledge of C++ or Python

## Learning Outcomes

After completing this lab, you will be able to:
- Define and create custom message types in ROS 2
- Use custom messages in publisher and subscriber nodes
- Create launch files to start multiple nodes at once
- Configure parameters through launch files
- Use launch file conditions and arguments

## Step-by-Step Instructions

### Step 1: Create a Custom Messages Package

1. Navigate to your workspace source directory:
   ```bash
   cd ~/ros2_ws/src
   ```

2. Create a new package for custom messages:
   ```bash
   ros2 pkg create --build-type ament_cmake my_robot_msgs --dependencies builtin_interfaces rosidl_default_generators
   ```

### Step 2: Define Custom Message Types

1. Create the msg directory:
   ```bash
   mkdir -p my_robot_msgs/msg
   ```

2. Create a RobotStatus message definition:
   ```bash
   touch my_robot_msgs/msg/RobotStatus.msg
   ```

3. Add the following content to RobotStatus.msg:
   ```
   # my_robot_msgs/msg/RobotStatus.msg
   string robot_name
   int32 battery_level
   float64[] position  # x, y, z coordinates
   bool is_moving
   builtin_interfaces/Time last_update
   ```

4. Create a SensorData message definition:
   ```bash
   touch my_robot_msgs/msg/SensorData.msg
   ```

5. Add the following content to SensorData.msg:
   ```
   # my_robot_msgs/msg/SensorData.msg
   builtin_interfaces/Time timestamp
   string sensor_name
   float64[] readings
   string sensor_type
   ```

6. Create a RobotCommand message definition:
   ```bash
   touch my_robot_msgs/msg/RobotCommand.msg
   ```

7. Add the following content to RobotCommand.msg:
   ```
   # my_robot_msgs/msg/RobotCommand.msg
   string command_type  # "move", "stop", "gripper", etc.
   float64[] parameters  # command parameters
   builtin_interfaces/Time execution_time
   bool emergency_stop
   ```

### Step 3: Update package.xml for Custom Messages

1. Edit package.xml to include message generation:
   ```xml
   <?xml version="1.0"?>
   <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
   <package format="3">
     <name>my_robot_msgs</name>
     <version>0.0.0</version>
     <description>Custom messages for robot applications</description>
     <maintainer email="user@example.com">User</maintainer>
     <license>Apache-2.0</license>

     <buildtool_depend>ament_cmake</buildtool_depend>

     <build_depend>builtin_interfaces</build_depend>
     <build_depend>rosidl_default_generators</build_depend>

     <exec_depend>builtin_interfaces</exec_depend>
     <exec_depend>rosidl_default_runtime</exec_depend>

     <test_depend>ament_lint_auto</test_depend>
     <test_depend>ament_lint_common</test_depend>

     <member_of_group>rosidl_interface_packages</member_of_group>

     <export>
       <build_type>ament_cmake</build_type>
     </export>
   </package>
   ```

### Step 4: Update CMakeLists.txt for Custom Messages

1. Edit CMakeLists.txt:
   ```bash
   nano my_robot_msgs/CMakeLists.txt
   ```

2. Update the CMakeLists.txt with the following content:
   ```cmake
   cmake_minimum_required(VERSION 3.8)
   project(my_robot_msgs)

   if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
     add_compile_options(-Wall -Wextra -Wpedantic)
   endif()

   # find dependencies
   find_package(ament_cmake REQUIRED)
   find_package(builtin_interfaces REQUIRED)
   find_package(rosidl_default_generators REQUIRED)

   # Define the message files
   set(msg_files
     "msg/RobotStatus.msg"
     "msg/SensorData.msg"
     "msg/RobotCommand.msg"
   )

   rosidl_generate_interfaces(${PROJECT_NAME}
     ${msg_files}
     DEPENDENCIES builtin_interfaces
   )

   if(BUILD_TESTING)
     find_package(ament_lint_auto REQUIRED)
     # the following line skips the linter which checks for copyrights
     # comment the line when a copyright and license is added to all source files
     set(ament_cmake_copyright_FOUND TRUE)
     # the following line skips cpplint (only works in a git repo)
     # comment the line when this package is in a git repo and when
     # a copyright and license is added to all source files
     set(ament_cmake_cpplint_FOUND TRUE)
     ament_lint_auto_find_test_dependencies()
   endif()

   ament_package()
   ```

### Step 5: Build Custom Messages

1. Navigate to workspace root:
   ```bash
   cd ~/ros2_ws
   ```

2. Build the custom messages package:
   ```bash
   colcon build --packages-select my_robot_msgs
   ```

3. Source the workspace:
   ```bash
   source install/setup.bash
   ```

### Step 6: Create a Package Using Custom Messages

1. Navigate back to src:
   ```bash
   cd ~/ros2_ws/src
   ```

2. Create a new package that uses custom messages:
   ```bash
   ros2 pkg create --build-type ament_cmake robot_controller --dependencies rclcpp rclpy my_robot_msgs std_msgs
   ```

### Step 7: Create a Publisher with Custom Messages (C++)

1. Create the publisher source file:
   ```bash
   touch robot_controller/src/robot_status_publisher.cpp
   ```

2. Add the following content:
   ```cpp
   #include "rclcpp/rclcpp.hpp"
   #include "my_robot_msgs/msg/robot_status.hpp"

   class RobotStatusPublisher : public rclcpp::Node
   {
   public:
     RobotStatusPublisher() : Node("robot_status_publisher"), count_(0)
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
       message.battery_level = 100 - (count_ % 101);  // Decreasing battery
       message.position = {1.0 + count_ * 0.1, 2.0 + count_ * 0.05, 0.0};
       message.is_moving = (count_ % 3 != 0);  // Moves 2/3 of the time
       message.last_update = this->get_clock()->now();

       RCLCPP_INFO(this->get_logger(),
         "Publishing: Robot %s, Battery: %d%%, Position: [%.2f, %.2f, %.2f], Moving: %s",
         message.robot_name.c_str(),
         message.battery_level,
         message.position[0], message.position[1], message.position[2],
         message.is_moving ? "true" : "false");

       publisher_->publish(message);
       count_++;
     }

     rclcpp::TimerBase::SharedPtr timer_;
     rclcpp::Publisher<my_robot_msgs::msg::RobotStatus>::SharedPtr publisher_;
     size_t count_;
   };

   int main(int argc, char * argv[])
   {
     rclcpp::init(argc, argv);
     rclcpp::spin(std::make_shared<RobotStatusPublisher>());
     rclcpp::shutdown();
     return 0;
   }
   ```

### Step 8: Create a Subscriber with Custom Messages (C++)

1. Create the subscriber source file:
   ```bash
   touch robot_controller/src/robot_status_subscriber.cpp
   ```

2. Add the following content:
   ```cpp
   #include "rclcpp/rclcpp.hpp"
   #include "my_robot_msgs/msg/robot_status.hpp"

   class RobotStatusSubscriber : public rclcpp::Node
   {
   public:
     RobotStatusSubscriber() : Node("robot_status_subscriber")
     {
       subscription_ = this->create_subscription<my_robot_msgs::msg::RobotStatus>(
         "robot_status", 10,
         [this](const my_robot_msgs::msg::RobotStatus::SharedPtr msg) {
           RCLCPP_INFO(this->get_logger(),
             "Received: Robot %s, Battery: %d%%, Position: [%.2f, %.2f, %.2f], Moving: %s",
             msg->robot_name.c_str(),
             msg->battery_level,
             msg->position[0], msg->position[1], msg->position[2],
             msg->is_moving ? "true" : "false");
         });
     }

   private:
     rclcpp::Subscription<my_robot_msgs::msg::RobotStatus>::SharedPtr subscription_;
   };

   int main(int argc, char * argv[])
   {
     rclcpp::init(argc, argv);
     rclcpp::spin(std::make_shared<RobotStatusSubscriber>());
     rclcpp::shutdown();
     return 0;
   }
   ```

### Step 9: Update robot_controller CMakeLists.txt

1. Edit CMakeLists.txt:
   ```bash
   nano robot_controller/CMakeLists.txt
   ```

2. Add the following content:
   ```cmake
   cmake_minimum_required(VERSION 3.8)
   project(robot_controller)

   if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
     add_compile_options(-Wall -Wextra -Wpedantic)
   endif()

   # find dependencies
   find_package(ament_cmake REQUIRED)
   find_package(rclcpp REQUIRED)
   find_package(rclpy REQUIRED)
   find_package(my_robot_msgs REQUIRED)
   find_package(std_msgs REQUIRED)

   add_executable(robot_status_publisher src/robot_status_publisher.cpp)
   ament_target_dependencies(robot_status_publisher
     rclcpp my_robot_msgs std_msgs)

   add_executable(robot_status_subscriber src/robot_status_subscriber.cpp)
   ament_target_dependencies(robot_status_subscriber
     rclcpp my_robot_msgs std_msgs)

   install(TARGETS
     robot_status_publisher
     robot_status_subscriber
     DESTINATION lib/${PROJECT_NAME}
   )

   if(BUILD_TESTING)
     find_package(ament_lint_auto REQUIRED)
     # the following line skips the linter which checks for copyrights
     # comment the line when a copyright and license is added to all source files
     set(ament_cmake_copyright_FOUND TRUE)
     # the following line skips cpplint (only works in a git repo)
     # comment the line when this package is in a git repo and when
     # a copyright and license is added to all source files
     set(ament_cmake_cpplint_FOUND TRUE)
     ament_lint_auto_find_test_dependencies()
   endif()

   ament_package()
   ```

### Step 10: Build and Test Custom Messages

1. Navigate to workspace root:
   ```bash
   cd ~/ros2_ws
   ```

2. Build both packages:
   ```bash
   colcon build --packages-select my_robot_msgs robot_controller
   ```

3. Source the workspace:
   ```bash
   source install/setup.bash
   ```

4. Terminal 1 - Run the custom message publisher:
   ```bash
   ros2 run robot_controller robot_status_publisher
   ```

5. Terminal 2 - Run the custom message subscriber:
   ```bash
   ros2 run robot_controller robot_status_subscriber
   ```

### Step 11: Create Launch Files

1. Create a launch directory in robot_controller:
   ```bash
   mkdir -p robot_controller/launch
   ```

2. Create a basic launch file:
   ```bash
   touch robot_controller/launch/robot_system.launch.py
   ```

3. Add the following content:
   ```python
   from launch import LaunchDescription
   from launch.actions import DeclareLaunchArgument
   from launch.substitutions import LaunchConfiguration
   from launch_ros.actions import Node

   def generate_launch_description():
       # Declare launch arguments
       robot_name_arg = DeclareLaunchArgument(
           'robot_name',
           default_value='default_robot',
           description='Name of the robot'
       )

       return LaunchDescription([
           robot_name_arg,
           Node(
               package='robot_controller',
               executable='robot_status_publisher',
               name='robot_status_publisher',
               parameters=[
                   {'robot_name': LaunchConfiguration('robot_name')}
               ],
               output='screen'
           ),
           Node(
               package='robot_controller',
               executable='robot_status_subscriber',
               name='robot_status_subscriber',
               output='screen'
           )
       ])
   ```

### Step 12: Create Advanced Launch Files

1. Create a launch file with conditions:
   ```bash
   touch robot_controller/launch/conditional_robot_system.launch.py
   ```

2. Add the following content:
   ```python
   from launch import LaunchDescription
   from launch.actions import DeclareLaunchArgument
   from launch.conditions import IfCondition
   from launch.substitutions import LaunchConfiguration
   from launch_ros.actions import Node

   def generate_launch_description():
       # Declare launch arguments
       use_sim_time = DeclareLaunchArgument(
           'use_sim_time',
           default_value='false',
           description='Use simulation time if true'
       )

       debug_mode = DeclareLaunchArgument(
           'debug_mode',
           default_value='false',
           description='Enable debug output if true'
       )

       return LaunchDescription([
           use_sim_time,
           debug_mode,
           Node(
               package='robot_controller',
               executable='robot_status_publisher',
               name='robot_status_publisher',
               parameters=[
                   {'use_sim_time': LaunchConfiguration('use_sim_time')}
               ],
               output='screen'
           ),
           Node(
               package='robot_controller',
               executable='robot_status_subscriber',
               name='robot_status_subscriber',
               parameters=[
                   {'use_sim_time': LaunchConfiguration('use_sim_time')}
               ],
               output='screen',
               condition=IfCondition(LaunchConfiguration('debug_mode'))
           )
       ])
   ```

### Step 13: Create Parameter Files

1. Create a config directory:
   ```bash
   mkdir -p robot_controller/config
   ```

2. Create a parameter file:
   ```bash
   touch robot_controller/config/robot_params.yaml
   ```

3. Add the following content:
   ```yaml
   # robot_controller/config/robot_params.yaml
   robot_status_publisher:
     ros__parameters:
       robot_name: "MyCustomRobot"
       update_rate: 1.0
       battery_depletion_rate: 0.5
       position_variance: 0.1

   robot_status_subscriber:
     ros__parameters:
       log_level: "INFO"
       filter_distance: 10.0
   ```

### Step 14: Update Launch File to Use Parameters

1. Create a parameterized launch file:
   ```bash
   touch robot_controller/launch/parameterized_robot_system.launch.py
   ```

2. Add the following content:
   ```python
   from launch import LaunchDescription
   from launch.actions import DeclareLaunchArgument
   from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
   from launch_ros.actions import Node
   from launch_ros.substitutions import FindPackageShare

   def generate_launch_description():
       # Declare launch arguments
       params_file_arg = DeclareLaunchArgument(
           'params_file',
           default_value=PathJoinSubstitution([
               FindPackageShare('robot_controller'),
               'config',
               'robot_params.yaml'
           ]),
           description='Full path to params file to load'
       )

       return LaunchDescription([
           params_file_arg,
           Node(
               package='robot_controller',
               executable='robot_status_publisher',
               name='robot_status_publisher',
               parameters=[LaunchConfiguration('params_file')],
               output='screen'
           ),
           Node(
               package='robot_controller',
               executable='robot_status_subscriber',
               name='robot_status_subscriber',
               parameters=[LaunchConfiguration('params_file')],
               output='screen'
           )
       ])
   ```

### Step 15: Test Launch Files

1. Run the basic launch file:
   ```bash
   ros2 launch robot_controller robot_system.launch.py robot_name:=MyRobot01
   ```

2. Run the conditional launch file:
   ```bash
   ros2 launch robot_controller conditional_robot_system.launch.py debug_mode:=true
   ```

3. Run the parameterized launch file:
   ```bash
   ros2 launch robot_controller parameterized_robot_system.launch.py
   ```

### Step 16: Explore Launch Tools

1. List all running processes from a launch file:
   ```bash
   ros2 launch --show-args robot_controller robot_system.launch.py
   ```

2. Dry run to see what would be launched:
   ```bash
   ros2 launch --dry-run robot_controller robot_system.launch.py
   ```

## Troubleshooting

### Common Issues and Solutions

1. **Message generation errors**: Ensure rosidl_default_generators is in build_depend and rosidl_default_runtime in exec_depend

2. **Package not found**: Make sure to source the workspace after building

3. **Launch file errors**: Check that all dependencies are properly declared in package.xml

4. **Parameter loading issues**: Verify YAML file syntax and paths

## Assessment Questions

1. What is the purpose of the `member_of_group` tag in package.xml for custom messages?

2. How do launch file arguments differ from parameters?

3. What are the advantages of using custom messages over standard message types?

## Summary

In this lab, you created custom message types and used them in ROS 2 nodes. You also learned to create launch files to manage complex robotic systems with multiple nodes, parameters, and conditions. Custom messages allow you to define application-specific data structures, while launch files provide a powerful way to orchestrate complete robotic systems.

This completes the ROS 2 fundamentals module. You now have a solid foundation in ROS 2 concepts, communication patterns, and system management.