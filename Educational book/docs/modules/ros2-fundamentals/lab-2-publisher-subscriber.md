---
sidebar_position: 7
---

# Lab 2: Publisher/Subscriber Implementation

## Objective

In this lab, you will create your first ROS 2 publisher and subscriber nodes. You'll learn how to implement the publish/subscribe communication pattern, which is fundamental to ROS 2 architecture.

## Prerequisites

- Completed Lab 1: ROS 2 Installation and Environment Setup
- Basic knowledge of C++ or Python
- Understanding of ROS 2 nodes and topics

## Learning Outcomes

After completing this lab, you will be able to:
- Create a ROS 2 publisher node that sends messages
- Create a ROS 2 subscriber node that receives messages
- Understand Quality of Service (QoS) settings
- Test publisher/subscriber communication

## Step-by-Step Instructions

### Step 1: Create a New Package

1. Navigate to your workspace source directory:
   ```bash
   cd ~/ros2_ws/src
   ```

2. Create a new package for this lab:
   ```bash
   ros2 pkg create --build-type ament_cmake ros2_publisher_subscriber --dependencies rclcpp rclpy std_msgs
   ```

### Step 2: Create the Publisher Node (C++)

1. Navigate to the package directory:
   ```bash
   cd ros2_publisher_subscriber
   ```

2. Create the publisher source file:
   ```bash
   mkdir -p src
   touch src/publisher_member_function.cpp
   ```

3. Edit the publisher file with the following content:
   ```cpp
   #include "rclcpp/rclcpp.hpp"
   #include "std_msgs/msg/string.hpp"

   class MinimalPublisher : public rclcpp::Node
   {
   public:
     MinimalPublisher()
     : Node("minimal_publisher"), count_(0)
     {
       publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
       timer_ = this->create_wall_timer(
         500ms, std::bind(&MinimalPublisher::timer_callback, this));
     }

   private:
     void timer_callback()
     {
       auto message = std_msgs::msg::String();
       message.data = "Hello, world! " + std::to_string(count_++);
       RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
       publisher_->publish(message);
     }
     rclcpp::TimerBase::SharedPtr timer_;
     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
     size_t count_;
   };

   int main(int argc, char * argv[])
   {
     rclcpp::init(argc, argv);
     rclcpp::spin(std::make_shared<MinimalPublisher>());
     rclcpp::shutdown();
     return 0;
   }
   ```

### Step 3: Create the Subscriber Node (C++)

1. Create the subscriber source file:
   ```bash
   touch src/subscriber_member_function.cpp
   ```

2. Edit the subscriber file with the following content:
   ```cpp
   #include "rclcpp/rclcpp.hpp"
   #include "std_msgs/msg/string.hpp"

   class MinimalSubscriber : public rclcpp::Node
   {
   public:
     MinimalSubscriber()
     : Node("minimal_subscriber")
     {
       subscription_ = this->create_subscription<std_msgs::msg::String>(
         "topic", 10,
         [this](const std_msgs::msg::String::SharedPtr msg) {
           RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
         });
     }

   private:
     rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
   };

   int main(int argc, char * argv[])
   {
     rclcpp::init(argc, argv);
     rclcpp::spin(std::make_shared<MinimalSubscriber>());
     rclcpp::shutdown();
     return 0;
   }
   ```

### Step 4: Update CMakeLists.txt

1. Edit the CMakeLists.txt file:
   ```bash
   nano CMakeLists.txt
   ```

2. Add the following executable definitions after the `ament_package()` call:
   ```cmake
   find_package(ament_cmake REQUIRED)
   find_package(rclcpp REQUIRED)
   find_package(std_msgs REQUIRED)

   add_executable(publisher_member_function src/publisher_member_function.cpp)
   ament_target_dependencies(publisher_member_function rclcpp std_msgs)

   add_executable(subscriber_member_function src/subscriber_member_function.cpp)
   ament_target_dependencies(subscriber_member_function rclcpp std_msgs)

   install(TARGETS
     publisher_member_function
     subscriber_member_function
     DESTINATION lib/${PROJECT_NAME}
   )

   ament_package()
   ```

### Step 5: Create Python Versions (Optional)

1. Create a Python directory:
   ```bash
   mkdir -p ros2_publisher_subscriber/launch
   ```

2. Create the publisher Python script:
   ```bash
   mkdir -p ros2_publisher_subscriber/publisher_subscriber_py
   touch ros2_publisher_subscriber/publisher_subscriber_py/__init__.py
   touch ros2_publisher_subscriber/publisher_subscriber_py/publisher_member_function.py
   ```

3. Add the following content to the publisher Python file:
   ```python
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String


   class MinimalPublisher(Node):

       def __init__(self):
           super().__init__('minimal_publisher')
           self.publisher_ = self.create_publisher(String, 'topic', 10)
           timer_period = 0.5  # seconds
           self.timer = self.create_timer(timer_period, self.timer_callback)
           self.i = 0

       def timer_callback(self):
           msg = String()
           msg.data = 'Hello World: %d' % self.i
           self.publisher_.publish(msg)
           self.get_logger().info('Publishing: "%s"' % msg.data)
           self.i += 1


   def main(args=None):
       rclpy.init(args=args)
       minimal_publisher = MinimalPublisher()
       rclpy.spin(minimal_publisher)
       minimal_publisher.destroy_node()
       rclpy.shutdown()


   if __name__ == '__main__':
       main()
   ```

4. Create the subscriber Python script:
   ```bash
   touch ros2_publisher_subscriber/publisher_subscriber_py/subscriber_member_function.py
   ```

5. Add the following content to the subscriber Python file:
   ```python
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String


   class MinimalSubscriber(Node):

       def __init__(self):
           super().__init__('minimal_subscriber')
           self.subscription = self.create_subscription(
               String,
               'topic',
               self.listener_callback,
               10)
           self.subscription  # prevent unused variable warning

       def listener_callback(self, msg):
           self.get_logger().info('I heard: "%s"' % msg.data)


   def main(args=None):
       rclpy.init(args=args)
       minimal_subscriber = MinimalSubscriber()
       rclpy.spin(minimal_subscriber)
       minimal_subscriber.destroy_node()
       rclpy.shutdown()


   if __name__ == '__main__':
       main()
   ```

### Step 6: Update package.xml

1. Edit the package.xml file to include the Python entry points:
   ```xml
   <exec_depend>rclpy</exec_depend>

   <export>
     <build_type>ament_cmake</build_type>
   </export>
   ```

2. Add the Python entry points at the end of the file:
   ```xml
   <exec_depend>rclpy</exec_depend>

   <export>
     <build_type>ament_cmake</build_type>
   </export>
   ```

Actually, let me update the package.xml file properly:
   ```bash
   nano package.xml
   ```

Add the following before the closing `</package>` tag:
```xml
  <exec_depend>rclpy</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### Step 7: Build and Test

1. Navigate back to the workspace root:
   ```bash
   cd ~/ros2_ws
   ```

2. Build the package:
   ```bash
   colcon build --packages-select ros2_publisher_subscriber
   ```

3. Source the workspace:
   ```bash
   source install/setup.bash
   ```

### Step 8: Run the Publisher and Subscriber

1. Terminal 1 - Run the publisher:
   ```bash
   ros2 run ros2_publisher_subscriber publisher_member_function
   ```

2. Terminal 2 - Run the subscriber:
   ```bash
   ros2 run ros2_publisher_subscriber subscriber_member_function
   ```

3. You should see the publisher sending messages and the subscriber receiving them.

### Step 9: Quality of Service (QoS) Exploration

1. Create a new publisher with different QoS settings:
   ```bash
   touch src/publisher_qos.cpp
   ```

2. Add content with reliable delivery:
   ```cpp
   #include "rclcpp/rclcpp.hpp"
   #include "std_msgs/msg/string.hpp"

   class QoSPublisher : public rclcpp::Node
   {
   public:
     QoSPublisher()
     : Node("qos_publisher"), count_(0)
     {
       // Create publisher with reliable QoS
       rclcpp::QoS qos(10);
       qos.reliable();
       qos.keep_last(10);

       publisher_ = this->create_publisher<std_msgs::msg::String>("qos_topic", qos);
       timer_ = this->create_wall_timer(
         1000ms, std::bind(&QoSPublisher::timer_callback, this));
     }

   private:
     void timer_callback()
     {
       auto message = std_msgs::msg::String();
       message.data = "QoS Message " + std::to_string(count_++);
       RCLCPP_INFO(this->get_logger(), "Publishing QoS: '%s'", message.data.c_str());
       publisher_->publish(message);
     }
     rclcpp::TimerBase::SharedPtr timer_;
     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
     size_t count_;
   };

   int main(int argc, char * argv[])
   {
     rclcpp::init(argc, argv);
     rclcpp::spin(std::make_shared<QoSPublisher>());
     rclcpp::shutdown();
     return 0;
   }
   ```

3. Update CMakeLists.txt to include the new executable:
   ```cmake
   add_executable(qos_publisher src/publisher_qos.cpp)
   ament_target_dependencies(qos_publisher rclcpp std_msgs)

   install(TARGETS
     publisher_member_function
     subscriber_member_function
     qos_publisher
     DESTINATION lib/${PROJECT_NAME}
   )
   ```

4. Rebuild the package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select ros2_publisher_subscriber
   source install/setup.bash
   ```

## Troubleshooting

### Common Issues and Solutions

1. **Build errors**: Ensure all dependencies are properly declared in CMakeLists.txt and package.xml

2. **Node not found**: Make sure you've sourced the workspace after building

3. **No communication**: Check that both nodes are on the same topic and QoS profiles are compatible

4. **Permission errors**: Don't run ROS 2 commands with sudo

## Assessment Questions

1. What is the purpose of the `rclcpp::QoS` object in ROS 2?

2. Explain the difference between `keep_last(10)` and `keep_all()` QoS history policies.

3. Why is it important to match QoS profiles between publishers and subscribers?

## Summary

In this lab, you created your first ROS 2 publisher and subscriber nodes in both C++ and Python. You learned about the publish/subscribe communication pattern and explored Quality of Service settings. This fundamental pattern is used throughout ROS 2 for asynchronous communication between nodes.

The next lab will focus on services and actions for synchronous and goal-oriented communication.