---
sidebar_position: 8
---

# Lab 3: Services and Actions Implementation

## Objective

In this lab, you will implement ROS 2 services for synchronous request/response communication and actions for goal-oriented, long-running tasks. You'll understand when to use each communication pattern.

## Prerequisites

- Completed Lab 1 and Lab 2
- Understanding of ROS 2 nodes and topics
- Basic knowledge of C++ or Python

## Learning Outcomes

After completing this lab, you will be able to:
- Create and use ROS 2 services for synchronous communication
- Implement ROS 2 actions for long-running tasks with feedback
- Understand the differences between topics, services, and actions
- Choose appropriate communication patterns for different use cases

## Step-by-Step Instructions

### Step 1: Add Service Dependencies to Package

1. Navigate to your workspace:
   ```bash
   cd ~/ros2_ws/src/ros2_publisher_subscriber
   ```

2. Update package.xml to include service dependencies:
   ```xml
   <depend>example_interfaces</depend>
   ```

3. Update CMakeLists.txt to include the service dependency:
   ```cmake
   find_package(example_interfaces REQUIRED)
   ```

### Step 2: Create a Service Server (C++)

1. Create the service server source file:
   ```bash
   touch src/service_server.cpp
   ```

2. Add the following content for an "Add Two Integers" service:
   ```cpp
   #include "rclcpp/rclcpp.hpp"
   #include "example_interfaces/srv/add_two_ints.hpp"

   class MinimalService : public rclcpp::Node
   {
   public:
     MinimalService() : Node("minimal_service")
     {
       service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
         "add_two_ints",
         [this](const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
                example_interfaces::srv::AddTwoInts::Response::SharedPtr response) {
           response->sum = request->a + request->b;
           RCLCPP_INFO(this->get_logger(), "Incoming request\na: %ld, b: %ld",
                       request->a, request->b);
           RCLCPP_INFO(this->get_logger(), "Sending back response: [%ld]", response->sum);
         });
     }

   private:
     rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
   };

   int main(int argc, char ** argv)
   {
     rclcpp::init(argc, argv);
     rclcpp::spin(std::make_shared<MinimalService>());
     rclcpp::shutdown();
     return 0;
   }
   ```

### Step 3: Create a Service Client (C++)

1. Create the service client source file:
   ```bash
   touch src/service_client.cpp
   ```

2. Add the following content:
   ```cpp
   #include "example_interfaces/srv/add_two_ints.hpp"
   #include "rclcpp/rclcpp.hpp"

   #include <chrono>
   #include <cstdlib>
   #include <memory>

   int main(int argc, char ** argv)
   {
     rclcpp::init(argc, argv);

     if (argc != 3) {
       RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_two_ints_client X Y");
       return 1;
     }

     std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_client");
     auto client = node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for service add_two_ints...");
     while (!client->wait_for_service(std::chrono::seconds(1))) {
       if (!rclcpp::ok()) {
         RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
         return 0;
       }
       RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
     }

     auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
     request->a = atoll(argv[1]);
     request->b = atoll(argv[2]);

     auto result = client->async_send_request(request);
     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending request: %ld + %ld", request->a, request->b);

     // Wait for the result
     if (rclcpp::spin_until_future_complete(node, result) ==
       rclcpp::FutureReturnCode::SUCCESS)
     {
       RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Result of add_two_ints: %ld",
                 result.get()->sum);
     } else {
       RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
     }

     rclcpp::shutdown();
     return 0;
   }
   ```

### Step 4: Update CMakeLists.txt for Services

1. Add the new executables to CMakeLists.txt:
   ```cmake
   add_executable(service_server src/service_server.cpp)
   ament_target_dependencies(service_server rclcpp example_interfaces)

   add_executable(service_client src/service_client.cpp)
   ament_target_dependencies(service_client rclcpp example_interfaces)

   install(TARGETS
     publisher_member_function
     subscriber_member_function
     qos_publisher
     service_server
     service_client
     DESTINATION lib/${PROJECT_NAME}
   )
   ```

### Step 5: Create an Action Server (Python)

1. Create the action server Python file:
   ```bash
   touch ros2_publisher_subscriber/publisher_subscriber_py/action_server.py
   ```

2. Add the following content for a Fibonacci action server:
   ```python
   import time

   import rclpy
   from rclpy.action import ActionServer
   from rclpy.callback_groups import ReentrantCallbackGroup
   from rclpy.node import Node

   from example_interfaces.action import Fibonacci


   class FibonacciActionServer(Node):

       def __init__(self):
           super().__init__('fibonacci_action_server')
           self._action_server = ActionServer(
               self,
               Fibonacci,
               'fibonacci',
               self.execute_callback,
               callback_group=ReentrantCallbackGroup())

       def execute_callback(self, goal_handle):
           self.get_logger().info('Executing goal...')

           # Send feedback to the client
           feedback_msg = Fibonacci.Feedback()
           feedback_msg.sequence = [0, 1]

           for i in range(1, goal_handle.request.order):
               # Check if there is a cancel request
               if goal_handle.is_cancel_requested:
                   goal_handle.canceled()
                   self.get_logger().info('Goal canceled')
                   return Fibonacci.Result()

               # Update feedback
               feedback_msg.sequence.append(
                   feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
               goal_handle.publish_feedback(feedback_msg)

               # Sleep to simulate work
               time.sleep(1)

           # Check if goal was canceled
           if goal_handle.is_cancel_requested:
               goal_handle.canceled()
               self.get_logger().info('Goal canceled')
               return Fibonacci.Result()

           # Complete the goal
           goal_handle.succeed()
           result = Fibonacci.Result()
           result.sequence = feedback_msg.sequence
           self.get_logger().info('Returning result: {0}'.format(result.sequence))

           return result


   def main(args=None):
       rclcpp.init(args=args)

       fibonacci_action_server = FibonacciActionServer()

       try:
           rclcpp.spin(fibonacci_action_server)
       except KeyboardInterrupt:
           pass
       finally:
           fibonacci_action_server.destroy_node()
           rclcpp.shutdown()


   if __name__ == '__main__':
       main()
   ```

### Step 6: Create an Action Client (Python)

1. Create the action client Python file:
   ```bash
   touch ros2_publisher_subscriber/publisher_subscriber_py/action_client.py
   ```

2. Add the following content:
   ```python
   import sys
   import time

   import rclpy
   from rclpy.action import ActionClient
   from rclpy.node import Node

   from example_interfaces.action import Fibonacci


   class FibonacciActionClient(Node):

       def __init__(self):
           super().__init__('fibonacci_action_client')
           self._action_client = ActionClient(
               self,
               Fibonacci,
               'fibonacci')

       def send_goal(self, order):
           goal_msg = Fibonacci.Goal()
           goal_msg.order = order

           self.get_logger().info('Waiting for action server...')
           self._action_client.wait_for_server()

           self.get_logger().info('Sending goal request...')

           send_goal_future = self._action_client.send_goal_async(
               goal_msg,
               feedback_callback=self.feedback_callback)

           send_goal_future.add_done_callback(self.goal_response_callback)

           return send_goal_future

       def goal_response_callback(self, future):
           goal_handle = future.result()
           if not goal_handle.accepted:
               self.get_logger().info('Goal rejected :(')
               return

           self.get_logger().info('Goal accepted :)')

           get_result_future = goal_handle.get_result_async()
           get_result_future.add_done_callback(self.get_result_callback)

       def get_result_callback(self, future):
           result = future.result().result
           self.get_logger().info('Result: {0}'.format(result.sequence))
           rclpy.shutdown()

       def feedback_callback(self, feedback_msg):
           feedback = feedback_msg.feedback
           self.get_logger().info(
               'Received feedback: {0}'.format(feedback.sequence))


   def main(args=None):
       rclpy.init(args=args)

       action_client = FibonacciActionClient()

       future = action_client.send_goal(int(sys.argv[1]) if len(sys.argv) > 1 else 10)

       # Wait for the server to respond
       rclpy.spin(action_client)


   if __name__ == '__main__':
       main()
   ```

### Step 7: Update package.xml for Python Entry Points

1. Add Python entry points to package.xml:
   ```xml
   <exec_depend>rclpy</exec_depend>

   <export>
     <build_type>ament_cmake</build_type>
   </export>

   <member_of_group>rosidl_interface_packages</member_of_group>
   ```

2. Also add the Python entry points section:
   ```xml
   <export>
     <build_type>ament_cmake</build_type>
     <ros_python_entry_points>
       <entry_point>fibonacci_action_server = publisher_subscriber_py.action_server:main</entry_point>
       <entry_point>fibonacci_action_client = publisher_subscriber_py.action_client:main</entry_point>
     </ros_python_entry_points>
   </export>
   ```

### Step 8: Build and Test Services

1. Navigate to workspace root:
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

4. Terminal 1 - Run the service server:
   ```bash
   ros2 run ros2_publisher_subscriber service_server
   ```

5. Terminal 2 - Run the service client:
   ```bash
   ros2 run ros2_publisher_subscriber service_client 2 3
   ```

### Step 9: Test Actions

1. Terminal 1 - Run the action server:
   ```bash
   ros2 run ros2_publisher_subscriber fibonacci_action_server
   ```

2. Terminal 2 - Run the action client:
   ```bash
   ros2 run ros2_publisher_subscriber fibonacci_action_client 5
   ```

### Step 10: Explore Built-in Tools

1. List available services:
   ```bash
   ros2 service list
   ```

2. Get information about a service:
   ```bash
   ros2 service info /add_two_ints
   ```

3. Call a service directly:
   ```bash
   ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"
   ```

4. List available actions:
   ```bash
   ros2 action list
   ```

5. Get information about an action:
   ```bash
   ros2 action info /fibonacci
   ```

## Communication Pattern Comparison

| Pattern | Type | Use Case | Example |
|---------|------|----------|---------|
| Topics | Asynchronous | Continuous data flow | Sensor data, robot pose |
| Services | Synchronous | Request/Response | Map saving, transform lookup |
| Actions | Asynchronous with feedback | Long-running tasks | Navigation, trajectory execution |

## Troubleshooting

### Common Issues and Solutions

1. **Service not found**: Ensure the service server is running before the client calls it

2. **Action client timeout**: Increase timeout values or ensure action server is running

3. **Python import errors**: Check that Python modules are properly declared in package.xml

4. **Callback group conflicts**: Use ReentrantCallbackGroup when nodes handle multiple callbacks

## Assessment Questions

1. What is the main difference between a service and an action in ROS 2?

2. When would you use a service instead of a topic for communication?

3. What are the three states an action goal can be in?

## Summary

In this lab, you implemented both services for synchronous communication and actions for long-running tasks with feedback. You learned when to use each communication pattern based on your application's requirements. Services are ideal for simple request/response interactions, while actions are perfect for complex tasks that may take time and need to provide ongoing feedback.

The next lab will focus on custom messages and launch files.