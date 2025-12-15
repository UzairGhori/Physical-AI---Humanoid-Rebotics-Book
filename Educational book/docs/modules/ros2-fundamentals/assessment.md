---
sidebar_position: 10
---

# ROS 2 Fundamentals - Assessment

## Overview

This assessment tests your understanding of ROS 2 fundamentals covered in this module. Complete all exercises to demonstrate proficiency in ROS 2 concepts, communication patterns, and system management.

## Multiple Choice Questions

1. What does ROS 2 stand for?
   a) Robot Operating System 2
   b) Robotic Open Software 2
   c) Real-time Operating System 2
   d) Robot Operation Suite 2

2. Which middleware does ROS 2 use for communication?
   a) TCP/IP
   b) Data Distribution Service (DDS)
   c) HTTP
   d) MQTT

3. What is the primary difference between a ROS 2 topic and a service?
   a) Topics are faster than services
   b) Topics provide asynchronous communication while services provide synchronous communication
   c) Services can transmit data while topics cannot
   d) There is no difference between them

4. Which Quality of Service (QoS) policy determines how many messages to store?
   a) Reliability
   b) Durability
   c) History
   d) Deadline

5. What command is used to list all available ROS 2 topics?
   a) ros2 show topics
   b) ros2 list topics
   c) ros2 topic list
   d) ros2 topics

6. Which is the most recent Long-Term Support (LTS) ROS 2 distribution as of 2025?
   a) Humble Hawksbill
   b) Iron Irwini
   c) Jazzy Jalisco
   d) Rolling Ridley

7. What is the purpose of the `ros2 doctor` command?
   a) To run system health checks and diagnose common issues
   b) To install ROS 2 packages
   c) To debug individual nodes
   d) To create new ROS 2 packages

## Practical Exercises

### Exercise 1: Publisher/Subscriber Implementation (30 points)

Create a ROS 2 package with a publisher that sends messages containing your name and the current timestamp every 2 seconds. Create a corresponding subscriber that prints the received messages to the console.

**Requirements:**
- Use C++ or Python
- Create a custom message with at least 2 fields
- Include proper error handling
- Document your code

### Exercise 2: Service Implementation (25 points)

Implement a ROS 2 service that accepts two numbers and returns their sum and product. Create both the service server and client.

**Requirements:**
- Define the service interface properly
- Handle edge cases (e.g., division by zero if applicable)
- Test the service with multiple inputs
- Include logging

### Exercise 3: Launch File Configuration (20 points)

Create a launch file that starts both the publisher and subscriber from Exercise 1, with configurable parameters for the update rate and message content.

**Requirements:**
- Use launch arguments
- Include parameter file loading
- Add conditional node startup
- Document the launch file

### Exercise 4: Custom Message Creation (25 points)

Design and implement a custom message for a robot's sensor data that includes:
- Timestamp
- Sensor type (enum: camera, lidar, imu, etc.)
- Sensor readings (array of floats)
- Confidence level (0.0 to 1.0)

Use this message in a simple publisher/subscriber example.

**Requirements:**
- Proper message definition
- Correct package.xml and CMakeLists.txt configuration
- Working publisher/subscriber example
- Proper data validation

## Short Answer Questions

1. Explain the difference between a ROS 2 action and a service. When would you use each? (10 points)

2. Describe the purpose of Quality of Service (QoS) settings in ROS 2. Provide examples of when you might use different QoS profiles. (10 points)

3. What are the advantages of using launch files in ROS 2? Provide at least 3 specific benefits. (10 points)

4. Explain the publish/subscribe communication pattern and give an example of when it would be appropriate to use it in a robotic system. (10 points)

## Hands-on Challenge (40 points)

Create a complete ROS 2 system that simulates a simple robot patrol scenario:

- A patrol controller node that sends navigation goals
- A robot simulator node that receives goals and publishes position updates
- A monitoring node that subscribes to position updates and logs the robot's status
- A service that allows external systems to query the robot's current status

**Requirements:**
- Use custom messages where appropriate
- Include proper error handling
- Use a launch file to start all nodes
- Document the system architecture
- Provide a README with instructions to run the system

## Grading Rubric

- Multiple Choice Questions: 1 point each (7 total)
- Practical Exercises: As marked above (100 total)
- Short Answer Questions: 10 points each (40 total)
- Hands-on Challenge: 40 points
- **Total: 187 points**

## Submission Requirements

1. All source code must be properly formatted and commented
2. Include a README file explaining how to build and run your solutions
3. Document any assumptions made during implementation
4. Provide evidence of testing (console output, screenshots, etc.)

## Resources Allowed

- ROS 2 documentation
- Your course materials
- Official ROS 2 tutorials
- Standard development tools

## Time Limit

This assessment should take approximately 4-6 hours to complete all sections.

## Evaluation Criteria

- Correctness of implementation
- Code quality and documentation
- Understanding of ROS 2 concepts
- Proper use of ROS 2 tools and patterns
- Problem-solving approach