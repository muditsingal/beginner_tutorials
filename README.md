# beginner_tutorials
ENPM808x ROS2 related assignment repo

## Overview
This repo contains the talker and listener nodes in ROS2 made with C++.
The *docs* folder contains results for cpp lint and cpp check.
The source files are: _publisher_member_function.cpp_ and _subscriber_member_function.cpp_

## Steps to run

1. Clone the repo using `git clone https://github.com/muditsingal/beginner_tutorials.git` inside your _ros2\_ws/src_ folder
2. Build your workspace using `colcon build`
3. Open 2 terminals
4. In first terminal start the talker by running the command: `ros2 run beginner_tutorials talker`
5. In second terminal start the subscriber by running the command: `ros2 run beginner_tutorials listener`
6. You should see messages being published and subscribed in respective terminals

## Dependencies
> ROS2 Humble
> rclcpp
> cpplint
> cppcheck
> clang-format
> colcon

