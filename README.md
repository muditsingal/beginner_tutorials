# beginner_tutorials
ENPM808x ROS2 related assignment repo

## Overview
This repo contains the talker and listener nodes and string change service in ROS2 made with C++. <br>
The *docs* folder contains results for cpp lint and cpp check. <br>
The source files are: change_string_id_server.cpp _publisher_member_function.cpp_ and _subscriber_member_function.cpp_ <br>
The string to be printed is present in data/print_string.txt. <br>
Launch file to launch the service, talker and listener is presetnt in launch folder: talker_listen_srv_launch.py.

## Steps to run

1. Clone the repo using `git clone https://github.com/muditsingal/beginner_tutorials.git` inside your _ros2\_ws/src_ folder
2. Build your workspace using `colcon build` from the ros2_ws folder
3. Open 2 terminals
4. In first terminal start the talker, listener, and service to change string by running the command: `ros2 launch beginner_tutorials talker_listen_srv_launch.py`
5. Observe that INFO messages are printed indicating successful publishing and subscribing of the string present in data/print_string.txt
6. You should also get logging for Warning, Error, and DEBUG severities in the package.


## Steps to get fatal error logging

1. Remove the file print_string.txt from *data* folder
2. Again launch all the necessary nodes from the launch file by using: `ros2 launch beginner_tutorials talker_listen_srv_launch.py` from the ros2_ws folder.
3. You should get a FATAL error from ROS.


## Dependencies
> ROS2 Humble
> rclcpp
> cpplint
> cppcheck
> clang-format
> colcon
