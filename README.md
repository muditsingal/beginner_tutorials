# beginner_tutorials
ENPM808x ROS2 related assignment repo

## Overview
This repo contains the talker and listener nodes and string change service in ROS2 made with C++. <br>
The source files are: change_string_id_server.cpp _publisher_member_function.cpp_ and _subscriber_member_function.cpp_ <br>
The string to be printed is present in data/print_string.txt. <br>
Launch file to launch the service, talker and listener is presetnt in launch folder: talker_listen_srv_launch.py. <br>
RQT console, cpp lint, and cpp check output is present in *results* folder.


## Steps to run

1. Clone the repo using `git clone https://github.com/muditsingal/beginner_tutorials.git` inside your _ros2\_ws/src_ folder
2. Build your workspace using `colcon build` from the ros2_ws folder
3. Open 2 terminals
4. In first terminal start the talker, listener, and service to change string by running the command: `ros2 launch beginner_tutorials talker_listen_srv_launch.py`
5. Observe that INFO messages are printed indicating successful publishing and subscribing of the string present in data/print_string.txt
6. You should also get logging for Warning, Error, and DEBUG severities in the package.

## Steps to record rosbag file

1. Make sure that this package is added to your workspace src folder and built successfully.
2. Launch the relevant nodes for talker, listener, and rosbag recorder using: `ros2 launch beginner_tutorials bringup_and_record_launch.py`.
3. Terminate and the recording using *Ctrl+C* keypress.
4. Inspect the bag file using `ros2 bag info rosbag2_<tiemstamp>/`<br>

- To disable rosbag recording: Set *rosbag_record* argument to *False* in *bringup_and_record_launch.py*.
- To record TF frames: `ros2 run tf2_tools view_frames`
- To inspect specific frames: `ros2 run tf2_ros tf2_echo world talk`

## Steps to run ROSBAG

1. Make sure that this package is added to your workspace src folder and built successfully.
2. Source the workspace: `source install/setup.bash`
3. ros2 run beginner_tutorials listener
4. ros2 bag play <path/to/rosbag_folder>
5. Observe the listener.

## Steps to run ROS test

1. `cd <path/to/ros/workspace>`
2. `source install/setup.bash`
3. ros2 run beginner_tutorials beginner_tutorials_test

## Steps to get fatal error logging

1. Remove the file print_string.txt from *data* folder
2. Again launch all the necessary nodes from the launch file by using: `ros2 launch beginner_tutorials talker_listen_srv_launch.py` from the ros2_ws folder.
3. You should get a FATAL error from ROS.

## Steps to call the service to change the underlying published string

1. Make sure a text file: print_string.txt is present in the data folder of the package.
2. Launch all the necessary nodes from the launch file by using: `ros2 launch beginner_tutorials talker_listen_srv_launch.py` from the ros2_ws folder.
3. From another terminal run `source ~/ros2_ws/install/setup.bash`.
4. Call the service using the command: `ros2 service call /beginner_tutorials_ns/service_change_string_cntr beginner_tutorials/srv/StringService "{msg_string: '808X assignment 2 service', service_id: 1}"`
5. In the first terminal the string being published and subscribed will be modified.

## CppLint, CppCheck, and Clangd formattting

```bash
# Cpp Lint
cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order --filter="-legal/copyright" $( find . -name *.cpp | grep -vE -e "^./build/" ) &> results/cpplint_output_a3.txt

# Cpp Check
cppcheck --enable=all --std=c++17 -I include/ --suppress=missingInclude $( find . -name *.cpp | grep -vE -e "^./build/" ) &> results/cppcheck_output_a3.txt

# Clangd format
clang-format -i --style=Google $(find . -name *.cpp -o -name *.hpp | grep -vE -e "^./build/")
```


## Dependencies
> ROS2 Humble <br>
> rclcpp <br>
> cpplint <br>
> cppcheck <br>
> clang-format <br>
> colcon <br>
