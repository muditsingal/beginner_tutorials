// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


/**
 * @file publisher_member_function.cpp
 * @author Mudit Singal (muditsingal@gmail.com)
 * @brief File to create a publisher (talker) of a string
 * @version 0.1
 * @date 2023-11-20
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

 private:
  void timer_callback() {
    const std::string pkg_path = "src/beginner_tutorials";
    // std::string pkg_path = ament_index_cpp::get_package_share_directory("beginner_tutorials");
    // std::cout<<pkg_path<<std::endl;


    std::ifstream in_file(pkg_path + "/data/print_string.txt");
    std::string file_contents = "Default contents, file not read ";

    if (in_file.is_open()) {
        std::string read_file_string((std::istreambuf_iterator<char>(in_file)),
                                  std::istreambuf_iterator<char>());

        // Close the file
        file_contents = read_file_string;
        in_file.close();
    }
    else {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("rclcpp"), "Error opening the file!");
        return;
    }
    auto message = std_msgs::msg::String();
    std::string file_txt;
    message.data = "Now printing from file contents: " + file_contents +
                   std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    RCLCPP_WARN(this->get_logger(), "Dummy Warning from publisher");
    RCLCPP_FATAL(this->get_logger(), "Dummy FATAL message from publisher");
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
