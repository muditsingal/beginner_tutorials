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
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

 private:
  void timer_callback() {
    const std::string pkg_path = "src/beginner_tutorials";

    std::ifstream in_file(pkg_path + "/data/print_string.txt");
    std::string file_contents = "Default contents, file not read ";
    geometry_msgs::msg::TransformStamped t;

    if (in_file.is_open()) {
        std::string read_file_string((std::istreambuf_iterator<char>
                          (in_file)), std::istreambuf_iterator<char>());

        // Close the file
        file_contents = read_file_string;
        in_file.close();
    } else {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("rclcpp"),
                                                    "Error opening the file!");
        return;
    }
    auto message = std_msgs::msg::String();
    message.data = "Now printing from file contents: " + file_contents +
                   std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    RCLCPP_WARN_STREAM(this->get_logger(), "Dummy Warning from publisher");
    RCLCPP_FATAL_STREAM(this->get_logger(), "Dummy FATAL msg from publisher");

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "talk";

    t.transform.translation.x = 1.5;
    t.transform.translation.y = 0.6;
    t.transform.translation.z = 0.0;


    tf2::Quaternion q;
    q.setRPY(0, 0, 1.57);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // Send the transformation
    tf_broadcaster_->sendTransform(t);

    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  size_t count_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
