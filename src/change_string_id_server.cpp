/**
 * @file change_string_id_server.cpp
 * @author Mudit Singal (muditsingal@gmail.com)
 * @brief Server cpp file to enable the string change service
 * @version 0.1
 * @date 2023-11-16
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>
#include <fstream>
#include "beginner_tutorials/srv/string_service.hpp"

void handle_service(const std::shared_ptr<beginner_tutorials::srv::StringService::Request> request,
                     std::shared_ptr<beginner_tutorials::srv::StringService::Response> response)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The entered string and id are: %s  %d", request->msg_string, request->service_id);

    if((request->msg_string).empty())
    {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "Empty string entered, default string won't be changed!");
      response->status = false;
      return;
    }

    const std::string pkg_path = "src/beginner_tutorials";
    // std::string pkg_path = ament_index_cpp::get_package_share_directory("beginner_tutorials");
    // std::cout<<pkg_path<<std::endl;


    std::ofstream out_file(pkg_path + "/data/print_string.txt", std::ios::trunc);

    if (!out_file.is_open()) {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("rclcpp"), "Error opening the file!");
        return;
    }

    // Write to the file
    out_file << request->msg_string << std::endl;

    // Close the file
    out_file.close();


    std::cout << request->msg_string << std::endl;



    response->status = true;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_DEBUG_ONCE(rclcpp::get_logger("rclcpp"), "String change service started!");
    auto node = rclcpp::Node::make_shared("string_change_server");
    auto server = node->create_service<beginner_tutorials::srv::StringService>("service_change_string_cntr", &handle_service);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Server for changing talker string and id ready!");

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
