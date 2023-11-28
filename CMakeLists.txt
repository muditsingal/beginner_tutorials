cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/StringService.srv"
)

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(
    talker
    rclcpp
    std_msgs
    tf2
    tf2_ros
    geometry_msgs
)

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

add_executable(server src/change_string_id_server.cpp)
ament_target_dependencies(server rclcpp std_msgs)

install(TARGETS
  talker
  listener
  server
  DESTINATION lib/${PROJECT_NAME})

rosidl_get_typesupport_target(cpp_typesupport_target
${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(server "${cpp_typesupport_target}")


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  set(TEST_FIXTURES test/test_tf2_broadcaster.cpp)

  ament_add_gtest(${PROJECT_NAME}_test ${TEST_FIXTURES})
  ament_target_dependencies(${PROJECT_NAME}_test rclcpp std_msgs geometry_msgs tf2 tf2_ros)

  install(TARGETS
    ${PROJECT_NAME}_test
    DESTINATION lib/${PROJECT_NAME}
  )

  rosidl_get_typesupport_target(${PROJECT_NAME}_test ${PROJECT_NAME}
    "rosidl_typesupport_cpp"
  )

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
