cmake_minimum_required(VERSION 3.15)
project(ros2_computer_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)

find_package(diagnostic_updater REQUIRED)

find_package(diagnostic_msgs REQUIRED)

add_library(
    ros2_computer_monitor
    SHARED
)
target_sources(
    ros2_computer_monitor
    PRIVATE
        src/cpu_monitor_node.cpp
)
target_compile_features(
    ros2_computer_monitor
    PUBLIC
        cxx_std_17
)
ament_target_dependencies(
    ros2_computer_monitor
        rclcpp
        rclcpp_components
        rcutils
        diagnostic_updater
        diagnostic_msgs
)
rclcpp_components_register_node(
    ros2_computer_monitor
    PLUGIN
        "ros2_computer_monitor::CPUMonitorNode"
    EXECUTABLE
        cpu_monitor_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
    TARGETS
        ros2_computer_monitor
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

ament_package()
