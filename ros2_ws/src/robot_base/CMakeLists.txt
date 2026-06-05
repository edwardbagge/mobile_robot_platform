cmake_minimum_required(VERSION 3.8)
project(robot_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(base_serial_bridge src/base_serial_bridge.cpp)
ament_target_dependencies(base_serial_bridge geometry_msgs rclcpp std_msgs)

add_executable(wheel_odometry src/wheel_odometry.cpp)
ament_target_dependencies(
  wheel_odometry
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
  tf2
  tf2_ros
)

add_executable(wheel_velocity_controller src/wheel_velocity_controller.cpp)
ament_target_dependencies(
  wheel_velocity_controller
  geometry_msgs
  rclcpp
  std_msgs
)

install(TARGETS
  base_serial_bridge
  wheel_odometry
  wheel_velocity_controller
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
