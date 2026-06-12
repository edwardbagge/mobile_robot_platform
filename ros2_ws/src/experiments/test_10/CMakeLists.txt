cmake_minimum_required(VERSION 3.8)
project(test_10)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(geometry_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(cmd_vel_serial_bridge_test src/cmd_vel_serial_bridge_test.cpp)
ament_target_dependencies(cmd_vel_serial_bridge_test rclcpp geometry_msgs)

install(TARGETS
  cmd_vel_serial_bridge_test
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
