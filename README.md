# Mobile Robot Platform

This repository contains the hardware files, ESP32 firmware, low-level hardware tests, and ROS 2 workspace for a small differential-drive mobile robot.

## Repository Layout

- [firmware/](firmware): ESP32 firmware for the robot base.
- [hardware/](hardware): CAD and manufacturing files for the chassis, motor holders, lidar platform, and wheel hub adapters.
- [testing/](testing): manual non-ROS hardware bring-up tests for motors, encoders, serial communication, and Raspberry Pi integration.
- [ros2_ws/](ros2_ws): ROS 2 workspace with the current base, lidar, SLAM mapping, and navigation bring-up packages.

## Main Documentation

- [Hardware notes](hardware/README.md)
- [Hardware test notes](testing/README.md)
- [ROS 2 workspace notes](ros2_ws/README.md)

## Current ROS 2 Entry Points

- Base only: `ros2 launch robot_bringup base_odom.launch.py`
- Base, lidar, and SLAM mapping: `ros2 launch robot_bringup mapping.launch.py`

See [ros2_ws/README.md](ros2_ws/README.md) for build, launch, mapping, and map-saving commands.
