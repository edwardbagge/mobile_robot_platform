# ROS 2 Workspace Notes

This workspace contains the current ROS 2 software stack for the mobile robot platform. It includes the base controller nodes, the lidar bring-up, and the SLAM mapping workflow used to test autonomous mapping on the hardware. The stack is intended for ROS 2 Jazzy and assumes the robot is connected through the ESP32-based base controller and an RPLIDAR sensor.

At runtime, the system follows a simple information flow. A motion command is sent on /cmd_vel, the wheel velocity controller converts that command into wheel-level PWM values, the serial bridge forwards those values to the ESP32 firmware, and the firmware reports encoder feedback back to ROS 2 through the left and right encoder topics. The resulting motion estimate is published as /odom and as TF for use by mapping and navigation tools.

## Core packages

- `robot_base`: the main motion-control stack for the robot base.
- `robot_bringup`: launch files and configuration for the base, lidar, mapping, and navigation workflow.
- `rplidar_ros`: the ROS driver for the RPLIDAR sensor.
- `learning` and `experiments`: small test and learning packages used during development.

## Current base bring-up

The simplest way to start the base stack is to launch the base bring-up file, which starts the serial bridge, wheel velocity controller, and odometry node together.

Build the base package:

```bash
cd ~/Documents/mobile_robot_platform/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_base robot_bringup
source install/setup.bash
```

Launch the base stack:

```bash
ros2 launch robot_bringup base_odom.launch.py
```

This launch starts:
- `base_serial_bridge`
- `wheel_velocity_controller`
- `wheel_odometry`

Default parameters come from [src/robot_bringup/config/floor_safe_params.yaml](src/robot_bringup/config/floor_safe_params.yaml).

## SLAM mapping runbook

The bring-up stack provides the core data needed by SLAM mapping:

- `/cmd_vel` for velocity commands.
- `/odom` and `odom -> base_link` TF from wheel odometry.
- `/scan` from the RPLIDAR.
- `base_link -> laser` static TF from `robot_bringup`.

Install the required ROS packages once before using the mapping workflow:

```bash
sudo apt update
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-nav2-map-server ros-jazzy-rviz2 ros-jazzy-teleop-twist-keyboard
sudo usermod -aG dialout $USER
```

Log out and back in after changing groups. For a clean start, use `mapping.launch.py` directly rather than launching the lower-level files separately.

Terminal 1: build and start mapping:

```bash
cd ~/Documents/mobile_robot_platform/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_base robot_bringup rplidar_ros
source install/setup.bash
mkdir -p maps
ls -l /dev/robot_base /dev/rplidar
ros2 launch robot_bringup mapping.launch.py
```

The robot uses stable device aliases for the serial connections:

```bash
/dev/robot_base
/dev/rplidar
```

These should be used consistently in the launch configuration. If either alias is missing, fix the hardware device setup before launching the stack.

Terminal 2: optional basic checks:

```bash
cd ~/Documents/mobile_robot_platform/ros2_ws
source install/setup.bash

ros2 topic echo /scan --once
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser
```

Terminal 3: drive the robot slowly while mapping:

```bash
cd ~/Documents/mobile_robot_platform/ros2_ws
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Terminal 4: save the generated map:

```bash
cd ~/Documents/mobile_robot_platform/ros2_ws
source install/setup.bash
mkdir -p maps
ros2 run nav2_map_server map_saver_cli -f "$PWD/maps/floor1" --ros-args -p map_subscribe_transient_local:=true -p save_map_timeout:=30.0
```

Check that the map files were written:

```bash
cd ~/Documents/mobile_robot_platform/ros2_ws
ls -l maps
```

## Nav2 notes

Nav2 is separate from the SLAM mapping workflow. Install the Nav2 packages when you are ready to test navigation:

```bash
sudo apt update
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
```

For a first Nav2 test while SLAM is still running:

```bash
ros2 launch robot_bringup nav2_navigation.launch.py
```

For Nav2 localization and navigation from a saved map:

```bash
ros2 launch robot_bringup nav2_map.launch.py map:=$HOME/Documents/mobile_robot_platform/ros2_ws/maps/floor1.yaml
```

Before relying on Nav2, add a robot-specific `nav2_params.yaml` for this platform. Tune robot radius or footprint, max velocity around `0.08-0.10 m/s`, acceleration limits, `/scan`, `/odom`, `base_link`, and costmap inflation. The current bring-up config publishes wheel odometry at `20.0 Hz` for Nav2.

## Base topic flow

The current command and feedback path is:

`/cmd_vel` -> `wheel_velocity_controller` -> `/wheel_pwm_cmd` -> `base_serial_bridge` -> ESP32

Encoder feedback returns as:

ESP32 -> `base_serial_bridge` -> `/left_encoder_ticks`, `/right_encoder_ticks`

Odometry output:

- `/odom`
- `odom -> base_link` TF by default

## Safety notes

The base stack includes two practical protections against bad encoder data:

- the controller stops publishing drive PWM if encoder feedback becomes stale,
- and both the controller and odometry logic reject implausible encoder jumps.

These protections are controlled mainly by the timeout and speed-related parameters in [src/robot_bringup/config/floor_safe_params.yaml](src/robot_bringup/config/floor_safe_params.yaml).

## Main files

- Base launch: [src/robot_bringup/launch/base_odom.launch.py](src/robot_bringup/launch/base_odom.launch.py)
- Lidar launch: [src/robot_bringup/launch/lidar.launch.py](src/robot_bringup/launch/lidar.launch.py)
- Robot launch: [src/robot_bringup/launch/robot.launch.py](src/robot_bringup/launch/robot.launch.py)
- SLAM launch: [src/robot_bringup/launch/slam.launch.py](src/robot_bringup/launch/slam.launch.py)
- Mapping launch: [src/robot_bringup/launch/mapping.launch.py](src/robot_bringup/launch/mapping.launch.py)
- Base and lidar params: [src/robot_bringup/config/floor_safe_params.yaml](src/robot_bringup/config/floor_safe_params.yaml)
- SLAM params: [src/robot_bringup/config/slam_params.yaml](src/robot_bringup/config/slam_params.yaml)
- Serial bridge: [src/robot_base/src/base_serial_bridge.cpp](src/robot_base/src/base_serial_bridge.cpp)
- Velocity controller: [src/robot_base/src/wheel_velocity_controller.cpp](src/robot_base/src/wheel_velocity_controller.cpp)
- Odometry: [src/robot_base/src/wheel_odometry.cpp](src/robot_base/src/wheel_odometry.cpp)