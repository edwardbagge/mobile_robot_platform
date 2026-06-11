# ROS 2 Workspace Notes

This workspace contains the current robot base stack, lidar bring-up, and the earlier incremental test packages used to reach the present setup.

## Current packages

`robot_base`
Differential-drive base nodes:
- `base_serial_bridge`: sends signed PWM commands to the ESP32 and republishes encoder ticks.
- `wheel_velocity_controller`: converts `/cmd_vel` into wheel PWM using encoder feedback.
- `wheel_odometry`: integrates encoder ticks into `/odom` and optional `odom -> base_link` TF.

`robot_bringup`
Launch and parameter files for the current base stack.

`rplidar_ros`
RPLIDAR driver package.

`simple_publisher`, `simple_subscriber`
Small ROS 2 learning / smoke-test packages.

`test_09` to `test_13`
Historical milestone packages kept for reference.

## Current base bring-up

Build the base package:

```bash
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

Default parameters come from [src/robot_bringup/config/floor_safe_params.yaml](/home/edward/Documents/mobile_robot_platform/ros2_ws/src/robot_bringup/config/floor_safe_params.yaml:1).

## SLAM and Nav2 runbook

The bring-up stack provides the core data needed by SLAM and navigation:

- `/cmd_vel` for velocity commands.
- `/odom` and `odom -> base_link` TF from wheel odometry.
- `/scan` from the RPLIDAR.
- `base_link -> laser` static TF from `robot_bringup`.

Install the missing ROS packages once:

```bash
sudo apt update
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-rviz2 ros-jazzy-teleop-twist-keyboard
sudo usermod -aG dialout $USER
```

Log out and back in after changing groups.

Build and launch the full robot stack in terminal 1:

```bash
cd ~/Documents/mobile_robot_platform/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_base robot_bringup rplidar_ros
source install/setup.bash

ros2 launch robot_bringup robot.launch.py
```

If `/dev/robot_base` or `/dev/rplidar` do not exist, pass the real serial devices:

```bash
ros2 launch robot_bringup robot.launch.py base_serial_port:=/dev/ttyACM0 lidar_serial_port:=/dev/ttyUSB0
```

Run basic checks in terminal 2:

```bash
source ~/Documents/mobile_robot_platform/ros2_ws/install/setup.bash

ros2 topic echo /scan --once
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser
```

Start online SLAM in terminal 3:

```bash
cd ~/Documents/mobile_robot_platform/ros2_ws
source install/setup.bash

ros2 launch slam_toolbox online_async_launch.py slam_params_file:=$PWD/src/robot_bringup/config/slam_params.yaml use_sim_time:=false
```

Drive the robot slowly while mapping:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Save the generated map:

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/floor1
```

For a first Nav2 test while SLAM is still running:

```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false
```

For Nav2 localization and navigation from a saved map:

```bash
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=false map:=$HOME/maps/floor1.yaml
```

Before relying on Nav2, add a robot-specific `nav2_params.yaml` for this platform. Tune robot radius or footprint, max velocity around `0.08-0.10 m/s`, acceleration limits, `/scan`, `/odom`, `base_link`, and costmap inflation. The current `wheel_odometry.publish_rate_hz` is `2.0`; Nav2 should be tested with about `20.0 Hz`.

## Base topic flow

The current command and feedback path is:

`/cmd_vel` -> `wheel_velocity_controller` -> `/wheel_pwm_cmd` -> `base_serial_bridge` -> ESP32

Encoder feedback returns as:

ESP32 -> `base_serial_bridge` -> `/left_encoder_ticks`, `/right_encoder_ticks`

Odometry output:

- `/odom`
- optional `odom -> base_link` TF

## Current safety behavior

The base stack now includes two protections against bad encoder data:

- `wheel_velocity_controller` stops publishing drive PWM if encoder feedback goes stale for longer than `encoder_timeout_s`.
- `wheel_velocity_controller` and `wheel_odometry` ignore implausible encoder tick jumps, which helps avoid bad control effort or fake odometry jumps after an ESP32 reboot or counter reset.

The main related parameters are:

- `command_timeout_s`
- `encoder_timeout_s`
- `max_wheel_speed_mps`
- `max_tick_jump_scale`
- `ticks_per_revolution`
- `wheel_radius_m`
- `wheel_base_m`

## Main files

- Launch: [src/robot_bringup/launch/base_odom.launch.py](/home/edward/Documents/mobile_robot_platform/ros2_ws/src/robot_bringup/launch/base_odom.launch.py:1)
- Base params: [src/robot_bringup/config/floor_safe_params.yaml](/home/edward/Documents/mobile_robot_platform/ros2_ws/src/robot_bringup/config/floor_safe_params.yaml:1)
- Serial bridge: [src/robot_base/src/base_serial_bridge.cpp](/home/edward/Documents/mobile_robot_platform/ros2_ws/src/robot_base/src/base_serial_bridge.cpp:1)
- Velocity controller: [src/robot_base/src/wheel_velocity_controller.cpp](/home/edward/Documents/mobile_robot_platform/ros2_ws/src/robot_base/src/wheel_velocity_controller.cpp:1)
- Odometry: [src/robot_base/src/wheel_odometry.cpp](/home/edward/Documents/mobile_robot_platform/ros2_ws/src/robot_base/src/wheel_odometry.cpp:1)

## Test progression

`test_09`
Robot command serial bridge from ROS 2 to the ESP32.

`test_10`
`/cmd_vel` to serial command bridge.

`test_11`
`/cmd_vel` control plus encoder feedback publishing.

`test_12`
Wheel odometry generation from encoder ticks.

`test_13`
Slamtec RPLIDAR bring-up, `/scan` publication, and RViz verification.
