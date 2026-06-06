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
