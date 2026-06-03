# robot_base

This package is the first non-test base bridge for the robot.

It keeps the known-good encoder feedback format from Tests 08/11, but replaces the old `f`, `b`, `l`, `r` serial commands with signed PWM commands:

```text
M <left_pwm> <right_pwm>
```

The node subscribes to `/cmd_vel`, converts linear/angular velocity into left/right wheel PWM, sends commands repeatedly to the ESP32, and publishes:

```text
/left_encoder_ticks
/right_encoder_ticks
```

First floor-safe run:

```bash
ros2 run robot_base base_serial_bridge --ros-args \
  -p max_pwm:=60 \
  -p max_wheel_speed_mps:=0.25 \
  -p command_timeout_s:=0.25
```

Then send a very small command:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.03}, angular: {z: 0.0}}" --once
```

Upload `firmware/robot_base_esp32/robot_base_esp32.ino` to the ESP32 before using this package.
