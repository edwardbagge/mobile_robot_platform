# robot_bringup

Launches the current floor-safe base controller and wheel odometry together.

Build:

```bash
cd ~/Documents/mobile_robot_platform/ros2_ws
colcon build --packages-select robot_base robot_bringup test_12
source install/setup.bash
```

Run:

```bash
ros2 launch robot_bringup base_odom.launch.py
```

Current floor-safe defaults:

```text
max_pwm = 100
min_pwm = 85
command_timeout_s = 0.15
ticks_per_revolution = 2800
wheel_radius_m = 0.041
```

Override an argument:

```bash
ros2 launch robot_bringup base_odom.launch.py command_timeout_s:=0.25
```
