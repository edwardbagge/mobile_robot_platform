# ROS 2 Experiments

This folder contains historical ROS 2 milestone packages from the robot bring-up process. They remain under `ros2_ws/src` so `colcon` can discover and build them, but they are not the current production robot stack.

- `test_09`: robot command serial bridge
- `test_10`: `/cmd_vel` serial bridge
- `test_11`: `/cmd_vel` bridge with encoder feedback
- `test_12`: wheel odometry from encoder ticks
- `test_13`: RPLIDAR bring-up and RViz verification
