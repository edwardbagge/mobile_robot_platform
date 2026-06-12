# Hardware Bring-Up Tests

This folder contains manual hardware tests that run outside ROS 2. Use these tests to validate wiring, motor direction, encoder behavior, serial communication, and Raspberry Pi to ESP32 integration before relying on the ROS 2 stack.

ROS 2 milestone packages are kept under `ros2_ws/src/experiments/test_09` to `ros2_ws/src/experiments/test_13` so `colcon` can discover and build them. New automated ROS 2 tests for current packages should live inside the relevant ROS package, for example `ros2_ws/src/robot_base/test/`.

## Test Sequence

1. `01_encoder_diagnostic`
   - ESP32 encoder diagnostic for checking basic encoder signal behavior.

2. `02_motor_direction_test`
   - ESP32 motor direction test for confirming motor wiring and driver polarity.

3. `03_motor_encoder_combined_test`
   - ESP32 combined motor and encoder test for checking that commanded motion produces matching encoder feedback.

4. `04_quadrature_direction_test`
   - ESP32 quadrature direction test for validating encoder channel order and sign.

5. `05_speed_estimation_test`
   - ESP32 speed estimation test for checking tick-to-speed calculations.

6. `06_rpi_serial_command_test`
   - ESP32 serial command test for Raspberry Pi to ESP32 communication.

7. `07_rpi_motor_command_test`
   - ESP32 motor command test driven from the Raspberry Pi command path.

8. `08_rpi_motor_encoder_feedback_test`
   - ESP32 and Raspberry Pi test for motor commands plus encoder feedback over serial.

## Hardware Requirements

- Tests `01` to `05` are ESP32-side hardware tests.
- Tests `06` to `08` require the Raspberry Pi serial connection to the ESP32.
- Keep the robot lifted or otherwise safe during motor tests.

## Scope

Use this folder for manual low-level hardware bring-up tests. Use `ros2_ws/` for ROS 2 packages, ROS launch files, SLAM mapping, and ROS package tests.
