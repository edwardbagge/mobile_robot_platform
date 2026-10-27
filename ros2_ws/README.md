# ROS 2 Workspace Notes

## Overview

This workspace contains the ROS 2 software used for the mobile robot platform developed in this thesis. It includes the motor control and odometry nodes, LiDAR driver, launch files, and configuration files needed to operate the robot and perform SLAM mapping.

The workspace uses ROS 2 Jazzy on a Raspberry Pi 5 running Ubuntu 24.04. An ESP32 microcontroller handles motor control signals and reads the motor encoders. The Raspberry Pi communicates with the ESP32 through USB.

The main packages are:

- `robot_base`: Motor control, serial communication, and wheel odometry nodes.
- `robot_bringup`: Launch files and configuration for operating the robot and running SLAM.
- `rplidar_ros`: ROS 2 driver for the RPLIDAR sensor.
- `learning` and `experiments`: Programs used during development and testing.

## Accessing the Raspberry Pi

The ROS 2 workspace runs on the Raspberry Pi. It can be accessed directly or remotely through SSH.

### Direct connection

Connect a monitor, keyboard, and mouse to the Raspberry Pi, log in to Ubuntu, and open a terminal. Run the commands in this README in that terminal.

### Remote connection using SSH

SSH allows another computer to control the Raspberry Pi over a network. The computers must be on the same network or otherwise be able to reach each other.

If SSH is not already configured, first access the Raspberry Pi directly and install the SSH server:

```bash
sudo apt update
sudo apt install openssh-server
sudo systemctl enable --now ssh
```

Check that the service is running:

```bash
sudo systemctl status ssh
```

Find the Raspberry Pi's IP address and username:

```bash
hostname -I
whoami
```

On the other computer (for example, in macOS Terminal), connect with:

```bash
ssh USERNAME@IP_ADDRESS
```

Replace the placeholders with the Raspberry Pi username and IP address. Verify the host details before accepting an SSH host key prompt, and enter the Raspberry Pi user's password if requested.

For tasks requiring multiple terminals, open another terminal window and connect again using the same SSH command. Enter `exit` to disconnect.

**Note:** SSH provides terminal access but does not automatically display graphical applications such as RViz 2.

## ROS 2 Prerequisites

These instructions assume that Ubuntu 24.04 and ROS 2 Jazzy are already installed on the Raspberry Pi and that the workspace source files are present. They are not a complete operating-system or ROS 2 installation guide.

Install the packages used for SLAM mapping, map saving, visualization, and keyboard control:

```bash
sudo apt update
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-nav2-map-server ros-jazzy-rviz2 ros-jazzy-teleop-twist-keyboard
```

`nav2_map_server` is used here to save maps. Autonomous navigation is not covered by this README.

Give the current user access to serial devices:

```bash
sudo usermod -aG dialout $USER
```

Log out and back in for the group change to take effect.

The workspace build also requires `colcon` and the dependencies of its ROS 2 packages. If these are not already installed, they must be set up before building; their original installation steps have not been verified for this robot.

## Building the Workspace

Open a terminal on the Raspberry Pi and navigate to the workspace:

```bash
cd ~/Documents/mobile_robot_platform/ros2_ws
```

If the repository is stored elsewhere, use its actual path.

Source ROS 2 Jazzy and build the main packages:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_base robot_bringup rplidar_ros
source install/setup.bash
```

The robot expects the ESP32 and LiDAR to be available through these stable serial-device aliases:

```text
/dev/robot_base
/dev/rplidar
```

Check that both exist:

```bash
ls -l /dev/robot_base /dev/rplidar
```

If an alias is missing, check the USB connection and device configuration before starting the robot. The aliases are not created by `colcon build`.

When opening a new terminal or SSH session, source the environment again:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/Documents/mobile_robot_platform/ros2_ws
source install/setup.bash
```

Rebuild the affected packages after source or installed configuration changes when needed.

## Starting the Robot

Before starting, check that the Raspberry Pi, ESP32, motor driver, and LiDAR are connected. Keep the robot in a safe position and make sure the Raspberry Pi's power cable cannot become caught in the wheels.

### Starting the base controller

In a terminal on the Raspberry Pi, run:

```bash
cd ~/Documents/mobile_robot_platform/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_bringup base_odom.launch.py
```

This starts:

- `base_serial_bridge`: Serial communication between the Raspberry Pi and ESP32.
- `wheel_velocity_controller`: Converts movement commands into wheel motor commands.
- `wheel_odometry`: Estimates movement from encoder data.

The base parameters are defined in `src/robot_bringup/config/floor_safe_params.yaml`.

Keep the launch terminal open. Press `Ctrl+C` to stop it.

### Checking ROS 2 connections

In a second terminal, source the environment as described above and inspect the running nodes and topics:

```bash
ros2 node list
ros2 topic list
```

The main topics are:

| Topic | Purpose |
| --- | --- |
| `/cmd_vel` | Receives movement commands |
| `/wheel_pwm_cmd` | Sends motor PWM commands |
| `/left_encoder_ticks` | Publishes left encoder readings |
| `/right_encoder_ticks` | Publishes right encoder readings |
| `/odom` | Publishes estimated robot movement |
| `/scan` | Publishes LiDAR measurements when the LiDAR driver is running |

Inspect encoder feedback:

```bash
ros2 topic echo /left_encoder_ticks
ros2 topic echo /right_encoder_ticks
```

Run these commands separately, using `Ctrl+C` to stop each continuous output. Encoder readings should change when the corresponding encoder rotates and communication is working.

Check the odometry rate and coordinate transforms:

```bash
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser
```

The `base_link` to `laser` transform requires the corresponding static transform to be running. These checks confirm publication and availability, not measurement accuracy.

### Starting the LiDAR separately

The LiDAR launch file can be run separately:

```bash
ros2 launch robot_bringup lidar.launch.py
```

In another sourced terminal, check for scan data:

```bash
ros2 topic echo /scan --once
```

The full mapping launch described below is intended to start the necessary components together, so avoid separately launching duplicate drivers during a mapping run.

## SLAM Mapping

SLAM Toolbox uses LiDAR measurements and wheel odometry to generate a map while the robot is driven manually. Check that the ESP32 and LiDAR are connected, the motors are powered, and the Raspberry Pi's power cable is clear of the wheels.

### Step 1: Start mapping

In the first terminal:

```bash
cd ~/Documents/mobile_robot_platform/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ls -l /dev/robot_base /dev/rplidar
ros2 launch robot_bringup mapping.launch.py
```

Keep this terminal running throughout mapping.

### Step 2: Check the data

Open a second terminal, source the workspace, and run these checks separately:

```bash
ros2 topic echo /scan --once
ros2 topic hz /odom
ros2 topic echo /map --once
```

The `/map` topic contains the occupancy grid generated by SLAM Toolbox. If no map appears, inspect the mapping launch terminal for errors. Published data alone do not guarantee an accurate map.

### Step 3: Drive the robot

Open a third terminal, source the workspace, and start keyboard control:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use the keys shown in the terminal to drive slowly through the area being mapped. Keep the teleoperation terminal active while sending commands. Observe the power cable and stop if the robot behaves unexpectedly or encoder feedback becomes unreliable.

### Step 4: Save the map

Leave SLAM running. In another sourced terminal, run:

```bash
cd ~/Documents/mobile_robot_platform/ros2_ws
mkdir -p maps
ros2 run nav2_map_server map_saver_cli \
  -f "$PWD/maps/floor1" \
  --ros-args \
  -p map_subscribe_transient_local:=true \
  -p save_map_timeout:=30.0
```

The command normally creates `maps/floor1.pgm` (occupancy-grid image) and `maps/floor1.yaml` (map metadata). Confirm that the files exist:

```bash
ls -lh maps/
```

Change `floor1` in the save command to use a different map name.

### Step 5: Transfer the map to a Mac

Open a **local Mac Terminal window, not an SSH session**, and run:

```bash
scp USERNAME@IP_ADDRESS:~/Documents/mobile_robot_platform/ros2_ws/maps/floor1.pgm ~/Downloads/
scp USERNAME@IP_ADDRESS:~/Documents/mobile_robot_platform/ros2_ws/maps/floor1.yaml ~/Downloads/
```

Replace the placeholders with the Raspberry Pi username and IP address. To copy the entire map directory instead:

```bash
scp -r USERNAME@IP_ADDRESS:~/Documents/mobile_robot_platform/ros2_ws/maps ~/Downloads/
```

### Step 6: Document the results

The `.pgm` file can be opened as an image. Keep the matching `.yaml` file, which contains the resolution, origin, and other map metadata.

For thesis documentation, retain the map files, a photograph of the mapped environment, and an RViz 2 screenshot if graphical access is available. Normal SSH access does not automatically show RViz 2. The exact RViz launch procedure still needs to be checked against the workspace's launch files.

### Step 7: Stop mapping

After saving the map, press `Ctrl+C` in the keyboard-control and mapping terminals. Confirm that the motors have stopped before disconnecting power.

## Troubleshooting

### Serial devices are not found

Check the expected aliases:

```bash
ls -l /dev/robot_base /dev/rplidar
```

Check USB connections and power. To inspect detected serial devices:

```bash
ls -l /dev/ttyUSB* /dev/ttyACM*
```

Some globs may report that no matching files exist; this does not necessarily mean the other device type is missing. If the USB device is detected but the expected alias is absent, check the Raspberry Pi's serial-device configuration. The exact alias-creation rules have not yet been documented.

### Permission denied on a serial device

Check group membership:

```bash
groups
```

If `dialout` is missing, run:

```bash
sudo usermod -aG dialout $USER
```

Then log out and back in.

### Package or launch file not found

Source the environment:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/Documents/mobile_robot_platform/ros2_ws
source install/setup.bash
```

If necessary, rebuild and source again:

```bash
colcon build --packages-select robot_base robot_bringup rplidar_ros
source install/setup.bash
ros2 pkg list | grep robot_bringup
```

### Encoder readings are missing or inconsistent

Inspect each encoder topic separately:

```bash
ros2 topic echo /left_encoder_ticks
ros2 topic echo /right_encoder_ticks
```

If readings remain at zero or behave unexpectedly while the wheels rotate, check encoder connections, ESP32 communication, and the relevant ROS 2 nodes. Intermittent encoder readings were observed during development; a topic existing does not prove its measurements are reliable.

### LiDAR scan data are missing

Check the alias and scan topic:

```bash
ls -l /dev/rplidar
ros2 topic echo /scan --once
```

Inspect the LiDAR or mapping launch terminal for errors. During development, LiDAR communication worked over a USB 2.0 connection, while problems were encountered with USB 3.0; trying the previously working connection may help.

### SLAM does not generate a usable map

Check available topics, LiDAR measurements, and odometry:

```bash
ros2 topic list
ros2 topic echo /scan --once
ros2 topic hz /odom
```

Inspect launch errors and verify the required coordinate transforms. Inaccurate encoder feedback or wheel odometry can distort the map.

## Workspace Structure and Main Files

| Package | Description |
| --- | --- |
| `robot_base` | Motor control, ESP32 serial communication, and wheel odometry |
| `robot_bringup` | Launch and configuration files for robot operation and SLAM |
| `rplidar_ros` | ROS 2 driver for the RPLIDAR |
| `learning` | Learning and development programs |
| `experiments` | Experimental programs and tests |

### Main launch files

Located in `src/robot_bringup/launch/`:

| File | Purpose |
| --- | --- |
| `base_odom.launch.py` | Starts serial bridge, wheel velocity controller, and odometry |
| `lidar.launch.py` | Starts the LiDAR driver |
| `robot.launch.py` | Robot bring-up launch file |
| `slam.launch.py` | SLAM-related launch file |
| `mapping.launch.py` | Starts the components used for SLAM mapping |

### Configuration files

Located in `src/robot_bringup/config/`:

| File | Purpose |
| --- | --- |
| `floor_safe_params.yaml` | Base controller and odometry parameters |
| `slam_params.yaml` | SLAM Toolbox parameters |

### Main robot control source files

Located in `src/robot_base/src/`:

| File | Purpose |
| --- | --- |
| `base_serial_bridge.cpp` | Serial communication between Raspberry Pi and ESP32 |
| `wheel_velocity_controller.cpp` | Converts movement commands into wheel motor commands |
| `wheel_odometry.cpp` | Calculates odometry from encoder measurements |

### Robot control and data flow

Movement commands follow this path:

```text
/cmd_vel -> wheel_velocity_controller -> /wheel_pwm_cmd -> base_serial_bridge -> ESP32
```

Encoder feedback returns through:

```text
ESP32 -> base_serial_bridge -> /left_encoder_ticks, /right_encoder_ticks
```

The odometry node publishes `/odom` and the `odom -> base_link` transform. The LiDAR driver publishes `/scan`, which SLAM Toolbox uses with odometry to generate a map.

The controller includes safeguards against stale encoder feedback and implausible encoder changes. Related parameters are in `floor_safe_params.yaml`.
