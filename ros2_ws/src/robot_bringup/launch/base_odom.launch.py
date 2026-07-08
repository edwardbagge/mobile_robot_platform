# base_odom.launch.py
# Launch file for the low-level robot base stack.
#
# This launch file starts the three ROS 2 nodes that form the core motion
# pipeline of the mobile robot: the serial bridge, the wheel velocity control
# loop, and the odometry estimator. Together, these nodes turn motion commands
# into motor actuation and provide the robot pose estimate used by navigation.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    base_serial_port = LaunchConfiguration('base_serial_port')
    default_params_file = PathJoinSubstitution(
        [FindPackageShare('robot_bringup'), 'config', 'floor_safe_params.yaml']
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'params_file',
                default_value=default_params_file,
                description='Robot base, controller, and wheel odometry parameter file.',
            ),
            DeclareLaunchArgument(
                'base_serial_port',
                default_value='/dev/robot_base',
                description='Stable ESP32 base serial device.',
            ),
            # Report the selected parameter file and serial device before startup.
            LogInfo(
                msg=[
                    'Starting robot base stack using ',
                    params_file,
                    ' and serial port ',
                    base_serial_port,
                    '.',
                ]
            ),
            # Start the serial bridge, which connects ROS 2 to the ESP32 firmware.
            Node(
                package='robot_base',
                executable='base_serial_bridge',
                name='base_serial_bridge',
                output='screen',
                parameters=[
                    params_file,
                    {
                        'serial_port': base_serial_port,
                        'command_source': 'wheel_pwm_cmd',
                    },
                ],
            ),
            # Start the feedback controller that turns desired wheel motion into PWM.
            Node(
                package='robot_base',
                executable='wheel_velocity_controller',
                name='wheel_velocity_controller',
                output='screen',
                parameters=[params_file],
            ),
            # Start the odometry node that estimates the robot pose from encoder data.
            Node(
                package='robot_base',
                executable='wheel_odometry',
                name='wheel_odometry',
                output='screen',
                parameters=[params_file],
            ),
        ]
    )
