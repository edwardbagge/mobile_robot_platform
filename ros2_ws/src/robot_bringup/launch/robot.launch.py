# robot.launch.py
# High-level launch file for the complete base and lidar bring-up.
#
# This file composes the base stack and the lidar stack into a single startup
# workflow so that the robot can publish both wheel odometry and laser scan data.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    base_launch = PathJoinSubstitution(
        [FindPackageShare('robot_bringup'), 'launch', 'base_odom.launch.py']
    )
    lidar_launch = PathJoinSubstitution(
        [FindPackageShare('robot_bringup'), 'launch', 'lidar.launch.py']
    )
    default_params_file = PathJoinSubstitution(
        [FindPackageShare('robot_bringup'), 'config', 'floor_safe_params.yaml']
    )

    params_file = LaunchConfiguration('params_file')
    base_serial_port = LaunchConfiguration('base_serial_port')
    lidar_serial_port = LaunchConfiguration('lidar_serial_port')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'params_file',
                default_value=default_params_file,
                description='Shared base and lidar parameter file.',
            ),
            DeclareLaunchArgument(
                'base_serial_port',
                default_value='/dev/robot_base',
                description='Stable ESP32 base serial device.',
            ),
            DeclareLaunchArgument(
                'lidar_serial_port',
                default_value='/dev/rplidar',
                description='Stable RPLIDAR serial device.',
            ),
            # Announce that the base and lidar subsystems are being started together.
            LogInfo(msg='Starting robot base and lidar bringup.'),
            # Launch the base stack, which includes the serial bridge, controller,
            # and odometry nodes.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(base_launch),
                launch_arguments={
                    'params_file': params_file,
                    'base_serial_port': base_serial_port,
                }.items(),
            ),
            # Launch the lidar stack so that scan data is available for mapping or
            # obstacle detection.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(lidar_launch),
                launch_arguments={
                    'params_file': params_file,
                    'lidar_serial_port': lidar_serial_port,
                }.items(),
            ),
        ]
    )
