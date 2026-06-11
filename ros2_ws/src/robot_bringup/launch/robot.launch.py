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
                description='Shared robot bringup parameter file.',
            ),
            DeclareLaunchArgument(
                'base_serial_port',
                default_value='/dev/robot_base',
                description='ESP32 USB serial device.',
            ),
            DeclareLaunchArgument(
                'lidar_serial_port',
                default_value='/dev/rplidar',
                description='RPLIDAR USB serial device.',
            ),
            LogInfo(msg='Starting complete robot bringup.'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(base_launch),
                launch_arguments={
                    'params_file': params_file,
                    'base_serial_port': base_serial_port,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(lidar_launch),
                launch_arguments={
                    'params_file': params_file,
                    'lidar_serial_port': lidar_serial_port,
                }.items(),
            ),
        ]
    )
