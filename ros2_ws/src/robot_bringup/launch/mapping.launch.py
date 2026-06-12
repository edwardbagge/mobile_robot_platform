from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_launch = PathJoinSubstitution(
        [FindPackageShare('robot_bringup'), 'launch', 'robot.launch.py']
    )
    slam_launch = PathJoinSubstitution(
        [FindPackageShare('robot_bringup'), 'launch', 'slam.launch.py']
    )
    default_params_file = PathJoinSubstitution(
        [FindPackageShare('robot_bringup'), 'config', 'floor_safe_params.yaml']
    )
    default_slam_params_file = PathJoinSubstitution(
        [FindPackageShare('robot_bringup'), 'config', 'slam_params.yaml']
    )

    robot_params_file = LaunchConfiguration('params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    base_serial_port = LaunchConfiguration('base_serial_port')
    lidar_serial_port = LaunchConfiguration('lidar_serial_port')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'params_file',
                default_value=default_params_file,
                description='Shared base and lidar parameter file.',
            ),
            DeclareLaunchArgument(
                'slam_params_file',
                default_value=default_slam_params_file,
                description='SLAM Toolbox online mapping parameter file.',
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
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use simulation clock.',
            ),
            LogInfo(msg='Starting robot base, lidar, and SLAM mapping stack.'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(robot_launch),
                launch_arguments={
                    'params_file': robot_params_file,
                    'base_serial_port': base_serial_port,
                    'lidar_serial_port': lidar_serial_port,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_launch),
                launch_arguments={
                    'slam_params_file': slam_params_file,
                    'use_sim_time': use_sim_time,
                }.items(),
            ),
        ]
    )
