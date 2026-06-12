from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    slam_launch = PathJoinSubstitution(
        [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py']
    )
    default_slam_params_file = PathJoinSubstitution(
        [FindPackageShare('robot_bringup'), 'config', 'slam_params.yaml']
    )

    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'slam_params_file',
                default_value=default_slam_params_file,
                description='SLAM Toolbox online mapping parameter file.',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use simulation clock.',
            ),
            LogInfo(
                msg=[
                    'Starting SLAM Toolbox online async mapping using ',
                    slam_params_file,
                    '.',
                ]
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
