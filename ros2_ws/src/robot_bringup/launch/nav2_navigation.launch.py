from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav2_navigation_launch = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py']
    )
    default_params_file = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'params', 'nav2_params.yaml']
    )

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'params_file',
                default_value=default_params_file,
                description='Nav2 parameter file.',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use simulation clock.',
            ),
            LogInfo(msg=['Starting Nav2 navigation stack with ', params_file, '.']),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_navigation_launch),
                launch_arguments={
                    'params_file': params_file,
                    'use_sim_time': use_sim_time,
                }.items(),
            ),
        ]
    )
