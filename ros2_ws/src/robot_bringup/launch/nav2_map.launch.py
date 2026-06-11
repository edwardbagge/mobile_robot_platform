from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav2_bringup_launch = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )
    default_params_file = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'params', 'nav2_params.yaml']
    )

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'map',
                default_value='',
                description='Saved map YAML file for localization and navigation.',
            ),
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
            LogInfo(msg=['Starting Nav2 map-based bringup with map ', map_file, '.']),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_bringup_launch),
                launch_arguments={
                    'map': map_file,
                    'params_file': params_file,
                    'slam': 'False',
                    'use_sim_time': use_sim_time,
                }.items(),
            ),
        ]
    )
