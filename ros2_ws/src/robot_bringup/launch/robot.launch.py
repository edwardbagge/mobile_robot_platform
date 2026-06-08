from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    base_launch = PathJoinSubstitution(
        [FindPackageShare('robot_bringup'), 'launch', 'base_odom.launch.py']
    )
    lidar_launch = PathJoinSubstitution(
        [FindPackageShare('robot_bringup'), 'launch', 'lidar.launch.py']
    )

    return LaunchDescription(
        [
            LogInfo(msg='Starting complete robot bringup.'),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(base_launch)),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(lidar_launch)),
        ]
    )
