from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial device for the RPLIDAR A2M8',
        ),
        DeclareLaunchArgument(
            'serial_baudrate',
            default_value='256000',
            description='Baud rate for the RPLIDAR A2M8',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='laser',
            description='TF frame published in LaserScan messages',
        ),
        DeclareLaunchArgument(
            'inverted',
            default_value='false',
            description='Whether to invert the scan data',
        ),
        DeclareLaunchArgument(
            'angle_compensate',
            default_value='true',
            description='Whether to enable angle compensation',
        ),
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': LaunchConfiguration('serial_port'),
                'serial_baudrate': LaunchConfiguration('serial_baudrate'),
                'frame_id': LaunchConfiguration('frame_id'),
                'inverted': LaunchConfiguration('inverted'),
                'angle_compensate': LaunchConfiguration('angle_compensate'),
            }],
        ),
    ])
