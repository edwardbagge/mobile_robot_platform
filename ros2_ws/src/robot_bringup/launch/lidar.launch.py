from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


def _load_lidar_defaults(params_path):
    with open(params_path, 'r', encoding='utf-8') as params_file:
        all_params = yaml.safe_load(params_file)

    return all_params['laser_static_transform']['ros__parameters']


def _launch_setup(context):
    params_file = LaunchConfiguration('params_file')
    lidar_serial_port = LaunchConfiguration('lidar_serial_port')
    params_path = params_file.perform(context)
    tf_defaults = _load_lidar_defaults(params_path)

    return [
        LogInfo(
            msg=[
                'Starting RPLIDAR with ',
                params_file,
                ' and serial port ',
                lidar_serial_port,
                '.',
            ]
        ),
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[
                params_file,
                {
                    'channel_type': 'serial',
                    'serial_port': lidar_serial_port,
                },
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_static_transform_publisher',
            arguments=[
                '--x',
                str(tf_defaults['x']),
                '--y',
                str(tf_defaults['y']),
                '--z',
                str(tf_defaults['z']),
                '--roll',
                str(tf_defaults['roll']),
                '--pitch',
                str(tf_defaults['pitch']),
                '--yaw',
                str(tf_defaults['yaw']),
                '--frame-id',
                str(tf_defaults['parent_frame_id']),
                '--child-frame-id',
                str(tf_defaults['child_frame_id']),
            ],
            output='screen',
        ),
    ]


def generate_launch_description():
    default_params_file = PathJoinSubstitution(
        [FindPackageShare('robot_bringup'), 'config', 'floor_safe_params.yaml']
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'params_file',
                default_value=default_params_file,
                description='Lidar and laser static transform parameter file.',
            ),
            DeclareLaunchArgument(
                'lidar_serial_port',
                default_value='/dev/rplidar',
                description='RPLIDAR USB serial device.',
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
