import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml


def _default_params_path():
    return os.path.join(
        os.path.dirname(__file__),
        '..',
        'config',
        'floor_safe_params.yaml',
    )


def _load_lidar_defaults():
    with open(_default_params_path(), 'r', encoding='utf-8') as params_file:
        all_params = yaml.safe_load(params_file)

    return (
        all_params['rplidar']['ros__parameters'],
        all_params['laser_static_transform']['ros__parameters'],
    )


def generate_launch_description():
    lidar_defaults, tf_defaults = _load_lidar_defaults()

    params_file = LaunchConfiguration('params_file')
    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    frame_id = LaunchConfiguration('frame_id')
    inverted = LaunchConfiguration('inverted')
    angle_compensate = LaunchConfiguration('angle_compensate')
    scan_mode = LaunchConfiguration('scan_mode')

    laser_x = LaunchConfiguration('laser_x')
    laser_y = LaunchConfiguration('laser_y')
    laser_z = LaunchConfiguration('laser_z')
    laser_roll = LaunchConfiguration('laser_roll')
    laser_pitch = LaunchConfiguration('laser_pitch')
    laser_yaw = LaunchConfiguration('laser_yaw')
    parent_frame_id = LaunchConfiguration('parent_frame_id')
    child_frame_id = LaunchConfiguration('child_frame_id')

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
                'serial_port',
                default_value=str(lidar_defaults['serial_port']),
                description='RPLIDAR USB serial device.',
            ),
            DeclareLaunchArgument(
                'serial_baudrate',
                default_value=str(lidar_defaults['serial_baudrate']),
                description='RPLIDAR serial baud rate.',
            ),
            DeclareLaunchArgument(
                'frame_id',
                default_value=str(lidar_defaults['frame_id']),
                description='Laser scan frame ID.',
            ),
            DeclareLaunchArgument(
                'inverted',
                default_value=str(lidar_defaults['inverted']).lower(),
                description='Whether scan data should be inverted.',
            ),
            DeclareLaunchArgument(
                'angle_compensate',
                default_value=str(lidar_defaults['angle_compensate']).lower(),
                description='Whether angle compensation should be enabled.',
            ),
            DeclareLaunchArgument(
                'scan_mode',
                default_value=str(lidar_defaults['scan_mode']),
                description='RPLIDAR scan mode.',
            ),
            DeclareLaunchArgument(
                'laser_x',
                default_value=str(tf_defaults['x']),
                description='Laser x offset from base_link in meters.',
            ),
            DeclareLaunchArgument(
                'laser_y',
                default_value=str(tf_defaults['y']),
                description='Laser y offset from base_link in meters.',
            ),
            DeclareLaunchArgument(
                'laser_z',
                default_value=str(tf_defaults['z']),
                description='Laser z offset from base_link in meters.',
            ),
            DeclareLaunchArgument(
                'laser_roll',
                default_value=str(tf_defaults['roll']),
                description='Laser roll offset in radians.',
            ),
            DeclareLaunchArgument(
                'laser_pitch',
                default_value=str(tf_defaults['pitch']),
                description='Laser pitch offset in radians.',
            ),
            DeclareLaunchArgument(
                'laser_yaw',
                default_value=str(tf_defaults['yaw']),
                description='Laser yaw offset in radians.',
            ),
            DeclareLaunchArgument(
                'parent_frame_id',
                default_value=str(tf_defaults['parent_frame_id']),
                description='Laser transform parent frame.',
            ),
            DeclareLaunchArgument(
                'child_frame_id',
                default_value=str(tf_defaults['child_frame_id']),
                description='Laser transform child frame.',
            ),
            LogInfo(msg=['Starting RPLIDAR on ', serial_port, ' with frame ', frame_id]),
            Node(
                package='rplidar_ros',
                executable='rplidar_node',
                name='rplidar_node',
                output='screen',
                parameters=[
                    params_file,
                    {
                        'channel_type': 'serial',
                        'serial_port': serial_port,
                        'serial_baudrate': ParameterValue(serial_baudrate, value_type=int),
                        'frame_id': frame_id,
                        'inverted': ParameterValue(inverted, value_type=bool),
                        'angle_compensate': ParameterValue(angle_compensate, value_type=bool),
                        'scan_mode': scan_mode,
                    },
                ],
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='laser_static_transform_publisher',
                arguments=[
                    '--x',
                    laser_x,
                    '--y',
                    laser_y,
                    '--z',
                    laser_z,
                    '--roll',
                    laser_roll,
                    '--pitch',
                    laser_pitch,
                    '--yaw',
                    laser_yaw,
                    '--frame-id',
                    parent_frame_id,
                    '--child-frame-id',
                    child_frame_id,
                ],
                output='screen',
            ),
        ]
    )
