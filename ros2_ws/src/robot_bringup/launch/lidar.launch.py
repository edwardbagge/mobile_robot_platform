from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


LASER_STATIC_TRANSFORM_KEYS = (
    'x',
    'y',
    'z',
    'roll',
    'pitch',
    'yaw',
    'parent_frame_id',
    'child_frame_id',
)


def _load_laser_static_transform(params_path):
    try:
        with open(params_path, 'r', encoding='utf-8') as params_file:
            all_params = yaml.safe_load(params_file) or {}
    except OSError as exc:
        raise RuntimeError(
            f'Unable to read lidar parameter file "{params_path}": {exc}'
        ) from exc
    except yaml.YAMLError as exc:
        raise RuntimeError(
            f'Invalid YAML in lidar parameter file "{params_path}": {exc}'
        ) from exc

    transform_node = all_params.get('laser_static_transform')
    if not isinstance(transform_node, dict):
        raise RuntimeError(
            f'Lidar parameter file "{params_path}" must define '
            'laser_static_transform.ros__parameters.'
        )

    transform_params = transform_node.get('ros__parameters')
    if not isinstance(transform_params, dict):
        raise RuntimeError(
            f'Lidar parameter file "{params_path}" must define '
            'laser_static_transform.ros__parameters.'
        )

    missing_keys = [
        key for key in LASER_STATIC_TRANSFORM_KEYS if key not in transform_params
    ]
    if missing_keys:
        raise RuntimeError(
            f'Lidar parameter file "{params_path}" is missing '
            'laser_static_transform.ros__parameters keys: '
            f'{", ".join(missing_keys)}.'
        )

    return transform_params


def _static_transform_arguments(transform_params):
    return [
        '--x',
        str(transform_params['x']),
        '--y',
        str(transform_params['y']),
        '--z',
        str(transform_params['z']),
        '--roll',
        str(transform_params['roll']),
        '--pitch',
        str(transform_params['pitch']),
        '--yaw',
        str(transform_params['yaw']),
        '--frame-id',
        str(transform_params['parent_frame_id']),
        '--child-frame-id',
        str(transform_params['child_frame_id']),
    ]


def _launch_setup(context):
    params_file = LaunchConfiguration('params_file')
    lidar_serial_port = LaunchConfiguration('lidar_serial_port')
    params_path = params_file.perform(context)
    transform_params = _load_laser_static_transform(params_path)

    return [
        LogInfo(
            msg=[
                'Starting RPLIDAR using ',
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
            arguments=_static_transform_arguments(transform_params),
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
                description='RPLIDAR and laser static transform parameter file.',
            ),
            DeclareLaunchArgument(
                'lidar_serial_port',
                default_value='/dev/rplidar',
                description='Stable RPLIDAR serial device.',
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
