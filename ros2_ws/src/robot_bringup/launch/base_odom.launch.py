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


def _load_floor_safe_defaults():
    with open(_default_params_path(), 'r', encoding='utf-8') as params_file:
        all_params = yaml.safe_load(params_file)

    return (
        all_params['base_serial_bridge']['ros__parameters'],
        all_params['wheel_odometry']['ros__parameters'],
    )


def generate_launch_description():
    base_defaults, odom_defaults = _load_floor_safe_defaults()

    params_file = LaunchConfiguration('params_file')
    serial_port = LaunchConfiguration('serial_port')
    command_rate_hz = LaunchConfiguration('command_rate_hz')
    command_timeout_s = LaunchConfiguration('command_timeout_s')
    base_wheel_base_m = LaunchConfiguration('base_wheel_base_m')
    max_wheel_speed_mps = LaunchConfiguration('max_wheel_speed_mps')
    max_pwm = LaunchConfiguration('max_pwm')
    min_pwm = LaunchConfiguration('min_pwm')
    startup_delay_s = LaunchConfiguration('startup_delay_s')

    publish_rate_hz = LaunchConfiguration('publish_rate_hz')
    ticks_per_revolution = LaunchConfiguration('ticks_per_revolution')
    wheel_radius_m = LaunchConfiguration('wheel_radius_m')
    odom_wheel_base_m = LaunchConfiguration('odom_wheel_base_m')
    odom_frame_id = LaunchConfiguration('odom_frame_id')
    base_frame_id = LaunchConfiguration('base_frame_id')

    default_params_file = PathJoinSubstitution(
        [FindPackageShare('robot_bringup'), 'config', 'floor_safe_params.yaml']
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'params_file',
                default_value=default_params_file,
                description='Base and odometry parameter file.',
            ),
            DeclareLaunchArgument(
                'serial_port',
                default_value=str(base_defaults['serial_port']),
                description='ESP32 USB serial device.',
            ),
            DeclareLaunchArgument(
                'command_rate_hz',
                default_value=str(base_defaults['command_rate_hz']),
                description='Rate used by robot_base to resend motor commands.',
            ),
            DeclareLaunchArgument(
                'command_timeout_s',
                default_value=str(base_defaults['command_timeout_s']),
                description='Stop command timeout used by robot_base.',
            ),
            DeclareLaunchArgument(
                'base_wheel_base_m',
                default_value=str(base_defaults['wheel_base_m']),
                description='Physical wheel base used for cmd_vel to wheel speed conversion.',
            ),
            DeclareLaunchArgument(
                'max_wheel_speed_mps',
                default_value=str(base_defaults['max_wheel_speed_mps']),
                description='Wheel speed represented by max_pwm in open-loop control.',
            ),
            DeclareLaunchArgument(
                'max_pwm',
                default_value=str(base_defaults['max_pwm']),
                description='Maximum signed PWM sent to the ESP32.',
            ),
            DeclareLaunchArgument(
                'min_pwm',
                default_value=str(base_defaults['min_pwm']),
                description='Minimum non-zero PWM used to overcome motor stiction.',
            ),
            DeclareLaunchArgument(
                'startup_delay_s',
                default_value=str(base_defaults['startup_delay_s']),
                description='Delay after opening serial to let ESP32 finish USB reset.',
            ),
            DeclareLaunchArgument(
                'publish_rate_hz',
                default_value=str(odom_defaults['publish_rate_hz']),
                description='Odometry publish rate.',
            ),
            DeclareLaunchArgument(
                'ticks_per_revolution',
                default_value=str(odom_defaults['ticks_per_revolution']),
                description='Encoder ticks per output wheel revolution using full quadrature.',
            ),
            DeclareLaunchArgument(
                'wheel_radius_m',
                default_value=str(odom_defaults['wheel_radius_m']),
                description='Wheel radius in meters for 81-82 mm diameter wheels.',
            ),
            DeclareLaunchArgument(
                'odom_wheel_base_m',
                default_value=str(odom_defaults['wheel_base_m']),
                description='Effective wheel base used for encoder odometry yaw integration.',
            ),
            DeclareLaunchArgument(
                'odom_frame_id',
                default_value=str(odom_defaults['odom_frame_id']),
                description='Odometry frame ID.',
            ),
            DeclareLaunchArgument(
                'base_frame_id',
                default_value=str(odom_defaults['base_frame_id']),
                description='Robot base frame ID.',
            ),
            LogInfo(msg='Starting robot base bridge and wheel odometry.'),
            LogInfo(
                msg=[
                    'Base params: max_pwm=',
                    max_pwm,
                    ', min_pwm=',
                    min_pwm,
                    ', wheel_base_m=',
                    base_wheel_base_m,
                    ', timeout=',
                    command_timeout_s,
                    ' s',
                ]
            ),
            LogInfo(
                msg=[
                    'Odom params: ticks_per_revolution=',
                    ticks_per_revolution,
                    ', wheel_radius_m=',
                    wheel_radius_m,
                    ', wheel_base_m=',
                    odom_wheel_base_m,
                ]
            ),
            Node(
                package='robot_base',
                executable='base_serial_bridge',
                name='base_serial_bridge',
                output='screen',
                parameters=[
                    params_file,
                    {
                        'serial_port': serial_port,
                        'command_rate_hz': ParameterValue(command_rate_hz, value_type=float),
                        'command_timeout_s': ParameterValue(command_timeout_s, value_type=float),
                        'wheel_base_m': ParameterValue(base_wheel_base_m, value_type=float),
                        'max_wheel_speed_mps': ParameterValue(
                            max_wheel_speed_mps,
                            value_type=float,
                        ),
                        'max_pwm': ParameterValue(max_pwm, value_type=int),
                        'min_pwm': ParameterValue(min_pwm, value_type=int),
                        'startup_delay_s': ParameterValue(startup_delay_s, value_type=float),
                    },
                ],
            ),
            Node(
                package='robot_base',
                executable='wheel_odometry',
                name='wheel_odometry',
                output='screen',
                parameters=[
                    params_file,
                    {
                        'publish_rate_hz': ParameterValue(publish_rate_hz, value_type=float),
                        'ticks_per_revolution': ParameterValue(
                            ticks_per_revolution,
                            value_type=float,
                        ),
                        'wheel_radius_m': ParameterValue(wheel_radius_m, value_type=float),
                        'wheel_base_m': ParameterValue(odom_wheel_base_m, value_type=float),
                        'odom_frame_id': odom_frame_id,
                        'base_frame_id': base_frame_id,
                    },
                ],
            ),
        ]
    )
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


def _load_floor_safe_defaults():
    with open(_default_params_path(), 'r', encoding='utf-8') as params_file:
        all_params = yaml.safe_load(params_file)

    return (
        all_params['base_serial_bridge']['ros__parameters'],
        all_params['wheel_odometry']['ros__parameters'],
    )


def generate_launch_description():
    base_defaults, odom_defaults = _load_floor_safe_defaults()

    params_file = LaunchConfiguration('params_file')
    serial_port = LaunchConfiguration('serial_port')
    command_rate_hz = LaunchConfiguration('command_rate_hz')
    command_timeout_s = LaunchConfiguration('command_timeout_s')
    wheel_base_m = LaunchConfiguration('wheel_base_m')
    max_wheel_speed_mps = LaunchConfiguration('max_wheel_speed_mps')
    max_pwm = LaunchConfiguration('max_pwm')
    min_pwm = LaunchConfiguration('min_pwm')
    startup_delay_s = LaunchConfiguration('startup_delay_s')

    publish_rate_hz = LaunchConfiguration('publish_rate_hz')
    ticks_per_revolution = LaunchConfiguration('ticks_per_revolution')
    wheel_radius_m = LaunchConfiguration('wheel_radius_m')
    odom_frame_id = LaunchConfiguration('odom_frame_id')
    base_frame_id = LaunchConfiguration('base_frame_id')

    default_params_file = PathJoinSubstitution(
        [FindPackageShare('robot_bringup'), 'config', 'floor_safe_params.yaml']
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'params_file',
                default_value=default_params_file,
                description='Base and odometry parameter file.',
            ),
            DeclareLaunchArgument(
                'serial_port',
                default_value=str(base_defaults['serial_port']),
                description='ESP32 USB serial device.',
            ),
            DeclareLaunchArgument(
                'command_rate_hz',
                default_value=str(base_defaults['command_rate_hz']),
                description='Rate used by robot_base to resend motor commands.',
            ),
            DeclareLaunchArgument(
                'command_timeout_s',
                default_value=str(base_defaults['command_timeout_s']),
                description='Stop command timeout used by robot_base.',
            ),
            DeclareLaunchArgument(
                'wheel_base_m',
                default_value=str(base_defaults['wheel_base_m']),
                description='Distance between left and right wheel contact centers.',
            ),
            DeclareLaunchArgument(
                'max_wheel_speed_mps',
                default_value=str(base_defaults['max_wheel_speed_mps']),
                description='Wheel speed represented by max_pwm in open-loop control.',
            ),
            DeclareLaunchArgument(
                'max_pwm',
                default_value=str(base_defaults['max_pwm']),
                description='Maximum signed PWM sent to the ESP32.',
            ),
            DeclareLaunchArgument(
                'min_pwm',
                default_value=str(base_defaults['min_pwm']),
                description='Minimum non-zero PWM used to overcome motor stiction.',
            ),
            DeclareLaunchArgument(
                'startup_delay_s',
                default_value=str(base_defaults['startup_delay_s']),
                description='Delay after opening serial to let ESP32 finish USB reset.',
            ),
            DeclareLaunchArgument(
                'publish_rate_hz',
                default_value=str(odom_defaults['publish_rate_hz']),
                description='Odometry publish rate.',
            ),
            DeclareLaunchArgument(
                'ticks_per_revolution',
                default_value=str(odom_defaults['ticks_per_revolution']),
                description='Encoder ticks per output wheel revolution using full quadrature.',
            ),
            DeclareLaunchArgument(
                'wheel_radius_m',
                default_value=str(odom_defaults['wheel_radius_m']),
                description='Wheel radius in meters for 81-82 mm diameter wheels.',
            ),
            DeclareLaunchArgument(
                'odom_frame_id',
                default_value=str(odom_defaults['odom_frame_id']),
                description='Odometry frame ID.',
            ),
            DeclareLaunchArgument(
                'base_frame_id',
                default_value=str(odom_defaults['base_frame_id']),
                description='Robot base frame ID.',
            ),
            LogInfo(msg='Starting robot base bridge and wheel odometry.'),
            LogInfo(
                msg=[
                    'Base params: max_pwm=',
                    max_pwm,
                    ', min_pwm=',
                    min_pwm,
                    ', timeout=',
                    command_timeout_s,
                    ' s',
                ]
            ),
            LogInfo(
                msg=[
                    'Odom params: ticks_per_revolution=',
                    ticks_per_revolution,
                    ', wheel_radius_m=',
                    wheel_radius_m,
                ]
            ),
            Node(
                package='robot_base',
                executable='base_serial_bridge',
                name='base_serial_bridge',
                output='screen',
                parameters=[
                    params_file,
                    {
                        'serial_port': serial_port,
                        'command_rate_hz': ParameterValue(command_rate_hz, value_type=float),
                        'command_timeout_s': ParameterValue(command_timeout_s, value_type=float),
                        'wheel_base_m': ParameterValue(wheel_base_m, value_type=float),
                        'max_wheel_speed_mps': ParameterValue(
                            max_wheel_speed_mps,
                            value_type=float,
                        ),
                        'max_pwm': ParameterValue(max_pwm, value_type=int),
                        'min_pwm': ParameterValue(min_pwm, value_type=int),
                        'startup_delay_s': ParameterValue(startup_delay_s, value_type=float),
                    },
                ],
            ),
            Node(
                package='test_12',
                executable='wheel_odometry_test',
                name='wheel_odometry',
                output='screen',
                parameters=[
                    params_file,
                    {
                        'publish_rate_hz': ParameterValue(publish_rate_hz, value_type=float),
                        'ticks_per_revolution': ParameterValue(
                            ticks_per_revolution,
                            value_type=float,
                        ),
                        'wheel_radius_m': ParameterValue(wheel_radius_m, value_type=float),
                        'wheel_base_m': ParameterValue(wheel_base_m, value_type=float),
                        'odom_frame_id': odom_frame_id,
                        'base_frame_id': base_frame_id,
                    },
                ],
            ),
        ]
    )
