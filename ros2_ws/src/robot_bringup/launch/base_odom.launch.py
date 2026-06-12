from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    base_serial_port = LaunchConfiguration('base_serial_port')
    default_params_file = PathJoinSubstitution(
        [FindPackageShare('robot_bringup'), 'config', 'floor_safe_params.yaml']
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'params_file',
                default_value=default_params_file,
                description='Robot base, controller, and wheel odometry parameter file.',
            ),
            DeclareLaunchArgument(
                'base_serial_port',
                default_value='/dev/robot_base',
                description='Stable ESP32 base serial device.',
            ),
            LogInfo(
                msg=[
                    'Starting robot base stack using ',
                    params_file,
                    ' and serial port ',
                    base_serial_port,
                    '.',
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
                        'serial_port': base_serial_port,
                        'command_source': 'wheel_pwm_cmd',
                    },
                ],
            ),
            Node(
                package='robot_base',
                executable='wheel_velocity_controller',
                name='wheel_velocity_controller',
                output='screen',
                parameters=[params_file],
            ),
            Node(
                package='robot_base',
                executable='wheel_odometry',
                name='wheel_odometry',
                output='screen',
                parameters=[params_file],
            ),
        ]
    )
