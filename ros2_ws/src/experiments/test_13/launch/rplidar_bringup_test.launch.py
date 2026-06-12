from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    driver_package = LaunchConfiguration('driver_package')
    driver_launch_file = LaunchConfiguration('driver_launch_file')
    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    frame_id = LaunchConfiguration('frame_id')
    inverted = LaunchConfiguration('inverted')
    angle_compensate = LaunchConfiguration('angle_compensate')
    scan_mode = LaunchConfiguration('scan_mode')

    upstream_launch = PathJoinSubstitution(
        [FindPackageShare(driver_package), 'launch', driver_launch_file]
    )

    return LaunchDescription(
            [
                DeclareLaunchArgument(
                    'driver_package',
                    default_value='rplidar_ros',
                    description='Installed Slamtec ROS 2 driver package name.',
                ),
                DeclareLaunchArgument(
                    'driver_launch_file',
                    default_value='view_rplidar_a2m8_launch.py',
                    description='Launch file inside the selected driver package.',
                ),
                DeclareLaunchArgument(
                    'serial_port',
                    default_value='/dev/ttyUSB0',
                    description='Serial device used by the lidar.',
                ),
                DeclareLaunchArgument(
                    'serial_baudrate',
                    default_value='115200',
                    description='Serial baud rate for the connected lidar model.',
                ),
                DeclareLaunchArgument(
                    'frame_id',
                    default_value='laser',
                    description='Frame ID assigned to the laser scan.',
                ),
                DeclareLaunchArgument(
                    'inverted',
                    default_value='false',
                    description='Whether the scan should be inverted.',
                ),
                DeclareLaunchArgument(
                    'angle_compensate',
                    default_value='true',
                    description='Whether angle compensation should be enabled.',
                ),
                DeclareLaunchArgument(
                    'scan_mode',
                    default_value='Sensitivity',
                    description='Driver scan mode to request from the lidar.',
                ),
                LogInfo(msg=['Starting test_13 lidar bring-up using package: ', driver_package]),
                LogInfo(msg=['Driver launch file: ', driver_launch_file]),
                LogInfo(msg=['Serial port: ', serial_port, ' | baud: ', serial_baudrate]),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(upstream_launch),
                    launch_arguments={
                        'serial_port': serial_port,
                        'serial_baudrate': serial_baudrate,
                        'frame_id': frame_id,
                        'inverted': inverted,
                        'angle_compensate': angle_compensate,
                        'scan_mode': scan_mode,
                    }.items(),
                ),
            ]
    )
