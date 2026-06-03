from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    command_rate_hz = LaunchConfiguration("command_rate_hz")
    command_timeout_s = LaunchConfiguration("command_timeout_s")
    wheel_base_m = LaunchConfiguration("wheel_base_m")
    max_wheel_speed_mps = LaunchConfiguration("max_wheel_speed_mps")
    max_pwm = LaunchConfiguration("max_pwm")
    min_pwm = LaunchConfiguration("min_pwm")
    startup_delay_s = LaunchConfiguration("startup_delay_s")

    publish_rate_hz = LaunchConfiguration("publish_rate_hz")
    ticks_per_revolution = LaunchConfiguration("ticks_per_revolution")
    wheel_radius_m = LaunchConfiguration("wheel_radius_m")
    odom_frame_id = LaunchConfiguration("odom_frame_id")
    base_frame_id = LaunchConfiguration("base_frame_id")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/ttyUSB0",
                description="ESP32 USB serial device.",
            ),
            DeclareLaunchArgument(
                "command_rate_hz",
                default_value="20.0",
                description="Rate used by robot_base to resend motor commands.",
            ),
            DeclareLaunchArgument(
                "command_timeout_s",
                default_value="0.15",
                description="Stop command timeout used by robot_base.",
            ),
            DeclareLaunchArgument(
                "wheel_base_m",
                default_value="0.16",
                description="Distance between left and right wheel contact centers.",
            ),
            DeclareLaunchArgument(
                "max_wheel_speed_mps",
                default_value="0.10",
                description="Wheel speed represented by max_pwm in open-loop control.",
            ),
            DeclareLaunchArgument(
                "max_pwm",
                default_value="100",
                description="Maximum signed PWM sent to the ESP32.",
            ),
            DeclareLaunchArgument(
                "min_pwm",
                default_value="85",
                description="Minimum non-zero PWM used to overcome motor stiction.",
            ),
            DeclareLaunchArgument(
                "startup_delay_s",
                default_value="2.5",
                description="Delay after opening serial to let ESP32 finish USB reset.",
            ),
            DeclareLaunchArgument(
                "publish_rate_hz",
                default_value="2.0",
                description="Odometry publish rate.",
            ),
            DeclareLaunchArgument(
                "ticks_per_revolution",
                default_value="2800.0",
                description="Encoder ticks per output wheel revolution using full quadrature.",
            ),
            DeclareLaunchArgument(
                "wheel_radius_m",
                default_value="0.041",
                description="Wheel radius in meters for 81-82 mm diameter wheels.",
            ),
            DeclareLaunchArgument(
                "odom_frame_id",
                default_value="odom",
                description="Odometry frame ID.",
            ),
            DeclareLaunchArgument(
                "base_frame_id",
                default_value="base_link",
                description="Robot base frame ID.",
            ),
            LogInfo(msg="Starting robot base bridge and wheel odometry."),
            LogInfo(
                msg=[
                    "Base params: max_pwm=",
                    max_pwm,
                    ", min_pwm=",
                    min_pwm,
                    ", timeout=",
                    command_timeout_s,
                    " s",
                ]
            ),
            LogInfo(
                msg=[
                    "Odom params: ticks_per_revolution=",
                    ticks_per_revolution,
                    ", wheel_radius_m=",
                    wheel_radius_m,
                ]
            ),
            Node(
                package="robot_base",
                executable="base_serial_bridge",
                name="base_serial_bridge",
                output="screen",
                parameters=[
                    {
                        "serial_port": serial_port,
                        "command_rate_hz": ParameterValue(command_rate_hz, value_type=float),
                        "command_timeout_s": ParameterValue(command_timeout_s, value_type=float),
                        "wheel_base_m": ParameterValue(wheel_base_m, value_type=float),
                        "max_wheel_speed_mps": ParameterValue(
                            max_wheel_speed_mps, value_type=float
                        ),
                        "max_pwm": ParameterValue(max_pwm, value_type=int),
                        "min_pwm": ParameterValue(min_pwm, value_type=int),
                        "startup_delay_s": ParameterValue(startup_delay_s, value_type=float),
                    }
                ],
            ),
            Node(
                package="test_12",
                executable="wheel_odometry_test",
                name="wheel_odometry",
                output="screen",
                parameters=[
                    {
                        "publish_rate_hz": ParameterValue(publish_rate_hz, value_type=float),
                        "ticks_per_revolution": ParameterValue(
                            ticks_per_revolution, value_type=float
                        ),
                        "wheel_radius_m": ParameterValue(wheel_radius_m, value_type=float),
                        "wheel_base_m": ParameterValue(wheel_base_m, value_type=float),
                        "odom_frame_id": odom_frame_id,
                        "base_frame_id": base_frame_id,
                    }
                ],
            ),
        ]
    )
