from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config_file = LaunchConfiguration("config_file")
    web_root = LaunchConfiguration("web_root")
    http_host = LaunchConfiguration("http_host")
    http_port = LaunchConfiguration("http_port")
    foxglove_port = LaunchConfiguration("foxglove_port")
    start_foxglove = LaunchConfiguration("start_foxglove")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=EnvironmentVariable("ROBOT_CONSOLE_CONFIG", default_value=""),
                description="Path to robot console YAML config",
            ),
            DeclareLaunchArgument(
                "web_root",
                default_value=EnvironmentVariable("ROBOT_CONSOLE_WEB_ROOT", default_value=""),
                description="Path to web static assets",
            ),
            DeclareLaunchArgument(
                "http_host",
                default_value=EnvironmentVariable("ROBOT_CONSOLE_HOST", default_value="0.0.0.0"),
                description="HTTP bind host",
            ),
            DeclareLaunchArgument(
                "http_port",
                default_value=EnvironmentVariable("ROBOT_CONSOLE_PORT", default_value="8080"),
                description="HTTP bind port",
            ),
            DeclareLaunchArgument(
                "foxglove_port",
                default_value=EnvironmentVariable("FOXGLOVE_PORT", default_value="8765"),
                description="Foxglove bridge port",
            ),
            DeclareLaunchArgument(
                "start_foxglove",
                default_value="false",
                description="Also launch foxglove_bridge inside this launch file",
            ),
            Node(
                package="robot_console",
                executable="api_node",
                name="robot_console_api",
                output="screen",
                parameters=[
                    {
                        "config_file": config_file,
                        "web_root": web_root,
                        "http_host": http_host,
                        "http_port": http_port,
                        "foxglove_port": foxglove_port,
                    }
                ],
            ),
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="foxglove_bridge",
                output="screen",
                parameters=[{"port": foxglove_port}],
                condition=IfCondition(start_foxglove),
            ),
        ]
    )
