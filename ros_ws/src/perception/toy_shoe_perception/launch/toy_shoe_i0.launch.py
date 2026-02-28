from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("toy_shoe_perception"))
    default_config = package_share / "config" / "inference_i0.yaml"

    config_file = LaunchConfiguration("config_file")
    model_engine_path = LaunchConfiguration("model_engine_path")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=str(default_config),
                description="Path to toy-shoe I0 inference parameter YAML",
            ),
            DeclareLaunchArgument(
                "model_engine_path",
                default_value=EnvironmentVariable("TOY_SHOE_ENGINE_PATH", default_value=""),
                description="Path to TensorRT engine (.engine) file",
            ),
            Node(
                package="toy_shoe_perception",
                executable="trt_detector_node",
                name="toy_shoe_trt_detector",
                output="screen",
                parameters=[
                    ParameterFile(config_file, allow_substs=True),
                    {"model_engine_path": model_engine_path},
                ],
            ),
            Node(
                package="toy_shoe_perception",
                executable="safety_gate_node",
                name="toy_shoe_safety_gate",
                output="screen",
                parameters=[ParameterFile(config_file, allow_substs=True)],
            ),
        ]
    )
