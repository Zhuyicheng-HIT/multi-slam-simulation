from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = LaunchConfiguration("config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    prefer_native = LaunchConfiguration("prefer_native_factor_diagnostics")
    default_config = get_package_share_directory("uf_lio_adapter") + "/config/lio_adapter.yaml"
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "prefer_native_factor_diagnostics", default_value="true"),
        Node(
            package="uf_lio_adapter",
            executable="lio_adapter",
            name="lio_adapter",
            parameters=[
                config,
                {
                    "use_sim_time": use_sim_time,
                    "prefer_native_factor_diagnostics": ParameterValue(
                        prefer_native, value_type=bool),
                },
            ],
            output="screen",
        ),
    ])
