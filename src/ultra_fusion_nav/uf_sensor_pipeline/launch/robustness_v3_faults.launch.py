from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_profiles = (
        get_package_share_directory("uf_sensor_pipeline")
        + "/config/robustness_v3_profiles.yaml"
    )
    arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("profile_path", default_value=default_profiles),
        DeclareLaunchArgument("profile", default_value="nominal"),
    ]
    defaults = {
        "native_lidar_input_topic": "/robustness/raw/native_lidar_factor",
        "native_lidar_output_topic": "/fast_lio/native_lidar_factor",
        "imu_input_topic": "/robustness/raw/imu",
        "imu_output_topic": "/sensors/imu",
        "gnss_input_topic": "/robustness/raw/gnss",
        "gnss_output_topic": "/sensors/gnss/fix",
        "optical_flow_input_topic": "/robustness/raw/optical_flow",
        "optical_flow_output_topic": "/sensors/optical_flow/rad",
        "vision_input_topic": "/robustness/raw/visual_tracks",
        "vision_output_topic": "/vision/feature_tracks",
    }
    arguments.extend(
        DeclareLaunchArgument(name, default_value=value)
        for name, value in defaults.items()
    )
    node = Node(
        package="uf_sensor_pipeline",
        executable="robustness_fault_injector",
        name="robustness_v3_fault_injector",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "profile_path": LaunchConfiguration("profile_path"),
            "profile": LaunchConfiguration("profile"),
            **{
                name: LaunchConfiguration(name) for name in defaults
            },
        }],
        output="screen",
    )
    return LaunchDescription([*arguments, node])
