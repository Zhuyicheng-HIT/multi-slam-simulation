from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sensor_share = get_package_share_directory("uf_sensor_pipeline")
    sensor_config = LaunchConfiguration("sensor_config")
    fusion_config = LaunchConfiguration("fusion_config")
    performance_output_path = LaunchConfiguration("performance_output_path")
    accuracy_output_path = LaunchConfiguration("accuracy_output_path")
    world_name = LaunchConfiguration("world_name")
    flow_truth_assistance = LaunchConfiguration("flow_truth_assistance")
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "sensor_config", default_value=sensor_share + "/config/sim_sensor_config.yaml"),
        DeclareLaunchArgument(
            "fusion_config", default_value=sensor_share + "/config/gps_flow_externalnav.yaml"),
        DeclareLaunchArgument("performance_output_path", default_value=""),
        DeclareLaunchArgument("accuracy_output_path", default_value=""),
        DeclareLaunchArgument("world_name", default_value="simple_apm_rgbd_mid360"),
        DeclareLaunchArgument("flow_truth_assistance", default_value="false"),
        Node(
            package="uf_sensor_pipeline",
            executable="fault_injector",
            name="fault_injector_gnss",
            parameters=[
                sensor_config,
                {
                    "use_sim_time": use_sim_time,
                    "output_topic": "/sensors/gnss/fix_unthrottled",
                },
            ],
            output="screen",
        ),
        Node(
            package="uf_sensor_pipeline",
            executable="gnss_metadata_relay",
            name="gnss_metadata_relay",
            parameters=[sensor_config, {"use_sim_time": use_sim_time}],
            output="screen",
        ),
        Node(
            package="uf_sensor_pipeline",
            executable="fault_injector",
            name="fault_injector_optical_flow",
            parameters=[sensor_config, {"use_sim_time": use_sim_time}],
            output="screen",
        ),
        Node(
            package="uf_sensor_pipeline",
            executable="gps_flow_fusion",
            name="gps_flow_fusion",
            parameters=[fusion_config, {"use_sim_time": use_sim_time}],
            output="screen",
        ),
        Node(
            package="uf_sensor_pipeline",
            executable="external_nav_gate",
            name="external_nav_gate",
            parameters=[fusion_config, {"use_sim_time": use_sim_time}],
            output="screen",
        ),
        Node(
            package="multi_slam_uav_sim",
            executable="simulation_performance_monitor",
            name="simulation_performance_monitor",
            parameters=[{
                "world_name": world_name,
                "use_sim_time": use_sim_time,
                "output_path": performance_output_path,
                "flow_truth_assistance": ParameterValue(
                    flow_truth_assistance, value_type=bool),
            }],
            output="screen",
        ),
        Node(
            package="multi_slam_uav_sim",
            executable="external_nav_accuracy",
            name="external_nav_accuracy",
            parameters=[{
                "world_name": world_name,
                "use_sim_time": use_sim_time,
                "output_path": accuracy_output_path,
            }],
            output="screen",
        ),
    ])
