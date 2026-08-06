import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = get_package_share_directory("uf_sensor_pipeline") + "/config/sim_sensor_config.yaml"
    config = LaunchConfiguration("config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_fcu_observation_bridge = LaunchConfiguration("enable_fcu_observation_bridge")
    enable_vision = LaunchConfiguration("enable_vision")
    enable_nmea_gnss = LaunchConfiguration("enable_nmea_gnss")
    optical_flow_input_topic = LaunchConfiguration("optical_flow_input_topic")
    gnss_input_topic = LaunchConfiguration("gnss_input_topic")
    d435_color_input_topic = LaunchConfiguration("d435_color_input_topic")
    d435_depth_input_topic = LaunchConfiguration("d435_depth_input_topic")
    fcu_flow_input_topic = LaunchConfiguration("fcu_flow_input_topic")
    fcu_flow_rad_input_topic = LaunchConfiguration("fcu_flow_rad_input_topic")
    fcu_range_input_topic = LaunchConfiguration("fcu_range_input_topic")
    scheduled_fault_modality = os.environ.get("UF_FAULT_MODALITY", "").strip()
    scheduled_fault = {}
    if scheduled_fault_modality:
        scheduled_fault = {
            "fault_type": os.environ.get("UF_FAULT_TYPE", "none"),
            "fault_start_s": float(os.environ.get("UF_FAULT_START_S", "0.0")),
            "fault_duration_s": float(os.environ.get("UF_FAULT_DURATION_S", "0.0")),
            "magnitude": float(os.environ.get("UF_FAULT_MAGNITUDE", "0.0")),
            "secondary_magnitude": float(
                os.environ.get("UF_FAULT_SECONDARY_MAGNITUDE", "0.0")
            ),
        }
    nodes = [
        Node(
            package="uf_sensor_pipeline",
            executable="nmea_gnss",
            name="nmea_gnss",
            parameters=[
                config,
                {
                    "use_sim_time": use_sim_time,
                    "port": LaunchConfiguration("nmea_port"),
                    "strict_checksum": LaunchConfiguration("nmea_strict_checksum"),
                },
            ],
            output="screen",
            condition=IfCondition(enable_nmea_gnss),
        ),
        Node(
            package="uf_sensor_pipeline",
            executable="gnss_metadata_relay",
            name="gnss_metadata_relay",
            parameters=[
                config,
                {
                    "use_sim_time": use_sim_time,
                    "input_topic": LaunchConfiguration("gnss_raw_input_topic"),
                },
            ],
            output="screen",
            condition=UnlessCondition(enable_nmea_gnss),
        ),
        Node(
            package="uf_sensor_pipeline",
            executable="fcu_observation_bridge",
            name="fcu_observation_bridge",
            parameters=[
                config,
                {
                    "use_sim_time": use_sim_time,
                    "flow_input_topic": fcu_flow_input_topic,
                    "flow_rad_input_topic": fcu_flow_rad_input_topic,
                    "range_input_topic": fcu_range_input_topic,
                },
            ],
            output="screen",
            condition=IfCondition(enable_fcu_observation_bridge),
        ),
        Node(
            package="uf_sensor_pipeline",
            executable="pointcloud_body_filter",
            name="pointcloud_body_filter",
            parameters=[config, {"use_sim_time": use_sim_time}],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="d435i_mount_tf",
            arguments=[
                "--x", LaunchConfiguration("d435_x"),
                "--y", LaunchConfiguration("d435_y"),
                "--z", LaunchConfiguration("d435_z"),
                "--roll", LaunchConfiguration("d435_roll"),
                "--pitch", LaunchConfiguration("d435_pitch"),
                "--yaw", LaunchConfiguration("d435_yaw"),
                "--frame-id", LaunchConfiguration("d435_parent_frame"),
                "--child-frame-id", LaunchConfiguration("d435_child_frame"),
            ],
            output="screen",
            condition=IfCondition(enable_vision),
        ),
    ]
    for modality in ("lidar", "imu", "gnss", "optical_flow", "depth", "color"):
        fault_parameters = scheduled_fault if modality == scheduled_fault_modality else {}
        source_parameters = {}
        if modality == "optical_flow":
            source_parameters = {"input_topic": optical_flow_input_topic}
        elif modality == "gnss":
            source_parameters = {"input_topic": gnss_input_topic}
        elif modality == "depth":
            source_parameters = {"input_topic": d435_depth_input_topic}
        elif modality == "color":
            source_parameters = {"input_topic": d435_color_input_topic}
        nodes.append(
            Node(
                package="uf_sensor_pipeline",
                executable="fault_injector",
                name=f"fault_injector_{modality}",
                parameters=[
                    config,
                    fault_parameters,
                    source_parameters,
                    {"use_sim_time": use_sim_time},
                ],
                output="screen",
                condition=(
                    IfCondition(enable_vision)
                    if modality in ("depth", "color")
                    else None
                ),
            )
        )
    nodes.append(
        Node(
            package="uf_sensor_pipeline",
            executable="sensor_contract_monitor",
            name="sensor_contract_monitor",
            parameters=[config, {"use_sim_time": use_sim_time}],
            output="screen",
            condition=IfCondition(enable_vision),
        )
    )
    nodes.append(
        Node(
            package="uf_sensor_pipeline",
            executable="sensor_contract_monitor",
            name="sensor_contract_monitor",
            parameters=[
                config,
                {
                    "use_sim_time": use_sim_time,
                    "active_modalities": [
                        "lidar", "imu", "gnss", "optical_flow"
                    ],
                },
            ],
            output="screen",
            condition=UnlessCondition(enable_vision),
        )
    )
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("enable_fcu_observation_bridge", default_value="false"),
        DeclareLaunchArgument("enable_vision", default_value="false"),
        DeclareLaunchArgument("enable_nmea_gnss", default_value="false"),
        DeclareLaunchArgument("nmea_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("nmea_strict_checksum", default_value="true"),
        DeclareLaunchArgument(
            "optical_flow_input_topic", default_value="/sim/optical_flow/rad"
        ),
        DeclareLaunchArgument(
            "gnss_input_topic", default_value="/mavros/global_position/raw/fix"
        ),
        DeclareLaunchArgument(
            "gnss_raw_input_topic", default_value="/mavros/gpsstatus/gps1/raw"
        ),
        DeclareLaunchArgument(
            "d435_color_input_topic",
            default_value="/front/d435i/color/image_raw",
        ),
        DeclareLaunchArgument(
            "d435_depth_input_topic",
            default_value="/front/d435i/aligned_depth_to_color/image_raw",
        ),
        DeclareLaunchArgument("d435_parent_frame", default_value="base_link"),
        DeclareLaunchArgument("d435_child_frame", default_value="d435i_link"),
        DeclareLaunchArgument("d435_x", default_value="0.20"),
        DeclareLaunchArgument("d435_y", default_value="0.0"),
        DeclareLaunchArgument("d435_z", default_value="0.02"),
        DeclareLaunchArgument("d435_roll", default_value="0.0"),
        DeclareLaunchArgument("d435_pitch", default_value="0.0"),
        DeclareLaunchArgument("d435_yaw", default_value="0.0"),
        DeclareLaunchArgument(
            "fcu_flow_input_topic",
            default_value="/mavros/optical_flow/raw/optical_flow",
        ),
        DeclareLaunchArgument("fcu_flow_rad_input_topic", default_value=""),
        DeclareLaunchArgument(
            "fcu_range_input_topic",
            default_value="/mavros/rangefinder/rangefinder",
        ),
        *nodes,
    ])
