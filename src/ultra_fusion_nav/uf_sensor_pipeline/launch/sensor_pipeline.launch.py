import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_config = get_package_share_directory("uf_sensor_pipeline") + "/config/sim_sensor_config.yaml"
    config = LaunchConfiguration("config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_fcu_observation_bridge = LaunchConfiguration("enable_fcu_observation_bridge")
    enable_vision = LaunchConfiguration("enable_vision")
    enable_nmea_gnss = LaunchConfiguration("enable_nmea_gnss")
    optical_flow_input_topic = LaunchConfiguration("optical_flow_input_topic")
    gnss_input_topic = LaunchConfiguration("gnss_input_topic")
    gnss_algorithm_rate_hz = LaunchConfiguration("gnss_algorithm_rate_hz")
    d435_color_input_topic = LaunchConfiguration("d435_color_input_topic")
    d435_depth_input_topic = LaunchConfiguration("d435_depth_input_topic")
    fcu_flow_input_topic = LaunchConfiguration("fcu_flow_input_topic")
    fcu_flow_rad_input_topic = LaunchConfiguration("fcu_flow_rad_input_topic")
    fcu_range_input_topic = LaunchConfiguration("fcu_range_input_topic")
    active_modalities = ParameterValue(
        LaunchConfiguration("active_modalities"), value_type=List[str]
    )
    enable_fault_injection = LaunchConfiguration("enable_fault_injection")
    enable_gnss = LaunchConfiguration("enable_gnss")
    enable_lidar = LaunchConfiguration("enable_lidar")
    publish_d435i_mount_tf = LaunchConfiguration("publish_d435i_mount_tf")
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
            condition=IfCondition(PythonExpression([
                "'", enable_gnss, "' == 'true' and '",
                enable_nmea_gnss, "' == 'true'"
            ])),
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
                    "output_rate_hz": gnss_algorithm_rate_hz,
                },
            ],
            output="screen",
            condition=IfCondition(PythonExpression([
                "'", enable_gnss, "' == 'true' and '",
                enable_nmea_gnss, "' == 'false'"
            ])),
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
            package="uf_pointcloud_body_filter_cpp",
            executable="pointcloud_body_filter_cpp",
            name="pointcloud_body_filter",
            parameters=[config, {"use_sim_time": use_sim_time}],
            output="screen",
            condition=IfCondition(enable_lidar),
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
            condition=IfCondition(PythonExpression([
                "'", enable_vision, "' == 'true' and '",
                publish_d435i_mount_tf, "' == 'true'"
            ])),
        ),
    ]
    for modality in ("lidar", "imu", "gnss", "optical_flow", "depth", "color"):
        fault_parameters = scheduled_fault if modality == scheduled_fault_modality else {}
        source_parameters = {}
        if modality == "optical_flow":
            source_parameters = {"input_topic": optical_flow_input_topic}
        elif modality == "gnss":
            source_parameters = {
                "input_topic": gnss_input_topic,
                "output_topic": "/sensors/gnss/fix_unthrottled",
            }
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
                condition=IfCondition(enable_fault_injection),
            )
        )
    nodes.append(
        Node(
            package="uf_sensor_pipeline",
            executable="fault_injector",
            name="fault_injector_gnss",
            parameters=[
                config,
                {"input_topic": gnss_input_topic, "use_sim_time": use_sim_time},
            ],
            output="screen",
            condition=IfCondition(PythonExpression([
                "'", enable_fault_injection, "' == 'true' and '",
                enable_nmea_gnss, "' == 'true'"
            ])),
        )
    )
    nodes.append(
        Node(
            package="uf_sensor_pipeline",
            executable="sensor_contract_monitor",
            name="sensor_contract_monitor",
            parameters=[config, {"use_sim_time": use_sim_time}],
            output="screen",
            condition=IfCondition(PythonExpression([
                "'", enable_fault_injection, "' == 'true' and '",
                enable_vision, "' == 'true'"
            ])),
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
                    "active_modalities": active_modalities,
                },
            ],
            output="screen",
            condition=UnlessCondition(enable_vision),
        )
    )
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "enable_fault_injection",
            default_value="true" if scheduled_fault_modality else "false",
            description="Start per-modality fault injectors (test/robustness only)",
        ),
        DeclareLaunchArgument("enable_lidar", default_value="true"),
        DeclareLaunchArgument("enable_gnss", default_value="true"),
        DeclareLaunchArgument("enable_fcu_observation_bridge", default_value="false"),
        DeclareLaunchArgument("enable_vision", default_value="false"),
        DeclareLaunchArgument(
            "publish_d435i_mount_tf",
            default_value="true",
            description=(
                "Publish the simulation D435i mount TF. Real hardware launch "
                "must disable this because its calibrated closure is owned by the "
                "hardware geometry contract."
            ),
        ),
        DeclareLaunchArgument("enable_nmea_gnss", default_value="false"),
        DeclareLaunchArgument(
            "active_modalities",
            default_value="[lidar, imu, gnss, optical_flow]",
            description="Modalities expected by the non-vision sensor contract",
        ),
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
            "gnss_algorithm_rate_hz",
            default_value="5.0",
            description="Paired fix/raw rate exposed to the companion estimator (5 Hz)",
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
        Node(
            package="uf_sensor_pipeline",
            executable="sensor_relay_manager",
            name="sensor_relay_manager",
            parameters=[config, {
                "use_sim_time": use_sim_time,
                "active_modalities": active_modalities,
                "lidar_input_topic": "/sensors/lidar/points_body_filtered",
                "lidar_output_topic": "/sensors/lidar/points",
                "imu_input_topic": "/livox/imu",
                "imu_output_topic": "/sensors/imu",
                "gnss_input_topic": gnss_input_topic,
                "gnss_output_topic": "/sensors/gnss/fix_unthrottled",
                "optical_flow_input_topic": optical_flow_input_topic,
                "optical_flow_output_topic": "/sensors/optical_flow/rad",
                "depth_input_topic": d435_depth_input_topic,
                "depth_output_topic": "/sensors/rgbd/depth",
                "color_input_topic": d435_color_input_topic,
                "color_output_topic": "/sensors/rgbd/color",
            }],
            output="screen",
            condition=UnlessCondition(enable_fault_injection),
        ),
        *nodes,
    ])
