from pathlib import Path
import math
import xml.etree.ElementTree as ET

import numpy as np
import yaml


REPO = Path(__file__).resolve().parents[2]


def test_integrated_mid360_model_owns_independent_200_hz_imu():
    model = ET.parse(REPO / "multi_slam_uav_sim/models/iris_apm_rgbd/model.sdf")
    imu = model.find(".//link[@name='mid360_link']/sensor[@name='mid360_sim_imu']")
    assert imu is not None
    assert imu.attrib["type"] == "imu"
    assert imu.findtext("topic") == "/mid360/imu"
    assert float(imu.findtext("update_rate")) == 200.0


def test_fastlio_and_backend_share_mid360_imu_route():
    config_path = REPO / "ultra_fusion_nav/uf_sensor_pipeline/config/sim_sensor_config.yaml"
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    assert config["fcu_observation_bridge"]["ros__parameters"]["imu_input_topic"] == "/livox/imu"
    imu_injector = config["fault_injector_imu"]["ros__parameters"]
    assert imu_injector["input_topic"] == "/livox/imu"
    assert imu_injector["output_topic"] == "/sensors/imu"
    assert imu_injector["restamp_output"] is False

    for name in (
        "fast_lio_sim_mid360.yaml",
        "fast_lio_sim_mid360_pointcloud.yaml",
        "fast_lio_sim_mid360_filtered_pointcloud.yaml",
    ):
        fastlio = yaml.safe_load(
            (REPO / "multi_slam_uav_sim/config" / name).read_text(encoding="utf-8")
        )
        assert fastlio["/**"]["ros__parameters"]["common"]["imu_topic"] == "/livox/imu"


def test_simulation_estimator_configuration_does_not_subscribe_fcu_imu():
    forbidden = ("/mavros/imu/data_raw", "/mavros/imu/data")
    files = (
        REPO / "ultra_fusion_nav/uf_sensor_pipeline/config/sim_sensor_config.yaml",
        REPO / "ultra_fusion_nav/uf_sensor_pipeline/config/gps_flow_externalnav.yaml",
        REPO / "ultra_fusion_nav/uf_sensor_pipeline/uf_sensor_pipeline/fcu_observation_bridge.py",
        REPO / "ultra_fusion_nav/uf_sensor_pipeline/uf_sensor_pipeline/gps_flow_fusion_node.py",
        REPO / "mid360_sim_bridge_cpp/src/gz_livox_bridge_node.cpp",
    )
    for path in files:
        text = path.read_text(encoding="utf-8")
        assert all(topic not in text for topic in forbidden), path


def test_simulator_imu_unit_scale_is_identity():
    config = yaml.safe_load(
        (REPO / "ultra_fusion_nav/uf_sensor_pipeline/config/sim_sensor_config.yaml").read_text(
            encoding="utf-8"
        )
    )
    assert config["fault_injector_imu"]["ros__parameters"]["imu_acceleration_scale"] == 1.0


def test_real_mid360_imu_unit_overlay_uses_g_to_si_scale():
    config = yaml.safe_load(
        (REPO / "ultra_fusion_nav/uf_sensor_pipeline/config/real_mid360_imu_units.yaml").read_text(
            encoding="utf-8"
        )
    )
    expected_uri = "package://uf_sensor_pipeline/config/real_mid360s_d435i_geometry.yaml"
    assert config["fcu_observation_bridge"]["ros__parameters"]["imu_input_topic"] == "/sensors/imu"
    assert config["sensor_relay_manager"]["ros__parameters"]["geometry_contract_file"] == expected_uri
    assert config["fault_injector_imu"]["ros__parameters"]["geometry_contract_file"] == expected_uri


def test_real_mid360_profile_uses_one_body_from_sensor_mount_contract():
    config = yaml.safe_load(
        (REPO / "ultra_fusion_nav/uf_sensor_pipeline/config/real_mid360s_d435i_geometry.yaml").read_text(
            encoding="utf-8"
        )
    )
    transform = config["transforms"]["body_lidar"]
    expected = np.asarray(
        [
            math.cos(math.radians(15.0)), 0.0, math.sin(math.radians(15.0)),
            0.0, 1.0, 0.0,
            -math.sin(math.radians(15.0)), 0.0, math.cos(math.radians(15.0)),
        ]
    )

    assert config["imu"]["acceleration_scale"] == 9.80665
    assert config["frames"]["body"] == "base_link"
    assert np.allclose(transform["rotation_matrix"], expected, atol=1.0e-10)
    assert transform["translation_m"] is None
    assert config["body_envelope"]["enabled"] is False
    assert config["body_envelope"]["primitives"] == []
    assert config["topics"]["lidar_raw"] == "/livox/lidar"
    assert config["topics"]["imu_raw"] == "/livox/imu"


def test_real_mount_rotation_maps_level_static_gravity_to_body_up():
    config = yaml.safe_load(
        (REPO / "ultra_fusion_nav/uf_sensor_pipeline/config/real_mid360s_d435i_geometry.yaml").read_text(
            encoding="utf-8"
        )
    )
    rotation = np.asarray(
        config["transforms"]["body_lidar"]["rotation_matrix"]
    ).reshape(3, 3)
    angle = math.radians(15.0)
    gravity_mid360_g = np.asarray([-math.sin(angle), 0.0, math.cos(angle)])

    gravity_body_mps2 = rotation @ gravity_mid360_g * 9.80665

    assert np.allclose(gravity_body_mps2, [0.0, 0.0, 9.80665], atol=1.0e-6)
    assert np.allclose(rotation.T @ rotation, np.eye(3), atol=1.0e-10)
    assert np.isclose(np.linalg.det(rotation), 1.0, atol=1.0e-10)


def test_real_launch_keeps_mid360_imu_on_canonical_raw_topic():
    source = (
        REPO / "mid360_reliable_mapper/launch/real_mid360_fastlio.launch.py"
    ).read_text(encoding="utf-8")

    assert "/livox/lidar_imu" not in source
    assert '("/livox/imu",' not in source


def test_simulation_mount_and_imu_contract_remain_unchanged():
    config = yaml.safe_load(
        (REPO / "ultra_fusion_nav/uf_sensor_pipeline/config/sim_sensor_config.yaml").read_text(
            encoding="utf-8"
        )
    )
    relay = config.get("sensor_relay_manager", {}).get("ros__parameters", {})
    body_filter = config["pointcloud_body_filter"]["ros__parameters"]

    assert relay.get("mid360_to_body_rotation", [1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                                                    0.0, 0.0, 1.0]) == [
        1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
    ]
    assert relay.get("imu_output_frame_id", "") == ""
    assert body_filter["lidar_to_body_rotation"] == [
        0.9659258263, 0.0, 0.2588190451,
        0.0, 1.0, 0.0,
        -0.2588190451, 0.0, 0.9659258263,
    ]


def test_fast_lio_internal_lidar_imu_extrinsic_stays_independent():
    config = yaml.safe_load(
        (REPO / "mid360_reliable_mapper/config/fast_lio_real_mid360.yaml").read_text(
            encoding="utf-8"
        )
    )["/**"]["ros__parameters"]["mapping"]

    assert config["extrinsic_T"] == [-0.011, -0.02329, 0.04412]
    assert config["extrinsic_R"] == [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]


def test_real_sensor_pipeline_uses_contract_and_suppresses_unmeasured_camera_tf():
    source = (
        REPO / "ultra_fusion_nav/uf_sensor_pipeline/launch/real_mid360s_sensor_pipeline.launch.py"
    ).read_text(encoding="utf-8")

    assert "real_mid360_imu_units.yaml" in source
    assert '"publish_d435i_mount_tf": "false"' in source
    assert 'default_value="[imu]"' in source
    assert "/livox/lidar_imu" not in source


def test_default_real_mount_tf_is_disabled_until_translation_is_measured():
    config = yaml.safe_load(
        (REPO / "mid360_reliable_mapper/config/mid360_mount_extrinsic.yaml").read_text(
            encoding="utf-8"
        )
    )

    assert config["enabled"] is False
    assert config["translation_status"] == "unmeasured"
    assert config["translation_m"] is None
    assert config["rotation_deg"]["pitch"] == 15.0
