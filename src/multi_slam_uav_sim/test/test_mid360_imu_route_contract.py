from pathlib import Path
import xml.etree.ElementTree as ET

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
    assert config["fault_injector_imu"]["ros__parameters"]["imu_acceleration_scale"] == 9.80665
