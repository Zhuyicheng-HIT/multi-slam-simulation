import pytest

from uf_global_pose_graph.configuration import load_pipeline_config


def test_yaml_configuration_converts_degrees_and_nested_limits(tmp_path):
    path = tmp_path / "config.yaml"
    path.write_text("""
graph:
  sequential_translation_sigma_m: 0.04
  sequential_rotation_sigma_deg: 2.0
  loop_translation_sigma_floor_m: 0.05
  loop_translation_sigma_ceiling_m: 0.18
  loop_rotation_sigma_deg: 4.0
correlation:
  endpoint_index_radius: 3
  translation_similarity_m: 0.30
  rotation_similarity_deg: 12.0
optimizer:
  robust_phi: 16.0
  minimum_loop_weight: 0.08
  maximum_function_evaluations: 150
  maximum_translation_correction_m: 1.0
  maximum_rotation_correction_deg: 30.0
  maximum_sequential_translation_strain_m: 0.20
  maximum_sequential_rotation_strain_deg: 10.0
""", encoding="utf-8")

    config = load_pipeline_config(path)

    assert config.sequential_translation_sigma_m == 0.04
    assert config.sequential_rotation_sigma_rad == pytest.approx(0.034906585)
    assert config.correlation.endpoint_index_radius == 3
    assert config.optimizer.maximum_rotation_correction_rad == pytest.approx(
        0.523598776)
