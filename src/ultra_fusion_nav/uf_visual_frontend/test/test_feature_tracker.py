import cv2
import numpy as np
import unittest

from uf_visual_frontend.feature_tracker import (
    RgbdFeatureTracker,
    grid_uniformity,
    pnp_observability,
)
from uf_visual_frontend.rgbd_feature_frontend import (
    CADENCE_PERIOD_S,
    depth_variance,
    inverse_depth_variance,
    visual_candidate_quality,
)


def synthetic_image(shift_x=0):
    image = np.zeros((240, 320), np.uint8)
    for y in range(30, 220, 30):
        for x in range(30, 300, 30):
            cv2.rectangle(image, (x + shift_x - 3, y - 3),
                          (x + shift_x + 3, y + 3), 255, -1)
    return image


def synthetic_depth_with_geometry():
    y, x = np.indices((240, 320))
    return (1400 + ((x // 40 + y // 30) % 5) * 350).astype(np.uint16)


def test_exact_pair_tracking_depth_and_pnp_evidence():
    tracker = RgbdFeatureTracker(max_features=120, fb_threshold_px=0.5)
    depth = np.full((240, 320), 2000, np.uint16)
    camera = np.asarray(
        [[250.0, 0.0, 160.0], [0.0, 250.0, 120.0], [0.0, 0.0, 1.0]])
    assert tracker.process(synthetic_image(), depth, camera) is None
    result = tracker.process(synthetic_image(2), depth, camera)
    assert len(result.current_pixels) >= 30
    assert np.mean(result.depth_valid) == 1.0
    assert np.median(result.forward_backward_error) < 0.1
    assert result.rotation is not None


def test_grid_uniformity_is_bounded_and_spatially_sensitive():
    spread = np.asarray([[x, y] for x in range(20, 320, 40)
                        for y in range(15, 240, 30)])
    clustered = np.asarray([[10 + x, 10 + y]
                           for x in range(4) for y in range(4)])
    assert 0.0 <= grid_uniformity(spread, 320, 240) <= 1.0
    assert grid_uniformity(
        spread, 320, 240) > grid_uniformity(
        clustered, 320, 240)


def test_candidate_quality_uses_rgbd_health_without_requiring_motion():
    tracker = RgbdFeatureTracker(max_features=120, fb_threshold_px=0.5)
    depth = synthetic_depth_with_geometry()
    camera = np.asarray(
        [[250.0, 0.0, 160.0], [0.0, 250.0, 120.0], [0.0, 0.0, 1.0]])
    assert tracker.process(synthetic_image(), depth, camera) is None
    informative = tracker.process(synthetic_image(2), depth, camera)
    quality = visual_candidate_quality(informative, 320, 240)
    assert quality.valid
    assert quality.geometric_tracks >= 20
    assert quality.median_parallax_px > 0.15

    tracker = RgbdFeatureTracker(max_features=120, fb_threshold_px=0.5)
    assert tracker.process(synthetic_image(), depth, camera) is None
    stationary = tracker.process(synthetic_image(), depth, camera)
    quality = visual_candidate_quality(
        stationary, 320, 240, minimum_parallax_px=1000.0)
    assert quality.valid
    assert quality.reason == "quality_valid"
    assert quality.median_parallax_px < 1.0e-6


def test_balanced_cadence_scan_is_strictly_ordered():
    periods = [
        CADENCE_PERIOD_S[name]
        for name in ("conservative", "balanced_light", "balanced", "balanced_plus", "dense")
    ]
    assert periods == sorted(periods, reverse=True)


def test_sparse_neighborhood_depth_rejects_range_and_edge_outliers():
    tracker = RgbdFeatureTracker(
        minimum_depth_m=0.30,
        maximum_depth_m=6.0,
        depth_neighborhood_radius_px=1,
        depth_minimum_support=3,
    )
    depth = np.zeros((30, 30), np.uint16)
    depth[9:12, 9:12] = 2000
    depth[10, 10] = 0
    depth[9, 9] = 5000
    depth[19:22, 19:22] = 200
    depth[24:27, 24:27] = 6500
    samples, valid, sigma = tracker._sample_depth(
        depth,
        np.asarray([[10.0, 10.0], [20.0, 20.0], [25.0, 25.0]]),
    )
    assert valid.tolist() == [True, False, False]
    assert abs(float(samples[0]) - 2.0) < 1.0e-6
    assert float(sigma[0]) >= 0.0049


def test_stationary_rgbd_tracks_remain_admissible_across_candidates():
    tracker = RgbdFeatureTracker(max_features=120, fb_threshold_px=0.5)
    depth = synthetic_depth_with_geometry()
    camera = np.asarray(
        [[250.0, 0.0, 160.0], [0.0, 250.0, 120.0], [0.0, 0.0, 1.0]])
    first = synthetic_image()
    assert tracker.process(first, depth, camera) is None
    stationary = tracker.process(first, depth, camera)
    assert visual_candidate_quality(stationary, 320, 240).valid
    next_stationary = tracker.process(first, depth, camera)
    assert visual_candidate_quality(next_stationary, 320, 240).valid


def test_per_track_depth_sigma_propagates_to_inverse_depth_variance():
    nominal = inverse_depth_variance(2.0, 0.005, 0.015)
    noisy = inverse_depth_variance(2.0, 0.20, 0.015)
    assert nominal > 0.0
    assert noisy > nominal


def test_current_depth_and_metric_variance_are_preserved_per_track():
    tracker = RgbdFeatureTracker(max_features=120, fb_threshold_px=0.5)
    camera = np.asarray(
        [[250.0, 0.0, 160.0], [0.0, 250.0, 120.0], [0.0, 0.0, 1.0]])
    previous_depth = np.full((240, 320), 2000, np.uint16)
    current_depth = np.full((240, 320), 2100, np.uint16)
    assert tracker.process(synthetic_image(), previous_depth, camera) is None
    result = tracker.process(synthetic_image(2), current_depth, camera)

    assert np.all(result.current_depth_valid)
    np.testing.assert_allclose(result.current_depth_m, 2.1, atol=1.0e-6)
    assert depth_variance(2.0, 0.005, 0.015) > 0.0
    assert depth_variance(2.0, 0.20, 0.015) > depth_variance(
        2.0, 0.005, 0.015
    )


def test_pnp_observability_rejects_rank_deficient_geometry_without_motion_gate():
    varied = np.asarray([
        [x, y, 1.5 + 0.4 * ((column + row) % 4)]
        for column, x in enumerate(np.linspace(-1.0, 1.0, 8))
        for row, y in enumerate(np.linspace(-0.7, 0.7, 6))
    ])
    line = np.asarray([
        [x, 0.0, 2.0] for x in np.linspace(-1.0, 1.0, 48)
    ])
    rank, condition = pnp_observability(varied, np.eye(3), np.zeros(3))
    assert rank == 6
    assert condition < 500.0
    rank, condition = pnp_observability(line, np.eye(3), np.zeros(3))
    assert rank < 6
    assert np.isinf(condition)


def test_feature_pool_excludes_out_of_range_depth_without_cropping_rgb():
    tracker = RgbdFeatureTracker(max_features=120, fb_threshold_px=0.5)
    image = synthetic_image()
    depth = np.full((240, 320), 7000, np.uint16)
    depth[:, :160] = 2000
    camera = np.asarray(
        [[250.0, 0.0, 160.0], [0.0, 250.0, 120.0], [0.0, 0.0, 1.0]])

    assert tracker.process(image, depth, camera) is None
    assert len(tracker.points) > 20
    assert np.all(tracker.points[:, 0] < 160.0)

    result = tracker.process(image, depth, camera)
    assert result is not None
    assert np.all(result.previous_pixels[:, 0] < 160.0)
    assert np.all(result.depth_valid)


def test_feature_pool_reseeds_when_valid_depth_region_moves():
    tracker = RgbdFeatureTracker(max_features=120, fb_threshold_px=0.5)
    image = synthetic_image()
    left_depth = np.full((240, 320), 7000, np.uint16)
    left_depth[:, :160] = 2000
    right_depth = np.full((240, 320), 7000, np.uint16)
    right_depth[:, 160:] = 2000
    camera = np.asarray(
        [[250.0, 0.0, 160.0], [0.0, 250.0, 120.0], [0.0, 0.0, 1.0]])

    assert tracker.process(image, left_depth, camera) is None
    tracker.process(image, right_depth, camera)

    assert len(tracker.points) > 20
    assert np.all(tracker.points[:, 0] >= 160.0)


class FeatureTrackerUnittest(unittest.TestCase):
    def test_exact_pair(self):
        test_exact_pair_tracking_depth_and_pnp_evidence()

    def test_grid(self):
        test_grid_uniformity_is_bounded_and_spatially_sensitive()

    def test_candidate_quality(self):
        test_candidate_quality_uses_rgbd_health_without_requiring_motion()

    def test_balanced_cadence_scan(self):
        test_balanced_cadence_scan_is_strictly_ordered()

    def test_sparse_neighborhood_depth(self):
        test_sparse_neighborhood_depth_rejects_range_and_edge_outliers()

    def test_stationary_rgbd_admission(self):
        test_stationary_rgbd_tracks_remain_admissible_across_candidates()

    def test_inverse_depth_variance(self):
        test_per_track_depth_sigma_propagates_to_inverse_depth_variance()

    def test_current_depth_geometry(self):
        test_current_depth_and_metric_variance_are_preserved_per_track()

    def test_pnp_observability(self):
        test_pnp_observability_rejects_rank_deficient_geometry_without_motion_gate()

    def test_out_of_range_depth_feature_mask(self):
        test_feature_pool_excludes_out_of_range_depth_without_cropping_rgb()

    def test_depth_feature_mask_reseed(self):
        test_feature_pool_reseeds_when_valid_depth_region_moves()
