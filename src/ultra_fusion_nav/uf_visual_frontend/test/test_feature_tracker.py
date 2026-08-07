import cv2
import numpy as np
import unittest

from uf_visual_frontend.feature_tracker import RgbdFeatureTracker, grid_uniformity
from uf_visual_frontend.rgbd_feature_frontend import visual_candidate_quality


def synthetic_image(shift_x=0):
    image = np.zeros((240, 320), np.uint8)
    for y in range(30, 220, 30):
        for x in range(30, 300, 30):
            cv2.rectangle(image, (x + shift_x - 3, y - 3),
                          (x + shift_x + 3, y + 3), 255, -1)
    return image


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


def test_candidate_quality_requires_information_not_only_a_tracked_frame():
    tracker = RgbdFeatureTracker(max_features=120, fb_threshold_px=0.5)
    depth = np.full((240, 320), 2000, np.uint16)
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
    quality = visual_candidate_quality(stationary, 320, 240)
    assert not quality.valid
    assert quality.reason == "insufficient_parallax"


class FeatureTrackerUnittest(unittest.TestCase):
    def test_exact_pair(self):
        test_exact_pair_tracking_depth_and_pnp_evidence()

    def test_grid(self):
        test_grid_uniformity_is_bounded_and_spatially_sensitive()

    def test_candidate_quality(self):
        test_candidate_quality_requires_information_not_only_a_tracked_frame()
