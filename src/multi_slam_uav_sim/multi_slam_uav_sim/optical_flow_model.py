import bisect
import math
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class TrackingResult:
    dx_px: float
    dy_px: float
    quality: int
    detected_count: int
    tracked_count: int
    inlier_count: int
    median_fb_error_px: float
    median_residual_px: float
    grid_coverage: float


def ros_flu_gyro_to_sensor_frd(gyro_xyz):
    """Convert ROS base_link FLU angular velocity to the flow sensor FRD axes."""
    gx, gy, gz = gyro_xyz
    return float(gx), -float(gy), -float(gz)


def pixel_flow_to_radians(dx_px, dy_px, fx_px, fy_px):
    if fx_px <= 0.0 or fy_px <= 0.0:
        raise ValueError("camera focal lengths must be positive")
    return math.atan2(float(dx_px), float(fx_px)), math.atan2(float(dy_px), float(fy_px))


def should_publish_accumulated_flow(
    dx_px,
    dy_px,
    quality,
    integration_s,
    min_displacement_px,
    max_integration_s,
    publish_low_quality,
):
    """Release a periodic sample, retaining accumulation as an opt-in mode.

    A physical optical-flow sensor keeps reporting when the measured motion is
    zero or its quality is low.  ``publish_low_quality`` therefore selects that
    periodic stream semantics; quality controls factor admission downstream.
    """
    if not math.isfinite(float(integration_s)) or integration_s <= 0.0:
        return False
    if publish_low_quality:
        return True
    displacement_px = math.hypot(float(dx_px), float(dy_px))
    observable = int(quality) > 0 and displacement_px >= float(min_displacement_px)
    expired = integration_s >= float(max_integration_s)
    return observable or (expired and int(quality) > 0)


def rate_limit_ready(elapsed_s, minimum_period_s, tolerance_ratio=0.98):
    """Accept the nearest discrete camera frame to a requested sensor rate.

    Gazebo's nominal 30 Hz camera currently advances by 33 ms, so two frames
    span 66 ms rather than exactly 1/15 s.  A strict comparison would select
    every third frame and silently turn a requested 15 Hz sensor into 10 Hz.
    The small tolerance keeps the nearest frame without allowing one-frame
    (30 Hz) publication.
    """
    elapsed = float(elapsed_s)
    period = float(minimum_period_s)
    tolerance = float(tolerance_ratio)
    if (
        not math.isfinite(elapsed)
        or not math.isfinite(period)
        or not math.isfinite(tolerance)
        or elapsed < 0.0
        or period <= 0.0
        or not 0.0 < tolerance <= 1.0
    ):
        return False
    return elapsed >= period * tolerance


def gazebo_downward_image_flow_to_mavlink(dx_px, dy_px, fx_px, fy_px):
    """Convert the Gazebo downward-camera image axes to OPTICAL_FLOW_RAD.

    For the downward mount, rightward sensor motion moves ground features
    toward decreasing image columns. MAVLink FRD recovers right displacement
    as ``-integrated_x * distance``, so the image-column angular displacement
    must be preserved here. The image row and integrated_y directions agree.
    The matching horizontal gyro projection is applied by
    :func:`gazebo_downward_gyro_to_mavlink`.
    """
    image_x, image_y = pixel_flow_to_radians(dx_px, dy_px, fx_px, fy_px)
    return image_x, image_y


def gazebo_downward_gyro_to_mavlink(gyro_frd):
    """Apply the same horizontal-axis mount transform to gyro integrals."""
    gx, gy, gz = (float(value) for value in gyro_frd)
    return gx, gy, gz


def scale_mavlink_translation(raw_flow_rad, gyro_rad, scale):
    """Scale only the translational remainder of OPTICAL_FLOW_RAD fields."""
    scale = float(scale)
    if not math.isfinite(scale) or scale <= 0.0:
        raise ValueError("optical-flow translation scale must be positive")
    return tuple(
        float(gyro) + scale * (float(raw) - float(gyro))
        for raw, gyro in zip(raw_flow_rad, gyro_rad)
    )


def compensated_planar_velocity(raw_flow_rad, gyro_rad, integration_s, distance_m):
    """Return sensor-FRD planar velocity from MAVLink OPTICAL_FLOW_RAD fields."""
    if integration_s <= 0.0 or distance_m <= 0.0:
        return float("nan"), float("nan")
    flow_x, flow_y = raw_flow_rad
    gyro_x, gyro_y = gyro_rad
    translational_x = (float(flow_x) - float(gyro_x)) / float(integration_s)
    translational_y = (float(flow_y) - float(gyro_y)) / float(integration_s)
    return translational_y * float(distance_m), -translational_x * float(distance_m)


def quaternion_rotate_inverse(quaternion_xyzw, vector_xyz):
    x, y, z, w = (float(value) for value in quaternion_xyzw)
    vector = np.asarray(vector_xyz, dtype=float)
    q_vector = np.asarray([x, y, z], dtype=float)
    # Unit-quaternion inverse rotation without constructing a matrix.
    return vector - 2.0 * w * np.cross(q_vector, vector) + 2.0 * np.cross(
        q_vector, np.cross(q_vector, vector)
    )


def quaternion_rotate(quaternion_xyzw, vector_xyz):
    x, y, z, w = (float(value) for value in quaternion_xyzw)
    vector = np.asarray(vector_xyz, dtype=float)
    q_vector = np.asarray([x, y, z], dtype=float)
    return vector + 2.0 * w * np.cross(q_vector, vector) + 2.0 * np.cross(
        q_vector, np.cross(q_vector, vector)
    )


def sensor_velocity_frd(world_velocity, body_to_world_quaternion, gyro_frd, lever_arm_frd):
    body_flu = quaternion_rotate_inverse(body_to_world_quaternion, world_velocity)
    body_frd = np.asarray([body_flu[0], -body_flu[1], -body_flu[2]], dtype=float)
    lever_velocity = np.cross(
        np.asarray(gyro_frd, dtype=float), np.asarray(lever_arm_frd, dtype=float)
    )
    return tuple(float(value) for value in body_frd + lever_velocity)


def synthesize_optical_flow(velocity_frd, gyro_integral_frd, integration_s, distance_m):
    """Synthesize MAVLink raw integrated flow from sensor motion and gyro."""
    if integration_s <= 0.0 or distance_m <= 0.0:
        return None
    vx, vy, _ = (float(value) for value in velocity_frd)
    gx, gy, _ = (float(value) for value in gyro_integral_frd)
    translational_x = -vy / float(distance_m) * float(integration_s)
    translational_y = vx / float(distance_m) * float(integration_s)
    return translational_x + gx, translational_y + gy


def sensor_displacement_frd(start_pose, end_pose, lever_arm_frd):
    start_position, start_quaternion = start_pose
    end_position, end_quaternion = end_pose
    lever_frd = np.asarray(lever_arm_frd, dtype=float)
    lever_flu = np.asarray([lever_frd[0], -lever_frd[1], -lever_frd[2]])
    start_sensor_world = np.asarray(start_position, dtype=float) + quaternion_rotate(
        start_quaternion, lever_flu
    )
    end_sensor_world = np.asarray(end_position, dtype=float) + quaternion_rotate(
        end_quaternion, lever_flu
    )
    delta_body_flu = quaternion_rotate_inverse(
        end_quaternion, end_sensor_world - start_sensor_world
    )
    return (
        float(delta_body_flu[0]),
        float(-delta_body_flu[1]),
        float(-delta_body_flu[2]),
    )


def synthesize_optical_flow_from_displacement(displacement_frd, gyro_integral_frd, distance_m):
    if distance_m <= 0.0:
        return None
    dx, dy, _ = (float(value) for value in displacement_frd)
    gx, gy, _ = (float(value) for value in gyro_integral_frd)
    return -dy / float(distance_m) + gx, dx / float(distance_m) + gy


def _sample_at(samples, timestamp_s, max_gap_s):
    times = [sample[0] for sample in samples]
    index = bisect.bisect_left(times, timestamp_s)
    if index < len(samples) and abs(times[index] - timestamp_s) <= 1.0e-9:
        return np.asarray(samples[index][1:4], dtype=float)
    if index == 0:
        if times[0] - timestamp_s <= max_gap_s:
            return np.asarray(samples[0][1:4], dtype=float)
        return None
    if index >= len(samples):
        if timestamp_s - times[-1] <= max_gap_s:
            return np.asarray(samples[-1][1:4], dtype=float)
        return None
    before = samples[index - 1]
    after = samples[index]
    gap = after[0] - before[0]
    if gap <= 0.0 or gap > max_gap_s:
        return None
    ratio = (timestamp_s - before[0]) / gap
    return np.asarray(before[1:4], dtype=float) + ratio * (
        np.asarray(after[1:4], dtype=float) - np.asarray(before[1:4], dtype=float)
    )


def integrate_gyro(samples, start_s, end_s, max_gap_s=0.12):
    """Trapezoid-integrate timestamped FRD gyro samples over one flow exposure."""
    if end_s <= start_s or not samples:
        return None
    ordered = sorted(samples, key=lambda sample: sample[0])
    start_value = _sample_at(ordered, start_s, max_gap_s)
    end_value = _sample_at(ordered, end_s, max_gap_s)
    if start_value is None or end_value is None:
        return None

    points = [(start_s, start_value)]
    points.extend(
        (sample[0], np.asarray(sample[1:4], dtype=float))
        for sample in ordered
        if start_s < sample[0] < end_s
    )
    points.append((end_s, end_value))
    integral = np.zeros(3, dtype=float)
    for before, after in zip(points[:-1], points[1:]):
        dt = after[0] - before[0]
        if dt <= 0.0 or dt > max_gap_s:
            return None
        integral += 0.5 * (before[1] + after[1]) * dt
    return tuple(float(value) for value in integral)


def integrate_preferred_gyro(
    primary_samples,
    fallback_samples,
    start_s,
    end_s,
    max_gap_s=0.12,
):
    """Use the source that actually covers the exposure, not its latest stamp."""
    primary = integrate_gyro(
        primary_samples, start_s, end_s, max_gap_s=max_gap_s
    )
    if primary is not None:
        return primary, "primary"
    fallback = integrate_gyro(
        fallback_samples, start_s, end_s, max_gap_s=max_gap_s
    )
    if fallback is not None:
        return fallback, "fallback"
    return None, "unavailable"


def _grid_coverage(points, width, height, cols=4, rows=3):
    if len(points) == 0 or width <= 0 or height <= 0:
        return 0.0
    occupied = set()
    for point in points:
        x = min(cols - 1, max(0, int(float(point[0]) / width * cols)))
        y = min(rows - 1, max(0, int(float(point[1]) / height * rows)))
        occupied.add((x, y))
    return len(occupied) / float(cols * rows)


def track_lk_flow(
    previous,
    current,
    max_corners=160,
    quality_level=0.01,
    min_feature_distance_px=7.0,
    fb_threshold_px=1.0,
    max_track_error=30.0,
    max_displacement_px=40.0,
    min_inliers=8,
):
    """Estimate aggregate image displacement with pyramidal LK and robust checks."""
    if previous is None or current is None or previous.shape != current.shape:
        return TrackingResult(0.0, 0.0, 0, 0, 0, 0, float("nan"), float("nan"), 0.0)
    if previous.ndim != 2 or current.ndim != 2:
        raise ValueError("LK tracker expects mono images")

    points = cv2.goodFeaturesToTrack(
        previous,
        maxCorners=max(8, int(max_corners)),
        qualityLevel=max(1.0e-4, float(quality_level)),
        minDistance=max(1.0, float(min_feature_distance_px)),
        blockSize=7,
        useHarrisDetector=False,
    )
    if points is None or len(points) == 0:
        return TrackingResult(0.0, 0.0, 0, 0, 0, 0, float("nan"), float("nan"), 0.0)

    lk_params = dict(
        winSize=(21, 21),
        maxLevel=3,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
    )
    forward, status_forward, error_forward = cv2.calcOpticalFlowPyrLK(
        previous, current, points, None, **lk_params
    )
    if forward is None:
        return TrackingResult(0.0, 0.0, 0, len(points), 0, 0, float("nan"), float("nan"), 0.0)
    backward, status_backward, _ = cv2.calcOpticalFlowPyrLK(
        current, previous, forward, None, **lk_params
    )
    if backward is None:
        return TrackingResult(0.0, 0.0, 0, len(points), 0, 0, float("nan"), float("nan"), 0.0)

    p0 = points.reshape(-1, 2)
    p1 = forward.reshape(-1, 2)
    p0_back = backward.reshape(-1, 2)
    status = status_forward.reshape(-1).astype(bool) & status_backward.reshape(-1).astype(bool)
    fb_error = np.linalg.norm(p0_back - p0, axis=1)
    track_error = error_forward.reshape(-1)
    displacement = p1 - p0
    displacement_norm = np.linalg.norm(displacement, axis=1)
    finite = np.isfinite(displacement).all(axis=1) & np.isfinite(fb_error) & np.isfinite(track_error)
    keep = (
        status
        & finite
        & (fb_error <= float(fb_threshold_px))
        & (track_error <= float(max_track_error))
        & (displacement_norm <= float(max_displacement_px))
    )
    tracked = int(np.count_nonzero(keep))
    if tracked == 0:
        return TrackingResult(0.0, 0.0, 0, len(points), 0, 0, float("nan"), float("nan"), 0.0)

    kept_displacement = displacement[keep]
    median = np.median(kept_displacement, axis=0)
    residual = np.linalg.norm(kept_displacement - median, axis=1)
    residual_threshold = max(0.75, float(np.median(residual)) * 2.5 + 0.25)
    inlier_mask = residual <= residual_threshold
    inliers = kept_displacement[inlier_mask]
    inlier_points = p0[keep][inlier_mask]
    inlier_count = len(inliers)
    if inlier_count < int(min_inliers):
        return TrackingResult(
            0.0,
            0.0,
            0,
            len(points),
            tracked,
            inlier_count,
            float(np.median(fb_error[keep])),
            float(np.median(residual)) if len(residual) else float("nan"),
            _grid_coverage(inlier_points, previous.shape[1], previous.shape[0]),
        )

    median = np.median(inliers, axis=0)
    median_fb = float(np.median(fb_error[keep][inlier_mask]))
    median_residual = float(np.median(np.linalg.norm(inliers - median, axis=1)))
    coverage = _grid_coverage(inlier_points, previous.shape[1], previous.shape[0])
    count_score = min(1.0, inlier_count / 80.0)
    survival_score = min(1.0, inlier_count / max(1.0, float(len(points))))
    fb_score = max(0.0, 1.0 - median_fb / max(1.0e-6, float(fb_threshold_px)))
    coherence_score = max(0.0, 1.0 - median_residual / 2.5)
    quality = int(round(255.0 * (
        0.30 * count_score
        + 0.20 * survival_score
        + 0.20 * fb_score
        + 0.20 * coherence_score
        + 0.10 * coverage
    )))
    return TrackingResult(
        float(median[0]),
        float(median[1]),
        max(0, min(255, quality)),
        len(points),
        tracked,
        inlier_count,
        median_fb,
        median_residual,
        coverage,
    )
