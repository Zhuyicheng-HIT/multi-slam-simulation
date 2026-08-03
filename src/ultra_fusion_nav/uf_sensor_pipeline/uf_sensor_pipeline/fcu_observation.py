import bisect
import math


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def valid_interval(previous_s, current_s, minimum_s=0.005, maximum_s=0.5):
    if previous_s is None or not math.isfinite(current_s):
        return None
    interval_s = float(current_s) - float(previous_s)
    if not math.isfinite(interval_s) or interval_s < minimum_s or interval_s > maximum_s:
        return None
    return interval_s


def legacy_flow_rate_to_sensor_frd(flow_rate_x_flu, flow_rate_y_flu, interval_s):
    """Convert MAVROS base_link FLU rates back to the flow sensor FRD convention."""
    return (
        float(flow_rate_x_flu) * float(interval_s),
        -float(flow_rate_y_flu) * float(interval_s),
    )


def legacy_pixel_flow_to_sensor_frd(flow_x_flu, flow_y_flu, fx_px, fy_px):
    """Decode MAVLink1 OPTICAL_FLOW pixels after FRD-to-FLU transport mapping."""
    if fx_px <= 0.0 or fy_px <= 0.0:
        raise ValueError("optical-flow focal lengths must be positive")
    return (
        math.atan2(float(flow_x_flu), float(fx_px)),
        -math.atan2(float(flow_y_flu), float(fy_px)),
    )


def _interpolate(samples, timestamp_s, maximum_gap_s):
    times = [sample[0] for sample in samples]
    index = bisect.bisect_left(times, timestamp_s)
    if index < len(samples) and abs(times[index] - timestamp_s) <= 1.0e-9:
        return tuple(float(value) for value in samples[index][1:])
    if index == 0 or index >= len(samples):
        return None
    before = samples[index - 1]
    after = samples[index]
    span_s = after[0] - before[0]
    if span_s <= 0.0 or span_s > maximum_gap_s:
        return None
    ratio = (timestamp_s - before[0]) / span_s
    return tuple(
        float(before[axis] + ratio * (after[axis] - before[axis]))
        for axis in range(1, 4)
    )


def integrate_flu_gyro_as_sensor_frd(samples, start_s, end_s, maximum_gap_s=0.12):
    """Trapezoid-integrate FCU ROS-FLU gyro samples in sensor FRD axes."""
    if end_s <= start_s or len(samples) < 2:
        return None
    ordered = sorted(samples, key=lambda sample: sample[0])
    start_value = _interpolate(ordered, start_s, maximum_gap_s)
    end_value = _interpolate(ordered, end_s, maximum_gap_s)
    if start_value is None or end_value is None:
        return None
    points = [(start_s, *start_value)]
    points.extend(sample for sample in ordered if start_s < sample[0] < end_s)
    points.append((end_s, *end_value))
    integral_flu = [0.0, 0.0, 0.0]
    for before, after in zip(points, points[1:]):
        dt = after[0] - before[0]
        if dt <= 0.0 or dt > maximum_gap_s:
            return None
        for axis in range(3):
            integral_flu[axis] += 0.5 * (before[axis + 1] + after[axis + 1]) * dt
    return (integral_flu[0], -integral_flu[1], -integral_flu[2])
