"""Deterministic 3D route generation for repeatable flight tests."""

from __future__ import annotations

import bisect
import math
from typing import Iterable, Sequence


Point3 = tuple[float, float, float]


def normalize_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    angle = float(angle)
    if not math.isfinite(angle):
        raise ValueError("angle must be finite")
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def backend_error_to_fcu_setpoint(
    backend_position: Point3,
    backend_target: Point3,
    fcu_position: Point3,
    backend_to_fcu_yaw: float,
    max_horizontal_offset: float,
    max_vertical_offset: float,
) -> Point3:
    """Convert unified-backend position error into an FCU-local setpoint.

    The mission target and feedback stay in the unified backend frame. MAVROS
    local position is used only as the origin required by APM's local setpoint
    interface; it is never used to decide whether the route was followed.
    """
    values = (
        *backend_position,
        *backend_target,
        *fcu_position,
        backend_to_fcu_yaw,
        max_horizontal_offset,
        max_vertical_offset,
    )
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("route-control values must be finite")
    if max_horizontal_offset <= 0.0 or max_vertical_offset <= 0.0:
        raise ValueError("route-control offset limits must be positive")

    error_x = backend_target[0] - backend_position[0]
    error_y = backend_target[1] - backend_position[1]
    error_z = backend_target[2] - backend_position[2]
    horizontal = math.hypot(error_x, error_y)
    if horizontal > max_horizontal_offset:
        scale = max_horizontal_offset / horizontal
        error_x *= scale
        error_y *= scale
    error_z = max(-max_vertical_offset, min(max_vertical_offset, error_z))

    cosine = math.cos(backend_to_fcu_yaw)
    sine = math.sin(backend_to_fcu_yaw)
    fcu_error_x = cosine * error_x - sine * error_y
    fcu_error_y = sine * error_x + cosine * error_y
    return (
        fcu_position[0] + fcu_error_x,
        fcu_position[1] + fcu_error_y,
        fcu_position[2] + error_z,
    )


def clamp_route_altitude_setpoint(
        command_z, origin_fcu_z, peak_rise, margin):
    """Bound route altitude without feeding FCU state into the estimator."""
    values = (command_z, origin_fcu_z, peak_rise, margin)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("route altitude guard values must be finite")
    if peak_rise < 0.0 or margin < 0.0:
        raise ValueError("route altitude guard limits must be non-negative")
    lower = origin_fcu_z - margin
    upper = origin_fcu_z + peak_rise + margin
    return min(max(command_z, lower), upper)


def generate_calibration_figure_eight(
    center: Point3,
    radius: float,
    samples: int = 161,
) -> list[Point3]:
    """Generate a level, closed figure-eight around the current hold point."""
    center = tuple(float(value) for value in center)
    radius = float(radius)
    samples = int(samples)
    if len(center) != 3 or not all(math.isfinite(value) for value in center):
        raise ValueError("calibration center must be a finite 3-vector")
    if not math.isfinite(radius) or radius <= 0.0 or samples < 9:
        raise ValueError("calibration radius must be positive and samples at least 9")

    cx, cy, cz = center
    points: list[Point3] = []
    for index in range(samples):
        phase = index / float(samples - 1)
        angle = 2.0 * math.pi * phase
        points.append((
            cx + radius * math.sin(angle),
            cy + radius * math.sin(angle) * math.cos(angle),
            cz,
        ))
    points[0] = center
    points[-1] = center
    return points


def generate_s_curve(
    longitudinal_span: float,
    lateral_amplitude: float,
    base_altitude: float,
    vertical_amplitude: float,
    samples: int = 241,
    vertical_cycles: int = 1,
) -> list[Point3]:
    """Generate one center-to-center S pass in the local x/y/z frame.

    The longitudinal axis is local y, lateral motion is local x, and altitude
    returns to ``base_altitude`` at both ends. One lateral sine cycle gives the
    two opposing lobes of an S. Integer vertical cycles preserve endpoint
    continuity when consecutive passes are reversed.
    """
    longitudinal_span = float(longitudinal_span)
    lateral_amplitude = float(lateral_amplitude)
    base_altitude = float(base_altitude)
    vertical_amplitude = float(vertical_amplitude)
    samples = int(samples)
    vertical_cycles = int(vertical_cycles)
    values = (
        longitudinal_span,
        lateral_amplitude,
        base_altitude,
        vertical_amplitude,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("S-curve dimensions must be finite")
    if longitudinal_span <= 0.0 or lateral_amplitude < 0.0:
        raise ValueError("S-curve span must be positive and amplitude nonnegative")
    if vertical_amplitude < 0.0 or samples < 3 or vertical_cycles < 1:
        raise ValueError("invalid S-curve altitude, sample count, or vertical cycles")

    points: list[Point3] = []
    for index in range(samples):
        phase = index / float(samples - 1)
        angle = 2.0 * math.pi * phase
        x = lateral_amplitude * math.sin(angle)
        y = longitudinal_span * (phase - 0.5)
        z = base_altitude + vertical_amplitude * math.sin(
            vertical_cycles * angle
        )
        points.append((x, y, z))
    return points


def generate_large_figure_eight(
    longitudinal_span: float,
    lateral_amplitude: float,
    base_altitude: float,
    peak_rise: float,
    samples: int = 481,
    rotation_deg: float = 158.0,
    altitude_power: int = 4,
) -> list[Point3]:
    """Generate a closed figure-eight with straight crossings and round lobes.

    Each lobe consists of a straight tangent, the major arc of a circle, and a
    second straight tangent. Mirroring and reversing the first lobe makes the
    two center-crossing branches continuous while sharing only the center
    point, never a route segment. Altitude changes only on the outer arcs.
    """
    longitudinal_span = float(longitudinal_span)
    lateral_amplitude = float(lateral_amplitude)
    base_altitude = float(base_altitude)
    peak_rise = float(peak_rise)
    rotation_deg = float(rotation_deg)
    samples = int(samples)
    altitude_power = int(altitude_power)
    values = (
        longitudinal_span,
        lateral_amplitude,
        base_altitude,
        peak_rise,
        rotation_deg,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("figure-eight dimensions must be finite")
    if longitudinal_span <= 0.0 or lateral_amplitude <= 0.0:
        raise ValueError("figure-eight span and amplitude must be positive")
    half_span = 0.5 * longitudinal_span
    if lateral_amplitude >= half_span:
        raise ValueError("figure-eight lobe radius must be smaller than half span")
    if peak_rise < 0.0 or samples < 9:
        raise ValueError("invalid figure-eight altitude or sample count")
    if samples % 2 == 0:
        raise ValueError("figure-eight sample count must be odd")
    if altitude_power < 2 or altitude_power % 2 != 0:
        raise ValueError("figure-eight altitude power must be an even integer")

    rotation = math.radians(rotation_deg)
    cosine = math.cos(rotation)
    sine = math.sin(rotation)
    radius = lateral_amplitude
    tangent_distance = math.sqrt(half_span * half_span - radius * radius)
    tangent_x = tangent_distance * tangent_distance / half_span
    tangent_y = radius * tangent_distance / half_span
    theta_upper = math.atan2(tangent_y, tangent_x - half_span)
    arc_angle = 2.0 * theta_upper
    arc_length = radius * arc_angle
    lobe_length = 2.0 * tangent_distance + arc_length

    first_lobe: list[Point3] = []
    lobe_samples = (samples + 1) // 2
    for index in range(lobe_samples):
        distance = lobe_length * index / float(lobe_samples - 1)
        altitude = base_altitude
        if distance <= tangent_distance:
            ratio = distance / tangent_distance
            axis_x = tangent_x * ratio
            axis_y = tangent_y * ratio
        elif distance <= tangent_distance + arc_length:
            arc_distance = distance - tangent_distance
            arc_phase = arc_distance / arc_length
            theta = theta_upper - arc_distance / radius
            axis_x = half_span + radius * math.cos(theta)
            axis_y = radius * math.sin(theta)
            altitude += (
                peak_rise
                * math.sin(math.pi * arc_phase) ** altitude_power
            )
        else:
            ratio = (
                distance - tangent_distance - arc_length
            ) / tangent_distance
            axis_x = tangent_x * (1.0 - ratio)
            axis_y = -tangent_y * (1.0 - ratio)
        first_lobe.append((
            cosine * axis_x - sine * axis_y,
            sine * axis_x + cosine * axis_y,
            altitude,
        ))

    second_lobe = [
        (-x, -y, z)
        for x, y, z in reversed(first_lobe)
    ]
    points = first_lobe + second_lobe[1:]
    center = (0.0, 0.0, base_altitude)
    points[0] = center
    points[len(points) // 2] = center
    points[-1] = center
    return points


def polyline_length(points: Sequence[Point3]) -> float:
    if len(points) < 2:
        return 0.0
    return sum(
        math.dist(first, second)
        for first, second in zip(points[:-1], points[1:])
    )


def resample_polyline(points: Iterable[Point3], spacing: float) -> list[Point3]:
    """Resample a polyline at approximately uniform arc-length spacing."""
    source = [tuple(float(value) for value in point) for point in points]
    spacing = float(spacing)
    if len(source) < 2:
        raise ValueError("polyline requires at least two points")
    if not math.isfinite(spacing) or spacing <= 0.0:
        raise ValueError("resampling spacing must be finite and positive")
    if any(len(point) != 3 or not all(math.isfinite(v) for v in point)
           for point in source):
        raise ValueError("polyline points must be finite 3-vectors")

    cumulative = [0.0]
    for first, second in zip(source[:-1], source[1:]):
        cumulative.append(cumulative[-1] + math.dist(first, second))
    total = cumulative[-1]
    if total <= 1.0e-9:
        raise ValueError("polyline length must be positive")

    targets = [index * spacing for index in range(int(total / spacing) + 1)]
    if not math.isclose(targets[-1], total, abs_tol=1.0e-9):
        targets.append(total)
    output: list[Point3] = []
    for target in targets:
        upper = min(len(cumulative) - 1, bisect.bisect_left(cumulative, target))
        if upper == 0:
            output.append(source[0])
            continue
        lower = upper - 1
        segment = cumulative[upper] - cumulative[lower]
        ratio = 0.0 if segment <= 1.0e-12 else (
            (target - cumulative[lower]) / segment
        )
        output.append(tuple(
            source[lower][axis]
            + ratio * (source[upper][axis] - source[lower][axis])
            for axis in range(3)
        ))
    output[0] = source[0]
    output[-1] = source[-1]
    return output
