"""Online Ultra-Fusion-style fixed-lag fusion backend.

FAST-LIO owns scan deskew, point-to-plane association, and its local map.  The
manifold backend owns navigation-state IMU propagation and joint optimization;
FAST-LIO odometry is only the first-state initializer when native LiDAR factors
are available.  FCU fused local position and Gazebo truth never enter the
estimator.
"""

from bisect import bisect_left, bisect_right
from collections import Counter, deque
import copy
from dataclasses import dataclass, replace
import gc
import json
import math
import os
from pathlib import Path as FilePath
import queue
import resource
import threading
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Time as RosTime
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from mavros_msgs.msg import OpticalFlowRad
from nav_msgs.msg import Odometry, Path
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import FluidPressure, Imu, NavSatFix, NavSatStatus
from geometry_msgs.msg import PoseStamped
from uf_interfaces.msg import (
    FusionEpoch,
    LidarCalibrationMotion,
    ReliabilityScore,
    RelocalizationResult,
    RgbdDirectTracks,
    RgbdGeometryTracks,
    SchedulerState,
    VisualFeatureTracks,
)

from .imu_preintegration import (
    ImuSample,
    _quat_to_rotvec,
    preintegrate,
    preintegrate_manifold,
)
from .barometer import LocalBarometerSegment
from .axis_reliability import (
    AxisReliabilityProfile,
    barometer_activation_required,
    combine_axis_reliability,
)
from .manifold_window import ManifoldSlidingWindowBackend, propagate_state
from .visual_reprojection import (
    RgbdDepthTrackBatch,
    RgbdDirectTrackBatch,
    VisualTrackBatch,
    rgbd_depth_residual_jacobians,
    rgbd_direct_residual_jacobians,
    validate_visual_linearization,
    visual_pose_observability,
)
from .visual_initialization import (
    OnlineVisualTimeCalibrator,
    VisualInitializationGate,
)
from .live_propagation import (
    auxiliary_keyframe_admission,
    live_propagation_admission,
    unified_odom_publication_decision,
    make_optimization_anchor,
    propagate_optimization_anchor,
    state_covariance_to_odometry_covariances,
)
from .scan_prediction import (
    build_scan_prediction,
    consume_cached_prediction,
    prediction_reusable,
    scan_request_ready,
    scan_request_stale,
)
from .spatiotemporal_calibration import (
    LidarMotionSample,
    OnlineSpatiotemporalCalibrator,
    effective_time_offset,
)
from .native_lidar import (
    NativeFactorBuffer,
    lidar_pose_observability,
    lidar_reliability_layers,
    lidar_vertical_observability,
    native_factor_from_message,
    quaternion_xyzw_to_rpy,
    matrix_to_pose_vector,
    pose_vector_to_matrix,
    right_perturbation_jacobian_rpy,
    rpy_to_quaternion_xyzw,
    rpy_to_rotation_matrix,
    transform_native_factor_map,
    validate_native_frame_contract,
    with_yaw_reference,
)
from .range_facet import (
    RangeFacetObservation,
    evaluate_range_facet,
)
from .window import SlidingWindowBackend
from uf_reliability.scoring import (
    optical_flow_displacement_frd,
    optical_flow_lever_arm_displacement_flu,
    optical_flow_los_prediction_flu,
    optical_flow_los_rate_apm,
    optical_flow_score,
)
from uf_reliability.flow_rotation_gate import (
    FlowRotationGateConfig,
    OpticalFlowRotationGate,
    interval_mean_absolute_yaw_rate,
    interval_mean_vector,
)

try:
    from fast_lio.msg import (
        BackendDeskewTrajectory,
        BackendStateSeed,
        FrontendScanRequest,
        NativeLidarFactor,
    )
except ImportError:  # pragma: no cover - unit tests run without the external overlay
    BackendStateSeed = None
    BackendDeskewTrajectory = None
    FrontendScanRequest = None
    NativeLidarFactor = None


WGS84_A_M = 6378137.0
WGS84_E2 = 6.69437999014e-3
MIN_FLOW_QUALITY = 20
MAX_COVARIANCE_INFLATION = 20.0
RAW_LIDAR_SCAN_TO_SCAN = 1


def select_nonlinear_iteration_budget(
    normal_iterations,
    initialization_iterations,
    recovery_iterations,
    state_count,
    recovery_active=False,
):
    """Keep routine tracking cheap while preserving difficult-state headroom."""
    normal = int(normal_iterations)
    initialization = int(initialization_iterations)
    recovery = int(recovery_iterations)
    if min(normal, initialization, recovery) < 1:
        raise ValueError("nonlinear iteration budgets must be positive")
    if bool(recovery_active):
        return recovery
    if int(state_count) <= 2:
        return initialization
    return normal


def lidar_calibration_motion_from_message(msg):
    """Validate that OSC motion is independent of IMU/backend estimation."""
    if not bool(msg.accepted) or not bool(msg.converged):
        raise ValueError("calibration motion did not pass registration gates")
    if int(msg.provenance) != RAW_LIDAR_SCAN_TO_SCAN:
        raise ValueError("calibration motion provenance is not raw LiDAR")
    if bool(msg.imu_aided) or bool(msg.backend_aided):
        raise ValueError(
            "calibration motion must be independent of IMU and backend")
    if str(msg.rotation_convention) != "R_L_previous_from_L_current":
        raise ValueError(
            "calibration motion rotation convention is incompatible")
    start_s = stamp_seconds(msg.start_stamp)
    end_s = stamp_seconds(msg.header.stamp)
    if (
        not math.isfinite(start_s)
        or not math.isfinite(end_s)
        or start_s <= 0.0
        or end_s <= start_s
    ):
        raise ValueError("calibration motion timestamp interval is invalid")
    inlier_ratio = float(msg.inlier_ratio)
    fitness_m2 = float(msg.fitness_score)
    residual_rms_m = float(msg.residual_rms_m)
    rotation_condition = float(msg.rotation_information_condition)
    rotation_information = np.asarray(
        msg.rotation_information_eigenvalues, dtype=float
    )
    if (
        not math.isfinite(inlier_ratio)
        or not 0.0 < inlier_ratio <= 1.0
        or not math.isfinite(fitness_m2)
        or fitness_m2 < 0.0
        or not math.isfinite(residual_rms_m)
        or residual_rms_m < 0.0
        or not math.isfinite(rotation_condition)
        or rotation_condition < 1.0
        or rotation_information.shape != (3,)
        or np.any(~np.isfinite(rotation_information))
        or np.any(rotation_information <= 0.0)
    ):
        raise ValueError("calibration motion quality evidence is invalid")
    quaternion = msg.relative_rotation
    rotation = rpy_to_rotation_matrix(quaternion_xyzw_to_rpy([
        float(quaternion.x), float(quaternion.y),
        float(quaternion.z), float(quaternion.w),
    ]))
    quality_weight = float(np.clip(
        inlier_ratio * math.exp(-residual_rms_m / 0.15), 0.05, 1.0
    ))
    return LidarMotionSample(start_s, end_s, rotation, quality_weight)


def seed_calibrator_rotation_nonblocking(calibrator, lock, rotation):
    """Seed the fixed extrinsic once without blocking the estimator worker."""
    if bool(calibrator.initial_rotation_set):
        return None, "already_initialized"
    if not lock.acquire(blocking=False):
        return None, "calibration_busy"
    try:
        if bool(calibrator.initial_rotation_set):
            return None, "already_initialized"
        calibrator.set_initial_rotation(rotation)
        return calibrator.last_update, "initialized"
    finally:
        lock.release()


def frontend_map_commit_decision(
        scheduler_health, scheduler_age_s, scheduler_timeout_s,
        lidar_factor_eligible, pose_covariance, allowed_health_states,
        maximum_position_variance_m2, maximum_orientation_variance_rad2):
    """Gate irreversible front-end map writes without suppressing odometry."""
    health = str(scheduler_health).upper()
    allowed = {str(value).upper() for value in allowed_health_states}
    if health not in allowed:
        return False, f"scheduler_{health.lower()}", math.inf, math.inf
    if (
        not math.isfinite(scheduler_age_s)
        or scheduler_age_s < 0.0
        or scheduler_age_s > scheduler_timeout_s
    ):
        return False, "scheduler_stale", math.inf, math.inf
    if not bool(lidar_factor_eligible):
        return False, "lidar_factor_rejected", math.inf, math.inf
    covariance = np.asarray(pose_covariance, dtype=float)
    if covariance.size != 36:
        return False, "pose_covariance_shape", math.inf, math.inf
    covariance = covariance.reshape(6, 6)
    diagonal = np.diag(covariance)
    if np.any(~np.isfinite(diagonal)) or np.any(diagonal < 0.0):
        return False, "pose_covariance_invalid", math.inf, math.inf
    position_variance = float(np.max(diagonal[:3]))
    orientation_variance = float(np.max(diagonal[3:]))
    if position_variance > maximum_position_variance_m2:
        return False, "position_variance", position_variance, orientation_variance
    if orientation_variance > maximum_orientation_variance_rad2:
        return False, "orientation_variance", position_variance, orientation_variance
    return True, "ok", position_variance, orientation_variance


def delayed_frontend_map_commit_candidate(states, stamps, delay_states):
    """Select a stabilized state without exposing a future window estimate.

    The LiDAR front-end cannot move points after inserting them into its map.
    A positive delay therefore selects an older state that has survived several
    fixed-lag optimizations.  The caller captures it before adding the next
    state, so the oldest candidate is committed only after that transaction
    succeeds and marginalizes it.
    """
    delay_states = int(delay_states)
    if delay_states < 0:
        raise ValueError("front-end map commit delay must be non-negative")
    states = list(states)
    stamps = [float(stamp) for stamp in stamps]
    if len(states) != len(stamps):
        raise ValueError("front-end map states and stamps must align")
    index = len(states) - 1 - delay_states
    if index < 0:
        return None
    state = np.asarray(states[index], dtype=float)
    if state.shape != (15,) or np.any(~np.isfinite(state)):
        raise ValueError("front-end map state must be a finite 15-vector")
    if not math.isfinite(stamps[index]) or stamps[index] <= 0.0:
        raise ValueError("front-end map state requires a source timestamp")
    return stamps[index], state.copy()


def attach_frontend_map_commit_eligibility(candidate, eligibility_by_stamp):
    """Attach the admission decision made for the candidate's own scan."""
    if candidate is None:
        return None
    stamp_s, state = candidate
    key = int(round(float(stamp_s) * 1.0e9))
    eligibility = eligibility_by_stamp.get(key)
    if eligibility is None:
        return stamp_s, state, False, "eligibility_missing"
    eligible, reason = eligibility
    return stamp_s, state, bool(eligible), str(reason)


def native_factor_epoch_alignment(
        map_from_lio, backend_trajectory_frontend_enabled):
    """Choose the map transform for a native factor at an epoch boundary.

    With backend-owned scan trajectories, FAST-LIO emits future factors in the
    unified map frame, so it must never receive the persistent local-to-map
    correction.  The in-flight old-epoch factor is discarded at the reset
    barrier.  A legacy independent front end keeps producing in its local frame
    and therefore needs the persistent alignment.
    """
    alignment = np.asarray(map_from_lio, dtype=float)
    if alignment.shape != (4, 4) or np.any(~np.isfinite(alignment)):
        raise ValueError("native factor epoch alignment must be finite 4x4")
    if backend_trajectory_frontend_enabled:
        return np.eye(4, dtype=float)
    return alignment


def native_factor_epoch_barrier_required(
        relocalization_applied_now, backend_trajectory_frontend_enabled):
    """Drop the old-epoch factor so the front end rematches after a reset."""
    return bool(
        relocalization_applied_now and backend_trajectory_frontend_enabled
    )


def native_factor_epoch_status(
        factor_reset_counter, current_reset_counter,
        relocalization_applied_now, backend_trajectory_frontend_enabled):
    """Classify a native factor without crossing a relocalization epoch."""
    if not backend_trajectory_frontend_enabled:
        # The independent FAST-LIO front end keeps factors in its persistent
        # local frame. map_from_lio applies the new global alignment below,
        # so its packet counter does not participate in backend epoch gating.
        return "current"
    if native_factor_epoch_barrier_required(
        relocalization_applied_now, backend_trajectory_frontend_enabled
    ):
        return "barrier"
    factor_epoch = int(factor_reset_counter)
    current_epoch = int(current_reset_counter)
    if factor_epoch < current_epoch:
        return "stale"
    if factor_epoch > current_epoch:
        return "future"
    return "current"


def enqueue_latest(work_queue, item):
    """Keep only the newest unprocessed native frame when the worker lags."""
    try:
        work_queue.put_nowait(item)
        return 0
    except queue.Full:
        pass
    discarded = 0
    while True:
        try:
            work_queue.get_nowait()
            work_queue.task_done()
            discarded += 1
        except queue.Empty:
            break
    try:
        work_queue.put_nowait(item)
    except queue.Full:
        return -1
    return discarded


def drain_work_queue(work_queue):
    """Remove queued work while keeping Queue.unfinished_tasks consistent."""
    removed = []
    while True:
        try:
            item = work_queue.get_nowait()
        except queue.Empty:
            break
        removed.append(item)
        work_queue.task_done()
    return removed


def retain_stamped_records_after(records, boundary_s):
    """Retain only observations belonging to the new estimator epoch."""
    boundary_s = float(boundary_s)
    if not math.isfinite(boundary_s):
        raise ValueError("epoch boundary must be finite")
    retained = []
    for record in records:
        try:
            stamp_s = float(record["stamp_s"])
        except (KeyError, TypeError, ValueError):
            continue
        if math.isfinite(stamp_s) and stamp_s > boundary_s:
            retained.append(record)
    return retained


def reanchor_imu_samples(samples, boundary_s):
    """Keep one IMU predecessor for interpolation plus all newer samples."""
    boundary_s = float(boundary_s)
    if not math.isfinite(boundary_s):
        raise ValueError("IMU epoch boundary must be finite")
    ordered = sorted(
        (
            sample for sample in samples
            if math.isfinite(float(sample.stamp_s))
        ),
        key=lambda sample: float(sample.stamp_s),
    )
    if not ordered:
        return []
    stamps = [float(sample.stamp_s) for sample in ordered]
    first_newer = bisect_right(stamps, boundary_s)
    return ordered[max(0, first_newer - 1):]


def backend_diagnostic_level_message(
    last_reason, optimization_errors, contract_violated, contract_reason
):
    if contract_violated:
        return (
            DiagnosticStatus.ERROR,
            f"scan_prediction_contract_violation:{contract_reason}",
        )
    healthy = str(last_reason) == "ok" and int(optimization_errors) == 0
    return (
        DiagnosticStatus.OK if healthy else DiagnosticStatus.WARN,
        str(last_reason),
    )


def directional_information(information, direction):
    information = np.asarray(information, dtype=float)
    direction = np.asarray(direction, dtype=float)
    if information.shape != (3, 3) or direction.shape != (3,):
        raise ValueError("directional information expects 3x3 and 3-vector")
    if np.any(~np.isfinite(information)) or np.any(~np.isfinite(direction)):
        return 0.0
    norm = float(np.linalg.norm(direction))
    if norm <= 1.0e-12:
        return 0.0
    unit = direction / norm
    return max(0.0, float(unit @ information @ unit))


def cap_weak_subspace_against_absolute_information(
    base_scale, previous_scale, lidar_information, absolute_information
):
    """Keep weak-mode LiDAR information no stronger than absolute aiding."""
    matrices = [
        np.asarray(value, dtype=float)
        for value in (
            base_scale, previous_scale, lidar_information,
            absolute_information,
        )
    ]
    if any(value.shape != (3, 3) for value in matrices):
        raise ValueError("subspace information cap expects 3x3 matrices")
    if any(np.any(~np.isfinite(value)) for value in matrices):
        return matrices[0].copy(), np.ones(3, dtype=float)
    base, previous, lidar, absolute = [
        0.5 * (value + value.T) for value in matrices
    ]
    values, vectors = np.linalg.eigh(base)
    result_values = values.copy()
    information_ratios = np.ones(3, dtype=float)
    for mode, (base_value, direction) in enumerate(zip(values, vectors.T)):
        if base_value >= 1.0 - 1.0e-9:
            result_values[mode] = 1.0
            continue
        previous_value = float(direction @ previous @ direction)
        previous_value = float(np.clip(previous_value, 0.0, base_value))
        lidar_value = directional_information(lidar, direction)
        absolute_value = directional_information(absolute, direction)
        ratio = (
            min(1.0, absolute_value / lidar_value)
            if lidar_value > 0.0 and absolute_value > 0.0 else 1.0
        )
        information_ratios[mode] = ratio
        result_values[mode] = min(base_value, previous_value * ratio)
    return vectors @ np.diag(result_values) @ vectors.T, information_ratios


@dataclass(frozen=True)
class StationaryImuInitialization:
    valid: bool
    reason: str
    accel_bias: tuple[float, float, float]
    gyro_bias: tuple[float, float, float]
    sample_count: int
    span_s: float
    accel_norm_mps2: float
    accel_residual_rms_mps2: float
    gyro_norm_radps: float
    gyro_residual_rms_radps: float


@dataclass(frozen=True)
class OptimizationIntegrity:
    valid: bool
    reason: str
    translation_correction_m: float
    rotation_correction_rad: float
    velocity_correction_mps: float
    accel_bias_correction_mps2: float
    gyro_bias_correction_radps: float
    initial_cost: float
    final_cost: float
    latest_information_rank: int
    latest_information_condition: float


def validate_optimized_state(
    initial_state,
    estimate,
    latest_information,
    initial_cost,
    final_cost,
    *,
    maximum_translation_correction_m,
    maximum_rotation_correction_rad,
    maximum_velocity_correction_mps,
    maximum_accel_bias_correction_mps2,
    maximum_gyro_bias_correction_radps,
    maximum_information_condition,
    information_rank_tolerance,
):
    """Gate one optimized keyframe before state publication or map insertion."""
    initial = np.asarray(initial_state, dtype=float)
    optimized = np.asarray(estimate, dtype=float)
    information = np.asarray(latest_information, dtype=float)
    initial_cost_value = float(initial_cost)
    final_cost_value = float(final_cost)

    def invalid_result(reason):
        return OptimizationIntegrity(
            False, reason,
            math.inf, math.inf, math.inf, math.inf, math.inf,
            initial_cost_value, final_cost_value, 0, math.inf,
        )

    integrity_checks = (
        (initial.shape != (15,), "invalid_initial_state_shape"),
        (optimized.shape != (15,), "invalid_optimized_state_shape"),
        (information.shape != (15, 15), "invalid_latest_information_shape"),
        (np.any(~np.isfinite(initial)), "nonfinite_initial_state"),
        (np.any(~np.isfinite(optimized)), "nonfinite_optimized_state"),
        (np.any(~np.isfinite(information)), "nonfinite_latest_information"),
        (not math.isfinite(initial_cost_value), "nonfinite_initial_cost"),
        (not math.isfinite(final_cost_value), "nonfinite_final_cost"),
    )
    for failed, reason in integrity_checks:
        if failed:
            return invalid_result(reason)

    relative_pose = np.linalg.inv(pose_vector_to_matrix(initial[:6])) @ (
        pose_vector_to_matrix(optimized[:6])
    )
    relative = matrix_to_pose_vector(relative_pose)
    translation = float(np.linalg.norm(optimized[:3] - initial[:3]))
    rotation = float(np.linalg.norm(relative[3:6]))
    velocity = float(np.linalg.norm(optimized[6:9] - initial[6:9]))
    accel_bias = float(np.linalg.norm(optimized[9:12] - initial[9:12]))
    gyro_bias = float(np.linalg.norm(optimized[12:15] - initial[12:15]))

    symmetric_information = 0.5 * (information + information.T)
    eigenvalues = np.linalg.eigvalsh(symmetric_information)
    scale = max(1.0, float(np.max(np.abs(eigenvalues))))
    if float(np.min(eigenvalues)) < -information_rank_tolerance * scale:
        rank = 0
        condition = math.inf
        reason = "indefinite_latest_information"
    else:
        active = eigenvalues > information_rank_tolerance * scale
        rank = int(np.count_nonzero(active))
        condition = (
            float(eigenvalues[-1] / eigenvalues[active][0])
            if rank > 0 else math.inf
        )
        reason = "ok"

    limits = (
        (translation, maximum_translation_correction_m, "translation_correction"),
        (rotation, maximum_rotation_correction_rad, "rotation_correction"),
        (velocity, maximum_velocity_correction_mps, "velocity_correction"),
        (accel_bias, maximum_accel_bias_correction_mps2, "accel_bias_correction"),
        (gyro_bias, maximum_gyro_bias_correction_radps, "gyro_bias_correction"),
    )
    cost_tolerance = 1.0e-10 * max(1.0, abs(initial_cost_value))
    if reason == "ok" and final_cost_value > initial_cost_value + cost_tolerance:
        reason = "optimization_cost_increased"
    if reason == "ok":
        for value, limit, name in limits:
            if value > limit:
                reason = f"excessive_{name}"
                break
    if reason == "ok" and (
        rank < 1 or condition > maximum_information_condition
    ):
        reason = "ill_conditioned_latest_information"

    return OptimizationIntegrity(
        reason == "ok", reason, translation, rotation, velocity,
        accel_bias, gyro_bias, initial_cost_value, final_cost_value,
        rank, condition,
    )


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def visual_time_calibration_imu_coverage(
        imu_samples, previous_stamp_s, current_stamp_s, candidate_offsets_s):
    """Return whether IMU samples fairly cover the complete offset search."""
    offsets = np.asarray(candidate_offsets_s, dtype=float).reshape(-1)
    stamps = np.asarray([
        float(sample.stamp_s) for sample in imu_samples
        if math.isfinite(float(sample.stamp_s))
    ], dtype=float)
    if not offsets.size or np.any(~np.isfinite(offsets)):
        raise ValueError("visual time-calibration offsets are invalid")
    if not stamps.size:
        return "wait_future"
    earliest_required = float(previous_stamp_s) + float(np.min(offsets))
    latest_required = float(current_stamp_s) + float(np.max(offsets))
    if float(np.min(stamps)) > earliest_required + 1.0e-12:
        return "missing_history"
    if float(np.max(stamps)) + 1.0e-12 < latest_required:
        return "wait_future"
    return "ready"


@dataclass(frozen=True)
class VisualStateAssociation:
    status: str
    reason: str
    previous_index: int
    current_index: int
    corrected_previous_stamp_s: float
    corrected_current_stamp_s: float
    nearest_previous_stamp_s: float
    nearest_current_stamp_s: float
    previous_delta_s: float
    current_delta_s: float
    missing_side: str


@dataclass(frozen=True)
class PendingVisualCandidate:
    candidate_id: int
    key: tuple[int, int]
    message: object
    arrival_ros_s: float
    arrival_wall_s: float


class GarbageCollectionProfiler:
    """Bounded, opt-in GC timing counters for runtime correlation only."""

    def __init__(self):
        self.started_ns = {}
        self.collections = [0, 0, 0]
        self.duration_ms = [0.0, 0.0, 0.0]
        self.callback = self._callback
        gc.callbacks.append(self.callback)

    def _callback(self, phase, info):
        generation = int(info.get("generation", -1))
        if generation < 0 or generation >= len(self.collections):
            return
        if phase == "start":
            self.started_ns[generation] = time.perf_counter_ns()
        elif phase == "stop":
            started_ns = self.started_ns.pop(generation, None)
            self.collections[generation] += 1
            if started_ns is not None:
                self.duration_ms[generation] += (
                    time.perf_counter_ns() - started_ns
                ) * 1.0e-6

    def snapshot(self):
        return {
            "collections": tuple(self.collections),
            "duration_ms": tuple(self.duration_ms),
            "counts": tuple(gc.get_count()),
        }

    def close(self):
        try:
            gc.callbacks.remove(self.callback)
        except ValueError:
            pass


def process_resource_snapshot():
    usage = resource.getrusage(resource.RUSAGE_SELF)
    return {
        "minor_faults": int(usage.ru_minflt),
        "major_faults": int(usage.ru_majflt),
        "voluntary_context_switches": int(usage.ru_nvcsw),
        "involuntary_context_switches": int(usage.ru_nivcsw),
        "user_cpu_s": float(usage.ru_utime),
        "system_cpu_s": float(usage.ru_stime),
    }


def process_resource_delta(before, after):
    return {
        name: float(after[name]) - float(before[name])
        for name in before
    }


def current_processor_and_frequency_khz():
    """Read the current Linux CPU and its frequency outside the timed solve."""
    processor = -1
    frequency_khz = None
    try:
        fields = FilePath("/proc/self/stat").read_text(
            encoding="ascii"
        ).split(") ", 1)[1].split()
        processor = int(fields[36])
        frequency_path = FilePath(
            f"/sys/devices/system/cpu/cpu{processor}/cpufreq/scaling_cur_freq"
        )
        frequency_khz = int(frequency_path.read_text(encoding="ascii").strip())
    except (OSError, ValueError, IndexError):
        pass
    return processor, frequency_khz


def associate_visual_states(
    previous_camera_stamp_s,
    current_camera_stamp_s,
    state_stamps,
    *,
    camera_to_imu_time_offset_s=0.0,
    tolerance_s=0.065,
):
    """Causally bind the two camera observations to active window states.

    ``camera_to_imu_time_offset_s`` follows ``t_imu = t_camera + td_C``.
    It adjusts the measurement time used for association and never rewrites a
    ROS message stamp. No interpolation is used: both observations must have a
    real state inside the unchanged tolerance.
    """
    stamps = np.asarray(tuple(state_stamps), dtype=float)
    previous = float(previous_camera_stamp_s) + float(
        camera_to_imu_time_offset_s
    )
    current = float(current_camera_stamp_s) + float(
        camera_to_imu_time_offset_s
    )

    def result(status, reason, *, previous_index=-1, current_index=-1,
               previous_nearest=math.nan, current_nearest=math.nan,
               previous_delta=math.inf, current_delta=math.inf,
               missing_side="none"):
        return VisualStateAssociation(
            status, reason, int(previous_index), int(current_index),
            previous, current, float(previous_nearest), float(current_nearest),
            float(previous_delta), float(current_delta), str(missing_side),
        )

    if (
        stamps.ndim != 1 or stamps.size < 2
        or np.any(~np.isfinite(stamps))
        or np.any(np.diff(stamps) <= 0.0)
        or not math.isfinite(previous) or not math.isfinite(current)
        or current <= previous or tolerance_s <= 0.0
    ):
        return result("reject", "invalid_timestamp_contract")
    start = float(stamps[0])
    end = float(stamps[-1])
    if previous < start - tolerance_s:
        return result("reject", "outside_active_window", missing_side="left")
    if current > end + tolerance_s:
        return result("wait", "waiting_for_right_state", missing_side="right")
    previous_index = int(np.argmin(np.abs(stamps - previous)))
    current_index = int(np.argmin(np.abs(stamps - current)))
    previous_nearest = float(stamps[previous_index])
    current_nearest = float(stamps[current_index])
    previous_delta = abs(previous_nearest - previous)
    current_delta = abs(current_nearest - current)
    # When the newest observation is close to the right edge, a later state
    # can still be the nearest causal association. Wait until that possibility
    # is exhausted instead of rejecting on callback arrival order.
    if current_delta > tolerance_s and end < current + tolerance_s:
        return result(
            "wait", "waiting_for_right_state",
            previous_index=previous_index,
            current_index=current_index,
            previous_nearest=previous_nearest,
            current_nearest=current_nearest,
            previous_delta=previous_delta,
            current_delta=current_delta,
            missing_side="right",
        )
    if previous_delta > tolerance_s:
        return result(
            "reject", "state_tolerance_mismatch",
            previous_index=previous_index,
            current_index=current_index,
            previous_nearest=previous_nearest,
            current_nearest=current_nearest,
            previous_delta=previous_delta,
            current_delta=current_delta,
            missing_side="left_gap",
        )
    if current_delta > tolerance_s:
        return result(
            "reject", "state_tolerance_mismatch",
            previous_index=previous_index,
            current_index=current_index,
            previous_nearest=previous_nearest,
            current_nearest=current_nearest,
            previous_delta=previous_delta,
            current_delta=current_delta,
            missing_side="right_gap",
        )
    if previous_index >= current_index:
        return result(
            "reject", "observations_map_to_same_or_reversed_state",
            previous_index=previous_index,
            current_index=current_index,
            previous_nearest=previous_nearest,
            current_nearest=current_nearest,
            previous_delta=previous_delta,
            current_delta=current_delta,
            missing_side="between",
        )
    return result(
        "associated", "associated",
        previous_index=previous_index,
        current_index=current_index,
        previous_nearest=previous_nearest,
        current_nearest=current_nearest,
        previous_delta=previous_delta,
        current_delta=current_delta,
    )


def ros_time_from_seconds(stamp_s):
    stamp_s = float(stamp_s)
    if not math.isfinite(stamp_s) or stamp_s < 0.0:
        raise ValueError("ROS timestamp must be finite and non-negative")
    seconds = int(math.floor(stamp_s))
    nanoseconds = int(round((stamp_s - seconds) * 1.0e9))
    if nanoseconds >= 1_000_000_000:
        seconds += 1
        nanoseconds -= 1_000_000_000
    return RosTime(sec=seconds, nanosec=nanoseconds)


def native_frame_odometry(header, factor):
    """Build a keyframe trigger without consuming FAST-LIO odometry."""
    message = Odometry()
    message.header = copy.deepcopy(header)
    message.header.frame_id = factor.map_frame
    message.child_frame_id = factor.state_frame
    position = factor.linearization_pose[:3]
    quaternion = rpy_to_quaternion_xyzw(factor.linearization_pose[3:6])
    message.pose.pose.position.x = float(position[0])
    message.pose.pose.position.y = float(position[1])
    message.pose.pose.position.z = float(position[2])
    message.pose.pose.orientation.x = quaternion[0]
    message.pose.pose.orientation.y = quaternion[1]
    message.pose.pose.orientation.z = quaternion[2]
    message.pose.pose.orientation.w = quaternion[3]
    return message


def frontend_activation_odometry(local_output, map_frame, body_frame):
    """Return the current local state used to unlock FAST-LIO requests."""
    output = copy.deepcopy(local_output)
    if output.header.frame_id != str(map_frame):
        raise ValueError("frontend activation pose must remain in the local map frame")
    if output.child_frame_id != str(body_frame):
        raise ValueError("frontend activation pose has an unexpected body frame")
    return output


def native_trigger_order_status(
        last_stamp_ns,
        last_sequence,
        stamp_ns,
        sequence):
    """Classify a NativeLidarFactor trigger without mutating backend state."""
    if last_stamp_ns is None or last_sequence is None:
        return "accept", 0
    same_stamp = int(stamp_ns) == int(last_stamp_ns)
    same_sequence = int(sequence) == int(last_sequence)
    if same_stamp and same_sequence:
        return "duplicate", 0
    if same_stamp != same_sequence:
        return "sequence_conflict", 0
    if int(stamp_ns) < int(last_stamp_ns):
        return "nonmonotonic", 0
    if int(sequence) < int(last_sequence):
        return "sequence_reset", 0
    return "accept", max(0, int(sequence) - int(last_sequence) - 1)


def quaternion_to_yaw(quaternion):
    x, y, z, w = (
        float(quaternion.x), float(quaternion.y),
        float(quaternion.z), float(quaternion.w),
    )
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm <= 1.0e-9:
        return 0.0
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_to_quaternion(yaw):
    half = 0.5 * float(yaw)
    return (0.0, 0.0, math.sin(half), math.cos(half))


def unwrap_yaw(previous_yaw, wrapped_yaw):
    """Keep yaw residuals continuous across the +/- pi branch cut."""
    if previous_yaw is None:
        return float(wrapped_yaw)
    delta = math.atan2(
        math.sin(float(wrapped_yaw) - float(previous_yaw)),
        math.cos(float(wrapped_yaw) - float(previous_yaw)),
    )
    return float(previous_yaw) + delta


def rotate_planar(forward, left, yaw):
    cosine, sine = math.cos(float(yaw)), math.sin(float(yaw))
    return (
        cosine * float(forward) - sine * float(left),
        sine * float(forward) + cosine * float(left),
    )


def frd_to_enu_delta(forward, right, yaw):
    return rotate_planar(float(forward), -float(right), yaw)


def geodetic_to_ecef(latitude_deg, longitude_deg, altitude_m):
    latitude = math.radians(float(latitude_deg))
    longitude = math.radians(float(longitude_deg))
    sin_latitude = math.sin(latitude)
    prime_vertical = WGS84_A_M / math.sqrt(
        1.0 - WGS84_E2 * sin_latitude * sin_latitude
    )
    return (
        (prime_vertical + altitude_m) * math.cos(latitude) * math.cos(longitude),
        (prime_vertical + altitude_m) * math.cos(latitude) * math.sin(longitude),
        (prime_vertical * (1.0 - WGS84_E2) + altitude_m) * sin_latitude,
    )


class LocalEnuProjector:
    def __init__(self, latitude_deg, longitude_deg, altitude_m):
        self.latitude = math.radians(float(latitude_deg))
        self.longitude = math.radians(float(longitude_deg))
        self.origin = geodetic_to_ecef(latitude_deg, longitude_deg, altitude_m)

    def project(self, latitude_deg, longitude_deg, altitude_m):
        x, y, z = geodetic_to_ecef(latitude_deg, longitude_deg, altitude_m)
        dx, dy, dz = x - self.origin[0], y - self.origin[1], z - self.origin[2]
        sin_latitude, cos_latitude = math.sin(
            self.latitude), math.cos(
            self.latitude)
        sin_longitude, cos_longitude = math.sin(
            self.longitude), math.cos(
            self.longitude)
        return (
            -sin_longitude * dx + cos_longitude * dy,
            -sin_latitude * cos_longitude * dx
            - sin_latitude * sin_longitude * dy
            + cos_latitude * dz,
            cos_latitude * cos_longitude * dx
            + cos_latitude * sin_longitude * dy
            + sin_latitude * dz,
        )


def scheduler_decision(weight=1.0, enabled=True, inflation=1.0):
    weight = max(0.0, min(1.0, float(weight)))
    inflation = max(1.0, min(MAX_COVARIANCE_INFLATION, float(inflation)))
    return {
        "factor_enabled": bool(enabled) and weight > 0.0,
        "reliability_weight": weight if enabled else 0.0,
        "covariance_inflation": inflation,
    }


def select_timestamped_reliability_score(records, source_stamp_s, tolerance_s):
    """Return the closest score measured for the same source observation."""
    target = float(source_stamp_s)
    tolerance = max(0.0, float(tolerance_s))
    if not math.isfinite(target):
        return None
    eligible = [
        record for record in records
        if math.isfinite(float(record.get("source_stamp_s", math.nan)))
        and abs(float(record["source_stamp_s"]) - target) <= tolerance
    ]
    if not eligible:
        return None
    return min(
        eligible,
        key=lambda record: (
            abs(float(record["source_stamp_s"]) - target),
            -int(record.get("sequence", 0)),
        ),
    )


def consume_timestamped_reliability_score(
    records, source_stamp_s, tolerance_s,
):
    """Select one timestamp-matched score and remove that exact record."""
    records = tuple(records)
    target = float(source_stamp_s)
    selected = select_timestamped_reliability_score(
        records, target, tolerance_s
    )
    if selected is None:
        nearest_error_s = min(
            (
                abs(float(record.get("source_stamp_s", math.inf)) - target)
                for record in records
                if math.isfinite(float(record.get("source_stamp_s", math.nan)))
            ),
            default=math.inf,
        )
        return None, records, nearest_error_s
    index = next(
        index for index, record in enumerate(records) if record is selected
    )
    error_s = abs(float(selected["source_stamp_s"]) - target)
    return selected, records[:index] + records[index + 1:], error_s


def visual_factor_score_wait_status(
    start_ros_s,
    start_wall_s,
    now_ros_s,
    now_wall_s,
    maximum_ros_wait_s,
    maximum_wall_wait_s,
):
    """Expire a missing score when either simulation or wall time runs out."""
    maximum_ros_wait_s = float(maximum_ros_wait_s)
    maximum_wall_wait_s = float(maximum_wall_wait_s)
    if maximum_ros_wait_s <= 0.0 or maximum_wall_wait_s <= 0.0:
        raise ValueError("visual factor score wait limits must be positive")
    ros_wait_s = max(0.0, float(now_ros_s) - float(start_ros_s))
    wall_wait_s = max(0.0, float(now_wall_s) - float(start_wall_s))
    status = (
        "expired"
        if ros_wait_s >= maximum_ros_wait_s
        or wall_wait_s >= maximum_wall_wait_s
        else "wait"
    )
    return status, ros_wait_s, wall_wait_s


def combine_visual_reliability_decisions(sensor_decision, factor_score):
    """Apply timestamp-matched candidate quality after camera-health gating."""
    combined = copy.deepcopy(sensor_decision)
    reasons = list(combined.get("reasons", ()))
    if factor_score is None:
        combined.update({
            "factor_enabled": False,
            "reliability_weight": 0.0,
            "covariance_inflation": MAX_COVARIANCE_INFLATION,
            "degradation_score": 1.0,
        })
        reasons.append("visual_factor_score_missing")
        combined["reasons"] = tuple(dict.fromkeys(reasons))
        return combined

    factor_weight = max(
        0.0, min(1.0, float(factor_score.get("weight", 0.0))))
    factor_valid = bool(factor_score.get("valid", False)) and factor_weight > 0.0
    for reason in factor_score.get("reasons", ()):
        tagged = f"factor:{reason}"
        if tagged not in reasons:
            reasons.append(tagged)
    combined["degradation_score"] = max(
        float(combined.get("degradation_score", 0.0)),
        float(factor_score.get("degradation_score", 1.0)),
    )
    if not factor_valid:
        combined.update({
            "factor_enabled": False,
            "reliability_weight": 0.0,
            "covariance_inflation": MAX_COVARIANCE_INFLATION,
        })
        reasons.append("visual_factor_score_invalid")
    else:
        combined["reliability_weight"] = min(
            float(combined.get("reliability_weight", 1.0)), factor_weight
        )
        combined["covariance_inflation"] = max(
            float(combined.get("covariance_inflation", 1.0)),
            min(MAX_COVARIANCE_INFLATION, 1.0 / max(0.05, factor_weight)),
        )
        combined["factor_enabled"] = bool(
            combined.get("factor_enabled", False)
            and combined["reliability_weight"] > 0.0
        )
    combined["reasons"] = tuple(dict.fromkeys(reasons))
    return combined


def visual_factor_score_for_mode(reliability_mode, matched_score):
    """Resolve candidate quality without mixing fixed and dynamic ablations."""
    mode = str(reliability_mode).lower()
    if mode == "dynamic":
        return matched_score
    if mode == "fixed":
        return {
            "valid": True,
            "weight": 1.0,
            "degradation_score": 0.0,
            "reasons": ("fixed_reliability_mode",),
        }
    raise ValueError("reliability_mode must be dynamic or fixed")


def visual_factor_score_source_stamp(factor_score, candidate_stamp_s):
    """Use the matched score stamp, or the candidate stamp in fixed mode."""
    return float(factor_score.get("source_stamp_s", candidate_stamp_s))


def visual_batch_information_scale(track_count, reference_tracks):
    """Cap correlated batch information at a reference track equivalent."""
    track_count = int(track_count)
    reference_tracks = int(reference_tracks)
    if track_count < 1 or reference_tracks < 1:
        raise ValueError("visual batch track counts must be positive")
    return max(1.0, track_count / reference_tracks)


def add_visual_observation_once(
    backend,
    previous_index,
    current_index,
    tracks,
    decision,
    add_rgbd_depth,
):
    """Insert exactly one factor representation for one D435i batch."""
    if bool(add_rgbd_depth()):
        return "rgbd_depth"
    backend.add_visual_reprojection(
        previous_index, current_index, tracks, decision=decision
    )
    return "paper_reprojection"


def apply_flow_rotation_gate(decision, gate_result):
    """Apply the low-latency FCU rotation gate after scheduler weighting."""
    gated = copy.deepcopy(decision)
    rotation_weight = max(0.0, min(1.0, float(gate_result.weight)))
    gated["degradation_score"] = max(
        float(gated.get("degradation_score", 0.0)), 1.0 - rotation_weight
    )
    reasons = list(gated.get("reasons", ()))
    if gate_result.phase != "ACTIVE" and gate_result.reason not in reasons:
        reasons.append(gate_result.reason)
    gated["reasons"] = reasons
    if gate_result.hard_disabled or rotation_weight <= 0.0:
        gated["factor_enabled"] = False
        gated["reliability_weight"] = 0.0
        gated["covariance_inflation"] = MAX_COVARIANCE_INFLATION
        return gated
    gated["reliability_weight"] = min(
        float(gated.get("reliability_weight", 1.0)), rotation_weight
    )
    gated["covariance_inflation"] = max(
        float(gated.get("covariance_inflation", 1.0)),
        min(
            MAX_COVARIANCE_INFLATION,
            1.0 / max(0.05, rotation_weight),
        ),
    )
    gated["factor_enabled"] = bool(
        gated.get("factor_enabled", True)
        and gated["reliability_weight"] > 0.0
    )
    return gated


def apply_lidar_anchor_floor(
    decision, minimum_effective_weight=0.10, maximum_inflation=5.0,
):
    """Retain enough LiDAR information while no independent yaw source exists."""
    minimum_effective_weight = float(minimum_effective_weight)
    maximum_inflation = float(maximum_inflation)
    if not 0.0 < minimum_effective_weight <= 1.0:
        raise ValueError("minimum effective LiDAR weight must be in (0, 1]")
    if maximum_inflation < 1.0:
        raise ValueError("maximum LiDAR anchor inflation must be at least one")
    protected = dict(decision)
    original_enabled = bool(protected.get("factor_enabled", True))
    original_weight = float(protected.get("reliability_weight", 1.0))
    original_inflation = float(protected.get("covariance_inflation", 1.0))
    inflation = min(original_inflation, maximum_inflation)
    minimum_weight = min(1.0, minimum_effective_weight * inflation)
    protected["factor_enabled"] = True
    protected["covariance_inflation"] = inflation
    protected["reliability_weight"] = max(original_weight, minimum_weight)
    if (
        not original_enabled
        or protected["reliability_weight"] != original_weight
        or inflation != original_inflation
    ):
        protected["anchor_override"] = True
    return protected


def lidar_bypass_allowed(
        preserve_lio_anchor, lidar_score_fresh,
        imu_score_fresh, imu_factor_enabled):
    """Require explicit opt-in plus a live inertial orientation backup."""
    return bool(
        not preserve_lio_anchor
        and lidar_score_fresh
        and imu_score_fresh
        and imu_factor_enabled
    )


def gnss_jump_rejected(current_position, gnss_position, gate_m=20.0):
    current = np.asarray(current_position, dtype=float)
    measurement = np.asarray(gnss_position, dtype=float)
    if current.shape != (3,) or measurement.shape != (3,):
        raise ValueError("GNSS jump gate expects two 3-vectors")
    if not np.all(
            np.isfinite(current)) or not np.all(
            np.isfinite(measurement)):
        return True
    return float(np.linalg.norm(current - measurement)) > float(gate_m)


def gnss_temporal_jump_rejected(
    previous_position,
    previous_stamp_s,
    current_position,
    current_stamp_s,
    gate_m=20.0,
    maximum_speed_mps=15.0,
):
    """Reject a GNSS jump using accepted raw fixes, not the fused estimate.

    Comparing a fix with a LiDAR-corrupted state creates a feedback failure:
    once LiDAR drifts, valid GNSS is mislabeled as a jump and can no longer
    recover the estimator. A temporal gate remains independent of that state.
    """
    if previous_position is None:
        return False
    previous = np.asarray(previous_position, dtype=float)
    current = np.asarray(current_position, dtype=float)
    if previous.shape != (3,) or current.shape != (3,):
        raise ValueError("GNSS temporal jump gate expects two 3-vectors")
    if not np.all(np.isfinite(previous)) or not np.all(np.isfinite(current)):
        return True
    previous_stamp_s = float(previous_stamp_s)
    current_stamp_s = float(current_stamp_s)
    if not math.isfinite(
            previous_stamp_s) or not math.isfinite(current_stamp_s):
        return True
    dt_s = current_stamp_s - previous_stamp_s
    if dt_s <= 0.0:
        return True
    gate_m = float(gate_m)
    maximum_speed_mps = float(maximum_speed_mps)
    if not math.isfinite(gate_m) or gate_m <= 0.0:
        raise ValueError("GNSS temporal jump distance gate must be positive")
    if not math.isfinite(maximum_speed_mps) or maximum_speed_mps <= 0.0:
        raise ValueError("GNSS temporal jump speed gate must be positive")
    allowed_distance = max(gate_m, maximum_speed_mps * dt_s)
    return float(np.linalg.norm(current - previous)) > allowed_distance


def select_gnss_observation(
    observations, state_stamp_s, maximum_age_s, future_tolerance_s,
):
    """Consume at most one GNSS fix for a state and never reuse a fix.

    The newest eligible fix is paired with the state. Older eligible fixes are
    superseded, while future fixes remain queued for a later state.
    """
    state_stamp_s = float(state_stamp_s)
    maximum_age_s = float(maximum_age_s)
    future_tolerance_s = float(future_tolerance_s)
    if (
        not math.isfinite(state_stamp_s)
        or not math.isfinite(maximum_age_s)
        or maximum_age_s <= 0.0
        or not math.isfinite(future_tolerance_s)
        or future_tolerance_s < 0.0
    ):
        raise ValueError("invalid GNSS observation selection limits")

    ordered = sorted(observations, key=lambda item: float(item["stamp_s"]))
    observations.clear()
    eligible = []
    stale_count = 0
    for observation in ordered:
        observation_stamp = float(observation["stamp_s"])
        age_s = state_stamp_s - observation_stamp
        if age_s > maximum_age_s:
            stale_count += 1
        elif observation_stamp <= state_stamp_s + future_tolerance_s:
            eligible.append(observation)
        else:
            observations.append(observation)
    if not eligible:
        return None, stale_count, 0
    return eligible[-1], stale_count, len(eligible) - 1


def time_compensate_gnss_observation(
    measured_position,
    measurement_covariance,
    observation_prediction,
    observation_prediction_covariance,
    factor_prediction,
    factor_prediction_covariance,
):
    """Transport a GNSS fix to the factor time with predicted motion."""
    measured = np.asarray(measured_position, dtype=float)
    measurement_variance = np.asarray(measurement_covariance, dtype=float)
    observation = np.asarray(observation_prediction, dtype=float)
    factor = np.asarray(factor_prediction, dtype=float)
    observation_covariance = np.asarray(
        observation_prediction_covariance, dtype=float
    )
    factor_covariance = np.asarray(
        factor_prediction_covariance, dtype=float
    )
    if (
        measured.shape != (3,)
        or measurement_variance.shape != (3,)
        or observation.shape != (3,)
        or factor.shape != (3,)
        or observation_covariance.shape != (3, 3)
        or factor_covariance.shape != (3, 3)
        or np.any(~np.isfinite(measured))
        or np.any(~np.isfinite(measurement_variance))
        or np.any(measurement_variance <= 0.0)
        or np.any(~np.isfinite(observation))
        or np.any(~np.isfinite(factor))
        or np.any(~np.isfinite(observation_covariance))
        or np.any(~np.isfinite(factor_covariance))
    ):
        raise ValueError("GNSS time compensation inputs are invalid")
    predicted_delta = factor - observation
    transport_variance = np.maximum(
        np.diag(factor_covariance) - np.diag(observation_covariance),
        0.0,
    )
    return (
        measured + predicted_delta,
        measurement_variance + transport_variance,
        predicted_delta,
    )


def gnss_covariance_diagonal(
    covariance, covariance_type, default_variance, minimum_variance=0.04,
):
    values = np.asarray(covariance, dtype=float)
    if values.size < 9:
        raise ValueError("GNSS covariance must contain nine entries")
    default_variance = float(default_variance)
    minimum_variance = float(minimum_variance)
    if (
        not math.isfinite(default_variance)
        or default_variance <= 0.0
        or not math.isfinite(minimum_variance)
        or minimum_variance <= 0.0
    ):
        raise ValueError("GNSS covariance limits must be positive")
    diagonal = values[[0, 4, 8]]
    if (
        int(covariance_type) == NavSatFix.COVARIANCE_TYPE_UNKNOWN
        or np.any(~np.isfinite(diagonal))
        or np.any(diagonal <= 0.0)
    ):
        return np.full(3, default_variance, dtype=float)
    return np.maximum(diagonal, minimum_variance)


def gnss_prefit_statistics(
    predicted_position,
    predicted_position_covariance,
    measured_position,
    measurement_covariance,
):
    """Return the GNSS prefit residual, innovation covariance, and NIS.

    The innovation covariance is S = P_position_predicted + R_gnss. Using R
    alone makes a valid fix look inconsistent whenever the propagated state is
    uncertain, which is exactly when an aiding factor is most valuable.
    """
    predicted = np.asarray(predicted_position, dtype=float)
    measured = np.asarray(measured_position, dtype=float)
    predicted_covariance = np.asarray(
        predicted_position_covariance, dtype=float
    )
    measurement_variance = np.asarray(measurement_covariance, dtype=float)
    if predicted.shape != (3,) or measured.shape != (3,):
        raise ValueError("GNSS prefit positions must be finite 3-vectors")
    if predicted_covariance.shape != (3, 3):
        raise ValueError("GNSS predicted position covariance must be 3x3")
    if measurement_variance.shape != (3,):
        raise ValueError("GNSS measurement covariance must be a 3-vector")
    if (
        np.any(~np.isfinite(predicted))
        or np.any(~np.isfinite(measured))
        or np.any(~np.isfinite(predicted_covariance))
        or np.any(~np.isfinite(measurement_variance))
        or np.any(measurement_variance <= 0.0)
    ):
        raise ValueError("GNSS prefit inputs must be finite and positive")
    predicted_covariance = 0.5 * (
        predicted_covariance + predicted_covariance.T
    )
    innovation_covariance = (
        predicted_covariance + np.diag(measurement_variance)
    )
    innovation_covariance = 0.5 * (
        innovation_covariance + innovation_covariance.T
    )
    residual = predicted - measured
    cholesky = np.linalg.cholesky(innovation_covariance)
    whitened = np.linalg.solve(cholesky, residual)
    nis = float(whitened @ whitened)
    if not math.isfinite(nis) or nis < 0.0:
        raise ValueError("GNSS prefit NIS is invalid")
    return residual, innovation_covariance, nis


def gnss_prefit_axis_nis(residual, innovation_covariance):
    """Return marginal horizontal (2-DoF) and vertical (1-DoF) NIS."""
    residual = np.asarray(residual, dtype=float)
    covariance = np.asarray(innovation_covariance, dtype=float)
    if residual.shape != (3,) or covariance.shape != (3, 3):
        raise ValueError("GNSS axis NIS requires a 3-vector and 3x3 covariance")
    if np.any(~np.isfinite(residual)) or np.any(~np.isfinite(covariance)):
        raise ValueError("GNSS axis NIS inputs must be finite")
    covariance = 0.5 * (covariance + covariance.T)
    horizontal_cholesky = np.linalg.cholesky(covariance[:2, :2])
    horizontal_whitened = np.linalg.solve(
        horizontal_cholesky, residual[:2]
    )
    vertical_variance = float(covariance[2, 2])
    if not math.isfinite(vertical_variance) or vertical_variance <= 0.0:
        raise ValueError("GNSS vertical innovation variance must be positive")
    horizontal_nis = float(horizontal_whitened @ horizontal_whitened)
    vertical_nis = float(residual[2] * residual[2] / vertical_variance)
    if (
        not math.isfinite(horizontal_nis)
        or horizontal_nis < 0.0
        or not math.isfinite(vertical_nis)
        or vertical_nis < 0.0
    ):
        raise ValueError("GNSS axis NIS is invalid")
    return horizontal_nis, vertical_nis


def gnss_axis_information_scale(nis, gate, minimum_scale=0.01):
    """Return a Huber-style information scale for one GNSS axis block.

    A valid, continuous GNSS observation must retain nonzero influence even
    when the estimator prediction has drifted outside the nominal NIS gate.
    Hard integrity failures are handled before this function is called.
    """
    nis = float(nis)
    gate = float(gate)
    minimum_scale = float(minimum_scale)
    if (
        not math.isfinite(nis)
        or nis < 0.0
        or not math.isfinite(gate)
        or gate <= 0.0
        or not math.isfinite(minimum_scale)
        or not 0.0 < minimum_scale <= 1.0
    ):
        raise ValueError("GNSS robust information inputs are invalid")
    if nis <= gate:
        return 1.0
    return max(minimum_scale, math.sqrt(gate / nis))


def bounded_axis_reanchor_target(prediction, measurement, maximum_step):
    """Move one absolute-axis target toward a measurement by a bounded step."""
    prediction = float(prediction)
    measurement = float(measurement)
    maximum_step = float(maximum_step)
    if (
        not math.isfinite(prediction)
        or not math.isfinite(measurement)
        or not math.isfinite(maximum_step)
        or maximum_step <= 0.0
    ):
        raise ValueError("axis reanchor inputs are invalid")
    innovation = measurement - prediction
    step = min(maximum_step, max(-maximum_step, innovation))
    return prediction + step, abs(innovation) > maximum_step


def pose_translation_profile_information(pose_hessian):
    """Profile each translation axis after eliminating the other pose axes."""
    information = np.asarray(pose_hessian, dtype=float)
    if information.shape != (6, 6) or np.any(~np.isfinite(information)):
        raise ValueError("pose information must be a finite 6x6 matrix")
    information = 0.5 * (information + information.T)
    profile = np.zeros(3, dtype=float)
    for axis in range(3):
        nuisance = np.asarray(
            [index for index in range(6) if index != axis], dtype=int
        )
        nuisance_information = information[np.ix_(nuisance, nuisance)]
        coupling = information[nuisance, axis]
        value = float(information[axis, axis]) - float(
            coupling
            @ np.linalg.pinv(nuisance_information, rcond=1.0e-9)
            @ coupling
        )
        profile[axis] = max(
            0.0, min(max(0.0, float(information[axis, axis])), value)
        )
    return profile


def scale_conditional_translation_normal(
    pose_hessian,
    pose_gradient,
    translation_information_scale,
):
    """Scale conditional translation information without weakening rotation."""
    information = np.asarray(pose_hessian, dtype=float)
    gradient = np.asarray(pose_gradient, dtype=float).reshape(-1)
    scales = np.asarray(translation_information_scale, dtype=float)
    if information.shape != (6, 6) or gradient.shape != (6,):
        raise ValueError("conditional pose normal requires 6-DoF inputs")
    if (
        np.any(~np.isfinite(information))
        or np.any(~np.isfinite(gradient))
        or scales.shape != (3,)
        or np.any(~np.isfinite(scales))
        or np.any(scales <= 0.0)
        or np.any(scales > 1.0)
    ):
        raise ValueError("conditional translation scaling inputs are invalid")
    information = 0.5 * (information + information.T)
    coupling = information[:3, 3:]
    rotation = information[3:, 3:]
    rotation_inverse = np.linalg.pinv(rotation, rcond=1.0e-9)
    schur = (
        information[:3, :3]
        - coupling @ rotation_inverse @ coupling.T
    )
    schur = 0.5 * (schur + schur.T)
    conditional_gradient = (
        gradient[:3] - coupling @ rotation_inverse @ gradient[3:]
    )
    root_scale = np.diag(np.sqrt(scales))
    scaled_schur = root_scale @ schur @ root_scale
    # Preserve the LiDAR factor's conditional optimum while reducing its
    # information.  A badly conditioned scan can otherwise encode an enormous
    # conditional correction even after its weak axis has been downweighted;
    # bound that correction before projecting it into the reduced normal.
    conditional_delta = np.linalg.pinv(schur, rcond=1.0e-9) @ (
        conditional_gradient
    )
    conditional_delta = np.clip(conditional_delta, -0.5, 0.5)
    scaled_conditional_gradient = scaled_schur @ conditional_delta

    scaled_information = information.copy()
    scaled_information[:3, :3] = (
        scaled_schur + coupling @ rotation_inverse @ coupling.T
    )
    scaled_information = 0.5 * (
        scaled_information + scaled_information.T
    )
    scaled_gradient = gradient.copy()
    scaled_gradient[:3] = (
        scaled_conditional_gradient
        + coupling @ rotation_inverse @ gradient[3:]
    )
    return scaled_information, scaled_gradient


def axis_observability_latch(
    support,
    latched,
    *,
    enter_support=0.35,
    exit_support=0.45,
):
    """Apply per-axis hysteresis without depending on another sensor."""
    support = np.asarray(support, dtype=float)
    latched = np.asarray(latched, dtype=bool)
    if support.shape != (3,) or latched.shape != (3,):
        raise ValueError("axis observability inputs must be 3-vectors")
    if (
        np.any(~np.isfinite(support))
        or np.any(support < 0.0)
        or np.any(support > 1.0)
        or not 0.0 <= enter_support < exit_support <= 1.0
    ):
        raise ValueError("axis observability evidence is invalid")
    return np.where(
        latched,
        support < float(exit_support),
        support < float(enter_support),
    )


def axis_map_protection(
    weak_axes,
    gnss_residual_xyz,
    *,
    gnss_fresh,
    barometer_active,
    gnss_disagreement_m=0.20,
):
    """Protect irreversible map writes while an independent axis disagrees."""
    weak_axes = np.asarray(weak_axes, dtype=bool)
    gnss_residual = np.asarray(gnss_residual_xyz, dtype=float)
    threshold = float(gnss_disagreement_m)
    if weak_axes.shape != (3,) or gnss_residual.shape != (3,):
        raise ValueError("axis map protection inputs must be 3-vectors")
    if not math.isfinite(threshold) or threshold <= 0.0:
        raise ValueError("axis map protection threshold must be positive")

    protected = np.zeros(3, dtype=bool)
    sources = ["none", "none", "none"]
    if bool(gnss_fresh):
        for axis in range(3):
            if (
                weak_axes[axis]
                and math.isfinite(float(gnss_residual[axis]))
                and abs(float(gnss_residual[axis])) >= threshold
            ):
                protected[axis] = True
                sources[axis] = "gnss_disagreement"
    if weak_axes[2] and bool(barometer_active):
        protected[2] = True
        sources[2] = (
            "barometer_fallback"
            if sources[2] == "none"
            else f"{sources[2]}+barometer_fallback"
        )
    return protected, tuple(sources)


def axis_information_handoff(
    lidar_information,
    lidar_support,
    alternative_information,
    latched,
    *,
    enabled_axes=(True, True, True),
    enter_support=0.15,
    exit_support=0.35,
    minimum_lidar_information_scale=1.0e-4,
    maximum_lidar_to_alternative_ratio=1.0,
):
    """Limit only weak LiDAR axes when a fresh alternative can own the axis.

    Sensor health and factor consistency remain separate admission layers. An
    axis is handed off only after its LiDAR observability crosses the enter
    threshold, and is restored after the higher exit threshold. If the
    alternative disappears, LiDAR immediately returns as the finite fallback.
    """
    lidar_information = np.asarray(lidar_information, dtype=float)
    lidar_support = np.asarray(lidar_support, dtype=float)
    alternative_information = np.asarray(
        alternative_information, dtype=float
    )
    latched = np.asarray(latched, dtype=bool)
    enabled_axes = np.asarray(enabled_axes, dtype=bool)
    if any(
        values.shape != (3,)
        for values in (
            lidar_information,
            lidar_support,
            alternative_information,
            latched,
            enabled_axes,
        )
    ):
        raise ValueError("axis handoff inputs must be 3-vectors")
    if (
        np.any(~np.isfinite(lidar_information))
        or np.any(lidar_information < 0.0)
        or np.any(~np.isfinite(lidar_support))
        or np.any(lidar_support < 0.0)
        or np.any(lidar_support > 1.0)
        or np.any(~np.isfinite(alternative_information))
        or np.any(alternative_information < 0.0)
    ):
        raise ValueError("axis handoff evidence must be finite and nonnegative")
    enter_support = float(enter_support)
    exit_support = float(exit_support)
    minimum_scale = float(minimum_lidar_information_scale)
    maximum_ratio = float(maximum_lidar_to_alternative_ratio)
    if (
        not 0.0 <= enter_support < exit_support <= 1.0
        or not 0.0 < minimum_scale <= 1.0
        or not math.isfinite(maximum_ratio)
        or maximum_ratio <= 0.0
    ):
        raise ValueError("axis handoff thresholds are invalid")

    scales = np.ones(3, dtype=float)
    next_latched = latched.copy()
    for axis in range(3):
        if not enabled_axes[axis]:
            next_latched[axis] = False
            continue
        alternative_available = alternative_information[axis] > 0.0
        if not alternative_available:
            next_latched[axis] = False
            continue
        if latched[axis]:
            next_latched[axis] = lidar_support[axis] < exit_support
        else:
            next_latched[axis] = lidar_support[axis] < enter_support
        if not next_latched[axis]:
            continue
        if lidar_information[axis] <= 1.0e-12:
            scales[axis] = minimum_scale
            continue
        target_information = maximum_ratio * alternative_information[axis]
        scales[axis] = min(
            1.0,
            max(minimum_scale, target_information / lidar_information[axis]),
        )
    return scales, next_latched


def apply_gnss_prefit_gate(
    scheduler_factor_decision,
    prefit_xy_nis,
    prefit_z_nis,
    xy_nis_gate=9.210,
    z_nis_gate=6.635,
    minimum_reliability_weight=0.05,
    minimum_axis_information_scale=0.01,
):
    """Apply factor consistency after the scheduler's sensor-health policy.

    The scheduler remains authoritative for sensor health. Current-observation
    innovation is applied once here as a per-axis robust information scale.
    This avoids the self-locking failure where an estimator drift causes GNSS
    to be removed precisely when it is needed to recover absolute position.
    """
    xy_nis_gate = float(xy_nis_gate)
    z_nis_gate = float(z_nis_gate)
    minimum_reliability_weight = float(minimum_reliability_weight)
    minimum_axis_information_scale = float(
        minimum_axis_information_scale
    )
    prefit_xy_nis = float(prefit_xy_nis)
    prefit_z_nis = float(prefit_z_nis)
    if (
        not math.isfinite(xy_nis_gate)
        or xy_nis_gate <= 0.0
        or not math.isfinite(z_nis_gate)
        or z_nis_gate <= 0.0
    ):
        raise ValueError("GNSS axis NIS gates must be positive")
    if not 0.0 < minimum_reliability_weight <= 1.0:
        raise ValueError("GNSS minimum reliability weight must be in (0, 1]")
    if not 0.0 < minimum_axis_information_scale <= 1.0:
        raise ValueError(
            "GNSS minimum axis information scale must be in (0, 1]"
        )
    if (
        not math.isfinite(prefit_xy_nis)
        or prefit_xy_nis < 0.0
        or not math.isfinite(prefit_z_nis)
        or prefit_z_nis < 0.0
    ):
        raise ValueError("GNSS prefit axis NIS must be finite and nonnegative")

    decision = dict(scheduler_factor_decision)
    scheduler_enabled = bool(decision.get("factor_enabled", False))
    scheduler_weight = max(
        0.0, min(1.0, float(decision.get("reliability_weight", 0.0)))
    )
    scheduler_inflation = max(
        1.0,
        min(
            MAX_COVARIANCE_INFLATION,
            float(decision.get("covariance_inflation", 1.0)),
        ),
    )
    xy_admitted = prefit_xy_nis <= xy_nis_gate
    z_admitted = prefit_z_nis <= z_nis_gate
    xy_information_scale = gnss_axis_information_scale(
        prefit_xy_nis, xy_nis_gate, minimum_axis_information_scale
    )
    z_information_scale = gnss_axis_information_scale(
        prefit_z_nis, z_nis_gate, minimum_axis_information_scale
    )
    decision.update({
        "prefit_xy_nis": prefit_xy_nis,
        "prefit_z_nis": prefit_z_nis,
        "gnss_xy_admitted": xy_admitted,
        "gnss_z_admitted": z_admitted,
        "gnss_xy_information_scale": xy_information_scale,
        "gnss_z_information_scale": z_information_scale,
        "gnss_recovery_floor": False,
    })
    if not scheduler_enabled:
        decision.update({
            "factor_enabled": False,
            "reliability_weight": 0.0,
            "covariance_inflation": MAX_COVARIANCE_INFLATION,
            "admission_reason": "scheduler_disabled",
        })
        return decision
    if scheduler_weight < minimum_reliability_weight:
        decision.update({
            "factor_enabled": False,
            "reliability_weight": 0.0,
            "covariance_inflation": MAX_COVARIANCE_INFLATION,
            "admission_reason": "reliability_below_minimum",
        })
        return decision

    decision.update({
        "factor_enabled": True,
        "reliability_weight": scheduler_weight,
        "covariance_inflation": scheduler_inflation,
        "gnss_recovery_floor": not xy_admitted and not z_admitted,
        "admission_reason": (
            "admitted_all_axes"
            if xy_admitted and z_admitted
            else "admitted_xy_with_z_robust"
            if xy_admitted
            else "admitted_z_with_xy_robust"
            if z_admitted
            else "admitted_robust_all_axes"
        ),
    })
    return decision


def covariance_update_due(last_stamp_s, current_stamp_s, update_period_s):
    current_stamp_s = float(current_stamp_s)
    update_period_s = float(update_period_s)
    if not math.isfinite(
            current_stamp_s) or not math.isfinite(update_period_s):
        raise ValueError("covariance update timing must be finite")
    if update_period_s <= 0.0:
        raise ValueError("covariance update period must be positive")
    if last_stamp_s is None:
        return True
    last_stamp_s = float(last_stamp_s)
    return current_stamp_s < last_stamp_s or (
        current_stamp_s - last_stamp_s >= update_period_s
    )


def path_sample_due(
    previous_position, previous_orientation, position, orientation,
    minimum_translation_m, minimum_rotation_rad,
):
    if previous_position is None or previous_orientation is None:
        return True
    previous_position = np.asarray(previous_position, dtype=float)
    previous_orientation = np.asarray(previous_orientation, dtype=float)
    position = np.asarray(position, dtype=float)
    orientation = np.asarray(orientation, dtype=float)
    if any(value.shape != (3,) for value in (
        previous_position, previous_orientation, position, orientation,
    )):
        raise ValueError("path sampling expects finite 3-vectors")
    if any(np.any(~np.isfinite(value)) for value in (
        previous_position, previous_orientation, position, orientation,
    )):
        raise ValueError("path sampling expects finite values")
    translation = float(np.linalg.norm(position - previous_position))
    rotation_delta = np.asarray([
        math.atan2(math.sin(delta), math.cos(delta))
        for delta in orientation - previous_orientation
    ])
    return bool(
        translation >= float(minimum_translation_m)
        or np.linalg.norm(rotation_delta) >= float(minimum_rotation_rad)
    )


def fused_motion_reference(previous_state, dt_s):
    """Propagate a short-horizon gate reference without using current LIO."""
    state = np.asarray(previous_state, dtype=float)
    dt_s = float(dt_s)
    if state.shape != (15,) or not np.all(np.isfinite(state)):
        raise ValueError("previous fused state must be a finite 15-vector")
    if not math.isfinite(dt_s) or dt_s <= 0.0:
        raise ValueError("propagation interval must be finite and positive")
    delta_position = state[6:9] * dt_s
    return {
        "position": state[:3] + delta_position,
        "delta_position": delta_position,
        "orientation": state[3:6].copy(),
        "yaw": float(state[5]),
    }


def manifold_motion_reference(previous_state, predicted_state):
    """Describe one backend-owned IMU propagation for gating and aiding."""
    previous = np.asarray(previous_state, dtype=float)
    predicted = np.asarray(predicted_state, dtype=float)
    if previous.shape != (15,) or predicted.shape != (15,):
        raise ValueError("manifold motion reference expects two 15-vectors")
    if np.any(~np.isfinite(previous)) or np.any(~np.isfinite(predicted)):
        raise ValueError("manifold motion reference must be finite")
    return {
        "position": predicted[:3].copy(),
        "delta_position": predicted[:3] - previous[:3],
        "orientation": predicted[3:6].copy(),
        "yaw": float(predicted[5]),
    }


def imu_interval_covered(latest_imu_stamp, target_stamp):
    if latest_imu_stamp is None:
        return False
    latest_imu_stamp = float(latest_imu_stamp)
    target_stamp = float(target_stamp)
    return (
        math.isfinite(latest_imu_stamp)
        and math.isfinite(target_stamp)
        and latest_imu_stamp >= target_stamp
    )


def committed_state_missing_imu_factor(
    state_committed,
    imu_factor_expected,
    factor_count_before,
    factor_count_after,
):
    """Report an actual missing IMU factor, not a preflight wait timeout."""
    return (
        bool(state_committed)
        and bool(imu_factor_expected)
        and int(factor_count_after) <= int(factor_count_before)
    )


def ordered_imu_samples(samples):
    return sorted(samples, key=lambda sample: float(sample.stamp_s))


def imu_samples_for_interval(samples, start_stamp, end_stamp):
    """Select an interval plus one interpolation sample on each side."""
    start_stamp = float(start_stamp)
    end_stamp = float(end_stamp)
    if (
        not math.isfinite(start_stamp)
        or not math.isfinite(end_stamp)
        or end_stamp < start_stamp
    ):
        return []
    before = None
    after = None
    selected = []
    for sample in samples:
        stamp = float(sample.stamp_s)
        if not math.isfinite(stamp):
            continue
        if stamp < start_stamp:
            if before is None or stamp > float(before.stamp_s):
                before = sample
        elif stamp <= end_stamp:
            selected.append(sample)
        elif after is None or stamp < float(after.stamp_s):
            after = sample
    if before is not None:
        selected.append(before)
    if after is not None:
        selected.append(after)
    return sorted(
        {float(sample.stamp_s): sample for sample in selected}.values(),
        key=lambda sample: float(sample.stamp_s),
    )


def prune_imu_buffer_before(buffer, cutoff_stamp):
    """Drop obsolete monotonic samples while retaining interpolation history."""
    cutoff_stamp = float(cutoff_stamp)
    if not math.isfinite(cutoff_stamp):
        return 0
    removed = 0
    while len(buffer) > 2:
        first_stamp = float(buffer[0].stamp_s)
        second_stamp = float(buffer[1].stamp_s)
        if not math.isfinite(first_stamp) or not math.isfinite(second_stamp):
            break
        if second_stamp < first_stamp or second_stamp > cutoff_stamp:
            break
        buffer.popleft()
        removed += 1
    return removed


def imu_samples_covering_interval(ordered_samples, start_stamp, end_stamp):
    """Keep the requested IMU interval plus one interpolation sample per side."""
    start_stamp = float(start_stamp)
    end_stamp = float(end_stamp)
    if not ordered_samples or end_stamp < start_stamp:
        return []
    stamps = [float(sample.stamp_s) for sample in ordered_samples]
    begin = max(0, bisect_left(stamps, start_stamp) - 1)
    end = min(len(ordered_samples), bisect_right(stamps, end_stamp) + 1)
    return ordered_samples[begin:end]


def imu_interval_status(samples, start_stamp, end_stamp, maximum_gap_s=0.10):
    start_stamp = float(start_stamp)
    end_stamp = float(end_stamp)
    maximum_gap_s = float(maximum_gap_s)
    if (
        not math.isfinite(start_stamp)
        or not math.isfinite(end_stamp)
        or end_stamp <= start_stamp
    ):
        return False, "invalid_interval", 0.0
    if not math.isfinite(maximum_gap_s) or maximum_gap_s <= 0.0:
        raise ValueError("maximum IMU gap must be finite and positive")
    stamps = sorted({
        float(sample.stamp_s)
        for sample in samples
        if math.isfinite(float(sample.stamp_s))
    })
    if len(stamps) < 2:
        return False, "insufficient_samples", 0.0
    start_index = bisect_right(stamps, start_stamp) - 1
    end_index = bisect_left(stamps, end_stamp)
    if start_index < 0 or end_index >= len(stamps):
        return False, "interval_not_covered", 0.0
    interval_stamps = stamps[start_index:end_index + 1]
    gaps = np.diff(interval_stamps)
    largest_gap = float(np.max(gaps)) if len(gaps) else 0.0
    if largest_gap > maximum_gap_s:
        return False, "sample_gap_exceeds_limit", largest_gap
    return True, "ok", largest_gap


def estimate_stationary_imu_bias(
    samples,
    reference_orientation,
    end_stamp_s,
    window_s=1.5,
    minimum_samples=40,
    minimum_span_s=0.8,
    maximum_mean_gyro_radps=0.08,
    maximum_gyro_residual_rms_radps=0.03,
    gravity_mps2=9.81,
    gravity_tolerance_mps2=0.60,
    maximum_accel_residual_rms_mps2=0.40,
):
    """Estimate one startup bias from an observably stationary FCU IMU window.

    The LiDAR orientation supplies only the gravity direction.  FCU EKF pose is
    deliberately not consumed, and the returned bias is used only to seed the
    first backend state before normal between-keyframe preintegration begins.
    """
    end_stamp_s = float(end_stamp_s)
    window_s = float(window_s)
    minimum_span_s = float(minimum_span_s)
    gravity_mps2 = float(gravity_mps2)
    if (
        not math.isfinite(end_stamp_s)
        or not math.isfinite(window_s)
        or window_s <= 0.0
        or not math.isfinite(minimum_span_s)
        or minimum_span_s <= 0.0
        or minimum_span_s > window_s
        or int(minimum_samples) < 2
        or not math.isfinite(gravity_mps2)
        or gravity_mps2 <= 0.0
    ):
        raise ValueError("invalid stationary IMU initialization limits")

    selected_by_stamp = {}
    start_stamp_s = end_stamp_s - window_s
    for sample in samples:
        stamp_s = float(sample.stamp_s)
        acceleration = np.asarray(sample.acceleration, dtype=float)
        angular_velocity = np.asarray(sample.angular_velocity, dtype=float)
        if (
            math.isfinite(stamp_s)
            and start_stamp_s <= stamp_s <= end_stamp_s
            and acceleration.shape == (3,)
            and angular_velocity.shape == (3,)
            and np.all(np.isfinite(acceleration))
            and np.all(np.isfinite(angular_velocity))
        ):
            selected_by_stamp[stamp_s] = (acceleration, angular_velocity)
    selected = sorted(selected_by_stamp.items())

    zero = (0.0, 0.0, 0.0)
    if len(selected) < 2:
        return StationaryImuInitialization(
            False, "insufficient_observation_span", zero, zero,
            len(selected), 0.0, 0.0, 0.0, 0.0, 0.0,
        )
    span_s = float(selected[-1][0] - selected[0][0])
    if span_s < minimum_span_s:
        return StationaryImuInitialization(
            False, "insufficient_observation_span", zero, zero,
            len(selected), span_s, 0.0, 0.0, 0.0, 0.0,
        )
    if len(selected) < int(minimum_samples):
        return StationaryImuInitialization(
            False, "insufficient_samples", zero, zero,
            len(selected), span_s, 0.0, 0.0, 0.0, 0.0,
        )

    acceleration = np.stack([item[1][0] for item in selected])
    angular_velocity = np.stack([item[1][1] for item in selected])
    mean_acceleration = np.mean(acceleration, axis=0)
    mean_angular_velocity = np.mean(angular_velocity, axis=0)
    accel_norm = float(np.linalg.norm(mean_acceleration))
    gyro_norm = float(np.linalg.norm(mean_angular_velocity))
    accel_residual_rms = float(np.sqrt(np.mean(np.sum(
        (acceleration - mean_acceleration) ** 2, axis=1
    ))))
    gyro_residual_rms = float(np.sqrt(np.mean(np.sum(
        (angular_velocity - mean_angular_velocity) ** 2, axis=1
    ))))

    reason = "ok"
    if gyro_norm > float(maximum_mean_gyro_radps):
        reason = "mean_angular_rate_exceeds_limit"
    elif gyro_residual_rms > float(maximum_gyro_residual_rms_radps):
        reason = "angular_rate_variation_exceeds_limit"
    elif abs(accel_norm - gravity_mps2) > float(gravity_tolerance_mps2):
        reason = "specific_force_not_gravity"
    elif accel_residual_rms > float(maximum_accel_residual_rms_mps2):
        reason = "specific_force_variation_exceeds_limit"

    if reason != "ok":
        return StationaryImuInitialization(
            False, reason, zero, zero, len(selected), span_s,
            accel_norm, accel_residual_rms, gyro_norm, gyro_residual_rms,
        )

    orientation = np.asarray(reference_orientation, dtype=float)
    if orientation.shape != (3,) or np.any(~np.isfinite(orientation)):
        raise ValueError("reference orientation must be a finite RPY vector")
    expected_specific_force = (
        rpy_to_rotation_matrix(orientation).T
        @ np.asarray([0.0, 0.0, gravity_mps2], dtype=float)
    )
    accel_bias = mean_acceleration - expected_specific_force
    return StationaryImuInitialization(
        True,
        "ok",
        tuple(float(value) for value in accel_bias),
        tuple(float(value) for value in mean_angular_velocity),
        len(selected),
        span_s,
        accel_norm,
        accel_residual_rms,
        gyro_norm,
        gyro_residual_rms,
    )


def inflate_manifold_imu_covariance(
    covariance, motion_scale, minimum_bias_variance,
):
    """Inflate a full IMU covariance by congruence while preserving SPD."""
    values = np.asarray(covariance, dtype=float)
    if values.shape == (15,):
        values = np.diag(values)
    elif values.size == 225:
        values = values.reshape(15, 15)
    if values.shape != (15, 15) or np.any(~np.isfinite(values)):
        raise ValueError(
            "manifold IMU covariance must be a finite 15x15 matrix")
    motion_scale = float(motion_scale)
    minimum_bias_variance = float(minimum_bias_variance)
    if (
        not math.isfinite(motion_scale)
        or motion_scale < 1.0
        or not math.isfinite(minimum_bias_variance)
        or minimum_bias_variance <= 0.0
    ):
        raise ValueError("IMU covariance inflation limits are invalid")
    scale = np.ones(15, dtype=float)
    scale[:9] = math.sqrt(motion_scale)
    inflated = scale[:, None] * values * scale[None, :]
    bias_diagonal = np.diag(inflated)[9:15]
    diagonal_addition = np.maximum(
        minimum_bias_variance - bias_diagonal, 0.0
    )
    inflated[9:15, 9:15] += np.diag(diagonal_addition)
    inflated = 0.5 * (inflated + inflated.T)
    try:
        np.linalg.cholesky(inflated)
    except np.linalg.LinAlgError as error:
        raise ValueError(
            "inflated IMU covariance is not positive definite") from error
    return inflated


def lidar_prediction_innovation(position, yaw, reference):
    """Compare LIO with a short-horizon prediction formed before current LIO."""
    current = np.asarray(position, dtype=float)
    predicted = np.asarray(reference["position"], dtype=float)
    if current.shape != (3,) or predicted.shape != (3,):
        raise ValueError("LiDAR prediction innovation expects two 3-vectors")
    if not np.all(np.isfinite(current)) or not np.all(np.isfinite(predicted)):
        raise ValueError("LiDAR prediction innovation must be finite")
    yaw_error = math.atan2(
        math.sin(float(yaw) - float(reference["yaw"])),
        math.cos(float(yaw) - float(reference["yaw"])),
    )
    return {
        "position_m": float(np.linalg.norm(current - predicted)),
        "yaw_rad": abs(float(yaw_error)),
    }


def lidar_prediction_gate(
    innovation, maximum_position_m, maximum_yaw_rad,
):
    """Reject a native LiDAR frame before its local geometry enters the window.

    The native packet is linearized in FAST-LIO's local map.  A large mismatch
    with the backend IMU prediction is therefore a frame/map-consistency fault,
    not a correction that the optimizer should be allowed to absorb.
    """
    if not isinstance(innovation, dict):
        raise ValueError("LiDAR prediction innovation must be a mapping")
    position = float(innovation.get("position_m", math.inf))
    yaw = float(innovation.get("yaw_rad", math.inf))
    maximum_position_m = float(maximum_position_m)
    maximum_yaw_rad = float(maximum_yaw_rad)
    if (
        not math.isfinite(position) or position < 0.0
        or not math.isfinite(yaw) or yaw < 0.0
        or not math.isfinite(maximum_position_m) or maximum_position_m <= 0.0
        or not math.isfinite(maximum_yaw_rad) or maximum_yaw_rad <= 0.0
    ):
        return False, "invalid_lidar_prediction_innovation"
    if position > maximum_position_m:
        return False, "lidar_prediction_position_gate"
    if yaw > maximum_yaw_rad:
        return False, "lidar_prediction_yaw_gate"
    return True, "ok"


def lidar_prediction_factor_admission(
    innovation,
    maximum_position_m,
    maximum_yaw_rad,
    consecutive_rejections=0,
    recovery_after_rejections=3,
    recovery_geometry_usable=False,
):
    """Apply the prediction check to one LiDAR factor, not the estimator.

    A rejected local-map factor must not stop IMU propagation or prevent GNSS
    and optical-flow factors at the same timestamp from updating the window.
    The next LiDAR packet is evaluated independently so a transient mismatch
    can recover without waiting for a relocalization reset. Local point-plane
    rank cannot prove that FAST-LIO's local map is aligned with the backend
    map, so a factor that fails this frame-consistency gate must never be
    reintroduced merely because its local geometry is well conditioned.  The
    recovery arguments remain in the helper contract for configuration
    compatibility; they cannot override a failed gate.
    """
    allowed, reason = lidar_prediction_gate(
        innovation, maximum_position_m, maximum_yaw_rad
    )
    previous = max(0, int(consecutive_rejections))
    recovery_after = int(recovery_after_rejections)
    if recovery_after < 1:
        raise ValueError("LiDAR recovery rejection count must be positive")
    if allowed:
        return {
            "factor_enabled": True,
            "reason": "ok",
            "consecutive_rejections": 0,
            "recovered": previous > 0,
            "recovery_floor": False,
        }
    consecutive = previous + 1
    return {
        "factor_enabled": False,
        "reason": reason,
        "consecutive_rejections": consecutive,
        "recovered": False,
        "recovery_floor": False,
    }


def mtf01p_range_sigma_m(
        distance_m,
        near_limit_m=2.0,
        near_sigma_m=0.04,
        far_relative_sigma=0.02):
    """Return the MTF-01P ranging accuracy as a one-sigma model.

    The vendor specification is 4 cm through 2 m and 2% above 2 m.  Keeping
    this as an explicit measurement model lets simulation and hardware use the
    same backend without pretending that the range is exact.
    """
    distance_m = float(distance_m)
    near_limit_m = float(near_limit_m)
    near_sigma_m = float(near_sigma_m)
    far_relative_sigma = float(far_relative_sigma)
    if (
        not math.isfinite(distance_m)
        or distance_m <= 0.0
        or near_limit_m <= 0.0
        or near_sigma_m <= 0.0
        or far_relative_sigma <= 0.0
    ):
        return math.inf
    if distance_m <= near_limit_m:
        return near_sigma_m
    return far_relative_sigma * distance_m


def optical_flow_displacement_covariance_m2(
        displacement_xy_m,
        distance_m,
        base_sigma_m=0.10,
        range_near_limit_m=2.0,
        range_near_sigma_m=0.04,
        range_far_relative_sigma=0.02):
    """Propagate range uncertainty into a conservative planar covariance."""
    displacement = np.asarray(displacement_xy_m, dtype=float).reshape(-1)
    distance_m = float(distance_m)
    base_sigma_m = float(base_sigma_m)
    if (
        displacement.size < 2
        or not np.all(np.isfinite(displacement[:2]))
        or not math.isfinite(distance_m)
        or distance_m <= 0.0
        or not math.isfinite(base_sigma_m)
        or base_sigma_m <= 0.0
    ):
        return None
    range_sigma_m = mtf01p_range_sigma_m(
        distance_m,
        range_near_limit_m,
        range_near_sigma_m,
        range_far_relative_sigma,
    )
    if not math.isfinite(range_sigma_m):
        return None
    relative_range_sigma = range_sigma_m / distance_m
    # Use an isotropic contribution based on total planar displacement. This
    # remains valid for both body-frame and map-frame factor variants.
    range_displacement_sigma = (
        float(np.linalg.norm(displacement[:2])) * relative_range_sigma
    )
    variance = base_sigma_m ** 2 + range_displacement_sigma ** 2
    return [variance, variance]


def mtf01p_flow_speed_gate(
        displacement_xy_m,
        integration_s,
        distance_m,
        maximum_speed_at_1m_mps=7.0,
        margin=1.10):
    """Check the MTF-01P angular-flow limit expressed as planar speed."""
    displacement = np.asarray(displacement_xy_m, dtype=float).reshape(-1)
    integration_s = float(integration_s)
    distance_m = float(distance_m)
    maximum_speed_at_1m_mps = float(maximum_speed_at_1m_mps)
    margin = float(margin)
    valid = (
        displacement.size >= 2
        and np.all(np.isfinite(displacement[:2]))
        and math.isfinite(integration_s)
        and integration_s > 0.0
        and math.isfinite(distance_m)
        and distance_m > 0.0
        and math.isfinite(maximum_speed_at_1m_mps)
        and maximum_speed_at_1m_mps > 0.0
        and math.isfinite(margin)
        and margin >= 1.0
    )
    if not valid:
        return False, math.inf, 0.0
    speed_mps = float(np.linalg.norm(displacement[:2])) / integration_s
    limit_mps = maximum_speed_at_1m_mps * distance_m * margin
    return speed_mps <= limit_mps, speed_mps, limit_mps


def flow_observation_delta(flow_records, yaw):
    """Aggregate valid MAVLink optical-flow increments into map ENU."""
    delta = np.zeros(2, dtype=float)
    delta_body = np.zeros(3, dtype=float)
    qualities = []
    distances = []
    total_integration_s = 0.0
    for flow in flow_records:
        distance = float(flow["distance_m"])
        if distance <= 0.0 or not math.isfinite(distance):
            continue
        displacement = optical_flow_displacement_frd(
            flow["integrated_x"], flow["integrated_y"],
            flow["integrated_xgyro"], flow["integrated_ygyro"],
            distance,
        )
        if displacement is None:
            continue
        delta += np.asarray(
            frd_to_enu_delta(displacement[0], displacement[1], yaw),
            dtype=float,
        )
        delta_body += np.asarray(
            [float(displacement[0]), -float(displacement[1]), 0.0],
            dtype=float,
        )
        qualities.append(float(flow["quality"]))
        distances.append(distance)
        integration_s = float(flow.get("integration_time_s", 0.0))
        if math.isfinite(integration_s) and integration_s > 0.0:
            total_integration_s += integration_s
    if not qualities:
        return None
    return {
        "delta_position": [float(delta[0]), float(delta[1]), 0.0],
        "delta_body": [float(value) for value in delta_body],
        "quality": float(np.mean(qualities)),
        "distance_m": float(np.mean(distances)),
        "sample_count": len(qualities),
        "integration_s": total_integration_s,
    }


def flow_los_observation(flow_records):
    """Aggregate APM-compensated flow LOS rates using exposure duration."""
    weighted_rates = []
    total_duration_s = 0.0
    weighted_distance = 0.0
    for flow in flow_records:
        try:
            integration_s = float(flow.get("integration_time_s", 0.0))
            distance_m = float(flow["distance_m"])
        except (KeyError, TypeError, ValueError):
            continue
        if (
            not math.isfinite(integration_s)
            or integration_s <= 0.0
            or not math.isfinite(distance_m)
            or distance_m <= 0.0
        ):
            continue
        rate = optical_flow_los_rate_apm(
            flow["integrated_x"], flow["integrated_y"],
            flow["integrated_xgyro"], flow["integrated_ygyro"],
            integration_s,
        )
        if rate is None:
            continue
        weighted_rates.append((rate, integration_s))
        total_duration_s += integration_s
        weighted_distance += distance_m * integration_s
    if not weighted_rates or total_duration_s <= 0.0:
        return None
    measurement = tuple(
        sum(rate[component] * duration for rate, duration in weighted_rates)
        / total_duration_s
        for component in range(2)
    )
    return {
        "measurement_radps": measurement,
        "distance_m": weighted_distance / total_duration_s,
        "integration_s": total_duration_s,
        "sample_count": len(weighted_rates),
    }


def _flow_record_is_future(item, current_stamp):
    try:
        stamp = float(item["stamp_s"])
    except (KeyError, TypeError, ValueError):
        return False
    return math.isfinite(stamp) and stamp > float(current_stamp)


def select_flow_records(
        flow_records,
        previous_stamp,
        current_stamp,
        max_age_s):
    """Select one flow interval without consuming future samples.

    The normal association interval is ``(previous_stamp, current_stamp]``.
    If it is empty, a recent late sample is used once as a bounded fallback.
    All samples at or before ``current_stamp`` are consumed so a late packet
    cannot be applied repeatedly to later intervals.
    """
    previous_stamp = float(previous_stamp)
    current_stamp = float(current_stamp)
    max_age_s = float(max_age_s)
    if (
        not math.isfinite(previous_stamp)
        or not math.isfinite(current_stamp)
        or not math.isfinite(max_age_s)
        or current_stamp <= previous_stamp
        or max_age_s < 0.0
    ):
        raise ValueError("invalid flow interval or age limit")

    records = list(flow_records)
    strict = []
    recent = []
    cutoff = current_stamp - max_age_s
    for item in records:
        try:
            stamp = float(item["stamp_s"])
        except (KeyError, TypeError, ValueError):
            continue
        if not math.isfinite(stamp):
            continue
        if previous_stamp < stamp <= current_stamp:
            strict.append(item)
        elif cutoff < stamp <= current_stamp:
            recent.append(item)

    if strict:
        return strict, [
            item for item in records if _flow_record_is_future(
                item, current_stamp)], False
    if recent:
        return recent, [
            item for item in records if _flow_record_is_future(
                item, current_stamp)], True
    return [], [
        item for item in records if _flow_record_is_future(
            item, current_stamp)], False


class UnifiedBackendNode(Node):
    def __init__(self):
        super().__init__("unified_backend_fusion")
        # The optimizer runs in its own worker thread. Keep high-rate ingress
        # callbacks in independent groups so a visual/calibration callback
        # cannot block delivery of the newest LiDAR factor or FCU IMU sample.
        self.native_callback_group = MutuallyExclusiveCallbackGroup()
        self.imu_callback_group = MutuallyExclusiveCallbackGroup()
        self.flow_callback_group = MutuallyExclusiveCallbackGroup()
        # Propagation is a publication-only safety path. Keep its timer out
        # of the default group so a slow visual/scheduler callback cannot
        # starve ExternalNav freshness while the worker is catching up.
        self.live_propagation_callback_group = (
            MutuallyExclusiveCallbackGroup()
        )
        self.imu_buffer_lock = threading.Lock()
        self.optimization_anchor_lock = threading.Lock()
        self.output_lock = threading.Lock()
        # Serializes optimizer commits, relocalization epochs and the final
        # generation check of publication-only IMU propagation.
        self.state_publication_lock = threading.RLock()
        defaults = {
            "lio_topic": "/lio/odom",
            "native_lidar_factor_topic": "/fast_lio/native_lidar_factor",
            "gnss_topic": "/sensors/gnss/fix",
            "barometer_topic": "/mavros/imu/static_pressure",
            "flow_topic": "/sensors/optical_flow/rad",
            "imu_topic": "/sensors/imu",
            "scheduler_topic": "/reliability/scheduler_state",
            "relocalization_result_topic": "/relocalization/result",
            "fusion_epoch_topic": "/fusion/unified/epoch",
            "relocalization_pending_timeout_s": 2.0,
            "relocalization_state_tolerance_s": 0.25,
            # Relocalization can be produced from a queued frontend/map
            # result while the single-writer backend is temporarily behind.
            # Keep the result causal by waiting for the backend timestamp;
            # never apply a future result directly to the current state.
            "relocalization_future_wait_timeout_s": 8.0,
            "relocalization_result_max_age_s": 2.0,
            "output_topic": "/fusion/unified/odom",
            "path_topic": "/fusion/unified/path",
            "diagnostic_topic": "/fusion/unified/diagnostics",
            "map_frame": "map",
            "body_frame": "base_link",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        # Keep high-rate flow ingress independent from native LiDAR and IMU;
        # the optimizer itself remains a single worker with one numeric thread.
        self.declare_parameter("executor_threads", 4)
        self.executor_threads = int(
            self.get_parameter("executor_threads").value
        )
        if self.executor_threads < 2:
            raise ValueError("the unified backend requires at least two executor threads")
        self.declare_parameter("flow_qos_depth", 32)
        self.flow_qos_depth = int(self.get_parameter("flow_qos_depth").value)
        if self.flow_qos_depth < 1:
            raise ValueError("flow_qos_depth must be positive")
        self.declare_parameter("window_size", 20)
        self.declare_parameter("backend_solver_mode", "manifold")
        self.declare_parameter("cpp_math_core_enabled", True)
        self.declare_parameter("cpp_math_core_required", False)
        self.declare_parameter("nonlinear_max_iterations", 2)
        self.declare_parameter("nonlinear_initialization_max_iterations", 4)
        self.declare_parameter("nonlinear_recovery_max_iterations", 4)
        self.declare_parameter("nonlinear_reintegration_max_iterations", 1)
        self.declare_parameter("nonlinear_damping", 1.0e-6)
        self.declare_parameter("nonlinear_convergence_threshold", 1.0e-5)
        self.declare_parameter("lidar_huber_delta", 2.5)
        self.declare_parameter("lm_max_trials", 6)
        self.declare_parameter("lm_damping_up", 10.0)
        self.declare_parameter("lm_damping_down", 0.3)
        self.declare_parameter("allow_lio_pose_fallback", False)
        self.declare_parameter("gnss_max_age_s", 2.0)
        self.declare_parameter("gnss_future_tolerance_s", 0.05)
        self.declare_parameter("flow_max_age_s", 1.0)
        self.declare_parameter("maximum_sensor_clock_skew_s", 5.0)
        self.declare_parameter("imu_buffer_s", 15.0)
        self.declare_parameter("imu_factor_wait_s", 0.080)
        self.declare_parameter("imu_nominal_gap_s", 0.10)
        self.declare_parameter("imu_max_gap_s", 0.30)
        self.declare_parameter("imu_startup_bias_initialization_enabled", True)
        self.declare_parameter("imu_startup_window_s", 1.5)
        self.declare_parameter("imu_startup_minimum_samples", 40)
        self.declare_parameter("imu_startup_minimum_span_s", 0.8)
        self.declare_parameter("imu_startup_maximum_mean_gyro_radps", 0.08)
        self.declare_parameter(
            "imu_startup_maximum_gyro_residual_rms_radps", 0.03)
        self.declare_parameter("imu_startup_gravity_tolerance_mps2", 0.60)
        self.declare_parameter(
            "imu_startup_maximum_accel_residual_rms_mps2", 0.40)
        self.declare_parameter("minimum_flow_quality", MIN_FLOW_QUALITY)
        self.declare_parameter("minimum_flow_distance_m", 0.08)
        self.declare_parameter("maximum_flow_distance_m", 12.0)
        self.declare_parameter("flow_base_displacement_sigma_m", 0.10)
        self.declare_parameter("flow_range_near_limit_m", 2.0)
        self.declare_parameter("flow_range_near_sigma_m", 0.04)
        self.declare_parameter("flow_range_far_relative_sigma", 0.02)
        self.declare_parameter("range_facet_enabled", False)
        self.declare_parameter("range_facet_minimum_support_points", 3)
        self.declare_parameter("range_facet_maximum_plane_rmse_m", 0.05)
        self.declare_parameter("range_facet_denominator_epsilon", 0.05)
        self.declare_parameter("range_facet_timestamp_tolerance_s", 0.08)
        self.declare_parameter("range_facet_facet_margin_m", 0.25)
        self.declare_parameter("range_facet_mahalanobis_gate", 9.0)
        self.declare_parameter("flow_maximum_speed_at_1m_mps", 7.0)
        self.declare_parameter("flow_speed_gate_margin", 1.10)
        self.declare_parameter("gnss_default_variance_m2", 4.0)
        self.declare_parameter("gnss_jump_gate_m", 20.0)
        self.declare_parameter("gnss_jump_speed_mps", 15.0)
        # 99% chi-square thresholds for independent horizontal and vertical
        # GNSS innovation blocks (2 and 1 degrees of freedom respectively).
        self.declare_parameter("gnss_xy_nis_gate", 9.210)
        self.declare_parameter("gnss_z_nis_gate", 6.635)
        self.declare_parameter("gnss_minimum_reliability_weight", 0.05)
        self.declare_parameter("gnss_minimum_axis_information_scale", 0.01)
        self.declare_parameter("gnss_z_reanchor_enabled", False)
        self.declare_parameter("gnss_z_reanchor_maximum_step_m", 0.15)
        self.declare_parameter("gnss_z_reanchor_minimum_consecutive", 2)
        self.declare_parameter("gnss_z_recovery_information_scale", 0.50)
        self.declare_parameter("axis_information_handoff_enabled", False)
        self.declare_parameter("axis_handoff_enable_x", False)
        self.declare_parameter("axis_handoff_enable_y", False)
        self.declare_parameter("axis_handoff_enable_z", True)
        self.declare_parameter("axis_handoff_enter_support", 0.35)
        self.declare_parameter("axis_handoff_exit_support", 0.45)
        self.declare_parameter(
            "axis_handoff_minimum_lidar_information_scale", 1.0e-6
        )
        self.declare_parameter(
            "axis_handoff_maximum_lidar_to_alternative_ratio", 1.0
        )
        self.declare_parameter("axis_handoff_gnss_rate_ratio", 0.25)
        self.declare_parameter("axis_handoff_rgbd_rate_ratio", 0.50)
        self.declare_parameter("axis_handoff_rgbd_freshness_s", 0.60)
        self.declare_parameter("axis_handoff_rgbd_minimum_support", 0.25)
        self.declare_parameter("axis_map_protection_enabled", False)
        self.declare_parameter(
            "axis_map_protection_gnss_disagreement_m", 0.20
        )
        self.declare_parameter("barometer_fallback_enabled", False)
        self.declare_parameter("barometer_baseline_window_s", 2.0)
        self.declare_parameter("barometer_minimum_baseline_samples", 8)
        self.declare_parameter("barometer_minimum_baseline_span_s", 0.8)
        self.declare_parameter("barometer_maximum_sample_age_s", 0.5)
        self.declare_parameter(
            "barometer_trusted_reference_maximum_age_s", 5.0
        )
        self.declare_parameter(
            "barometer_reference_maximum_gnss_residual_m", 0.20
        )
        self.declare_parameter(
            "barometer_reference_minimum_gnss_information_scale", 0.50
        )
        self.declare_parameter("barometer_scale_height_m", 8434.5)
        self.declare_parameter(
            "barometer_default_height_variance_m2", 0.25
        )
        self.declare_parameter("barometer_maximum_relative_height_m", 30.0)
        self.declare_parameter("barometer_prefit_nis_gate", 9.0)
        self.declare_parameter("barometer_minimum_information_scale", 0.05)
        self.declare_parameter(
            "barometer_activate_on_gnss_z_inconsistency", True
        )
        self.declare_parameter("optical_flow_yaw_coupling_enabled", True)
        self.declare_parameter("flow_rotation_lower_yaw_rate_radps", 0.08)
        self.declare_parameter("flow_rotation_upper_yaw_rate_radps", 0.30)
        self.declare_parameter("flow_rotation_recovery_dwell_s", 0.8)
        self.declare_parameter("flow_rotation_recovery_ramp_s", 1.5)
        self.declare_parameter("flow_rotation_minimum_translation_m", 0.01)
        self.declare_parameter("flow_rotation_recovery_max_base_score", 0.55)
        self.declare_parameter("flow_rotation_imu_max_gap_s", 0.12)
        self.declare_parameter("flow_rotation_allow_compensated", True)
        self.declare_parameter("flow_los_diagnostics_enabled", True)
        # ROS FLU body coordinates. Replace this simulation mount with the
        # measured optical-flow-to-IMU lever arm on the aircraft.
        self.declare_parameter("flow_sensor_offset_body_m", [0.0, 0.0, -0.35])
        self.declare_parameter("flow_lever_arm_compensation_enabled", True)
        self.declare_parameter("imu_factor_enabled", True)
        self.declare_parameter("preserve_lio_anchor", False)
        self.declare_parameter("lidar_anchor_minimum_effective_weight", 0.10)
        self.declare_parameter(
            "lidar_anchor_maximum_covariance_inflation", 5.0)
        self.declare_parameter("native_lidar_factor_enabled", True)
        self.declare_parameter("lidar_subspace_enabled", False)
        self.declare_parameter("lidar_subspace_weak_threshold", 0.15)
        self.declare_parameter("lidar_subspace_exit_threshold", 0.25)
        self.declare_parameter("lidar_subspace_weak_scale", 0.10)
        self.declare_parameter("input_trigger_mode", "native_factor")
        self.declare_parameter("live_propagation_enabled", True)
        self.declare_parameter("live_propagation_rate_hz", 10.0)
        self.declare_parameter(
            "unified_odom_output_mode", "fixed_rate_propagated")
        self.declare_parameter(
            "live_propagation_lidar_silence_timeout_s", 0.25)
        self.declare_parameter(
            "live_propagation_maximum_output_age_s", 0.20)
        self.declare_parameter("live_propagation_minimum_interval_s", 0.08)
        self.declare_parameter("live_propagation_maximum_imu_age_s", 0.20)
        self.declare_parameter("auxiliary_keyframe_enabled", True)
        self.declare_parameter(
            "auxiliary_keyframe_lidar_silence_timeout_s", 0.35)
        self.declare_parameter("auxiliary_keyframe_minimum_interval_s", 0.20)
        self.declare_parameter("auxiliary_keyframe_maximum_imu_age_s", 0.20)
        self.declare_parameter("native_lidar_factor_tolerance_s", 0.005)
        self.declare_parameter("native_lidar_factor_wait_s", 0.030)
        self.declare_parameter("native_lidar_minimum_matches", 50)
        self.declare_parameter("native_lidar_qos_depth", 1)
        self.declare_parameter("native_worker_queue_size", 1)
        self.declare_parameter("native_worker_latest_only_enabled", True)
        self.declare_parameter("imu_qos_depth", 64)
        self.declare_parameter("imu_covariance_scale", 50.0)
        self.declare_parameter("imu_bias_random_walk_variance", 1.0e-4)
        self.declare_parameter("imu_reintegration_accel_bias_threshold", 0.05)
        self.declare_parameter("imu_reintegration_gyro_bias_threshold", 0.005)
        self.declare_parameter("marginal_rank_tolerance", 1.0e-9)
        self.declare_parameter("marginal_covariance_update_period_s", 1.0)
        self.declare_parameter("performance_profiling_enabled", False)
        self.declare_parameter("performance_profiling_capacity", 4096)
        self.declare_parameter("performance_trace_path", "")
        self.declare_parameter("online_calibration_enabled", True)
        self.declare_parameter("calibration_estimate_rotation", False)
        self.declare_parameter(
            "calibration_motion_topic", "/calibration/lidar_relative_motion"
        )
        # Keep OSC shadow-only until a locked bundle is also injected into
        # front-end deskew/time association with Eq. (32) pose preservation.
        self.declare_parameter("calibration_apply_locked_values", False)
        self.declare_parameter("calibration_apply_locked_time_offset", False)
        self.declare_parameter("calibration_apply_locked_rotation", False)
        self.declare_parameter("calibration_window_s", 5.0)
        self.declare_parameter("calibration_minimum_pairs", 8)
        self.declare_parameter("calibration_time_offset_range_s", 0.10)
        self.declare_parameter("calibration_time_offset_step_s", 0.005)
        self.declare_parameter("calibration_minimum_correlation", 0.70)
        self.declare_parameter("calibration_minimum_correlation_margin", 0.002)
        self.declare_parameter(
            "calibration_minimum_time_peak_separation_s", 0.020)
        self.declare_parameter(
            "calibration_minimum_time_accumulated_rotation_rad", 0.25)
        self.declare_parameter(
            "calibration_minimum_excitation_eigenvalue", 1.0e-4)
        self.declare_parameter("calibration_minimum_excitation_ratio", 0.05)
        self.declare_parameter(
            "calibration_minimum_accumulated_rotation_rad", 0.25)
        self.declare_parameter(
            "calibration_minimum_rotation_inlier_ratio", 0.70)
        self.declare_parameter(
            "calibration_maximum_rotation_residual_rad", 0.08)
        self.declare_parameter("calibration_sharp_turn_rate_radps", 1.5)
        self.declare_parameter("calibration_solve_period_s", 1.0)
        self.declare_parameter(
            "calibration_minimum_lock_candidate_separation_s", 1.0)
        self.declare_parameter("calibration_time_unlock_count", 3)
        self.declare_parameter("scheduler_timeout_s", 1.0)
        self.declare_parameter("reliability_mode", "dynamic")
        self.declare_parameter("fixed_lidar_weight", 1.0)
        self.declare_parameter("fixed_gnss_weight", 1.0)
        self.declare_parameter("fixed_imu_weight", 1.0)
        self.declare_parameter("fixed_optical_flow_weight", 1.0)
        self.declare_parameter("fixed_vision_weight", 1.0)
        self.declare_parameter("fixed_covariance_inflation", 1.0)
        self.declare_parameter("visual_factor_mode", "disabled")
        self.declare_parameter("visual_tracks_topic", "/vision/feature_tracks")
        self.declare_parameter(
            "rgbd_geometry_tracks_topic", "/vision/rgbd_geometry_tracks"
        )
        self.declare_parameter(
            "rgbd_direct_tracks_topic", "/vision/rgbd_direct_tracks"
        )
        self.declare_parameter("rgbd_direct_factor_minimum_tracks", 12)
        self.declare_parameter("rgbd_direct_factor_maximum_tracks", 32)
        self.declare_parameter("rgbd_direct_factor_maximum_depth_rmse_m", 0.20)
        self.declare_parameter(
            "rgbd_direct_factor_maximum_photometric_rmse", 0.60
        )
        self.declare_parameter("rgbd_direct_depth_information_scale", 0.25)
        self.declare_parameter(
            "rgbd_direct_photometric_information_scale", 0.10
        )
        self.declare_parameter("rgbd_depth_factor_enabled", False)
        self.declare_parameter("rgbd_depth_factor_tolerance_s", 0.010)
        self.declare_parameter("rgbd_depth_factor_minimum_tracks", 12)
        self.declare_parameter("rgbd_depth_factor_maximum_tracks", 32)
        self.declare_parameter("rgbd_depth_factor_maximum_rmse_m", 0.20)
        self.declare_parameter("rgbd_depth_factor_information_scale", 0.25)
        self.declare_parameter(
            "rgbd_depth_healthy_lidar_profile_information", 8000.0
        )
        self.declare_parameter("rgbd_depth_healthy_lidar_stride", 4)
        self.declare_parameter(
            "visual_factor_score_topic", "/reliability/vision_factor_score"
        )
        self.declare_parameter("visual_factor_score_tolerance_s", 0.010)
        self.declare_parameter("visual_factor_score_max_wait_s", 0.25)
        self.declare_parameter("visual_factor_score_max_wall_wait_s", 0.25)
        self.declare_parameter("visual_factor_score_history_size", 128)
        self.declare_parameter("visual_time_offset_s", 0.0)
        self.declare_parameter("visual_time_calibration_enabled", True)
        self.declare_parameter("visual_time_calibration_apply_locked", True)
        self.declare_parameter("visual_time_calibration_window_s", 12.0)
        self.declare_parameter("visual_time_calibration_minimum_pairs", 8)
        self.declare_parameter("visual_time_calibration_range_s", 0.120)
        self.declare_parameter("visual_time_calibration_step_s", 0.002)
        self.declare_parameter(
            "visual_time_calibration_minimum_correlation", 0.65
        )
        self.declare_parameter(
            "visual_time_calibration_minimum_margin", 0.002
        )
        self.declare_parameter(
            "visual_time_calibration_peak_exclusion_s", 0.010
        )
        self.declare_parameter(
            "visual_time_calibration_reject_boundary_candidates", True
        )
        self.declare_parameter("visual_time_calibration_lock_count", 3)
        self.declare_parameter(
            "visual_time_calibration_stability_s", 0.006
        )
        self.declare_parameter(
            "visual_time_calibration_minimum_accumulated_rotation_rad", 0.10
        )
        self.declare_parameter(
            "visual_time_calibration_minimum_interval_rotation_rad", 0.001
        )
        self.declare_parameter(
            "visual_time_calibration_minimum_lock_candidate_separation_s", 1.0
        )
        self.declare_parameter("visual_initialization_enabled", True)
        self.declare_parameter("visual_initialization_minimum_batches", 3)
        self.declare_parameter(
            "visual_initialization_require_time_lock", True
        )
        self.declare_parameter("visual_state_tolerance_s", 0.065)
        self.declare_parameter("visual_pending_enabled", True)
        self.declare_parameter("visual_pending_latest_horizon_s", 0.25)
        self.declare_parameter("visual_pending_max_wait_s", 0.25)
        self.declare_parameter("visual_pending_max_wall_wait_s", 0.35)
        self.declare_parameter("visual_pending_max_queue", 3)
        self.declare_parameter(
            "visual_timing_diagnostic_topic", "/fusion/unified/visual_timing"
        )
        self.declare_parameter("visual_minimum_tracks", 20)
        self.declare_parameter("visual_pnp_minimum_inlier_ratio", 0.50)
        self.declare_parameter("visual_pnp_minimum_information_rank", 6)
        self.declare_parameter(
            "visual_pnp_maximum_condition_number", 500.0
        )
        self.declare_parameter(
            "visual_pnp_maximum_mean_reprojection_error_px", 2.0
        )
        self.declare_parameter("visual_information_reference_tracks", 20)
        self.declare_parameter("visual_pixel_sigma_normalized", 0.002)
        self.declare_parameter("visual_inverse_depth_variance_scale", 0.01)
        self.declare_parameter("visual_state_consistency_enabled", True)
        self.declare_parameter(
            "visual_state_innovation_maximum_rmse_px", 6.0
        )
        self.declare_parameter("visual_minimum_jacobian_rank", 6)
        self.declare_parameter(
            "visual_maximum_jacobian_condition_number", 5.0e4
        )
        self.declare_parameter(
            "visual_minimum_projectable_track_ratio", 0.80
        )
        self.declare_parameter(
            "visual_rotation_body_camera", [
                1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        self.declare_parameter(
            "visual_translation_body_camera_m", [
                0.0, 0.0, 0.0])
        self.declare_parameter("publish_path_length", 2000)
        self.declare_parameter("path_publish_period_s", 0.5)
        self.declare_parameter("path_minimum_translation_m", 0.05)
        self.declare_parameter("path_minimum_rotation_rad", 0.02)
        self.declare_parameter("relocalization_enabled", True)
        self.declare_parameter("transactional_update_enabled", True)
        self.declare_parameter(
            "marginal_prior_suppress_historical_lidar_weak", False
        )
        self.declare_parameter("lidar_prediction_gate_enabled", True)
        self.declare_parameter("lidar_prediction_gate_max_position_m", 1.0)
        self.declare_parameter("lidar_prediction_gate_max_yaw_rad", 0.50)
        self.declare_parameter(
            "lidar_prediction_gate_recovery_after_rejections", 3)
        self.declare_parameter("lidar_prediction_recovery_weight", 0.20)
        self.declare_parameter("lidar_prediction_recovery_inflation", 5.0)
        self.declare_parameter(
            "optimization_max_translation_correction_m", 1.0)
        self.declare_parameter(
            "optimization_max_rotation_correction_rad", 0.50)
        self.declare_parameter("optimization_max_velocity_correction_mps", 5.0)
        self.declare_parameter(
            "optimization_max_accel_bias_correction_mps2", 1.5)
        self.declare_parameter(
            "optimization_max_gyro_bias_correction_radps", 0.30)
        self.declare_parameter(
            "optimization_max_information_condition", 1.0e12)
        self.declare_parameter("frontend_state_seed_enabled", False)
        self.declare_parameter(
            "frontend_state_seed_topic", "/fusion/unified/frontend_state_seed"
        )
        self.declare_parameter(
            "frontend_map_pose_topic", "/fusion/unified/map_pose"
        )
        self.declare_parameter(
            "frontend_activation_pose_topic",
            "/fusion/unified/frontend_activation_odom",
        )
        self.declare_parameter(
            "frontend_map_commit_allowed_health_states",
            ["NORMAL", "DEGRADED", "RECOVERED"],
        )
        self.declare_parameter("frontend_map_max_position_variance_m2", 4.0)
        self.declare_parameter(
            "frontend_map_max_orientation_variance_rad2", 0.25)
        self.declare_parameter("frontend_map_commit_delay_states", 7)
        self.declare_parameter("frontend_scan_prediction_enabled", True)
        self.declare_parameter(
            "frontend_scan_request_topic", "/fast_lio/frontend_scan_request"
        )
        self.declare_parameter(
            "backend_deskew_trajectory_topic",
            "/fusion/unified/backend_deskew_trajectory",
        )
        self.declare_parameter("scan_prediction_maximum_begin_gap_s", 0.02)
        self.declare_parameter("scan_prediction_timestamp_tolerance_s", 0.002)
        self.declare_parameter("scan_prediction_state_tolerance", 1.0e-8)
        self.declare_parameter("scan_prediction_cache_size", 8)
        self.declare_parameter("scan_prediction_missing_factor_grace_s", 0.5)
        self.declare_parameter("scan_prediction_contract_failure_threshold", 3)
        self.declare_parameter("scan_prediction_contract_request_timeout_s", 1.0)

        self.performance_profiling_enabled = bool(
            self.get_parameter("performance_profiling_enabled").value
        )
        self.performance_profiling_capacity = max(
            64,
            int(self.get_parameter("performance_profiling_capacity").value),
        )
        self.performance_trace_path = str(
            self.get_parameter("performance_trace_path").value
        )

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.body_frame = str(self.get_parameter("body_frame").value)
        self.frontend_map_pose_topic = str(
            self.get_parameter("frontend_map_pose_topic").value
        )
        self.frontend_activation_pose_topic = str(
            self.get_parameter("frontend_activation_pose_topic").value
        )
        self.frontend_map_commit_allowed_health_states = tuple(
            str(value).upper() for value in self.get_parameter(
                "frontend_map_commit_allowed_health_states"
            ).value
        )
        self.frontend_map_max_position_variance_m2 = float(
            self.get_parameter("frontend_map_max_position_variance_m2").value
        )
        self.frontend_map_max_orientation_variance_rad2 = float(
            self.get_parameter("frontend_map_max_orientation_variance_rad2").value)
        self.frontend_map_commit_delay_states = int(
            self.get_parameter("frontend_map_commit_delay_states").value
        )
        if (
            not self.frontend_map_pose_topic
            or not self.frontend_activation_pose_topic
            or not self.frontend_map_commit_allowed_health_states
            or not math.isfinite(self.frontend_map_max_position_variance_m2)
            or self.frontend_map_max_position_variance_m2 <= 0.0
            or not math.isfinite(self.frontend_map_max_orientation_variance_rad2)
            or self.frontend_map_max_orientation_variance_rad2 <= 0.0
            or self.frontend_map_commit_delay_states < 0
        ):
            raise ValueError(
                "front-end map commit gate parameters are invalid")
        self.gnss_max_age_s = float(self.get_parameter("gnss_max_age_s").value)
        self.gnss_future_tolerance_s = float(
            self.get_parameter("gnss_future_tolerance_s").value
        )
        self.maximum_sensor_clock_skew_s = max(
            0.0,
            float(self.get_parameter("maximum_sensor_clock_skew_s").value),
        )
        if self.gnss_max_age_s <= 0.0 or self.gnss_future_tolerance_s < 0.0:
            raise ValueError("GNSS timing limits are invalid")
        self.flow_max_age_s = float(self.get_parameter("flow_max_age_s").value)
        self.imu_buffer_s = float(self.get_parameter("imu_buffer_s").value)
        self.imu_factor_wait_s = float(
            self.get_parameter("imu_factor_wait_s").value
        )
        self.imu_nominal_gap_s = float(
            self.get_parameter("imu_nominal_gap_s").value
        )
        self.imu_max_gap_s = float(self.get_parameter("imu_max_gap_s").value)
        if (
            self.imu_factor_wait_s < 0.0
            or self.imu_nominal_gap_s <= 0.0
            or self.imu_max_gap_s < self.imu_nominal_gap_s
        ):
            raise ValueError("IMU timing limits are invalid")
        self.imu_startup_bias_initialization_enabled = bool(
            self.get_parameter("imu_startup_bias_initialization_enabled").value
        )
        self.imu_startup_window_s = float(
            self.get_parameter("imu_startup_window_s").value
        )
        self.imu_startup_minimum_samples = int(
            self.get_parameter("imu_startup_minimum_samples").value
        )
        self.imu_startup_minimum_span_s = float(
            self.get_parameter("imu_startup_minimum_span_s").value
        )
        self.imu_startup_maximum_mean_gyro_radps = float(
            self.get_parameter("imu_startup_maximum_mean_gyro_radps").value
        )
        self.imu_startup_maximum_gyro_residual_rms_radps = float(
            self.get_parameter(
                "imu_startup_maximum_gyro_residual_rms_radps"
            ).value
        )
        self.imu_startup_gravity_tolerance_mps2 = float(
            self.get_parameter("imu_startup_gravity_tolerance_mps2").value
        )
        self.imu_startup_maximum_accel_residual_rms_mps2 = float(
            self.get_parameter(
                "imu_startup_maximum_accel_residual_rms_mps2"
            ).value
        )
        if (
            self.imu_startup_window_s <= 0.0
            or self.imu_startup_minimum_samples < 2
            or self.imu_startup_minimum_span_s <= 0.0
            or self.imu_startup_minimum_span_s > self.imu_startup_window_s
            or self.imu_startup_maximum_mean_gyro_radps <= 0.0
            or self.imu_startup_maximum_gyro_residual_rms_radps <= 0.0
            or self.imu_startup_gravity_tolerance_mps2 <= 0.0
            or self.imu_startup_maximum_accel_residual_rms_mps2 <= 0.0
        ):
            raise ValueError(
                "stationary IMU initialization limits are invalid")
        self.minimum_flow_quality = int(
            self.get_parameter("minimum_flow_quality").value)
        self.minimum_flow_distance_m = float(
            self.get_parameter("minimum_flow_distance_m").value)
        self.maximum_flow_distance_m = float(
            self.get_parameter("maximum_flow_distance_m").value)
        self.flow_base_displacement_sigma_m = float(
            self.get_parameter("flow_base_displacement_sigma_m").value)
        self.flow_range_near_limit_m = float(
            self.get_parameter("flow_range_near_limit_m").value)
        self.flow_range_near_sigma_m = float(
            self.get_parameter("flow_range_near_sigma_m").value)
        self.flow_range_far_relative_sigma = float(
            self.get_parameter("flow_range_far_relative_sigma").value)
        self.range_facet_enabled = bool(
            self.get_parameter("range_facet_enabled").value)
        self.range_facet_minimum_support_points = int(
            self.get_parameter("range_facet_minimum_support_points").value)
        self.range_facet_maximum_plane_rmse_m = float(
            self.get_parameter("range_facet_maximum_plane_rmse_m").value)
        self.range_facet_denominator_epsilon = float(
            self.get_parameter("range_facet_denominator_epsilon").value)
        self.range_facet_timestamp_tolerance_s = float(
            self.get_parameter("range_facet_timestamp_tolerance_s").value)
        self.range_facet_facet_margin_m = float(
            self.get_parameter("range_facet_facet_margin_m").value)
        self.range_facet_mahalanobis_gate = float(
            self.get_parameter("range_facet_mahalanobis_gate").value)
        self.flow_maximum_speed_at_1m_mps = float(
            self.get_parameter("flow_maximum_speed_at_1m_mps").value)
        self.flow_speed_gate_margin = float(
            self.get_parameter("flow_speed_gate_margin").value)
        if (
            self.minimum_flow_distance_m < 0.08
            or self.maximum_flow_distance_m > 12.0
            or self.minimum_flow_distance_m >= self.maximum_flow_distance_m
            or self.flow_base_displacement_sigma_m <= 0.0
            or self.flow_range_near_limit_m <= 0.0
            or self.flow_range_near_sigma_m <= 0.0
            or self.flow_range_far_relative_sigma <= 0.0
            or self.flow_maximum_speed_at_1m_mps <= 0.0
            or self.flow_speed_gate_margin < 1.0
            or self.range_facet_minimum_support_points < 3
            or self.range_facet_maximum_plane_rmse_m <= 0.0
            or self.range_facet_denominator_epsilon <= 0.0
            or self.range_facet_timestamp_tolerance_s < 0.0
            or self.range_facet_facet_margin_m < 0.0
            or self.range_facet_mahalanobis_gate <= 0.0
        ):
            raise ValueError("MTF-01P flow measurement limits are invalid")
        self.gnss_default_variance = float(
            self.get_parameter("gnss_default_variance_m2").value)
        self.gnss_jump_gate_m = float(
            self.get_parameter("gnss_jump_gate_m").value)
        self.gnss_jump_speed_mps = float(
            self.get_parameter("gnss_jump_speed_mps").value)
        self.gnss_xy_nis_gate = float(
            self.get_parameter("gnss_xy_nis_gate").value)
        self.gnss_z_nis_gate = float(
            self.get_parameter("gnss_z_nis_gate").value)
        self.gnss_minimum_reliability_weight = float(
            self.get_parameter("gnss_minimum_reliability_weight").value)
        self.gnss_minimum_axis_information_scale = float(
            self.get_parameter(
                "gnss_minimum_axis_information_scale"
            ).value
        )
        self.gnss_z_reanchor_enabled = bool(
            self.get_parameter("gnss_z_reanchor_enabled").value
        )
        self.gnss_z_reanchor_maximum_step_m = float(
            self.get_parameter("gnss_z_reanchor_maximum_step_m").value
        )
        self.gnss_z_reanchor_minimum_consecutive = int(
            self.get_parameter(
                "gnss_z_reanchor_minimum_consecutive"
            ).value
        )
        self.gnss_z_recovery_information_scale = float(
            self.get_parameter("gnss_z_recovery_information_scale").value
        )
        self.axis_information_handoff_enabled = bool(
            self.get_parameter("axis_information_handoff_enabled").value
        )
        self.axis_handoff_enabled_axes = np.asarray([
            bool(self.get_parameter("axis_handoff_enable_x").value),
            bool(self.get_parameter("axis_handoff_enable_y").value),
            bool(self.get_parameter("axis_handoff_enable_z").value),
        ], dtype=bool)
        self.axis_handoff_enter_support = float(
            self.get_parameter("axis_handoff_enter_support").value
        )
        self.axis_handoff_exit_support = float(
            self.get_parameter("axis_handoff_exit_support").value
        )
        self.axis_handoff_minimum_lidar_information_scale = float(
            self.get_parameter(
                "axis_handoff_minimum_lidar_information_scale"
            ).value
        )
        self.axis_handoff_maximum_lidar_to_alternative_ratio = float(
            self.get_parameter(
                "axis_handoff_maximum_lidar_to_alternative_ratio"
            ).value
        )
        self.axis_handoff_gnss_rate_ratio = float(
            self.get_parameter("axis_handoff_gnss_rate_ratio").value
        )
        self.axis_handoff_rgbd_rate_ratio = float(
            self.get_parameter("axis_handoff_rgbd_rate_ratio").value
        )
        self.axis_handoff_rgbd_freshness_s = float(
            self.get_parameter("axis_handoff_rgbd_freshness_s").value
        )
        self.axis_handoff_rgbd_minimum_support = float(
            self.get_parameter("axis_handoff_rgbd_minimum_support").value
        )
        self.axis_map_protection_enabled = bool(
            self.get_parameter("axis_map_protection_enabled").value
        )
        self.axis_map_protection_gnss_disagreement_m = float(
            self.get_parameter(
                "axis_map_protection_gnss_disagreement_m"
            ).value
        )
        self.barometer_fallback_enabled = bool(
            self.get_parameter("barometer_fallback_enabled").value
        )
        self.barometer_prefit_nis_gate = float(
            self.get_parameter("barometer_prefit_nis_gate").value
        )
        self.barometer_minimum_information_scale = float(
            self.get_parameter("barometer_minimum_information_scale").value
        )
        self.barometer_reference_maximum_gnss_residual_m = float(
            self.get_parameter(
                "barometer_reference_maximum_gnss_residual_m"
            ).value
        )
        self.barometer_reference_minimum_gnss_information_scale = float(
            self.get_parameter(
                "barometer_reference_minimum_gnss_information_scale"
            ).value
        )
        self.barometer_activate_on_gnss_z_inconsistency = bool(
            self.get_parameter(
                "barometer_activate_on_gnss_z_inconsistency"
            ).value
        )
        if (
            not math.isfinite(self.gnss_default_variance)
            or self.gnss_default_variance <= 0.0
            or not math.isfinite(self.gnss_xy_nis_gate)
            or self.gnss_xy_nis_gate <= 0.0
            or not math.isfinite(self.gnss_z_nis_gate)
            or self.gnss_z_nis_gate <= 0.0
            or not 0.0 < self.gnss_minimum_reliability_weight <= 1.0
            or not 0.0 < self.gnss_minimum_axis_information_scale <= 1.0
            or self.gnss_z_reanchor_maximum_step_m <= 0.0
            or self.gnss_z_reanchor_minimum_consecutive < 1
            or not 0.0 < self.gnss_z_recovery_information_scale <= 1.0
            or self.gnss_z_recovery_information_scale
            < self.gnss_minimum_axis_information_scale
            or not 0.0 <= self.axis_handoff_enter_support
            < self.axis_handoff_exit_support <= 1.0
            or not 0.0
            < self.axis_handoff_minimum_lidar_information_scale <= 1.0
            or self.axis_handoff_maximum_lidar_to_alternative_ratio <= 0.0
            or self.axis_handoff_gnss_rate_ratio <= 0.0
            or self.axis_handoff_rgbd_rate_ratio <= 0.0
            or self.axis_handoff_rgbd_freshness_s <= 0.0
            or not 0.0 <= self.axis_handoff_rgbd_minimum_support <= 1.0
            or not math.isfinite(
                self.axis_map_protection_gnss_disagreement_m
            )
            or self.axis_map_protection_gnss_disagreement_m <= 0.0
            or self.barometer_prefit_nis_gate <= 0.0
            or not 0.0 < self.barometer_minimum_information_scale <= 1.0
            or self.barometer_reference_maximum_gnss_residual_m <= 0.0
            or not 0.0
            < self.barometer_reference_minimum_gnss_information_scale <= 1.0
        ):
            raise ValueError("GNSS and axis handoff limits are invalid")
        self.barometer_segment = LocalBarometerSegment(
            baseline_window_s=float(self.get_parameter(
                "barometer_baseline_window_s").value),
            minimum_baseline_samples=int(self.get_parameter(
                "barometer_minimum_baseline_samples").value),
            minimum_baseline_span_s=float(self.get_parameter(
                "barometer_minimum_baseline_span_s").value),
            maximum_sample_age_s=float(self.get_parameter(
                "barometer_maximum_sample_age_s").value),
            maximum_trusted_reference_age_s=float(self.get_parameter(
                "barometer_trusted_reference_maximum_age_s").value),
            require_trusted_reference=True,
            scale_height_m=float(self.get_parameter(
                "barometer_scale_height_m").value),
            default_height_variance_m2=float(self.get_parameter(
                "barometer_default_height_variance_m2").value),
            maximum_relative_height_m=float(self.get_parameter(
                "barometer_maximum_relative_height_m").value),
        )
        self.optical_flow_yaw_coupling_enabled = bool(
            self.get_parameter("optical_flow_yaw_coupling_enabled").value)
        self.flow_rotation_allow_compensated = bool(
            self.get_parameter("flow_rotation_allow_compensated").value)
        self.flow_rotation_recovery_max_base_score = float(
            self.get_parameter("flow_rotation_recovery_max_base_score").value)
        self.flow_rotation_imu_max_gap_s = float(
            self.get_parameter("flow_rotation_imu_max_gap_s").value)
        self.flow_los_diagnostics_enabled = bool(
            self.get_parameter("flow_los_diagnostics_enabled").value)
        self.flow_lever_arm_compensation_enabled = bool(
            self.get_parameter("flow_lever_arm_compensation_enabled").value)
        flow_sensor_offset = tuple(
            float(value) for value in self.get_parameter(
                "flow_sensor_offset_body_m").value
        )
        if len(flow_sensor_offset) != 3 or not all(
            math.isfinite(value) for value in flow_sensor_offset
        ):
            raise ValueError(
                "flow_sensor_offset_body_m must be a finite 3-vector")
        self.flow_sensor_offset_body_m = np.asarray(
            flow_sensor_offset, dtype=float
        )
        self.imu_factor_enabled = bool(
            self.get_parameter("imu_factor_enabled").value)
        self.preserve_lio_anchor = bool(
            self.get_parameter("preserve_lio_anchor").value)
        self.lidar_anchor_minimum_effective_weight = float(
            self.get_parameter("lidar_anchor_minimum_effective_weight").value
        )
        self.lidar_anchor_maximum_covariance_inflation = float(
            self.get_parameter("lidar_anchor_maximum_covariance_inflation").value)
        if not 0.0 < self.lidar_anchor_minimum_effective_weight <= 1.0:
            raise ValueError(
                "lidar_anchor_minimum_effective_weight must be in (0, 1]"
            )
        if self.lidar_anchor_maximum_covariance_inflation < 1.0:
            raise ValueError(
                "lidar_anchor_maximum_covariance_inflation must be at least one"
            )
        native_requested = bool(
            self.get_parameter("native_lidar_factor_enabled").value
        )
        self.lidar_subspace_enabled = bool(
            self.get_parameter("lidar_subspace_enabled").value
        )
        self.lidar_subspace_weak_threshold = float(
            self.get_parameter("lidar_subspace_weak_threshold").value
        )
        self.lidar_subspace_exit_threshold = float(
            self.get_parameter("lidar_subspace_exit_threshold").value
        )
        self.lidar_subspace_weak_scale = float(
            self.get_parameter("lidar_subspace_weak_scale").value
        )
        if not (
            0.0 < self.lidar_subspace_weak_threshold
            < self.lidar_subspace_exit_threshold <= 1.0
            and 0.0 < self.lidar_subspace_weak_scale <= 1.0
        ):
            raise ValueError("LiDAR subspace limits are invalid")
        self.native_lidar_enabled = bool(
            native_requested and NativeLidarFactor is not None)
        self.native_worker_latest_only_enabled = bool(
            self.get_parameter("native_worker_latest_only_enabled").value
        )
        requested_trigger_mode = str(
            self.get_parameter("input_trigger_mode").value
        ).lower()
        if requested_trigger_mode not in {"native_factor", "lio_pair"}:
            raise ValueError(
                "input_trigger_mode must be native_factor or lio_pair")
        self.input_trigger_mode = requested_trigger_mode
        if self.input_trigger_mode == "native_factor" and not self.native_lidar_enabled:
            self.input_trigger_mode = "lio_pair"
        self.live_propagation_enabled = bool(
            self.get_parameter("live_propagation_enabled").value
        )
        self.unified_odom_output_mode = str(
            self.get_parameter("unified_odom_output_mode").value
        ).lower()
        if self.unified_odom_output_mode not in {
            "fixed_rate_propagated", "lidar_event_propagated", "legacy_hybrid"
        }:
            raise ValueError(
                "unified_odom_output_mode must be fixed_rate_propagated, "
                "lidar_event_propagated, or legacy_hybrid"
            )
        if (
            self.unified_odom_output_mode in {
                "fixed_rate_propagated", "lidar_event_propagated"
            }
            and not self.live_propagation_enabled
        ):
            raise ValueError(
                "fixed_rate_propagated requires live_propagation_enabled"
            )
        self.live_propagation_rate_hz = float(
            self.get_parameter("live_propagation_rate_hz").value
        )
        self.live_propagation_lidar_silence_timeout_s = float(
            self.get_parameter("live_propagation_lidar_silence_timeout_s").value)
        self.live_propagation_maximum_output_age_s = float(
            self.get_parameter("live_propagation_maximum_output_age_s").value)
        self.live_propagation_minimum_interval_s = float(
            self.get_parameter("live_propagation_minimum_interval_s").value
        )
        self.live_propagation_maximum_imu_age_s = float(
            self.get_parameter("live_propagation_maximum_imu_age_s").value
        )
        self.auxiliary_keyframe_enabled = bool(
            self.get_parameter("auxiliary_keyframe_enabled").value
        )
        self.auxiliary_keyframe_lidar_silence_timeout_s = float(
            self.get_parameter(
                "auxiliary_keyframe_lidar_silence_timeout_s"
            ).value
        )
        self.auxiliary_keyframe_minimum_interval_s = float(
            self.get_parameter("auxiliary_keyframe_minimum_interval_s").value
        )
        self.auxiliary_keyframe_maximum_imu_age_s = float(
            self.get_parameter("auxiliary_keyframe_maximum_imu_age_s").value
        )
        if (
            self.live_propagation_rate_hz <= 0.0
            or self.live_propagation_lidar_silence_timeout_s < 0.0
            or self.live_propagation_maximum_output_age_s <= 0.0
            or self.live_propagation_minimum_interval_s <= 0.0
            or self.live_propagation_maximum_imu_age_s <= 0.0
            or self.auxiliary_keyframe_lidar_silence_timeout_s < 0.0
            or self.auxiliary_keyframe_minimum_interval_s <= 0.0
            or self.auxiliary_keyframe_maximum_imu_age_s <= 0.0
        ):
            raise ValueError(
                "live propagation or auxiliary keyframe timing parameters are invalid"
            )
        self.native_lidar_tolerance_s = float(
            self.get_parameter("native_lidar_factor_tolerance_s").value
        )
        self.native_lidar_wait_s = float(
            self.get_parameter("native_lidar_factor_wait_s").value
        )
        self.native_lidar_minimum_matches = int(
            self.get_parameter("native_lidar_minimum_matches").value
        )
        if self.native_lidar_tolerance_s < 0.0 or self.native_lidar_wait_s < 0.0:
            raise ValueError("native LiDAR timing limits must be non-negative")
        if self.native_lidar_minimum_matches < 1:
            raise ValueError("native LiDAR minimum matches must be positive")
        self.imu_covariance_scale = float(
            self.get_parameter("imu_covariance_scale").value)
        self.imu_bias_random_walk_variance = float(
            self.get_parameter("imu_bias_random_walk_variance").value)
        self.imu_reintegration_accel_bias_threshold = float(
            self.get_parameter("imu_reintegration_accel_bias_threshold").value)
        self.imu_reintegration_gyro_bias_threshold = float(
            self.get_parameter("imu_reintegration_gyro_bias_threshold").value)
        self.marginal_rank_tolerance = float(
            self.get_parameter("marginal_rank_tolerance").value)
        self.marginal_covariance_update_period_s = float(
            self.get_parameter("marginal_covariance_update_period_s").value
        )
        if (
            not math.isfinite(self.imu_reintegration_accel_bias_threshold)
            or self.imu_reintegration_accel_bias_threshold <= 0.0
            or not math.isfinite(self.imu_reintegration_gyro_bias_threshold)
            or self.imu_reintegration_gyro_bias_threshold <= 0.0
            or not math.isfinite(self.marginal_rank_tolerance)
            or self.marginal_rank_tolerance <= 0.0
            or not math.isfinite(self.marginal_covariance_update_period_s)
            or self.marginal_covariance_update_period_s <= 0.0
        ):
            raise ValueError(
                "IMU reintegration and marginalization limits are invalid")
        self.online_calibration_enabled = bool(
            self.get_parameter("online_calibration_enabled").value
        )
        self.calibration_estimate_rotation = bool(
            self.get_parameter("calibration_estimate_rotation").value
        )
        self.calibration_apply_locked_values = bool(
            self.get_parameter("calibration_apply_locked_values").value
        )
        self.calibration_apply_locked_time_offset = bool(
            self.calibration_apply_locked_values
            or self.get_parameter(
                "calibration_apply_locked_time_offset"
            ).value
        )
        self.calibration_apply_locked_rotation = bool(
            self.calibration_apply_locked_values
            or self.get_parameter("calibration_apply_locked_rotation").value
        )
        if (
            self.calibration_apply_locked_rotation
            and not self.calibration_estimate_rotation
        ):
            raise ValueError(
                "fixed online extrinsics cannot apply a rotation estimate")
        if self.calibration_apply_locked_time_offset:
            self.calibration_mode = "time_apply"
        elif self.calibration_apply_locked_rotation:
            self.calibration_mode = "rotation_apply"
        elif (
            self.online_calibration_enabled
            and not self.calibration_estimate_rotation
        ):
            self.calibration_mode = "time_shadow_fixed_extrinsic"
        else:
            self.calibration_mode = "shadow"
        calibration_kwargs = {
            "window_s": float(self.get_parameter("calibration_window_s").value),
            "minimum_pairs": int(self.get_parameter("calibration_minimum_pairs").value),
            "time_offset_range_s": float(
                self.get_parameter("calibration_time_offset_range_s").value
            ),
            "time_offset_step_s": float(
                self.get_parameter("calibration_time_offset_step_s").value
            ),
            "minimum_correlation": float(
                self.get_parameter("calibration_minimum_correlation").value
            ),
            "minimum_correlation_margin": float(
                self.get_parameter("calibration_minimum_correlation_margin").value
            ),
            "minimum_time_peak_separation_s": float(
                self.get_parameter(
                    "calibration_minimum_time_peak_separation_s"
                ).value
            ),
            "minimum_time_accumulated_rotation_rad": float(
                self.get_parameter(
                    "calibration_minimum_time_accumulated_rotation_rad"
                ).value
            ),
            "minimum_excitation_eigenvalue": float(
                self.get_parameter("calibration_minimum_excitation_eigenvalue").value
            ),
            "minimum_excitation_ratio": float(
                self.get_parameter("calibration_minimum_excitation_ratio").value
            ),
            "minimum_accumulated_rotation_rad": float(
                self.get_parameter(
                    "calibration_minimum_accumulated_rotation_rad"
                ).value
            ),
            "minimum_rotation_inlier_ratio": float(
                self.get_parameter(
                    "calibration_minimum_rotation_inlier_ratio"
                ).value
            ),
            "maximum_rotation_residual_rad": float(
                self.get_parameter("calibration_maximum_rotation_residual_rad").value
            ),
            "sharp_turn_rate_radps": float(
                self.get_parameter("calibration_sharp_turn_rate_radps").value
            ),
            "solve_period_s": float(
                self.get_parameter("calibration_solve_period_s").value
            ),
            "minimum_time_lock_candidate_separation_s": float(
                self.get_parameter(
                    "calibration_minimum_lock_candidate_separation_s"
                ).value
            ),
            "time_unlock_count": int(
                self.get_parameter("calibration_time_unlock_count").value
            ),
            "estimate_rotation": self.calibration_estimate_rotation,
        }
        self.calibrator = OnlineSpatiotemporalCalibrator(**calibration_kwargs)
        self.last_calibration_update = self.calibrator.last_update
        self.calibration_lock = threading.Lock()
        self.last_calibration_motion_reason = "independent_lidar_motion_missing"
        self.scheduler_timeout_s = float(
            self.get_parameter("scheduler_timeout_s").value)
        self.reliability_mode = str(
            self.get_parameter("reliability_mode").value).lower()
        if self.reliability_mode not in {"dynamic", "fixed"}:
            raise ValueError("reliability_mode must be dynamic or fixed")
        self.fixed_weights = {
            modality: float(self.get_parameter(f"fixed_{modality}_weight").value)
            for modality in ("lidar", "gnss", "imu", "optical_flow", "vision")
        }
        if any(
            not math.isfinite(weight) or not 0.0 <= weight <= 1.0
            for weight in self.fixed_weights.values()
        ):
            raise ValueError("fixed modality weights must be finite in [0, 1]")
        self.visual_factor_mode = str(
            self.get_parameter("visual_factor_mode").value
        ).lower()
        if self.visual_factor_mode not in {
            "disabled", "paper_reprojection", "rgbd_direct"
        }:
            raise ValueError(
                "visual_factor_mode must be disabled, paper_reprojection, "
                "or rgbd_direct")
        self.rgbd_direct_factor_minimum_tracks = int(
            self.get_parameter("rgbd_direct_factor_minimum_tracks").value
        )
        self.rgbd_direct_factor_maximum_tracks = int(
            self.get_parameter("rgbd_direct_factor_maximum_tracks").value
        )
        self.rgbd_direct_factor_maximum_depth_rmse_m = float(
            self.get_parameter(
                "rgbd_direct_factor_maximum_depth_rmse_m"
            ).value
        )
        self.rgbd_direct_factor_maximum_photometric_rmse = float(
            self.get_parameter(
                "rgbd_direct_factor_maximum_photometric_rmse"
            ).value
        )
        self.rgbd_direct_depth_information_scale = float(
            self.get_parameter("rgbd_direct_depth_information_scale").value
        )
        self.rgbd_direct_photometric_information_scale = float(
            self.get_parameter(
                "rgbd_direct_photometric_information_scale"
            ).value
        )
        if (
            self.rgbd_direct_factor_minimum_tracks < 4
            or self.rgbd_direct_factor_maximum_tracks
            < self.rgbd_direct_factor_minimum_tracks
            or self.rgbd_direct_factor_maximum_depth_rmse_m <= 0.0
            or self.rgbd_direct_factor_maximum_photometric_rmse <= 0.0
            or not 0.0 < self.rgbd_direct_depth_information_scale <= 1.0
            or not 0.0
            < self.rgbd_direct_photometric_information_scale <= 1.0
        ):
            raise ValueError("RGB-D direct factor configuration is invalid")
        self.rgbd_depth_factor_enabled = bool(
            self.get_parameter("rgbd_depth_factor_enabled").value
        )
        self.rgbd_depth_factor_tolerance_s = float(
            self.get_parameter("rgbd_depth_factor_tolerance_s").value
        )
        self.rgbd_depth_factor_minimum_tracks = int(
            self.get_parameter("rgbd_depth_factor_minimum_tracks").value
        )
        self.rgbd_depth_factor_maximum_tracks = int(
            self.get_parameter("rgbd_depth_factor_maximum_tracks").value
        )
        self.rgbd_depth_factor_maximum_rmse_m = float(
            self.get_parameter("rgbd_depth_factor_maximum_rmse_m").value
        )
        self.rgbd_depth_factor_information_scale = float(
            self.get_parameter("rgbd_depth_factor_information_scale").value
        )
        self.rgbd_depth_healthy_lidar_profile_information = float(
            self.get_parameter(
                "rgbd_depth_healthy_lidar_profile_information"
            ).value
        )
        self.rgbd_depth_healthy_lidar_stride = int(
            self.get_parameter("rgbd_depth_healthy_lidar_stride").value
        )
        if (
            self.rgbd_depth_factor_tolerance_s < 0.0
            or self.rgbd_depth_factor_minimum_tracks < 4
            or self.rgbd_depth_factor_maximum_tracks
            < self.rgbd_depth_factor_minimum_tracks
            or self.rgbd_depth_factor_maximum_rmse_m <= 0.0
            or not 0.0 < self.rgbd_depth_factor_information_scale <= 1.0
            or self.rgbd_depth_healthy_lidar_profile_information < 0.0
            or self.rgbd_depth_healthy_lidar_stride < 1
        ):
            raise ValueError("RGB-D depth factor configuration is invalid")
        self.visual_time_offset_s = float(
            self.get_parameter("visual_time_offset_s").value)
        self.visual_time_calibration_enabled = bool(
            self.get_parameter("visual_time_calibration_enabled").value
        )
        self.visual_time_calibration_apply_locked = bool(
            self.get_parameter("visual_time_calibration_apply_locked").value
        )
        self.visual_time_calibrator = OnlineVisualTimeCalibrator(
            initial_offset_s=self.visual_time_offset_s,
            window_s=float(self.get_parameter(
                "visual_time_calibration_window_s").value),
            minimum_pairs=int(self.get_parameter(
                "visual_time_calibration_minimum_pairs").value),
            offset_range_s=float(self.get_parameter(
                "visual_time_calibration_range_s").value),
            offset_step_s=float(self.get_parameter(
                "visual_time_calibration_step_s").value),
            minimum_correlation=float(self.get_parameter(
                "visual_time_calibration_minimum_correlation").value),
            minimum_correlation_margin=float(self.get_parameter(
                "visual_time_calibration_minimum_margin").value),
            minimum_peak_separation_s=float(self.get_parameter(
                "visual_time_calibration_peak_exclusion_s").value),
            reject_boundary_candidates=bool(self.get_parameter(
                "visual_time_calibration_reject_boundary_candidates").value),
            lock_count=int(self.get_parameter(
                "visual_time_calibration_lock_count").value),
            stability_tolerance_s=float(self.get_parameter(
                "visual_time_calibration_stability_s").value),
            minimum_accumulated_rotation_rad=float(self.get_parameter(
                "visual_time_calibration_minimum_accumulated_rotation_rad"
            ).value),
            minimum_interval_rotation_rad=float(self.get_parameter(
                "visual_time_calibration_minimum_interval_rotation_rad"
            ).value),
            minimum_lock_candidate_separation_s=float(self.get_parameter(
                "visual_time_calibration_minimum_lock_candidate_separation_s"
            ).value),
        )
        self.last_visual_time_calibration = (
            self.visual_time_calibrator.last_update
        )
        self.visual_time_calibration_lock = threading.Lock()
        self.visual_time_calibration_pending_lock = threading.Lock()
        self.visual_time_calibration_drain_lock = threading.Lock()
        self.visual_time_calibration_pending = deque()
        self.visual_time_calibration_pending_capacity = 256
        self.visual_time_calibration_vote_history = deque(maxlen=32)
        self.visual_initialization_enabled = bool(
            self.get_parameter("visual_initialization_enabled").value
        )
        self.visual_initializer = VisualInitializationGate(
            minimum_batches=int(self.get_parameter(
                "visual_initialization_minimum_batches").value),
            require_time_lock=bool(self.get_parameter(
                "visual_initialization_require_time_lock").value),
        )
        self.visual_state_tolerance_s = float(
            self.get_parameter("visual_state_tolerance_s").value
        )
        self.visual_pending_enabled = bool(
            self.get_parameter("visual_pending_enabled").value
        )
        self.visual_pending_max_wait_s = float(
            self.get_parameter("visual_pending_max_wait_s").value
        )
        self.visual_pending_latest_horizon_s = float(
            self.get_parameter("visual_pending_latest_horizon_s").value
        )
        self.visual_pending_max_wall_wait_s = float(
            self.get_parameter("visual_pending_max_wall_wait_s").value
        )
        self.visual_pending_max_queue = int(
            self.get_parameter("visual_pending_max_queue").value
        )
        self.visual_factor_score_topic = str(
            self.get_parameter("visual_factor_score_topic").value
        )
        self.visual_factor_score_tolerance_s = float(
            self.get_parameter("visual_factor_score_tolerance_s").value
        )
        self.visual_factor_score_max_wait_s = float(
            self.get_parameter("visual_factor_score_max_wait_s").value
        )
        self.visual_factor_score_max_wall_wait_s = float(
            self.get_parameter("visual_factor_score_max_wall_wait_s").value
        )
        self.visual_factor_score_history_size = int(
            self.get_parameter("visual_factor_score_history_size").value
        )
        self.visual_minimum_tracks = int(
            self.get_parameter("visual_minimum_tracks").value)
        self.visual_pnp_minimum_inlier_ratio = float(
            self.get_parameter("visual_pnp_minimum_inlier_ratio").value
        )
        self.visual_pnp_minimum_information_rank = int(
            self.get_parameter("visual_pnp_minimum_information_rank").value
        )
        self.visual_pnp_maximum_condition_number = float(
            self.get_parameter("visual_pnp_maximum_condition_number").value
        )
        self.visual_pnp_maximum_mean_reprojection_error_px = float(
            self.get_parameter(
                "visual_pnp_maximum_mean_reprojection_error_px"
            ).value
        )
        self.visual_information_reference_tracks = int(
            self.get_parameter("visual_information_reference_tracks").value
        )
        self.visual_pixel_sigma_normalized = float(
            self.get_parameter("visual_pixel_sigma_normalized").value
        )
        self.visual_inverse_depth_variance_scale = float(
            self.get_parameter("visual_inverse_depth_variance_scale").value
        )
        self.visual_state_consistency_enabled = bool(
            self.get_parameter("visual_state_consistency_enabled").value
        )
        self.visual_state_innovation_maximum_rmse_px = float(
            self.get_parameter(
                "visual_state_innovation_maximum_rmse_px"
            ).value
        )
        self.visual_minimum_projectable_track_ratio = float(
            self.get_parameter(
                "visual_minimum_projectable_track_ratio"
            ).value
        )
        self.visual_minimum_jacobian_rank = int(
            self.get_parameter("visual_minimum_jacobian_rank").value
        )
        self.visual_maximum_jacobian_condition_number = float(
            self.get_parameter(
                "visual_maximum_jacobian_condition_number"
            ).value
        )
        self.visual_rotation_body_camera = np.asarray(self.get_parameter(
            "visual_rotation_body_camera").value, dtype=float).reshape(3, 3)
        self.visual_translation_body_camera = np.asarray(
            self.get_parameter("visual_translation_body_camera_m").value, dtype=float)
        if (
            self.visual_state_tolerance_s <= 0.0 or self.visual_minimum_tracks < 4
            or not 0.0 < self.visual_pnp_minimum_inlier_ratio <= 1.0
            or not 1 <= self.visual_pnp_minimum_information_rank <= 6
            or self.visual_pnp_maximum_condition_number <= 1.0
            or self.visual_pnp_maximum_mean_reprojection_error_px <= 0.0
            or self.visual_information_reference_tracks < 1
            or self.visual_pending_latest_horizon_s <= 0.0
            or self.visual_pending_max_wait_s <= 0.0
            or self.visual_pending_max_wall_wait_s <= 0.0
            or self.visual_pending_max_queue < 2
            or not self.visual_factor_score_topic
            or self.visual_factor_score_tolerance_s < 0.0
            or self.visual_factor_score_max_wait_s <= 0.0
            or self.visual_factor_score_max_wall_wait_s <= 0.0
            or self.visual_factor_score_history_size < 2
            or self.visual_pixel_sigma_normalized <= 0.0
            or self.visual_inverse_depth_variance_scale < 0.0
            or self.visual_state_innovation_maximum_rmse_px <= 0.0
            or not 0.0 < self.visual_minimum_projectable_track_ratio <= 1.0
            or not 1 <= self.visual_minimum_jacobian_rank <= 6
            or self.visual_maximum_jacobian_condition_number <= 1.0
            or self.visual_translation_body_camera.shape != (3,)
            or not np.allclose(
                self.visual_rotation_body_camera.T @ self.visual_rotation_body_camera,
                np.eye(3), atol=1.0e-5,
            )
        ):
            raise ValueError("visual reprojection configuration is invalid")
        self.fixed_covariance_inflation = float(
            self.get_parameter("fixed_covariance_inflation").value)
        if (
            not math.isfinite(self.fixed_covariance_inflation)
            or self.fixed_covariance_inflation < 1.0
        ):
            raise ValueError("fixed_covariance_inflation must be at least one")
        self.max_path = int(self.get_parameter("publish_path_length").value)
        self.path_publish_period_s = float(
            self.get_parameter("path_publish_period_s").value
        )
        self.path_minimum_translation_m = float(
            self.get_parameter("path_minimum_translation_m").value
        )
        self.path_minimum_rotation_rad = float(
            self.get_parameter("path_minimum_rotation_rad").value
        )
        if (
            self.path_publish_period_s <= 0.0
            or self.path_minimum_translation_m < 0.0
            or self.path_minimum_rotation_rad < 0.0
        ):
            raise ValueError("path publication limits are invalid")
        self.relocalization_enabled = bool(
            self.get_parameter("relocalization_enabled").value
        )
        self.relocalization_pending_timeout_s = max(0.2, float(
            self.get_parameter("relocalization_pending_timeout_s").value), )
        self.relocalization_future_wait_timeout_s = max(
            self.relocalization_pending_timeout_s,
            float(self.get_parameter(
                "relocalization_future_wait_timeout_s"
            ).value),
        )
        self.relocalization_state_tolerance_s = max(0.01, float(
            self.get_parameter("relocalization_state_tolerance_s").value), )
        self.relocalization_result_max_age_s = max(
            self.relocalization_state_tolerance_s,
            float(self.get_parameter("relocalization_result_max_age_s").value),
        )
        self.backend_solver_mode = str(
            self.get_parameter("backend_solver_mode").value
        ).lower()
        if self.backend_solver_mode not in {"manifold", "linear"}:
            raise ValueError("backend_solver_mode must be manifold or linear")
        self.allow_lio_pose_fallback = bool(
            self.get_parameter("allow_lio_pose_fallback").value
        )
        self.transactional_update_enabled = bool(
            self.get_parameter("transactional_update_enabled").value
        )
        self.marginal_prior_suppress_historical_lidar_weak = bool(
            self.get_parameter(
                "marginal_prior_suppress_historical_lidar_weak"
            ).value
        )
        self.lidar_prediction_gate_enabled = bool(
            self.get_parameter("lidar_prediction_gate_enabled").value
        )
        self.lidar_prediction_gate_max_position_m = float(
            self.get_parameter("lidar_prediction_gate_max_position_m").value
        )
        self.lidar_prediction_gate_max_yaw_rad = float(
            self.get_parameter("lidar_prediction_gate_max_yaw_rad").value
        )
        self.lidar_prediction_gate_recovery_after_rejections = int(
            self.get_parameter(
                "lidar_prediction_gate_recovery_after_rejections"
            ).value
        )
        self.lidar_prediction_recovery_weight = float(
            self.get_parameter("lidar_prediction_recovery_weight").value
        )
        self.lidar_prediction_recovery_inflation = float(
            self.get_parameter("lidar_prediction_recovery_inflation").value
        )
        if (
            self.lidar_prediction_gate_max_position_m <= 0.0
            or not math.isfinite(self.lidar_prediction_gate_max_position_m)
            or self.lidar_prediction_gate_max_yaw_rad <= 0.0
            or not math.isfinite(self.lidar_prediction_gate_max_yaw_rad)
            or self.lidar_prediction_gate_recovery_after_rejections < 1
            or self.lidar_prediction_recovery_weight <= 0.0
            or self.lidar_prediction_recovery_weight > 1.0
            or not math.isfinite(self.lidar_prediction_recovery_weight)
            or self.lidar_prediction_recovery_inflation < 1.0
            or not math.isfinite(self.lidar_prediction_recovery_inflation)
        ):
            raise ValueError(
                "LiDAR prediction gate and recovery parameters are invalid")
        self.optimization_integrity_limits = {
            "maximum_translation_correction_m": float(self.get_parameter(
                "optimization_max_translation_correction_m").value),
            "maximum_rotation_correction_rad": float(self.get_parameter(
                "optimization_max_rotation_correction_rad").value),
            "maximum_velocity_correction_mps": float(self.get_parameter(
                "optimization_max_velocity_correction_mps").value),
            "maximum_accel_bias_correction_mps2": float(self.get_parameter(
                "optimization_max_accel_bias_correction_mps2").value),
            "maximum_gyro_bias_correction_radps": float(self.get_parameter(
                "optimization_max_gyro_bias_correction_radps").value),
            "maximum_information_condition": float(self.get_parameter(
                "optimization_max_information_condition").value),
            "information_rank_tolerance": self.marginal_rank_tolerance,
        }
        if any(
            not math.isfinite(value) or value <= 0.0
            for value in self.optimization_integrity_limits.values()
        ):
            raise ValueError("optimization integrity limits must be positive")
        if self.transactional_update_enabled and self.backend_solver_mode != "manifold":
            raise ValueError(
                "transactional updates require the manifold backend")
        self.frontend_state_seed_enabled = bool(
            self.get_parameter("frontend_state_seed_enabled").value
        )
        if self.frontend_state_seed_enabled and BackendStateSeed is None:
            raise ValueError(
                "frontend state seed requires the patched FAST-LIO message overlay"
            )
        self.frontend_scan_prediction_enabled = bool(
            self.get_parameter("frontend_scan_prediction_enabled").value
        )
        self.scan_prediction_maximum_begin_gap_s = float(
            self.get_parameter("scan_prediction_maximum_begin_gap_s").value
        )
        self.scan_prediction_timestamp_tolerance_s = float(
            self.get_parameter("scan_prediction_timestamp_tolerance_s").value
        )
        self.scan_prediction_state_tolerance = float(
            self.get_parameter("scan_prediction_state_tolerance").value
        )
        self.scan_prediction_cache_size = int(
            self.get_parameter("scan_prediction_cache_size").value
        )
        self.scan_prediction_missing_factor_grace_s = float(
            self.get_parameter("scan_prediction_missing_factor_grace_s").value
        )
        self.scan_prediction_contract_failure_threshold = int(
            self.get_parameter(
                "scan_prediction_contract_failure_threshold"
            ).value
        )
        self.scan_prediction_contract_request_timeout_s = float(
            self.get_parameter(
                "scan_prediction_contract_request_timeout_s"
            ).value
        )
        if self.frontend_scan_prediction_enabled:
            if self.backend_solver_mode != "manifold":
                raise ValueError(
                    "front-end scan prediction requires manifold backend")
            if not self.native_lidar_enabled or self.input_trigger_mode != "native_factor":
                raise ValueError(
                    "front-end scan prediction requires native-factor triggering"
                )
            if FrontendScanRequest is None or BackendDeskewTrajectory is None:
                raise ValueError(
                    "front-end scan prediction requires patched FAST-LIO messages"
                )
            if self.input_trigger_mode != "native_factor":
                raise ValueError(
                    "front-end scan prediction requires native-factor triggering"
                )
        if (
            self.scan_prediction_maximum_begin_gap_s < 0.0
            or self.scan_prediction_timestamp_tolerance_s <= 0.0
            or self.scan_prediction_state_tolerance <= 0.0
            or self.scan_prediction_cache_size < 1
            or self.scan_prediction_missing_factor_grace_s <= 0.0
            or self.scan_prediction_contract_failure_threshold < 1
            or self.scan_prediction_contract_request_timeout_s
            <= self.scan_prediction_missing_factor_grace_s
        ):
            raise ValueError("scan prediction limits are invalid")
        self.native_lidar_qos_depth = int(
            self.get_parameter("native_lidar_qos_depth").value
        )
        self.native_worker_queue_size = int(
            self.get_parameter("native_worker_queue_size").value
        )
        self.imu_qos_depth = int(self.get_parameter("imu_qos_depth").value)
        if (
            self.native_lidar_qos_depth < 1
            or self.native_worker_queue_size < 1
            or self.imu_qos_depth < 2
        ):
            raise ValueError("sensor QoS depths must be positive")
        self.native_lidar_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=self.native_lidar_qos_depth,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.frontend_scan_request_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=4,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.backend_trajectory_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=4,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.imu_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=self.imu_qos_depth,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        window_size = max(2, int(self.get_parameter("window_size").value))
        self.window_size = window_size
        self.nonlinear_max_iterations = int(
            self.get_parameter("nonlinear_max_iterations").value
        )
        self.nonlinear_initialization_max_iterations = int(
            self.get_parameter("nonlinear_initialization_max_iterations").value
        )
        self.nonlinear_recovery_max_iterations = int(
            self.get_parameter("nonlinear_recovery_max_iterations").value
        )
        self.nonlinear_reintegration_max_iterations = int(
            self.get_parameter("nonlinear_reintegration_max_iterations").value
        )
        select_nonlinear_iteration_budget(
            self.nonlinear_max_iterations,
            self.nonlinear_initialization_max_iterations,
            self.nonlinear_recovery_max_iterations,
            state_count=3,
        )
        if self.nonlinear_reintegration_max_iterations < 1:
            raise ValueError(
                "nonlinear_reintegration_max_iterations must be positive"
            )
        if self.frontend_map_commit_delay_states >= self.window_size:
            raise ValueError(
                "front-end map commit delay must be smaller than window_size"
            )
        if self.backend_solver_mode == "manifold":
            self.backend = ManifoldSlidingWindowBackend(
                max_states=window_size,
                max_iterations=self.nonlinear_max_iterations,
                damping=float(self.get_parameter("nonlinear_damping").value),
                convergence_threshold=float(
                    self.get_parameter("nonlinear_convergence_threshold").value
                ),
                lidar_huber_delta=float(
                    self.get_parameter("lidar_huber_delta").value
                ),
                lm_max_trials=int(self.get_parameter("lm_max_trials").value),
                lm_damping_up=float(
                    self.get_parameter("lm_damping_up").value
                ),
                lm_damping_down=float(
                    self.get_parameter("lm_damping_down").value
                ),
                marginal_rank_tolerance=self.marginal_rank_tolerance,
                cpp_math_core_enabled=bool(
                    self.get_parameter("cpp_math_core_enabled").value
                ),
                profiling_enabled=self.performance_profiling_enabled,
                profiling_capacity=self.performance_profiling_capacity,
            )
            self.backend.suppress_historical_lidar_weak = (
                self.marginal_prior_suppress_historical_lidar_weak
            )
        else:
            self.backend = SlidingWindowBackend(max_states=window_size)
        cpp_math_required = bool(
            self.get_parameter("cpp_math_core_required").value
        )
        cpp_math_active = bool(
            getattr(self.backend, "cpp_math_core_enabled", False)
        )
        if cpp_math_required and not cpp_math_active:
            raise RuntimeError(
                "C++ backend math core is required but unavailable; build and "
                "source uf_backend_core_cpp or disable cpp_math_core_required"
            )
        self.path = Path()
        self.path.poses = []
        self.imu_buffer = deque(maxlen=10000)
        self.flow_buffer = deque(maxlen=3000)
        self.flow_buffer_lock = threading.Lock()
        self.flow_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=self.flow_qos_depth,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.flow_rotation_gate = OpticalFlowRotationGate(
            FlowRotationGateConfig(
                lower_yaw_rate_radps=float(self.get_parameter(
                    "flow_rotation_lower_yaw_rate_radps").value),
                upper_yaw_rate_radps=float(self.get_parameter(
                    "flow_rotation_upper_yaw_rate_radps").value),
                recovery_dwell_s=float(self.get_parameter(
                    "flow_rotation_recovery_dwell_s").value),
                recovery_ramp_s=float(self.get_parameter(
                    "flow_rotation_recovery_ramp_s").value),
                minimum_translation_m=float(self.get_parameter(
                    "flow_rotation_minimum_translation_m").value),
                allow_compensated_rotation=self.flow_rotation_allow_compensated,
            )
        )
        self.native_lidar_buffer = NativeFactorBuffer(max_size=128)
        self.relocalization_lock = threading.Lock()
        self.pending_relocalization = None
        self.map_from_lio = np.eye(4, dtype=float)
        self.native_work_queue = queue.Queue(
            maxsize=self.native_worker_queue_size)
        self.native_worker_stop = threading.Event()
        self.native_worker_thread = None
        self.pending_lio = deque(maxlen=32)
        self.pending_imu_lio = deque(maxlen=64)
        self.gnss_lock = threading.Lock()
        self.gnss_buffer = deque(maxlen=512)
        self.latest_gnss = None
        self.last_gnss_admitted = None
        self.barometer_lock = threading.Lock()
        self.projector = None
        self.lio_origin = None
        self.last_lio_stamp = None
        self.last_lio_position = None
        self.last_lio_yaw = 0.0
        self.optimization_anchor = None
        self.optimization_anchor_generation = 0
        self.last_unified_output_stamp_s = None
        self.last_imu_arrival_stamp = None
        self.imu_max_positive_arrival_gap_s = 0.0
        self.scheduler = {}
        self.scheduler_reasons = {}
        self.scheduler_arrival = None
        self.scheduler_health = "UNAVAILABLE"
        self.scores = {}
        self.visual_lock = threading.Lock()
        self.visual_tracks = deque(maxlen=64)
        self.rgbd_geometry_lock = threading.Lock()
        # Keep one frame of pairing slack: feature, geometry and direct topics
        # are published separately, so strict one-slot queues can overwrite a
        # matching sibling before the executor delivers it.
        self.rgbd_geometry_tracks = deque(maxlen=2)
        self.rgbd_direct_lock = threading.Lock()
        self.rgbd_direct_tracks = deque(maxlen=2)
        self.pending_visual_candidates = deque()
        self.pending_visual_keys = set()
        self.visual_candidate_sequence = 0
        self.visual_factor_scores = deque(
            maxlen=self.visual_factor_score_history_size
        )
        self.visual_factor_score_wait_started = {}
        self.visual_factor_score_sequence = 0
        self.last_visual_factor_score_stamp_s = -1.0
        self.last_visual_factor_score_weight = 0.0
        self.last_visual_factor_score_degradation = 1.0
        self.last_visual_factor_score_match_error_s = -1.0
        self.last_visual_factor_score_reasons = ()
        self.last_visual_combined_reasons = ()
        self.visual_state_stamps = deque(maxlen=self.window_size)
        self.visual_timing_reason_counts = Counter()
        self.last_visual_reason = "disabled"
        self.last_visual_reprojection_rmse_normalized = -1.0
        self.last_visual_reprojection_residual_dimension = 0
        self.last_visual_prefit_rmse_normalized = -1.0
        self.last_visual_prefit_rmse_px = -1.0
        self.last_visual_prefit_valid_track_ratio = -1.0
        self.last_visual_prefit_jacobian_rank = 0
        self.last_visual_prefit_jacobian_condition = -1.0
        self.last_visual_prefit_nis_per_dof = -1.0
        self.last_visual_prefit_information_trace = -1.0
        self.last_visual_prefit_information_max_eigenvalue = -1.0
        self.last_visual_pnp_inlier_ratio = -1.0
        self.last_visual_pnp_information_rank = 0
        self.last_visual_pnp_condition_number = -1.0
        self.last_visual_pnp_mean_reprojection_error_px = -1.0
        self.last_visual_batch_information_scale = 1.0
        self.last_rgbd_depth_reason = "disabled"
        self.last_rgbd_depth_track_count = 0
        self.last_rgbd_depth_prefit_rmse_m = -1.0
        self.last_rgbd_depth_stamp_s = -1.0
        self.last_rgbd_depth_axis_profile_information = np.zeros(
            3, dtype=float
        )
        self.last_rgbd_depth_axis_support = np.zeros(3, dtype=float)
        self.last_rgbd_direct_reason = "disabled"
        self.last_rgbd_direct_track_count = 0
        self.last_rgbd_direct_depth_rmse_m = -1.0
        self.last_rgbd_direct_photometric_rmse = -1.0
        self.last_rgbd_direct_photometric_information_scale = 1.0
        self.rgbd_depth_candidate_sequence = 0
        self.axis_handoff_latched = np.zeros(3, dtype=bool)
        self.lidar_axis_observability_latched = np.zeros(3, dtype=bool)
        self.last_lidar_axis_information_scale = np.ones(3, dtype=float)
        self.last_axis_handoff_alternative_information = np.zeros(
            3, dtype=float
        )
        self.last_axis_handoff_gnss_information = np.zeros(3, dtype=float)
        self.last_axis_handoff_rgbd_information = np.zeros(3, dtype=float)
        self.last_axis_handoff_barometer_information = np.zeros(3, dtype=float)
        self.last_axis_map_protected = np.zeros(3, dtype=bool)
        self.last_axis_map_protection_sources = ("none", "none", "none")
        self.last_axis_reliability = np.zeros(3, dtype=float)
        self.last_axis_degradation = np.ones(3, dtype=float)
        self.last_axis_global_reliability = np.zeros(3, dtype=float)
        self.last_axis_supporting_sources = ((), (), ())
        self.counts = {
            "lio": 0, "published": 0, "lidar_factors": 0,
            "lidar_disabled": 0,
            "gnss_factor_attempts": 0,
            "gnss_factor_records": 0,
            "gnss_factors": 0,
            "gnss_disabled_scheduler": 0,
            "gnss_rejected_nis": 0,
            "gnss_rejected_low_weight": 0,
            "gnss_invalid_fix_rejected": 0,
            "gnss_xy_rejected_nis": 0,
            "gnss_z_rejected_nis": 0,
            "gnss_xy_admitted": 0,
            "gnss_z_admitted": 0,
            "gnss_xy_robust_downweighted": 0,
            "gnss_z_robust_downweighted": 0,
            "gnss_z_reanchor_attempts": 0,
            "gnss_z_reanchor_factors": 0,
            "gnss_z_recovery_factors": 0,
            "gnss_all_axes_inconsistent": 0,
            "gnss_prefit_recovery_floor": 0,
            "gnss_prefit_valid": 0,
            "gnss_prefit_invalid": 0,
            "gnss_prefit_covariance_unavailable": 0,
            "gnss_provisional_bootstrap_admitted": 0,
            "gnss_jump_rejected": 0,
            "gnss_received": 0, "gnss_consumed": 0,
            "gnss_duplicates": 0, "gnss_out_of_order": 0,
            "gnss_stale_discarded": 0, "gnss_superseded": 0,
            "barometer_received": 0,
            "barometer_invalid": 0,
            "barometer_factor_attempts": 0,
            "barometer_factors": 0,
            "barometer_segments_started": 0,
            "barometer_segments_ended": 0,
            "barometer_reference_updates": 0,
            "barometer_reference_rejected": 0,
            "flow_received": 0,
            "flow_factor_attempts": 0, "flow_factors": 0,
            "flow_clock_mismatch": 0,
            "flow_disabled_scheduler": 0,
            "flow_disabled_quality": 0,
            "flow_disabled_speed": 0,
            "flow_disabled_rotation": 0,
            "visual_received": 0, "visual_factor_attempts": 0,
            "visual_factors": 0, "visual_rejected_time": 0,
            "visual_rejected_tracks": 0,
            "rgbd_geometry_received": 0,
            "rgbd_geometry_superseded": 0,
            "rgbd_geometry_matched": 0,
            "rgbd_geometry_missing": 0,
            "rgbd_direct_received": 0,
            "rgbd_direct_superseded": 0,
            "rgbd_direct_matched": 0,
            "rgbd_direct_missing": 0,
            "rgbd_direct_factor_attempts": 0,
            "rgbd_direct_factors": 0,
            "rgbd_direct_rejected_tracks": 0,
            "rgbd_direct_rejected_prefit": 0,
            "rgbd_direct_photometric_downweighted": 0,
            "rgbd_depth_factor_attempts": 0,
            "rgbd_depth_factors": 0,
            "rgbd_depth_rejected_tracks": 0,
            "rgbd_depth_rejected_prefit": 0,
            "rgbd_depth_skipped_healthy_lidar": 0,
            "native_lidar_axis_handoff_frames": 0,
            "native_lidar_axis_handoff_x": 0,
            "native_lidar_axis_handoff_y": 0,
            "native_lidar_axis_handoff_z": 0,
            "native_lidar_axis_conditional_factors": 0,
            "native_lidar_axis_map_protected_frames": 0,
            "native_lidar_axis_map_protected_x": 0,
            "native_lidar_axis_map_protected_y": 0,
            "native_lidar_axis_map_protected_z": 0,
            "visual_window_associated_candidates": 0,
            "visual_solver_accepted": 0,
            "visual_solver_rejected": 0,
            "visual_pending_enqueued": 0,
            "visual_pending_superseded": 0,
            "visual_pending_waits": 0,
            "visual_pending_expired": 0,
            "visual_pending_overflow": 0,
            "visual_prebootstrap_dropped": 0,
            "visual_pending_pre_window_dropped": 0,
            "visual_duplicate_candidates": 0,
            "visual_factor_scores_received": 0,
            "visual_factor_score_waits": 0,
            "visual_factor_score_matched": 0,
            "visual_factor_score_missing": 0,
            "visual_factor_score_invalid": 0,
            "visual_quality_rejected_dv": 0,
            "visual_state_consistency_rejected": 0,
            "visual_linearization_invalid": 0,
            "visual_pnp_observability_rejected": 0,
            "visual_batch_information_normalized": 0,
            "visual_initialization_waits": 0,
            "visual_initializations": 0,
            "visual_time_calibration_updates": 0,
            "visual_time_calibration_accepted": 0,
            "visual_time_calibration_rejected": 0,
            "visual_time_calibration_geometry_rejected": 0,
            "visual_time_calibration_pending_enqueued": 0,
            "visual_time_calibration_pending_overflow": 0,
            "visual_time_calibration_imu_history_missing": 0,
            "flow_los_diagnostic_samples": 0,
            "flow_los_diagnostic_invalid": 0,
            "flow_lever_arm_compensated": 0,
            "flow_lever_arm_unavailable": 0,
            "flow_lever_arm_per_exposure": 0,
            "flow_lever_arm_interval_fallback": 0,
            "flow_range_facet_accepted": 0,
            "flow_range_facet_rejected": 0,
            "imu_factors": 0, "imu_invalid": 0, "optimization_errors": 0,
            "imu_reintegrations": 0,
            "imu_reintegrations_deferred": 0,
            "calibration_updates": 0, "calibration_accepted": 0,
            "calibration_frozen": 0,
            "calibration_motion_received": 0,
            "calibration_motion_rejected": 0,
            "calibration_seed_initialized": 0,
            "calibration_seed_lock_busy": 0,
            "lidar_anchor_overrides": 0, "imu_residual_updates": 0,
            "imu_residual_errors": 0,
            "native_lidar_received": 0, "native_lidar_invalid": 0,
            "native_lidar_factors": 0, "native_lidar_hard_disabled": 0,
            "native_lidar_pose_fallbacks": 0, "native_lidar_pair_timeouts": 0,
            "native_lidar_relinearized": 0,
            "native_lidar_condensed_fallbacks": 0,
            "native_lidar_directionally_degenerate": 0,
            "native_lidar_prediction_gate_rejections": 0,
            "native_lidar_prediction_gate_recoveries": 0,
            "native_lidar_prediction_recovery_factors": 0,
            "native_lidar_epoch_stale_rejected": 0,
            "native_lidar_epoch_future_rejected": 0,
            "native_trigger_only_frames": 0,
            "native_trigger_duplicates": 0,
            "native_trigger_nonmonotonic": 0,
            "native_trigger_sequence_conflicts": 0,
            "native_trigger_sequence_gaps": 0,
            "native_trigger_terminal_stale": 0,
            "native_trigger_waiting_for_initial_factor": 0,
            "native_worker_queue_overflow": 0,
            "native_worker_queue_superseded": 0,
            "native_worker_queue_discarded": 0,
            "native_worker_latest_skipped": 0,
            "native_worker_errors": 0,
            "optimized_states_committed": 0,
            "optimized_odom_published": 0,
            "optimized_odom_nonmonotonic_suppressed": 0,
            "optimized_odom_mode_suppressed": 0,
            "optimized_odom_anchor_only": 0,
            "frontend_activation_published": 0,
            "live_propagation_attempts": 0,
            "live_propagation_published": 0,
            "live_propagation_rejected": 0,
            "auxiliary_keyframe_attempts": 0,
            "auxiliary_keyframe_committed": 0,
            "auxiliary_keyframe_rejected": 0,
            "auxiliary_keyframe_errors": 0,
            "lio_pose_inputs_ignored": 0,
            "imu_propagated_initializations": 0,
            "imu_pair_timeouts": 0,
            "imu_received": 0,
            "imu_nonmonotonic_arrivals": 0,
            "imu_startup_waits": 0,
            "imu_startup_bias_accepted": 0,
            "imu_startup_bias_rejected": 0,
            "relocalization_resets": 0,
            "relocalization_rejections": 0,
            "relocalization_expired": 0,
            "relocalization_epoch_factor_drops": 0,
            "marginal_covariance_updates": 0,
            "marginal_covariance_errors": 0,
            "marginal_covariance_reuses": 0,
            "anchor_covariance_propagations": 0,
            "path_samples": 0,
            "path_messages": 0,
            "optimization_rejected": 0,
            "optimization_rollbacks": 0,
            "frontend_state_seeds": 0,
            "scan_prediction_requests": 0,
            "scan_prediction_published": 0,
            "scan_prediction_rejected": 0,
            "scan_prediction_cache_hits": 0,
            "scan_prediction_cache_misses": 0,
            "scan_prediction_reuse_rejected": 0,
            "scan_prediction_deferred": 0,
            "scan_prediction_deferred_released": 0,
            "scan_prediction_duplicate_requests": 0,
            "scan_prediction_stale_requests": 0,
            "scan_prediction_missing_factor_skips": 0,
            "scan_prediction_contract_trips": 0,
            "scan_prediction_contract_recoveries": 0,
            "scan_prediction_contract_output_suppressed": 0,
            "native_consumed_without_state_commit": 0,
            "frontend_map_pose_published": 0,
            "frontend_map_pose_rejected": 0,
            "frontend_map_pose_waiting": 0,
            "frontend_map_pose_duplicates": 0,
        }
        self.imu_invalid_reasons = {}
        self.last_reason = "waiting_for_lio"
        self.last_scan_prediction_reason = "none"
        self.last_covariance_source = "fixed_startup"
        self.last_state_covariance = None
        self.last_covariance_stamp_s = None
        self.last_path_sample_position = None
        self.last_path_sample_orientation = None
        self.last_path_publish_stamp_s = None
        self.last_callback_ms = 0.0
        self.phase_timing = {
            name: {
                "count": 0,
                "total_ms": 0.0,
                "max_ms": 0.0,
                "last_ms": 0.0,
                "samples": (
                    deque(maxlen=self.performance_profiling_capacity)
                    if self.performance_profiling_enabled else None
                ),
            }
            for name in (
                "prepare", "pre_state", "snapshot", "add_state", "lidar_factor",
                "gnss_factor", "flow_factor", "visual_association",
                "visual_factor_construction", "barometer_factor", "imu_factor",
                "aux_factors",
                "optimize", "reintegrate", "integrity_check", "commit",
                "post_optimize", "publish",
            )
        }
        self.current_cycle_phase = None
        self.performance_cycle_trace = deque(
            maxlen=self.performance_profiling_capacity
        )
        self.gc_profiler = (
            GarbageCollectionProfiler()
            if self.performance_profiling_enabled else None
        )
        self.last_imu_reason = "unavailable"
        self.last_imu_startup_reason = "not_attempted"
        self.last_imu_startup_sample_count = 0
        self.last_imu_startup_span_s = 0.0
        self.last_imu_startup_accel_bias = np.zeros(3, dtype=float)
        self.last_imu_startup_gyro_bias = np.zeros(3, dtype=float)
        self.last_imu_preintegration_residual_mahalanobis = -1.0
        self.last_gnss_prefit_nis = -1.0
        self.last_gnss_prefit_xy_nis = -1.0
        self.last_gnss_prefit_z_nis = -1.0
        self.last_gnss_xy_admitted = False
        self.last_gnss_z_admitted = False
        self.last_gnss_xy_information_scale = 0.0
        self.last_gnss_z_information_scale = 0.0
        self.last_gnss_factor_covariance = np.full(
            3, math.inf, dtype=float
        )
        self.last_gnss_solver_information = np.zeros(3, dtype=float)
        self.gnss_z_reanchor_consecutive = 0
        self.last_gnss_z_reanchor_applied = False
        self.last_gnss_z_reanchor_target_m = math.nan
        self.last_gnss_prefit_residual_norm_m = -1.0
        self.last_gnss_prefit_residual_xyz = np.full(3, math.nan, dtype=float)
        self.last_gnss_prefit_stamp_s = -1.0
        self.last_gnss_degradation_score = 1.0
        self.last_gnss_reliability_weight = 0.0
        self.last_gnss_effective_information_scale = 0.0
        self.last_gnss_admission_reason = "not_attempted"
        self.last_gnss_time_compensation_age_s = 0.0
        self.last_gnss_time_compensation_delta_m = np.zeros(3, dtype=float)
        self.last_gnss_time_compensation_variance_m2 = np.zeros(
            3, dtype=float
        )
        self.last_gnss_time_compensation_reason = "not_attempted"
        self.last_barometer_reason = "inactive"
        self.last_barometer_segment_id = 0
        self.last_barometer_prefit_residual_m = -1.0
        self.last_barometer_information_scale = 0.0
        self.last_barometer_variance_m2 = math.inf
        self.last_barometer_stamp_s = -1.0
        self.last_barometer_measurement_height_m = math.nan
        self.last_barometer_anchor_source = "none"
        self.last_barometer_anchor_reference_age_s = math.inf
        self.last_barometer_reference_reason = "not_attempted"
        self.last_barometer_reference_stamp_s = -1.0
        self.last_barometer_reference_z_m = math.nan
        self.last_imu_residual_error = "none"
        self.last_exception = "none"
        self.last_flow_reason = "unavailable"
        self.last_flow_factor_type = "unavailable"
        self.last_flow_rotation_phase = "unavailable"
        self.last_flow_rotation_weight = 0.0
        self.last_flow_yaw_rate_abs_radps = -1.0
        self.last_flow_los_diagnostic = None
        self.last_flow_lever_arm_displacement = None
        self.last_flow_speed_mps = -1.0
        self.last_flow_speed_limit_mps = -1.0
        self.last_flow_range_sigma_m = -1.0
        self.last_flow_covariance_m2 = math.inf
        self.flow_los_residual_no_lever_norms = deque(maxlen=5000)
        self.flow_los_residual_norms = deque(maxlen=5000)
        self.flow_los_lever_arm_norms = deque(maxlen=5000)
        self.flow_lever_arm_displacement_norms = deque(maxlen=5000)
        self.last_lidar_prediction_position_innovation_m = -1.0
        self.last_lidar_prediction_yaw_innovation_rad = -1.0
        self.last_lidar_source = "unavailable"
        self.last_native_sequence = -1
        self.last_native_input_stamp_ns = None
        self.last_native_input_sequence = None
        self.last_native_input_arrival_s = None
        self.last_native_callback_source_age_s = -1.0
        self.last_native_worker_source_age_s = -1.0
        self.last_output_source_age_s = -1.0
        self.last_output_position_variance_m2 = math.inf
        self.last_output_orientation_variance_rad2 = math.inf
        self.last_output_velocity_variance_m2ps2 = math.inf
        self.maximum_output_position_variance_m2 = 0.0
        self.maximum_output_orientation_variance_rad2 = 0.0
        self.maximum_auxiliary_position_variance_m2 = 0.0
        self.last_scan_request_arrival_s = None
        self.last_live_propagation_reason = "not_attempted"
        self.last_auxiliary_keyframe_reason = "not_attempted"
        self.last_output_source = "none"
        self.last_state_trigger_source = "none"
        self.last_native_factor_reset_counter = 0
        self.last_native_matches = 0
        self.last_native_stamp_error_ms = -1.0
        self.last_native_effective_rank = 0
        self.last_native_translation_rank = 0
        self.last_native_rotation_rank = 0
        self.last_native_condition_number = math.inf
        self.last_native_characteristic_range_m = 0.0
        self.last_native_normalized_eigenvalues = np.zeros(6, dtype=float)
        self.last_native_vertical_raw_information = 0.0
        self.last_native_vertical_profile_information = 0.0
        self.last_native_vertical_coupling_retention_ratio = 0.0
        self.last_native_normal_z_energy_fraction = 0.0
        self.last_native_horizontal_plane_fraction = -1.0
        self.last_native_axis_raw_information = np.zeros(3, dtype=float)
        self.last_native_axis_profile_information = np.zeros(3, dtype=float)
        self.last_native_axis_coupling_retention_ratio = np.zeros(3, dtype=float)
        self.last_native_axis_relative_support = np.zeros(3, dtype=float)
        self.last_native_translation_profile_information = np.zeros(
            (3, 3), dtype=float
        )
        self.last_native_translation_normalized_eigenvalues = np.zeros(
            3, dtype=float
        )
        self.last_native_translation_eigenvectors = np.zeros(
            (3, 3), dtype=float
        )
        self.last_native_weakest_translation_direction = np.zeros(3, dtype=float)
        self.last_lidar_subspace_scale = np.eye(3, dtype=float)
        self.previous_lidar_subspace_scale = np.eye(3, dtype=float)
        self.last_lidar_subspace_absolute_information_ratio = np.ones(3)
        self.lidar_subspace_episode_active = False
        self.last_lidar_subspace_weak_modes = 0
        self.last_lidar_subspace_information_scale = np.ones(3, dtype=float)
        self.last_native_health_degradation = 1.0
        self.last_native_consistency_degradation = 0.0
        self.last_native_observability_degradation = np.ones(3, dtype=float)
        self.last_native_combined_degradation = np.ones(3, dtype=float)
        self.last_native_isotropic_information_support = np.zeros(3, dtype=float)
        self.last_output = None
        self.backend_solve_count = 0
        self.backend_solve_ms_total = 0.0
        self.backend_solve_ms_max = 0.0
        self.last_optimization_integrity = OptimizationIntegrity(
            False, "not_evaluated", 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0, math.inf
        )
        self.optimization_integrity_reason_counts = Counter()
        self.native_lidar_prediction_gate_latched = False
        self.native_lidar_prediction_gate_consecutive_rejections = 0
        self.last_native_lidar_prediction_gate_reason = "not_evaluated"
        self.frontend_state_seed_sequence = 0
        self.state_reset_counter = 0
        self.pending_relocalization_candidate_id = 0
        self.pending_relocalization_transaction_id = 0
        self.pending_relocalization_deadline_s = None
        self.last_applied_relocalization_transaction_id = 0
        self.fusion_session_id = int(time.time_ns()) & ((1 << 64) - 1)
        self.scan_prediction_cache = deque(
            maxlen=self.scan_prediction_cache_size
        )
        self.scan_prediction_by_sequence = {}
        self.scan_prediction_contract_lock = threading.RLock()
        self.scan_prediction_contract_established = False
        self.scan_prediction_contract_violated = False
        self.scan_prediction_contract_consecutive_failures = 0
        self.scan_prediction_contract_reason = "waiting_for_handshake"
        self.scan_prediction_contract_first_failure_sequence = -1
        self.scan_prediction_contract_first_failure_stamp_s = -1.0
        self.last_native_consumed_sequence = -1
        self.pending_scan_requests = {}
        self.pending_scan_request_first_seen_s = {}
        self.pending_scan_request_lock = threading.Lock()
        self.scan_prediction_pub = None
        self.deskew_trajectory_pub = None
        self.scheduler_estimator_support = 0.0
        self.active_transaction_snapshot = None
        self.last_frontend_map_pose_reason = "not_evaluated"
        self.last_frontend_map_position_variance_m2 = math.inf
        self.last_frontend_map_orientation_variance_rad2 = math.inf
        self.last_frontend_map_pose_stamp_s = None
        self.last_frontend_map_pose_delay_s = -1.0
        self.last_lidar_map_eligible = False
        self.last_lidar_map_reason = "not_evaluated"
        self.frontend_map_eligibility_by_stamp = {}
        self.frontend_map_eligibility_order = deque()
        self.frontend_map_eligibility_capacity = max(32, 4 * self.window_size)
        self.last_relocalization_reset_stats = {}

        self.odom_pub = self.create_publisher(
            Odometry, str(self.get_parameter("output_topic").value), 20)
        self.frontend_map_pose_pub = self.create_publisher(
            Odometry, self.frontend_map_pose_topic, 20)
        # The LiDAR frontend needs a current local pose to start its next
        # request. This must not share the delayed map-write stream or the
        # unified output stream.
        self.frontend_activation_pose_pub = self.create_publisher(
            Odometry, self.frontend_activation_pose_topic, 20)
        self.path_pub = self.create_publisher(
            Path, str(self.get_parameter("path_topic").value), 10)
        self.diagnostic_pub = self.create_publisher(
            DiagnosticArray, str(
                self.get_parameter("diagnostic_topic").value), 10)
        self.visual_timing_pub = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter("visual_timing_diagnostic_topic").value),
            100,
        )
        self.fusion_epoch_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.fusion_epoch_pub = self.create_publisher(
            FusionEpoch,
            str(self.get_parameter("fusion_epoch_topic").value),
            self.fusion_epoch_qos,
        )
        self._publish_fusion_epoch(
            0.0, 0, 0, applied=False, reason="fusion_session_started"
        )
        self.frontend_state_seed_pub = None
        if self.frontend_state_seed_enabled:
            self.frontend_state_seed_pub = self.create_publisher(
                BackendStateSeed,
                str(self.get_parameter("frontend_state_seed_topic").value),
                self.native_lidar_qos,
            )
        if self.frontend_scan_prediction_enabled:
            self.deskew_trajectory_pub = self.create_publisher(
                BackendDeskewTrajectory,
                str(self.get_parameter("backend_deskew_trajectory_topic").value),
                self.backend_trajectory_qos,
            )
            self.create_subscription(
                FrontendScanRequest,
                str(self.get_parameter("frontend_scan_request_topic").value),
                self._scan_request,
                self.frontend_scan_request_qos,
                callback_group=self.native_callback_group,
            )
        if self.input_trigger_mode == "lio_pair":
            self.create_subscription(
                Odometry, str(self.get_parameter("lio_topic").value),
                self._lio, 20,
                callback_group=self.native_callback_group)
        if self.native_lidar_enabled:
            self.create_subscription(
                NativeLidarFactor,
                str(self.get_parameter("native_lidar_factor_topic").value),
                self._native_lidar,
                self.native_lidar_qos,
                callback_group=self.native_callback_group,
            )
        if self.input_trigger_mode == "lio_pair" and (
            self.native_lidar_enabled or self.backend_solver_mode == "manifold"
        ):
            self.create_timer(
                0.010,
                self._drain_pending_inputs,
                callback_group=self.native_callback_group,
            )
        self.create_subscription(
            NavSatFix, str(self.get_parameter("gnss_topic").value),
            self._gnss, qos_profile_sensor_data)
        if self.barometer_fallback_enabled:
            self.create_subscription(
                FluidPressure,
                str(self.get_parameter("barometer_topic").value),
                self._barometer,
                qos_profile_sensor_data,
            )
        self.create_subscription(
            OpticalFlowRad, str(self.get_parameter("flow_topic").value),
            self._flow, self.flow_qos,
            callback_group=self.flow_callback_group)
        self.create_subscription(
            Imu, str(self.get_parameter("imu_topic").value),
            self._imu, self.imu_qos,
            callback_group=self.imu_callback_group)
        self.create_subscription(
            SchedulerState, str(self.get_parameter("scheduler_topic").value),
            self._scheduler, 20)
        self.create_subscription(
            VisualFeatureTracks,
            str(self.get_parameter("visual_tracks_topic").value),
            self._visual_tracks,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            RgbdGeometryTracks,
            str(self.get_parameter("rgbd_geometry_tracks_topic").value),
            self._rgbd_geometry_tracks,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            RgbdDirectTracks,
            str(self.get_parameter("rgbd_direct_tracks_topic").value),
            self._rgbd_direct_tracks,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            ReliabilityScore,
            self.visual_factor_score_topic,
            self._visual_factor_score,
            qos_profile_sensor_data,
        )
        if self.relocalization_enabled:
            self.create_subscription(
                RelocalizationResult,
                str(self.get_parameter("relocalization_result_topic").value),
                self._relocalization_result, 10)
        if self.online_calibration_enabled:
            self.create_subscription(
                LidarCalibrationMotion,
                str(self.get_parameter("calibration_motion_topic").value),
                self._lidar_calibration_motion,
                qos_profile_sensor_data,
            )
        for modality in ("lidar", "gnss", "imu", "optical_flow", "vision"):
            self.create_subscription(
                ReliabilityScore, f"/reliability/{modality}_score",
                lambda msg, name=modality: self._score(name, msg),
                qos_profile_sensor_data,
            )
        if self.input_trigger_mode == "native_factor":
            self.native_worker_thread = threading.Thread(
                target=self._native_worker_loop,
                name="uf-native-factor-worker",
                daemon=True,
            )
            self.native_worker_thread.start()
            if self.live_propagation_enabled:
                self.create_timer(
                    1.0 / self.live_propagation_rate_hz,
                    self._publish_live_propagation,
                    callback_group=self.live_propagation_callback_group,
                )
        self.create_timer(1.0, self._diagnostics)
        if native_requested and NativeLidarFactor is None:
            self.get_logger().warning(
                "FAST-LIO NativeLidarFactor is unavailable; using lio_pair trigger mode. "
                "Source the patched FAST-LIO overlay before launching the backend.")
        self.get_logger().info(
            f"Unified backend active: solver={self.backend_solver_mode}; "
            f"reliability_mode={self.reliability_mode}; native LiDAR + GNSS/flow; "
            f"input_trigger={self.input_trigger_mode}; "
            f"native_lidar={'on' if self.native_lidar_enabled else 'fallback'}; "
            f"preserve_lio_anchor={'on' if self.preserve_lio_anchor else 'off'}; "
            f"lio_pose_fallback={'on' if self.allow_lio_pose_fallback else 'off'}; "
            f"IMU preintegration={'on' if self.imu_factor_enabled else 'off'}; "
            f"math_core="
            f"{'cpp_eigen' if cpp_math_active else 'python_numpy'}; "
            f"live_propagation="
            f"{'on' if self.live_propagation_enabled else 'off'}")

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1.0e-9

    @staticmethod
    def _age_s(now_s, received_s):
        if received_s is None or received_s > now_s:
            return math.inf
        return now_s - received_s

    def _set_scan_prediction_contract_violation(
        self, reason, sequence, stamp_s
    ):
        if self.scan_prediction_contract_violated:
            return
        if self.scan_prediction_contract_first_failure_sequence < 0:
            self.scan_prediction_contract_first_failure_sequence = int(sequence)
            self.scan_prediction_contract_first_failure_stamp_s = float(stamp_s)
        self.scan_prediction_contract_violated = True
        self.scan_prediction_contract_reason = str(reason)
        self.counts["scan_prediction_contract_trips"] += 1
        logger = self.get_logger() if hasattr(self, "_logger") else None
        if logger is not None:
            logger.error(
                "Scan prediction contract violated: "
                f"reason={reason}; sequence={sequence}; stamp_s={stamp_s:.9g}; "
                "suppressing unified fusion output until a valid cache hit"
            )

    def _record_scan_prediction_contract_failure(
        self, reason, sequence, stamp_s
    ):
        if not self.frontend_scan_prediction_enabled:
            return
        with self.scan_prediction_contract_lock:
            if self.scan_prediction_contract_consecutive_failures == 0:
                self.scan_prediction_contract_first_failure_sequence = int(sequence)
                self.scan_prediction_contract_first_failure_stamp_s = float(stamp_s)
            self.scan_prediction_contract_consecutive_failures += 1
            if (
                self.scan_prediction_contract_consecutive_failures
                >= self.scan_prediction_contract_failure_threshold
            ):
                self._set_scan_prediction_contract_violation(
                    f"consecutive_{reason}", sequence, stamp_s
                )

    def _record_scan_prediction_contract_success(self):
        if not self.frontend_scan_prediction_enabled:
            return
        with self.scan_prediction_contract_lock:
            recovered = self.scan_prediction_contract_violated
            self.scan_prediction_contract_established = True
            self.scan_prediction_contract_violated = False
            self.scan_prediction_contract_consecutive_failures = 0
            self.scan_prediction_contract_reason = "ok"
            if recovered:
                self.counts["scan_prediction_contract_recoveries"] += 1
                logger = self.get_logger() if hasattr(self, "_logger") else None
                if logger is not None:
                    logger.warning(
                        "Scan prediction contract recovered after a valid cache hit"
                    )

    def _scan_prediction_contract_allows_output(self, now_s=None):
        if not getattr(self, "frontend_scan_prediction_enabled", False):
            return True
        if now_s is None:
            now_s = self._now_s()
        with self.scan_prediction_contract_lock:
            if (
                not self.scan_prediction_contract_violated
                and self.scan_prediction_contract_established
                and self._age_s(now_s, self.last_scan_request_arrival_s)
                > self.scan_prediction_contract_request_timeout_s
            ):
                self._set_scan_prediction_contract_violation(
                    "request_timeout",
                    getattr(self, "last_native_sequence", -1),
                    now_s,
                )
            return not self.scan_prediction_contract_violated

    def _effective_visual_time_offset_s(self):
        update = self.last_visual_time_calibration
        if (
            self.visual_time_calibration_enabled
            and self.visual_time_calibration_apply_locked
            and update.locked
        ):
            return float(update.time_offset_s)
        return float(self.visual_time_offset_s)

    def _update_visual_time_calibration(self, msg, previous, current):
        if not self.visual_time_calibration_enabled or not bool(msg.pnp_valid):
            return
        try:
            rotation = np.asarray(
                msg.pnp_rotation_previous_to_current, dtype=float
            ).reshape(3, 3)
        except (AttributeError, ValueError):
            self.counts["visual_time_calibration_rejected"] += 1
            return
        with self.visual_time_calibration_pending_lock:
            if (
                len(self.visual_time_calibration_pending)
                >= self.visual_time_calibration_pending_capacity
            ):
                self.visual_time_calibration_pending.popleft()
                self.counts["visual_time_calibration_pending_overflow"] += 1
                self.counts["visual_time_calibration_rejected"] += 1
            self.visual_time_calibration_pending.append((
                float(previous), float(current), rotation.copy(),
            ))
            self.counts["visual_time_calibration_pending_enqueued"] += 1
        self._drain_visual_time_calibration()

    def _apply_visual_time_calibration(
            self, previous, current, rotation, imu_samples):
        try:
            with self.visual_time_calibration_lock:
                update = self.visual_time_calibrator.update(
                    previous,
                    current,
                    rotation,
                    imu_samples,
                    self.visual_rotation_body_camera,
                )
                self.last_visual_time_calibration = update
            if update.accepted:
                self.visual_time_calibration_vote_history.append({
                    "previous_s": float(previous),
                    "current_s": float(current),
                    "candidate_offset_s": float(update.candidate_offset_s),
                    "locked_offset_s": float(update.time_offset_s),
                    "correlation": float(update.correlation),
                    "margin": float(update.margin),
                    "pair_count": int(update.pair_count),
                })
            self.counts["visual_time_calibration_updates"] += 1
            if update.accepted:
                self.counts["visual_time_calibration_accepted"] += 1
            else:
                self.counts["visual_time_calibration_rejected"] += 1
        except (AttributeError, ValueError, np.linalg.LinAlgError):
            self.counts["visual_time_calibration_rejected"] += 1

    def _drain_visual_time_calibration(self):
        if not self.visual_time_calibration_enabled:
            return
        with self.visual_time_calibration_drain_lock:
            imu_samples = self._imu_snapshot()
            while True:
                with self.visual_time_calibration_pending_lock:
                    if not self.visual_time_calibration_pending:
                        return
                    previous, current, rotation = (
                        self.visual_time_calibration_pending[0]
                    )
                coverage = visual_time_calibration_imu_coverage(
                    imu_samples,
                    previous,
                    current,
                    self.visual_time_calibrator.candidate_offsets_s,
                )
                if coverage == "wait_future":
                    return
                with self.visual_time_calibration_pending_lock:
                    pending = self.visual_time_calibration_pending.popleft()
                if coverage == "missing_history":
                    self.counts[
                        "visual_time_calibration_imu_history_missing"
                    ] += 1
                    self.counts["visual_time_calibration_rejected"] += 1
                    continue
                self._apply_visual_time_calibration(
                    pending[0], pending[1], pending[2], imu_samples
                )

    def _visual_pnp_metrics(self, message):
        selected = self._selected_visual_tracks(message)
        depth_eligible = self._visual_depth_eligible_tracks(message)
        inlier_ratio = len(selected) / max(1, len(depth_eligible))
        reprojection = np.asarray([
            float(track.reprojection_error_px) for track in selected
            if math.isfinite(float(track.reprojection_error_px))
            and float(track.reprojection_error_px) >= 0.0
        ])
        mean_reprojection = (
            float(np.mean(reprojection)) if reprojection.size else math.inf
        )
        rank, condition = 0, math.inf
        if selected and bool(message.pnp_valid):
            anchor = np.asarray([
                [track.previous_x, track.previous_y] for track in selected
            ])
            inverse_depth = np.asarray([
                track.inverse_depth for track in selected
            ])
            points3d = np.column_stack((
                anchor / inverse_depth[:, None], 1.0 / inverse_depth,
            ))
            try:
                rotation = np.asarray(
                    message.pnp_rotation_previous_to_current, dtype=float
                ).reshape(3, 3)
                translation = np.asarray(
                    message.pnp_translation_previous_to_current_m, dtype=float
                ).reshape(3)
                rank, condition = visual_pose_observability(
                    points3d, rotation, translation
                )
            except (AttributeError, ValueError):
                rank, condition = 0, math.inf
        return {
            "selected": selected,
            "inlier_ratio": inlier_ratio,
            "mean_reprojection_px": mean_reprojection,
            "rank": rank,
            "condition": condition,
        }

    def _visual_pnp_admissible(self, message, metrics):
        return bool(
            message.pnp_valid
            and len(metrics["selected"]) >= self.visual_minimum_tracks
            and metrics["inlier_ratio"] >= self.visual_pnp_minimum_inlier_ratio
            and metrics["rank"] >= self.visual_pnp_minimum_information_rank
            and math.isfinite(metrics["condition"])
            and metrics["condition"]
            <= self.visual_pnp_maximum_condition_number
            and metrics["mean_reprojection_px"]
            <= self.visual_pnp_maximum_mean_reprojection_error_px
        )

    def _rgbd_geometry_tracks(self, msg):
        current = stamp_seconds(msg.header.stamp)
        previous = stamp_seconds(msg.previous_stamp)
        if (
            not math.isfinite(current) or not math.isfinite(previous)
            or previous <= 0.0 or current <= previous
        ):
            return
        with self.rgbd_geometry_lock:
            if len(self.rgbd_geometry_tracks) == self.rgbd_geometry_tracks.maxlen:
                self.counts["rgbd_geometry_superseded"] += 1
            self.rgbd_geometry_tracks.append(copy.deepcopy(msg))
        self.counts["rgbd_geometry_received"] += 1

    def _matched_rgbd_geometry(self, visual_message):
        target_previous = stamp_seconds(visual_message.previous_stamp)
        target_current = stamp_seconds(visual_message.header.stamp)
        tolerance = self.rgbd_depth_factor_tolerance_s
        with self.rgbd_geometry_lock:
            self.rgbd_geometry_tracks = deque(
                (
                    message for message in self.rgbd_geometry_tracks
                    if stamp_seconds(message.header.stamp)
                    >= target_current - tolerance
                ),
                maxlen=2,
            )
            if not self.rgbd_geometry_tracks:
                return None
            errors = [
                max(
                    abs(stamp_seconds(message.previous_stamp) - target_previous),
                    abs(stamp_seconds(message.header.stamp) - target_current),
                )
                for message in self.rgbd_geometry_tracks
            ]
            index = int(np.argmin(errors))
            if errors[index] > tolerance:
                return None
            message = self.rgbd_geometry_tracks[index]
            del self.rgbd_geometry_tracks[index]
        self.counts["rgbd_geometry_matched"] += 1
        return message

    def _rgbd_direct_tracks(self, msg):
        current = stamp_seconds(msg.header.stamp)
        previous = stamp_seconds(msg.previous_stamp)
        if (
            not math.isfinite(current) or not math.isfinite(previous)
            or previous <= 0.0 or current <= previous
        ):
            return
        with self.rgbd_direct_lock:
            if len(self.rgbd_direct_tracks) == self.rgbd_direct_tracks.maxlen:
                self.counts["rgbd_direct_superseded"] += 1
            self.rgbd_direct_tracks.append(copy.deepcopy(msg))
        self.counts["rgbd_direct_received"] += 1

    def _matched_rgbd_direct(self, visual_message):
        target_previous = stamp_seconds(visual_message.previous_stamp)
        target_current = stamp_seconds(visual_message.header.stamp)
        tolerance = self.rgbd_depth_factor_tolerance_s
        with self.rgbd_direct_lock:
            self.rgbd_direct_tracks = deque(
                (
                    message for message in self.rgbd_direct_tracks
                    if stamp_seconds(message.header.stamp)
                    >= target_current - tolerance
                ),
                maxlen=2,
            )
            if not self.rgbd_direct_tracks:
                return None
            errors = [
                max(
                    abs(stamp_seconds(message.previous_stamp) - target_previous),
                    abs(stamp_seconds(message.header.stamp) - target_current),
                )
                for message in self.rgbd_direct_tracks
            ]
            index = int(np.argmin(errors))
            if errors[index] > tolerance:
                return None
            message = self.rgbd_direct_tracks[index]
            del self.rgbd_direct_tracks[index]
        self.counts["rgbd_direct_matched"] += 1
        return message

    def _add_rgbd_direct_factor(
            self, visual_message, previous_index, current_index, decision):
        self.counts["rgbd_direct_factor_attempts"] += 1
        self.last_rgbd_direct_photometric_information_scale = 1.0
        direct = self._matched_rgbd_direct(visual_message)
        if direct is None:
            self.last_rgbd_direct_reason = "matching_direct_tracks_missing"
            self.counts["rgbd_direct_missing"] += 1
            return False
        candidates = []
        for track in direct.tracks:
            values = (
                track.previous_x, track.previous_y,
                track.previous_depth_m, track.previous_depth_variance_m2,
                track.current_x, track.current_y,
                track.current_depth_m, track.current_depth_variance_m2,
                track.previous_intensity, track.current_intensity,
                track.current_gradient_x_normalized,
                track.current_gradient_y_normalized,
                track.photometric_variance,
            )
            if (
                track.track_age < 2
                or any(not math.isfinite(float(value)) for value in values)
                or track.previous_depth_m <= 0.0
                or track.current_depth_m <= 0.0
                or track.previous_depth_variance_m2 <= 0.0
                or track.current_depth_variance_m2 <= 0.0
                or track.photometric_variance <= 0.0
            ):
                continue
            candidates.append((
                float(track.previous_depth_variance_m2)
                + float(track.current_depth_variance_m2),
                int(track.grid_cell),
                track,
            ))
        candidates.sort(key=lambda item: item[0])
        per_cell = Counter()
        selected = []
        for _, grid_cell, track in candidates:
            if per_cell[grid_cell] >= 2:
                continue
            selected.append(track)
            per_cell[grid_cell] += 1
            if len(selected) >= self.rgbd_direct_factor_maximum_tracks:
                break
        self.last_rgbd_direct_track_count = len(selected)
        if len(selected) < self.rgbd_direct_factor_minimum_tracks:
            self.last_rgbd_direct_reason = (
                f"insufficient_tracks:{len(selected)}"
            )
            self.counts["rgbd_direct_rejected_tracks"] += 1
            return False
        batch_scale = visual_batch_information_scale(
            len(selected), self.visual_information_reference_tracks
        )
        depth_variance = np.asarray([
            track.previous_depth_variance_m2
            + track.current_depth_variance_m2
            for track in selected
        ])
        depth_variance *= (
            batch_scale / self.rgbd_direct_depth_information_scale
        )
        photometric_variance = np.asarray([
            track.photometric_variance for track in selected
        ])
        photometric_variance *= (
            batch_scale / self.rgbd_direct_photometric_information_scale
        )
        try:
            batch = RgbdDirectTrackBatch(
                np.asarray([
                    [track.previous_x, track.previous_y]
                    for track in selected
                ]),
                np.asarray([
                    [track.current_x, track.current_y]
                    for track in selected
                ]),
                np.asarray([track.previous_depth_m for track in selected]),
                np.asarray([track.current_depth_m for track in selected]),
                depth_variance,
                np.asarray([track.previous_intensity for track in selected]),
                np.asarray([track.current_intensity for track in selected]),
                np.asarray([[
                    track.current_gradient_x_normalized,
                    track.current_gradient_y_normalized,
                ] for track in selected]),
                photometric_variance,
                self.visual_rotation_body_camera,
                self.visual_translation_body_camera,
            )
            values = rgbd_direct_residual_jacobians(
                self.backend.state(previous_index),
                self.backend.state(current_index),
                batch,
            )
            valid_ratio = values[6].size / max(1, batch.track_count)
            depth_rmse = (
                float(np.sqrt(np.mean(values[0] ** 2)))
                if values[0].size else math.inf
            )
            photometric_rmse = (
                float(np.sqrt(np.mean(values[1] ** 2)))
                if values[1].size else math.inf
            )
            self.last_rgbd_direct_depth_rmse_m = depth_rmse
            self.last_rgbd_direct_photometric_rmse = photometric_rmse
            if (
                valid_ratio < self.visual_minimum_projectable_track_ratio
                or not math.isfinite(depth_rmse)
                or not math.isfinite(photometric_rmse)
                or depth_rmse
                > self.rgbd_direct_factor_maximum_depth_rmse_m
            ):
                self.last_rgbd_direct_reason = (
                    f"prefit_rejected:depth_rmse={depth_rmse:.6f}:"
                    f"photo_rmse={photometric_rmse:.6f}:"
                    f"valid_ratio={valid_ratio:.6f}"
                )
                self.counts["rgbd_direct_rejected_prefit"] += 1
                return False
            photo_limit = self.rgbd_direct_factor_maximum_photometric_rmse
            photo_information_scale = min(
                1.0,
                (photo_limit / max(photo_limit, photometric_rmse)) ** 2,
            )
            self.last_rgbd_direct_photometric_information_scale = (
                photo_information_scale
            )
            if photo_information_scale < 1.0:
                # Keep the metric depth rows from this D435 batch and reduce
                # only the inconsistent texture rows. The source still enters
                # the window exactly once as one combined RGB-D factor.
                batch = RgbdDirectTrackBatch(
                    batch.anchor_normalized,
                    batch.current_normalized,
                    batch.anchor_depth_m,
                    batch.current_depth_m,
                    batch.depth_variance_m2,
                    batch.previous_intensity,
                    batch.current_intensity,
                    batch.current_gradient_normalized,
                    batch.photometric_variance / max(
                        1.0e-4, photo_information_scale
                    ),
                    batch.rotation_body_camera,
                    batch.translation_body_camera,
                )
                self.counts[
                    "rgbd_direct_photometric_downweighted"
                ] += 1
            effective_weight = (
                float(decision.get("reliability_weight", 0.0))
                / max(
                    1.0,
                    float(decision.get("covariance_inflation", 1.0)),
                )
                if bool(decision.get("factor_enabled", False)) else 0.0
            )
            depth_jacobian = values[3][:, :6]
            depth_information = depth_jacobian.T @ (
                (
                    effective_weight
                    / batch.depth_variance_m2[values[6]]
                )[:, None]
                * depth_jacobian
            )
            axis_profile = pose_translation_profile_information(
                depth_information
            )
            maximum_profile = max(1.0e-12, float(np.max(axis_profile)))
            self.last_rgbd_depth_axis_profile_information = axis_profile
            self.last_rgbd_depth_axis_support = axis_profile / maximum_profile
            self.last_rgbd_depth_stamp_s = (
                stamp_seconds(visual_message.header.stamp)
                + (
                    self._effective_visual_time_offset_s()
                    if hasattr(self, "last_visual_time_calibration")
                    else float(getattr(self, "visual_time_offset_s", 0.0))
                )
            )
            self.backend.add_rgbd_direct(
                previous_index, current_index, batch, decision=decision
            )
        except (ValueError, FloatingPointError) as error:
            self.last_rgbd_direct_reason = f"invalid_batch:{error}"
            self.counts["rgbd_direct_rejected_tracks"] += 1
            return False
        self.last_rgbd_direct_reason = (
            "accepted"
            if self.last_rgbd_direct_photometric_information_scale >= 1.0
            else "accepted_photometric_downweighted"
        )
        self.counts["rgbd_direct_factors"] += 1
        return True

    def _add_rgbd_depth_factor(
            self, visual_message, previous_index, current_index, decision):
        if not self.rgbd_depth_factor_enabled:
            self.last_rgbd_depth_reason = "disabled"
            return False
        self.counts["rgbd_depth_factor_attempts"] += 1
        geometry = self._matched_rgbd_geometry(visual_message)
        if geometry is None:
            self.last_rgbd_depth_reason = "matching_geometry_missing"
            self.counts["rgbd_geometry_missing"] += 1
            return False
        self.rgbd_depth_candidate_sequence += 1
        lidar_vertical_strong = bool(
            getattr(self, "last_lidar_map_eligible", False)
            and getattr(
                self, "last_native_vertical_profile_information", 0.0
            ) >= self.rgbd_depth_healthy_lidar_profile_information
        )
        if (
            lidar_vertical_strong
            and (self.rgbd_depth_candidate_sequence - 1)
            % self.rgbd_depth_healthy_lidar_stride != 0
        ):
            self.last_rgbd_depth_reason = "skipped_healthy_lidar_z"
            self.counts["rgbd_depth_skipped_healthy_lidar"] += 1
            return False
        candidates = []
        for track in geometry.tracks:
            values = (
                track.previous_x,
                track.previous_y,
                track.previous_depth_m,
                track.previous_depth_variance_m2,
                track.current_depth_m,
                track.current_depth_variance_m2,
            )
            if (
                track.track_age < 2
                or any(not math.isfinite(float(value)) for value in values)
                or track.previous_depth_m <= 0.0
                or track.current_depth_m <= 0.0
                or track.previous_depth_variance_m2 <= 0.0
                or track.current_depth_variance_m2 <= 0.0
            ):
                continue
            variance = (
                float(track.previous_depth_variance_m2)
                + float(track.current_depth_variance_m2)
            )
            candidates.append((variance, int(track.grid_cell), track))
        candidates.sort(key=lambda item: item[0])
        per_cell = Counter()
        selected = []
        for _, grid_cell, track in candidates:
            if per_cell[grid_cell] >= 2:
                continue
            selected.append(track)
            per_cell[grid_cell] += 1
            if len(selected) >= self.rgbd_depth_factor_maximum_tracks:
                break
        self.last_rgbd_depth_track_count = len(selected)
        if len(selected) < self.rgbd_depth_factor_minimum_tracks:
            self.last_rgbd_depth_reason = (
                f"insufficient_tracks:{len(selected)}"
            )
            self.counts["rgbd_depth_rejected_tracks"] += 1
            return False
        anchor = np.asarray([
            [track.previous_x, track.previous_y] for track in selected
        ])
        anchor_depth = np.asarray([
            track.previous_depth_m for track in selected
        ])
        current_depth = np.asarray([
            track.current_depth_m for track in selected
        ])
        variance = np.asarray([
            track.previous_depth_variance_m2
            + track.current_depth_variance_m2
            for track in selected
        ])
        variance *= visual_batch_information_scale(
            len(selected), self.visual_information_reference_tracks
        ) / self.rgbd_depth_factor_information_scale
        try:
            batch = RgbdDepthTrackBatch(
                anchor,
                anchor_depth,
                current_depth,
                variance,
                self.visual_rotation_body_camera,
                self.visual_translation_body_camera,
            )
            residual, _, current_jacobian, valid = (
                rgbd_depth_residual_jacobians(
                    self.backend.state(previous_index),
                    self.backend.state(current_index),
                    batch,
                )
            )
            valid_ratio = residual.size / max(1, batch.track_count)
            rmse = (
                float(np.sqrt(np.mean(residual * residual)))
                if residual.size else math.inf
            )
            self.last_rgbd_depth_prefit_rmse_m = rmse
            if (
                valid_ratio < self.visual_minimum_projectable_track_ratio
                or not math.isfinite(rmse)
                or rmse > self.rgbd_depth_factor_maximum_rmse_m
            ):
                self.last_rgbd_depth_reason = (
                    f"prefit_rejected:rmse={rmse:.6f}:"
                    f"valid_ratio={valid_ratio:.6f}"
                )
                self.counts["rgbd_depth_rejected_prefit"] += 1
                return False
            effective_weight = (
                float(decision.get("reliability_weight", 0.0))
                / max(1.0, float(decision.get("covariance_inflation", 1.0)))
                if bool(decision.get("factor_enabled", False)) else 0.0
            )
            pose_information = current_jacobian[:, :6].T @ (
                (effective_weight / batch.variance_m2[valid])[:, None]
                * current_jacobian[:, :6]
            )
            axis_profile = pose_translation_profile_information(
                pose_information
            )
            maximum_profile = max(1.0e-12, float(np.max(axis_profile)))
            self.last_rgbd_depth_axis_profile_information = axis_profile
            self.last_rgbd_depth_axis_support = (
                axis_profile / maximum_profile
            )
            visual_time_offset_s = float(
                getattr(self, "visual_time_offset_s", 0.0)
            )
            if hasattr(self, "last_visual_time_calibration"):
                visual_time_offset_s = self._effective_visual_time_offset_s()
            self.last_rgbd_depth_stamp_s = (
                stamp_seconds(visual_message.header.stamp)
                + visual_time_offset_s
            )
            self.backend.add_rgbd_depth(
                previous_index, current_index, batch, decision=decision
            )
        except (ValueError, FloatingPointError) as error:
            self.last_rgbd_depth_reason = f"invalid_batch:{error}"
            self.counts["rgbd_depth_rejected_tracks"] += 1
            return False
        self.last_rgbd_depth_reason = "accepted"
        self.counts["rgbd_depth_factors"] += 1
        return True

    def _visual_tracks(self, msg):
        current = stamp_seconds(msg.header.stamp)
        previous = stamp_seconds(msg.previous_stamp)
        if (
            not math.isfinite(current) or not math.isfinite(previous)
            or previous <= 0.0 or current <= previous
        ):
            self.last_visual_reason = "invalid_track_timestamps"
            return
        pnp_metrics = self._visual_pnp_metrics(msg)
        if self._visual_pnp_admissible(msg, pnp_metrics):
            self._update_visual_time_calibration(msg, previous, current)
        else:
            self.counts["visual_time_calibration_geometry_rejected"] += 1
        self.counts["visual_received"] += 1
        key = (
            int(round(previous * 1.0e9)),
            int(round(current * 1.0e9)),
        )
        with self.visual_lock:
            self.visual_candidate_sequence += 1
            candidate = PendingVisualCandidate(
                self.visual_candidate_sequence,
                key,
                copy.deepcopy(msg),
                self._now_s(),
                time.monotonic(),
            )
            if self.visual_pending_enabled:
                # Camera data can arrive before the native LiDAR worker has
                # committed two real states. Keep feeding the shadow time
                # calibrator above, but never retain an observation that cannot
                # form a causal two-state factor.
                if len(self.visual_state_stamps) < 2:
                    self.counts["visual_prebootstrap_dropped"] += 1
                    self.visual_timing_reason_counts[
                        "prebootstrap_window_unavailable"
                    ] += 1
                    self.last_visual_reason = (
                        "prebootstrap_window_unavailable"
                    )
                    return
                if key in self.pending_visual_keys:
                    self.counts["visual_duplicate_candidates"] += 1
                    self.last_visual_reason = "duplicate_candidate"
                    return
                latest_cutoff = (
                    current - self.visual_pending_latest_horizon_s
                )
                while self.pending_visual_candidates:
                    oldest = self.pending_visual_candidates[0]
                    oldest_current = stamp_seconds(
                        oldest.message.header.stamp
                    )
                    if oldest_current >= latest_cutoff:
                        break
                    dropped = self.pending_visual_candidates.popleft()
                    self.pending_visual_keys.discard(dropped.key)
                    self.visual_factor_score_wait_started.pop(
                        dropped.key, None
                    )
                    self.counts["visual_pending_superseded"] += 1
                    self.counts["visual_rejected_time"] += 1
                    self.visual_timing_reason_counts[
                        "superseded_by_newer_candidate"
                    ] += 1
                    self._publish_visual_timing(
                        dropped,
                        "rejected",
                        "superseded_by_newer_candidate",
                        (),
                    )
                if len(self.pending_visual_candidates) >= self.visual_pending_max_queue:
                    dropped = self.pending_visual_candidates.popleft()
                    self.pending_visual_keys.discard(dropped.key)
                    self.visual_factor_score_wait_started.pop(dropped.key, None)
                    self.counts["visual_pending_overflow"] += 1
                    self.counts["visual_rejected_time"] += 1
                    self.visual_timing_reason_counts["queue_overflow"] += 1
                    self._publish_visual_timing(
                        dropped, "rejected", "queue_overflow", ()
                    )
                self.pending_visual_candidates.append(candidate)
                self.pending_visual_keys.add(key)
                self.counts["visual_pending_enqueued"] += 1
            else:
                self.visual_tracks.append(candidate)

    def _visual_factor_score(self, msg):
        self.counts["visual_factor_scores_received"] += 1
        source_stamp_s = stamp_seconds(msg.header.stamp)
        degradation_score = float(msg.degradation_score)
        reliability_weight = float(msg.reliability_weight)
        if (
            not math.isfinite(source_stamp_s) or source_stamp_s <= 0.0
            or not math.isfinite(degradation_score)
            or not math.isfinite(reliability_weight)
        ):
            self.counts["visual_factor_score_invalid"] += 1
            return
        degradation_score = max(0.0, min(1.0, degradation_score))
        reliability_weight = max(0.0, min(1.0, reliability_weight))
        valid = bool(msg.valid) and reliability_weight > 0.0
        with self.visual_lock:
            self.visual_factor_score_sequence += 1
            record = {
                "sequence": self.visual_factor_score_sequence,
                "source_stamp_s": source_stamp_s,
                "degradation_score": degradation_score,
                "weight": reliability_weight if valid else 0.0,
                "valid": valid,
                "reasons": tuple(msg.reasons),
                "arrival_ros_s": self._now_s(),
                "arrival_wall_s": time.monotonic(),
            }
            self.visual_factor_scores.append(record)
        self.last_visual_factor_score_stamp_s = source_stamp_s
        self.last_visual_factor_score_weight = float(record["weight"])
        self.last_visual_factor_score_degradation = float(
            record["degradation_score"]
        )
        self.last_visual_factor_score_reasons = tuple(record["reasons"])

    def _matched_visual_factor_score(self, source_stamp_s):
        with self.visual_lock:
            matched, retained, error_s = consume_timestamped_reliability_score(
                self.visual_factor_scores,
                source_stamp_s,
                self.visual_factor_score_tolerance_s,
            )
            if matched is not None:
                self.visual_factor_scores = deque(
                    retained, maxlen=self.visual_factor_score_history_size
                )
        self.last_visual_factor_score_match_error_s = (
            float(error_s) if math.isfinite(error_s) else -1.0
        )
        if matched is not None:
            self.last_visual_factor_score_reasons = tuple(
                matched.get("reasons", ())
            )
        return matched

    def _publish_visual_timing(
        self,
        candidate,
        outcome,
        reason,
        state_stamps,
        association=None,
    ):
        message = candidate.message
        now_ros_s = self._now_s()
        now_wall_s = time.monotonic()
        stamps = tuple(float(value) for value in state_stamps)
        previous_stamp = stamp_seconds(message.previous_stamp)
        current_stamp = stamp_seconds(message.header.stamp)
        lidar_intervals = np.diff(stamps) if len(stamps) >= 2 else np.asarray([])
        values = {
            "candidate_id": candidate.candidate_id,
            "outcome": outcome,
            "reason": reason,
            "visual_previous_stamp_s": previous_stamp,
            "visual_timestamp_s": current_stamp,
            "arrival_ros_s": candidate.arrival_ros_s,
            "arrival_wall_s": candidate.arrival_wall_s,
            "ros_sim_time_s": now_ros_s,
            "active_window_start_s": stamps[0] if stamps else -1.0,
            "active_window_end_s": stamps[-1] if stamps else -1.0,
            "visual_frontend_latency_s": max(
                0.0, candidate.arrival_ros_s - current_stamp
            ),
            "backend_queue_latency_s": max(
                0.0, now_ros_s - candidate.arrival_ros_s
            ),
            "backend_queue_wall_latency_s": max(
                0.0, now_wall_s - candidate.arrival_wall_s
            ),
            "keyframe_interval_s": current_stamp - previous_stamp,
            "lidar_state_interval_median_s": (
                float(np.median(lidar_intervals)) if lidar_intervals.size else -1.0
            ),
            "camera_imu_time_offset_s": self._effective_visual_time_offset_s(),
            "camera_imu_time_offset_locked": (
                self.last_visual_time_calibration.locked
            ),
            "pending_queue_size": len(self.pending_visual_candidates),
        }
        if association is not None:
            values.update({
                "corrected_previous_stamp_s": (
                    association.corrected_previous_stamp_s
                ),
                "corrected_visual_timestamp_s": (
                    association.corrected_current_stamp_s
                ),
                "nearest_previous_state_stamp_s": (
                    association.nearest_previous_stamp_s
                ),
                "nearest_state_timestamp_s": (
                    association.nearest_current_stamp_s
                ),
                "delta_previous_state_s": association.previous_delta_s,
                "delta_to_nearest_state_s": association.current_delta_s,
                "previous_state_index": association.previous_index,
                "current_state_index": association.current_index,
                "missing_side": association.missing_side,
            })
        status = DiagnosticStatus()
        status.name = "visual_time_association"
        status.hardware_id = "d435i_to_unified_window"
        status.level = (
            DiagnosticStatus.OK if outcome in {"associated", "accepted"}
            else DiagnosticStatus.WARN
        )
        status.message = str(reason)
        status.values = [
            KeyValue(key=str(name), value=str(value))
            for name, value in values.items()
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status.append(status)
        self.visual_timing_pub.publish(array)

    @staticmethod
    def _selected_visual_tracks(message):
        return [
            track for track in message.tracks
            if track.depth_valid and track.klt_inlier and track.geometric_inlier
            and track.track_age >= 2 and track.inverse_depth > 0.0
            and math.isfinite(track.inverse_depth)
            and math.isfinite(track.previous_x) and math.isfinite(track.previous_y)
            and math.isfinite(track.current_x) and math.isfinite(track.current_y)
        ]

    @staticmethod
    def _visual_depth_eligible_tracks(message):
        return [
            track for track in message.tracks
            if track.depth_valid and track.klt_inlier and track.track_age >= 2
            and track.inverse_depth > 0.0
            and math.isfinite(track.inverse_depth)
            and math.isfinite(track.previous_x) and math.isfinite(track.previous_y)
            and math.isfinite(track.current_x) and math.isfinite(track.current_y)
        ]

    def _add_visual_message_factor(
        self, message, previous_index, current_index, factor_score=None
    ):
        self.counts["visual_factor_attempts"] += 1
        if self.visual_factor_mode == "disabled":
            self.last_visual_reason = "disabled_by_parameter"
            return False
        if self.backend_solver_mode != "manifold":
            self.last_visual_reason = "paper_factor_requires_manifold_backend"
            return False
        decision = combine_visual_reliability_decisions(
            self._decision("vision", default_enabled=True), factor_score
        )
        self.last_visual_combined_reasons = tuple(decision.get("reasons", ()))
        if (
            not bool(decision.get("factor_enabled", False))
            or float(decision.get("reliability_weight", 0.0)) <= 0.0
        ):
            self.last_visual_reason = (
                "vision_factor_score_invalid"
                if factor_score is not None and not bool(factor_score.get("valid"))
                else "vision_frs_gate_disabled"
            )
            if factor_score is not None and not bool(factor_score.get("valid")):
                self.counts["visual_factor_score_invalid"] += 1
            self.counts["visual_quality_rejected_dv"] += 1
            return False
        if self.visual_factor_mode == "rgbd_direct":
            # Metric RGB-D performs its own depth and photometric prefit. PnP
            # remains diagnostic evidence, but it must not gate this distinct
            # observation model or initialize a factor that is never added.
            pnp_metrics = self._visual_pnp_metrics(message)
            self.last_visual_pnp_inlier_ratio = pnp_metrics["inlier_ratio"]
            self.last_visual_pnp_information_rank = pnp_metrics["rank"]
            self.last_visual_pnp_condition_number = pnp_metrics["condition"]
            self.last_visual_pnp_mean_reprojection_error_px = (
                pnp_metrics["mean_reprojection_px"]
            )
            self.last_visual_prefit_rmse_normalized = -1.0
            self.last_visual_prefit_rmse_px = -1.0
            self.last_visual_prefit_valid_track_ratio = -1.0
            self.last_visual_prefit_jacobian_rank = 0
            self.last_visual_prefit_jacobian_condition = -1.0
            self.last_visual_prefit_nis_per_dof = -1.0
            self.last_visual_prefit_information_trace = 0.0
            self.last_visual_prefit_information_max_eigenvalue = 0.0
            if not self._add_rgbd_direct_factor(
                message, previous_index, current_index, decision
            ):
                self.last_visual_reason = (
                    f"rgbd_direct:{self.last_rgbd_direct_reason}"
                )
                return False
            self.last_visual_reason = "accepted_rgbd_direct"
            self.counts["visual_factors"] += 1
            return True
        pnp_metrics = self._visual_pnp_metrics(message)
        selected = pnp_metrics["selected"]
        if len(selected) < self.visual_minimum_tracks:
            self.last_visual_reason = f"insufficient_geometric_tracks:{len(selected)}"
            self.counts["visual_rejected_tracks"] += 1
            return False
        pnp_inlier_ratio = pnp_metrics["inlier_ratio"]
        mean_reprojection = pnp_metrics["mean_reprojection_px"]
        pnp_rank = pnp_metrics["rank"]
        pnp_condition = pnp_metrics["condition"]
        anchor = np.asarray([[track.previous_x, track.previous_y]
                            for track in selected])
        current = np.asarray([[track.current_x, track.current_y]
                             for track in selected])
        inverse_depth = np.asarray([track.inverse_depth for track in selected])
        self.last_visual_pnp_inlier_ratio = pnp_inlier_ratio
        self.last_visual_pnp_information_rank = pnp_rank
        self.last_visual_pnp_condition_number = pnp_condition
        self.last_visual_pnp_mean_reprojection_error_px = mean_reprojection
        if not self._visual_pnp_admissible(message, pnp_metrics):
            self.last_visual_reason = (
                "visual_pnp_observability:"
                f"inlier_ratio={pnp_inlier_ratio:.6f}:"
                f"rank={pnp_rank}:condition={pnp_condition:.6g}:"
                f"mean_reprojection_px={mean_reprojection:.6f}"
            )
            self.counts["visual_pnp_observability_rejected"] += 1
            return False
        depth_variance = np.asarray([
            max(0.0, float(track.inverse_depth_variance)) for track in selected
        ])
        variance = (
            self.visual_pixel_sigma_normalized ** 2
            + self.visual_inverse_depth_variance_scale * depth_variance
        )
        information_scale = visual_batch_information_scale(
            len(selected), self.visual_information_reference_tracks
        )
        variance *= information_scale
        self.last_visual_batch_information_scale = information_scale
        self.counts["visual_batch_information_normalized"] += int(
            information_scale > 1.0
        )
        try:
            tracks = VisualTrackBatch(
                anchor, current, inverse_depth, variance,
                self.visual_rotation_body_camera,
                self.visual_translation_body_camera,
            )
            camera_matrix = np.asarray(
                message.camera_matrix, dtype=float
            ).reshape(3, 3)
            check = validate_visual_linearization(
                self.backend.state(previous_index),
                self.backend.state(current_index),
                tracks,
                camera_matrix[0, 0],
                camera_matrix[1, 1],
                maximum_reprojection_rmse_px=(
                    self.visual_state_innovation_maximum_rmse_px
                ),
                minimum_valid_track_ratio=(
                    self.visual_minimum_projectable_track_ratio
                ),
                minimum_jacobian_rank=self.visual_minimum_jacobian_rank,
                maximum_jacobian_condition_number=(
                    self.visual_maximum_jacobian_condition_number
                ),
            )
            self.last_visual_prefit_rmse_normalized = (
                check.reprojection_rmse_normalized
            )
            self.last_visual_prefit_rmse_px = check.reprojection_rmse_px
            self.last_visual_prefit_valid_track_ratio = check.valid_track_ratio
            self.last_visual_prefit_jacobian_rank = check.jacobian_rank
            self.last_visual_prefit_jacobian_condition = (
                check.jacobian_condition_number
            )
            self.last_visual_prefit_nis_per_dof = check.whitened_nis_per_dof
            self.last_visual_prefit_information_trace = check.information_trace
            self.last_visual_prefit_information_max_eigenvalue = (
                check.information_max_eigenvalue
            )
            if self.visual_state_consistency_enabled and not check.valid:
                self.last_visual_reason = (
                    f"visual_linearization:{check.reason}:"
                    f"rmse_px={check.reprojection_rmse_px:.6f}:"
                    f"valid_ratio={check.valid_track_ratio:.6f}:"
                    f"rank={check.jacobian_rank}:"
                    f"condition={check.jacobian_condition_number:.6g}:"
                    f"nis_per_dof={check.whitened_nis_per_dof:.6g}"
                )
                self.counts["visual_state_consistency_rejected"] += 1
                self.counts["visual_linearization_invalid"] += int(
                    check.reason != "state_innovation_reprojection_rmse"
                )
                return False
            if self.visual_initialization_enabled:
                was_ready = self.visual_initializer.ready
                initialization = self.visual_initializer.observe(
                    geometrically_valid=(
                        check.valid and bool(message.pnp_valid)
                    ),
                    time_locked=(
                        not self.visual_time_calibration_enabled
                        or self.last_visual_time_calibration.locked
                    ),
                )
                if initialization.ready and not was_ready:
                    self.counts["visual_initializations"] += 1
                if not initialization.ready:
                    self.last_visual_reason = initialization.reason
                    self.counts["visual_initialization_waits"] += 1
                    return False
            factor_representation = add_visual_observation_once(
                self.backend,
                previous_index,
                current_index,
                tracks,
                decision,
                lambda: self._add_rgbd_depth_factor(
                    message,
                    previous_index,
                    current_index,
                    decision,
                ),
            )
        except (ValueError, FloatingPointError) as error:
            self.last_visual_reason = f"invalid_track_batch:{error}"
            self.counts["visual_rejected_tracks"] += 1
            return False
        self.last_visual_reason = {
            "rgbd_depth": "accepted_rgbd_depth",
            "rgbd_direct": "accepted_rgbd_direct",
        }.get(factor_representation, "accepted_paper_reprojection")
        self.counts["visual_factors"] += 1
        return True

    def _legacy_visual_factor(
            self,
            previous_stamp,
            current_stamp,
            previous_index,
            current_index):
        self.counts["visual_factor_attempts"] += 1
        if self.visual_factor_mode == "disabled":
            self.last_visual_reason = "disabled_by_parameter"
            return False
        if self.backend_solver_mode != "manifold":
            self.last_visual_reason = "paper_factor_requires_manifold_backend"
            return False
        visual_time_offset_s = self._effective_visual_time_offset_s()
        # Convert backend state stamps back to the camera clock before looking
        # up a legacy queued message.  The forward association below uses the
        # inverse relation: t_imu = t_camera + td_C.
        corrected_previous = float(previous_stamp) - visual_time_offset_s
        corrected_current = float(current_stamp) - visual_time_offset_s
        with self.visual_lock:
            candidates = list(self.visual_tracks)
            while self.visual_tracks and (
                stamp_seconds(self.visual_tracks[0].message.header.stamp)
                < corrected_previous - self.visual_state_tolerance_s
            ):
                self.visual_tracks.popleft()
        if not candidates:
            self.last_visual_reason = "no_feature_tracks"
            self.counts["visual_rejected_time"] += 1
            return False

        def timing_error(candidate):
            message = candidate.message
            return max(
                abs(stamp_seconds(message.previous_stamp) - corrected_previous),
                abs(stamp_seconds(message.header.stamp) - corrected_current),
            )
        candidate = min(candidates, key=timing_error)
        message = candidate.message
        error_s = timing_error(candidate)
        if error_s > self.visual_state_tolerance_s:
            self.last_visual_reason = f"state_time_mismatch:{error_s:.6f}"
            self.counts["visual_rejected_time"] += 1
            association = associate_visual_states(
                stamp_seconds(message.previous_stamp),
                stamp_seconds(message.header.stamp),
                (previous_stamp, current_stamp),
                camera_to_imu_time_offset_s=visual_time_offset_s,
                tolerance_s=self.visual_state_tolerance_s,
            )
            self.visual_timing_reason_counts["legacy_state_time_mismatch"] += 1
            self._publish_visual_timing(
                candidate,
                "rejected",
                self.last_visual_reason,
                (previous_stamp, current_stamp),
                association,
            )
            return False
        selected = self._selected_visual_tracks(message)
        if len(selected) < self.visual_minimum_tracks:
            self.last_visual_reason = f"insufficient_geometric_tracks:{len(selected)}"
            self.counts["visual_rejected_tracks"] += 1
            return False
        factor_score = visual_factor_score_for_mode(
            self.reliability_mode,
            self._matched_visual_factor_score(
                stamp_seconds(message.header.stamp)
            ) if self.reliability_mode == "dynamic" else None,
        )
        if factor_score is None:
            self.last_visual_reason = "visual_factor_score_missing"
            self.counts["visual_factor_score_missing"] += 1
            self.counts["visual_quality_rejected_dv"] += 1
            return False
        self.counts["visual_factor_score_matched"] += 1
        # Avoid double-counting the legacy attempt inside the common helper.
        self.counts["visual_factor_attempts"] -= 1
        accepted = self._add_visual_message_factor(
            message, previous_index, current_index, factor_score=factor_score
        )
        association = associate_visual_states(
            stamp_seconds(message.previous_stamp),
            stamp_seconds(message.header.stamp),
            (previous_stamp, current_stamp),
            camera_to_imu_time_offset_s=visual_time_offset_s,
            tolerance_s=self.visual_state_tolerance_s,
        )
        self._publish_visual_timing(
            candidate,
            "accepted" if accepted else "rejected",
            self.last_visual_reason,
            (previous_stamp, current_stamp),
            association,
        )
        return accepted

    def _stage_pending_visual_factors(self, state_stamps):
        """Add all causally associable observations; return staged candidates."""
        staged = []
        consumed = set()
        latest_state = float(state_stamps[-1]) if state_stamps else -math.inf
        now_ros = self._now_s()
        now_wall = time.monotonic()
        with self.visual_lock:
            candidates = tuple(self.pending_visual_candidates)
        for candidate in candidates:
            message = candidate.message
            visual_time_offset_s = self._effective_visual_time_offset_s()
            corrected_current = (
                stamp_seconds(message.header.stamp) + visual_time_offset_s
            )
            association_started = time.perf_counter_ns()
            association = associate_visual_states(
                stamp_seconds(message.previous_stamp),
                stamp_seconds(message.header.stamp),
                state_stamps,
                camera_to_imu_time_offset_s=visual_time_offset_s,
                tolerance_s=self.visual_state_tolerance_s,
            )
            self._record_phase_timing(
                "visual_association", association_started
            )
            # A candidate whose left observation predates the active window can
            # never become associable by waiting for another right-hand state.
            # Remove it before applying generic wait/expiry accounting so a
            # startup backlog cannot masquerade as runtime transport latency.
            if (
                association.status == "reject"
                and association.reason == "outside_active_window"
                and association.missing_side == "left"
            ):
                consumed.add(candidate.key)
                self.counts["visual_pending_pre_window_dropped"] += 1
                self.counts["visual_rejected_time"] += 1
                reason = "pre_window_stale"
                self.visual_timing_reason_counts[reason] += 1
                self.last_visual_reason = reason
                self._publish_visual_timing(
                    candidate, "rejected", reason, state_stamps, association
                )
                continue
            sim_wait = max(0.0, latest_state - corrected_current)
            wall_wait = max(0.0, now_wall - candidate.arrival_wall_s)
            expired = (
                sim_wait > self.visual_pending_max_wait_s
                or wall_wait > self.visual_pending_max_wall_wait_s
            )
            if association.status == "wait" and not expired:
                self.counts["visual_pending_waits"] += 1
                self.last_visual_reason = association.reason
                continue
            if association.status != "associated" or expired:
                reason = (
                    "pending_wait_expired" if expired else association.reason
                )
                consumed.add(candidate.key)
                self.counts["visual_pending_expired"] += int(expired)
                self.counts["visual_rejected_time"] += 1
                self.visual_timing_reason_counts[reason] += 1
                self.last_visual_reason = reason
                self._publish_visual_timing(
                    candidate, "rejected", reason, state_stamps, association
                )
                continue
            factor_score = visual_factor_score_for_mode(
                self.reliability_mode,
                self._matched_visual_factor_score(
                    stamp_seconds(message.header.stamp)
                ) if self.reliability_mode == "dynamic" else None,
            )
            if factor_score is None:
                with self.visual_lock:
                    wait_started = self.visual_factor_score_wait_started.get(
                        candidate.key
                    )
                    started_waiting = wait_started is None
                    if started_waiting:
                        wait_started = (now_ros, now_wall)
                        self.visual_factor_score_wait_started[candidate.key] = (
                            wait_started
                        )
                wait_status, _, _ = visual_factor_score_wait_status(
                    wait_started[0],
                    wait_started[1],
                    now_ros,
                    now_wall,
                    self.visual_factor_score_max_wait_s,
                    self.visual_factor_score_max_wall_wait_s,
                )
                if wait_status == "wait" and not expired:
                    if started_waiting:
                        self.counts["visual_factor_score_waits"] += 1
                    self.last_visual_reason = "waiting_for_visual_factor_score"
                    continue
                consumed.add(candidate.key)
                self.counts["visual_factor_score_missing"] += 1
                self.counts["visual_quality_rejected_dv"] += 1
                self.last_visual_reason = "visual_factor_score_missing"
                self.visual_timing_reason_counts[
                    "visual_factor_score_missing"
                ] += 1
                self._publish_visual_timing(
                    candidate,
                    "rejected",
                    self.last_visual_reason,
                    state_stamps,
                    association,
                )
                continue
            consumed.add(candidate.key)
            with self.visual_lock:
                self.visual_factor_score_wait_started.pop(candidate.key, None)
            self.counts["visual_factor_score_matched"] += 1
            self.last_visual_factor_score_stamp_s = (
                visual_factor_score_source_stamp(
                    factor_score,
                    stamp_seconds(message.header.stamp),
                )
            )
            self.last_visual_factor_score_weight = float(
                factor_score["weight"]
            )
            self.last_visual_factor_score_degradation = float(
                factor_score["degradation_score"]
            )
            self.counts["visual_window_associated_candidates"] += 1
            construction_started = time.perf_counter_ns()
            accepted = self._add_visual_message_factor(
                message,
                association.previous_index,
                association.current_index,
                factor_score=factor_score,
            )
            self._record_phase_timing(
                "visual_factor_construction", construction_started
            )
            if accepted:
                staged.append(candidate)
            self._publish_visual_timing(
                candidate,
                "accepted" if accepted else "rejected",
                self.last_visual_reason,
                state_stamps,
                association,
            )
        if consumed:
            with self.visual_lock:
                self.pending_visual_candidates = deque(
                    candidate for candidate in self.pending_visual_candidates
                    if candidate.key not in consumed
                )
                self.pending_visual_keys.difference_update(consumed)
                for key in consumed:
                    self.visual_factor_score_wait_started.pop(key, None)
        return staged

    def _score(self, modality, msg):
        self.scores[modality] = {
            "degradation_score": float(msg.degradation_score),
            "weight": float(msg.reliability_weight) if msg.valid else 0.0,
            "valid": bool(msg.valid),
            "reasons": tuple(msg.reasons),
            "received_ros_s": stamp_seconds(msg.header.stamp),
        }

    def _scheduler(self, msg):
        lengths = (
            len(msg.modality_names), len(msg.reliability_weights),
            len(msg.covariance_inflation), len(msg.factor_enabled),
        )
        if min(lengths) != max(lengths):
            self.last_reason = "malformed_scheduler_state"
            return
        raw_reasons = tuple(getattr(msg, "reasons", ()))
        if raw_reasons and len(raw_reasons) != len(msg.modality_names):
            self.last_reason = "malformed_scheduler_reasons"
            return
        self.scheduler = {
            name: (float(weight), bool(enabled), float(inflation))
            for name, weight, enabled, inflation in zip(
                msg.modality_names, msg.reliability_weights,
                msg.factor_enabled, msg.covariance_inflation,
            )
        }
        self.scheduler_reasons = {
            name: tuple(
                reason for reason in (
                    raw_reasons[index].split(",")
                    if raw_reasons else ()
                )
                if reason
            )
            for index, name in enumerate(msg.modality_names)
        }
        self.scheduler_health = str(msg.health_state)
        self.scheduler_estimator_support = float(msg.estimator_support)
        self.scheduler_arrival = stamp_seconds(msg.header.stamp)

    def _lidar_calibration_motion(self, msg):
        self.counts["calibration_motion_received"] += 1
        try:
            motion = lidar_calibration_motion_from_message(msg)
            imu_samples = self._imu_snapshot()
            with self.calibration_lock:
                update = self.calibrator.update(motion, imu_samples)
                self.last_calibration_update = update
            self.last_calibration_motion_reason = update.reason
            if update.reason != "update_throttled":
                self.counts["calibration_updates"] += 1
            if update.accepted:
                self.counts["calibration_accepted"] += 1
            if update.reason == "sharp_turn_frozen":
                self.counts["calibration_frozen"] += 1
        except (ValueError, np.linalg.LinAlgError) as error:
            self.counts["calibration_motion_rejected"] += 1
            self.last_calibration_motion_reason = (
                f"rejected:{type(error).__name__}:{error}"
            )

    @staticmethod
    def _pose_msg_to_matrix(pose):
        orientation = quaternion_xyzw_to_rpy([
            float(pose.orientation.x), float(pose.orientation.y),
            float(pose.orientation.z), float(pose.orientation.w),
        ])
        transform = np.eye(4, dtype=float)
        transform[:3, :3] = rpy_to_rotation_matrix(orientation)
        transform[:3, 3] = [
            float(pose.position.x), float(pose.position.y),
            float(pose.position.z),
        ]
        if np.any(~np.isfinite(transform)):
            raise ValueError("relocalization pose is non-finite")
        return transform

    def _relocalization_result(self, msg):
        if int(
            msg.state) != int(
            RelocalizationResult.SUCCESS) or not bool(
                msg.accepted):
            if int(msg.state) == int(RelocalizationResult.FAILED):
                self.counts["relocalization_rejections"] += 1
            return
        if self.backend_solver_mode != "manifold":
            self.counts["relocalization_rejections"] += 1
            self.last_reason = "relocalization_requires_manifold_backend"
            return
        try:
            transaction_id = int(msg.transaction_id)
            if transaction_id <= 0:
                raise ValueError("relocalization transaction id is invalid")
            if transaction_id <= self.last_applied_relocalization_transaction_id:
                raise ValueError("relocalization transaction is stale")
            alignment = self._pose_msg_to_matrix(msg.map_from_lio)
            recovered_pose = self._pose_msg_to_matrix(msg.pose.pose)
            source_pose = self._pose_msg_to_matrix(msg.source_lio_pose)
            reconstructed = alignment @ source_pose
            position_error = float(np.linalg.norm(
                reconstructed[:3, 3] - recovered_pose[:3, 3]
            ))
            rotation_error = float(np.linalg.norm(
                matrix_to_pose_vector(
                    np.linalg.inv(reconstructed) @ recovered_pose
                )[3:6]
            ))
            if position_error > 0.05 or rotation_error > 0.05:
                raise ValueError(
                    "relocalization pose and map alignment disagree")
            stamp = stamp_seconds(msg.header.stamp)
            if stamp <= 0.0:
                raise ValueError("relocalization result timestamp is invalid")
            if self.backend.state_count <= 0 or self.last_lio_stamp is None:
                raise ValueError(
                    "relocalization requires an initialized state")
            result_age_s = float(self.last_lio_stamp) - stamp
            result_is_future = (
                result_age_s < -self.relocalization_state_tolerance_s
            )
            future_wait_timeout_s = max(
                self.relocalization_pending_timeout_s,
                float(getattr(
                    self, "relocalization_future_wait_timeout_s",
                    self.relocalization_pending_timeout_s,
                )),
            )
            if result_is_future and (
                -result_age_s > future_wait_timeout_s
            ):
                raise ValueError(
                    "relocalization result is too far ahead of backend state")
            if result_age_s > self.relocalization_result_max_age_s:
                raise ValueError("relocalization result is stale")
            with self.relocalization_lock:
                if self.pending_relocalization is not None:
                    if (
                        self.pending_relocalization_transaction_id
                        == transaction_id
                    ):
                        return
                    raise ValueError(
                        "another relocalization transaction is pending")
                self.pending_relocalization = (
                    stamp, alignment, recovered_pose, source_pose,
                    np.asarray(msg.pose.covariance, dtype=float),
                )
                self.pending_relocalization_candidate_id = int(
                    msg.candidate_id)
                self.pending_relocalization_transaction_id = transaction_id
                self.pending_relocalization_deadline_s = (
                    self._now_s() + (
                        future_wait_timeout_s
                        if result_is_future
                        else self.relocalization_pending_timeout_s
                    )
                )
            self.last_reason = (
                "relocalization_waiting_for_backend_state"
                if result_is_future
                else "relocalization_pending_window_reset"
            )
        except (ValueError, IndexError, TypeError) as error:
            self.counts["relocalization_rejections"] += 1
            self.last_reason = f"relocalization_invalid_result:{type(error).__name__}"
            self.last_exception = f"{type(error).__name__}:{error}"

    def _apply_pending_relocalization(self, stamp):
        with self.relocalization_lock:
            pending = self.pending_relocalization
            if pending is None or float(pending[0]) >= float(stamp):
                return False
            deadline_s = getattr(
                self, "pending_relocalization_deadline_s", None
            )
            if deadline_s is not None and self._now_s() > float(deadline_s):
                self.pending_relocalization = None
                self.pending_relocalization_candidate_id = 0
                self.pending_relocalization_transaction_id = 0
                self.pending_relocalization_deadline_s = None
                self.counts["relocalization_expired"] += 1
                self.last_reason = "relocalization_pending_expired"
                return False
            self.pending_relocalization = None
            candidate_id = int(getattr(
                self, "pending_relocalization_candidate_id", 0
            ))
            transaction_id = int(getattr(
                self, "pending_relocalization_transaction_id", 0
            ))
            self.pending_relocalization_candidate_id = 0
            self.pending_relocalization_transaction_id = 0
            self.pending_relocalization_deadline_s = None
        result_stamp, alignment, _, _, covariance = pending
        if self.backend.state_count <= 0 or self.last_lio_stamp is None:
            raise ValueError(
                "relocalization commit requires a current backend state")
        anchor_stamp = float(self.last_lio_stamp)
        current_state = np.asarray(self.backend.state(-1), dtype=float).copy()
        if current_state.shape != (15,) or np.any(~np.isfinite(current_state)):
            raise ValueError("relocalization commit state is invalid")
        previous_alignment = np.asarray(self.map_from_lio, dtype=float)
        if previous_alignment.shape != (
                4, 4) or np.any(
                ~np.isfinite(previous_alignment)):
            raise ValueError("current map alignment is invalid")
        epoch_correction = alignment @ np.linalg.inv(previous_alignment)
        corrected_pose = epoch_correction @ pose_vector_to_matrix(
            current_state[:6])
        recovered = current_state.copy()
        recovered[:6] = matrix_to_pose_vector(corrected_pose)
        recovered[6:9] = epoch_correction[:3, :3] @ current_state[6:9]
        prior_variance = np.ones(15, dtype=float) * 1.0
        if covariance.size >= 36 and np.all(np.isfinite(covariance[:36])):
            prior_variance[:3] = np.maximum(
                1.0e-4, [covariance[0], covariance[7], covariance[14]]
            )
            prior_variance[3:6] = np.maximum(
                1.0e-4, [covariance[21], covariance[28], covariance[35]]
            )
        backend_snapshot = self.backend.snapshot()
        with self.state_publication_lock:
            previous_anchor = self._optimization_anchor_snapshot()
            self._clear_optimization_anchor()
        try:
            self.backend.reset(recovered, covariance=prior_variance)
        except Exception:
            self.backend.restore(backend_snapshot)
            if previous_anchor is not None:
                with self.state_publication_lock:
                    with self.optimization_anchor_lock:
                        generation = self.optimization_anchor_generation + 1
                        self.optimization_anchor_generation = generation
                        self.optimization_anchor = make_optimization_anchor(
                            previous_anchor.stamp_s,
                            previous_anchor.state,
                            previous_anchor.covariance,
                            generation,
                            previous_anchor.reset_counter,
                        )
            with self.relocalization_lock:
                if self.pending_relocalization is None:
                    self.pending_relocalization = pending
                    self.pending_relocalization_candidate_id = candidate_id
                    self.pending_relocalization_transaction_id = transaction_id
                    self.pending_relocalization_deadline_s = deadline_s
            raise
        self.map_from_lio = alignment
        if self.lio_origin is not None:
            origin_homogeneous = np.concatenate((
                np.asarray(self.lio_origin, dtype=float), [1.0],
            ))
            self.lio_origin = (epoch_correction @ origin_homogeneous)[:3]
        self.last_lio_stamp = anchor_stamp
        self.last_lio_position = recovered[:3].copy()
        self.last_lio_yaw = float(recovered[5])
        self._reset_relocalization_epoch_buffers(anchor_stamp)
        self.last_state_covariance = np.diag(prior_variance)
        self.last_covariance_stamp_s = anchor_stamp
        self.last_covariance_source = "relocalization_prior"
        self.last_relocalization_reset_stats.update({
            "result_age_s": anchor_stamp - float(result_stamp),
            "correction_translation_m": float(np.linalg.norm(
                epoch_correction[:3, 3]
            )),
            "correction_rotation_rad": float(np.linalg.norm(
                matrix_to_pose_vector(epoch_correction)[3:6]
            )),
        })
        self.native_lidar_prediction_gate_latched = False
        self.counts["relocalization_resets"] += 1
        with self.state_publication_lock:
            self.state_reset_counter += 1
            self.last_applied_relocalization_transaction_id = transaction_id
            # Cross-topic consumers must see the epoch before any state
            # derived from the corrected anchor can become publishable.
            self._publish_fusion_epoch(stamp, candidate_id, transaction_id)
            self._commit_optimization_anchor(
                anchor_stamp, recovered, self.last_state_covariance
            )
        self.last_reason = "relocalization_window_reset_applied"
        return True

    def _publish_fusion_epoch(
        self,
        stamp_s,
        candidate_id,
        transaction_id,
        applied=True,
        reason="relocalization_window_reset_applied",
    ):
        publisher = getattr(self, "fusion_epoch_pub", None)
        if publisher is None:
            return
        message = FusionEpoch()
        message.header.stamp = ros_time_from_seconds(float(stamp_s))
        message.header.frame_id = self.map_frame
        message.applied = bool(applied)
        message.session_id = int(self.fusion_session_id)
        message.transaction_id = int(transaction_id)
        message.reset_counter = int(self.state_reset_counter)
        message.candidate_id = int(candidate_id)
        message.reason = str(reason)
        publisher.publish(message)

    def _reset_relocalization_epoch_buffers(self, result_stamp):
        """Invalidate measurements and predictions derived from the old epoch."""
        result_stamp = float(result_stamp)
        stats = {}

        with self.imu_buffer_lock:
            previous_count = len(self.imu_buffer)
            retained_imu = reanchor_imu_samples(
                self.imu_buffer, result_stamp
            )
            self.imu_buffer = deque(
                retained_imu, maxlen=self.imu_buffer.maxlen
            )
            stats["imu_discarded"] = previous_count - len(retained_imu)

        with self.gnss_lock:
            previous_count = len(self.gnss_buffer)
            retained_gnss = retain_stamped_records_after(
                self.gnss_buffer, result_stamp
            )
            self.gnss_buffer = deque(
                retained_gnss, maxlen=self.gnss_buffer.maxlen
            )
            self.latest_gnss = (
                max(
                    retained_gnss,
                    key=lambda item: float(item["stamp_s"]),
                )
                if retained_gnss else None
            )
            stats["gnss_discarded"] = previous_count - len(retained_gnss)

        with self.flow_buffer_lock:
            previous_count = len(self.flow_buffer)
            retained_flow = retain_stamped_records_after(
                self.flow_buffer, result_stamp
            )
            self.flow_buffer = deque(
                retained_flow, maxlen=self.flow_buffer.maxlen
            )
            stats["flow_discarded"] = previous_count - len(retained_flow)

        if hasattr(self, "visual_lock"):
            with self.visual_lock:
                stats["visual_discarded"] = (
                    len(self.visual_tracks) + len(self.pending_visual_candidates)
                )
                self.visual_tracks.clear()
                self.pending_visual_candidates.clear()
                self.pending_visual_keys.clear()
                self.visual_factor_scores.clear()
                self.visual_factor_score_wait_started.clear()
                self.visual_state_stamps = deque(
                    [result_stamp], maxlen=self.window_size
                )
            if self.visual_initialization_enabled:
                self.visual_initializer.reset("relocalization_reset")
        if hasattr(self, "rgbd_geometry_lock"):
            with self.rgbd_geometry_lock:
                stats["rgbd_geometry_discarded"] = len(
                    self.rgbd_geometry_tracks
                )
                self.rgbd_geometry_tracks.clear()
        if hasattr(self, "rgbd_direct_lock"):
            with self.rgbd_direct_lock:
                stats["rgbd_direct_discarded"] = len(
                    self.rgbd_direct_tracks
                )
                self.rgbd_direct_tracks.clear()

        stats["native_buffer_discarded"] = self.native_lidar_buffer.clear()
        stats["pending_lio_discarded"] = len(self.pending_lio)
        stats["pending_imu_lio_discarded"] = len(self.pending_imu_lio)
        self.pending_lio.clear()
        self.pending_imu_lio.clear()

        removed_work = drain_work_queue(self.native_work_queue)
        stats["native_worker_discarded"] = len(removed_work)
        if removed_work:
            self.counts["native_worker_queue_discarded"] += len(removed_work)
            discarded_sequences = []
            for item in removed_work:
                try:
                    discarded_sequences.append(int(item[1].scan_sequence))
                except (AttributeError, IndexError, TypeError, ValueError):
                    continue
            if discarded_sequences:
                self.last_native_consumed_sequence = max(
                    self.last_native_consumed_sequence,
                    max(discarded_sequences),
                )

        stats["scan_predictions_discarded"] = (
            len(self.scan_prediction_cache)
            + len(self.scan_prediction_by_sequence)
        )
        self.scan_prediction_cache.clear()
        self.scan_prediction_by_sequence.clear()
        with self.pending_scan_request_lock:
            stats["scan_requests_discarded"] = len(
                self.pending_scan_requests
            )
            self.pending_scan_requests.clear()
            self.pending_scan_request_first_seen_s.clear()

        self.path.poses = []
        self.last_path_sample_position = None
        self.last_path_sample_orientation = None
        self.last_path_publish_stamp_s = None
        self.last_output = None
        self.last_state_covariance = None
        self.last_covariance_stamp_s = None
        self.last_covariance_source = "relocalization_reset"
        self.last_gnss_prefit_nis = -1.0
        self.last_gnss_prefit_xy_nis = -1.0
        self.last_gnss_prefit_z_nis = -1.0
        self.last_gnss_xy_admitted = False
        self.last_gnss_z_admitted = False
        self.last_gnss_xy_information_scale = 0.0
        self.last_gnss_z_information_scale = 0.0
        if hasattr(self, "last_gnss_factor_covariance"):
            self.last_gnss_factor_covariance.fill(math.inf)
            self.gnss_z_reanchor_consecutive = 0
            self.last_gnss_z_reanchor_applied = False
            self.last_gnss_z_reanchor_target_m = math.nan
        self.last_gnss_prefit_residual_norm_m = -1.0
        if hasattr(self, "last_gnss_prefit_residual_xyz"):
            self.last_gnss_prefit_residual_xyz.fill(math.nan)
        self.last_gnss_prefit_stamp_s = -1.0
        self.last_gnss_degradation_score = 1.0
        self.last_gnss_reliability_weight = 0.0
        self.last_gnss_effective_information_scale = 0.0
        self.last_gnss_admission_reason = "relocalization_reset"
        if hasattr(self, "axis_handoff_latched"):
            self.axis_handoff_latched.fill(False)
            self.lidar_axis_observability_latched.fill(False)
            self.last_lidar_axis_information_scale.fill(1.0)
            self.last_axis_handoff_alternative_information.fill(0.0)
            self.last_axis_handoff_gnss_information.fill(0.0)
            self.last_axis_handoff_rgbd_information.fill(0.0)
            self.last_axis_handoff_barometer_information.fill(0.0)
            self.last_axis_map_protected.fill(False)
            self.last_axis_map_protection_sources = (
                "none", "none", "none"
            )
        if hasattr(self, "last_rgbd_depth_stamp_s"):
            self.last_rgbd_depth_stamp_s = -1.0
            self.last_rgbd_depth_axis_profile_information.fill(0.0)
            self.last_rgbd_depth_axis_support.fill(0.0)
        if hasattr(self, "barometer_segment"):
            with self.barometer_lock:
                self.barometer_segment.reset("relocalization_reset")
            self.last_barometer_reason = "relocalization_reset"
            self.last_barometer_segment_id = self.barometer_segment.segment_id
            self.last_barometer_prefit_residual_m = -1.0
            self.last_barometer_information_scale = 0.0
            self.last_barometer_variance_m2 = math.inf
            self.last_barometer_stamp_s = -1.0
            self.last_barometer_measurement_height_m = math.nan
            self.last_barometer_anchor_source = "none"
            self.last_barometer_anchor_reference_age_s = math.inf
            self.last_barometer_reference_reason = "relocalization_reset"
            self.last_barometer_reference_stamp_s = -1.0
            self.last_barometer_reference_z_m = math.nan
        if hasattr(self, "last_axis_reliability"):
            self.last_axis_reliability.fill(0.0)
            self.last_axis_degradation.fill(1.0)
            self.last_axis_global_reliability.fill(0.0)
            self.last_axis_supporting_sources = ((), (), ())
        self.last_scan_prediction_reason = "relocalization_reset"
        self.last_live_propagation_reason = "relocalization_reset"
        self.last_output_source = "none"
        self.active_transaction_snapshot = None
        self.last_frontend_map_pose_reason = "relocalization_reset"
        self.last_frontend_map_position_variance_m2 = math.inf
        self.last_frontend_map_orientation_variance_rad2 = math.inf
        self.last_frontend_map_pose_stamp_s = None
        self.last_frontend_map_pose_delay_s = -1.0
        self.last_lidar_map_eligible = False
        self.last_lidar_map_reason = "relocalization_reset"
        if hasattr(self, "frontend_map_eligibility_by_stamp"):
            self.frontend_map_eligibility_by_stamp.clear()
            self.frontend_map_eligibility_order.clear()
        self.last_relocalization_reset_stats = stats

    def _decision(self, modality, default_enabled=False):
        if self.reliability_mode == "fixed":
            weight = self.fixed_weights.get(modality, 1.0)
            decision = scheduler_decision(
                weight,
                default_enabled and weight > 0.0,
                self.fixed_covariance_inflation,
            )
            decision["reasons"] = ("fixed_reliability_mode",)
            return decision
        now = self._now_s()
        # LIO is the local estimator anchor. A missing/stale diagnostic must
        # not silently remove its pose factor and leave rotation unobservable.
        score_fresh = self._score_is_fresh(modality, now)
        if modality == "lidar" and not score_fresh:
            decision = scheduler_decision(1.0, default_enabled, 1.0)
            decision["reasons"] = ("lidar_anchor_without_fresh_score",)
            return decision
        if self._age_s(
                now,
                self.scheduler_arrival) <= self.scheduler_timeout_s:
            item = self.scheduler.get(modality)
            if item is not None:
                decision = scheduler_decision(item[0], item[1], item[2])
                decision["reasons"] = self.scheduler_reasons.get(modality, ())
                return self._protect_lidar_anchor(
                    modality, decision, now, score_fresh
                )
        item = self.scores.get(modality)
        if item is not None and self._age_s(
            now, item["received_ros_s"]
        ) <= self.scheduler_timeout_s:
            decision = scheduler_decision(item["weight"], item["valid"], 1.0)
            decision["reasons"] = item.get("reasons", ())
            return self._protect_lidar_anchor(
                modality, decision, now, score_fresh
            )
        decision = scheduler_decision(1.0, default_enabled, 1.0)
        decision["reasons"] = ("scheduler_unavailable_default",)
        return decision

    def _update_lidar_subspace_projector(self):
        """Update the raw-factor projector without touching marginal priors."""
        self.previous_lidar_subspace_scale = (
            self.last_lidar_subspace_scale.copy()
        )
        eigenvalues = np.asarray(
            self.last_native_translation_normalized_eigenvalues, dtype=float
        )
        eigenvectors = np.asarray(
            self.last_native_translation_eigenvectors, dtype=float
        )
        if (
            not self.lidar_subspace_enabled
            or eigenvalues.shape != (3,)
            or eigenvectors.shape != (3, 3)
            or np.any(~np.isfinite(eigenvalues))
            or np.any(~np.isfinite(eigenvectors))
            or float(np.max(eigenvalues)) <= 0.0
        ):
            self.lidar_subspace_episode_active = False
            self.last_lidar_subspace_weak_modes = 0
            self.last_lidar_subspace_information_scale = np.ones(3)
            self.last_lidar_subspace_scale = np.eye(3)
            return
        weak = eigenvalues < (
            self.lidar_subspace_exit_threshold
            if self.lidar_subspace_episode_active
            else self.lidar_subspace_weak_threshold
        )
        weak_modes = int(np.count_nonzero(weak))
        self.lidar_subspace_episode_active = weak_modes > 0
        if not self.lidar_subspace_episode_active:
            self.last_lidar_subspace_weak_modes = 0
            self.last_lidar_subspace_information_scale = np.ones(3)
            self.last_lidar_subspace_scale = np.eye(3)
            return
        mode_scale = np.where(weak, self.lidar_subspace_weak_scale, 1.0)
        self.last_lidar_subspace_weak_modes = weak_modes
        self.last_lidar_subspace_information_scale = mode_scale.copy()
        self.last_lidar_subspace_scale = (
            eigenvectors @ np.diag(mode_scale) @ eigenvectors.T
        )
        if self.backend_solver_mode == "manifold":
            self.backend.set_lidar_subspace_scale(self.last_lidar_subspace_scale)

    def _cap_lidar_subspace_with_current_gnss(self):
        if (
            not self.lidar_subspace_episode_active
            or self.backend_solver_mode != "manifold"
        ):
            self.last_lidar_subspace_absolute_information_ratio = np.ones(3)
            return
        information_diagnostic = getattr(
            self.backend, "active_lidar_solver_information", None
        )
        if not callable(information_diagnostic):
            return
        lidar_information, _ = information_diagnostic()
        absolute_information = np.diag(self.last_gnss_solver_information)
        (
            self.last_lidar_subspace_scale,
            self.last_lidar_subspace_absolute_information_ratio,
        ) = cap_weak_subspace_against_absolute_information(
            self.last_lidar_subspace_scale,
            self.previous_lidar_subspace_scale,
            lidar_information,
            absolute_information,
        )
        self.backend.set_lidar_subspace_scale(self.last_lidar_subspace_scale)

    def _axis_handoff_alternative_information(
        self, stamp_s, *, include_barometer=True
    ):
        """Return fresh per-LiDAR-frame axis information from other sensors."""
        gnss_information = np.zeros(3, dtype=float)
        rgbd_information = np.zeros(3, dtype=float)
        prefit_age = float(stamp_s) - self.last_gnss_prefit_stamp_s
        decision = self._decision("gnss", default_enabled=True)
        covariance = np.asarray(
            self.last_gnss_factor_covariance, dtype=float
        )
        if (
            0.0 <= prefit_age <= self.gnss_max_age_s
            and bool(decision.get("factor_enabled", False))
            and covariance.shape == (3,)
            and np.all(np.isfinite(covariance))
            and np.all(covariance > 0.0)
        ):
            effective_weight = (
                float(decision.get("reliability_weight", 0.0))
                / max(
                    1.0,
                    float(decision.get("covariance_inflation", 1.0)),
                )
            )
            axis_scale = np.asarray([
                self.last_gnss_xy_information_scale,
                self.last_gnss_xy_information_scale,
                self.last_gnss_z_information_scale,
            ])
            gnss_information = (
                self.axis_handoff_gnss_rate_ratio
                * effective_weight
                * axis_scale
                / covariance
            )
        rgbd_age = float(stamp_s) - self.last_rgbd_depth_stamp_s
        if 0.0 <= rgbd_age <= self.axis_handoff_rgbd_freshness_s:
            rgbd_information = (
                self.axis_handoff_rgbd_rate_ratio
                * self.last_rgbd_depth_axis_profile_information
            )
            rgbd_information = np.where(
                self.last_rgbd_depth_axis_support
                >= self.axis_handoff_rgbd_minimum_support,
                rgbd_information,
                0.0,
            )
        barometer_information = np.zeros(3, dtype=float)
        barometer_age = float(stamp_s) - self.last_barometer_stamp_s
        if (
            include_barometer
            and 0.0 <= barometer_age
            <= self.barometer_segment.maximum_sample_age_s
            and self.barometer_segment.active
            and self.last_barometer_information_scale > 0.0
            and math.isfinite(self.last_barometer_variance_m2)
            and self.last_barometer_variance_m2 > 0.0
        ):
            barometer_information[2] = (
                self.last_barometer_information_scale
                / self.last_barometer_variance_m2
            )
        self.last_axis_handoff_gnss_information = gnss_information
        self.last_axis_handoff_rgbd_information = rgbd_information
        self.last_axis_handoff_barometer_information = barometer_information
        self.last_axis_handoff_alternative_information = (
            gnss_information + rgbd_information + barometer_information
        )
        return self.last_axis_handoff_alternative_information.copy()

    @staticmethod
    def _bounded_reliability(value):
        value = float(value)
        return min(1.0, max(0.0, value)) if math.isfinite(value) else 0.0

    def _update_axis_reliability(self, stamp_s):
        """Publish an OR-style position support view without changing factors."""
        now_s = self._now_s()
        lidar_health = self._bounded_reliability(
            1.0 - self.last_native_health_degradation
        )
        lidar_consistency = self._bounded_reliability(
            1.0 - self.last_native_consistency_degradation
        )
        profiles = [AxisReliabilityProfile(
            "lidar",
            lidar_health,
            [lidar_consistency] * 3,
            self.last_native_isotropic_information_support,
            [False, False, False],
        )]

        gnss_age_s = float(stamp_s) - self.last_gnss_prefit_stamp_s
        gnss_fresh = bool(
            self.last_gnss_prefit_stamp_s > 0.0
            and -self.gnss_future_tolerance_s <= gnss_age_s
            <= self.gnss_max_age_s
        )
        profiles.append(AxisReliabilityProfile(
            "gnss",
            self._bounded_reliability(
                self.last_gnss_reliability_weight if gnss_fresh else 0.0
            ),
            [
                self.last_gnss_xy_information_scale,
                self.last_gnss_xy_information_scale,
                self.last_gnss_z_information_scale,
            ],
            [1.0, 1.0, 1.0],
            [True, True, True],
        ))

        rgbd_age_s = float(stamp_s) - self.last_rgbd_depth_stamp_s
        rgbd_fresh = bool(
            0.0 <= rgbd_age_s <= self.axis_handoff_rgbd_freshness_s
        )
        vision_decision = self._decision("vision", default_enabled=True)
        rgbd_health = (
            float(vision_decision.get("reliability_weight", 0.0))
            / max(
                1.0,
                float(vision_decision.get("covariance_inflation", 1.0)),
            )
            if rgbd_fresh and bool(
                vision_decision.get("factor_enabled", False)
            ) else 0.0
        )
        profiles.append(AxisReliabilityProfile(
            "rgbd",
            self._bounded_reliability(rgbd_health),
            [1.0, 1.0, 1.0],
            self.last_rgbd_depth_axis_support,
            [False, False, False],
        ))

        flow_decision = self._decision(
            "optical_flow", default_enabled=True
        )
        flow_fresh = self._score_is_fresh("optical_flow", now_s)
        flow_health = (
            float(flow_decision.get("reliability_weight", 0.0))
            / max(
                1.0,
                float(flow_decision.get("covariance_inflation", 1.0)),
            )
            if flow_fresh
            and bool(flow_decision.get("factor_enabled", False))
            and self.last_flow_reason == "accepted" else 0.0
        )
        profiles.append(AxisReliabilityProfile(
            "optical_flow",
            self._bounded_reliability(flow_health),
            [self.last_flow_rotation_weight] * 2 + [1.0],
            [1.0, 1.0, 0.0],
            [False, False, False],
        ))

        # IMU is not an absolute position reference, but it is the
        # indispensable continuous predictor on all three axes.  Keep its
        # axis support tied only to stream freshness and factor admission;
        # low excitation and preintegration NIS remain diagnostics and must
        # not remove the IMU from the propagation support view.
        imu_score_fresh, imu_factor_enabled = self._imu_backup_ready(now_s)
        imu_health = 1.0 if imu_score_fresh and imu_factor_enabled else 0.0
        profiles.append(AxisReliabilityProfile(
            "imu",
            imu_health,
            [1.0, 1.0, 1.0],
            [1.0, 1.0, 1.0],
            [False, False, False],
        ))

        with self.barometer_lock:
            barometer_active = bool(self.barometer_segment.active)
        profiles.append(AxisReliabilityProfile(
            "barometer",
            1.0 if barometer_active else 0.0,
            [1.0, 1.0, self.last_barometer_information_scale],
            [0.0, 0.0, 1.0],
            [False, False, False],
        ))
        summary = combine_axis_reliability(profiles)
        self.last_axis_reliability = np.asarray(
            summary.reliability_xyz, dtype=float
        )
        self.last_axis_degradation = np.asarray(
            summary.degradation_xyz, dtype=float
        )
        self.last_axis_global_reliability = np.asarray(
            summary.global_reliability_xyz, dtype=float
        )
        self.last_axis_supporting_sources = (
            summary.supporting_sources_xyz
        )

    def _score_is_fresh(self, modality, now):
        item = self.scores.get(modality)
        return bool(
            item is not None
            and item["valid"]
            and self._age_s(now, item["received_ros_s"])
            <= self.scheduler_timeout_s
        )

    def _imu_backup_ready(self, now):
        score_fresh = self._score_is_fresh("imu", now)
        scheduler_fresh = (
            self._age_s(now, self.scheduler_arrival)
            <= self.scheduler_timeout_s
        )
        if scheduler_fresh:
            item = self.scheduler.get("imu")
            factor_enabled = bool(
                item is not None and item[1] and float(item[0]) > 0.0
            )
        else:
            item = self.scores.get("imu")
            factor_enabled = bool(
                score_fresh and item is not None and item["weight"] > 0.0
            )
        return score_fresh, factor_enabled

    def _protect_lidar_anchor(self, modality, decision, now, score_fresh):
        if modality != "lidar":
            return decision
        if self.preserve_lio_anchor:
            return apply_lidar_anchor_floor(
                decision,
                self.lidar_anchor_minimum_effective_weight,
                self.lidar_anchor_maximum_covariance_inflation,
            )
        if decision["factor_enabled"]:
            return decision
        imu_score_fresh, imu_factor_enabled = self._imu_backup_ready(now)
        if lidar_bypass_allowed(
            self.preserve_lio_anchor, score_fresh,
            imu_score_fresh, imu_factor_enabled,
        ):
            return decision
        decision["factor_enabled"] = True
        decision["reliability_weight"] = max(
            0.05, decision["reliability_weight"])
        decision["anchor_override"] = True
        decision["covariance_inflation"] = max(
            1.0,
            min(MAX_COVARIANCE_INFLATION, decision["covariance_inflation"]),
        )
        return decision

    def _imu_snapshot(self):
        with self.imu_buffer_lock:
            return list(self.imu_buffer)

    def _imu_interval_snapshot(self, start_stamp, end_stamp):
        with self.imu_buffer_lock:
            return imu_samples_for_interval(
                self.imu_buffer, start_stamp, end_stamp
            )

    def _latest_lidar_frontend_activity_s(self):
        # LiDAR activity is an input fact. Unified output timestamps include
        # IMU propagation and must never feed back into this gate.
        activities = (getattr(self, "last_native_input_arrival_s", None),)
        valid = [
            float(value) for value in activities
            if value is not None and math.isfinite(float(value))
        ]
        return max(valid) if valid else None

    def _optimization_anchor_snapshot(self):
        with self.optimization_anchor_lock:
            return self.optimization_anchor

    def _commit_optimization_anchor(self, stamp_s, state, covariance):
        with self.optimization_anchor_lock:
            generation = self.optimization_anchor_generation + 1
            anchor = make_optimization_anchor(
                stamp_s,
                state,
                covariance,
                generation,
                self.state_reset_counter,
            )
            self.optimization_anchor_generation = generation
            self.optimization_anchor = anchor
        self.counts["optimized_states_committed"] += 1
        return anchor

    def _clear_optimization_anchor(self):
        with self.optimization_anchor_lock:
            self.optimization_anchor_generation += 1
            self.optimization_anchor = None

    def _last_unified_output_stamp_snapshot(self):
        with self.output_lock:
            return self.last_unified_output_stamp_s

    def _finalize_manifold_imu_measurement(self, measurement):
        covariance = inflate_manifold_imu_covariance(
            measurement.covariance,
            self.imu_covariance_scale,
            self.imu_bias_random_walk_variance,
        )
        return replace(
            measurement,
            covariance=tuple(float(value) for value in covariance.ravel()),
        )

    def _live_imu_measurement(self, anchor, target_stamp_s, samples):
        calibration_offset = effective_time_offset(
            self.last_calibration_update,
            self.online_calibration_enabled
            and self.calibration_apply_locked_time_offset,
            time_locked=getattr(
                getattr(self, "calibrator", None),
                "time_locked",
                getattr(self.last_calibration_update, "locked", False),
            ),
        )
        start_s = float(anchor.stamp_s) + calibration_offset
        end_s = float(target_stamp_s) + calibration_offset
        ordered = ordered_imu_samples(samples)
        stamps = [sample.stamp_s for sample in ordered]
        begin = max(0, bisect_left(stamps, start_s) - 1)
        end = min(len(ordered), bisect_right(stamps, end_s) + 1)
        measurement = preintegrate_manifold(
            ordered[begin:end],
            start_s,
            end_s,
            accel_bias=np.asarray(anchor.state[9:12], dtype=float),
            gyro_bias=np.asarray(anchor.state[12:15], dtype=float),
            max_gap_s=self.imu_max_gap_s,
        )
        if not measurement.valid:
            return None, measurement.reason
        return self._finalize_manifold_imu_measurement(measurement), "ok"

    def _reject_live_propagation(self, reason):
        self.last_live_propagation_reason = str(reason)
        self.counts["live_propagation_rejected"] += 1

    def _publish_live_propagation(self, _anchor_retry=True, _event_triggered=False):
        """Publish dead-reckoned odometry without touching the factor graph."""
        if (
            not self.live_propagation_enabled
            or self.backend_solver_mode != "manifold"
            or not self.imu_factor_enabled
            or self.native_worker_stop.is_set()
        ):
            return
        if not self._scan_prediction_contract_allows_output():
            self.counts["scan_prediction_contract_output_suppressed"] += 1
            self._reject_live_propagation("scan_prediction_contract_violation")
            return
        anchor = self._optimization_anchor_snapshot()
        if anchor is None:
            self.last_live_propagation_reason = "anchor_unavailable"
            return
        samples = self._imu_snapshot()
        if len(samples) < 2:
            self.last_live_propagation_reason = "imu_unavailable"
            return
        latest_imu_stamp_s = max(float(sample.stamp_s) for sample in samples)
        calibration_offset = effective_time_offset(
            self.last_calibration_update,
            self.online_calibration_enabled
            and self.calibration_apply_locked_time_offset,
            time_locked=getattr(
                getattr(self, "calibrator", None),
                "time_locked",
                getattr(self.last_calibration_update, "locked", False),
            ),
        )
        target_stamp_s = latest_imu_stamp_s - calibration_offset
        now_s = self._now_s()
        self.counts["live_propagation_attempts"] += 1
        admitted, reason = live_propagation_admission(
            now_s,
            latest_imu_stamp_s,
            target_stamp_s,
            anchor.stamp_s,
            self._last_unified_output_stamp_snapshot(),
            self._latest_lidar_frontend_activity_s(),
            self.live_propagation_lidar_silence_timeout_s,
            self.live_propagation_maximum_output_age_s,
            self.live_propagation_minimum_interval_s,
            self.live_propagation_maximum_imu_age_s,
            getattr(self, "unified_odom_output_mode", "legacy_hybrid")
            == "fixed_rate_propagated"
            or (
                getattr(self, "unified_odom_output_mode", "legacy_hybrid")
                == "lidar_event_propagated" and _event_triggered
            ),
        )
        if not admitted:
            self._reject_live_propagation(reason)
            return
        try:
            measurement, reason = self._live_imu_measurement(
                anchor, target_stamp_s, samples
            )
            if measurement is None:
                self._reject_live_propagation(f"imu_{reason}")
                return
            propagated = propagate_optimization_anchor(
                anchor, target_stamp_s, measurement
            )
        except (ValueError, IndexError, np.linalg.LinAlgError) as error:
            self.last_exception = f"live_{type(error).__name__}:{error}"
            self._reject_live_propagation(
                f"propagation_error:{type(error).__name__}"
            )
            return

        # A scan request or optimizer commit may arrive while preintegration is
        # running. Recheck both barriers before publishing the derived state.
        with self.state_publication_lock:
            current_anchor = self._optimization_anchor_snapshot()
            if (
                current_anchor is None
                or current_anchor.generation != propagated.anchor_generation
                or current_anchor.reset_counter
                != propagated.anchor_reset_counter
            ):
                self._reject_live_propagation("anchor_changed")
                if _anchor_retry:
                    # Retry once from the newest committed anchor.  This keeps
                    # the fixed-rate timer as the sole unified-odom writer.
                    try:
                        published_before_retry = self.counts["live_propagation_published"]
                        self._publish_live_propagation(
                            _anchor_retry=False,
                            _event_triggered=_event_triggered,
                        )
                        if self.counts["live_propagation_published"] == published_before_retry:
                            self.last_live_propagation_reason = "anchor_changed"
                    except Exception:
                        # A concurrently changing test/epoch may invalidate
                        # the retry; retain the causal rejection reason.
                        self.last_live_propagation_reason = "anchor_changed"
                return
            if current_anchor.reset_counter != self.state_reset_counter:
                self._reject_live_propagation("epoch_changed")
                return
            admitted, reason = live_propagation_admission(
                self._now_s(),
                latest_imu_stamp_s,
                target_stamp_s,
                current_anchor.stamp_s,
                self._last_unified_output_stamp_snapshot(),
                self._latest_lidar_frontend_activity_s(),
                self.live_propagation_lidar_silence_timeout_s,
                self.live_propagation_maximum_output_age_s,
                self.live_propagation_minimum_interval_s,
                self.live_propagation_maximum_imu_age_s,
                getattr(self, "unified_odom_output_mode", "legacy_hybrid")
                == "fixed_rate_propagated"
                or (
                    getattr(self, "unified_odom_output_mode", "legacy_hybrid")
                    == "lidar_event_propagated" and _event_triggered
                ),
            )
            if not admitted:
                self._reject_live_propagation(reason)
                return
            header = Odometry().header
            header.stamp = ros_time_from_seconds(propagated.stamp_s)
            header.frame_id = self.map_frame
            published = self._publish_live_odom(
                header,
                np.asarray(propagated.state, dtype=float),
                np.asarray(propagated.covariance, dtype=float).reshape(15, 15),
            )
        if published:
            self.last_live_propagation_reason = "ok"
        else:
            self._reject_live_propagation("nonmonotonic_output")

    def _imu(self, msg):
        stamp = stamp_seconds(msg.header.stamp)
        if stamp <= 0.0:
            self.counts["imu_invalid"] += 1
            self.imu_invalid_reasons["missing_timestamp"] = (
                self.imu_invalid_reasons.get("missing_timestamp", 0) + 1
            )
            return
        sample = ImuSample(
            stamp,
            (
                float(msg.linear_acceleration.x),
                float(msg.linear_acceleration.y),
                float(msg.linear_acceleration.z),
            ),
            (
                float(msg.angular_velocity.x),
                float(msg.angular_velocity.y),
                float(msg.angular_velocity.z),
            ),
        )
        with self.imu_buffer_lock:
            if self.last_imu_arrival_stamp is not None:
                arrival_delta = stamp - self.last_imu_arrival_stamp
                if arrival_delta <= 0.0:
                    self.counts["imu_nonmonotonic_arrivals"] += 1
                else:
                    self.imu_max_positive_arrival_gap_s = max(
                        self.imu_max_positive_arrival_gap_s, arrival_delta
                    )
            self.last_imu_arrival_stamp = stamp
            self.counts["imu_received"] += 1
            self.imu_buffer.append(sample)
            if self.last_lio_stamp is not None:
                cutoff = self.last_lio_stamp - self.imu_buffer_s
                prune_imu_buffer_before(self.imu_buffer, cutoff)
        self._drain_visual_time_calibration()

    def _flow_stamp(self, stamp):
        now_s = self._now_s()
        if stamp <= 0.0:
            self.counts["flow_clock_mismatch"] += 1
            self.last_flow_reason = "missing_sensor_timestamp"
            return None
        if (
            now_s > 0.0
            and abs(stamp - now_s) > self.maximum_sensor_clock_skew_s
        ):
            self.counts["flow_clock_mismatch"] += 1
            self.last_flow_reason = "sensor_clock_domain_mismatch"
            return None
        return stamp

    def _flow(self, msg):
        self.counts["flow_received"] += 1
        stamp = self._flow_stamp(stamp_seconds(msg.header.stamp))
        if stamp is None:
            return
        with self.flow_buffer_lock:
            self.flow_buffer.append({
                "stamp_s": stamp,
                "integrated_x": float(msg.integrated_x),
                "integrated_y": float(msg.integrated_y),
                "integrated_xgyro": float(msg.integrated_xgyro),
                "integrated_ygyro": float(msg.integrated_ygyro),
                "integrated_zgyro": float(msg.integrated_zgyro),
                "integration_time_s": float(msg.integration_time_us) * 1.0e-6,
                "quality": int(msg.quality),
                "distance_m": float(msg.distance),
            })

    def _gnss(self, msg):
        if msg.status.status < NavSatStatus.STATUS_FIX:
            return
        values = (
            float(
                msg.latitude), float(
                msg.longitude), float(
                msg.altitude))
        if not all(math.isfinite(value) for value in values):
            return
        if self.projector is None:
            self.projector = LocalEnuProjector(*values)
        covariance = gnss_covariance_diagonal(
            msg.position_covariance,
            msg.position_covariance_type,
            self.gnss_default_variance,
        ).tolist()
        position_enu = np.asarray(self.projector.project(*values), dtype=float)
        stamp_s = stamp_seconds(msg.header.stamp)
        if stamp_s <= 0.0:
            return
        temporal_jump = False
        if self.last_gnss_admitted is not None:
            temporal_jump = gnss_temporal_jump_rejected(
                self.last_gnss_admitted["position_enu"],
                self.last_gnss_admitted["stamp_s"],
                position_enu,
                stamp_s,
                self.gnss_jump_gate_m,
                self.gnss_jump_speed_mps,
            )
        if not temporal_jump:
            self.last_gnss_admitted = {
                "stamp_s": stamp_s,
                "position_enu": position_enu.copy(),
            }
        observation = {
            "stamp_s": stamp_s,
            "position_enu": position_enu,
            "covariance": covariance,
            "status": int(msg.status.status),
            "temporal_jump": temporal_jump,
        }
        with self.gnss_lock:
            self.counts["gnss_received"] += 1
            if any(
                abs(float(item["stamp_s"]) - stamp_s) <= 1.0e-9
                for item in self.gnss_buffer
            ):
                self.counts["gnss_duplicates"] += 1
                return
            if self.gnss_buffer and stamp_s < float(
                    self.gnss_buffer[-1]["stamp_s"]):
                self.counts["gnss_out_of_order"] += 1
            ordered = sorted(
                [*self.gnss_buffer, observation],
                key=lambda item: float(item["stamp_s"]),
            )
            self.gnss_buffer.clear()
            self.gnss_buffer.extend(ordered[-self.gnss_buffer.maxlen:])
            self.latest_gnss = observation

    def _barometer(self, msg):
        stamp_s = stamp_seconds(msg.header.stamp)
        pressure_pa = float(msg.fluid_pressure)
        pressure_variance_pa2 = float(msg.variance)
        with self.barometer_lock:
            accepted = self.barometer_segment.add_sample(
                stamp_s, pressure_pa, pressure_variance_pa2
            )
        if accepted:
            self.counts["barometer_received"] += 1
        else:
            self.counts["barometer_invalid"] += 1

    def _imu_factor(
        self, previous_stamp, current_stamp, previous_orientation,
        previous_index, current_index,
    ):
        samples = ordered_imu_samples(self._imu_snapshot())
        if not self.imu_factor_enabled or len(samples) < 2:
            self.last_imu_reason = (
                "disabled" if not self.imu_factor_enabled else "insufficient_samples")
            return None
        stamps = [sample.stamp_s for sample in samples]
        start = max(0, bisect_left(stamps, previous_stamp) - 1)
        end = min(len(samples), bisect_right(stamps, current_stamp) + 1)
        result = preintegrate(
            samples[start:end], previous_stamp, current_stamp,
            max_gap_s=self.imu_max_gap_s,
        )
        self.last_imu_reason = result.reason
        if not result.valid:
            self._record_imu_invalid(result.reason)
            return None
        map_rotation = rpy_to_rotation_matrix(previous_orientation)
        delta_position = map_rotation @ np.asarray(
            result.delta_position, dtype=float)
        delta_velocity = map_rotation @ np.asarray(
            result.delta_velocity, dtype=float)
        right_to_rpy = np.linalg.inv(
            right_perturbation_jacobian_rpy(previous_orientation)
        )
        delta_rotation = right_to_rpy @ _quat_to_rotvec(
            np.asarray(result.delta_quaternion)
        )
        position_accel_jacobian = map_rotation @ np.asarray(
            result.jacobian_delta_position_accel_bias).reshape(3, 3)
        position_gyro_jacobian = map_rotation @ np.asarray(
            result.jacobian_delta_position_gyro_bias).reshape(3, 3)
        velocity_accel_jacobian = map_rotation @ np.asarray(
            result.jacobian_delta_velocity_accel_bias).reshape(3, 3)
        velocity_gyro_jacobian = map_rotation @ np.asarray(
            result.jacobian_delta_velocity_gyro_bias).reshape(3, 3)
        rotation_gyro_jacobian = right_to_rpy @ np.asarray(
            result.jacobian_delta_rotation_gyro_bias).reshape(3, 3)
        nominal_covariance = np.maximum(
            np.asarray(result.covariance, dtype=float), 1.0e-6
        )
        optimization_covariance = np.maximum(
            nominal_covariance * self.imu_covariance_scale, 1.0e-6
        )
        bias_random_walk_covariance = np.full(
            6, self.imu_bias_random_walk_variance
        )
        decision = self._decision("imu", default_enabled=True)
        self.backend.add_bias_aware_imu(
            previous_index, current_index, result.dt_s,
            delta_position, delta_velocity, delta_rotation,
            position_accel_jacobian.ravel(), position_gyro_jacobian.ravel(),
            velocity_accel_jacobian.ravel(), velocity_gyro_jacobian.ravel(),
            rotation_gyro_jacobian.ravel(),
            # The data-layer preintegrator already adds gravity.  Passing a
            # zero vector here avoids adding it twice; the eventual manifold
            # implementation will carry gravity outside the delta instead.
            gravity=(0.0, 0.0, 0.0),
            covariance=optimization_covariance,
            bias_random_walk_covariance=bias_random_walk_covariance,
            decision=decision,
        )
        self.counts["imu_factors"] += 1
        return np.concatenate(
            (nominal_covariance, bias_random_walk_covariance))

    def _manifold_imu_measurement(
        self, previous_stamp, current_stamp, previous_state,
        ordered_samples=None,
    ):
        samples = (
            ordered_imu_samples(self._imu_snapshot())
            if ordered_samples is None else ordered_samples
        )
        if not self.imu_factor_enabled or len(samples) < 2:
            self.last_imu_reason = (
                "disabled" if not self.imu_factor_enabled else "insufficient_samples")
            return None
        calibration_offset = effective_time_offset(
            self.last_calibration_update,
            self.online_calibration_enabled
            and self.calibration_apply_locked_time_offset,
            time_locked=getattr(
                getattr(self, "calibrator", None),
                "time_locked",
                getattr(self.last_calibration_update, "locked", False),
            ),
        )
        imu_previous_stamp = float(previous_stamp) + calibration_offset
        imu_current_stamp = float(current_stamp) + calibration_offset
        stamps = [sample.stamp_s for sample in samples]
        start = max(0, bisect_left(stamps, imu_previous_stamp) - 1)
        end = min(len(samples), bisect_right(stamps, imu_current_stamp) + 1)
        result = preintegrate_manifold(
            samples[start:end],
            imu_previous_stamp,
            imu_current_stamp,
            accel_bias=np.asarray(previous_state[9:12], dtype=float),
            gyro_bias=np.asarray(previous_state[12:15], dtype=float),
            max_gap_s=self.imu_max_gap_s,
        )
        self.last_imu_reason = result.reason
        if not result.valid:
            self._record_imu_invalid(result.reason)
            return None
        return self._finalize_manifold_imu_measurement(result)

    def _add_manifold_imu_factor(
            self,
            previous_index,
            current_index,
            measurement):
        if measurement is None:
            return None
        self.backend.add_imu_preintegrated(
            previous_index,
            current_index,
            measurement,
            decision=self._decision("imu", default_enabled=True),
        )
        self.counts["imu_factors"] += 1
        return np.asarray(measurement.covariance, dtype=float)

    def _record_phase_timing(self, name, started_ns):
        elapsed_ms = (time.perf_counter_ns() - started_ns) * 1.0e-6
        timing = self.phase_timing[name]
        timing["count"] += 1
        timing["total_ms"] += elapsed_ms
        timing["max_ms"] = max(timing["max_ms"], elapsed_ms)
        timing["last_ms"] = elapsed_ms
        if timing["samples"] is not None:
            timing["samples"].append(elapsed_ms)
        if self.current_cycle_phase is not None:
            self.current_cycle_phase[name] = (
                self.current_cycle_phase.get(name, 0.0) + elapsed_ms
            )
        return elapsed_ms

    def _phase_mean_ms(self, name):
        timing = self.phase_timing[name]
        return timing["total_ms"] / max(1, timing["count"])

    def _phase_profile_summary(self):
        if not self.performance_profiling_enabled:
            return {}
        summary = {}
        for name, timing in self.phase_timing.items():
            if not timing["samples"]:
                continue
            values = np.fromiter(timing["samples"], dtype=float)
            summary[name] = {
                "count": int(values.size),
                "p50_ms": float(np.percentile(values, 50)),
                "p90_ms": float(np.percentile(values, 90)),
                "p95_ms": float(np.percentile(values, 95)),
                "max_ms": float(np.max(values)),
            }
        return summary

    def _begin_performance_cycle(self, callback_started_ns, stamp_s):
        if not self.performance_profiling_enabled:
            return None
        begin_profile = getattr(self.backend, "begin_profile_cycle", None)
        if begin_profile is not None:
            begin_profile()
        self.current_cycle_phase = {
            "pre_state": float(self.phase_timing["pre_state"]["last_ms"])
        }
        return {
            "stamp_s": float(stamp_s),
            "callback_started_ns": int(callback_started_ns),
            "wall_started_s": time.monotonic(),
            "resource": process_resource_snapshot(),
            "gc": self.gc_profiler.snapshot(),
            "factor_counts": {
                "lidar": int(self.counts["native_lidar_factors"]),
                "imu": int(self.counts["imu_factors"]),
                "gnss": int(self.counts["gnss_factors"]),
                "barometer": int(self.counts["barometer_factors"]),
                "flow": int(self.counts["flow_factors"]),
                "visual": int(self.counts["visual_factors"]),
            },
        }

    @staticmethod
    def _factor_name_counts(records):
        counts = Counter(record.name for record in records if record.enabled)
        return {
            "lidar": int(
                counts["lidar_point_plane"]
                + counts["lidar_point_plane_condensed"]
            ),
            "imu": int(counts["imu_preintegrated"]),
            "gnss": int(counts["gnss"]),
            "barometer": int(counts["barometer_local_z"]),
            "flow": int(
                counts["optical_flow"] + counts["optical_flow_body"]
            ),
            "visual": int(counts["visual_reprojection"]),
            "prior": int(counts["prior"] + counts["marginal_prior"]),
            "total": int(sum(counts.values())),
        }

    def _finish_performance_cycle(
        self, context, staged_visual_candidates, state_committed
    ):
        if context is None:
            return
        callback_total_ms = (
            time.perf_counter_ns() - context["callback_started_ns"]
        ) * 1.0e-6
        finish_profile = getattr(self.backend, "finish_profile_cycle", None)
        backend_profile = finish_profile() if finish_profile is not None else {}
        resource_after = process_resource_snapshot()
        gc_after = self.gc_profiler.snapshot()
        processor, frequency_khz = current_processor_and_frequency_khz()
        try:
            rss_bytes = int(
                FilePath("/proc/self/statm").read_text(
                    encoding="ascii"
                ).split()[1]
            ) * int(os.sysconf("SC_PAGE_SIZE"))
        except (OSError, ValueError, IndexError):
            rss_bytes = -1
        factor_records = self.backend.factor_summary()
        active_factor_counts = self._factor_name_counts(factor_records)
        active_lidar_records = [
            record for record in factor_records
            if record.enabled
            and record.name
            in {"lidar_point_plane", "lidar_point_plane_condensed"}
        ]
        state_stamps = tuple(float(value) for value in self.visual_state_stamps)
        active_lidar_ages = []
        active_lidar_state_indices = []
        for record in active_lidar_records:
            for index in record.state_indices:
                active_lidar_state_indices.append(int(index))
                if 0 <= int(index) < len(state_stamps):
                    active_lidar_ages.append(
                        max(
                            0.0,
                            float(context["stamp_s"])
                            - state_stamps[int(index)],
                        )
                    )
        factor_counters = {
            "lidar": int(self.counts["native_lidar_factors"]),
            "imu": int(self.counts["imu_factors"]),
            "gnss": int(self.counts["gnss_factors"]),
            "barometer": int(self.counts["barometer_factors"]),
            "flow": int(self.counts["flow_factors"]),
            "visual": int(self.counts["visual_factors"]),
        }
        factor_counts = {
            name: value - context["factor_counts"][name]
            for name, value in factor_counters.items()
        }
        factor_counts["total"] = int(sum(factor_counts.values()))
        current_lidar_record = (
            active_lidar_records[-1]
            if state_committed
            and factor_counts.get("lidar", 0) > 0
            and active_lidar_records
            else None
        )
        lidar_effective_weight = (
            float(current_lidar_record.effective_weight)
            if current_lidar_record is not None
            and current_lidar_record.enabled
            else 0.0
        )
        axis_root_scale = np.diag(
            np.sqrt(
                np.clip(
                    self.last_lidar_axis_information_scale, 0.0, 1.0
                )
            )
        )
        effective_translation_information = (
            lidar_effective_weight
            * axis_root_scale
            @ self.last_native_translation_profile_information
            @ axis_root_scale
        )
        subspace_eigenvalues, subspace_eigenvectors = np.linalg.eigh(
            0.5 * (
                self.last_lidar_subspace_scale
                + self.last_lidar_subspace_scale.T
            )
        )
        subspace_root_scale = (
            subspace_eigenvectors
            @ np.diag(np.sqrt(np.clip(subspace_eigenvalues, 0.0, 1.0)))
            @ subspace_eigenvectors.T
        )
        effective_translation_information = (
            subspace_root_scale
            @ effective_translation_information
            @ subspace_root_scale
        )
        effective_eigenvalues, effective_eigenvectors = np.linalg.eigh(
            0.5
            * (
                effective_translation_information
                + effective_translation_information.T
            )
        )
        weakest_direction = self.last_native_weakest_translation_direction
        lidar_weak_information = directional_information(
            effective_translation_information, weakest_direction
        )
        gnss_information_matrix = np.diag(
            self.last_gnss_solver_information
            if factor_counts.get("gnss", 0) > 0
            else np.zeros(3, dtype=float)
        )
        gnss_weak_information = directional_information(
            gnss_information_matrix, weakest_direction
        )
        active_lidar_information = np.zeros((3, 3), dtype=float)
        active_lidar_information_count = 0
        active_lidar_information_diagnostic = getattr(
            self.backend, "active_lidar_solver_information", None
        )
        if callable(active_lidar_information_diagnostic):
            (
                active_lidar_information,
                active_lidar_information_count,
            ) = active_lidar_information_diagnostic()
        active_lidar_weak_information = directional_information(
            active_lidar_information, weakest_direction
        )
        prior_projection = getattr(
            self.backend, "marginal_prior_translation_diagnostic", None
        )
        marginal_prior_diagnostic = dict(
            prior_projection(self.last_lidar_subspace_scale)
            if callable(prior_projection)
            else {}
        )
        marginal_prior_diagnostic.update(dict(getattr(
            self.backend, "last_marginal_prior_diagnostic", {}
        )))
        optimized_state = None
        if state_committed and self.backend.state_count > 0:
            optimized_state = self.backend.state(
                self.backend.state_count - 1
            ).tolist()
        phases = dict(self.current_cycle_phase or {})
        phases["callback_total"] = float(callback_total_ms)
        trace = {
            "schema_version": 2,
            "stamp_s": context["stamp_s"],
            "transaction_index": int(self.counts["lio"]),
            "native_scan_sequence": int(self.last_native_sequence),
            "wall_started_s": context["wall_started_s"],
            "wall_finished_s": time.monotonic(),
            "nonlinear_iteration_budget": int(context.get(
                "nonlinear_iteration_budget",
                getattr(self.backend, "last_iteration_budget", 1),
            )),
            "phases_ms": phases,
            "solver_profile_ms": backend_profile,
            "solver_duration_ms": float(
                backend_profile.get(
                    "optimize_total", getattr(self.backend, "last_solve_ms", 0.0)
                )
            ),
            "window_state_count": int(self.backend.state_count),
            "factor_counts": active_factor_counts,
            "factor_counts_added": factor_counts,
            "active_factor_counts": active_factor_counts,
            "lidar_correspondence_count": int(self.last_native_matches),
            "lidar_prediction": {
                "position_innovation_m": float(
                    self.last_lidar_prediction_position_innovation_m
                ),
                "yaw_innovation_rad": float(
                    self.last_lidar_prediction_yaw_innovation_rad
                ),
                "gate_reason": str(
                    self.last_native_lidar_prediction_gate_reason
                ),
                "recovery_floor": bool(context.get(
                    "lidar_prediction_recovery_floor", False
                )),
                "hard_rejected": bool(context.get(
                    "lidar_prediction_gate_rejected", False
                )),
                "factor_weight": float(context.get(
                    "lidar_factor_weight", 0.0
                )),
                "covariance_inflation": float(context.get(
                    "lidar_factor_inflation", MAX_COVARIANCE_INFLATION
                )),
                "effective_weight": float(lidar_effective_weight),
                "solver_admitted": bool(
                    state_committed
                    and factor_counts.get("lidar", 0) > 0
                    and current_lidar_record is not None
                    and current_lidar_record.enabled
                ),
                "map_eligible": bool(context.get(
                    "lidar_map_eligible", False
                )),
                "map_reason": str(context.get(
                    "lidar_map_reason", "not_evaluated"
                )),
            },
            "lidar_observability": {
                "health_degradation": float(
                    self.last_native_health_degradation
                ),
                "consistency_degradation": float(
                    self.last_native_consistency_degradation
                ),
                "observability_degradation_xyz": (
                    self.last_native_observability_degradation.tolist()
                ),
                "combined_degradation_xyz": (
                    self.last_native_combined_degradation.tolist()
                ),
                "isotropic_information_support_xyz": (
                    self.last_native_isotropic_information_support.tolist()
                ),
                "axis_profile_information": (
                    self.last_native_axis_profile_information.tolist()
                ),
                "axis_relative_support": (
                    self.last_native_axis_relative_support.tolist()
                ),
                "handoff_information_scale_xyz": (
                    self.last_lidar_axis_information_scale.tolist()
                ),
                "handoff_latched_xyz": self.axis_handoff_latched.tolist(),
                "alternative_information_per_lidar_xyz": (
                    self.last_axis_handoff_alternative_information.tolist()
                ),
                "gnss_information_per_lidar_xyz": (
                    self.last_axis_handoff_gnss_information.tolist()
                ),
                "rgbd_information_per_lidar_xyz": (
                    self.last_axis_handoff_rgbd_information.tolist()
                ),
                "barometer_information_per_lidar_xyz": (
                    self.last_axis_handoff_barometer_information.tolist()
                ),
                "translation_profile_information": (
                    self.last_native_translation_profile_information.tolist()
                ),
                "translation_normalized_eigenvalues": (
                    self.last_native_translation_normalized_eigenvalues.tolist()
                ),
                "translation_eigenvectors_row_major": (
                    self.last_native_translation_eigenvectors.tolist()
                ),
                "effective_translation_information": (
                    effective_translation_information.tolist()
                ),
                "effective_translation_eigenvalues": (
                    effective_eigenvalues.tolist()
                ),
                "effective_translation_eigenvectors_row_major": (
                    effective_eigenvectors.tolist()
                ),
                "subspace_projector_row_major": (
                    self.last_lidar_subspace_scale.tolist()
                ),
                "subspace_mode_information_scale": (
                    self.last_lidar_subspace_information_scale.tolist()
                ),
                "subspace_absolute_information_ratio": (
                    self.last_lidar_subspace_absolute_information_ratio.tolist()
                ),
                "subspace_weak_mode_count": int(
                    self.last_lidar_subspace_weak_modes
                ),
                "subspace_episode_active": bool(
                    self.lidar_subspace_episode_active
                ),
                "weakest_translation_direction": (
                    self.last_native_weakest_translation_direction.tolist()
                ),
                "horizontal_plane_fraction": float(
                    self.last_native_horizontal_plane_fraction
                ),
            },
            "visual_candidates_staged": int(len(staged_visual_candidates)),
            "active_lidar_state_indices": active_lidar_state_indices,
            "active_lidar_ages_s": active_lidar_ages,
            "active_lidar_factor_count": int(len(active_lidar_records)),
            "marginalization_happened": bool(
                backend_profile.get("marginalization_happened", False)
            ),
            "marginal_prior_diagnostic": marginal_prior_diagnostic,
            "state_committed": bool(state_committed),
            "optimized_state": optimized_state,
            "last_reason": str(self.last_reason),
            "resource_delta": process_resource_delta(
                context["resource"], resource_after
            ),
            "rss_bytes": int(rss_bytes),
            "gc": {
                "collections": [
                    int(after) - int(before)
                    for before, after in zip(
                        context["gc"]["collections"], gc_after["collections"]
                    )
                ],
                "duration_ms": [
                    float(after) - float(before)
                    for before, after in zip(
                        context["gc"]["duration_ms"], gc_after["duration_ms"]
                    )
                ],
                "allocation_counts_after": list(gc_after["counts"]),
            },
            "processor": int(processor),
            "cpu_frequency_khz": frequency_khz,
            "load_average": list(os.getloadavg()),
            "optimization_errors": int(self.counts["optimization_errors"]),
            "integrity_rejects": int(
                sum(
                    count
                    for reason, count in
                    self.optimization_integrity_reason_counts.items()
                    if reason != "ok"
                )
            ),
            "rollbacks": int(self.counts["optimization_rollbacks"]),
            "integrity": {
                "reason": self.last_optimization_integrity.reason,
                "translation_correction_m": float(
                    self.last_optimization_integrity.translation_correction_m
                ),
                "rotation_correction_rad": float(
                    self.last_optimization_integrity.rotation_correction_rad
                ),
                "velocity_correction_mps": float(
                    self.last_optimization_integrity.velocity_correction_mps
                ),
                "accel_bias_correction_mps2": float(
                    self.last_optimization_integrity.accel_bias_correction_mps2
                ),
                "gyro_bias_correction_radps": float(
                    self.last_optimization_integrity.gyro_bias_correction_radps
                ),
            },
            "scan_prediction": {
                "cache_hits": int(self.counts["scan_prediction_cache_hits"]),
                "cache_misses": int(self.counts["scan_prediction_cache_misses"]),
                "reuse_rejected": int(
                    self.counts["scan_prediction_reuse_rejected"]
                ),
                "last_reason": self.last_scan_prediction_reason,
                "last_imu_reason": self.last_imu_reason,
            },
            "gnss_prefit": {
                "stamp_s": float(self.last_gnss_prefit_stamp_s),
                "xy_nis": float(self.last_gnss_prefit_xy_nis),
                "z_nis": float(self.last_gnss_prefit_z_nis),
                "xy_admitted": bool(self.last_gnss_xy_admitted),
                "z_admitted": bool(self.last_gnss_z_admitted),
                "xy_information_scale": float(
                    self.last_gnss_xy_information_scale
                ),
                "z_information_scale": float(
                    self.last_gnss_z_information_scale
                ),
                "residual_xyz_m": [
                    float(value) if np.isfinite(value) else None
                    for value in self.last_gnss_prefit_residual_xyz
                ],
                "solver_information_diagonal": (
                    np.diag(gnss_information_matrix).tolist()
                ),
                "weak_direction_information": float(gnss_weak_information),
                "lidar_weak_direction_information": float(
                    lidar_weak_information
                ),
                "active_window_lidar_factor_count": int(
                    active_lidar_information_count
                ),
                "active_window_lidar_solver_information": (
                    active_lidar_information.tolist()
                ),
                "active_window_lidar_solver_weak_direction_information": float(
                    active_lidar_weak_information
                ),
                "lidar_to_gnss_weak_information_ratio": (
                    float(lidar_weak_information / gnss_weak_information)
                    if gnss_weak_information > 0.0 else None
                ),
                "active_window_lidar_solver_to_gnss_weak_information_ratio": (
                    float(
                        active_lidar_weak_information
                        / gnss_weak_information
                    )
                    if gnss_weak_information > 0.0 else None
                ),
                "reason": str(self.last_gnss_admission_reason),
                "time_compensation_age_s": float(
                    self.last_gnss_time_compensation_age_s
                ),
                "time_compensation_delta_m": (
                    self.last_gnss_time_compensation_delta_m.tolist()
                ),
                "time_compensation_variance_m2": (
                    self.last_gnss_time_compensation_variance_m2.tolist()
                ),
                "time_compensation_reason": str(
                    self.last_gnss_time_compensation_reason
                ),
            },
            "barometer_fallback": {
                "active": bool(self.barometer_segment.active),
                "segment_id": int(self.last_barometer_segment_id),
                "reason": str(self.last_barometer_reason),
                "prefit_residual_m": float(
                    self.last_barometer_prefit_residual_m
                ),
                "information_scale": float(
                    self.last_barometer_information_scale
                ),
                "measurement_height_m": (
                    float(self.last_barometer_measurement_height_m)
                    if math.isfinite(self.last_barometer_measurement_height_m)
                    else None
                ),
            },
            "axis_reliability": {
                "reliability_xyz": self.last_axis_reliability.tolist(),
                "degradation_xyz": self.last_axis_degradation.tolist(),
                "global_reliability_xyz": (
                    self.last_axis_global_reliability.tolist()
                ),
                "supporting_sources_xyz": [
                    list(values)
                    for values in self.last_axis_supporting_sources
                ],
            },
        }
        self.performance_cycle_trace.append(trace)
        self.current_cycle_phase = None

    def _write_performance_trace(self):
        if not self.performance_trace_path or not self.performance_cycle_trace:
            return
        output = FilePath(self.performance_trace_path).expanduser()
        output.parent.mkdir(parents=True, exist_ok=True)
        temporary = output.with_name(output.name + ".tmp")
        with temporary.open("w", encoding="utf-8") as stream:
            for record in self.performance_cycle_trace:
                stream.write(json.dumps(record, sort_keys=True, allow_nan=False))
                stream.write("\n")
        os.replace(temporary, output)

    def _manifold_imu_bias_changed(self, previous_index, measurement):
        """Check whether the optimized start bias invalidates this delta."""
        if measurement is None or self.backend.state_count <= previous_index:
            return False
        optimized = self.backend.state(previous_index)
        accel_delta = np.linalg.norm(
            optimized[9:12]
            - np.asarray(measurement.accel_bias_linearization, dtype=float)
        )
        gyro_delta = np.linalg.norm(
            optimized[12:15]
            - np.asarray(measurement.gyro_bias_linearization, dtype=float)
        )
        return bool(
            accel_delta > self.imu_reintegration_accel_bias_threshold
            or gyro_delta > self.imu_reintegration_gyro_bias_threshold
        )

    def _gnss_prefit_prediction(self, stamp, manifold_measurement):
        """Propagate the last optimized covariance to the GNSS factor time."""
        if manifold_measurement is None:
            return None, "imu_preintegration_unavailable"
        anchor = self._optimization_anchor_snapshot()
        if anchor is None:
            return None, "optimization_anchor_unavailable"
        try:
            propagated = propagate_optimization_anchor(
                anchor, stamp, manifold_measurement
            )
            state = np.asarray(propagated.state, dtype=float)
            covariance = np.asarray(propagated.covariance, dtype=float).reshape(
                15, 15
            )
            if (
                state.shape != (15,)
                or np.any(~np.isfinite(state))
                or np.any(~np.isfinite(covariance))
            ):
                raise ValueError("propagated GNSS prediction is invalid")
            return (state[:3].copy(), covariance[:3, :3].copy()), "ok"
        except (ValueError, np.linalg.LinAlgError) as error:
            return None, f"{type(error).__name__}:{error}"

    def _gnss_factor(
        self, stamp, position, index, predicted_position_covariance=None,
        prediction_reason="unavailable", scheduler_factor_decision=None,
        factor_velocity=None,
    ):
        self.last_gnss_solver_information = np.zeros(3, dtype=float)
        if self.projector is None or self.lio_origin is None:
            return
        with self.gnss_lock:
            observation, stale_count, superseded_count = select_gnss_observation(
                self.gnss_buffer,
                stamp,
                self.gnss_max_age_s,
                self.gnss_future_tolerance_s,
            )
            self.counts["gnss_stale_discarded"] += stale_count
            self.counts["gnss_superseded"] += superseded_count
            if observation is not None:
                self.counts["gnss_consumed"] += 1
        if observation is None:
            return
        self.counts["gnss_factor_attempts"] += 1
        gnss_position = np.asarray(self.lio_origin) + np.asarray(
            observation["position_enu"], dtype=float)
        covariance = np.asarray(observation["covariance"], dtype=float)
        if bool(observation.get("temporal_jump", False)):
            self.counts["gnss_jump_rejected"] += 1
            self.last_gnss_admission_reason = "temporal_jump_gate"
            return
        if int(observation.get("status", -1)) < 0:
            self.counts["gnss_invalid_fix_rejected"] += 1
            self.last_gnss_admission_reason = "invalid_fix_gate"
            return
        if scheduler_factor_decision is None:
            scheduler_factor_decision = self._decision(
                "gnss", default_enabled=True
            )
        if not bool(scheduler_factor_decision.get("factor_enabled", False)):
            self.counts["gnss_disabled_scheduler"] += 1
            self.last_gnss_reliability_weight = float(
                scheduler_factor_decision.get("reliability_weight", 0.0)
            )
            self.last_gnss_effective_information_scale = 0.0
            self.last_gnss_admission_reason = "scheduler_disabled"
            return
        if predicted_position_covariance is None:
            self.counts["gnss_prefit_covariance_unavailable"] += 1
            self.last_gnss_admission_reason = (
                f"prefit_covariance_unavailable:{prediction_reason}"
            )
            return
        factor_prediction = np.asarray(position, dtype=float).copy()
        factor_prediction_covariance = np.asarray(
            predicted_position_covariance, dtype=float
        ).copy()
        prefit_position = factor_prediction.copy()
        prefit_position_covariance = factor_prediction_covariance.copy()
        solver_gnss_position = gnss_position.copy()
        solver_covariance = covariance.copy()
        temporal_delta = np.zeros(3, dtype=float)
        if factor_velocity is not None:
            velocity = np.asarray(factor_velocity, dtype=float)
            age_s = float(stamp) - float(observation["stamp_s"])
            try:
                if (
                    velocity.shape != (3,)
                    or np.any(~np.isfinite(velocity))
                    or not math.isfinite(age_s)
                    or age_s < -self.gnss_future_tolerance_s
                    or age_s > self.gnss_max_age_s
                ):
                    raise ValueError("invalid constant-velocity interval")
                temporal_delta = velocity * age_s
                observation_position = factor_prediction - temporal_delta
                (
                    solver_gnss_position,
                    solver_covariance,
                    temporal_delta,
                ) = time_compensate_gnss_observation(
                    gnss_position,
                    covariance,
                    observation_position,
                    factor_prediction_covariance,
                    factor_prediction,
                    factor_prediction_covariance,
                )
            except ValueError as error:
                self.last_gnss_time_compensation_reason = (
                    f"invalid:{error}"
                )
            else:
                prefit_position = np.asarray(
                    observation_position, dtype=float
                ).copy()
                prefit_position_covariance = np.asarray(
                    factor_prediction_covariance, dtype=float
                ).copy()
                self.last_gnss_time_compensation_age_s = max(
                    0.0, age_s
                )
                self.last_gnss_time_compensation_delta_m = (
                    temporal_delta.copy()
                )
                self.last_gnss_time_compensation_variance_m2 = (
                    solver_covariance - covariance
                )
                self.last_gnss_time_compensation_reason = "applied"
        try:
            innovation, innovation_covariance, prefit_nis = gnss_prefit_statistics(
                prefit_position,
                prefit_position_covariance,
                gnss_position,
                covariance,
            )
            prefit_xy_nis, prefit_z_nis = gnss_prefit_axis_nis(
                innovation, innovation_covariance
            )
            decision = apply_gnss_prefit_gate(
                scheduler_factor_decision,
                prefit_xy_nis,
                prefit_z_nis,
                self.gnss_xy_nis_gate,
                self.gnss_z_nis_gate,
                self.gnss_minimum_reliability_weight,
                self.gnss_minimum_axis_information_scale,
            )
        except (ValueError, np.linalg.LinAlgError) as error:
            self.counts["gnss_prefit_invalid"] += 1
            self.last_gnss_admission_reason = (
                f"prefit_invalid:{type(error).__name__}:{error}"
            )
            return

        self.counts["gnss_prefit_valid"] += 1
        self.last_gnss_prefit_nis = float(prefit_nis)
        self.last_gnss_prefit_xy_nis = float(prefit_xy_nis)
        self.last_gnss_prefit_z_nis = float(prefit_z_nis)
        self.last_gnss_xy_admitted = bool(decision["gnss_xy_admitted"])
        self.last_gnss_z_admitted = bool(decision["gnss_z_admitted"])
        self.last_gnss_xy_information_scale = float(
            decision["gnss_xy_information_scale"]
        )
        self.last_gnss_z_information_scale = float(
            decision["gnss_z_information_scale"]
        )
        self.last_gnss_prefit_residual_norm_m = float(
            np.linalg.norm(innovation)
        )
        self.last_gnss_prefit_residual_xyz = np.asarray(
            innovation, dtype=float
        ).copy()
        self.last_gnss_prefit_stamp_s = float(observation["stamp_s"])
        self.last_gnss_factor_covariance = covariance.copy()
        self.last_gnss_degradation_score = 1.0 - min(
            1.0,
            max(
                0.0,
                float(scheduler_factor_decision.get("reliability_weight", 0.0)),
            ),
        )
        self.last_gnss_reliability_weight = float(
            decision["reliability_weight"]
        )
        self.last_gnss_effective_information_scale = float(
            decision["reliability_weight"]
            / decision["covariance_inflation"]
        )
        self.last_gnss_admission_reason = str(decision["admission_reason"])
        if not decision["factor_enabled"]:
            if decision["admission_reason"] == "scheduler_disabled":
                self.counts["gnss_disabled_scheduler"] += 1
            elif decision["admission_reason"] == "reliability_below_minimum":
                self.counts["gnss_rejected_low_weight"] += 1
            return
        if decision["gnss_xy_admitted"]:
            self.counts["gnss_xy_admitted"] += 1
        else:
            self.counts["gnss_xy_rejected_nis"] += 1
            self.counts["gnss_xy_robust_downweighted"] += 1
        self.last_gnss_z_reanchor_applied = False
        if decision["gnss_z_admitted"]:
            self.counts["gnss_z_admitted"] += 1
            self.gnss_z_reanchor_consecutive = 0
        else:
            self.counts["gnss_z_rejected_nis"] += 1
            self.counts["gnss_z_robust_downweighted"] += 1
            self.counts["gnss_z_reanchor_attempts"] = (
                self.counts.get("gnss_z_reanchor_attempts", 0) + 1
            )
            lidar_z_weak = bool(
                np.asarray(
                    getattr(
                        self,
                        "last_native_isotropic_information_support",
                        np.ones(3),
                    ),
                    dtype=float,
                )[2]
                < float(getattr(self, "axis_handoff_enter_support", 0.35))
            )
            self.gnss_z_reanchor_consecutive = (
                int(getattr(self, "gnss_z_reanchor_consecutive", 0)) + 1
                if lidar_z_weak else 0
            )
            if (
                bool(getattr(self, "gnss_z_reanchor_enabled", False))
                and lidar_z_weak
                and self.gnss_z_reanchor_consecutive
                >= int(getattr(
                    self, "gnss_z_reanchor_minimum_consecutive", 2
                ))
            ):
                reanchor_target, _ = bounded_axis_reanchor_target(
                    factor_prediction[2],
                    solver_gnss_position[2],
                    float(getattr(
                        self, "gnss_z_reanchor_maximum_step_m", 0.15
                    )),
                )
                solver_gnss_position[2] = reanchor_target
                self.last_gnss_z_reanchor_applied = True
                self.last_gnss_z_reanchor_target_m = reanchor_target
                # The bounded target prevents one stale map/prior epoch from
                # causing a transaction rollback.  Once the raw GNSS stream
                # is healthy and LiDAR Z is weak, retain a finite recovery
                # amount of the same GNSS factor instead of its NIS-derived
                # downweight; the next samples continue the bounded motion.
                recovery_scale = max(
                    float(getattr(
                        self, "gnss_z_information_scale",
                        decision["gnss_z_information_scale"],
                    )),
                    float(getattr(
                        self, "gnss_z_recovery_information_scale", 0.50
                    )),
                )
                decision["gnss_z_information_scale"] = min(
                    1.0, recovery_scale
                )
                decision["gnss_z_bounded_recovery"] = True
                decision["admission_reason"] = (
                    "admitted_xy_with_z_bounded_recovery"
                )
                self.last_gnss_z_information_scale = float(
                    decision["gnss_z_information_scale"]
                )
                self.last_gnss_effective_information_scale = float(
                    decision["reliability_weight"]
                    / decision["covariance_inflation"]
                )
                self.counts["gnss_z_reanchor_factors"] = (
                    self.counts.get("gnss_z_reanchor_factors", 0) + 1
                )
                self.counts["gnss_z_recovery_factors"] = (
                    self.counts.get("gnss_z_recovery_factors", 0) + 1
                )
        if decision["gnss_recovery_floor"]:
            self.counts["gnss_all_axes_inconsistent"] += 1
            self.counts["gnss_prefit_recovery_floor"] += 1
        solver_covariance[:2] /= decision["gnss_xy_information_scale"]
        solver_covariance[2] /= decision["gnss_z_information_scale"]
        effective_weight = (
            float(decision["reliability_weight"])
            / float(decision["covariance_inflation"])
        )
        self.last_gnss_solver_information = (
            effective_weight / solver_covariance
        )
        self.backend.add_gnss(
            index,
            solver_gnss_position,
            covariance=solver_covariance,
            decision=decision)
        self.counts["gnss_factor_records"] += 1
        self.counts["gnss_factors"] += 1
        if "gnss_provisional_bootstrap" in decision.get("reasons", ()):
            self.counts["gnss_provisional_bootstrap_admitted"] += 1

    def _update_barometer_trusted_reference(self, stamp_s, position_z_m):
        """Cache pressure/Z only while at least one absolute-Z source is sound."""
        if not self.barometer_fallback_enabled:
            self.last_barometer_reference_reason = "disabled"
            return False
        gnss_age_s = float(stamp_s) - self.last_gnss_prefit_stamp_s
        gnss_observation_fresh = bool(
            0.0 <= gnss_age_s <= self.gnss_max_age_s
        )
        residual_z_m = float(self.last_gnss_prefit_residual_xyz[2])
        gnss_trusted = bool(
            self.last_gnss_z_admitted
            and 0.0 <= gnss_age_s <= self.gnss_max_age_s
            and math.isfinite(residual_z_m)
            and abs(residual_z_m)
            <= self.barometer_reference_maximum_gnss_residual_m
            and self.last_gnss_z_information_scale
            >= self.barometer_reference_minimum_gnss_information_scale
        )
        lidar_support_z = float(
            self.last_native_isotropic_information_support[2]
        )
        lidar_trusted = bool(
            lidar_support_z >= self.axis_handoff_exit_support
            and self.last_native_health_degradation <= 0.50
            and self.last_native_consistency_degradation <= 0.50
        )
        rgbd_information_z = float(
            self.last_axis_handoff_rgbd_information[2]
        )
        rgbd_trusted = bool(
            rgbd_information_z >= self.axis_handoff_rgbd_minimum_support
        )
        # A fresh GNSS disagreement is independent evidence that the local
        # map Z may already be drifting.  In that case it is unsafe to let
        # LiDAR/RGB-D overwrite the pre-fallback pressure datum.  Local-only
        # sources may establish the datum only when GNSS is genuinely absent,
        # as in an indoor segment.
        if gnss_observation_fresh:
            trusted_sources = ["gnss_z"] if gnss_trusted else []
        else:
            trusted_sources = [
                source for source, available in (
                    ("lidar_z", lidar_trusted),
                    ("rgbd_z", rgbd_trusted),
                ) if available
            ]
        if not trusted_sources:
            self.counts["barometer_reference_rejected"] += 1
            self.last_barometer_reference_reason = "z_reference_not_trusted"
            return False
        with self.barometer_lock:
            if self.barometer_segment.active:
                self.last_barometer_reference_reason = (
                    "active_segment_reference_frozen"
                )
                return False
            updated = self.barometer_segment.update_trusted_reference(
                stamp_s, position_z_m
            )
            reference_stamp_s = (
                self.barometer_segment.trusted_reference_stamp_s
            )
            reason = self.barometer_segment.last_reason
        self.last_barometer_reference_reason = str(reason)
        if not updated:
            self.counts["barometer_reference_rejected"] += 1
            return False
        self.counts["barometer_reference_updates"] += 1
        self.last_barometer_reference_reason = (
            "trusted_reference_updated:" + ",".join(trusted_sources)
        )
        self.last_barometer_reference_stamp_s = float(reference_stamp_s)
        self.last_barometer_reference_z_m = float(position_z_m)
        return True

    def _barometer_factor(self, stamp_s, position_z_m, index):
        """Add a Z-only pressure factor only while stronger Z sources are absent."""
        if not self.barometer_fallback_enabled:
            self.last_barometer_reason = "disabled"
            return False
        # Do not use a previous pressure factor to decide whether this new
        # segment is needed.  The current pressure sample becomes alternative
        # Z information only after this factor is admitted below.
        self._axis_handoff_alternative_information(
            stamp_s, include_barometer=False
        )
        alternative_z_information = float(
            self.last_axis_handoff_rgbd_information[2]
        )
        if self.last_gnss_z_admitted:
            alternative_z_information += float(
                self.last_axis_handoff_gnss_information[2]
            )
        support_threshold = (
            self.axis_handoff_exit_support
            if self.barometer_segment.active
            else self.axis_handoff_enter_support
        )
        lidar_z_weak = bool(
            self.last_native_isotropic_information_support[2]
            < support_threshold
        )
        fallback_required = barometer_activation_required(
            lidar_z_weak=lidar_z_weak,
            alternative_z_information=alternative_z_information,
            stamp_s=stamp_s,
            gnss_prefit_stamp_s=self.last_gnss_prefit_stamp_s,
            gnss_max_age_s=self.gnss_max_age_s,
            gnss_z_admitted=self.last_gnss_z_admitted,
            gnss_z_nis=self.last_gnss_prefit_z_nis,
            gnss_z_nis_gate=self.barometer_prefit_nis_gate,
            enabled=self.barometer_activate_on_gnss_z_inconsistency,
        )
        if not fallback_required:
            # Keep the datum causal even when the current transaction later
            # takes a different factor path. This is the last healthy state
            # before a local Z fallback is requested.
            self._update_barometer_trusted_reference(
                stamp_s, position_z_m
            )
        with self.barometer_lock:
            was_active = self.barometer_segment.active
            measurement = self.barometer_segment.measurement(
                stamp_s, position_z_m, fallback_required
            )
            is_active = self.barometer_segment.active
            reason = self.barometer_segment.last_reason
            segment_id = self.barometer_segment.segment_id
            anchor_source = self.barometer_segment.anchor_source
            anchor_reference_age_s = (
                self.barometer_segment.anchor_reference_age_s
            )
        if is_active and not was_active:
            self.counts["barometer_segments_started"] += 1
        elif was_active and not is_active:
            self.counts["barometer_segments_ended"] += 1
        self.last_barometer_reason = str(reason)
        self.last_barometer_segment_id = int(segment_id)
        self.last_barometer_anchor_source = str(anchor_source)
        self.last_barometer_anchor_reference_age_s = float(
            anchor_reference_age_s
        )
        if measurement is None:
            self.last_barometer_information_scale = 0.0
            self.last_barometer_variance_m2 = math.inf
            self.last_barometer_stamp_s = -1.0
            return False
        self.counts["barometer_factor_attempts"] += 1
        residual_m = float(position_z_m) - float(measurement.height_m)
        nis = residual_m * residual_m / float(measurement.variance_m2)
        information_scale = gnss_axis_information_scale(
            nis,
            self.barometer_prefit_nis_gate,
            self.barometer_minimum_information_scale,
        )
        solver_variance_m2 = (
            float(measurement.variance_m2) / information_scale
        )
        self.backend.add_barometer_local_z(
            index,
            measurement.height_m,
            solver_variance_m2,
            decision=scheduler_decision(1.0, True, 1.0),
        )
        self.counts["barometer_factors"] += 1
        self.last_barometer_prefit_residual_m = residual_m
        self.last_barometer_information_scale = information_scale
        self.last_barometer_variance_m2 = float(measurement.variance_m2)
        self.last_barometer_stamp_s = float(measurement.stamp_s)
        self.last_barometer_measurement_height_m = float(measurement.height_m)
        return True

    def _flow_los_diagnostic(self, records, previous_state,
                             previous_stamp, current_stamp,
                             angular_samples=None):
        """Evaluate APM LOS residuals without adding a new optimization factor."""
        if not self.flow_los_diagnostics_enabled:
            self.last_flow_los_diagnostic = None
            return None
        observation = flow_los_observation(records)
        if observation is None or previous_state is None:
            self.counts["flow_los_diagnostic_invalid"] += 1
            self.last_flow_los_diagnostic = None
            return None
        state = np.asarray(previous_state, dtype=float)
        if state.shape != (15,) or np.any(~np.isfinite(state)):
            self.counts["flow_los_diagnostic_invalid"] += 1
            self.last_flow_los_diagnostic = None
            return None
        if angular_samples is None:
            angular_samples = sorted([
                (sample.stamp_s, sample.angular_velocity)
                for sample in self._imu_snapshot()
            ])
        angular_velocity = interval_mean_vector(
            angular_samples,
            previous_stamp,
            current_stamp,
            self.flow_rotation_imu_max_gap_s,
        )
        if angular_velocity is None:
            self.counts["flow_los_diagnostic_invalid"] += 1
            self.last_flow_los_diagnostic = None
            return None
        rotation_body_to_map = rpy_to_rotation_matrix(state[3:6])
        velocity_body = rotation_body_to_map.T @ state[6:9]
        angular_velocity = tuple(
            float(angular_velocity[index]) - float(state[12 + index])
            for index in range(3)
        )
        prediction_without_lever = optical_flow_los_prediction_flu(
            velocity_body,
            angular_velocity,
            (0.0, 0.0, 0.0),
            observation["distance_m"],
        )
        prediction_with_lever = optical_flow_los_prediction_flu(
            velocity_body,
            angular_velocity,
            self.flow_sensor_offset_body_m,
            observation["distance_m"],
        )
        if prediction_without_lever is None or prediction_with_lever is None:
            self.counts["flow_los_diagnostic_invalid"] += 1
            self.last_flow_los_diagnostic = None
            return None
        measurement = observation["measurement_radps"]
        residual_without_lever = tuple(
            float(measurement[index]) - float(prediction_without_lever[index])
            for index in range(2)
        )
        residual = tuple(
            float(measurement[index]) - float(prediction_with_lever[index])
            for index in range(2)
        )
        lever_delta = tuple(
            float(prediction_with_lever[index])
            - float(prediction_without_lever[index])
            for index in range(2)
        )
        residual_norm = float(np.linalg.norm(residual))
        residual_without_lever_norm = float(
            np.linalg.norm(residual_without_lever)
        )
        lever_norm = float(np.linalg.norm(lever_delta))
        diagnostic = {
            "measurement_radps": tuple(float(value) for value in measurement),
            "prediction_without_lever_radps": tuple(
                float(value) for value in prediction_without_lever
            ),
            "prediction_with_lever_radps": tuple(
                float(value) for value in prediction_with_lever
            ),
            "residual_radps": residual,
            "residual_norm_radps": residual_norm,
            "residual_without_lever_radps": residual_without_lever,
            "residual_without_lever_norm_radps": residual_without_lever_norm,
            "lever_delta_radps": lever_delta,
            "lever_delta_norm_radps": lever_norm,
            "distance_m": float(observation["distance_m"]),
            "integration_s": float(observation["integration_s"]),
            "sample_count": int(observation["sample_count"]),
            "angular_velocity_body_flu": tuple(float(value) for value in angular_velocity),
            "velocity_body_flu": tuple(float(value) for value in velocity_body),
            "sensor_offset_body_m": tuple(
                float(value) for value in self.flow_sensor_offset_body_m
            ),
            "state_source": "previous_optimized_state",
        }
        self.last_flow_los_diagnostic = diagnostic
        self.counts["flow_los_diagnostic_samples"] += 1
        self.flow_los_residual_no_lever_norms.append(
            residual_without_lever_norm)
        self.flow_los_residual_norms.append(residual_norm)
        self.flow_los_lever_arm_norms.append(lever_norm)
        return diagnostic

    def _flow_lever_arm_correction(
            self,
            records,
            previous_stamp,
            current_stamp,
            previous_state,
            angular_samples=None):
        """Estimate sensor-point motion to remove from horizontal flow delta.

        Prefer one IMU integration for each optical-flow exposure.  This keeps
        a delayed or variable-rate flow packet from borrowing angular motion
        from the rest of the LiDAR keyframe.  The older keyframe-wide estimate
        remains a bounded fallback when an exposure interval is not covered by
        the FCU IMU buffer.
        """
        if not self.flow_lever_arm_compensation_enabled:
            self.last_flow_lever_arm_displacement = None
            return np.zeros(3, dtype=float), {
                "enabled": 0.0,
                "valid": 0.0,
                "reason": "disabled",
                "integration_s": 0.0,
                "source": "disabled",
            }
        observation = flow_los_observation(records)
        integration_s = (
            float(
                observation["integration_s"]) if observation is not None else float(
                current_stamp -
                previous_stamp))
        if not math.isfinite(integration_s) or integration_s <= 0.0:
            self.counts["flow_lever_arm_unavailable"] += 1
            self.last_flow_lever_arm_displacement = None
            return np.zeros(3, dtype=float), {
                "enabled": 1.0,
                "valid": 0.0,
                "reason": "invalid_integration",
                "integration_s": 0.0,
                "source": "none",
            }
        if angular_samples is None:
            angular_samples = sorted([
                (
                    sample.stamp_s,
                    tuple(float(value) for value in sample.angular_velocity),
                )
                for sample in self._imu_snapshot()
            ])
        bias = np.zeros(3, dtype=float)
        if previous_state is not None:
            state = np.asarray(previous_state, dtype=float)
            if state.shape == (15,) and np.all(np.isfinite(state)):
                bias = np.asarray(state[12:15], dtype=float)

        # Each MAVLink optical-flow timestamp is the end of its integration
        # interval.  Use the corresponding FCU IMU samples when possible.
        per_exposure = []
        for flow in records:
            try:
                end_s = float(flow["stamp_s"])
                duration_s = float(flow["integration_time_s"])
            except (KeyError, TypeError, ValueError):
                per_exposure = []
                break
            if (
                not math.isfinite(end_s) or not math.isfinite(duration_s)
                or duration_s <= 0.0
            ):
                per_exposure = []
                break
            angular_velocity = interval_mean_vector(
                angular_samples,
                end_s - duration_s,
                end_s,
                self.flow_rotation_imu_max_gap_s,
            )
            if angular_velocity is None:
                per_exposure = []
                break
            angular_velocity = tuple(
                float(angular_velocity[index]) - float(bias[index])
                for index in range(3)
            )
            correction = optical_flow_lever_arm_displacement_flu(
                angular_velocity,
                self.flow_sensor_offset_body_m,
                duration_s,
            )
            if correction is None:
                per_exposure = []
                break
            per_exposure.append(np.asarray(correction, dtype=float))

        source = "per_exposure_imu"
        if per_exposure and len(per_exposure) == len(records):
            correction = np.sum(per_exposure, axis=0)
            self.counts["flow_lever_arm_per_exposure"] += 1
        else:
            angular_velocity = interval_mean_vector(
                angular_samples,
                previous_stamp,
                current_stamp,
                self.flow_rotation_imu_max_gap_s,
            )
            if angular_velocity is None:
                self.counts["flow_lever_arm_unavailable"] += 1
                self.last_flow_lever_arm_displacement = None
                return np.zeros(3, dtype=float), {
                    "enabled": 1.0,
                    "valid": 0.0,
                    "reason": "imu_interval_unavailable",
                    "integration_s": integration_s,
                    "source": "none",
                }
            angular_velocity = tuple(
                float(angular_velocity[index]) - float(bias[index])
                for index in range(3)
            )
            correction = optical_flow_lever_arm_displacement_flu(
                angular_velocity,
                self.flow_sensor_offset_body_m,
                integration_s,
            )
            source = "keyframe_interval_fallback"
            self.counts["flow_lever_arm_interval_fallback"] += 1
        if correction is None:
            self.counts["flow_lever_arm_unavailable"] += 1
            self.last_flow_lever_arm_displacement = None
            return np.zeros(3, dtype=float), {
                "enabled": 1.0,
                "valid": 0.0,
                "reason": "invalid_gyro_or_mount",
                "integration_s": integration_s,
                "source": "none",
            }
        correction = np.asarray(correction, dtype=float)
        self.counts["flow_lever_arm_compensated"] += 1
        self.last_flow_lever_arm_displacement = tuple(
            float(value) for value in correction)
        self.flow_lever_arm_displacement_norms.append(
            float(np.linalg.norm(correction[:2]))
        )
        return correction, {
            "enabled": 1.0,
            "valid": 1.0,
            "reason": "compensated",
            "integration_s": integration_s,
            "source": source,
        }

    def _build_range_facet_observation(
            self, records, flow_observation, native_factor, current_state,
            current_stamp):
        """Build one conservative ray/plane observation from native matches."""
        if not self.range_facet_enabled or native_factor is None:
            return None, None
        normals = getattr(native_factor, "plane_normals", None)
        points = getattr(native_factor, "plane_points", None)
        if normals is None or points is None:
            return None, "missing_native_plane_support"
        normals = np.asarray(normals, dtype=float).reshape(-1, 3)
        points = np.asarray(points, dtype=float).reshape(-1, 3)
        valid = np.all(np.isfinite(normals), axis=1) & np.all(
            np.isfinite(points), axis=1)
        normals = normals[valid]
        points = points[valid]
        if normals.shape[0] < self.range_facet_minimum_support_points:
            return None, "insufficient_native_plane_support"
        # FAST-LIO's native message carries plane_points in the map frame.  It
        # is tempting to apply T_body_sensor here because the lidar points are
        # sensor-frame samples, but the plane points are already the fitted map
        # facet used by the authoritative point-to-plane residual.  Keep this
        # experimental copy in that same frame and do not double-transform it.
        normal_norms = np.linalg.norm(normals, axis=1)
        valid_norms = normal_norms > 1.0e-9
        normals = normals[valid_norms] / normal_norms[valid_norms, None]
        points = points[valid_norms]
        if normals.shape[0] < self.range_facet_minimum_support_points:
            return None, "invalid_native_plane_normals"
        # One native scan contains several unrelated surfaces.  Build a small
        # deterministic normal/offset cluster instead of averaging all rows;
        # averaging would create a plane that does not exist in the scene.
        candidate_clusters = []
        normal_alignment = 0.97
        offset_tolerance = max(0.03, self.range_facet_maximum_plane_rmse_m)
        candidate_count = min(normals.shape[0], 32)
        candidate_indices = np.linspace(
            0, normals.shape[0] - 1, candidate_count, dtype=int)
        for candidate_normal in normals[candidate_indices]:
            oriented_signs = np.where(
                normals @ candidate_normal >= 0.0, 1.0, -1.0)
            oriented = normals * oriented_signs[:, None]
            normal_mask = oriented @ candidate_normal >= normal_alignment
            if int(np.count_nonzero(normal_mask)) < (
                self.range_facet_minimum_support_points
            ):
                continue
            normal = np.mean(oriented[normal_mask], axis=0)
            normal_norm = float(np.linalg.norm(normal))
            if normal_norm <= 1.0e-9:
                continue
            normal /= normal_norm
            candidate_points = points[normal_mask]
            signed_offsets = candidate_points @ normal
            # Select the densest offset neighborhood for this orientation.
            offset_count = min(signed_offsets.size, 32)
            offset_indices = np.linspace(
                0, signed_offsets.size - 1, offset_count, dtype=int)
            for center in signed_offsets[offset_indices]:
                offset_mask = np.abs(signed_offsets - center) <= offset_tolerance
                if int(np.count_nonzero(offset_mask)) < (
                    self.range_facet_minimum_support_points
                ):
                    continue
                support_candidate = candidate_points[offset_mask]
                offset_candidate = -float(
                    normal @ np.mean(support_candidate, axis=0))
                errors = support_candidate @ normal + offset_candidate
                rmse_candidate = float(np.sqrt(np.mean(errors * errors)))
                candidate_clusters.append((
                    support_candidate, normal, rmse_candidate,
                    offset_candidate,
                ))
        if not candidate_clusters:
            return None, "inconsistent_native_plane_normals"
        range_stamp = float(current_stamp)
        stamped = []
        for record in records:
            try:
                stamp = float(record["stamp_s"])
            except (KeyError, TypeError, ValueError):
                continue
            if math.isfinite(stamp):
                stamped.append(stamp)
        if stamped:
            range_stamp = min(stamped, key=lambda value: abs(value - current_stamp))
        sensor_rotation_body = np.asarray([
            [0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0],
            [-1.0, 0.0, 0.0],
        ], dtype=float)
        state = np.asarray(current_state, dtype=float)
        body_rotation = rpy_to_rotation_matrix(state[3:6])
        # A scan contains unrelated wall, floor, and ceiling facets.  Test
        # every compact candidate against the actual range ray first, then
        # choose the strongest admissible facet.
        evaluated = []
        rejection_reasons = []
        for support, normal, plane_rmse, offset in candidate_clusters:
            support_centroid = np.mean(support, axis=0)
            support_radius = float(np.max(
                np.linalg.norm(support - support_centroid, axis=1)
            ))
            normal_sigma = min(
                0.25,
                max(0.002, plane_rmse / max(support_radius, 0.10)),
            )
            plane_sigma = max(0.005, plane_rmse)
            plane_covariance = np.diag(np.array([
                normal_sigma * normal_sigma,
                normal_sigma * normal_sigma,
                normal_sigma * normal_sigma,
                plane_sigma * plane_sigma,
            ], dtype=float))
            observation = RangeFacetObservation(
                stamp_s=range_stamp,
                measured_range_m=float(flow_observation["distance_m"]),
                ray_direction_sensor=np.asarray([1.0, 0.0, 0.0], dtype=float),
                sensor_translation_body=np.asarray(
                    self.flow_sensor_offset_body_m, dtype=float),
                sensor_rotation_body=sensor_rotation_body,
                plane_normal_world=normal,
                plane_offset=offset,
                support_points_world=support,
                plane_rmse_m=plane_rmse,
                facet_stamp_s=float(native_factor.stamp_s),
                plane_covariance=plane_covariance,
            )
            result = evaluate_range_facet(
                observation,
                state[:3],
                body_rotation,
                range_sigma_m=float(self.last_flow_range_sigma_m),
                min_range_m=self.minimum_flow_distance_m,
                max_range_m=self.maximum_flow_distance_m,
                minimum_support_points=(
                    self.range_facet_minimum_support_points
                ),
                maximum_plane_rmse_m=self.range_facet_maximum_plane_rmse_m,
                denominator_epsilon=self.range_facet_denominator_epsilon,
                facet_margin_m=self.range_facet_facet_margin_m,
                timestamp_tolerance_s=self.range_facet_timestamp_tolerance_s,
                mahalanobis_gate=self.range_facet_mahalanobis_gate,
            )
            if result.accepted:
                evaluated.append((
                    support.shape[0], plane_rmse, observation, result,
                ))
            else:
                rejection_reasons.append(result.reason)
        if not evaluated:
            if rejection_reasons:
                for preferred in (
                    "nonpositive_intersection", "parallel_facet",
                    "outside_facet", "mahalanobis", "facet_timestamp",
                ):
                    if preferred in rejection_reasons:
                        return None, preferred
                return None, rejection_reasons[0]
            return None, "inconsistent_native_plane_normals"
        _, _, observation, result = max(
            evaluated, key=lambda item: (item[0], -item[1])
        )
        return observation, result

    def _flow_factor(self, previous_stamp, current_stamp, previous_yaw,
                     previous_index, current_index, lio_delta,
                     previous_state=None, ordered_imu=None,
                     native_factor=None, current_state=None):
        self.counts["flow_factor_attempts"] += 1
        with self.flow_buffer_lock:
            records, remaining, delayed = select_flow_records(
                self.flow_buffer,
                previous_stamp,
                current_stamp,
                self.flow_max_age_s,
            )
            self.flow_buffer = deque(remaining, maxlen=3000)
        if not records:
            self.last_flow_reason = "no_samples"
            return
        scheduler_factor_decision = self._decision(
            "optical_flow", default_enabled=True
        )
        if not bool(scheduler_factor_decision.get("factor_enabled", False)):
            self.counts["flow_disabled_scheduler"] += 1
            self.last_flow_reason = "scheduler_disabled"
            return
        observation = flow_observation_delta(records, previous_yaw)
        if observation is None:
            self.last_flow_reason = "no_valid_observation"
            return
        if ordered_imu is None:
            ordered_imu = ordered_imu_samples(self._imu_snapshot())
        flow_interval_start = float(previous_stamp)
        flow_interval_end = float(current_stamp)
        for record in records:
            try:
                record_end = float(record["stamp_s"])
                record_duration = float(record["integration_time_s"])
            except (KeyError, TypeError, ValueError):
                continue
            if math.isfinite(record_end):
                flow_interval_end = max(flow_interval_end, record_end)
            if math.isfinite(record_duration) and record_duration > 0.0:
                flow_interval_start = min(
                    flow_interval_start, record_end - record_duration
                )
        flow_imu = imu_samples_covering_interval(
            ordered_imu, flow_interval_start, flow_interval_end
        )
        angular_samples = [
            (
                sample.stamp_s,
                tuple(float(value) for value in sample.angular_velocity),
            )
            for sample in flow_imu
        ]
        flow_delta_body_sensor = np.asarray(
            observation["delta_body"], dtype=float)
        lever_correction, lever_evidence = self._flow_lever_arm_correction(
            records, previous_stamp, current_stamp, previous_state,
            angular_samples,
        )
        flow_delta_body = flow_delta_body_sensor - lever_correction
        # Keep the output planar: rotation-induced vertical motion is never
        # allowed to leak into the horizontal optical-flow factor.
        flow_delta_body[2] = 0.0
        flow_delta_position = np.asarray(
            frd_to_enu_delta(
                float(flow_delta_body[0]),
                -float(flow_delta_body[1]),
                previous_yaw,
            ),
            dtype=float,
        )
        flow_delta_position = np.asarray(
            [flow_delta_position[0], flow_delta_position[1], 0.0], dtype=float
        )
        flow_displacement = [float(value) for value in flow_delta_position]
        los_diagnostic = self._flow_los_diagnostic(
            records, previous_state, previous_stamp, current_stamp,
            angular_samples,
        )
        score, evidence, reasons = optical_flow_score(
            flow_displacement,
            [float(lio_delta[0]), float(lio_delta[1])],
            observation["quality"], observation["distance_m"],
        )
        decision = scheduler_factor_decision
        decision["degradation_score"] = float(score)
        decision["evidence"] = evidence
        decision["reasons"] = list(reasons)
        decision["evidence"].update({
            "flow_lever_arm_compensation_enabled": lever_evidence["enabled"],
            "flow_lever_arm_compensation_valid": lever_evidence["valid"],
            "flow_lever_arm_integration_s": lever_evidence["integration_s"],
            "flow_lever_arm_source": lever_evidence["source"],
            "flow_lever_arm_displacement_x_m": float(lever_correction[0]),
            "flow_lever_arm_displacement_y_m": float(lever_correction[1]),
            "flow_lever_arm_displacement_norm_m": float(
                np.linalg.norm(lever_correction[:2])
            ),
            "flow_delta_sensor_x_m": float(flow_delta_body_sensor[0]),
            "flow_delta_sensor_y_m": float(flow_delta_body_sensor[1]),
            "flow_delta_body_x_m": float(flow_delta_body[0]),
            "flow_delta_body_y_m": float(flow_delta_body[1]),
            "flow_delta_position_sensor_x_m": float(
                observation["delta_position"][0]
            ),
            "flow_delta_position_sensor_y_m": float(
                observation["delta_position"][1]
            ),
            "flow_delta_position_compensated_x_m": float(flow_displacement[0]),
            "flow_delta_position_compensated_y_m": float(flow_displacement[1]),
        })
        speed_ok, flow_speed_mps, flow_speed_limit_mps = mtf01p_flow_speed_gate(
            flow_delta_body_sensor[:2],
            observation["integration_s"],
            observation["distance_m"],
            self.flow_maximum_speed_at_1m_mps,
            self.flow_speed_gate_margin,
        )
        flow_covariance = optical_flow_displacement_covariance_m2(
            flow_delta_body_sensor[:2],
            observation["distance_m"],
            self.flow_base_displacement_sigma_m,
            self.flow_range_near_limit_m,
            self.flow_range_near_sigma_m,
            self.flow_range_far_relative_sigma,
        )
        if flow_covariance is None:
            flow_covariance = [
                self.flow_base_displacement_sigma_m ** 2,
                self.flow_base_displacement_sigma_m ** 2,
            ]
        range_sigma_m = mtf01p_range_sigma_m(
            observation["distance_m"],
            self.flow_range_near_limit_m,
            self.flow_range_near_sigma_m,
            self.flow_range_far_relative_sigma,
        )
        self.last_flow_speed_mps = flow_speed_mps
        self.last_flow_speed_limit_mps = flow_speed_limit_mps
        self.last_flow_range_sigma_m = range_sigma_m
        self.last_flow_covariance_m2 = float(flow_covariance[0])
        decision["evidence"].update({
            "flow_sample_count": observation["sample_count"],
            "flow_total_integration_s": observation["integration_s"],
            "flow_speed_mps": flow_speed_mps,
            "flow_speed_limit_mps": flow_speed_limit_mps,
            "flow_speed_limit_valid": 1.0 if speed_ok else 0.0,
            "flow_range_sigma_m": range_sigma_m,
            "flow_factor_variance_m2": float(flow_covariance[0]),
        })
        if lever_evidence["valid"] < 0.5 and lever_evidence["enabled"] > 0.5:
            decision["evidence"]["flow_lever_arm_unavailable"] = 1.0
            decision["reasons"].append(
                f"flow_lever_arm_{lever_evidence['reason']}"
            )
        if los_diagnostic is None:
            decision["evidence"]["flow_los_diagnostic_valid"] = 0.0
        else:
            decision["evidence"].update({
                "flow_los_diagnostic_valid": 1.0,
                "flow_los_measurement_x_radps": los_diagnostic[
                    "measurement_radps"][0],
                "flow_los_measurement_y_radps": los_diagnostic[
                    "measurement_radps"][1],
                "flow_los_prediction_x_radps": los_diagnostic[
                    "prediction_with_lever_radps"][0],
                "flow_los_prediction_y_radps": los_diagnostic[
                    "prediction_with_lever_radps"][1],
                "flow_los_residual_norm_radps": los_diagnostic[
                    "residual_norm_radps"],
                "flow_los_residual_without_lever_norm_radps": los_diagnostic[
                    "residual_without_lever_norm_radps"],
                "flow_los_lever_delta_norm_radps": los_diagnostic[
                    "lever_delta_norm_radps"],
                "flow_los_distance_m": los_diagnostic["distance_m"],
                "flow_los_integration_s": los_diagnostic["integration_s"],
            })
        quality_or_distance_invalid = (
            observation["quality"] < self.minimum_flow_quality
            or not self.minimum_flow_distance_m
            <= observation["distance_m"]
            <= self.maximum_flow_distance_m
            or not speed_ok
        )
        imu_yaw_samples = [
            (stamp_s, float(angular_velocity[2]))
            for stamp_s, angular_velocity in angular_samples
        ]
        yaw_rate = interval_mean_absolute_yaw_rate(
            imu_yaw_samples,
            previous_stamp,
            current_stamp,
            self.flow_rotation_imu_max_gap_s,
        )
        translation_norm = float(np.linalg.norm(
            np.asarray(flow_delta_body, dtype=float)[:2]
        ))
        rotation_gate = self.flow_rotation_gate.update(
            current_stamp,
            yaw_rate,
            translation_norm,
            flow_displacement is not None and not quality_or_distance_invalid,
            rotation_compensated=flow_displacement is not None,
        )
        decision["evidence"].update({
            "fcu_yaw_rate_abs_radps": rotation_gate.yaw_rate_abs_radps,
            "rotation_gate_weight": rotation_gate.weight,
            "rotation_gate_phase_code": OpticalFlowRotationGate.PHASE_CODES[
                rotation_gate.phase
            ],
            "rotation_gate_translation_ready": (
                1.0 if rotation_gate.translation_ready else 0.0
            ),
        })
        decision = apply_flow_rotation_gate(decision, rotation_gate)
        self.last_flow_rotation_phase = rotation_gate.phase
        self.last_flow_rotation_weight = rotation_gate.weight
        self.last_flow_yaw_rate_abs_radps = rotation_gate.yaw_rate_abs_radps
        if quality_or_distance_invalid:
            decision["factor_enabled"] = False
            decision["reliability_weight"] = 0.0
            decision["covariance_inflation"] = MAX_COVARIANCE_INFLATION
            if not speed_ok:
                self.counts["flow_disabled_speed"] += 1
                self.last_flow_reason = "sensor_speed_limit_gate"
            else:
                self.counts["flow_disabled_quality"] += 1
                self.last_flow_reason = "quality_or_distance_gate"
        elif rotation_gate.hard_disabled:
            self.counts["flow_disabled_rotation"] += 1
            self.last_flow_reason = rotation_gate.reason
        elif rotation_gate.phase != "ACTIVE":
            self.last_flow_reason = rotation_gate.reason
        else:
            self.last_flow_reason = "accepted"
        range_observation = None
        range_result = None
        if self.range_facet_enabled and current_state is not None:
            range_observation, range_result = (
                self._build_range_facet_observation(
                    records, observation, native_factor, current_state,
                    current_stamp
                )
            )
            if range_observation is None:
                self.counts["flow_range_facet_rejected"] += 1
                rejection_reason = str(range_result or "unavailable")
                rejection_reasons = getattr(
                    self, "range_facet_rejection_reasons", None)
                if rejection_reasons is None:
                    rejection_reasons = Counter()
                    self.range_facet_rejection_reasons = rejection_reasons
                rejection_reasons[rejection_reason] += 1
                self.last_range_facet_rejection_reason = rejection_reason
                decision["evidence"]["range_facet_rejected"] = 1.0
                decision["evidence"]["range_facet_rejection_reason"] = (
                    rejection_reason
                )
            else:
                self.last_range_facet_rejection_reason = "accepted"
                decision["evidence"].update({
                    "range_facet_accepted": 1.0,
                    "range_facet_residual_m": float(
                        range_result.residual_m
                    ),
                    "range_facet_support_count": int(
                        range_result.support_count
                    ),
                    "range_facet_plane_rmse_m": float(
                        range_observation.plane_rmse_m
                    ),
                    "range_facet_predicted_m": float(
                        range_result.predicted_range_m
                    ),
                })
        if self.optical_flow_yaw_coupling_enabled:
            if range_observation is not None:
                self.backend.add_optical_flow_range_body(
                    previous_index, current_index, flow_delta_body.tolist(),
                    range_observation,
                    range_result.measurement_variance_m2,
                    previous_yaw,
                    covariance=flow_covariance, decision=decision,
                )
                self.counts["flow_range_facet_accepted"] += 1
                self.last_flow_factor_type = "body_horizontal_2d_range_facet"
            else:
                self.backend.add_optical_flow_body(
                    previous_index, current_index, flow_delta_body.tolist(),
                    previous_yaw,
                    covariance=flow_covariance, decision=decision,
                )
                self.last_flow_factor_type = "body_horizontal_2d"
        else:
            self.backend.add_optical_flow(
                previous_index, current_index, flow_delta_position[:2].tolist(),
                covariance=flow_covariance, decision=decision,
            )
            self.last_flow_factor_type = "map_horizontal_2d"
        if (
            bool(decision.get("factor_enabled", True))
            and float(decision.get("reliability_weight", 1.0)) > 0.0
        ):
            self.counts["flow_factors"] += 1

    def _native_lidar(self, msg):
        self.counts["native_lidar_received"] += 1
        try:
            factor = native_factor_from_message(msg)
        except (ValueError, TypeError) as error:
            self.counts["native_lidar_invalid"] += 1
            self.last_reason = f"invalid_native_lidar:{type(error).__name__}"
            self.last_exception = f"{type(error).__name__}:{error}"
            return
        try:
            validate_native_frame_contract(
                factor, self.map_frame, self.body_frame)
        except ValueError as error:
            self.counts["native_lidar_invalid"] += 1
            self.last_reason = "invalid_native_lidar_frame_contract"
            self.last_exception = f"{type(error).__name__}:{error}"
            return
        if self.input_trigger_mode == "native_factor":
            self._dispatch_native_frame(msg.header, factor)
            return
        self.native_lidar_buffer.push(factor)
        self._drain_pending_inputs()

    def _dispatch_native_frame(self, header, factor):
        stamp_ns = int(factor.stamp_ns)
        sequence = int(factor.scan_sequence)
        if sequence <= self.last_native_consumed_sequence:
            self.counts["native_trigger_terminal_stale"] += 1
            return
        status, gap = native_trigger_order_status(
            self.last_native_input_stamp_ns,
            self.last_native_input_sequence,
            stamp_ns,
            sequence,
        )
        if status == "duplicate":
            self.counts["native_trigger_duplicates"] += 1
            return
        if status in {"sequence_conflict", "sequence_reset"}:
            self.counts["native_trigger_sequence_conflicts"] += 1
            self.last_reason = f"native_trigger_{status}"
            return
        if status == "nonmonotonic":
            self.counts["native_trigger_nonmonotonic"] += 1
            self.last_reason = "nonmonotonic_native_trigger"
            return
        self.last_native_input_arrival_s = self._now_s()
        self.last_native_callback_source_age_s = (
            self.last_native_input_arrival_s - float(factor.stamp_s)
        )
        self.counts["native_trigger_sequence_gaps"] += gap
        self.last_native_input_stamp_ns = stamp_ns
        self.last_native_input_sequence = sequence
        discarded = enqueue_latest(
            self.native_work_queue, (copy.deepcopy(header), factor)
        )
        if discarded < 0:
            self.counts["native_worker_queue_overflow"] += 1
            self.last_reason = "native_worker_queue_enqueue_failed"
        elif discarded:
            self.counts["native_worker_queue_superseded"] += discarded
            self.counts["native_worker_queue_discarded"] += discarded

    def _native_worker_loop(self):
        while not self.native_worker_stop.is_set():
            try:
                header, factor = self.native_work_queue.get(timeout=0.05)
            except queue.Empty:
                self._process_auxiliary_keyframe_if_due()
                continue
            try:
                # The worker may dequeue a frame just before a newer frame is
                # enqueued. Once a newer frame is already waiting, doing the
                # full sliding-window solve for the older one only increases
                # source age and can trigger a rollback storm. Drop this
                # frame before it mutates the backend; the newer frame remains
                # the sole candidate for the next transaction.
                if (
                    self.native_worker_latest_only_enabled
                    and self._native_worker_frame_superseded(factor)
                ):
                    continue
                self._process_native_worker_frame(header, factor)
            except Exception as error:  # one bad packet must not kill the worker
                if self.native_worker_stop.is_set() or not rclpy.ok():
                    self.active_transaction_snapshot = None
                    self.last_reason = "shutdown"
                    break
                if self.active_transaction_snapshot is not None:
                    self.backend.restore(self.active_transaction_snapshot)
                    self.active_transaction_snapshot = None
                    self.counts["optimization_rollbacks"] += 1
                self.counts["native_worker_errors"] += 1
                self.last_reason = f"native_worker_error:{type(error).__name__}"
                self.last_exception = f"{type(error).__name__}:{error}"
                self._consume_native_sequence(
                    int(factor.scan_sequence), state_committed=False
                )
            finally:
                self.native_work_queue.task_done()

    def _native_worker_frame_superseded(self, factor):
        """Return true when a newer native frame is already queued.

        The queue is inspected under its mutex so the decision is consistent
        with enqueue. The current frame is released as an intentional
        latest-only drop, distinct from a frame that entered optimization and
        failed its integrity checks.
        """
        current_sequence = int(factor.scan_sequence)
        try:
            with self.native_work_queue.mutex:
                pending = list(self.native_work_queue.queue)
        except (AttributeError, RuntimeError):
            return False
        for item in pending:
            try:
                pending_sequence = int(item[1].scan_sequence)
            except (IndexError, AttributeError, TypeError, ValueError):
                continue
            if pending_sequence > current_sequence:
                self.counts["native_worker_latest_skipped"] += 1
                self.last_reason = "native_worker_latest_only_skip"
                self._consume_native_sequence(
                    current_sequence,
                    state_committed=False,
                    intentional_latest_skip=True,
                )
                return True
        return False

    def _process_auxiliary_keyframe_if_due(self):
        if (
            not self.auxiliary_keyframe_enabled
            or self.input_trigger_mode != "native_factor"
            or self.backend.state_count == 0
            or self.last_lio_stamp is None
        ):
            self.last_auxiliary_keyframe_reason = "disabled_or_uninitialized"
            return
        imu_samples = self._imu_snapshot()
        latest_imu_stamp_s = (
            float(imu_samples[-1].stamp_s) if imu_samples else None
        )
        now_s = self._now_s()
        admitted, reason = auxiliary_keyframe_admission(
            now_s,
            latest_imu_stamp_s,
            self.last_lio_stamp,
            self.last_native_input_arrival_s,
            self.auxiliary_keyframe_lidar_silence_timeout_s,
            self.auxiliary_keyframe_minimum_interval_s,
            self.auxiliary_keyframe_maximum_imu_age_s,
        )
        self.last_auxiliary_keyframe_reason = reason
        if not admitted:
            return
        self.counts["auxiliary_keyframe_attempts"] += 1
        state = np.asarray(self.backend.state(-1), dtype=float)
        message = Odometry()
        message.header.stamp = ros_time_from_seconds(latest_imu_stamp_s)
        message.header.frame_id = self.map_frame
        message.child_frame_id = self.body_frame
        message.pose.pose.position.x = float(state[0])
        message.pose.pose.position.y = float(state[1])
        message.pose.pose.position.z = float(state[2])
        qx, qy, qz, qw = rpy_to_quaternion_xyzw(state[3:6])
        message.pose.pose.orientation.x = qx
        message.pose.pose.orientation.y = qy
        message.pose.pose.orientation.z = qz
        message.pose.pose.orientation.w = qw
        try:
            previous_stamp_s = float(self.last_lio_stamp)
            self._process_lio(message, None)
            if (
                self.last_reason == "ok"
                and self.last_lio_stamp is not None
                and self.last_lio_stamp > previous_stamp_s
            ):
                self.counts["auxiliary_keyframe_committed"] += 1
                self.last_state_trigger_source = "auxiliary_keyframe"
                self.last_auxiliary_keyframe_reason = "ok"
                self.maximum_auxiliary_position_variance_m2 = max(
                    self.maximum_auxiliary_position_variance_m2,
                    self.last_output_position_variance_m2,
                )
            else:
                self.counts["auxiliary_keyframe_rejected"] += 1
                self.last_auxiliary_keyframe_reason = self.last_reason
        except Exception as error:
            if self.active_transaction_snapshot is not None:
                self.backend.restore(self.active_transaction_snapshot)
                self.active_transaction_snapshot = None
                self.counts["optimization_rollbacks"] += 1
            self.counts["auxiliary_keyframe_errors"] += 1
            self.last_auxiliary_keyframe_reason = (
                f"error:{type(error).__name__}"
            )
            self.last_exception = f"{type(error).__name__}:{error}"

    def _process_native_worker_frame(self, header, factor):
        self.last_native_worker_source_age_s = (
            self._now_s() - float(factor.stamp_s)
        )
        message = native_frame_odometry(header, factor)
        stamp = factor.stamp_s
        imu_factor_count_before = self.counts["imu_factors"]
        imu_factor_expected = (
            self.backend_solver_mode == "manifold"
            and self.imu_factor_enabled
            and self.last_lio_stamp is not None
        )
        if (
            imu_factor_expected
        ):
            deadline = self._now_s() + self.imu_factor_wait_s
            covered = False
            while not self.native_worker_stop.is_set():
                covered, _, _ = imu_interval_status(
                    self._imu_snapshot(),
                    self.last_lio_stamp,
                    stamp,
                    self.imu_max_gap_s,
                )
                if covered or self._now_s() >= deadline:
                    break
                self.native_worker_stop.wait(0.002)
        if not self.native_worker_stop.is_set():
            self._process_lio(message, factor)
            state_committed = (
                self.last_lio_stamp is not None
                and abs(float(self.last_lio_stamp) - float(factor.stamp_s))
                <= self.scan_prediction_timestamp_tolerance_s
                and self.last_reason == "ok"
            )
            if state_committed:
                self.last_state_trigger_source = "native_lidar"
                # An IMU callback can arrive after the preflight wait expires
                # but before _process_lio takes its final buffer snapshot.  A
                # timeout is real only when the committed transition did not
                # receive an IMU factor.
                if committed_state_missing_imu_factor(
                    state_committed,
                    imu_factor_expected,
                    imu_factor_count_before,
                    self.counts["imu_factors"],
                ):
                    self.counts["imu_pair_timeouts"] += 1
            self._consume_native_sequence(
                int(factor.scan_sequence), state_committed=state_committed
            )

    def _consume_native_sequence(
            self, sequence, state_committed, intentional_latest_skip=False):
        """Release the next scan after this factor reaches a terminal outcome."""
        if not state_committed and not intentional_latest_skip:
            self.counts["native_consumed_without_state_commit"] += 1
        self.last_native_consumed_sequence = max(
            self.last_native_consumed_sequence, int(sequence)
        )
        self._release_pending_scan_requests()

    def stop_native_worker(self):
        self.native_worker_stop.set()
        if self.native_worker_thread is not None:
            self.native_worker_thread.join(timeout=2.0)

    def _lio(self, msg):
        if self.input_trigger_mode == "native_factor":
            self.counts["lio_pose_inputs_ignored"] += 1
            return
        if not self.native_lidar_enabled:
            self._dispatch_lio(msg, None)
            return
        stamp = stamp_seconds(msg.header.stamp)
        if stamp <= 0.0:
            self.last_reason = "missing_lio_timestamp"
            return
        factor = self.native_lidar_buffer.pop_nearest(
            stamp, self.native_lidar_tolerance_s
        )
        if factor is not None:
            self._dispatch_lio(msg, factor)
            return
        self.pending_lio.append((self._now_s(), msg))

    def _dispatch_lio(self, msg, native_factor):
        stamp = stamp_seconds(msg.header.stamp)
        if stamp <= 0.0:
            self.last_reason = "missing_lio_timestamp"
            return
        imu_ready = True
        if self.last_lio_stamp is not None:
            imu_ready, _, _ = imu_interval_status(
                self._imu_interval_snapshot(self.last_lio_stamp, stamp),
                self.last_lio_stamp,
                stamp,
                self.imu_max_gap_s,
            )
        needs_imu_wait = (
            self.backend_solver_mode == "manifold"
            and self.imu_factor_enabled
            and self.last_lio_stamp is not None
            and not imu_ready
        )
        if needs_imu_wait:
            self.pending_imu_lio.append(
                (self._now_s(), msg, native_factor)
            )
            return
        self._process_lio(msg, native_factor)

    def _drain_pending_inputs(self):
        self._drain_pending_lio()
        self._drain_pending_imu_lio()

    def _drain_pending_lio(self):
        if not self.pending_lio:
            return
        now = self._now_s()
        while self.pending_lio:
            arrival, msg = self.pending_lio[0]
            if arrival > now:
                self.pending_lio.clear()
                return
            stamp = stamp_seconds(msg.header.stamp)
            if stamp <= 0.0:
                self.pending_lio.popleft()
                self.last_reason = "missing_lio_timestamp"
                continue
            factor = self.native_lidar_buffer.pop_nearest(
                stamp, self.native_lidar_tolerance_s
            )
            if factor is None and now - arrival < self.native_lidar_wait_s:
                break
            self.pending_lio.popleft()
            if factor is None:
                self.counts["native_lidar_pair_timeouts"] += 1
            self._dispatch_lio(msg, factor)

    def _drain_pending_imu_lio(self):
        if not self.pending_imu_lio:
            return
        now = self._now_s()
        while self.pending_imu_lio:
            arrival, msg, native_factor = self.pending_imu_lio[0]
            if arrival > now:
                self.pending_imu_lio.clear()
                return
            stamp = stamp_seconds(msg.header.stamp)
            if stamp <= 0.0:
                self.pending_imu_lio.popleft()
                self.last_reason = "missing_lio_timestamp"
                continue
            covered, _, _ = imu_interval_status(
                self._imu_interval_snapshot(self.last_lio_stamp, stamp),
                self.last_lio_stamp,
                stamp,
                self.imu_max_gap_s,
            )
            timed_out = now - arrival >= self.imu_factor_wait_s
            if not covered and not timed_out:
                break
            self.pending_imu_lio.popleft()
            if not covered:
                self.counts["imu_pair_timeouts"] += 1
            self._process_lio(msg, native_factor)

    def _process_lio(self, msg, native_factor):
        started = time.perf_counter_ns()
        stamp = stamp_seconds(msg.header.stamp)
        if stamp <= 0.0:
            self.last_reason = "missing_lio_timestamp"
            return
        relocalization_applied_now = self._apply_pending_relocalization(stamp)
        if native_factor is not None:
            self.last_native_factor_reset_counter = int(
                native_factor.reset_counter
            )
            native_epoch_status = native_factor_epoch_status(
                native_factor.reset_counter,
                self.state_reset_counter,
                relocalization_applied_now,
                self.frontend_scan_prediction_enabled,
            )
        elif native_factor_epoch_barrier_required(
            relocalization_applied_now, self.frontend_scan_prediction_enabled
        ):
            native_epoch_status = "barrier"
        else:
            native_epoch_status = "current"
        if native_epoch_status == "barrier":
            # This factor was matched with the old-epoch trajectory. Applying
            # the pose correction to its already-selected planes would retain
            # stale correspondences. Consume it at the epoch barrier and let
            # FAST-LIO rematch the next scan using the corrected trajectory.
            self.counts["relocalization_epoch_factor_drops"] += 1
            self.last_reason = "relocalization_epoch_barrier"
            return
        if native_epoch_status == "stale":
            self.counts["native_lidar_epoch_stale_rejected"] += 1
            self.last_reason = "native_lidar_stale_epoch"
            return
        if native_epoch_status == "future":
            self.counts["native_lidar_epoch_future_rejected"] += 1
            self.last_reason = "native_lidar_future_epoch"
            return
        native_prediction_gate_rejected = False
        native_prediction_recovery_floor = False
        native_observability = None
        native_vertical = None
        native_raw_correspondences_available = False
        if native_factor is not None:
            factor_alignment = native_factor_epoch_alignment(
                self.map_from_lio,
                self.frontend_scan_prediction_enabled,
            )
            if not np.allclose(factor_alignment, np.eye(4), atol=1.0e-12):
                native_factor = transform_native_factor_map(
                    native_factor, factor_alignment
                )
            msg = native_frame_odometry(msg.header, native_factor)
            stamp = native_factor.stamp_s
        else:
            factor_alignment = native_factor_epoch_alignment(
                self.map_from_lio,
                self.frontend_scan_prediction_enabled,
            )
        if native_factor is None and not np.allclose(
            factor_alignment, np.eye(4), atol=1.0e-12
        ):
            local_pose = np.concatenate((
                np.asarray([
                    float(msg.pose.pose.position.x),
                    float(msg.pose.pose.position.y),
                    float(msg.pose.pose.position.z),
                ]),
                quaternion_xyzw_to_rpy([
                    float(msg.pose.pose.orientation.x),
                    float(msg.pose.pose.orientation.y),
                    float(msg.pose.pose.orientation.z),
                    float(msg.pose.pose.orientation.w),
                ]),
            ))
            aligned_pose = matrix_to_pose_vector(
                factor_alignment @ pose_vector_to_matrix(local_pose)
            )
            msg = copy.deepcopy(msg)
            msg.pose.pose.position.x = float(aligned_pose[0])
            msg.pose.pose.position.y = float(aligned_pose[1])
            msg.pose.pose.position.z = float(aligned_pose[2])
            qx, qy, qz, qw = rpy_to_quaternion_xyzw(aligned_pose[3:6])
            msg.pose.pose.orientation.x = qx
            msg.pose.pose.orientation.y = qy
            msg.pose.pose.orientation.z = qz
            msg.pose.pose.orientation.w = qw
        if self.last_lio_stamp is not None and stamp <= self.last_lio_stamp:
            self.last_reason = "nonmonotonic_lio_stamp"
            return
        if (
            self.backend.state_count == 0
            and native_factor is not None
            and not native_factor.correspondences_valid
        ):
            self.counts["native_trigger_waiting_for_initial_factor"] += 1
            self.last_reason = "waiting_for_initial_native_lidar_factor"
            return
        pose = msg.pose.pose
        position = np.asarray([float(pose.position.x), float(
            pose.position.y), float(pose.position.z)], dtype=float, )
        orientation = quaternion_xyzw_to_rpy([
            float(pose.orientation.x), float(pose.orientation.y),
            float(pose.orientation.z), float(pose.orientation.w),
        ])
        orientation[2] = unwrap_yaw(
            self.last_lio_yaw if self.last_lio_stamp is not None else None,
            float(orientation[2]),
        )
        yaw = float(orientation[2])
        previous_state = (
            self.backend.state(-1) if self.backend.state_count > 0 else None
        )
        if self.last_lio_stamp is None:
            imu_window_start = (
                stamp - self.imu_startup_window_s - self.imu_max_gap_s
            )
        else:
            calibration_offset = effective_time_offset(
                self.last_calibration_update,
                self.online_calibration_enabled
                and self.calibration_apply_locked_time_offset,
                time_locked=getattr(
                    getattr(self, "calibrator", None),
                    "time_locked",
                    getattr(self.last_calibration_update, "locked", False),
                ),
            )
            imu_window_start = (
                self.last_lio_stamp
                + min(0.0, calibration_offset)
                - self.imu_max_gap_s
            )
        imu_window_end = (
            stamp
            + (
                max(0.0, calibration_offset)
                if self.last_lio_stamp is not None else 0.0
            )
            + self.imu_max_gap_s
        )
        cycle_imu_samples = self._imu_interval_snapshot(
            imu_window_start, imu_window_end
        )
        manifold_measurement = None
        reference = None
        scan_prediction = None
        if previous_state is None:
            initial_state = np.zeros(15, dtype=float)
            initial_state[:3] = position
            initial_state[3:6] = orientation
            if (
                self.backend_solver_mode == "manifold"
                and self.imu_startup_bias_initialization_enabled
            ):
                startup = estimate_stationary_imu_bias(
                    cycle_imu_samples,
                    orientation,
                    stamp,
                    window_s=self.imu_startup_window_s,
                    minimum_samples=self.imu_startup_minimum_samples,
                    minimum_span_s=self.imu_startup_minimum_span_s,
                    maximum_mean_gyro_radps=(
                        self.imu_startup_maximum_mean_gyro_radps
                    ),
                    maximum_gyro_residual_rms_radps=(
                        self.imu_startup_maximum_gyro_residual_rms_radps
                    ),
                    gravity_tolerance_mps2=(
                        self.imu_startup_gravity_tolerance_mps2
                    ),
                    maximum_accel_residual_rms_mps2=(
                        self.imu_startup_maximum_accel_residual_rms_mps2
                    ),
                )
                self.last_imu_startup_reason = startup.reason
                self.last_imu_startup_sample_count = startup.sample_count
                self.last_imu_startup_span_s = startup.span_s
                if startup.reason == "insufficient_observation_span":
                    self.counts["imu_startup_waits"] += 1
                    self.last_reason = "waiting_for_stationary_imu_window"
                    return
                if startup.valid:
                    initial_state[9:12] = startup.accel_bias
                    initial_state[12:15] = startup.gyro_bias
                    self.last_imu_startup_accel_bias = np.asarray(
                        startup.accel_bias, dtype=float
                    )
                    self.last_imu_startup_gyro_bias = np.asarray(
                        startup.gyro_bias, dtype=float
                    )
                    self.counts["imu_startup_bias_accepted"] += 1
                else:
                    self.counts["imu_startup_bias_rejected"] += 1
            elif self.backend_solver_mode == "manifold":
                self.last_imu_startup_reason = "disabled"
        elif (
            self.frontend_scan_prediction_enabled
            and native_factor is not None
        ):
            if native_factor.scan_end_s <= native_factor.scan_begin_s:
                self.counts["optimization_rejected"] += 1
                self.last_reason = "scan_prediction_missing_exact_interval"
                self._record_scan_prediction_contract_failure(
                    "missing_exact_interval",
                    native_factor.scan_sequence,
                    stamp,
                )
                return
            scan_prediction, prediction_reason = consume_cached_prediction(
                self.scan_prediction_by_sequence,
                sequence=native_factor.scan_sequence,
                previous_stamp_s=self.last_lio_stamp,
                scan_end_s=native_factor.scan_end_s,
                current_previous_state=previous_state,
                timestamp_tolerance_s=self.scan_prediction_timestamp_tolerance_s,
                state_tolerance=self.scan_prediction_state_tolerance,
            )
            if scan_prediction is None:
                if prediction_reason == "cache_miss":
                    self.counts["scan_prediction_cache_misses"] += 1
                else:
                    self.counts["scan_prediction_reuse_rejected"] += 1
                self.counts["optimization_rejected"] += 1
                self.last_reason = f"scan_prediction_not_reusable:{prediction_reason}"
                self._record_scan_prediction_contract_failure(
                    prediction_reason,
                    native_factor.scan_sequence,
                    stamp,
                )
                return
            self.counts["scan_prediction_cache_hits"] += 1
            self._record_scan_prediction_contract_success()
            self.scan_prediction_cache.append(scan_prediction)
            initial_state = scan_prediction.end_state.copy()
            manifold_measurement = scan_prediction.measurement
            self.counts["imu_propagated_initializations"] += 1
            reference = manifold_motion_reference(
                previous_state, initial_state)
        elif self.backend_solver_mode == "manifold":
            manifold_measurement = self._manifold_imu_measurement(
                self.last_lio_stamp, stamp, previous_state,
                cycle_imu_samples,
            )
            if manifold_measurement is not None:
                initial_state = propagate_state(
                    previous_state, manifold_measurement)
                self.counts["imu_propagated_initializations"] += 1
            else:
                initial_state = previous_state.copy()
                initial_state[:3] += previous_state[6:9] * (
                    stamp - self.last_lio_stamp
                )
            reference = manifold_motion_reference(
                previous_state, initial_state)
        else:
            initial_state = previous_state.copy()
            initial_state[:3] = position
            initial_state[3:6] = orientation
            reference = fused_motion_reference(
                previous_state, stamp - self.last_lio_stamp,
            )
        if native_factor is not None and native_factor.correspondences_valid:
            native_observability = lidar_pose_observability(native_factor)
            native_vertical = lidar_vertical_observability(native_factor)
            native_raw_correspondences_available = all(
                value is not None
                for value in (
                    native_factor.lidar_points,
                    native_factor.plane_normals,
                    native_factor.plane_points,
                    native_factor.lidar_to_body_rotation,
                    native_factor.lidar_to_body_translation,
                )
            )
            self.last_native_effective_rank = native_observability.effective_rank
            self.last_native_translation_rank = (
                native_observability.translation_rank
            )
            self.last_native_rotation_rank = native_observability.rotation_rank
            self.last_native_condition_number = (
                native_observability.condition_number
            )
            self.last_native_characteristic_range_m = (
                native_observability.characteristic_range_m
            )
            self.last_native_normalized_eigenvalues = np.asarray(
                native_observability.normalized_eigenvalues, dtype=float
            )
            self.last_native_vertical_raw_information = (
                native_vertical.raw_information
            )
            self.last_native_vertical_profile_information = (
                native_vertical.profile_information
            )
            self.last_native_vertical_coupling_retention_ratio = (
                native_vertical.coupling_retention_ratio
            )
            self.last_native_normal_z_energy_fraction = (
                native_vertical.normal_z_energy_fraction
            )
            self.last_native_horizontal_plane_fraction = (
                native_vertical.horizontal_plane_fraction
            )
            self.last_native_axis_raw_information = np.asarray(
                native_vertical.axis_raw_information, dtype=float
            )
            self.last_native_axis_profile_information = np.asarray(
                native_vertical.axis_profile_information, dtype=float
            )
            self.last_native_axis_coupling_retention_ratio = np.asarray(
                native_vertical.axis_coupling_retention_ratio, dtype=float
            )
            self.last_native_axis_relative_support = np.asarray(
                native_vertical.axis_relative_support, dtype=float
            )
            self.last_native_translation_profile_information = np.asarray(
                native_vertical.translation_profile_information, dtype=float
            ).reshape(3, 3)
            self.last_native_translation_normalized_eigenvalues = np.asarray(
                native_vertical.translation_normalized_eigenvalues, dtype=float
            )
            self.last_native_translation_eigenvectors = np.asarray(
                native_vertical.translation_eigenvectors, dtype=float
            ).reshape(3, 3)
            self.last_native_weakest_translation_direction = np.asarray(
                native_vertical.weakest_translation_direction, dtype=float
            )
            self._update_lidar_subspace_projector()
            if native_observability.effective_rank < 6:
                self.counts["native_lidar_directionally_degenerate"] += 1

        innovation = None
        if reference is not None:
            innovation = lidar_prediction_innovation(position, yaw, reference)
            self.last_lidar_prediction_position_innovation_m = innovation[
                "position_m"
            ]
            self.last_lidar_prediction_yaw_innovation_rad = innovation["yaw_rad"]
        native_reliability_layers = None
        if native_factor is not None and native_factor.correspondences_valid:
            native_reliability_layers = lidar_reliability_layers(
                native_factor,
                native_vertical,
                position_innovation_m=(
                    None if innovation is None else innovation["position_m"]
                ),
                yaw_innovation_rad=(
                    None if innovation is None else innovation["yaw_rad"]
                ),
                position_innovation_scale_m=(
                    self.lidar_prediction_gate_max_position_m
                ),
                yaw_innovation_scale_rad=self.lidar_prediction_gate_max_yaw_rad,
            )
            self.last_native_health_degradation = (
                native_reliability_layers.health_degradation
            )
            self.last_native_consistency_degradation = (
                native_reliability_layers.consistency_degradation
            )
            self.last_native_observability_degradation = np.asarray(
                native_reliability_layers.observability_degradation_xyz,
                dtype=float,
            )
            self.last_native_combined_degradation = np.asarray(
                native_reliability_layers.combined_degradation_xyz,
                dtype=float,
            )
            self.last_native_isotropic_information_support = np.asarray(
                native_reliability_layers.isotropic_information_support_xyz,
                dtype=float,
            )
            self.lidar_axis_observability_latched = axis_observability_latch(
                self.last_native_isotropic_information_support,
                self.lidar_axis_observability_latched,
                enter_support=self.axis_handoff_enter_support,
                exit_support=self.axis_handoff_exit_support,
            )
            if (
                native_factor is not None
                and native_factor.correspondences_valid
                and self.lidar_prediction_gate_enabled
                and innovation is not None
            ):
                gate = lidar_prediction_factor_admission(
                    innovation,
                    self.lidar_prediction_gate_max_position_m,
                    self.lidar_prediction_gate_max_yaw_rad,
                    self.native_lidar_prediction_gate_consecutive_rejections,
                    self.lidar_prediction_gate_recovery_after_rejections,
                    recovery_geometry_usable=bool(
                        native_observability is not None
                        and native_observability.effective_rank == 6
                        and native_observability.translation_rank == 3
                        and native_observability.rotation_rank == 3
                        and native_raw_correspondences_available
                        and native_factor.matched_points
                        >= self.native_lidar_minimum_matches
                    ),
                )
                self.native_lidar_prediction_gate_consecutive_rejections = int(
                    gate["consecutive_rejections"]
                )
                self.last_native_lidar_prediction_gate_reason = str(
                    gate["reason"]
                )
                if gate["recovered"]:
                    self.counts[
                        "native_lidar_prediction_gate_recoveries"
                    ] += 1
                if int(gate["consecutive_rejections"]) > 0:
                    self.counts["native_lidar_prediction_gate_rejections"] += 1
                native_prediction_recovery_floor = bool(
                    gate["recovery_floor"]
                )
                if not gate["factor_enabled"]:
                    native_prediction_gate_rejected = True
            else:
                self.last_native_lidar_prediction_gate_reason = "not_evaluated"
        self._record_phase_timing("pre_state", started)
        performance_context = self._begin_performance_cycle(started, stamp)
        snapshot_started = time.perf_counter_ns()
        transaction_snapshot = (self.backend.snapshot(
        ) if self.transactional_update_enabled else None)
        self._record_phase_timing("snapshot", snapshot_started)
        self.active_transaction_snapshot = transaction_snapshot
        frontend_map_candidate = None
        if self.frontend_map_commit_delay_states > 0:
            with self.visual_lock:
                committed_stamps = list(self.visual_state_stamps)
            frontend_map_candidate = attach_frontend_map_commit_eligibility(
                delayed_frontend_map_commit_candidate(
                    self.backend.states(),
                    committed_stamps,
                    self.frontend_map_commit_delay_states,
                ),
                self.frontend_map_eligibility_by_stamp,
            )
        add_state_started = time.perf_counter_ns()
        current_index = self.backend.add_state(initial_state)
        transaction_visual_state_stamps = list(self.visual_state_stamps)
        transaction_visual_state_stamps.append(float(stamp))
        transaction_visual_state_stamps = transaction_visual_state_stamps[
            -self.backend.state_count:
        ]
        staged_visual_candidates = []
        self._record_phase_timing("add_state", add_state_started)
        if self.last_lio_stamp is None:
            self.lio_origin = position.copy()
            prior_covariance = np.full(15, 1.0e-4)
            if self.backend_solver_mode == "manifold":
                startup_bias_accepted = (
                    self.counts["imu_startup_bias_accepted"] > 0
                )
                prior_covariance = np.asarray(
                    [1.0e-4] * 6
                    + [1.0] * 3
                    + [0.10 ** 2 if startup_bias_accepted else 0.50 ** 2] * 3
                    + [0.01 ** 2 if startup_bias_accepted else 0.05 ** 2] * 3,
                    dtype=float,
                )
            self.backend.add_prior(
                current_index,
                initial_state.copy(),
                covariance=prior_covariance,
            )
        # Add the causal local-pressure factor before computing LiDAR axis
        # handoff.  The pressure observation is still inserted exactly once;
        # this ordering only lets its current Z information protect a weak
        # LiDAR axis in the same transaction.
        barometer_started = time.perf_counter_ns()
        barometer_active = False
        if self.last_lio_stamp is not None and reference is not None:
            self._barometer_factor(
                stamp, reference["position"][2], current_index
            )
            with self.barometer_lock:
                barometer_active = bool(self.barometer_segment.active)
        elif self.barometer_fallback_enabled:
            self.last_barometer_reason = "waiting_for_previous_state"
        self._record_phase_timing("barometer_factor", barometer_started)

        # Admit the current GNSS observation before assembling the native
        # LiDAR factor.  Axis handoff must use the same-time GNSS/barometer
        # information, rather than the previous cycle's cached admission.
        # The factor is still added exactly once; this only fixes the causal
        # ordering of the independent factor blocks.
        if self.last_lio_stamp is not None:
            previous_index = current_index - 1
            gnss_started = time.perf_counter_ns()
            gnss_factor_decision = self._decision(
                "gnss", default_enabled=True
            )
            if bool(gnss_factor_decision.get("factor_enabled", False)):
                gnss_prediction, gnss_prediction_reason = (
                    self._gnss_prefit_prediction(stamp, manifold_measurement)
                )
            else:
                gnss_prediction = None
                gnss_prediction_reason = "scheduler_disabled_before_prediction"
            if gnss_prediction is None:
                gnss_position_prediction = reference["position"]
                gnss_position_covariance = None
            else:
                gnss_position_prediction, gnss_position_covariance = (
                    gnss_prediction
                )
            self._gnss_factor(
                stamp,
                gnss_position_prediction,
                current_index,
                gnss_position_covariance,
                gnss_prediction_reason,
                gnss_factor_decision,
                initial_state[6:9],
            )
            self._record_phase_timing("gnss_factor", gnss_started)

        self._cap_lidar_subspace_with_current_gnss()

        lidar_factor_started = time.perf_counter_ns()
        lidar_decision = self._decision("lidar", default_enabled=True)
        if native_prediction_recovery_floor:
            scheduler_weight = float(
                lidar_decision.get("reliability_weight", 0.0)
            )
            if (
                not bool(lidar_decision.get("factor_enabled", False))
                or not math.isfinite(scheduler_weight)
                or scheduler_weight <= 0.0
            ):
                native_prediction_recovery_floor = False
                native_prediction_gate_rejected = True
                self.last_native_lidar_prediction_gate_reason = (
                    "lidar_prediction_recovery_scheduler_disabled"
                )
            else:
                lidar_decision["reliability_weight"] = min(
                    scheduler_weight,
                    self.lidar_prediction_recovery_weight,
                )
                lidar_decision["covariance_inflation"] = max(
                    float(lidar_decision.get("covariance_inflation", 1.0)),
                    self.lidar_prediction_recovery_inflation,
                )
        if native_prediction_gate_rejected:
            lidar_decision["factor_enabled"] = False
            lidar_decision["reliability_weight"] = 0.0
            lidar_decision["covariance_inflation"] = MAX_COVARIANCE_INFLATION
            self.counts["native_lidar_hard_disabled"] += 1
            self.last_lidar_source = "native_prediction_gate_rejected"
        if lidar_decision.get("anchor_override", False):
            self.counts["lidar_anchor_overrides"] += 1
        lidar_factor_added = False
        if native_factor is not None:
            native_factor = with_yaw_reference(native_factor, yaw)
            if (
                self.online_calibration_enabled
                and not native_prediction_gate_rejected
                and not native_prediction_recovery_floor
                and native_factor.correspondences_valid
                and native_factor.lidar_to_body_rotation is not None
            ):
                try:
                    update, seed_reason = seed_calibrator_rotation_nonblocking(
                        self.calibrator,
                        self.calibration_lock,
                        native_factor.lidar_to_body_rotation,
                    )
                    if update is not None:
                        self.last_calibration_update = update
                        self.counts["calibration_seed_initialized"] += 1
                    elif seed_reason == "calibration_busy":
                        self.counts["calibration_seed_lock_busy"] += 1
                    calibration_update = self.last_calibration_update
                    if (
                        self.calibration_apply_locked_rotation
                        and self.calibrator.rotation_locked
                    ):
                        native_factor = replace(
                            native_factor, lidar_to_body_rotation=(
                                calibration_update.lidar_to_body_rotation.copy()), )
                except (ValueError, np.linalg.LinAlgError) as error:
                    self.last_exception = (
                        f"calibration_{type(error).__name__}:{error}"
                    )
            self.last_native_sequence = int(native_factor.scan_sequence)
            self.last_native_matches = int(native_factor.matched_points)
            self.last_native_stamp_error_ms = abs(
                native_factor.stamp_s - stamp
            ) * 1000.0
        if native_factor is not None and not native_factor.correspondences_valid:
            lidar_decision["factor_enabled"] = False
            lidar_decision["reliability_weight"] = 0.0
            lidar_decision["covariance_inflation"] = MAX_COVARIANCE_INFLATION
            self.counts["native_trigger_only_frames"] += 1
            self.last_lidar_source = "native_frame_without_correspondences"
        if (
            native_factor is not None
            and native_factor.correspondences_valid
            and not native_prediction_gate_rejected
        ):
            if native_factor.matched_points < self.native_lidar_minimum_matches:
                lidar_decision["factor_enabled"] = False
                lidar_decision["reliability_weight"] = 0.0
                lidar_decision["covariance_inflation"] = MAX_COVARIANCE_INFLATION
                self.counts["native_lidar_hard_disabled"] += 1
            raw_correspondences_available = (
                native_raw_correspondences_available
            )
            if self.backend_solver_mode == "manifold" and raw_correspondences_available:
                self.last_lidar_axis_information_scale = np.ones(
                    3, dtype=float
                )
                if (
                    self.axis_information_handoff_enabled
                    and bool(lidar_decision.get("factor_enabled", False))
                    and native_reliability_layers is not None
                ):
                    lidar_effective_weight = (
                        float(lidar_decision.get("reliability_weight", 0.0))
                        / max(
                            1.0,
                            float(
                                lidar_decision.get(
                                    "covariance_inflation", 1.0
                                )
                            ),
                        )
                    )
                    lidar_information = (
                        self.last_native_axis_profile_information
                        * lidar_effective_weight
                    )
                    alternative_information = (
                        self._axis_handoff_alternative_information(stamp)
                    )
                    (
                        self.last_lidar_axis_information_scale,
                        self.axis_handoff_latched,
                    ) = axis_information_handoff(
                        lidar_information,
                        self.last_native_isotropic_information_support,
                        alternative_information,
                        self.axis_handoff_latched,
                        enabled_axes=self.axis_handoff_enabled_axes,
                        enter_support=self.axis_handoff_enter_support,
                        exit_support=self.axis_handoff_exit_support,
                        minimum_lidar_information_scale=(
                            self.axis_handoff_minimum_lidar_information_scale
                        ),
                        maximum_lidar_to_alternative_ratio=(
                            self.axis_handoff_maximum_lidar_to_alternative_ratio
                        ),
                    )
                    handed_off = self.last_lidar_axis_information_scale < 1.0
                    if np.any(handed_off):
                        self.counts["native_lidar_axis_handoff_frames"] += 1
                        for axis, name in enumerate(("x", "y", "z")):
                            if handed_off[axis]:
                                self.counts[
                                    f"native_lidar_axis_handoff_{name}"
                                ] += 1
                axis_scaled = np.any(
                    self.last_lidar_axis_information_scale < 1.0
                )
                self.backend.add_native_lidar_correspondences(
                    current_index,
                    native_factor,
                    decision=lidar_decision,
                    axis_information_scale=(
                        self.last_lidar_axis_information_scale
                    ),
                )
                if self.lidar_subspace_episode_active:
                    self.backend.set_lidar_subspace_scale(
                        self.last_lidar_subspace_scale
                    )
                self.counts["native_lidar_relinearized"] += 1
                if axis_scaled:
                    self.counts[
                        "native_lidar_axis_conditional_factors"
                    ] += 1
                    self.last_lidar_source = (
                        "native_point_to_plane_axis_scaled_relinearized"
                    )
                else:
                    self.last_lidar_source = (
                        "native_point_to_plane_relinearized"
                    )
                lidar_factor_added = True
            elif self.backend_solver_mode == "manifold" and all(
                value is not None
                for value in (
                    native_factor.pose_hessian_right,
                    native_factor.pose_gradient_right,
                )
            ):
                self.backend.add_native_lidar_normal(
                    current_index,
                    native_factor.linearization_pose,
                    native_factor.pose_hessian_right,
                    native_factor.pose_gradient_right,
                    native_factor.measurement_variance,
                    native_factor.matched_points,
                    native_factor.residual_squared,
                    decision=lidar_decision,
                )
                self.counts["native_lidar_condensed_fallbacks"] += 1
                self.last_lidar_source = "native_point_to_plane_condensed"
                lidar_factor_added = True
            elif self.backend_solver_mode == "linear":
                self.backend.add_native_lidar_normal(
                    current_index,
                    native_factor.linearization_pose,
                    native_factor.pose_hessian,
                    native_factor.pose_gradient,
                    native_factor.measurement_variance,
                    native_factor.matched_points,
                    native_factor.residual_squared,
                    decision=lidar_decision,
                )
                self.last_lidar_source = "native_point_to_plane_linear"
                lidar_factor_added = True
            else:
                self.last_lidar_source = "native_factor_incompatible"
            if lidar_factor_added:
                self.counts["native_lidar_factors"] += 1
                if native_prediction_recovery_floor:
                    self.counts[
                        "native_lidar_prediction_recovery_factors"
                    ] += 1
                    self.last_lidar_source += "_recovery_floor"
        if native_factor is None and (
                self.backend_solver_mode == "linear" or self.allow_lio_pose_fallback):
            self.backend.add_lidar_pose(
                current_index, position, orientation,
                covariance=[0.05 ** 2] * 3 + [0.03 ** 2] * 3,
                decision=lidar_decision,
            )
            self.counts["native_lidar_pose_fallbacks"] += 1
            self.last_lidar_source = "lio_pose_fallback"
            lidar_factor_added = True
        elif native_factor is None:
            self.counts["lio_pose_inputs_ignored"] += 1
            self.last_lidar_source = "native_factor_missing_lio_pose_ignored"
        if lidar_factor_added:
            self.counts["lidar_factors"] += 1
        if lidar_factor_added and not lidar_decision["factor_enabled"]:
            self.counts["lidar_disabled"] += 1
        if native_prediction_recovery_floor:
            self.last_lidar_map_eligible = False
            self.last_lidar_map_reason = "prediction_recovery_floor"
        elif native_prediction_gate_rejected:
            self.last_lidar_map_eligible = False
            self.last_lidar_map_reason = (
                f"prediction_gate:{self.last_native_lidar_prediction_gate_reason}"
            )
        elif native_factor is None:
            self.last_lidar_map_eligible = False
            self.last_lidar_map_reason = "native_factor_missing"
        elif not native_factor.correspondences_valid:
            self.last_lidar_map_eligible = False
            self.last_lidar_map_reason = "correspondences_invalid"
        elif not lidar_factor_added:
            self.last_lidar_map_eligible = False
            self.last_lidar_map_reason = "factor_not_added"
        elif not bool(lidar_decision["factor_enabled"]):
            self.last_lidar_map_eligible = False
            self.last_lidar_map_reason = "factor_disabled"
        else:
            self.last_lidar_map_eligible = True
            self.last_lidar_map_reason = "ok"
        if performance_context is not None:
            performance_context["lidar_prediction_recovery_floor"] = bool(
                native_prediction_recovery_floor
            )
            performance_context["lidar_prediction_gate_rejected"] = bool(
                native_prediction_gate_rejected
            )
            performance_context["lidar_factor_weight"] = float(
                lidar_decision.get("reliability_weight", 0.0)
            )
            performance_context["lidar_factor_inflation"] = float(
                lidar_decision.get(
                    "covariance_inflation", MAX_COVARIANCE_INFLATION
                )
            )
            performance_context["lidar_map_eligible"] = bool(
                self.last_lidar_map_eligible
            )
            performance_context["lidar_map_reason"] = str(
                self.last_lidar_map_reason
            )
        self._record_phase_timing("lidar_factor", lidar_factor_started)
        imu_diagnostic_covariance = None
        aux_factors_started = time.perf_counter_ns()
        if self.last_lio_stamp is not None:
            flow_started = time.perf_counter_ns()
            self._flow_factor(
                self.last_lio_stamp, stamp, reference["yaw"],
                previous_index, current_index, reference["delta_position"],
                previous_state=previous_state,
                ordered_imu=cycle_imu_samples,
                native_factor=native_factor,
                current_state=initial_state,
            )
            self._record_phase_timing("flow_factor", flow_started)
            if self.visual_pending_enabled:
                staged_visual_candidates = self._stage_pending_visual_factors(
                    transaction_visual_state_stamps
                )
            else:
                if self._legacy_visual_factor(
                    self.last_lio_stamp, stamp, previous_index, current_index
                ):
                    staged_visual_candidates = [None]
            gnss_age_s = float(stamp) - self.last_gnss_prefit_stamp_s
            gnss_fresh = bool(
                self.last_gnss_prefit_stamp_s > 0.0
                and -self.gnss_future_tolerance_s <= gnss_age_s
                <= self.gnss_max_age_s
            )
            (
                self.last_axis_map_protected,
                self.last_axis_map_protection_sources,
            ) = axis_map_protection(
                self.lidar_axis_observability_latched,
                self.last_gnss_prefit_residual_xyz,
                gnss_fresh=gnss_fresh,
                barometer_active=barometer_active,
                gnss_disagreement_m=(
                    self.axis_map_protection_gnss_disagreement_m
                ),
            )
            if (
                self.axis_map_protection_enabled
                and self.last_lidar_map_eligible
                and np.any(self.last_axis_map_protected)
            ):
                protected_labels = []
                for axis, name in enumerate(("x", "y", "z")):
                    if not self.last_axis_map_protected[axis]:
                        continue
                    self.counts[
                        f"native_lidar_axis_map_protected_{name}"
                    ] += 1
                    protected_labels.append(
                        f"{name}:{self.last_axis_map_protection_sources[axis]}"
                    )
                self.counts[
                    "native_lidar_axis_map_protected_frames"
                ] += 1
                self.last_lidar_map_eligible = False
                self.last_lidar_map_reason = (
                    "axis_protection:" + ",".join(protected_labels)
                )
            if performance_context is not None:
                performance_context["lidar_map_eligible"] = bool(
                    self.last_lidar_map_eligible
                )
                performance_context["lidar_map_reason"] = str(
                    self.last_lidar_map_reason
                )
                performance_context["lidar_axis_weak_xyz"] = (
                    self.lidar_axis_observability_latched.tolist()
                )
                performance_context["lidar_axis_map_protected_xyz"] = (
                    self.last_axis_map_protected.tolist()
                )
            self._update_axis_reliability(stamp)
            if performance_context is not None:
                performance_context["axis_reliability_xyz"] = (
                    self.last_axis_reliability.tolist()
                )
                performance_context["axis_global_reliability_xyz"] = (
                    self.last_axis_global_reliability.tolist()
                )
            self._record_phase_timing("barometer_factor", barometer_started)
            imu_factor_started = time.perf_counter_ns()
            if self.backend_solver_mode == "manifold":
                imu_diagnostic_covariance = self._add_manifold_imu_factor(
                    previous_index, current_index, manifold_measurement
                )
            else:
                imu_diagnostic_covariance = self._imu_factor(
                    self.last_lio_stamp, stamp, reference["orientation"],
                    previous_index, current_index,
                )
            self._record_phase_timing("imu_factor", imu_factor_started)
        self._record_phase_timing("aux_factors", aux_factors_started)
        self._record_phase_timing("prepare", started)
        post_optimize_started = time.perf_counter_ns()
        state_committed = False
        commit_started = None
        try:
            optimize_started = time.perf_counter_ns()
            nonlinear_iteration_budget = select_nonlinear_iteration_budget(
                self.nonlinear_max_iterations,
                self.nonlinear_initialization_max_iterations,
                self.nonlinear_recovery_max_iterations,
                self.backend.state_count,
                recovery_active=(
                    relocalization_applied_now
                    or native_prediction_recovery_floor
                    or str(self.scheduler_health).upper() == "RELOCALIZING"
                ),
            )
            if performance_context is not None:
                performance_context["nonlinear_iteration_budget"] = int(
                    nonlinear_iteration_budget
                )
            if self.backend_solver_mode == "manifold":
                self.backend.optimize(max_iterations=nonlinear_iteration_budget)
            else:
                self.backend.optimize()
            self._record_phase_timing("optimize", optimize_started)
            solve_ms = float(getattr(self.backend, "last_solve_ms", 0.0))
            self.backend_solve_count += 1
            self.backend_solve_ms_total += solve_ms
            self.backend_solve_ms_max = max(
                self.backend_solve_ms_max, solve_ms)
            # A preintegrated delta is linearized at the start-state bias. The
            # first nonlinear solve can move that bias enough to invalidate the
            # delta, especially after a long or dynamic interval. Recompute it
            # once at the optimized start bias, replace the active factor, and
            # solve again. This keeps the window's IMU factor consistent without
            # feeding the FCU's fused pose back into the estimator.
            # A second solve after bias re-integration is useful only when the
            # worker is otherwise caught up. If a newer native frame is
            # already waiting, defer this optional refinement to the next
            # committed cycle so the keyframe clock does not fall further
            # behind the sensor stream. The original IMU factor remains in
            # this transaction; no observation is duplicated.
            reintegration_deferred = False
            try:
                reintegration_deferred = not self.native_work_queue.empty()
            except (AttributeError, RuntimeError):
                reintegration_deferred = False
            if reintegration_deferred and performance_context is not None:
                performance_context["imu_reintegration_deferred"] = True
                self.counts["imu_reintegrations_deferred"] += 1
            if (
                self.backend_solver_mode == "manifold"
                and manifold_measurement is not None
                and self.last_lio_stamp is not None
                and self._manifold_imu_bias_changed(
                    current_index - 1, manifold_measurement
                )
                and not reintegration_deferred
            ):
                updated_measurement = self._manifold_imu_measurement(
                    self.last_lio_stamp,
                    stamp,
                    self.backend.state(current_index - 1),
                    cycle_imu_samples,
                )
                if updated_measurement is not None and self.backend.replace_imu_preintegrated(
                        current_index - 1, current_index, updated_measurement):
                    manifold_measurement = updated_measurement
                    imu_diagnostic_covariance = np.asarray(
                        updated_measurement.covariance, dtype=float
                    )
                    self.counts["imu_reintegrations"] += 1
                    reintegration_started = time.perf_counter_ns()
                    if self.backend_solver_mode == "manifold":
                        self.backend.optimize(
                            max_iterations=(
                                self.nonlinear_reintegration_max_iterations
                            )
                        )
                    else:
                        self.backend.optimize()
                    self._record_phase_timing(
                        "reintegrate", reintegration_started)
                    second_solve_ms = float(
                        getattr(self.backend, "last_solve_ms", 0.0)
                    )
                    self.backend_solve_count += 1
                    self.backend_solve_ms_total += second_solve_ms
                    self.backend_solve_ms_max = max(
                        self.backend_solve_ms_max, second_solve_ms
                    )
            post_optimize_started = time.perf_counter_ns()
            if imu_diagnostic_covariance is not None:
                try:
                    residual = self.backend.latest_factor_residual(
                        "imu_preintegrated", covariance=imu_diagnostic_covariance)
                    if residual is not None:
                        self.last_imu_preintegration_residual_mahalanobis = (
                            residual.mahalanobis_squared
                        )
                        self.counts["imu_residual_updates"] += 1
                        self.last_imu_residual_error = "none"
                    else:
                        self.last_imu_preintegration_residual_mahalanobis = -1.0
                        self.last_imu_residual_error = "factor_not_found"
                except (ValueError, IndexError) as error:
                    self.last_imu_preintegration_residual_mahalanobis = -1.0
                    self.last_imu_residual_error = (
                        f"{type(error).__name__}:{error}"
                    )
                    self.counts["imu_residual_errors"] += 1
            else:
                self.last_imu_preintegration_residual_mahalanobis = -1.0
            if self.backend_solver_mode == "manifold":
                visual_residual = self.backend.latest_factor_rmse(
                    "visual_reprojection"
                )
                if visual_residual is not None:
                    (
                        self.last_visual_reprojection_rmse_normalized,
                        self.last_visual_reprojection_residual_dimension,
                    ) = visual_residual
            estimate = self.backend.state(current_index)
            if self.transactional_update_enabled:
                integrity_started = time.perf_counter_ns()
                self.last_optimization_integrity = validate_optimized_state(
                    initial_state,
                    estimate,
                    self.backend.latest_state_information(),
                    self.backend.last_initial_cost,
                    self.backend.last_cost,
                    **self.optimization_integrity_limits,
                )
                self._record_phase_timing(
                    "integrity_check", integrity_started
                )
                self.optimization_integrity_reason_counts[
                    self.last_optimization_integrity.reason
                ] += 1
                if not self.last_optimization_integrity.valid:
                    self.backend.restore(transaction_snapshot)
                    self.active_transaction_snapshot = None
                    self.counts["visual_solver_rejected"] += len(
                        staged_visual_candidates
                    )
                    self.counts["optimization_rejected"] += 1
                    self.counts["optimization_rollbacks"] += 1
                    self.last_reason = (
                        "optimization_rejected:"
                        f"{self.last_optimization_integrity.reason}"
                    )
                    self.last_callback_ms = (
                        time.perf_counter_ns() - started
                    ) * 1.0e-6
                    self._finish_performance_cycle(
                        performance_context,
                        staged_visual_candidates,
                        state_committed,
                    )
                    return
            commit_started = time.perf_counter_ns()
            publish_started = time.perf_counter_ns()
            self._publish(
                msg.header,
                estimate,
                manifold_measurement,
                frontend_map_candidate,
            )
            eligibility_key = int(round(float(stamp) * 1.0e9))
            self.frontend_map_eligibility_by_stamp[eligibility_key] = (
                bool(self.last_lidar_map_eligible),
                str(self.last_lidar_map_reason),
            )
            self.frontend_map_eligibility_order.append(eligibility_key)
            while (
                len(self.frontend_map_eligibility_order)
                > self.frontend_map_eligibility_capacity
            ):
                expired_key = self.frontend_map_eligibility_order.popleft()
                self.frontend_map_eligibility_by_stamp.pop(expired_key, None)
            self._record_phase_timing("publish", publish_started)
            self.active_transaction_snapshot = None
            self.counts["lio"] += 1
            with self.visual_lock:
                self.visual_state_stamps = deque(
                    transaction_visual_state_stamps, maxlen=self.window_size
                )
            self.counts["visual_solver_accepted"] += len(
                staged_visual_candidates
            )
            self.last_reason = "ok"
            state_committed = True
        except (np.linalg.LinAlgError, ValueError, IndexError) as error:
            if transaction_snapshot is not None:
                self.backend.restore(transaction_snapshot)
                self.active_transaction_snapshot = None
                self.counts["visual_solver_rejected"] += len(
                    staged_visual_candidates
                )
                self.counts["optimization_rollbacks"] += 1
            self.counts["optimization_errors"] += 1
            self.last_reason = f"optimization_error:{type(error).__name__}"
            self.last_exception = f"{type(error).__name__}:{error}"
        self._record_phase_timing("post_optimize", post_optimize_started)
        if state_committed:
            self.last_lio_stamp = stamp
            self.last_lio_position = np.asarray(
                estimate[:3], dtype=float).copy()
            self.last_lio_yaw = float(estimate[5])
            self._update_barometer_trusted_reference(
                stamp, float(estimate[2])
            )
        if commit_started is not None:
            self._record_phase_timing("commit", commit_started)
        self.last_callback_ms = (time.perf_counter_ns() - started) * 1.0e-6
        self._finish_performance_cycle(
            performance_context, staged_visual_candidates, state_committed
        )

    def _publish_frontend_state_seed(self, header, state, covariance):
        if self.frontend_state_seed_pub is None:
            return
        if stamp_seconds(header.stamp) <= 0.0:
            raise ValueError("frontend state seed requires a source timestamp")
        state = np.asarray(state, dtype=float)
        covariance = np.asarray(covariance, dtype=float)
        if state.shape != (15,) or covariance.shape != (15, 15):
            raise ValueError(
                "frontend state seed has an invalid state dimension")
        if np.any(~np.isfinite(state)) or np.any(~np.isfinite(covariance)):
            raise ValueError("frontend state seed must be finite")
        orientation = rpy_to_quaternion_xyzw(state[3:6])
        message = BackendStateSeed()
        message.header = copy.deepcopy(header)
        message.header.frame_id = self.map_frame
        message.map_frame = self.map_frame
        message.body_frame = self.body_frame
        message.sequence = int(self.frontend_state_seed_sequence)
        message.reset_counter = int(self.state_reset_counter)
        support = (
            1.0 if self.reliability_mode == "fixed"
            else self.scheduler_estimator_support
        )
        message.quality = int(round(100.0 * min(1.0, max(0.0, support))))
        message.valid = True
        message.position = [float(value) for value in state[:3]]
        message.orientation_xyzw = [float(value) for value in orientation]
        message.velocity_map = [float(value) for value in state[6:9]]
        message.accel_bias = [float(value) for value in state[9:12]]
        message.gyro_bias = [float(value) for value in state[12:15]]
        message.covariance = [float(value) for value in covariance.ravel()]
        self.frontend_state_seed_pub.publish(message)
        self.frontend_state_seed_sequence += 1
        self.counts["frontend_state_seeds"] += 1

    @staticmethod
    def _state_orientation_xyzw(state):
        return rpy_to_quaternion_xyzw(np.asarray(state[3:6], dtype=float))

    def _publish_scan_request(self, factor):
        if self.scan_prediction_pub is None:
            return
        request = FrontendScanRequest()
        request.header.stamp = ros_time_from_seconds(factor.stamp_s)
        request.header.frame_id = self.map_frame
        request.scan_sequence = int(factor.scan_sequence)
        request.scan_begin_stamp = ros_time_from_seconds(
            float(factor.scan_begin_s))
        request.scan_end_stamp = ros_time_from_seconds(
            float(factor.scan_end_s))
        request.map_frame = self.map_frame
        request.body_frame = self.body_frame
        request.sensor_frame = str(factor.sensor_frame)
        request.point_count = int(factor.candidate_points)
        self.scan_prediction_pub.publish(request)

    def _publish_deskew_trajectory(self, prediction, covariance, reason=None):
        if self.deskew_trajectory_pub is None:
            return
        message = BackendDeskewTrajectory()
        message.header.stamp = ros_time_from_seconds(prediction.scan_end_s)
        message.header.frame_id = self.map_frame
        message.scan_sequence = int(prediction.sequence)
        message.reset_counter = int(self.state_reset_counter)
        message.quality = (
            int(round(100.0 * prediction.quality)) if prediction.valid else 0
        )
        message.valid = bool(prediction.valid)
        message.reason = str(prediction.reason if reason is None else reason)
        message.map_frame = self.map_frame
        message.body_frame = self.body_frame
        message.scan_begin_stamp = ros_time_from_seconds(
            prediction.scan_begin_s)
        message.scan_end_stamp = ros_time_from_seconds(prediction.scan_end_s)
        message.begin_position = [float(value)
                                  for value in prediction.begin_state[:3]]
        message.begin_orientation_xyzw = [
            float(value) for value in self._state_orientation_xyzw(
                prediction.begin_state)]
        message.end_position = [float(value)
                                for value in prediction.end_state[:3]]
        message.end_orientation_xyzw = [
            float(value) for value in self._state_orientation_xyzw(
                prediction.end_state)]
        message.end_velocity_map = [float(value)
                                    for value in prediction.end_state[6:9]]
        message.accel_bias = [float(value)
                              for value in prediction.end_state[9:12]]
        message.gyro_bias = [float(value)
                             for value in prediction.end_state[12:15]]
        matrix = np.asarray(covariance, dtype=float)
        if matrix.shape != (15, 15) or np.any(~np.isfinite(matrix)):
            matrix = np.eye(15, dtype=float)
        message.end_covariance = [float(value) for value in matrix.ravel()]
        self.deskew_trajectory_pub.publish(message)
        self.counts["scan_prediction_published"] += 1

    def _scan_request(self, message):
        """Serialize trajectory N after native factor N-1 is consumed."""
        if not self.frontend_scan_prediction_enabled:
            return
        # A scan request is the earliest reliable heartbeat from the LiDAR
        # frontend. Treat it as activity even before correspondences arrive so
        # publication-only IMU propagation cannot race an in-flight scan.
        now_s = self._now_s()
        self.last_scan_request_arrival_s = now_s
        self.counts["scan_prediction_requests"] += 1
        sequence = int(message.scan_sequence)
        if scan_request_stale(self.last_native_consumed_sequence, sequence):
            self.counts["scan_prediction_stale_requests"] += 1
            return
        skip_missing = False
        with self.pending_scan_request_lock:
            if not scan_request_ready(
                self.last_native_consumed_sequence, sequence
            ):
                if sequence in self.pending_scan_requests:
                    self.counts["scan_prediction_duplicate_requests"] += 1
                first_seen_s = self.pending_scan_request_first_seen_s.setdefault(
                    sequence, now_s
                )
                self.pending_scan_requests[sequence] = copy.deepcopy(message)
                self.counts["scan_prediction_deferred"] += 1
                if (
                    now_s >= first_seen_s
                    and now_s - first_seen_s
                    >= self.scan_prediction_missing_factor_grace_s
                ):
                    missing_count = max(
                        0, sequence - self.last_native_consumed_sequence - 1
                    )
                    self.last_native_consumed_sequence = sequence - 1
                    self.counts["scan_prediction_missing_factor_skips"] += (
                        missing_count
                    )
                    self.pending_scan_requests.pop(sequence, None)
                    self.pending_scan_request_first_seen_s.pop(sequence, None)
                    skip_missing = True
                else:
                    return
        if skip_missing:
            self.last_scan_prediction_reason = "missing_predecessor_skipped"
        if sequence in self.scan_prediction_by_sequence:
            self.counts["scan_prediction_duplicate_requests"] += 1
        self._produce_scan_prediction(message)

    def _release_pending_scan_requests(self):
        ready = []
        with self.pending_scan_request_lock:
            for sequence in sorted(self.pending_scan_requests):
                if scan_request_ready(
                    self.last_native_consumed_sequence, sequence
                ):
                    ready.append(self.pending_scan_requests.pop(sequence))
                    self.pending_scan_request_first_seen_s.pop(sequence, None)
        for message in ready:
            self.counts["scan_prediction_deferred_released"] += 1
            self._produce_scan_prediction(message)

    def _produce_scan_prediction(self, message):
        """Produce one trajectory against the latest committed backend state."""
        sequence = int(message.scan_sequence)
        begin_s = stamp_seconds(message.scan_begin_stamp)
        end_s = stamp_seconds(message.scan_end_stamp)
        anchor = self._optimization_anchor_snapshot()
        if anchor is None:
            self.counts["scan_prediction_rejected"] += 1
            self.last_scan_prediction_reason = "backend_state_unavailable"
            return
        previous_stamp_s = float(anchor.stamp_s)
        previous_state = np.asarray(anchor.state, dtype=float)
        anchor_covariance = np.asarray(
            anchor.covariance,
            dtype=float).reshape(
            15,
            15)
        cached = self.scan_prediction_by_sequence.get(sequence)
        if cached is not None:
            reusable, _cached_reason = prediction_reusable(
                cached,
                sequence=sequence,
                previous_stamp_s=previous_stamp_s,
                scan_end_s=end_s,
                current_previous_state=previous_state,
                timestamp_tolerance_s=self.scan_prediction_timestamp_tolerance_s,
                state_tolerance=self.scan_prediction_state_tolerance,
            )
            begin_matches = (
                abs(float(cached.scan_begin_s) - begin_s)
                <= self.scan_prediction_timestamp_tolerance_s
            )
            if reusable and begin_matches:
                self.counts["scan_prediction_cache_hits"] += 1
                cached_propagated = propagate_optimization_anchor(
                    anchor, end_s, cached.measurement
                )
                self._publish_deskew_trajectory(
                    cached,
                    np.asarray(
                        cached_propagated.covariance, dtype=float
                    ).reshape(15, 15),
                    reason="cached",
                )
                return
            self.scan_prediction_by_sequence.pop(sequence, None)
            self.counts["scan_prediction_reuse_rejected"] += 1
        prediction = build_scan_prediction(
            sequence,
            previous_stamp_s,
            begin_s,
            end_s,
            previous_state,
            ordered_imu_samples(self._imu_snapshot()),
            maximum_begin_gap_s=self.scan_prediction_maximum_begin_gap_s,
            nominal_imu_gap_s=self.imu_nominal_gap_s,
            maximum_imu_gap_s=self.imu_max_gap_s,
        )
        prediction_covariance = anchor_covariance
        if prediction.valid:
            finalized_measurement = self._finalize_manifold_imu_measurement(
                prediction.measurement
            )
            prediction = replace(
                prediction, measurement=finalized_measurement
            )
            propagated = propagate_optimization_anchor(
                anchor, end_s, finalized_measurement
            )
            prediction_covariance = np.asarray(
                propagated.covariance, dtype=float
            ).reshape(15, 15)
        self.scan_prediction_by_sequence[sequence] = prediction
        while len(
                self.scan_prediction_by_sequence) > self.scan_prediction_cache_size:
            self.scan_prediction_by_sequence.pop(
                next(iter(self.scan_prediction_by_sequence)))
        if not prediction.valid:
            self.counts["scan_prediction_rejected"] += 1
            self.last_scan_prediction_reason = prediction.reason
            self._publish_deskew_trajectory(
                prediction, anchor_covariance, reason=prediction.reason
            )
            return
        self.last_scan_prediction_reason = "ok"
        self._publish_deskew_trajectory(prediction, prediction_covariance)

    @staticmethod
    def _fallback_state_covariance():
        return np.diag(np.asarray(
            [0.05 ** 2, 0.05 ** 2, 0.10 ** 2]
            + [0.03 ** 2] * 3
            + [0.50 ** 2, 0.50 ** 2, 0.75 ** 2]
            + [0.50 ** 2] * 3
            + [0.05 ** 2] * 3,
            dtype=float,
        ))

    def _optimized_state_covariance(self, output_stamp_s, measurement):
        state_covariance = None
        if self.backend_solver_mode == "manifold":
            update_due = covariance_update_due(
                self.last_covariance_stamp_s,
                output_stamp_s,
                self.marginal_covariance_update_period_s,
            )
            if update_due:
                try:
                    candidate = self.backend.marginal_covariance(-1)
                    if candidate.shape != (
                            15, 15) or np.any(
                            ~np.isfinite(candidate)):
                        raise ValueError(
                            "optimizer returned an invalid covariance")
                    self.last_state_covariance = candidate
                    self.last_covariance_stamp_s = output_stamp_s
                    self.counts["marginal_covariance_updates"] += 1
                    self.last_covariance_source = "window_marginal"
                except (ValueError, IndexError, np.linalg.LinAlgError) as error:
                    self.counts["marginal_covariance_errors"] += 1
                    self.last_covariance_source = "fixed_fallback"
                    self.last_exception = f"covariance_{type(error).__name__}:{error}"
            elif measurement is not None:
                anchor = self._optimization_anchor_snapshot()
                if anchor is not None:
                    try:
                        predicted = propagate_optimization_anchor(
                            anchor, output_stamp_s, measurement
                        )
                        self.last_state_covariance = np.asarray(
                            predicted.covariance, dtype=float
                        ).reshape(15, 15)
                        self.counts["anchor_covariance_propagations"] += 1
                        self.last_covariance_source = "imu_propagated_anchor"
                    except (ValueError, np.linalg.LinAlgError):
                        self.counts["marginal_covariance_reuses"] += 1
                        self.last_covariance_source = "window_marginal_cached"
            elif self.last_state_covariance is not None:
                self.counts["marginal_covariance_reuses"] += 1
                self.last_covariance_source = "window_marginal_cached"
            state_covariance = self.last_state_covariance
        if state_covariance is None:
            state_covariance = self._fallback_state_covariance()
        state_covariance = np.asarray(state_covariance, dtype=float)
        if state_covariance.shape != (
                15, 15) or np.any(
                ~np.isfinite(state_covariance)):
            raise ValueError("backend state covariance must be finite 15x15")
        return 0.5 * (state_covariance + state_covariance.T)

    def _build_odometry(self, header, state, state_covariance):
        state = np.asarray(state, dtype=float)
        state_covariance = np.asarray(state_covariance, dtype=float)
        if state.shape != (15,) or np.any(~np.isfinite(state)):
            raise ValueError("backend output state must be a finite 15-vector")
        if state_covariance.shape != (
                15, 15) or np.any(
                ~np.isfinite(state_covariance)):
            raise ValueError("backend output covariance must be finite 15x15")
        orientation = np.asarray(state[3:6], dtype=float)
        qx, qy, qz, qw = rpy_to_quaternion_xyzw(orientation)
        body_velocity = rpy_to_rotation_matrix(orientation).T @ np.asarray(
            state[6:9], dtype=float
        )
        output = Odometry()
        output.header = copy.deepcopy(header)
        output.header.frame_id = self.map_frame
        output.child_frame_id = self.body_frame
        output.pose.pose.position.x = float(state[0])
        output.pose.pose.position.y = float(state[1])
        output.pose.pose.position.z = float(state[2])
        output.pose.pose.orientation.x = qx
        output.pose.pose.orientation.y = qy
        output.pose.pose.orientation.z = qz
        output.pose.pose.orientation.w = qw
        output.twist.twist.linear.x = float(body_velocity[0])
        output.twist.twist.linear.y = float(body_velocity[1])
        output.twist.twist.linear.z = float(body_velocity[2])
        pose_covariance, velocity_covariance = (
            state_covariance_to_odometry_covariances(state, state_covariance)
        )
        output.pose.covariance = [float(value)
                                  for value in pose_covariance.ravel()]
        for row in range(3):
            for column in range(3):
                output.twist.covariance[row * 6 + column] = float(
                    velocity_covariance[row, column]
                )
        # Angular velocity is not part of the optimized state.
        output.twist.covariance[21] = 1.0e6
        output.twist.covariance[28] = 1.0e6
        output.twist.covariance[35] = 1.0e6
        return output

    def _publish_unified_odom(self, output, source):
        output_stamp_s = stamp_seconds(output.header.stamp)
        if output_stamp_s <= 0.0:
            raise ValueError("backend output requires a source timestamp")
        if not self._scan_prediction_contract_allows_output():
            self.counts["scan_prediction_contract_output_suppressed"] += 1
            return False
        with self.output_lock:
            admitted, reason = unified_odom_publication_decision(
                getattr(self, "unified_odom_output_mode", "legacy_hybrid"),
                source,
                output_stamp_s,
                self.last_unified_output_stamp_s,
            )
            if not admitted:
                if source == "optimized":
                    if reason == "source_not_owner":
                        self.counts["optimized_odom_mode_suppressed"] += 1
                    else:
                        self.counts[
                            "optimized_odom_nonmonotonic_suppressed"
                        ] += 1
                return False
            pose_diagonal = [
                float(output.pose.covariance[index])
                for index in (0, 7, 14, 21, 28, 35)
            ]
            velocity_diagonal = [
                float(output.twist.covariance[index])
                for index in (0, 7, 14)
            ]
            self.last_output_position_variance_m2 = max(pose_diagonal[:3])
            self.last_output_orientation_variance_rad2 = max(
                pose_diagonal[3:]
            )
            self.last_output_velocity_variance_m2ps2 = max(
                velocity_diagonal
            )
            self.maximum_output_position_variance_m2 = max(
                getattr(self, "maximum_output_position_variance_m2", 0.0),
                self.last_output_position_variance_m2,
            )
            self.maximum_output_orientation_variance_rad2 = max(
                getattr(
                    self,
                    "maximum_output_orientation_variance_rad2",
                    0.0,
                ),
                self.last_output_orientation_variance_rad2,
            )
            self.odom_pub.publish(output)
            self.last_unified_output_stamp_s = output_stamp_s
            self.last_output = output
            self.last_output_source = str(source)
            self.last_output_source_age_s = self._now_s() - output_stamp_s
            self.counts["published"] += 1
            if source == "optimized":
                self.counts["optimized_odom_published"] += 1
            elif source == "imu_propagated":
                self.counts["live_propagation_published"] += 1
            return True

    def _publish_live_odom(self, header, state, state_covariance):
        output = self._build_odometry(header, state, state_covariance)
        return self._publish_unified_odom(output, "imu_propagated")

    def _publish(
            self, header, state, measurement=None,
            frontend_map_candidate=None):
        output_stamp_s = stamp_seconds(header.stamp)
        if output_stamp_s <= 0.0:
            raise ValueError("backend output requires a source timestamp")
        state = np.asarray(state, dtype=float)
        orientation = np.asarray(state[3:6], dtype=float)
        with self.state_publication_lock:
            state_covariance = self._optimized_state_covariance(
                output_stamp_s, measurement
            )
            local_output = self._build_odometry(
                header, state, state_covariance
            )
            output = local_output
            self._commit_optimization_anchor(
                output_stamp_s, state, state_covariance
            )
            if getattr(
                self, "unified_odom_output_mode", "legacy_hybrid"
            ) == "legacy_hybrid":
                self._publish_unified_odom(output, "optimized")
            else:
                self.counts["optimized_odom_anchor_only"] += 1
            self.frontend_activation_pose_pub.publish(
                frontend_activation_odometry(
                    local_output, self.map_frame, self.body_frame
                )
            )
            self.counts["frontend_activation_published"] += 1
        if (
            getattr(self, "unified_odom_output_mode", "legacy_hybrid")
            == "lidar_event_propagated"
            and self.live_propagation_enabled
        ):
            # Propagation runs after the anchor lock is released. The IMU
            # callback remains ingestion-only and never performs this work.
            self._publish_live_propagation(_event_triggered=True)
        map_output = local_output
        map_lidar_eligible = bool(self.last_lidar_map_eligible)
        map_lidar_reason = str(self.last_lidar_map_reason)
        if self.frontend_map_commit_delay_states > 0:
            if frontend_map_candidate is None:
                self.last_frontend_map_pose_reason = (
                    "waiting_for_stabilized_state"
                )
                self.last_frontend_map_position_variance_m2 = math.inf
                self.last_frontend_map_orientation_variance_rad2 = math.inf
                self.last_frontend_map_pose_delay_s = -1.0
                self.counts["frontend_map_pose_waiting"] += 1
                map_output = None
            else:
                (
                    map_stamp_s,
                    map_state,
                    map_lidar_eligible,
                    map_lidar_reason,
                ) = frontend_map_candidate
                map_header = copy.deepcopy(header)
                map_header.stamp = ros_time_from_seconds(map_stamp_s)
                map_header.frame_id = self.map_frame
                map_output = self._build_odometry(
                    map_header, map_state, state_covariance
                )
                self.last_frontend_map_pose_delay_s = max(
                    0.0, output_stamp_s - map_stamp_s
                )
        if map_output is not None:
            map_allowed, map_reason, position_variance, orientation_variance = (
                frontend_map_commit_decision(
                    self.scheduler_health,
                    self._age_s(self._now_s(), self.scheduler_arrival),
                    self.scheduler_timeout_s,
                    map_lidar_eligible,
                    map_output.pose.covariance,
                    self.frontend_map_commit_allowed_health_states,
                    self.frontend_map_max_position_variance_m2,
                    self.frontend_map_max_orientation_variance_rad2,
                )
            )
            if not map_lidar_eligible and map_reason == "lidar_factor_rejected":
                map_reason = map_lidar_reason
            self.last_frontend_map_pose_reason = map_reason
            self.last_frontend_map_position_variance_m2 = position_variance
            self.last_frontend_map_orientation_variance_rad2 = orientation_variance
            map_stamp_s = stamp_seconds(map_output.header.stamp)
            if (
                map_allowed
                and self.last_frontend_map_pose_stamp_s is not None
                and map_stamp_s <= self.last_frontend_map_pose_stamp_s
            ):
                map_allowed = False
                self.last_frontend_map_pose_reason = "duplicate_or_nonmonotonic"
                self.counts["frontend_map_pose_duplicates"] += 1
            if map_allowed:
                self.frontend_map_pose_pub.publish(map_output)
                self.last_frontend_map_pose_stamp_s = map_stamp_s
                self.counts["frontend_map_pose_published"] += 1
            elif self.last_frontend_map_pose_reason != "duplicate_or_nonmonotonic":
                self.counts["frontend_map_pose_rejected"] += 1
        self._publish_frontend_state_seed(header, state, state_covariance)
        if path_sample_due(
            self.last_path_sample_position,
            self.last_path_sample_orientation,
            state[:3],
            orientation,
            self.path_minimum_translation_m,
            self.path_minimum_rotation_rad,
        ):
            pose = PoseStamped()
            pose.header = copy.deepcopy(output.header)
            pose.pose = copy.deepcopy(output.pose.pose)
            self.path.poses.append(pose)
            if len(self.path.poses) > self.max_path:
                self.path.poses = self.path.poses[-self.max_path:]
            self.last_path_sample_position = np.asarray(
                state[:3], dtype=float).copy()
            self.last_path_sample_orientation = orientation.copy()
            self.counts["path_samples"] += 1
        if covariance_update_due(
            self.last_path_publish_stamp_s,
            output_stamp_s,
            self.path_publish_period_s,
        ):
            self.path.header = copy.deepcopy(output.header)
            self.path_pub.publish(self.path)
            self.last_path_publish_stamp_s = output_stamp_s
            self.counts["path_messages"] += 1
        return state_covariance

    @staticmethod
    def _key(name, value):
        item = KeyValue()
        item.key = str(name)
        item.value = str(value)
        return item

    def _record_imu_invalid(self, reason):
        reason = str(reason)
        self.counts["imu_invalid"] += 1
        self.imu_invalid_reasons[reason] = (
            self.imu_invalid_reasons.get(reason, 0) + 1
        )

    def log_final_summary(self):
        try:
            self._write_performance_trace()
        except OSError as error:
            print(f"Performance trace write failed: {type(error).__name__}:{error}")
        finally:
            if self.gc_profiler is not None:
                self.gc_profiler.close()
        average_solve_ms = (
            self.backend_solve_ms_total / self.backend_solve_count
            if self.backend_solve_count > 0 else 0.0
        )
        flow_los_residual_mean = (
            float(np.mean(self.flow_los_residual_norms))
            if self.flow_los_residual_norms else -1.0
        )
        flow_los_residual_no_lever_mean = (
            float(np.mean(self.flow_los_residual_no_lever_norms))
            if self.flow_los_residual_no_lever_norms else -1.0
        )
        flow_los_residual_p95 = (
            float(np.percentile(self.flow_los_residual_norms, 95))
            if self.flow_los_residual_norms else -1.0
        )
        flow_los_residual_no_lever_p95 = (
            float(np.percentile(self.flow_los_residual_no_lever_norms, 95))
            if self.flow_los_residual_no_lever_norms else -1.0
        )
        flow_los_lever_mean = (
            float(np.mean(self.flow_los_lever_arm_norms))
            if self.flow_los_lever_arm_norms else -1.0
        )
        flow_los_lever_p95 = (
            float(np.percentile(self.flow_los_lever_arm_norms, 95))
            if self.flow_los_lever_arm_norms else -1.0
        )
        flow_lever_displacement_mean = (
            float(np.mean(self.flow_lever_arm_displacement_norms))
            if self.flow_lever_arm_displacement_norms else -1.0
        )
        flow_lever_displacement_p95 = (
            float(np.percentile(self.flow_lever_arm_displacement_norms, 95))
            if self.flow_lever_arm_displacement_norms else -1.0
        )
        ownership = (
            "native_relinearized="
            f"{self.counts['native_lidar_relinearized']};"
            "native_condensed="
            f"{self.counts['native_lidar_condensed_fallbacks']};"
            "lio_pose_fallbacks="
            f"{self.counts['native_lidar_pose_fallbacks']};"
            "lio_pose_ignored="
            f"{self.counts['lio_pose_inputs_ignored']};"
            "imu_propagated="
            f"{self.counts['imu_propagated_initializations']};"
            f"imu_factors={self.counts['imu_factors']};"
            f"imu_pair_timeouts={self.counts['imu_pair_timeouts']};"
            f"imu_reintegrations={self.counts['imu_reintegrations']};"
            "imu_reintegrations_deferred="
            f"{self.counts['imu_reintegrations_deferred']};"
            f"calibration_accepted={self.counts['calibration_accepted']};"
            "calibration_motion_received="
            f"{self.counts['calibration_motion_received']};"
            "calibration_motion_rejected="
            f"{self.counts['calibration_motion_rejected']};"
            "calibration_seed_initialized="
            f"{self.counts['calibration_seed_initialized']};"
            "calibration_seed_lock_busy="
            f"{self.counts['calibration_seed_lock_busy']};"
            "calibration_mode="
            f"{self.calibration_mode};"
            f"calibration_time_offset_s={self.last_calibration_update.time_offset_s:.6f}")
        invalid_reasons = ",".join(
            f"{name}:{count}"
            for name, count in sorted(self.imu_invalid_reasons.items())
        ) or "none"
        integrity_reasons = ",".join(
            f"{name}:{count}"
            for name, count in sorted(
                self.optimization_integrity_reason_counts.items()
            )
        ) or "none"
        print(
            "Unified backend final summary: "
            f"solver={self.backend_solver_mode};"
            f"input_trigger={self.input_trigger_mode};"
            f"last_state_trigger={self.last_state_trigger_source};"
            "optimized_states_committed="
            f"{self.counts['optimized_states_committed']};"
            "unified_odom_output_mode="
            f"{self.unified_odom_output_mode};"
            "optimized_odom_published="
            f"{self.counts['optimized_odom_published']};"
            "optimized_odom_anchor_only="
            f"{self.counts['optimized_odom_anchor_only']};"
            "optimized_odom_nonmonotonic_suppressed="
            f"{self.counts['optimized_odom_nonmonotonic_suppressed']};"
            "optimized_odom_mode_suppressed="
            f"{self.counts['optimized_odom_mode_suppressed']};"
            "frontend_activation_published="
            f"{self.counts['frontend_activation_published']};"
            "live_propagation_published="
            f"{self.counts['live_propagation_published']};"
            "live_propagation_rejected="
            f"{self.counts['live_propagation_rejected']};"
            "auxiliary_keyframe_attempts="
            f"{self.counts['auxiliary_keyframe_attempts']};"
            "auxiliary_keyframe_committed="
            f"{self.counts['auxiliary_keyframe_committed']};"
            "auxiliary_keyframe_rejected="
            f"{self.counts['auxiliary_keyframe_rejected']};"
            "auxiliary_keyframe_errors="
            f"{self.counts['auxiliary_keyframe_errors']};"
            "output_position_variance_m2="
            f"{self.last_output_position_variance_m2:.9g};"
            "output_orientation_variance_rad2="
            f"{self.last_output_orientation_variance_rad2:.9g};"
            "output_velocity_variance_m2ps2="
            f"{self.last_output_velocity_variance_m2ps2:.9g};"
            "maximum_output_position_variance_m2="
            f"{self.maximum_output_position_variance_m2:.9g};"
            "maximum_output_orientation_variance_rad2="
            f"{self.maximum_output_orientation_variance_rad2:.9g};"
            "maximum_auxiliary_position_variance_m2="
            f"{self.maximum_auxiliary_position_variance_m2:.9g};"
            f"published={self.counts['published']};"
            f"last_reason={self.last_reason};"
            f"optimization_errors={self.counts['optimization_errors']};"
            f"optimization_rejected={self.counts['optimization_rejected']};"
            f"optimization_rollbacks={self.counts['optimization_rollbacks']};"
            f"optimization_integrity_counts={integrity_reasons};"
            f"frontend_state_seeds={self.counts['frontend_state_seeds']};"
            f"solve_mean_ms={average_solve_ms:.3f};"
            f"solve_max_ms={self.backend_solve_ms_max:.3f};{ownership};"
            f"imu_invalid={self.counts['imu_invalid']};"
            f"imu_invalid_reasons={invalid_reasons};"
            f"imu_received={self.counts['imu_received']};"
            f"imu_startup_reason={self.last_imu_startup_reason};"
            "imu_startup_samples="
            f"{self.last_imu_startup_sample_count};"
            f"imu_startup_span_s={self.last_imu_startup_span_s:.6f};"
            "imu_startup_accel_bias="
            f"{','.join(f'{value:.9g}' for value in self.last_imu_startup_accel_bias)};"
            "imu_startup_gyro_bias="
            f"{','.join(f'{value:.9g}' for value in self.last_imu_startup_gyro_bias)};"
            "imu_startup_bias_accepted="
            f"{self.counts['imu_startup_bias_accepted']};"
            "imu_startup_bias_rejected="
            f"{self.counts['imu_startup_bias_rejected']};"
            "imu_nonmonotonic_arrivals="
            f"{self.counts['imu_nonmonotonic_arrivals']};"
            "imu_max_positive_arrival_gap_s="
            f"{self.imu_max_positive_arrival_gap_s:.6f};"
            f"native_pair_timeouts={self.counts['native_lidar_pair_timeouts']};"
            f"native_received={self.counts['native_lidar_received']};"
            f"native_queue_overflow={self.counts['native_worker_queue_overflow']};"
            "native_queue_superseded="
            f"{self.counts['native_worker_queue_superseded']};"
            "native_queue_discarded="
            f"{self.counts['native_worker_queue_discarded']};"
            "native_latest_skipped="
            f"{self.counts['native_worker_latest_skipped']};"
            f"native_worker_errors={self.counts['native_worker_errors']};"
            f"native_rank={self.last_native_effective_rank};"
            f"native_translation_rank={self.last_native_translation_rank};"
            f"native_rotation_rank={self.last_native_rotation_rank};"
            "native_condition="
            f"{self.last_native_condition_number:.9g};"
            "native_degenerate_frames="
            f"{self.counts['native_lidar_directionally_degenerate']};"
            "native_prediction_gate_rejections="
            f"{self.counts['native_lidar_prediction_gate_rejections']};"
            "native_prediction_gate_recoveries="
            f"{self.counts['native_lidar_prediction_gate_recoveries']};"
            "native_prediction_recovery_factors="
            f"{self.counts['native_lidar_prediction_recovery_factors']};"
            "native_prediction_gate_consecutive_rejections="
            f"{self.native_lidar_prediction_gate_consecutive_rejections};"
            "native_prediction_gate_reason="
            f"{self.last_native_lidar_prediction_gate_reason};"
            "native_epoch_stale_rejected="
            f"{self.counts['native_lidar_epoch_stale_rejected']};"
            "native_epoch_future_rejected="
            f"{self.counts['native_lidar_epoch_future_rejected']};"
            "native_factor_reset_counter="
            f"{self.last_native_factor_reset_counter};"
            "native_prediction_gate_latched="
            f"{int(self.native_lidar_prediction_gate_latched)};"
            f"scan_requests={self.counts['scan_prediction_requests']};"
            f"scan_predictions={self.counts['scan_prediction_published']};"
            f"scan_rejected={self.counts['scan_prediction_rejected']};"
            f"scan_last_reason={self.last_scan_prediction_reason};"
            f"scan_deferred={self.counts['scan_prediction_deferred']};"
            "scan_deferred_released="
            f"{self.counts['scan_prediction_deferred_released']};"
            "scan_duplicate_requests="
            f"{self.counts['scan_prediction_duplicate_requests']};"
            "scan_stale_requests="
            f"{self.counts['scan_prediction_stale_requests']};"
            "native_consumed_without_state_commit="
            f"{self.counts['native_consumed_without_state_commit']};"
            f"scan_cache_hits={self.counts['scan_prediction_cache_hits']};"
            f"scan_cache_misses={self.counts['scan_prediction_cache_misses']};"
            "scan_reuse_rejected="
            f"{self.counts['scan_prediction_reuse_rejected']};"
            "frontend_map_pose_published="
            f"{self.counts['frontend_map_pose_published']};"
            "frontend_map_pose_rejected="
            f"{self.counts['frontend_map_pose_rejected']};"
            "frontend_map_pose_waiting="
            f"{self.counts['frontend_map_pose_waiting']};"
            "frontend_map_pose_duplicates="
            f"{self.counts['frontend_map_pose_duplicates']};"
            "native_axis_map_protected_frames="
            f"{self.counts['native_lidar_axis_map_protected_frames']};"
            "frontend_map_pose_delay_states="
            f"{self.frontend_map_commit_delay_states};"
            "frontend_map_pose_delay_s="
            f"{self.last_frontend_map_pose_delay_s:.6f};"
            f"frontend_map_pose_reason={self.last_frontend_map_pose_reason};"
            f"gnss_received={self.counts['gnss_received']};"
            f"gnss_consumed={self.counts['gnss_consumed']};"
            f"gnss_records={self.counts['gnss_factor_records']};"
            f"gnss_factors={self.counts['gnss_factors']};"
            f"gnss_scheduler_disabled={self.counts['gnss_disabled_scheduler']};"
            f"gnss_nis_rejected={self.counts['gnss_rejected_nis']};"
            f"gnss_xy_nis_rejected={self.counts['gnss_xy_rejected_nis']};"
            f"gnss_z_nis_rejected={self.counts['gnss_z_rejected_nis']};"
            "gnss_xy_robust_downweighted="
            f"{self.counts['gnss_xy_robust_downweighted']};"
            "gnss_z_robust_downweighted="
            f"{self.counts['gnss_z_robust_downweighted']};"
            "gnss_z_reanchor_factors="
            f"{self.counts['gnss_z_reanchor_factors']};"
            "gnss_z_recovery_factors="
            f"{self.counts['gnss_z_recovery_factors']};"
            "gnss_all_axes_inconsistent="
            f"{self.counts['gnss_all_axes_inconsistent']};"
            "gnss_prefit_recovery_floor="
            f"{self.counts['gnss_prefit_recovery_floor']};"
            f"gnss_last_reason={self.last_gnss_admission_reason};"
            f"gnss_duplicates={self.counts['gnss_duplicates']};"
            f"gnss_stale={self.counts['gnss_stale_discarded']};"
            f"flow_received={self.counts['flow_received']};"
            f"flow_attempts={self.counts['flow_factor_attempts']};"
            f"flow_factors={self.counts['flow_factors']};"
            "flow_scheduler_disabled="
            f"{self.counts['flow_disabled_scheduler']};"
            f"flow_disabled_quality={self.counts['flow_disabled_quality']};"
            f"flow_disabled_speed={self.counts['flow_disabled_speed']};"
            f"flow_disabled_rotation={self.counts['flow_disabled_rotation']};"
            f"flow_clock_mismatch={self.counts['flow_clock_mismatch']};"
            f"visual_received={self.counts['visual_received']};"
            f"visual_attempts={self.counts['visual_factor_attempts']};"
            f"visual_factors={self.counts['visual_factors']};"
            "visual_factor_scores_received="
            f"{self.counts['visual_factor_scores_received']};"
            "visual_factor_scores_matched="
            f"{self.counts['visual_factor_score_matched']};"
            "visual_factor_score_waits="
            f"{self.counts['visual_factor_score_waits']};"
            "visual_factor_score_missing="
            f"{self.counts['visual_factor_score_missing']};"
            f"visual_rejected_time={self.counts['visual_rejected_time']};"
            f"visual_rejected_tracks={self.counts['visual_rejected_tracks']};"
            "visual_prebootstrap_dropped="
            f"{self.counts['visual_prebootstrap_dropped']};"
            "visual_pending_pre_window_dropped="
            f"{self.counts['visual_pending_pre_window_dropped']};"
            "visual_state_consistency_rejected="
            f"{self.counts['visual_state_consistency_rejected']};"
            "visual_pnp_observability_rejected="
            f"{self.counts['visual_pnp_observability_rejected']};"
            "visual_initialized="
            f"{int(self.visual_initializer.ready)};"
            "visual_initialization_batches="
            f"{self.visual_initializer.consecutive_batches};"
            "visual_time_offset_s="
            f"{self._effective_visual_time_offset_s():.9g};"
            "visual_time_offset_locked="
            f"{int(self.last_visual_time_calibration.locked)};"
            "visual_time_calibration_reason="
            f"{self.last_visual_time_calibration.reason};"
            f"visual_last_reason={self.last_visual_reason};"
            "visual_factor_score_match_error_s="
            f"{self.last_visual_factor_score_match_error_s:.9g};"
            "visual_prefit_rmse_normalized="
            f"{self.last_visual_prefit_rmse_normalized:.9g};"
            f"visual_prefit_rmse_px={self.last_visual_prefit_rmse_px:.9g};"
            "visual_prefit_valid_track_ratio="
            f"{self.last_visual_prefit_valid_track_ratio:.9g};"
            "visual_prefit_jacobian_rank="
            f"{self.last_visual_prefit_jacobian_rank};"
            "visual_prefit_jacobian_condition="
            f"{self.last_visual_prefit_jacobian_condition:.9g};"
            "visual_prefit_nis_per_dof="
            f"{self.last_visual_prefit_nis_per_dof:.9g};"
            "visual_pnp_inlier_ratio="
            f"{self.last_visual_pnp_inlier_ratio:.9g};"
            "visual_pnp_information_rank="
            f"{self.last_visual_pnp_information_rank};"
            "visual_pnp_condition_number="
            f"{self.last_visual_pnp_condition_number:.9g};"
            "visual_batch_information_scale="
            f"{self.last_visual_batch_information_scale:.9g};"
            f"rgbd_depth_reason={self.last_rgbd_depth_reason};"
            f"rgbd_depth_tracks={self.last_rgbd_depth_track_count};"
            "rgbd_depth_prefit_rmse_m="
            f"{self.last_rgbd_depth_prefit_rmse_m:.9g};"
            f"rgbd_direct_reason={self.last_rgbd_direct_reason};"
            f"rgbd_direct_tracks={self.last_rgbd_direct_track_count};"
            "rgbd_direct_depth_rmse_m="
            f"{self.last_rgbd_direct_depth_rmse_m:.9g};"
            "rgbd_direct_photometric_rmse="
            f"{self.last_rgbd_direct_photometric_rmse:.9g};"
            "rgbd_direct_photometric_information_scale="
            f"{self.last_rgbd_direct_photometric_information_scale:.9g};"
            "visual_reprojection_rmse_normalized="
            f"{self.last_visual_reprojection_rmse_normalized:.9g};"
            "visual_reprojection_residual_dimension="
            f"{self.last_visual_reprojection_residual_dimension};"
            f"flow_last_reason={self.last_flow_reason};"
            f"flow_rotation_phase={self.last_flow_rotation_phase};"
            f"flow_los_samples={self.counts['flow_los_diagnostic_samples']};"
            f"flow_los_invalid={self.counts['flow_los_diagnostic_invalid']};"
            f"flow_los_residual_mean_radps={flow_los_residual_mean:.9g};"
            f"flow_los_residual_p95_radps={flow_los_residual_p95:.9g};"
            "flow_los_residual_no_lever_mean_radps="
            f"{flow_los_residual_no_lever_mean:.9g};"
            "flow_los_residual_no_lever_p95_radps="
            f"{flow_los_residual_no_lever_p95:.9g};"
            f"flow_los_lever_mean_radps={flow_los_lever_mean:.9g};"
            f"flow_los_lever_p95_radps={flow_los_lever_p95:.9g};"
            f"flow_lever_arm_displacement_mean_m={flow_lever_displacement_mean:.9g};"
            f"flow_lever_arm_displacement_p95_m={flow_lever_displacement_p95:.9g};"
            f"prepare_mean_ms={self._phase_mean_ms('prepare'):.3f};"
            f"prepare_max_ms={self.phase_timing['prepare']['max_ms']:.3f};"
            f"pre_state_mean_ms={self._phase_mean_ms('pre_state'):.3f};"
            f"snapshot_mean_ms={self._phase_mean_ms('snapshot'):.3f};"
            f"add_state_mean_ms={self._phase_mean_ms('add_state'):.3f};"
            f"lidar_factor_mean_ms={self._phase_mean_ms('lidar_factor'):.3f};"
            f"aux_factors_mean_ms={self._phase_mean_ms('aux_factors'):.3f};"
            f"optimize_mean_ms={self._phase_mean_ms('optimize'):.3f};"
            f"optimize_max_ms={self.phase_timing['optimize']['max_ms']:.3f};"
            f"reintegrate_mean_ms={self._phase_mean_ms('reintegrate'):.3f};"
            f"reintegrate_max_ms={self.phase_timing['reintegrate']['max_ms']:.3f};"
            f"post_optimize_mean_ms={self._phase_mean_ms('post_optimize'):.3f};"
            f"post_optimize_max_ms={self.phase_timing['post_optimize']['max_ms']:.3f};"
            f"publish_mean_ms={self._phase_mean_ms('publish'):.3f};"
            f"publish_max_ms={self.phase_timing['publish']['max_ms']:.3f};"
            f"covariance_source={self.last_covariance_source};"
            "marginal_covariance_errors="
            f"{self.counts['marginal_covariance_errors']}", flush=True, )

    def _diagnostics(self):
        self._scan_prediction_contract_allows_output()
        average_solve_ms = (
            self.backend_solve_ms_total / self.backend_solve_count
            if self.backend_solve_count else 0.0
        )
        flow_los_residual_p95 = (
            float(np.percentile(self.flow_los_residual_norms, 95))
            if self.flow_los_residual_norms else -1.0
        )
        flow_los_residual_no_lever_p95 = (
            float(np.percentile(self.flow_los_residual_no_lever_norms, 95))
            if self.flow_los_residual_no_lever_norms else -1.0
        )
        flow_los_lever_p95 = (
            float(np.percentile(self.flow_los_lever_arm_norms, 95))
            if self.flow_los_lever_arm_norms else -1.0
        )
        flow_lever_displacement_p95 = (
            float(np.percentile(self.flow_lever_arm_displacement_norms, 95))
            if self.flow_lever_arm_displacement_norms else -1.0
        )
        diagnostic = DiagnosticStatus()
        diagnostic.name = "unified_backend_fusion"
        diagnostic.hardware_id = "companion_computer"
        diagnostic.level, diagnostic.message = backend_diagnostic_level_message(
            self.last_reason,
            self.counts["optimization_errors"],
            self.scan_prediction_contract_violated,
            self.scan_prediction_contract_reason,
        )
        diagnostic.values = [
            self._key("scheduler_health", self.scheduler_health),
            self._key(
                "frontend_map_pose_reason", self.last_frontend_map_pose_reason
            ),
            self._key(
                "frontend_map_position_variance_m2",
                f"{self.last_frontend_map_position_variance_m2:.9g}",
            ),
            self._key(
                "frontend_map_orientation_variance_rad2",
                f"{self.last_frontend_map_orientation_variance_rad2:.9g}",
            ),
            self._key(
                "frontend_map_pose_delay_states",
                self.frontend_map_commit_delay_states,
            ),
            self._key(
                "frontend_map_pose_delay_s",
                f"{self.last_frontend_map_pose_delay_s:.9g}",
            ),
            self._key("lidar_map_eligible", self.last_lidar_map_eligible),
            self._key("lidar_map_reason", self.last_lidar_map_reason),
            self._key("reliability_mode", self.reliability_mode),
            self._key("backend_solver_mode", self.backend_solver_mode),
            self._key(
                "cpp_math_core_active",
                getattr(self.backend, "cpp_math_core_enabled", False),
            ),
            self._key("window_states", self.backend.state_count),
            self._key("window_factors", self.backend.factor_count),
            self._key(
                "nonlinear_iterations",
                getattr(self.backend, "last_iterations", 1),
            ),
            self._key(
                "nonlinear_iteration_budget",
                getattr(self.backend, "last_iteration_budget", 1),
            ),
            self._key(
                "lm_rejected_steps",
                getattr(self.backend, "last_rejected_steps", 0),
            ),
            self._key(
                "lm_damping",
                f"{getattr(self.backend, 'last_damping', 0.0):.9g}",
            ),
            self._key(
                "backend_solve_ms",
                f"{getattr(self.backend, 'last_solve_ms', 0.0):.3f}",
            ),
            self._key(
                "backend_solve_wall_ms",
                f"{getattr(self.backend, 'last_solve_ms', 0.0):.3f}",
            ),
            self._key(
                "backend_solve_mean_ms",
                f"{average_solve_ms:.3f}",
            ),
            self._key(
                "backend_solve_max_ms", f"{self.backend_solve_ms_max:.3f}",
            ),
            self._key(
                "backend_marginalization_ms",
                f"{getattr(self.backend, 'last_marginalization_ms', 0.0):.3f}",
            ),
            self._key(
                "backend_marginalization_wall_ms",
                f"{getattr(self.backend, 'last_marginalization_ms', 0.0):.3f}",
            ),
            self._key(
                "backend_cost",
                f"{getattr(self.backend, 'last_cost', 0.0):.9g}",
            ),
            self._key("callback_ms", f"{self.last_callback_ms:.3f}"),
            self._key("callback_wall_ms", f"{self.last_callback_ms:.3f}"),
            self._key("algorithm_clock", "ros_sim_time"),
            self._key("covariance_source", self.last_covariance_source),
            self._key(
                "optimization_integrity_reason",
                self.last_optimization_integrity.reason,
            ),
            self._key(
                "optimization_integrity_counts",
                ",".join(
                    f"{name}:{count}"
                    for name, count in sorted(
                        self.optimization_integrity_reason_counts.items()
                    )
                ) or "none",
            ),
            self._key(
                "optimization_translation_correction_m",
                f"{self.last_optimization_integrity.translation_correction_m:.9g}",
            ),
            self._key(
                "optimization_rotation_correction_rad",
                f"{self.last_optimization_integrity.rotation_correction_rad:.9g}",
            ),
            self._key(
                "optimization_velocity_correction_mps",
                f"{self.last_optimization_integrity.velocity_correction_mps:.9g}",
            ),
            self._key(
                "optimization_accel_bias_correction_mps2",
                f"{self.last_optimization_integrity.accel_bias_correction_mps2:.9g}",
            ),
            self._key(
                "optimization_gyro_bias_correction_radps",
                f"{self.last_optimization_integrity.gyro_bias_correction_radps:.9g}",
            ),
            self._key(
                "optimization_information_rank",
                self.last_optimization_integrity.latest_information_rank,
            ),
            self._key(
                "optimization_initial_cost",
                f"{self.last_optimization_integrity.initial_cost:.9g}",
            ),
            self._key(
                "optimization_final_cost",
                f"{self.last_optimization_integrity.final_cost:.9g}",
            ),
            self._key(
                "optimization_information_condition",
                f"{self.last_optimization_integrity.latest_information_condition:.9g}",
            ),
            self._key(
                "frontend_state_seed_enabled", self.frontend_state_seed_enabled
            ),
            self._key(
                "scan_prediction_contract_established",
                self.scan_prediction_contract_established,
            ),
            self._key(
                "scan_prediction_contract_valid",
                not self.scan_prediction_contract_violated,
            ),
            self._key(
                "scan_prediction_contract_reason",
                self.scan_prediction_contract_reason,
            ),
            self._key(
                "scan_prediction_contract_consecutive_failures",
                self.scan_prediction_contract_consecutive_failures,
            ),
            self._key(
                "scan_prediction_contract_failure_threshold",
                self.scan_prediction_contract_failure_threshold,
            ),
            self._key(
                "scan_prediction_contract_first_failure_sequence",
                self.scan_prediction_contract_first_failure_sequence,
            ),
            self._key(
                "scan_prediction_contract_first_failure_stamp_s",
                f"{self.scan_prediction_contract_first_failure_stamp_s:.9g}",
            ),
            self._key(
                "scan_prediction_contract_trips",
                self.counts["scan_prediction_contract_trips"],
            ),
            self._key(
                "scan_prediction_contract_recoveries",
                self.counts["scan_prediction_contract_recoveries"],
            ),
            self._key(
                "scan_prediction_contract_output_suppressed",
                self.counts["scan_prediction_contract_output_suppressed"],
            ),
            self._key("state_reset_counter", self.state_reset_counter),
            self._key("last_imu_reason", self.last_imu_reason),
            self._key("calibration_reason", self.last_calibration_update.reason),
            self._key(
                "calibration_motion_reason", self.last_calibration_motion_reason
            ),
            self._key(
                "calibration_mode",
                self.calibration_mode,
            ),
            self._key(
                "calibration_motion_received",
                self.counts["calibration_motion_received"],
            ),
            self._key(
                "calibration_motion_rejected",
                self.counts["calibration_motion_rejected"],
            ),
            self._key(
                "calibration_seed_initialized",
                self.counts["calibration_seed_initialized"],
            ),
            self._key(
                "calibration_seed_lock_busy",
                self.counts["calibration_seed_lock_busy"],
            ),
            self._key(
                "calibration_time_offset_s",
                f"{self.last_calibration_update.time_offset_s:.9g}",
            ),
            self._key(
                "calibration_time_correlation",
                f"{self.last_calibration_update.time_correlation:.9g}",
            ),
            self._key(
                "calibration_time_margin",
                f"{self.last_calibration_update.time_margin:.9g}",
            ),
            self._key(
                "calibration_rotation_residual_rad",
                f"{self.last_calibration_update.rotation_residual_rad:.9g}",
            ),
            self._key(
                "calibration_pair_count",
                self.last_calibration_update.pair_count,
            ),
            self._key(
                "calibration_time_candidate_valid",
                self.calibrator.last_time_candidate.valid,
            ),
            self._key(
                "calibration_time_candidate_offset_s",
                f"{self.calibrator.last_time_candidate.offset_s:.9g}",
            ),
            self._key(
                "calibration_time_candidate_pairs",
                self.calibrator.last_time_candidate.pair_count,
            ),
            self._key(
                "calibration_time_candidate_reason",
                self.calibrator.last_time_candidate.reason,
            ),
            self._key(
                "calibration_excitation_eigenvalues",
                ",".join(
                    f"{value:.9g}"
                    for value in self.last_calibration_update.excitation_eigenvalues
                ),
            ),
            self._key(
                "calibration_excitation_ratio",
                f"{self.calibrator.last_excitation_ratio:.9g}",
            ),
            self._key(
                "calibration_accumulated_rotation_rad",
                f"{self.calibrator.last_accumulated_rotation_rad:.9g}",
            ),
            self._key(
                "calibration_unweighted_accumulated_rotation_rad",
                f"{self.calibrator.last_unweighted_accumulated_rotation_rad:.9g}",
            ),
            self._key(
                "calibration_weighted_accumulated_rotation_rad",
                f"{self.calibrator.last_weighted_accumulated_rotation_rad:.9g}",
            ),
            self._key(
                "calibration_imu_accumulated_rotation_rad",
                f"{self.calibrator.last_imu_accumulated_rotation_rad:.9g}",
            ),
            self._key(
                "calibration_motion_weight_mean",
                f"{self.calibrator.last_motion_weight_mean:.9g}",
            ),
            self._key(
                "calibration_rotation_inlier_ratio",
                f"{self.calibrator.last_rotation_inlier_ratio:.9g}",
            ),
            self._key("calibration_time_locked", self.calibrator.time_locked),
            self._key(
                "calibration_time_lock_candidate_count",
                self.calibrator.time_lock_candidate_count,
            ),
            self._key(
                "calibration_time_lock_conflict_count",
                self.calibrator.time_lock_conflict_count,
            ),
            self._key(
                "calibration_time_lock_revocations",
                self.calibrator.time_lock_revocations,
            ),
            self._key(
                "calibration_rotation_locked", self.calibrator.rotation_locked
            ),
            self._key("calibration_locked", self.last_calibration_update.locked),
            self._key("imu_startup_reason", self.last_imu_startup_reason),
            self._key(
                "imu_startup_sample_count", self.last_imu_startup_sample_count
            ),
            self._key(
                "imu_startup_span_s", f"{self.last_imu_startup_span_s:.9g}"
            ),
            self._key(
                "imu_startup_accel_bias",
                ",".join(
                    f"{value:.9g}" for value in self.last_imu_startup_accel_bias
                ),
            ),
            self._key(
                "imu_startup_gyro_bias",
                ",".join(
                    f"{value:.9g}" for value in self.last_imu_startup_gyro_bias
                ),
            ),
            self._key(
                "imu_max_positive_arrival_gap_s",
                f"{self.imu_max_positive_arrival_gap_s:.9g}",
            ),
            self._key("last_imu_residual_error", self.last_imu_residual_error),
            self._key("last_exception", self.last_exception),
            self._key(
                "imu_preintegration_residual_mahalanobis",
                f"{self.last_imu_preintegration_residual_mahalanobis:.9g}",
            ),
            self._key(
                "gnss_prefit_nis", f"{self.last_gnss_prefit_nis:.9g}"
            ),
            self._key(
                "gnss_prefit_xy_nis", f"{self.last_gnss_prefit_xy_nis:.9g}"
            ),
            self._key(
                "gnss_prefit_z_nis", f"{self.last_gnss_prefit_z_nis:.9g}"
            ),
            self._key("gnss_xy_admitted", self.last_gnss_xy_admitted),
            self._key("gnss_z_admitted", self.last_gnss_z_admitted),
            self._key(
                "gnss_xy_information_scale",
                f"{self.last_gnss_xy_information_scale:.9g}",
            ),
            self._key(
                "gnss_z_information_scale",
                f"{self.last_gnss_z_information_scale:.9g}",
            ),
            self._key(
                "gnss_z_reanchor_applied",
                self.last_gnss_z_reanchor_applied,
            ),
            self._key(
                "gnss_z_reanchor_target_m",
                f"{self.last_gnss_z_reanchor_target_m:.9g}",
            ),
            self._key(
                "gnss_z_reanchor_consecutive",
                self.gnss_z_reanchor_consecutive,
            ),
            self._key(
                "gnss_z_reanchor_attempts",
                self.counts["gnss_z_reanchor_attempts"],
            ),
            self._key(
                "gnss_z_reanchor_factors",
                self.counts["gnss_z_reanchor_factors"],
            ),
            self._key(
                "gnss_z_recovery_factors",
                self.counts["gnss_z_recovery_factors"],
            ),
            self._key(
                "gnss_prefit_residual_norm_m",
                f"{self.last_gnss_prefit_residual_norm_m:.9g}",
            ),
            self._key(
                "gnss_prefit_residual_xyz_m",
                ",".join(
                    f"{value:.9g}"
                    for value in self.last_gnss_prefit_residual_xyz
                ),
            ),
            self._key(
                "gnss_prefit_stamp_s", f"{self.last_gnss_prefit_stamp_s:.9g}"
            ),
            self._key(
                "gnss_degradation_score",
                f"{self.last_gnss_degradation_score:.9g}",
            ),
            self._key(
                "gnss_reliability_weight",
                f"{self.last_gnss_reliability_weight:.9g}",
            ),
            self._key(
                "gnss_effective_information_scale",
                f"{self.last_gnss_effective_information_scale:.9g}",
            ),
            self._key(
                "gnss_admission_reason", self.last_gnss_admission_reason
            ),
            self._key(
                "lidar_prediction_position_innovation_m",
                f"{self.last_lidar_prediction_position_innovation_m:.9g}",
            ),
            self._key(
                "lidar_prediction_yaw_innovation_rad",
                f"{self.last_lidar_prediction_yaw_innovation_rad:.9g}",
            ),
            self._key(
                "native_lidar_prediction_gate_enabled",
                self.lidar_prediction_gate_enabled,
            ),
            self._key(
                "native_lidar_prediction_gate_latched",
                self.native_lidar_prediction_gate_latched,
            ),
            self._key(
                "native_lidar_prediction_gate_rejections",
                self.counts["native_lidar_prediction_gate_rejections"],
            ),
            self._key(
                "native_lidar_prediction_gate_recoveries",
                self.counts["native_lidar_prediction_gate_recoveries"],
            ),
            self._key(
                "native_lidar_prediction_recovery_factors",
                self.counts["native_lidar_prediction_recovery_factors"],
            ),
            self._key(
                "native_lidar_prediction_gate_consecutive_rejections",
                self.native_lidar_prediction_gate_consecutive_rejections,
            ),
            self._key(
                "native_lidar_prediction_gate_reason",
                self.last_native_lidar_prediction_gate_reason,
            ),
            self._key(
                "native_lidar_epoch_stale_rejected",
                self.counts["native_lidar_epoch_stale_rejected"],
            ),
            self._key(
                "native_lidar_epoch_future_rejected",
                self.counts["native_lidar_epoch_future_rejected"],
            ),
            self._key(
                "native_lidar_factor_reset_counter",
                self.last_native_factor_reset_counter,
            ),
            self._key(
                "native_lidar_prediction_gate_max_position_m",
                f"{self.lidar_prediction_gate_max_position_m:.9g}",
            ),
            self._key(
                "native_lidar_prediction_gate_recovery_after_rejections",
                self.lidar_prediction_gate_recovery_after_rejections,
            ),
            self._key(
                "native_lidar_prediction_recovery_weight",
                f"{self.lidar_prediction_recovery_weight:.9g}",
            ),
            self._key(
                "native_lidar_prediction_recovery_inflation",
                f"{self.lidar_prediction_recovery_inflation:.9g}",
            ),
            self._key("lidar_factor_source", self.last_lidar_source),
            self._key("state_trigger_source", self.last_state_trigger_source),
            self._key(
                "live_propagation_reason",
                self.last_live_propagation_reason,
            ),
            self._key(
                "unified_odom_output_mode", self.unified_odom_output_mode
            ),
            self._key(
                "optimized_states_committed",
                self.counts["optimized_states_committed"],
            ),
            self._key(
                "optimized_odom_published",
                self.counts["optimized_odom_published"],
            ),
            self._key(
                "optimized_odom_anchor_only",
                self.counts["optimized_odom_anchor_only"],
            ),
            self._key(
                "optimized_odom_nonmonotonic_suppressed",
                self.counts["optimized_odom_nonmonotonic_suppressed"],
            ),
            self._key(
                "optimized_odom_mode_suppressed",
                self.counts["optimized_odom_mode_suppressed"],
            ),
            self._key(
                "live_propagation_attempts",
                self.counts["live_propagation_attempts"],
            ),
            self._key(
                "live_propagation_published",
                self.counts["live_propagation_published"],
            ),
            self._key(
                "live_propagation_rejected",
                self.counts["live_propagation_rejected"],
            ),
            self._key(
                "frontend_activation_published",
                self.counts["frontend_activation_published"],
            ),
            self._key(
                "auxiliary_keyframe_reason",
                self.last_auxiliary_keyframe_reason,
            ),
            self._key(
                "auxiliary_keyframe_attempts",
                self.counts["auxiliary_keyframe_attempts"],
            ),
            self._key(
                "auxiliary_keyframe_committed",
                self.counts["auxiliary_keyframe_committed"],
            ),
            self._key(
                "auxiliary_keyframe_rejected",
                self.counts["auxiliary_keyframe_rejected"],
            ),
            self._key(
                "auxiliary_keyframe_errors",
                self.counts["auxiliary_keyframe_errors"],
            ),
            self._key(
                "output_position_variance_m2",
                f"{self.last_output_position_variance_m2:.9g}",
            ),
            self._key(
                "output_orientation_variance_rad2",
                f"{self.last_output_orientation_variance_rad2:.9g}",
            ),
            self._key(
                "output_velocity_variance_m2ps2",
                f"{self.last_output_velocity_variance_m2ps2:.9g}",
            ),
            self._key(
                "maximum_output_position_variance_m2",
                f"{self.maximum_output_position_variance_m2:.9g}",
            ),
            self._key(
                "maximum_output_orientation_variance_rad2",
                f"{self.maximum_output_orientation_variance_rad2:.9g}",
            ),
            self._key(
                "maximum_auxiliary_position_variance_m2",
                f"{self.maximum_auxiliary_position_variance_m2:.9g}",
            ),
            self._key(
                "optimization_anchor_generation",
                self.optimization_anchor_generation,
            ),
            self._key("output_source", self.last_output_source),
            self._key(
                "native_lidar_callback_source_age_s",
                f"{self.last_native_callback_source_age_s:.9g}",
            ),
            self._key(
                "native_lidar_worker_source_age_s",
                f"{self.last_native_worker_source_age_s:.9g}",
            ),
            self._key(
                "output_source_age_s",
                f"{self.last_output_source_age_s:.9g}",
            ),
            self._key("native_lidar_sequence", self.last_native_sequence),
            self._key("native_lidar_matches", self.last_native_matches),
            self._key("native_lidar_effective_rank", self.last_native_effective_rank),
            self._key("native_lidar_translation_rank", self.last_native_translation_rank),
            self._key("native_lidar_rotation_rank", self.last_native_rotation_rank),
            self._key(
                "native_lidar_condition_number",
                f"{self.last_native_condition_number:.9g}",
            ),
            self._key(
                "native_lidar_characteristic_range_m",
                f"{self.last_native_characteristic_range_m:.9g}",
            ),
            self._key(
                "native_lidar_normalized_eigenvalues",
                ",".join(
                    f"{value:.6g}"
                    for value in self.last_native_normalized_eigenvalues
                ),
            ),
            self._key(
                "native_lidar_vertical_raw_information",
                f"{self.last_native_vertical_raw_information:.9g}",
            ),
            self._key(
                "native_lidar_vertical_profile_information",
                f"{self.last_native_vertical_profile_information:.9g}",
            ),
            self._key(
                "native_lidar_vertical_coupling_retention_ratio",
                f"{self.last_native_vertical_coupling_retention_ratio:.9g}",
            ),
            self._key(
                "native_lidar_normal_z_energy_fraction",
                f"{self.last_native_normal_z_energy_fraction:.9g}",
            ),
            self._key(
                "native_lidar_horizontal_plane_fraction",
                f"{self.last_native_horizontal_plane_fraction:.9g}",
            ),
            self._key(
                "native_lidar_axis_raw_information_xyz",
                ",".join(
                    f"{value:.9g}"
                    for value in self.last_native_axis_raw_information
                ),
            ),
            self._key(
                "native_lidar_axis_profile_information_xyz",
                ",".join(
                    f"{value:.9g}"
                    for value in self.last_native_axis_profile_information
                ),
            ),
            self._key(
                "native_lidar_axis_coupling_retention_xyz",
                ",".join(
                    f"{value:.9g}"
                    for value in self.last_native_axis_coupling_retention_ratio
                ),
            ),
            self._key(
                "native_lidar_axis_relative_support_xyz",
                ",".join(
                    f"{value:.9g}"
                    for value in self.last_native_axis_relative_support
                ),
            ),
            self._key(
                "native_lidar_translation_profile_information",
                ",".join(
                    f"{value:.9g}"
                    for value in self.last_native_translation_profile_information.reshape(-1)
                ),
            ),
            self._key(
                "native_lidar_translation_normalized_eigenvalues",
                ",".join(
                    f"{value:.9g}"
                    for value in self.last_native_translation_normalized_eigenvalues
                ),
            ),
            self._key(
                "native_lidar_weakest_translation_direction_xyz",
                ",".join(
                    f"{value:.9g}"
                    for value in self.last_native_weakest_translation_direction
                ),
            ),
            self._key(
                "native_lidar_health_degradation",
                f"{self.last_native_health_degradation:.9g}",
            ),
            self._key(
                "native_lidar_consistency_degradation",
                f"{self.last_native_consistency_degradation:.9g}",
            ),
            self._key(
                "native_lidar_observability_degradation_xyz",
                ",".join(
                    f"{value:.9g}"
                    for value in self.last_native_observability_degradation
                ),
            ),
            self._key(
                "native_lidar_combined_degradation_xyz",
                ",".join(
                    f"{value:.9g}"
                    for value in self.last_native_combined_degradation
                ),
            ),
            self._key(
                "native_lidar_isotropic_information_support_xyz",
                ",".join(
                    f"{value:.9g}"
                    for value in self.last_native_isotropic_information_support
                ),
            ),
            self._key(
                "native_lidar_axis_information_scale_xyz",
                ",".join(
                    f"{value:.9g}"
                    for value in self.last_lidar_axis_information_scale
                ),
            ),
            self._key(
                "axis_handoff_latched_xyz",
                ",".join(
                    "1" if value else "0"
                    for value in self.axis_handoff_latched
                ),
            ),
            self._key(
                "lidar_axis_observability_weak_xyz",
                ",".join(
                    "1" if value else "0"
                    for value in self.lidar_axis_observability_latched
                ),
            ),
            self._key(
                "lidar_axis_map_protected_xyz",
                ",".join(
                    "1" if value else "0"
                    for value in self.last_axis_map_protected
                ),
            ),
            self._key(
                "lidar_axis_map_protection_sources_xyz",
                ",".join(self.last_axis_map_protection_sources),
            ),
            self._key(
                "axis_handoff_alternative_information_xyz",
                ",".join(
                    f"{value:.9g}"
                    for value in self.last_axis_handoff_alternative_information
                ),
            ),
            self._key(
                "axis_reliability_xyz",
                ",".join(
                    f"{value:.9g}" for value in self.last_axis_reliability
                ),
            ),
            self._key(
                "axis_degradation_xyz",
                ",".join(
                    f"{value:.9g}" for value in self.last_axis_degradation
                ),
            ),
            self._key(
                "axis_global_reliability_xyz",
                ",".join(
                    f"{value:.9g}"
                    for value in self.last_axis_global_reliability
                ),
            ),
            self._key(
                "axis_supporting_sources_xyz",
                ";".join(
                    ",".join(values)
                    for values in self.last_axis_supporting_sources
                ),
            ),
            self._key("barometer_fallback_active", self.barometer_segment.active),
            self._key("barometer_segment_id", self.last_barometer_segment_id),
            self._key("barometer_reason", self.last_barometer_reason),
            self._key(
                "barometer_anchor_source",
                self.last_barometer_anchor_source,
            ),
            self._key(
                "barometer_anchor_reference_age_s",
                f"{self.last_barometer_anchor_reference_age_s:.9g}",
            ),
            self._key(
                "barometer_reference_reason",
                self.last_barometer_reference_reason,
            ),
            self._key(
                "barometer_reference_stamp_s",
                f"{self.last_barometer_reference_stamp_s:.9g}",
            ),
            self._key(
                "barometer_reference_z_m",
                f"{self.last_barometer_reference_z_m:.9g}",
            ),
            self._key(
                "barometer_prefit_residual_m",
                f"{self.last_barometer_prefit_residual_m:.9g}",
            ),
            self._key(
                "barometer_information_scale",
                f"{self.last_barometer_information_scale:.9g}",
            ),
            self._key(
                "barometer_measurement_height_m",
                f"{self.last_barometer_measurement_height_m:.9g}",
            ),
            self._key(
                "native_lidar_stamp_error_ms",
                f"{self.last_native_stamp_error_ms:.9g}",
            ),
            self._key("pending_lio_messages", len(self.pending_lio)),
            self._key("pending_imu_lio_messages", len(self.pending_imu_lio)),
            self._key("pending_native_worker_frames", self.native_work_queue.qsize()),
            self._key("last_flow_reason", self.last_flow_reason),
            self._key("last_flow_factor_type", self.last_flow_factor_type),
            self._key("range_facet_enabled", self.range_facet_enabled),
            self._key(
                "flow_range_facet_accepted",
                self.counts["flow_range_facet_accepted"],
            ),
            self._key(
                "flow_range_facet_rejected",
                self.counts["flow_range_facet_rejected"],
            ),
            self._key(
                "range_facet_last_rejection_reason",
                getattr(self, "last_range_facet_rejection_reason", "none"),
            ),
            self._key(
                "range_facet_rejection_reasons",
                ",".join(
                    f"{key}:{value}"
                    for key, value in sorted(
                        getattr(self, "range_facet_rejection_reasons", {})
                        .items()
                    )
                ),
            ),
            self._key("flow_rotation_phase", self.last_flow_rotation_phase),
            self._key(
                "flow_rotation_weight", f"{self.last_flow_rotation_weight:.9g}"
            ),
            self._key(
                "flow_yaw_rate_abs_radps",
                f"{self.last_flow_yaw_rate_abs_radps:.9g}",
            ),
            self._key(
                "flow_speed_mps", f"{self.last_flow_speed_mps:.9g}"
            ),
            self._key(
                "flow_speed_limit_mps",
                f"{self.last_flow_speed_limit_mps:.9g}",
            ),
            self._key(
                "flow_range_sigma_m",
                f"{self.last_flow_range_sigma_m:.9g}",
            ),
            self._key(
                "flow_factor_variance_m2",
                f"{self.last_flow_covariance_m2:.9g}",
            ),
            self._key(
                "flow_los_diagnostic_valid",
                0 if self.last_flow_los_diagnostic is None else 1,
            ),
            self._key(
                "flow_los_residual_p95_radps",
                f"{flow_los_residual_p95:.9g}",
            ),
            self._key(
                "flow_los_residual_no_lever_p95_radps",
                f"{flow_los_residual_no_lever_p95:.9g}",
            ),
            self._key(
                "flow_los_lever_p95_radps",
                f"{flow_los_lever_p95:.9g}",
            ),
            self._key(
                "flow_los_sensor_offset_body_m",
                ",".join(
                    f"{value:.6g}" for value in self.flow_sensor_offset_body_m
                ),
            ),
            self._key(
                "flow_lever_arm_compensation_enabled",
                self.flow_lever_arm_compensation_enabled,
            ),
            self._key(
                "flow_lever_arm_displacement_p95_m",
                f"{flow_lever_displacement_p95:.9g}",
            ),
        ]
        diagnostic.values.extend(
            self._key(name, value) for name, value in self.counts.items()
        )
        diagnostic.values.extend([
            self._key(
                "visual_reprojection_rmse_normalized",
                f"{self.last_visual_reprojection_rmse_normalized:.9g}",
            ),
            self._key(
                "visual_reprojection_residual_dimension",
                self.last_visual_reprojection_residual_dimension,
            ),
            self._key(
                "visual_prefit_rmse_normalized",
                f"{self.last_visual_prefit_rmse_normalized:.9g}",
            ),
            self._key(
                "visual_prefit_rmse_px",
                f"{self.last_visual_prefit_rmse_px:.9g}",
            ),
            self._key(
                "visual_prefit_valid_track_ratio",
                f"{self.last_visual_prefit_valid_track_ratio:.9g}",
            ),
            self._key(
                "visual_prefit_jacobian_rank",
                self.last_visual_prefit_jacobian_rank,
            ),
            self._key(
                "visual_prefit_jacobian_condition",
                f"{self.last_visual_prefit_jacobian_condition:.9g}",
            ),
            self._key(
                "visual_prefit_nis_per_dof",
                f"{self.last_visual_prefit_nis_per_dof:.9g}",
            ),
            self._key(
                "visual_prefit_information_trace",
                f"{self.last_visual_prefit_information_trace:.9g}",
            ),
            self._key(
                "visual_prefit_information_max_eigenvalue",
                f"{self.last_visual_prefit_information_max_eigenvalue:.9g}",
            ),
            self._key(
                "visual_pnp_inlier_ratio",
                f"{self.last_visual_pnp_inlier_ratio:.9g}",
            ),
            self._key(
                "visual_pnp_information_rank",
                self.last_visual_pnp_information_rank,
            ),
            self._key(
                "visual_pnp_condition_number",
                f"{self.last_visual_pnp_condition_number:.9g}",
            ),
            self._key(
                "visual_pnp_mean_reprojection_error_px",
                f"{self.last_visual_pnp_mean_reprojection_error_px:.9g}",
            ),
            self._key(
                "visual_batch_information_scale",
                f"{self.last_visual_batch_information_scale:.9g}",
            ),
            self._key("rgbd_depth_reason", self.last_rgbd_depth_reason),
            self._key(
                "rgbd_depth_tracks", self.last_rgbd_depth_track_count
            ),
            self._key(
                "rgbd_depth_prefit_rmse_m",
                f"{self.last_rgbd_depth_prefit_rmse_m:.9g}",
            ),
            self._key("rgbd_direct_reason", self.last_rgbd_direct_reason),
            self._key("rgbd_direct_tracks", self.last_rgbd_direct_track_count),
            self._key(
                "rgbd_direct_depth_rmse_m",
                f"{self.last_rgbd_direct_depth_rmse_m:.9g}",
            ),
            self._key(
                "rgbd_direct_photometric_rmse",
                f"{self.last_rgbd_direct_photometric_rmse:.9g}",
            ),
            self._key(
                "rgbd_direct_photometric_information_scale",
                f"{self.last_rgbd_direct_photometric_information_scale:.9g}",
            ),
            self._key("visual_initialized", self.visual_initializer.ready),
            self._key(
                "visual_initialization_batches",
                self.visual_initializer.consecutive_batches,
            ),
            self._key(
                "visual_time_offset_s",
                f"{self._effective_visual_time_offset_s():.9g}",
            ),
            self._key(
                "visual_time_calibration_locked_offset_s",
                f"{self.last_visual_time_calibration.time_offset_s:.9g}",
            ),
            self._key(
                "visual_time_calibration_candidate_offset_s",
                f"{self.last_visual_time_calibration.candidate_offset_s:.9g}",
            ),
            self._key(
                "visual_time_offset_locked",
                self.last_visual_time_calibration.locked,
            ),
            self._key(
                "visual_time_calibration_correlation",
                f"{self.last_visual_time_calibration.correlation:.9g}",
            ),
            self._key(
                "visual_time_calibration_margin",
                f"{self.last_visual_time_calibration.margin:.9g}",
            ),
            self._key(
                "visual_time_calibration_pair_count",
                self.last_visual_time_calibration.pair_count,
            ),
            self._key(
                "visual_time_calibration_reason",
                self.last_visual_time_calibration.reason,
            ),
            self._key(
                "visual_time_calibration_vote_history",
                json.dumps(list(self.visual_time_calibration_vote_history)),
            ),
            self._key("visual_pending_enabled", self.visual_pending_enabled),
            self._key(
                "visual_pending_queue_size",
                len(self.pending_visual_candidates),
            ),
            self._key(
                "visual_factor_score_history_size",
                len(self.visual_factor_scores),
            ),
            self._key(
                "visual_factor_score_stamp_s",
                f"{self.last_visual_factor_score_stamp_s:.9g}",
            ),
            self._key(
                "visual_factor_score_weight",
                f"{self.last_visual_factor_score_weight:.9g}",
            ),
            self._key(
                "visual_factor_score_degradation",
                f"{self.last_visual_factor_score_degradation:.9g}",
            ),
            self._key(
                "visual_factor_score_match_error_s",
                f"{self.last_visual_factor_score_match_error_s:.9g}",
            ),
            self._key(
                "visual_factor_score_reasons",
                ",".join(self.last_visual_factor_score_reasons) or "none",
            ),
            self._key(
                "visual_combined_reliability_reasons",
                ",".join(self.last_visual_combined_reasons) or "none",
            ),
            self._key(
                "visual_state_window_start_s",
                self.visual_state_stamps[0] if self.visual_state_stamps else -1.0,
            ),
            self._key(
                "visual_state_window_end_s",
                self.visual_state_stamps[-1] if self.visual_state_stamps else -1.0,
            ),
            self._key(
                "visual_timing_reason_counts",
                ",".join(
                    f"{name}:{count}" for name, count
                    in sorted(self.visual_timing_reason_counts.items())
                ) or "none",
            ),
        ])
        diagnostic.values.append(self._key(
            "performance_profiling_enabled",
            self.performance_profiling_enabled,
        ))
        if self.performance_profiling_enabled:
            profiles = {
                f"callback_{name}": values
                for name, values in self._phase_profile_summary().items()
            }
            profiles.update({
                f"solver_{name}": values
                for name, values in getattr(
                    self.backend, "profile_summary", lambda: {})()
                .items()
            })
            for name, values in sorted(profiles.items()):
                diagnostic.values.append(self._key(
                    f"profile_{name}_count", values["count"]
                ))
                for percentile_name in ("p50_ms", "p90_ms", "p95_ms", "max_ms"):
                    diagnostic.values.append(self._key(
                        f"profile_{name}_{percentile_name}",
                        f"{values[percentile_name]:.6f}",
                    ))
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status.append(diagnostic)
        self.diagnostic_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = UnifiedBackendNode()
    executor = MultiThreadedExecutor(num_threads=node.executor_threads)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        executor.shutdown()
        node.stop_native_worker()
        node.log_final_summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
