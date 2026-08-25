import math


def clamp(value, low=0.0, high=1.0):
    return max(low, min(high, float(value)))


def weighted_score(terms):
    weighted_terms = [
        (None if value is None else clamp(value), max(0.0, float(weight)))
        for value, weight in terms
    ]
    total_weight = sum(weight for _, weight in weighted_terms)
    if total_weight <= 0.0:
        return 1.0, 0.0
    available_weight = sum(weight for value, weight in weighted_terms if value is not None)
    score = sum(value * weight for value, weight in weighted_terms if value is not None)
    return clamp(score / total_weight), clamp(available_weight / total_weight)


def planar_norm(value):
    if value is None:
        return None
    if isinstance(value, (int, float)):
        return abs(float(value))
    values = list(value)
    if len(values) < 2:
        raise ValueError("planar residual requires at least two components")
    return math.hypot(float(values[0]), float(values[1]))


def apm_optical_flow_compensated_los(
    integrated_x, integrated_y, integrated_xgyro, integrated_ygyro,
):
    """Return APM's compensated LOS integrals in the sensor axes.

    ArduPilot first negates the raw optical-flow convention and then adds the
    body-rate integral: ``flowRadXYcomp = -rawFlowRates + bodyRadXY``.  Keeping
    this operation explicit prevents a later axis conversion from accidentally
    applying the gyro correction twice.
    """
    values = (
        integrated_x, integrated_y, integrated_xgyro, integrated_ygyro,
    )
    if not all(math.isfinite(float(value)) for value in values):
        return None
    return (
        -float(integrated_x) + float(integrated_xgyro),
        -float(integrated_y) + float(integrated_ygyro),
    )


def optical_flow_displacement_frd(integrated_x, integrated_y,
                                  integrated_xgyro, integrated_ygyro,
                                  ground_distance_m):
    """Convert APM-compensated flow into planar sensor-FRD displacement."""
    if not math.isfinite(float(ground_distance_m)):
        return None
    if float(ground_distance_m) <= 0.0:
        return None
    compensated = apm_optical_flow_compensated_los(
        integrated_x, integrated_y, integrated_xgyro, integrated_ygyro,
    )
    if compensated is None:
        return None
    compensated_x, compensated_y = compensated
    return (
        -compensated_y * float(ground_distance_m),
        compensated_x * float(ground_distance_m),
    )


def optical_flow_velocity_frd(integrated_x, integrated_y,
                              integrated_xgyro, integrated_ygyro,
                              integration_time_s, ground_distance_m):
    """Return APM-compatible sensor-FRD velocity from one flow exposure."""
    integration_time_s = float(integration_time_s)
    if not math.isfinite(integration_time_s) or integration_time_s <= 0.0:
        return None
    displacement = optical_flow_displacement_frd(
        integrated_x, integrated_y, integrated_xgyro, integrated_ygyro,
        ground_distance_m,
    )
    if displacement is None:
        return None
    return tuple(float(value) / integration_time_s for value in displacement)


def optical_flow_lever_arm_displacement_flu(
        angular_velocity_body_flu, lever_arm_body_flu, integration_time_s):
    """Return sensor-to-body displacement from ``omega x r`` in ROS FLU.

    The optical-flow sensor measures the velocity of its own origin.  For a
    body/IMU-origin horizontal displacement factor, the sensor-point motion
    contribution is removed once as ``(omega x r) * dt``.  The returned
    vector remains in body FLU coordinates; callers may discard ``z`` when
    the factor is planar.
    """
    try:
        angular_velocity = tuple(float(value) for value in angular_velocity_body_flu)
        lever_arm = tuple(float(value) for value in lever_arm_body_flu)
        duration = float(integration_time_s)
    except (TypeError, ValueError):
        return None
    if (
        len(angular_velocity) != 3 or len(lever_arm) != 3
        or not math.isfinite(duration) or duration <= 0.0
        or not all(math.isfinite(value) for value in angular_velocity)
        or not all(math.isfinite(value) for value in lever_arm)
    ):
        return None
    omega_x, omega_y, omega_z = angular_velocity
    arm_x, arm_y, arm_z = lever_arm
    return (
        (omega_y * arm_z - omega_z * arm_y) * duration,
        (omega_z * arm_x - omega_x * arm_z) * duration,
        (omega_x * arm_y - omega_y * arm_x) * duration,
    )


def optical_flow_los_rate_apm(integrated_x, integrated_y,
                              integrated_xgyro, integrated_ygyro,
                              integration_time_s):
    """Return APM-compensated optical-flow LOS rates in sensor axes."""
    integration_time_s = float(integration_time_s)
    if not math.isfinite(integration_time_s) or integration_time_s <= 0.0:
        return None
    compensated = apm_optical_flow_compensated_los(
        integrated_x, integrated_y, integrated_xgyro, integrated_ygyro,
    )
    if compensated is None:
        return None
    return tuple(float(value) / integration_time_s for value in compensated)


def optical_flow_los_prediction_flu(
        velocity_body_flu, angular_velocity_body_flu,
        lever_arm_body_flu, ground_distance_m):
    """Predict APM LOS rates after converting the backend body to FLU.

    ArduPilot's native FRD equation is ``[v_y/r, -v_x/r]``.  The backend
    uses the ROS-style FLU body convention, so the equivalent sensor-axis
    prediction is ``[-v_y/r, -v_x/r]``.  The lever-arm velocity is included
    as ``omega x r`` before the LOS projection.
    """
    try:
        velocity = tuple(float(value) for value in velocity_body_flu)
        angular_velocity = tuple(float(value) for value in angular_velocity_body_flu)
        lever_arm = tuple(float(value) for value in lever_arm_body_flu)
    except (TypeError, ValueError):
        return None
    if len(velocity) != 3 or len(angular_velocity) != 3 or len(lever_arm) != 3:
        return None
    distance = float(ground_distance_m)
    if (
        not math.isfinite(distance) or distance <= 0.0
        or not all(math.isfinite(value) for value in velocity)
        or not all(math.isfinite(value) for value in angular_velocity)
        or not all(math.isfinite(value) for value in lever_arm)
    ):
        return None
    omega_x, omega_y, omega_z = angular_velocity
    arm_x, arm_y, arm_z = lever_arm
    lever_velocity = (
        omega_y * arm_z - omega_z * arm_y,
        omega_z * arm_x - omega_x * arm_z,
        omega_x * arm_y - omega_y * arm_x,
    )
    sensor_velocity = tuple(
        velocity[index] + lever_velocity[index] for index in range(3)
    )
    return (
        -sensor_velocity[1] / distance,
        -sensor_velocity[0] / distance,
    )


def finalize_score(score, coverage, evidence, reasons):
    complete = coverage >= 1.0 - 1.0e-9
    evidence["evidence_weight_coverage"] = coverage
    evidence["score_complete"] = 1.0 if complete else 0.0
    if not complete:
        reasons.append("incomplete_paper_evidence")
    return score, evidence, reasons


def gnss_integrity_quality(fix_type, satellites_visible, hdop,
                           minimum_satellites=5, good_satellites=10,
                           good_hdop=1.0, maximum_hdop=4.0):
    """Build q_fix for Eq. 23 from FCU-reported GPS_RAW_INT integrity fields."""
    if fix_type is None:
        return None, {
            "fix_type": -1.0,
            "satellite_count": -1.0,
            "hdop": -1.0,
            "satellite_quality": -1.0,
            "dop_quality": -1.0,
        }, ["fcu_gnss_metadata_unavailable"]
    fix_type = int(fix_type)
    fix_quality = 1.0 if fix_type >= 3 else (0.5 if fix_type == 2 else 0.0)
    satellite_quality = None
    if satellites_visible is not None and int(satellites_visible) != 255:
        span = max(1, int(good_satellites) - int(minimum_satellites))
        satellite_quality = clamp((int(satellites_visible) - int(minimum_satellites)) / span)
    dop_quality = None
    if hdop is not None and math.isfinite(float(hdop)):
        span = max(1.0e-6, float(maximum_hdop) - float(good_hdop))
        dop_quality = clamp((float(maximum_hdop) - float(hdop)) / span)
    available = [value for value in (satellite_quality, dop_quality) if value is not None]
    if available:
        metadata_quality = sum(available) / len(available)
        quality = fix_quality * (0.6 + 0.4 * metadata_quality)
    else:
        quality = fix_quality
    reasons = []
    if fix_quality < 1.0:
        reasons.append("weak_fcu_fix_type")
    if satellite_quality is None:
        reasons.append("satellite_count_unavailable")
    elif satellite_quality < 0.5:
        reasons.append("few_satellites")
    if dop_quality is None:
        reasons.append("dop_unavailable")
    elif dop_quality < 0.5:
        reasons.append("large_hdop")
    evidence = {
        "fix_type": float(fix_type),
        "satellite_count": (
            -1.0 if satellites_visible is None else float(satellites_visible)
        ),
        "hdop": -1.0 if hdop is None else float(hdop),
        "satellite_quality": (
            -1.0 if satellite_quality is None else satellite_quality
        ),
        "dop_quality": -1.0 if dop_quality is None else dop_quality,
    }
    return quality, evidence, reasons


def lidar_score(hessian_eigenvalues, normal_covariance_eigenvalues, axial_penalty,
                matched_points, match_reference=1000, tau_lambda=1.0,
                tau_kappa=1.0e5, tau_normal=0.02,
                weights=(0.35, 0.20, 0.20, 0.25)):
    eigenvalues = sorted(max(0.0, float(value)) for value in hessian_eigenvalues)
    normal_eigenvalues = sorted(max(0.0, float(value)) for value in normal_covariance_eigenvalues)
    lambda_1 = eigenvalues[0] if eigenvalues else 0.0
    lambda_6 = eigenvalues[-1] if eigenvalues else 0.0
    condition = lambda_6 / (lambda_1 + 1.0e-12)
    phi_h = 0.5 * (
        min(1.0, tau_lambda / max(lambda_1, 1.0e-12))
        + min(1.0, condition / tau_kappa)
    )
    lambda_3_normal = normal_eigenvalues[0] if normal_eigenvalues else 0.0
    normal_term = tau_normal / (tau_normal + lambda_3_normal)
    match_term = 1.0 - min(1.0, float(matched_points) / max(1.0, float(match_reference)))
    score, coverage = weighted_score([
        (phi_h, weights[0]),
        (normal_term, weights[1]),
        (axial_penalty, weights[2]),
        (match_term, weights[3]),
    ])
    evidence = {
        "lambda_1_hessian": lambda_1,
        "hessian_condition": condition,
        "phi_h_eq19": phi_h,
        "lambda_min_normal_covariance": lambda_3_normal,
        "normal_term_eq19": normal_term,
        "axial_penalty_eq19": float(axial_penalty),
        "matched_points": float(matched_points),
        "match_reference": float(match_reference),
        "match_term_eq19": match_term,
    }
    reasons = []
    if phi_h > 0.5:
        reasons.append("weak_hessian_eq19")
    if normal_term > 0.5:
        reasons.append("poor_normal_diversity_eq19")
    if axial_penalty > 0.5:
        reasons.append("weak_axis_eq19")
    if match_term > 0.5:
        reasons.append("few_matches_eq19")
    return finalize_score(score, coverage, evidence, reasons)


def lidar_innovation_score(position_innovation_m, yaw_innovation_rad,
                           tau_position_m=0.50, tau_yaw_rad=0.35,
                           weights=(0.70, 0.30)):
    """Score LiDAR disagreement with a prediction that excludes current LiDAR."""
    def term(value, threshold):
        if value is None or not math.isfinite(float(value)) or float(value) < 0.0:
            return None
        return clamp(float(value) / max(1.0e-9, float(threshold)))

    position_term = term(position_innovation_m, tau_position_m)
    yaw_term = term(yaw_innovation_rad, tau_yaw_rad)
    score, coverage = weighted_score([
        (position_term, weights[0]),
        (yaw_term, weights[1]),
    ])
    evidence = {
        "lidar_prediction_position_innovation_m": (
            -1.0 if position_innovation_m is None else float(position_innovation_m)
        ),
        "lidar_prediction_yaw_innovation_rad": (
            -1.0 if yaw_innovation_rad is None else float(yaw_innovation_rad)
        ),
        "lidar_prediction_position_term": (
            -1.0 if position_term is None else position_term
        ),
        "lidar_prediction_yaw_term": -1.0 if yaw_term is None else yaw_term,
        "lidar_prediction_innovation_score": score,
    }
    reasons = []
    if position_term is None:
        reasons.append("lidar_prediction_position_unavailable")
    elif position_term > 0.5:
        reasons.append("large_lidar_prediction_position_innovation")
    if yaw_term is None:
        reasons.append("lidar_prediction_yaw_unavailable")
    elif yaw_term > 0.5:
        reasons.append("large_lidar_prediction_yaw_innovation")
    return finalize_score(score, coverage, evidence, reasons)


def lidar_factor_score(paper_result, innovation_result, approximate_geometry,
                       approximate_geometry_weight=0.20,
                       native_geometry_weight=0.60):
    """Build LiDAR pose-factor risk without map-admission evidence.

    An external Hessian is useful for soft covariance inflation, but cannot
    independently remove the LIO pose factor.  Only native geometry plus a
    complete LiDAR-free innovation can authorise a future binary hard gate.
    """
    paper_score, paper_evidence, paper_reasons = paper_result
    innovation_score, innovation_evidence, innovation_reasons = innovation_result
    innovation_complete = (
        innovation_evidence.get("score_complete", 0.0) >= 0.5
    )
    geometry_weight = clamp(
        approximate_geometry_weight if approximate_geometry
        else native_geometry_weight
    )
    innovation_weight = 1.0 - geometry_weight
    if innovation_complete:
        score = clamp(
            geometry_weight * float(paper_score)
            + innovation_weight * float(innovation_score)
        )
        coverage = 1.0
    else:
        # Keep available geometry as a soft diagnostic, but mark the result
        # hard-gate-ineligible.  The external geometry is still sufficient for
        # a conservative continuous weight during backend startup.
        score = clamp(geometry_weight * float(paper_score))
        coverage = (
            1.0 if paper_evidence.get("score_complete", 0.0) >= 0.5 else 0.0
        )
    evidence = dict(paper_evidence)
    evidence.update({
        "paper_score_eq19": float(paper_score),
        "geometry_source_approximate": 1.0 if approximate_geometry else 0.0,
        "geometry_weight_factor_score": geometry_weight,
        "innovation_weight_factor_score": innovation_weight,
        "innovation_complete_factor_score": (
            1.0 if innovation_complete else 0.0
        ),
        "hard_gate_allowed": (
            1.0 if (not approximate_geometry and innovation_complete) else 0.0
        ),
        "lidar_factor_score": score,
    })
    for key, value in innovation_evidence.items():
        if key not in ("evidence_weight_coverage", "score_complete"):
            evidence[key] = value
    reasons = list(paper_reasons) + list(innovation_reasons)
    if approximate_geometry:
        reasons.append("approximate_geometry_soft_only")
    if not innovation_complete:
        reasons.append("incomplete_lidar_prediction_innovation")
    return finalize_score(score, coverage, evidence, reasons)


def lidar_factor_score_for_mode(
    paper_result,
    innovation_result,
    approximate_geometry,
    mode="hybrid",
    approximate_geometry_weight=0.20,
    native_geometry_weight=0.60,
):
    """Select the published Eq. 19 score or the guarded project extension."""
    mode = str(mode).strip().lower()
    if mode == "hybrid":
        return lidar_factor_score(
            paper_result,
            innovation_result,
            approximate_geometry,
            approximate_geometry_weight,
            native_geometry_weight,
        )
    if mode != "paper_eq19":
        raise ValueError("LiDAR factor score mode must be hybrid or paper_eq19")

    score, source_evidence, source_reasons = paper_result
    evidence = dict(source_evidence)
    evidence.update({
        "paper_score_eq19": float(score),
        "geometry_source_approximate": (
            1.0 if approximate_geometry else 0.0
        ),
        "geometry_weight_factor_score": 1.0,
        "innovation_weight_factor_score": 0.0,
        "innovation_complete_factor_score": 0.0,
        "hard_gate_allowed": 1.0,
        "lidar_factor_score": float(score),
    })
    reasons = list(source_reasons) + ["paper_eq19_only"]
    return float(score), evidence, reasons


def lidar_map_score(residual_p95_m, spatial_coverage, dynamic_ratio,
                    uncertain_ratio, feature_repeatability,
                    tau_residual_m=0.15, tau_dynamic_ratio=0.20,
                    tau_uncertain_ratio=0.25,
                    weights=(0.25, 0.20, 0.20, 0.15, 0.20),
                    map_quality_diagnostic=None):
    """Score project-owned map-admission risk, never pose-factor risk.

    `map_quality` is intentionally excluded because the adapter currently
    derives it from dynamic ratio and repeatability.
    """
    def finite_term(value, transform):
        if value is None or not math.isfinite(float(value)):
            return None
        return clamp(transform(float(value)))

    residual_term = finite_term(
        residual_p95_m,
        lambda value: value / max(1.0e-9, float(tau_residual_m)),
    )
    coverage_term = finite_term(spatial_coverage, lambda value: 1.0 - value)
    dynamic_term = finite_term(
        dynamic_ratio,
        lambda value: value / max(1.0e-9, float(tau_dynamic_ratio)),
    )
    uncertain_term = finite_term(
        uncertain_ratio,
        lambda value: value / max(1.0e-9, float(tau_uncertain_ratio)),
    )
    repeatability_term = finite_term(
        feature_repeatability, lambda value: 1.0 - value,
    )
    score, coverage = weighted_score([
        (residual_term, weights[0]),
        (coverage_term, weights[1]),
        (dynamic_term, weights[2]),
        (uncertain_term, weights[3]),
        (repeatability_term, weights[4]),
    ])
    evidence = {
        "map_residual_p95_m": -1.0 if residual_p95_m is None else float(residual_p95_m),
        "map_residual_term": -1.0 if residual_term is None else residual_term,
        "map_spatial_coverage": -1.0 if spatial_coverage is None else float(spatial_coverage),
        "map_spatial_coverage_term": -1.0 if coverage_term is None else coverage_term,
        "map_dynamic_ratio": -1.0 if dynamic_ratio is None else float(dynamic_ratio),
        "map_dynamic_ratio_term": -1.0 if dynamic_term is None else dynamic_term,
        "map_uncertain_ratio": -1.0 if uncertain_ratio is None else float(uncertain_ratio),
        "map_uncertain_ratio_term": -1.0 if uncertain_term is None else uncertain_term,
        "map_feature_repeatability": (
            -1.0 if feature_repeatability is None else float(feature_repeatability)
        ),
        "map_repeatability_term": -1.0 if repeatability_term is None else repeatability_term,
        "map_quality_diagnostic": (
            -1.0 if map_quality_diagnostic is None else float(map_quality_diagnostic)
        ),
        "map_visibility_evidence_available": 0.0,
        "map_hard_gate_allowed": 0.0,
        "lidar_map_score": score,
    }
    reasons = ["map_risk_not_used_for_lidar_pose_factor"]
    if dynamic_term is not None and dynamic_term > 0.5:
        reasons.append("high_dynamic_ratio_map_risk")
    if uncertain_term is not None and uncertain_term > 0.5:
        reasons.append("high_uncertain_ratio_map_risk")
    if repeatability_term is not None and repeatability_term > 0.5:
        reasons.append("low_repeatability_map_risk")
    return finalize_score(score, coverage, evidence, reasons)


def augment_lidar_score(
        paper_result, residual_p95_m, spatial_coverage, dynamic_ratio,
        uncertain_ratio, feature_repeatability, map_quality,
        tau_residual_m=0.15, tau_dynamic_ratio=0.20,
        tau_uncertain_ratio=0.25, extension_reference=0.35,
        paper_weight=0.70,
        extension_weights=(0.20, 0.15, 0.20, 0.10, 0.15, 0.20)):
    """Fuse Eq. 19 with explicit project-owned map-protection evidence."""
    paper_score, paper_evidence, paper_reasons = paper_result

    def finite_term(value, transform):
        if value is None or not math.isfinite(float(value)):
            return None
        return clamp(transform(float(value)))

    residual_term = finite_term(
        residual_p95_m,
        lambda value: value / max(1.0e-9, float(tau_residual_m)),
    )
    coverage_term = finite_term(spatial_coverage, lambda value: 1.0 - value)
    dynamic_term = finite_term(
        dynamic_ratio,
        lambda value: value / max(1.0e-9, float(tau_dynamic_ratio)),
    )
    uncertain_term = finite_term(
        uncertain_ratio,
        lambda value: value / max(1.0e-9, float(tau_uncertain_ratio)),
    )
    repeatability_term = finite_term(
        feature_repeatability, lambda value: 1.0 - value,
    )
    map_quality_term = finite_term(map_quality, lambda value: 1.0 - value)
    extension_raw, extension_coverage = weighted_score([
        (residual_term, extension_weights[0]),
        (coverage_term, extension_weights[1]),
        (dynamic_term, extension_weights[2]),
        (uncertain_term, extension_weights[3]),
        (repeatability_term, extension_weights[4]),
        (map_quality_term, extension_weights[5]),
    ])
    extension_score = clamp(
        extension_raw / max(1.0e-9, float(extension_reference))
    )
    paper_weight = clamp(paper_weight)
    score = clamp(
        paper_weight * float(paper_score)
        + (1.0 - paper_weight) * extension_score
    )
    evidence = dict(paper_evidence)
    evidence.update({
        "paper_score_eq19": float(paper_score),
        "residual_p95_m_extension": float(residual_p95_m),
        "residual_term_extension": -1.0 if residual_term is None else residual_term,
        "spatial_coverage_extension": float(spatial_coverage),
        "spatial_coverage_term_extension": (
            -1.0 if coverage_term is None else coverage_term
        ),
        "dynamic_ratio_extension": float(dynamic_ratio),
        "dynamic_ratio_term_extension": -1.0 if dynamic_term is None else dynamic_term,
        "uncertain_ratio_extension": float(uncertain_ratio),
        "uncertain_ratio_term_extension": (
            -1.0 if uncertain_term is None else uncertain_term
        ),
        "feature_repeatability_extension": float(feature_repeatability),
        "repeatability_term_extension": (
            -1.0 if repeatability_term is None else repeatability_term
        ),
        "map_quality_extension": float(map_quality),
        "map_quality_term_extension": (
            -1.0 if map_quality_term is None else map_quality_term
        ),
        "extension_composite_raw": extension_raw,
        "extension_reference": float(extension_reference),
        "extension_score_normalized": extension_score,
        "extension_evidence_weight_coverage": extension_coverage,
        "paper_weight_final_score": paper_weight,
        "final_score_eq19_with_extensions": score,
    })
    reasons = list(paper_reasons)
    if extension_score > 0.5:
        reasons.append("map_protection_degraded_extension")
    if extension_coverage < 1.0 - 1.0e-9:
        reasons.append("incomplete_map_protection_evidence")
    return score, evidence, reasons


def gnss_score(q_fix, covariance_trace_m2, innovation_mahalanobis,
               tau_covariance=25.0, tau_innovation=5.0,
               weights=(0.25, 0.20, 0.55), hard_jump=False):
    fix_term = 1.0 - clamp(q_fix)
    covariance_term = min(1.0, max(0.0, covariance_trace_m2) / tau_covariance)
    innovation_term = None
    if innovation_mahalanobis >= 0.0:
        innovation_term = min(1.0, innovation_mahalanobis / tau_innovation)
    score, coverage = weighted_score([
        (fix_term, weights[0]),
        (covariance_term, weights[1]),
        (innovation_term, weights[2]),
    ])
    evidence = {
        "q_fix_eq23": float(q_fix),
        "covariance_trace_m2": float(covariance_trace_m2),
        "covariance_term_eq23": covariance_term,
        "innovation_mahalanobis": float(innovation_mahalanobis),
        "innovation_term_eq23": -1.0 if innovation_term is None else innovation_term,
    }
    reasons = []
    if fix_term > 0.5:
        reasons.append("invalid_fix_eq23")
    if covariance_term > 0.5:
        reasons.append("large_covariance_eq23")
    if innovation_term is not None and innovation_term > 0.5:
        reasons.append("large_innovation_eq23")
    if hard_jump:
        # A detected position discontinuity is an integrity failure, not just
        # another soft term.  Eq. 23 remains in the evidence for diagnostics,
        # while the scheduler receives a score that must disable this factor.
        score = 1.0
        evidence["jump_hard_gate"] = 1.0
        reasons.append("jump_hard_gate_eq23")
    return finalize_score(score, coverage, evidence, reasons)


def imu_score(excitation, preintegration_residual_mahalanobis, saturation,
              tau_imu=5.0, weights=(0.35, 0.45, 0.20)):
    excitation_term = 1.0 - clamp(excitation)
    residual_term = None
    if preintegration_residual_mahalanobis >= 0.0:
        residual_term = min(1.0, preintegration_residual_mahalanobis / tau_imu)
    saturation_term = 1.0 if saturation else 0.0
    score, coverage = weighted_score([
        (excitation_term, weights[0]),
        (residual_term, weights[1]),
        (saturation_term, weights[2]),
    ])
    evidence = {
        "excitation_eta_eq21": float(excitation),
        "excitation_term_eq21": excitation_term,
        "preintegration_residual_mahalanobis": float(preintegration_residual_mahalanobis),
        "residual_term_eq21": -1.0 if residual_term is None else residual_term,
        "saturation_indicator_eq21": saturation_term,
    }
    reasons = []
    if excitation_term > 0.5:
        reasons.append("low_excitation_observability_eq21")
    if residual_term is not None and residual_term > 0.5:
        reasons.append("large_preintegration_residual_eq21")
    if saturation:
        reasons.append("saturation_eq21")
    return finalize_score(score, coverage, evidence, reasons)


def imu_health_admission(
    paper_result,
    *,
    sample_finite=True,
    saturation=False,
    stream_valid=True,
    timestamp_valid=True,
    noise_anomaly=None,
    bias_anomaly=None,
):
    """Convert Eq. 21 diagnostics into the scheduler-facing IMU health gate.

    Low excitation is an observability diagnostic and preintegration NIS is a
    factor-consistency diagnostic.  Neither means that the propagation IMU is
    unhealthy, so they must not lower its admission weight.  Only direct
    hardware/data-path failures are hard gates here.  ``None`` for a noise or
    bias anomaly explicitly means that no validated detector is available.
    """
    paper_score, paper_evidence, paper_reasons = paper_result
    evidence = dict(paper_evidence)
    saturation = bool(
        saturation
        or float(evidence.get("saturation_indicator_eq21", 0.0)) >= 0.5
    )
    paper_coverage = float(evidence.get("evidence_weight_coverage", 0.0))
    paper_complete = float(evidence.get("score_complete", 0.0))
    evidence["paper_score_eq21"] = float(paper_score)
    evidence["paper_evidence_weight_coverage_eq21"] = paper_coverage
    evidence["paper_score_complete_eq21"] = paper_complete
    evidence["imu_observability_degradation_diagnostic"] = float(
        evidence.get("excitation_term_eq21", -1.0)
    )
    evidence["imu_factor_consistency_degradation_diagnostic"] = float(
        evidence.get("residual_term_eq21", -1.0)
    )

    health_failures = []
    if not bool(stream_valid):
        health_failures.append("imu_stream_outage_health_gate")
    if not bool(timestamp_valid):
        health_failures.append("imu_timestamp_health_gate")
    if not bool(sample_finite):
        health_failures.append("imu_nonfinite_sample_health_gate")
    if bool(saturation):
        health_failures.append("imu_saturation_health_gate")
    if noise_anomaly is True:
        health_failures.append("imu_noise_health_gate")
    if bias_anomaly is True:
        health_failures.append("imu_bias_health_gate")

    score = 1.0 if health_failures else 0.0
    evidence.update({
        "imu_health_admission_score": score,
        "imu_health_sample_finite": 1.0 if sample_finite else 0.0,
        "imu_health_stream_valid": 1.0 if stream_valid else 0.0,
        "imu_health_timestamp_valid": 1.0 if timestamp_valid else 0.0,
        "imu_health_saturation_clear": 0.0 if saturation else 1.0,
        "imu_noise_anomaly_check_available": (
            0.0 if noise_anomaly is None else 1.0
        ),
        "imu_bias_anomaly_check_available": (
            0.0 if bias_anomaly is None else 1.0
        ),
        "imu_health_admission_only": 1.0,
        # Admission evidence is complete even when the optional Eq. 21 NIS or
        # unvalidated noise/bias detectors are unavailable.
        "evidence_weight_coverage": 1.0,
        "score_complete": 1.0,
    })
    reasons = [
        f"diagnostic_only:{reason}" for reason in paper_reasons
    ]
    reasons.extend(health_failures)
    if noise_anomaly is None:
        reasons.append("imu_noise_health_gate_unavailable")
    if bias_anomaly is None:
        reasons.append("imu_bias_health_gate_unavailable")
    return finalize_score(score, 1.0, evidence, reasons)


def optical_flow_score(delta_position_flow_m, delta_position_prediction_m,
                       quality, ground_distance_m, tau_translation=0.30,
                       weights=(0.60, 0.25, 0.15),
                       allow_prediction_fallback=False):
    flow_norm = planar_norm(delta_position_flow_m)
    prediction_norm = planar_norm(delta_position_prediction_m)
    increment_residual = None
    increment_term = None
    if delta_position_flow_m is not None and delta_position_prediction_m is not None:
        if isinstance(delta_position_flow_m, (int, float)):
            increment_residual = abs(
                float(delta_position_flow_m) - float(delta_position_prediction_m)
            )
        else:
            flow = list(delta_position_flow_m)
            prediction = list(delta_position_prediction_m)
            increment_residual = math.hypot(
                float(flow[0]) - float(prediction[0]),
                float(flow[1]) - float(prediction[1]),
            )
        increment_term = min(1.0, increment_residual / tau_translation)
    quality_term = 1.0 - clamp(float(quality) / 255.0)
    distance_term = 0.0 if 0.10 <= float(ground_distance_m) <= 30.0 else 1.0
    prediction_fallback = bool(
        allow_prediction_fallback
        and increment_term is None
        and flow_norm is not None
    )
    if prediction_fallback:
        # A stale/missing LIO prediction must not turn an otherwise usable
        # optical-flow sample into an invalid factor.  This branch deliberately
        # omits Eq. 22's innovation term and records that loss of evidence.
        score, _ = weighted_score([
            (quality_term, weights[1]),
            (distance_term, weights[2]),
        ])
        coverage = 1.0
    else:
        score, coverage = weighted_score([
            (increment_term, weights[0]),
            (quality_term, weights[1]),
            (distance_term, weights[2]),
        ])
    evidence = {
        "delta_position_flow_m": -1.0 if flow_norm is None else flow_norm,
        "delta_position_prediction_m": (
            -1.0 if prediction_norm is None else prediction_norm
        ),
        "increment_residual_m_eq22_adapted": (
            -1.0 if increment_residual is None else increment_residual
        ),
        "increment_term_eq22_adapted": -1.0 if increment_term is None else increment_term,
        "quality": float(quality),
        "ground_distance_m": float(ground_distance_m),
    }
    reasons = []
    if increment_term is None:
        reasons.append("increment_prediction_unavailable_eq22_adapted")
        if prediction_fallback:
            reasons.append("prediction_fallback_quality_distance_only")
    elif increment_term > 0.5:
        reasons.append("increment_inconsistent_eq22_adapted")
    if quality_term > 0.5:
        reasons.append("low_quality_extension")
    if distance_term > 0.5:
        reasons.append("invalid_ground_distance_extension")
    if prediction_fallback:
        evidence["prediction_fallback_eq22_adapted"] = 1.0
    return finalize_score(score, coverage, evidence, reasons)


def vision_score(feature_count, feature_reference, spatial_uniformity,
                 reprojection_residual_px, depth_valid_ratio,
                 tau_reprojection_px=3.0,
                 weights=(0.30, 0.25, 0.25, 0.20)):
    feature_term = 1.0 - min(1.0, float(feature_count) / max(1.0, float(feature_reference)))
    uniformity_term = 1.0 - clamp(spatial_uniformity)
    reprojection_term = None
    if reprojection_residual_px >= 0.0:
        reprojection_term = min(1.0, reprojection_residual_px / tau_reprojection_px)
    depth_term = 1.0 - clamp(depth_valid_ratio)
    score, coverage = weighted_score([
        (feature_term, weights[0]),
        (uniformity_term, weights[1]),
        (reprojection_term, weights[2]),
        (depth_term, weights[3]),
    ])
    evidence = {
        "feature_count_eq20": float(feature_count),
        "feature_reference_eq20": float(feature_reference),
        "feature_term_eq20": feature_term,
        "spatial_uniformity_eq20": float(spatial_uniformity),
        "uniformity_term_eq20": uniformity_term,
        "reprojection_residual_px_eq20": float(reprojection_residual_px),
        "reprojection_term_eq20": -1.0 if reprojection_term is None else reprojection_term,
        "depth_valid_ratio_extension": float(depth_valid_ratio),
        "depth_term_extension": depth_term,
    }
    reasons = []
    if feature_term > 0.5:
        reasons.append("few_features_eq20")
    if uniformity_term > 0.5:
        reasons.append("poor_spatial_distribution_eq20")
    if reprojection_term is not None and reprojection_term > 0.5:
        reasons.append("large_reprojection_residual_eq20")
    if depth_term > 0.5:
        reasons.append("depth_holes_extension")
    return finalize_score(score, coverage, evidence, reasons)
