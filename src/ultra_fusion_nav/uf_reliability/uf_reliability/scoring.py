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


def optical_flow_displacement_frd(integrated_x, integrated_y,
                                  integrated_xgyro, integrated_ygyro,
                                  ground_distance_m):
    """Invert MAVLink OPTICAL_FLOW_RAD geometry into planar sensor-FRD displacement."""
    values = (
        integrated_x, integrated_y, integrated_xgyro, integrated_ygyro,
        ground_distance_m,
    )
    if not all(math.isfinite(float(value)) for value in values):
        return None
    if float(ground_distance_m) <= 0.0:
        return None
    translational_x = float(integrated_x) - float(integrated_xgyro)
    translational_y = float(integrated_y) - float(integrated_ygyro)
    return (
        translational_y * float(ground_distance_m),
        -translational_x * float(ground_distance_m),
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


def optical_flow_score(delta_position_flow_m, delta_position_prediction_m,
                       quality, ground_distance_m, tau_translation=0.30,
                       weights=(0.60, 0.25, 0.15)):
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
    elif increment_term > 0.5:
        reasons.append("increment_inconsistent_eq22_adapted")
    if quality_term > 0.5:
        reasons.append("low_quality_extension")
    if distance_term > 0.5:
        reasons.append("invalid_ground_distance_extension")
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
