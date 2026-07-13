import math


def clamp(value, low=0.0, high=1.0):
    return max(low, min(high, float(value)))


def normalized_sum(terms):
    available = [(clamp(value), float(weight)) for value, weight in terms if value is not None]
    total = sum(weight for _, weight in available)
    return clamp(sum(value * weight for value, weight in available) / total) if total else 1.0


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
    score = normalized_sum([
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
    return score, evidence, reasons


def gnss_score(q_fix, covariance_trace_m2, innovation_mahalanobis,
               tau_covariance=25.0, tau_innovation=5.0,
               weights=(0.25, 0.20, 0.55)):
    fix_term = 1.0 - clamp(q_fix)
    covariance_term = min(1.0, max(0.0, covariance_trace_m2) / tau_covariance)
    innovation_term = None
    if innovation_mahalanobis >= 0.0:
        innovation_term = min(1.0, innovation_mahalanobis / tau_innovation)
    score = normalized_sum([
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
    return score, evidence, reasons


def imu_score(excitation, preintegration_residual_mahalanobis, saturation,
              tau_imu=5.0, weights=(0.35, 0.45, 0.20)):
    excitation_term = 1.0 - clamp(excitation)
    residual_term = None
    if preintegration_residual_mahalanobis >= 0.0:
        residual_term = min(1.0, preintegration_residual_mahalanobis / tau_imu)
    saturation_term = 1.0 if saturation else 0.0
    score = normalized_sum([
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
        reasons.append("low_excitation_eq21")
    if residual_term is not None and residual_term > 0.5:
        reasons.append("large_preintegration_residual_eq21")
    if saturation:
        reasons.append("saturation_eq21")
    return score, evidence, reasons


def optical_flow_score(delta_position_flow_m, delta_position_prediction_m,
                       quality, ground_distance_m, tau_translation=0.30,
                       weights=(0.60, 0.25, 0.15)):
    increment_residual = abs(float(delta_position_flow_m) - float(delta_position_prediction_m))
    increment_term = min(1.0, increment_residual / tau_translation)
    quality_term = 1.0 - clamp(float(quality) / 255.0)
    distance_term = 0.0 if 0.10 <= float(ground_distance_m) <= 30.0 else 1.0
    score = normalized_sum([
        (increment_term, weights[0]),
        (quality_term, weights[1]),
        (distance_term, weights[2]),
    ])
    evidence = {
        "delta_position_flow_m": float(delta_position_flow_m),
        "delta_position_prediction_m": float(delta_position_prediction_m),
        "increment_residual_m_eq22_adapted": increment_residual,
        "increment_term_eq22_adapted": increment_term,
        "quality": float(quality),
        "ground_distance_m": float(ground_distance_m),
    }
    reasons = []
    if increment_term > 0.5:
        reasons.append("increment_inconsistent_eq22_adapted")
    if quality_term > 0.5:
        reasons.append("low_quality_extension")
    if distance_term > 0.5:
        reasons.append("invalid_ground_distance_extension")
    return score, evidence, reasons


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
    score = normalized_sum([
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
    return score, evidence, reasons
