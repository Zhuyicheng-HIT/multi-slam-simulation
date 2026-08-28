"""Offline SE(3) pose graph optimization with robust loop-edge quarantine."""

from dataclasses import dataclass

import numpy as np
from scipy.optimize import least_squares

from .se3 import compose, inverse, se3_exp, se3_log


@dataclass(frozen=True)
class OptimizerConfig:
    robust_phi: float = 25.0
    minimum_loop_weight: float = 0.05
    maximum_function_evaluations: int = 300
    maximum_translation_correction_m: float = 2.0
    maximum_rotation_correction_rad: float = np.deg2rad(45.0)
    maximum_sequential_translation_strain_m: float = 0.25
    maximum_sequential_rotation_strain_rad: float = np.deg2rad(15.0)

    def __post_init__(self):
        positive = (
            self.robust_phi,
            self.maximum_translation_correction_m,
            self.maximum_rotation_correction_rad,
            self.maximum_sequential_translation_strain_m,
            self.maximum_sequential_rotation_strain_rad,
        )
        if any(value <= 0.0 for value in positive):
            raise ValueError("optimizer limits must be positive")
        if not 0.0 <= self.minimum_loop_weight <= 1.0:
            raise ValueError("minimum_loop_weight must be in [0, 1]")
        if self.maximum_function_evaluations <= 0:
            raise ValueError("maximum_function_evaluations must be positive")


@dataclass(frozen=True)
class OptimizationResult:
    corrected_poses: dict
    converged: bool
    metrics: dict
    loop_status: dict


def edge_error(edge, poses):
    predicted = compose(inverse(poses[edge.source_id]), poses[edge.target_id])
    return se3_log(compose(inverse(edge.measurement), predicted))


def _whiten(error, edge):
    if edge.translation_sigma_m <= 0.0 or edge.rotation_sigma_rad <= 0.0:
        raise ValueError("edge sigmas must be positive")
    return np.concatenate((
        error[:3] / edge.translation_sigma_m,
        error[3:] / edge.rotation_sigma_rad,
    ))


def _dcs_weight(whitened_error, phi):
    chi_squared = float(np.dot(whitened_error, whitened_error))
    scale = min(1.0, (2.0 * phi) / (phi + chi_squared))
    return scale * scale


def _pose_dictionary(nodes, state):
    poses = {nodes[0].keyframe_id: nodes[0].original_pose}
    for index, node in enumerate(nodes[1:]):
        delta = state[index * 6:(index + 1) * 6]
        poses[node.keyframe_id] = compose(se3_exp(delta), node.original_pose)
    return poses


def _residual_function(nodes, sequential_edges, loop_edges, phi):
    def residual(state):
        poses = _pose_dictionary(nodes, state)
        blocks = []
        for edge in sequential_edges:
            blocks.append(_whiten(edge_error(edge, poses), edge))
        for edge in loop_edges:
            whitened = _whiten(edge_error(edge, poses), edge)
            blocks.append(
                edge.correlation_scale *
                np.sqrt(_dcs_weight(whitened, phi)) * whitened
            )
        return np.concatenate(blocks) if blocks else np.empty(0)
    return residual


def _solve(nodes, sequential_edges, loop_edges, config, initial_state=None):
    variables = 6 * (len(nodes) - 1)
    state = np.zeros(variables) if initial_state is None else np.asarray(
        initial_state, dtype=float)
    residual = _residual_function(
        nodes,
        sequential_edges,
        loop_edges,
        config.robust_phi)
    initial_cost = 0.5 * float(np.dot(residual(state), residual(state)))
    solved = least_squares(
        residual,
        state,
        jac="2-point",
        method="trf",
        x_scale="jac",
        max_nfev=config.maximum_function_evaluations,
        ftol=1.0e-12,
        xtol=1.0e-12,
        gtol=1.0e-12,
    )
    singular_values = np.linalg.svd(solved.jac, compute_uv=False)
    threshold = max(solved.jac.shape) * \
        np.finfo(float).eps * singular_values[0]
    rank = int(np.count_nonzero(singular_values > threshold))
    condition = float(singular_values[0] / singular_values[-1]
                      ) if singular_values[-1] > threshold else float("inf")
    return solved, initial_cost, rank, condition


def optimize_pose_graph(nodes, sequential_edges, loop_edges, config=None):
    config = config or OptimizerConfig()
    nodes = list(nodes)
    sequential_edges = list(sequential_edges)
    loop_edges = list(loop_edges)
    if len(nodes) < 2:
        raise ValueError("pose graph needs at least two nodes")
    node_ids = [node.keyframe_id for node in nodes]
    if len(node_ids) != len(set(node_ids)):
        raise ValueError("duplicate graph node")
    valid_ids = set(node_ids)
    for edge in sequential_edges + loop_edges:
        if edge.source_id not in valid_ids or edge.target_id not in valid_ids:
            raise ValueError("edge references unknown node")

    first, initial_cost, _, _ = _solve(
        nodes, sequential_edges, loop_edges, config)
    first_poses = _pose_dictionary(nodes, first.x)
    loop_status = {}
    active_loops = []
    for edge in loop_edges:
        whitened = _whiten(edge_error(edge, first_poses), edge)
        weight = _dcs_weight(whitened, config.robust_phi)
        decision = (
            "active"
            if weight >= config.minimum_loop_weight
            else "quarantined_low_robust_weight"
        )
        loop_status[edge.audit_id] = {
            "decision": decision,
            "robust_weight": weight,
            "chi_squared": float(np.dot(whitened, whitened)),
        }
        if decision == "active":
            active_loops.append(edge)

    solved, _, rank, condition = _solve(
        nodes, sequential_edges, active_loops, config, initial_state=first.x
    )
    corrected = _pose_dictionary(nodes, solved.x)
    final_residual = _residual_function(
        nodes, sequential_edges, active_loops, config.robust_phi
    )(solved.x)
    final_cost = 0.5 * float(np.dot(final_residual, final_residual))

    corrections = {
        node.keyframe_id: se3_log(
            compose(corrected[node.keyframe_id], inverse(node.original_pose)))
        for node in nodes
    }
    max_translation = max(np.linalg.norm(
        value[:3]) for value in corrections.values())
    max_rotation = max(np.linalg.norm(value[3:])
                       for value in corrections.values())
    sequential_errors = [edge_error(edge, corrected)
                         for edge in sequential_edges]
    max_sequential_translation = max(
        (np.linalg.norm(value[:3]) for value in sequential_errors), default=0.0
    )
    max_sequential_rotation = max(
        (np.linalg.norm(value[3:]) for value in sequential_errors), default=0.0
    )
    full_rank = 6 * (len(nodes) - 1)
    protected = (
        bool(solved.success)
        and np.all(np.isfinite(solved.x))
        and final_cost <= initial_cost + 1.0e-9
        and rank == full_rank
        and max_translation <= config.maximum_translation_correction_m
        and max_rotation <= config.maximum_rotation_correction_rad
        and max_sequential_translation <= config.maximum_sequential_translation_strain_m
        and max_sequential_rotation <= config.maximum_sequential_rotation_strain_rad
    )
    if not protected:
        raise RuntimeError(
            "pose graph solution failed convergence or distortion protection")

    metrics = {
        "success": True,
        "solver_status": int(solved.status),
        "solver_message": solved.message,
        "function_evaluations": int(solved.nfev),
        "initial_robust_cost": initial_cost,
        "final_robust_cost": final_cost,
        "effective_rank": rank,
        "variables": full_rank,
        "jacobian_condition_number": condition,
        "active_loop_edges": len(active_loops),
        "quarantined_loop_edges": len(loop_edges) - len(active_loops),
        "max_translation_correction_m": float(max_translation),
        "max_rotation_correction_rad": float(max_rotation),
        "max_sequential_translation_strain_m": float(max_sequential_translation),
        "max_sequential_rotation_strain_rad": float(max_sequential_rotation),
    }
    return OptimizationResult(corrected, True, metrics, loop_status)
