"""Small, deterministic SE(3) helpers using xyzw quaternion convention."""

import numpy as np
from scipy.spatial.transform import Rotation


def _skew(vector):
    x, y, z = np.asarray(vector, dtype=np.float64).reshape(3)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def _check_transform(transform):
    value = np.asarray(transform, dtype=np.float64)
    if value.shape != (4, 4) or not np.all(np.isfinite(value)):
        raise ValueError("transform must be a finite 4x4 matrix")
    if not np.allclose(value[3], [0.0, 0.0, 0.0, 1.0], atol=1.0e-10):
        raise ValueError("invalid homogeneous transform")
    rotation = value[:3, :3]
    if (
        not np.allclose(
            rotation.T @ rotation,
            np.eye(3),
            atol=1.0e-6,
            rtol=0.0,
        )
        or not np.isclose(np.linalg.det(rotation), 1.0, atol=1.0e-6)
    ):
        raise ValueError("transform rotation must belong to SO(3)")
    return value


def pose_matrix(translation, quaternion_xyzw):
    translation = np.asarray(translation, dtype=np.float64).reshape(3)
    quaternion = np.asarray(quaternion_xyzw, dtype=np.float64).reshape(4)
    norm = np.linalg.norm(quaternion)
    if not np.all(np.isfinite(np.concatenate(
            (translation, quaternion)))) or norm <= 1.0e-12:
        raise ValueError("pose is nonfinite or has zero quaternion")
    output = np.eye(4)
    output[:3, :3] = Rotation.from_quat(quaternion / norm).as_matrix()
    output[:3, 3] = translation
    return output


def compose(left, right):
    return _check_transform(left) @ _check_transform(right)


def inverse(transform):
    value = _check_transform(transform)
    output = np.eye(4)
    output[:3, :3] = value[:3, :3].T
    output[:3, 3] = -output[:3, :3] @ value[:3, 3]
    return output


def se3_exp(twist):
    value = np.asarray(twist, dtype=np.float64).reshape(6)
    if not np.all(np.isfinite(value)):
        raise ValueError("twist must be finite")
    rho = value[:3]
    phi = value[3:]
    theta = np.linalg.norm(phi)
    phi_hat = _skew(phi)
    if theta < 1.0e-8:
        rotation = np.eye(3) + phi_hat + 0.5 * phi_hat @ phi_hat
        left_jacobian = np.eye(3) + 0.5 * phi_hat + (phi_hat @ phi_hat) / 6.0
    else:
        rotation = Rotation.from_rotvec(phi).as_matrix()
        left_jacobian = (
            np.eye(3)
            + ((1.0 - np.cos(theta)) / (theta * theta)) * phi_hat
            + ((theta - np.sin(theta)) / (theta ** 3)) * (phi_hat @ phi_hat)
        )
    output = np.eye(4)
    output[:3, :3] = rotation
    output[:3, 3] = left_jacobian @ rho
    return output


def se3_log(transform):
    value = _check_transform(transform)
    phi = Rotation.from_matrix(value[:3, :3]).as_rotvec()
    theta = np.linalg.norm(phi)
    phi_hat = _skew(phi)
    if theta < 1.0e-8:
        inverse_left_jacobian = np.eye(
            3) - 0.5 * phi_hat + (phi_hat @ phi_hat) / 12.0
    else:
        coefficient = (
            1.0 / (theta * theta)
            - (1.0 + np.cos(theta)) / (2.0 * theta * np.sin(theta))
        )
        inverse_left_jacobian = np.eye(
            3) - 0.5 * phi_hat + coefficient * (phi_hat @ phi_hat)
    rho = inverse_left_jacobian @ value[:3, 3]
    return np.concatenate((rho, phi))


def transform_to_pose(transform):
    value = _check_transform(transform)
    return value[:3, 3].copy(), Rotation.from_matrix(value[:3, :3]).as_quat()
