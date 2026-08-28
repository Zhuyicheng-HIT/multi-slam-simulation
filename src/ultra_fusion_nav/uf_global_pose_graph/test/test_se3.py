import numpy as np
import pytest

from uf_global_pose_graph.se3 import (
    compose,
    inverse,
    pose_matrix,
    se3_exp,
    se3_log,
)


def test_se3_exp_log_round_trip():
    twist = np.array([0.31, -0.17, 0.08, 0.13, -0.09, 0.21])

    recovered = se3_log(se3_exp(twist))

    np.testing.assert_allclose(recovered, twist, atol=1.0e-10)


def test_pose_inverse_composes_to_identity():
    pose = pose_matrix(
        np.array([1.2, -0.4, 0.7]),
        np.array([0.1, -0.2, 0.3, 0.92]),
    )

    np.testing.assert_allclose(
        compose(
            pose,
            inverse(pose)),
        np.eye(4),
        atol=1.0e-12)


@pytest.mark.parametrize(
    "rotation",
    (
        np.diag([1.0, 1.0, -1.0]),
        np.diag([1.0, 1.0, 1.01]),
    ),
)
def test_se3_rejects_reflection_and_nonorthogonal_rotation(rotation):
    transform = np.eye(4)
    transform[:3, :3] = rotation

    with pytest.raises(ValueError, match="SO\\(3\\)"):
        inverse(transform)
