from __future__ import annotations

import numpy as np

from calibration_math import calibrated_ee_transform_world, estimated_ee_point_world, solve_pivot
from lerobot.utils.robot_utils import quaternion_to_matrix


def _pose(x, y, z, qw, qx, qy, qz):
    return quaternion_to_matrix(np.array([x, y, z, qw, qx, qy, qz], dtype=np.float64), input_format="wxyz")


def test_solve_pivot_recovers_tracker_to_ee_pos():
    tracker_to_ee = np.array([0.035, -0.012, 0.081], dtype=np.float64)
    fixed_world = np.array([0.45, -0.16, 0.32], dtype=np.float64)
    rotations = [
        _pose(0, 0, 0, 1, 0, 0, 0),
        _pose(0, 0, 0, 0.9238795, 0.3826834, 0, 0),
        _pose(0, 0, 0, 0.9238795, 0, 0.3826834, 0),
        _pose(0, 0, 0, 0.9238795, 0, 0, 0.3826834),
        _pose(0, 0, 0, 0.8660254, 0.2886751, 0.2886751, 0.2886751),
    ]

    samples = []
    for transform in rotations:
        rotation = transform[:3, :3]
        transform[:3, 3] = fixed_world - rotation @ tracker_to_ee
        samples.append(transform)

    result = solve_pivot(samples)

    assert np.allclose(result.tracker_to_ee_pos, tracker_to_ee, atol=1e-6)
    assert np.allclose(result.fixed_point_world, fixed_world, atol=1e-6)
    assert result.rmse_m < 1e-6
    assert result.rank == 6
    for sample in samples:
        assert np.allclose(estimated_ee_point_world(sample, result.tracker_to_ee_pos), fixed_world, atol=1e-6)


def test_calibrated_ee_transform_matches_estimated_point():
    world_from_tracker = _pose(0.10, -0.20, 0.30, 0.9238795, 0, 0, 0.3826834)
    tracker_to_ee = np.array([0.035, -0.012, 0.081], dtype=np.float64)

    world_from_ee = calibrated_ee_transform_world(world_from_tracker, tracker_to_ee)

    assert np.allclose(world_from_ee[:3, 3], estimated_ee_point_world(world_from_tracker, tracker_to_ee))
    assert np.allclose(world_from_ee[:3, :3], world_from_tracker[:3, :3])
