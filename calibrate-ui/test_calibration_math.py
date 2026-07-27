from __future__ import annotations

import numpy as np

from calibration_math import (
    SOLIDWORKS_TRACKER_TO_EE_POS_M,
    SOLIDWORKS_TRACKER_TO_EE_QUAT_WXYZ,
    SOLIDWORKS_TRACKER_TO_EE_RPY_DEG,
    calibrated_ee_transform_world,
    estimated_ee_point_world,
    rpy_degrees_to_quat_wxyz,
    solve_pivot,
)
from lerobot.utils.robot_utils import quaternion_to_matrix


def _pose(x, y, z, qw, qx, qy, qz):
    return quaternion_to_matrix(np.array([x, y, z, qw, qx, qy, qz], dtype=np.float64), input_format="wxyz")


def _quat_same_rotation(a, b):
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    return np.allclose(a, b, atol=1e-5) or np.allclose(a, -b, atol=1e-5)


def test_solve_pivot_recovers_tracker_to_ee_pos():
    tracker_to_ee = np.array([0.035, -0.012, 0.081], dtype=np.float64)
    tracker_to_ee_quat = rpy_degrees_to_quat_wxyz(12.0, -4.0, 33.0)
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

    result = solve_pivot(samples, tracker_to_ee_quat=tracker_to_ee_quat)

    assert np.allclose(result.tracker_to_ee_pos, tracker_to_ee, atol=1e-6)
    assert _quat_same_rotation(result.tracker_to_ee_quat, tracker_to_ee_quat)
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


def test_calibrated_ee_transform_applies_tracker_to_ee_quat():
    world_from_tracker = _pose(0.10, -0.20, 0.30, 0.9238795, 0, 0, 0.3826834)
    tracker_to_ee_pos = np.array([0.035, -0.012, 0.081], dtype=np.float64)
    tracker_to_ee_quat = rpy_degrees_to_quat_wxyz(30.0, -10.0, 45.0)

    world_from_ee = calibrated_ee_transform_world(world_from_tracker, tracker_to_ee_pos, tracker_to_ee_quat)
    expected_tracker_from_ee = quaternion_to_matrix(
        np.concatenate([tracker_to_ee_pos, tracker_to_ee_quat]),
        input_format="wxyz",
    )

    assert np.allclose(world_from_ee, world_from_tracker @ expected_tracker_from_ee)


def test_solidworks_rpy_matches_solidworks_quat():
    for side in ("left", "right"):
        quat_from_rpy = rpy_degrees_to_quat_wxyz(*SOLIDWORKS_TRACKER_TO_EE_RPY_DEG[side])
        assert _quat_same_rotation(quat_from_rpy, SOLIDWORKS_TRACKER_TO_EE_QUAT_WXYZ[side])


def test_solidworks_full_transform_constants_are_applied():
    world_from_tracker = np.eye(4, dtype=np.float64)

    for side in ("left", "right"):
        pos = np.asarray(SOLIDWORKS_TRACKER_TO_EE_POS_M[side], dtype=np.float64)
        quat = np.asarray(SOLIDWORKS_TRACKER_TO_EE_QUAT_WXYZ[side], dtype=np.float64)
        world_from_ee = calibrated_ee_transform_world(world_from_tracker, pos, quat)
        expected = quaternion_to_matrix(np.concatenate([pos, quat]), input_format="wxyz")

        assert np.allclose(world_from_ee, expected, atol=1e-8)
