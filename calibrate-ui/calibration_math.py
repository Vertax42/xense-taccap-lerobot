#!/usr/bin/env python
"""Math helpers for TacCap tracker-to-EE pivot calibration.

The calibration target is the gripper's fixed contact point / EE origin. During
sampling that point is held on one fixed point in space while the tracker is
rotated through several poses. The solve estimates the point's coordinates in
the tracker frame.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from lerobot.utils.robot_utils import quaternion_to_matrix

IDENTITY_QUAT_WXYZ = (1.0, 0.0, 0.0, 0.0)

# Same Pico->world remap used by lerobot.teleoperators.pico4.tracker.
PICO_TO_WORLD_R = np.array(
    [
        [0.0, 0.0, -1.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ],
    dtype=np.float64,
)


@dataclass(frozen=True)
class PivotResult:
    tracker_to_ee_pos: np.ndarray
    tracker_to_ee_quat: np.ndarray
    fixed_point_world: np.ndarray
    residuals_m: np.ndarray
    rmse_m: float
    mean_error_m: float
    max_error_m: float
    rank: int
    condition: float


def raw_pico_pose_wxyz_to_world_matrix(raw_pose: np.ndarray) -> np.ndarray:
    """Convert raw Pico tracker pose to a world-frame transform.

    Args:
        raw_pose: [x, y, z, qw, qx, qy, qz] in the Pico/XRT frame.

    Returns:
        4x4 T_world_tracker using the TacCap gravity-aligned world convention.
    """
    pose = np.asarray(raw_pose, dtype=np.float64)
    if pose.shape != (7,):
        raise ValueError(f"raw_pose must have shape (7,), got {pose.shape}")

    t_pico_tracker = quaternion_to_matrix(pose, input_format="wxyz").astype(np.float64)
    pico_to_world = np.eye(4, dtype=np.float64)
    pico_to_world[:3, :3] = PICO_TO_WORLD_R
    world_to_pico = np.eye(4, dtype=np.float64)
    world_to_pico[:3, :3] = PICO_TO_WORLD_R.T
    return pico_to_world @ t_pico_tracker @ world_to_pico


def solve_pivot(
    world_from_tracker_samples: list[np.ndarray],
    min_samples: int = 4,
) -> PivotResult:
    """Solve tracker-frame EE origin from fixed-point pivot samples.

    Each sample obeys:
        t_i + R_i @ p_tracker_ee = p_world_fixed

    Unknowns are p_tracker_ee and p_world_fixed. The returned quaternion is
    identity because fixed-point pivot calibration does not observe EE axes.
    """
    if len(world_from_tracker_samples) < min_samples:
        raise ValueError(f"Need at least {min_samples} samples, got {len(world_from_tracker_samples)}")

    a_rows: list[np.ndarray] = []
    b_rows: list[np.ndarray] = []
    for sample in world_from_tracker_samples:
        t_world_tracker = np.asarray(sample, dtype=np.float64)
        if t_world_tracker.shape != (4, 4):
            raise ValueError(f"Each sample must be a 4x4 matrix, got {t_world_tracker.shape}")
        rotation = t_world_tracker[:3, :3]
        translation = t_world_tracker[:3, 3]
        a_rows.append(np.hstack([rotation, -np.eye(3, dtype=np.float64)]))
        b_rows.append(-translation)

    a = np.vstack(a_rows)
    b = np.concatenate(b_rows)
    solution, *_rest = np.linalg.lstsq(a, b, rcond=None)
    rank = int(np.linalg.matrix_rank(a))
    singular_values = np.linalg.svd(a, compute_uv=False)
    if singular_values.size == 0 or singular_values[-1] <= 1e-12:
        condition = float("inf")
    else:
        condition = float(singular_values[0] / singular_values[-1])

    tracker_to_ee_pos = solution[:3]
    fixed_point_world = solution[3:6]

    errors: list[float] = []
    for sample in world_from_tracker_samples:
        rotation = sample[:3, :3]
        translation = sample[:3, 3]
        predicted = translation + rotation @ tracker_to_ee_pos
        errors.append(float(np.linalg.norm(predicted - fixed_point_world)))
    residuals = np.asarray(errors, dtype=np.float64)

    return PivotResult(
        tracker_to_ee_pos=tracker_to_ee_pos,
        tracker_to_ee_quat=np.asarray(IDENTITY_QUAT_WXYZ, dtype=np.float64),
        fixed_point_world=fixed_point_world,
        residuals_m=residuals,
        rmse_m=float(np.sqrt(np.mean(np.square(residuals)))),
        mean_error_m=float(np.mean(residuals)),
        max_error_m=float(np.max(residuals)),
        rank=rank,
        condition=condition,
    )


def estimated_ee_point_world(world_from_tracker: np.ndarray, tracker_to_ee_pos: np.ndarray) -> np.ndarray:
    """World position of the calibrated EE origin for one tracker pose."""
    transform = np.asarray(world_from_tracker, dtype=np.float64)
    pos = np.asarray(tracker_to_ee_pos, dtype=np.float64)
    return transform[:3, 3] + transform[:3, :3] @ pos


def calibrated_ee_transform_world(world_from_tracker: np.ndarray, tracker_to_ee_pos: np.ndarray) -> np.ndarray:
    """World-frame transform of the calibrated EE frame for one tracker pose.

    Fixed-point pivot calibration estimates only the tracker-frame EE origin.
    Until a separate orientation calibration is added, EE axes are kept aligned
    with the tracker axes and only the local origin offset is applied.
    """
    transform = np.asarray(world_from_tracker, dtype=np.float64)
    pos = np.asarray(tracker_to_ee_pos, dtype=np.float64)
    if transform.shape != (4, 4):
        raise ValueError(f"world_from_tracker must have shape (4, 4), got {transform.shape}")
    if pos.shape != (3,):
        raise ValueError(f"tracker_to_ee_pos must have shape (3,), got {pos.shape}")
    tracker_from_ee = np.eye(4, dtype=np.float64)
    tracker_from_ee[:3, 3] = pos
    return transform @ tracker_from_ee


def format_vector(values: np.ndarray | list[float] | tuple[float, ...], precision: int = 6) -> str:
    arr = np.asarray(values, dtype=np.float64).reshape(-1)
    return "[" + ", ".join(f"{float(v):.{precision}f}" for v in arr) + "]"


def cli_vector(values: np.ndarray | list[float] | tuple[float, ...], precision: int = 9) -> str:
    arr = np.asarray(values, dtype=np.float64).reshape(-1)
    return "[" + ",".join(f"{float(v):.{precision}g}" for v in arr) + "]"
