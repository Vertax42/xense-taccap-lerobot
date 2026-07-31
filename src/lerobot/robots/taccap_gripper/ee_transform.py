#!/usr/bin/env python

# Copyright 2026 The XenseRobotics Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""
Tracker → EEF TCP rigid transform for the TacCap-Gripper.

The Pico4 tracker is bolted to the gripper, so what the tracker reports is the
*tracker's* pose, not the TCP we actually want to record. This module owns the
constant rigid offset between the two, for both sides, and is the single place
the numbers live.

Frames
------
``Pico4TrackerReader`` already emits **world-frame** poses (X forward, Y left,
Z up, gravity-aligned) and then right-multiplies the offset::

    T_world_tcp = T_world_tracker @ X  # tracker.py

so ``X`` is body-fixed: it rides along with the gripper and is valid at any
runtime orientation. That is why the operator may start a UMI session with the
gripper pointing anywhere.

The catch that makes the CAD numbers unusable as-is
---------------------------------------------------
``tracker.py`` builds the world pose by conjugation, ``T = G · T · Gᵀ`` with
``G = PICO_TO_WORLD_R``. Conjugation re-expresses the *reference* frame **and
re-labels the tracker's own body axes**: ``+X`` of the frame ``X`` lives in is
the tracker's physical ``−Z_pico`` axis. Measured: a local ``+X`` offset of
0.1 m after conjugation moves the point by ``[0, +0.1, 0]`` in world, whereas
0.1 m along ``+X`` in the Pico frame then re-based gives ``[+0.1, 0, 0]``.

So a CAD number expressed in world/Pico axes has to be re-based first. With
``R_cad`` = the tracker frame's three axes as unit vectors in world coordinates,
in the CAD reference pose::

    T_world_tracker rotation = R_cad · Gᵀ
    X = [ G·R_cadᵀ ,  G·R_cadᵀ · delta_world ]

Both the rotation and the translation fall out of the same ``R_cad``.

The CAD reference pose
----------------------
``delta_world`` below was measured with the **EE frame aligned to the world
frame** (``R_world_ee = I``). That is a property of the measurement, not a
restriction on use: once re-based into the tracker body frame the transform is
orientation-independent (verified over random poses — see the module's
self-check in the repo notes).

Side symmetry
-------------
Left and right grippers are mirror images about the **XZ plane** (``y → −y``),
so only the right side carries measured numbers; the left is derived with
``M = diag(1, -1, 1)``::

    t_left = M · t_right          R_left = M · R_right · M

``det(M·R·M) = det(R) = +1``, so the mirrored result is a proper rotation, not a
reflection — it is a valid rigid transform and needs no fix-up.
"""

from __future__ import annotations

import numpy as np

from lerobot.teleoperators.pico4.tracker import PICO_TO_WORLD_R
from lerobot.utils.robot_utils import get_logger, matrix_to_pose7d, quaternion_to_matrix

logger = get_logger("taccap_ee_transform")

SIDES = ("left", "right")

# Mirror about the XZ plane: the left gripper is the right one flipped in y.
MIRROR_XZ = np.diag([1.0, -1.0, 1.0])

# ---------------------------------------------------------------- CAD numbers
#
# Source: media/right_eef_tcp.jpg — SolidWorks 测量 on 总装.SLDASM, between the
# assembly's "Tracker frame" and "EE frame", right gripper, distance 195.25 mm.
#
# SIGN CORRECTION: the dialog exports all three deltas positive, but only X is.
# Y and Z are negative on the real part. Do not "fix" these back to match the
# screenshot — the screenshot is what is wrong.
RIGHT_DELTA_WORLD_M = np.array([+0.179266402, -0.029203296, -0.071650795], dtype=np.float64)

# The tracker frame's three axes as unit vectors in world coordinates, in the
# CAD reference pose (EE frame aligned to world). Needed to re-base
# RIGHT_DELTA_WORLD_M into the tracker body frame — see cad_to_code_frame.
#
# PENDING: the screenshot only carries Euler angles (Φ=132.54° θ=45.00°
# Ψ=17.04°) and the SolidWorks convention for that dialog (axis order,
# intrinsic vs extrinsic) is not documented, so it cannot be reconstructed
# without guessing. Read the 3x3 out of SolidWorks with the measurement's
# reference coordinate system set to "Tracker frame" and drop it in here.
#
# Until then tracker_to_tcp() falls back to identity, i.e. the pre-existing
# behaviour where the recorded TCP *is* the tracker pose.
RIGHT_R_CAD: np.ndarray | None = None

_IDENTITY_POSE = (np.zeros(3, dtype=np.float64), np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64))

_warned_uncalibrated = False


def cad_to_code_frame(delta_world: np.ndarray, r_cad: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Re-base a CAD-measured tracker→EE offset into the frame ``tracker.py`` wants.

    Args:
        delta_world: tracker origin → EE origin, in world axes, measured in the
            reference pose where the EE frame is aligned with the world frame.
        r_cad: the tracker frame's axes as unit vectors in world coordinates, in
            that same reference pose (3x3, proper rotation).

    Returns:
        ``(pos, quat_wxyz)`` ready for ``tracker_to_ee_pos`` / ``_quat``.
    """
    delta_world = np.asarray(delta_world, dtype=np.float64).reshape(3)
    r_cad = np.asarray(r_cad, dtype=np.float64).reshape(3, 3)

    if not np.allclose(r_cad @ r_cad.T, np.eye(3), atol=1e-6):
        raise ValueError(f"r_cad is not orthonormal:\n{r_cad}")
    det = float(np.linalg.det(r_cad))
    if not np.isclose(det, 1.0, atol=1e-6):
        raise ValueError(
            f"r_cad must be a proper rotation (det=+1), got det={det:.6f}. "
            "A det of -1 means the axes are left-handed — check the axis order."
        )

    rot = PICO_TO_WORLD_R @ r_cad.T
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = rot
    matrix[:3, 3] = rot @ delta_world
    pose = matrix_to_pose7d(matrix, output_format="wxyz")
    return pose[:3].copy(), pose[3:7].copy()


def mirror_xz(pos: np.ndarray, quat: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Mirror a rigid transform about the XZ plane (``y → −y``).

    ``t' = M·t`` and ``R' = M·R·M``. Conjugating by the reflection twice keeps
    the determinant at +1, so the result is a rotation rather than a reflection
    and can be handed back as a quaternion unchanged.
    """
    pos = np.asarray(pos, dtype=np.float64).reshape(3)
    quat = np.asarray(quat, dtype=np.float64).reshape(4)

    matrix = quaternion_to_matrix(np.concatenate([pos, quat]), input_format="wxyz")
    mirrored = np.eye(4, dtype=np.float64)
    mirrored[:3, :3] = MIRROR_XZ @ matrix[:3, :3] @ MIRROR_XZ
    mirrored[:3, 3] = MIRROR_XZ @ matrix[:3, 3]
    pose = matrix_to_pose7d(mirrored, output_format="wxyz")
    return pose[:3].copy(), pose[3:7].copy()


def tracker_to_tcp(side: str) -> tuple[np.ndarray, np.ndarray]:
    """``(pos, quat_wxyz)`` from the tracker frame to the TCP, for ``side``.

    The right side carries the measured CAD numbers; the left is the right
    mirrored about the XZ plane. Falls back to identity (TCP == tracker, the
    behaviour before this module existed) while ``RIGHT_R_CAD`` is unset,
    warning once so a run cannot silently record uncalibrated poses.
    """
    global _warned_uncalibrated

    side = str(side).strip().lower()
    if side not in SIDES:
        raise ValueError(f"side must be one of {SIDES}, got {side!r}.")

    if RIGHT_R_CAD is None:
        if not _warned_uncalibrated:
            _warned_uncalibrated = True
            logger.warn(
                "tracker→TCP transform is not calibrated (RIGHT_R_CAD is unset in "
                "ee_transform.py), so the recorded tcp.* is the TRACKER pose, off by "
                "the ~195 mm handle offset. Fill in the tracker frame's 3x3 from CAD."
            )
        return _IDENTITY_POSE[0].copy(), _IDENTITY_POSE[1].copy()

    pos, quat = cad_to_code_frame(RIGHT_DELTA_WORLD_M, RIGHT_R_CAD)
    if side == "left":
        pos, quat = mirror_xz(pos, quat)
    return pos, quat


def resolve_tracker_to_ee(
    side: str,
    pos: tuple[float, float, float] | None,
    quat: tuple[float, float, float, float] | None,
) -> tuple[np.ndarray, np.ndarray]:
    """Pick the configured override, else this side's built-in transform.

    ``None`` means "use the built-in value"; either component can be overridden
    on its own, so a rig with a re-machined mount can pin just the translation.
    """
    built_in_pos, built_in_quat = tracker_to_tcp(side)
    out_pos = built_in_pos if pos is None else np.asarray(pos, dtype=np.float64).reshape(3)
    out_quat = built_in_quat if quat is None else np.asarray(quat, dtype=np.float64).reshape(4)
    return out_pos, out_quat
