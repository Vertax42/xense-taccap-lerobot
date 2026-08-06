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

Where the numbers come from
---------------------------
A SolidWorks macro reads the assembly's ``Tracker frame`` and ``EE frame``
coordinate systems and reports ``^Tracker T_EE`` directly — EE expressed in the
tracker frame, which is exactly the factor above. Both sides are measured; the
left is **not** derived by mirroring (see below).

Cross-checked against the standalone measurement in ``media/right_eef_tcp.jpg``
(the same two frames, reported in world axes at the reference pose where the EE
frame is aligned to world, so ``R_world_ee = I``)::

    delta_world  ==  R_tracker_eeᵀ · t_tracker_ee
    predicted (179.266402, -29.203296, -71.650795) mm
    screenshot  (179.266402, -29.203296, -71.650795) mm      agree to 0.000 mm

That also settles the screenshot's sign question: its Y and Z really are
negative, as the macro reports independently.

Side symmetry — checked, not assumed
------------------------------------
The two sides are near-mirrors about the XZ plane (``y → −y``) but not exactly:

* rotation agrees with the mirror to **0.03°**
* translation differs by **1.27 mm** (``|t|`` 195.25 mm right vs 194.09 left)

So both sides carry their own measured values and :func:`mirror_xz` is kept only
as a consistency check. Deriving one side from the other would bake in that
1.27 mm.

The ``G`` re-basing — settled, no re-base needed
-----------------------------------------------
``tracker.py`` builds the world pose by conjugation, ``T = G · T · Gᵀ`` with
``G = PICO_TO_WORLD_R``, which re-expresses the reference frame **and re-labels
the tracker's own body axes**. So the CAD ``Tracker frame`` either matches that
re-labelled convention (use the values as-is) or the device's physical datum (a
further ``G`` first). The two land the EE 168 mm apart, 51° off each other, and
both at the right 195 mm from the tracker — so the numbers alone cannot tell
them apart, only looking at the drawn frames can.

**Checked on hardware (2026-08-02): as-is is correct.** The EE frame sits at the
two-finger midpoint in the Rerun ``/world`` view, so ``APPLY_G_REBASE`` stays
``False``. This is a property of the CAD file, not of a session — it does not
need re-checking unless the assembly's coordinate systems are redrawn.
"""

from __future__ import annotations

import numpy as np

from lerobot.teleoperators.pico4.tracker import PICO_TO_WORLD_R
from lerobot.utils.robot_utils import get_logger, matrix_to_pose7d, quaternion_to_matrix

logger = get_logger("taccap_ee_transform")

SIDES = ("left", "right")

# Mirror about the XZ plane: the left gripper is the right one flipped in y.
MIRROR_XZ = np.diag([1.0, -1.0, 1.0])

# ------------------------------------------------------------ measured values
#
# ^Tracker T_EE straight out of the SolidWorks macro (assembly `总装.SLDASM`,
# coordinate systems "Tracker frame" and "EE frame"). Metres, and quaternions in
# this repo's scalar-first [qw, qx, qy, qz].
#
# These are LEADER (patch `m`) grippers. The follower body is a different
# design — different joint origins, flipped jaw axis, fingertips 21 mm further
# out — so these values do not carry over; see `tracker_to_tcp`.
TRACKER_TO_EE_POS_M: dict[str, tuple[float, float, float]] = {
    "left": (-0.160768654, -0.105859381, 0.024897320),
    "right": (-0.161933698, 0.106110099, 0.025322636),
}
TRACKER_TO_EE_QUAT_WXYZ: dict[str, tuple[float, float, float, float]] = {
    "left": (0.136862131, -0.378705573, 0.913588080, -0.056636271),
    "right": (0.136839046, 0.378463784, 0.913688271, 0.056692009),
}

# Whether to re-base the values above through G before use — see the module
# docstring. Verified False on hardware (2026-08-02): the EE frame lands on the
# two-finger midpoint in Rerun, so the CAD `Tracker frame` is already in the
# re-labelled convention and the values apply directly.
#
# Kept as a flag rather than deleted because it is the one thing that would have
# to change if the assembly's coordinate systems are ever redrawn, and because
# flipping it is how you would re-test that.
APPLY_G_REBASE = False

_warned_follower = False


def rebase_through_g(pos: np.ndarray, quat: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Re-express a device-frame transform in the G-relabelled body frame.

    Needed only if the CAD ``Tracker frame`` was drawn on the device's physical
    datum rather than in the world-axis convention ``tracker.py`` conjugates into
    — see ``APPLY_G_REBASE``. Left-multiplying by ``G`` converts a vector from
    device coordinates to the re-labelled ones.
    """
    matrix = quaternion_to_matrix(
        np.concatenate([np.asarray(pos, dtype=np.float64).reshape(3), np.asarray(quat, dtype=np.float64).reshape(4)]),
        input_format="wxyz",
    )
    rebased = np.eye(4, dtype=np.float64)
    rebased[:3, :3] = PICO_TO_WORLD_R @ matrix[:3, :3]
    rebased[:3, 3] = PICO_TO_WORLD_R @ matrix[:3, 3]
    pose = matrix_to_pose7d(rebased, output_format="wxyz")
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


def tracker_to_tcp(side: str, role: str = "leader") -> tuple[np.ndarray, np.ndarray]:
    """``(pos, quat_wxyz)`` from the tracker frame to the TCP, for ``side``.

    Both sides carry their own measured values; nothing is derived by mirroring
    (the two differ by 1.27 mm, see the module docstring).

    Only **leader** bodies are measured. The follower gripper is a different
    design — its URDF has different joint origins, a flipped jaw axis and
    fingertips 21 mm further out — so the leader numbers would put its TCP about
    2 cm off. Asking for a follower warns once and returns the leader value
    rather than failing a run mid-session.
    """
    global _warned_follower

    side = str(side).strip().lower()
    if side not in SIDES:
        raise ValueError(f"side must be one of {SIDES}, got {side!r}.")

    if str(role).strip().lower() not in ("leader", "master", "m") and not _warned_follower:
        _warned_follower = True
        logger.warn(
            f"tracker→TCP is only measured for leader grippers; using the leader value for "
            f"role={role!r}. The follower body differs (fingertips ~21 mm further out), so "
            "its TCP will be off by roughly that much until the follower is measured too."
        )

    pos = np.array(TRACKER_TO_EE_POS_M[side], dtype=np.float64)
    quat = np.array(TRACKER_TO_EE_QUAT_WXYZ[side], dtype=np.float64)
    if APPLY_G_REBASE:
        pos, quat = rebase_through_g(pos, quat)
    return pos, quat


def resolve_tracker_to_ee(
    side: str,
    pos: tuple[float, float, float] | None,
    quat: tuple[float, float, float, float] | None,
    role: str = "leader",
) -> tuple[np.ndarray, np.ndarray]:
    """Pick the configured override, else this side's built-in transform.

    ``None`` means "use the built-in value"; either component can be overridden
    on its own, so a rig with a re-machined mount can pin just the translation.
    """
    built_in_pos, built_in_quat = tracker_to_tcp(side, role)
    out_pos = built_in_pos if pos is None else np.asarray(pos, dtype=np.float64).reshape(3)
    out_quat = built_in_quat if quat is None else np.asarray(quat, dtype=np.float64).reshape(4)
    return out_pos, out_quat
