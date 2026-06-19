#!/usr/bin/env python

# Copyright 2026 The XenseRobotics Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""
Configuration for the bimanual TacCap-Gripper handheld data-collection rig.

Two independent TacCap-Gripper units (left + right), each = a motor-driven jaw
(encoder read-only), two embedded visuotactile sensors, a wrist UVC camera, an
IMU, plus a Pico4 Ultra motion tracker mounted on top for 6-DoF pose.

**Serial auto-discovery.** You no longer list device serials. The robot scans the
connected hardware at construct/connect time and assigns each gripper, tactile
sensor and wrist camera to ``left``/``right`` by the Xense serial rule (odd
sequence → left, even → right; patch ``m`` → Master/Leader, ``s`` → Slave/
Follower). See ``serial_discovery.py``. A serial that does not conform, or a side
whose hardware is missing/duplicated, raises a clear error so the config and the
physical serials can never drift out of alignment.

The Pico4 motion tracker is **also auto-discovered**: when ``enable_tracker`` is on,
the connected trackers are enumerated from the XenseVR PC service at startup and
assigned to ``left``/``right`` by the Pico serial rule (second-to-last digit odd →
left, even → right; e.g. ``PC2310MLL3200496G`` → ``6`` → right). A bimanual rig
requires one tracker per side; a missing / duplicate / malformed tracker raises a
clear error. Set ``enable_tracker=false`` to record tactile + gripper only.

To bypass the tracker side rule (e.g. a tracker whose serial does not follow it),
set ``left_tracker_serial`` / ``right_tracker_serial`` — a pinned side uses the
given serial verbatim and never touches enumeration; un-pinned sides still
auto-discover by rule.
"""

from dataclasses import dataclass, field

from ..config import RobotConfig

_SIDES = ("left", "right")

_DEFAULT_INIT_TCP_POSE = (
    0.693307,
    -0.114902,
    0.14589,
    0.004567,
    0.003238,
    0.999984,
    0.001246,
)


@RobotConfig.register_subclass("bi_taccap_gripper")
@dataclass
class BiTaccapGripperConfig(RobotConfig):
    """Configuration for the bimanual TacCap-Gripper data-collection rig.

    Grippers, tactile sensors and wrist cameras are auto-discovered by serial
    rule — no serials are listed here. Gripper position is normalised via
    ``clip(position_rad / {side}_gripper_open_rad, 0, 1)`` (0 = closed, fixed by
    the SDK's ``Encoder.set_zero()``; 1 = mechanical max).
    """

    # ---- Discovery --------------------------------------------------------
    role: str = "leader"
    """Which device role to bind for the handheld rig: ``leader`` (Master, patch
    ``m``) or ``follower`` (Slave, patch ``s``). Discovery binds only this role
    and errors if a side resolves to the other."""

    expected_tactiles_per_side: int = 2
    """How many tactile sensors each gripper carries (obs keys
    ``{side}_tactile_0`` / ``{side}_tactile_1``). Discovery errors if a side has
    a different count, catching a mis-installed/mis-burned sensor."""

    enable_tracker: bool = True
    """Auto-discover the Pico4 motion tracker(s) and record 6-DoF pose. When on,
    the XenseVR PC service is queried at startup and each connected tracker is
    assigned to left/right by its serial's second-to-last digit (odd → left, even
    → right). A bimanual rig must have one tracker per side (else an error). Set
    False to record tactile + gripper only (no PC service needed)."""

    # ---- Left TacCap unit -------------------------------------------------
    left_enable_gripper: bool = True
    left_enable_imu: bool = False
    left_gripper_open_rad: float = 1.7

    left_tracker_serial: str | None = None
    """Manually pin the left Pico4 tracker serial, bypassing the
    second-to-last-digit side rule. ``None`` = auto-discover by rule; when set, the
    serial is used verbatim (no enumeration, no rule check). Only when ``enable_tracker``."""
    left_tracker_to_ee_pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    left_tracker_to_ee_quat: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    left_enable_init_pose_alignment: bool = False
    left_init_tcp_pose: tuple[float, float, float, float, float, float, float] = (
        _DEFAULT_INIT_TCP_POSE
    )

    left_enable_wrist_camera: bool = True

    # ---- Right TacCap unit ------------------------------------------------
    right_enable_gripper: bool = True
    right_enable_imu: bool = False
    right_gripper_open_rad: float = 1.7

    right_tracker_serial: str | None = None
    """Manually pin the right Pico4 tracker serial, bypassing the
    second-to-last-digit side rule. ``None`` = auto-discover by rule; when set, the
    serial is used verbatim (no enumeration, no rule check). Only when ``enable_tracker``."""
    right_tracker_to_ee_pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    right_tracker_to_ee_quat: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    right_enable_init_pose_alignment: bool = False
    right_init_tcp_pose: tuple[float, float, float, float, float, float, float] = (
        _DEFAULT_INIT_TCP_POSE
    )

    right_enable_wrist_camera: bool = True

    # ---- Shared -----------------------------------------------------------
    tracker_wait_timeout: float = 10.0
    """Seconds to wait for the first valid tracker pose at connect time (both sides)."""

    tactile_fps: int = 30
    tactile_output_types: list[str] = field(default_factory=lambda: ["rectify"])
    """Defaults applied to every discovered tactile sensor. Single output type →
    one (H, W, 3) image (rectify is inference-free, landscape (400, 700, 3)).
    Width/height auto-derive from the SDK rectify_size — don't hard-code."""

    wrist_camera_width: int = 640
    wrist_camera_height: int = 480
    wrist_camera_fps: int = 30

    def __post_init__(self):
        super().__post_init__()
        if self.role.strip().lower() not in (
            "leader",
            "master",
            "follower",
            "slave",
        ):
            raise ValueError(
                f"role must be leader/master or follower/slave, got {self.role!r}."
            )
        for side in _SIDES:
            if getattr(self, f"{side}_enable_gripper") and getattr(
                self, f"{side}_gripper_open_rad"
            ) <= 0:
                raise ValueError(
                    f"{side}_gripper_open_rad must be positive, got "
                    f"{getattr(self, f'{side}_gripper_open_rad')}. Closed=0 is fixed by the "
                    "SDK's Encoder.set_zero(); open_rad is the mechanical-max angle "
                    "(TC-GU-01 default 1.7)."
                )
