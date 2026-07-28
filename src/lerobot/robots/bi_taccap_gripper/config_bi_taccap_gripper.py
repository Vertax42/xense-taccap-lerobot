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
    ``{side}_tactile_left`` / ``{side}_tactile_right``). Sensors are paired to a
    gripper by USB hub (hub → gripper firmware SN → ``side``); ``left`` / ``right``
    finger comes from the GSPS serial's last digit (odd→left sensor, even→right).
    Discovery errors if a side has a different count, catching a mis-installed/
    mis-burned sensor."""

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
    """The **recorded** tactile stream, applied to every discovered sensor. Exactly
    one output type → one (H, W, 3) image per sensor, i.e. one dataset video key.
    Default ``rectify``, the unsubtracted image: the amplified ``difference`` view
    is easier to read live but is taken against a baseline captured at sensor init,
    so any pressure resting on a gel at connect would be subtracted out of the
    whole recording. Width/height auto-derive from the SDK rectify_size — don't
    hard-code."""

    tactile_display_output_types: list[str] = field(default_factory=lambda: ["difference"])
    """Extra tactile streams requested for **display only**. Default ``difference``
    (SDK ``OutputType.AugDifference``), which amplifies deformation the raw
    ``rectify`` image barely shows, so it is what the operator watches in Rerun.

    Each type is published under ``{camera}_{type}`` (e.g.
    ``left_tactile_right_difference``). Those keys are deliberately absent from
    ``observation_features`` — they never reach the dataset — and ``display_features``
    puts them in front of Rerun *instead of* the recorded stream. Both image types
    are inference-free and come from one ``selectSensorInfo`` call, so the extra
    stream is cheap; an empty list skips it (Rerun then shows the recorded stream).
    The difference baseline is taken at sensor init, so keep all four fingers
    unloaded at connect."""

    tactile_diff_gain: float | None = 1.0
    """Linear gain applied to the ``difference`` image
    (``ctx_patch.process.diff_gain``, stock 1.5), i.e. to the display stream only.
    1.0 gives roughly a third less per-pixel temporal noise and no clipping; it
    scales signal and noise alike, so SNR is unchanged. None leaves the sensor's
    flashed value."""

    wrist_camera_width: int = 640
    wrist_camera_height: int = 480
    wrist_camera_fps: int = 30

    # ---- Insight head camera ----------------------------------------------
    enable_head_camera: bool = False
    """Enable one process-global Insight RGB/VIO head camera."""
    head_camera_library_path: str | None = None
    head_camera_width: int = 1024
    head_camera_height: int = 768
    """Landscape crop taken from the sensor's fixed 1088x1920 portrait frame.

    1024x768 is 4:3 at 0.94x of the largest 4:3 region available, keeping
    72.0 x 57.2 of the 72.0 x 104.1 degrees the camera actually delivers.
    """
    head_camera_crop_bias: float = 0.5
    """Where that crop sits on the tall axis: 0.0 top, 0.5 centre, 1.0 bottom."""
    head_camera_fps: int = 30
    head_camera_startup_timeout_s: float = 5.0
    head_camera_stale_after_s: float = 0.2
    head_camera_stale_timeout_s: float = 3.0

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
        if self.enable_head_camera:
            if self.head_camera_width <= 0 or self.head_camera_height <= 0:
                raise ValueError(
                    "head_camera_width/head_camera_height must be positive, got "
                    f"{self.head_camera_width}x{self.head_camera_height}."
                )
            if self.head_camera_fps <= 0:
                raise ValueError("head_camera_fps must be positive.")
            if self.head_camera_startup_timeout_s <= 0:
                raise ValueError("head_camera_startup_timeout_s must be positive.")
            if self.head_camera_stale_after_s <= 0:
                raise ValueError("head_camera_stale_after_s must be positive.")
            if self.head_camera_stale_timeout_s <= 0:
                raise ValueError("head_camera_stale_timeout_s must be positive.")
        # One recorded stream per sensor: observation_features declares a single
        # (H, W, 3) per tactile camera, so a second recorded type would silently
        # hand build_dataset_frame a dict instead of an image. Extra live views
        # belong in tactile_display_output_types.
        if len(self.tactile_output_types) != 1:
            raise ValueError(
                "tactile_output_types must name exactly one recorded output type, "
                f"got {self.tactile_output_types}. For an extra Rerun-only view use "
                "--robot.tactile_display_output_types."
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
