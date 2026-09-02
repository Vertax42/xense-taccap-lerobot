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
sequence → left, even → right; patch ``m`` → leader, ``s`` → follower). See ``serial_discovery.py``. A serial that does not conform, or a side
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
from ..taccap_gripper.common import (
    build_head_camera_configs,
    validate_robot_id,
    validate_wrist_undistort_size,
)

_SIDES = ("left", "right")

# NOTE: no init-pose alignment fields here. Re-basing recorded poses onto a
# robot's home pose needs that robot present and localised at connect time, which
# a handheld capture rig does not have — and it would tie the dataset to one arm.
# Base-frame differences are cancelled downstream by the relative-to-current pose
# representation instead. ``Pico4TrackerReader`` still implements the alignment
# for live teleoperation; it is simply not wired up on the capture path.


@RobotConfig.register_subclass("bi_taccap_gripper")
@dataclass
class BiTaccapGripperConfig(RobotConfig):
    """Configuration for the bimanual TacCap-Gripper data-collection rig.

    Grippers, tactile sensors and wrist cameras are auto-discovered by serial
    rule — no serials are listed here. Gripper position is normalised via
    ``clip(position_rad / {side}_gripper_open_rad, 0, 1)`` for followers (0 =
    closed, fixed by the SDK's ``Encoder.set_zero()``; 1 = mechanical max).
    Leaders use their own firmware-stored travel span and refuse to connect
    without one.
    """

    # ---- Discovery --------------------------------------------------------
    role: str = "leader"
    """Which device role to bind for the handheld rig: ``leader`` (patch ``m``)
    or ``follower`` (patch ``s``). Discovery binds only this role
    and errors if a side resolves to the other."""

    enable_tactile: bool = True
    """Wire the tactile sensors at all. ``False`` skips discovery, the camera
    configs and the observation keys entirely — the sensors are simply not part
    of the rig for this run.

    Off is a diagnostic, not a way to operate: tactile data is the reason this
    gripper exists. It is here because a rig whose cameras will not all open is
    hard to reason about, and being able to take the four tactile streams out
    (or, with the wrist flags, take the wrist streams out) turns "something
    fails" into an arithmetic question about USB isochronous budget — one
    bus, 480 Mbit/s, ~384 of it available. See the README's troubleshooting
    section.

    Prefer this over setting ``expected_tactiles_per_side`` to 0: the count says
    how many sensors a gripper *carries*, and discovery errors when it finds a
    different number, so it is the wrong knob for "not this time"."""

    expected_tactiles_per_side: int = 2
    """How many tactile sensors each gripper carries (obs keys
    ``{side}_tactile_left`` / ``{side}_tactile_right``). Sensors are paired to a
    gripper by USB hub (hub → gripper firmware SN → ``side``); ``left`` / ``right``
    finger comes from the GSPS serial's last digit (odd→left sensor, even→right).
    Discovery errors if a side has a different count, catching a mis-installed/
    mis-burned sensor — which is the point, and why ``enable_tactile`` rather
    than a count of 0 is how you take the sensors out of a run."""

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
    """Encoder reading (rad) at which the left jaw counts as fully open.
    **Followers only** — a leader normalises from its own stored travel span,
    and connect() refuses a leader that has none rather than falling back to
    this constant (see ``common.open_gripper``). Calibrate with
    ``examples/calibrate.py left``."""

    left_tracker_serial: str | None = None
    """Manually pin the left Pico4 tracker serial, bypassing the
    second-to-last-digit side rule. ``None`` = auto-discover by rule; when set, the
    serial is used verbatim (no enumeration, no rule check). Only when ``enable_tracker``."""
    left_tracker_to_ee_pos: tuple[float, float, float] | None = None
    left_tracker_to_ee_quat: tuple[float, float, float, float] | None = None
    """Tracker → EE rigid mount transform for this side. ``None`` (default) =
    the built-in value from ``ee_transform.tracker_to_tcp("left")``, measured off
    the CAD assembly (both sides are measured; neither is mirrored from the
    other). Set to override."""

    left_enable_wrist_camera: bool = True

    # ---- Right TacCap unit ------------------------------------------------
    right_enable_gripper: bool = True
    right_enable_imu: bool = False
    right_gripper_open_rad: float = 1.7
    """Encoder reading (rad) at which the right jaw counts as fully open.
    **Followers only** — a leader normalises from its own stored travel span,
    and connect() refuses a leader that has none rather than falling back to
    this constant (see ``common.open_gripper``). Calibrate with
    ``examples/calibrate.py right``."""

    right_tracker_serial: str | None = None
    """Manually pin the right Pico4 tracker serial, bypassing the
    second-to-last-digit side rule. ``None`` = auto-discover by rule; when set, the
    serial is used verbatim (no enumeration, no rule check). Only when ``enable_tracker``."""
    right_tracker_to_ee_pos: tuple[float, float, float] | None = None
    right_tracker_to_ee_quat: tuple[float, float, float, float] | None = None
    """Tracker → EE rigid mount transform for this side. ``None`` (default) =
    the built-in value from ``ee_transform.tracker_to_tcp("right")``, measured
    off the CAD assembly. Set to override."""

    right_enable_wrist_camera: bool = True

    # ---- Shared -----------------------------------------------------------
    tracker_wait_timeout: float = 10.0
    """Seconds to wait for the first valid tracker pose at connect time (both sides)."""

    gripper_stream_hz: int = 100
    """Rate at which the gripper firmware pushes encoder samples (and, with that
    side's ``enable_imu``, IMU samples) over the MCU link. ``get_observation`` then
    reads a cache that the SDK's transport thread keeps current, and the record
    loop never waits on the bus for the jaw.

    ``0`` restores per-frame polling: ``Encoder::read_once``, a synchronous
    ``GetEncoder`` command and ACK wait on the record loop, once per gripper per
    frame — two round-trips on a bimanual rig —, across a USB bus six cameras are saturating. Sub-millisecond on a
    quiet bus by the SDK's own measurement; its tail is unbounded, which is the
    problem for a loop with single-digit milliseconds of headroom.

    The firmware divides a 1 kHz tick, so only divisors of 1000 arrive exactly
    (the SDK warns when it rounds). 100 is its default and gives a jaw reading
    at most 10 ms old — the same order of staleness every camera frame already
    has through ``async_read``. Leaders only: follower firmware streams motor
    status and nothing else, so a follower keeps polling whatever this says. If
    the stream cannot be brought up the guard logs and polls; it does not
    refuse to record."""

    tactile_fps: int = 30
    tactile_output_types: list[str] = field(default_factory=lambda: ["rectify"])
    """The **recorded** tactile stream, applied to every discovered sensor. Exactly
    one output type → one (H, W, 3) image per sensor, i.e. one dataset video key.
    Default ``rectify``, the unsubtracted image: the amplified ``difference`` view
    is taken against a baseline captured at sensor init, so any pressure resting on
    a gel at connect would be subtracted out of the whole recording. Width/height auto-derive from the SDK rectify_size — don't
    hard-code."""

    tactile_display_output_types: list[str] = field(default_factory=lambda: ["rectify"])
    """The tactile stream the operator watches in **Rerun**, which need not be the
    recorded one. Default ``rectify``, the same type ``tactile_output_types``
    records: what is on screen is what lands on disk.

    A display type equal to the recorded one collapses to a single sensor request
    and no extra key — Rerun is fed the recorded ``{side}_tactile_{left,right}``.
    Any other type is published under ``{camera}_{type}`` (e.g.
    ``left_tactile_right_difference``), deliberately absent from
    ``observation_features`` — it never reaches the dataset — and
    ``display_features`` puts it in front of Rerun *instead of* the recorded
    stream. The alternative to know about is ``difference`` (SDK
    ``OutputType.AugDifference``), which amplifies deformation against the rest
    baseline; it is inference-free and comes from the same ``selectSensorInfo``
    call, so it costs little. Its baseline is taken at sensor init, so if you turn
    it back on keep all four fingers unloaded at connect. An empty list also means
    "show the recorded stream".

    ``difference`` **was** the default, for the gel this rig shipped with: raw
    ``rectify`` barely showed deformation on it. The silicone was changed in
    2026-08 and contact now reads directly off ``rectify``, so the amplified view
    is no longer worth putting a non-recorded stream in front of the operator."""

    tactile_diff_gain: float | None = 1.0
    """Linear gain applied to the ``difference`` image
    (``ctx_patch.process.diff_gain``, stock 1.5). Inert unless ``difference`` is
    actually requested, which the defaults no longer do; it is applied at
    ``Sensor.create()`` regardless, so it holds the moment that view is put back.
    1.0 gives roughly a third less per-pixel temporal noise and no clipping; it
    scales signal and noise alike, so SNR is unchanged. None leaves the sensor's
    flashed value."""

    wrist_camera_width: int = 640
    wrist_camera_height: int = 480
    wrist_camera_fps: int = 30

    wrist_undistort: bool = False
    """Rectify both wrist fisheye streams before the frames are recorded.

    Shared by the two arms like the other ``wrist_camera_*`` settings, but the
    intrinsics are per unit: each gripper is asked for its own, so one arm can be
    rectified from its flash while the other falls back to the SDK reference.

    **Off by default, and turning it on changes what lands in the dataset**: a
    rectified ``{side}_wrist`` and a raw fisheye one have identical shape and
    dtype, so the two are not interchangeable and nothing downstream can tell
    them apart. Which of the two each arm used is written into
    ``meta/hardware.json`` for exactly that reason. See the single-arm config for
    what the reference fallback costs."""

    wrist_undistort_balance: float = 0.0
    """Output focal length, ``0`` = the calibrated value (also the PC calibration
    tool's default), ``1`` = 0.70x for the widest field of view at the cost of
    more black border. Only fx/fy move; the principal point stays put. Clamped to
    [0, 1] by the SDK."""

    wrist_camera_fourcc: str | None = "MJPG"
    """Pixel format to negotiate with both wrist cameras. Defaults to MJPEG
    because the alternative OpenCV would pick on its own (YUYV) reserves enough
    USB isochronous bandwidth to starve the tactile sensors sharing that
    gripper's hub — see ``build_wrist_camera_config``. Doubly so here: a bimanual
    rig runs six UVC devices, three per gripper hub. ``None`` leaves the choice
    to OpenCV; ``"YUYV"`` forces the uncompressed stream."""

    # ---- Pico head camera ---------------------------------------------------
    enable_head_camera: bool = False
    """Stream the headset's stereo camera as ``left_head`` /
    ``right_head`` (one key per eye), plus the headset
    pose as ``head_camera.*``. Shares the Pico SDK connection with the
    trackers, so it needs the headset app streaming either way."""
    head_camera_eyes: str = "both"
    """``"both"`` records the eyes side by side, ``"left"``/``"right"`` one of
    them. Merged frames are ``head_camera_height x (2 * head_camera_width)``."""
    head_camera_width: int = 640
    head_camera_height: int = 480
    """Per-eye size, following the stereo convention that width is one eye and
    a merge doubles it. Only 640x480 (the headset app's own default, and this
    one), 1024x768 and 1280x960 are supported — all 4:3, matching the sensor
    (PICO's camera-access API caps a frame at 2328x1748, which is 4:3, so a
    16:9 request would crop or stretch rather than widen the field of view).
    The headset is what produces the frames, so this has to match the app's
    Resolution setting or ``connect()`` fails on the first frame's size."""
    head_camera_fps: int = 30
    head_camera_startup_timeout_s: float = 5.0
    head_camera_stale_after_s: float = 0.2
    head_camera_pair_max_skew_ms: float = 20.0
    """How far apart the two eyes' timestamps may be and still count as one
    stereo capture, when their sequence numbers differ."""

    @property
    def tactiles_per_side(self) -> int:
        """How many tactile sensors to actually look for: the configured count,
        or 0 when ``enable_tactile`` is off.

        One accessor so the two fields cannot be read inconsistently — every
        site that used to ask for ``expected_tactiles_per_side`` (discovery, the
        SDK-required check, side resolution, the camera-config builder) means
        this instead.
        """
        return self.expected_tactiles_per_side if self.enable_tactile else 0

    def __post_init__(self):
        super().__post_init__()
        # Required here, not left optional as upstream has it, and a bare
        # number is expanded to ``bi_taccap_<n>`` — see ``validate_robot_id``.
        self.id = validate_robot_id(self.id, self.type)
        if self.role.strip().lower() not in (
            "leader",
            "master",
            "follower",
            "slave",
        ):
            raise ValueError(f"role must be leader or follower, got {self.role!r}.")
        if self.enable_head_camera:
            # Delegate to the camera config so there is one definition of what
            # a valid mode is, rather than a copy here that can drift from it.
            build_head_camera_configs(self)
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
        if self.wrist_undistort:
            if not any(getattr(self, f"{side}_enable_wrist_camera") for side in _SIDES):
                raise ValueError(
                    "wrist_undistort=True but neither arm has a wrist camera enabled — there "
                    "is no wrist stream to rectify."
                )
            validate_wrist_undistort_size(self.wrist_camera_width, self.wrist_camera_height)

        for side in _SIDES:
            if getattr(self, f"{side}_enable_gripper") and getattr(self, f"{side}_gripper_open_rad") <= 0:
                raise ValueError(
                    f"{side}_gripper_open_rad must be positive, got "
                    f"{getattr(self, f'{side}_gripper_open_rad')}. Closed=0 is fixed by the "
                    "SDK's Encoder.set_zero(); open_rad is the mechanical-max angle "
                    "(TC-GU-01 default 1.7)."
                )
