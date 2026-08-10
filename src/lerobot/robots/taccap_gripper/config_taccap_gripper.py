#!/usr/bin/env python

# Copyright 2026 The XenseRobotics Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""
Configuration for the TacCap-Gripper handheld data-collection device.

Hardware:
- TacCap-Gripper handheld unit (XenseRobotics): motor-driven jaw, two
  embedded visuotactile sensors, wrist UVC camera, encoder, IMU.
  Driven by the ``xense.taccap`` SDK (``taccap-gripper`` PyPI package).
- Pico4 Ultra independent motion tracker physically mounted on top to
  provide 6-DoF pose. Reached via ``xensevr_pc_service_sdk``.

**Serial auto-discovery.** The gripper, its two tactile sensors and its wrist
camera are scanned from the connected hardware and matched by the Xense serial
rule (odd sequence → left, even → right; patch ``m`` → leader, ``s`` →
follower) — no serials are listed here (see ``serial_discovery.py``). With
both grippers connected, set ``side`` to pick one; otherwise the single connected
gripper is used. A non-conforming serial raises a clear error.

Recorded pose frame: our world frame (X forward away from base, Y left,
Z up, gravity-aligned) — ``Pico4TrackerReader`` applies the same Pico→world
remap the controller uses. The world origin is the headset position at
Unity-app launch time.
"""

from dataclasses import dataclass, field

from ..config import RobotConfig
from .common import build_head_camera_configs, validate_robot_id


@RobotConfig.register_subclass("taccap_gripper")
@dataclass
class TaccapGripperConfig(RobotConfig):
    """Configuration for the TacCap-Gripper handheld data-collection device.

    Gripper, tactile sensors and wrist camera are auto-discovered by serial rule.
    Gripper position is normalised via ``clip(position_rad / open_rad, 0, 1)``.
    ``position_rad`` is the SDK's cooked (post-zero, >=0-clamped) reading.
    The closed endpoint is **always 0** -- the SDK's ``Encoder.set_zero()``
    command latches the closed pose into firmware. Run the SDK's
    ``examples/calibrate.py`` once per device to set the zero; then this
    config only needs the mechanical-max ``gripper_open_rad`` (default
    1.7 = TC-GU-01 hardware stop).
    """

    # ---- Discovery --------------------------------------------------------
    role: str = "leader"
    """Device role to bind: ``leader`` (patch ``m``) or ``follower`` (patch ``s``)."""

    side: str | None = None
    """Which gripper to use, ``left`` or ``right``. ``None`` = auto when exactly
    one matching gripper/camera is connected; required when both sides are present."""

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
    """How many tactile sensors the gripper carries (obs keys ``tactile_left`` /
    ``tactile_right``). Sensors are paired to the gripper by USB hub; ``left`` /
    ``right`` finger comes from the GSPS serial's last digit (odd→left sensor,
    even→right). Discovery errors on a different count — which is the point, and
    why ``enable_tactile`` rather than a count of 0 is how you take the sensors
    out of a run."""

    # ---- TacCap gripper ---------------------------------------------------
    enable_gripper: bool = True
    enable_imu: bool = False
    """If True, also publish ``imu.{accel,gyro,mag}.{x,y,z}`` per observation."""

    gripper_open_rad: float = 1.7
    """Encoder reading (rad) when the jaw is fully open (gripper.pos = 1).
    Default 1.7 ~= TC-GU-01 mechanical limit (~97 deg). Closed (gripper.pos = 0)
    is always 0 rad -- set via the SDK's Encoder.set_zero().

    **Followers only.** A leader normalises from its own stored travel span, and
    connect() now refuses a leader that has none rather than falling back to
    this constant: one number cannot describe every unit ever built, and a jaw
    scaled by the wrong one produces a dataset nothing downstream can tell from
    a jaw that was never opened fully. Run
    ``third_party/taccap-gripper/python/examples/calibrate.py <left|right>``
    once per leader instead."""

    # ---- Pico4 Ultra tracker ---------------------------------------------
    enable_tracker: bool = True
    """Auto-discover the Pico4 motion tracker and record 6-DoF pose. When on, the
    XenseVR PC service is queried at startup; the tracker whose serial's
    second-to-last digit matches this unit's side (odd → left, even → right) is
    pinned. Set False to record tactile/gripper only (no PC service needed)."""

    tracker_serial: str | None = None
    """Manually pin this unit's Pico4 tracker serial, bypassing the
    second-to-last-digit side rule. ``None`` (default) = auto-discover by rule.
    When set, the serial is used **verbatim** — no PC-service enumeration, no
    rule check — the escape hatch for a tracker whose serial does not follow the
    rule (or when enumeration is flaky). Only consulted when ``enable_tracker``."""

    tracker_to_ee_pos: tuple[float, float, float] | None = None
    """Translation from the tracker frame to the gripper end-effector frame
    (meters). ``None`` (default) = this side's built-in value from
    ``ee_transform.tracker_to_tcp``, derived from the CAD mount geometry. Set it
    to override — e.g. a re-machined mount. The two components are independent,
    so the translation can be pinned while the rotation stays built-in."""

    tracker_to_ee_quat: tuple[float, float, float, float] | None = None
    """Rotation from the tracker frame to the gripper end-effector frame,
    [qw, qx, qy, qz]. ``None`` (default) = this side's built-in value (see
    ``tracker_to_ee_pos``)."""

    tracker_wait_timeout: float = 10.0
    """Seconds to wait for the first valid tracker pose at connect time."""

    # NOTE: no init-pose alignment here. Re-basing recorded poses onto a robot's
    # home pose needs that robot present and localised at connect time, which is
    # exactly what a handheld capture rig does not have — and it would tie the
    # dataset to one arm. Base-frame differences are instead cancelled downstream
    # by the relative-to-current pose representation. ``Pico4TrackerReader`` still
    # implements the alignment for live teleoperation; it is simply not wired up
    # on the capture path. See the README.

    # ---- Tactile sensors (Xense; auto-discovered by serial) --------------
    tactile_fps: int = 30
    tactile_output_types: list[str] = field(default_factory=lambda: ["rectify"])
    """The **recorded** tactile stream, applied to every discovered sensor. Exactly
    one output type (each sensor contributes one (H, W, 3) image to
    ``observation_features``, hence one dataset video key). Default ``rectify``:
    the unsubtracted image, which keeps every bit the sensor saw. The amplified
    ``difference`` view is easier to read live but is destructive — it is taken
    against a baseline captured at sensor init, so any pressure resting on the
    gel at connect is subtracted away for the whole run, and that must not reach
    the dataset. See ``tactile_display_output_types`` for the live view.
    Width/height are auto-derived from the SDK's rectify_size (do not hard-code)."""

    tactile_display_output_types: list[str] = field(default_factory=lambda: ["difference"])
    """Extra tactile streams requested from the sensor for **display only**.
    Default ``difference`` (SDK ``OutputType.AugDifference``): on this gel the raw
    ``rectify`` image carries so little visible deformation that contact is hard
    to read, and the augmented difference against the rest baseline amplifies it,
    so this is what the operator watches in Rerun.

    Each type here is published under ``{camera}_{type}`` (e.g.
    ``tactile_left_difference``). Those keys are deliberately absent from
    ``observation_features``, so ``build_dataset_frame`` never sees them and they
    never land on disk; they are what ``display_features`` puts in front of Rerun
    *instead of* the recorded stream. Both image types are inference-free and come
    from the same ``selectSensorInfo`` call, so the extra stream is cheap. Set to
    an empty list to skip it entirely (Rerun then shows the recorded stream)."""

    tactile_diff_gain: float | None = 1.0
    """Linear gain the SDK applies to the ``difference`` image
    (``ctx_patch.process.diff_gain``), i.e. to the display stream only. The sensors
    ship at 1.5, which is noisy and clips on this gel; 1.0 measures ~1.18 grey
    levels of per-pixel temporal noise instead of ~1.77 and stops saturating.
    Because it scales signal and noise together the SNR is unchanged — it buys
    headroom, not clarity. Set to None to leave whatever the sensor was flashed
    with."""

    # ---- Wrist camera (OpenCV UVC; auto-discovered by serial) ------------
    enable_wrist_camera: bool = True
    """Wire the wrist UVC camera under observation key ``wrist_cam`` (resolved
    from /dev/v4l/by-id by the discovered XC… serial)."""

    wrist_camera_width: int = 640
    wrist_camera_height: int = 480
    wrist_camera_fps: int = 30

    wrist_camera_fourcc: str | None = "MJPG"
    """Pixel format to negotiate with the wrist camera. Defaults to MJPEG
    because the alternative OpenCV would pick on its own (YUYV) reserves enough
    USB isochronous bandwidth to starve the tactile sensors sharing this
    gripper's hub — see ``build_wrist_camera_config``. ``None`` leaves the choice
    to OpenCV; ``"YUYV"`` forces the uncompressed stream, which is lossless but
    only fits when few enough cameras share the USB root port."""

    # ---- Pico head camera ---------------------------------------------------
    enable_head_camera: bool = False
    """Stream the headset's stereo camera as ``left_head`` /
    ``right_head`` (one key per eye), plus the headset
    pose as ``head_camera.*``. There is one headset regardless of how many
    grippers are in use, so this is the same stream the bimanual robot records
    — do not enable it on two single-arm processes at once and expect two
    independent views."""
    head_camera_eyes: str = "both"
    """``"both"`` records the eyes side by side, ``"left"``/``"right"`` one of
    them. Merged frames are ``head_camera_height x (2 * head_camera_width)``."""
    head_camera_width: int = 1024
    head_camera_height: int = 768
    """Per-eye size, following the stereo convention that width is one eye and
    a merge doubles it. Only 1024x768 and 1280x960 are supported — both 4:3,
    matching the sensor (PICO's camera-access API caps a frame at 2328x1748,
    which is 4:3, so a 16:9 request would crop or stretch rather than widen
    the field of view)."""
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
        # number is expanded to ``taccap_<n>`` — see ``validate_robot_id``.
        self.id = validate_robot_id(self.id, self.type)

        if self.enable_head_camera:
            # Delegate to the camera config so there is one definition of what
            # a valid mode is, rather than a copy here that can drift from it.
            build_head_camera_configs(self)

        if self.role.strip().lower() not in ("leader", "master", "follower", "slave"):
            # master/slave are legacy spellings kept for compatibility; see
            # serial_discovery._ROLE_ALIASES.
            raise ValueError(f"role must be leader or follower, got {self.role!r}.")
        if self.side is not None and self.side.strip().lower() not in ("left", "right"):
            raise ValueError(f"side must be left, right, or None, got {self.side!r}.")

        if self.enable_gripper and self.gripper_open_rad <= 0:
            raise ValueError(
                f"gripper_open_rad must be positive, got {self.gripper_open_rad}. "
                "Closed=0 is fixed by the SDK's Encoder.set_zero(); open_rad "
                "is the mechanical-max angle (TC-GU-01 default 1.7)."
            )

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
