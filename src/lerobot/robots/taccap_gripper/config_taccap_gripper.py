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
rule (odd sequence → left, even → right; patch ``m`` → Master/Leader, ``s`` →
Slave/Follower) — no serials are listed here (see ``serial_discovery.py``). With
both grippers connected, set ``side`` to pick one; otherwise the single connected
gripper is used. A non-conforming serial raises a clear error.

Recorded pose frame: our world frame (X forward away from base, Y left,
Z up, gravity-aligned) — ``Pico4TrackerReader`` applies the same Pico→world
remap the controller uses. The world origin is the headset position at
Unity-app launch time.
"""

from dataclasses import dataclass, field

from ..config import RobotConfig


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
    """Device role to bind: ``leader`` (Master, patch ``m``) or ``follower``
    (Slave, patch ``s``)."""

    side: str | None = None
    """Which gripper to use, ``left`` or ``right``. ``None`` = auto when exactly
    one matching gripper/camera is connected; required when both sides are present."""

    expected_tactiles_per_side: int = 2
    """How many tactile sensors the gripper carries (obs keys ``tactile_left`` /
    ``tactile_right``). Sensors are paired to the gripper by USB hub; ``left`` /
    ``right`` finger comes from the GSPS serial's last digit (odd→left sensor,
    even→right). Discovery errors on a different count."""

    # ---- TacCap gripper ---------------------------------------------------
    enable_gripper: bool = True
    enable_imu: bool = False
    """If True, also publish ``imu.{accel,gyro,mag}.{x,y,z}`` per observation."""

    gripper_open_rad: float = 1.7
    """Encoder reading (rad) when the jaw is fully open (gripper.pos = 1).
    Default 1.7 ~= TC-GU-01 mechanical limit (~97 deg). Override per unit
    if your sample varies. Closed (gripper.pos = 0) is always 0 rad --
    set via the SDK's Encoder.set_zero() (run calibrate.py once)."""

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

    tracker_to_ee_pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    """Translation from the tracker frame to the gripper end-effector frame
    (meters). Defaults to zero (i.e. EE coincides with tracker)."""

    tracker_to_ee_quat: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    """Rotation from the tracker frame to the gripper end-effector frame,
    [qw, qx, qy, qz]. Defaults to identity."""

    tracker_wait_timeout: float = 10.0
    """Seconds to wait for the first valid tracker pose at connect time."""

    # ---- UMI-style init-pose alignment (reserved, off by default) --------
    enable_init_pose_alignment: bool = False
    """If True, snapshot the first valid tracker pose at connect time
    and compute a rigid transform so all subsequent recorded poses are
    in the same frame as ``init_tcp_pose`` (typically the deployment
    robot's base frame at the home configuration). Mirrors
    ``vive_tracker``'s UMI behaviour. Off by default
    pending live verification on real deployment hardware."""

    init_tcp_pose: tuple[float, float, float, float, float, float, float] = (
        0.693307, -0.114902, 0.14589, 0.004567, 0.003238, 0.999984, 0.001246,
    )
    """Robot TCP pose at the operator's "init" stance, as
    ``[x, y, z, qw, qx, qy, qz]``, in the world frame. Default is an example
    deployment robot's home pose. Only consumed when
    ``enable_init_pose_alignment`` is True."""

    # ---- Tactile sensors (Xense; auto-discovered by serial) --------------
    tactile_fps: int = 30
    tactile_output_types: list[str] = field(default_factory=lambda: ["rectify"])
    """Defaults applied to every discovered tactile sensor. A single output type
    yields one (H, W, 3) image; ``rectify`` is inference-free. Width/height are
    auto-derived from the SDK's rectify_size (do not hard-code them — the rectify
    array is (400, 700, 3))."""

    # ---- Wrist camera (OpenCV UVC; auto-discovered by serial) ------------
    enable_wrist_camera: bool = True
    """Wire the wrist UVC camera under observation key ``wrist_cam`` (resolved
    from /dev/v4l/by-id by the discovered XC… serial)."""

    wrist_camera_width: int = 640
    wrist_camera_height: int = 480
    wrist_camera_fps: int = 30

    def __post_init__(self):
        super().__post_init__()

        if self.role.strip().lower() not in ("leader", "master", "follower", "slave"):
            raise ValueError(
                f"role must be leader/master or follower/slave, got {self.role!r}."
            )
        if self.side is not None and self.side.strip().lower() not in ("left", "right"):
            raise ValueError(f"side must be left, right, or None, got {self.side!r}.")

        if self.enable_gripper and self.gripper_open_rad <= 0:
            raise ValueError(
                f"gripper_open_rad must be positive, got {self.gripper_open_rad}. "
                "Closed=0 is fixed by the SDK's Encoder.set_zero(); open_rad "
                "is the mechanical-max angle (TC-GU-01 default 1.7)."
            )
