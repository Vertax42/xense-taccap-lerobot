#!/usr/bin/env python

# Copyright 2026 The XenseRobotics Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""
TacCap-Gripper handheld data-collection device for LeRobot.

This is a passive recording device — ``send_action()`` is a no-op. The
gripper motor is **not** enabled; we read the encoder only. Pose comes
from a Pico4 Ultra independent tracker physically mounted on top
(``Pico4TrackerReader``). Tactile and wrist cameras are configured via
the standard ``cameras`` framework.

The gripper, its two tactile sensors and its wrist camera are
**auto-discovered by serial rule** (``serial_discovery.py``) — no serials are
listed in the config.

Observation features:
    tcp.x, tcp.y, tcp.z              -- Pico4 tracker → EE position (m)
    tcp.r1..tcp.r6                   -- 6D rotation representation
    gripper.pos                      -- normalised jaw [0=closed, 1=open]
    imu.accel.{x,y,z} (optional)     -- m/s²
    imu.gyro.{x,y,z}  (optional)     -- rad/s
    imu.mag.{x,y,z}   (optional)     -- µT
    tactile_left / tactile_right     -- recorded tactile frames (sensor on
                                        left/right finger), ``rectify`` by default
    wrist_cam                        -- wrist UVC frame (if enable_wrist_camera)
    left_head / right_head     -- Pico headset camera, one key per eye
                                        (if enable_head_camera; head_camera_eyes
                                        can select a single eye)
    head_camera.x/y/z/r1..r6         -- headset pose, same world frame as tcp.*
                                        (also an action -- see action_features)

Display-only keys (present in ``get_observation()`` and in ``display_features``,
absent from ``observation_features``, so Rerun shows them but the dataset never
sees them — see ``tactile_display_output_types``):
    tactile_{left,right}_difference  -- amplified deformation view of the same
                                        sensor read
"""

from __future__ import annotations

from functools import cached_property
from typing import Any

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.utils.robot_utils import get_logger

from ..robot import Robot
from . import serial_discovery as disco
from .camera_health import CameraReadGuard
from .common import (
    HEAD_POSE_KEYS,
    POSE_KEYS,
    HeadSkewMonitor,
    build_head_camera_configs,
    build_tactile_camera_configs,
    build_wrist_camera_config,
    connect_cameras_parallel,
    disconnect_cameras_parallel,
    open_gripper,
    prewarm_tactile_config_cache,
    read_gripper_normalized,
    read_head_pose,
    split_camera_read,
    swap_tactile_display_features,
    tactile_camera_output_types,
)
from .config_taccap_gripper import TaccapGripperConfig
from .ee_transform import resolve_tracker_to_ee

# ---- TacCap-Gripper SDK -----------------------------------------------------
try:
    from xense.taccap import (
        FollowerGripper,
        LeaderGripper,
    )

    TACCAP_SDK_AVAILABLE = True
    _TACCAP_SDK_IMPORT_ERROR: ImportError | None = None
except ImportError as e:
    TACCAP_SDK_AVAILABLE = False
    _TACCAP_SDK_IMPORT_ERROR = e

# ---- Pico4 tracker reader ---------------------------------------------------
try:
    from lerobot.teleoperators.pico4.tracker import Pico4TrackerReader

    PICO4_TRACKER_AVAILABLE = True
except ImportError:
    PICO4_TRACKER_AVAILABLE = False


class TaccapGripper(Robot):
    """TacCap-Gripper handheld data-collection device.

    The device is operated manually — there is no action signal applied
    to the gripper (motor stays disabled). It records pose + jaw state
    + tactile + wrist for downstream policy learning.

    6D rotation convention (matches ``vive_tracker``):
        r1..r3 = first column of the rotation matrix
        r4..r6 = second column of the rotation matrix
    """

    config_class = TaccapGripperConfig
    name = "taccap_gripper"

    def __init__(self, config: TaccapGripperConfig):
        super().__init__(config)
        self.config = config
        self.logger = get_logger(f"TaccapGripper-{config.id or 'default'}")
        self._role = disco.normalize_role(config.role)

        # Tactile discovery now pairs sensors to a gripper by USB hub, so it also
        # needs the SDK (scan_grippers) to resolve the hub's side.
        needs_sdk = config.enable_gripper or config.expected_tactiles_per_side > 0
        if needs_sdk and not TACCAP_SDK_AVAILABLE:
            raise ImportError(
                "xense.taccap SDK not available. Build it from the vendored "
                "submodule third_party/taccap-gripper (run setup_env.sh --install). "
                f"Original import error: {_TACCAP_SDK_IMPORT_ERROR!r}"
            ) from _TACCAP_SDK_IMPORT_ERROR
        if config.enable_tracker and not PICO4_TRACKER_AVAILABLE:
            raise ImportError(
                "Pico4TrackerReader not available. Ensure src/lerobot/teleoperators/pico4/tracker.py is importable."
            )

        # Hardware handles, populated on connect.
        self._gripper: Any = None  # Leader/FollowerGripper
        self._endpoints: Any = None  # xense.taccap.GripperEndpoints
        self._tracker: Pico4TrackerReader | None = None

        # Discover devices → resolve which side this single unit is, then build its
        # tactile + wrist camera configs. Tactiles are paired to a gripper by USB
        # hub (scans the serial bus); wrist cameras are filesystem-only.
        self._disc_tactiles = (
            disco.discover_tactiles_by_hub(self._role)
            if config.expected_tactiles_per_side
            else {"left": {}, "right": {}}
        )
        self._disc_cameras = disco.discover_wrist_cameras(self._role) if config.enable_wrist_camera else {}
        self._side = self._resolve_side()
        self._camera_configs = self._build_camera_configs(self._side)
        self.cameras = make_cameras_from_configs(self._camera_configs)
        self._head_pose_warned = False
        self._head_skew = HeadSkewMonitor(config.head_camera_pair_max_skew_ms, self.logger)

        # Auto-discover the Pico4 motion tracker for this unit's side: enumerate
        # from the XenseVR PC service and pick the one whose serial's second-to-last
        # digit matches this side (strict). Drives the pose schema (pre-connect).
        self._tracker_sn: str | None = None
        if config.enable_tracker:
            self._tracker_sn = disco.resolve_pico_trackers(
                (self._side,),
                {self._side: config.tracker_serial},
                lambda: Pico4TrackerReader.list_serial_numbers(
                    device_wait_timeout=config.tracker_wait_timeout,
                    logger_name=config.id or "robot",
                ),
            )[self._side]
            source = "manual" if (config.tracker_serial or "").strip() else "rule"
            self.logger.info(f"Pico4 tracker ({self._side}): {self._tracker_sn} ({source})")

        self._is_connected = False
        # Where gripper.pos comes from once connected: "firmware" (the device's
        # own encoder-max calibration) or "config" (gripper_open_rad).
        self._gripper_norm_source = "config"

        # Graceful degradation on mid-episode camera loss (hot-unplug, hub drop):
        # substitutes the last good frame and trips ``device_lost`` so the caller
        # can stop cleanly and save what was recorded. See ``camera_health``.
        self._cam_guard = CameraReadGuard(self._camera_configs, self.logger)

    # ------------------------------------------------------------------ discovery

    def _resolve_side(self) -> str:
        """Pick the gripper side: ``config.side`` wins; otherwise infer from the
        discovered devices (camera when the wrist is enabled, else tactiles)."""
        if self.config.side:
            return self.config.side.strip().lower()
        if self.config.enable_wrist_camera:
            present = set(self._disc_cameras.keys())
        elif self.config.expected_tactiles_per_side:
            n = self.config.expected_tactiles_per_side
            present = {s for s in disco.SIDES if len(self._disc_tactiles.get(s, {})) == n}
        else:
            present = set()
        if len(present) == 1:
            return next(iter(present))
        if not present:
            raise RuntimeError(
                f"No {self._role} TacCap device discovered to infer a side; connect one or set --robot.side=left|right."
            )
        raise RuntimeError(f"Both sides present {sorted(present)}; set --robot.side=left|right to pick one.")

    def _build_camera_configs(self, side: str) -> dict[str, Any]:
        """Build ``tactile_{left,right}`` + ``wrist_cam`` configs for ``side``.

        Also fills ``self._tactile_display_keys`` (camera → {output type →
        display-only observation key}), the map ``get_observation`` and
        ``display_features`` use to route the extra, unrecorded views.
        """
        parity = "odd" if side == "left" else "even"
        configs: dict[str, Any] = {}
        self._tactile_display_keys: dict[str, dict[str, str]] = {}
        n_exp = self.config.expected_tactiles_per_side
        if n_exp:
            tactile_configs, self._tactile_display_keys = build_tactile_camera_configs(
                self._disc_tactiles.get(side, {}),
                side=side,
                key_prefix="",
                expected=n_exp,
                fps=self.config.tactile_fps,
                output_types=tactile_camera_output_types(
                    list(self.config.tactile_output_types),
                    list(self.config.tactile_display_output_types),
                ),
                diff_gain=self.config.tactile_diff_gain,
            )
            configs.update(tactile_configs)
        if self.config.enable_wrist_camera:
            sn = self._disc_cameras.get(side)
            if not sn:
                raise ValueError(
                    f"No {self._role} wrist camera found for the {side} side (rule: {side} == {parity} sequence)."
                )
            configs["wrist_cam"] = build_wrist_camera_config(
                sn,
                width=self.config.wrist_camera_width,
                height=self.config.wrist_camera_height,
                fps=self.config.wrist_camera_fps,
                fourcc=self.config.wrist_camera_fourcc,
            )

        if self.config.enable_head_camera:
            configs.update(build_head_camera_configs(self.config))
        return configs

    # ------------------------------------------------------------------ schema

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        features: dict[str, type | tuple] = {}

        if self._tracker_sn is not None:
            for k in POSE_KEYS:
                features[f"tcp.{k}"] = float

        if self.config.enable_gripper:
            features["gripper.pos"] = float

        if self.config.enable_imu:
            for axis in ("x", "y", "z"):
                features[f"imu.accel.{axis}"] = float
                features[f"imu.gyro.{axis}"] = float
                features[f"imu.mag.{axis}"] = float

        if self.config.enable_head_camera:
            for key in HEAD_POSE_KEYS:
                features[f"head_camera.{key}"] = float

        # ``frame_width`` differs from ``width`` only for the stereo head
        # camera, where ``width`` is one eye and a merged frame is twice that.
        for cam_name, cam_cfg in self._camera_configs.items():
            features[cam_name] = (cam_cfg.height, getattr(cam_cfg, "frame_width", cam_cfg.width), 3)

        return features

    @cached_property
    def display_features(self) -> dict[str, type | tuple]:
        """Rerun-facing schema: ``observation_features`` with each tactile camera's
        recorded stream swapped for its display-only view(s), in place.

        The dataset gets ``rectify`` (``observation_features``), the operator gets
        the amplified ``difference`` (this). Swapping rather than adding keeps the
        recorded stream out of the viewer entirely — same tile count, same Rerun
        image bandwidth as before the split — and keeps tactile in the same slot
        of the blueprint. Cameras with no display-only view are passed through, so
        without ``tactile_display_output_types`` this is just
        ``observation_features``.
        """
        features = swap_tactile_display_features(self.observation_features, self._tactile_display_keys)

        # Display-only: the tracker's own pose, so the viewer can draw it next to
        # the EE frame and show the mount transform. Absent from
        # observation_features, so it never reaches a dataset.
        if self._tracker_sn is not None:
            for k in POSE_KEYS:
                features[f"tracker.{k}"] = float
        return features

    @cached_property
    def action_features(self) -> dict[str, type]:
        """The 'demonstration' action this device emits when used as a teleop.

        Pose (tcp.x/y/z, tcp.r1-r6) + gripper.pos, and the headset pose
        (head_camera.x/y/z/r1..r6) when the head camera is on.

        The head pose is an action, not just context: what the operator looked at
        while demonstrating is something a policy is meant to reproduce, and it
        is in the same world frame and the same position + 6D rotation layout as
        ``tcp.*``, so it can be commanded the same way. Recording it only as an
        observation would leave it out of the shifted-frame pairing that makes
        the rest of the row a "where to move next" target.

        Still no image data — that lives in observation only.
        """
        features: dict[str, type] = {}
        if self._tracker_sn is not None:
            for k in POSE_KEYS:
                features[f"tcp.{k}"] = float
        if self.config.enable_gripper:
            features["gripper.pos"] = float
        if self.config.enable_head_camera:
            for k in HEAD_POSE_KEYS:
                features[f"head_camera.{k}"] = float
        return features

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def device_lost(self) -> bool:
        """True once any camera has been detected as physically lost mid-episode
        (hot-unplug / hub drop). The record loop polls this to stop cleanly and
        save the in-progress episode instead of crashing on the next read."""
        return self._cam_guard.lost

    @property
    def is_calibrated(self) -> bool:
        """The TacCap-Gripper uses factory calibration; we only need the
        gripper open/closed endpoints, which live in the config."""
        return self.is_connected

    # ------------------------------------------------------------------ lifecycle

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.logger.info(f"Connecting TacCap-Gripper ({self._side})...")
        self._cam_guard.reset()  # a reconnect must not inherit the last session's losses

        # 1. Gripper — auto-discovered by serial (side + role) on the bus. MCU
        #    transport only; cameras come from the LeRobot camera framework.
        if self.config.enable_gripper:
            grippers = disco.discover_grippers(self._role)
            self._endpoints = grippers.get(self._side)
            if self._endpoints is None:
                raise RuntimeError(f"No {self._role} gripper discovered for the {self._side} side.")
            gripper_cls = LeaderGripper if self._role == "leader" else FollowerGripper
            self.logger.info(
                f"  TacCap-Gripper: side={self._endpoints.side} role={self._endpoints.role} "
                f"fw_sn={self._endpoints.firmware_sn!r} mcu={self._endpoints.mcu_serial!r}"
            )
            self._gripper, self._gripper_norm_source = open_gripper(
                gripper_cls,
                self._endpoints.mcu_device,
                is_leader=self._role == "leader",
                open_rad=self.config.gripper_open_rad,
                logger=self.logger,
            )
            self.logger.info(f"  ✅ {gripper_cls.__name__} attached (MCU-only, read-only — motor stays disabled)")

        # 2. Pico4 tracker.
        if self._tracker_sn is not None:
            # None means "use this side's built-in mount transform"; anything
            # else is an explicit override (see ee_transform).
            ee_pos, ee_quat = resolve_tracker_to_ee(
                self._side, self.config.tracker_to_ee_pos, self.config.tracker_to_ee_quat
            )
            self.logger.info(f"tracker→TCP ({self._side}): pos={ee_pos.tolist()} quat={ee_quat.tolist()}")
            self._tracker = Pico4TrackerReader(
                tracker_sn=self._tracker_sn,
                tracker_to_ee_pos=ee_pos,
                tracker_to_ee_quat=ee_quat,
                device_wait_timeout=self.config.tracker_wait_timeout,
                logger_name=self.config.id or "robot",
            )
            # No init-pose alignment: poses stay in the world frame and the
            # deployment robot's base is cancelled downstream by the
            # relative-to-current representation, not baked in here.
            self._tracker.connect()
            self.logger.info("  ✅ Pico4 tracker connected (world frame)")

        # 3. Cameras (tactile + wrist, auto-discovered in __init__).
        #    Pre-warm the config cache sequentially first so the parallel connect
        #    below never triggers a Sunplus flash read (device reset) mid-open.
        prewarm_tactile_config_cache(self._camera_configs, self.logger)
        #    Then connect concurrently — each camera's V4L2 open + warmup overlaps
        #    in time rather than summing (cf. v0.4.4 bi_arx5). Configs now come
        #    from the cache (no flash read), so no device reset during connect.
        connect_cameras_parallel(self.cameras, self.logger)

        self._is_connected = True
        self.logger.info(f"✅ {self} connected.")

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")

        self.logger.info(f"Disconnecting {self}...")

        disconnect_cameras_parallel(self.cameras, self.logger)

        if self._tracker is not None:
            try:
                self._tracker.disconnect()
            except Exception as e:  # pragma: no cover
                self.logger.error(f"  Pico4 tracker disconnect error: {e}")
            self._tracker = None

        if self._gripper is not None:
            try:
                if getattr(self._gripper, "is_streaming", False):
                    self._gripper.stop_streaming()
            except Exception as e:  # pragma: no cover
                self.logger.warn(f"  stop_streaming raised: {e}")
            # Gripper has no explicit close; transport is released on GC.
            self._gripper = None

        self._endpoints = None
        self._is_connected = False
        self.logger.info(f"✅ {self} disconnected.")

    def calibrate(self) -> None:
        """Encoder zero is set out-of-band via the SDK's
        ``examples/calibrate.py`` (sends ``Encoder.set_zero()``). Once
        per device; afterwards ``position_rad`` is in [0, ~1.7]."""
        pass

    def configure(self) -> None:
        pass

    # ------------------------------------------------------------------ data

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")

        obs: dict[str, Any] = {}

        if self._tracker is not None:
            obs.update(self._tracker.get_action())
            # Display-only (see display_features): dropped by
            # select_display_observation's counterpart on the recording path.
            obs.update(self._tracker.get_tracker_display())

        if self.config.enable_gripper and self._gripper is not None:
            obs["gripper.pos"] = read_gripper_normalized(self._gripper, self.config.gripper_open_rad, self.logger)

        if self.config.enable_imu and self._gripper is not None:
            try:
                imu = self._gripper.imu.read_once()
                accel = imu.accel_mps2
                gyro = imu.gyro_radps
                mag = imu.mag_uT
                obs["imu.accel.x"] = float(accel[0])
                obs["imu.accel.y"] = float(accel[1])
                obs["imu.accel.z"] = float(accel[2])
                obs["imu.gyro.x"] = float(gyro[0])
                obs["imu.gyro.y"] = float(gyro[1])
                obs["imu.gyro.z"] = float(gyro[2])
                obs["imu.mag.x"] = float(mag[0])
                obs["imu.mag.y"] = float(mag[1])
                obs["imu.mag.z"] = float(mag[2])
            except Exception as e:
                self.logger.warn(f"IMU read failed: {e}")

        if self.config.enable_head_camera:
            head, self._head_pose_warned = read_head_pose(self.logger, self._head_pose_warned)
            obs.update(head)
            self._head_skew.check(self.cameras)

        # The head cameras need no special case here — they are ordinary
        # cameras, and the pose comes from the SDK above, not from a frame.
        for cam_name, cam in self.cameras.items():
            obs.update(
                split_camera_read(
                    cam_name,
                    self._cam_guard.read(cam_name, cam),
                    self._tactile_display_keys.get(cam_name),
                )
            )

        return obs

    def send_action(self, action: dict[str, Any] | None = None) -> dict[str, Any]:
        """No-op: this is a passive demonstration device. We never command
        the jaw motor — the operator drives the gripper mechanically."""
        return action or {}

    def get_action(self) -> dict[str, Any]:
        """Return the same pose + gripper dict that the device emits as a
        teleoperator, i.e. exactly the keys in :attr:`action_features`.

        The record loop does not call this — it takes the ``action_features``
        subset of the observation it already sampled, so the row costs one
        hardware read. This exists for the teleoperator role, and reads the
        headset pose again rather than sharing that sample.
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")
        action: dict[str, Any] = {}
        if self._tracker is not None:
            action.update(self._tracker.get_action())
        if self.config.enable_gripper and self._gripper is not None:
            action["gripper.pos"] = read_gripper_normalized(self._gripper, self.config.gripper_open_rad, self.logger)
        if self.config.enable_head_camera:
            head, self._head_pose_warned = read_head_pose(self.logger, self._head_pose_warned)
            action.update(head)
        return action

    # ------------------------------------------------------------------ helpers

    def get_endpoints(self):
        """Hardware discovery info populated on connect (None otherwise)."""
        return self._endpoints
