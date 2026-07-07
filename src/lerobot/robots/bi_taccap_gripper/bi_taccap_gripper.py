#!/usr/bin/env python

# Copyright 2026 The XenseRobotics Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""
Bimanual TacCap-Gripper handheld data-collection device for LeRobot.

Two independent TacCap-Gripper units driven as one robot. Like the single
``taccap_gripper`` this is a passive recording device — ``send_action()`` is a
no-op (the jaw motors stay disabled; we read encoders only). Pose comes from a
per-side Pico4 Ultra tracker (``Pico4TrackerReader``); tactile + wrist cameras go
through the standard ``cameras`` framework.

Devices are **auto-discovered by serial rule** (``serial_discovery.py``): the two
grippers, four tactile sensors and two wrist cameras are scanned from the
connected hardware and assigned to ``left``/``right`` by serial (odd → left, even
→ right) and role (Master/Leader vs Slave/Follower). No serials are listed in the
config; a non-conforming or missing/duplicated device raises a clear error.

Observation features (per side ``{s}`` in left/right):
    {s}_tcp.x/y/z, {s}_tcp.r1..r6   -- Pico4 tracker → EE 6D pose (if enable_tracker)
    {s}_gripper.pos                 -- normalised jaw [0=closed, 1=open]
    {s}_imu.accel/gyro/mag.{x,y,z}  -- optional
    {s}_wrist                       -- wrist UVC frame (if enable_wrist_camera)
    {s}_tactile_left / {s}_tactile_right -- tactile frames (sensor on left/right finger)
"""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor, as_completed
from functools import cached_property
from typing import Any

import numpy as np

from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.cameras.xense.configuration_xense import XenseTactileCameraConfig
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.utils.robot_utils import get_logger

from ..robot import Robot
from ..taccap_gripper import serial_discovery as disco
from ..taccap_gripper.taccap_gripper import (
    prewarm_tactile_config_cache,
    resolve_wrist_camera_path,
)
from .config_bi_taccap_gripper import BiTaccapGripperConfig

_SIDES = ("left", "right")

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


class BiTaccapGripper(Robot):
    """Bimanual TacCap-Gripper handheld data-collection device.

    Operated manually — no action is applied to either jaw. Emits ``left_``/
    ``right_`` prefixed pose + jaw + tactile + wrist for downstream learning.

    6D rotation convention (matches ``vive_tracker`` / single ``taccap_gripper``):
        r1..r3 = first column of the rotation matrix
        r4..r6 = second column of the rotation matrix
    """

    config_class = BiTaccapGripperConfig
    name = "bi_taccap_gripper"

    def __init__(self, config: BiTaccapGripperConfig):
        super().__init__(config)
        self.config = config
        self.logger = get_logger(f"BiTaccapGripper-{config.id or 'default'}")
        self._role = disco.normalize_role(config.role)

        any_gripper = any(getattr(config, f"{s}_enable_gripper") for s in _SIDES)
        # Tactile discovery now pairs sensors to a gripper by USB hub, so it also
        # needs the SDK (scan_grippers) to resolve each hub's side.
        needs_sdk = any_gripper or config.expected_tactiles_per_side > 0
        if needs_sdk and not TACCAP_SDK_AVAILABLE:
            raise ImportError(
                "xense.taccap SDK not available. Build it from the vendored "
                "submodule third_party/taccap-gripper (run setup_env.sh --install). "
                f"Original import error: {_TACCAP_SDK_IMPORT_ERROR!r}"
            ) from _TACCAP_SDK_IMPORT_ERROR
        if config.enable_tracker and not PICO4_TRACKER_AVAILABLE:
            raise ImportError(
                "Pico4TrackerReader not available. Ensure "
                "src/lerobot/teleoperators/pico4/tracker.py is importable."
            )

        # Per-side hardware handles, populated on connect.
        self._gripper: dict[str, Any] = {s: None for s in _SIDES}  # Leader/FollowerGripper
        self._endpoints: dict[str, Any] = {s: None for s in _SIDES}  # GripperEndpoints
        self._tracker: dict[str, Pico4TrackerReader | None] = {s: None for s in _SIDES}

        # Auto-discover tactile + wrist cameras and build their configs so the
        # observation schema is ready before connect(). Tactiles are paired to a
        # gripper by USB hub, so this scans the serial bus (grippers must be
        # powered at construction); wrist cameras are filesystem-only.
        self._camera_configs = self._discover_camera_configs()
        self.cameras = make_cameras_from_configs(self._camera_configs)

        # Auto-discover the Pico4 motion tracker(s): enumerate from the XenseVR PC
        # service and assign one per side by serial (second-to-last digit, strict).
        # Drives the pose schema, so it runs here (pre-connect) like the cameras.
        self._tracker_sn_by_side: dict[str, str] = {}
        if config.enable_tracker:
            self._tracker_sn_by_side = disco.resolve_pico_trackers(
                _SIDES,
                {s: getattr(config, f"{s}_tracker_serial") for s in _SIDES},
                lambda: Pico4TrackerReader.list_serial_numbers(
                    device_wait_timeout=config.tracker_wait_timeout,
                    logger_name=config.id or "bi",
                ),
            )
            self.logger.info(f"Pico4 trackers: {self._tracker_sn_by_side}")

        self._is_connected = False

        # Graceful-degradation state for mid-episode camera loss: keep the last
        # good frame per camera so a hot-unplug degrades to a stale/black frame
        # instead of crashing the record loop, and remember which cameras died
        # so the caller can stop cleanly and save what was recorded.
        self._last_cam_frame: dict[str, np.ndarray] = {}
        self._lost_cameras: set[str] = set()

    # ------------------------------------------------------------------ discovery

    def _discover_camera_configs(self) -> dict[str, Any]:
        """Build the tactile + wrist camera configs from serial auto-discovery.

        Tactiles (``{side}_tactile_{left,right}``) are paired to a gripper by USB
        hub (hub → gripper firmware SN → side) and keyed by finger (GSPS last
        digit); wrist cameras (``{side}_wrist``) come from ``/dev/v4l/by-id``.
        Counts are validated per side so a mis-installed sensor is caught here,
        not mid-episode.
        """
        n_exp = self.config.expected_tactiles_per_side
        tactiles = disco.discover_tactiles_by_hub(self._role) if n_exp else {"left": {}, "right": {}}
        want_wrist = any(getattr(self.config, f"{s}_enable_wrist_camera") for s in _SIDES)
        cameras = disco.discover_wrist_cameras(self._role) if want_wrist else {}

        configs: dict[str, Any] = {}
        for side in _SIDES:
            parity = "odd" if side == "left" else "even"
            if n_exp:
                got = tactiles.get(side, {})
                if len(got) != n_exp:
                    raise ValueError(
                        f"Expected {n_exp} {side} tactile sensors (on the {side} "
                        f"gripper's USB hub), found {len(got)}: {sorted(got.values())}."
                    )
                for finger, sn in sorted(got.items()):
                    configs[f"{side}_tactile_{finger}"] = XenseTactileCameraConfig(
                        serial_number=sn,
                        fps=self.config.tactile_fps,
                        output_types=list(self.config.tactile_output_types),
                    )
            if getattr(self.config, f"{side}_enable_wrist_camera"):
                sn = cameras.get(side)
                if not sn:
                    raise ValueError(
                        f"No {self._role} wrist camera found for the {side} side "
                        f"(rule: {side} == {parity} sequence)."
                    )
                configs[f"{side}_wrist"] = OpenCVCameraConfig(
                    index_or_path=resolve_wrist_camera_path(sn),
                    width=self.config.wrist_camera_width,
                    height=self.config.wrist_camera_height,
                    fps=self.config.wrist_camera_fps,
                )
        return configs

    # ------------------------------------------------------------------ schema

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        features: dict[str, type | tuple] = {}

        for side in _SIDES:
            if side in self._tracker_sn_by_side:
                for k in ("x", "y", "z", "r1", "r2", "r3", "r4", "r5", "r6"):
                    features[f"{side}_tcp.{k}"] = float
            if getattr(self.config, f"{side}_enable_gripper"):
                features[f"{side}_gripper.pos"] = float
            if getattr(self.config, f"{side}_enable_imu"):
                for axis in ("x", "y", "z"):
                    features[f"{side}_imu.accel.{axis}"] = float
                    features[f"{side}_imu.gyro.{axis}"] = float
                    features[f"{side}_imu.mag.{axis}"] = float

        # Tactile + wrist cameras (keys already left_/right_ prefixed).
        for cam_name, cam_cfg in self._camera_configs.items():
            features[cam_name] = (cam_cfg.height, cam_cfg.width, 3)

        return features

    @cached_property
    def action_features(self) -> dict[str, type]:
        """The 'demonstration' action the rig emits (pose + jaw per side, no cameras)."""
        features: dict[str, type] = {}
        for side in _SIDES:
            if side in self._tracker_sn_by_side:
                for k in ("x", "y", "z", "r1", "r2", "r3", "r4", "r5", "r6"):
                    features[f"{side}_tcp.{k}"] = float
            if getattr(self.config, f"{side}_enable_gripper"):
                features[f"{side}_gripper.pos"] = float
        return features

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def device_lost(self) -> bool:
        """True once any camera has been detected as physically lost mid-episode
        (hot-unplug / hub drop). The record loop polls this to stop cleanly and
        save the in-progress episode instead of crashing on the next read."""
        return bool(self._lost_cameras)

    @property
    def is_calibrated(self) -> bool:
        """Factory calibration; the only per-unit step is the gripper encoder
        zero, which lives in firmware (set once via the SDK's calibrate.py)."""
        return self.is_connected

    # ------------------------------------------------------------------ lifecycle

    def _connect_cameras_parallel(self) -> None:
        """Open all cameras concurrently — each camera's V4L2 open + warmup
        overlaps in time instead of summing (cf. v0.4.4 bi_arx5)."""
        if not self.cameras:
            return
        n = len(self.cameras)
        self.logger.info(f"  Connecting {n} camera(s) in parallel...")
        with ThreadPoolExecutor(max_workers=min(n, 8)) as executor:
            futures = {executor.submit(cam.connect): name for name, cam in self.cameras.items()}
            for fut in as_completed(futures):
                name = futures[fut]
                try:
                    fut.result()
                except Exception as e:
                    self.logger.error(f"  Camera '{name}' connect failed: {e}")
                    raise
        self.logger.info(f"  ✅ {n} camera(s) connected")

    def _disconnect_cameras_parallel(self) -> None:
        if not self.cameras:
            return

        def _close(cam):
            if cam.is_connected:
                cam.disconnect()

        n = len(self.cameras)
        with ThreadPoolExecutor(max_workers=min(n, 8)) as executor:
            futures = {executor.submit(_close, cam): name for name, cam in self.cameras.items()}
            for fut in as_completed(futures):
                name = futures[fut]
                try:
                    fut.result()
                except Exception as e:  # pragma: no cover — best-effort teardown
                    self.logger.error(f"  Camera '{name}' disconnect error: {e}")

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.logger.info("Connecting BiTacCap-Gripper...")

        # 1. Grippers — auto-discovered by serial (side + role) on the bus.
        enabled_gripper_sides = tuple(
            s for s in _SIDES if getattr(self.config, f"{s}_enable_gripper")
        )
        grippers = disco.discover_grippers(self._role) if enabled_gripper_sides else {}
        gripper_cls = LeaderGripper if self._role == "leader" else FollowerGripper

        for side in _SIDES:
            # Gripper (MCU transport only; cameras come from the LeRobot camera
            # framework, so open_cameras stays False).
            if getattr(self.config, f"{side}_enable_gripper"):
                endpoints = grippers.get(side)
                if endpoints is None:
                    raise RuntimeError(
                        f"No {self._role} gripper discovered for the {side} side."
                    )
                self._endpoints[side] = endpoints
                self.logger.info(
                    f"  [{side}] TacCap-Gripper: side={endpoints.side} role={endpoints.role} "
                    f"fw_sn={endpoints.firmware_sn!r} mcu={endpoints.mcu_serial!r}"
                )
                self._gripper[side] = gripper_cls(endpoints.mcu_device)
                self.logger.info(
                    f"  [{side}] ✅ {gripper_cls.__name__} attached (MCU-only, read-only)"
                )

            # 2. Pico4 tracker (auto-discovered SN per side, pinned here).
            if side in self._tracker_sn_by_side:
                tracker = Pico4TrackerReader(
                    tracker_sn=self._tracker_sn_by_side[side],
                    tracker_to_ee_pos=getattr(self.config, f"{side}_tracker_to_ee_pos"),
                    tracker_to_ee_quat=getattr(self.config, f"{side}_tracker_to_ee_quat"),
                    device_wait_timeout=self.config.tracker_wait_timeout,
                    logger_name=f"{self.config.id or 'bi'}-{side}",
                )
                init_pose = (
                    np.asarray(
                        getattr(self.config, f"{side}_init_tcp_pose"), dtype=np.float64
                    )
                    if getattr(self.config, f"{side}_enable_init_pose_alignment")
                    else None
                )
                tracker.connect(current_tcp_pose_quat=init_pose)
                self._tracker[side] = tracker
                if init_pose is not None:
                    self.logger.info(
                        f"  [{side}] ✅ Pico4 tracker connected with UMI alignment"
                    )
                else:
                    self.logger.info(
                        f"  [{side}] ✅ Pico4 tracker connected (world frame)"
                    )

        # 3. Cameras (tactile + wrist, auto-discovered in __init__).
        #    Pre-warm the config cache sequentially first so the parallel connect
        #    below never triggers a Sunplus flash read (device reset) mid-open.
        prewarm_tactile_config_cache(self._camera_configs, self.logger)
        #    Then connect concurrently — each camera's V4L2 open + warmup overlaps
        #    in time rather than summing (cf. v0.4.4 bi_arx5). Configs now come
        #    from the cache (no flash read), so no device reset during connect.
        self._connect_cameras_parallel()

        self._is_connected = True
        self.logger.info(f"✅ {self} connected.")

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")

        self.logger.info(f"Disconnecting {self}...")

        self._disconnect_cameras_parallel()

        for side in _SIDES:
            tracker = self._tracker[side]
            if tracker is not None:
                try:
                    tracker.disconnect()
                except Exception as e:  # pragma: no cover
                    self.logger.error(f"  [{side}] Pico4 tracker disconnect error: {e}")
                self._tracker[side] = None

            gripper = self._gripper[side]
            if gripper is not None:
                try:
                    if getattr(gripper, "is_streaming", False):
                        gripper.stop_streaming()
                except Exception as e:  # pragma: no cover
                    self.logger.warn(f"  [{side}] stop_streaming raised: {e}")
                # Gripper has no explicit close; transport released on GC.
                self._gripper[side] = None
            self._endpoints[side] = None

        self._is_connected = False
        self.logger.info(f"✅ {self} disconnected.")

    def calibrate(self) -> None:
        """Encoder zero is set out-of-band per unit via the SDK's calibrate.py."""
        pass

    def configure(self) -> None:
        pass

    # ------------------------------------------------------------------ data

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")

        obs: dict[str, Any] = {}

        for side in _SIDES:
            if side in self._tracker_sn_by_side and self._tracker[side] is not None:
                for k, v in self._tracker[side].get_action().items():
                    obs[f"{side}_{k}"] = v

            if getattr(self.config, f"{side}_enable_gripper") and self._gripper[side] is not None:
                obs[f"{side}_gripper.pos"] = self._read_gripper_normalized(side)

            if getattr(self.config, f"{side}_enable_imu") and self._gripper[side] is not None:
                try:
                    imu = self._gripper[side].imu.read_once()
                    accel, gyro, mag = imu.accel_mps2, imu.gyro_radps, imu.mag_uT
                    for i, axis in enumerate(("x", "y", "z")):
                        obs[f"{side}_imu.accel.{axis}"] = float(accel[i])
                        obs[f"{side}_imu.gyro.{axis}"] = float(gyro[i])
                        obs[f"{side}_imu.mag.{axis}"] = float(mag[i])
                except Exception as e:
                    self.logger.warn(f"  [{side}] IMU read failed: {e}")

        for cam_name, cam in self.cameras.items():
            obs[cam_name] = self._read_camera_or_fallback(cam_name, cam)

        return obs

    def _read_camera_or_fallback(self, cam_name: str, cam: Any) -> np.ndarray:
        """Read one camera, degrading gracefully on physical loss.

        A hot-unplugged camera (USB drop / hub power loss) makes its background
        read thread die; the next ``async_read`` raises ``RuntimeError`` ("read
        thread is not running"). Letting that propagate crashes the record loop
        and loses the in-progress episode. Instead we substitute the last good
        frame (or a black frame on first-read loss), flag the camera as lost so
        ``device_lost`` trips, and let the caller stop cleanly and save.

        ``TimeoutError`` (a transient slow/dropped frame) is treated the same way
        but is NOT flagged as lost — those recover on their own."""
        try:
            frame = cam.async_read()
            self._last_cam_frame[cam_name] = frame
            return frame
        except TimeoutError as e:
            self.logger.warning(f"  [{cam_name}] frame timeout, reusing last frame: {e}")
            return self._fallback_frame(cam_name)
        except Exception as e:
            if cam_name not in self._lost_cameras:
                self.logger.error(f"  [{cam_name}] camera lost mid-episode: {e}")
                self._lost_cameras.add(cam_name)
            return self._fallback_frame(cam_name)

    def _fallback_frame(self, cam_name: str) -> np.ndarray:
        """Last good frame for this camera, or a black frame of the declared
        (H, W, 3) shape if none was ever captured."""
        cached = self._last_cam_frame.get(cam_name)
        if cached is not None:
            return cached
        cfg = self._camera_configs[cam_name]
        frame = np.zeros((cfg.height, cfg.width, 3), dtype=np.uint8)
        self._last_cam_frame[cam_name] = frame
        return frame

    def send_action(self, action: dict[str, Any] | None = None) -> dict[str, Any]:
        """No-op: passive demonstration device. The operators drive the jaws
        mechanically — we never command either motor."""
        return action or {}

    def get_action(self) -> dict[str, Any]:
        """Return the prefixed pose + gripper dict the rig emits as a teleoperator
        (used when recording demonstrations)."""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")
        action: dict[str, Any] = {}
        for side in _SIDES:
            if side in self._tracker_sn_by_side and self._tracker[side] is not None:
                for k, v in self._tracker[side].get_action().items():
                    action[f"{side}_{k}"] = v
            if getattr(self.config, f"{side}_enable_gripper") and self._gripper[side] is not None:
                action[f"{side}_gripper.pos"] = self._read_gripper_normalized(side)
        return action

    # ------------------------------------------------------------------ helpers

    def _read_gripper_normalized(self, side: str) -> float:
        """Read cooked encoder position (rad ≥ 0 post-``set_zero``) and normalise
        to [0, 1] via ``clip(rad / {side}_gripper_open_rad, 0, 1)``."""
        try:
            sample = self._gripper[side].encoder.read_once()
            rad = float(sample.position_rad)
        except Exception as e:
            self.logger.warn(f"  [{side}] Encoder read failed: {e}")
            return 0.0
        opened = getattr(self.config, f"{side}_gripper_open_rad")
        if opened <= 0.0:  # guarded in config.__post_init__ but be defensive
            return 0.0
        return float(np.clip(rad / opened, 0.0, 1.0))

    def get_endpoints(self):
        """Per-side hardware discovery info populated on connect."""
        return dict(self._endpoints)
