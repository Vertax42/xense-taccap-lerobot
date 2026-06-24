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
    tactile_0 / tactile_1            -- tactile frames
    wrist_cam                        -- wrist UVC frame (if enable_wrist_camera)
"""

from __future__ import annotations

import glob
import os
import time
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
from . import serial_discovery as disco
from .config_taccap_gripper import TaccapGripperConfig

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


def resolve_wrist_camera_path(serial: str) -> str:
    """Resolve a wrist UVC camera serial (e.g. ``"XCA24Z0003m"``) to its stable
    ``/dev/v4l/by-id`` capture path. The serial is encoded in the by-id name; we
    match the ``index0`` (capture) node. Unlike Xense tactile sensors, the wrist
    UVC camera is not enumerable via ``xensesdk.Sensor.scanSerialNumber`` — its
    USB iSerial is non-unique (e.g. ``01.00.00``), so by-id (which encodes the
    model serial) is the reliable handle."""
    matches = sorted(glob.glob(f"/dev/v4l/by-id/*{serial}*-video-index0"))
    if not matches:
        raise RuntimeError(
            f"No wrist camera matching serial {serial!r} under /dev/v4l/by-id/ "
            "(plugged in? check `ls /dev/v4l/by-id/`)."
        )
    if len(matches) > 1:
        raise RuntimeError(
            f"Multiple wrist cameras match serial {serial!r}: {matches}. Use a more "
            "specific serial."
        )
    return matches[0]


# Config-cache key xensesdk uses for its per-serial config cache. Mirrors the
# constant baked into the SDK (xensesdk.core.ctx_builders); if it ever drifts the
# pre-warm just fails to decrypt and the SDK falls back to its own flash read.
_XENSE_CONFIG_CACHE_PSWD = "Wz8mmWz2ALJ6X5Ic"


def _wait_nodes_settle(serials, logger, timeout_s: float = 15.0) -> None:
    """Wait until each serial's ``/dev/v4l/by-id`` capture node is back + openable
    after a flash-read reset re-enumerated it."""
    deadline = time.perf_counter() + timeout_s
    for sn in serials:
        settled = False
        while time.perf_counter() < deadline:
            matches = glob.glob(f"/dev/v4l/by-id/*{sn}*-video-index0")
            if matches:
                try:
                    fd = os.open(os.path.realpath(matches[0]), os.O_RDWR)
                    os.close(fd)
                    settled = True
                    break
                except OSError:
                    pass
            time.sleep(0.2)
        if not settled:
            logger.warning(
                f"  Sensor {sn} V4L2 node did not settle within {timeout_s:.0f}s after pre-warm"
            )


def prewarm_tactile_config_cache(camera_configs: dict[str, Any], logger) -> None:
    """Warm the xensesdk per-serial config cache for tactile sensors **before**
    opening any camera.

    A Sunplus (0x1300) flash read resets/re-enumerates the sensor. Doing that
    concurrently (the parallel camera connect) on a cold cache races the SDK's
    non-thread-safe flash lib and moves camera nodes mid-open. Reading here,
    sequentially and only for **uncached Sunplus** sensors, makes cold start
    safe; then we wait for the nodes to settle. A warm cache is just a cheap
    ``exists()`` stat — no flash read, no reset, no extra cache decrypt (the SDK
    still reads the cache once at connect)."""
    serials = [
        cfg.serial_number
        for cfg in camera_configs.values()
        if isinstance(cfg, XenseTactileCameraConfig) and getattr(cfg, "serial_number", None)
    ]
    if not serials:
        return
    try:
        from xensesdk.flash import FlashClient
        from xensesdk.flash.sunplus_backend import is_sunplus
        from xensesdk.core.ctx_builders import CONFIG_CACHE_DIR
        from xensesdk.utils.encrypt import encrypt_config_file
    except Exception as e:  # SDK without the Sunplus/xbin path — nothing to do.
        logger.debug(f"Config pre-warm unavailable ({e}); skipping")
        return

    uncached = [
        sn for sn in serials if is_sunplus(sn) and not (CONFIG_CACHE_DIR / sn).exists()
    ]
    if not uncached:
        return  # warm cache: cheap stat only, no flash read / reset

    logger.info(
        f"  Pre-warming config cache (cold start) for {len(uncached)} Sunplus sensor(s): {uncached}"
    )
    client = FlashClient()
    try:
        for sn in uncached:
            try:
                patch = client.read_patch(serial_number=sn)  # reads flash -> resets device
                CONFIG_CACHE_DIR.mkdir(parents=True, exist_ok=True)
                encrypt_config_file(
                    patch, CONFIG_CACHE_DIR / sn, password=_XENSE_CONFIG_CACHE_PSWD, format="xbin"
                )
            except Exception as e:
                logger.warning(f"  Config pre-warm failed for {sn}: {e}")
    finally:
        client.cleanup()

    _wait_nodes_settle(uncached, logger)


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

        if config.enable_gripper and not TACCAP_SDK_AVAILABLE:
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

        # Hardware handles, populated on connect.
        self._gripper: Any = None  # Leader/FollowerGripper
        self._endpoints: Any = None  # xense.taccap.GripperEndpoints
        self._tracker: Pico4TrackerReader | None = None

        # Filesystem-only discovery (cheap, no hardware open) → resolve which side
        # this single unit is, then build its tactile + wrist camera configs.
        self._disc_tactiles = (
            disco.discover_tactiles()
            if config.expected_tactiles_per_side
            else {"left": [], "right": []}
        )
        self._disc_cameras = (
            disco.discover_wrist_cameras(self._role)
            if config.enable_wrist_camera
            else {}
        )
        self._side = self._resolve_side()
        self._camera_configs = self._build_camera_configs(self._side)
        self.cameras = make_cameras_from_configs(self._camera_configs)

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
            present = {s for s in disco.SIDES if len(self._disc_tactiles.get(s, [])) == n}
        else:
            present = set()
        if len(present) == 1:
            return next(iter(present))
        if not present:
            raise RuntimeError(
                f"No {self._role} TacCap device discovered to infer a side; "
                "connect one or set --robot.side=left|right."
            )
        raise RuntimeError(
            f"Both sides present {sorted(present)}; set --robot.side=left|right to pick one."
        )

    def _build_camera_configs(self, side: str) -> dict[str, Any]:
        """Build ``tactile_i`` + ``wrist_cam`` configs for ``side`` from discovery."""
        parity = "odd" if side == "left" else "even"
        configs: dict[str, Any] = {}
        n_exp = self.config.expected_tactiles_per_side
        if n_exp:
            got = self._disc_tactiles.get(side, [])
            if len(got) != n_exp:
                raise ValueError(
                    f"Expected {n_exp} {side} tactile sensors ({parity} sequence), "
                    f"found {len(got)}: {got}."
                )
            for i, sn in enumerate(got):
                configs[f"tactile_{i}"] = XenseTactileCameraConfig(
                    serial_number=sn,
                    fps=self.config.tactile_fps,
                    output_types=list(self.config.tactile_output_types),
                )
        if self.config.enable_wrist_camera:
            sn = self._disc_cameras.get(side)
            if not sn:
                raise ValueError(
                    f"No {self._role} wrist camera found for the {side} side "
                    f"(rule: {side} == {parity} sequence)."
                )
            configs["wrist_cam"] = OpenCVCameraConfig(
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

        if self._tracker_sn is not None:
            for k in ("x", "y", "z", "r1", "r2", "r3", "r4", "r5", "r6"):
                features[f"tcp.{k}"] = float

        if self.config.enable_gripper:
            features["gripper.pos"] = float

        if self.config.enable_imu:
            for axis in ("x", "y", "z"):
                features[f"imu.accel.{axis}"] = float
                features[f"imu.gyro.{axis}"] = float
                features[f"imu.mag.{axis}"] = float

        for cam_name, cam_cfg in self._camera_configs.items():
            features[cam_name] = (cam_cfg.height, cam_cfg.width, 3)

        return features

    @cached_property
    def action_features(self) -> dict[str, type]:
        """The 'demonstration' action this device emits when used as a teleop.

        Pose (tcp.x/y/z, tcp.r1-r6) + gripper.pos.
        No camera data — that lives in observation only.
        """
        features: dict[str, type] = {}
        if self._tracker_sn is not None:
            for k in ("x", "y", "z", "r1", "r2", "r3", "r4", "r5", "r6"):
                features[f"tcp.{k}"] = float
        if self.config.enable_gripper:
            features["gripper.pos"] = float
        return features

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        """The TacCap-Gripper uses factory calibration; we only need the
        gripper open/closed endpoints, which live in the config."""
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

        self.logger.info(f"Connecting TacCap-Gripper ({self._side})...")

        # 1. Gripper — auto-discovered by serial (side + role) on the bus. MCU
        #    transport only; cameras come from the LeRobot camera framework.
        if self.config.enable_gripper:
            grippers = disco.discover_grippers(self._role)
            self._endpoints = grippers.get(self._side)
            if self._endpoints is None:
                raise RuntimeError(
                    f"No {self._role} gripper discovered for the {self._side} side."
                )
            gripper_cls = LeaderGripper if self._role == "leader" else FollowerGripper
            self.logger.info(
                f"  TacCap-Gripper: side={self._endpoints.side} role={self._endpoints.role} "
                f"fw_sn={self._endpoints.firmware_sn!r} mcu={self._endpoints.mcu_serial!r}"
            )
            self._gripper = gripper_cls(self._endpoints.mcu_device)
            self.logger.info(
                f"  ✅ {gripper_cls.__name__} attached (MCU-only, read-only — motor stays disabled)"
            )

        # 2. Pico4 tracker.
        if self._tracker_sn is not None:
            self._tracker = Pico4TrackerReader(
                tracker_sn=self._tracker_sn,
                tracker_to_ee_pos=self.config.tracker_to_ee_pos,
                tracker_to_ee_quat=self.config.tracker_to_ee_quat,
                device_wait_timeout=self.config.tracker_wait_timeout,
                logger_name=self.config.id or "robot",
            )
            init_pose = (
                np.asarray(self.config.init_tcp_pose, dtype=np.float64)
                if self.config.enable_init_pose_alignment
                else None
            )
            self._tracker.connect(current_tcp_pose_quat=init_pose)
            if init_pose is not None:
                self.logger.info(
                    f"  ✅ Pico4 tracker connected with UMI alignment "
                    f"(init_tcp_pose={init_pose.tolist()})"
                )
            else:
                self.logger.info("  ✅ Pico4 tracker connected (world frame)")

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

        if self.config.enable_gripper and self._gripper is not None:
            obs["gripper.pos"] = self._read_gripper_normalized()

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

        for cam_name, cam in self.cameras.items():
            obs[cam_name] = cam.async_read()

        return obs

    def send_action(self, action: dict[str, Any] | None = None) -> dict[str, Any]:
        """No-op: this is a passive demonstration device. We never command
        the jaw motor — the operator drives the gripper mechanically."""
        return action or {}

    def get_action(self) -> dict[str, Any]:
        """Return the same pose + gripper dict that the device emits as a
        teleoperator (used when recording demonstrations)."""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")
        action: dict[str, Any] = {}
        if self._tracker is not None:
            action.update(self._tracker.get_action())
        if self.config.enable_gripper and self._gripper is not None:
            action["gripper.pos"] = self._read_gripper_normalized()
        return action

    # ------------------------------------------------------------------ helpers

    def _read_gripper_normalized(self) -> float:
        """Read cooked encoder position (rad ≥ 0 post-``set_zero``) and
        normalise to [0, 1] via ``clip(rad / open_rad, 0, 1)``."""
        try:
            sample = self._gripper.encoder.read_once()
            rad = float(sample.position_rad)
        except Exception as e:
            self.logger.warn(f"Encoder read failed: {e}")
            return 0.0
        opened = self.config.gripper_open_rad
        if opened <= 0.0:  # guarded in config.__post_init__ but be defensive
            return 0.0
        return float(np.clip(rad / opened, 0.0, 1.0))

    def get_endpoints(self):
        """Hardware discovery info populated on connect (None otherwise)."""
        return self._endpoints
