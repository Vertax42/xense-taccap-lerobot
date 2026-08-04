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
    left_headcam / right_headcam     -- Pico headset camera, one key per eye
                                        (if enable_head_camera; head_camera_eyes
                                        can select a single eye)
    head_camera.x/y/z/r1..r6         -- headset pose, same world frame as tcp.*

Display-only keys (present in ``get_observation()`` and in ``display_features``,
absent from ``observation_features``, so Rerun shows them but the dataset never
sees them — see ``tactile_display_output_types``):
    tactile_{left,right}_difference  -- amplified deformation view of the same
                                        sensor read
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
        raise RuntimeError(f"Multiple wrist cameras match serial {serial!r}: {matches}. Use a more specific serial.")
    return matches[0]


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
            logger.warn(f"  Sensor {sn} V4L2 node did not settle within {timeout_s:.0f}s after pre-warm")


def prewarm_tactile_config_cache(camera_configs: dict[str, Any], logger) -> None:
    """Warm the xensesdk per-serial config cache for tactile sensors **before**
    opening any camera.

    The first open of a sensor reads its flash, which resets/re-enumerates the
    device. Doing that concurrently (the parallel camera connect) on a cold
    cache races the SDK's non-thread-safe flash lib and moves camera nodes
    mid-open. Opening each uncached sensor here — sequentially, one at a time —
    forces those flash reads to happen in a controlled order; then we wait for
    the nodes to settle. A warm cache is just a cheap ``exists()`` stat: no
    flash read, no reset (the SDK still reads the cache once at connect).

    We deliberately drive this through plain ``Sensor.create``/``release``
    rather than reaching into the flash backend and writing the cache
    ourselves. The SDK owns the cache format and its encryption key, and an
    earlier version of this function reimplemented that write — which meant
    hardcoding the SDK's key in this file and pinning two private APIs
    (``FlashClient``, ``is_sunplus``). Both were renamed upstream, so the
    import failed, the whole pre-warm silently no-op'd through its except
    branch, and the cold-start race it exists to prevent was live again. The
    public path costs ~2s per uncached sensor on cold start and nothing when
    warm, and cannot rot the same way."""
    serials = [
        cfg.serial_number
        for cfg in camera_configs.values()
        if isinstance(cfg, XenseTactileCameraConfig) and getattr(cfg, "serial_number", None)
    ]
    if not serials:
        return
    try:
        from xensesdk import Sensor
        from xensesdk.core.ctx_builders import CONFIG_CACHE_DIR
    except Exception as e:  # No SDK — the tactile cameras will fail later anyway.
        logger.debug(f"Config pre-warm unavailable ({e}); skipping")
        return

    uncached = [sn for sn in serials if not (CONFIG_CACHE_DIR / sn).exists()]
    if not uncached:
        return  # warm cache: cheap stat only, no flash read / reset

    logger.info(f"  Pre-warming config cache (cold start) for {len(uncached)} sensor(s): {uncached}")
    for sn in uncached:
        try:
            # disable_infer keeps this to the flash read + cache write; the real
            # connect re-creates the sensor with the caller's actual settings.
            sensor = Sensor.create(sn, disable_infer=True)
            sensor.release()
        except Exception as e:
            logger.warn(f"  Config pre-warm failed for {sn}: {e}")

    _wait_nodes_settle(uncached, logger)


# ---- Recorded vs display-only tactile streams -------------------------------
# One sensor read carries two views: the recorded one (rectify — everything the
# sensor saw) and the display one (the amplified difference the operator reads
# contact from). The SDK hands both back from a single ``selectSensorInfo``, so
# the split is purely a matter of which observation key each lands on: the
# recorded view keeps the camera's own key and is the only one in
# ``observation_features``; each display view gets a ``{camera}_{type}`` sibling
# that only Rerun ever sees. Shared with ``bi_taccap_gripper``.


def tactile_display_key(cam_name: str, output_type: str) -> str:
    """Observation key carrying ``output_type`` as a display-only view of ``cam_name``."""
    return f"{cam_name}_{output_type}"


def tactile_camera_output_types(record_types: list[str], display_types: list[str]) -> list[str]:
    """Output types to ask one tactile sensor for: recorded first, then the
    display-only ones (deduplicated, order preserved).

    A display type that is also the recorded type collapses to a single request —
    the recorded key then simply doubles as the displayed one. This only catches
    identical spellings; the authoritative de-duplication happens in
    ``XenseTactileCameraConfig.__post_init__``, which resolves "DIFFERENCE" and
    "difference" to the same enum. Read the types back off the config (not from
    here) when deciding which of them are display-only.
    """
    types = list(record_types)
    types += [t for t in display_types if t not in types]
    return types


HEAD_POSE_KEYS = ("x", "y", "z", "r1", "r2", "r3", "r4", "r5", "r6")


HEAD_CAMERA_KEYS = {"left": "left_headcam", "right": "right_headcam"}


def build_head_camera_configs(config: Any) -> dict[str, Any]:
    """One ``PicoCameraConfig`` per recorded eye, keyed by observation name.

    Each eye becomes its own camera and its own video key —
    ``left_headcam`` / ``right_headcam`` — rather than one merged
    double-width frame. Two independent streams are what the headset
    actually sends, so this is the shape with the least translation, and it
    lets a consumer take one eye without decoding both.

    The cost is that nothing downstream can see whether a given frame's two
    eyes came from the same capture; ``read_head_camera_skew`` exists to
    keep that visible.

    Shared so the single and bimanual robots cannot drift apart on which
    fields feed the cameras; both expose the same ``--robot.head_camera_*``
    surface.
    """
    from lerobot.cameras.pico import PicoCameraConfig

    eyes = ("left", "right") if config.head_camera_eyes == "both" else (config.head_camera_eyes,)
    return {
        HEAD_CAMERA_KEYS[eye]: PicoCameraConfig(
            width=config.head_camera_width,
            height=config.head_camera_height,
            fps=config.head_camera_fps,
            eyes=eye,
            startup_timeout_s=config.head_camera_startup_timeout_s,
            stale_after_s=config.head_camera_stale_after_s,
            pair_max_skew_ms=config.head_camera_pair_max_skew_ms,
        )
        for eye in eyes
    }


def read_head_camera_skew(cameras: dict[str, Any], max_skew_ms: float) -> float | None:
    """How far apart the two head-camera eyes' last frames are, in ms.

    Returns None when both eyes are not being recorded, or when either has
    not produced a frame yet. Returns a negative value to mean "same
    sequence number", which is a definitive match and makes the timestamp
    comparison moot.

    Recording the eyes as separate keys drops the pairing guarantee the
    merged frame had, so this is the only thing standing between a
    mis-synchronised stereo pair and a dataset that looks fine.
    """
    left, right = cameras.get("left_headcam"), cameras.get("right_headcam")
    if left is None or right is None:
        return None
    lm, rm = left.last_frame_meta(), right.last_frame_meta()
    if lm is None or rm is None:
        return None
    if int(lm["frame_sequence"]) == int(rm["frame_sequence"]):
        return -1.0
    skew = abs(int(lm["timestamp_ns"]) - int(rm["timestamp_ns"])) / 1e6
    return skew if skew > max_skew_ms else 0.0


def read_head_pose(logger: Any, warned: bool) -> tuple[dict[str, float], bool]:
    """Headset pose as ``head_camera.{x,y,z,r1..r6}``, in the world frame.

    Read from the Pico SDK rather than the camera, and remapped with the same
    Pico→world rotation the trackers use — so unlike the Insight VIO pose this
    replaces, it shares a frame with ``tcp.*`` and the two can be compared.

    Never raises: the head pose is supplementary and losing it should not take
    an episode down. Returns zeros if the SDK has nothing, warning once.

    Args:
        logger: used for the one-shot warning.
        warned: whether the warning has already been emitted.

    Returns:
        ``(observation_keys, warned)`` — pass ``warned`` back in next call.
    """
    import numpy as np

    from lerobot.teleoperators.pico4 import xrt_session
    from lerobot.teleoperators.pico4.tracker import PICO_TO_WORLD_R
    from lerobot.utils.robot_utils import (
        matrix_to_pose7d,
        quaternion_to_matrix,
        quaternion_to_rotation_6d,
    )

    try:
        xrt = xrt_session.module()
        pose = None if xrt is None else xrt.get_headset_pose()
        if pose is None or len(pose) < 7:
            raise ValueError(f"headset pose unavailable (got {pose!r})")

        # SDK order is xyzw; everything downstream is wxyz.
        raw = np.asarray(pose, dtype=np.float64)
        wxyz = np.array([raw[0], raw[1], raw[2], raw[6], raw[3], raw[4], raw[5]])

        t_world_head = quaternion_to_matrix(wxyz, input_format="wxyz")
        g = np.eye(4)
        g[:3, :3] = PICO_TO_WORLD_R
        t_world_head = g @ t_world_head @ g.T

        out = matrix_to_pose7d(t_world_head, output_format="wxyz")
        values = (*out[:3], *quaternion_to_rotation_6d(*out[3:7]))
    except Exception as e:
        if not warned:
            warned = True
            logger.warn(f"Head pose unavailable, recording zeros: {e}")
        values = (0.0,) * 9

    return {f"head_camera.{k}": float(v) for k, v in zip(HEAD_POSE_KEYS, values, strict=True)}, warned


def split_camera_read(cam_name: str, frame: Any, display_keys: dict[str, str] | None = None) -> dict[str, Any]:
    """Fan one camera read out into observation keys.

    A tactile sensor asked for several output types returns
    ``{output_type: array}`` (``XenseTactileCamera._format_read_result``); every
    other camera returns a bare array. ``display_keys`` maps output type →
    display-only observation key for this camera; whichever type is left over is
    the recorded one and keeps ``cam_name``.
    """
    if not isinstance(frame, dict):
        return {cam_name: frame}

    display_keys = display_keys or {}
    obs: dict[str, Any] = {}
    for output_type, data in frame.items():
        obs[display_keys.get(output_type, cam_name)] = data
    return obs


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
        self._headcam_skew_count = 0
        self._headcam_warn_at = 0.0

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
            got = self._disc_tactiles.get(side, {})
            if len(got) != n_exp:
                raise ValueError(
                    f"Expected {n_exp} {side} tactile sensors (on the {side} "
                    f"gripper's USB hub), found {len(got)}: {sorted(got.values())}."
                )
            output_types = tactile_camera_output_types(
                list(self.config.tactile_output_types),
                list(self.config.tactile_display_output_types),
            )
            for finger, sn in sorted(got.items()):
                cam_name = f"tactile_{finger}"
                cfg = XenseTactileCameraConfig(
                    serial_number=sn,
                    fps=self.config.tactile_fps,
                    output_types=output_types,
                    diff_gain=self.config.tactile_diff_gain,
                )
                configs[cam_name] = cfg
                # Read the types back off the camera config: it normalises the
                # config strings ("DIFFERENCE", "XenseOutputType.DIFFERENCE", …)
                # into the same enum whose .value keys the read dict. Recorded
                # type came first, so everything after it is display-only.
                display_keys = {
                    output_type.value: tactile_display_key(cam_name, output_type.value)
                    for output_type in cfg.output_types[1:]
                }
                if display_keys:
                    self._tactile_display_keys[cam_name] = display_keys
        if self.config.enable_wrist_camera:
            sn = self._disc_cameras.get(side)
            if not sn:
                raise ValueError(
                    f"No {self._role} wrist camera found for the {side} side (rule: {side} == {parity} sequence)."
                )
            configs["wrist_cam"] = OpenCVCameraConfig(
                index_or_path=resolve_wrist_camera_path(sn),
                width=self.config.wrist_camera_width,
                height=self.config.wrist_camera_height,
                fps=self.config.wrist_camera_fps,
            )

        if self.config.enable_head_camera:
            configs.update(build_head_camera_configs(self.config))
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
        features: dict[str, type | tuple] = {}
        for key, spec in self.observation_features.items():
            display_keys = self._tactile_display_keys.get(key)
            if display_keys:
                for display_key in display_keys.values():
                    features[display_key] = spec
            else:
                features[key] = spec

        # Display-only: the tracker's own pose, so the viewer can draw it next to
        # the EE frame and show the mount transform. Absent from
        # observation_features, so it never reaches a dataset.
        if self._tracker_sn is not None:
            for k in ("x", "y", "z", "r1", "r2", "r3", "r4", "r5", "r6"):
                features[f"tracker.{k}"] = float
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
                raise RuntimeError(f"No {self._role} gripper discovered for the {self._side} side.")
            gripper_cls = LeaderGripper if self._role == "leader" else FollowerGripper
            self.logger.info(
                f"  TacCap-Gripper: side={self._endpoints.side} role={self._endpoints.role} "
                f"fw_sn={self._endpoints.firmware_sn!r} mcu={self._endpoints.mcu_serial!r}"
            )
            self._gripper = self._open_gripper(gripper_cls, self._endpoints.mcu_device)
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
            # Display-only (see display_features): dropped by
            # select_display_observation's counterpart on the recording path.
            obs.update(self._tracker.get_tracker_display())

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

        if self.config.enable_head_camera:
            head, self._head_pose_warned = read_head_pose(self.logger, self._head_pose_warned)
            obs.update(head)

        if self.config.enable_head_camera:
            skew = read_head_camera_skew(self.cameras, self.config.head_camera_pair_max_skew_ms)
            if skew is not None and skew > 0.0:
                self._headcam_skew_count += 1
                now = time.monotonic()
                if now - self._headcam_warn_at > 5.0:
                    self._headcam_warn_at = now
                    self.logger.warn(
                        f"Head camera eyes are {skew:.1f}ms apart (limit "
                        f"{self.config.head_camera_pair_max_skew_ms:.0f}ms); "
                        f"{self._headcam_skew_count} frames so far. left_headcam and "
                        "right_headcam are recorded as separate keys, so a mismatched "
                        "pair is not otherwise visible in the dataset."
                    )

        # The head cameras need no special case here — they are ordinary
        # cameras, and the pose comes from the SDK above, not from a frame.
        for cam_name, cam in self.cameras.items():
            obs.update(split_camera_read(cam_name, cam.async_read(), self._tactile_display_keys.get(cam_name)))

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

    def _open_gripper(self, gripper_cls, mcu_device: str):
        """Open the gripper, asking the firmware to normalise the jaw position.

        ``LeaderGripper(normalize_position=True)`` reads the encoder-max
        calibration off the device (``Cmd::EncoderMaxCal``, firmware >= V2.1) and
        installs a converter, so every sample's ``position`` is the true opening
        of *this* unit in [0, 1]. That beats dividing by a config constant, which
        is one number for every gripper ever built: a unit whose real travel is
        1.30 rad would only ever read 0.76 at fully open.

        Two things fall back to ``gripper_open_rad`` instead:

        * **Followers** — ``EncoderMaxCal`` is leader-only, and the follower
          class does not take the flag at all.
        * **Uncalibrated or pre-V2.1 leaders** — the constructor raises. Rather
          than fail the session we re-open with ``encoder_max_rad`` supplied from
          the host, which is the same arithmetic as before, and say so once. Run
          ``fisheye_cal.py measure-encoder-max`` to move a unit onto the firmware
          value.
        """
        if gripper_cls is not LeaderGripper:
            self.logger.info(f"  Jaw normalised by config gripper_open_rad={self.config.gripper_open_rad} (follower)")
            return gripper_cls(mcu_device)

        try:
            gripper = gripper_cls(mcu_device, normalize_position=True)
        except Exception as e:
            self._gripper_norm_source = "config"
            self.logger.warn(
                f"Firmware encoder-max calibration unavailable ({e}); falling back to "
                f"gripper_open_rad={self.config.gripper_open_rad}. gripper.pos will not reach 1.0 "
                "if this unit's real travel differs. Fix with: python "
                "third_party/taccap-gripper/python/examples/fisheye_cal.py measure-encoder-max"
            )
            return gripper_cls(mcu_device, normalize_position=True, encoder_max_rad=self.config.gripper_open_rad)

        self._gripper_norm_source = "firmware"
        self.logger.info("  Jaw normalised by the firmware's encoder-max calibration")
        return gripper

    def _read_gripper_normalized(self) -> float:
        """Jaw opening in [0, 1] — 0 closed (the encoder zero), 1 fully open.

        Prefers the SDK's ``position``, which the firmware's own encoder-max
        calibration produces (see :meth:`_open_gripper`). It is ``nan`` when no
        converter is installed — a follower, or a leader that fell back — so the
        radians path stays as the backstop rather than letting a nan reach the
        dataset.
        """
        try:
            sample = self._gripper.encoder.read_once()
        except Exception as e:
            self.logger.warn(f"Encoder read failed: {e}")
            return 0.0

        normalised = float(getattr(sample, "position", float("nan")))
        if np.isfinite(normalised):
            return float(np.clip(normalised, 0.0, 1.0))

        opened = self.config.gripper_open_rad
        if opened <= 0.0:  # guarded in config.__post_init__ but be defensive
            return 0.0
        return float(np.clip(float(sample.position_rad) / opened, 0.0, 1.0))

    def get_endpoints(self):
        """Hardware discovery info populated on connect (None otherwise)."""
        return self._endpoints
