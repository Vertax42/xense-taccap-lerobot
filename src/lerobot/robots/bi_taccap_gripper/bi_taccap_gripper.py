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
→ right) and role (leader vs follower). No serials are listed in the
config; a non-conforming or missing/duplicated device raises a clear error.

Observation features (per side ``{s}`` in left/right):
    {s}_tcp.x/y/z, {s}_tcp.r1..r6   -- Pico4 tracker → EE 6D pose (if enable_tracker)
    {s}_gripper.pos                 -- normalised jaw [0=closed, 1=open]
    {s}_imu.accel/gyro/mag.{x,y,z}  -- optional
    {s}_wrist                       -- wrist UVC frame (if enable_wrist_camera)
    {s}_tactile_left / {s}_tactile_right -- recorded tactile frames (sensor on
                                       left/right finger), ``rectify`` by default
    left_head / right_head    -- Pico headset camera, one key per eye (if
                                       enable_head_camera). NOTE these name the
                                       headset's EYES, not the left/right arm.
    head_camera.x/y/z/r1..r6        -- headset pose, same world frame as *_tcp.*
                                       (also an action -- see action_features)

Display-only keys (in ``get_observation()`` and ``display_features``, absent from
``observation_features``, so Rerun shows them but the dataset never sees them).
None by default: ``tactile_display_output_types`` is ``rectify``, the recorded
type, so Rerun watches the recorded ``{s}_tactile_{left,right}``. Ask for a
different type there and each one appears as a sibling key, e.g.
    {s}_tactile_{left,right}_difference -- amplified deformation view of the same
                                       sensor read
"""

from __future__ import annotations

from functools import cached_property
from typing import Any

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.utils.robot_utils import get_logger

from ..robot import Robot
from ..taccap_gripper import serial_discovery as disco
from ..taccap_gripper.camera_health import CameraReadGuard
from ..taccap_gripper.common import (
    HEAD_POSE_KEYS,
    POSE_KEYS,
    GripperReadGuard,
    HeadSkewMonitor,
    build_hardware_manifest,
    build_head_camera_configs,
    build_tactile_camera_configs,
    build_wrist_camera_config,
    connect_cameras_parallel,
    disconnect_cameras_parallel,
    hardware_manifest_unit,
    open_gripper,
    prewarm_tactile_config_cache,
    read_head_pose,
    resolve_wrist_undistorter,
    split_camera_read,
    swap_tactile_display_features,
    tactile_camera_output_types,
    wrist_undistort_record,
)
from ..taccap_gripper.ee_transform import resolve_tracker_to_ee
from .config_bi_taccap_gripper import BiTaccapGripperConfig

_SIDES = ("left", "right")

# ---- TacCap-Gripper SDK -----------------------------------------------------
try:
    from xense.taccap import (
        FISHEYE_FALLBACK_CAL,
        FisheyeUndistorter,
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
        self.logger = get_logger(f"BiTaccapGripper-{config.id}")
        self._role = disco.normalize_role(config.role)

        any_gripper = any(getattr(config, f"{s}_enable_gripper") for s in _SIDES)
        # Tactile discovery now pairs sensors to a gripper by USB hub, so it also
        # needs the SDK (scan_grippers) to resolve each hub's side.
        needs_sdk = any_gripper or config.tactiles_per_side > 0
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

        # Per-side hardware handles, populated on connect.
        self._gripper: dict[str, Any] = dict.fromkeys(_SIDES)  # Leader/FollowerGripper
        self._endpoints: dict[str, Any] = dict.fromkeys(_SIDES)  # GripperEndpoints
        # {observation key: FisheyeUndistorter}, empty unless wrist_undistort is
        # on. The flag is shared by both arms but the intrinsics are per unit, so
        # one arm can rectify from its own flash while the other falls back.
        self._wrist_undistorters: dict[str, Any] = {}
        self._wrist_undistort_source: dict[str, str | None] = dict.fromkeys(_SIDES)
        self._tracker: dict[str, Pico4TrackerReader | None] = dict.fromkeys(_SIDES)
        # Auto-discover tactile + wrist cameras and build their configs so the
        # observation schema is ready before connect(). Tactiles are paired to a
        # gripper by USB hub, so this scans the serial bus (grippers must be
        # powered at construction); wrist cameras are filesystem-only.
        self._camera_configs = self._discover_camera_configs()
        self.cameras = make_cameras_from_configs(self._camera_configs)
        self._head_pose_warned = False
        self._head_skew = HeadSkewMonitor(config.head_camera_pair_max_skew_ms, self.logger)

        # Auto-discover the Pico4 motion tracker(s): enumerate from the XenseVR PC
        # service and assign one per side by serial (second-to-last digit, strict).
        # Drives the pose schema, so it runs here (pre-connect) like the cameras.
        # Where each side's gripper.pos comes from once connected: "firmware"
        # (that unit's own encoder-max calibration) or "config" (gripper_open_rad).
        self._gripper_norm_source: dict[str, str] = dict.fromkeys(_SIDES, "config")
        self._tracker_sn_by_side: dict[str, str] = {}
        if config.enable_tracker:
            self._tracker_sn_by_side = disco.resolve_pico_trackers(
                _SIDES,
                {s: getattr(config, f"{s}_tracker_serial") for s in _SIDES},
                lambda: Pico4TrackerReader.list_serial_numbers(
                    device_wait_timeout=config.tracker_wait_timeout,
                    logger_name=config.id,
                ),
            )
            self.logger.info(f"Pico4 trackers: {self._tracker_sn_by_side}")

        self._is_connected = False

        # Graceful degradation on mid-episode camera loss (hot-unplug, hub drop):
        # substitutes the last good frame and trips ``device_lost`` so the caller
        # can stop cleanly and save what was recorded. See ``camera_health``.
        self._cam_guard = CameraReadGuard(self._camera_configs, self.logger)
        self._enc_guard = GripperReadGuard(self.logger)

    # ------------------------------------------------------------------ discovery

    def _discover_camera_configs(self) -> dict[str, Any]:
        """Build the tactile + wrist camera configs from serial auto-discovery.

        Tactiles (``{side}_tactile_{left,right}``) are paired to a gripper by USB
        hub (hub → gripper firmware SN → side) and keyed by finger (GSPS last
        digit); wrist cameras (``{side}_wrist``) come from ``/dev/v4l/by-id``.
        Counts are validated per side so a mis-installed sensor is caught here,
        not mid-episode.

        Also fills ``self._tactile_display_keys`` (camera → {output type →
        display-only observation key}), the map ``get_observation`` and
        ``display_features`` use to route the extra, unrecorded views.
        """
        n_exp = self.config.tactiles_per_side
        tactiles = disco.discover_tactiles_by_hub(self._role) if n_exp else {"left": {}, "right": {}}
        # Kept for ``hardware_manifest``: the camera configs downstream carry the
        # serials too, but only as an opaque field of each backend's config.
        self._disc_tactiles = tactiles
        want_wrist = any(getattr(self.config, f"{s}_enable_wrist_camera") for s in _SIDES)
        cameras = disco.discover_wrist_cameras(self._role) if want_wrist else {}

        configs: dict[str, Any] = {}
        self._tactile_display_keys: dict[str, dict[str, str]] = {}
        output_types = tactile_camera_output_types(
            list(self.config.tactile_output_types),
            list(self.config.tactile_display_output_types),
        )
        for side in _SIDES:
            parity = "odd" if side == "left" else "even"
            if n_exp:
                tactile_configs, display_keys = build_tactile_camera_configs(
                    tactiles.get(side, {}),
                    side=side,
                    key_prefix=f"{side}_",
                    expected=n_exp,
                    fps=self.config.tactile_fps,
                    output_types=output_types,
                    diff_gain=self.config.tactile_diff_gain,
                )
                configs.update(tactile_configs)
                self._tactile_display_keys.update(display_keys)
            if getattr(self.config, f"{side}_enable_wrist_camera"):
                sn = cameras.get(side)
                if not sn:
                    raise ValueError(
                        f"No {self._role} wrist camera found for the {side} side (rule: {side} == {parity} sequence)."
                    )
                configs[f"{side}_wrist"] = build_wrist_camera_config(
                    sn,
                    width=self.config.wrist_camera_width,
                    height=self.config.wrist_camera_height,
                    fps=self.config.wrist_camera_fps,
                    fourcc=self.config.wrist_camera_fourcc,
                )

        if self.config.enable_head_camera:
            configs.update(build_head_camera_configs(self.config))
        return configs

    @property
    def tactile_runtimes(self) -> dict[str, bytes]:
        """Each connected tactile sensor's runtime bundle, keyed by serial.

        Recorded into the dataset so the derived channels (depth, force,
        difference) can be rebuilt from the ``rectify`` stream later without the
        physical sensor — and without the risk that it has since been
        recalibrated, which changes the reference image the reconstruction is
        measured against.

        Sensors that cannot produce one are simply absent: that costs the derived
        channels for those episodes, which is not a reason to fail a recording.
        """
        runtimes: dict[str, bytes] = {}
        for camera in self.cameras.values():
            export = getattr(camera, "export_runtime_config", None)
            serial = getattr(camera, "serial_number", None)
            if export is None or not serial:
                continue
            blob = export()
            if blob:
                runtimes[serial] = blob
        return runtimes

    @property
    def hardware_manifest(self) -> dict[str, Any]:
        """Which physical devices this rig is, for the recording to carry along.

        Read it **while connected**: the gripper serials are the firmware SNs
        read over the wire at ``connect()``, and ``_release()`` drops the
        endpoints again, so before/after they come out ``None``. Tactile serials
        come from construction-time discovery and are always there.

        ``--robot.id`` is only a station label; this is the identity.
        """
        return build_hardware_manifest(
            robot_type=self.name,
            robot_id=self.id,
            role=self._role,
            units=[
                hardware_manifest_unit(
                    side,
                    endpoints=self._endpoints[side],
                    tactile_serials=self._disc_tactiles.get(side, {}),
                    key_prefix=f"{side}_",
                    wrist_undistort=wrist_undistort_record(
                        has_wrist_camera=f"{side}_wrist" in self._camera_configs,
                        source=self._wrist_undistort_source[side],
                        balance=self.config.wrist_undistort_balance,
                    ),
                )
                for side in _SIDES
            ],
        )

    # ------------------------------------------------------------------ schema

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        features: dict[str, type | tuple] = {}

        for side in _SIDES:
            if side in self._tracker_sn_by_side:
                for k in POSE_KEYS:
                    features[f"{side}_tcp.{k}"] = float
            if getattr(self.config, f"{side}_enable_gripper"):
                features[f"{side}_gripper.pos"] = float
            if getattr(self.config, f"{side}_enable_imu"):
                for axis in ("x", "y", "z"):
                    features[f"{side}_imu.accel.{axis}"] = float
                    features[f"{side}_imu.gyro.{axis}"] = float
                    features[f"{side}_imu.mag.{axis}"] = float

        if self.config.enable_head_camera:
            for key in HEAD_POSE_KEYS:
                features[f"head_camera.{key}"] = float

        # Tactile + wrist cameras (keys already left_/right_ prefixed).
        # ``frame_width`` differs from ``width`` only for the stereo head
        # camera, where ``width`` is one eye and a merged frame is twice that.
        for cam_name, cam_cfg in self._camera_configs.items():
            width = getattr(cam_cfg, "frame_width", cam_cfg.width)
            features[cam_name] = (cam_cfg.height, width, 3)

        return features

    @cached_property
    def display_features(self) -> dict[str, type | tuple]:
        """Rerun-facing schema: ``observation_features`` with each tactile camera's
        recorded stream swapped for its display-only view(s), in place.

        By default there is nothing to swap: ``tactile_display_output_types`` names
        the recorded type (``rectify``), so the viewer watches the recorded stream
        and this is ``observation_features``. Point it at another type — the
        amplified ``difference``, say — and the swap kicks in: swapping rather than
        adding keeps the recorded stream out of the viewer entirely, four tactile
        tiles and not eight, with no extra Rerun image bandwidth. Cameras with no
        display-only view are passed through.
        """
        features = swap_tactile_display_features(self.observation_features, self._tactile_display_keys)

        # Display-only: each tracker's own pose, so the viewer can draw it next
        # to that side's EE frame and show the mount transform. Absent from
        # observation_features, so it never reaches a dataset.
        for side in _SIDES:
            if side in self._tracker_sn_by_side:
                for k in POSE_KEYS:
                    features[f"{side}_tracker.{k}"] = float
        return features

    @cached_property
    def action_features(self) -> dict[str, type]:
        """The 'demonstration' action the rig emits: pose + jaw per side, plus the
        headset pose when the head camera is on. No image data.

        ``head_camera.*`` is unprefixed and appears once, because there is one
        headset for the two arms. It belongs in the action for the same reason
        ``{side}_tcp.*`` does: where the operator looked while demonstrating is
        something a policy reproduces, it is in the same world frame and the same
        position + 6D rotation layout, and only keys in ``action_features`` take
        part in the shifted-frame pairing that makes a row a "where to move next"
        target.
        """
        features: dict[str, type] = {}
        for side in _SIDES:
            if side in self._tracker_sn_by_side:
                for k in POSE_KEYS:
                    features[f"{side}_tcp.{k}"] = float
            if getattr(self.config, f"{side}_enable_gripper"):
                features[f"{side}_gripper.pos"] = float
        if self.config.enable_head_camera:
            for k in HEAD_POSE_KEYS:
                features[f"head_camera.{k}"] = float
        return features

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def device_lost(self) -> bool:
        """True once a camera **or a jaw encoder** has been detected as
        physically lost mid-episode (hot-unplug / hub drop / brown-out). The
        record loop polls this to stop cleanly and save the in-progress episode
        instead of recording through the loss.

        The encoder half matters as much as the camera half: a dead encoder used
        to report ``0.0``, an ordinary "closed" reading, into both the
        observation and the action for every remaining frame — a loss that left
        no trace in the data at all, unlike a frozen image."""
        return self._cam_guard.lost or self._enc_guard.lost

    @property
    def is_calibrated(self) -> bool:
        """Factory calibration; the only per-unit step is the gripper encoder
        zero, which lives in firmware (set once via the SDK's calibrate.py)."""
        return self.is_connected

    # ------------------------------------------------------------------ lifecycle

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.logger.info("Connecting BiTacCap-Gripper...")

        try:
            self._connect_devices()
        except BaseException:
            # Includes KeyboardInterrupt: Ctrl+C during a slow startup is the
            # most likely way to land here, and it must release like any other
            # failure. Without this the gripper serial ports, the xrt session
            # and every camera that did open stayed held by a process that then
            # often failed to exit cleanly, so the next run met a busy device.
            self.logger.error("Connect failed part-way; releasing what was already open")
            self._release()
            raise

        self._is_connected = True
        self.logger.info(f"✅ {self} connected.")

    def _connect_devices(self) -> None:
        """Bring up gripper(s), tracker(s) and cameras.

        Split out of ``connect`` so a failure anywhere in here unwinds through
        one place. The body is unchanged; it sits at the same indentation a
        method body needs, which is why this move is verbatim.
        """
        self._cam_guard.reset()  # a reconnect must not inherit the last session's losses
        self._enc_guard.reset()

        # 1. Grippers — auto-discovered by serial (side + role) on the bus.
        enabled_gripper_sides = tuple(s for s in _SIDES if getattr(self.config, f"{s}_enable_gripper"))
        grippers = disco.discover_grippers(self._role) if enabled_gripper_sides else {}
        gripper_cls = LeaderGripper if self._role == "leader" else FollowerGripper

        for side in _SIDES:
            # Gripper (MCU transport only; cameras come from the LeRobot camera
            # framework, so open_cameras stays False).
            if getattr(self.config, f"{side}_enable_gripper"):
                endpoints = grippers.get(side)
                if endpoints is None:
                    raise RuntimeError(f"No {self._role} gripper discovered for the {side} side.")
                self._endpoints[side] = endpoints
                self.logger.info(
                    f"  [{side}] TacCap-Gripper: side={endpoints.side} role={endpoints.role} "
                    f"fw_sn={endpoints.firmware_sn!r} mcu={endpoints.mcu_serial!r}"
                )
                self._gripper[side], self._gripper_norm_source[side] = open_gripper(
                    gripper_cls,
                    endpoints.mcu_device,
                    is_leader=self._role == "leader",
                    open_rad=getattr(self.config, f"{side}_gripper_open_rad"),
                    logger=self.logger,
                    label=f"[{side}] ",
                )
                self.logger.info(f"  [{side}] ✅ {gripper_cls.__name__} attached (MCU-only, read-only)")

            if self.config.wrist_undistort and f"{side}_wrist" in self._camera_configs:
                # Per arm: each gripper is asked for its own intrinsics, so one
                # side falling back to the SDK reference does not drag the other
                # onto it. Built before the cameras open so an uncalibrated unit
                # warns during connect, not mid-recording.
                undistorter, source = resolve_wrist_undistorter(
                    self._gripper[side],
                    undistorter_cls=FisheyeUndistorter,
                    fallback_cal=FISHEYE_FALLBACK_CAL,
                    balance=self.config.wrist_undistort_balance,
                    logger=self.logger,
                    label=f"[{side}] ",
                )
                self._wrist_undistorters[f"{side}_wrist"] = undistorter
                self._wrist_undistort_source[side] = source

            # 2. Pico4 tracker (auto-discovered SN per side, pinned here).
            if side in self._tracker_sn_by_side:
                # None means "use this side's built-in mount transform"; the
                # left value is the right one mirrored (see ee_transform).
                ee_pos, ee_quat = resolve_tracker_to_ee(
                    side,
                    getattr(self.config, f"{side}_tracker_to_ee_pos"),
                    getattr(self.config, f"{side}_tracker_to_ee_quat"),
                )
                self.logger.info(f"  [{side}] tracker→TCP: pos={ee_pos.tolist()} quat={ee_quat.tolist()}")
                tracker = Pico4TrackerReader(
                    tracker_sn=self._tracker_sn_by_side[side],
                    tracker_to_ee_pos=ee_pos,
                    tracker_to_ee_quat=ee_quat,
                    device_wait_timeout=self.config.tracker_wait_timeout,
                    logger_name=f"{self.config.id}-{side}",
                )
                # No init-pose alignment — see the note in the config module.
                tracker.connect()
                self._tracker[side] = tracker
                self.logger.info(f"  [{side}] ✅ Pico4 tracker connected (world frame)")

        # 3. Cameras (tactile + wrist, auto-discovered in __init__).
        #    Pre-warm the config cache sequentially first so the parallel connect
        #    below never triggers a Sunplus flash read (device reset) mid-open.
        prewarm_tactile_config_cache(self._camera_configs, self.logger)
        #    Then connect concurrently — each camera's V4L2 open + warmup overlaps
        #    in time rather than summing (cf. v0.4.4 bi_arx5). Configs now come
        #    from the cache (no flash read), so no device reset during connect.
        connect_cameras_parallel(self.cameras, self.logger)

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")

        self.logger.info(f"Disconnecting {self}...")
        self._release()
        self.logger.info(f"✅ {self} disconnected.")

    def _release(self) -> None:
        """Release everything currently held, tolerating a partial hold.

        Split out of ``disconnect`` so ``connect`` can call it when it fails
        part-way. It cannot go through ``disconnect``: that refuses on a robot
        whose ``_is_connected`` was never set, which is exactly the state a
        failed connect leaves behind — and the same reason ``_safe_disconnect``
        in the teleop script skipped such a robot entirely, leaking the gripper
        serial ports and the xrt session along with the cameras.
        """
        disconnect_cameras_parallel(self.cameras, self.logger)

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
            self._wrist_undistort_source[side] = None

        self._wrist_undistorters = {}
        self._is_connected = False

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
                # Display-only (see display_features).
                obs.update(self._tracker[side].get_tracker_display(prefix=f"{side}_"))

            if getattr(self.config, f"{side}_enable_gripper") and self._gripper[side] is not None:
                obs[f"{side}_gripper.pos"] = self._enc_guard.read(
                    side, self._gripper[side], getattr(self.config, f"{side}_gripper_open_rad"), f"[{side}] "
                )

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

        if self.config.enable_head_camera:
            head, self._head_pose_warned = read_head_pose(self.logger, self._head_pose_warned)
            obs.update(head)
            self._head_skew.check(self.cameras)

        # The head cameras need no special case here — they are ordinary
        # cameras, and the pose comes from the SDK above, not from a frame.
        # Insight bundled the two, which is why this used to be read apart.
        for cam_name, cam in self.cameras.items():
            frame = self._cam_guard.read(cam_name, cam)
            # After the guard, not before — see the single-arm robot for why
            # (the guard spots a frozen stream by frame identity, and apply()
            # returns a fresh array every call).
            undistorter = self._wrist_undistorters.get(cam_name)
            if undistorter is not None:
                frame = undistorter.apply(frame)
            obs.update(
                split_camera_read(
                    cam_name,
                    frame,
                    self._tactile_display_keys.get(cam_name),
                )
            )

        return obs

    def send_action(self, action: dict[str, Any] | None = None) -> dict[str, Any]:
        """No-op: passive demonstration device. The operators drive the jaws
        mechanically — we never command either motor."""
        return action or {}

    def get_action(self) -> dict[str, Any]:
        """Return the prefixed pose + gripper dict the rig emits as a teleoperator,
        i.e. exactly the keys in :attr:`action_features`.

        The record loop does not call this — it takes the ``action_features``
        subset of the observation it already sampled, so the row costs one
        hardware read. This exists for the teleoperator role, and reads the
        headset pose again rather than sharing that sample.
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")
        action: dict[str, Any] = {}
        for side in _SIDES:
            if side in self._tracker_sn_by_side and self._tracker[side] is not None:
                for k, v in self._tracker[side].get_action().items():
                    action[f"{side}_{k}"] = v
            if getattr(self.config, f"{side}_enable_gripper") and self._gripper[side] is not None:
                action[f"{side}_gripper.pos"] = self._enc_guard.read(
                    side, self._gripper[side], getattr(self.config, f"{side}_gripper_open_rad"), f"[{side}] "
                )
        if self.config.enable_head_camera:
            head, self._head_pose_warned = read_head_pose(self.logger, self._head_pose_warned)
            action.update(head)
        return action

    # ------------------------------------------------------------------ helpers

    def get_endpoints(self):
        """Per-side hardware discovery info populated on connect."""
        return dict(self._endpoints)
