#!/usr/bin/env python

# Copyright 2026 The XenseRobotics Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Everything the single and bimanual TacCap grippers share.

The two robots differ only in how many units they drive and how their
observation keys are prefixed; the device handling underneath is the same. This
module is that sameness, kept in one place so the two cannot drift — which they
did once, when only the bimanual side learned to survive a camera unplug.

It lives in a module of its own rather than in ``taccap_gripper.py`` so that the
bimanual robot does not have to import the single robot's implementation to
reach a helper, and so the *config* modules can validate their head-camera
fields without importing a robot at all (that import used to be deferred into
``__post_init__`` to dodge a cycle).

Stateless helpers take a ``label`` used only to prefix log lines: ``""`` for the
single gripper, ``"[left] "`` for one arm of the bimanual rig.
"""

from __future__ import annotations

import glob
import json
import os
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any

import numpy as np

from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.cameras.xense.configuration_xense import XenseTactileCameraConfig
from lerobot.utils.constants import TACCAP_HARDWARE_MANIFEST_PATH, TACCAP_RUNTIME_DIR

# 6D rotation convention (matches ``vive_tracker``): r1..r3 is the first column
# of the rotation matrix, r4..r6 the second. Position first, so the tuple is the
# full 9-DoF pose layout used by ``tcp.*``, ``tracker.*`` and ``head_camera.*``
# alike — one definition instead of the six copies these robots used to carry.
POSE_KEYS = ("x", "y", "z", "r1", "r2", "r3", "r4", "r5", "r6")

HEAD_POSE_KEYS = POSE_KEYS

HEAD_CAMERA_KEYS = {"left": "left_head", "right": "right_head"}

__all__ = [
    "POSE_KEYS",
    "HEAD_POSE_KEYS",
    "HEAD_CAMERA_KEYS",
    "HARDWARE_MANIFEST_PATH",
    "RUNTIME_DIR",
    "GripperReadGuard",
    "HeadSkewMonitor",
    "build_hardware_manifest",
    "build_head_camera_configs",
    "build_tactile_camera_configs",
    "build_wrist_camera_config",
    "connect_cameras_parallel",
    "disconnect_cameras_parallel",
    "epoch_for_episode",
    "hardware_manifest_unit",
    "manifest_epochs",
    "open_gripper",
    "prewarm_tactile_config_cache",
    "read_gripper_normalized",
    "read_head_camera_skew",
    "read_head_pose",
    "resolve_wrist_camera_path",
    "split_camera_read",
    "swap_tactile_display_features",
    "tactile_runtime_for_key",
    "tactile_serial_for_key",
    "tactile_camera_output_types",
    "tactile_display_key",
    "validate_robot_id",
    "write_hardware_manifest",
    "write_tactile_runtimes",
]


# ---------------------------------------------------------------- wrist camera


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


def build_wrist_camera_config(
    serial: str, *, width: int, height: int, fps: int, fourcc: str | None
) -> OpenCVCameraConfig:
    """``OpenCVCameraConfig`` for one gripper's wrist UVC camera.

    Lives here for the same reason the tactile and head builders do: the single
    and bimanual robots differ only in the observation key they file the result
    under (``wrist_cam`` vs ``{side}_wrist``), and this was the one camera whose
    construction they each carried a copy of.

    ``fourcc`` matters more than it looks. Left at ``None``, OpenCV's V4L2
    backend picks the format itself, and its preference order puts YUYV ahead of
    MJPEG — so a 640x480@30 wrist camera negotiates ~147 Mbit/s of *uncompressed*
    video and the UVC driver reserves a top isochronous altsetting (~196 Mbit/s)
    to carry it. A gripper's hub carries three UVC devices (two tactile sensors
    plus this camera) behind one xHCI root port, and a root port only has ~384
    Mbit/s of isochronous budget, so three uncompressed streams overrun it: the
    third ``open()`` fails with ``Not enough bandwidth for altsetting N`` in dmesg,
    surfacing as a camera that "cannot be opened" even though its ``/dev/video*``
    node is right there. MJPEG is what buys the headroom back.
    """
    return OpenCVCameraConfig(
        index_or_path=resolve_wrist_camera_path(serial),
        width=width,
        height=height,
        fps=fps,
        fourcc=fourcc,
    )


# ------------------------------------------------------------ tactile pre-warm


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


# ------------------------------------------- recorded vs display-only tactile
# One sensor read carries two views: the recorded one (rectify — everything the
# sensor saw) and the display one (the amplified difference the operator reads
# contact from). The SDK hands both back from a single ``selectSensorInfo``, so
# the split is purely a matter of which observation key each lands on: the
# recorded view keeps the camera's own key and is the only one in
# ``observation_features``; each display view gets a ``{camera}_{type}`` sibling
# that only Rerun ever sees.


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


def build_tactile_camera_configs(
    discovered: dict[str, str],
    *,
    side: str,
    key_prefix: str,
    expected: int,
    fps: int,
    output_types: list[str],
    diff_gain: float,
) -> tuple[dict[str, Any], dict[str, dict[str, str]]]:
    """Camera configs for one gripper's tactile sensors, plus their display map.

    ``discovered`` is ``{finger: serial}`` from ``discover_tactiles_by_hub`` for
    this side. ``key_prefix`` is ``""`` for the single gripper (keys
    ``tactile_left``) and ``"{side}_"`` for the bimanual rig (``left_tactile_left``).

    Returns ``(configs, display_keys)`` where ``display_keys`` maps camera name →
    {output type → display-only observation key}. The count is checked here so a
    mis-installed sensor is caught at construction rather than mid-episode.
    """
    if len(discovered) != expected:
        raise ValueError(
            f"Expected {expected} {side} tactile sensors (on the {side} "
            f"gripper's USB hub), found {len(discovered)}: {sorted(discovered.values())}."
        )

    configs: dict[str, Any] = {}
    display_keys_by_camera: dict[str, dict[str, str]] = {}
    for finger, sn in sorted(discovered.items()):
        cam_name = f"{key_prefix}tactile_{finger}"
        cfg = XenseTactileCameraConfig(
            serial_number=sn,
            fps=fps,
            output_types=output_types,
            diff_gain=diff_gain,
        )
        configs[cam_name] = cfg
        # Read the types back off the camera config: it normalises the config
        # strings ("DIFFERENCE", "XenseOutputType.DIFFERENCE", …) into the same
        # enum whose .value keys the read dict. Recorded type came first, so
        # everything after it is display-only.
        display_keys = {
            output_type.value: tactile_display_key(cam_name, output_type.value) for output_type in cfg.output_types[1:]
        }
        if display_keys:
            display_keys_by_camera[cam_name] = display_keys
    return configs, display_keys_by_camera


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


def swap_tactile_display_features(
    observation_features: dict[str, type | tuple],
    tactile_display_keys: dict[str, dict[str, str]],
) -> dict[str, type | tuple]:
    """``observation_features`` with each tactile camera's recorded stream
    swapped for its display-only view(s), in place.

    The dataset gets ``rectify`` (``observation_features``), the operator gets
    the amplified ``difference`` (this). Swapping rather than adding keeps the
    recorded stream out of the viewer entirely — same tile count, same Rerun
    image bandwidth as before the split — and keeps tactile in the same slot of
    the blueprint. Cameras with no display-only view are passed through, so
    without ``tactile_display_output_types`` this is the input unchanged.
    """
    features: dict[str, type | tuple] = {}
    for key, spec in observation_features.items():
        display_keys = tactile_display_keys.get(key)
        if display_keys:
            for display_key in display_keys.values():
                features[display_key] = spec
        else:
            features[key] = spec
    return features


# ------------------------------------------------------------------ head camera


def build_head_camera_configs(config: Any) -> dict[str, Any]:
    """One ``PicoCameraConfig`` per recorded eye, keyed by observation name.

    Each eye becomes its own camera and its own video key —
    ``left_head`` / ``right_head`` — rather than one merged
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
    left, right = cameras.get("left_head"), cameras.get("right_head")
    if left is None or right is None:
        return None
    lm, rm = left.last_frame_meta(), right.last_frame_meta()
    if lm is None or rm is None:
        return None
    if int(lm["frame_sequence"]) == int(rm["frame_sequence"]):
        return -1.0
    skew = abs(int(lm["timestamp_ns"]) - int(rm["timestamp_ns"])) / 1e6
    return skew if skew > max_skew_ms else 0.0


class HeadSkewMonitor:
    """Counts mis-paired head-camera frames and warns at most every 5 s.

    Stateful, so it is an object rather than a function: the running count is
    the useful part of the message ("47 frames so far" reads very differently
    from one line), and rate-limiting needs somewhere to remember the last
    warning.
    """

    WARN_INTERVAL_S = 5.0

    def __init__(self, max_skew_ms: float, logger: Any):
        self._max_skew_ms = max_skew_ms
        self._logger = logger
        self._count = 0
        self._warned_at = 0.0

    @property
    def skewed_frames(self) -> int:
        return self._count

    def check(self, cameras: dict[str, Any]) -> None:
        skew = read_head_camera_skew(cameras, self._max_skew_ms)
        if skew is None or skew <= 0.0:
            return
        self._count += 1
        now = time.monotonic()
        if now - self._warned_at <= self.WARN_INTERVAL_S:
            return
        self._warned_at = now
        self._logger.warn(
            f"Head camera eyes are {skew:.1f}ms apart (limit {self._max_skew_ms:.0f}ms); "
            f"{self._count} frames so far. left_head and right_head are recorded as "
            "separate keys, so a mismatched pair is not otherwise visible in the dataset."
        )


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


# ------------------------------------------------------------ camera lifecycle


def connect_cameras_parallel(cameras: dict[str, Any], logger: Any) -> None:
    """Open all cameras concurrently — each camera's V4L2 open + warmup
    overlaps in time instead of summing (cf. v0.4.4 bi_arx5).

    All or nothing: if any camera fails, the ones that opened are closed again
    before the error propagates. Leaving them open is what made a single
    failure sticky — the ``with ThreadPoolExecutor`` block waits for the
    remaining submissions on the way out, so a raise here abandoned up to
    ``n-1`` *successfully opened* devices with nothing holding a reference. A
    process that then failed to exit cleanly kept ``/dev/video*`` busy, and the
    next run failed on a different camera with ``VIDIOC_REQBUFS: Device or
    resource busy`` — which reads like new hardware trouble rather than the
    wreckage of the previous attempt.

    Catches ``BaseException`` on purpose: Ctrl+C during startup is the most
    likely way to land here, and it must roll back like any other failure.
    """
    if not cameras:
        return
    n = len(cameras)
    logger.info(f"  Connecting {n} camera(s) in parallel...")
    try:
        with ThreadPoolExecutor(max_workers=min(n, 8)) as executor:
            futures = {executor.submit(cam.connect): name for name, cam in cameras.items()}
            for fut in as_completed(futures):
                name = futures[fut]
                try:
                    fut.result()
                except Exception as e:
                    logger.error(f"  Camera '{name}' connect failed: {e}")
                    raise
    except BaseException:
        logger.error("  Rolling back: closing the cameras that did open")
        disconnect_cameras_parallel(cameras, logger)
        raise
    logger.info(f"  ✅ {n} camera(s) connected")


def disconnect_cameras_parallel(cameras: dict[str, Any], logger: Any) -> None:
    """Close all cameras concurrently. Best-effort: a camera that fails to close
    is logged and the rest still get their turn."""
    if not cameras:
        return

    def _close(cam):
        if cam.is_connected:
            cam.disconnect()

    n = len(cameras)
    with ThreadPoolExecutor(max_workers=min(n, 8)) as executor:
        futures = {executor.submit(_close, cam): name for name, cam in cameras.items()}
        for fut in as_completed(futures):
            name = futures[fut]
            try:
                fut.result()
            except Exception as e:  # pragma: no cover — best-effort teardown
                logger.error(f"  Camera '{name}' disconnect error: {e}")


# ----------------------------------------------------------------- jaw encoder


def open_gripper(
    gripper_cls: Any,
    mcu_device: str,
    *,
    is_leader: bool,
    open_rad: float,
    logger: Any,
    label: str = "",
) -> tuple[Any, str]:
    """Open a gripper, asking the firmware to normalise the jaw position.

    ``LeaderGripper(normalize_position=True)`` reads the encoder-max
    calibration off the device (``Cmd::EncoderMaxCal``, firmware >= V2.1) and
    installs a converter, so every sample's ``position`` is the true opening
    of *this* unit in [0, 1]. That beats dividing by a config constant, which
    is one number for every gripper ever built: a unit whose real travel is
    1.30 rad would only ever read 0.76 at fully open.

    A leader whose travel span the firmware cannot supply is a **hard error**.
    We used to fall back to ``open_rad`` and warn, but a warning at connect
    scrolls past and the session then records a whole dataset of ``gripper.pos``
    values scaled by the wrong constant — one number for every gripper ever
    built, where a unit whose real travel is 1.30 rad reads 0.76 at fully open.
    Nothing downstream can tell that apart from a jaw that was never opened all
    the way, so the damage is silent and only visible once someone trains on it.
    Refusing to start costs one calibration run; not refusing costs the data.

    **Followers** still normalise from ``open_rad``: ``EncoderMaxCal`` is
    leader-only and the follower class does not take the flag at all.

    Returns ``(gripper, norm_source)`` where ``norm_source`` is ``"firmware"``
    for leaders and ``"config"`` for followers.

    Raises:
        RuntimeError: the leader's encoder-max calibration is unavailable —
            never stored, or firmware older than V2.1.
    """
    if not is_leader:
        logger.info(f"  {label}Jaw normalised by config gripper_open_rad={open_rad} (follower)")
        return gripper_cls(mcu_device), "config"

    try:
        gripper = gripper_cls(mcu_device, normalize_position=True)
    except Exception as e:
        raise RuntimeError(
            f"{label}This leader gripper has no encoder-max calibration, so its jaw travel "
            f"is unknown and gripper.pos cannot be computed ({type(e).__name__}: {e}).\n"
            "\n"
            "Calibrate it once, then re-run:\n"
            "\n"
            "    python third_party/taccap-gripper/python/examples/calibrate.py <left|right>\n"
            "\n"
            "That walks you through latching the closed pose as the encoder zero and then "
            "storing the fully-open angle as the travel span (Cmd::EncoderMaxCal), which is "
            "what the firmware divides by. Until it is stored the firmware cannot report a "
            "normalised position at all.\n"
            "\n"
            "If this unit's firmware is older than V2.1 it does not implement EncoderMaxCal "
            "and needs an OTA update first (examples/ota_update.py)."
        ) from e

    logger.info(f"  {label}Jaw normalised by the firmware's encoder-max calibration")
    return gripper, "firmware"


# ------------------------------------------------------- wrist fisheye undistort

#: The wrist lens is calibrated at this size and the firmware record carries the
#: 8 intrinsic/distortion floats and **no image size**, so serving another
#: resolution would mean guessing a scale factor and rectifying wrongly without
#: a trace. ``FisheyeUndistorter`` rejects anything else; the configs check it at
#: CLI-parse time so it does not wait until ``connect()`` to say so.
WRIST_CALIB_SIZE = (640, 480)


def validate_wrist_undistort_size(width: int, height: int) -> None:
    """Reject a wrist resolution the fisheye intrinsics do not describe.

    Called from both configs' ``__post_init__`` so the failure lands at
    CLI-parse time. ``FisheyeUndistorter`` would raise at ``connect()`` anyway —
    after the grippers, tracker and every other camera are already open — and the
    error there does not name the flag that caused it.
    """
    if (width, height) != WRIST_CALIB_SIZE:
        w, h = WRIST_CALIB_SIZE
        raise ValueError(
            f"wrist_undistort=True requires the wrist camera at {w}x{h}, got {width}x{height}. "
            f"The firmware's fisheye record holds only the 8 intrinsic/distortion floats and no "
            f"image size, so another resolution would mean guessing a scale factor and rectifying "
            f"wrongly with nothing in the frames to show it. Either drop --robot.wrist_undistort "
            f"or record at {w}x{h}."
        )


def resolve_wrist_undistorter(
    gripper: Any,
    *,
    undistorter_cls: Any,
    fallback_cal: Any,
    balance: float,
    logger: Any,
    label: str = "",
) -> tuple[Any, str]:
    """Build the undistorter for one wrist camera, and say where its numbers came from.

    We own the wrist UVC device (the cameras come from the LeRobot camera
    framework, and ``open_gripper`` leaves the SDK's ``open_cameras`` at its
    default ``False``), so the SDK's own ``Config::undistort_wrist`` never
    applies to us. ``Calibration::resolve_fisheye`` exists precisely for this
    case: it hands callers that own the device the *identical* read-and-fall-back
    answer the SDK uses internally, instead of each consumer re-deriving it and
    drifting from the other.

    Falls back to the SDK's reference intrinsics rather than refusing, matching
    the SDK: every unit carries the same lens on the same sensor, so the shared
    numbers beat no rectification at all. It is **approximate** — lens placement
    varies per assembly so the principal point drifts — hence the warning, and
    hence ``hardware_manifest_unit`` records which of the two was used. A warning
    at connect scrolls past; the manifest is what can still be checked afterwards.

    ``gripper`` may be ``None`` (``enable_gripper=False``), which leaves no MCU to
    ask and therefore no way to read this unit's own calibration.

    ``undistorter_cls`` (``FisheyeUndistorter``) and ``fallback_cal``
    (``FISHEYE_FALLBACK_CAL``) are injected rather than imported here, for the
    same reason ``open_gripper`` takes ``gripper_cls``: this module is helper
    code the robots share, and importing the SDK in it would make every one of
    these helpers untestable on a machine without the SDK — which includes CI.

    Returns:
        ``(undistorter, source)`` where ``source`` is ``"unit"`` when the numbers
        came off this gripper's flash and ``"reference"`` when they did not.
    """
    if gripper is None:
        cal, is_reference, reason = (
            fallback_cal,
            True,
            "the gripper MCU is disabled (--robot.enable_gripper=false), so this unit's own calibration cannot be read",
        )
    else:
        cal, is_reference, reason = gripper.calibration.resolve_fisheye()

    if is_reference:
        logger.warn(
            f"  {label}Wrist undistortion is using the SDK's REFERENCE intrinsics because "
            f"{reason}. Rectification will be approximate — lens placement varies per "
            f"assembly, so the principal point drifts, and anything measuring in pixels off "
            f"these frames needs a real calibration. Store this unit's own with: "
            f"python third_party/taccap-gripper/python/examples/fisheye_cal.py set-fisheye"
        )

    width, height = WRIST_CALIB_SIZE
    undistorter = undistorter_cls(cal, width, height, balance)
    source = "reference" if is_reference else "unit"
    logger.info(
        f"  {label}Wrist fisheye undistortion on (balance={undistorter.balance:.2f}, "
        f"focal_scale={undistorter.focal_scale:.3f}, calibration={source})"
    )
    return undistorter, source


#: How long the jaw encoder may fail continuously before the gripper counts as
#: physically lost. Shorter than :data:`camera_health.CAM_FREEZE_TIMEOUT_S`
#: because an encoder read is one serial round-trip with nothing to buffer, and
#: every extra second of tolerance is another ~30 frames of stale jaw in the
#: dataset.
ENCODER_LOSS_TIMEOUT_S = 1.0


class GripperReadGuard:
    """Reads jaw encoders on behalf of a robot, degrading gracefully on loss.

    The counterpart of :class:`camera_health.CameraReadGuard`, and deliberately
    the same shape: hold the per-gripper state the detection needs, expose
    ``lost`` for the robot's ``device_lost``, and let the robot call
    :meth:`read` instead of reasoning about failures itself.

    Without it a gripper whose USB hub browned out mid-episode kept feeding
    ``0.0`` — a legal, unremarkable "closed" value — into both the observation
    and the action for the rest of the recording, while the record loop, which
    stops cleanly for a lost *camera*, had no idea anything was wrong.

    Args:
        logger: robot logger. Loss is reported once per gripper, not once per
            frame: the old per-frame warning interleaved with the observation
            table and made the terminal unreadable exactly when an operator
            most needed to see what was happening.
        timeout_s: see :data:`ENCODER_LOSS_TIMEOUT_S`.
    """

    def __init__(self, logger: Any, timeout_s: float = ENCODER_LOSS_TIMEOUT_S) -> None:
        self._logger = logger
        self._timeout_s = timeout_s
        self._last_good: dict[str, float] = {}
        self._failing_since: dict[str, float] = {}
        self._lost: set[str] = set()

    @property
    def lost(self) -> bool:
        """True once any gripper has been detected as physically lost."""
        return bool(self._lost)

    @property
    def lost_grippers(self) -> frozenset[str]:
        """Keys of the grippers detected as lost, for reporting."""
        return frozenset(self._lost)

    def reset(self) -> None:
        """Drop all per-gripper state. Call on ``connect()`` so a reconnect does
        not start out already flagged as lost by the previous session."""
        self._last_good.clear()
        self._failing_since.clear()
        self._lost.clear()

    def read(self, key: str, gripper: Any, open_rad: float, label: str = "") -> float:
        """Read one jaw encoder, holding the last good value across a blip.

        A single failed round-trip is not evidence of anything — the bus is
        shared and the device is polled twice per frame — so it degrades to the
        last good reading. Only continuous failure for ``timeout_s`` counts as
        loss, at which point ``lost`` trips and the record loop stops the
        episode instead of recording through it.

        The first read failing is different: there is no last good value to hold
        and nothing sensible to record, so that trips loss immediately.
        """
        try:
            value = read_gripper_normalized(gripper, open_rad)
        except Exception as e:
            return self._degrade(key, label, e)

        self._last_good[key] = value
        self._failing_since.pop(key, None)
        return value

    def _degrade(self, key: str, label: str, exc: Exception) -> float:
        now = time.monotonic()
        started = self._failing_since.setdefault(key, now)
        last_good = self._last_good.get(key)

        if last_good is None or now - started >= self._timeout_s:
            if key not in self._lost:
                self._lost.add(key)
                reason = "never read successfully" if last_good is None else f"failing for {now - started:.1f}s"
                self._logger.error(f"  {label}Gripper encoder lost ({reason}): {exc}")
        elif started == now:
            # First failure of this episode of trouble — say so once, then stay
            # quiet until it either recovers or is declared lost.
            self._logger.warn(f"  {label}Encoder read failed, holding the last value: {exc}")

        return 0.0 if last_good is None else last_good


def read_gripper_normalized(gripper: Any, open_rad: float) -> float:
    """Jaw opening in [0, 1] — 0 closed (the encoder zero), 1 fully open.

    Prefers the SDK's ``position``, which the firmware's own encoder-max
    calibration produces (see :func:`open_gripper`). It is ``nan`` when no
    converter is installed, which since :func:`open_gripper` started refusing
    uncalibrated leaders means a **follower** — so the radians path stays as
    the backstop rather than letting a nan reach the dataset.

    Raises whatever the bus raises. It used to answer ``0.0`` on any failure,
    which is a perfectly ordinary "jaw closed" reading — so a gripper that
    dropped off the bus mid-episode wrote a plausible lie into every remaining
    frame, as an observation *and* as an action, with nothing in the data to
    show for it afterwards. Deciding what a failed read means is
    :class:`GripperReadGuard`'s job; this function only converts.
    """
    sample = gripper.encoder.read_once()

    normalised = float(getattr(sample, "position", float("nan")))
    if np.isfinite(normalised):
        return float(np.clip(normalised, 0.0, 1.0))

    if open_rad <= 0.0:  # guarded in config.__post_init__ but be defensive
        return 0.0
    return float(np.clip(float(sample.position_rad) / open_rad, 0.0, 1.0))


# -------------------------------------------------------------------- robot id


def validate_robot_id(robot_id: str | None, robot_type: str) -> str:
    """Require ``--robot.id``, expand a bare number, and return it stripped.

    Upstream leaves ``RobotConfig.id`` optional, defaulting to ``None`` — which
    is how terminal output came to read ``None BiTaccapGripper`` and how a run
    could be recorded with nothing naming the rig it came from. Here it is the
    station label, and a capture rig that cannot say which station it is is not
    configured. Enforced in each config's ``__post_init__`` rather than by
    changing the base dataclass, which belongs to upstream.

    **A bare number is expanded against the robot type**, so ``--robot.id=0``
    stores ``taccap_0`` on a single rig and ``bi_taccap_0`` on a bimanual one.
    Typing the prefix was pure ceremony — it repeats what ``--robot.type``
    already said, and getting it wrong (``--robot.type=bi_taccap_gripper
    --robot.id=taccap_0``) produced a label that quietly disagreed with the
    rig. The ``_gripper`` suffix is dropped: the label names a station, and a
    station is not a gripper — the gripper is one of the parts you swap out of
    it.

    Anything that is not all digits is taken verbatim, which keeps every
    existing ``--robot.id=taccap_0`` working — and its calibration file, which
    is named after this value — and leaves room for a rig named after a room.
    The *identity* of the hardware is still the serials in
    ``meta/hardware.json``, not this string.
    """
    if robot_id is None or not robot_id.strip():
        raise ValueError(
            "--robot.id is required: the station label for this rig, e.g. "
            "--robot.id=0 (0 / 1 / …, one per rig — a bimanual rig is one rig). "
            "A bare number is expanded against --robot.type, so 0 becomes "
            f"{robot_type.removesuffix('_gripper')}_0; pass a full string to name a rig "
            "something else. It names the seat, not the hardware in it, so it stays put "
            "when a gripper is swapped; the device serials go into the recorded dataset's "
            "meta/hardware.json instead."
        )

    stripped = robot_id.strip()
    if stripped.isdigit():
        return f"{robot_type.removesuffix('_gripper')}_{stripped}"
    return stripped


# ------------------------------------------------------------ hardware manifest

HARDWARE_MANIFEST_PATH = TACCAP_HARDWARE_MANIFEST_PATH
"""Where a recorded dataset keeps its TacCap hardware manifest, relative to the
dataset root.

Deliberately a file of its own rather than a key in ``meta/info.json``: that
file's schema belongs to upstream lerobot, and a fork-local key in it would
collide on the next v5.x sync. ``robot.id`` does not reach the dataset at all
(``LeRobotDataset.create`` takes only ``robot_type``), so this manifest is the
only thing tying recorded episodes to the physical devices that produced them.

The same reasoning is why a swap mid-dataset is recorded here as another
``epochs`` entry rather than as a per-episode column in
``meta/episodes/*.parquet``: adding a column part-way through makes the dataset
unreadable. ``datasets.Dataset.from_parquet`` casts every later file to the
*first* file's schema, so a dataset whose early files predate the column raises
``CastError`` on load — not a missing column, the whole dataset. (The bare
``pyarrow.dataset`` path is worse: it drops the column silently.)
"""


def hardware_manifest_unit(
    side: str,
    *,
    endpoints: Any,
    tactile_serials: dict[str, str],
    key_prefix: str,
    wrist_undistort: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """One TacCap unit's identity: its gripper, and the tactiles on that gripper.

    ``endpoints`` is the unit's ``GripperEndpoints`` (``None`` when the gripper is
    off, or when the robot is not connected — the SN is read over the wire). The
    serial recorded is the **firmware** SN (``Cmd::GetSn``), never the CH343
    ``mcu_serial``: the latter identifies the USB-serial adapter and changes when
    the adapter does, so it is not the device's identity.

    ``side`` is which gripper the unit is; ``finger`` is which sensor on it. Both
    are called left/right and they are independent — 单左双右 is applied once to
    the gripper's own serial and again to each tactile's — so each entry also
    carries the ``observation_key`` it feeds (``{key_prefix}tactile_{finger}``),
    letting a dataset column trace back to a physical sensor without re-deriving
    the naming rule.

    ``wrist_undistort`` records whether this unit's wrist frames were rectified
    and from whose intrinsics — ``{"applied": bool, "calibration": "unit" |
    "reference", "balance": float}``, or ``None`` when there is no wrist camera,
    in which case the key is left out entirely.

    That may look like it contradicts leaving trackers and wrist cameras out of
    the manifest, but it does not: what is excluded is an accessory's *identity*,
    and this records how the recorded frames were *processed*. A rectified
    ``wrist_cam`` and a raw fisheye one have the same shape and dtype, so without
    this nothing downstream — or later — can tell which it is holding. It rides
    in ``units``, so changing it part-way through a dataset opens a new epoch on
    its own (see ``write_hardware_manifest``).
    """
    unit: dict[str, Any] = {
        "side": side,
        "gripper_sn": getattr(endpoints, "firmware_sn", None) or None,
        "tactile_sensors": [
            {
                "finger": finger,
                "observation_key": f"{key_prefix}tactile_{finger}",
                "serial": sn,
            }
            for finger, sn in sorted(tactile_serials.items())
        ],
    }
    if wrist_undistort is not None:
        unit["wrist_undistort"] = wrist_undistort
    return unit


def wrist_undistort_record(*, has_wrist_camera: bool, source: str | None, balance: float) -> dict[str, Any] | None:
    """The ``wrist_undistort`` entry for one unit's manifest, or ``None`` to omit it.

    ``None`` — and therefore no key at all — means "this unit has no wrist
    camera", so there is nothing for the question to be about. A unit that *has*
    one always records an answer, including ``{"applied": False}``: absent and
    false would otherwise be indistinguishable, and "we did not rectify" is a
    fact about the data worth stating rather than inferring from a missing key.

    ``balance`` is left out when nothing was applied — it describes a
    rectification that did not happen.
    """
    if not has_wrist_camera:
        return None
    if source is None:
        return {"applied": False}
    return {"applied": True, "calibration": source, "balance": float(balance)}


def build_hardware_manifest(
    *,
    robot_type: str,
    robot_id: str | None,
    role: str,
    units: list[dict[str, Any]],
) -> dict[str, Any]:
    """Wrap per-unit entries into the manifest both TacCap robots emit.

    ``robot_id`` is the station label (``--robot.id``, e.g. ``taccap_0``) and is
    free to be ``None``; the serials below are the actual identity, and they are
    what a dataset should be trusted to be traced by.

    ``units`` describes what is connected *now*. Which episodes it produced is
    decided at write time (``write_hardware_manifest``), because only the dataset
    knows how many episodes precede this run.
    """
    return {
        "robot_type": robot_type,
        "robot_id": robot_id,
        "role": role,
        "units": units,
    }


def manifest_epochs(manifest: dict[str, Any]) -> list[dict[str, Any]]:
    """The manifest's hardware epochs, oldest first, for old and new files alike.

    A manifest written before epochs existed has a bare ``units`` and no episode
    bounds. It is reported as a single open-ended epoch (``from_episode`` 0,
    ``to_episode`` ``None``) rather than being rejected: those datasets are real
    and mostly single-rig.

    ``to_episode`` is exclusive, matching ``dataset_from_index`` /
    ``dataset_to_index``. It is ``None`` on the epoch still being recorded — the
    count is only known once the run ends, and guessing would be worse than
    saying "still open".

    .. warning::
       An open-ended single epoch means *"nothing here says the rig changed"*,
       **not** *"the rig did not change"*. Pre-epoch manifests could not express
       a swap, so a consumer that must not mix rigs (per-sensor tactile solving,
       for one) has to verify it some other way.
    """
    epochs = manifest.get("epochs")
    if isinstance(epochs, list):
        return epochs
    # Pre-epoch file: the station labels sit at the top level, so lift them in
    # to give callers one shape to read regardless of when the file was written.
    return [
        {
            "from_episode": 0,
            "to_episode": None,
            "robot_id": manifest.get("robot_id"),
            "role": manifest.get("role"),
            "units": manifest.get("units", []),
        }
    ]


def epoch_for_episode(manifest: dict[str, Any], episode_index: int) -> dict[str, Any] | None:
    """The epoch that recorded ``episode_index``, or ``None`` if none claims it.

    ``None`` is a real answer, not an error: an episode past the last closed
    epoch belongs to hardware nobody recorded. Callers that would otherwise
    attribute it to the wrong rig need to see that.
    """
    for epoch in manifest_epochs(manifest):
        start = epoch.get("from_episode") or 0
        end = epoch.get("to_episode")
        if episode_index >= start and (end is None or episode_index < end):
            return epoch
    return None


RUNTIME_DIR = TACCAP_RUNTIME_DIR
"""Where a dataset keeps the tactile runtime bundles its episodes need.

Rebuilding depth / force / difference from the recorded ``rectify`` stream needs
the recording sensor's own runtime config. Keeping it *in* the dataset is what
makes that reconstruction possible from the dataset alone — no hunting down the
physical unit months later, and no risk that the unit has since been recalibrated
into a different reference.

Files are named ``<serial>-<local timestamp>.bin``, because a serial alone is not
unique over time: a sensor pulled for maintenance and reinstalled keeps its
serial and comes back with a new reference image, so ``<serial>.bin`` would have
the later run overwrite the earlier one — silently re-deriving those episodes
against a calibration they were never recorded with.

The timestamp rather than a content hash: every export is a fresh AES-GCM
encryption with a random salt and IV, so two exports of the *same* sensor state
have different bytes. A digest would therefore never collapse duplicates while
also saying nothing a reader can use. The time is what the file actually is —
this unit's calibration as of that moment.

The name carries wall-clock time in ``CAPTURE_TZ`` (UTC+08:00, where the rigs
run) and no offset marker, so it reads as the hour someone was standing at the
bench. That makes it a label, not a timestamp to compute with — the authoritative
value is ``recorded_at`` on the epoch, which is full ISO-8601 with offset. Two
readers, two jobs; the ambiguous-looking one is the one nothing parses.

One bundle per recording session is the right granularity, not an accident of
naming: the reference image is captured fresh at every ``Sensor.create()``, so
each session genuinely has its own. Measured on an untouched sensor, reusing the
previous session's bundle moves ``Depth`` by 2.9e-4 and ``ForceResultant`` by
7.3e-3 — the noise floor, and ~1000x smaller than the error from using another
*unit's* bundle. Small, but there is no reason to accept it when the correct
bundle costs 841 KB.
"""

#: Where the rigs are. Fixed offset rather than a named zone: China has observed
#: no DST since 1991, so there is no ambiguous hour for a bare local time to land
#: in, and a fixed offset needs no tz database to reproduce.
CAPTURE_TZ = timezone(timedelta(hours=8), "UTC+08:00")


def _runtime_filename(serial: str, taken_at: datetime) -> str:
    return f"{serial}-{taken_at.astimezone(CAPTURE_TZ).strftime('%Y%m%dT%H%M%S')}.bin"


def write_tactile_runtimes(
    root: str | Path, runtimes: dict[str, bytes], taken_at: datetime | None = None
) -> dict[str, str]:
    """Write each sensor's runtime bundle and return ``{serial: relative path}``.

    Every bundle from one call shares ``taken_at``, so a rig's sensors are named
    for the session rather than for whichever microsecond each export finished
    in. Paths are relative to the dataset root, which is what goes in the
    manifest.
    """
    taken_at = taken_at or datetime.now(CAPTURE_TZ)
    written: dict[str, str] = {}
    for serial, blob in sorted(runtimes.items()):
        if not blob:
            continue
        relative = f"{RUNTIME_DIR}/{_runtime_filename(serial, taken_at)}"
        path = Path(root) / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        # Two exports in the same second would otherwise land on one name and the
        # second would win, leaving an epoch pointing at a bundle that is not its
        # own. Rare, but the failure is silent, so it gets a suffix instead.
        suffix = 1
        while path.exists() and path.read_bytes() != blob:
            relative = f"{RUNTIME_DIR}/{_runtime_filename(serial, taken_at)[:-4]}-{suffix}.bin"
            path = Path(root) / relative
            suffix += 1
        if not path.exists():
            path.write_bytes(blob)
        written[serial] = relative
    return written


def _stamp_runtimes(units: list[dict[str, Any]], paths: dict[str, str]) -> list[dict[str, Any]]:
    """Copy ``units`` with each tactile sensor pointing at its runtime bundle.

    A sensor with no bundle is left without the key rather than given a null:
    absent means "not recorded", and a null would read like "recorded as
    nothing". Consumers check for the key.
    """
    stamped = []
    for unit in units:
        sensors = []
        for sensor in unit.get("tactile_sensors") or []:
            relative = paths.get(sensor.get("serial"))
            sensors.append({**sensor, "runtime": relative} if relative else dict(sensor))
        stamped.append({**unit, "tactile_sensors": sensors})
    return stamped


def _epoch_payload(manifest: dict[str, Any]) -> dict[str, Any]:
    """What a run records about the rig: the devices, and where they were run.

    ``units`` is the identity that matters downstream — which physical sensor fed
    which observation key. ``robot_id`` / ``role`` ride along as context: the
    label names a *station*, and the devices are the parts you swap in and out of
    it (see ``validate_robot_id``). So the same gripper moved to another PC is a
    change worth recording, but it is not a change of hardware, and a consumer
    picking a per-sensor calibration keys on ``units`` alone.
    """
    return {
        "robot_id": manifest.get("robot_id"),
        "role": manifest.get("role"),
        "units": manifest.get("units", []),
    }


def tactile_serial_for_key(manifest: dict[str, Any], episode_index: int, observation_key: str) -> str | None:
    """Which physical sensor fed ``observation_key`` in ``episode_index``.

    The one lookup every consumer of this file actually wants, so it lives here
    rather than being re-derived — getting it wrong is not visible in the output.
    Deriving the multimodal tactile channels (depth, force, difference) needs the
    sensor's own runtime config, and solving a stream against another unit's
    calibration does not fail loudly: it reports plausible depth and force from
    an untouched gel.

    ``None`` means the manifest cannot answer — no epoch covers that episode, or
    no sensor in it feeds that key. Callers must treat that as "do not derive",
    never as "use whichever sensor is nearest".
    """
    epoch = epoch_for_episode(manifest, episode_index)
    if epoch is None:
        return None
    for unit in epoch.get("units") or []:
        for sensor in unit.get("tactile_sensors") or []:
            if sensor.get("observation_key") == observation_key:
                return sensor.get("serial")
    return None


def tactile_runtime_for_key(manifest: dict[str, Any], episode_index: int, observation_key: str) -> str | None:
    """Path (dataset-relative) of the runtime bundle needed to rebuild this
    stream's derived channels, or ``None`` if the dataset does not carry one.

    ``None`` covers both "no epoch claims that episode" and "recorded before
    bundles were stored". Either way the honest move is to skip derivation for
    those episodes rather than reach for another bundle: a bundle from a
    different unit — or from the same unit after a recalibration — turns an
    untouched gel into plausible depth and force, with nothing in the output
    saying so.
    """
    epoch = epoch_for_episode(manifest, episode_index)
    if epoch is None:
        return None
    for unit in epoch.get("units") or []:
        for sensor in unit.get("tactile_sensors") or []:
            if sensor.get("observation_key") == observation_key:
                return sensor.get("runtime")
    return None


def _describe_change(previous: dict[str, Any] | None, current: dict[str, Any]) -> str:
    """Name what actually changed, so the log points at the thing to check.

    The two grippers are swapped independently — a customer may replace only the
    left one — so "hardware changed" alone would leave someone diffing the whole
    manifest to find out which arm to look at.
    """
    if previous is None:
        return "Hardware recorded"

    def by_side(units: Any) -> dict[str, Any]:
        return {unit.get("side"): unit for unit in units or []}

    def serials(unit: Any) -> tuple:
        return (
            unit.get("gripper_sn"),
            tuple(s.get("serial") for s in unit.get("tactile_sensors") or []),
        )

    before, after = by_side(previous.get("units")), by_side(current["units"])
    swapped = sorted(
        side
        for side in before.keys() | after.keys()
        if serials(before.get(side) or {}) != serials(after.get(side) or {})
    )
    if swapped:
        return f"Hardware changed on the {' and '.join(swapped)} unit(s)"
    if previous.get("robot_id") != current.get("robot_id"):
        return f"Station changed ({previous.get('robot_id')} -> {current.get('robot_id')})"
    # Same serials, same station: what differs is the runtime bundle, which the
    # sensor re-captures at every ``Sensor.create()``. Calling that "hardware
    # changed" would send someone hunting for a swap that never happened.
    return "New capture session on the same hardware"


def write_hardware_manifest(
    root: str | Path,
    manifest: dict[str, Any],
    logger: Any,
    *,
    episode_index: int = 0,
    runtimes: dict[str, bytes] | None = None,
) -> Path:
    """Write ``manifest`` to ``root/meta/hardware.json`` and return the path.

    ``episode_index`` is how many episodes the dataset already holds, i.e. the
    first index this run will write. It is what closes the open epoch and opens
    the next one, so a caller that resumes must pass it
    (``dataset.num_episodes``); the default only suits a fresh dataset.

    Three cases:

    * **New dataset** — one open epoch starting at ``episode_index``.
    * **Resumed on the same rig** — nothing to record; the open epoch already
      covers what is about to be written.
    * **Resumed on a different rig** — close the open epoch at ``episode_index``
      and append a new one. Both stay named, and every episode keeps pointing at
      the devices that actually produced it.

    "Different" means anything in ``units`` / ``robot_id`` / ``role`` changed.
    Note that this includes running the *same* devices from another PC: the
    station label changes while the hardware does not. That is recorded rather
    than refused, because refusing would also drop a device swap that happened
    to coincide with the move — and the devices are the part downstream cannot
    guess. Consumers key on ``units``; an epoch boundary with identical ``units``
    is simply the rig moving.

    Only ``robot_type`` is treated as a hard mismatch: single vs bimanual changes
    the observation keys, so it is not the same dataset at all, and epochs do not
    model that.

    The swap case used to keep the original file and warn. The warning went to
    the log and never reached the dataset, so afterwards nothing on disk said the
    rig had changed, while the manifest quietly misattributed every episode
    recorded after the swap. Downstream that is worse than a loud failure:
    solving a tactile stream against another sensor's calibration yields
    plausible depth and force from an untouched gel — no error, no signal.

    ``runtimes`` maps a tactile serial to its runtime bundle
    (``XenseCamera.export_runtime_config()``). Given one, each bundle is written
    under ``meta/runtimes/`` and the epoch's sensors point at it, so the derived
    tactile channels can be rebuilt from the dataset alone. A sensor that comes
    back from maintenance with a new reference image produces a different bundle
    under the same serial, which counts as a change and opens its own epoch.

    A truncated or unreadable file is still left alone: whatever episodes are
    already there have provenance we cannot reconstruct, and clobbering it would
    destroy the only record of it.
    """
    path = Path(root) / HARDWARE_MANIFEST_PATH
    # Deliberately not part of ``payload``: the comparison below asks "is this the
    # same rig", and a clock reading is different every run. It rides on the epoch
    # once one is actually opened.
    recorded_at = datetime.now(CAPTURE_TZ)
    payload = _epoch_payload(manifest)
    if runtimes:
        # Written before the comparison below, so a re-calibrated sensor — same
        # serial, new reference image, hence a new bundle path — is seen as a
        # change and opens its own epoch. That is the point: the old episodes
        # must keep pointing at the bundle that was current when they were
        # recorded.
        payload["units"] = _stamp_runtimes(payload["units"], write_tactile_runtimes(root, runtimes, recorded_at))

    if not path.exists():
        path.parent.mkdir(parents=True, exist_ok=True)
        fresh = {
            "robot_type": manifest.get("robot_type"),
            "epochs": [
                {
                    "from_episode": int(episode_index),
                    "to_episode": None,
                    "recorded_at": recorded_at.isoformat(timespec="seconds"),
                    **payload,
                }
            ],
        }
        path.write_text(json.dumps(fresh, indent=2) + "\n")
        logger.info(f"Hardware manifest written to {path}")
        return path

    try:
        existing = json.loads(path.read_text())
    except (OSError, ValueError) as e:
        logger.warn(f"Could not read the existing hardware manifest {path} ({e}); leaving it alone.")
        return path

    if existing.get("robot_type") != manifest.get("robot_type"):
        logger.warn(
            f"Hardware manifest {path} was recorded on {existing.get('robot_type')!r} but this run "
            f"is {manifest.get('robot_type')!r}; keeping the original. A different robot type means "
            f"different observation keys, which is not a hardware swap."
        )
        return path

    epochs = manifest_epochs(existing)
    last = epochs[-1] if epochs else None
    if last is not None and all(last.get(key) == value for key, value in payload.items()):
        return path  # same rig; the open epoch already covers this run

    updated = [dict(epoch) for epoch in epochs]
    if updated:
        updated[-1]["to_episode"] = int(episode_index)
    updated.append(
        {
            "from_episode": int(episode_index),
            "to_episode": None,
            "recorded_at": recorded_at.isoformat(timespec="seconds"),
            **payload,
        }
    )

    path.write_text(json.dumps({"robot_type": existing.get("robot_type"), "epochs": updated}, indent=2) + "\n")
    logger.info(
        f"{_describe_change(last, payload)} at episode {episode_index}; recorded as a new epoch "
        f"in {path}. This dataset now spans {len(updated)} configurations."
    )
    return path
