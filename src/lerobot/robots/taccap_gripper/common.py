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
import os
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from typing import Any

import numpy as np

from lerobot.cameras.xense.configuration_xense import XenseTactileCameraConfig

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
    "HeadSkewMonitor",
    "build_head_camera_configs",
    "build_tactile_camera_configs",
    "connect_cameras_parallel",
    "disconnect_cameras_parallel",
    "open_gripper",
    "prewarm_tactile_config_cache",
    "read_gripper_normalized",
    "read_head_camera_skew",
    "read_head_pose",
    "resolve_wrist_camera_path",
    "split_camera_read",
    "swap_tactile_display_features",
    "tactile_camera_output_types",
    "tactile_display_key",
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
    overlaps in time instead of summing (cf. v0.4.4 bi_arx5)."""
    if not cameras:
        return
    n = len(cameras)
    logger.info(f"  Connecting {n} camera(s) in parallel...")
    with ThreadPoolExecutor(max_workers=min(n, 8)) as executor:
        futures = {executor.submit(cam.connect): name for name, cam in cameras.items()}
        for fut in as_completed(futures):
            name = futures[fut]
            try:
                fut.result()
            except Exception as e:
                logger.error(f"  Camera '{name}' connect failed: {e}")
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


def read_gripper_normalized(gripper: Any, open_rad: float, logger: Any, label: str = "") -> float:
    """Jaw opening in [0, 1] — 0 closed (the encoder zero), 1 fully open.

    Prefers the SDK's ``position``, which the firmware's own encoder-max
    calibration produces (see :func:`open_gripper`). It is ``nan`` when no
    converter is installed, which since :func:`open_gripper` started refusing
    uncalibrated leaders means a **follower** — so the radians path stays as
    the backstop rather than letting a nan reach the dataset.
    """
    try:
        sample = gripper.encoder.read_once()
    except Exception as e:
        logger.warn(f"  {label}Encoder read failed: {e}")
        return 0.0

    normalised = float(getattr(sample, "position", float("nan")))
    if np.isfinite(normalised):
        return float(np.clip(normalised, 0.0, 1.0))

    if open_rad <= 0.0:  # guarded in config.__post_init__ but be defensive
        return 0.0
    return float(np.clip(float(sample.position_rad) / open_rad, 0.0, 1.0))
