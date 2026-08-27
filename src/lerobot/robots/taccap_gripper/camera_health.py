#!/usr/bin/env python

# Copyright 2026 The XenseRobotics Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Mid-episode camera-loss handling, shared by the single and bimanual TacCap
grippers.

A handheld capture rig gets unplugged: a USB hub browns out, a wrist camera's
cable is tugged, a tactile sensor drops off. Without this, both failures cost an
episode — one by crashing the record loop, the other by silently writing stale
frames. :class:`CameraReadGuard` turns both into "keep the last good frame, flag
the rig as lost", which lets ``lerobot_record`` stop cleanly and save what was
already captured (see its ``device_lost`` check).

Shared so the two robots cannot drift apart on it — they already did once, and
the single-gripper side was the one recording through an unplug.
"""

from __future__ import annotations

import time
from typing import Any

import numpy as np

from lerobot.utils.robot_utils import capture_recording_active

# Freeze timeout for graceful degradation. If a camera's ``async_read`` keeps
# returning the *same* frame object for this long, the stream is treated as
# physically lost. This covers Xense tactile sensors, whose background read
# thread survives a hot-unplug (it only stops on ``DeviceNotConnectedError``)
# and whose ``async_read`` restarts the thread and returns the last cached
# frame forever without raising — unlike the OpenCV wrist camera, which raises
# once its read thread dies. Keep this well above the slowest camera's frame
# interval so a sensor-slower-than-the-sample-loop rate mismatch never trips it.
CAM_FREEZE_TIMEOUT_S = 2.0


class CameraReadGuard:
    """Reads cameras on behalf of a robot, degrading gracefully on physical loss.

    Holds the per-camera state the detection needs (last good frame, when a
    genuinely new one last arrived, which cameras are gone), so a robot only has
    to own one of these and call :meth:`read` in place of ``cam.async_read()``.

    Args:
        camera_configs: the robot's ``{name: CameraConfig}``, used to shape a
            black frame when a camera dies before ever producing one.
        logger: robot logger; loss is reported once per camera.
        freeze_timeout_s: see :data:`CAM_FREEZE_TIMEOUT_S`.
    """

    def __init__(
        self,
        camera_configs: dict[str, Any],
        logger: Any,
        freeze_timeout_s: float = CAM_FREEZE_TIMEOUT_S,
    ) -> None:
        self._camera_configs = camera_configs
        self._logger = logger
        self._freeze_timeout_s = freeze_timeout_s
        # Last good frame per camera, so a hot-unplug degrades to a stale frame
        # instead of crashing the record loop. Value is a dict of frames for a
        # multi-output tactile sensor (recorded + display-only views), a single
        # array for every other camera.
        self._last_frame: dict[str, np.ndarray | dict[str, np.ndarray]] = {}
        # Monotonic time each camera last produced a genuinely new frame object,
        # used to detect a frozen (non-raising) stream — see :meth:`read`.
        self._last_new_frame_t: dict[str, float] = {}
        self._lost: set[str] = set()
        # Per-camera freshness tally for the episode being recorded, keyed
        # ``{cam: [reads, stale, gaps, longest_run, current_run]}``. Counted only
        # while a dataset is being written (`capture_recording_active`), since a
        # stale frame outside an episode is not recorded and costs nothing.
        self._freshness: dict[str, list[int]] = {}

    @property
    def lost(self) -> bool:
        """True once any camera has been detected as physically lost."""
        return bool(self._lost)

    @property
    def lost_cameras(self) -> frozenset[str]:
        """Names of the cameras detected as lost, for reporting."""
        return frozenset(self._lost)

    def reset(self) -> None:
        """Drop all per-camera state. Call on ``connect()`` so a reconnect does
        not start out already flagged as lost by the previous session."""
        self._last_frame.clear()
        self._last_new_frame_t.clear()
        self._lost.clear()
        self._freshness.clear()

    def read(self, cam_name: str, cam: Any) -> np.ndarray | dict[str, np.ndarray]:
        """Read one camera, degrading gracefully on physical loss.

        Two distinct loss modes are handled, because the camera classes behave
        differently on a hot-unplug:

        * **Read raises** — an OpenCV/UVC wrist camera's background thread dies
          after repeated failures and the next ``async_read`` raises
          ``RuntimeError`` ("read thread is not running"). Letting that propagate
          crashes the record loop and loses the in-progress episode.
        * **Read freezes** — a Xense tactile sensor keeps its background thread
          alive on error (it only stops on ``DeviceNotConnectedError``) and its
          ``async_read`` restarts the thread and returns the *same* cached frame
          object indefinitely, so an unplug never raises. We detect this by
          watching for a genuinely new frame object; if none arrives within
          ``freeze_timeout_s`` the stream is treated as lost. Without this,
          recording would silently continue writing stale tactile frames.

        In both cases we substitute the last good frame (or a black frame on
        first-read loss) and flag the camera as lost so :attr:`lost` trips,
        letting the caller stop cleanly and save.

        ``TimeoutError`` (a transient slow/dropped frame) reuses the last frame
        but is NOT flagged as lost — those recover on their own.
        """
        now = time.monotonic()
        try:
            frame = cam.async_read()
        except TimeoutError as e:
            self._logger.warn(f"  [{cam_name}] frame timeout, reusing last frame: {e}")
            return self._fallback_frame(cam_name)
        except Exception as e:
            self.flag_lost(cam_name, f"camera lost mid-episode: {e}")
            return self._fallback_frame(cam_name)

        # Freeze detection: a live stream yields a fresh array each frame; a
        # frozen one returns the identical cached object every call (the Xense
        # read thread stores one formatted result per successful read and
        # ``async_read`` hands it back by reference). Only reset the clock on a
        # genuinely new object, so a sensor slower than the sample loop (which
        # legitimately re-reads one frame) does not trip it.
        prev = self._last_frame.get(cam_name)
        fresh = prev is None or frame is not prev
        if fresh:
            self._last_new_frame_t[cam_name] = now
        else:
            frozen_for = now - self._last_new_frame_t.get(cam_name, now)
            if frozen_for > self._freeze_timeout_s:
                self.flag_lost(
                    cam_name,
                    f"no new frame for {frozen_for:.1f}s (stream frozen, sensor likely unplugged)",
                )
        self._note_freshness(cam_name, fresh)

        self._last_frame[cam_name] = frame
        return frame

    def _note_freshness(self, cam_name: str, fresh: bool) -> None:
        """Tally whether this read got a new frame or the previous one again.

        Identity, not pixels. A resting tactile gel barely changes between
        frames, so any content-based "is this a duplicate" test would flag the
        normal case; and the recorded stream is lossily encoded, which destroys
        bit-exactness in both directions. But a real capture allocates a new
        array (``_format_read_result`` returns ``np.ascontiguousarray`` of a
        reversed view), so ``frame is prev`` is exact and free — it is already
        computed above for freeze detection.

        Only counted while a dataset is being written: a stale frame served
        during encoder warm-up, the reset phase or the save/encode gap is not
        recorded and costs nothing.
        """
        if not capture_recording_active():
            return
        tally = self._freshness.setdefault(cam_name, [0, 0, 0, 0, 0])
        tally[0] += 1
        if fresh:
            tally[4] = 0
            return
        tally[1] += 1
        if tally[4] == 0:
            tally[2] += 1  # a new run of stale frames starts here
        tally[4] += 1
        tally[3] = max(tally[3], tally[4])

    def stale_frame_report(self) -> list[str]:
        """One line per camera that served a stale frame, then reset the tally.

        Two different problems show up in the same numbers, which is why the run
        length is reported and not just the percentage:

        * **a long run** is a capture stall — the background thread fell behind
          and ``async_read`` handed back the same frame N times in a row. See
          ``CaptureStallMonitor``, which reports the same event from the capture
          side; the two numbers should agree, and if they do not, something else
          is producing stale frames.
        * **many single-frame runs** are the steady-state beat between two
          unsynchronised 30 Hz loops — the sensor's background capture and the
          record loop each free-run at the same nominal rate, so the phase drifts
          and the loop occasionally samples twice before a new frame lands. This
          is tolerated by design (``CAM_FREEZE_TIMEOUT_S`` is set well above a
          frame interval precisely so it never trips), but nothing measured how
          often it happens until now.
        """
        lines = []
        for cam_name, (reads, stale, gaps, longest, _) in sorted(self._freshness.items()):
            if not stale or not reads:
                continue
            lines.append(
                f"  [{cam_name}] {stale}/{reads} frames served stale ({stale / reads * 100:.1f}%): "
                f"{gaps} gap(s), longest {longest} frame(s)"
            )
        self._freshness.clear()
        return lines

    def flag_lost(self, cam_name: str, reason: str) -> None:
        """Mark a camera as physically lost so :attr:`lost` trips. Idempotent;
        logs once per camera."""
        if cam_name not in self._lost:
            self._logger.error(f"  [{cam_name}] {reason}")
            self._lost.add(cam_name)

    def _fallback_frame(self, cam_name: str) -> np.ndarray | dict[str, np.ndarray]:
        """Last good frame for this camera, or a black frame of the declared
        (H, W, 3) shape if none was ever captured.

        The width is the camera's ``frame_width`` where it has one — the head
        camera's ``width`` is per-eye and a merged frame is twice that, which is
        also what the robots declare in ``observation_features``. Using ``width``
        here would hand back a half-width black frame on first-read loss.

        A tactile sensor asked for several output types reads as a dict, so the
        first-read fallback has to be shaped the same way — one black frame per
        requested type — or ``split_camera_read`` would drop the display keys.
        """
        cached = self._last_frame.get(cam_name)
        if cached is not None:
            return cached
        cfg = self._camera_configs[cam_name]
        black = np.zeros((cfg.height, getattr(cfg, "frame_width", cfg.width), 3), dtype=np.uint8)
        output_types = getattr(cfg, "output_types", None) or []
        frame: np.ndarray | dict[str, np.ndarray] = (
            {output_type.value: black.copy() for output_type in output_types} if len(output_types) > 1 else black
        )
        self._last_frame[cam_name] = frame
        return frame
