#!/usr/bin/env python

# Copyright 2026 The XenseRobotics Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Head camera fed by the Pico headset through the XenseVR PC Service.

The headset encodes each eye as JPEG and sends it as a ``0x30`` custom
message; the service relays the bytes untouched and the pybind layer keeps
only the newest frame per eye. Nothing upstream pairs the eyes — they arrive
as two independent streams, each with its own ``frame_sequence`` and
``timestamp_ns`` — so pairing is this adapter's job. See
``doc/pico_camera_integration.md`` for the payload layout.
"""

from __future__ import annotations

import time
from typing import Any

import numpy as np
from numpy.typing import NDArray

from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.utils.robot_utils import get_logger

from ..camera import Camera
from .configuration_pico import PicoCameraConfig

_EYE_INDEX = {"left": 0, "right": 1}


class PicoCamera(Camera):
    """One head camera view assembled from the headset's per-eye JPEG streams.

    With ``eyes="both"`` the two eyes are decoded and concatenated
    horizontally into a single ``height x (2 * width)`` RGB image, the same
    shape convention the ZED stereo path uses (one image, one video key,
    per-eye width doubled on merge). With ``eyes="left"`` or ``"right"`` only
    that eye is decoded.

    The SDK connection is shared with the Pico tracker through
    :mod:`lerobot.teleoperators.pico4.xrt_session`; disconnecting this camera
    will not close a connection the tracker is still using.
    """

    def __init__(self, config: PicoCameraConfig):
        super().__init__(config)
        self.config = config
        self._is_connected = False
        self._xrt: Any = None
        self.logger = get_logger("PicoCamera")

        self._eyes: tuple[str, ...] = ("left", "right") if config.eyes == "both" else (config.eyes,)
        self._last_rgb: NDArray[np.uint8] | None = None
        self._last_host_ns: int | None = None
        self._last_seq: dict[str, int | None] = dict.fromkeys(self._eyes)
        # Reusing the previous frame is normal on a stereo mismatch, but it is
        # also how a genuinely broken stream looks, so it is counted and
        # surfaced periodically rather than passed over in silence.
        self._unpaired_count = 0
        self._decode_fail_count = 0
        self._last_warn_ns = 0

    def __str__(self) -> str:
        return f"PicoCamera(head_rgb, eyes={self.config.eyes})"

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @staticmethod
    def find_cameras() -> list[dict[str, Any]]:
        """There is exactly one head camera, and it is not enumerable.

        The frames arrive over the service connection rather than as a device
        node, so there is nothing to scan; whether frames are flowing only
        becomes knowable after ``connect()``.
        """
        return [{"name": "Pico head camera", "type": "pico", "id": "head_rgb"}]

    def connect(self, warmup: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        from lerobot.teleoperators.pico4 import xrt_session

        self._xrt, did_init = xrt_session.acquire("the Pico head camera")
        try:
            missing = [
                name
                for name in ("has_pico_camera_frame", "get_pico_camera_frame_metadata", "get_pico_camera_frame_jpeg")
                if not hasattr(self._xrt, name)
            ]
            if missing:
                raise ImportError(
                    f"xensevr_pc_service_sdk is missing {', '.join(missing)} — the installed "
                    "pybind predates Pico camera support. Rebuild it from "
                    "src/lerobot/teleoperators/pico4/xensevr-pc-service-pybind/."
                )

            self._is_connected = True
            if warmup:
                self._wait_until_ready()
            self.logger.info(
                f"{self} connected ({self.config.width}x{self.config.height} per eye "
                f"-> {self.config.frame_width}x{self.config.height} recorded, "
                f"{'SDK initialized' if did_init else 'reusing SDK'})"
            )
        except Exception:
            self._is_connected = False
            xrt_session.release()
            self._xrt = None
            raise

    def _wait_until_ready(self) -> None:
        deadline = time.monotonic() + self.config.startup_timeout_s
        last_error = "no frames received"
        while time.monotonic() < deadline:
            try:
                self.read()
                return
            except (RuntimeError, TimeoutError, ValueError) as e:
                last_error = str(e)
            time.sleep(0.02)
        raise ConnectionError(
            f"No Pico camera frame within {self.config.startup_timeout_s:.1f}s: {last_error}. "
            "Check that the headset app is running and streaming, that 'Send' is ticked in "
            "XenseVR-Toolkit, and that the PC Service is up."
        )

    def _grab_eye(self, eye: str) -> tuple[dict[str, Any], bytes] | None:
        """Metadata + JPEG for one eye, or None if nothing new is cached."""
        idx = _EYE_INDEX[eye]
        if not self._xrt.has_pico_camera_frame(idx):
            return None
        meta = self._xrt.get_pico_camera_frame_metadata(idx)
        jpeg = bytes(self._xrt.get_pico_camera_frame_jpeg(idx))
        return meta, jpeg

    def _check_size(self, eye: str, meta: dict[str, Any]) -> None:
        w, h = int(meta["width"]), int(meta["height"])
        if (w, h) != (self.config.width, self.config.height):
            raise ValueError(
                f"Pico {eye} eye is sending {w}x{h} but this camera is configured for "
                f"{self.config.width}x{self.config.height}. Rescaling here would silently "
                "change the recorded field of view, so fix one side to match the other: "
                "either set --robot.head_camera_width/_height, or change the headset app."
            )

    def _decode(self, jpeg: bytes) -> NDArray[np.uint8] | None:
        import cv2

        arr = np.frombuffer(jpeg, dtype=np.uint8)
        bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if bgr is None:
            return None
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    def read(self) -> NDArray[np.uint8]:
        if not self.is_connected or self._xrt is None:
            raise DeviceNotConnectedError(f"{self} is not connected")

        grabbed: dict[str, tuple[dict[str, Any], bytes]] = {}
        for eye in self._eyes:
            got = self._grab_eye(eye)
            if got is not None:
                self._check_size(eye, got[0])
                grabbed[eye] = got

        if len(grabbed) != len(self._eyes):
            return self._reuse_or_fail("waiting for frames from both eyes")

        if len(self._eyes) == 2 and not self._eyes_paired(grabbed):
            self._unpaired_count += 1
            self._maybe_warn()
            return self._reuse_or_fail("left and right frames are not from the same capture")

        planes = []
        for eye in self._eyes:
            meta, jpeg = grabbed[eye]
            rgb = self._decode(jpeg)
            if rgb is None:
                self._decode_fail_count += 1
                self._maybe_warn()
                return self._reuse_or_fail(f"{eye} eye JPEG failed to decode")
            planes.append(rgb)
            self._last_seq[eye] = int(meta["frame_sequence"])

        self._last_rgb = planes[0] if len(planes) == 1 else np.hstack(planes)
        self._last_host_ns = time.time_ns()
        return self._last_rgb

    def _eyes_paired(self, grabbed: dict[str, tuple[dict[str, Any], bytes]]) -> bool:
        """Whether the two cached frames come from one capture.

        Same ``frame_sequence`` is the definitive answer when the headset
        stamps both eyes alike. It may not — the payload spec only calls the
        field an incrementing counter, without saying whether it is shared or
        per-eye — so a timestamp window is the fallback rather than the
        primary test.
        """
        left, right = grabbed["left"][0], grabbed["right"][0]
        if int(left["frame_sequence"]) == int(right["frame_sequence"]):
            return True
        skew_ms = abs(int(left["timestamp_ns"]) - int(right["timestamp_ns"])) / 1e6
        return skew_ms <= self.config.pair_max_skew_ms

    def _reuse_or_fail(self, why: str) -> NDArray[np.uint8]:
        if self._last_rgb is None:
            raise RuntimeError(f"no Pico camera frame available yet ({why})")
        return self._last_rgb

    def _maybe_warn(self) -> None:
        """Rate-limited so a persistently broken stream does not flood the log."""
        now = time.monotonic_ns()
        if now - self._last_warn_ns < 5_000_000_000:
            return
        self._last_warn_ns = now
        self.logger.warn(
            f"{self}: reusing previous frame — {self._unpaired_count} stereo mismatches, "
            f"{self._decode_fail_count} decode failures so far."
        )

    def async_read(self, timeout_ms: float = 200) -> NDArray[np.uint8]:
        # Latest-cache semantics, matching the other UMI-path cameras: the
        # recording loop sets the cadence and must never block on a camera.
        del timeout_ms
        return self.read()

    def read_latest(self, max_age_ms: int = 500) -> NDArray[np.uint8]:
        rgb = self.read()
        if self._last_host_ns is None:
            raise RuntimeError("no Pico camera frame received yet")
        age_ms = (time.time_ns() - self._last_host_ns) / 1e6
        if age_ms > max_age_ms:
            raise TimeoutError(f"Pico camera cache is {age_ms:.1f}ms old (limit={max_age_ms}ms).")
        return rgb

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")

        from lerobot.teleoperators.pico4 import xrt_session

        self._is_connected = False
        self._xrt = None
        closed = xrt_session.release()
        self.logger.info(
            f"{self} disconnected ({'XenseVR SDK closed' if closed else 'SDK left open for other subscribers'})."
        )
