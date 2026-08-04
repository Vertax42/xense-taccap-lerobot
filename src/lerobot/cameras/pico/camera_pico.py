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
only the newest frame per eye. See ``doc/pico_camera_integration.md`` for the
payload layout.

Frames are collected by a background thread and read from its cache, the same
shape as the OpenCV and RealSense cameras. Here it also buys stereo alignment:
the eyes arrive as separate messages, so a reader sampling them directly can
catch one updated and not the other. :mod:`.stereo_poller` explains that.
"""

from __future__ import annotations

import threading
import time
from typing import Any

import numpy as np
from numpy.typing import NDArray

from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.utils.robot_utils import get_logger

from ..camera import Camera
from .configuration_pico import PicoCameraConfig
from .stereo_poller import StereoPoller

# One poller serves every PicoCamera in the process. Two single-eye cameras
# recording separate keys have to agree on which capture they are showing, and
# that can only be decided somewhere that sees both.
_poller_lock = threading.Lock()
_poller: StereoPoller | None = None
_poller_users: int = 0


def _acquire_poller(xrt: Any, expect_size: tuple[int, int]) -> StereoPoller:
    """Start (or join) the shared poller. Pair with :func:`_release_poller`.

    It always watches both eyes, whatever any one camera records. Widening the
    set later would mean replacing the poller, and a camera that already holds
    a reference would go on reading the replaced one — which is exactly the
    stall that showed up the first time this was written that way: the first
    camera froze on the last frame before the swap while the second advanced,
    so every pair mismatched.
    """
    global _poller, _poller_users
    with _poller_lock:
        if _poller is None:
            _poller = StereoPoller(xrt, ("left", "right"), expect_size)
            _poller.start()
        _poller_users += 1
        return _poller


def _release_poller() -> None:
    global _poller, _poller_users
    with _poller_lock:
        _poller_users = max(0, _poller_users - 1)
        if _poller_users == 0 and _poller is not None:
            _poller.stop()
            _poller = None


class PicoCamera(Camera):
    """One eye of the Pico headset camera, or both merged side by side.

    ``eyes="left"`` / ``"right"`` gives that eye at ``height x width``;
    ``eyes="both"`` concatenates them horizontally into
    ``height x (2 * width)``, the convention the ZED stereo path uses. Either
    way the frames come from the shared poller, so two single-eye cameras
    recording separate keys always show the same capture.

    The SDK connection is shared with the Pico tracker through
    :mod:`lerobot.teleoperators.pico4.xrt_session`; disconnecting this camera
    will not close a connection the tracker is still using.
    """

    def __init__(self, config: PicoCameraConfig):
        super().__init__(config)
        self.config = config
        self._is_connected = False
        self._xrt: Any = None
        self._poller: StereoPoller | None = None
        self.logger = get_logger("PicoCamera")

        self._eyes: tuple[str, ...] = ("left", "right") if config.eyes == "both" else (config.eyes,)
        self._last_rgb: NDArray[np.uint8] | None = None
        self._last_host_ns: int | None = None
        self._last_meta: dict[str, dict[str, Any]] = {}

    def __str__(self) -> str:
        return f"PicoCamera(eyes={self.config.eyes})"

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
        return [{"name": "Pico head camera", "type": "pico", "id": "pico_headcam"}]

    def connect(self, warmup: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        from lerobot.teleoperators.pico4 import xrt_session

        self._xrt, did_init = xrt_session.acquire("the Pico head camera")
        try:
            missing = [
                name
                for name in (
                    "has_pico_camera_frame",
                    "get_pico_camera_frame_metadata",
                    "get_pico_camera_frame_jpeg",
                )
                if not hasattr(self._xrt, name)
            ]
            if missing:
                raise ImportError(
                    f"xensevr_pc_service_sdk is missing {', '.join(missing)} — the installed "
                    "pybind predates Pico camera support. Rebuild it from "
                    "src/lerobot/teleoperators/pico4/xensevr-pc-service-pybind/."
                )

            self._poller = _acquire_poller(self._xrt, (self.config.width, self.config.height))
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
            if self._poller is not None:
                _release_poller()
                self._poller = None
            xrt_session.release()
            self._xrt = None
            raise

    def _wait_until_ready(self) -> None:
        deadline = time.monotonic() + self.config.startup_timeout_s
        while time.monotonic() < deadline:
            assert self._poller is not None
            self._poller.raise_if_size_mismatch()
            if self._poller.wait_for_frame(0.1) and all(self._poller.latest(eye) is not None for eye in self._eyes):
                return
        raise ConnectionError(
            f"No Pico camera frame within {self.config.startup_timeout_s:.1f}s. "
            "Check that the headset app is running and streaming, that 'Send' is ticked in "
            "XenseVR-Toolkit, and that the PC Service is up."
        )

    def read(self) -> NDArray[np.uint8]:
        if not self.is_connected or self._poller is None:
            raise DeviceNotConnectedError(f"{self} is not connected")

        self._poller.raise_if_size_mismatch()

        planes = []
        for eye in self._eyes:
            got = self._poller.latest(eye)
            if got is None:
                if self._last_rgb is None:
                    raise RuntimeError(f"no Pico camera frame available yet ({eye} eye)")
                return self._last_rgb
            meta, rgb = got
            planes.append(rgb)
            self._last_meta[eye] = meta

        self._last_rgb = planes[0] if len(planes) == 1 else np.hstack(planes)
        self._last_host_ns = time.time_ns()
        return self._last_rgb

    def async_read(self, timeout_ms: float = 200) -> NDArray[np.uint8]:
        # Latest-cache semantics, matching the other UMI-path cameras: the
        # recording loop sets the cadence and must never block on a camera.
        # The poller is already running ahead of it, so there is nothing to
        # wait for.
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

    def last_frame_meta(self, eye: str | None = None) -> dict[str, Any] | None:
        """Metadata behind this camera's last successful read.

        ``eye`` defaults to the only eye when this camera reads one. Returns
        None before the first successful read.
        """
        if eye is None:
            if len(self._eyes) != 1:
                raise ValueError(f"{self} reads {len(self._eyes)} eyes; name which one.")
            eye = self._eyes[0]
        return self._last_meta.get(eye)

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")

        from lerobot.teleoperators.pico4 import xrt_session

        self._is_connected = False
        if self._poller is not None:
            _release_poller()
            self._poller = None
        self._xrt = None
        closed = xrt_session.release()
        self.logger.info(
            f"{self} disconnected ({'XenseVR SDK closed' if closed else 'SDK left open for other subscribers'})."
        )
