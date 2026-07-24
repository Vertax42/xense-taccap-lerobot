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

"""LeRobot camera adapter for the multimodal Insight9 head camera."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np
from numpy.typing import NDArray

from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.utils.robot_utils import get_logger, quaternion_to_rotation_6d

from ..camera import Camera
from .configuration_insight9 import Insight9CameraConfig


@dataclass(frozen=True)
class Insight9Snapshot:
    """Latest decoded RGB frame and raw-frame VIO pose from one SDK snapshot."""

    rgb: NDArray[np.uint8]
    vio_position: tuple[float, float, float]
    vio_rotation_6d: tuple[float, float, float, float, float, float]


class Insight9Camera(Camera):
    """Own the Insight9 SDK once and expose RGB plus raw VIO snapshots.

    The generic Camera methods return RGB only. ``read_snapshot_latest`` is the
    API used by ``BiTaccapGripper`` so RGB and VIO are sampled together without
    opening the native SDK more than once.
    """

    config_class = Insight9CameraConfig

    def __init__(self, config: Insight9CameraConfig):
        super().__init__(config)
        self.config = config
        self.logger = get_logger("Insight9Camera")

        self._sdk_camera: Any | None = None
        self._is_connected = False
        self._last_good_rgb: NDArray[np.uint8] | None = None
        self._last_good_color: Any | None = None
        self._last_vio: Any | None = None
        self._last_seen_color_frame_index: int | None = None
        self._last_stale_warning_t = 0.0
        self._last_decode_warning_t = 0.0

    def __str__(self) -> str:
        return "Insight9Camera(head_rgb)"

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @staticmethod
    def find_cameras() -> list[dict[str, Any]]:
        # The native SDK performs its own UVC/HID discovery during connect().
        # Returning an empty list keeps generic camera discovery side-effect free.
        return []

    def connect(self, warmup: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        try:
            from insight9_umi_camera import Insight9HeadCamera
        except ImportError as e:
            raise ImportError(
                "insight9-python-interface is required for the Insight9 head camera. "
                "Initialize third_party/insight9-python-interface and run setup_env.sh --install."
            ) from e

        self._reset_cache()
        self._sdk_camera = Insight9HeadCamera(
            library_path=self.config.library_path,
            enable_images=True,
            enable_imu=False,
            enable_vio=True,
        )
        try:
            self._sdk_camera.start()
            self._is_connected = True
            if warmup:
                self._wait_until_ready()
            self.logger.info(
                f"{self} connected via {self._sdk_camera.library_path} "
                f"({self.width}x{self.height} @ dataset {self.fps}fps)"
            )
        except Exception:
            self._close_sdk()
            raise

    def _wait_until_ready(self) -> None:
        deadline = time.monotonic() + self.config.startup_timeout_s
        last_error = "no samples received"
        while time.monotonic() < deadline:
            try:
                self.read_snapshot_latest()
                return
            except (RuntimeError, TimeoutError) as e:
                last_error = str(e)
            time.sleep(0.02)
        raise ConnectionError(
            "Insight9 did not produce a complete decodable RGB/VIO sample within "
            f"{self.config.startup_timeout_s:.1f}s: {last_error}. Check native/UVC mode, "
            "USB bandwidth, and /dev/hidraw* permissions."
        )

    def read(self) -> NDArray[np.uint8]:
        return self.read_snapshot_latest().rgb

    def async_read(self, timeout_ms: float = 200) -> NDArray[np.uint8]:
        # Intentional latest-cache semantics for UMI recording. ``timeout_ms`` is
        # accepted for Camera API compatibility but no fresh-frame wait occurs.
        del timeout_ms
        return self.read_snapshot_latest().rgb

    def read_latest(self, max_age_ms: int = 500) -> NDArray[np.uint8]:
        snapshot = self.read_snapshot_latest()
        if self._last_good_color is None:
            raise RuntimeError("no decodable Insight9 RGB frame received yet")
        age_ms = _age_seconds(time.time_ns(), self._last_good_color.host_time_ns) * 1000
        if age_ms > max_age_ms:
            raise TimeoutError(
                f"Insight9 RGB cache is {age_ms:.1f}ms old (limit={max_age_ms}ms)."
            )
        return snapshot.rgb

    def read_snapshot_latest(self) -> Insight9Snapshot:
        if not self.is_connected or self._sdk_camera is None:
            raise DeviceNotConnectedError(f"{self} is not connected")

        raw = self._sdk_camera.latest()
        sample_host_time_ns = time.time_ns()

        if raw.vio is not None:
            self._last_vio = raw.vio

        color = raw.color
        if color is not None and color.frame_index != self._last_seen_color_frame_index:
            self._last_seen_color_frame_index = int(color.frame_index)
            decoded, error = self._decode_color(color)
            if decoded is not None:
                self._last_good_rgb = decoded
                self._last_good_color = color
            else:
                self._warn_decode(error)

        if self._last_good_rgb is None or self._last_good_color is None:
            raise RuntimeError("no decodable Insight9 RGB frame received yet")
        if self._last_vio is None:
            raise RuntimeError("no Insight9 VIO sample received yet")

        color_age_s = _age_seconds(sample_host_time_ns, self._last_good_color.host_time_ns)
        vio_age_s = _age_seconds(sample_host_time_ns, self._last_vio.host_time_ns)
        ages = (color_age_s, vio_age_s)
        self._raise_if_stale(ages)
        self._warn_if_stale(ages)

        rotation_6d = quaternion_to_rotation_6d(
            float(self._last_vio.qw),
            float(self._last_vio.qx),
            float(self._last_vio.qy),
            float(self._last_vio.qz),
        )
        return Insight9Snapshot(
            rgb=self._last_good_rgb,
            vio_position=(
                float(self._last_vio.px),
                float(self._last_vio.py),
                float(self._last_vio.pz),
            ),
            vio_rotation_6d=tuple(float(value) for value in rotation_6d),
        )

    def _decode_color(self, image: Any) -> tuple[NDArray[np.uint8] | None, str]:
        if image.pixel_format != "mjpeg":
            return None, f"unsupported pixel format {image.pixel_format!r}"

        payload = image.data
        if self.config.strict_jpeg:
            payload, error = _jpeg_payload(payload)
            if payload is None:
                return None, error

        encoded = np.frombuffer(payload, dtype=np.uint8)
        bgr = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if bgr is None:
            return None, "cv2.imdecode returned None"

        expected_shape = (int(self.height), int(self.width), 3)
        if bgr.shape != expected_shape:
            raise RuntimeError(
                "Insight9 RGB shape does not match the configured dataset schema: "
                f"actual={bgr.shape}, expected={expected_shape}. Update head_camera_width/height; "
                "the adapter intentionally does not write resolution settings to the device."
            )
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), ""

    def _warn_decode(self, error: str) -> None:
        now = time.monotonic()
        if now - self._last_decode_warning_t >= 1.0:
            self.logger.warn(f"Insight9 RGB decode failed; holding last good frame: {error}")
            self._last_decode_warning_t = now

    def _warn_if_stale(self, ages: tuple[float, float]) -> None:
        if max(ages) <= self.config.stale_after_s:
            return
        now = time.monotonic()
        if now - self._last_stale_warning_t < 1.0:
            return
        self.logger.warn(
            "Insight9 latest cache is stale: "
            f"rgb={ages[0]:.3f}s vio={ages[1]:.3f}s "
            f"threshold={self.config.stale_after_s:.3f}s"
        )
        self._last_stale_warning_t = now

    def _raise_if_stale(self, ages: tuple[float, float]) -> None:
        stale_streams = [
            f"{name}={age_s:.3f}s"
            for name, age_s in zip(("rgb", "vio"), ages, strict=True)
            if age_s > self.config.stale_timeout_s
        ]
        if not stale_streams:
            return
        raise TimeoutError(
            "Insight9 head data stopped updating; aborting recording: "
            f"{' '.join(stale_streams)} limit={self.config.stale_timeout_s:.3f}s"
        )

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")
        self._close_sdk()
        self.logger.info(f"{self} disconnected")

    def _close_sdk(self) -> None:
        camera = self._sdk_camera
        self._sdk_camera = None
        self._is_connected = False
        try:
            if camera is not None:
                camera.close()
        finally:
            self._reset_cache()

    def _reset_cache(self) -> None:
        self._last_good_rgb = None
        self._last_good_color = None
        self._last_vio = None
        self._last_seen_color_frame_index = None
        self._last_stale_warning_t = 0.0
        self._last_decode_warning_t = 0.0


def _age_seconds(sample_host_time_ns: int, callback_host_time_ns: int) -> float:
    return max(0.0, (int(sample_host_time_ns) - int(callback_host_time_ns)) / 1e9)


def _jpeg_payload(data: bytes) -> tuple[bytes | None, str]:
    """Validate an MJPEG/JPEG payload and trim bytes following the first EOI."""

    if len(data) < 4:
        return None, "too_short"
    if data[:2] != b"\xff\xd8":
        return None, "missing_soi"

    i = 2
    while i < len(data):
        if data[i] != 0xFF:
            return None, f"expected_marker_at_{i}"
        while i < len(data) and data[i] == 0xFF:
            i += 1
        if i >= len(data):
            return None, "missing_marker_after_ff"

        marker = data[i]
        i += 1
        if marker == 0xD9:
            return data[:i], "ok"
        if marker == 0xDA:
            if i + 2 > len(data):
                return None, "truncated_sos_length"
            segment_length = int.from_bytes(data[i : i + 2], "big")
            if segment_length < 2:
                return None, "invalid_sos_length"
            scan_start = i + segment_length
            if scan_start > len(data):
                return None, "truncated_sos_segment"
            end = data.find(b"\xff\xd9", scan_start)
            if end < 0:
                return None, "missing_eoi"
            return data[: end + 2], "ok"
        if marker == 0x01 or 0xD0 <= marker <= 0xD8:
            continue

        if i + 2 > len(data):
            return None, f"truncated_segment_length_0x{marker:02x}"
        segment_length = int.from_bytes(data[i : i + 2], "big")
        if segment_length < 2:
            return None, f"invalid_segment_length_0x{marker:02x}"
        segment_end = i + segment_length
        if segment_end > len(data):
            return None, f"truncated_segment_0x{marker:02x}"
        i = segment_end

    return None, "missing_eoi"
