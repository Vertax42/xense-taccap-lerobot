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

from dataclasses import dataclass
from typing import ClassVar

from ..configs import CameraConfig

# The two modes the headset app is configured to emit. Both are 4:3, matching
# the sensor: PICO's camera-access API caps a frame at 2328x1748 (= 1.332), so
# a 16:9 request would be a crop or a stretch rather than more field of view.
#
# Unlike the ZED path in XenseVR-RobotVision-PC, which silently falls back to
# HD720 when asked for a resolution it does not have, an unlisted size here is
# an error. That silent fallback is exactly why that repo's README documents a
# frame size the code never produces.
SUPPORTED_MODES: tuple[tuple[int, int], ...] = ((1024, 768), (1280, 960))


@CameraConfig.register_subclass("pico")
@dataclass
class PicoCameraConfig(CameraConfig):
    """Head camera streamed from the Pico headset via the XenseVR PC Service.

    The headset sends each eye as its own ``0x30`` message carrying a JPEG;
    the service relays them and the pybind layer keeps the newest frame per
    eye. This adapter pairs the two, decodes them and returns one image.

    ``width``/``height`` describe **one eye**, following the same convention as
    the ZED stereo path (``--width`` there is per-eye and the merge doubles it
    internally). With the default ``eyes="both"`` the recorded frame is
    therefore ``height x (2 * width)`` — 768x2048 by default. The two eyes are
    concatenated at full resolution rather than squeezed into a single-eye
    budget, so nothing is thrown away; the cost is one extra video key's worth
    of pixels.

    Set ``eyes`` to ``"left"`` or ``"right"`` to record a single eye at
    ``height x width``, which halves both the JPEG decoding and the encoder
    load and sidesteps pairing entirely.

    Changing ``width``, ``height`` or ``eyes`` changes the recorded frame, so
    episodes recorded either side of such a change are not comparable.
    """

    SUPPORTED_MODES: ClassVar[tuple[tuple[int, int], ...]] = SUPPORTED_MODES

    eyes: str = "both"
    """``"both"`` (side-by-side), ``"left"`` or ``"right"``."""

    startup_timeout_s: float = 5.0
    """How long ``connect()`` waits for the first frame before giving up."""

    stale_after_s: float = 0.2
    """Age past which a cached frame is reported as stale."""

    pair_max_skew_ms: float = 20.0
    """Fallback pairing window when the two eyes carry different sequence
    numbers. Frames further apart than this are not treated as a stereo pair.
    At 30 fps one frame period is ~33 ms, so the default rejects a mismatch of
    a full frame while tolerating ordinary jitter."""

    def __post_init__(self) -> None:
        if self.fps is None:
            self.fps = 30
        if self.width is None:
            self.width = SUPPORTED_MODES[0][0]
        if self.height is None:
            self.height = SUPPORTED_MODES[0][1]

        if self.fps <= 0:
            raise ValueError("Pico camera fps must be positive.")

        if (self.width, self.height) not in SUPPORTED_MODES:
            modes = " or ".join(f"{w}x{h}" for w, h in SUPPORTED_MODES)
            raise ValueError(
                f"Pico camera supports {modes} per eye, got {self.width}x{self.height}. "
                "These are the modes the headset app emits; a different size would have to "
                "be added there first."
            )

        if self.eyes not in ("both", "left", "right"):
            raise ValueError(f"Pico camera eyes must be 'both', 'left' or 'right', got {self.eyes!r}.")

        if self.startup_timeout_s <= 0:
            raise ValueError("Pico camera startup_timeout_s must be positive.")
        if self.stale_after_s <= 0:
            raise ValueError("Pico camera stale_after_s must be positive.")
        if self.pair_max_skew_ms <= 0:
            raise ValueError("Pico camera pair_max_skew_ms must be positive.")

    @property
    def frame_width(self) -> int:
        """Width of the image this adapter returns (both eyes when merged)."""
        return self.width * 2 if self.eyes == "both" else self.width
