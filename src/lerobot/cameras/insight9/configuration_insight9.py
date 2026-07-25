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

from ..configs import CameraConfig


@CameraConfig.register_subclass("insight9")
@dataclass
class Insight9CameraConfig(CameraConfig):
    """LeRobot adapter configuration for the Insight9 RGB/VIO head camera.

    ``width``/``height`` describe the frame shape expected by the dataset. They
    are validated against the first decoded frame and are intentionally not
    written back to the device: the current Insight9 firmware keeps emitting
    1088x1920 even when its resolution parameter is changed.
    """

    library_path: str | None = None
    startup_timeout_s: float = 5.0
    stale_after_s: float = 0.2
    stale_timeout_s: float = 3.0
    strict_jpeg: bool = True

    def __post_init__(self) -> None:
        if self.fps is None:
            self.fps = 30
        if self.width is None:
            self.width = 1088
        if self.height is None:
            self.height = 1920
        if self.fps <= 0:
            raise ValueError("Insight9 fps must be positive.")
        if self.width <= 0 or self.height <= 0:
            raise ValueError(f"Insight9 width/height must be positive, got {self.width}x{self.height}.")
        if self.startup_timeout_s <= 0:
            raise ValueError("Insight9 startup_timeout_s must be positive.")
        if self.stale_after_s <= 0:
            raise ValueError("Insight9 stale_after_s must be positive.")
        if self.stale_timeout_s <= 0:
            raise ValueError("Insight9 stale_timeout_s must be positive.")
