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


@CameraConfig.register_subclass("insight")
@dataclass
class InsightCameraConfig(CameraConfig):
    """LeRobot adapter configuration for the Insight RGB/VIO head camera.

    ``width``/``height`` are the shape the dataset stores, which is *not* what
    the sensor emits. The RGB stream has exactly one mode, 1088x1920 portrait,
    and no amount of configuration changes that - the USB descriptors advertise a
    single frame descriptor and ROS native mode offers nothing extra. The adapter
    therefore crops a landscape window out of the tall frame and scales it.

    The default 1024x768 is 4:3 at 0.94x of the largest 4:3 region that fits
    (1088x816), so it is essentially native detail rather than upsampling, and
    both dimensions divide by 32, which is what stride-32 vision backbones and
    video encoders want. It keeps 72.0 x 57.2 degrees of the delivered
    72.0 x 104.1. 16:9 is available - 1024x576 - but costs a further 12.7 degrees
    vertically, usually the wrong trade when the frame has to show a work surface
    and a gripper at once.

    ``crop_bias`` slides that window along the tall axis: 0.0 keeps the top of
    the frame, 1.0 the bottom. Tune it to how the camera is actually mounted;
    the centre is rarely where the subject is.

    Changing width, height or crop_bias changes the recorded field of view, so
    episodes recorded either side of such a change are not comparable.
    """

    library_path: str | None = None
    startup_timeout_s: float = 5.0
    stale_after_s: float = 0.2
    stale_timeout_s: float = 3.0
    strict_jpeg: bool = True
    crop_bias: float = 0.5

    def __post_init__(self) -> None:
        if self.fps is None:
            self.fps = 30
        if self.width is None:
            self.width = 1024
        if self.height is None:
            self.height = 768
        if self.fps <= 0:
            raise ValueError("Insight fps must be positive.")
        if self.width <= 0 or self.height <= 0:
            raise ValueError(f"Insight width/height must be positive, got {self.width}x{self.height}.")
        if self.width < self.height:
            raise ValueError(
                f"Insight width/height describe a landscape crop, got {self.width}x{self.height}. "
                "The sensor frame is portrait and the adapter crops rather than rotates, so a "
                "portrait target would ask for a region taller than the source."
            )
        if not 0.0 <= self.crop_bias <= 1.0:
            raise ValueError(f"Insight crop_bias must be within 0.0..1.0, got {self.crop_bias}.")
        if self.startup_timeout_s <= 0:
            raise ValueError("Insight startup_timeout_s must be positive.")
        if self.stale_after_s <= 0:
            raise ValueError("Insight stale_after_s must be positive.")
        if self.stale_timeout_s <= 0:
            raise ValueError("Insight stale_timeout_s must be positive.")
