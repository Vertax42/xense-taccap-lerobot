#!/usr/bin/env python

# Copyright 2026 The XenseRobotics Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Configuration for the XTac-UMI-G1 rig: the bimanual TacCap grippers plus the
Pico headset.

Physically this is ``bi_taccap_gripper`` with the headset streaming, and the
class says exactly that by inheriting from it. It exists as a *registered robot
type* rather than a flag because ``robot_type`` in a recorded dataset comes from
``robot.name`` — a class attribute bound to the draccus registry key — so a
config field could change what was recorded but never what the recording claimed
to be. Head-enabled runs under ``bi_taccap_gripper`` therefore wrote 29 state
dims and 8 cameras under a label meaning 20 and 6, and nothing at record time
could notice. Twelve datasets on disk were mislabelled this way.

With the head part of the type, ``--robot.type=`` alone decides the shape and the
label, and the two cannot disagree.
"""

from dataclasses import dataclass
from typing import ClassVar

from ..bi_taccap_gripper.config_bi_taccap_gripper import BiTaccapGripperConfig
from ..config import RobotConfig


@RobotConfig.register_subclass("xtac_umi_g1")
@dataclass
class XtacUmiG1Config(BiTaccapGripperConfig):
    """Bimanual TacCap grippers + Pico headset.

    Every gripper, tactile, tracker and wrist-camera field is inherited
    unchanged — this type differs from ``bi_taccap_gripper`` in one respect, and
    the class should keep showing that. The recorded shape is 29 state dims
    (20 + the 9-component head pose) and 8 cameras (6 + ``left_head`` /
    ``right_head``).
    """

    records_head: ClassVar[bool] = True

    enable_head_camera: bool = True
    """Fixed by the type. ``BiTaccapGripperConfig.__post_init__`` rejects a value
    that contradicts ``records_head``, so this cannot be turned off here — record
    with ``--robot.type=bi_taccap_gripper`` for the no-headset shape."""
