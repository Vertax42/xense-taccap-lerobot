#!/usr/bin/env python

# Copyright 2026 The XenseRobotics Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""The XTac-UMI-G1 robot: ``BiTaccapGripper`` with the headset always on."""

from ..bi_taccap_gripper.bi_taccap_gripper import BiTaccapGripper
from .config_xtac_umi_g1 import XtacUmiG1Config


class XtacUmiG1(BiTaccapGripper):
    """Bimanual TacCap grippers + Pico headset.

    No behaviour of its own: every head branch in ``BiTaccapGripper`` already
    keys off ``config.enable_head_camera``, which the config pins to True for
    this type. What this class contributes is ``name``, and ``name`` is the
    whole point — it is what ``lerobot-record`` writes as the dataset's
    ``robot_type``, so a headset recording finally says so on disk.

    Calibration is shared with ``bi_taccap_gripper`` in the only sense that
    matters: there is none. ``calibrate()`` is a no-op and the encoder zero lives
    in each unit's firmware, so the per-name calibration directory
    ``Robot.__init__`` creates stays empty for both types.
    """

    config_class = XtacUmiG1Config
    name = "xtac_umi_g1"
