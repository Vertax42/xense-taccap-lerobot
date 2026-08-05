#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
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

"""Test doubles for the device factories.

``make_robot_from_config`` / ``make_teleoperator_from_config`` locate a device
class by importing the *parent package* of its config's module and looking for a
class whose ``config_class`` matches (``lerobot.utils.import_utils``). For the
real devices that parent is ``lerobot.robots.<device>``, whose ``__init__``
exports the pair. ``tests.mocks`` had no ``__init__`` at all, so the candidate
imported as an empty namespace package and every factory lookup failed — which
is why ``test_control_robot`` could not build a MockRobot.

Only the robot/teleop pair is re-exported here; the motor-bus and serial mocks
are imported directly by the tests that need them and would pull ``mock_serial``
into every collection.
"""

from tests.mocks.mock_robot import MockRobot, MockRobotConfig
from tests.mocks.mock_teleop import MockTeleop, MockTeleopConfig

__all__ = ["MockRobot", "MockRobotConfig", "MockTeleop", "MockTeleopConfig"]
