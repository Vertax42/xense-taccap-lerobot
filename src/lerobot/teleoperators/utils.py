# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
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

from enum import Enum
from typing import TYPE_CHECKING, cast

from lerobot.utils.import_utils import make_device_from_device_class

from .config import TeleoperatorConfig

if TYPE_CHECKING:
    from .teleoperator import Teleoperator


class TeleopEvents(Enum):
    """Shared constants for teleoperator events across teleoperators."""

    SUCCESS = "success"
    FAILURE = "failure"
    RERECORD_EPISODE = "rerecord_episode"
    IS_INTERVENTION = "is_intervention"
    TERMINATE_EPISODE = "terminate_episode"
    BACK_HOME = "back_home"
    

def make_teleoperator_from_config(config: TeleoperatorConfig) -> "Teleoperator":
    # All teleoperators (pico4 / bi_pico4) resolve through the draccus
    # ChoiceRegistry + config_class lookup.
    try:
        return cast("Teleoperator", make_device_from_device_class(config))
    except Exception as e:
        raise ValueError(f"Error creating teleoperator with config {config}: {e}") from e
