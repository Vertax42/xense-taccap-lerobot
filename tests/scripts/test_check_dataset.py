#!/usr/bin/env python

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
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

from lerobot.datasets.dataset_tools import (
    TACCAP_6_CAMERA_FEATURE_KEYS,
    TACCAP_8_CAMERA_FEATURE_KEYS,
)
from lerobot.scripts.lerobot_check_dataset import classify_camera_format


def _state_names(head_camera_dims: bool) -> list[str]:
    names = []
    for side in ("left", "right"):
        names.extend(
            f"{side}_tcp.{key}" for key in ("x", "y", "z", "r1", "r2", "r3", "r4", "r5", "r6")
        )
    names.extend(["left_gripper.pos", "right_gripper.pos"])
    if head_camera_dims:
        names.extend(f"head_camera.{key}" for key in ("x", "y", "z", "r1", "r2", "r3", "r4", "r5", "r6"))
    return names


def _info(camera_keys, head_camera_dims: bool) -> dict:
    names = _state_names(head_camera_dims)
    camera_features = {
        key: {
            "dtype": "video",
            "shape": (4, 6, 3),
            "names": ["height", "width", "channels"],
        }
        for key in camera_keys
    }
    return {
        "features": {
            "action": {"dtype": "float32", "shape": (len(names),), "names": names},
            "observation.state": {"dtype": "float32", "shape": (len(names),), "names": names},
            **camera_features,
        }
    }


def test_classify_6_camera_format():
    camera_format, errors, warnings = classify_camera_format(
        _info(TACCAP_6_CAMERA_FEATURE_KEYS, head_camera_dims=False)
    )
    assert camera_format == "6"
    assert errors == []
    assert warnings == []


def test_classify_8_camera_format():
    camera_format, errors, warnings = classify_camera_format(
        _info(TACCAP_8_CAMERA_FEATURE_KEYS, head_camera_dims=True)
    )
    assert camera_format == "8"
    assert errors == []
    assert warnings == []


def test_classify_8_camera_missing_head_dims():
    camera_format, errors, warnings = classify_camera_format(
        _info(TACCAP_8_CAMERA_FEATURE_KEYS, head_camera_dims=False)
    )
    assert camera_format == "8"
    assert len(errors) == 2
    assert warnings == []


def test_classify_6_camera_with_head_dims():
    camera_format, errors, warnings = classify_camera_format(
        _info(TACCAP_6_CAMERA_FEATURE_KEYS, head_camera_dims=True)
    )
    assert camera_format == "6"
    assert len(errors) == 2
    assert warnings == []


def test_classify_unknown_camera_format():
    camera_keys = TACCAP_6_CAMERA_FEATURE_KEYS | {"observation.images.extra"}
    camera_format, errors, warnings = classify_camera_format(
        _info(camera_keys, head_camera_dims=False)
    )
    assert camera_format is None
    assert errors == []
    assert len(warnings) == 1


def test_classify_missing_vector_names():
    info = _info(TACCAP_6_CAMERA_FEATURE_KEYS, head_camera_dims=False)
    info["features"]["action"]["names"] = None
    info["features"]["observation.state"]["names"] = None

    camera_format, errors, warnings = classify_camera_format(info)
    assert camera_format == "6"
    assert errors == []
    assert len(warnings) == 2
