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

from unittest.mock import MagicMock, patch

import draccus
import pytest

from lerobot.scripts.lerobot_edit_dataset import (
    ConvertImageToVideoConfig,
    DeleteEpisodesConfig,
    EditDatasetConfig,
    InfoConfig,
    MergeConfig,
    ModifyTasksConfig,
    OperationConfig,
    RemoveFeatureConfig,
    SplitConfig,
    _validate_config,
    get_output_path,
    handle_convert_image_to_video,
)


def parse_cfg(cli_args: list[str]) -> EditDatasetConfig:
    """Helper to parse CLI args into an EditDatasetConfig via draccus."""
    return draccus.parse(EditDatasetConfig, args=cli_args)


class TestOperationTypeParsing:
    """Test that --operation.type correctly selects the right config subclass."""

    @pytest.mark.parametrize(
        "type_name, expected_cls",
        [
            ("delete_episodes", DeleteEpisodesConfig),
            ("split", SplitConfig),
            ("merge", MergeConfig),
            ("remove_feature", RemoveFeatureConfig),
            ("modify_tasks", ModifyTasksConfig),
            ("convert_image_to_video", ConvertImageToVideoConfig),
            ("info", InfoConfig),
        ],
    )
    def test_operation_type_resolves_correct_class(self, type_name, expected_cls):
        cfg = parse_cfg(["--repo_id", "test/repo", "--new_repo_id", "test/merged", "--operation.type", type_name])
        assert isinstance(cfg.operation, expected_cls), (
            f"Expected {expected_cls.__name__}, got {type(cfg.operation).__name__}"
        )

    def test_merge_requires_new_repo_id(self):
        cfg = parse_cfg(["--operation.type", "merge"])
        with pytest.raises(ValueError, match="--new_repo_id is required for merge"):
            _validate_config(cfg)

    def test_non_merge_requires_repo_id(self):
        cfg = parse_cfg(["--operation.type", "delete_episodes"])
        with pytest.raises(ValueError, match="--repo_id is required for delete_episodes"):
            _validate_config(cfg)

    @pytest.mark.parametrize(
        "type_name, expected_cls",
        [
            ("delete_episodes", DeleteEpisodesConfig),
            ("split", SplitConfig),
            ("merge", MergeConfig),
            ("remove_feature", RemoveFeatureConfig),
            ("modify_tasks", ModifyTasksConfig),
            ("convert_image_to_video", ConvertImageToVideoConfig),
            ("info", InfoConfig),
        ],
    )
    def test_get_choice_name_roundtrips(self, type_name, expected_cls):
        cfg = parse_cfg(["--repo_id", "test/repo", "--new_repo_id", "test/merged", "--operation.type", type_name])
        resolved_name = OperationConfig.get_choice_name(type(cfg.operation))
        assert resolved_name == type_name


def test_get_output_path_prefers_local_dataset_root(tmp_path):
    root = tmp_path / "pusht"

    output_repo_id, output_path = get_output_path(
        repo_id="lerobot/pusht",
        new_repo_id=None,
        root=root,
        new_root=None,
    )

    assert output_repo_id == "lerobot/pusht"
    assert output_path == root


def test_get_output_path_places_new_local_dataset_next_to_source(tmp_path):
    root = tmp_path / "pusht"

    output_repo_id, output_path = get_output_path(
        repo_id="lerobot/pusht",
        new_repo_id="lerobot/pusht_filtered",
        root=root,
        new_root=None,
    )

    assert output_repo_id == "lerobot/pusht_filtered"
    assert output_path == tmp_path / "lerobot" / "pusht_filtered"


def test_convert_image_to_video_uses_local_root(tmp_path):
    root = tmp_path / "pusht_image"
    dataset = MagicMock()
    dataset.meta.video_keys = []
    dataset.meta.features = {}

    cfg = EditDatasetConfig(
        operation=ConvertImageToVideoConfig(),
        repo_id="pusht_image",
        root=str(root),
    )

    with (
        patch("lerobot.scripts.lerobot_edit_dataset.LeRobotDataset", return_value=dataset),
        patch("lerobot.scripts.lerobot_edit_dataset.convert_image_to_video_dataset") as mock_convert,
    ):
        mock_convert.return_value = dataset
        handle_convert_image_to_video(cfg)

    assert mock_convert.call_args.kwargs["output_dir"] == tmp_path / "pusht_image_video"
