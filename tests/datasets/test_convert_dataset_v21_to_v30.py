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

from pathlib import Path

import pytest

from lerobot.datasets.v30.convert_dataset_v21_to_v30 import convert_dataset


def _write_minimal_v21_info(root: Path) -> None:
    (root / "meta").mkdir(parents=True, exist_ok=True)
    (root / "meta" / "info.json").write_text('{"codebase_version": "v2.1", "features": {}}')


def test_convert_dataset_refuses_to_delete_current_root_when_backup_exists(tmp_path):
    root = tmp_path / "ds"
    old_root = tmp_path / "ds_old"
    _write_minimal_v21_info(root)
    old_root.mkdir()

    with pytest.raises(FileExistsError, match="Refusing to delete"):
        convert_dataset("test/repo", root=root, push_to_hub=False)


def test_convert_dataset_refuses_remote_fallback_when_backup_exists(tmp_path):
    old_root = tmp_path / "ds_old"
    old_root.mkdir()

    with pytest.raises(FileNotFoundError, match="previous conversion backup"):
        convert_dataset("test/repo", root=tmp_path / "ds", push_to_hub=False)


def test_convert_dataset_refuses_missing_explicit_root(tmp_path):
    with pytest.raises(FileNotFoundError, match="does not exist"):
        convert_dataset("test/repo", root=tmp_path / "ds", push_to_hub=False)


def test_convert_dataset_refuses_to_delete_staging_dir(tmp_path):
    root = tmp_path / "ds"
    _write_minimal_v21_info(root)
    (tmp_path / "ds_v30").mkdir()

    with pytest.raises(FileExistsError, match="staging directory"):
        convert_dataset("test/repo", root=root, push_to_hub=False)
