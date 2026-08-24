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

"""TacCap's fork-local `meta/` survives the operations that derive a dataset.

`LeRobotDatasetMetadata.create` builds a fresh standard metadata set, so anything
this fork adds under `meta/` is dropped unless a copy step carries it. That is not
a cosmetic loss: `hardware.json` is the only record of which physical sensor fed
which tactile stream, and solving a stream against another unit's calibration
reports plausible depth and force rather than failing.
"""

from pathlib import Path
from types import SimpleNamespace

import pytest

from lerobot.datasets.dataset_tools import (
    _copy_taccap_local_metadata,
    _reject_merge_of_taccap_hardware_metadata,
)
from lerobot.utils.constants import TACCAP_HARDWARE_MANIFEST_PATH, TACCAP_RUNTIME_DIR


def make_source(root: Path, *, manifest: bool = True, runtimes: int = 0) -> Path:
    (root / "meta").mkdir(parents=True, exist_ok=True)
    if manifest:
        (root / TACCAP_HARDWARE_MANIFEST_PATH).write_text('{"robot_type": "bi_taccap_gripper"}')
    for i in range(runtimes):
        bundle = root / TACCAP_RUNTIME_DIR / f"GSPS01A29Z{i:04d}-20260824T101500.bin"
        bundle.parent.mkdir(parents=True, exist_ok=True)
        bundle.write_bytes(b"runtime-%d" % i)
    return root


class TestCopy:
    def test_the_manifest_survives(self, tmp_path):
        src = make_source(tmp_path / "src")
        dst = tmp_path / "dst"
        _copy_taccap_local_metadata(src, dst)
        assert (dst / TACCAP_HARDWARE_MANIFEST_PATH).read_text() == (src / TACCAP_HARDWARE_MANIFEST_PATH).read_text()

    def test_the_runtime_bundles_survive(self, tmp_path):
        """The manifest names the sensors; the bundles are what actually lets a
        stream be re-solved. Carrying one without the other is half a record."""
        src = make_source(tmp_path / "src", runtimes=3)
        dst = tmp_path / "dst"
        _copy_taccap_local_metadata(src, dst)
        assert sorted(p.name for p in (dst / TACCAP_RUNTIME_DIR).iterdir()) == sorted(
            p.name for p in (src / TACCAP_RUNTIME_DIR).iterdir()
        )

    def test_a_source_without_them_is_not_an_error(self, tmp_path):
        """Upstream datasets have neither. The copy is additive, never required."""
        src = make_source(tmp_path / "src", manifest=False)
        dst = tmp_path / "dst"
        _copy_taccap_local_metadata(src, dst)
        assert not (dst / TACCAP_HARDWARE_MANIFEST_PATH).exists()

    def test_it_does_not_clobber_unrelated_destination_files(self, tmp_path):
        src = make_source(tmp_path / "src")
        dst = tmp_path / "dst"
        (dst / "meta").mkdir(parents=True)
        (dst / "meta" / "info.json").write_text("{}")
        _copy_taccap_local_metadata(src, dst)
        assert (dst / "meta" / "info.json").read_text() == "{}"


class TestMergeRefuses:
    def fake(self, root: Path, repo_id: str, *, manifest: bool):
        make_source(root, manifest=manifest)
        return SimpleNamespace(root=root, repo_id=repo_id)

    def test_merging_manifest_carrying_datasets_raises(self, tmp_path):
        """Each input maps the same observation keys to different physical sensors
        and the merged episodes interleave them, so no single manifest describes
        the result. Dropping it silently would leave a dataset that looks complete
        and derives tactile channels from the wrong calibration."""
        datasets = [
            self.fake(tmp_path / "a", "org/a", manifest=True),
            self.fake(tmp_path / "b", "org/b", manifest=True),
        ]
        with pytest.raises(NotImplementedError) as e:
            _reject_merge_of_taccap_hardware_metadata(datasets)
        assert "org/a" in str(e.value) and "org/b" in str(e.value)

    def test_one_carrier_is_enough_to_refuse(self, tmp_path):
        """Merging a manifest-carrying dataset with one that has none is *more*
        ambiguous, not less: the result would silently claim every episode came
        from the sensors named in the only manifest present."""
        datasets = [
            self.fake(tmp_path / "a", "org/a", manifest=True),
            self.fake(tmp_path / "b", "org/b", manifest=False),
        ]
        with pytest.raises(NotImplementedError):
            _reject_merge_of_taccap_hardware_metadata(datasets)

    def test_plain_upstream_datasets_still_merge(self, tmp_path):
        datasets = [
            self.fake(tmp_path / "a", "org/a", manifest=False),
            self.fake(tmp_path / "b", "org/b", manifest=False),
        ]
        _reject_merge_of_taccap_hardware_metadata(datasets)  # 不抛即通过


def test_every_derived_dataset_path_carries_the_metadata():
    """A new operation that derives a dataset must call the copy helper. Without
    this, adding one silently reintroduces the loss — which is invisible until
    someone tries to re-derive tactile channels months later.
    """
    import inspect

    from lerobot.datasets import dataset_tools

    derives = [
        "delete_episodes",
        "split_dataset",
        "modify_features",
        "convert_8_to_6_cameras",
        "convert_image_to_video_dataset",
    ]
    for name in derives:
        source = inspect.getsource(getattr(dataset_tools, name))
        assert "_copy_taccap_local_metadata" in source, f"{name} drops meta/hardware.json"

    merge_source = inspect.getsource(dataset_tools.merge_datasets)
    assert "_reject_merge_of_taccap_hardware_metadata" in merge_source
