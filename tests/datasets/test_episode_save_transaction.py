# Copyright 2026 The XenseRobotics Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

from pathlib import Path

import numpy as np
import pyarrow.parquet as pq
import pytest
import torch

from lerobot.datasets import lerobot_dataset
from lerobot.datasets.lerobot_dataset import LeRobotDataset


def _state_features() -> dict:
    return {"state": {"dtype": "float32", "shape": (2,), "names": None}}


def _video_features() -> dict:
    return {
        "observation.images.cam": {
            "dtype": "video",
            "shape": (64, 96, 3),
            "names": ["height", "width", "channels"],
        },
        "action": {"dtype": "float32", "shape": (2,), "names": ["a", "b"]},
    }


def _add_frames(dataset: LeRobotDataset, num_frames: int, task: str = "task") -> None:
    for _ in range(num_frames):
        frame = {"task": task}
        for key, ft in dataset.features.items():
            if key in ("index", "episode_index", "task_index", "timestamp", "frame_index"):
                continue
            if ft["dtype"] == "video":
                frame[key] = np.random.randint(0, 256, ft["shape"], dtype=np.uint8)
            elif ft["dtype"] == "image":
                frame[key] = np.random.rand(*ft["shape"])
            elif ft["dtype"] == "float32":
                frame[key] = torch.randn(ft["shape"], dtype=torch.float32)
            elif ft["dtype"] == "string":
                continue
            else:
                raise ValueError(ft["dtype"])
        dataset.add_frame(frame)


def _data_path(dataset: LeRobotDataset) -> Path:
    latest = dataset.latest_episode
    return dataset.root / dataset.meta.data_path.format(
        chunk_index=latest["data/chunk_index"],
        file_index=latest["data/file_index"],
    )


def test_save_episode_appends_data_atomically(tmp_path, empty_lerobot_dataset_factory):
    dataset = empty_lerobot_dataset_factory(
        root=tmp_path / "dataset",
        features=_state_features(),
        use_videos=False,
    )
    try:
        _add_frames(dataset, 2, task="task_0")
        dataset.save_episode()
        _add_frames(dataset, 3, task="task_1")
        dataset.save_episode()

        assert dataset.meta.total_episodes == 2
        assert dataset.meta.total_frames == 5
        assert pq.read_table(_data_path(dataset)).num_rows == 5
        assert _data_path(dataset).stat().st_mode & 0o777 == 0o644
        assert not list(tmp_path.rglob("*.parquet.tmp"))
    finally:
        dataset.finalize()


def test_save_episode_rolls_back_when_metadata_commit_fails(tmp_path, monkeypatch):
    dataset = LeRobotDataset.create(
        repo_id="test/rollback",
        fps=10,
        features=_video_features(),
        root=tmp_path / "dataset",
        use_videos=True,
        streaming_encoding=True,
    )
    try:
        _add_frames(dataset, 2, task="task_0")
        dataset.save_episode()

        data_before = _data_path(dataset).read_bytes()
        video_key = "observation.images.cam"
        latest_ep = dataset.meta.latest_episode
        video_before = (
            dataset.root
            / dataset.meta.video_path.format(
                video_key=video_key,
                chunk_index=latest_ep[f"videos/{video_key}/chunk_index"][0],
                file_index=latest_ep[f"videos/{video_key}/file_index"][0],
            )
        ).read_bytes()
        info_before = dataset.meta.info.copy()

        def fail_meta_save(*args, **kwargs):
            raise RuntimeError("simulated metadata commit failure")

        _add_frames(dataset, 3, task="task_1")
        monkeypatch.setattr(dataset.meta, "save_episode", fail_meta_save)
        with pytest.raises(RuntimeError, match="simulated metadata commit failure"):
            dataset.save_episode()

        assert _data_path(dataset).read_bytes() == data_before
        latest_ep = dataset.meta.latest_episode
        video_after = (
            dataset.root
            / dataset.meta.video_path.format(
                video_key=video_key,
                chunk_index=latest_ep[f"videos/{video_key}/chunk_index"][0],
                file_index=latest_ep[f"videos/{video_key}/file_index"][0],
            )
        ).read_bytes()
        assert video_after == video_before
        assert dataset.meta.info["total_episodes"] == info_before["total_episodes"]
        assert dataset.meta.info["total_frames"] == info_before["total_frames"]
        assert not list(tmp_path.rglob(".lerobot_episode_txn_*"))
        assert not list(tmp_path.rglob(".lerobot_episode_stage_*"))
    finally:
        dataset.finalize()


def test_save_episode_cleans_staging_when_streaming_finish_fails(tmp_path, monkeypatch):
    dataset = LeRobotDataset.create(
        repo_id="test/staging_fail",
        fps=10,
        features=_video_features(),
        root=tmp_path / "dataset",
        use_videos=True,
        streaming_encoding=True,
    )
    try:
        _add_frames(dataset, 2, task="task_0")

        def fail_finish():
            raise RuntimeError("simulated encoder failure")

        monkeypatch.setattr(dataset._streaming_encoder, "finish_episode", fail_finish)
        with pytest.raises(RuntimeError, match="simulated encoder failure"):
            dataset.save_episode()

        assert dataset.meta.total_episodes == 0
        assert dataset.meta.total_frames == 0
        data_dir = dataset.root / "data"
        if data_dir.exists():
            assert not list(data_dir.rglob("*.parquet"))
        assert not list(tmp_path.rglob(".lerobot_episode_txn_*"))
        assert not list(tmp_path.rglob(".lerobot_episode_stage_*"))
    finally:
        dataset.finalize()


def test_save_episode_preserves_data_when_atomic_write_fails(tmp_path, empty_lerobot_dataset_factory, monkeypatch):
    dataset = empty_lerobot_dataset_factory(
        root=tmp_path / "dataset",
        features=_state_features(),
        use_videos=False,
    )
    try:
        _add_frames(dataset, 2, task="task_0")
        dataset.save_episode()
        data_path = _data_path(dataset)
        before = data_path.read_bytes()

        def fail_atomic_write(path, table):
            raise RuntimeError("simulated parquet failure")

        original_atomic_write = lerobot_dataset._atomic_write_parquet
        monkeypatch.setattr(lerobot_dataset, "_atomic_write_parquet", fail_atomic_write)
        _add_frames(dataset, 3, task="task_1")
        with pytest.raises(RuntimeError, match="simulated parquet failure"):
            dataset.save_episode()
        monkeypatch.setattr(lerobot_dataset, "_atomic_write_parquet", original_atomic_write)

        assert data_path.read_bytes() == before
        assert dataset.meta.total_episodes == 1
        assert dataset.meta.total_frames == 2
        assert not list(tmp_path.rglob("*.parquet.tmp"))
    finally:
        dataset.finalize()


def test_save_episode_rolls_back_flushed_episode_metadata(tmp_path, empty_lerobot_dataset_factory, monkeypatch):
    dataset = empty_lerobot_dataset_factory(
        root=tmp_path / "dataset",
        features=_state_features(),
        use_videos=False,
        metadata_buffer_size=1,
    )
    try:
        for task in ("task_0", "task_1", "task_2"):
            _add_frames(dataset, 1, task=task)
            dataset.save_episode()

        episode_path = next((dataset.root / "meta" / "episodes").rglob("*.parquet"))
        episodes_before = pq.read_table(episode_path).num_rows
        assert episodes_before == 3

        def fail_info_write(*args, **kwargs):
            raise RuntimeError("simulated metadata commit failure")

        _add_frames(dataset, 1, task="task_3")
        original_write_info = lerobot_dataset.write_info
        monkeypatch.setattr(lerobot_dataset, "write_info", fail_info_write)
        with pytest.raises(RuntimeError, match="simulated metadata commit failure"):
            dataset.save_episode()
        monkeypatch.setattr(lerobot_dataset, "write_info", original_write_info)

        assert pq.read_table(episode_path).num_rows == episodes_before
        assert dataset.meta.total_episodes == 3
        assert dataset.meta.total_frames == 3
        assert dataset.episode_buffer["task"] == ["task_3"]
        assert dataset.episode_buffer["size"] == 1
    finally:
        dataset.finalize()


def test_save_episode_rolls_back_on_keyboard_interrupt(tmp_path, empty_lerobot_dataset_factory, monkeypatch):
    dataset = empty_lerobot_dataset_factory(
        root=tmp_path / "dataset",
        features=_state_features(),
        use_videos=False,
    )
    try:
        _add_frames(dataset, 2, task="task_0")
        dataset.save_episode()
        data_path = _data_path(dataset)
        before = data_path.read_bytes()

        def interrupt_atomic_write(path, table):
            raise KeyboardInterrupt

        original_atomic_write = lerobot_dataset._atomic_write_parquet
        monkeypatch.setattr(lerobot_dataset, "_atomic_write_parquet", interrupt_atomic_write)
        _add_frames(dataset, 3, task="task_1")
        with pytest.raises(KeyboardInterrupt):
            dataset.save_episode()
        monkeypatch.setattr(lerobot_dataset, "_atomic_write_parquet", original_atomic_write)

        assert data_path.read_bytes() == before
        assert dataset.meta.total_episodes == 1
        assert dataset.meta.total_frames == 2
        assert not list(tmp_path.rglob(".lerobot_episode_txn_*"))
        assert not list(tmp_path.rglob(".lerobot_episode_stage_*"))
    finally:
        dataset.finalize()
