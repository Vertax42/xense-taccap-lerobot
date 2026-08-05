#!/usr/bin/env python

# Copyright 2026 The XenseRobotics Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Mid-episode camera-loss handling (``robots/taccap_gripper/camera_health.py``).

No hardware: the loss modes are reproduced with fake cameras, which is the point
— the real ones can only be provoked by physically pulling a USB cable, so this
behaviour was previously only ever exercised by accident, in the field, during a
recording someone cared about.
"""

import numpy as np
import pytest

from lerobot.robots.taccap_gripper.camera_health import CameraReadGuard


class FakeConfig:
    """Stands in for a CameraConfig: only the shape fields are read."""

    def __init__(self, height, width, frame_width=None, output_types=None):
        self.height = height
        self.width = width
        if frame_width is not None:
            self.frame_width = frame_width
        if output_types is not None:
            self.output_types = output_types


class FakeOutputType:
    """Stands in for XenseOutputType — only ``.value`` is read."""

    def __init__(self, value):
        self.value = value


class FakeLogger:
    def __init__(self):
        self.records = []

    def _add(self, level):
        return lambda msg: self.records.append((level, msg))

    def __getattr__(self, name):
        if name in ("warn", "error", "info", "debug"):
            return self._add(name)
        raise AttributeError(name)

    def count(self, level):
        return sum(1 for lvl, _ in self.records if lvl == level)


class FakeCamera:
    """``mode`` selects the failure being reproduced.

    ``live`` returns a fresh array each call, as a healthy camera's read thread
    does; ``freeze`` returns the identical object forever, which is what an
    unplugged Xense tactile sensor does (its thread survives the error and
    ``async_read`` keeps handing back the last cached result by reference).
    """

    def __init__(self, mode, shape=(4, 6, 3)):
        self.mode = mode
        self.shape = shape
        self._cached = np.zeros(shape, dtype=np.uint8)
        self.calls = 0

    def async_read(self):
        self.calls += 1
        if self.mode == "raise":
            raise RuntimeError("read thread is not running")
        if self.mode == "timeout":
            raise TimeoutError("no frame within 200 ms")
        if self.mode == "freeze":
            return self._cached
        return np.zeros(self.shape, dtype=np.uint8)

    def produce_new_frame(self):
        self._cached = np.zeros(self.shape, dtype=np.uint8)


@pytest.fixture
def logger():
    return FakeLogger()


def test_read_passes_live_frames_through(logger):
    guard = CameraReadGuard({"wrist_cam": FakeConfig(4, 6)}, logger)
    cam = FakeCamera("live")

    frame = guard.read("wrist_cam", cam)

    assert frame.shape == (4, 6, 3)
    assert guard.lost is False
    assert logger.records == []


def test_raising_camera_is_flagged_lost_and_does_not_propagate(logger):
    """An unplugged UVC wrist camera raises. Letting that through would crash
    the record loop and lose the in-progress episode."""
    guard = CameraReadGuard({"wrist_cam": FakeConfig(4, 6)}, logger)

    frame = guard.read("wrist_cam", FakeCamera("raise"))

    assert frame.shape == (4, 6, 3)  # black stand-in of the declared shape
    assert guard.lost is True
    assert guard.lost_cameras == frozenset({"wrist_cam"})
    assert logger.count("error") == 1


def test_loss_is_logged_once_per_camera(logger):
    guard = CameraReadGuard({"wrist_cam": FakeConfig(4, 6)}, logger)
    cam = FakeCamera("raise")

    for _ in range(5):
        guard.read("wrist_cam", cam)

    assert logger.count("error") == 1, "a dead camera must not spam the log every frame"


def test_timeout_reuses_last_frame_without_flagging_loss(logger):
    """A dropped frame is transient and recovers on its own; treating it as loss
    would end an episode over one slow read."""
    guard = CameraReadGuard({"wrist_cam": FakeConfig(4, 6)}, logger)
    good = guard.read("wrist_cam", FakeCamera("live"))

    reused = guard.read("wrist_cam", FakeCamera("timeout"))

    assert reused is good
    assert guard.lost is False
    assert logger.count("warn") == 1
    assert logger.count("error") == 0


def test_frozen_stream_is_flagged_after_the_timeout(monkeypatch, logger):
    """The silent failure: an unplugged tactile sensor never raises, it just
    repeats its last frame. Undetected, the dataset fills with stale images."""
    clock = {"t": 1000.0}
    monkeypatch.setattr(
        "lerobot.robots.taccap_gripper.camera_health.time.monotonic",
        lambda: clock["t"],
    )
    guard = CameraReadGuard({"tactile_left": FakeConfig(4, 6)}, logger, freeze_timeout_s=2.0)
    cam = FakeCamera("freeze")

    guard.read("tactile_left", cam)
    clock["t"] += 1.9
    guard.read("tactile_left", cam)
    assert guard.lost is False, "must not trip before the timeout elapses"

    clock["t"] += 0.2
    guard.read("tactile_left", cam)

    assert guard.lost is True
    assert "frozen" in logger.records[-1][1]


def test_camera_slower_than_the_loop_is_not_mistaken_for_frozen(monkeypatch, logger):
    """A 10 fps sensor read at 30 fps legitimately repeats frames. Only the
    absence of *any* new frame object counts as frozen."""
    clock = {"t": 1000.0}
    monkeypatch.setattr(
        "lerobot.robots.taccap_gripper.camera_health.time.monotonic",
        lambda: clock["t"],
    )
    guard = CameraReadGuard({"tactile_left": FakeConfig(4, 6)}, logger, freeze_timeout_s=2.0)
    cam = FakeCamera("freeze")

    for _ in range(20):
        for _ in range(3):  # three reads of the same frame, as a slow sensor gives
            guard.read("tactile_left", cam)
            clock["t"] += 0.5
        cam.produce_new_frame()

    assert guard.lost is False


def test_first_read_fallback_for_multi_output_tactile_keeps_the_dict_shape(logger):
    """A tactile sensor asked for several output types reads as a dict. A bare
    array here would drop the display-only keys in ``split_camera_read``."""
    cfg = FakeConfig(4, 6, output_types=[FakeOutputType("rectify"), FakeOutputType("difference")])
    guard = CameraReadGuard({"tactile_left": cfg}, logger)

    frame = guard.read("tactile_left", FakeCamera("raise"))

    assert isinstance(frame, dict)
    assert set(frame) == {"rectify", "difference"}
    assert all(v.shape == (4, 6, 3) for v in frame.values())
    assert frame["rectify"] is not frame["difference"], "views must not alias one buffer"


def test_first_read_fallback_uses_frame_width_for_merged_stereo(logger):
    """``width`` on a Pico head camera is per-eye; a merged frame is twice that,
    and that is what the robots declare in ``observation_features``."""
    guard = CameraReadGuard({"head": FakeConfig(768, 1024, frame_width=2048)}, logger)

    frame = guard.read("head", FakeCamera("raise"))

    assert frame.shape == (768, 2048, 3)


def test_last_good_frame_is_kept_after_loss(logger):
    """Degrade to the last real image, not to black, when there is one."""
    guard = CameraReadGuard({"wrist_cam": FakeConfig(4, 6)}, logger)
    live = FakeCamera("live")
    good = guard.read("wrist_cam", live)

    after = guard.read("wrist_cam", FakeCamera("raise"))

    assert after is good


def test_reset_clears_state_for_a_reconnect(logger):
    """Without this a reconnect starts out flagged, and the next recording stops
    on the first frame."""
    guard = CameraReadGuard({"wrist_cam": FakeConfig(4, 6)}, logger)
    guard.read("wrist_cam", FakeCamera("raise"))
    assert guard.lost is True

    guard.reset()

    assert guard.lost is False
    assert guard.lost_cameras == frozenset()
    # and a fresh loss is reported again rather than swallowed as a duplicate
    guard.read("wrist_cam", FakeCamera("raise"))
    assert logger.count("error") == 2


def test_losses_are_tracked_per_camera(logger):
    guard = CameraReadGuard({"a": FakeConfig(4, 6), "b": FakeConfig(4, 6)}, logger)

    guard.read("a", FakeCamera("raise"))
    guard.read("b", FakeCamera("live"))

    assert guard.lost_cameras == frozenset({"a"})
