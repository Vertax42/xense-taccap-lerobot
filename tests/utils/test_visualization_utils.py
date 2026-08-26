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

import importlib
import sys
from enum import Enum
from types import SimpleNamespace

import numpy as np
import pytest

from lerobot.utils.constants import OBS_STATE, OBS_STR


class TransitionKey(str, Enum):
    OBSERVATION = "observation"
    ACTION = "action"


# NOTE on ``static=``: upstream logged images with ``static=True``. This fork
# dropped it (71b84ff4) because rerun never garbage-collects a static entity, so
# at 30 fps x N cameras memory grew without bound for the whole session — and a
# capture rig with four tactile sensors plus wrist and head cameras is exactly
# the shape that hits it. The assertions below therefore require that images are
# logged NON-static; flipping one back would be re-introducing that leak.


@pytest.fixture
def mock_rerun(monkeypatch):
    """
    Provide a mock `rerun` module so tests don't depend on the real library.
    Also reload the module-under-test so it binds to this mock `rr`.
    """
    calls = []

    class DummyScalar:
        def __init__(self, value):
            self.value = float(value)

    class DummyImage:
        def __init__(self, arr):
            self.arr = arr

        def compress(self, **kwargs):
            """``log_rerun_data`` calls this whenever ``compress_images`` is on
            (its default), so the stub has to offer it or every image path in
            these tests dies on AttributeError. Returns self: the assertions
            below identify the logged entity by type name, and a real
            ``rr.Image.compress()`` yields a different type that would not tell
            us anything more here."""
            self.compressed = True
            self.compress_kwargs = kwargs
            return self

    def dummy_log(key, obj=None, **kwargs):
        # Accept either positional `obj` or keyword `entity` and record remaining kwargs.
        if obj is None and "entity" in kwargs:
            obj = kwargs.pop("entity")
        calls.append((key, obj, kwargs))

    dummy_rr = SimpleNamespace(
        Scalars=DummyScalar,
        Image=DummyImage,
        log=dummy_log,
        init=lambda *a, **k: None,
        spawn=lambda *a, **k: None,
    )

    # Inject fake module into sys.modules
    monkeypatch.setitem(sys.modules, "rerun", dummy_rr)

    # Now import and reload the module under test, to bind to our rerun mock
    import lerobot.utils.visualization_utils as vu

    importlib.reload(vu)

    # Expose both the reloaded module and the call recorder
    yield vu, calls


def _keys(calls):
    """Helper to extract just the keys logged to rr.log"""
    return [k for (k, _obj, _kw) in calls]


def _obj_for(calls, key):
    """Find the first object logged under a given key."""
    for k, obj, _kw in calls:
        if k == key:
            return obj
    raise KeyError(f"Key {key} not found in calls: {calls}")


def _kwargs_for(calls, key):
    for k, _obj, kw in calls:
        if k == key:
            return kw
    raise KeyError(f"Key {key} not found in calls: {calls}")


def test_log_rerun_data_envtransition_scalars_and_image(mock_rerun):
    vu, calls = mock_rerun

    # Build EnvTransition dict
    obs = {
        f"{OBS_STATE}.temperature": np.float32(25.0),
        # CHW image should be converted to HWC for rr.Image
        "observation.camera": np.zeros((3, 10, 20), dtype=np.uint8),
    }
    act = {
        "action.throttle": 0.7,
        # 1D array should log individual Scalars with suffix _i
        "action.vector": np.array([1.0, 2.0], dtype=np.float32),
    }
    transition = {
        TransitionKey.OBSERVATION: obs,
        TransitionKey.ACTION: act,
    }

    # Extract observation and action data from transition like in the real call sites
    obs_data = transition.get(TransitionKey.OBSERVATION, {})
    action_data = transition.get(TransitionKey.ACTION, {})
    vu.log_rerun_data(observation=obs_data, action=action_data)

    # We expect:
    # - observation.state.temperature -> Scalars
    # - observation.camera -> Image (HWC), logged NON-static
    # - action.throttle -> Scalars
    # - action.vector_0, action.vector_1 -> Scalars
    expected_keys = {
        f"{OBS_STATE}.temperature",
        "observation.camera",
        "action.throttle",
        "action.vector_0",
        "action.vector_1",
    }
    assert set(_keys(calls)) == expected_keys

    # Check scalar types and values
    temp_obj = _obj_for(calls, f"{OBS_STATE}.temperature")
    assert type(temp_obj).__name__ == "DummyScalar"
    assert temp_obj.value == pytest.approx(25.0)

    throttle_obj = _obj_for(calls, "action.throttle")
    assert type(throttle_obj).__name__ == "DummyScalar"
    assert throttle_obj.value == pytest.approx(0.7)

    v0 = _obj_for(calls, "action.vector_0")
    v1 = _obj_for(calls, "action.vector_1")
    assert type(v0).__name__ == "DummyScalar"
    assert type(v1).__name__ == "DummyScalar"
    assert v0.value == pytest.approx(1.0)
    assert v1.value == pytest.approx(2.0)

    # Check image handling: CHW -> HWC
    img_obj = _obj_for(calls, "observation.camera")
    assert type(img_obj).__name__ == "DummyImage"
    assert img_obj.arr.shape == (10, 20, 3)  # transposed
    assert "static" not in _kwargs_for(calls, "observation.camera")  # see NOTE on static= above


def test_log_rerun_data_plain_list_ordering_and_prefixes(mock_rerun):
    vu, calls = mock_rerun

    # First dict without prefixes treated as observation
    # Second dict without prefixes treated as action
    obs_plain = {
        "temp": 1.5,
        # Already HWC image => should stay as-is
        "img": np.zeros((5, 6, 3), dtype=np.uint8),
        "none": None,  # should be skipped
    }
    act_plain = {
        "throttle": 0.3,
        "vec": np.array([9, 8, 7], dtype=np.float32),
    }

    # Extract observation and action data from list like the old function logic did
    # First dict was treated as observation, second as action
    vu.log_rerun_data(observation=obs_plain, action=act_plain)

    # Expected keys with auto-prefixes
    expected = {
        "observation.temp",
        "observation.img",
        "action.throttle",
        "action.vec_0",
        "action.vec_1",
        "action.vec_2",
    }
    logged = set(_keys(calls))
    assert logged == expected

    # Scalars
    t = _obj_for(calls, "observation.temp")
    assert type(t).__name__ == "DummyScalar"
    assert t.value == pytest.approx(1.5)

    throttle = _obj_for(calls, "action.throttle")
    assert type(throttle).__name__ == "DummyScalar"
    assert throttle.value == pytest.approx(0.3)

    # Image stays HWC
    img = _obj_for(calls, "observation.img")
    assert type(img).__name__ == "DummyImage"
    assert img.arr.shape == (5, 6, 3)
    assert "static" not in _kwargs_for(calls, "observation.img")  # see NOTE on static= above

    # Vectors
    for i, val in enumerate([9, 8, 7]):
        o = _obj_for(calls, f"action.vec_{i}")
        assert type(o).__name__ == "DummyScalar"
        assert o.value == pytest.approx(val)


def test_log_rerun_data_kwargs_only(mock_rerun):
    vu, calls = mock_rerun

    vu.log_rerun_data(
        observation={"observation.temp": 10.0, "observation.gray": np.zeros((8, 8, 1), dtype=np.uint8)},
        action={"action.a": 1.0},
    )

    keys = set(_keys(calls))
    assert "observation.temp" in keys
    assert "observation.gray" in keys
    assert "action.a" in keys

    temp = _obj_for(calls, "observation.temp")
    assert type(temp).__name__ == "DummyScalar"
    assert temp.value == pytest.approx(10.0)

    img = _obj_for(calls, "observation.gray")
    assert type(img).__name__ == "DummyImage"
    assert img.arr.shape == (8, 8, 1)  # remains HWC
    assert "static" not in _kwargs_for(calls, "observation.gray")  # see NOTE on static= above

    a = _obj_for(calls, "action.a")
    assert type(a).__name__ == "DummyScalar"
    assert a.value == pytest.approx(1.0)


def test_images_are_not_compressed_by_default(mock_rerun):
    """JPEG encoding runs inline on the caller's thread. Measured on a bimanual
    rig with a head camera (4 tactile + 2 wrist + 2 eyes) it costs ~13 ms per
    frame against a 33 ms budget at 30 fps, versus ~3 ms uncompressed — enough
    on its own to produce [slow_frame] overruns while recording."""
    vu, calls = mock_rerun

    vu.log_rerun_data(observation={"observation.cam": np.zeros((4, 6, 3), dtype=np.uint8)})

    img = _obj_for(calls, "observation.cam")
    assert type(img).__name__ == "DummyImage"
    assert not getattr(img, "compressed", False)


def test_compression_can_be_turned_back_on(mock_rerun):
    """Worth its loop time when the viewer is on another machine."""
    vu, calls = mock_rerun

    vu.log_rerun_data(
        observation={"observation.cam": np.zeros((4, 6, 3), dtype=np.uint8)},
        compress_images=True,
    )

    assert _obj_for(calls, "observation.cam").compressed is True


def test_log_images_false_drops_images_but_keeps_scalars(mock_rerun):
    """The point of the knob: thinning the camera tiles must not make the
    tcp.* / gripper.pos plots sparse."""
    vu, calls = mock_rerun

    vu.log_rerun_data(
        observation={
            "observation.temp": 1.5,
            "observation.cam": np.zeros((4, 6, 3), dtype=np.uint8),
        },
        action={"action.a": 0.5},
        log_images=False,
    )

    keys = _keys(calls)
    assert "observation.cam" not in keys
    assert "observation.temp" in keys
    assert "action.a" in keys


def test_log_images_false_still_logs_1d_arrays_as_scalars(mock_rerun):
    """A 1-D array is a vector of scalars, not an image — it must survive."""
    vu, calls = mock_rerun

    vu.log_rerun_data(
        observation={"observation.vec": np.array([1.0, 2.0], dtype=np.float32)},
        log_images=False,
    )

    assert set(_keys(calls)) == {"observation.vec_0", "observation.vec_1"}


# ---------------------------------------------------------------- RerunLogSink
#
# The sink exists to keep Rerun off the record loop's critical path: logging
# inline measured 27.2 ms/frame against a 24.1 ms viewer-off baseline on a
# bimanual eight-image payload, and those few milliseconds are what produced the
# `[slow_frame] ... overrun=` warnings the data team worked around by recording
# with the viewer off. So the properties worth pinning are: the caller never
# waits, and nothing the worker hits can reach the caller.


class _RecordingViz:
    """Stand-in for TaccapTrajectoryViz: records what it was handed."""

    def __init__(self, on_log=None):
        self.logged = []
        self.resets = 0
        self._on_log = on_log

    def log(self, data):
        if self._on_log is not None:
            self._on_log(data)
        self.logged.append(data)

    def reset(self):
        self.resets += 1


def _drain(sink, timeout=2.0):
    """Wait for the worker to have nothing left, without closing the sink."""
    import time

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        with sink._slot_lock:
            empty = sink._slot is None
        if empty:
            time.sleep(0.02)  # let the in-flight frame finish
            return True
        time.sleep(0.005)
    return False


def test_sink_logs_frames_and_feeds_the_trajectory_viz(mock_rerun):
    vu, calls = mock_rerun
    viz = _RecordingViz()
    sink = vu.RerunLogSink(traj_viz=viz)
    try:
        sink.submit({"gripper.pos": 0.5}, {"gripper.pos": 0.5})
        assert _drain(sink)
    finally:
        sink.close()

    assert f"{OBS_STR}.gripper.pos" in _keys(calls)
    assert viz.logged == [{"gripper.pos": 0.5}]


def test_close_drains_the_pending_frame(mock_rerun):
    """A frame submitted right before close() still reaches the viewer, so the
    display ends on the last thing recorded rather than one frame short."""
    vu, calls = mock_rerun
    sink = vu.RerunLogSink()
    sink.submit({"gripper.pos": 1.25}, {})
    sink.close()

    assert _obj_for(calls, f"{OBS_STR}.gripper.pos").value == pytest.approx(1.25)


def test_submit_never_blocks_and_drops_to_the_latest_frame(mock_rerun):
    """The whole point: a stalled viewer costs the caller nothing, and what it
    misses is the *stale* frames — the newest one always survives."""
    import threading
    import time

    vu, calls = mock_rerun
    release = threading.Event()
    first = threading.Event()

    def block_once(_data):
        if not first.is_set():
            first.set()
            release.wait(5.0)

    viz = _RecordingViz(on_log=block_once)
    sink = vu.RerunLogSink(traj_viz=viz)
    try:
        sink.submit({"gripper.pos": 0.0}, {})
        assert first.wait(2.0), "worker never picked up the first frame"

        # Worker is wedged. Every one of these lands on the caller's thread.
        t0 = time.perf_counter()
        for i in range(1, 51):
            sink.submit({"gripper.pos": float(i)}, {})
        elapsed_ms = (time.perf_counter() - t0) * 1e3

        assert elapsed_ms < 50, f"submit blocked: 50 frames took {elapsed_ms:.1f}ms"
        assert sink._dropped == 49, f"expected 49 stale frames dropped, got {sink._dropped}"

        release.set()
        assert _drain(sink)
    finally:
        release.set()
        sink.close()

    # 49 dropped, so the viewer saw the first frame and the newest one only.
    assert viz.logged == [{"gripper.pos": 0.0}, {"gripper.pos": 50.0}]


def test_reset_trajectory_clears_trails_and_drops_the_queued_frame(mock_rerun):
    """Between episodes: the frame still in flight belongs to the take that just
    ended and must not seed the new trail."""
    import threading

    vu, _calls = mock_rerun
    release = threading.Event()
    first = threading.Event()

    def block_once(_data):
        if not first.is_set():
            first.set()
            release.wait(5.0)

    viz = _RecordingViz(on_log=block_once)
    sink = vu.RerunLogSink(traj_viz=viz)
    try:
        sink.submit({"gripper.pos": 0.0}, {})
        assert first.wait(2.0)
        sink.submit({"gripper.pos": 1.0}, {})  # queued behind the wedged worker

        sink.reset_trajectory()
        with sink._slot_lock:
            assert sink._slot is None, "reset_trajectory left a stale frame queued"
        assert viz.resets == 1

        release.set()
        assert _drain(sink)
    finally:
        release.set()
        sink.close()

    assert viz.logged == [{"gripper.pos": 0.0}]


def test_a_failing_viewer_never_reaches_the_caller(mock_rerun):
    """A dead viewer must not take a recording down, and must not spam a warning
    per frame — that would just trade one flood of log lines for another."""
    vu, _calls = mock_rerun

    def boom(*_a, **_k):
        raise RuntimeError("viewer went away")

    vu.rr.log = boom
    sink = vu.RerunLogSink()
    try:
        for i in range(10):
            sink.submit({"gripper.pos": float(i)}, {})
            assert _drain(sink)
    finally:
        sink.close()

    assert sink._failures > 0
    assert sink._failure_logged is True  # warned once, then counted
