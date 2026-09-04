#!/usr/bin/env python

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

from lerobot.cameras.xense.camera_xense import XenseTactileCamera


def test_disconnect_suspends_sdk_stream_before_release():
    calls = []

    class FakeSDKCamera:
        def suspend_stream(self):
            calls.append("suspend")

    class FakeContext:
        def get_handle(self, key):
            assert key == "cam"
            return FakeSDKCamera()

    class FakeSensor:
        ctx = FakeContext()

        def release(self):
            calls.append("release")

    camera = object.__new__(XenseTactileCamera)
    camera.serial_number = "GSPS01A29Z0021"
    camera.sensor = FakeSensor()
    camera.thread = None

    camera.disconnect()

    assert calls == ["suspend", "release"]
    assert camera.sensor is None


def _camera(sensor):
    camera = object.__new__(XenseTactileCamera)
    camera.serial_number = "GSPS01A29Z0021"
    camera.sensor = sensor
    camera.thread = None
    return camera


def test_release_still_runs_when_the_sdk_has_no_camera_handle():
    """The suspend reaches into SDK internals that only xensesdk 2.1.2/2.1.3 are
    known to expose. On any other version that probe must not cost us the
    release: a leaked camera is worse than the crash the suspend guards
    against, and that version never had the crash."""
    calls = []

    class FakeContext:
        def get_handle(self, key):
            raise AttributeError("no such handle")

    class FakeSensor:
        ctx = FakeContext()

        def release(self):
            calls.append("release")

    camera = _camera(FakeSensor())
    camera.disconnect()

    assert calls == ["release"]
    assert camera.sensor is None


def test_release_still_runs_when_suspend_itself_fails():
    calls = []

    class FakeSDKCamera:
        def suspend_stream(self):
            raise RuntimeError("stream already gone")

    class FakeContext:
        def get_handle(self, key):
            return FakeSDKCamera()

    class FakeSensor:
        ctx = FakeContext()

        def release(self):
            calls.append("release")

    camera = _camera(FakeSensor())
    camera.disconnect()

    assert calls == ["release"]
    assert camera.sensor is None


def test_a_missing_handle_is_not_suspended():
    """`get_handle` returning None is a normal answer, not an error."""
    calls = []

    class FakeContext:
        def get_handle(self, key):
            return None

    class FakeSensor:
        ctx = FakeContext()

        def release(self):
            calls.append("release")

    camera = _camera(FakeSensor())
    camera.disconnect()

    assert calls == ["release"]
