#!/usr/bin/env python

# Copyright 2026 The XenseRobotics Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""The pure helpers shared by the single and bimanual TacCap grippers, plus the
tracker→TCP mount transform.

These are the pieces both robots import rather than re-implement, so a change
here moves both at once — which is the point, and the reason they are worth
pinning. Nothing below touches hardware or either vendor SDK.
"""

import functools
import json
import time

import numpy as np
import pytest

from lerobot.cameras.pico import SUPPORTED_MODES
from lerobot.robots.bi_taccap_gripper.config_bi_taccap_gripper import BiTaccapGripperConfig
from lerobot.robots.taccap_gripper.camera_health import CameraReadGuard
from lerobot.robots.taccap_gripper.common import (
    HARDWARE_MANIFEST_PATH,
    HEAD_CAMERA_KEYS,
    HEAD_POSE_KEYS,
    POSE_KEYS,
    GripperReadGuard,
    HeadSkewMonitor,
    build_hardware_manifest,
    build_head_camera_configs,
    build_tactile_camera_configs,
    check_dataset_station,
    connect_cameras_parallel,
    epoch_for_episode,
    hardware_manifest_unit,
    manifest_epochs,
    manifest_robot_ids,
    open_gripper,
    read_head_camera_skew,
    resolve_wrist_undistorter,
    split_camera_read,
    swap_tactile_display_features,
    tactile_camera_output_types,
    tactile_display_key,
    tactile_serial_for_key,
    validate_robot_id,
    validate_wrist_undistort_size,
    wrist_undistort_record,
    write_hardware_manifest,
)
from lerobot.robots.taccap_gripper.config_taccap_gripper import TaccapGripperConfig
from lerobot.robots.taccap_gripper.ee_transform import resolve_tracker_to_ee, tracker_to_tcp


class TestTactileOutputTypes:
    def test_recorded_type_comes_first(self):
        """Order is load-bearing: everything after the first entry is treated as
        display-only when the display-key map is built."""
        assert tactile_camera_output_types(["rectify"], ["difference"]) == ["rectify", "difference"]

    def test_display_type_equal_to_the_recorded_one_collapses(self):
        """Asking the sensor for the same view twice would be pure waste; the
        recorded key then doubles as the displayed one."""
        assert tactile_camera_output_types(["rectify"], ["rectify"]) == ["rectify"]

    def test_no_display_types_leaves_just_the_recorded_one(self):
        assert tactile_camera_output_types(["rectify"], []) == ["rectify"]

    def test_repeats_within_the_display_list_are_left_to_the_config(self):
        """This helper only guards the recorded-vs-display collision. Repeats
        *within* the display list — and any spelling variants — are collapsed
        authoritatively by ``XenseTactileCameraConfig.__post_init__``, which is
        where the strings become enums; see the test below."""
        assert tactile_camera_output_types(["rectify"], ["difference", "depth", "difference"]) == [
            "rectify",
            "difference",
            "depth",
            "difference",
        ]

    def test_input_lists_are_not_mutated(self):
        recorded = ["rectify"]
        display = ["difference"]
        tactile_camera_output_types(recorded, display)
        assert recorded == ["rectify"]
        assert display == ["difference"]


class TestTactileOutputTypesReachTheConfigDeduplicated:
    """The other half of the split above: whatever this helper passes through,
    the camera config collapses to one entry per output type.

    It matters because ``read()`` keys its result dict by output type, so a
    duplicate would take a key away from a caller expecting one entry each —
    and the robots read the types back *off the config* when deciding which are
    display-only, precisely so they see the post-normalisation list.
    """

    def test_repeats_and_spelling_variants_collapse_to_one_enum_each(self):
        from lerobot.cameras.xense.configuration_xense import XenseTactileCameraConfig

        requested = tactile_camera_output_types(["rectify"], ["difference", "DIFFERENCE", "XenseOutputType.DIFFERENCE"])
        cfg = XenseTactileCameraConfig(serial_number="GSPS01A25Z0011", output_types=requested)

        values = [output_type.value for output_type in cfg.output_types]
        assert values == ["rectify", "difference"]

    def test_recorded_type_stays_first_after_normalisation(self):
        """The robots take ``output_types[1:]`` as the display-only views, so
        the recorded one has to survive normalisation in position 0."""
        from lerobot.cameras.xense.configuration_xense import XenseTactileCameraConfig

        cfg = XenseTactileCameraConfig(
            serial_number="GSPS01A25Z0011",
            output_types=tactile_camera_output_types(["rectify"], ["difference"]),
        )
        assert cfg.output_types[0].value == "rectify"
        assert [t.value for t in cfg.output_types[1:]] == ["difference"]


class TestTactileDisplayKey:
    def test_key_shape(self):
        assert tactile_display_key("tactile_left", "difference") == "tactile_left_difference"

    def test_bimanual_prefix_is_preserved(self):
        assert tactile_display_key("left_tactile_right", "difference") == "left_tactile_right_difference"


class TestSplitCameraRead:
    def test_plain_array_keeps_the_camera_name(self):
        frame = np.zeros((4, 4, 3), dtype=np.uint8)
        assert split_camera_read("wrist_cam", frame) == {"wrist_cam": frame}

    def test_multi_output_read_routes_display_types_to_their_own_keys(self):
        rectify = np.zeros((4, 4, 3), dtype=np.uint8)
        difference = np.ones((4, 4, 3), dtype=np.uint8)

        obs = split_camera_read(
            "tactile_left",
            {"rectify": rectify, "difference": difference},
            {"difference": "tactile_left_difference"},
        )

        assert obs["tactile_left"] is rectify
        assert obs["tactile_left_difference"] is difference

    def test_type_without_a_display_key_is_the_recorded_one(self):
        """Whatever is left over after the display map keeps the camera's own
        key — that is how the recorded stream is identified."""
        rectify = np.zeros((4, 4, 3), dtype=np.uint8)
        obs = split_camera_read("tactile_left", {"rectify": rectify}, {})
        assert obs == {"tactile_left": rectify}

    def test_no_display_map_collapses_every_type_onto_the_camera_key(self):
        obs = split_camera_read("tactile_left", {"rectify": np.zeros((2, 2, 3), dtype=np.uint8)})
        assert list(obs) == ["tactile_left"]


class FakeHeadCamera:
    def __init__(self, meta):
        self._meta = meta

    def last_frame_meta(self):
        return self._meta


class FakeLogger:
    def __init__(self):
        self.warnings = []
        self.infos = []
        self.errors = []

    def warn(self, msg):
        self.warnings.append(msg)

    def info(self, msg):
        self.infos.append(msg)

    def error(self, msg):
        self.errors.append(msg)

    def __getattr__(self, name):
        if name == "debug":
            return lambda *a, **k: None
        raise AttributeError(name)


class FakeCamera:
    """A camera that opens (or refuses to) and remembers whether it was closed."""

    def __init__(self, fail: BaseException | None = None):
        self.fail = fail
        self.is_connected = False
        self.disconnect_calls = 0

    def connect(self):
        if self.fail is not None:
            raise self.fail
        self.is_connected = True

    def disconnect(self):
        self.disconnect_calls += 1
        self.is_connected = False


class TestConnectCamerasParallelRollsBack:
    """One camera failing must not leave the others open.

    This is the failure that made a bad rig get worse the more you retried: the
    cameras that opened stayed open with nothing referencing them, and a process
    that then did not exit cleanly held their /dev nodes, so the next run died on
    ``VIDIOC_REQBUFS: Device or resource busy`` on a *different*, healthy camera.
    """

    def test_all_or_nothing_on_failure(self):
        good_a, good_b = FakeCamera(), FakeCamera()
        bad = FakeCamera(fail=ConnectionError("cannot open"))
        cameras = {"good_a": good_a, "bad": bad, "good_b": good_b}

        with pytest.raises(ConnectionError):
            connect_cameras_parallel(cameras, FakeLogger())

        assert not good_a.is_connected
        assert not good_b.is_connected
        assert good_a.disconnect_calls == 1
        assert good_b.disconnect_calls == 1

    def test_keyboard_interrupt_rolls_back_too(self):
        """Ctrl+C during startup is the most common way to hit this."""
        good = FakeCamera()
        interrupted = FakeCamera(fail=KeyboardInterrupt())
        cameras = {"good": good, "interrupted": interrupted}

        with pytest.raises(KeyboardInterrupt):
            connect_cameras_parallel(cameras, FakeLogger())

        assert not good.is_connected
        assert good.disconnect_calls == 1

    def test_success_leaves_every_camera_open(self):
        cameras = {"a": FakeCamera(), "b": FakeCamera()}
        connect_cameras_parallel(cameras, FakeLogger())
        assert all(cam.is_connected for cam in cameras.values())
        assert all(cam.disconnect_calls == 0 for cam in cameras.values())


class FakeEncoderSample:
    def __init__(self, position: float):
        self.position = position


class FakeEncoderGripper:
    """A gripper whose encoder can be told to start failing."""

    def __init__(self, position: float = 0.5):
        self.position = position
        self.failing = False
        self.encoder = self

    def read_once(self):
        if self.failing:
            raise OSError("SerialBus::write: Input/output error")
        return FakeEncoderSample(self.position)


class TestGripperReadGuard:
    """A jaw encoder that dies mid-episode must not keep reporting a jaw value.

    The old behaviour answered 0.0 on any failure — indistinguishable from a
    closed gripper — as both an observation and an action, for every remaining
    frame of the recording.
    """

    def test_reads_through_when_healthy(self):
        guard = GripperReadGuard(FakeLogger())
        assert guard.read("left", FakeEncoderGripper(0.75), 1.0) == pytest.approx(0.75)
        assert not guard.lost

    def test_a_single_failure_holds_the_last_value(self):
        guard = GripperReadGuard(FakeLogger())
        gripper = FakeEncoderGripper(0.75)
        guard.read("left", gripper, 1.0)

        gripper.failing = True
        held = guard.read("left", gripper, 1.0)

        assert held == pytest.approx(0.75), "a blip must not be reported as a closed jaw"
        assert not guard.lost, "one failed round-trip is not evidence of loss"

    def test_continuous_failure_trips_loss(self):
        guard = GripperReadGuard(FakeLogger(), timeout_s=0.05)
        gripper = FakeEncoderGripper(0.75)
        guard.read("left", gripper, 1.0)

        gripper.failing = True
        guard.read("left", gripper, 1.0)
        time.sleep(0.06)
        guard.read("left", gripper, 1.0)

        assert guard.lost
        assert guard.lost_grippers == frozenset({"left"})

    def test_failing_before_any_good_read_is_lost_at_once(self):
        """No last good value to hold, so there is nothing worth recording."""
        guard = GripperReadGuard(FakeLogger())
        gripper = FakeEncoderGripper()
        gripper.failing = True

        assert guard.read("left", gripper, 1.0) == 0.0
        assert guard.lost

    def test_recovery_clears_the_failure_window(self):
        guard = GripperReadGuard(FakeLogger(), timeout_s=0.05)
        gripper = FakeEncoderGripper(0.75)
        guard.read("left", gripper, 1.0)

        gripper.failing = True
        guard.read("left", gripper, 1.0)
        gripper.failing = False
        guard.read("left", gripper, 1.0)

        gripper.failing = True
        guard.read("left", gripper, 1.0)  # window restarts here, not at the earlier blip
        assert not guard.lost

    def test_loss_is_logged_once_not_every_frame(self):
        """The old per-frame warning interleaved with the observation table and
        made the terminal unreadable exactly when it mattered."""
        logger = FakeLogger()
        guard = GripperReadGuard(logger, timeout_s=0.0)
        gripper = FakeEncoderGripper()
        gripper.failing = True

        for _ in range(50):
            guard.read("left", gripper, 1.0)

        assert len(logger.errors) == 1

    def test_reset_clears_loss(self):
        guard = GripperReadGuard(FakeLogger(), timeout_s=0.0)
        gripper = FakeEncoderGripper()
        gripper.failing = True
        guard.read("left", gripper, 1.0)
        assert guard.lost

        guard.reset()
        assert not guard.lost
        assert guard.lost_grippers == frozenset()


class TestReadHeadCameraSkew:
    """The eyes are recorded as separate keys, so a mismatched stereo pair is
    invisible in the dataset. This is the only thing that surfaces it."""

    def test_matching_sequence_numbers_are_a_definitive_pair(self):
        cameras = {
            "left_head": FakeHeadCamera({"frame_sequence": 7, "timestamp_ns": 1_000_000_000}),
            "right_head": FakeHeadCamera({"frame_sequence": 7, "timestamp_ns": 1_500_000_000}),
        }
        # Negative means "same sequence", which settles it regardless of the
        # timestamps — those can differ for one capture.
        assert read_head_camera_skew(cameras, max_skew_ms=20.0) == -1.0

    def test_within_the_window_reports_zero(self):
        cameras = {
            "left_head": FakeHeadCamera({"frame_sequence": 7, "timestamp_ns": 1_000_000_000}),
            "right_head": FakeHeadCamera({"frame_sequence": 8, "timestamp_ns": 1_005_000_000}),
        }
        assert read_head_camera_skew(cameras, max_skew_ms=20.0) == 0.0

    def test_beyond_the_window_reports_the_skew_in_ms(self):
        cameras = {
            "left_head": FakeHeadCamera({"frame_sequence": 7, "timestamp_ns": 1_000_000_000}),
            "right_head": FakeHeadCamera({"frame_sequence": 8, "timestamp_ns": 1_050_000_000}),
        }
        assert read_head_camera_skew(cameras, max_skew_ms=20.0) == pytest.approx(50.0)

    def test_single_eye_recording_has_no_pair_to_check(self):
        cameras = {"left_head": FakeHeadCamera({"frame_sequence": 7, "timestamp_ns": 0})}
        assert read_head_camera_skew(cameras, max_skew_ms=20.0) is None

    def test_before_either_eye_has_a_frame(self):
        cameras = {"left_head": FakeHeadCamera(None), "right_head": FakeHeadCamera(None)}
        assert read_head_camera_skew(cameras, max_skew_ms=20.0) is None


class HeadConfig:
    """The ``--robot.head_camera_*`` surface both robots expose."""

    def __init__(self, eyes="both"):
        self.head_camera_eyes = eyes
        self.head_camera_width = 1024
        self.head_camera_height = 768
        self.head_camera_fps = 30
        self.head_camera_startup_timeout_s = 5.0
        self.head_camera_stale_after_s = 0.2
        self.head_camera_pair_max_skew_ms = 20.0


class TestBuildHeadCameraConfigs:
    def test_both_eyes_become_two_cameras(self):
        configs = build_head_camera_configs(HeadConfig("both"))
        assert set(configs) == {"left_head", "right_head"}

    def test_each_camera_records_a_single_eye(self):
        """One key per eye, not one merged double-width frame — so a consumer
        can take one eye without decoding both."""
        configs = build_head_camera_configs(HeadConfig("both"))
        assert configs["left_head"].eyes == "left"
        assert configs["right_head"].eyes == "right"
        # and therefore the recorded width is per-eye, not doubled
        assert configs["left_head"].frame_width == 1024

    @pytest.mark.parametrize("eye", ["left", "right"])
    def test_selecting_one_eye_builds_only_that_camera(self, eye):
        configs = build_head_camera_configs(HeadConfig(eye))
        assert set(configs) == {HEAD_CAMERA_KEYS[eye]}

    def test_config_fields_are_passed_through(self):
        configs = build_head_camera_configs(HeadConfig("left"))
        cfg = configs["left_head"]
        assert (cfg.width, cfg.height, cfg.fps) == (1024, 768, 30)
        assert cfg.pair_max_skew_ms == 20.0

    def test_unsupported_resolution_is_rejected_rather_than_rescaled(self):
        """A silent resize would change the recorded field of view with no
        trace in the dataset."""
        config = HeadConfig("left")
        config.head_camera_width = 1920
        config.head_camera_height = 1080
        with pytest.raises(ValueError, match="supports"):
            build_head_camera_configs(config)

    @pytest.mark.parametrize("size", [(640, 480), (1024, 768), (1280, 960)])
    def test_every_mode_the_headset_app_offers_is_accepted(self, size):
        """The app's Resolution setting has these three; a size it can emit
        must not be rejected here, or that setting becomes unusable."""
        config = HeadConfig("left")
        config.head_camera_width, config.head_camera_height = size
        assert (configs := build_head_camera_configs(config))["left_head"].width == size[0]
        assert configs["left_head"].height == size[1]


class TestHeadCameraDefaultResolution:
    """The default has to be the headset app's own default, or enabling the
    head camera fails on the first frame's size until someone finds the flags."""

    @pytest.mark.parametrize("config_class", [TaccapGripperConfig, BiTaccapGripperConfig], ids=["single", "bimanual"])
    def test_defaults_to_the_app_default_mode(self, config_class):
        assert (config_class.head_camera_width, config_class.head_camera_height) == (640, 480)

    def test_app_default_is_first_in_supported_modes(self):
        assert SUPPORTED_MODES[0] == (640, 480)


class TestHeadPoseKeys:
    def test_nine_dof_pose(self):
        """3 position + 6D rotation, the same layout as ``tcp.*``."""
        assert HEAD_POSE_KEYS == ("x", "y", "z", "r1", "r2", "r3", "r4", "r5", "r6")

    def test_head_pose_keys_is_the_shared_pose_layout(self):
        """``head_camera.*`` is remapped into the same world frame as ``tcp.*``
        and carries the same 9 components, so it uses the same tuple rather than
        a parallel copy that could drift."""
        assert HEAD_POSE_KEYS is POSE_KEYS


class TestHeadSkewMonitor:
    """Rate-limited counter around ``read_head_camera_skew``."""

    def _cameras(self, left_seq, left_ns, right_seq, right_ns):
        return {
            "left_head": FakeHeadCamera({"frame_sequence": left_seq, "timestamp_ns": left_ns}),
            "right_head": FakeHeadCamera({"frame_sequence": right_seq, "timestamp_ns": right_ns}),
        }

    def test_paired_frames_do_not_warn(self, monkeypatch):
        logger = FakeLogger()
        monitor = HeadSkewMonitor(20.0, logger)
        monitor.check(self._cameras(7, 0, 7, 5_000_000))
        assert monitor.skewed_frames == 0
        assert logger.warnings == []

    def test_skewed_frames_are_counted_and_warned(self):
        logger = FakeLogger()
        monitor = HeadSkewMonitor(20.0, logger)
        monitor.check(self._cameras(7, 0, 8, 50_000_000))
        assert monitor.skewed_frames == 1
        assert len(logger.warnings) == 1
        assert "50.0ms apart" in logger.warnings[0]

    def test_warnings_are_rate_limited_but_the_count_is_not(self, monkeypatch):
        """At 30 fps a persistent mismatch would otherwise emit 30 lines a
        second; the running total is what makes the one line useful."""
        clock = {"t": 1000.0}
        monkeypatch.setattr("lerobot.robots.taccap_gripper.common.time.monotonic", lambda: clock["t"])
        logger = FakeLogger()
        monitor = HeadSkewMonitor(20.0, logger)

        for _ in range(100):
            monitor.check(self._cameras(7, 0, 8, 50_000_000))
            clock["t"] += 0.01  # 100 frames over one second

        assert monitor.skewed_frames == 100
        assert len(logger.warnings) == 1

        clock["t"] += HeadSkewMonitor.WARN_INTERVAL_S + 0.1
        monitor.check(self._cameras(7, 0, 8, 50_000_000))
        assert len(logger.warnings) == 2
        assert "101 frames so far" in logger.warnings[1]

    def test_single_eye_never_warns(self):
        logger = FakeLogger()
        monitor = HeadSkewMonitor(20.0, logger)
        monitor.check({"left_head": FakeHeadCamera({"frame_sequence": 1, "timestamp_ns": 0})})
        assert logger.warnings == []


class TestBuildTactileCameraConfigs:
    """Both robots build their tactile cameras through this; the only difference
    is the key prefix."""

    DISCOVERED = {"left": "GSPS01A25Z0011", "right": "GSPS01A25Z0012"}

    def _build(self, key_prefix, **kwargs):
        return build_tactile_camera_configs(
            self.DISCOVERED,
            side="left",
            key_prefix=key_prefix,
            expected=2,
            fps=30,
            output_types=["rectify", "difference"],
            diff_gain=1.0,
            **kwargs,
        )

    def test_single_gripper_keys_are_unprefixed(self):
        configs, _ = self._build("")
        assert set(configs) == {"tactile_left", "tactile_right"}

    def test_bimanual_keys_carry_the_side_prefix(self):
        configs, _ = self._build("left_")
        assert set(configs) == {"left_tactile_left", "left_tactile_right"}

    def test_serials_land_on_the_finger_from_discovery(self):
        configs, _ = self._build("")
        assert configs["tactile_left"].serial_number == "GSPS01A25Z0011"
        assert configs["tactile_right"].serial_number == "GSPS01A25Z0012"

    def test_display_keys_cover_every_non_recorded_type(self):
        _, display_keys = self._build("left_")
        assert display_keys == {
            "left_tactile_left": {"difference": "left_tactile_left_difference"},
            "left_tactile_right": {"difference": "left_tactile_right_difference"},
        }

    def test_no_display_types_means_no_display_map(self):
        configs, display_keys = build_tactile_camera_configs(
            self.DISCOVERED,
            side="left",
            key_prefix="",
            expected=2,
            fps=30,
            output_types=["rectify"],
            diff_gain=1.0,
        )
        assert display_keys == {}
        assert len(configs) == 2

    def test_wrong_sensor_count_raises_naming_the_side(self):
        """Caught at construction, not mid-episode."""
        with pytest.raises(ValueError, match="Expected 2 left tactile sensors"):
            build_tactile_camera_configs(
                {"left": "GSPS01A25Z0011"},
                side="left",
                key_prefix="",
                expected=2,
                fps=30,
                output_types=["rectify"],
                diff_gain=1.0,
            )


class TestValidateRobotId:
    """``--robot.id`` is required on both TacCap configs. Upstream leaves it
    optional, which is how terminal output came to read ``None
    BiTaccapGripper`` and how a run could be recorded with nothing naming the
    rig it came from."""

    def test_a_station_label_passes_through(self):
        assert validate_robot_id("taccap_0", "taccap_gripper") == "taccap_0"

    def test_missing_id_names_the_flag_and_the_convention(self):
        with pytest.raises(ValueError, match="--robot.id is required"):
            validate_robot_id(None, "taccap_gripper")

    @pytest.mark.parametrize("blank", ["", "   ", "\t"])
    def test_blank_is_as_absent_as_none(self, blank):
        """``--robot.id=""`` would otherwise satisfy a bare None check and then
        name a rig nothing at all."""
        with pytest.raises(ValueError, match="--robot.id is required"):
            validate_robot_id(blank, "taccap_gripper")

    def test_surrounding_whitespace_is_stripped(self):
        """It reaches a calibration filename and the manifest; ``taccap_0 `` and
        ``taccap_0`` must not be two stations."""
        assert validate_robot_id("  taccap_0  ", "taccap_gripper") == "taccap_0"

    def test_the_format_itself_is_not_policed(self):
        """The convention is taccap_<n>, but identity lives in the serials, so a
        rig named after a room is allowed."""
        assert validate_robot_id("lab-b-bench-3", "taccap_gripper") == "lab-b-bench-3"

    @pytest.mark.parametrize(
        ("raw", "robot_type", "expected"),
        [
            ("0", "taccap_gripper", "taccap_0"),
            ("1", "taccap_gripper", "taccap_1"),
            ("12", "taccap_gripper", "taccap_12"),
            ("0", "bi_taccap_gripper", "bi_taccap_0"),
            ("3", "bi_taccap_gripper", "bi_taccap_3"),
        ],
    )
    def test_a_bare_number_is_expanded_against_the_type(self, raw, robot_type, expected):
        """``--robot.id=0`` is the whole point: the prefix repeats what
        ``--robot.type`` already said, and typing it by hand is how you end up
        with ``--robot.type=bi_taccap_gripper --robot.id=taccap_0``."""
        assert validate_robot_id(raw, robot_type) == expected

    def test_the_expansion_drops_gripper(self):
        """The label names a station, and a station is not a gripper — the
        gripper is one of the parts you swap out of it."""
        assert "gripper" not in validate_robot_id("0", "taccap_gripper")
        assert "gripper" not in validate_robot_id("0", "bi_taccap_gripper")

    def test_a_number_is_stripped_before_it_is_expanded(self):
        assert validate_robot_id("  2  ", "taccap_gripper") == "taccap_2"

    @pytest.mark.parametrize("raw", ["taccap_0", "bi_taccap_7", "lab-b-bench-3", "0a", "rig-0"])
    def test_anything_not_all_digits_is_taken_verbatim(self, raw):
        """Existing rigs pass the full label, and it reaches a calibration
        filename — expanding those would orphan every calibration on disk."""
        assert validate_robot_id(raw, "bi_taccap_gripper") == raw


class FakeEndpoints:
    """Stand-in for the SDK's ``GripperEndpoints``, which only connected
    hardware produces."""

    def __init__(self, firmware_sn):
        self.firmware_sn = firmware_sn
        self.mcu_serial = "0001"  # the CH343 adapter's — must never be recorded


class TestHardwareManifestUnit:
    """The dataset's only link back to the physical devices, so what goes in it
    and what each field means are both worth pinning."""

    TACTILES = {"left": "GSPS01A25Z0011", "right": "GSPS01A25Z0012"}

    def test_gripper_serial_is_the_firmware_one(self):
        """Not ``mcu_serial``: that identifies the USB-serial adapter and changes
        when the adapter does, so it is not the device's identity."""
        unit = hardware_manifest_unit(
            "left",
            endpoints=FakeEndpoints("TCGU01A24Z0001m"),
            tactile_serials=self.TACTILES,
            key_prefix="left_",
        )
        assert unit["gripper_sn"] == "TCGU01A24Z0001m"
        assert "0001" not in str(unit).replace("TCGU01A24Z0001m", "")

    def test_side_and_finger_are_recorded_separately(self):
        """They are independent left/rights — 单左双右 applied to the gripper's
        own serial and again to each tactile's — so a right-hand gripper carries
        a left finger like any other."""
        unit = hardware_manifest_unit(
            "right",
            endpoints=FakeEndpoints("TCGU01A24Z0002m"),
            tactile_serials=self.TACTILES,
            key_prefix="right_",
        )
        assert unit["side"] == "right"
        assert [s["finger"] for s in unit["tactile_sensors"]] == ["left", "right"]

    def test_each_tactile_carries_the_observation_key_it_feeds(self):
        """So a dataset column traces back to a sensor without re-deriving the
        naming rule."""
        unit = hardware_manifest_unit(
            "right",
            endpoints=FakeEndpoints("TCGU01A24Z0002m"),
            tactile_serials=self.TACTILES,
            key_prefix="right_",
        )
        assert [(s["observation_key"], s["serial"]) for s in unit["tactile_sensors"]] == [
            ("right_tactile_left", "GSPS01A25Z0011"),
            ("right_tactile_right", "GSPS01A25Z0012"),
        ]

    def test_single_gripper_keys_are_unprefixed(self):
        unit = hardware_manifest_unit(
            "left",
            endpoints=FakeEndpoints("TCGU01A24Z0001m"),
            tactile_serials=self.TACTILES,
            key_prefix="",
        )
        assert [s["observation_key"] for s in unit["tactile_sensors"]] == ["tactile_left", "tactile_right"]

    def test_disabled_or_disconnected_gripper_records_null_not_a_guess(self):
        """``_release()`` drops the endpoints, and ``enable_gripper=false`` never
        creates them; either way the honest answer is "no serial"."""
        unit = hardware_manifest_unit("left", endpoints=None, tactile_serials={}, key_prefix="left_")
        assert unit == {"side": "left", "gripper_sn": None, "tactile_sensors": []}


class TestWriteHardwareManifest:
    MANIFEST = build_hardware_manifest(
        robot_type="taccap_gripper",
        robot_id="taccap_0",
        role="leader",
        units=[
            hardware_manifest_unit(
                "left",
                endpoints=FakeEndpoints("TCGU01A24Z0001m"),
                tactile_serials={"left": "GSPS01A25Z0011", "right": "GSPS01A25Z0012"},
                key_prefix="",
            )
        ],
    )

    def _swapped(self, gripper_sn="TCGU01A24Z0003m"):
        other = json.loads(json.dumps(self.MANIFEST))
        other["units"][0]["gripper_sn"] = gripper_sn
        return other

    def test_written_under_meta_not_into_info_json(self, tmp_path):
        """``meta/info.json`` is upstream's schema; a fork-local key there would
        collide on the next v5.x sync."""
        path = write_hardware_manifest(tmp_path, self.MANIFEST, FakeLogger())
        assert path == tmp_path / HARDWARE_MANIFEST_PATH
        assert not (tmp_path / "meta" / "info.json").exists()

    def test_a_new_dataset_gets_one_open_epoch(self, tmp_path):
        """``to_episode`` stays null until something closes it: the episode count
        is only known once the run ends, and guessing would be worse than saying
        "still open"."""
        path = write_hardware_manifest(tmp_path, self.MANIFEST, FakeLogger())
        on_disk = json.loads(path.read_text())
        assert on_disk["robot_type"] == "taccap_gripper"
        # Dataset-level, beside robot_type: nothing may resume this dataset from
        # another station, so the label is an invariant and is stated as one.
        assert on_disk["robot_id"] == "taccap_0"
        (written,) = on_disk["epochs"]
        # Its shape is checked where it is the subject; dropped here so this
        # stays a test about the epoch boundary rather than about the clock.
        assert written.pop("recorded_at")
        assert written == {
            "from_episode": 0,
            "to_episode": None,
            "robot_id": "taccap_0",
            "role": "leader",
            "units": self.MANIFEST["units"],
        }

    def test_rewriting_the_same_hardware_is_a_silent_no_op(self, tmp_path):
        write_hardware_manifest(tmp_path, self.MANIFEST, FakeLogger())
        before = (tmp_path / HARDWARE_MANIFEST_PATH).read_text()

        logger = FakeLogger()
        write_hardware_manifest(tmp_path, self.MANIFEST, logger, episode_index=12)

        assert logger.warnings == []
        assert (tmp_path / HARDWARE_MANIFEST_PATH).read_text() == before

    def test_resuming_on_other_hardware_appends_an_epoch(self, tmp_path):
        """Both rigs stay named and every episode keeps pointing at the devices
        that produced it. Keeping only the first manifest (what this used to do)
        left the file quietly wrong for everything recorded after the swap."""
        write_hardware_manifest(tmp_path, self.MANIFEST, FakeLogger())

        write_hardware_manifest(tmp_path, self._swapped(), FakeLogger(), episode_index=57)

        epochs = json.loads((tmp_path / HARDWARE_MANIFEST_PATH).read_text())["epochs"]
        assert [(e["from_episode"], e["to_episode"]) for e in epochs] == [(0, 57), (57, None)]
        assert epochs[0]["units"][0]["gripper_sn"] == "TCGU01A24Z0001m"
        assert epochs[1]["units"][0]["gripper_sn"] == "TCGU01A24Z0003m"

    def test_a_swap_is_reported_but_is_not_a_warning(self, tmp_path):
        """It is now recorded rather than lost, so it is news, not a problem."""
        write_hardware_manifest(tmp_path, self.MANIFEST, FakeLogger())
        logger = FakeLogger()
        write_hardware_manifest(tmp_path, self._swapped(), logger, episode_index=57)
        assert logger.warnings == []

    def test_three_rigs_chain_without_gaps_or_overlap(self, tmp_path):
        """Bounds are half-open, so each episode belongs to exactly one epoch."""
        write_hardware_manifest(tmp_path, self.MANIFEST, FakeLogger())
        write_hardware_manifest(tmp_path, self._swapped("TCGU01A24Z0003m"), FakeLogger(), episode_index=57)
        write_hardware_manifest(tmp_path, self._swapped("TCGU01A24Z0004m"), FakeLogger(), episode_index=90)

        epochs = json.loads((tmp_path / HARDWARE_MANIFEST_PATH).read_text())["epochs"]
        assert [(e["from_episode"], e["to_episode"]) for e in epochs] == [(0, 57), (57, 90), (90, None)]

    def test_another_station_cannot_resume_the_dataset(self, tmp_path):
        """One dataset is one station. A task recorded across two rigs mixes
        their calibration, timing and mounting into episodes nothing downstream
        can separate — and identical ``units`` do not make it one rig, because
        the station is the seat, not the hardware in it."""
        write_hardware_manifest(tmp_path, self.MANIFEST, FakeLogger())
        before = (tmp_path / HARDWARE_MANIFEST_PATH).read_text()
        moved = json.loads(json.dumps(self.MANIFEST))
        moved["robot_id"] = "taccap_9"

        with pytest.raises(ValueError, match="taccap_0.*taccap_9"):
            write_hardware_manifest(tmp_path, moved, FakeLogger(), episode_index=57)

        # Refused before anything was written: the file still describes only the
        # episodes that are actually there.
        assert (tmp_path / HARDWARE_MANIFEST_PATH).read_text() == before

    def test_the_refusal_names_the_way_out(self, tmp_path):
        """The fix is a new dataset or the original rig, and the message has to
        say so — whoever hits this is mid-session with the hardware in hand."""
        write_hardware_manifest(tmp_path, self.MANIFEST, FakeLogger())
        moved = json.loads(json.dumps(self.MANIFEST))
        moved["robot_id"] = "taccap_9"

        with pytest.raises(ValueError) as excinfo:
            write_hardware_manifest(tmp_path, moved, FakeLogger(), episode_index=57)
        assert "repo_id" in str(excinfo.value)

    def test_the_station_check_precedes_the_swap_epoch(self, tmp_path):
        """A device swap that coincides with a move to another PC is still a
        refusal, not an epoch: the dataset is what is being protected, and it
        cannot span two stations however much else changed with them."""
        write_hardware_manifest(tmp_path, self.MANIFEST, FakeLogger())
        moved_and_swapped = self._swapped()
        moved_and_swapped["robot_id"] = "taccap_9"

        with pytest.raises(ValueError, match="taccap_9"):
            write_hardware_manifest(tmp_path, moved_and_swapped, FakeLogger(), episode_index=57)

    def test_the_station_label_survives_a_hardware_swap(self, tmp_path):
        """The top-level label describes the dataset, so appending an epoch must
        carry it over — the rewrite replaces the whole file."""
        write_hardware_manifest(tmp_path, self.MANIFEST, FakeLogger())
        write_hardware_manifest(tmp_path, self._swapped(), FakeLogger(), episode_index=57)

        on_disk = json.loads((tmp_path / HARDWARE_MANIFEST_PATH).read_text())
        assert on_disk["robot_id"] == "taccap_0"
        assert [e["robot_id"] for e in on_disk["epochs"]] == ["taccap_0", "taccap_0"]

    def test_an_unlabelled_dataset_can_still_be_resumed(self, tmp_path):
        """Recorded before ``--robot.id`` was required. The file cannot say the
        station changed, and refusing on that would strand real datasets — same
        reading as an open epoch: "nothing here says it changed"."""
        legacy = json.loads(json.dumps(self.MANIFEST))
        legacy["robot_id"] = None
        write_hardware_manifest(tmp_path, legacy, FakeLogger())

        logger = FakeLogger()
        write_hardware_manifest(tmp_path, self.MANIFEST, logger, episode_index=57)

        on_disk = json.loads((tmp_path / HARDWARE_MANIFEST_PATH).read_text())
        assert on_disk["robot_id"] == "taccap_0"
        assert [e["robot_id"] for e in on_disk["epochs"]] == [None, "taccap_0"]
        assert logger.warnings == []

    def test_swapping_one_arm_carries_the_other_over_untouched(self, tmp_path):
        """The two grippers are swapped independently — a customer may replace
        only the left one. The new epoch has to describe the *whole* rig, or the
        arm that did not change would have no serial for those episodes."""
        bimanual = build_hardware_manifest(
            robot_type="bi_taccap_gripper",
            robot_id="bi_taccap_0",
            role="leader",
            units=[
                hardware_manifest_unit(
                    side,
                    endpoints=FakeEndpoints(f"TCGU01A24Z000{n}m"),
                    tactile_serials={"left": f"GSPS01A25Z00{n}1", "right": f"GSPS01A25Z00{n}2"},
                    key_prefix=f"{side}_",
                )
                for n, side in enumerate(("left", "right"), start=1)
            ],
        )
        write_hardware_manifest(tmp_path, bimanual, FakeLogger())

        swapped_left = json.loads(json.dumps(bimanual))
        swapped_left["units"][0]["gripper_sn"] = "TCGU01A24Z0009m"
        logger = FakeLogger()
        write_hardware_manifest(tmp_path, swapped_left, logger, episode_index=40)

        epochs = json.loads((tmp_path / HARDWARE_MANIFEST_PATH).read_text())["epochs"]
        assert len(epochs) == 2
        assert epochs[1]["units"][0]["gripper_sn"] == "TCGU01A24Z0009m"  # the swap
        assert epochs[1]["units"][1] == bimanual["units"][1]  # right arm carried over whole
        assert logger.warnings == []

    def test_a_different_robot_type_is_kept_apart_not_appended(self, tmp_path):
        """Single vs bimanual changes the observation keys, so it is not the same
        dataset at all — epochs do not model that."""
        write_hardware_manifest(tmp_path, self.MANIFEST, FakeLogger())
        other_type = json.loads(json.dumps(self.MANIFEST))
        other_type["robot_type"] = "bi_taccap_gripper"

        logger = FakeLogger()
        write_hardware_manifest(tmp_path, other_type, logger, episode_index=57)

        on_disk = json.loads((tmp_path / HARDWARE_MANIFEST_PATH).read_text())
        assert on_disk["robot_type"] == "taccap_gripper"
        assert len(on_disk["epochs"]) == 1
        assert len(logger.warnings) == 1
        assert "bi_taccap_gripper" in logger.warnings[0]

    def test_an_unreadable_manifest_is_left_alone(self, tmp_path):
        """A truncated file is someone else's problem to fix; clobbering it would
        destroy the provenance of whatever episodes are already there."""
        path = tmp_path / HARDWARE_MANIFEST_PATH
        path.parent.mkdir(parents=True)
        path.write_text("{ truncated")

        logger = FakeLogger()
        write_hardware_manifest(tmp_path, self.MANIFEST, logger)

        assert path.read_text() == "{ truncated"
        assert len(logger.warnings) == 1


class TestStationCheck:
    """One dataset, one station. The label is checked off ``--robot.id`` alone,
    so this can run before a single device is powered up."""

    MANIFEST = build_hardware_manifest(
        robot_type="taccap_gripper",
        robot_id="taccap_0",
        role="leader",
        units=[],
    )

    def test_a_dataset_with_no_manifest_is_not_a_mismatch(self, tmp_path):
        """Recorded before manifests existed, or not recorded yet at all. There
        is no station on record to disagree with."""
        check_dataset_station(tmp_path, "taccap_0")

    def test_the_same_station_passes(self, tmp_path):
        write_hardware_manifest(tmp_path, self.MANIFEST, FakeLogger())
        check_dataset_station(tmp_path, "taccap_0")

    def test_another_station_raises_before_any_hardware_is_touched(self, tmp_path):
        write_hardware_manifest(tmp_path, self.MANIFEST, FakeLogger())
        with pytest.raises(ValueError, match="taccap_1"):
            check_dataset_station(tmp_path, "taccap_1")

    def test_an_unreadable_manifest_is_left_to_the_writer(self, tmp_path):
        """It complains about a truncated file already. Raising here too would
        turn one problem into two errors, and the earlier one would blame the
        station for a mismatch nobody can read."""
        path = tmp_path / HARDWARE_MANIFEST_PATH
        path.parent.mkdir(parents=True)
        path.write_text("{ truncated")

        check_dataset_station(tmp_path, "taccap_1")

    def test_a_pre_epoch_manifest_is_still_enforced(self, tmp_path):
        """Old shape, no ``epochs`` — the label sits at the top level, which is
        where the check has to find it or it silently passes everything."""
        path = tmp_path / HARDWARE_MANIFEST_PATH
        path.parent.mkdir(parents=True)
        path.write_text(json.dumps({"robot_type": "taccap_gripper", "robot_id": "taccap_0", "units": []}))

        with pytest.raises(ValueError, match="taccap_0"):
            check_dataset_station(tmp_path, "taccap_1")

    def test_an_epoch_only_label_is_still_enforced(self, tmp_path):
        """Written after epochs but before the top-level key. The label is only
        on the epochs, and a check that read the top level alone would let
        another rig resume every dataset recorded in that window."""
        path = tmp_path / HARDWARE_MANIFEST_PATH
        path.parent.mkdir(parents=True)
        path.write_text(
            json.dumps(
                {
                    "robot_type": "taccap_gripper",
                    "epochs": [{"from_episode": 0, "to_episode": None, "robot_id": "taccap_0", "units": []}],
                }
            )
        )

        with pytest.raises(ValueError, match="taccap_0"):
            check_dataset_station(tmp_path, "taccap_1")

    def test_a_run_with_no_label_of_its_own_cannot_disagree(self, tmp_path):
        """``--robot.id`` is required of both TacCap robots, so this is another
        caller. Unknown is not a mismatch, in either direction."""
        write_hardware_manifest(tmp_path, self.MANIFEST, FakeLogger())
        check_dataset_station(tmp_path, None)


class TestManifestRobotIds:
    def test_both_places_are_read_and_blanks_dropped(self):
        """Absent is the absence of an answer, not a station to compare against;
        reporting it as ``None`` would make an unlabelled epoch look like a
        second rig."""
        manifest = {
            "robot_id": "taccap_0",
            "epochs": [
                {"robot_id": None, "units": []},
                {"robot_id": "taccap_0", "units": []},
                {"units": []},
            ],
        }
        assert manifest_robot_ids(manifest) == {"taccap_0"}

    def test_a_dataset_that_predates_the_label_names_no_station(self):
        assert manifest_robot_ids({"robot_type": "taccap_gripper", "units": []}) == set()


class TestManifestEpochs:
    """Reading side. Old files have no epochs at all, so this is where a wrong
    assumption turns into an episode attributed to the wrong sensor."""

    UNITS_A = [{"side": "left", "gripper_sn": "A", "tactile_sensors": []}]
    UNITS_B = [{"side": "left", "gripper_sn": "B", "tactile_sensors": []}]

    def test_a_pre_epoch_manifest_reads_as_one_open_epoch(self):
        """Those datasets are real and mostly single-rig; rejecting them would
        strand every recording made before epochs existed."""
        legacy = {
            "robot_type": "taccap_gripper",
            "robot_id": "taccap_0",
            "role": "leader",
            "units": self.UNITS_A,
        }
        assert manifest_epochs(legacy) == [
            {
                "from_episode": 0,
                "to_episode": None,
                "robot_id": "taccap_0",  # lifted from the top level
                "role": "leader",
                "units": self.UNITS_A,
            }
        ]

    def test_an_open_epoch_claims_every_later_episode(self):
        legacy = {"units": self.UNITS_A}
        assert epoch_for_episode(legacy, 0)["units"] == self.UNITS_A
        assert epoch_for_episode(legacy, 10_000)["units"] == self.UNITS_A

    def test_each_episode_resolves_to_the_rig_that_recorded_it(self):
        manifest = {
            "epochs": [
                {"from_episode": 0, "to_episode": 57, "units": self.UNITS_A},
                {"from_episode": 57, "to_episode": None, "units": self.UNITS_B},
            ]
        }
        assert epoch_for_episode(manifest, 56)["units"] == self.UNITS_A
        assert epoch_for_episode(manifest, 57)["units"] == self.UNITS_B  # half-open
        assert epoch_for_episode(manifest, 1_000)["units"] == self.UNITS_B

    def test_an_episode_no_epoch_claims_is_none_not_the_nearest_rig(self):
        """Past the last closed epoch nobody recorded that hardware. Falling back
        to the nearest rig would attribute it to devices that did not produce it
        — which is the failure this whole mechanism exists to prevent."""
        closed = {"epochs": [{"from_episode": 0, "to_episode": 57, "units": self.UNITS_A}]}
        assert epoch_for_episode(closed, 57) is None


class TestTactileSerialForKey:
    """The lookup post-processing actually performs: which sensor fed this video
    stream in this episode. Getting it wrong is invisible in the output, so the
    unanswerable cases matter as much as the answerable ones."""

    MANIFEST = {
        "epochs": [
            {
                "from_episode": 0,
                "to_episode": 40,
                "units": [
                    {
                        "side": "left",
                        "tactile_sensors": [{"observation_key": "left_tactile_left", "serial": "GSPS01A30Z0015"}],
                    }
                ],
            },
            {
                "from_episode": 40,
                "to_episode": None,
                "units": [
                    {
                        "side": "left",
                        "tactile_sensors": [{"observation_key": "left_tactile_left", "serial": "GSPS01A30Z0088"}],
                    }
                ],
            },
        ]
    }

    def test_the_same_key_resolves_to_different_sensors_across_a_swap(self):
        assert tactile_serial_for_key(self.MANIFEST, 39, "left_tactile_left") == "GSPS01A30Z0015"
        assert tactile_serial_for_key(self.MANIFEST, 40, "left_tactile_left") == "GSPS01A30Z0088"

    def test_an_unknown_key_is_none_not_the_first_sensor(self):
        """Falling back to "some sensor on that rig" is exactly the misattribution
        this file exists to prevent."""
        assert tactile_serial_for_key(self.MANIFEST, 0, "right_tactile_left") is None

    def test_an_episode_outside_every_epoch_is_none(self):
        closed = {"epochs": [{"from_episode": 0, "to_episode": 10, "units": self.MANIFEST["epochs"][0]["units"]}]}
        assert tactile_serial_for_key(closed, 10, "left_tactile_left") is None

    def test_a_pre_epoch_manifest_still_answers(self):
        """Those datasets are the majority today; the lookup has to work on them."""
        legacy = {"robot_type": "taccap_gripper", "units": self.MANIFEST["epochs"][0]["units"]}
        assert tactile_serial_for_key(legacy, 9999, "left_tactile_left") == "GSPS01A30Z0015"


class TestSwapTactileDisplayFeatures:
    def test_recorded_tactile_key_is_replaced_not_added(self):
        """Same tile count in Rerun, and the recorded stream never reaches the
        viewer."""
        observation = {"tcp.x": float, "tactile_left": (4, 6, 3), "wrist_cam": (8, 8, 3)}
        display = swap_tactile_display_features(
            observation, {"tactile_left": {"difference": "tactile_left_difference"}}
        )
        assert set(display) == {"tcp.x", "tactile_left_difference", "wrist_cam"}
        assert display["tactile_left_difference"] == (4, 6, 3)

    def test_cameras_without_a_display_view_pass_through(self):
        observation = {"tactile_left": (4, 6, 3)}
        assert swap_tactile_display_features(observation, {}) == observation

    def test_several_display_views_all_appear(self):
        observation = {"tactile_left": (4, 6, 3)}
        display = swap_tactile_display_features(
            observation,
            {"tactile_left": {"difference": "tactile_left_difference", "depth": "tactile_left_depth"}},
        )
        assert set(display) == {"tactile_left_difference", "tactile_left_depth"}

    def test_key_order_is_preserved(self):
        """Rerun lays tiles out in schema order; a swap must keep tactile in the
        same slot of the blueprint."""
        observation = {"a": float, "tactile_left": (4, 6, 3), "z": float}
        display = swap_tactile_display_features(
            observation, {"tactile_left": {"difference": "tactile_left_difference"}}
        )
        assert list(display) == ["a", "tactile_left_difference", "z"]


class TestTrackerToTcp:
    @pytest.mark.parametrize("side", ["left", "right"])
    def test_returns_a_position_and_a_unit_quaternion(self, side):
        pos, quat = tracker_to_tcp(side)
        assert pos.shape == (3,)
        assert quat.shape == (4,)
        assert np.linalg.norm(quat) == pytest.approx(1.0)

    def test_sides_are_measured_separately_not_mirrored(self):
        """Both sides carry their own measured values; a change that starts
        deriving one from the other would make these identical up to a sign."""
        left_pos, _ = tracker_to_tcp("left")
        right_pos, _ = tracker_to_tcp("right")
        assert not np.allclose(left_pos, right_pos)
        assert not np.allclose(left_pos, right_pos * np.array([1.0, -1.0, 1.0]))

    def test_unknown_side_raises(self):
        with pytest.raises(ValueError, match="side must be one of"):
            tracker_to_tcp("middle")

    def test_side_is_case_and_space_insensitive(self):
        assert np.allclose(tracker_to_tcp("  LEFT ")[0], tracker_to_tcp("left")[0])


class TestResolveTrackerToEe:
    def test_none_means_use_the_built_in_transform(self):
        built_in = tracker_to_tcp("left")
        pos, quat = resolve_tracker_to_ee("left", None, None)
        assert np.allclose(pos, built_in[0])
        assert np.allclose(quat, built_in[1])

    def test_position_can_be_overridden_alone(self):
        """A rig with a re-machined mount pins just the translation."""
        built_in_quat = tracker_to_tcp("left")[1]
        pos, quat = resolve_tracker_to_ee("left", (0.1, 0.2, 0.3), None)
        assert np.allclose(pos, [0.1, 0.2, 0.3])
        assert np.allclose(quat, built_in_quat)

    def test_rotation_can_be_overridden_alone(self):
        built_in_pos = tracker_to_tcp("left")[0]
        pos, quat = resolve_tracker_to_ee("left", None, (1.0, 0.0, 0.0, 0.0))
        assert np.allclose(pos, built_in_pos)
        assert np.allclose(quat, [1.0, 0.0, 0.0, 0.0])

    def test_overrides_are_returned_as_float_arrays(self):
        pos, quat = resolve_tracker_to_ee("right", [0.1, 0.2, 0.3], [1, 0, 0, 0])
        assert pos.dtype == np.float64
        assert quat.dtype == np.float64


class FakeGripper:
    """Stands in for LeaderGripper / FollowerGripper.

    ``normalize_position=True`` is what asks the firmware for its stored travel
    span; ``has_encoder_max=False`` reproduces a unit that never had one stored
    (or firmware older than V2.1), where the real SDK constructor raises.
    """

    def __init__(self, mcu_device, normalize_position=False, encoder_max_rad=None, *, has_encoder_max=True):
        if normalize_position and not has_encoder_max and encoder_max_rad is None:
            raise ValueError("Cmd::EncoderMaxCal returned no calibration")
        self.mcu_device = mcu_device
        self.normalize_position = normalize_position
        self.encoder_max_rad = encoder_max_rad

    @classmethod
    def uncalibrated(cls):
        """A constructor that refuses `normalize_position=True`, as the SDK does."""
        return functools.partial(cls, has_encoder_max=False)


class TestOpenGripper:
    """A leader with no stored travel span must stop the session.

    It used to warn and divide by the config constant instead. That number is
    one value for every unit ever built, so a gripper whose real travel differs
    records a `gripper.pos` that never reaches 1.0 — indistinguishable
    downstream from a jaw the operator never opened all the way. The failure is
    silent and only shows up once someone trains on the data, which is why it
    is worth refusing to start.
    """

    def test_calibrated_leader_normalises_from_firmware(self):
        logger = FakeLogger()
        gripper, source = open_gripper(FakeGripper, "/dev/ttyACM0", is_leader=True, open_rad=1.7, logger=logger)
        assert source == "firmware"
        assert gripper.normalize_position is True
        # the host constant must not be handed to a calibrated leader
        assert gripper.encoder_max_rad is None

    def test_uncalibrated_leader_raises(self):
        logger = FakeLogger()
        with pytest.raises(RuntimeError) as excinfo:
            open_gripper(FakeGripper.uncalibrated(), "/dev/ttyACM0", is_leader=True, open_rad=1.7, logger=logger)
        assert "no encoder-max calibration" in str(excinfo.value)

    def test_the_error_names_the_calibration_command(self):
        """The operator has to be able to act on it without reading the source."""
        with pytest.raises(RuntimeError) as excinfo:
            open_gripper(FakeGripper.uncalibrated(), "/dev/ttyACM0", is_leader=True, open_rad=1.7, logger=FakeLogger())
        message = str(excinfo.value)
        assert "calibrate.py <left|right>" in message
        assert "EncoderMaxCal" in message
        assert "V2.1" in message  # the pre-V2.1 firmware case needs an OTA first

    def test_the_error_carries_the_side_label_and_the_original_cause(self):
        with pytest.raises(RuntimeError) as excinfo:
            open_gripper(
                FakeGripper.uncalibrated(),
                "/dev/ttyACM0",
                is_leader=True,
                open_rad=1.7,
                logger=FakeLogger(),
                label="[left] ",
            )
        assert str(excinfo.value).startswith("[left] ")
        # chained, so the SDK's own message is not lost
        assert isinstance(excinfo.value.__cause__, ValueError)
        assert "EncoderMaxCal" in str(excinfo.value.__cause__)

    def test_it_does_not_silently_fall_back_to_the_config_constant(self):
        """Guards the specific regression: re-opening with encoder_max_rad set."""
        with pytest.raises(RuntimeError):
            open_gripper(FakeGripper.uncalibrated(), "/dev/ttyACM0", is_leader=True, open_rad=1.7, logger=FakeLogger())

    def test_follower_still_normalises_from_the_config_constant(self):
        """EncoderMaxCal is leader-only and the follower class does not take the
        flag, so followers are unaffected by the stricter leader path."""
        logger = FakeLogger()
        gripper, source = open_gripper(FakeGripper, "/dev/ttyACM0", is_leader=False, open_rad=1.7, logger=logger)
        assert source == "config"
        assert gripper.normalize_position is False
        assert any("gripper_open_rad=1.7" in m for m in logger.infos)


# --------------------------------------------------------- wrist fisheye undistort


class FakeCalibration:
    """``gripper.calibration``, whose ``resolve_fisheye`` the SDK implements by
    reading flash. Returns the SDK's own tuple shape."""

    def __init__(self, cal, is_reference, reason):
        self._answer = (cal, is_reference, reason)
        self.calls = 0

    def resolve_fisheye(self, *args, **kwargs):
        self.calls += 1
        return self._answer


class FakeUndistortGripper:
    def __init__(self, calibration):
        self.calibration = calibration


class FakeUndistorter:
    """Stand-in for the SDK's ``FisheyeUndistorter``, recording what it was
    handed. The focal-length maths behind ``balance`` belongs to the SDK and is
    tested there; what matters here is which calibration and size reach it."""

    def __init__(self, calibration, width, height, balance):
        self.calibration = calibration
        self.width = width
        self.height = height
        self.balance = balance
        self.focal_scale = 1.0


UNIT_CAL = object()  # this gripper's own, read from flash
REFERENCE_CAL = object()  # the SDK's FISHEYE_FALLBACK_CAL


def _resolve(gripper, **kwargs):
    kwargs.setdefault("balance", 0.0)
    kwargs.setdefault("logger", FakeLogger())
    return resolve_wrist_undistorter(gripper, undistorter_cls=FakeUndistorter, fallback_cal=REFERENCE_CAL, **kwargs)


class TestResolveWristUndistorter:
    """Which intrinsics a recording rectified with is not visible in the frames,
    so the source this reports is what the manifest — and any later audit — has
    to go on."""

    def test_this_units_own_calibration_is_used_without_warning(self):
        cal = FakeCalibration(UNIT_CAL, False, "")
        logger = FakeLogger()
        undistorter, source = _resolve(FakeUndistortGripper(cal), logger=logger)
        assert source == "unit"
        assert cal.calls == 1
        assert undistorter.calibration is UNIT_CAL
        assert logger.warnings == []
        # The firmware record carries no image size, so this is the only one the
        # intrinsics describe — see validate_wrist_undistort_size.
        assert (undistorter.width, undistorter.height) == (640, 480)

    def test_reference_fallback_warns_and_says_why(self):
        """The fallback is deliberately not an error, so the warning is the only
        thing standing between an approximate rectification and a whole dataset
        recorded without anyone noticing."""
        logger = FakeLogger()
        undistorter, source = _resolve(
            FakeUndistortGripper(FakeCalibration(REFERENCE_CAL, True, "the wrist lens has never been calibrated")),
            logger=logger,
        )
        assert source == "reference"
        assert undistorter.calibration is REFERENCE_CAL
        assert len(logger.warnings) == 1
        warning = logger.warnings[0]
        assert "REFERENCE" in warning
        assert "never been calibrated" in warning  # the SDK's reason is passed through
        assert "fisheye_cal.py set-fisheye" in warning  # and how to fix it

    def test_no_gripper_means_no_calibration_to_read(self):
        """``--robot.enable_gripper=false`` leaves no MCU to ask. That is the
        reference case too, but for a reason the SDK never sees, so we supply it."""
        logger = FakeLogger()
        undistorter, source = _resolve(None, logger=logger)
        assert source == "reference"
        assert undistorter.calibration is REFERENCE_CAL
        assert "enable_gripper" in logger.warnings[0]

    def test_balance_reaches_the_undistorter(self):
        undistorter, _ = _resolve(FakeUndistortGripper(FakeCalibration(UNIT_CAL, False, "")), balance=1.0)
        assert undistorter.balance == pytest.approx(1.0)


class TestValidateWristUndistortSize:
    def test_the_calibrated_size_passes(self):
        validate_wrist_undistort_size(640, 480)

    def test_any_other_size_is_refused_at_parse_time(self):
        """The firmware record carries no image size, so another resolution means
        guessing a scale factor and rectifying wrongly with nothing in the frames
        to show it."""
        with pytest.raises(ValueError) as excinfo:
            validate_wrist_undistort_size(1280, 960)
        assert "640x480" in str(excinfo.value)
        assert "1280x960" in str(excinfo.value)


class TestWristUndistortRecord:
    def test_no_wrist_camera_omits_the_key_entirely(self):
        """Absent means the question does not apply, not that the answer is no."""
        assert wrist_undistort_record(has_wrist_camera=False, source=None, balance=0.0) is None

    def test_a_wrist_camera_always_records_an_answer(self):
        """Including "we did not rectify" — absent and false would otherwise be
        indistinguishable in a recorded dataset."""
        assert wrist_undistort_record(has_wrist_camera=True, source=None, balance=0.0) == {"applied": False}

    def test_applied_records_which_calibration_and_the_balance(self):
        assert wrist_undistort_record(has_wrist_camera=True, source="reference", balance=0.5) == {
            "applied": True,
            "calibration": "reference",
            "balance": 0.5,
        }


class FrozenCamera:
    """A stream that keeps handing back the *same* object — how an unplugged
    Xense sensor behaves, since its read thread survives the error."""

    def __init__(self):
        self.frame = np.zeros((480, 640, 3), dtype=np.uint8)

    def async_read(self):
        return self.frame


class RectifiedFrozenCamera(FrozenCamera):
    """The same dead stream, but with rectification applied *before* the guard
    sees it — a fresh array every call, exactly like ``FisheyeUndistorter.apply``.
    """

    def async_read(self):
        return self.frame.copy()


class TestFreezeDetectionConstrainsWhereUndistortionGoes:
    """Why ``get_observation`` rectifies *after* ``CameraReadGuard.read``.

    The guard spots a dead-but-not-raising stream by frame object identity, so
    anything that hands it a fresh array each call — rectifying upstream by
    wrapping the camera, say — makes a frozen wrist camera undetectable and the
    recording silently keeps writing stale frames.
    """

    def test_a_frozen_stream_is_flagged(self):
        guard = CameraReadGuard({}, FakeLogger(), freeze_timeout_s=0.0)
        camera = FrozenCamera()
        guard.read("wrist_cam", camera)  # first read establishes the baseline
        guard.read("wrist_cam", camera)
        assert guard.lost

    def test_a_fresh_array_each_call_hides_the_same_dead_stream(self):
        """Not a feature — the hazard the ordering avoids. If this ever stops
        holding, the comment in get_observation can go."""
        guard = CameraReadGuard({}, FakeLogger(), freeze_timeout_s=0.0)
        camera = RectifiedFrozenCamera()
        for _ in range(5):
            guard.read("wrist_cam", camera)
        assert not guard.lost


class TestWristUndistortOpensItsOwnEpoch:
    """Rectified and raw frames are indistinguishable in the dataset, so a change
    of setting part-way through has to be recorded as its own epoch."""

    def _manifest(self, **wrist):
        return build_hardware_manifest(
            robot_type="taccap_gripper",
            robot_id="taccap_0",
            role="leader",
            units=[
                hardware_manifest_unit(
                    "left",
                    endpoints=FakeEndpoints("TCGU01A24Z0001m"),
                    tactile_serials={"left": "GSPS01A25Z0011"},
                    key_prefix="",
                    wrist_undistort=wrist_undistort_record(**wrist),
                )
            ],
        )

    def test_turning_it_on_mid_dataset_closes_the_old_epoch(self, tmp_path):
        off = self._manifest(has_wrist_camera=True, source=None, balance=0.0)
        on = self._manifest(has_wrist_camera=True, source="unit", balance=0.0)

        write_hardware_manifest(tmp_path, off, FakeLogger())
        path = write_hardware_manifest(tmp_path, on, FakeLogger(), episode_index=7)

        epochs = json.loads(path.read_text())["epochs"]
        assert len(epochs) == 2
        assert epochs[0]["to_episode"] == 7
        assert epochs[0]["units"][0]["wrist_undistort"]["applied"] is False
        assert epochs[1]["units"][0]["wrist_undistort"]["applied"] is True

    def test_changing_only_the_balance_also_opens_one(self, tmp_path):
        """It changes the geometry of every recorded frame, so it is a different
        configuration even though the hardware is identical."""
        write_hardware_manifest(
            tmp_path, self._manifest(has_wrist_camera=True, source="unit", balance=0.0), FakeLogger()
        )
        path = write_hardware_manifest(
            tmp_path,
            self._manifest(has_wrist_camera=True, source="unit", balance=1.0),
            FakeLogger(),
            episode_index=4,
        )
        assert len(json.loads(path.read_text())["epochs"]) == 2

    def test_an_unchanged_setting_does_not(self, tmp_path):
        same = self._manifest(has_wrist_camera=True, source="unit", balance=0.0)
        write_hardware_manifest(tmp_path, same, FakeLogger())
        path = write_hardware_manifest(tmp_path, same, FakeLogger(), episode_index=9)
        assert len(json.loads(path.read_text())["epochs"]) == 1


class FakeStreamSample:
    def __init__(self, position: float, position_rad: float = float("nan")):
        self.position = position
        self.position_rad = position_rad


class FakeImuSample:
    def __init__(self, seq: int):
        import numpy as np

        self.accel_mps2 = np.array([seq, 0.0, 9.8], dtype=np.float32)
        self.gyro_radps = np.zeros(3, dtype=np.float32)
        self.mag_uT = np.zeros(3, dtype=np.float32)


class FakeStreamingGripper:
    """A leader whose firmware can push encoder (and IMU) samples.

    ``start_streaming`` delivers one sample of each source synchronously, the
    way a real stream produces its first frame within a few milliseconds; the
    test then pushes more by hand. ``read_once`` counts calls so a test can
    prove the bus was not touched.
    """

    def __init__(self, position: float = 0.5, fail_start: bool = False, silent: bool = False):
        self.position = position
        self.fail_start = fail_start
        self.silent = silent
        self.is_streaming = False
        self.stopped = 0
        self.read_once_calls = 0
        self.imu_read_once_calls = 0
        self.encoder = self._Encoder(self)
        self.imu = self._Imu(self)
        self.rates: tuple[int, int] | None = None

    class _Component:
        def __init__(self, outer):
            self.outer = outer
            self.callbacks: dict[int, object] = {}
            self._next = 1

        def on_data(self, cb):
            sub_id = self._next
            self._next += 1
            self.callbacks[sub_id] = cb
            return sub_id

        def off(self, sub_id):
            self.callbacks.pop(sub_id)

    class _Encoder(_Component):
        def read_once(self):
            self.outer.read_once_calls += 1
            return FakeStreamSample(self.outer.position)

        def push(self, position: float):
            for cb in self.callbacks.values():
                cb(FakeStreamSample(position))

    class _Imu(_Component):
        def read_once(self):
            self.outer.imu_read_once_calls += 1
            return FakeImuSample(-1)

        def push(self, seq: int):
            for cb in self.callbacks.values():
                cb(FakeImuSample(seq))

    def start_streaming(self, imu_hz: int, encoder_hz: int):
        if self.fail_start:
            raise OSError("StartStream NACK")
        self.rates = (imu_hz, encoder_hz)
        self.is_streaming = True
        if not self.silent:
            self.encoder.push(self.position)
            if imu_hz > 0:
                self.imu.push(0)

    def stop_streaming(self):
        self.is_streaming = False
        self.stopped += 1


class TestGripperReadGuardStream:
    """A streamed gripper must keep the record loop off the bus — and keep the
    polled path's loss semantics, translated to "the stream went silent"."""

    def test_reads_come_from_the_stream_not_the_bus(self):
        guard = GripperReadGuard(FakeLogger())
        gripper = FakeStreamingGripper(0.25)

        assert guard.subscribe("left", gripper, encoder_hz=100) is True
        assert gripper.rates == (0, 100), "IMU source must be OFF when not asked for"

        assert guard.read("left", gripper, 1.0) == pytest.approx(0.25)
        gripper.encoder.push(0.75)
        assert guard.read("left", gripper, 1.0) == pytest.approx(0.75)
        assert gripper.read_once_calls == 0, "a streamed gripper must never be polled"
        assert not guard.lost

    def test_position_rad_backstop_applies_to_streamed_samples(self):
        guard = GripperReadGuard(FakeLogger())
        gripper = FakeStreamingGripper()
        guard.subscribe("left", gripper, encoder_hz=100)

        for cb in gripper.encoder.callbacks.values():
            cb(FakeStreamSample(float("nan"), position_rad=0.85))
        assert guard.read("left", gripper, 1.7) == pytest.approx(0.5)

    def test_a_silent_stream_holds_then_trips_loss(self):
        import time

        logger = FakeLogger()
        guard = GripperReadGuard(logger, timeout_s=0.05)
        gripper = FakeStreamingGripper(0.6)
        guard.subscribe("left", gripper, encoder_hz=100)
        assert guard.read("left", gripper, 1.0) == pytest.approx(0.6)

        time.sleep(0.08)  # longer than timeout_s with nothing pushed
        held = guard.read("left", gripper, 1.0)

        assert held == pytest.approx(0.6), "silence degrades to the last good value, never to 0.0"
        assert guard.lost, "silence spanning timeout_s is loss, immediately — the clock started at the last sample"
        assert "left" in guard.lost_grippers

    def test_stream_failure_falls_back_to_polling(self):
        logger = FakeLogger()
        guard = GripperReadGuard(logger)
        gripper = FakeStreamingGripper(0.4, fail_start=True)

        assert guard.subscribe("left", gripper, encoder_hz=100, imu=True) is False
        assert gripper.encoder.callbacks == {} and gripper.imu.callbacks == {}, "subscriptions must be torn down"
        assert any("polling the bus once per frame" in m for m in logger.warnings)

        assert guard.read("left", gripper, 1.0) == pytest.approx(0.4)
        assert gripper.read_once_calls == 1
        assert guard.read_imu("left", gripper).accel_mps2[0] == -1
        assert gripper.imu_read_once_calls == 1

    def test_no_first_sample_falls_back_to_polling(self):
        guard = GripperReadGuard(FakeLogger())
        gripper = FakeStreamingGripper(0.4, silent=True)

        assert guard.subscribe("left", gripper, encoder_hz=100, first_sample_timeout_s=0.03) is False
        assert gripper.stopped == 1, "a stream that delivers nothing is stopped, not left running"
        assert guard.read("left", gripper, 1.0) == pytest.approx(0.4)
        assert gripper.read_once_calls == 1

    def test_imu_is_streamed_alongside_when_asked(self):
        guard = GripperReadGuard(FakeLogger())
        gripper = FakeStreamingGripper()
        assert guard.subscribe("left", gripper, encoder_hz=100, imu=True) is True
        assert gripper.rates == (100, 100)

        assert guard.read_imu("left", gripper).accel_mps2[0] == 0
        gripper.imu.push(7)
        assert guard.read_imu("left", gripper).accel_mps2[0] == 7
        assert gripper.imu_read_once_calls == 0

    def test_unsubscribe_all_stops_the_stream_and_restores_polling(self):
        guard = GripperReadGuard(FakeLogger())
        left, right = FakeStreamingGripper(0.1), FakeStreamingGripper(0.9)
        guard.subscribe("left", left, encoder_hz=100)
        guard.subscribe("right", right, encoder_hz=100, imu=True)

        guard.unsubscribe_all()

        for g in (left, right):
            assert g.encoder.callbacks == {} and g.imu.callbacks == {}
            assert g.is_streaming is False and g.stopped == 1
        assert guard.read("left", left, 1.0) == pytest.approx(0.1)
        assert left.read_once_calls == 1, "after unsubscribe the guard polls again"
