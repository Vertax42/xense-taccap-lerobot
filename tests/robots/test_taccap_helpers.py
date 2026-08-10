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
    connect_cameras_parallel,
    hardware_manifest_unit,
    open_gripper,
    read_head_camera_skew,
    split_camera_read,
    swap_tactile_display_features,
    tactile_camera_output_types,
    tactile_display_key,
    validate_robot_id,
    write_hardware_manifest,
)
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

    def test_written_under_meta_not_into_info_json(self, tmp_path):
        """``meta/info.json`` is upstream's schema; a fork-local key there would
        collide on the next v5.x sync."""
        path = write_hardware_manifest(tmp_path, self.MANIFEST, FakeLogger())
        assert path == tmp_path / HARDWARE_MANIFEST_PATH
        assert json.loads(path.read_text()) == self.MANIFEST
        assert not (tmp_path / "meta" / "info.json").exists()

    def test_rewriting_the_same_hardware_is_a_silent_no_op(self, tmp_path):
        write_hardware_manifest(tmp_path, self.MANIFEST, FakeLogger())
        logger = FakeLogger()
        write_hardware_manifest(tmp_path, self.MANIFEST, logger)
        assert logger.warnings == []

    def test_resuming_on_other_hardware_warns_and_keeps_the_original(self, tmp_path):
        """The episodes already in the dataset came from the devices named in
        the file; overwriting would misattribute every one of them."""
        write_hardware_manifest(tmp_path, self.MANIFEST, FakeLogger())
        swapped = json.loads(json.dumps(self.MANIFEST))
        swapped["units"][0]["gripper_sn"] = "TCGU01A24Z0003m"

        logger = FakeLogger()
        write_hardware_manifest(tmp_path, swapped, logger)

        on_disk = json.loads((tmp_path / HARDWARE_MANIFEST_PATH).read_text())
        assert on_disk == self.MANIFEST
        assert len(logger.warnings) == 1
        assert "TCGU01A24Z0003m" in logger.warnings[0]  # names what is connected now

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
