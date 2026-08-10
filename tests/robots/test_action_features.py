#!/usr/bin/env python

# Copyright 2026 The XenseRobotics Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""What lands in the recorded *action* column, for both TacCap robots.

Only keys in ``action_features`` take part in the record loop's shifted-frame
pairing (``action[t]`` with ``obs[t-1]``), so this schema decides what a policy
can be asked to reproduce. A key that is observation-only is context; a key here
is a target. The head pose belongs in the second group — where the operator
looked while demonstrating is part of the demonstration.

Hardware-free: discovery and the camera factory are patched out, so these build
the real robot objects without touching a bus.
"""

from unittest.mock import patch

import pytest

from lerobot.robots.bi_taccap_gripper.bi_taccap_gripper import BiTaccapGripper
from lerobot.robots.bi_taccap_gripper.config_bi_taccap_gripper import BiTaccapGripperConfig
from lerobot.robots.taccap_gripper.config_taccap_gripper import TaccapGripperConfig
from lerobot.robots.taccap_gripper.taccap_gripper import TaccapGripper

TACTILES = {
    "left": {"left": "GSPS01A28Z0005", "right": "GSPS01A28Z0006"},
    "right": {"left": "GSPS01A28Z0011", "right": "GSPS01A28Z0012"},
}
WRIST = {"left": "XCA24Z0021m", "right": "XCA24Z0024m"}
POSE_SUFFIXES = {"x", "y", "z", "r1", "r2", "r3", "r4", "r5", "r6"}


def build(cls, config, trackers):
    """Construct a robot far enough to read its schema, with no bus and no SDK.

    The availability flags are forced on because the constructor refuses to
    build without them, and CI has no vendor SDK at all — on a developer machine
    the import can also fail on library ordering (the SDK's conda OpenCV against
    cv2's vendored libtiff), which is why lerobot_record preloads it. Neither
    matters here: nothing below reaches a code path that calls into the SDK.
    """
    module = cls.__module__
    with (
        patch(f"{module}.TACCAP_SDK_AVAILABLE", True),
        patch(f"{module}.PICO4_TRACKER_AVAILABLE", True),
        patch("lerobot.robots.taccap_gripper.serial_discovery.discover_tactiles_by_hub", return_value=TACTILES),
        patch("lerobot.robots.taccap_gripper.serial_discovery.discover_wrist_cameras", return_value=WRIST),
        patch("lerobot.robots.taccap_gripper.serial_discovery.resolve_pico_trackers", return_value=trackers),
        # Patched where build_wrist_camera_config calls it, not in the robot's
        # namespace: the robots stopped importing this name when the wrist config
        # construction moved into common.py, and patching a name a module no
        # longer has raises. Stubbing only the /dev lookup also leaves the real
        # config builder running, so width/height/fps/fourcc still flow through.
        patch(
            "lerobot.robots.taccap_gripper.common.resolve_wrist_camera_path",
            side_effect=lambda sn: f"/dev/{sn}",
        ),
        # patched in the robot's own namespace: both robots do
        # `from lerobot.cameras.utils import make_cameras_from_configs`, so they
        # hold a direct reference and patching the source module is a no-op —
        # the real factory would run and import xensesdk, which CI lacks.
        patch(f"{module}.make_cameras_from_configs", return_value={}),
    ):
        return cls(config)


def head_keys(features):
    return {k for k in features if k.startswith("head_camera.")}


class TestSingleArmActionFeatures:
    def _robot(self, **kwargs):
        return build(TaccapGripper, TaccapGripperConfig(id="taccap_0", side="left", **kwargs), {"left": "PC1"})

    def test_head_pose_is_absent_without_the_head_camera(self):
        assert head_keys(self._robot(enable_head_camera=False).action_features) == set()

    def test_head_pose_is_an_action_when_the_head_camera_is_on(self):
        keys = head_keys(self._robot(enable_head_camera=True).action_features)
        assert keys == {f"head_camera.{s}" for s in POSE_SUFFIXES}

    def test_head_pose_is_in_both_observation_and_action(self):
        """Same as tcp.*: recorded as state, and available as a target."""
        robot = self._robot(enable_head_camera=True)
        assert head_keys(robot.observation_features) == head_keys(robot.action_features)

    def test_images_stay_out_of_the_action(self):
        """The head *camera* is context; only its pose is a target."""
        robot = self._robot(enable_head_camera=True)
        assert "left_head" in robot.observation_features
        assert "left_head" not in robot.action_features
        assert not any(isinstance(v, tuple) for v in robot.action_features.values())


class TestEnableTactile:
    """``enable_tactile=False`` must remove the sensors from the schema, not just
    skip opening them.

    The switch exists so a rig whose cameras will not all open can be bisected
    against the USB isochronous budget — take the four tactile streams out, see
    whether the rest fits. That only answers the question if the sensors are
    genuinely absent from the run.
    """

    def _single(self, **kwargs):
        return build(TaccapGripper, TaccapGripperConfig(id="taccap_0", side="left", **kwargs), {"left": "PC1"})

    def _bimanual(self, **kwargs):
        return build(BiTaccapGripper, BiTaccapGripperConfig(id="taccap_0", **kwargs), {"left": "PC1", "right": "PC2"})

    @staticmethod
    def _tactile_keys(features):
        return {k for k in features if "tactile" in k}

    def test_single_arm_drops_every_tactile_key(self):
        assert self._tactile_keys(self._single(enable_tactile=False).observation_features) == set()

    def test_bimanual_drops_every_tactile_key(self):
        assert self._tactile_keys(self._bimanual(enable_tactile=False).observation_features) == set()

    def test_tactile_is_on_by_default(self):
        assert self._tactile_keys(self._bimanual().observation_features)

    def test_disabling_leaves_the_rest_of_the_schema_alone(self):
        """Only the tactile keys go; wrist, jaw and pose are untouched."""
        with_tactile = self._bimanual().observation_features
        without = self._bimanual(enable_tactile=False).observation_features
        assert set(with_tactile) - set(without) == self._tactile_keys(with_tactile)

    def test_the_count_still_applies_when_enabled(self):
        """enable_tactile gates; expected_tactiles_per_side still says how many
        a gripper carries, and discovery still validates against it."""
        cfg = BiTaccapGripperConfig(id="taccap_0", expected_tactiles_per_side=2)
        assert cfg.tactiles_per_side == 2
        assert (
            BiTaccapGripperConfig(id="taccap_0", enable_tactile=False, expected_tactiles_per_side=2).tactiles_per_side
            == 0
        )


class TestBimanualActionFeatures:
    def _robot(self, **kwargs):
        return build(BiTaccapGripper, BiTaccapGripperConfig(id="taccap_0", **kwargs), {"left": "PC1", "right": "PC2"})

    def test_head_pose_is_absent_without_the_head_camera(self):
        assert head_keys(self._robot(enable_head_camera=False).action_features) == set()

    def test_head_pose_is_an_action_when_the_head_camera_is_on(self):
        keys = head_keys(self._robot(enable_head_camera=True).action_features)
        assert keys == {f"head_camera.{s}" for s in POSE_SUFFIXES}

    def test_head_pose_is_unprefixed_and_appears_once(self):
        """One headset serves both arms, so it is not per-side — unlike
        {side}_tcp.*, which the same rig reports twice."""
        features = self._robot(enable_head_camera=True).action_features
        assert not any(k.startswith(("left_head_camera", "right_head_camera")) for k in features)
        assert len(head_keys(features)) == len(POSE_SUFFIXES)

    def test_per_side_keys_are_unaffected(self):
        """Turning the head camera on must not disturb the arm columns."""
        without = self._robot(enable_head_camera=False).action_features
        with_head = self._robot(enable_head_camera=True).action_features
        sided = {k for k in with_head if k.startswith(("left_", "right_"))}
        assert sided == set(without)


class TestRobotIdIsRequired:
    """Both configs refuse to build without ``--robot.id``. It reaches the log
    prefix, the calibration filename and the recorded hardware manifest, so an
    unnamed rig means a run nothing identifies the station of."""

    @pytest.mark.parametrize(
        "make",
        [
            lambda **kw: TaccapGripperConfig(side="left", **kw),
            lambda **kw: BiTaccapGripperConfig(**kw),
        ],
        ids=["single", "bimanual"],
    )
    def test_omitting_it_fails_at_config_time(self, make):
        """At config time, i.e. before any device is touched — the CLI parse
        raises rather than a rig spinning up and recording anonymously."""
        with pytest.raises(ValueError, match="--robot.id is required"):
            make()

    @pytest.mark.parametrize(
        "make",
        [
            lambda **kw: TaccapGripperConfig(side="left", **kw),
            lambda **kw: BiTaccapGripperConfig(**kw),
        ],
        ids=["single", "bimanual"],
    )
    def test_it_is_stored_stripped(self, make):
        assert make(id="  taccap_1 ").id == "taccap_1"


@pytest.mark.parametrize("enable_head_camera", [False, True])
def test_action_features_is_a_subset_of_observation_features(enable_head_camera):
    """The record loop builds the action by selecting action_features out of the
    observation it already sampled, so a key here that the observation does not
    carry would silently drop out of the dataset."""
    robot = build(
        BiTaccapGripper,
        BiTaccapGripperConfig(id="taccap_0", enable_head_camera=enable_head_camera),
        {"left": "PC1", "right": "PC2"},
    )
    assert set(robot.action_features) <= set(robot.observation_features)
