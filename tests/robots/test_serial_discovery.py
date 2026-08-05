#!/usr/bin/env python

# Copyright 2026 The XenseRobotics Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""The serial / USB-topology rules that assign devices to left and right
(``robots/taccap_gripper/serial_discovery.py``).

These decide which physical sensor lands on which observation key. Get one
backwards and nothing crashes — the rig records a left finger's contact under
``tactile_right`` and the dataset is quietly wrong, which is why the module goes
out of its way to raise instead of guess. The tests below pin both halves: the
mapping when the hardware conforms, and the refusal when it does not.

Hardware-free: the rules are pure functions, and the two that touch the bus are
driven through fake ``scan_grippers`` / ``by-path`` layouts.
"""

import pytest

from lerobot.robots.taccap_gripper import serial_discovery as disco


class TestNormalizeRole:
    @pytest.mark.parametrize(
        ("given", "expected"),
        [
            ("leader", "leader"),
            ("master", "leader"),
            ("m", "leader"),
            ("LEADER", "leader"),
            ("  Master  ", "leader"),
            ("follower", "follower"),
            ("slave", "follower"),
            ("s", "follower"),
        ],
    )
    def test_aliases(self, given, expected):
        assert disco.normalize_role(given) == expected

    def test_unknown_role_raises(self):
        with pytest.raises(ValueError, match="Unknown role"):
            disco.normalize_role("leader-ish")


class TestSideOfSequence:
    """单左双右 — the last digit decides the side."""

    @pytest.mark.parametrize("sequence", ["0001", "0003", "0007", "1239"])
    def test_odd_is_left(self, sequence):
        assert disco.side_of_sequence(sequence) == "left"

    @pytest.mark.parametrize("sequence", ["0002", "0004", "0008", "1230"])
    def test_even_is_right(self, sequence):
        assert disco.side_of_sequence(sequence) == "right"

    @pytest.mark.parametrize("bad", ["", "000A", "abc"])
    def test_without_a_trailing_digit_raises(self, bad):
        with pytest.raises(ValueError, match="trailing digit"):
            disco.side_of_sequence(bad)


class TestParseCameraSerial:
    def test_documented_example(self):
        assert disco.parse_camera_serial("XCA24Z0003m") == ("left", "leader")

    def test_even_sequence_and_slave_patch(self):
        assert disco.parse_camera_serial("XCA24Z0004s") == ("right", "follower")

    @pytest.mark.parametrize(
        "bad",
        [
            "XCA24Z0003",  # no role patch
            "XCA24Z003m",  # 3-digit sequence
            "GSPS01A24Z0003",  # a tactile serial
            "xca24z0003m",  # lower case
            "",
        ],
    )
    def test_non_conforming_serial_raises(self, bad):
        with pytest.raises(ValueError, match="does not match the rule"):
            disco.parse_camera_serial(bad)


class TestPicoTrackerSide:
    """A different serial system from the Xense one: the tracker's side is the
    SECOND-to-last digit, not the last."""

    def test_documented_example(self):
        assert disco.pico_tracker_side("PC2310MLL3200496G") == "right"

    def test_odd_second_to_last_digit_is_left(self):
        assert disco.pico_tracker_side("PC2310MLL3200495G") == "left"

    def test_only_the_second_to_last_digit_counts(self):
        """Guards against someone 'fixing' this to the Xense 单左双右 rule, which
        reads a different position. Both serials below have earlier digits of
        the opposite parity to their verdict."""
        assert disco.pico_tracker_side("PC2310MLL3200425G") == "left"  # ends 2,5,G: 5 decides, not the even 2
        assert disco.pico_tracker_side("PC2310MLL3200516G") == "right"  # ends 1,6,G: 6 decides, not the odd 1

    @pytest.mark.parametrize("bad", ["", "G", "PC2310MLLXG", "PC2310MLL32004G6"])
    def test_without_a_digit_in_that_position_raises(self, bad):
        with pytest.raises(ValueError, match="second-to-last"):
            disco.pico_tracker_side(bad)


class TestAssignPicoTrackers:
    LEFT = "PC2310MLL3200495G"
    RIGHT = "PC2310MLL3200496G"

    def test_one_per_side(self):
        assert disco.assign_pico_trackers([self.LEFT, self.RIGHT]) == {
            "left": self.LEFT,
            "right": self.RIGHT,
        }

    def test_order_of_discovery_does_not_matter(self):
        assert disco.assign_pico_trackers([self.RIGHT, self.LEFT]) == {
            "left": self.LEFT,
            "right": self.RIGHT,
        }

    def test_single_side_rig_ignores_the_other_side(self):
        """A one-armed rig may leave the other arm's tracker powered; only the
        requested sides are policed."""
        assert disco.assign_pico_trackers([self.LEFT, self.RIGHT], sides=("left",)) == {"left": self.LEFT}

    def test_two_trackers_on_one_side_raises(self):
        with pytest.raises(ValueError, match="Multiple Pico4 trackers map to the right side"):
            disco.assign_pico_trackers([self.RIGHT, "PC2310MLL3200498G"])

    def test_missing_side_raises(self):
        with pytest.raises(ValueError, match="No Pico4 tracker found for the left side"):
            disco.assign_pico_trackers([self.RIGHT])

    def test_missing_side_error_points_at_the_crowded_side(self):
        """The usual cause of an empty side is a mis-burned serial putting both
        trackers on the other one; the message has to say so, or the operator
        goes looking for a tracker that is sitting right there.

        Only reachable when the crowded side was not itself requested — when it
        was, the duplicate check fires first and names it directly (covered by
        ``test_two_trackers_on_one_side_raises``).
        """
        with pytest.raises(ValueError, match="check that serial's 2nd-to-last digit"):
            disco.assign_pico_trackers([self.RIGHT, "PC2310MLL3200498G"], sides=("left",))


class TestResolvePicoTrackers:
    LEFT = "PC2310MLL3200495G"
    RIGHT = "PC2310MLL3200496G"

    def test_falls_back_to_the_rule_when_nothing_is_pinned(self):
        resolved = disco.resolve_pico_trackers(
            ("left", "right"), {"left": None, "right": None}, lambda: [self.LEFT, self.RIGHT]
        )
        assert resolved == {"left": self.LEFT, "right": self.RIGHT}

    def test_a_pinned_side_bypasses_the_rule_entirely(self):
        """The escape hatch for a tracker whose serial does not follow the rule:
        the pinned serial is used verbatim, parity notwithstanding."""
        resolved = disco.resolve_pico_trackers(
            ("left",), {"left": "WEIRD-SERIAL-0000"}, lambda: pytest.fail("must not enumerate")
        )
        assert resolved == {"left": "WEIRD-SERIAL-0000"}

    def test_fully_pinned_rig_never_enumerates(self):
        """Enumeration blocks on the PC service; a rig that pinned both sides
        should not wait on it."""
        resolved = disco.resolve_pico_trackers(
            ("left", "right"),
            {"left": "A0", "right": "B1"},
            lambda: pytest.fail("must not enumerate when every side is pinned"),
        )
        assert resolved == {"left": "A0", "right": "B1"}

    def test_pinning_one_side_still_rules_the_other(self):
        resolved = disco.resolve_pico_trackers(
            ("left", "right"), {"left": "PINNED", "right": None}, lambda: [self.RIGHT]
        )
        assert resolved == {"left": "PINNED", "right": self.RIGHT}

    def test_blank_override_is_treated_as_unset(self):
        resolved = disco.resolve_pico_trackers(("left",), {"left": "   "}, lambda: [self.LEFT])
        assert resolved == {"left": self.LEFT}


class TestHubOfBypath:
    """A device's hub is its USB port path minus its own port — this is what
    pairs two tactile sensors to the gripper sharing their hub."""

    def test_gripper_and_tactile_on_one_hub_agree(self):
        gripper = disco._hub_of_bypath("pci-0000:00:14.0-usb-0:6.1:1.0")
        tactile = disco._hub_of_bypath("pci-0000:00:14.0-usb-0:6.4:1.0-video-index0")
        assert gripper == tactile == "0:6"

    def test_a_different_hub_does_not_collide(self):
        assert disco._hub_of_bypath("pci-0000:00:14.0-usb-0:7.1:1.0") == "0:7"

    def test_device_directly_on_a_root_port(self):
        assert disco._hub_of_bypath("pci-0000:00:14.0-usb-0:6:1.0") == "0:6"

    def test_deeper_chain_keeps_the_full_parent_path(self):
        assert disco._hub_of_bypath("pci-0000:00:14.0-usb-0:6.4.2:1.0") == "0:6.4"

    def test_non_usb_path_gives_none(self):
        assert disco._hub_of_bypath("pci-0000:00:14.0-scsi-0:0:0:0") is None


class TestDiscoverTactilesByHub:
    """The composite rule: hub → gripper → side, and finger from the GSPS last
    digit. Driven through fake bus/filesystem layers."""

    LEFT_HUB = "0:6"
    RIGHT_HUB = "0:7"

    def _install(self, monkeypatch, *, hub_sides, tactiles):
        """``tactiles`` is {by-id basename: hub}."""
        monkeypatch.setattr(disco, "_gripper_hub_sides", lambda role: hub_sides)
        monkeypatch.setattr(disco.glob, "glob", lambda pattern: list(tactiles))
        monkeypatch.setattr(disco.os.path, "basename", lambda p: p)
        monkeypatch.setattr(disco, "_device_hub", lambda path, bypath_dir: tactiles[path])

    def test_pairs_each_hub_to_its_gripper_side_and_finger(self, monkeypatch):
        self._install(
            monkeypatch,
            hub_sides={
                self.LEFT_HUB: ("left", "TCGU01A24Z0001m"),
                self.RIGHT_HUB: ("right", "TCGU01A24Z0002m"),
            },
            tactiles={
                "usb-GSPS01A25Z0011-video-index0": self.LEFT_HUB,  # odd  -> left finger
                "usb-GSPS01A25Z0012-video-index0": self.LEFT_HUB,  # even -> right finger
                "usb-GSPS01A25Z0013-video-index0": self.RIGHT_HUB,
                "usb-GSPS01A25Z0014-video-index0": self.RIGHT_HUB,
            },
        )

        got = disco.discover_tactiles_by_hub("leader")

        assert got == {
            "left": {"left": "GSPS01A25Z0011", "right": "GSPS01A25Z0012"},
            "right": {"left": "GSPS01A25Z0013", "right": "GSPS01A25Z0014"},
        }

    def test_the_gripper_decides_the_side_not_the_tactile_serial(self, monkeypatch):
        """Sensors whose own serials are all odd still land on the right side if
        that is where their gripper says they are. This is the rule that makes
        the firmware SN, not the sensor serial, authoritative for side."""
        self._install(
            monkeypatch,
            hub_sides={self.LEFT_HUB: ("right", "TCGU01A24Z0002m")},
            tactiles={
                "usb-GSPS01A25Z0011-video-index0": self.LEFT_HUB,
                "usb-GSPS01A25Z0012-video-index0": self.LEFT_HUB,
            },
        )

        got = disco.discover_tactiles_by_hub("leader")

        assert got["right"] == {"left": "GSPS01A25Z0011", "right": "GSPS01A25Z0012"}
        assert got["left"] == {}

    def test_two_sensors_on_one_hub_with_the_same_finger_raises(self, monkeypatch):
        self._install(
            monkeypatch,
            hub_sides={self.LEFT_HUB: ("left", "TCGU01A24Z0001m")},
            tactiles={
                "usb-GSPS01A25Z0011-video-index0": self.LEFT_HUB,
                "usb-GSPS01A25Z0013-video-index0": self.LEFT_HUB,  # also odd
            },
        )

        with pytest.raises(ValueError, match="resolve to the left finger"):
            disco.discover_tactiles_by_hub("leader")

    def test_tactile_hub_without_a_gripper_raises(self, monkeypatch):
        self._install(
            monkeypatch,
            hub_sides={self.LEFT_HUB: ("left", "TCGU01A24Z0001m")},
            tactiles={"usb-GSPS01A25Z0013-video-index0": self.RIGHT_HUB},
        )

        with pytest.raises(ValueError, match="no matching leader gripper"):
            disco.discover_tactiles_by_hub("leader")

    def test_two_grippers_claiming_one_side_raises(self, monkeypatch):
        """A mis-burned firmware SN. Assigning both would silently drop one
        pair from the schema and record half the tactile data."""
        self._install(
            monkeypatch,
            hub_sides={
                self.LEFT_HUB: ("left", "TCGU01A24Z0001m"),
                self.RIGHT_HUB: ("left", "TCGU01A24Z0003m"),
            },
            tactiles={
                "usb-GSPS01A25Z0011-video-index0": self.LEFT_HUB,
                "usb-GSPS01A25Z0013-video-index0": self.RIGHT_HUB,
            },
        )

        with pytest.raises(ValueError, match="Two leader grippers claim the left side"):
            disco.discover_tactiles_by_hub("leader")

    def test_malformed_tactile_serial_raises(self, monkeypatch):
        self._install(
            monkeypatch,
            hub_sides={self.LEFT_HUB: ("left", "TCGU01A24Z0001m")},
            tactiles={"usb-GSPS01A25Z011-video-index0": self.LEFT_HUB},
        )

        # A 3-digit sequence does not match the by-id extractor at all, so the
        # sensor is simply not seen — the hub then has no sensors and the result
        # is empty rather than mis-assigned.
        assert disco.discover_tactiles_by_hub("leader") == {"left": {}, "right": {}}
