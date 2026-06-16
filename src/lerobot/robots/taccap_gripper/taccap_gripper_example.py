#!/usr/bin/env python

# Copyright 2026 The XenseRobotics Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""
Standalone smoke test for TaccapGripper.

The gripper, its tactile sensors and its wrist camera are auto-discovered by
serial rule — no serials are supplied. Pass ``--side`` only when both grippers
are connected.

Usage:
    # Gripper + tactile + wrist (auto-discovered); pick a side if both present.
    python -m lerobot.robots.taccap_gripper.taccap_gripper_example --side left

    # Cameras + gripper only, no wrist camera:
    python -m lerobot.robots.taccap_gripper.taccap_gripper_example --side left --no-wrist

    # Add the Pico4 tracker (pose); pin its PT- serial to enable pose.
    python -m lerobot.robots.taccap_gripper.taccap_gripper_example \\
        --side left --tracker --tracker-sn PT-XXXXXXXXXXXX

    # Bind the Slave/Follower units instead of the Master/Leader ones:
    python -m lerobot.robots.taccap_gripper.taccap_gripper_example --role follower

The jaw closed position is fixed at 0 rad by the SDK's ``Encoder.set_zero()``;
only the open angle (``--open-rad``, default 1.7 for TC-GU-01) is configurable.

Run the SDK's calibration once per device before using this script:
    python third_party/taccap-gripper/python/examples/calibrate.py <SN>

The script prints 10 observation frames (scalar fields + image shapes)
then disconnects.
"""

from __future__ import annotations

import argparse
import pprint
import time

from lerobot.robots.taccap_gripper import TaccapGripper, TaccapGripperConfig


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", default=None, choices=["left", "right"],
                        help="Which gripper (only needed when both are connected).")
    parser.add_argument("--role", default="leader", choices=["leader", "follower"],
                        help="Device role to bind (default leader/Master).")
    parser.add_argument("--tracker", action="store_true",
                        help="Enable the Pico4 motion tracker.")
    parser.add_argument("--tracker-sn", default=None,
                        help="Pico4 tracker serial (required to record pose).")
    parser.add_argument("--no-wrist", action="store_true",
                        help="Disable the wrist UVC camera.")
    parser.add_argument("--imu", action="store_true",
                        help="Enable IMU readings.")
    parser.add_argument("--open-rad", type=float, default=1.7,
                        help="Encoder rad when jaw fully open (TC-GU-01 ~= 1.7).")
    parser.add_argument("--frames", type=int, default=10,
                        help="How many observation frames to print.")
    args = parser.parse_args()

    cfg = TaccapGripperConfig(
        side=args.side,
        role=args.role,
        enable_gripper=True,
        enable_imu=args.imu,
        gripper_open_rad=args.open_rad,
        enable_tracker=args.tracker,
        tracker_sn=args.tracker_sn,
        enable_wrist_camera=not args.no_wrist,
    )

    robot = TaccapGripper(cfg)

    print("[example] observation features:")
    pprint.pprint(robot.observation_features)
    print("[example] action features:")
    pprint.pprint(robot.action_features)

    robot.connect()
    try:
        for i in range(args.frames):
            obs = robot.get_observation()
            scalars = {k: v for k, v in obs.items() if not hasattr(v, "shape")}
            shapes = {k: v.shape for k, v in obs.items() if hasattr(v, "shape")}
            print(f"[{i:02d}] {scalars}  +  {shapes}")
            time.sleep(0.1)
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
