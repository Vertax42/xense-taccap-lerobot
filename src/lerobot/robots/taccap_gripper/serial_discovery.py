#!/usr/bin/env python

# Copyright 2026 The XenseRobotics Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""
Serial-rule auto-discovery for TacCap-Gripper devices.

Xense device serials encode *side* (left/right) and *role* (master/leader vs
slave/follower) by rule, so a rig's grippers, tactile sensors and wrist cameras
can be discovered and assigned automatically instead of hand-listed per side.

Serial grammar
--------------
- Gripper ``TCGU01A24Z0001m`` : ``TCGU01`` + batch(``A24``) + line(``Z``|``A``) +
  sequence(``NNNN``) + patch(``m``|``s``)
- Tactile ``GSPS01A24Z0001``  : ``GSPS01`` + batch + line + sequence
- Camera  ``XCA24Z0001m``     : ``XC`` + batch + line + sequence + patch(``m``|``s``)

Rule: the **last digit of the 4-digit sequence is odd → left, even → right**;
patch ``m`` → Master/Leader, ``s`` → Slave/Follower. A wrist camera's sequence
matches the gripper it is mounted on.

Discovery sources (enumeration only — nothing is opened here):
- grippers : ``xense.taccap.scan_grippers()`` — endpoints already carry
  ``.side`` / ``.role`` / ``.firmware_sn`` (the SDK parses the firmware serial).
- tactile  : ``/dev/v4l/by-id`` USB-video names (the GSPS serial is embedded).
- cameras  : ``/dev/v4l/by-id`` USB-video names (the XC serial is embedded).

Tactile + camera discovery is filesystem-only (no SDK, no hardware open), so it
is safe to run at robot construction time to populate the observation schema.
Gripper discovery touches the serial bus and runs at ``connect()``.

Every helper raises ``ValueError`` with the offending serial(s) when something
does not conform to the rule, so a mis-burned or mis-installed device is caught
rather than silently mis-mapped.
"""

from __future__ import annotations

import glob
import re
from typing import Any

try:
    from xense.taccap import scan_grippers

    TACCAP_SDK_AVAILABLE = True
except ImportError:
    TACCAP_SDK_AVAILABLE = False

SIDES = ("left", "right")

_BYID_DIR = "/dev/v4l/by-id"

# Full-grammar validators (the documented rule).
_GRIPPER_RE = re.compile(r"^TCGU01[A-Z]\d{2}[ZA](\d{4})([ms])$")
_TACTILE_RE = re.compile(r"^GSPS01[A-Z]\d{2}[ZA](\d{4})$")
_CAMERA_RE = re.compile(r"^XC[A-Z]\d{2}[ZA](\d{4})([ms])$")

# Serial extractors for the /dev/v4l/by-id device names.
_TACTILE_BYID_RE = re.compile(r"(GSPS01[A-Z]\d{2}[ZA]\d{4})")
_CAMERA_BYID_RE = re.compile(r"(XC[A-Z]\d{2}[ZA]\d{4}[ms])")

_PATCH_ROLE = {"m": "leader", "s": "follower"}
_ROLE_ALIASES = {
    "leader": "leader",
    "master": "leader",
    "m": "leader",
    "follower": "follower",
    "slave": "follower",
    "s": "follower",
}


def normalize_role(role: str) -> str:
    """Map a user-facing role string to canonical ``leader``/``follower``."""
    key = str(role).strip().lower()
    if key not in _ROLE_ALIASES:
        raise ValueError(
            f"Unknown role {role!r}; expected one of: leader/master or follower/slave."
        )
    return _ROLE_ALIASES[key]


def side_of_sequence(sequence: str) -> str:
    """Odd trailing digit → ``left``, even → ``right`` (the Xense side rule)."""
    if not sequence or not sequence[-1].isdigit():
        raise ValueError(
            f"Serial sequence {sequence!r} has no trailing digit to derive a side from."
        )
    return "left" if int(sequence[-1]) % 2 == 1 else "right"


# ---- Pico4 motion tracker (separate serial system: e.g. PC2310MLL3200496G) --
# The Pico4 tracker SN is NOT an Xense serial; side is encoded in the
# **second-to-last digit** (odd → left, even → right), and trackers are
# enumerated from the XenseVR PC service at connect time (not the filesystem).


def pico_tracker_side(sn: str) -> str:
    """Side of a Pico4 motion tracker from its serial's **second-to-last digit**
    (odd → left, even → right), e.g. ``PC2310MLL3200496G`` → ``6`` → right.

    Raises ``ValueError`` if the second-to-last character is not a digit.
    """
    if len(sn) < 2 or not sn[-2].isdigit():
        raise ValueError(
            f"Pico4 tracker serial {sn!r} has no digit in the second-to-last "
            "position to derive a side (expected e.g. PC2310MLL3200496G)."
        )
    return "left" if int(sn[-2]) % 2 == 1 else "right"


def assign_pico_trackers(serials, sides=SIDES) -> dict[str, str]:
    """Map discovered Pico4 tracker serials to ``{side: sn}`` by the
    second-to-last-digit rule, requiring exactly one tracker per requested side.

    Strict: raises ``ValueError`` on a malformed serial, two trackers resolving
    to the same side, or a requested side with no tracker — so a partial or
    mis-paired rig is caught rather than silently recording zero pose.
    """
    grouped: dict[str, list[str]] = {"left": [], "right": []}
    for sn in serials:
        grouped[pico_tracker_side(sn)].append(sn)
    result: dict[str, str] = {}
    for side in sides:
        parity = "odd" if side == "left" else "even"
        got = grouped.get(side, [])
        if len(got) > 1:
            raise ValueError(
                f"Multiple Pico4 trackers map to the {side} side "
                f"(2nd-to-last digit {parity}): {got}."
            )
        if not got:
            raise ValueError(
                f"No Pico4 tracker found for the {side} side (need a serial whose "
                f"2nd-to-last digit is {parity}). Discovered: {list(serials)}."
            )
        result[side] = got[0]
    return result


def resolve_pico_trackers(sides, manual, enumerate_serials) -> dict[str, str]:
    """Resolve ``{side: tracker_sn}`` for ``sides``, honoring manual overrides.

    For each side, a non-empty ``manual[side]`` serial is used **verbatim** — no
    rule check, no enumeration — the escape hatch for a tracker whose serial does
    not follow the second-to-last-digit side rule, or when the PC-service
    enumeration is flaky. Sides left unset (``None``/empty) are filled by
    ``assign_pico_trackers`` (the parity rule).

    ``enumerate_serials`` is a zero-arg callable returning the connected tracker
    serials; it is invoked **only** when at least one side needs the rule, so a
    fully-pinned rig never blocks on enumeration.
    """
    pinned = {s: str(manual.get(s) or "").strip() for s in sides}
    pinned = {s: v for s, v in pinned.items() if v}
    need_rule = tuple(s for s in sides if s not in pinned)
    result: dict[str, str] = dict(pinned)
    if need_rule:
        result.update(assign_pico_trackers(list(enumerate_serials()), need_rule))
    return {s: result[s] for s in sides}


def _require_sdk() -> None:
    if not TACCAP_SDK_AVAILABLE:
        raise ImportError(
            "xense.taccap SDK not available — required for gripper auto-discovery. "
            "Build the vendored submodule (run setup_env.sh --install)."
        )


def parse_camera_serial(sn: str) -> tuple[str, str]:
    """Return ``(side, role)`` for a wrist-camera serial like ``XCA24Z0003m``.

    The SDK's ``parse_serial`` reports ``valid=False`` for the 2-char ``XC``
    prefix, so cameras get this dedicated check. Raises ``ValueError`` if the
    serial does not match the documented grammar.
    """
    m = _CAMERA_RE.match(sn)
    if not m:
        raise ValueError(
            f"Camera serial {sn!r} does not match the rule "
            "XC<batch><line><seq><m|s> (e.g. XCA24Z0003m)."
        )
    return side_of_sequence(m.group(1)), _PATCH_ROLE[m.group(2)]


def _byid_serials(extract_re: re.Pattern) -> list[str]:
    """Unique serials extracted from the ``/dev/v4l/by-id`` device names.

    Each USB video device exposes ``-video-index0`` and ``-video-index1`` nodes;
    we de-dupe by the serial so a device is counted once.
    """
    found: set[str] = set()
    for path in glob.glob(f"{_BYID_DIR}/*"):
        m = extract_re.search(path)
        if m:
            found.add(m.group(1))
    return sorted(found)


def discover_tactiles() -> dict[str, list[str]]:
    """``{'left': [GSPS…], 'right': [GSPS…]}`` from ``/dev/v4l/by-id`` (validated)."""
    grouped: dict[str, list[str]] = {"left": [], "right": []}
    for sn in _byid_serials(_TACTILE_BYID_RE):
        m = _TACTILE_RE.match(sn)
        if not m:
            raise ValueError(
                f"Tactile serial {sn!r} does not match the rule "
                "GSPS01<batch><line><seq> (e.g. GSPS01A24Z0003)."
            )
        grouped[side_of_sequence(m.group(1))].append(sn)
    for side in SIDES:
        grouped[side].sort()
    return grouped


def discover_wrist_cameras(role: str) -> dict[str, str]:
    """``{side: XC…}`` for the requested role from ``/dev/v4l/by-id`` (validated)."""
    role = normalize_role(role)
    grouped: dict[str, list[str]] = {"left": [], "right": []}
    for sn in _byid_serials(_CAMERA_BYID_RE):
        side, sn_role = parse_camera_serial(sn)
        if sn_role == role:
            grouped[side].append(sn)
    result: dict[str, str] = {}
    for side in SIDES:
        if len(grouped[side]) > 1:
            raise ValueError(
                f"Found {len(grouped[side])} {role} wrist cameras for the {side} "
                f"side: {grouped[side]}. Sequence numbers must be unique per side."
            )
        if grouped[side]:
            result[side] = grouped[side][0]
    return result


def discover_grippers(role: str) -> dict[str, Any]:
    """``{side: GripperEndpoints}`` for the requested role via ``scan_grippers()``."""
    _require_sdk()
    role = normalize_role(role)
    grouped: dict[str, list[Any]] = {"left": [], "right": []}
    for ep in scan_grippers():
        # Cross-check the firmware serial against the rule, then trust the SDK's
        # parsed side/role (endpoints already carry them).
        if ep.firmware_sn and not _GRIPPER_RE.match(ep.firmware_sn):
            raise ValueError(
                f"Gripper firmware serial {ep.firmware_sn!r} does not match the rule "
                "TCGU01<batch><line><seq><m|s> (e.g. TCGU01A24Z0001m)."
            )
        if ep.role.name.lower() != role:
            continue
        side = ep.side.name.lower()
        if side in grouped:
            grouped[side].append(ep)
    result: dict[str, Any] = {}
    for side in SIDES:
        if len(grouped[side]) > 1:
            sns = [e.firmware_sn for e in grouped[side]]
            raise ValueError(
                f"Found {len(grouped[side])} {role} grippers for the {side} side: "
                f"{sns}. Firmware serials are supposed to be unique."
            )
        if grouped[side]:
            result[side] = grouped[side][0]
    return result


def discover_taccap(
    role: str,
    sides: tuple[str, ...] = SIDES,
    expected_tactiles_per_side: int = 2,
    with_gripper: bool = True,
    with_wrist_camera: bool = True,
) -> dict[str, dict]:
    """Discover + validate every TacCap device for the requested ``sides``.

    Returns ``{side: {"gripper": endpoints|None, "tactile_serials": [...],
    "wrist_camera_serial": str|None}}``. Raises ``ValueError`` naming the
    offending serial(s) when a requested side's hardware is missing or
    mis-aligned with the rule (avoids a silent definition/serial mismatch).

    ``with_gripper`` is the only flag that touches the serial bus; leave it
    ``False`` for a filesystem-only (tactile + camera) pass at construction time.
    """
    role = normalize_role(role)
    grippers = discover_grippers(role) if with_gripper else {}
    tactiles = discover_tactiles() if expected_tactiles_per_side else {"left": [], "right": []}
    cameras = discover_wrist_cameras(role) if with_wrist_camera else {}

    result: dict[str, dict] = {}
    for side in sides:
        parity = "odd" if side == "left" else "even"
        info: dict[str, Any] = {
            "gripper": None,
            "tactile_serials": [],
            "wrist_camera_serial": None,
        }
        if with_gripper:
            if side not in grippers:
                raise ValueError(
                    f"No {role} gripper found for the {side} side "
                    f"(rule: {side} == {parity} sequence)."
                )
            info["gripper"] = grippers[side]
        if expected_tactiles_per_side:
            got = tactiles.get(side, [])
            if len(got) != expected_tactiles_per_side:
                raise ValueError(
                    f"Expected {expected_tactiles_per_side} {side} tactile sensors "
                    f"({parity} sequence), found {len(got)}: {got}."
                )
            info["tactile_serials"] = got
        if with_wrist_camera:
            if side not in cameras:
                raise ValueError(
                    f"No {role} wrist camera found for the {side} side "
                    f"(rule: {side} == {parity} sequence)."
                )
            info["wrist_camera_serial"] = cameras[side]
        result[side] = info
    return result
