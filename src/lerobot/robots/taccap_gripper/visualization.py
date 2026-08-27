#!/usr/bin/env python

# Copyright 2026 The XenseRobotics Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""
Rerun 3D trajectory visualisation for TacCap-Gripper devices.

Adds an example-style 3D world view on top of LeRobot's default scalar/image
panels: each gripper is drawn as a labelled ellipsoid with a local axis triad at
its live Pico4 pose, leaving a fading breadcrumb trail behind it (fading for
real: the alpha ramps along the trail) — the same
"where has the gripper been" effect as
``third_party/taccap-gripper/python/examples/rerun_dual_with_tracker.py``.

Unlike that example (which renders the raw Pico ``LEFT_HAND_Y_UP`` frame), our
recorded pose is already remapped into our world frame (X forward, Y left, Z up,
gravity-aligned), so the scene declares ``FLU`` — the preset that names all three
axes, which is what lets the viewer point its initial camera down +X.

The pose comes from the observation/action dict the robot already emits:
``tcp.x/y/z`` + ``tcp.r1..r6`` for the single unit, ``{side}_tcp.*`` per side for
the bimanual rig. ``tcp.r1..r3`` / ``tcp.r4..r6`` are the first two columns of the
rotation matrix (``rotation_6d_to_quaternion``). When no ``tcp.*`` keys are present
(``enable_tracker=false``) the viewer detects zero sides and every call is a no-op.
"""

from __future__ import annotations

from collections import deque
from typing import Any

import numpy as np
import rerun as rr
import rerun.blueprint as rrb

from lerobot.utils.robot_utils import get_logger, rotation_6d_to_quaternion

logger = get_logger("taccap_viz")

# Per-side marker / trail colour. Empty key = the single unprefixed unit.
_SIDE_COLOR = {
    "left": (255, 80, 80),
    "right": (80, 160, 255),
    "": (120, 220, 120),
    # The headset, distinct from either gripper so the three read apart.
    "head": (230, 190, 60),
}

# Trail rendering. 90 samples is ~3 s at 30 fps: long enough to read the
# stroke the operator just made, short enough that two of them do not knot
# together. The old 300 (~10 s) at flat opacity was the thicket.
_TRAIL_CHUNKS = 8
_TRAIL_ALPHA_MIN = 25
_TRAIL_RADIUS = 0.0022

_ROT_KEYS = ("r1", "r2", "r3", "r4", "r5", "r6")
_POSE_KEYS = ("x", "y", "z", *_ROT_KEYS)


def _quat_xyzw_from_6d(r6d) -> list[float]:
    """``tcp.r1..r6`` → rerun quaternion ``[qx, qy, qz, qw]``.

    ``rotation_6d_to_quaternion`` returns ``[qw, qx, qy, qz]``; rerun's
    ``Quaternion(xyzw=...)`` wants the scalar last, so we reorder here.
    """
    qw, qx, qy, qz = rotation_6d_to_quaternion(np.asarray(r6d, dtype=np.float64))
    return [float(qx), float(qy), float(qz), float(qw)]


def _eye_order(key: str) -> tuple[int, str]:
    """Sort camera keys left-then-right, as they sit on the operator.

    Plain sorting puts ``right_head`` before ``left_head``, which reads
    backwards next to a pair of images.
    """
    side = 0 if key.startswith("left") else 1 if key.startswith("right") else 2
    return (side, key)


class TaccapTrajectoryViz:
    """Stateful Rerun 3D trajectory overlay for one TacCap (single or bimanual).

    Construct from a robot's ``observation_features`` (to learn which sides carry
    a tracker pose), call :meth:`setup` once after ``init_rerun``, then
    :meth:`log` every loop with the freshest obs/action dict.
    """

    def __init__(
        self,
        observation_features: dict[str, Any],
        trail_max: int = 90,
        signals: str = "all",
        show_trajectory: bool = True,
    ) -> None:
        self._obs_features = dict(observation_features)
        self._trail_max = trail_max
        # Suppresses the 3D pose trail only. The rest of the layout still
        # applies, because this class owns the whole blueprint - gating it off
        # here would drop the viewer back to auto-layout.
        self._show_trajectory = show_trajectory
        # Which scalars the time-series panel shows: ``"all"`` (gripper.pos +
        # tcp.* + imu.*) or ``"gripper"`` (only the jaw position channel(s)).
        self._signals = signals

        # Discover the sides that actually carry a tracker pose. ``prefix`` is the
        # obs-key prefix ("" / "left_" / "right_"); ``name`` labels the entity.
        self._sides: list[tuple[str, str]] = []
        for key in self._obs_features:
            if key == "tcp.x":
                self._sides.append(("", "gripper"))
            elif key.endswith("_tcp.x"):
                name = key[: -len("_tcp.x")]
                self._sides.append((f"{name}_", name))

        self._trails: dict[str, deque] = {name: deque(maxlen=trail_max) for _, name in self._sides}
        self._static_logged: set[str] = set()

    @property
    def has_poses(self) -> bool:
        """True when a tracker pose should be drawn in 3D."""
        return bool(self._sides) and self._show_trajectory

    @property
    def active(self) -> bool:
        """True when there is anything worth laying out.

        Poses are not required. Without a tracker there is no 3D trail, but the
        cameras and scalars still benefit from a deliberate layout - and if this
        returned False there, no blueprint would be sent at all and Rerun would
        fall back to auto-layout, which scatters seven camera streams and thirty
        scalars across equally-sized tiles.
        """
        return bool(self._sides) or bool(self._image_keys()) or bool(self._scalar_keys())

    # ------------------------------------------------------------ key grouping

    def _image_keys(self) -> list[str]:
        return [k for k, v in self._obs_features.items() if isinstance(v, tuple)]

    def _scalar_keys(self) -> list[str]:
        return [k for k, v in self._obs_features.items() if not isinstance(v, tuple)]

    # ------------------------------------------------------------------ setup

    def setup(self) -> None:
        """Log the static world scene and send the example-style blueprint.

        Both are best-effort: a blueprint mismatch is downgraded to a warning so
        the auto-created ``/world`` view still renders the trajectory.
        """
        if not self.active:
            return
        if self.has_poses:
            self._log_world_static()
        try:
            rr.send_blueprint(self._build_blueprint())
        except Exception as e:  # pragma: no cover — viewer-side, never fatal
            logger.warning(
                f"trajectory blueprint not applied ({type(e).__name__}: {e}); falling back to Rerun auto-layout"
            )

    def reset(self) -> None:
        """Clear every side's breadcrumb trail (e.g. at a new episode)."""
        for trail in self._trails.values():
            trail.clear()

    def _log_world_static(self) -> None:
        # FLU == X forward, Y left, Z up, which is exactly our world convention.
        # RIGHT_HAND_Z_UP (used before) only declares handedness and the up axis,
        # leaving X/Y unlabelled — so the viewer had nothing to tell it which way
        # "forward" was and picked a default azimuth. Declaring the forward axis
        # is both more accurate and what orients the initial camera down +X.
        rr.log("world", rr.ViewCoordinates.FLU, static=True)
        axis_len, axis_rad = 0.3, 0.008
        rr.log(
            "world/origin/axes",
            rr.Arrows3D(
                origins=[[0, 0, 0]] * 3,
                vectors=[[axis_len, 0, 0], [0, axis_len, 0], [0, 0, axis_len]],
                colors=[[255, 50, 50], [50, 255, 50], [50, 50, 255]],
                radii=axis_rad,
                labels=["+X forward", "+Y left", "+Z up"],
            ),
            static=True,
        )

    def _build_blueprint(self) -> rrb.Blueprint:
        """Lay the viewer out by what each stream is for, not by how many there are.

        Rerun's auto-layout gives every entity an equal tile, which buries the
        streams an operator actually watches under four tactile pads and thirty
        scalar plots. So: the 3D trail takes the large left tile, the cameras
        stack down the right in the order you read them — head for context,
        wrists for the close-up, tactiles last — and the scalars are grouped
        into tabs rather than piled into one illegible plot.
        """
        img_keys = self._image_keys()
        # Match the suffix, not a "head" prefix: the keys are left_head /
        # right_head, so a prefix test silently dropped them into the tactile
        # bucket and they ended up in that grid. Scalars named head_camera.*
        # are not a risk here — img_keys only carries the image features.
        head = sorted((k for k in img_keys if k.endswith("_head")), key=_eye_order)
        wrist = sorted((k for k in img_keys if "wrist" in k), key=_eye_order)
        tactile = [k for k in img_keys if k not in head and k not in wrist]

        def view(key: str) -> rrb.Spatial2DView:
            return rrb.Spatial2DView(name=key, origin=f"/observation.{key}")

        # Left column: the 3D trail when there is one, else the largest camera
        # group, so the biggest tile is never an empty panel.
        primary = None
        if self.has_poses:
            # line_grid=False drops Rerun's built-in floor grid; we only want the
            # origin axes, gripper markers and trail.
            primary = rrb.Spatial3DView(name="trajectory", origin="/world", line_grid=False)

        # Right column, top to bottom: head cameras, then the wrists directly
        # under them (they show the same scene at two scales, so reading down
        # the column goes from context to close-up), then the tactile grid.
        secondary: list[Any] = []
        if head:
            secondary.append(rrb.Horizontal(*(view(k) for k in head), name="head"))
        if wrist:
            secondary.append(rrb.Horizontal(*(view(k) for k in wrist), name="wrist"))
        if tactile:
            secondary.append(rrb.Grid(*(view(k) for k in tactile), name="tactile"))
        if primary is None and secondary:
            primary, secondary = secondary[0], secondary[1:]

        if primary is not None and secondary:
            top = rrb.Horizontal(primary, rrb.Vertical(*secondary), column_shares=[3, 2])
        elif primary is not None:
            top = primary
        elif secondary:
            top = rrb.Vertical(*secondary)
        else:
            top = rrb.TimeSeriesView(name="signals", origin="/")

        signals = self._signals_view()
        contents = rrb.Vertical(top, signals, row_shares=[3, 2]) if signals else top
        return rrb.Blueprint(
            contents,
            rrb.BlueprintPanel(state="collapsed"),
            rrb.TimePanel(state="collapsed"),
        )

    def _signals_view(self):
        """Bottom panel, one tab per group of scalars.

        A single plot of every scalar is unreadable: jaw position, metres of VIO
        translation, unit-length rotation components and IMU accelerations share
        no axis. Splitting them by unit keeps each tab's y-axis meaningful.

        ``signals="gripper"`` narrows the default tab to the jaw channel(s), but
        the other tabs stay available rather than being dropped.
        """
        tabs: list[Any] = []

        def series(name: str, keys: list[str]) -> None:
            if keys:
                tabs.append(
                    rrb.TimeSeriesView(
                        name=name,
                        origin="/",
                        contents=[f"+ /observation.{k}" for k in keys],
                    )
                )

        scalars = self._scalar_keys()
        series("gripper.pos", [k for k in scalars if k.endswith("gripper.pos")])
        series(
            "head VIO position",
            [k for k in scalars if k.startswith("head_camera.") and k.split(".")[-1] in ("x", "y", "z")],
        )
        series(
            "head VIO rotation",
            [k for k in scalars if k.startswith("head_camera.") and k.split(".")[-1].startswith("r")],
        )
        series("tcp pose", [k for k in scalars if "_tcp." in k or k.startswith("tcp.")])
        series("tracker pose", [k for k in scalars if "_tracker." in k or k.startswith("tracker.")])
        series("imu", [k for k in scalars if "_imu." in k or k.startswith("imu.")])

        if not tabs:
            return rrb.TimeSeriesView(name="signals", origin="/") if scalars else None
        if len(tabs) == 1:
            return tabs[0]
        # "all" keeps a catch-all tab so nothing is unreachable from the layout.
        if self._signals != "gripper":
            tabs.append(rrb.TimeSeriesView(name="all", origin="/"))
        return rrb.Tabs(*tabs, active_tab=0)

    # ------------------------------------------------------------------ per-step

    def log(self, data: dict[str, Any] | None) -> None:
        """Update each side's pose marker + trail from ``data`` (obs or action)."""
        if not self.active or not data:
            return
        for prefix, name in self._sides:
            pose = self._extract_pose(data, prefix, "tcp")
            if pose is None:
                continue
            self._log_static_once(name)
            self._log_pose(name, pose)
            self._log_trail(name, pose)

        # The headset, when the head camera is on. It shares the gripper's
        # world frame — the same Pico→world remap is applied to both — so
        # drawing them together shows where the operator was looking relative
        # to what their hands were doing. No trail: the head wanders
        # continuously and its breadcrumb would bury the gripper trails.
        head = self._extract_pose(data, "head_camera.", "")
        if head is not None:
            self._log_static_once("head")
            self._log_pose("head", head)

    def _extract_pose(self, data: dict, prefix: str, frame: str) -> tuple | None:
        # ``frame`` is the middle segment ("tcp"); an empty one lets a caller
        # pass a fully-formed prefix such as "head_camera.".
        stem = f"{prefix}{frame}." if frame else prefix
        keys = [f"{stem}{k}" for k in _POSE_KEYS]
        if not all(k in data and data[k] is not None for k in keys):
            return None
        vals = [float(data[k]) for k in keys]
        return (vals[0], vals[1], vals[2], vals[3:9])  # (x, y, z, r6d)

    def _log_static_once(self, name: str) -> None:
        if name in self._static_logged:
            return
        ent = f"world/{name}"
        color = _SIDE_COLOR.get(name, _SIDE_COLOR[""])
        if name == "head":
            # Smaller and shorter-axed than a gripper: it is context for what
            # the hands are doing, not the thing being watched.
            half_sizes, axes_len, alpha = [[0.045, 0.030, 0.030]], 0.07, 160
        else:
            half_sizes, axes_len, alpha = [[0.035, 0.035, 0.02]], 0.10, 220
        rr.log(
            f"{ent}/mesh",
            rr.Ellipsoids3D(centers=[[0.0, 0.0, 0.0]], half_sizes=half_sizes, colors=[(*color, alpha)]),
        )
        rr.log(
            f"{ent}/axes",
            rr.Arrows3D(
                origins=[[0, 0, 0]] * 3,
                vectors=[[axes_len, 0, 0], [0, axes_len, 0], [0, 0, axes_len]],
                colors=[[255, 80, 80], [80, 255, 80], [80, 80, 255]],
                radii=0.004,
            ),
        )
        label = "HEAD" if name == "head" else ("EE" if name == "gripper" else f"{name.upper()} EE")
        rr.log(
            f"{ent}/label",
            rr.Points3D([[0, 0, axes_len]], labels=[label], colors=[color], radii=0.004),
        )
        self._static_logged.add(name)

    def _log_pose(self, name: str, pose: tuple) -> None:
        x, y, z, r6d = pose
        rr.log(
            f"world/{name}",
            rr.Transform3D(
                translation=[x, y, z],
                quaternion=rr.Quaternion(xyzw=_quat_xyzw_from_6d(r6d)),
            ),
        )

    def _log_trail(self, name: str, pose: tuple) -> None:
        """Recent path, fading out towards the oldest end.

        Drawn as a few chunks rather than one polyline so each can carry its
        own alpha. A per-point gradient would be smoother but costs a strip
        per sample; at this length the steps are not visible anyway.

        Two grippers at full opacity for ten seconds was a thicket you could
        not read either trail out of — the fade and the shorter window are
        what make two of them legible at once.
        """
        x, y, z, _ = pose
        trail = self._trails[name]
        trail.append([x, y, z])
        if len(trail) < 2:
            return

        pts = list(trail)
        r, g, b = _SIDE_COLOR.get(name, _SIDE_COLOR[""])
        chunks, colors = [], []
        n = _TRAIL_CHUNKS
        for i in range(n):
            lo = len(pts) * i // n
            hi = len(pts) * (i + 1) // n + 1  # overlap by one so chunks join up
            seg = pts[lo:hi]
            if len(seg) < 2:
                continue
            # Oldest chunk barely there, newest at full strength.
            alpha = int(_TRAIL_ALPHA_MIN + (255 - _TRAIL_ALPHA_MIN) * (i + 1) / n)
            chunks.append(seg)
            colors.append([r, g, b, alpha])
        if not chunks:
            return
        rr.log(
            f"world/trails/{name}",
            rr.LineStrips3D(chunks, colors=colors, radii=_TRAIL_RADIUS),
        )
