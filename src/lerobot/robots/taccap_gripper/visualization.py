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
its live Pico4 pose, leaving a fading breadcrumb trail behind it — the same
"where has the gripper been" effect as
``third_party/taccap-gripper/python/examples/rerun_dual_with_tracker.py``.

Unlike that example (which renders the raw Pico ``LEFT_HAND_Y_UP`` frame), our
recorded pose is already remapped into our world frame (X forward, Y left, Z up,
gravity-aligned), so the scene uses ``RIGHT_HAND_Z_UP``.

The pose comes from the observation/action dict the robot already emits:
``tcp.x/y/z`` + ``tcp.r1..r6`` for the single unit, ``{side}_tcp.*`` per side for
the bimanual rig. ``tcp.r1..r3`` / ``tcp.r4..r6`` are the first two columns of the
rotation matrix (``rotation_6d_to_quaternion``). When no ``tcp.*`` keys are present
(``enable_tracker=false``) the viewer detects zero sides and every call is a no-op.
"""

from __future__ import annotations

import logging
from collections import deque
from typing import Any

import numpy as np
import rerun as rr
import rerun.blueprint as rrb

from lerobot.utils.robot_utils import rotation_6d_to_quaternion

logger = logging.getLogger(__name__)

# Per-side marker / trail colour. Empty key = the single unprefixed unit.
_SIDE_COLOR = {
    "left": (255, 80, 80),
    "right": (80, 160, 255),
    "": (120, 220, 120),
}

_ROT_KEYS = ("r1", "r2", "r3", "r4", "r5", "r6")
_POSE_KEYS = ("x", "y", "z", *_ROT_KEYS)


def _quat_xyzw_from_6d(r6d) -> list[float]:
    """``tcp.r1..r6`` → rerun quaternion ``[qx, qy, qz, qw]``.

    ``rotation_6d_to_quaternion`` returns ``[qw, qx, qy, qz]``; rerun's
    ``Quaternion(xyzw=...)`` wants the scalar last, so we reorder here.
    """
    qw, qx, qy, qz = rotation_6d_to_quaternion(np.asarray(r6d, dtype=np.float64))
    return [float(qx), float(qy), float(qz), float(qw)]


class TaccapTrajectoryViz:
    """Stateful Rerun 3D trajectory overlay for one TacCap (single or bimanual).

    Construct from a robot's ``observation_features`` (to learn which sides carry
    a tracker pose), call :meth:`setup` once after ``init_rerun``, then
    :meth:`log` every loop with the freshest obs/action dict.
    """

    def __init__(
        self,
        observation_features: dict[str, Any],
        trail_max: int = 300,
        signals: str = "all",
    ) -> None:
        self._obs_features = dict(observation_features)
        self._trail_max = trail_max
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

        self._trails: dict[str, deque] = {
            name: deque(maxlen=trail_max) for _, name in self._sides
        }
        self._static_logged: set[str] = set()

    @property
    def active(self) -> bool:
        """True only when at least one side carries a tracker pose to draw."""
        return bool(self._sides)

    # ------------------------------------------------------------------ setup

    def setup(self) -> None:
        """Log the static world scene and send the example-style blueprint.

        Both are best-effort: a blueprint mismatch is downgraded to a warning so
        the auto-created ``/world`` view still renders the trajectory.
        """
        if not self.active:
            return
        self._log_world_static()
        try:
            rr.send_blueprint(self._build_blueprint())
        except Exception as e:  # pragma: no cover — viewer-side, never fatal
            logger.warning(
                f"trajectory blueprint not applied ({type(e).__name__}: {e}); "
                "falling back to Rerun auto-layout"
            )

    def reset(self) -> None:
        """Clear every side's breadcrumb trail (e.g. at a new episode)."""
        for trail in self._trails.values():
            trail.clear()

    def _log_world_static(self) -> None:
        rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
        axis_len, axis_rad = 0.3, 0.008
        rr.log(
            "world/origin/axes",
            rr.Arrows3D(
                origins=[[0, 0, 0]] * 3,
                vectors=[[axis_len, 0, 0], [0, axis_len, 0], [0, 0, axis_len]],
                colors=[[255, 50, 50], [50, 255, 50], [50, 50, 255]],
                radii=axis_rad,
            ),
            static=True,
        )

    def _build_blueprint(self) -> rrb.Blueprint:
        # line_grid=False drops Rerun's built-in 3D floor grid (we only want the
        # origin axes + gripper markers + trajectory trail).
        spatial = rrb.Spatial3DView(name="trajectory", origin="/world", line_grid=False)

        # One 2D view per camera image entity (logged as ``observation.<key>``).
        # Tactiles first, then the wrist cameras (left_wrist / right_wrist) last —
        # i.e. the wrists sit after right_tactile_1 in the grid.
        img_keys = [k for k, v in self._obs_features.items() if isinstance(v, tuple)]
        img_keys = [k for k in img_keys if "wrist" not in k] + [
            k for k in img_keys if "wrist" in k
        ]
        img_views = [
            rrb.Spatial2DView(name=k, origin=f"/observation.{k}") for k in img_keys
        ]

        top = (
            rrb.Horizontal(spatial, rrb.Grid(*img_views), column_shares=[3, 2])
            if img_views
            else spatial
        )
        return rrb.Blueprint(
            rrb.Vertical(top, self._signals_view(), row_shares=[3, 2]),
            rrb.BlueprintPanel(state="collapsed"),
            rrb.TimePanel(state="collapsed"),
        )

    def _signals_view(self) -> rrb.TimeSeriesView:
        """Bottom time-series panel. ``signals="gripper"`` restricts it to the
        gripper position channel(s) (``observation.{side}gripper.pos``); ``"all"``
        shows every scalar (gripper.pos, tcp.*, imu.*)."""
        if self._signals == "gripper":
            gripper_keys = [k for k in self._obs_features if k.endswith("gripper.pos")]
            if gripper_keys:
                return rrb.TimeSeriesView(
                    name="gripper.pos",
                    origin="/",
                    contents=[f"+ /observation.{k}" for k in gripper_keys],
                )
        return rrb.TimeSeriesView(name="signals", origin="/")

    # ------------------------------------------------------------------ per-step

    def log(self, data: dict[str, Any] | None) -> None:
        """Update each side's pose marker + trail from ``data`` (obs or action)."""
        if not self.active or not data:
            return
        for prefix, name in self._sides:
            pose = self._extract_pose(data, prefix)
            if pose is None:
                continue
            self._log_static_once(name)
            self._log_pose(name, pose)
            self._log_trail(name, pose)

    def _extract_pose(self, data: dict, prefix: str) -> tuple | None:
        keys = [f"{prefix}tcp.{k}" for k in _POSE_KEYS]
        if not all(k in data and data[k] is not None for k in keys):
            return None
        vals = [float(data[k]) for k in keys]
        return (vals[0], vals[1], vals[2], vals[3:9])  # (x, y, z, r6d)

    def _log_static_once(self, name: str) -> None:
        if name in self._static_logged:
            return
        ent = f"world/{name}"
        color = _SIDE_COLOR.get(name, _SIDE_COLOR[""])
        rr.log(
            f"{ent}/mesh",
            rr.Ellipsoids3D(
                centers=[[0.0, 0.0, 0.0]],
                half_sizes=[[0.035, 0.035, 0.02]],
                colors=[(*color, 220)],
            ),
        )
        axes_len = 0.10
        rr.log(
            f"{ent}/axes",
            rr.Arrows3D(
                origins=[[0, 0, 0]] * 3,
                vectors=[[axes_len, 0, 0], [0, axes_len, 0], [0, 0, axes_len]],
                colors=[[255, 80, 80], [80, 255, 80], [80, 80, 255]],
                radii=0.004,
            ),
        )
        label = name.upper() if name != "gripper" else "GRIPPER"
        rr.log(
            f"{ent}/label",
            rr.Points3D([[0, 0, 0.10]], labels=[label], colors=[color], radii=0.004),
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
        x, y, z, _ = pose
        trail = self._trails[name]
        trail.append([x, y, z])
        if len(trail) < 2:
            return
        color = _SIDE_COLOR.get(name, _SIDE_COLOR[""])
        rr.log(
            f"world/trails/{name}",
            rr.LineStrips3D([list(trail)], colors=[color], radii=0.003),
        )
