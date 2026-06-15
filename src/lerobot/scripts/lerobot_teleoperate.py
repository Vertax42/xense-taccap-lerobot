# Copyright 2026 XenseRobotics Inc. team. All rights reserved.
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

"""
Stream a self-driven TacCap-Gripper (single or bimanual) to Rerun.

These handheld rigs are their own teleoperator: ``lerobot-teleoperate`` just
samples ``robot.get_observation()`` at ``--fps`` and visualises it (data-stream
+ Rerun). No separate teleoperator device is required.

Example (bimanual TacCap-Gripper):

```shell
lerobot-teleoperate \
    --robot.type=bi_taccap_gripper \
    --fps=30 \
    --display_data=true
```
"""

import time
import traceback
from dataclasses import asdict, dataclass
from pprint import pformat

import numpy as np
import rerun as rr

from lerobot.configs import parser
from lerobot.robots import (  # noqa: F401
    Robot,
    RobotConfig,
    bi_taccap_gripper,
    make_robot_from_config,
    taccap_gripper,
)
from lerobot.teleoperators import (  # noqa: F401
    Teleoperator,
    TeleoperatorConfig,
    bi_pico4,
    make_teleoperator_from_config,
    pico4,
)
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.robot_utils import (
    get_logger,
    precise_sleep,
)
from lerobot.utils.utils import move_cursor_up
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data

logger = get_logger("Teleoperate")


# Self-driven, sensor-only robots: they have no teleoperator — running them through
# lerobot-teleoperate just streams get_observation() to Rerun (data-stream + viz).
# An optional --teleop satisfies the CLI but is never read. Mirrors v0.4.4's
# xense_flare / bi_xense_flare_grippers data-collection path.
SELF_DRIVEN_TELEOP_ROBOTS = frozenset({"taccap_gripper", "bi_taccap_gripper"})


@dataclass
class TeleoperateConfig:
    robot: RobotConfig
    # Self-driven robots (taccap_gripper / bi_taccap_gripper) need no teleoperator.
    teleop: TeleoperatorConfig | None = None
    # Limit the maximum frames per second.
    fps: int = 60
    teleop_time_s: float | None = None
    # Display all cameras on screen
    display_data: bool = False
    # Display data on a remote Rerun server
    display_ip: str | None = None
    # Port of the remote Rerun server
    display_port: int | None = None
    # Whether to display compressed images in Rerun (JPEG) to lower memory/IPC load. Set False for lossless display.
    display_compressed_images: bool = True
    # Print per-step timing breakdown instead of action values.
    debug_timing: bool = False
    # Dryrun mode: print actions but do not send to robot
    dryrun: bool = False


# ---------------------------------------------------------------------------
# Helper utilities
# ---------------------------------------------------------------------------


def _safe_disconnect(obj, name: str) -> None:
    if obj is None:
        return
    try:
        if obj.is_connected:
            obj.disconnect()
            logger.info(f"{name} disconnected")
    except Exception as e:
        logger.error(f"Error disconnecting {name}: {e}\n{traceback.format_exc()}")


def _cleanup(robot, teleop, display_data: bool) -> None:
    if display_data:
        try:
            rr.rerun_shutdown()
        except Exception as e:
            logger.warning(f"Error shutting down rerun: {e}")
    _safe_disconnect(teleop, teleop.__class__.__name__ if teleop else "teleop")
    _safe_disconnect(robot, robot.__class__.__name__ if robot else "robot")


def _print_obs_state(obs: dict, display_len: int, status: str) -> None:
    """Print scalar observation values with a status tag (used during reset/moving)."""
    scalar_keys = [k for k, v in obs.items() if not isinstance(v, np.ndarray)]
    col = max((len(k) for k in scalar_keys), default=display_len)
    print("\n" + "-" * (col + 18))
    print(f"{'NAME':<{col}} | {'OBS':>10}  {status}")
    for k in scalar_keys:
        print(f"{k:<{col}} | {float(obs[k]):>10.4f}")
    move_cursor_up(len(scalar_keys) + 5)


# ---------------------------------------------------------------------------
# Self-driven teleop loop (TacCap-Gripper)
# ---------------------------------------------------------------------------
def self_driven_teleop_loop(
    robot: Robot,
    fps: int,
    display_data: bool = False,
    duration: float | None = None,
    display_compressed_images: bool = True,
    debug_timing: bool = False,
):
    """Data-stream + Rerun visualisation loop for self-driven, sensor-only robots
    (``taccap_gripper`` / ``bi_taccap_gripper``).

    These robots have no teleoperator: we only read ``robot.get_observation()`` and
    stream it to Rerun with an empty action. ``send_action`` is a no-op, so nothing
    is ever commanded. Mirrors v0.4.4's ``bi_xense_flare_grippers_teleop_loop``.
    """
    display_len = max((len(key) for key in robot.observation_features), default=20)
    start = time.perf_counter()

    while True:
        loop_start = time.perf_counter()

        obs_t0 = time.perf_counter()
        obs = robot.get_observation()
        obs_time_ms = (time.perf_counter() - obs_t0) * 1e3

        if display_data:
            log_rerun_data(
                observation=obs,
                action={},
                compress_images=display_compressed_images,
            )
            if not debug_timing:
                scalar_items = [
                    (k, v) for k, v in obs.items() if not isinstance(v, np.ndarray)
                ]
                print("\n" + "-" * (display_len + 12))
                print(f"{'NAME':<{display_len}} | {'OBS':>9}")
                for key, value in scalar_items:
                    print(f"{key:<{display_len}} | {float(value):>9.4f}")
                move_cursor_up(len(scalar_items) + 3)

        dt_s = time.perf_counter() - loop_start
        precise_sleep(max(1 / fps - dt_s, 0.0))
        loop_s = time.perf_counter() - loop_start

        if debug_timing:
            cam_count = sum(1 for v in obs.values() if isinstance(v, np.ndarray))
            print(
                f"\r\033[K"
                f"obs: {obs_time_ms:5.1f}ms | "
                f"loop: {loop_s * 1e3:5.1f}ms | "
                f"target: {1e3 / fps:.1f}ms | "
                f"eff: {(1 / fps) / loop_s * 100:5.1f}% | "
                f"cams: {cam_count}",
                end="",
                flush=True,
            )
        elif not display_data:
            print(f"Self-driven loop time: {loop_s * 1e3:.2f}ms ({1 / loop_s:.0f} Hz)")
            move_cursor_up(1)

        if duration is not None and time.perf_counter() - start >= duration:
            return


@parser.wrap()
def teleoperate(cfg: TeleoperateConfig):
    logger.info(pformat(asdict(cfg)))
    if cfg.dryrun:
        logger.warn(
            "DRYRUN MODE ENABLED - Actions will be printed but NOT sent to robot"
        )

    if cfg.display_data:
        teleop_name = cfg.teleop.type if cfg.teleop else "none"
        session_name = f"teleop_{cfg.robot.type}_{teleop_name}"
        init_rerun(session_name=session_name, ip=cfg.display_ip, port=cfg.display_port)

    display_compressed_images = (
        True
        if (
            cfg.display_data
            and cfg.display_ip is not None
            and cfg.display_port is not None
        )
        else cfg.display_compressed_images
    )

    robot = None
    teleop = None

    try:
        if cfg.robot.type not in SELF_DRIVEN_TELEOP_ROBOTS:
            raise ValueError(
                f"This build only supports self-driven TacCap-Gripper robots "
                f"{sorted(SELF_DRIVEN_TELEOP_ROBOTS)}; got {cfg.robot.type!r}."
            )

        # --- taccap_gripper / bi_taccap_gripper (self-driven, data-stream + Rerun) ---
        logger.info(
            f"Detected {cfg.robot.type} (self-driven) — streaming observations to Rerun"
        )
        robot = make_robot_from_config(cfg.robot)
        robot.connect()
        # Self-driven robots have no teleoperator; an optional --teleop is accepted
        # only so it can be connected/torn down symmetrically, but it is never read.
        if cfg.teleop is not None:
            teleop = make_teleoperator_from_config(cfg.teleop)
            teleop.connect()
        try:
            self_driven_teleop_loop(
                robot=robot,
                fps=cfg.fps,
                display_data=cfg.display_data,
                duration=cfg.teleop_time_s,
                display_compressed_images=display_compressed_images,
                debug_timing=cfg.debug_timing,
            )
        except KeyboardInterrupt:
            pass

    except Exception as e:
        logger.error(f"Error in teleoperation: {e}\n{traceback.format_exc()}")
    finally:
        _cleanup(robot, teleop, cfg.display_data)


def main():
    register_third_party_plugins()
    teleoperate()


if __name__ == "__main__":
    main()
