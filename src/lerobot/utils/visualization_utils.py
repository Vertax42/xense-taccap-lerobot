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

import logging
import numbers
import os
import threading
from typing import Any

import numpy as np
import rerun as rr

from .constants import ACTION, ACTION_PREFIX, OBS_PREFIX, OBS_STR

logger = logging.getLogger(__name__)

RobotAction = dict[str, Any]
RobotObservation = dict[str, Any]


def init_rerun(session_name: str = "lerobot_control_loop", ip: str | None = None, port: int | None = None) -> None:
    """
    Initializes the Rerun SDK for visualizing the control loop.

    Args:
        session_name: Name of the Rerun session.
        ip: Optional IP for connecting to a Rerun server.
        port: Optional port for connecting to a Rerun server.
    """
    batch_size = os.getenv("RERUN_FLUSH_NUM_BYTES", "8000")
    os.environ["RERUN_FLUSH_NUM_BYTES"] = batch_size
    rr.init(session_name)
    memory_limit = os.getenv("LEROBOT_RERUN_MEMORY_LIMIT", "10%")
    if ip and port:
        rr.connect_grpc(url=f"rerun+http://{ip}:{port}/proxy")
    else:
        rr.spawn(memory_limit=memory_limit)


def select_display_observation(
    observation: RobotObservation | None, display_features: dict[str, Any] | None
) -> RobotObservation | None:
    """Narrow an observation to the keys a robot wants on screen.

    A robot may emit keys meant for one consumer only: the TacCap grippers read
    each tactile sensor twice per frame, recording the ``rectify`` image while
    showing the amplified ``difference`` one, and expose the viewer's half of
    that split as ``display_features``. Filtering here keeps the recorded stream
    out of Rerun without touching what goes to ``dataset.add_frame``.

    ``display_features`` of None (any robot that draws no such distinction)
    passes the observation straight through.
    """
    if not display_features or observation is None:
        return observation
    return {k: v for k, v in observation.items() if k in display_features}


def _is_scalar(x):
    return isinstance(x, (float | numbers.Real | np.integer | np.floating)) or (
        isinstance(x, np.ndarray) and x.ndim == 0
    )


def log_rerun_data(
    observation: RobotObservation | None = None,
    action: RobotAction | None = None,
    compress_images: bool = False,
    log_images: bool = True,
) -> None:
    """
    Logs observation and action data to Rerun for real-time visualization.

    This function iterates through the provided observation and action dictionaries and sends their contents
    to the Rerun viewer. It handles different data types appropriately:
    - Scalars values (floats, ints) are logged as `rr.Scalars`.
    - 3D NumPy arrays that resemble images (e.g., with 1, 3, or 4 channels first) are transposed
      from CHW to HWC format, (optionally) compressed to JPEG and logged as `rr.Image` or `rr.EncodedImage`.
    - 1D NumPy arrays are logged as a series of individual scalars, with each element indexed.
    - Other multi-dimensional arrays are flattened and logged as individual scalars.

    Keys are automatically namespaced with "observation." or "action." if not already present.

    Args:
        observation: An optional dictionary containing observation data to log.
        action: An optional dictionary containing action data to log.
        compress_images: JPEG-encode images before logging. Off by default,
                         because the encode runs inline on the caller's thread —
                         on a bimanual rig with a head camera (four tactile, two
                         wrist, two eyes) it measures ~15 ms per frame versus
                         ~3 ms uncompressed. Called through :class:`RerunLogSink`
                         that thread is a worker, not the record loop, so the
                         cost is paid in dropped display frames rather than
                         overrun ones. It buys lower viewer memory and IPC
                         bandwidth, which is worth having when the viewer is on
                         another machine (``--display_ip``) and not much
                         otherwise.
        log_images:      Log image entities at all. Scalars are always logged, so
                         turning this off thins the camera tiles without making
                         the pose and jaw plots sparse — the way to spend less
                         time here while keeping every curve at full rate.
    """
    if observation:
        for k, v in observation.items():
            if v is None:
                continue
            key = k if str(k).startswith(OBS_PREFIX) else f"{OBS_STR}.{k}"

            if _is_scalar(v):
                rr.log(key, rr.Scalars(float(v)))
            elif isinstance(v, np.ndarray):
                arr = v
                # Convert CHW -> HWC when needed
                if arr.ndim == 3 and arr.shape[0] in (1, 3, 4) and arr.shape[-1] not in (1, 3, 4):
                    arr = np.transpose(arr, (1, 2, 0))
                if arr.ndim == 1:
                    for i, vi in enumerate(arr):
                        rr.log(f"{key}_{i}", rr.Scalars(float(vi)))
                elif log_images:
                    img_entity = rr.Image(arr).compress() if compress_images else rr.Image(arr)
                    rr.log(key, entity=img_entity)

    if action:
        for k, v in action.items():
            if v is None:
                continue
            key = k if str(k).startswith(ACTION_PREFIX) else f"{ACTION}.{k}"

            if _is_scalar(v):
                rr.log(key, rr.Scalars(float(v)))
            elif isinstance(v, np.ndarray):
                if v.ndim == 1:
                    for i, vi in enumerate(v):
                        rr.log(f"{key}_{i}", rr.Scalars(float(vi)))
                else:
                    # Fall back to flattening higher-dimensional arrays
                    flat = v.flatten()
                    for i, vi in enumerate(flat):
                        rr.log(f"{key}_{i}", rr.Scalars(float(vi)))


class RerunLogSink:
    """Log to Rerun from a background thread, so the viewer costs the caller nothing.

    ``log_rerun_data`` measures ~3.2 ms per frame on a bimanual rig with the head
    camera (eight images), and its tail runs to ~10 ms whenever the viewer
    backpressures the SDK's flush. Called inline from a 30 fps record loop that
    already spends ~25 ms reading hardware, that is exactly the few milliseconds
    that turn frames into ``[slow_frame] ... overrun=`` — which is why the data
    team ended up recording with ``--display_data=false``.

    Rerun's Rust bindings release the GIL for the heavy part of ``rr.log``, and on
    Linux the loop paces itself with ``time.sleep``, so a worker does its work
    inside the window the loop is idle anyway. Measured on that same eight-image
    payload against a 24.1 ms viewer-off baseline: **27.2 ms inline, 24.1 ms
    here** — the whole cost, tail included, leaves the loop.

    The queue is **one frame deep and latest-wins**. If the viewer falls behind,
    display frames are dropped rather than blocking capture: what is on screen is
    always worth less than the loop that writes the dataset. Drops are counted and
    reported once, at :meth:`close`.

    Two consequences of logging off-loop, both display-only and both deliberate:

    * Frames reach Rerun a few milliseconds late, so ``log_time`` lags capture by
      up to one loop period. Nothing downstream reads it — the dataset carries its
      own timestamps.
    * The worker holds the images the loop just read while the loop moves on. They
      are the same arrays ``dataset.add_frame`` already owns, so a camera that
      recycled its buffer could tear a *displayed* frame. It cannot touch what is
      recorded.
    """

    def __init__(self, traj_viz: Any = None, name: str = "rerun-sink") -> None:
        self._traj_viz = traj_viz
        # Depth-1 slot rather than a Queue: "latest wins" is the whole policy,
        # and a Queue would need draining to express it.
        self._slot: tuple | None = None
        self._slot_lock = threading.Lock()
        self._wake = threading.Event()
        # Serialises the worker's traj_viz.log against reset_trajectory() from
        # the caller's thread. Held only for the trail update (sub-millisecond),
        # never for the image logging.
        self._viz_lock = threading.Lock()
        self._stop = threading.Event()
        self._submitted = 0
        self._dropped = 0
        self._failures = 0
        self._failure_logged = False
        self._thread = threading.Thread(target=self._run, name=name, daemon=True)
        self._thread.start()

    def submit(
        self,
        observation: RobotObservation | None,
        action: RobotAction | None = None,
        compress_images: bool = False,
        log_images: bool = True,
    ) -> None:
        """Hand one frame to the worker. Never blocks; never raises.

        Replaces whatever the worker has not picked up yet — see the class
        docstring on why dropping is the right failure mode here.
        """
        with self._slot_lock:
            if self._slot is not None:
                self._dropped += 1
            self._slot = (observation, action, compress_images, log_images)
            self._submitted += 1
        self._wake.set()

    def reset_trajectory(self) -> None:
        """Clear the trajectory viz's breadcrumb trails (e.g. at a new episode).

        Also drops any frame still queued: it belongs to the take that just
        ended, and letting it through would seed the new trail with a stale
        point.
        """
        if self._traj_viz is None:
            return
        with self._slot_lock:
            if self._slot is not None:
                self._dropped += 1
                self._slot = None
        with self._viz_lock:
            self._traj_viz.reset()

    def close(self, timeout: float = 2.0) -> None:
        """Drain the pending frame, stop the worker, and report what was dropped."""
        self._stop.set()
        self._wake.set()
        self._thread.join(timeout)
        if self._thread.is_alive():
            logger.warning(f"Rerun sink thread did not exit within {timeout:.1f}s.")
        if self._dropped:
            logger.info(
                f"Rerun display: {self._dropped}/{self._submitted} frames dropped to keep the "
                "loop on time (viewer-side only; nothing recorded was affected)."
            )
        if self._failures:
            logger.warning(f"Rerun display: {self._failures} frames failed to log.")

    # ---- worker ------------------------------------------------------------

    def _run(self) -> None:
        while True:
            # Read the stop flag *before* claiming the slot, so a frame
            # submitted before close() still gets logged on the way out.
            stopping = self._stop.is_set()
            with self._slot_lock:
                item, self._slot = self._slot, None
            if item is not None:
                self._log_one(item)
                continue
            if stopping:
                return
            self._wake.wait(0.1)
            self._wake.clear()

    def _log_one(self, item: tuple) -> None:
        observation, action, compress_images, log_images = item
        try:
            log_rerun_data(
                observation=observation,
                action=action or {},
                compress_images=compress_images,
                log_images=log_images,
            )
            if self._traj_viz is not None:
                with self._viz_lock:
                    self._traj_viz.log(observation)
        except Exception as e:
            # A dead viewer must never take a recording down with it. Warn once,
            # then just count: the point of this class is to stop the display
            # from generating per-frame log noise.
            self._failures += 1
            if not self._failure_logged:
                self._failure_logged = True
                logger.warning(
                    f"Rerun logging failed ({type(e).__name__}: {e}); display frames will be "
                    "dropped from here on. Recording is unaffected."
                )
