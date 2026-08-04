#!/usr/bin/env python

# Copyright 2026 The XenseRobotics Inc. team. All rights reserved.
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

"""Background poller that publishes matched stereo pairs from the Pico headset.

The other cameras in this package run a background thread that reads the
device continuously and caches the newest frame, so the record loop never
waits on hardware (``OpenCVCamera._read_loop``, and the same shape in
RealSense). The Pico head camera had no such thread: it asked the SDK for the
newest frame from inside the record loop, which meant the sampling instant was
whatever the loop happened to be doing.

That is what made the eyes drift apart. The headset sends each eye as its own
message and the left one lands first, so sampling at 30 Hz against a 30 fps
stream lands in the gap between them often enough to matter — measured at 7%
of frames, left ahead by one or two, never behind. The frames themselves were
fine; the moment we looked at them was not.

Polling well above the stream rate closes that gap: every sequence number is
seen as it arrives, so a pair can be held back until both halves are in.
Nothing is published until then, and what a camera reads is always two views
of the same instant.

One poller serves both eyes rather than one per camera. Pairing needs to see
both, and it also halves the SDK calls — two independent pollers would each
ask for both eyes anyway.
"""

from __future__ import annotations

import threading
from typing import Any

import numpy as np
from numpy.typing import NDArray

from lerobot.utils.robot_utils import get_logger

_EYE_INDEX = {"left": 0, "right": 1}

# Poll period. The stream measured 29.9 fps (~33 ms), so this samples each
# frame about four times — enough to see both halves of a pair land without
# spinning hot. It is not a frame rate: nothing is published unless the
# sequence number actually moved.
_POLL_PERIOD_S = 1.0 / 120.0

# How many un-paired frames to keep per eye while waiting for the other half.
# One would do for the measured lag (one or two frames); four is slack for a
# hiccup, and caps the memory at four JPEGs an eye (~18 KB each).
_PENDING_PER_EYE = 4


class StereoPoller:
    """Polls both eyes and publishes the newest frame(s) they agree on.

    With both eyes subscribed, a pair is published only once both carry the
    same ``frame_sequence`` — so a reader can never see left and right from
    different captures. With one eye subscribed there is nothing to match and
    its frames are published as they arrive.
    """

    def __init__(self, xrt: Any, eyes: tuple[str, ...], expect_size: tuple[int, int]):
        self._xrt = xrt
        self._eyes = tuple(eyes)
        self._expect_size = expect_size
        self.logger = get_logger("PicoStereoPoller")

        self._lock = threading.Lock()
        self._new_frame_event = threading.Event()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

        # eye -> {sequence: (meta, decoded_rgb)} awaiting its counterpart.
        self._pending: dict[str, dict[int, tuple[dict[str, Any], NDArray[np.uint8]]]] = {eye: {} for eye in self._eyes}
        # Published: eye -> (meta, rgb), all from one sequence.
        self._published: dict[str, tuple[dict[str, Any], NDArray[np.uint8]]] = {}
        self._published_seq: int | None = None

        self._size_error: str | None = None
        self.decode_failures = 0
        self.dropped_unpaired = 0

    # ---- lifecycle ----------------------------------------------------------

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._poll_loop, name="pico-stereo-poller", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._thread = None

    @property
    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    # ---- reading ------------------------------------------------------------

    def wait_for_frame(self, timeout_s: float) -> bool:
        """Block until a pair has been published. False on timeout."""
        return self._new_frame_event.wait(timeout=timeout_s)

    def latest(self, eye: str) -> tuple[dict[str, Any], NDArray[np.uint8]] | None:
        """Newest published (metadata, RGB) for one eye, or None.

        When both eyes are subscribed, the value returned here and the one for
        the other eye are guaranteed to share a ``frame_sequence``.
        """
        with self._lock:
            got = self._published.get(eye)
            if got is None:
                return None
            meta, rgb = got
            return meta, rgb

    def raise_if_size_mismatch(self) -> None:
        """Re-raise a size mismatch seen by the thread, on the caller's stack.

        Rescaling would quietly change the recorded field of view, so this is
        an error rather than something to paper over — but it is detected in
        the background, and an exception there would only be logged.
        """
        with self._lock:
            msg = self._size_error
        if msg:
            raise ValueError(msg)

    # ---- internals ----------------------------------------------------------

    def _poll_loop(self) -> None:
        while not self._stop_event.wait(_POLL_PERIOD_S):
            try:
                self._poll_once()
            except Exception as e:  # pragma: no cover — defensive
                self.logger.warn(f"Pico stereo poll failed: {e}")

    def _poll_once(self) -> None:
        import cv2

        for eye in self._eyes:
            idx = _EYE_INDEX[eye]
            if not self._xrt.has_pico_camera_frame(idx):
                continue
            meta = self._xrt.get_pico_camera_frame_metadata(idx)
            seq = int(meta["frame_sequence"])

            with self._lock:
                known = seq in self._pending[eye] or (self._published_seq is not None and seq <= self._published_seq)
            if known:
                continue

            w, h = int(meta["width"]), int(meta["height"])
            if (w, h) != self._expect_size:
                with self._lock:
                    self._size_error = (
                        f"Pico {eye} eye is sending {w}x{h} but the camera is configured "
                        f"for {self._expect_size[0]}x{self._expect_size[1]}. Rescaling would "
                        "silently change the recorded field of view, so fix one side to "
                        "match the other: either set --robot.head_camera_width/_height, or "
                        "change the headset app."
                    )
                continue

            jpeg = bytes(self._xrt.get_pico_camera_frame_jpeg(idx))
            bgr = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if bgr is None:
                self.decode_failures += 1
                continue
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

            with self._lock:
                self._pending[eye][seq] = (meta, rgb)
                # Bound the wait: an eye that never gets its counterpart (the
                # other one stalled, say) must not grow without limit.
                if len(self._pending[eye]) > _PENDING_PER_EYE:
                    oldest = min(self._pending[eye])
                    del self._pending[eye][oldest]
                    self.dropped_unpaired += 1

        self._publish_newest_complete()

    def _publish_newest_complete(self) -> None:
        with self._lock:
            # Match only across eyes that have actually produced something. The
            # poller always watches both, but if the headset is sending one —
            # or the other has not started yet — waiting for a pair that will
            # never exist would publish nothing at all.
            active = [eye for eye in self._eyes if self._pending[eye] or eye in self._published]
            if not active:
                return
            common = set(self._pending[active[0]])
            for eye in active[1:]:
                common &= set(self._pending[eye])
            if not common:
                return
            seq = max(common)
            if self._published_seq is not None and seq <= self._published_seq:
                return

            self._published = {eye: self._pending[eye][seq] for eye in active}
            self._published_seq = seq
            # Everything at or below the published sequence is now spent.
            for eye in active:
                self._pending[eye] = {s: v for s, v in self._pending[eye].items() if s > seq}

        self._new_frame_event.set()
