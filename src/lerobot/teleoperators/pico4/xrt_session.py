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

"""Process-wide lifecycle for the XenseVR PC Service SDK (``xrt``).

``xrt.init()`` is a process-level singleton: it opens one connection to the
service and spawns background threads behind it. Two unrelated consumers now
need that connection at the same time — :class:`~lerobot.teleoperators.pico4.
tracker.Pico4TrackerReader` for the 6-DoF tracker poses, and
:class:`~lerobot.cameras.pico.camera_pico.PicoCamera` for the head camera
frames the service relays as ``0x30`` custom messages. Both are optional and
either can be turned off, so neither can own the SDK.

Hence the hold count here. Whoever needs the SDK alive calls :func:`acquire`
and later :func:`release`; the SDK is closed only when the last holder lets
go. Without this, turning the head camera off at the end of an episode would
also drop the tracker's connection.

``close()`` matters more than it looks: the SDK's background ``std::thread``s
are joinable, and destroying them at process exit without joining calls
``std::terminate()`` — a core dump on the way out. :func:`shutdown` is
registered with ``atexit`` on first load so that even a path which never
releases (discovery-only use, or an exception between load and acquire) still
tears the SDK down cleanly.
"""

from __future__ import annotations

import atexit
import threading
from types import ModuleType

from lerobot.utils.robot_utils import get_logger

logger = get_logger("xrt_session")

_lock = threading.Lock()
_xrt: ModuleType | None = None
_holders: int = 0
_atexit_registered: bool = False

_IMPORT_HINT = "Build the pybind under src/lerobot/teleoperators/pico4/xensevr-pc-service-pybind/."


def _load_locked(purpose: str) -> tuple[ModuleType, bool]:
    """Import + ``init()`` the SDK if it is not up yet. Caller holds ``_lock``.

    Returns the module and whether this call is what initialized it, so the
    caller can log "initialized" versus "reusing singleton" as before.
    """
    global _xrt, _atexit_registered

    if _xrt is not None:
        return _xrt, False

    try:
        import xensevr_pc_service_sdk as xrt
    except ImportError as e:
        raise ImportError(f"xensevr_pc_service_sdk is required for {purpose}. {_IMPORT_HINT}") from e

    xrt.init()
    _xrt = xrt
    if not _atexit_registered:
        atexit.register(shutdown)
        _atexit_registered = True
    return _xrt, True


def ensure_loaded(purpose: str = "the Pico4 SDK") -> tuple[ModuleType, bool]:
    """Bring the SDK up without taking a hold on it.

    For callers that only need a few reads and are happy for the atexit hook
    to do the closing — tracker discovery being the case that exists today.
    Taking a hold here instead would leak one per discovery call.

    Args:
        purpose: named in the ImportError when the pybind is missing.

    Returns:
        ``(xrt_module, did_initialize)``.
    """
    with _lock:
        return _load_locked(purpose)


def acquire(purpose: str = "the Pico4 SDK") -> tuple[ModuleType, bool]:
    """Bring the SDK up and register one holder. Pair with :func:`release`.

    Returns:
        ``(xrt_module, did_initialize)``.
    """
    global _holders
    with _lock:
        xrt, did_init = _load_locked(purpose)
        _holders += 1
        return xrt, did_init


def hold() -> None:
    """Register a holder for an SDK that is already up.

    Split out from :func:`acquire` for the tracker, which loads the SDK early
    in ``connect()`` but only commits to holding it once the device actually
    resolved and its poller thread is running. A failure in between leaves the
    SDK loaded with no holder, which the atexit hook cleans up — the behaviour
    before this module existed.
    """
    global _holders
    with _lock:
        _holders += 1


def release() -> bool:
    """Drop one holder, closing the SDK when the last one lets go.

    Returns:
        True if this call closed the SDK, so the caller can log accordingly.
    """
    global _xrt, _holders
    with _lock:
        if _holders > 0:
            _holders -= 1
        if _holders > 0 or _xrt is None:
            return False
        # Swallowed on purpose: a failing close() must not mask whatever the
        # caller was actually doing (usually its own teardown). The caller logs
        # the lifecycle ("last reader; SDK closed"); state is reset either way.
        # But it is logged rather than dropped — a close() that did not take is
        # exactly what leaves the SDK's joinable threads to std::terminate() the
        # process on the way out, and silently discarding the reason turns that
        # into an unexplained crash at exit.
        try:
            _xrt.close()
        except Exception as e:  # pragma: no cover — teardown-only path
            logger.warning(f"xrt.close() failed on release ({type(e).__name__}: {e}); SDK state reset anyway")
        _xrt = None
        return True


def shutdown() -> None:
    """Close the SDK unconditionally. Registered with ``atexit``; idempotent."""
    global _xrt, _holders
    with _lock:
        if _xrt is not None:
            if _holders:
                # Reached atexit with holders still registered: someone skipped
                # its release(). Harmless here (we close anyway) but it means a
                # disconnect path did not run, so say so while there is still a
                # log to say it in.
                logger.debug(f"atexit shutdown with {_holders} hold(s) outstanding; closing SDK anyway")
            try:
                _xrt.close()
            except Exception as e:  # pragma: no cover — exit-only path
                logger.warning(f"xrt.close() failed at shutdown ({type(e).__name__}: {e})")
        _xrt = None
        _holders = 0


def module() -> ModuleType | None:
    """The live SDK module, or None if it is not up."""
    return _xrt


def holders() -> int:
    """Current hold count. For tests and diagnostics."""
    return _holders
