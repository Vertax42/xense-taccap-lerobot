# Copyright 2025 The HuggingFace & XenseRobotics Inc. team. All rights reserved.
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
import os
import platform
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
import spdlog

SPDLOG_PATTERN = "[%D %T.%e] [%n] [%^%l%$] %v"
# Aspirational: this spdlog pybind exposes only `set_level` on a Sink, so a
# pattern can be set on the logger but not per sink. Both sinks therefore render
# SPDLOG_PATTERN; the file sink simply drops the `%^`/`%$` colour markers, which
# only the ansicolor console sink emits. Wire this up if the binding ever grows
# Sink.set_pattern.
FILE_LOG_PATTERN = "[%Y-%m-%d %H:%M:%S.%e] [%n] [%l] %v"

# Global log directory — set via XENSE_LOG_DIR env var, defaults to ~/xenselogs.
_LOG_DIR = Path(os.environ.get("XENSE_LOG_DIR", Path.home() / "xenselogs"))
_LOG_SESSION = datetime.now().strftime("%Y%m%d_%H%M%S")
_MAX_LOG_FILES = 15

# Default console verbosity for every logger in the process. Overridable per rig
# without touching code — handy when a station is misbehaving and you want DEBUG
# for one session only.
DEFAULT_CONSOLE_LEVEL = os.environ.get("XENSE_LOG_LEVEL", "INFO")

_SPDLOG_LEVEL_MAP = {
    "TRACE": spdlog.LogLevel.TRACE,
    "DEBUG": spdlog.LogLevel.DEBUG,
    "INFO": spdlog.LogLevel.INFO,
    "WARN": spdlog.LogLevel.WARN,
    "WARNING": spdlog.LogLevel.WARN,
    "ERR": spdlog.LogLevel.ERR,
    "ERROR": spdlog.LogLevel.ERR,
    "CRITICAL": spdlog.LogLevel.CRITICAL,
    "OFF": spdlog.LogLevel.OFF,
}

# Third-party loggers that emit per-request/per-frame chatter at INFO or DEBUG.
# They are pinned to WARNING so the shared session file stays readable; anything
# actually wrong still gets through.
_NOISY_STDLIB_LOGGERS = (
    "asyncio",
    "botocore",
    "datasets",
    "filelock",
    "fsspec",
    "huggingface_hub",
    "matplotlib",
    "numba",
    "PIL",
    "urllib3",
)

# Shared sinks. One file sink per session and one console sink per level, shared
# by every logger: a sink owns a lock and a flush, so minting a fresh one per
# logger multiplies both for no benefit.
_file_sink: spdlog.Sink | None = None
_console_sinks: dict[int, spdlog.Sink] = {}
_loggers: dict[tuple[str, str], spdlog.Logger] = {}
_LOG_LOCK = threading.RLock()


def _get_file_sink() -> spdlog.Sink | None:
    """Lazily create a shared file sink for the current session."""
    global _file_sink
    if _file_sink is not None:
        return _file_sink
    with _LOG_LOCK:
        if _file_sink is not None:
            return _file_sink
        try:
            _LOG_DIR.mkdir(parents=True, exist_ok=True)
            # Rotate: keep newest _MAX_LOG_FILES - 1, drop the rest.
            log_files = sorted(_LOG_DIR.glob("*.log"), key=lambda f: f.stat().st_mtime)
            while len(log_files) >= _MAX_LOG_FILES:
                log_files.pop(0).unlink(missing_ok=True)
            log_path = _LOG_DIR / f"session_{_LOG_SESSION}.log"
            sink = spdlog.basic_file_sink_mt(str(log_path))
            sink.set_level(spdlog.LogLevel.DEBUG)
            _file_sink = sink
            return _file_sink
        except Exception:
            return None


def _get_console_sink(level: spdlog.LogLevel) -> spdlog.Sink:
    """Return the process-wide console sink for ``level``, creating it once."""
    with _LOG_LOCK:
        sink = _console_sinks.get(int(level))
        if sink is None:
            sink = spdlog.stdout_color_sink_mt()
            sink.set_level(level)
            _console_sinks[int(level)] = sink
        return sink


def session_log_path() -> Path:
    """Path of this session's log file (whether or not the sink opened)."""
    return _LOG_DIR / f"session_{_LOG_SESSION}.log"


class _SinkLogger(spdlog.SinkLogger):
    """``spdlog.SinkLogger`` that also answers to the stdlib method names.

    spdlog spells them ``warn`` and (for exceptions) ``error``; the stdlib spells
    them ``warning`` and ``exception``. Both spellings appear across this tree
    because loggers were migrated from ``logging`` piecemeal, and the mismatch is
    invisible until the branch that logs actually runs — an AttributeError raised
    from inside a teleoperation loop, at the exact moment something else already
    went wrong. Aliasing costs nothing and removes the failure mode.

    The instances are C-extension objects with no ``__dict__``, so this has to be
    a subclass rather than an attribute patched on after construction.
    """

    warning = spdlog.SinkLogger.warn
    exception = spdlog.SinkLogger.error


def get_logger(name: str, loglevel: str | None = None) -> spdlog.Logger:
    """Create (or fetch) a spdlog logger with console + file output.

    Console shows ``loglevel`` and above with colors. The file sink captures
    DEBUG+ to ``~/xenselogs/session_<timestamp>.log`` (override the directory
    via the ``XENSE_LOG_DIR`` env var). Old log files in that directory are
    rotated; only the most recent :data:`_MAX_LOG_FILES` are kept.

    Loggers are cached per ``(name, loglevel)``. Calling this twice for the same
    name — a robot reconnecting, a module imported from two entry points — hands
    back the same logger instead of stacking another console sink onto stdout,
    which is how the same line ends up printed twice.

    Args:
        name: Logger name.
        loglevel: Console log level: TRACE/DEBUG/INFO/WARN/ERR/CRITICAL/OFF.
            Defaults to ``$XENSE_LOG_LEVEL`` or INFO.

    Returns:
        spdlog logger that fans out to the console and the shared session file.
    """
    level_name = (loglevel or DEFAULT_CONSOLE_LEVEL).upper()
    key = (name, level_name)

    with _LOG_LOCK:
        cached = _loggers.get(key)
        if cached is not None:
            return cached

        console_level = _SPDLOG_LEVEL_MAP.get(level_name, spdlog.LogLevel.INFO)
        sinks = [_get_console_sink(console_level)]

        file_sink = _get_file_sink()
        if file_sink is not None:
            sinks.append(file_sink)

        logger = _SinkLogger(name, sinks)
        logger.set_pattern(SPDLOG_PATTERN)
        # Logger-level DEBUG so the file sink sees everything; each sink filters
        # to its own configured level.
        logger.set_level(spdlog.LogLevel.DEBUG)
        logger.flush_on(spdlog.LogLevel.WARN)
        _loggers[key] = logger
        return logger


class _StdlibToSpdlogHandler(logging.Handler):
    """Forward stdlib ``logging`` records into spdlog.

    Most of this tree logs through :func:`get_logger`, but upstream lerobot,
    ``xensesdk`` and libav all log through the stdlib. Without a bridge those
    lines take a different path to a different stream with a different format —
    they are the ``INFO 2026-08-27 16:58:52 eo_utils.py:189`` lines, whose
    location field is the *tail* of the path truncated to 15 characters
    (``video_utils.py`` reads as ``eo_utils.py``), and they never reach the
    session file at all. Routing them here gives one format, one stream and one
    file for the whole process.

    The spdlog logger name is the record's module, not ``root`` — upstream logs
    via the root logger, so the module is the only part that identifies anything.
    """

    _LEVEL_METHODS = (
        (logging.CRITICAL, "critical"),
        (logging.ERROR, "error"),
        (logging.WARNING, "warn"),
        (logging.INFO, "info"),
        (logging.DEBUG, "debug"),
    )

    def __init__(self, level: int = logging.INFO):
        super().__init__(level=level)
        self._level_name = logging.getLevelName(level)

    @staticmethod
    def _logger_name(record: logging.LogRecord) -> str:
        # Root records carry no useful name; named third-party loggers do.
        if record.name and record.name != "root":
            return record.name
        return record.module or "python"

    def emit(self, record: logging.LogRecord) -> None:
        try:
            message = record.getMessage().rstrip()
            # Warnings and worse are the ones someone will go read the source
            # for, so they keep their origin; INFO stays uncluttered. Records
            # routed by ``logging.captureWarnings`` are the exception: the
            # formatted warning already opens with "<file>:<line>: UserWarning",
            # and appending warnings.py's own location on top says nothing.
            if record.levelno >= logging.WARNING and record.name != "py.warnings":
                message = f"{message} ({record.filename}:{record.lineno})"
            if record.exc_info:
                message = f"{message}\n{logging.Formatter().formatException(record.exc_info)}"
            logger = get_logger(self._logger_name(record), self._level_name)
            for levelno, method in self._LEVEL_METHODS:
                if record.levelno >= levelno:
                    getattr(logger, method)(message)
                    return
            logger.debug(message)
        except Exception:
            self.handleError(record)


def install_stdlib_bridge(console_level: str | None = None, quiet_third_party: bool = True) -> logging.Handler:
    """Point the stdlib root logger at spdlog and return the installed handler.

    Idempotent: re-installing replaces the previous bridge rather than stacking a
    second one. Existing root handlers are dropped, which is what the caller
    wants — leaving them in place is how every line gets printed twice.

    The handler filters at ``console_level`` *before* spdlog sees the record, so
    the DEBUG-level session file does not fill up with third-party DEBUG chatter
    that nothing asked for.
    """
    level_name = (console_level or DEFAULT_CONSOLE_LEVEL).upper()
    level = logging.getLevelName(level_name)
    if not isinstance(level, int):
        level = logging.INFO

    root = logging.getLogger()
    for existing in list(root.handlers):
        root.removeHandler(existing)

    handler = _StdlibToSpdlogHandler(level=level)
    root.addHandler(handler)
    # NOTSET on the root logger means "hand everything to the handlers"; the
    # handler's own level is the filter.
    root.setLevel(logging.NOTSET)

    # `warnings.warn` output otherwise goes straight to stderr, unformatted and
    # absent from the session file.
    logging.captureWarnings(True)

    if quiet_third_party:
        for noisy in _NOISY_STDLIB_LOGGERS:
            logging.getLogger(noisy).setLevel(logging.WARNING)

    return handler


# Whether captured frames are currently being written into a dataset. A stall
# only costs anything when its stale frames land in an episode: during encoder
# warm-up, the between-episode reset and the save/encode gap, nothing is
# recording and a stalled capture damages nothing, so warning about it is noise.
# The measured run showed exactly that — the first stalls of a session fired
# during `[streaming_encoder] warming up`, entirely before "Recording episode 0".
#
# Process-global on purpose: a camera is a leaf object with no reference back to
# the recording session, and "is a dataset being written right now" is genuinely
# a property of the process, not of any one camera. Defaults to active so a
# consumer that never manages the gate (teleoperate, ad-hoc scripts) still gets
# the diagnostic; `lerobot-record` brackets its episodes explicitly.
_RECORDING_ACTIVE = threading.Event()
_RECORDING_ACTIVE.set()


def set_capture_recording(active: bool) -> None:
    """Open or close the window in which capture stalls are worth reporting."""
    if active:
        _RECORDING_ACTIVE.set()
    else:
        _RECORDING_ACTIVE.clear()


def capture_recording_active() -> bool:
    """Whether captured frames are currently being recorded."""
    return _RECORDING_ACTIVE.is_set()


class CaptureStallMonitor:
    """Report stalls in a camera's **background** capture loop.

    Replaces the per-call ``logger.debug(f"{self} read took: {ms:.1f}ms")`` that
    each backend used to emit. On a bimanual rig that is six cameras at 30 Hz —
    180 records a second, ~1 MiB a minute — none of which reaches the console
    (the sink filters at INFO) and all of which reaches the DEBUG-level session
    file, where it buries everything else. The number was only ever interesting
    when it was large.

    **What a stall here means.** Every backend runs a background thread that
    captures into ``latest_frame``; the record loop takes frames with
    ``async_read()``, which returns that cached frame **without blocking**. So a
    slow capture never stalls the record loop — it means the loop keeps being
    handed the *same* frame, and roughly ``duration / budget`` recorded frames
    are duplicates of one image. That is a data-quality problem, not a timing
    one, and the messages below say so: reading them as "the loop was blocked"
    sends you looking in the wrong place.

    Nothing else sees this. ``[slow_frame]`` in the record loop cannot — the loop
    is not blocked. ``CameraReadGuard``'s freeze detection cannot either: its
    ``CAM_FREEZE_TIMEOUT_S`` is 2s, deliberately well above any frame interval so
    a slow sensor never trips it, and the stalls that matter here are shorter.

    The first stall in a window is logged as it happens, so onset keeps a
    timestamp; the rest of the window is counted and folded into one summary,
    which carries the worst stall's own wall-clock time — that is what tells you
    whether it landed inside an episode or in the encode/save gap between two.

    ``str(owner)`` is resolved only when a warning is actually emitted — building
    it per call is the cost this class exists to avoid.
    """

    def __init__(
        self,
        logger: Any,
        owner: Any = None,
        *,
        label: str = "capture",
        overrun_factor: float = 2.0,
        min_overrun_ms: float = 5.0,
        report_every_s: float = 10.0,
    ):
        self._logger = logger
        self._owner = owner
        self._label = label
        self._overrun_factor = overrun_factor
        self._min_overrun_ms = min_overrun_ms
        self._report_every_s = report_every_s
        self._window_start: float | None = None
        self._total = 0
        self._slow = 0
        self._worst_ms = 0.0
        self._worst_at = ""
        self._worst_stale = 0
        # Whether the window that just closed had stalls. Gates the onset line: a
        # camera stalling window after window should cost one summary each, not a
        # summary plus a "first stall" that repeats it.
        self._was_slow = False
        # Stalls already reported individually this window (0 or 1). The summary
        # covers whatever this does not, so a camera stalling exactly once per
        # window keeps being reported instead of going quiet after its onset line.
        self._announced = 0

    def _prefix(self) -> str:
        return f"{self._owner} " if self._owner is not None else ""

    def _threshold_ms(self, budget_ms: float | None) -> float | None:
        if not budget_ms or budget_ms <= 0:
            # No configured rate means no budget to overrun, so nothing to say.
            return None
        # Both terms matter: the factor keeps fast cameras from tripping on a
        # couple of milliseconds, the floor keeps very high frame rates from
        # warning on jitter that is under a millisecond of real delay.
        return max(budget_ms * self._overrun_factor, budget_ms + self._min_overrun_ms)

    def _reset(self, now: float) -> None:
        self._window_start = now
        self._was_slow = self._slow > 0
        self._total = 0
        self._slow = 0
        self._worst_ms = 0.0
        self._worst_at = ""
        self._worst_stale = 0
        self._announced = 0

    def observe(self, duration_ms: float, budget_ms: float | None = None) -> None:
        """Record one capture's duration and warn if the budget is clearly blown.

        A no-op while nothing is being recorded — see :data:`_RECORDING_ACTIVE`.
        The window is dropped rather than carried across the pause, so the first
        stall of the next episode is reported as an onset instead of being folded
        into a summary spanning the gap.
        """
        threshold_ms = self._threshold_ms(budget_ms)
        if threshold_ms is None:
            return

        if not _RECORDING_ACTIVE.is_set():
            self._reset(time.perf_counter())
            # _reset carries "the window that just closed was bad" forward; across
            # a recording pause there is nothing to carry, and window_start=None
            # makes the next active capture start a fresh window rather than one
            # already stretched across the gap.
            self._was_slow = False
            self._window_start = None
            return

        now = time.perf_counter()
        if self._window_start is None:
            self._window_start = now
        self._total += 1

        if duration_ms > threshold_ms:
            self._slow += 1
            # Frames a non-blocking consumer was handed the stale image for.
            stale = max(1, int(duration_ms / budget_ms))
            if duration_ms > self._worst_ms:
                self._worst_ms = duration_ms
                self._worst_stale = stale
                self._worst_at = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            # Onset only: the first stall after a clean window, so the moment
            # things went wrong keeps its own timestamp. While it stays wrong,
            # the window summary below is the whole report.
            if self._slow == 1 and not self._was_slow:
                self._announced = 1
                self._logger.warning(
                    f"{self._prefix()}background {self._label} stalled {duration_ms:.1f}ms "
                    f"({stale}x the {budget_ms:.1f}ms budget) — async_read() served the same frame "
                    f"throughout, so ~{stale} recorded frames repeat one image. "
                    f"Further stalls are summarised every {self._report_every_s:g}s, not logged one by one"
                )

        elapsed = now - self._window_start
        if elapsed >= self._report_every_s:
            if self._slow > self._announced:
                self._logger.warning(
                    f"{self._prefix()}{self._slow}/{self._total} {self._label}s over "
                    f"{threshold_ms:.1f}ms in the last {elapsed:.1f}s "
                    f"(worst {self._worst_ms:.1f}ms at {self._worst_at}, ~{self._worst_stale} frames stale)"
                )
            self._reset(now)


def busy_wait(seconds):
    if platform.system() == "Darwin" or platform.system() == "Windows":
        # On Mac and Windows, `time.sleep` is not accurate and we need to use this while loop trick,
        # but it consumes CPU cycles.
        end_time = time.perf_counter() + seconds
        while time.perf_counter() < end_time:
            pass
    else:
        # On Linux time.sleep is accurate
        if seconds > 0:
            time.sleep(seconds)


def precise_sleep(seconds: float, spin_threshold: float = 0.010, sleep_margin: float = 0.005):
    """
    Wait for `seconds` with better precision than time.sleep alone at the expense of more CPU usage.

    Parameters:
      - seconds: duration to wait
      - spin_threshold: if remaining <= spin_threshold -> spin; otherwise sleep (seconds). Default 10ms
      - sleep_margin: when sleeping leave this much time before deadline to avoid oversleep. Default 5ms

    Note:
        The default parameters are chosen to prioritize timing accuracy over CPU usage for the common 30 FPS use case.
    """
    if seconds <= 0:
        return

    system = platform.system()
    if system in ("Darwin", "Windows"):
        end_time = time.perf_counter() + seconds
        while True:
            remaining = end_time - time.perf_counter()
            if remaining <= 0:
                break
            if remaining > spin_threshold:
                time.sleep(max(remaining - sleep_margin, 0))
            else:
                pass
    else:
        time.sleep(seconds)


def xyz_rpy_to_matrix(pose: np.ndarray) -> np.ndarray:
    """
    Convert position and RPY angles to 4x4 transformation matrix.

    Args:
        pose: 6D array [x, y, z, roll, pitch, yaw]
              - x, y, z: Position coordinates
              - roll, pitch, yaw: Euler angles in radians

    Returns:
        4x4 transformation matrix
    """
    if pose.shape != (6,):
        raise ValueError(f"Expected pose array of shape (6,), got {pose.shape}")

    x, y, z = pose[0], pose[1], pose[2]
    roll, pitch, yaw = pose[3], pose[4], pose[5]

    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    rot_matrix = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, x],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, y],
            [-sp, cp * sr, cp * cr, z],
            [0, 0, 0, 1],
        ]
    )
    return rot_matrix


def quaternion_to_matrix(
    pose: np.ndarray,
    input_format: str = "wxyz",
) -> np.ndarray:
    """
    Convert position and quaternion to 4x4 transformation matrix.

    Args:
        pose: 7D array containing position and quaternion.
              Format depends on input_format parameter:
              - "xyzw": [x, y, z, qx, qy, qz, qw] (scalar-last)
              - "wxyz": [x, y, z, qw, qx, qy, qz] (scalar-first)
        input_format: Quaternion format, either "xyzw" (scalar-last) or "wxyz" (scalar-first).
                      Default is "xyzw".

    Returns:
        4x4 transformation matrix
    """
    if pose.shape != (7,):
        raise ValueError(f"Expected pose array of shape (7,), got {pose.shape}")

    x, y, z = pose[0], pose[1], pose[2]

    if input_format == "xyzw":
        qx, qy, qz, qw = pose[3], pose[4], pose[5], pose[6]
    elif input_format == "wxyz":
        qw, qx, qy, qz = pose[3], pose[4], pose[5], pose[6]
    else:
        raise ValueError(f"Unknown input_format: {input_format}. Expected 'xyzw' or 'wxyz'.")

    rot_matrix = np.array(
        [
            [
                1 - 2 * qy * qy - 2 * qz * qz,
                2 * qx * qy - 2 * qz * qw,
                2 * qx * qz + 2 * qy * qw,
                x,
            ],
            [
                2 * qx * qy + 2 * qz * qw,
                1 - 2 * qx * qx - 2 * qz * qz,
                2 * qy * qz - 2 * qx * qw,
                y,
            ],
            [
                2 * qx * qz - 2 * qy * qw,
                2 * qy * qz + 2 * qx * qw,
                1 - 2 * qx * qx - 2 * qy * qy,
                z,
            ],
            [0, 0, 0, 1],
        ]
    )
    return rot_matrix


def matrix_to_pose7d(matrix: np.ndarray, output_format: str = "wxyz") -> np.ndarray:
    """
    Convert 4x4 transformation matrix to 7D pose [x, y, z, qw, qx, qy, qz].

    Args:
        matrix: 4x4 transformation matrix
        output_format: Quaternion output format:
            - "xyzw": [x, y, z, qx, qy, qz, qw] (scalar-last)
            - "wxyz": [x, y, z, qw, qx, qy, qz] (scalar-first)
            Default is "wxyz".

    Returns:
        7D array containing position and quaternion
    """
    # Extract position
    x = matrix[0, 3]
    y = matrix[1, 3]
    z = matrix[2, 3]

    # Extract rotation matrix
    rot_matrix = matrix[:3, :3]

    # Calculate quaternion using Shepperd's method
    trace = rot_matrix[0, 0] + rot_matrix[1, 1] + rot_matrix[2, 2]

    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (rot_matrix[2, 1] - rot_matrix[1, 2]) * s
        qy = (rot_matrix[0, 2] - rot_matrix[2, 0]) * s
        qz = (rot_matrix[1, 0] - rot_matrix[0, 1]) * s
    elif rot_matrix[0, 0] > rot_matrix[1, 1] and rot_matrix[0, 0] > rot_matrix[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rot_matrix[0, 0] - rot_matrix[1, 1] - rot_matrix[2, 2])
        qw = (rot_matrix[2, 1] - rot_matrix[1, 2]) / s
        qx = 0.25 * s
        qy = (rot_matrix[0, 1] + rot_matrix[1, 0]) / s
        qz = (rot_matrix[0, 2] + rot_matrix[2, 0]) / s
    elif rot_matrix[1, 1] > rot_matrix[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rot_matrix[1, 1] - rot_matrix[0, 0] - rot_matrix[2, 2])
        qw = (rot_matrix[0, 2] - rot_matrix[2, 0]) / s
        qx = (rot_matrix[0, 1] + rot_matrix[1, 0]) / s
        qy = 0.25 * s
        qz = (rot_matrix[1, 2] + rot_matrix[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + rot_matrix[2, 2] - rot_matrix[0, 0] - rot_matrix[1, 1])
        qw = (rot_matrix[1, 0] - rot_matrix[0, 1]) / s
        qx = (rot_matrix[0, 2] + rot_matrix[2, 0]) / s
        qy = (rot_matrix[1, 2] + rot_matrix[2, 1]) / s
        qz = 0.25 * s

    if output_format == "xyzw":
        return np.array([x, y, z, qx, qy, qz, qw])
    elif output_format == "wxyz":
        return np.array([x, y, z, qw, qx, qy, qz])
    else:
        raise ValueError(f"Unknown output_format: {output_format}. Use 'xyzw' or 'wxyz'.")


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Convert Euler angles (roll, pitch, yaw) to quaternion [qw, qx, qy, qz].

    Uses ZYX intrinsic rotation order (yaw → pitch → roll), which is:
    - First rotate around Z-axis by yaw
    - Then rotate around Y-axis by pitch
    - Finally rotate around X-axis by roll

    This is consistent with Flexiv SDK convention and aerospace/aviation standard.

    Args:
        roll: Rotation around x-axis in radians
        pitch: Rotation around y-axis in radians
        yaw: Rotation around z-axis in radians

    Returns:
        np.ndarray of shape (4,) in [qw, qx, qy, qz] format.
    """
    cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)
    cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
    cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)

    return np.array(
        [
            cr * cp * cy + sr * sp * sy,  # qw
            sr * cp * cy - cr * sp * sy,  # qx
            cr * sp * cy + sr * cp * sy,  # qy
            cr * cp * sy - sr * sp * cy,  # qz
        ],
        dtype=np.float32,
    )


def quaternion_to_euler(qw: float, qx: float, qy: float, qz: float) -> np.ndarray:
    """Convert quaternion [qw, qx, qy, qz] to Euler angles (roll, pitch, yaw).

    Uses ZYX intrinsic rotation order, consistent with Flexiv SDK and aerospace standard.
    This is the inverse of euler_to_quaternion().

    Note: Gimbal lock occurs when pitch ≈ ±90°, causing roll and yaw to become coupled.

    Args:
        qw: Quaternion scalar component
        qx: Quaternion x component
        qy: Quaternion y component
        qz: Quaternion z component

    Returns:
        np.ndarray of shape (3,) in [roll, pitch, yaw] order (radians):
        - roll: Rotation around x-axis, range [-π, π]
        - pitch: Rotation around y-axis, range [-π/2, π/2]
        - yaw: Rotation around z-axis, range [-π, π]
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation) with gimbal lock handling
    sinp = np.clip(2.0 * (qw * qy - qz * qx), -1.0, 1.0)
    pitch = np.arcsin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return np.array([roll, pitch, yaw], dtype=np.float32)


def quaternion_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Multiply two quaternions q1 * q2.

    Args:
        q1: First quaternion [qw, qx, qy, qz]
        q2: Second quaternion [qw, qx, qy, qz]

    Returns:
        np.ndarray of shape (4,) representing q1 * q2 in [qw, qx, qy, qz] format.
    """
    w1, x1, y1, z1 = q1[0], q1[1], q1[2], q1[3]
    w2, x2, y2, z2 = q2[0], q2[1], q2[2], q2[3]

    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=np.float32,
    )


def slerp_quaternion(q1: np.ndarray, q2: np.ndarray, t: float, input_format: str = "wxyz") -> np.ndarray:
    """Spherical Linear Interpolation (SLERP) between two quaternions.

    Args:
        q1: First quaternion [qw, qx, qy, qz]
        q2: Second quaternion [qw, qx, qy, qz]
        t: Interpolation factor [0, 1], where 0 returns q1 and 1 returns q2
        input_format: Input quaternion format:
            - "wxyz": [qw, qx, qy, qz] format (Flexiv, scipy)
            - "xyzw": [qx, qy, qz, qw] format (Pico4, ROS, OpenGL)
            Default is "wxyz".

    Returns:
        Interpolated quaternion [qw, qx, qy, qz]
    """
    q1 = normalize_quaternion(q1, input_format=input_format)
    q2 = normalize_quaternion(q2, input_format=input_format)

    dot = np.dot(q1, q2)

    if dot < 0.0:
        q2 = -q2
        dot = -dot

    dot = np.clip(dot, -1.0, 1.0)

    if abs(dot) > 0.9995:
        result = q1 + t * (q2 - q1)
        return normalize_quaternion(result, input_format=input_format)

    theta = np.arccos(abs(dot))
    sin_theta = np.sin(theta)

    w1 = np.sin((1 - t) * theta) / sin_theta
    w2 = np.sin(t * theta) / sin_theta

    result = w1 * q1 + w2 * q2

    return normalize_quaternion(result, input_format=input_format)


def normalize_quaternion(q: np.ndarray, input_format: str = "wxyz") -> np.ndarray:
    """Normalize quaternion and convert to [qw, qx, qy, qz] format (Flexiv convention).

    Args:
        q: Quaternion as numpy array with 4 elements
        input_format: Input quaternion format:
            - "wxyz": [qw, qx, qy, qz] format (Flexiv, scipy)
            - "xyzw": [qx, qy, qz, qw] format (Pico4, ROS, OpenGL)

    Returns:
        Normalized quaternion in [qw, qx, qy, qz] format (Flexiv convention)
    """
    q = np.asarray(q, dtype=np.float32)
    if q.ndim > 1:
        q = q.flatten()
    if len(q) != 4:
        raise ValueError(f"Quaternion must have 4 components, got {len(q)}")

    # Check norm and normalize if needed
    norm = np.linalg.norm(q)
    if norm < 1e-10:
        # Invalid quaternion, return identity in input_format
        if input_format == "wxyz":
            return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        elif input_format == "xyzw":
            return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        else:
            raise ValueError(f"Unknown input_format: {input_format}. Use 'wxyz' or 'xyzw'.")

    # Skip normalization if already unit quaternion (|norm - 1| < tolerance)
    if abs(norm - 1.0) > 1e-6:
        q = q / norm

    # Convert to [qw, qx, qy, qz] format
    if input_format == "wxyz":
        # Already in [qw, qx, qy, qz] format
        return q.astype(np.float32)
    elif input_format == "xyzw":
        # Convert from [qx, qy, qz, qw] to [qw, qx, qy, qz]
        return np.array([q[3], q[0], q[1], q[2]], dtype=np.float32)
    else:
        raise ValueError(f"Unknown input_format: {input_format}. Use 'wxyz' or 'xyzw'.")


# =============================================================================
# 6D Rotation Representation (Continuous Rotation Representation)
# Reference: "On the Continuity of Rotation Representations in Neural Networks"
#            Zhou et al., CVPR 2019
#
# 6D representation uses the first two columns of a rotation matrix.
# Advantages over Euler angles and quaternions:
#   - Continuous: No discontinuities at ±180° boundaries (unlike Euler angles)
#   - No double-cover: Unlike quaternions where q and -q represent the same rotation
#   - Better for neural network learning in robotics applications
# =============================================================================


def quaternion_to_rotation_6d(qw: float, qx: float, qy: float, qz: float) -> np.ndarray:
    """Convert quaternion to 6D rotation representation.

    The 6D representation consists of the first two columns of the rotation matrix,
    which can be used to uniquely reconstruct the full rotation matrix via
    Gram-Schmidt orthogonalization.

    Args:
        qw: Quaternion scalar component
        qx: Quaternion x component
        qy: Quaternion y component
        qz: Quaternion z component

    Returns:
        np.ndarray of shape (6,) containing [r1, r2, r3, r4, r5, r6] where:
        - [r1, r2, r3] is the first column of the rotation matrix
        - [r4, r5, r6] is the second column of the rotation matrix
    """
    # Rotation matrix from quaternion
    # First column
    r1 = 1.0 - 2.0 * (qy * qy + qz * qz)
    r2 = 2.0 * (qx * qy + qz * qw)
    r3 = 2.0 * (qx * qz - qy * qw)

    # Second column
    r4 = 2.0 * (qx * qy - qz * qw)
    r5 = 1.0 - 2.0 * (qx * qx + qz * qz)
    r6 = 2.0 * (qy * qz + qx * qw)

    return np.array([r1, r2, r3, r4, r5, r6], dtype=np.float32)


def rotation_6d_to_quaternion(r6d: np.ndarray, ensure_positive_w: bool = True) -> np.ndarray:
    """Convert 6D rotation representation to quaternion.

    Uses Gram-Schmidt orthogonalization to reconstruct the rotation matrix
    from the 6D representation, then converts to quaternion.

    Args:
        r6d: 6D rotation representation [r1, r2, r3, r4, r5, r6]
        ensure_positive_w: If True, ensure qw >= 0 for consistent output.
                          This doesn't change the rotation (q and -q are equivalent).

    Returns:
        np.ndarray of shape (4,) in [qw, qx, qy, qz] format
    """
    r6d = np.asarray(r6d, dtype=np.float64)  # Use float64 for numerical stability
    if r6d.shape != (6,):
        raise ValueError(f"Expected r6d array of shape (6,), got {r6d.shape}")

    # Extract the two column vectors
    a1 = r6d[:3]
    a2 = r6d[3:6]

    # Gram-Schmidt orthogonalization
    b1 = a1 / np.linalg.norm(a1)
    b2 = a2 - np.dot(b1, a2) * b1
    b2 = b2 / np.linalg.norm(b2)
    b3 = np.cross(b1, b2)

    # Construct rotation matrix (columns are b1, b2, b3)
    rot_matrix = np.column_stack([b1, b2, b3])

    # Convert rotation matrix to quaternion using Shepperd's method
    trace = rot_matrix[0, 0] + rot_matrix[1, 1] + rot_matrix[2, 2]

    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (rot_matrix[2, 1] - rot_matrix[1, 2]) * s
        qy = (rot_matrix[0, 2] - rot_matrix[2, 0]) * s
        qz = (rot_matrix[1, 0] - rot_matrix[0, 1]) * s
    elif rot_matrix[0, 0] > rot_matrix[1, 1] and rot_matrix[0, 0] > rot_matrix[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rot_matrix[0, 0] - rot_matrix[1, 1] - rot_matrix[2, 2])
        qw = (rot_matrix[2, 1] - rot_matrix[1, 2]) / s
        qx = 0.25 * s
        qy = (rot_matrix[0, 1] + rot_matrix[1, 0]) / s
        qz = (rot_matrix[0, 2] + rot_matrix[2, 0]) / s
    elif rot_matrix[1, 1] > rot_matrix[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rot_matrix[1, 1] - rot_matrix[0, 0] - rot_matrix[2, 2])
        qw = (rot_matrix[0, 2] - rot_matrix[2, 0]) / s
        qx = (rot_matrix[0, 1] + rot_matrix[1, 0]) / s
        qy = 0.25 * s
        qz = (rot_matrix[1, 2] + rot_matrix[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + rot_matrix[2, 2] - rot_matrix[0, 0] - rot_matrix[1, 1])
        qw = (rot_matrix[1, 0] - rot_matrix[0, 1]) / s
        qx = (rot_matrix[0, 2] + rot_matrix[2, 0]) / s
        qy = (rot_matrix[1, 2] + rot_matrix[2, 1]) / s
        qz = 0.25 * s

    q = np.array([qw, qx, qy, qz], dtype=np.float32)

    # Normalize
    q = q / np.linalg.norm(q)

    # Ensure qw >= 0 for consistent output (q and -q represent the same rotation)
    if ensure_positive_w and q[0] < 0:
        q = -q

    return q


def pose7d_to_pose9d(pose: np.ndarray, input_format: str = "wxyz") -> np.ndarray:
    """Convert 7D pose (position + quaternion) to 9D pose (position + 6D rotation).

    This conversion is useful for neural network training as 6D rotation
    representation is continuous and avoids discontinuities present in
    Euler angles and the double-cover issue of quaternions.

    Args:
        pose: 7D array containing position and quaternion.
              Format depends on input_format:
              - "wxyz": [x, y, z, qw, qx, qy, qz] (scalar-first, Flexiv convention)
              - "xyzw": [x, y, z, qx, qy, qz, qw] (scalar-last, ROS convention)
        input_format: Quaternion format in the input pose.

    Returns:
        np.ndarray of shape (9,): [x, y, z, r1, r2, r3, r4, r5, r6]
        where r1-r6 is the 6D rotation representation.
    """
    pose = np.asarray(pose, dtype=np.float32)
    if pose.shape != (7,):
        raise ValueError(f"Expected pose array of shape (7,), got {pose.shape}")

    x, y, z = pose[0], pose[1], pose[2]

    if input_format == "wxyz":
        qw, qx, qy, qz = pose[3], pose[4], pose[5], pose[6]
    elif input_format == "xyzw":
        qx, qy, qz, qw = pose[3], pose[4], pose[5], pose[6]
    else:
        raise ValueError(f"Unknown input_format: {input_format}. Expected 'wxyz' or 'xyzw'.")

    r6d = quaternion_to_rotation_6d(qw, qx, qy, qz)

    return np.concatenate([[x, y, z], r6d]).astype(np.float32)


def pose9d_to_pose7d(pose: np.ndarray, output_format: str = "wxyz", ensure_positive_w: bool = True) -> np.ndarray:
    """Convert 9D pose (position + 6D rotation) to 7D pose (position + quaternion).

    This is the inverse of pose7d_to_pose9d(), used to convert neural network
    outputs back to quaternion format for robot control.

    Args:
        pose: 9D array [x, y, z, r1, r2, r3, r4, r5, r6]
              where r1-r6 is the 6D rotation representation.
        output_format: Quaternion format for output:
              - "wxyz": [x, y, z, qw, qx, qy, qz] (scalar-first, Flexiv convention)
              - "xyzw": [x, y, z, qx, qy, qz, qw] (scalar-last, ROS convention)
        ensure_positive_w: If True, ensure qw >= 0 for consistent output.

    Returns:
        np.ndarray of shape (7,) containing position and quaternion.
    """
    pose = np.asarray(pose, dtype=np.float32)
    if pose.shape != (9,):
        raise ValueError(f"Expected pose array of shape (9,), got {pose.shape}")

    x, y, z = pose[0], pose[1], pose[2]
    r6d = pose[3:9]

    quat = rotation_6d_to_quaternion(r6d, ensure_positive_w=ensure_positive_w)
    qw, qx, qy, qz = quat[0], quat[1], quat[2], quat[3]

    if output_format == "wxyz":
        return np.array([x, y, z, qw, qx, qy, qz], dtype=np.float32)
    elif output_format == "xyzw":
        return np.array([x, y, z, qx, qy, qz, qw], dtype=np.float32)
    else:
        raise ValueError(f"Unknown output_format: {output_format}. Expected 'wxyz' or 'xyzw'.")
