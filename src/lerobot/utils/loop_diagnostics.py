#!/usr/bin/env python

# Copyright 2026 The XenseRobotics Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Make a recording session's log self-sufficient for diagnosing overruns.

The record loop's ``[slow_frame]`` warning used to say *that* a frame overran
and nothing about *why*, and the log around it said nothing about the machine
it ran on. A rig in the field that reports "overrun 一直报错" then needs
someone on the box to find out whether it was the observation, the encoder
queue, a late ``sleep`` wake-up, the GIL, or simply four cores doing eight
streams of AV1. This module is what lets that be read off the session file
instead:

- :func:`host_summary` — one ``[session]`` line's worth of facts about the
  host: CPU, cores (total and *usable*, which differ under ``taskset`` or a
  container), memory, GPU, kernel, governor (a laptop on ``powersave`` wakes
  late), load, git commit, GIL switch interval.
- :class:`EpisodeLoopStats` — the per-take tally the loop feeds each
  iteration: body time, per-phase time, sleep overshoot, overruns. It also
  owns the overrun logging *policy*: the first few per take in full, the rest
  to the DEBUG file only, plus a periodic summary so a loop that overruns on
  every frame produces a readable log rather than 30 lines a second.

Nothing here logs. It formats and decides; the record script holds the logger.
"""

from __future__ import annotations

import gc
import os
import platform
import resource
import shutil
import subprocess
import sys
import time
from collections import defaultdict
from pathlib import Path
from typing import Any

import numpy as np

# ------------------------------------------------------------------ host facts


def _read_text(path: str) -> str:
    try:
        return Path(path).read_text()
    except OSError:
        return ""


def _proc_field(path: str, key: str) -> str:
    for line in _read_text(path).splitlines():
        if line.startswith(key):
            return line.split(":", 1)[1].strip()
    return ""


def _cpu_model() -> str:
    return _proc_field("/proc/cpuinfo", "model name") or platform.processor() or "unknown"


def _mem_total_gb() -> float | None:
    kb = _proc_field("/proc/meminfo", "MemTotal").split()
    return round(int(kb[0]) / 1e6, 1) if kb else None


def _governor() -> str:
    return _read_text("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor").strip() or "n/a"


def _gpu_name() -> str:
    try:
        import torch

        return torch.cuda.get_device_name(0) if torch.cuda.is_available() else "none"
    except Exception:  # noqa: BLE001 - purely informational
        return "unknown"


def _git_commit() -> str:
    """Short SHA of the checkout ``lerobot`` was imported from, or ``unknown``.

    A log from another machine is only as useful as knowing which code wrote
    it; ``lerobot.__version__`` tracks upstream and says nothing about this
    fork's commits.
    """
    git = shutil.which("git")
    if git is None:
        return "unknown"
    try:
        import lerobot

        out = subprocess.run(
            [git, "-C", str(Path(lerobot.__file__).resolve().parent), "rev-parse", "--short", "HEAD"],
            capture_output=True,
            text=True,
            timeout=2,
            check=False,
        )
        return out.stdout.strip() or "unknown"
    except Exception:  # noqa: BLE001 - git absent, not a checkout, timeout
        return "unknown"


def process_rss_mb() -> int | None:
    kb = _proc_field("/proc/self/status", "VmRSS").split()
    return int(kb[0]) // 1024 if kb else None


def process_threads() -> int | None:
    n = _proc_field("/proc/self/status", "Threads")
    return int(n) if n.isdigit() else None


def host_summary() -> dict[str, Any]:
    """Facts about this host that bear on whether a 30 Hz loop can hold."""
    try:
        usable = len(os.sched_getaffinity(0))
    except AttributeError:  # not Linux
        usable = os.cpu_count()
    try:
        load = round(os.getloadavg()[0], 1)
    except OSError:
        load = None
    return {
        "host": platform.node(),
        "cpu": _cpu_model(),
        "cores": os.cpu_count(),
        "usable": usable,
        "mem_gb": _mem_total_gb(),
        "gpu": _gpu_name(),
        "kernel": platform.release(),
        "python": platform.python_version(),
        "governor": _governor(),
        "load": load,
        "git": _git_commit(),
        "switchinterval_ms": round(sys.getswitchinterval() * 1e3, 1),
        "gc_threshold": gc.get_threshold(),
    }


def format_kv(items: dict[str, Any]) -> str:
    """``k=v`` pairs, quoting values with spaces so the line stays greppable."""
    parts = []
    for k, v in items.items():
        s = str(v)
        parts.append(f'{k}="{s}"' if " " in s else f"{k}={s}")
    return " ".join(parts)


# ------------------------------------------------------------ per-take tally


def _pct(values: list[float], q: float) -> float:
    return float(np.percentile(values, q)) if values else 0.0


def _p50_p99_max(values: list[float]) -> str:
    if not values:
        return "n/a"
    return f"p50={_pct(values, 50):.1f} p99={_pct(values, 99):.1f} max={max(values):.1f}"


class EpisodeLoopStats:
    """What one take's record-loop iterations cost, and how to report it.

    Fed by the loop each iteration (:meth:`record_iteration`), each sleep
    (:meth:`record_sleep`) and each overrun (:meth:`note_overrun`); read back
    as one ``[loop_summary]`` line (:meth:`summary_line`) once the take's
    disposition is known.

    The summary carries the things a per-frame warning cannot:

    - **effective fps** against nominal. The loop can run slow *without ever
      overrunning* — the body stays inside the budget and ``time.sleep`` wakes
      late on a busy box — and since ``add_frame`` stamps ``frame_index / fps``
      the dataset silently claims the nominal rate. Measured: 28.7–29.5 fps on
      a 4-core pin, nothing logged. This is the only place it shows.
    - **sleep overshoot** on its own, so that case reads as what it is.
    - **per-phase p99/max**: obs vs build vs add_frame vs display. ``add``
      ballooning with ``obs`` flat is the encoder queue back-pressuring
      (``feed_frame`` blocks up to 100 ms when a queue is full); ``obs``
      cameras at milliseconds is the GIL, not a camera (``async_read`` is a
      lock and a lookup).
    - **process CPU / RSS / peak threads / load**, so a log from a machine
      nobody can ssh into still says how loaded it was.
    - **GC collections** per generation, so a periodic stall can be matched
      or ruled out.

    Overrun logging policy: the first :attr:`FULL_DETAIL_OVERRUNS` of a take
    are worth a WARN line each; after that the per-frame line goes to DEBUG
    (the session file keeps it, the console does not) and a WARN summary is
    emitted at most every :attr:`SUMMARY_EVERY_S` seconds naming the window's
    worst. A loop overrunning on every frame therefore leaves a log that can
    still be read.
    """

    FULL_DETAIL_OVERRUNS = 5
    SUMMARY_EVERY_S = 5.0

    def __init__(self, fps: float) -> None:
        self.fps = fps
        self.budget_ms = 1e3 / fps if fps > 0 else 0.0
        self.body_ms: list[float] = []
        self.sleep_late_ms: list[float] = []
        self.phase_ms: dict[str, list[float]] = defaultdict(list)
        self.frames = 0
        self.overruns = 0
        self.worst_overrun_ms = 0.0
        self.worst_overrun_t = 0.0
        self._window_started: float | None = None
        self._window_overruns = 0
        self._window_worst_ms = 0.0
        self._window_worst_detail = ""
        self._t0 = time.perf_counter()
        self._ru0 = resource.getrusage(resource.RUSAGE_SELF)
        self._gc0 = [s["collections"] for s in gc.get_stats()]
        self._threads_peak = process_threads() or 0
        self._finished: dict[str, Any] | None = None

    # ---- feeding --------------------------------------------------------

    def record_iteration(self, body_ms: float, phases: dict[str, float], recorded: bool) -> None:
        self.body_ms.append(body_ms)
        for name, ms in phases.items():
            self.phase_ms[name].append(ms)
        if recorded:
            self.frames += 1
        # /proc is cheap but not free; once a second is plenty for a peak.
        if len(self.body_ms) % 30 == 0:
            self._threads_peak = max(self._threads_peak, process_threads() or 0)

    def record_sleep(self, requested_ms: float, actual_ms: float) -> None:
        self.sleep_late_ms.append(actual_ms - requested_ms)

    def note_overrun(
        self, t_s: float, overrun_ms: float, detail: str, now: float | None = None
    ) -> tuple[str, str | None]:
        """Register an overrun; say how loudly to log it.

        Returns ``(level, window_summary)``: ``level`` is ``"warn"`` for the
        first few of the take and ``"debug"`` afterwards; ``window_summary``
        is a line to WARN when a summary window has elapsed, else ``None``.
        """
        now = time.perf_counter() if now is None else now
        self.overruns += 1
        if overrun_ms > self.worst_overrun_ms:
            self.worst_overrun_ms, self.worst_overrun_t = overrun_ms, t_s

        level = "warn" if self.overruns <= self.FULL_DETAIL_OVERRUNS else "debug"

        if self._window_started is None:
            self._window_started = now
        self._window_overruns += 1
        if overrun_ms > self._window_worst_ms:
            self._window_worst_ms, self._window_worst_detail = overrun_ms, detail

        summary = None
        if now - self._window_started >= self.SUMMARY_EVERY_S:
            summary = (
                f"[slow_frame_summary] {self._window_overruns} overrun(s) in the last "
                f"{now - self._window_started:.1f}s, worst {self._window_worst_ms:.1f}ms: {self._window_worst_detail}"
            )
            self._window_started = now
            self._window_overruns = 0
            self._window_worst_ms = 0.0
            self._window_worst_detail = ""
        return level, summary

    # ---- reporting ------------------------------------------------------

    def finish(self) -> None:
        """Freeze wall time, CPU, GC and load as of now — the end of the take.

        The summary is printed after the reset phase (only then is the take's
        disposition known), so without this the wall clock would include the
        reset and the effective fps would read low for no reason.
        """
        if self._finished is not None:
            return
        wall_s = time.perf_counter() - self._t0
        ru = resource.getrusage(resource.RUSAGE_SELF)
        cpu_s = (ru.ru_utime - self._ru0.ru_utime) + (ru.ru_stime - self._ru0.ru_stime)
        gc_now = [s["collections"] for s in gc.get_stats()]
        try:
            load = f"{os.getloadavg()[0]:.1f}"
        except OSError:
            load = "n/a"
        self._threads_peak = max(self._threads_peak, process_threads() or 0)
        self._finished = {
            "wall_s": wall_s,
            "cpu_pct": 100.0 * cpu_s / wall_s if wall_s > 0 else 0.0,
            "gc_delta": "/".join(str(b - a) for a, b in zip(self._gc0, gc_now, strict=False)),
            "load": load,
            "rss_mb": process_rss_mb(),
        }

    @property
    def wall_s(self) -> float:
        self.finish()
        assert self._finished is not None
        return float(self._finished["wall_s"])

    def summary_line(self) -> str:
        self.finish()
        assert self._finished is not None
        wall_s = self._finished["wall_s"]
        cpu_pct = self._finished["cpu_pct"]
        gc_delta = self._finished["gc_delta"]
        load = self._finished["load"]
        eff_fps = self.frames / wall_s if wall_s > 0 else 0.0

        phases = " ".join(f"{name} {_pct(ms, 99):.1f}/{max(ms):.1f}" for name, ms in self.phase_ms.items() if ms)
        worst = (
            f"overruns {self.overruns}, worst {self.worst_overrun_ms:.1f}ms at t={self.worst_overrun_t:.2f}s"
            if self.overruns
            else "overruns 0"
        )
        return (
            f"{self.frames} frames in {wall_s:.2f}s = {eff_fps:.1f} fps "
            f"(nominal {self.fps:g}; dataset timestamps assume nominal) | "
            f"body ms {_p50_p99_max(self.body_ms)} (budget {self.budget_ms:.1f}) | {worst} | "
            f"sleep-late ms {_p50_p99_max(self.sleep_late_ms)} | "
            f"phases p99/max ms: {phases or 'n/a'} | "
            f"cpu {cpu_pct:.0f}% rss {self._finished['rss_mb'] or '?'}MB threads-peak {self._threads_peak} load {load} | "
            f"gc gen0/1/2 {gc_delta}"
        )
