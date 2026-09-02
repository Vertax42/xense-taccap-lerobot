# Copyright 2026 The XenseRobotics Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""The record loop's diagnostics: what a session log must carry to be read
without the machine it came from."""

import time

from lerobot.utils.loop_diagnostics import EpisodeLoopStats, format_kv, host_summary


class TestHostSummary:
    def test_carries_the_facts_that_decide_whether_30hz_holds(self):
        facts = host_summary()
        for key in (
            "host",
            "cpu",
            "cores",
            "usable",
            "gpu",
            "kernel",
            "python",
            "governor",
            "git",
            "switchinterval_ms",
        ):
            assert key in facts, key
        assert facts["usable"] <= facts["cores"]

    def test_format_kv_quotes_values_with_spaces_so_the_line_stays_greppable(self):
        line = format_kv({"cpu": "Intel Core Ultra", "cores": 24, "gpu": "none"})
        assert line == 'cpu="Intel Core Ultra" cores=24 gpu=none'


class TestEpisodeLoopStats:
    """One take's tally, and the overrun logging policy that keeps a loop
    overrunning every frame from writing 30 identical lines a second."""

    def test_first_overruns_are_loud_the_rest_go_to_the_debug_file(self):
        stats = EpisodeLoopStats(fps=30)
        levels = [stats.note_overrun(t_s=i * 0.033, overrun_ms=5.0, detail="x", now=100.0)[0] for i in range(8)]
        assert levels == ["warn"] * EpisodeLoopStats.FULL_DETAIL_OVERRUNS + ["debug"] * 3
        assert stats.overruns == 8

    def test_a_window_summary_names_the_worst_overrun_and_resets(self):
        stats = EpisodeLoopStats(fps=30)
        assert stats.note_overrun(1.0, 3.0, "small", now=0.0)[1] is None
        assert stats.note_overrun(1.5, 12.0, "big one", now=1.0)[1] is None
        _, summary = stats.note_overrun(2.0, 4.0, "later", now=EpisodeLoopStats.SUMMARY_EVERY_S + 0.5)
        assert summary is not None
        assert "3 overrun(s)" in summary and "worst 12.0ms" in summary and "big one" in summary
        # The next window starts empty.
        _, again = stats.note_overrun(9.0, 1.0, "next", now=2 * EpisodeLoopStats.SUMMARY_EVERY_S + 1.5)
        assert again is not None and "1 overrun(s)" in again
        assert stats.worst_overrun_ms == 12.0 and stats.worst_overrun_t == 1.5

    def test_summary_line_reports_effective_fps_against_nominal(self):
        stats = EpisodeLoopStats(fps=30)
        for i in range(10):
            stats.record_iteration(body_ms=2.0, phases={"obs": 0.5, "add": 1.0}, recorded=i > 0)
            stats.record_sleep(requested_ms=31.0, actual_ms=31.4)
        stats.finish()
        line = stats.summary_line()

        assert line.startswith("9 frames in ")
        assert "(nominal 30; dataset timestamps assume nominal)" in line
        assert "body ms p50=2.0 p99=2.0 max=2.0 (budget 33.3)" in line
        assert "overruns 0" in line
        assert "sleep-late ms p50=0.4" in line
        assert "phases p99/max ms: obs 0.5/0.5 add 1.0/1.0" in line
        assert "cpu " in line and "threads-peak" in line and "gc gen0/1/2" in line

    def test_finish_freezes_the_clock_before_the_reset_phase(self):
        stats = EpisodeLoopStats(fps=30)
        stats.record_iteration(1.0, {}, recorded=True)
        stats.finish()
        wall = stats.wall_s
        time.sleep(0.05)
        assert stats.wall_s == wall, "the reset that follows a take must not stretch the take's wall clock"
        assert stats.summary_line().startswith("1 frames in ")

    def test_empty_take_still_summarises(self):
        line = EpisodeLoopStats(fps=30).summary_line()
        assert line.startswith("0 frames") and "body ms n/a" in line and "phases p99/max ms: n/a" in line
