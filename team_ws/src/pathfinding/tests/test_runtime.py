"""Review 2 findings 4 (stamp check), 6 (threads) and 8 (planner process, depth, journal dir). No ROS."""
import json
import math
from pathlib import Path
import threading
import time

import pytest

from dronehacks_pathplan.runtime import (Journal, PlanningLoop, make_run_dir, match_pose,
                                         offboard_stamp_verdict)

PKG = Path(__file__).resolve().parents[1] / "dronehacks_pathplan"


# ---------------------------------------------------------------- 6. executor / planning thread / journal
def test_node_uses_one_single_threaded_executor_and_plans_off_it():
    src = (PKG / "node.py").read_text(encoding="utf-8")
    assert "SingleThreadedExecutor()" in src and "MultiThreadedExecutor(" not in src
    assert "self.on_plan" in src and "create_timer(.2, self.on_plan" not in src    # not an executor timer
    assert "PlanningLoop(self.on_plan" in src and "node.planning.start()" in src
    assert "Journal(self.journal_path)" in src and "_jlock" not in src


def test_planning_loop_runs_on_its_own_thread_and_a_slow_step_never_blocks_the_caller():
    seen, started = [], threading.Event()

    def slow_step():                                # waits on the planner process like a real step
        seen.append(threading.current_thread().name)
        started.set()
        time.sleep(.3)

    loop = PlanningLoop(slow_step, .05)
    t0 = time.monotonic()
    loop.start()
    assert time.monotonic() - t0 < .05              # start() returns at once
    assert started.wait(2.0) and seen[0] == "planning"
    loop.stop()
    n = len(seen)
    time.sleep(.2)
    assert len(seen) == n                           # stopped: no more steps


def test_planning_loop_survives_an_exception():
    calls, errors = [], []

    def step():
        calls.append(1)
        if len(calls) == 1:
            raise ValueError("boom")

    loop = PlanningLoop(step, .01, on_error=errors.append).start()
    time.sleep(.2)
    loop.stop()
    assert len(calls) > 2 and "boom" in errors[0]


def test_journal_log_only_queues_and_the_writer_thread_writes(tmp_path):
    j = Journal(tmp_path / "journal.jsonl")
    real = j._file

    class SlowFile:                                 # a disk that takes 0.2 s per write
        def write(self, s):
            time.sleep(.2)
            real.write(s)

        def flush(self):
            real.flush()

        def close(self):
            real.close()

    j._file = SlowFile()
    t0 = time.monotonic()
    for i in range(5):
        j.log({"kind": "x", "i": i})
    assert time.monotonic() - t0 < .1               # the caller never waited on the disk
    j.close(timeout=5.0)
    lines = [json.loads(s) for s in (tmp_path / "journal.jsonl").read_text().splitlines()]
    assert [r["i"] for r in lines] == list(range(5))
    j.log({"kind": "after close"})                  # callbacks firing during shutdown: ignored


# ---------------------------------------------------------------- 8. journal dir fallback
def test_run_dir_falls_back_when_the_configured_dir_is_unwritable(tmp_path):
    blocker = tmp_path / "a_file"
    blocker.write_text("")
    first, second = tmp_path / "home" / "flyx-runs", tmp_path / "tmp" / "flyx-runs"
    out = make_run_dir(blocker / "runs", "pathplan-1", fallbacks=(first, second))   # under a file: fails
    assert out == first / "pathplan-1" and out.is_dir()
    (tmp_path / "home2").write_text("")                                             # first fallback fails too
    out = make_run_dir(blocker / "runs", "pathplan-2", fallbacks=(tmp_path / "home2" / "x", second))
    assert out == second / "pathplan-2"
    assert make_run_dir(tmp_path / "ok", "p", fallbacks=(first,)) == tmp_path / "ok" / "p"


# ---------------------------------------------------------------- 8. depth frames after a stall
def _poses(t0, t1, dt=1 / 30):
    q = (0.0, 0.0, 0.0, 1.0)
    return [(t0 + i * dt, (i * .01, 0.0, .9), q) for i in range(int((t1 - t0) / dt))]


def test_depth_frame_matched_to_its_odometry():
    stats = {}
    pos, q, rate = match_pose(_poses(10.0, 12.0), 11.0, stats)
    assert stats == {"matched": 1} and abs(pos[0] - .3) < .02 and rate == 0.0


def test_unmatched_frame_after_a_stall_is_skipped_once_stamps_have_matched():
    poses = _poses(10.0, 11.0) + _poses(13.0, 14.0)            # a 2 s hole in the pose history
    stats = {}
    assert match_pose(poses, 12.0, stats) is not None and stats["fallback"] == 1   # stamps never matched yet
    stats = {"matched": 20}
    assert match_pose(poses, 12.0, stats) is None and stats["unmatched"] == 1    # skipped, not misplaced
    assert match_pose(poses, 13.5, stats) is not None


# ---------------------------------------------------------------- 8. planner process death
def test_planner_process_that_dies_before_ready_raises_instead_of_hanging():
    from dronehacks_pathplan.planner_core import PlannerProcess
    t0 = time.monotonic()
    with pytest.raises(RuntimeError, match="did not start"):
        PlannerProcess(start_timeout_s=60.0, no_such_argument=1)        # the child dies at once
    assert time.monotonic() - t0 < 30.0


def test_planner_process_reset_after_death_does_not_raise():
    from dronehacks_pathplan.planner_core import PlannerProcess
    p = PlannerProcess(planner_name="theta")
    try:
        assert p.update([-2.515, .914], math.pi / 2).mode == "move"
        p.proc.kill()
        p.proc.join(5.0)
        assert p.update([-2.515, 1.0], math.pi / 2).mode == "failed"
        p.reset()                                                       # used to raise BrokenPipeError
        assert p.update([-2.515, 1.0], math.pi / 2).mode == "failed"
        assert p.events[-1]["event"] == "planner process stopped"
    finally:
        p.close()


# ---------------------------------------------------------------- 4. Offboard stamps vs PX4's clock
def _stream(rate_hz, secs, stamp_of):
    """[(arrival s, stamp us)] for a stream arriving at rate_hz for secs (arrival = wall time)."""
    return [(i / rate_hz, int(stamp_of(i / rate_hz))) for i in range(int(secs * rate_hz))]


PX4_BOOT_US = 50e6                                   # PX4 clock 50 s at the first sample


def _odom(px4_rate=1.0):
    return _stream(50, 5, lambda a: PX4_BOOT_US + a * px4_rate * 1e6)


def test_stamps_on_px4_time_base_pass():
    lvl, msg = offboard_stamp_verdict(_stream(10, 5, lambda a: PX4_BOOT_US + a * 1e6 - 3e3), _odom())
    assert lvl == "OK", msg


def test_sim_time_stamps_drifting_against_px4_fail():
    # sim at 0.8x: our stamps advance 0.8 s per second against a PX4 clock running at wall rate
    lvl, msg = offboard_stamp_verdict(_stream(10, 5, lambda a: PX4_BOOT_US + .8 * a * 1e6), _odom())
    assert lvl == "FAIL" and "drift" in msg


def test_stamps_behind_px4_fail_and_future_stamps_fail():
    lvl, msg = offboard_stamp_verdict(_stream(10, 5, lambda a: PX4_BOOT_US + a * 1e6 - 1.5e6), _odom())
    assert lvl == "FAIL" and "stale" in msg
    lvl, msg = offboard_stamp_verdict(_stream(10, 5, lambda a: PX4_BOOT_US + a * 1e6 + 2e6), _odom())
    assert lvl == "FAIL" and "future" in msg


def test_wall_clock_stamps_against_px4_without_timesync_are_ok():
    # UXRCE_DDS_SYNCT=0 (lockstep sim): PX4 time since boot vs Unix time; PX4 clamps to its own now
    unix_us = 1.78e15
    lvl, msg = offboard_stamp_verdict(_stream(10, 5, lambda a: unix_us + a * 1e6), _odom(.8))
    assert lvl == "OK" and "clamps" in msg


def test_sim_time_stamps_against_synced_px4_fail():
    unix_us = 1.78e15                                # timesync on: PX4 publishes in agent (Unix) time
    lvl, msg = offboard_stamp_verdict(_stream(10, 5, lambda a: 300e6 + a * 1e6),
                                      _stream(50, 5, lambda a: unix_us + a * 1e6))
    assert lvl == "FAIL" and "wrap" in msg


def test_too_few_samples_warn():
    assert offboard_stamp_verdict([], _odom())[0] == "WARN"
