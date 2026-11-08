"""The planner + depth mapping, separate from ROS (ported from flyx/fly_pathplan.py).

``PlannerCore`` is everything the FlyX runs taught us about turning depth images into a map
the planner can trust, plus the planner itself:

* depth returns within ``WALL_SNAP_M`` of a published wall count as that wall (pose error and
  stereo noise invented phantom walls next to real ones: FlyX run 6 lost the passage past M1);
* a *new* obstacle cell must be seen in 2 of ~3 frames (+2 per hit, -1 per frame, confirm at 3);
* returns that a slightly wrong pose invents are dropped (``drop_misregistered``);
* the carrot is pulled in until the straight line to it stays out of the keep-out (PX4 flies
  straight at a pose setpoint, so a far carrot round a bend cuts the corner into the wall).

Two ways to run it, same interface (``update``, ``events``, ``cleared``, ``pad_xy``, ``reset``):

``PlannerProcess``  its own OS process, lower CPU priority. The default on the drone and in the
                    sim: planning + depth projection are pure-Python CPU work and, sharing the ROS
                    process's GIL, stalled the setpoint stream for 0.4-1.0 s (FlyX runs 4 and 6:
                    "odometry stale", then PX4 offboard_control_signal_lost).
``InlinePlanner``   in the calling thread. Tests and the closed-loop check use this.
"""
from __future__ import annotations

import os
import signal
import threading
import time

import numpy as np

WALL_SNAP_M = .25       # depth returns this close to a published wall are that wall
HIT_CONFIRM = 3         # votes for a new obstacle cell: +2 per frame that hits it, -1 per frame
# Keep moving while the nose turns toward the route up to this far off; stop and turn in place only
# beyond it. With the heading capped at 0.25 rad/s, the planner's default 30 deg made the drone stop
# and turn in place 21-26 times per flight, and a stop followed by an in-place turn is what loses
# visual odometry. From jolie_course/upload_to_flyx.py, the version that completed the whole FlyX course.
TURN_IN_PLACE_DEG = 60.0


def safe_carrot_path(cost, spec, xy, path, carrot_m):
    """[xy, setpoint]: the point carrot_m along the plan, pulled in until the straight line to
    it stays out of the keep-out (fake-PX4 runs: a far carrot round a bend "escaped" 18 cm
    from the net)."""
    from pathplan.grid_search import line_cost
    from pathplan.online import _point_along

    def cell(p):
        return np.clip((np.asarray(p)[::-1] + spec.half_size) / spec.resolution - .5, 0, cost.shape[0] - 1)

    for d in (carrot_m, .35, .25, .15):
        target = _point_along(path, d)
        if np.isfinite(line_cost(cost, cell(xy), [cell(target)])[0]):
            return np.array([xy, target])
    return np.array([xy, _point_along(path, .1)])


class PlannerCore:
    """OnlinePlanner on the published course + depth voting. One ``step`` per planning tick."""

    def __init__(self, planner_name="theta", altitude=.9, camera_offset=(.115, 0.0, .029),
                 pitch_down_deg=0.0, carrot_m=.5):
        from pathplan.depth_projection import CameraMount
        self.args = (planner_name, altitude, tuple(camera_offset), pitch_down_deg, carrot_m)
        self.altitude, self.carrot_m = altitude, carrot_m
        self.mount = CameraMount(offset=tuple(camera_offset), pitch_down_deg=pitch_down_deg)
        self.reset()

    def reset(self):
        """A fresh planner (a new flight starts from checkpoint 1 with only the published map)."""
        from pathplan.ros_node import build_planner
        from pathplan.world import clearance
        self.planner, self.spec, self.env, self.prior = build_planner(self.args[0], self.altitude)
        self.planner.cfg.turn_in_place_deg = TURN_IN_PLACE_DEG
        self.near_wall = (clearance(self.prior, self.spec) < WALL_SNAP_M) & ~self.prior
        self.votes = np.zeros(self.prior.shape, np.int8)
        self.quality = {"frames": 0, "hits": 0, "hits_on_published_walls": 0, "hits_new_confirmed": 0,
                        "hits_new_ignored": 0}

    @property
    def pad_xy(self):
        return [float(v) for v in self.planner.checkpoints[-1].centre]

    def observe_frame(self, frame):
        """frame = (data, width, height, encoding, step, K, course xyz, body->course rotation)."""
        from pathplan.depth_projection import Intrinsics, decode_depth_image, drop_misregistered, project_depth
        data, width, height, encoding, step, k, xyz, rotation = frame
        depth = decode_depth_image(data, width, height, encoding, step)
        cells = project_depth(depth, Intrinsics.from_k(k, width, height), xyz, rotation, self.spec,
                              altitude=self.altitude, half_height=self.env.half_height + self.env.tracking_margin,
                              mount=self.mount, expected=self.prior)
        q = self.quality
        q["frames"] += 1
        raw_hit = cells[1]
        if len(raw_hit):                        # diagnostic: a wrong camera mount shows up here
            on_wall = (self.prior | self.near_wall)[raw_hit[:, 0], raw_hit[:, 1]]
            q["hits"] += len(raw_hit)
            q["hits_on_published_walls"] += int(on_wall.sum())
        free, hit, behind = drop_misregistered(*cells, self.prior, self.near_wall)
        self.votes = np.maximum(self.votes - 1, 0).astype(np.int8)
        if len(hit):
            r, c = hit[:, 0], hit[:, 1]
            self.votes[r, c] = np.minimum(self.votes[r, c] + 2, 6)
            known = self.planner.belief.occ[r, c]
            ok = known | (self.votes[r, c] >= HIT_CONFIRM)
            q["hits_new_confirmed"] += int((ok & ~known).sum())
            q["hits_new_ignored"] += int((~ok).sum())
            hit = hit[ok]
        self.planner.observe(free, hit, behind)

    def step(self, xy, yaw, frame=None):
        """(Command, events, cleared, ms, quality) for the vehicle at course xy / yaw."""
        t0 = time.perf_counter()
        if frame is not None:
            self.observe_frame(frame)
        cmd = self.planner.update(np.asarray(xy, float), float(yaw))
        if cmd.mode == "move" and cmd.path is not None and len(cmd.path) >= 2:
            cmd.path = safe_carrot_path(self.planner.belief.cost, self.spec, np.asarray(xy, float), cmd.path,
                                        self.carrot_m)
        return (cmd, list(self.planner.events), list(self.planner.cleared), 1e3 * (time.perf_counter() - t0),
                dict(self.quality))


class _FrameSlot:
    """The latest depth frame, handed from the depth callback to the next planning step."""

    def __init__(self):
        self.frame, self.lock = None, threading.Lock()

    def give_frame(self, frame):
        with self.lock:
            self.frame = frame

    def take(self):
        with self.lock:
            frame, self.frame = self.frame, None
        return frame


class InlinePlanner(_FrameSlot):
    """PlannerCore in the calling thread (tests, closed-loop check, ``planner_process:=false``)."""

    def __init__(self, **kwargs):
        super().__init__()
        self.core = PlannerCore(**kwargs)
        self.events, self.cleared, self.ms, self.depth_quality = [], [], 0.0, {}

    @property
    def pad_xy(self):
        return self.core.pad_xy

    @property
    def n_checkpoints(self):
        return len(self.core.planner.checkpoints)

    def update(self, xy, yaw):
        cmd, self.events, self.cleared, self.ms, self.depth_quality = self.core.step(xy, yaw, self.take())
        return cmd

    def reset(self):
        self.core.reset()
        self.events, self.cleared = [], []
        self.take()

    def close(self):
        pass


def _child(conn, kwargs):
    """The planner process: answers ('update', xy, yaw, frame) / ('reset',) / None (exit)."""
    signal.signal(signal.SIGINT, signal.SIG_IGN)        # the ROS process handles Ctrl-C
    try:
        os.nice(10)                                      # below the setpoint stream (FlyX run 6)
    except (OSError, AttributeError):                    # AttributeError: no os.nice (Windows tests)
        pass
    core = PlannerCore(**kwargs)
    conn.send(("ready", core.pad_xy, len(core.planner.checkpoints)))
    while True:
        msg = conn.recv()
        if msg is None:
            return
        if msg[0] == "reset":
            core.reset()
            conn.send(("ready", core.pad_xy, len(core.planner.checkpoints)))
            continue
        _, xy, yaw, frame = msg
        conn.send(core.step(xy, yaw, frame))


class PlannerProcess(_FrameSlot):
    """PlannerCore in its own process ('spawn': safe to start after rclpy threads exist)."""

    def __init__(self, start_timeout_s=120.0, **kwargs):
        import multiprocessing as mp
        super().__init__()
        self.start_timeout_s = float(start_timeout_s)
        ctx = mp.get_context("spawn")
        self.conn, child = ctx.Pipe()
        self.proc = ctx.Process(target=_child, args=(child, kwargs), daemon=True)
        self.proc.start()
        # Drop our copy of the child's end: with it open, a child that dies before "ready" left
        # recv() waiting forever; now its death reads as EOF here.
        child.close()
        try:
            self._ready(self._recv(self.start_timeout_s))
        except (EOFError, OSError, TimeoutError) as e:
            self.proc.kill()
            self.proc.join(2.0)
            raise RuntimeError(f"planner process did not start ({type(e).__name__}: {e}; "
                               f"exit code {self.proc.exitcode})") from e
        self.events, self.cleared, self.ms, self.depth_quality = [], [], 0.0, {}
        self.call_lock = threading.Lock()
        self.alive = True

    def _recv(self, timeout_s):
        if not self.conn.poll(timeout_s):
            raise TimeoutError(f"no answer from the planner process in {timeout_s:.0f} s")
        return self.conn.recv()

    def _ready(self, msg):
        if msg[0] != "ready":
            raise RuntimeError("planner process did not start")
        _, self._pad_xy, self.n_checkpoints = msg

    @property
    def pad_xy(self):
        return self._pad_xy

    def update(self, xy, yaw):
        """One planning step. Blocks the calling (planning) thread only; the pipe wait frees the GIL."""
        frame = self.take()
        try:
            if not self.alive:                      # died, or a reset timed out (the pipe is out of step)
                raise EOFError("planner process stopped")
            with self.call_lock:
                self.conn.send(("update", [float(v) for v in xy], float(yaw), frame))
                cmd, self.events, self.cleared, self.ms, self.depth_quality = self.conn.recv()
            return cmd
        except (EOFError, OSError, BrokenPipeError):
            from pathplan.online import Command
            self.alive = False
            self.events = self.events + [{"event": "planner process stopped"}]
            return Command("failed", reason="planner process stopped")

    def reset(self):
        """A fresh planner for the next flight. After the process died this no longer raises
        (it raised BrokenPipeError on the planning thread): the next update() reports 'failed'."""
        try:
            with self.call_lock:
                self.conn.send(("reset",))
                self._ready(self._recv(self.start_timeout_s))
            self.events, self.cleared = [], []
        except (EOFError, OSError, TimeoutError):          # BrokenPipeError is an OSError
            self.alive = False
            self.events, self.cleared = [{"event": "planner process stopped"}], []
        self.take()

    def close(self):
        try:
            self.conn.send(None)
        except OSError:
            pass
        self.proc.join(2.0)
        if self.proc.is_alive():
            self.proc.terminate()
