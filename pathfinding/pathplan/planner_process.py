"""The planner and depth projection in their own process (shared by flyx/ and ros2/).

Both are pure-Python CPU work and, sharing the ROS process's GIL, stalled the PX4 / odometry
callbacks for 0.4 s at a time (FlyX run 4: "odometry stale (0.42s)" in TAKEOFF; reproduced on a
loaded CPU with simulated depth).  This process runs at lower CPU priority than the ROS process
(FlyX run 6: the control loop stalled 1.05 s when the planner got busy at the start of NAVIGATE).

``PlannerProcess`` forks, so create it before ``rclpy.init`` (no ROS threads yet).  It can be
configured at once (flyx: everything is known from the command line) or later with
``configure`` (ROS node: the parameters only exist once the node does).
"""
import os
import signal
import threading
import time

import numpy as np

WALL_SNAP_M = .25       # depth returns this close to a published wall are that wall (pose error,
                        # stereo noise): FlyX run 6 lost the tight passage past maze wall M1 to them
HIT_CONFIRM = 3         # vote threshold for a new obstacle cell: +2 per frame that hits it, -1 per
                        # frame, so it must be seen in 2 of ~3 frames (single noisy frames are ignored)
SOLID_BEHIND_M = .20    # behind a newly seen (unmapped) obstacle face, this deep counts as solid
MAX_TURN_RATE = .5      # rad/s: depth frames taken while turning faster are dropped (pose error)


# Keep moving while the nose turns toward the route up to this far off; stop and turn in place only
# beyond it. With the heading capped at 0.25 rad/s (VIO), the planner's 30 deg made the drone stop
# and turn in place 21-26 times per fake-PX4 flight (49-58 % of NAVIGATE), and a full stop followed
# by turning in place is what lost visual odometry in Suraj's Gazebo v131. His flyx_course keeps
# moving through turns (nose 0.7 m ahead, <= 0.25 rad/s); the mission slows down while turning.
TURN_IN_PLACE_DEG = 60.0


def safe_carrot_path(cost, spec, xy, path, carrot_m):
    """[xy, setpoint]: the point carrot_m along the plan, pulled in until the straight line to
    it stays out of the keep-out. PX4 flies straight at the setpoint, so a far carrot round a
    bend cuts the corner into the wall (fake-PX4 runs: "escape" 18 cm from the net)."""
    from .grid_search import line_cost
    from .online import _point_along
    cell = lambda p: np.clip((np.asarray(p)[::-1] + spec.half_size) / spec.resolution - .5, 0, cost.shape[0] - 1)
    # Check the straight line from the drone - or, if the drone itself is inside the keep-out (a near
    # miss), from the route's first point: from a blocked start every line reads as blocked, the carrot
    # shrank to 0.1 m and the drone crawled along the keep-out edge, grazing the wall (fake-PX4 clutter-01).
    rc = np.rint(cell(xy)).astype(int)
    start = xy if np.isfinite(cost[rc[0], rc[1]]) or len(path) < 2 else np.asarray(path[1], float)
    for d in (carrot_m, .35, .25, .15):
        target = _point_along(path, d)
        if np.isfinite(line_cost(cost, cell(start), [cell(target)])[0]):
            return np.array([xy, target])
    return np.array([xy, _point_along(path, .1)])


def planner_process(conn, build_planner):
    """Child process: wait for the config, build the planner, then answer (xy, yaw, depth frame)."""
    signal.signal(signal.SIGINT, signal.SIG_IGN)          # the parent handles Ctrl-C
    try:
        os.nice(10)
    except OSError:
        pass
    config = conn.recv()
    if config is None:
        return
    planner_name, altitude, camera_offset, pitch_down_deg, carrot_m = config
    from .depth_projection import CameraMount, Intrinsics, decode_depth_image, drop_misregistered, project_depth
    from .world import clearance
    planner, spec, env, prior = build_planner(planner_name, altitude)
    planner.cfg.turn_in_place_deg = TURN_IN_PLACE_DEG
    near_wall = (clearance(prior, spec) < WALL_SNAP_M) & ~prior
    mount = CameraMount(offset=tuple(camera_offset), pitch_down_deg=pitch_down_deg)
    votes = np.zeros(prior.shape, np.int8)
    quality = {"hits": 0, "hits_on_published_walls": 0, "hits_new_confirmed": 0, "hits_new_ignored": 0}
    conn.send(("ready", [float(v) for v in planner.checkpoints[-1].centre]))
    while True:
        msg = conn.recv()
        if msg is None:
            return
        xy, yaw, frame = msg
        t0 = time.perf_counter()
        if frame is not None:
            data, width, height, encoding, step, k, xyz, rotation = frame
            depth = decode_depth_image(data, width, height, encoding, step)
            cells = project_depth(depth, Intrinsics.from_k(k, width, height), xyz, rotation, spec,
                                  altitude=altitude, half_height=env.half_height + env.tracking_margin,
                                  mount=mount, expected=prior)
            raw_hit = cells[1]
            if len(raw_hit):                           # diagnostic: a wrong camera mount shows up here
                on_wall = (prior | near_wall)[raw_hit[:, 0], raw_hit[:, 1]]
                quality["hits"] += len(raw_hit)
                quality["hits_on_published_walls"] += int(on_wall.sum())
            free, hit, behind = drop_misregistered(*cells, prior, near_wall)
            votes = np.maximum(votes - 1, 0).astype(np.int8)
            if len(hit):
                votes[hit[:, 0], hit[:, 1]] = np.minimum(votes[hit[:, 0], hit[:, 1]] + 2, 6)
                known = planner.belief.occ[hit[:, 0], hit[:, 1]]
                ok = known | (votes[hit[:, 0], hit[:, 1]] >= HIT_CONFIRM)
                quality["hits_new_confirmed"] += int((ok & ~known).sum())
                quality["hits_new_ignored"] += int((~ok).sum())
                hit = hit[ok]
            if len(hit) and len(behind) and planner.map_resets == 0:
                # unexpected obstacles: the camera only sees their front face; the first SOLID_BEHIND_M
                # behind a confirmed new hit is solid (typical obstacle size), so the route keeps its
                # clearance from the unseen side. Fake-PX4 clutter courses: almost every contact was a
                # graze on the hidden side of a cylinder the camera had seen. Off once the planner has
                # had to forget its depth map: if this inference ever sealed a passage that is really
                # open, re-adding it would just get stuck again (back to suspect-only, the old rule).
                new = hit[~prior[hit[:, 0], hit[:, 1]]]
                if len(new):
                    d = np.linalg.norm(behind[:, None, :] - new[None, :, :], axis=-1).min(axis=1) * spec.resolution
                    hit = np.vstack([hit, behind[d <= SOLID_BEHIND_M]])
            planner.observe(free, hit, behind)
        cmd = planner.update(xy, yaw)
        if cmd.mode == "move" and cmd.path is not None and len(cmd.path) >= 2:
            cmd.path = safe_carrot_path(planner.belief.cost, spec, np.asarray(xy, float), cmd.path, carrot_m)
        conn.send((cmd, planner.events, planner.cleared, 1e3 * (time.perf_counter() - t0), quality))


class PlannerProcess:
    """Runs ``planner_process``; offers what the missions use (update, events, cleared, pad_xy)."""

    def __init__(self, build_planner, config=None):
        import multiprocessing as mp
        ctx = mp.get_context("fork")
        self.conn, child = ctx.Pipe()
        self.proc = ctx.Process(target=planner_process, daemon=True, args=(child, build_planner))
        self.proc.start()
        self.pad_xy = None
        self.events, self.cleared, self.ms, self.depth_quality = [], [], 0.0, {}
        self.frame, self.lock = None, threading.Lock()      # latest depth frame, from the depth callback
        if config is not None:
            self.configure(*config)

    def configure(self, planner_name, altitude, camera_offset, pitch_down_deg, carrot_m):
        """Build the planner in the child (a few seconds); sets ``pad_xy``."""
        self.conn.send((planner_name, altitude, tuple(camera_offset), pitch_down_deg, carrot_m))
        ready, self.pad_xy = self.conn.recv()
        if ready != "ready":
            raise RuntimeError("planner process did not start")

    def give_frame(self, frame):
        with self.lock:
            self.frame = frame

    def update(self, xy, yaw):
        """One planning step. Blocks only the calling thread; waiting on the pipe frees the GIL."""
        with self.lock:
            frame, self.frame = self.frame, None
        try:
            self.conn.send((xy, yaw, frame))
            cmd, self.events, self.cleared, self.ms, self.depth_quality = self.conn.recv()
            return cmd
        except (EOFError, OSError):
            from .online import Command
            self.events = self.events + [{"event": "planner process stopped"}]
            return Command("failed", reason="planner process stopped")

    def close(self):
        try:
            self.conn.send(None)
        except OSError:
            pass
        self.proc.join(2.0)
        if self.proc.is_alive():
            self.proc.terminate()
