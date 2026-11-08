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
WALL_SNAP_DRIFT_M = .45 # ... and this close while recent depth says the pose has drifted (below)
DRIFT_ON_WALL = .85     # recent share of depth hits on published walls below which the pose has drifted
SOLID_BEHIND_M = .20    # behind a newly seen (unmapped) obstacle face, this deep counts as solid
MAX_TURN_RATE = .5      # rad/s: depth frames taken while turning faster are dropped (pose error)


# Keep moving while the nose turns toward the route up to this far off; stop and turn in place only
# beyond it. With the heading capped at 0.25 rad/s (VIO), the planner's 30 deg made the drone stop
# and turn in place 21-26 times per fake-PX4 flight (49-58 % of NAVIGATE), and a full stop followed
# by turning in place is what lost visual odometry in Suraj's Gazebo v131. His flyx_course keeps
# moving through turns (nose 0.7 m ahead, <= 0.25 rad/s); the mission slows down while turning.
TURN_IN_PLACE_DEG = 90.0
# Was 60 with a 0.7 m look-ahead: in the real sim the drone still spent a lot of each sharp (~90 deg)
# corner stopped, turning in place at 0.25 rad/s (6 s per 90 deg). The nose now starts turning 1.0 m
# before a corner and the drone keeps moving - slowly, the carrot shrinks with the heading error -
# through corners up to 90 deg. PX4 flies the corner sideways while the nose catches up.
LOOKAHEAD_M = 1.0

# ------------------------------------------------------------------------------------------------
# [v5] switches on top of Jolie's v4c (origin/jolie-flyx-v2 1d44162, FlyX 5/5). Each is one marked
# patch; all False = v4c exactly. The PlannerProcess config may carry a 6th element, a dict that
# overrides these (the offline A/B passes it); the flight file uses the defaults below.
V5_OPTIONS = {
    "plan_cache": True,      # do not repeat a search that already failed on the same inputs (CPU)
    "look_guard": True,      # look around at most LOOK_GUARD_RAD, toward the side that sees farther
    "corner_view": False,    # first-corner-vio "view" rule: lead the heading toward a >= 1.5 m view
    "forget_votes": False,   # "forget depth map" also forgets the hit votes (phantoms must re-confirm)
    "forget_snap": True,     # after a "forget depth map", use the drift snap (0.45 m) until the next checkpoint
    "solid_relax": False,    # no inferred-solid cells behind a hit within SOLID_MIN_WALL_M of a published wall
    "solid_off": False,      # no inferred-solid cells at all (proven's rule: suspect cells only)
    "soft_margin": None,     # planning soft margin override, m (v4c 0.30; proven 0.15); None = keep
}
SOLID_MIN_WALL_M = .45       # [v5 solid_relax] (turns-explore 0244404: a misregistered wall return just beyond
                             # the 0.25 m snap must not seal a narrow published opening such as the tube mouth)
LOOK_GUARD_RAD = np.pi       # [v5 look_guard] v4c: a full turn


def view_distance(occ, spec, xy, heading, cap=2.0, step=.05):
    """How far the forward camera sees along ``heading`` on the occupancy map (m, capped)."""
    d = np.arange(step, cap + 1e-9, step)
    pts = np.asarray(xy, float) + d[:, None] * np.array([np.cos(heading), np.sin(heading)])
    rc = spec.to_cell(pts)                                     # clipped: the border (net) is occupied
    hit = occ[rc[:, 0], rc[:, 1]]
    return float(d[int(np.argmax(hit))]) if hit.any() else cap


def long_view_sign(occ, spec, xy, yaw):
    """[v5 look_guard, from turns-explore 3105f4e] +1 (left) / -1 (right): the side whose headings
    30-120 deg off the nose see farther on the belief map; None if within 0.1 m (then v4c's rule:
    toward the published route)."""
    if xy is None or yaw is None:
        return None
    side = lambda s: float(np.mean([view_distance(occ, spec, xy, yaw + s * np.deg2rad(a)) for a in (30, 60, 90, 120)]))
    left, right = side(1.0), side(-1.0)
    if abs(left - right) < .1:
        return None
    return 1.0 if left > right else -1.0


# [v5 corner_view] Corner view rule, ported from origin/first-corner-vio jolie_course/cornerfix
# (variant "view": flew FlyX on the proven file; Gazebo 8/8; SLAM features at the first corner 39.9
# vs 13.2) via turns-explore 33e7c37. Gazebo VIO diverged at the first corner when the nose swept the
# net 0.7-0.8 m away and features starved. Uses only the planner's belief map, never sim truth.
CORNER_TURN_DEG = 45.0     # a route turn sharper than this ...
CORNER_WITHIN_M = 1.2      # ... this far ahead = corner mode
CORNER_TURNING_DEG = 20.0  # nose still this far off the route direction = corner mode
CORNER_SEG_M = .3          # the route's direction is measured over this length
VIEW_ENOUGH_M = 1.5        # forward view on the known map that counts as far enough
VIEW_MAX_DEV_DEG = 40      # never more than this beyond the planner's heading
VIEW_STEP_DEG = 10


def route_turn_ahead(path, within=CORNER_WITHIN_M, seg=CORNER_SEG_M):
    """Signed largest change of the route's direction within ``within`` m ahead (rad, + = left)."""
    from .online import _point_along, _wrap
    path = np.asarray(path, float)
    p0, p1 = _point_along(path, 0.0), _point_along(path, seg)
    if np.linalg.norm(p1 - p0) < 1e-6:
        return 0.0
    h0 = float(np.arctan2(p1[1] - p0[1], p1[0] - p0[0]))
    best = 0.0
    for s in np.arange(.1, within + 1e-9, .1):
        a, b = _point_along(path, s), _point_along(path, s + seg)
        if np.linalg.norm(b - a) < .05:                        # the route ends here
            break
        d = _wrap(float(np.arctan2(b[1] - a[1], b[0] - a[0])) - h0)
        if abs(d) > abs(best):
            best = d
    return best


def view_heading(occ, spec, xy, want, sign):
    """The planner's heading, or up to VIEW_MAX_DEV_DEG further into the turn: the first that sees
    VIEW_ENOUGH_M, else the one that sees farthest (a deviation must gain > 0.1 m)."""
    best, best_d = want, view_distance(occ, spec, xy, want)
    if best_d >= VIEW_ENOUGH_M or sign == 0:
        return want
    for k in range(VIEW_STEP_DEG, VIEW_MAX_DEV_DEG + 1, VIEW_STEP_DEG):
        h = want + sign * np.deg2rad(k)
        d = view_distance(occ, spec, xy, h)
        if d >= VIEW_ENOUGH_M:
            return float(np.arctan2(np.sin(h), np.cos(h)))
        if d > best_d + .1:
            best, best_d = h, d
    return float(np.arctan2(np.sin(best), np.cos(best)))


def corner_rule(planner, cmd, xy, yaw):
    """[v5 corner_view] Near a sharp route turn, steer the heading toward a far surface (at most
    VIEW_MAX_DEV_DEG past the planner's heading, into the turn). Sticky while the route ends within
    CORNER_WITHIN_M; never swings back against the turn within one corner (Gazebo v304). Only plain
    moves (no near miss / back out); the mission's carrot shrinks with the larger heading error."""
    from .online import _wrap
    if cmd.mode != "move" or cmd.reason or cmd.yaw is None or cmd.path is None or len(cmd.path) < 2:
        return
    path = np.asarray(cmd.path, float)
    turn = route_turn_ahead(path, CORNER_WITHIN_M, CORNER_SEG_M)
    err = _wrap(cmd.yaw - yaw)
    sharp = abs(turn) > np.deg2rad(CORNER_TURN_DEG)
    turning = abs(err) >= np.deg2rad(CORNER_TURNING_DEG)
    sticky = getattr(planner, "_corner_sign", 0.0)
    route_ends = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum()) < CORNER_WITHIN_M
    if not (sharp or turning or (sticky and route_ends)):
        planner._corner_sign, planner._corner_h = 0.0, None
        return
    sign = float(np.sign(turn)) if sharp else sticky if sticky else float(np.sign(err))
    if sign != sticky:
        planner._corner_h = None
    planner._corner_sign = sign
    if sharp or (sticky and route_ends):                       # never while merely turning: bounded
        want = float(cmd.yaw)
        h = view_heading(planner.belief.occ, planner.spec, xy, want, sign)
        last = getattr(planner, "_corner_h", None)
        if last is not None and sign * _wrap(h - last) < 0:
            dev = float(np.clip(sign * _wrap(last - want), 0.0, np.deg2rad(VIEW_MAX_DEV_DEG)))
            h = _wrap(want + sign * dev)
        planner._corner_h = h
        if abs(_wrap(h - cmd.yaw)) > 1e-6:
            cmd.yaw, cmd.reason = h, "corner-view"


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
    planner_name, altitude, camera_offset, pitch_down_deg, carrot_m, *rest = config
    opts = {**V5_OPTIONS, **(dict(rest[0]) if rest and rest[0] else {})}
    from .depth_projection import CameraMount, Intrinsics, decode_depth_image, drop_misregistered, project_depth
    from .world import clearance
    planner, spec, env, prior = build_planner(planner_name, altitude)
    planner.cfg.turn_in_place_deg = TURN_IN_PLACE_DEG
    planner.cfg.lookahead_m = LOOKAHEAD_M
    if opts["soft_margin"] is not None:
        # [v5 soft_margin] A/B only: re-cost the published map with another soft margin
        import dataclasses
        from .belief import BeliefMap
        env = dataclasses.replace(env, soft_margin=float(opts["soft_margin"]))
        planner.env = env
        planner.belief = BeliefMap(prior, spec, env, planner.cfg.suspect_penalty)
        planner.prior_cost = planner.belief.cost.copy()
        planner._near = env.hard + env.soft_margin
    planner.plan_cache = bool(opts["plan_cache"])
    if opts["look_guard"]:
        # [v5 look_guard] FlyX: a stop then an in-place spin near walls is what loses VIO (v3+view at the
        # tunnel mouth; run 6). Look around at most LOOK_GUARD_RAD, first toward the side that sees farther.
        planner.look_max_rad = LOOK_GUARD_RAD
        planner.look_sign_fn = lambda pl: long_view_sign(pl.belief.occ, pl.spec, pl.pos, pl.yaw)
    resets_seen, snap_cp = 0, None                   # [v5 forget_votes / forget_snap] state
    wall_dist = clearance(prior, spec)
    near_wall_normal = (wall_dist < WALL_SNAP_M) & ~prior
    near_wall_drift = (wall_dist < WALL_SNAP_DRIFT_M) & ~prior
    near_wall, on_wall_avg = near_wall_normal, 1.0
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
                on_wall = (prior | near_wall_normal)[raw_hit[:, 0], raw_hit[:, 1]]
                quality["hits"] += len(raw_hit)
                quality["hits_on_published_walls"] += int(on_wall.sum())
                # Drift-aware phantom filter: when recent depth agrees poorly with the published walls
                # the pose estimate has drifted, and wall returns land beside the walls as phantom
                # obstacles (real sim run 10: 63 % on walls, phantoms inside the 0.68 m-wide tunnel ->
                # "no path" -> turning in the tunnel mouth -> VIO lost -> crash). Widen what counts as
                # "that wall" then. Never moves the pose (unlike the failed drift correction).
                on_wall_avg = .9 * on_wall_avg + .1 * float(on_wall.mean())
                near_wall = near_wall_drift if on_wall_avg < DRIFT_ON_WALL else near_wall_normal
                quality["drift_filter_frames"] = quality.get("drift_filter_frames", 0) + int(on_wall_avg < DRIFT_ON_WALL)
            snap = near_wall
            if snap_cp is not None and snap_cp == planner.index:
                # [v5 forget_snap] phantoms sealed the route at this checkpoint: returns near a published
                # wall are that wall (drift snap) until the next checkpoint, so they cannot re-seal it
                snap = near_wall_drift
                quality["forget_snap_frames"] = quality.get("forget_snap_frames", 0) + 1
            free, hit, behind = drop_misregistered(*cells, prior, snap)
            votes = np.maximum(votes - 1, 0).astype(np.int8)
            if len(hit):
                votes[hit[:, 0], hit[:, 1]] = np.minimum(votes[hit[:, 0], hit[:, 1]] + 2, 6)
                known = planner.belief.occ[hit[:, 0], hit[:, 1]]
                ok = known | (votes[hit[:, 0], hit[:, 1]] >= HIT_CONFIRM)
                quality["hits_new_confirmed"] += int((ok & ~known).sum())
                quality["hits_new_ignored"] += int((~ok).sum())
                hit = hit[ok]
            if len(hit) and len(behind) and planner.map_resets == 0 and not opts["solid_off"]:
                # unexpected obstacles: the camera only sees their front face; the first SOLID_BEHIND_M
                # behind a confirmed new hit is solid (typical obstacle size), so the route keeps its
                # clearance from the unseen side. Fake-PX4 clutter courses: almost every contact was a
                # graze on the hidden side of a cylinder the camera had seen. Off once the planner has
                # had to forget its depth map: if this inference ever sealed a passage that is really
                # open, re-adding it would just get stuck again (back to suspect-only, the old rule).
                new = hit[~prior[hit[:, 0], hit[:, 1]]]
                if opts["solid_relax"] and len(new):     # [v5 solid_relax]
                    new = new[wall_dist[new[:, 0], new[:, 1]] > SOLID_MIN_WALL_M]
                if len(new):
                    d = np.linalg.norm(behind[:, None, :] - new[None, :, :], axis=-1).min(axis=1) * spec.resolution
                    hit = np.vstack([hit, behind[d <= SOLID_BEHIND_M]])
            planner.observe(free, hit, behind)
        cmd = planner.update(xy, yaw)
        forgot = sum(1 for e in planner.events if e.get("event") == "forget depth map")
        if forgot > resets_seen:
            resets_seen = forgot
            if opts["forget_votes"]:
                votes[:] = 0                         # [v5 forget_votes] phantoms must be seen again 2 of 3
            if opts["forget_snap"]:
                snap_cp = planner.index              # [v5 forget_snap]
        if opts["corner_view"] and cmd.mode == "move" and cmd.path is not None and len(cmd.path) >= 2:
            corner_rule(planner, cmd, np.asarray(xy, float), float(yaw))      # [v5 corner_view]
        if cmd.mode == "move" and cmd.path is not None and len(cmd.path) >= 2:
            # half-length step while stepping out of the keep-out after a near miss
            step = carrot_m * (.5 if cmd.reason == "near miss" else 1.0)
            cmd.path = safe_carrot_path(planner.belief.cost, spec, np.asarray(xy, float), cmd.path, step)
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
