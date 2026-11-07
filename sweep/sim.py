"""Offline closed-loop sim of Jolie's FlyX-proven program: her mission state machine and planner
process (exact embedded code, knobs as source edits) flying a PX4/cuVSLAM response model over
perturbed courses.

What runs unchanged from upload_to_flyx.py
    flyx.mission.Mission         tick() at 20 Hz, plan() at 5 Hz (carrot, slow turn, yaw cap,
                                 odometry-pause hold, pad centring, flight limit)
    pathplan.planner_process     the planner process loop itself (depth votes, WALL_SNAP,
                                 HIT_CONFIRM, drop_misregistered, safe_carrot_path), in a thread
    pathplan.online / belief / grid_search / world / depth_projection
What the sim supplies
    PX4 + vehicle      team_mission.rl.dynamics.DroneSim (position P loop, velocity lag, accel cap,
                       yaw loop, link delay, cuVSLAM drift + noise), parameters per level
    depth camera       a 32FC1 image rendered from the TRUE pose against the TRUE (perturbed)
                       course, projected by the planner with the ESTIMATED pose (so drift, noise
                       and stale poses misregister depth exactly as on the drone)
    program stalls     whole program frozen (no ticks, no planning); PX4 holds the last setpoint
    odometry pauses    no odometry for 0.3-1.4 s (the mission holds, the planner sees a stale pose)
    VIO risk           hazard of an estimate jump (+ pause, sometimes a loss) driven by the TRUE
                       yaw rate (> 0.35 rad/s) and by turning in place
    evaluation         true clearance of the disc centre (KD-tree on a 1 cm truth grid, ~1 cm);
                       touch = closer than the 0.28 m Sky Beta radius (counted, prop tip grazes);
                       crash = closer than CRASH (0.25 m, >= 3 cm into the wall: ends the run)
"""
from __future__ import annotations

import math
import os
import queue
import sys
import threading
from collections import deque
import time
import types
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
for p in (HERE, REPO / "pathfinding", REPO / "pathfinding" / "pathplan", REPO / "team_ws" / "src" / "team_mission"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import embedded as E                                      # noqa: E402
from team_mission.rl.dynamics import DroneSim, DynamicsRanges   # noqa: E402

DT = .05                      # control period (the program's 20 Hz timer)
PLAN_DT = .2                  # planning loop (5 Hz, or slower when a step takes longer)
# Planner step latency (deterministic CPU model): the result of mission.plan() reaches the control
# loop this long after the pose it was planned from; the loop waits max(PLAN_DT, latency).
# calibrate_latency.py on GB10 (flown code, original project_depth): a frame step takes 77 ms
# median, +16 ms with a replan; the FlyX lab CPU is slower and loaded, so ~1.3x that.
LAT_BASE, LAT_FRAME, LAT_REPLAN = .02, .08, .03
RADIUS = .28                  # Sky Beta footprint proxy
CRASH = .25                   # 3 cm past the prop tips: a strike, not a graze
ALTITUDE = .9
CAM_OFFSET = (.115, 0.0, .029)
TAKEOFF_S = 7.0               # FlyX: TAKEOFF -> NAVIGATE took 7.0 s (counts toward max_flight_s)
TRUTH_N = 610                 # evaluation / rendering grid: 610 x 610 (1.0 cm)
TRUTH_RES = 2 * 3.048 / TRUTH_N
WALL_TOP = 1.727
# depth image: OAK-D-like 72 deg HFOV, 640 x 400; project_depth samples every 8th pixel
IMG_W, IMG_H = 640, 400
FX = (IMG_W / 2) / math.tan(math.radians(36))
K = [FX, 0, IMG_W / 2, 0, FX, IMG_H / 2, 0, 0, 1]


# ------------------------------------------------------------------------------- levels
@dataclass
class Level:
    name: str
    wall_shift: float = 0.0           # m, uniform along/across for movable walls (variants.py)
    clutter: tuple = (0, 0)           # unmapped poles near the route
    clutter_radius: tuple = (.10, .18)
    dyn: dict = field(default_factory=dict)       # DynamicsRanges overrides
    depth_noise: float = 0.0          # z-depth std = k * z^2
    stall_rate: float = 0.0           # program stalls per second (FlyX: 86 in ~137 s = 0.63/s)
    stall_long_p: float = 0.0         # share of stalls that are long (0.8 .. stall_max s)
    stall_max: float = 1.95
    pause_every_s: float = 0.0        # mean interval between odometry pauses (0 = none)
    pause_s: tuple = (.3, 1.4)
    vio_scale: float = 0.0            # VIO hazard multiplier
    jump_m: float = .10               # std of a VIO estimate jump (per horizontal axis)
    # depth frame registered with the odometry this many 50 ms ticks older than the image
    # (pose_at matches stamps to within 0.05 s; residual stamp error and odometry delay remain)
    depth_lag: tuple = (0, 0)
    # "yawrate": hazard from yaw rate alone (+ turning in place); "yawspeed": hazard from yaw rate
    # above 0.35 rad/s times speed (the calibrated finding: divergence only when yawing fast while moving)
    vio_model: str = "yawrate"
    # real-time factor: physics seconds per program (wall) second. The drone is 1.0; the FlyX lab
    # runs ~0.8 while the program, its 5 Hz planner, yaw cap and stalls run on the wall clock.
    rtf: float = 1.0


NOMINAL_DYN = dict(kp_xy=(.95, .95), kp_z=(1., 1.), tau=(.5, .5), vel_i=(1., 1.), max_accel=(3., 3.),
                   px4_max_speed=(1.5, 1.5), k_yaw=(2.25, 2.25), max_yaw_rate=(.9, .9), yaw_tau=(.15, .15),
                   delay_steps=(1, 1), pos_noise=(0., 0.), vel_noise=(0., 0.), drift_per_sqrt_m=(0., 0.),
                   drift_rate=(0., 0.), yaw_drift_rate=(0., 0.), disturbance=(0., 0.), disturbance_tau=(1., 1.))


def _dyn(**kw):
    d = dict(NOMINAL_DYN)
    d.update(kw)
    return d


LEVELS = {
    # exactly the published course, ideal estimate, no stalls: the calibration level
    "L0": Level("L0"),
    # "mess with it slightly"
    "L1": Level("L1", wall_shift=.05, clutter=(0, 1), depth_noise=.003, depth_lag=(0, 1), stall_rate=.3, stall_long_p=.01,
                stall_max=1.0, pause_every_s=60, pause_s=(.3, .6), vio_scale=.5,
                dyn=_dyn(kp_xy=(.9, 1.0), tau=(.35, .6), delay_steps=(1, 2), pos_noise=(.002, .005),
                         vel_noise=(.01, .02), drift_per_sqrt_m=(0., .005), drift_rate=(0., .001),
                         yaw_drift_rate=(0., .003), disturbance=(0., .1))),
    # what FlyX actually measured: stalls 0.63/s up to 1.95 s, pauses 0.3-1.4 s, lag 0.35-1.0 s
    "L2": Level("L2", wall_shift=.10, clutter=(1, 2), depth_noise=.006, depth_lag=(0, 1), stall_rate=.63, stall_long_p=.03,
                pause_every_s=25, vio_scale=1.0,
                dyn=_dyn(kp_xy=(.85, 1.05), tau=(.35, 1.0), delay_steps=(1, 3), pos_noise=(.003, .01),
                         vel_noise=(.01, .04), drift_per_sqrt_m=(.0, .01), drift_rate=(0., .002),
                         yaw_drift_rate=(0., .005), disturbance=(0., .2))),
    "L3": Level("L3", wall_shift=.15, clutter=(1, 3), depth_noise=.01, depth_lag=(1, 2), stall_rate=.8, stall_long_p=.05,
                stall_max=2.5, pause_every_s=12, pause_s=(.3, 2.0), vio_scale=2.0, jump_m=.15,
                dyn=_dyn(kp_xy=(.8, 1.1), tau=(.5, 1.2), delay_steps=(2, 4), pos_noise=(.005, .015),
                         vel_noise=(.02, .06), drift_per_sqrt_m=(.005, .02), drift_rate=(0., .003),
                         yaw_drift_rate=(0., .01), disturbance=(0., .3))),
    "L4": Level("L4", wall_shift=.20, clutter=(2, 4), clutter_radius=(.12, .22), depth_noise=.015, depth_lag=(1, 3),
                stall_rate=1.0, stall_long_p=.08, stall_max=3.0, pause_every_s=8, pause_s=(.3, 2.5),
                vio_scale=3.0, jump_m=.2,
                dyn=_dyn(kp_xy=(.75, 1.15), tau=(.6, 1.4), delay_steps=(3, 5), pos_noise=(.01, .02),
                         vel_noise=(.03, .08), drift_per_sqrt_m=(.01, .03), drift_rate=(.001, .004),
                         yaw_drift_rate=(.005, .015), disturbance=(.1, .35))),
}

# ---- calibrated to 15 real PX4 SITL ulogs (branch dynamics-calib 4964e62,
# team_ws/src/team_mission/config/dynamics_calibrated.yaml): Kp 0.952, tau 0.83 s (runs 0.45-0.97),
# vel_i 0.66, delay ~0, k_yaw 4.04, yaw_tau 0.109 s, max yaw rate 1.41. VIO divergence (4 of 15 runs)
# came from yaw rate 0.46-0.80 rad/s WHILE MOVING 0.41-0.53 m/s at the first 90 deg corner, so the
# calibrated levels use the yaw-rate x speed hazard (vio_model "yawspeed").
CAL_POINT = _dyn(kp_xy=(.952, .952), tau=(.83, .83), vel_i=(.66, .66), delay_steps=(0, 0), k_yaw=(4.04, 4.04),
                 yaw_tau=(.109, .109), max_yaw_rate=(1.41, 1.41))
CAL_RANGES = dict(kp_xy=(.8, 1.25), kp_z=(.85, 1.15), tau=(.3713, 1.0477), vel_i=(0., .9956), max_accel=(1., 3.),
                  px4_max_speed=(1., 2.), k_yaw=(2.6292, 5.4606), max_yaw_rate=(.6, 1.62), yaw_tau=(.0558, .1739),
                  delay_steps=(0, 6), pos_noise=(.002, .02), vel_noise=(.01, .06), drift_per_sqrt_m=(0., .0541),
                  drift_rate=(0., .0247), yaw_drift_rate=(0., .01), disturbance=(0., .25), disturbance_tau=(.5, 3.))
LEVELS.update({
    # Gazebo-like: published course, fitted point dynamics, ideal estimate, yaw x speed VIO hazard
    "C0": Level("C0", dyn=CAL_POINT, vio_scale=1.0, vio_model="yawspeed"),
    # the full calibrated ranges + what FlyX measured (stalls, pauses), walls +-10 cm, 1-2 poles
    "C2": Level("C2", wall_shift=.10, clutter=(1, 2), depth_noise=.006, depth_lag=(0, 1), stall_rate=.63,
                stall_long_p=.03, pause_every_s=25, vio_scale=1.0, vio_model="yawspeed", dyn=dict(CAL_RANGES)),
    # heavier world on the calibrated ranges
    "C3": Level("C3", wall_shift=.15, clutter=(1, 3), depth_noise=.01, depth_lag=(1, 2), stall_rate=.8,
                stall_long_p=.05, stall_max=2.5, pause_every_s=12, pause_s=(.3, 2.0), vio_scale=1.5,
                vio_model="yawspeed", jump_m=.15, dyn=dict(CAL_RANGES)),
})


# ------------------------------------------------------------------------- the world
_COURSE = {}


def published():
    if not _COURSE:
        from drl.config import load_config          # pathfinding/drl: the published course model
        _, _COURSE["course"], _COURSE["route"] = load_config(str(REPO / "pathfinding/config/course_v3.yaml"))
    return _COURSE["course"], _COURSE["route"]


def truth_world(mods, level: Level, seed: int, cache_dir: Path | None = None):
    """True course for (level, seed): occupancy at the planning grid (5 cm) and at TRUTH_RES, the
    true clearance at TRUTH_RES, and what changed. Drawn like pathplan.harness (rejects variants
    with no legal route)."""
    key = f"v3-ws{level.wall_shift:.3f}-cl{level.clutter[0]}{level.clutter[1]}-" \
          f"r{level.clutter_radius[0]:.2f}{level.clutter_radius[1]:.2f}-s{seed}"
    if cache_dir is not None:
        f = Path(cache_dir) / f"truth-{key}.npz"
        if f.exists():
            z = np.load(f, allow_pickle=True)
            return _finish(dict(occ5=z["occ5"], occ_f=z["occ_f"], walls_f=z["walls_f"], info=z["info"].item()))
    from variants import perturb
    W = mods["pathplan.world"]
    course, route = published()
    spec, env = W.GridSpec(), W.Envelope()
    slab = .13 + .08
    prior, cps = E.prior_and_checkpoints(mods)
    start = np.array([-2.515, .914])
    rng = np.random.default_rng(10_000 + seed)
    nominal = _static_route(mods, W.cost_grid(prior, spec, W.Envelope()), spec, cps, start)
    keep_clear = [(start, .8), (np.asarray(cps[-1].centre), .8)] + [(cp.centre, .45) for cp in cps]
    info = {"changes": []}
    for attempt in range(40):
        c = course
        info = {"changes": [], "attempt": attempt}
        if level.wall_shift > 0:
            c, i = perturb(c, "walls", rng, wall_shift=level.wall_shift)
            info["changes"] += i["changes"]
        n_cl = int(rng.integers(level.clutter[0], level.clutter[1] + 1)) if level.clutter[1] > 0 else 0
        if n_cl > 0:
            c, i = perturb(c, "clutter", rng, route_xy=nominal, keep_clear=keep_clear,
                           clutter_radius=level.clutter_radius, clutter_count=(n_cl, n_cl))
            info["changes"] += i["changes"]
        occ5 = W.occupancy(c, spec, altitude=ALTITUDE, half_height=slab)
        if c is course:
            break
        if _static_route(mods, W.cost_grid(occ5, spec, W.Envelope()), spec, cps, start) is not None:
            break
    fspec = W.GridSpec(resolution=TRUTH_RES)
    occ_f = W.occupancy(c, fspec, altitude=ALTITUDE, half_height=slab, fine=TRUTH_RES)
    walls = occ_f.copy()                      # clearance to walls/obstacles only; the net is analytic
    walls[0, :] = walls[-1, :] = walls[:, 0] = walls[:, -1] = False
    out = dict(occ5=occ5, occ_f=occ_f, walls_f=walls, info=info)
    if cache_dir is not None:
        Path(cache_dir).mkdir(parents=True, exist_ok=True)
        tmp = Path(cache_dir) / f"tmp-{key}-{os.getpid()}.npz"
        np.savez_compressed(tmp, occ5=occ5, occ_f=occ_f, walls_f=walls, info=np.array(info, dtype=object))
        tmp.replace(f)
    return _finish(out)


def _finish(t):
    """KD-tree over occupied 1 cm wall cells: clearance(p) = distance to the nearest cell centre
    minus half a cell (the occupancy already inflates obstacles by 0.75 cm: within ~1 cm)."""
    from scipy.spatial import cKDTree
    rc = np.argwhere(t["walls_f"])
    t["tree"] = cKDTree((rc[:, ::-1] + .5) * TRUTH_RES - 3.048)
    return t


def _static_route(mods, cost, spec, cps, start_xy):
    from_cell = tuple(int(v) for v in spec.to_cell(start_xy))
    O, G = mods["pathplan.online"], mods["pathplan.grid_search"]
    out, cur = [], from_cell
    env = mods["pathplan.world"].Envelope()
    for cp in cps:
        goal = O.snap_goal(cost, spec, env, cp, .1)
        if goal is None:
            return None
        path, _, _ = G.astar(cost, cur, goal)
        if not path:
            return None
        out.extend(path if not out else path[1:])
        cur = goal
    return spec.to_xy(np.array(out))


class DepthRenderer:
    """32FC1 depth image from the true pose: 2-D ray cast per pixel column on the true slab
    occupancy (walls extruded floor .. 1.727 m), floor at z = 0. Only the pixels project_depth
    samples (every 8th) are filled; the rest are NaN (no return)."""

    def __init__(self, occ_f, half_size=3.048, res=TRUTH_RES):
        self.occ, self.hs, self.res = occ_f, half_size, res
        self.u = np.arange(0, IMG_W, 8)
        self.v = np.arange(0, IMG_H, 8)
        self.left = -(self.u - IMG_W / 2) / FX          # FLU left per unit forward, per column
        self.up = -(self.v - IMG_H / 2) / FX            # FLU up per unit forward, per row
        self.h = np.sqrt(1 + self.left ** 2)
        self.t = np.arange(0.0, 4.2, res / 2)
        self.img = np.full((IMG_H, IMG_W), np.nan, np.float32)

    def render(self, xy, yaw, z_cam, rng, noise_k=0.0):
        az = yaw + np.arctan(self.left)
        dirs = np.column_stack([np.cos(az), np.sin(az)])
        pts = np.asarray(xy)[None, None, :] + self.t[None, :, None] * dirs[:, None, :]
        rc = np.floor((pts[..., ::-1] + self.hs) / self.res).astype(int)
        n = self.occ.shape[0]
        inside = ((rc >= 0) & (rc < n)).all(-1)
        rc = np.clip(rc, 0, n - 1)
        hit = self.occ[rc[..., 0], rc[..., 1]] | ~inside
        first = np.where(hit.any(1), hit.argmax(1), len(self.t) - 1)
        D = np.where(hit.any(1), self.t[first], np.inf)                      # horizontal distance
        lam_w = (D / self.h)[None, :]                                        # optical z at the wall
        up = self.up[:, None]
        with np.errstate(invalid="ignore"):
            z_at_wall = z_cam + lam_w * up
        wall_ok = np.isfinite(lam_w) & (z_at_wall >= 0) & (z_at_wall <= WALL_TOP)
        with np.errstate(divide="ignore", invalid="ignore"):
            lam_f = np.where(up < 0, -z_cam / up, np.inf) * np.ones_like(lam_w)
        depth = np.where(wall_ok, np.minimum(lam_w, lam_f), lam_f)
        depth = np.where(np.isfinite(depth) & (depth < 9.0), depth, np.nan)
        if noise_k > 0:
            depth = depth + rng.normal(0, 1, depth.shape) * noise_k * depth ** 2
        self.img[np.ix_(self.v, self.u)] = depth.astype(np.float32)
        return self.img


# ------------------------------------------------------------------- planner process
class _ThreadConn:
    def __init__(self, inbox, outbox):
        self.inbox, self.outbox = inbox, outbox

    def recv(self):
        return self.inbox.get()

    def send(self, msg):
        self.outbox.put(msg)


class InlinePlannerProcess:
    """upload_to_flyx's PlannerProcess, with the child process replaced by a thread running the
    very same planner_process() function (signal/os.nice stubbed: threads cannot set them)."""

    def __init__(self, mods, build_planner, config):
        pp = mods["pathplan.planner_process"]
        pp.signal = types.SimpleNamespace(signal=lambda *a: None, SIGINT=2, SIG_IGN=None)
        pp.os = types.SimpleNamespace(nice=lambda n: None)
        to_child, to_parent = queue.Queue(), queue.Queue()
        self._child = _ThreadConn(to_child, to_parent)
        self.conn = _ThreadConn(to_parent, to_child)
        self.thread = threading.Thread(target=pp.planner_process, args=(self._child, build_planner), daemon=True)
        self.thread.start()
        self.events, self.cleared, self.ms, self.depth_quality = [], [], 0.0, {}
        self.frame = None
        self.conn.send(tuple(config))
        ready, self.pad_xy = self.conn.recv()
        assert ready == "ready"

    def give_frame(self, frame):
        self.frame = frame

    def update(self, xy, yaw):
        frame, self.frame = self.frame, None
        self.conn.send((xy, yaw, frame))
        cmd, self.events, self.cleared, self.ms, self.depth_quality = self.conn.recv()
        return cmd

    def close(self):
        self.conn.send(None)
        self.thread.join(2.0)


# ------------------------------------------------------------------------- one run
def _quat_ned(yaw_ned):
    return (math.cos(yaw_ned / 2), 0.0, 0.0, math.sin(yaw_ned / 2))


def _poisson_intervals(rng, rate, T, dur_fn):
    """Sorted [(start, end)] of events at ``rate`` per second over [0, T)."""
    out, t = [], 0.0
    if rate <= 0:
        return out
    while True:
        t += rng.exponential(1 / rate)
        if t >= T:
            return out
        d = dur_fn()
        out.append((t, t + d))
        t += d


def _in(intervals, t, k):
    """(inside, new index) for monotonically increasing t."""
    while k < len(intervals) and intervals[k][1] <= t:
        k += 1
    return (k < len(intervals) and intervals[k][0] <= t), k


def run(knobs, level_name, seed, *, cache_dir=None, t_max=185.0, trace=False, level=None, latency=None,
        fast_depth=True, timing_log=None):
    """Fly one (config, level, seed). Same seed -> same course, dynamics, stalls, pauses and noise
    streams for every config (common random numbers)."""
    wall0 = time.perf_counter()
    level = level or LEVELS[level_name]
    mods = E.install(knobs)
    if fast_depth:
        _install_fast_depth(mods)
    k = E.normalise(knobs)
    M, PX = mods["flyx.mission"], mods["flyx.px4_link"]
    truth = truth_world(mods, level, seed, cache_dir)
    occ_f, tree = truth["occ_f"], truth["tree"]

    def true_clear(xy):
        return float(tree.query(xy)[0]) - TRUTH_RES / 2

    # independent streams, all keyed by the seed only (common random numbers across configs)
    ss = np.random.SeedSequence([seed, 7])
    r_dyn, r_est, r_stall, r_pause, r_vio, r_depth = (np.random.default_rng(s) for s in ss.spawn(6))
    sim = DroneSim(dt=DT * level.rtf)
    sim.sample(DynamicsRanges.from_dict(level.dyn), r_dyn)
    start_xy, start_yaw = np.array([-2.515, .914]), math.pi / 2
    sim.reset([start_xy[0], start_xy[1], ALTITUDE], start_yaw, r_est)

    stalls = _poisson_intervals(r_stall, level.stall_rate, t_max, lambda: (
        r_stall.uniform(.8, level.stall_max) if r_stall.random() < level.stall_long_p
        else min(.15 + r_stall.exponential(.12), .8)))
    pauses = _poisson_intervals(r_pause, 1 / level.pause_every_s if level.pause_every_s else 0, t_max,
                                lambda: r_pause.uniform(*level.pause_s))
    vio_pauses = []                                  # added as VIO events happen

    built = {}
    factory = E.build_planner_factory(mods)

    def build(*a, **kw):
        out = factory(*a, **kw)
        built["planner"] = out[0]
        return out
    planner = InlinePlannerProcess(mods, build, ("theta", ALTITUDE, CAM_OFFSET, 0.0, k["carrot_m"]))
    online = built["planner"]
    lat = dict(base=LAT_BASE, frame=LAT_FRAME, replan=LAT_REPLAN, **(latency or {}))
    pending = None                                   # (apply time, target, planner_status)
    cfg = M.MissionConfig(execute=True, altitude=ALTITUDE, carrot_m=k["carrot_m"], pad_xy=tuple(planner.pad_xy))
    mission = M.Mission(planner, cfg)
    frame = PX.CourseFrame.from_launch(position_ned=[0.0, 0.0, 0.0], heading_ned=0.0, launch_xy=list(cfg.launch_xy),
                                       body_agl=cfg.launch_body_agl, physical_heading_in_course=cfg.launch_heading,
                                       placement_reviewed=True)
    mission.frame = frame
    north = frame.north_heading_in_course
    tel = M.Telemetry(now=0.0, sim_now=0.0, odom_t=0.0, status_t=0.0, arming_state=PX.ARMING_STATE_ARMED,
                      nav_state=PX.NAV_STATE_OFFBOARD, landed_t=0.0, landed=False, local_t=0.0,
                      xy_valid=True, z_valid=True)
    mission.state, mission._t_state, mission._t_takeoff, mission._sim_takeoff = "NAVIGATE", 0.0, -TAKEOFF_S, -TAKEOFF_S
    mission._takeoff_xy = start_xy.copy()
    mission._last_sp = None
    renderer = DepthRenderer(occ_f)
    max_turn_rate = mods["pathplan.planner_process"].MAX_TURN_RATE

    pose_hist = deque(maxlen=8)

    def set_odom(t):
        p_est, _, yaw_est = sim.estimate()
        tel.odom_t = t
        tel.position_ned = tuple(frame.to_ned(p_est))
        tel.q_ned_frd = _quat_ned(north - yaw_est)
        pose_hist.append(PX.odom_to_course(frame, tel.position_ned, tel.q_ned_frd))
        return p_est, yaw_est

    p_est, yaw_est = set_odom(0.0)
    mission.pose_course = PX.odom_to_course(frame, tel.position_ned, tel.q_ned_frd)
    sp_xyz, sp_yaw = np.array([start_xy[0], start_xy[1], ALTITUDE]), start_yaw
    t, next_plan = 0.0, 0.0
    ks = kp_ = kv = 0
    outcome, why, t_end = "timeout", "", None
    clears, stall_time = [], 0.0
    min_clear, contact_xy, touches, in_wall = np.inf, None, 0, False
    min_net, net_touches, in_net = np.inf, 0, False
    vio_events, vio_losses, fast_yaw_s, inplace_turn_s, fast_yaw_moving_s = 0, 0, 0.0, 0.0, 0.0
    max_track_err, track_err = 0.0, []
    path_len, last_true = 0.0, sim.p[:2].copy()
    trace_rows = []
    plan_steps = 0
    pause_until = -1.0
    while t < t_max:
        stalled, ks = _in(stalls, t, ks)
        paused, kp_ = _in(pauses, t, kp_)
        paused = paused or t < pause_until
        if stalled:
            stall_time += DT
        if not stalled and not paused:
            p_est, yaw_est = set_odom(t)
        if not stalled:
            tel.now, tel.sim_now = t, t * level.rtf
            tel.status_t = tel.local_t = tel.landed_t = t
            if pending is not None and t + 1e-9 >= pending[0]:
                mission.target, mission.planner_status = pending[1], pending[2]
                pending = None
            if pending is None and t + 1e-9 >= next_plan:
                gave = False
                if mission.planning_allowed() and not paused and abs(sim.yaw_rate) <= max_turn_rate:
                    img = renderer.render(sim.p[:2] + _rot(sim.yaw) @ np.array(CAM_OFFSET[:2]), sim.yaw,
                                          sim.p[2] + CAM_OFFSET[2], r_depth, level.depth_noise)
                    lag = int(r_depth.integers(level.depth_lag[0], level.depth_lag[1] + 1))
                    xyz, _, rotation = pose_hist[max(-len(pose_hist), -1 - lag)] if lag else mission.pose_course
                    planner.give_frame((img.tobytes(), IMG_W, IMG_H, "32FC1", IMG_W * 4, K, xyz, rotation))
                    gave = True
                calls0 = online.stats["plan_calls"]
                old = (mission.target, mission.planner_status)
                mission.plan()
                step_s = lat["base"] + lat["frame"] * gave + lat["replan"] * (online.stats["plan_calls"] - calls0)
                if step_s > 1e-9 and old[0] is not None:
                    pending = (t + step_s, mission.target, mission.planner_status)
                    mission.target, mission.planner_status = old
                next_plan = t + max(PLAN_DT, step_s)
                plan_steps += 1
                if timing_log is not None:      # measured compute of the flown code, for calibration
                    timing_log.append((gave, online.stats["plan_calls"] - calls0, planner.ms))
            out = mission.tick(tel)
            if out.state != "NAVIGATE":
                note = mission.events[-1]["note"] if mission.events else ""
                t_end = t
                if out.state == "LAND" and note.startswith("landing pad reached"):
                    outcome = "finished"
                else:
                    outcome, why = "landed_early", note
                break
            if out.setpoint is not None:
                pos_ned, yaw_ned = out.setpoint
                sp_xyz = frame.to_course(np.asarray(pos_ned, float))
                sp_yaw = north - yaw_ned
        sim.step(sp_xyz, sp_yaw)
        t += DT
        xy = sim.p[:2].copy()
        path_len += float(np.linalg.norm(xy - last_true))
        last_true = xy
        c = true_clear(xy)
        c_net = 3.048 - max(abs(xy[0]), abs(xy[1]))
        clears.append(min(c, c_net))
        min_clear = min(min_clear, c)
        if c_net < RADIUS and not in_net:
            net_touches += 1
        in_net = c_net < RADIUS
        min_net = min(min_net, c_net)
        err = float(np.linalg.norm(sim.p[:2] - (p_est[:2] if p_est is not None else xy)))
        omega, speed = abs(sim.yaw_rate), float(np.linalg.norm(sim.v[:2]))
        if omega > .35:
            fast_yaw_s += DT
            if speed > .3:
                fast_yaw_moving_s += DT
        inplace = speed < .1 and omega > .1
        if inplace:
            inplace_turn_s += DT
        # VIO hazard (per second): fast turns dominate (30/40 losses > 0.35 rad/s); turning in place
        # after a stop also lost it; a small floor for everything else
        if level.vio_scale > 0:
            if level.vio_model == "yawspeed":
                # 0.47/s at 0.6 rad/s and 0.47 m/s: ~50 % over a 1.5 s corner (4 of 15 runs diverged)
                hz = level.vio_scale * (.0005 + 2.0 * max(omega - .35, 0.0) * speed / .5)
            else:
                fast = min(max((omega - .30) / .10, 0.0), 1.0)
                hz = level.vio_scale * (.001 + .20 * fast + (.012 if inplace else 0.0))
            if r_vio.random() < hz * DT:
                vio_events += 1
                sim.bias[:2] += r_vio.normal(0, level.jump_m, 2)
                sim.yaw_bias += r_vio.normal(0, math.radians(3))
                if r_vio.random() < .2:
                    vio_losses += 1
                    pause_until = t + r_vio.uniform(1.5, 4.0)       # > 3 s: the mission lands
                else:
                    pause_until = t + r_vio.uniform(.3, 1.0)
        if trace:
            trace_rows.append((t, *sim.p[:2], sim.yaw, *sp_xyz[:2], c, mission.planner_status, stalled, paused))
        if c < RADIUS and not in_wall:
            touches += 1
            contact_xy = contact_xy or xy.round(3).tolist()
        in_wall = c < RADIUS
        if c < CRASH:
            outcome, t_end = "crash", t
            break
    planner.close()
    cl = np.asarray(clears) if clears else np.zeros(1)
    pad = np.asarray(planner.pad_xy)
    res = dict(level=level.name, seed=seed, outcome=outcome, why=why,
               course_s=round(t_end, 2) if t_end is not None else None,
               checkpoints=len(planner.cleared), cleared=";".join(c["name"][:1] for c in planner.cleared),
               min_wall=round(float(min_clear), 4), min_net=round(float(min_net), 4), net_touches=net_touches,
               min_clear=round(float(min(min_clear, min_net)), 4), p5_clear=round(float(np.percentile(cl, 5)), 4),
               touches=touches, contact_xy=contact_xy, pad_err=round(float(np.linalg.norm(sim.p[:2] - pad)), 3),
               vio_events=vio_events, vio_losses=vio_losses, fast_yaw_s=round(fast_yaw_s, 2), fast_yaw_moving_s=round(fast_yaw_moving_s, 2),
               inplace_turn_s=round(inplace_turn_s, 2), stall_s=round(stall_time, 2),
               path_m=round(path_len, 2), escapes=sum(e.get("event") == "escape" for e in planner.events),
               lookarounds=sum(e.get("event") == "look around" for e in planner.events),
               planner_events=len(planner.events), plan_steps=plan_steps,
               replans=online.stats["plan_calls"], wall_s=round(time.perf_counter() - wall0, 2))
    if trace:
        res["trace"] = trace_rows
        res["events"] = planner.events
        res["mission_events"] = mission.events
    return res


_FAST_OK = {}


def _install_fast_depth(mods, check=True):
    """Swap in fastdepth.project_depth (identical output; verified once per process on frames
    rendered from several poses of the published course)."""
    import fastdepth
    dp, W = mods["pathplan.depth_projection"], mods["pathplan.world"]
    fast = fastdepth.make_fast(dp)
    if check and not _FAST_OK:
        prior, _ = E.prior_and_checkpoints(mods)
        t = truth_world(mods, LEVELS["L0"], 0, HERE / "cache")
        r = DepthRenderer(t["occ_f"])
        rng = np.random.default_rng(1)
        spec = W.GridSpec()
        intr = dp.Intrinsics.from_k(K, IMG_W, IMG_H)
        for xy, yaw in [((-2.5, 1.2), 1.57), ((-2.0, 2.6), 0.2), ((0.3, 1.0), -1.0), ((1.8, -1.0), 3.0)]:
            img = r.render(np.array(xy), yaw, .93, rng, .01).astype(float)
            for e in (.0, .05):
                args = (img, intr, (xy[0] + e, xy[1] - e, .9), dp.body_to_course(yaw + e), spec)
                kw = dict(altitude=.9, half_height=.21, mount=dp.CameraMount(offset=CAM_OFFSET), expected=prior)
                a, b = dp.project_depth(*args, **kw), fast(*args, **kw)
                assert fastdepth.same(a, b), "fast project_depth differs from the flown one"
        _FAST_OK["ok"] = True
    dp.project_depth = fast


def _rot(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


if __name__ == "__main__":
    import json
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--level", default="L0")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--knob", nargs=2, action="append", default=[], metavar=("NAME", "VALUE"))
    a = ap.parse_args()
    r = run({n: float(v) for n, v in a.knob}, a.level, a.seed, cache_dir=HERE / "cache")
    print(json.dumps(r, indent=1))
