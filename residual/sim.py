"""Closed-loop simulator for Jolie's FlyX-proven program, with the residual optionally on.

What runs is the code that flew, loaded from the ``_SOURCES`` embedded in
jolie_course/upload_to_flyx.py (never from pathfinding/):

* ``flyx.mission.Mission``: her whole state machine (WAIT_HEALTH .. TAKEOFF, NAVIGATE, pad centring,
  LAND, every airborne guard, odometry-pause hold, failsafe ride-out, yaw cap, slow carrot);
* ``pathplan.planner_process.planner_process``: her planner child loop, run unmodified on a thread
  with an in-memory pipe (depth decode, ``project_depth``, ``drop_misregistered``, hit votes,
  ``OnlinePlanner`` with TURN_IN_PLACE_DEG = 60, ``safe_carrot_path``);
* the embedded course map and checkpoints (``_build_planner_embedded``).

The world around it:

* PX4 + airframe + cuVSLAM: team_mission/rl/dynamics.py ``DroneSim`` (position P loop -> velocity
  loop with lag and saturation, yaw loop, link delay, estimator drift / noise, disturbances), with
  a small PX4 mode model (Offboard, arm, COM_OF_LOSS_T -> failsafe hold and return, auto-land);
* depth: a synthetic 32FC1 depth image ray-cast from the TRUE pose against the TRUE (perturbed)
  course, passed with the ESTIMATED pose, so estimator error misplaces depth exactly as on the drone;
* lab starvation: program stalls (nothing ticks, PX4 flies the last setpoint), odometry pauses,
  slow-topic silences;
* visual odometry loss: a hazard rising with true yaw rate and with turning in place, giving drift
  jumps or estimate loss (PX4 reports xy invalid -> her guard lands);
* truth: contact / clearance against the perturbed course, checkpoints from the true trajectory.

Ground truth is used only for the world and for scoring; the residual sees ``residual_obs``.
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace
import importlib.util
import math
import queue
import signal as _signal
import sys
import threading
import types
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
BASE_FILE = REPO / "jolie_course" / "upload_to_flyx.py"
# The bases a residual can sit on. "drift" = Jolie's map-alignment (VIO drift correction) build:
# origin/jolie-drift-correction f7b97d3, single file rebuilt with its tools/build_single_file.py
# (identical to the committed pathfinding/flyx/upload_to_flyx.py there), frozen here.
BASES = {"proven": BASE_FILE, "drift": REPO / "residual" / "bases" / "upload_to_flyx_drift.py"}
ALIGN_GAIN, ALIGN_STEP_M, ALIGN_MAX_M = .6, .04, 1.5     # her node's apply_alignment (drift base only)
LAUNCH_XYZ = np.array([-2.515, .914, .05])
LAUNCH_YAW = math.pi / 2
DT = 0.05                              # control tick, LAB WALL seconds (20 Hz monotonic timer)
RTF = 0.8                              # FlyX lab sim runs at ~0.8x real time (coordinator, FlyX run)
PLAN_PERIOD = 0.2
MAX_SIM_S = 200.0
FOOTPRINT_R = 0.28                     # Sky Beta prop-tip radius: closer than this to a wall = contact
CLEARANCE_TARGET = 0.30

_M = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])    # course (ENU-like) <-> NED, own inverse

_BASE = None
_BASE_NAME = None
_STATIC = {}


def base(variant=None):
    """Jolie's single file as a module; its import installs the embedded packages (pathplan, flyx).
    Both bases use the same package names, so switching variant re-installs them."""
    global _BASE, _BASE_NAME
    variant = variant or _BASE_NAME or "proven"
    if _BASE is not None and variant != _BASE_NAME:
        for k in [k for k in sys.modules if k.split(".")[0] in ("pathplan", "flyx")]:
            del sys.modules[k]
        _BASE = None
    if _BASE is None:
        spec = importlib.util.spec_from_file_location("jolie_" + variant, BASES[variant])
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        pp = sys.modules["pathplan.planner_process"]
        # the child loop ignores SIGINT and renices itself; neither works on a thread / on Windows
        pp.signal = types.SimpleNamespace(signal=lambda *a: None, SIGINT=_signal.SIGINT, SIG_IGN=_signal.SIG_IGN)
        pp.os = types.SimpleNamespace(nice=lambda n: 0)
        _BASE, _BASE_NAME = mod, variant
    return _BASE


def _load_by_path(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def dynamics_module():
    if "team_mission.rl.dynamics" not in sys.modules:
        p = str(REPO / "team_ws" / "src" / "team_mission")
        if p not in sys.path:
            sys.path.insert(0, p)
    import team_mission.rl.dynamics as dyn
    return dyn


def static_world():
    """Published course, variants module, nominal route, keep-clear zones (cached per process)."""
    if _STATIC:
        return _STATIC
    base()
    pf = str(REPO / "pathfinding")
    if pf not in sys.path:
        sys.path.insert(0, pf)
    from drl.config import load_config
    from pathplan.grid_search import astar
    from pathplan.online import snap_goal
    from pathplan.world import cost_grid, occupancy
    _, course, route = load_config(str(REPO / "pathfinding" / "config" / "course_v3.yaml"))
    variants = _load_by_path("residual_variants", REPO / "pathfinding" / "pathplan" / "variants.py")
    planner, spec, env, prior = base()._build_planner_embedded()
    cps = planner.checkpoints

    def route_through(cost):
        cur, out = tuple(int(v) for v in spec.to_cell(LAUNCH_XYZ[:2])), []
        for cp in cps:
            goal = snap_goal(cost, spec, env, cp, .1)
            if goal is None:
                return None
            path, _, _ = astar(cost, cur, goal)
            if not path:
                return None
            out.extend(path if not out else path[1:])
            cur = goal
        return spec.to_xy(np.array(out))

    nominal = route_through(cost_grid(prior, spec, env))
    keep = [(LAUNCH_XYZ[:2], .8), (np.asarray(route.centre[route.names.index("pad_hover")][:2]), .8)]
    keep += [(cp.centre, .45) for cp in cps]
    _STATIC.update(course=course, variants=variants, spec=spec, env=env, prior=prior, checkpoints=cps,
                   nominal=nominal, keep_clear=keep, route_through=route_through,
                   fine=type(spec)(resolution=.025), occupancy=occupancy, cost_grid=cost_grid)
    return _STATIC


# ------------------------------------------------------------------------------------ levels
@dataclass(frozen=True)
class Level:
    name: str
    variant: str = "nominal"           # nominal | walls | mixed (walls + clutter)
    wall_shift: float = 0.0
    clutter: tuple = (1, 3)
    estimator: float = 0.0             # x dynamics.py default noise / drift / link-delay / disturbance ranges
    full_dynamics: bool = False        # domain-randomised PX4 / airframe ranges
    vio_scale: float = 0.0             # VIO-loss hazard multiplier (0 = risk counted, no events)
    launch_error: float = 0.0          # m / (x 0.1 rad) placement error on the mark
    depth_noise: float = 0.0           # stereo depth noise, sigma = k * z^2
    drift_per_m: float = 0.0           # systematic VIO drift: up to this many m per m flown (random direction)
    yaw_drift_per_m: float = 0.0       # and up to this heading drift, rad per m flown (random sign)
    dyn_yaml: str = ""                 # dynamics ranges from this YAML (DynamicsRanges.from_dict) instead
    vio_model: str = "yaw"             # yaw: hazard(yaw rate, in-place); corner: calibrated divergence model
    stall_rate: float = 0.0            # program stalls per second
    stall_max: float = 0.0
    odom_pause_rate: float = 0.0
    slow_silence_rate: float = 0.0
    description: str = ""


LEVELS = [
    Level("L0", description="published course, nominal dynamics"),
    Level("L1", "walls", .08, description="maze/chicane walls shifted up to 8 cm"),
    Level("L2", "mixed", .15, description="walls up to 15 cm + 1-3 unmapped posts"),
    Level("L3", "mixed", .15, estimator=.5, vio_scale=1.0, launch_error=.015, depth_noise=.01,
          description="+ half of dynamics.py's cuVSLAM noise/drift/delay/disturbance ranges, VIO-loss hazard, launch error 1.5 cm, depth noise"),
    Level("L4", "mixed", .15, estimator=.5, vio_scale=1.0, launch_error=.015, depth_noise=.01,
          stall_rate=.2, stall_max=1.2, odom_pause_rate=.05, slow_silence_rate=.02,
          description="+ lab starvation: stalls <= 1.2 s, odometry pauses 0.3-1.4 s, slow topics silent 2-6 s"),
    Level("L5", "mixed", .15, estimator=1.0, full_dynamics=True, vio_scale=1.5, launch_error=.03, depth_noise=.015,
          stall_rate=.4, stall_max=2.0, odom_pause_rate=.08, slow_silence_rate=.03,
          description="+ full dynamics.py estimator ranges, randomised PX4 gains/lag, FlyX-run-7 stalls (0.4/s, <= 2 s), VIO x1.5, launch error 3 cm"),
]
# Drift levels (for the map-alignment base; both bases are scored on them): the reported pose drifts
# away from the true pose while the camera sees from the true pose.
DRIFT_LEVELS = [
    Level("D1", "mixed", .15, drift_per_m=.08, yaw_drift_per_m=math.radians(.5),
          description="L2 + systematic VIO drift 0-8 cm/m and heading drift 0-0.5 deg/m"),
    Level("D2", "mixed", .15, estimator=.5, vio_scale=1.0, launch_error=.015, depth_noise=.01,
          stall_rate=.2, stall_max=1.2, odom_pause_rate=.05, slow_silence_rate=.02,
          drift_per_m=.08, yaw_drift_per_m=math.radians(.5), description="L4 + the D1 drift"),
]
# Calibrated levels (evaluation): dynamics-calib's ranges fitted to 15 real PX4 ulogs, FlyX lab
# starvation, and its VIO finding -- divergence where yaw rate x speed is high at the first
# corridor-to-maze corner, the estimate then drifting smoothly at 0.09-0.13 m/s with PX4 following.
CALIB_YAML = str(REPO / "residual" / "config" / "dynamics_calibrated.yaml")
CALIB_LEVELS = [
    Level("C0", dyn_yaml=CALIB_YAML, vio_model="corner", vio_scale=1.0, launch_error=.015, depth_noise=.01,
          stall_rate=.2, stall_max=1.2, odom_pause_rate=.05, slow_silence_rate=.02,
          description="published course, calibrated dynamics YAML (dynamics-calib 4964e62), lab starvation as L4, corner VIO divergence"),
    Level("C1", "mixed", .15, dyn_yaml=CALIB_YAML, vio_model="corner", vio_scale=1.0, launch_error=.015, depth_noise=.01,
          stall_rate=.2, stall_max=1.2, odom_pause_rate=.05, slow_silence_rate=.02,
          description="C0 on the L2 course perturbations (walls <= 15 cm + posts)"),
]
LEVEL = {lv.name: lv for lv in LEVELS + DRIFT_LEVELS + CALIB_LEVELS}
VIO_CORNER_XY = np.array([-2.35, 2.5])      # first corridor-to-maze 90 deg corner (~18-20 s in)


def corner_hazard(xy, yaw_rate, speed):
    """Per-second divergence probability (dynamics-calib): rises with |yaw rate| x speed, mostly at
    the first corridor-to-maze corner. Gazebo: 5 starts in 4/15 runs at 0.19-0.42 rad m/s^2;
    turning in place (speed ~0) at 1.14 rad/s did not diverge."""
    x = abs(yaw_rate) * speed - .12
    if x <= 0:
        return 0.0
    near = np.linalg.norm(np.asarray(xy) - VIO_CORNER_XY) < 1.0
    return 1.5 * x * (1.0 if near else .15)


# Calibrated to Jolie's completed FlyX run (lab wall clock, sim ~0.8x real time): NAVIGATE 120.85 s wall
# = ~97 s sim, TAKEOFF 7.0 s wall = 5.6 s sim. PX4's nominal MPC_XY_P 0.95 / 0.35 s lag gave NAVIGATE
# 71 s sim, too fast; an EFFECTIVE P of 0.7 with a 0.5 s velocity lag gives 97-101 s (README).
NOMINAL_PX4 = dict(kp_xy=.7, tau=.5, kp_z=.35)


def dynamics_ranges(level):
    dyn = dynamics_module()
    nominal = dyn.DynamicsRanges.nominal()
    if level.dyn_yaml:
        import yaml
        return dyn.DynamicsRanges.from_dict(yaml.safe_load(Path(level.dyn_yaml).read_text()))
    if level.full_dynamics:
        # every dynamics.py range (incl. its full estimator ranges), PX4 gains/lag around the calibrated
        # nominal (velocity lag 0.35-1.0 s as measured in the real runs)
        return replace(dyn.DynamicsRanges(), kp_xy=(.55, .9), tau=(.35, 1.0), kp_z=(.3, .5))
    # nominal: calibrated PX4 tracking + the few-mm jitter every real cuVSLAM pose has (a noise-free
    # pose is not nominal: her planner's replan-by-distance relies on it, see README)
    r = replace(nominal, pos_noise=(.003, .003), vel_noise=(.01, .01), **{k: (v, v) for k, v in NOMINAL_PX4.items()})
    if level.estimator:
        d, k = dyn.DynamicsRanges(), float(level.estimator)
        sc = lambda rg, lo=0.0: (max(lo, rg[0] * k), max(lo, rg[1] * k))
        r = replace(r, pos_noise=sc(d.pos_noise, .003), vel_noise=sc(d.vel_noise, .01),
                    drift_per_sqrt_m=sc(d.drift_per_sqrt_m), drift_rate=sc(d.drift_rate),
                    yaw_drift_rate=sc(d.yaw_drift_rate), disturbance=sc(d.disturbance),
                    delay_steps=(d.delay_steps[0], int(round(d.delay_steps[1] * k))))
    return r


# ------------------------------------------------------------------------------------ helpers
class _Conn:
    def __init__(self, inbox, outbox):
        self.inbox, self.outbox = inbox, outbox

    def send(self, x):
        self.outbox.put(x)

    def recv(self):
        return self.inbox.get()


class InProcessPlanner:
    """PlannerProcess with Jolie's child loop on a thread (same messages, same code)."""

    def __init__(self, build_planner, config):
        pp = sys.modules["pathplan.planner_process"]
        a, b = queue.Queue(), queue.Queue()
        self.conn = _Conn(b, a)
        self.thread = threading.Thread(target=pp.planner_process, args=(_Conn(a, b), build_planner), daemon=True)
        self.thread.start()
        self.pad_xy = None
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
        cmd, self.events, self.cleared, self.ms, self.depth_quality, *rest = self.conn.recv()
        self.align = rest[0] if rest else None          # drift base: map-alignment shift
        return cmd

    def close(self):
        self.conn.send(None)
        self.thread.join(5.0)


class DepthRenderer:
    """Synthetic OAK-D-like depth (32FC1) from the true pose on the true course. Walls are
    vertical extrusions to WALL_TOP; rows below the horizon return the floor if it is nearer."""
    W, H, WALL_TOP = 320, 40, 1.70             # project_depth samples every 8th pixel: 40 x 5 rays

    def __init__(self, truth_fine, spec_fine, rng, noise=0.0):
        self.occ, self.spec, self.rng, self.noise = truth_fine, spec_fine, rng, noise
        self.fx = self.fy = (self.W / 2) / math.tan(math.radians(36.0))
        self.cx, self.cy = self.W / 2, self.H / 2
        self.k = [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0]
        u = np.arange(0, self.W, 8)
        self.u = u
        self.theta = np.arctan(-(u - self.cx) / self.fx)          # left of the optical axis = +
        self.t = np.arange(.05, 4.0, self.spec.resolution / 2)
        self.slope = -(np.arange(self.H) - self.cy) / self.fy      # up per metre forward

    def render(self, pos, yaw):
        cam = np.asarray(pos, float) + np.array([math.cos(yaw), math.sin(yaw), 0.0]) * .115 + np.array([0, 0, .029])
        ang = yaw + self.theta
        pts = cam[None, None, :2] + self.t[None, :, None] * np.stack([np.cos(ang), np.sin(ang)], 1)[:, None, :]
        rc = np.floor((pts[..., ::-1] + self.spec.half_size) / self.spec.resolution).astype(int)
        inside = ((rc >= 0) & (rc < self.occ.shape[0])).all(-1)
        rc = np.clip(rc, 0, self.occ.shape[0] - 1)
        hit = self.occ[rc[..., 0], rc[..., 1]] | ~inside
        first = np.where(hit.any(1), hit.argmax(1), -1)
        r_h = np.where(first >= 0, self.t[np.maximum(first, 0)], np.inf)
        z_wall = r_h * np.cos(self.theta)                          # forward (optical z) distance
        s = self.slope[:, None]
        h = cam[2]
        with np.errstate(divide="ignore", invalid="ignore"):
            z_floor = np.where(s < 0, h / -s, np.inf)
            top = h + s * z_wall[None, :]
            depth = np.where(top > self.WALL_TOP, np.nan, z_wall[None, :])
            depth = np.fmin(depth, z_floor)
        depth = np.where(np.isfinite(depth) & (depth <= 3.5), depth, np.nan)
        if self.noise:
            depth = depth * (1 + self.rng.normal(0, self.noise, depth.shape[1]) * depth)
        img = np.full((self.H, self.W), np.nan, np.float32)
        img[:, self.u] = depth                                     # project_depth samples stride 8
        return img


def _q_from_yaw_ned(psi):
    return (math.cos(psi / 2), 0.0, 0.0, math.sin(psi / 2))


def _rot_flu_to_course(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _schedule(rng, rate, lo, hi, horizon, dist="uniform"):
    out, t = [], 0.0
    if rate <= 0:
        return out
    while True:
        t += rng.exponential(1.0 / rate)
        if t > horizon:
            return out
        d = float(min(hi, lo + rng.exponential(.25))) if dist == "exp" else float(rng.uniform(lo, hi))
        out.append((t, t + d))
        t += d


def _active(sched, t):
    return any(a <= t < b for a, b in sched)


# ------------------------------------------------------------------------------------ world
def make_world(level, rng):
    st = static_world()
    spec, env, course = st["spec"], st["env"], st["course"]
    slab = env.half_height + env.tracking_margin
    for _ in range(50):
        if level.variant == "nominal":
            vcourse, info = course, {"kind": "nominal"}
        else:
            vcourse, info = st["variants"].perturb(course, level.variant, rng, route_xy=st["nominal"],
                                                   keep_clear=st["keep_clear"], wall_shift=level.wall_shift,
                                                   clutter_count=level.clutter)
        truth_plan = st["prior"] if level.variant == "nominal" else \
            st["occupancy"](vcourse, spec, altitude=.9, half_height=slab)
        if level.variant == "nominal" or st["route_through"](st["cost_grid"](truth_plan, spec, env)) is not None:
            break
    key = ("fine", id(vcourse)) if level.variant != "nominal" else ("fine", "nominal")
    fine = st["fine"]
    if key in _STATIC:
        truth_fine, clear_fine = _STATIC[key]
    else:
        from pathplan.world import clearance
        truth_fine = st["occupancy"](vcourse, fine, altitude=.9, half_height=env.half_height, fine=.0125)
        clear_fine = clearance(truth_fine, fine)
        if level.variant == "nominal":
            _STATIC[key] = (truth_fine, clear_fine)
    return {"info": info, "truth_fine": truth_fine, "clear_fine": clear_fine, "fine": fine, "course": vcourse}


def exact_clearance(course, xyz, half_height, hi=.5):
    """Distance (m, capped at ``hi``) from the drone's vertical slab at xyz to the nearest true
    obstacle: exact point-to-rectangle for walls/boxes and point-to-circle for posts (rounded corners,
    unlike Course.collides' inflated rectangles), bisection on the tube shell, and the net. The grid
    clearance is only a pre-filter (max-pooling makes it read 1-3 cm short)."""
    p = np.asarray(xyz, float)
    d = min(hi, course.half_size - abs(p[0]), course.half_size - abs(p[1]))
    if len(course.box_c):
        dz = np.abs(p[2] - course.box_c[:, 2]) < course.box_h[:, 2] + half_height
        if dz.any():
            c, u, h = course.box_c[dz], course.box_u[dz], course.box_h[dz]
            e = p[:2] - c[:, :2]
            lu = np.abs(e[:, 0] * u[:, 0] + e[:, 1] * u[:, 1]) - h[:, 0]
            ln = np.abs(-e[:, 0] * u[:, 1] + e[:, 1] * u[:, 0]) - h[:, 1]
            d = min(d, float(np.hypot(np.maximum(lu, 0), np.maximum(ln, 0)).min()))
    if len(course.cyl_xy):
        up = p[2] - half_height < course.cyl_top
        if up.any():
            d = min(d, float((np.linalg.norm(course.cyl_xy[up] - p[:2], axis=1) - course.cyl_r[up]).min()))
    if course.tube is not None and d > 0:
        tube_only = _TUBE_ONLY.get("tube")   # the tube is never perturbed
        if tube_only is None:
            tube_only = _TUBE_ONLY["tube"] = replace(course, box_c=np.zeros((0, 3)), box_u=np.zeros((0, 2)),
                                                         box_h=np.zeros((0, 3)), cyl_xy=np.zeros((0, 2)),
                                                         cyl_r=np.zeros(0), cyl_top=np.zeros(0))
        q, lo, top = p[None], 0.0, d
        if tube_only.collides(q, top, half_height)[0]:
            for _ in range(10):
                mid = (lo + top) / 2
                if tube_only.collides(q, mid, half_height)[0]:
                    top = mid
                else:
                    lo = mid
            d = lo
    return max(d, 0.0)


_TUBE_ONLY = {}


class FakePX4:
    """Just enough PX4 for her state machine: Offboard needs a setpoint stream, arm needs Offboard,
    no setpoint for COM_OF_LOSS_T -> failsafe hold (returns once setpoints flow), Land descends."""
    OF_LOSS_T = 1.0

    def __init__(self):
        self.nav_state, self.arming_state, self.landed = 2, 1, True       # POSCTL, disarmed
        self.failsafe, self.user_intention = False, 2
        self.sp, self.sp_t, self.stream_since = None, -1e9, None
        self.hold, self.land_xy, self.resume_t = None, None, None
        self.land_cmds = 0

    def setpoint(self, xyz_world, yaw_world, t):
        if self.sp_t < t - .5:
            self.stream_since = t
        self.sp, self.sp_t = (np.asarray(xyz_world, float), float(yaw_world)), t

    def command(self, cmd, t, est_xyz):
        if cmd.command == 176 and self.sp_t > t - .5:
            self.nav_state = self.user_intention = 14
        elif cmd.command == 400 and self.nav_state == 14:
            self.arming_state = 2
        elif cmd.command == 21 and self.arming_state == 2:
            self.land_cmds += 1
            if self.nav_state != 18:
                self.nav_state, self.user_intention, self.failsafe = 18, 18, False
                self.land_xy = np.asarray(est_xyz, float).copy()

    def target(self, t, est_xyz, est_yaw):
        """Where PX4's controller is flying (world frame, PX4 believes est_xyz)."""
        if self.arming_state != 2:
            return None
        if self.nav_state == 18:
            z = max(0.0, est_xyz[2] - .5)                        # MPC_LAND_SPEED-ish descent
            return np.array([self.land_xy[0], self.land_xy[1], z]), est_yaw
        if self.user_intention == 14:
            if t - self.sp_t > self.OF_LOSS_T:                            # setpoints stopped
                if not self.failsafe:
                    self.failsafe, self.nav_state, self.hold = True, 4, (np.asarray(est_xyz, float).copy(), est_yaw)
                    self.resume_t = None
                return self.hold
            if self.failsafe:                                             # setpoints are back
                self.resume_t = self.resume_t if self.resume_t is not None else t
                if t - self.resume_t >= .3:
                    self.failsafe, self.nav_state = False, 14
                else:
                    return self.hold
            return self.sp
        return np.asarray(est_xyz, float), est_yaw


# ------------------------------------------------------------------------------------ episode
def vio_hazard(yaw_rate, speed):
    """Per-second probability of a VIO event. Calibrated to the real runs only qualitatively:
    30/40 losses above 0.35 rad/s, Gazebo completions at 0.25, in-place turns after full stops lost it."""
    w = abs(yaw_rate)
    if w < .1:
        return .0002
    h = .0015 * math.exp((w - .25) / .05)
    if speed < .08:
        h *= 3.0
    return min(h, 5.0)


@dataclass
class EpisodeResult:
    level: str
    seed: int
    residual: bool
    complete: bool = False
    outcome: str = ""
    course_time: float = float("nan")      # NAVIGATE duration (sim s) of a completed run
    navigate_time: float = float("nan")
    state_times: dict = field(default_factory=dict)
    cps_true: int = 0
    contact: bool = False
    min_clearance: float = float("inf")
    time_below_target: float = 0.0
    clearance_deficit: float = 0.0          # integral of max(0, 0.30 - clearance) dt, m s
    vio_risk: float = 0.0                   # expected VIO events (integrated hazard)
    vio_events: int = 0
    vio_losses: int = 0
    stalls: int = 0
    failsafe_blips: int = 0
    smooth: float = 0.0
    applied: int = 0
    zeroed: int = 0
    mission_state: str = ""
    note: str = ""
    info: dict = field(default_factory=dict)
    variant: str = "proven"
    align_total_m: float = 0.0
    drift_final_m: float = 0.0
    traj: list = field(default_factory=list)

    def fitness(self):
        """Episode return used for training (truth only here, never in the policy's input)."""
        f = 40.0 * self.complete + 8.0 * self.cps_true
        if self.complete:
            f += 0.5 * (180.0 - self.course_time)
        f -= 80.0 * self.contact
        f -= 100.0 * self.clearance_deficit
        f -= 30.0 * self.vio_risk + 5.0 * self.vio_events
        f -= 2.0 * self.smooth
        return f


def run_episode(level, seed, policy_params=None, residual=False, record=False, max_s=MAX_SIM_S, variant="proven"):
    """One flight. residual=False flies Jolie's Mission class itself; residual=True the subclass
    from residual/runtime.py with ``policy_params`` (None -> always zero correction)."""
    if isinstance(level, str):
        level = LEVEL[level]
    b = base(variant)
    rt = sys.modules.get("residual.runtime") or __import__("residual.runtime", fromlist=["x"])
    rng = np.random.default_rng(seed)
    world = make_world(level, np.random.default_rng([seed, 1]))
    rng_dyn, rng_ev, rng_cam = (np.random.default_rng([seed, k]) for k in (2, 3, 4))
    dyn = dynamics_module()
    DTS = DT * RTF                          # sim seconds per control tick
    sim = dyn.DroneSim(dt=DTS)
    sim.sample(dynamics_ranges(level), rng_dyn)
    launch_true = LAUNCH_XYZ.copy()
    yaw0 = LAUNCH_YAW
    if level.launch_error:
        launch_true[:2] += rng.normal(0, level.launch_error, 2)
        yaw0 += rng.normal(0, level.launch_error * .5)
    sim.reset(launch_true, yaw0, rng_dyn)
    drift_dir = rng.uniform(0, 2 * math.pi)
    drift_rate = rng.uniform(0, level.drift_per_m)
    yaw_drift = rng.uniform(-level.yaw_drift_per_m, level.yaw_drift_per_m)
    align_total = np.zeros(2)
    diverge_until, diverge_vel = -1.0, np.zeros(3)

    carrot = .5
    build = b._build_planner_embedded
    MissionCls = b.Mission
    if residual:
        build = rt.wrap_build_planner(build, carrot)
        MissionCls = rt.make_residual_mission(b.Mission, policy_params)
    planner = InProcessPlanner(build, ("theta", .9, (.115, 0.0, .029), 0.0, carrot))
    mission = MissionCls(planner, b.MissionConfig(execute=True, altitude=.9, carrot_m=carrot,
                                                  launch_xy=(-2.515, .914), launch_heading=1.5707963,
                                                  launch_body_agl=.05, pad_xy=tuple(planner.pad_xy)))
    renderer = DepthRenderer(world["truth_fine"], world["fine"], rng_cam, level.depth_noise)
    px4 = FakePX4()
    tel = b.Telemetry(now=0.0)
    res = EpisodeResult(level.name, seed, residual, info=world["info"], variant=variant)
    stalls = _schedule(rng_ev, level.stall_rate, .15, level.stall_max, MAX_SIM_S, "exp")
    odom_pauses = _schedule(rng_ev, level.odom_pause_rate, .3, 1.4, MAX_SIM_S)
    slow_silences = _schedule(rng_ev, level.slow_silence_rate, 2.0, 6.0, MAX_SIM_S)
    res.stalls = sum(1 for a, _ in stalls if a > 5)
    p0 = launch_true.copy()                 # PX4's local origin: where the EKF started
    cps = static_world()["checkpoints"]
    clear, fine = world["clear_fine"], world["fine"]
    next_plan, t = 0.0, 0.0
    vio_pause_until, vio_loss_start, vio_loss_until, vio_loss_jump = -1.0, -1.0, -1.0, None
    t_takeoff = None
    prev_true = sim.p[:2].copy()
    cp_i = 0
    other_ok = False
    prev_res = None
    step = 0
    prev_state, state_t = None, {}
    try:
        while t < max_s:
            w = step * DT                       # lab wall clock: what the program's monotonic clock reads
            t = w * RTF                         # sim time: physics, PX4, /clock
            step += 1
            p_est, v_est, yaw_est = sim.estimate()
            est_ned = _M @ (p_est - p0)
            # ------------------------------------------------ telemetry arrives (unless paused)
            odom_ok = not _active(odom_pauses, w) and t >= vio_pause_until and t >= vio_loss_until
            if odom_ok:
                tel.odom_t, tel.position_ned = w, tuple(est_ned)
                tel.q_ned_frd = _q_from_yaw_ned(math.pi / 2 - yaw_est)
            if not _active(slow_silences, w):
                tel.status_t, tel.local_t, tel.landed_t = w, w, w
                tel.arming_state, tel.nav_state = px4.arming_state, px4.nav_state
                tel.failsafe, tel.user_intention = px4.failsafe, px4.user_intention
                tel.landed = px4.landed
                # PX4 dead-reckons through a short VIO loss; after 1 s without it xy is invalid
                tel.xy_valid = not (vio_loss_start <= t < vio_loss_until and t - vio_loss_start >= 1.0)
                tel.z_valid = True
            tel.now, tel.sim_now = w, t
            # ------------------------------------------------ the program (unless stalled)
            stalled = _active(stalls, w)
            if not stalled:
                out = mission.tick(tel)
                if out.setpoint is not None:
                    pos_ned, yaw_ned = out.setpoint
                    px4.setpoint(_M @ np.asarray(pos_ned, float) + p0, math.pi / 2 - yaw_ned, t)
                if out.command is not None:
                    px4.command(out.command, t, p_est)
                if w + 1e-9 >= next_plan:
                    next_plan = w + PLAN_PERIOD
                    if mission.frame is not None and mission.planning_allowed() and mission.pose_course is not None:
                        if abs(sim.yaw_rate) <= b.MAX_TURN_RATE:
                            img = renderer.render(sim.p, sim.yaw)
                            xyz, _, rot = mission.pose_course
                            planner.give_frame((img.tobytes(), renderer.W, renderer.H, "32FC1", renderer.W * 4,
                                                renderer.k, np.asarray(xyz, float), rot))
                        mission.plan()
                        if variant == "drift" and getattr(planner, "align", None) is not None and mission.frame is not None:
                            step_a = ALIGN_GAIN * np.asarray(planner.align, float)      # = her apply_alignment
                            na = float(np.linalg.norm(step_a))
                            if na > ALIGN_STEP_M:
                                step_a *= ALIGN_STEP_M / na
                            if np.linalg.norm(align_total + step_a) <= ALIGN_MAX_M:
                                mission.frame.origin_in_course = mission.frame.origin_in_course + np.array([*step_a, 0.0])
                                align_total += step_a
                        if residual:
                            r = np.array(mission.residual)
                            if prev_res is not None:
                                res.smooth += float(np.sum(((r - prev_res) / np.array([.7, .16, .4])) ** 2)) * PLAN_PERIOD
                            prev_res = r
            if mission.state in ("DONE", "FAILED", "TAKEN_OVER"):
                break
            if mission.state != prev_state:
                state_t[mission.state] = t
                prev_state = mission.state
            if mission.state == "TAKEOFF" and t_takeoff is None:
                t_takeoff = t
            # ------------------------------------------------ PX4 + airframe
            tgt = px4.target(t, p_est, yaw_est)
            if tgt is None:
                sim.step(sim.p + sim.bias, sim.yaw + sim.yaw_bias, ground_z=LAUNCH_XYZ[2])
            else:
                sim.step(tgt[0], tgt[1], ground_z=LAUNCH_XYZ[2])
            if drift_rate or yaw_drift:                     # systematic VIO drift with distance flown
                moved = float(np.linalg.norm(sim.p[:2] - prev_true))
                sim.bias[:2] += drift_rate * moved * np.array([math.cos(drift_dir), math.sin(drift_dir)])
                sim.yaw_bias += yaw_drift * moved
            if px4.arming_state == 2:
                px4.landed = sim.p[2] < LAUNCH_XYZ[2] + .03 and px4.nav_state == 18
                if px4.landed:
                    px4.arming_state, px4.nav_state = 1, 2
            if px4.failsafe and not getattr(px4, "_blip", False):
                res.failsafe_blips += 1
            px4._blip = px4.failsafe
            # ------------------------------------------------ visual odometry
            speed = float(np.linalg.norm(sim.v[:2]))
            if sim.p[2] > .3 and level.vio_model == "corner":
                h = corner_hazard(sim.p[:2], sim.yaw_rate, speed)
                res.vio_risk += h * DTS
                if level.vio_scale and t >= diverge_until and rng_ev.random() < h * level.vio_scale * DTS:
                    res.vio_events += 1
                    ang = rng_ev.uniform(0, 2 * math.pi)
                    diverge_vel = rng_ev.uniform(.09, .13) * np.array([math.cos(ang), math.sin(ang), 0.0])
                    diverge_until = t + float(rng_ev.uniform(2.0, 6.0))
            if t < diverge_until:
                sim.bias += diverge_vel * DTS          # the estimate slides; PX4 follows it
            if sim.p[2] > .3 and level.vio_model == "yaw":
                h = vio_hazard(sim.yaw_rate, speed)
                res.vio_risk += h * DTS
                if level.vio_scale and t >= vio_loss_until and rng_ev.random() < h * level.vio_scale * DTS:
                    res.vio_events += 1
                    ang = rng_ev.uniform(0, 2 * math.pi)
                    if rng_ev.random() < .25:                       # estimate lost
                        res.vio_losses += 1
                        vio_loss_start = t
                        vio_loss_until = t + float(rng_ev.uniform(.6, 2.5))
                        vio_loss_jump = rng_ev.uniform(.05, .25) * np.array([math.cos(ang), math.sin(ang), 0])
                    else:                                           # drift jump + short pause
                        sim.bias += rng_ev.uniform(.05, .25) * np.array([math.cos(ang), math.sin(ang), 0])
                        sim.yaw_bias += rng_ev.normal(0, .03)
                        vio_pause_until = t + float(rng_ev.uniform(.3, .8))
            if t < vio_loss_until:
                sim.bias += rng_ev.normal(0, .15 * math.sqrt(DTS), 3) * np.array([1, 1, .2])   # dead reckoning
            elif vio_loss_jump is not None:
                sim.bias += vio_loss_jump
                vio_loss_jump = None
            # ------------------------------------------------ truth: contact, clearance, checkpoints
            p = sim.p[:2].copy()
            if sim.p[2] > .3 and px4.arming_state == 2:
                rc = fine.to_cell(p)
                c = float(clear[rc[0], rc[1]])
                if c < .45:
                    c = exact_clearance(world["course"], sim.p, static_world()["env"].half_height)
                res.min_clearance = min(res.min_clearance, c)
                if c < CLEARANCE_TARGET:
                    res.time_below_target += DTS
                    res.clearance_deficit += (CLEARANCE_TARGET - c) * DTS
                if c < FOOTPRINT_R:
                    res.contact = True
                    res.outcome = "contact"
                    break
            if cp_i < len(cps) - 1:
                cp = cps[cp_i]
                if cp.crossed(prev_true, p, FOOTPRINT_R) or (cp.other_route and any(
                        c.get("name") == cp.name and c.get("via") == "other" for c in planner.cleared)):
                    cp_i += 1
            prev_true = p
            if record and step % 2 == 0:
                res.traj.append([round(t, 2), *np.round(sim.p, 3).tolist(), round(sim.yaw, 3), mission.state])
    finally:
        planner.close()
    pad = cps[-1]
    landed_on_pad = cp_i == len(cps) - 1 and np.linalg.norm(sim.p[:2] - pad.centre) <= .762 - FOOTPRINT_R
    res.cps_true = cp_i + (1 if landed_on_pad and mission.state == "DONE" else 0)
    land_note = next((e["note"] for e in reversed(mission.events) if e["to"] == "LAND"), "")
    res.mission_state, res.note = mission.state, land_note
    if not res.outcome:
        if mission.state == "DONE" and land_note.startswith("landing pad reached") and res.cps_true == len(cps):
            res.outcome = "complete"
        elif mission.state == "DONE":
            res.outcome = "landed: " + land_note[:40]
        else:
            res.outcome = mission.state.lower()
    res.complete = res.outcome == "complete" and t_takeoff is not None and (t - t_takeoff) <= 180.0
    if "NAVIGATE" in state_t:
        res.navigate_time = state_t.get("LAND", t) - state_t["NAVIGATE"]
    res.state_times = {k: round(v, 2) for k, v in state_t.items()}
    if res.complete:
        res.course_time = res.navigate_time
    res.align_total_m = float(np.linalg.norm(align_total))
    res.drift_final_m = float(np.linalg.norm(sim.bias[:2]))
    if residual:
        res.applied = mission.residual_stats["applied"]
        res.zeroed = mission.residual_stats["zero"]
    if not math.isfinite(res.min_clearance):
        res.min_clearance = float("nan")
    return res


if __name__ == "__main__":
    import time
    sys.path.insert(0, str(REPO))
    for lv in sys.argv[1:] or ["L0"]:
        t0 = time.perf_counter()
        r = run_episode(lv, 0)
        print(lv, r.outcome, r.note, f"nav={r.navigate_time:.1f}", r.state_times, f"cps={r.cps_true}", f"minclr={r.min_clearance:.3f}",
              f"vio={r.vio_risk:.3f}/{r.vio_events}", f"fs={r.failsafe_blips}", f"wall={time.perf_counter() - t0:.1f}s")
