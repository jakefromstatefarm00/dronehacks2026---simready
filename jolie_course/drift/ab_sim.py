"""Offline A/B of Jolie's map alignment (VIO drift correction): one simulated flight.

Runs the *real* FlyX program logic of a source tree (``pathfinding/`` of a branch checkout):
``flyx.mission.Mission`` (state machine, carrot, yaw limiter, pad centring) and
``pathplan.planner_process.planner_process`` (depth projection, wall snap, votes, planner, and on
the drift branch ``align_to_map``), on a thread with an in-memory pipe instead of a forked process.
What ``fly_pathplan.FlyNode`` does around them (pose lookup at the frame stamp, turn-rate filter,
5 Hz planning, 20 Hz control, ``apply_alignment``) is re-implemented here line for line.

The world around them:
* PX4: a position P loop + first-order velocity lag (``px4_like``) flying the ESTIMATED pose to
  the setpoint, or Jolie's fake-PX4 kinematics (``fake``: straight at <= 1 m/s).
* Estimator (VIO) drift: the reported pose = true pose + e, with e a random walk in distance
  flown (bias ``drift`` m/m in a slowly wandering direction + white part), a heading error that
  grows ``head`` deg/m (motion integrates in a rotated frame), and optional jumps.
* Depth camera: ``fake_px4.render_depth`` (640x400, FlyX intrinsics, 13 Hz) ray-cast from the TRUE
  pose on the TRUE (perturbed) course; optional stereo noise, dropout, outliers, mount error.
* Evaluation on TRUE geometry: contacts (clearance < 0.28 m, Sky Beta radius), min clearance,
  checkpoints crossed by the true track, true landing error.

Common random numbers: the variant, drift-vs-distance function and noise streams depend only on
the seed, never on the arm.
"""
from __future__ import annotations

import copy
import math
import os
import queue
import sys
import threading
import time
import types
from dataclasses import dataclass, asdict, field

import numpy as np

# ------------------------------------------------------------------ FlyX constants
K = (421.016, 0.0, 319.25, 0.0, 421.016, 200.66, 0.0, 0.0, 1.0)     # FlyX lab CameraInfo
W, H = 640, 400
CONTACT_R = .28
DEBUG = os.environ.get("AB_DEBUG") == "1"


@dataclass
class Case:
    arm: str = "base"              # base | drift | safe
    seed: int = 0
    drift: float = 0.0             # m per m flown (bias)
    head: float = 0.0              # deg per m flown (heading drift, random sign)
    jumps: float = 0.0             # expected VIO jumps per 10 m flown (each ~N(0, 0.08 m) per axis)
    variant: str = "nominal"       # nominal | walls | walls_big | clutter | mixed
    noise: str = "clean"           # clean | noisy | bad
    mount_dx: float = 0.0          # TRUE camera offset error vs the configured (.115, 0, .029), m
    mount_dy: float = 0.0
    mount_yaw_deg: float = 0.0     # TRUE camera yaw error
    pose_lag_s: float = 0.0        # frame pose taken this much too early (stamp mismatch in turns)
    dynamics: str = "px4_like"     # px4_like | fake
    # alignment parameters (drift arm: Jolie's; safe arm: overridden)
    gain: float = .6
    step_m: float = .04
    cap_m: float = 1.5
    safe: dict = field(default_factory=dict)


# ------------------------------------------------------------------ source tree import
_TREE = None


def use_tree(path):
    """Import pathplan/flyx from ``<path>`` (a branch's pathfinding/ directory). Once per process."""
    global _TREE
    path = os.path.abspath(path)
    if _TREE is not None:
        if _TREE != path:
            raise RuntimeError(f"process already uses {_TREE}")
        return
    sys.path.insert(0, path)
    _TREE = path
    import pathplan.planner_process as pp
    pp.os = types.SimpleNamespace(nice=lambda n: None)          # not on Windows / not needed
    pp.signal = types.SimpleNamespace(signal=lambda *a: None, SIGINT=2, SIG_IGN=None)
    import pathplan.depth_projection as dp
    dp.np = _FastUniqueNumpy()                                # identical results, O(N)
    dp._orig_align = getattr(dp, "align_to_map", None)       # None on the base branch


class _FastUniqueNumpy:
    """numpy, except ``unique(cells, axis=0)`` of (K, 2) non-negative grid cells is done with a
    mask instead of a sort: the same sorted unique rows, ~50x faster (project_depth's hot spot)."""

    def __getattr__(self, name):
        return getattr(np, name)

    @staticmethod
    def unique(a, *args, **kw):
        if (not args and kw == {"axis": 0} and isinstance(a, np.ndarray) and a.ndim == 2 and a.shape[1] == 2
                and a.dtype.kind in "iu" and len(a) and a.min() >= 0 and a.max() < 1024):
            mask = np.zeros((int(a[:, 0].max()) + 1, int(a[:, 1].max()) + 1), bool)
            mask[a[:, 0], a[:, 1]] = True
            return np.argwhere(mask).astype(a.dtype)
        return np.unique(a, *args, **kw)


_CACHE = {}


def _published():
    if "pub" not in _CACHE:
        from pathplan.harness import START_XY, load_published, static_route
        from pathplan.world import Envelope, GridSpec, checkpoints_from_route, cost_grid, occupancy
        from pathplan.ros_node import build_planner
        course, route = load_published()
        spec, env = GridSpec(), Envelope()
        slab = env.half_height + env.tracking_margin
        cps = checkpoints_from_route(route, course, chicane="window")
        prior = occupancy(course, spec, altitude=.9, half_height=slab)
        nominal = static_route(cost_grid(prior, spec, env), spec, env, cps, START_XY)
        keep = [(START_XY, .8), (np.asarray(route.centre[route.names.index("pad_hover")][:2]), .8)]
        keep += [(cp.centre, .45) for cp in cps]
        built = build_planner("theta", .9)
        _CACHE["pub"] = dict(course=course, route=route, spec=spec, env=env, cps=cps, prior=prior,
                             nominal=nominal, keep=keep, built=built)
    return _CACHE["pub"]


def make_truth(variant, seed):
    """(render occupancy, true clearance, info) of the course this seed flies."""
    key = ("truth", variant, seed)
    if key in _CACHE:
        return _CACHE[key]
    from pathplan.variants import perturb
    from pathplan.world import clearance, occupancy, cost_grid
    from pathplan.harness import START_XY, static_route
    P = _published()
    rng = np.random.default_rng(10_000 + seed)
    course, info = P["course"], {"kind": "nominal", "changes": []}
    if variant != "nominal":
        kind = {"walls_big": "walls"}.get(variant, variant)
        kw = {"wall_shift": .30} if variant == "walls_big" else {}
        for _ in range(30):
            c, info = perturb(P["course"], kind, rng, route_xy=P["nominal"], keep_clear=P["keep"], **kw)
            t = occupancy(c, P["spec"], altitude=.9, half_height=P["env"].half_height + P["env"].tracking_margin)
            if static_route(cost_grid(t, P["spec"], P["env"]), P["spec"], P["env"], P["cps"], START_XY) is not None:
                course = c
                break
    render = occupancy(course, P["spec"], altitude=.9, half_height=.05)
    true_clear = clearance(occupancy(course, P["spec"], altitude=.9, half_height=.13), P["spec"])
    out = (render, true_clear, info)
    for k in [k for k in _CACHE if k != "pub"][:-20]:
        del _CACHE[k]
    _CACHE[key] = out
    return out


# ------------------------------------------------------------------ depth rendering (fake_px4)
def render_depth(occ, spec, cam_xy, yaw, z, rng=None, noise="clean"):
    """``fake_px4.render_depth`` with the camera position given directly (course frame, true pose),
    plus optional stereo noise. Returns uint16 mm."""
    fx, cx, fy, cy = K[0], K[2], K[4], K[5]
    ang = np.arctan((np.arange(W) - cx) / fx)
    dirs = np.stack([np.cos(yaw - ang), np.sin(yaw - ang)], 1)
    r = np.arange(.05, 4.0, .025)
    rc = spec.to_cell(np.asarray(cam_xy)[None, None] + r[None, :, None] * dirs[:, None])
    hit = occ[rc[..., 0], rc[..., 1]]
    wall = np.where(hit.any(1), r[np.argmax(hit, 1)], np.inf)
    slope = (np.arange(H) - cy)[:, None] / fy
    zc = z + .029
    floor = np.where(slope > 1e-6, zc / np.maximum(slope, 1e-6), np.inf)
    top_ok = zc - slope * wall[None] <= 1.73
    horiz = np.where(floor < wall[None], floor, np.where(top_ok, wall[None], np.inf))
    depth = horiz * np.cos(ang)[None]
    if noise != "clean" and rng is not None:
        lvl = {"noisy": (.01, .015, .05, .005), "bad": (.02, .03, .15, .02)}[noise]
        s0, k2, drop, outl = lvl
        fin = np.isfinite(depth)
        # stereo error grows with z^2; correlated over 8x8 blocks (the projection samples stride 8)
        blk = rng.normal(0, 1, (H // 8 + 1, W // 8 + 1)).repeat(8, 0).repeat(8, 1)[:H, :W]
        depth = depth + np.where(fin, (s0 + k2 * np.where(fin, depth, 0) ** 2) * blk, 0)
        depth[rng.random((H, W)) < drop] = 0
        o = rng.random((H, W)) < outl
        depth[o] = rng.uniform(.4, 3.5, o.sum())
    return np.where(np.isfinite(depth) & (depth > 0) & (depth < 10), depth * 1000, 0).astype("<u2")


# ------------------------------------------------------------------ planner process on a thread
class _Pipe:
    def __init__(self, inbox, outbox):
        self.inbox, self.outbox = inbox, outbox

    def send(self, x):
        self.outbox.put(x)

    def recv(self):
        return self.inbox.get()


class ThreadPlanner:
    """``PlannerProcess`` with the same planner_process body, on a thread."""

    def __init__(self, config, align_override=None):
        import pathplan.planner_process as pp
        import pathplan.depth_projection as dp
        a, b = queue.Queue(), queue.Queue()
        self.conn, child = _Pipe(b, a), _Pipe(a, b)
        built = _published()["built"]
        self._built = built
        if dp._orig_align is not None:
            dp.align_to_map = align_override or dp._orig_align
        def body():
            try:
                pp.planner_process(child, lambda name, alt: copy.deepcopy(built))
            except Exception as e:                         # never leave the sim waiting on the pipe
                import traceback
                child.send(("error", traceback.format_exc()))

        self.thread = threading.Thread(target=body, daemon=True)
        self.thread.start()
        self.events, self.cleared, self.ms, self.depth_quality = [], [], 0.0, {}
        self.align = None
        self.frame = None
        self.conn.send(tuple(config))
        ready, self.pad_xy = self.conn.recv()

    def give_frame(self, frame):
        self.frame = frame

    def update(self, xy, yaw):
        frame, self.frame = self.frame, None
        if frame is not None and callable(frame[0]):
            frame = (frame[0](),) + tuple(frame[1:])
        self.conn.send((xy, yaw, frame))
        out = self.conn.recv()
        if out[0] == "error":
            raise RuntimeError("planner thread: " + out[1])
        cmd, self.events, self.cleared, self.ms, self.depth_quality = out[:5]
        self.align = out[5] if len(out) > 5 else None
        return cmd

    def close(self):
        self.conn.send(None)
        self.thread.join(2)


# ------------------------------------------------------------------ drift model
class Drift:
    """Estimator error as a function of distance flown s (common random numbers across arms)."""

    def __init__(self, case, seed):
        rng = np.random.default_rng(20_000 + seed)
        self.ds = .05
        n = 6000                                    # 300 m of path
        theta = rng.uniform(0, 2 * np.pi) + np.cumsum(rng.normal(0, .25 * math.sqrt(self.ds), n))
        bias = case.drift * self.ds * np.column_stack([np.cos(theta), np.sin(theta)])
        white = rng.normal(0, .3 * case.drift * math.sqrt(self.ds), (n, 2))
        jumps = np.zeros((n, 2))
        if case.jumps > 0:
            hit = rng.random(n) < case.jumps / 10 * self.ds
            jumps[hit] = rng.normal(0, .08, (hit.sum(), 2))
        self.inc = bias + white + jumps             # per ds, in the course frame
        self.head_sign = rng.choice([-1.0, 1.0])
        self.head_rate = math.radians(case.head) * self.head_sign
        self.s, self.k = 0.0, 0

    def advance(self, ds):
        """Error increment (course xy) and heading increment for ``ds`` more metres."""
        self.s += ds
        k1 = min(int(self.s / self.ds), len(self.inc) - 1)
        d = self.inc[self.k:k1].sum(0) if k1 > self.k else np.zeros(2)
        self.k = k1
        return d, self.head_rate * ds


def _rot(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


# ------------------------------------------------------------------ one flight
def run_case(case: Case, record_track=False):
    from flyx.mission import Mission, MissionConfig, Telemetry
    from flyx.px4_link import odom_to_course, yaw_from_q_ned
    from pathplan.planner_process import MAX_TURN_RATE
    P = _published()
    spec, cps = P["spec"], P["cps"]
    render_occ, true_clear, vinfo = make_truth(case.variant, case.seed)

    override = None
    if case.arm == "safe":
        from align_safe import make_safe_align
        override = make_safe_align(case.safe)
    planner = ThreadPlanner(("theta", .9, (.115, 0.0, .029), 0.0, .5), override)
    mission = Mission(planner, MissionConfig(execute=True, altitude=.9, carrot_m=.5, launch_xy=(-2.515, .914),
                                             launch_heading=1.5707963, launch_body_agl=.05,
                                             pad_xy=tuple(planner.pad_xy)))
    drift = Drift(case, case.seed)
    nrng = np.random.default_rng(30_000 + case.seed)

    # state. Course frame 0 (the launch frame) = NED rotated; true pose in course, estimate in NED.
    launch = np.array([-2.515, .914])
    M2 = np.array([[0., 1.], [1., 0.]])           # NED xy -> course xy (heading 0 on the launch mark)
    true_xy, true_z, true_yaw = launch.copy(), .05, math.pi / 2
    est_ned = np.zeros(3)                         # PX4 estimate (NED); z down, launch at 0
    yaw_ned = 0.0                                 # estimated NED yaw
    e_xy = np.zeros(2)                            # estimate - truth, course frame 0
    dpsi = 0.0                                    # estimated - true course yaw
    v = np.zeros(3)
    arming, nav, landed = 1, 4, True
    sp = None
    tel = Telemetry(now=0.0)
    dt = .02
    t = 0.0
    yaw_hist = []                                 # (t, est NED yaw)
    pose_hist = []                                # (t, est_ned, yaw_ned)
    frame_pending = None
    align_total, align_steps, align_capped = np.zeros(2), 0, False
    pending_align = []                            # (apply at t, shift)
    recent = []                                   # safe arm: last measured shifts
    m = dict(contact_steps=0, contact_events=0, min_clear=9.0, nav_t0=None, land_t=None, max_err=0.0,
             max_align=0.0, err_samples=[], frames=0, turning=0, aligns=0, track=[])
    in_contact = False
    next_plan, next_depth, next_ctrl = 0.0, 0.0, 0.0
    end_t = float(os.environ.get("AB_END", "200"))
    true_track = [true_xy.copy()]
    sim_planner_ms = []
    gate_info = {"capped": False}

    def q_of(y):
        return (math.cos(y / 2), 0.0, 0.0, math.sin(y / 2))

    while t < end_t:
        if DEBUG and int(round(t / dt)) % 250 == 0:
            import psutil
            print(f"t={t:6.1f} state={mission.state} plan={mission.planner_status} true={true_xy.round(2)} "
                  f"err={np.linalg.norm(e_xy):.2f} align={align_total.round(3)} cont={m['contact_steps']} "
                  f"rss={psutil.Process().memory_info().rss / 2**20:.0f}MB", flush=True)
        # ---------------- telemetry (50 Hz)
        tel.now = tel.sim_now = t
        tel.odom_t, tel.position_ned, tel.q_ned_frd = t, tuple(est_ned), q_of(yaw_ned)
        tel.status_t = tel.landed_t = tel.local_t = t
        tel.arming_state, tel.nav_state, tel.landed = arming, nav, landed
        tel.xy_valid = tel.z_valid = True
        tel.user_intention = nav
        pose_hist.append((t, est_ned.copy(), yaw_ned))
        if len(pose_hist) > 400:
            del pose_hist[:100]

        # ---------------- deferred alignment (planning thread finishes ~0.1 s after it started)
        while pending_align and pending_align[0][0] <= t:
            _, shift = pending_align.pop(0)
            if mission.frame is None:
                continue
            nmed = int(case.safe.get("median", 0)) if case.arm == "safe" else 0
            if nmed > 1:                      # SAFE: median of the last n measured shifts
                recent.append(np.asarray(shift, float))
                del recent[:-nmed]
                if len(recent) < nmed:
                    continue
                shift = np.median(np.asarray(recent), axis=0)
            step = case.gain * np.asarray(shift, float)
            n = float(np.linalg.norm(step))
            if n > case.step_m:
                step *= case.step_m / n
            total = align_total + step
            if np.linalg.norm(total) > case.cap_m:
                gate_info["capped"] = True
                continue
            mission.frame.origin_in_course = mission.frame.origin_in_course + np.array([step[0], step[1], 0.0])
            align_total, align_steps = total, align_steps + 1

        # ---------------- depth camera (13 Hz): fly_pathplan.on_depth
        if t >= next_depth:
            next_depth += 1 / 13
            if mission.frame is not None and mission.planning_allowed():
                m["frames"] += 1
                # pose at the stamp (pose_lag_s: a stale pose is matched)
                ts = t - case.pose_lag_s
                tp, pn, py = min(pose_hist, key=lambda p: abs(p[0] - ts))
                t0, _, py0 = min(pose_hist, key=lambda p: abs(p[0] - (ts - .15)))
                dyaw = py - py0
                rate = abs(math.atan2(math.sin(dyaw), math.cos(dyaw))) / max(tp - t0, .05)
                xyz, _, rotation = odom_to_course(mission.frame, pn, q_of(py))
                if rate > MAX_TURN_RATE:
                    m["turning"] += 1
                else:
                    cyaw = true_yaw + math.radians(case.mount_yaw_deg)
                    off = np.array([.115 + case.mount_dx, case.mount_dy])
                    cam = true_xy + _rot(true_yaw) @ off
                    # rendered only if the planner takes this frame (it keeps the latest one)
                    render = (lambda cam=cam, cyaw=cyaw, z=true_z:
                              render_depth(render_occ, spec, cam, cyaw, z, nrng, case.noise).tobytes())
                    planner.give_frame((render, W, H, "16UC1", W * 2, list(K), xyz, rotation))

        # ---------------- planning (5 Hz)
        if t >= next_plan:
            next_plan += .2
            cmd = mission.plan()
            if planner.align is not None and mission.frame is not None and case.arm != "base":
                m["aligns"] += 1
                pending_align.append((t + .1, planner.align))

        # ---------------- control (20 Hz)
        if t >= next_ctrl:
            next_ctrl += .05
            out = mission.tick(tel)
            if out.setpoint is not None:
                sp = (np.asarray(out.setpoint[0], float), float(out.setpoint[1]))
            if out.command is not None:
                c = out.command.command
                if c == 176:
                    nav = 14
                elif c == 400 and nav == 14:
                    arming, landed = 2, False
                elif c == 21:
                    nav = 18
            if mission.state == "NAVIGATE" and m["nav_t0"] is None:
                m["nav_t0"] = t
            if mission.state == "LAND" and m["land_t"] is None:
                m["land_t"] = t
                m["land_note"] = mission.events[-1]["note"] if mission.events else ""
            if mission.state in ("DONE", "FAILED", "TAKEN_OVER"):
                break

        # ---------------- PX4 + physics (50 Hz)
        if arming == 2 and nav == 14 and sp is not None:
            err = sp[0] - est_ned
            if case.dynamics == "fake":
                n = np.linalg.norm(err)
                vcmd = err / dt if n < dt else err / n * 1.0
                v = vcmd
            else:
                vref = 1.0 * err
                vref[2] = 1.0 * err[2]
                sxy = np.linalg.norm(vref[:2])
                if sxy > 1.5:
                    vref[:2] *= 1.5 / sxy
                a = (vref - v) / .3
                an = np.linalg.norm(a)
                if an > 3.0:
                    a *= 3.0 / an
                v = v + a * dt
            turn = math.atan2(math.sin(sp[1] - yaw_ned), math.cos(sp[1] - yaw_ned))
            yrate = max(-1.0, min(1.0, (turn / dt) if case.dynamics == "fake" else 2.5 * turn))
            yaw_ned += yrate * dt
        elif nav == 18:
            v = np.array([0.0, 0.0, .5])
            if est_ned[2] >= 0.0:
                v[:] = 0
                if arming == 2:
                    arming, landed = 1, True
        else:
            v = np.zeros(3)
        d_est_ned = v * dt
        # the estimate moves by d_est (in the rotated estimator frame); the truth by R(-dpsi) d_est
        d_est_c = M2 @ d_est_ned[:2]
        d_true = _rot(-dpsi) @ d_est_c
        true_xy = true_xy + d_true
        est_ned = est_ned + d_est_ned
        true_z = .05 - est_ned[2]
        ds = float(np.linalg.norm(d_true))
        if ds > 0 and -est_ned[2] > .3:
            de, dh = drift.advance(ds)
            # drift: the estimate wanders by de (course frame) without the vehicle moving
            est_ned[:2] = est_ned[:2] + M2.T @ de
            e_xy = e_xy + de
            dpsi += dh
        e_xy = (launch + M2 @ est_ned[:2]) - true_xy
        true_yaw = (math.pi / 2 - yaw_ned) - dpsi
        t += dt

        # ---------------- evaluation on TRUE geometry (airborne)
        if true_z > .5 and arming == 2:
            rc = spec.to_cell(true_xy)
            c = float(true_clear[rc[0], rc[1]])
            m["min_clear"] = min(m["min_clear"], c)
            if c < CONTACT_R:
                m["contact_steps"] += 1
                if not in_contact:
                    m["contact_events"] += 1
                in_contact = True
            else:
                in_contact = False
            if mission.frame is not None and mission.state == "NAVIGATE":
                est_c = mission.frame.to_course(est_ned)[:2]
                resid = float(np.linalg.norm(est_c - true_xy))
                m["max_err"] = max(m["max_err"], resid)
                m["err_samples"].append(resid)
                m["max_align"] = max(m["max_align"], float(np.linalg.norm(align_total)))
                if record_track and int(round(t / dt)) % 25 == 0:
                    m["track"].append([round(t, 2), *true_xy.round(3).tolist(), *est_c.round(3).tolist(),
                                       *align_total.round(3).tolist(), round(c, 3)])
        if len(true_track) == 0 or np.linalg.norm(true_xy - true_track[-1]) > .02:
            true_track.append(true_xy.copy())

    planner.close()
    # ---------------- true checkpoint crossing (in order) and landing error
    trk = np.asarray(true_track)
    k, crossed = 0, 0
    for cp in cps:
        if cp.kind == "pad":
            break
        hit = False
        while k < len(trk) - 1:
            if cp.reached(trk[k], trk[k + 1], CONTACT_R):
                hit = True
                k += 1
                break
            k += 1
        if not hit:
            break
        crossed += 1
    pad = cps[-1].centre
    pad_err = float(np.linalg.norm(true_xy - pad))
    n_line = sum(cp.kind != "pad" for cp in cps)
    landed_ok = mission.state == "DONE" and str(m.get("land_note", "")).startswith("landing pad reached")
    complete = landed_ok and crossed == n_line and pad_err <= .762 - CONTACT_R
    est_err_end = float(np.linalg.norm(mission.frame.to_course(est_ned)[:2] - true_xy)) if mission.frame else 0.0
    es = m["err_samples"]
    res = dict(
        arm=case.arm, seed=case.seed, drift=case.drift, head=case.head, jumps=case.jumps,
        variant=case.variant, noise=case.noise, mount_dx=case.mount_dx, mount_dy=case.mount_dy,
        mount_yaw_deg=case.mount_yaw_deg, pose_lag_s=case.pose_lag_s, dynamics=case.dynamics,
        state=mission.state, land_note=m.get("land_note", ""), complete=bool(complete),
        checkpoints_true=crossed, checkpoints_planner=len(planner.cleared),
        contact_steps=m["contact_steps"], contact_events=m["contact_events"],
        min_clear=round(m["min_clear"], 3), pad_err=round(pad_err, 3),
        course_s=round((m["land_t"] or t) - (m["nav_t0"] or 0.0), 2) if m["nav_t0"] is not None else None,
        drift_end=round(float(np.linalg.norm(e_xy)), 3), head_end_deg=round(math.degrees(dpsi), 2),
        align_total=round(float(np.linalg.norm(align_total)), 3), align_xy=[round(float(x), 3) for x in align_total],
        align_steps=align_steps, align_capped=gate_info["capped"], max_align=round(m["max_align"], 3),
        resid_end=round(est_err_end, 3), resid_max=round(m["max_err"], 3),
        resid_rms=round(float(np.sqrt(np.mean(np.square(es)))) if es else 0.0, 3),
        frames=m["frames"], turning_frames=m["turning"], flown_m=round(drift.s, 1),
        vinfo=vinfo.get("kind"))
    if record_track:
        res["track"] = trk.round(3).tolist()
        res["log"] = m["track"]          # t, true xy, estimated xy, correction xy, true clearance
        res["vchanges"] = vinfo.get("changes")
    return res


if __name__ == "__main__":
    import argparse
    import json
    ap = argparse.ArgumentParser()
    ap.add_argument("tree")
    ap.add_argument("--arm", default="base")
    ap.add_argument("--drift", type=float, default=0.0)
    ap.add_argument("--variant", default="nominal")
    ap.add_argument("--noise", default="clean")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--dynamics", default="px4_like")
    ap.add_argument("--track", default="", help="save the run with its track/log to this JSON")
    a = ap.parse_args()
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    use_tree(a.tree)
    t0 = time.time()
    r = run_case(Case(arm=a.arm, drift=a.drift, variant=a.variant, noise=a.noise, seed=a.seed, dynamics=a.dynamics),
                 record_track=bool(a.track))
    if a.track:
        json.dump(r, open(a.track, "w"))
        r = {k: v for k, v in r.items() if k not in ("track", "log")}
    r["wall_s"] = round(time.time() - t0, 1)
    print(json.dumps(r))
