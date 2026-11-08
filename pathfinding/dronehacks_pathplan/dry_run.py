"""Closed-loop dry run, no ROS and no simulator: does the mission fly the course?

    python3 -m dronehacks_pathplan.dry_run                 # published course, simulated depth
    python3 -m dronehacks_pathplan.dry_run --no-depth      # published map only (the stock drone)
    python3 -m dronehacks_pathplan.dry_run --dropout 40 1.0 --carry 0.4 0.2 20

The real ``Mission`` and the real planner (``PlannerCore``) fly a simple drone model through a
model of the Sky Beta pipeline, with the pilot's steps scripted exactly as on the drone:

    pilot arms (Altitude) -> switches to Position -> flight_interface enters Offboard (only with a
    team setpoint in the last second, on the ground) -> mission flies -> LOW_HOLD -> pilot lands

What is modelled: the 0.3 m setpoint floor, auto-offboard's rules, PX4 position tracking
(P-gain + speed limit + lag, yaw-rate limit), PX4 land + auto-disarm, odometry dropouts, a
drone carried after SLAM started (``--carry``), a ray-cast depth camera on the published walls.
What is not: aerodynamics, cuVSLAM drift, real depth noise, ROS timing. It answers "is the
logic right"; the ROS pipeline is checked by ``selftest.launch.py``, flight by the simulator.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import json
from math import atan2, cos, hypot, pi, sin
import sys
from pathlib import Path

import numpy as np

if __package__ in (None, ""):                       # python3 dry_run.py from a checkout
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from dronehacks_pathplan.mission import Mission, MissionConfig, Telemetry, wrap  # noqa: E402

# FlyX lab depth camera (640x400), scaled down for speed
K_LAB = (421.016, 0.0, 319.25, 0.0, 421.016, 200.66, 0.0, 0.0, 1.0)


def render_depth(occ, spec, xy, z, yaw, K=K_LAB, W=640, H=400, cam_fwd=.115, cam_up=.029, wall_top=1.73):
    """Depth image (m along the optical axis, inf = no return) of the course walls (occupancy at
    flight height) and the floor, from a camera on a drone at course xy/z facing course yaw."""
    fx, cx, fy, cy = K[0], K[2], K[4], K[5]
    cam = np.asarray(xy, float) + cam_fwd * np.array([cos(yaw), sin(yaw)])
    ang = np.arctan((np.arange(W) - cx) / fx)                     # + = right of the heading
    dirs = np.stack([np.cos(yaw - ang), np.sin(yaw - ang)], 1)
    r = np.arange(.05, 4.0, .025)
    rc = spec.to_cell(cam[None, None] + r[None, :, None] * dirs[:, None])
    hit = occ[rc[..., 0], rc[..., 1]]
    wall = np.where(hit.any(1), r[np.argmax(hit, 1)], np.inf)
    slope = (np.arange(H) - cy)[:, None] / fy                      # + = below the horizon
    zc = z + cam_up
    floor = np.where(slope > 1e-6, zc / np.maximum(slope, 1e-6), np.inf)
    top_ok = zc - slope * wall[None] <= wall_top
    horiz = np.where(floor < wall[None], floor, np.where(top_ok, wall[None], np.inf))
    return horiz * np.cos(ang)[None]


@dataclass
class Vehicle:
    """PX4 + airframe, in the odom ENU frame. Starts where SLAM started: origin, facing +x."""
    pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    vel: np.ndarray = field(default_factory=lambda: np.zeros(3))
    yaw: float = 0.0
    armed: bool = False
    nav: int = 0                            # 0 MANUAL, 1 ALTCTL, 2 POSCTL, 14 OFFBOARD, 18 AUTO_LAND
    landed: bool = True
    sp: tuple | None = None                 # what flight_interface streams (odom xyz, yaw or None)
    landed_since: float | None = None
    max_yaw_rate: float = 0.0

    def step(self, dt, now, kp=.95, vmax=1.0, tau=.3, yaw_rate=.8):
        target_v = np.zeros(3)
        if self.armed and self.nav == 14 and self.sp is not None:
            err = np.asarray(self.sp[0]) - self.pos
            target_v = kp * err
            n = np.linalg.norm(target_v[:2])
            if n > vmax:
                target_v[:2] *= vmax / n
            target_v[2] = float(np.clip(target_v[2], -.7, 1.0))
            if self.sp[1] is not None:
                turn = wrap(self.sp[1] - self.yaw)
                d = float(np.clip(turn, -yaw_rate * dt, yaw_rate * dt))
                self.yaw = wrap(self.yaw + d)
                self.max_yaw_rate = max(self.max_yaw_rate, abs(d) / dt)
        elif self.armed and self.nav == 18:
            target_v = np.array([0.0, 0.0, -.4])
        self.vel += (target_v - self.vel) * min(1.0, dt / tau)
        self.pos += self.vel * dt
        if self.pos[2] <= 0.0:
            self.pos[2], self.vel[2] = 0.0, max(self.vel[2], 0.0)
        self.landed = bool(self.pos[2] < .02 and self.vel[2] <= .05)
        if self.armed and self.landed:
            self.landed_since = self.landed_since if self.landed_since is not None else now
            if self.nav == 18 and now - self.landed_since > 2.0:                 # COM_DISARM_LAND
                self.armed, self.nav = False, 2
            elif self.nav != 14 and self.nav != 18 and now - self.landed_since > 10.0:   # COM_DISARM_PRFLT
                self.armed = False
        else:
            self.landed_since = None


class FlightInterfaceModel:
    """skybeta_flight_interface: 0.3 m floor above the first odometry sample, auto-offboard rules."""

    def __init__(self, min_alt=.3):
        self.min_alt, self.origin = min_alt, None
        self.sp, self.sp_t = None, None
        self.seen_disarmed, self.attempted, self.armed_prev, self.nav_prev = False, False, False, 0

    def odometry(self, pos):
        if self.origin is None:
            self.origin = np.asarray(pos, float).copy()

    def setpoint(self, sp, now):
        oz = self.origin[2] if self.origin is not None else 0.0
        p = np.asarray(sp[0], float).copy()
        p[2] = max(p[2], oz + self.min_alt)
        self.sp, self.sp_t = (p, sp[1]), now

    def step(self, v: Vehicle, now):
        v.sp = self.sp
        if not v.armed:
            self.seen_disarmed, self.attempted = True, False
        switched_to_position = v.nav == 2 and self.nav_prev != 2
        if (v.armed and self.seen_disarmed and not self.attempted and switched_to_position and v.landed
                and self.sp_t is not None and now - self.sp_t < 1.0):
            self.attempted = True
            v.nav = 14
        self.nav_prev = v.nav

    def state(self, v: Vehicle):
        return {"px4_link": True, "armed": v.armed, "landed": v.landed, "nav_state": v.nav,
                "offboard_active": v.nav == 14, "local_valid": True, "frozen": False,
                "origin_z": None if self.origin is None else float(self.origin[2])}


def run(depth=True, planner="theta", dropout=None, carry=None, pilot_arm_s=6.0, takeover_s=None,
        max_s=260.0, dt=.05, seed=0, verbose=True):
    """Fly once. Returns a summary dict (also the assertions' input in tests/test_closed_loop.py)."""
    from dronehacks_pathplan.planner_core import InlinePlanner
    from pathplan.world import clearance
    planner_obj = InlinePlanner(planner_name=planner)
    core = planner_obj.core
    cfg = MissionConfig()
    mission = Mission(planner_obj, cfg)
    clear = clearance(core.prior, core.spec)
    v, fi = Vehicle(), FlightInterfaceModel(cfg.min_altitude_m)
    rng = np.random.default_rng(seed)
    from pathplan.ros_node import OdomFrame
    truth_frame = OdomFrame()                       # SLAM started on the launch mark, facing +y
    if carry is not None:                           # SLAM started elsewhere; carried to the mark
        cx, cy, cyaw = carry
        # odom origin is offset: the drone on the mark sits at odom (cx, cy) facing cyaw (deg)
        v.pos[:2], v.yaw = [cx, cy], np.radians(cyaw)
        truth_frame = OdomFrame.latched(v.pos, (0, 0, sin(v.yaw / 2), cos(v.yaw / 2)))
    t, last_odom, last_plan, last_depth = 0.0, -1.0, -1.0, -1.0
    tel = Telemetry(now=0.0)
    log = {"states": [], "path": [], "min_clearance": np.inf, "max_sp_yaw_rate": 0.0,
           "setpoints_while_not_ready": 0, "hold_requests": 0}
    pilot = {"armed_at": None, "position_at": None, "land_at": None}
    prev_sp_yaw, prev_t = None, None
    land_request_since = None
    W, H = 160, 100
    K = (K_LAB[0] / 4, 0, K_LAB[2] / 4, 0, K_LAB[4] / 4, K_LAB[5] / 4, 0, 0, 1)
    while t < max_s:
        # ---- the pilot
        if pilot["armed_at"] is None and t >= pilot_arm_s and mission.state == "READY":
            v.armed, v.nav, pilot["armed_at"] = True, 1, t               # arm in Altitude
        if pilot["armed_at"] is not None and pilot["position_at"] is None and t >= pilot["armed_at"] + 1.0:
            v.nav, pilot["position_at"] = 2, t                          # switch to Position
        if takeover_s is not None and pilot["position_at"] and t >= pilot["position_at"] + takeover_s and v.nav == 14:
            v.nav = 2                                                    # pilot takes it back
        # ---- PX4 + flight interface
        fi.step(v, t)
        v.step(dt, t)
        # ---- sensors
        in_dropout = dropout is not None and dropout[0] <= t - (pilot["position_at"] or 1e9) < dropout[0] + dropout[1]
        if not in_dropout and t - last_odom >= 1 / 30:
            fi.odometry(v.pos)
            last_odom = t
            tel.odom_t = t
            tel.position = tuple(v.pos + rng.normal(0, .003, 3))
            tel.q_xyzw = (0.0, 0.0, sin(v.yaw / 2), cos(v.yaw / 2))
        st = fi.state(v)
        tel.state_t = t
        for k, val in st.items():
            setattr(tel, k, val)
        tel.now, tel.flight_clock = t, t
        # ---- the mission (20 Hz)
        out = mission.tick(tel)
        if out.reset_planner:
            planner_obj.reset()
        if out.setpoint is not None:
            if mission.state in ("WAIT_HEALTH",):
                log["setpoints_while_not_ready"] += 1
            fi.setpoint(out.setpoint, t)
            if out.setpoint[1] is not None and mission.state in ("NAVIGATE", "PAD", "TAKEOFF", "DESCEND"):
                if prev_sp_yaw is not None:
                    log["max_sp_yaw_rate"] = max(log["max_sp_yaw_rate"], abs(wrap(out.setpoint[1] - prev_sp_yaw)) / (t - prev_t))
                prev_sp_yaw, prev_t = out.setpoint[1], t
        if out.hold_request:
            log["hold_requests"] += 1
        if not log["states"] or log["states"][-1] != mission.state:
            log["states"].append(mission.state)
            if verbose:
                note = mission.events[-1]["note"] if mission.events else ""
                print(f"  t={t:6.1f} s  {mission.state:12s} {note}")
        # the pilot lands when asked (after 2 s of LOW_HOLD), like on the drone
        if out.land_request:
            land_request_since = land_request_since if land_request_since is not None else t
            if t - land_request_since > 2.0 and v.nav == 14:
                v.nav, pilot["land_at"] = 18, t
        # after a takeover or a fault the pilot lands too (2 s later)
        if mission.state in ("TAKEN_OVER", "FAULT") and v.armed and v.nav != 18:
            pilot["takeover_at"] = pilot.get("takeover_at") or t
            if t - pilot["takeover_at"] > 2.0:
                v.nav, pilot["land_at"] = 18, t
        # ---- planning (5 Hz) and depth (5 Hz)
        if t - last_plan >= .2:
            last_plan = t
            if depth and mission.planning_allowed() and t - last_depth >= .2:
                last_depth = t
                xyz, yaw, rot = truth_frame.odometry_to_course(v.pos, (0, 0, sin(v.yaw / 2), cos(v.yaw / 2)))
                img = render_depth(core.prior, core.spec, xyz[:2], xyz[2], yaw, K, W, H)
                mm = np.where(np.isfinite(img) & (img < 10), img * 1000, 0).astype("<u2")
                xyz_m, _, rot_m = mission.pose_course
                planner_obj.give_frame((mm.tobytes(), W, H, "16UC1", W * 2, K, xyz_m, rot_m))
            mission.plan()
        # ---- record (truth, course frame)
        if v.armed and not v.landed:
            xyz, _, _ = truth_frame.odometry_to_course(v.pos, (0, 0, sin(v.yaw / 2), cos(v.yaw / 2)))
            r, c = core.spec.to_cell(xyz[:2])
            log["min_clearance"] = min(log["min_clearance"], float(clear[r, c]))
            if int(t / dt) % 10 == 0:
                log["path"].append([round(float(xyz[0]), 3), round(float(xyz[1]), 3), round(float(xyz[2]), 3)])
        if mission.state == "DONE":
            break
        t += dt
    xyz, _, _ = truth_frame.odometry_to_course(v.pos, (0, 0, sin(v.yaw / 2), cos(v.yaw / 2)))
    pad = np.asarray(planner_obj.pad_xy)
    log.update(final_state=mission.state, result=mission.result, t=round(t, 1),
               cleared=[c["name"] for c in planner_obj.cleared], landed_at=[round(float(xyz[0]), 3), round(float(xyz[1]), 3)],
               pad_error_m=round(float(hypot(*(xyz[:2] - pad))), 3), armed=v.armed,
               max_vehicle_yaw_rate=round(v.max_yaw_rate, 3), events=mission.events,
               flight_s=None if pilot["position_at"] is None else round((pilot["land_at"] or t) - pilot["position_at"], 1))
    log["min_clearance"] = round(float(log["min_clearance"]), 3)
    log["max_sp_yaw_rate"] = round(log["max_sp_yaw_rate"], 3)
    return log


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-depth", action="store_true", help="no depth camera (the stock drone)")
    ap.add_argument("--planner", default="theta", choices=["theta", "astar", "static"])
    ap.add_argument("--dropout", type=float, nargs=2, metavar=("AT_S", "FOR_S"),
                    help="odometry stops AT_S after the switch to Position, for FOR_S seconds")
    ap.add_argument("--carry", type=float, nargs=3, metavar=("X", "Y", "YAW_DEG"),
                    help="SLAM started elsewhere: on the mark the drone sits at odom X Y, heading YAW_DEG")
    ap.add_argument("--takeover", type=float, metavar="S", help="pilot takes over S seconds after Position")
    ap.add_argument("--json", type=Path, help="write the full log here")
    a = ap.parse_args(argv)
    print("dry run: pilot arms at 6 s, Position 1 s later; mission flies; pilot lands when asked")
    log = run(depth=not a.no_depth, planner=a.planner, dropout=a.dropout, carry=a.carry, takeover_s=a.takeover)
    ok = log["final_state"] == "DONE" and len(log["cleared"]) == 5 and log["pad_error_m"] < .15
    print("=" * 72)
    print(f"RESULT: {'PASS' if ok else 'CHECK'}  final {log['final_state']} - {log['result']}")
    print(f"checkpoints {len(log['cleared'])}/5: {', '.join(log['cleared'])}")
    print(f"flight (Position -> pilot lands) {log['flight_s']} s; landed {log['pad_error_m']} m from the pad centre")
    print(f"closest approach to a published wall (centre): {log['min_clearance']} m; "
          f"commanded heading rate <= {log['max_sp_yaw_rate']} rad/s")
    if a.json:
        a.json.write_text(json.dumps(log, indent=1, default=float))
        print(f"log: {a.json}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
