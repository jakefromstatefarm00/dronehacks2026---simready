"""Mission state machine for the FlyX lab (pure Python, no ROS): fed telemetry, returns
what to publish.  The ROS wrapper (``fly_pathplan.py``) only moves messages in and out.

    WAIT_HEALTH --(fresh pose, valid estimator, landed, for healthy_for_s)--> READY
    READY --shadow--> SHADOW          plans and reports; never publishes to PX4
    READY --execute--> STREAM          hold setpoints for stream_before_offboard_s (PX4 needs them)
    STREAM --> WAIT_OFFBOARD           request Offboard; wait for nav_state == OFFBOARD
    WAIT_OFFBOARD --> WAIT_ARM         request arm; wait for arming_state == ARMED
    WAIT_ARM --> TAKEOFF               climb straight up to ``altitude`` (course frame, m)
    TAKEOFF --> NAVIGATE               OnlinePlanner through the checkpoints, carrot setpoints
    NAVIGATE --(pad reached)--> LAND   PX4 native land; then wait for landed + disarmed
    LAND --> DONE

Airborne guards (TAKEOFF / NAVIGATE): stale pose, invalid estimator, flight time over
``max_flight_s`` or a planner failure -> LAND.  If PX4 leaves Offboard by itself (pilot or
failsafe) -> TAKEN_OVER: stop publishing anything and leave the vehicle to PX4.
Refused Offboard or arm -> FAILED before anything moved.  All times are simulation seconds.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import math

import numpy as np

from .px4_link import (ARMING_STATE_ARMED, ARMING_STATE_DISARMED, NAV_STATE_AUTO_LAND, NAV_STATE_OFFBOARD, POSE_FRAME_NED, Command,
                       course_to_ned_setpoint, odom_to_course, yaw_from_q_ned)


@dataclass
class Telemetry:
    """Latest PX4 reports. ``now`` and the ``*_t`` arrival stamps (None = never seen) are on a
    monotonic clock: sim time and WSL wall time both jump (~1 s steps), which made fresh odometry
    look stale and landed the first FlyX run. ``sim_now`` (sim time) is used only for the flight limit."""
    now: float
    sim_now: float | None = None
    odom_t: float | None = None
    position_ned: tuple = (0.0, 0.0, 0.0)
    q_ned_frd: tuple = (1.0, 0.0, 0.0, 0.0)
    pose_frame: int = POSE_FRAME_NED
    status_t: float | None = None
    arming_state: int = ARMING_STATE_DISARMED
    nav_state: int = 0
    landed_t: float | None = None
    landed: bool = False
    local_t: float | None = None
    xy_valid: bool = False
    z_valid: bool = False
    failsafe: bool = False                 # vehicle_status: PX4 failsafe active
    user_intention: int = 0                # vehicle_status.nav_state_user_intention
    foreign: str = ""                      # other programs publishing PX4 setpoints ('' = none)


@dataclass
class Output:
    state: str
    setpoint: tuple | None = None          # (position NED (3,), yaw NED) -> stream with an Offboard heartbeat
    command: Command | None = None         # send once
    note: str = ""


@dataclass
class MissionConfig:
    execute: bool = False
    altitude: float = .9                   # course frame, m above the floor (below the 1.73 m walls)
    carrot_m: float = .5                   # setpoint this far along the plan (position setpoints are not speed-capped)
    takeoff_tolerance_m: float = .10
    healthy_for_s: float = 2.0
    stream_before_offboard_s: float = 1.0
    ack_timeout_s: float = 3.0
    max_flight_s: float = 175.0            # rule 7.9: 3-minute run limit, with margin
    # Freshness. On the ground (before Offboard/arm) everything must be fresh. In the air the lab's
    # topics pause (Suraj's FlyX journals: odometry 0.7 s, local position / land-detected 2.4-3.3 s,
    # program stalls up to 1.8 s) and landing on every pause ended runs 3 and 4. So in the air:
    # odometry older than pose_max_age_s HOLDS the last setpoint (PX4 keeps flying on its own
    # estimate); only odometry_outage_s lands. Slow topics land after slow_lost_s of silence; a
    # fresh "estimator invalid" still lands at once.
    pose_max_age_s: float = .3
    # in the air, a stale-odometry *backstop*: PX4 flies on its own estimate meanwhile and a real
    # estimator failure still lands at once via xy/z_valid. 3 s landed a fake-PX4 run mid-course on a
    # 1.8 s freeze + 1.2 s odometry gap; FlyX freezes reach ~1.95 s plus ~0.7 s odometry pauses.
    odometry_outage_s: float = 5.0
    status_max_age_s: float = 3.0          # on the ground; vehicle_status ~1.7 Hz in FlyX
    local_max_age_s: float = 2.0           # on the ground; only its xy/z validity flags are used
    slow_lost_s: float = 10.0              # in the air: status / local position silence that lands
    pad_centre_m: float = .08              # at the pad: centre over it to within this before Land
    pad_centre_timeout_s: float = 8.0
    pad_xy: tuple | None = None            # pad centre, course frame (from the planner)
    offboard_loss_s: float = .5
    # PX4 drops to its failsafe (Land) when our setpoints stop > COM_OF_LOSS_T (1 s) and returns to
    # Offboard by itself once they resume (FlyX run 7: 8 such blips during 1-2 s program stalls).
    # While PX4 is in failsafe but still wants Offboard, keep streaming for up to this long.
    failsafe_resume_s: float = 5.0
    launch_xy: tuple = (-2.515, .914)      # launch mark, course frame
    launch_heading: float = float(np.pi / 2)   # vehicle heading on the mark, course frame (facing +y)
    launch_body_agl: float = .05           # body origin above the floor when sitting on the mark
    resend_s: float = 1.0                  # repeat Offboard / arm / Land until PX4 shows it (lossy link)
    takeoff_timeout_s: float = 10.0        # then accept being within takeoff_loose_m of the altitude
    takeoff_loose_m: float = .25
    plan_above_m: float = .5               # no mapping/planning below this (depth from the floor)
    plan_in_takeoff: bool = False          # [v5] False: map and plan only from NAVIGATE (at altitude); v4c True
    # Visual odometry dies on fast turns: Suraj's flyx_course data had 30 of 40 VIO losses above
    # 0.35 rad/s, and 3/3 Gazebo completions at 0.25; our FlyX run 6 lost xy after turning in place.
    yaw_rate_max: float = .25              # rad/s cap on the commanded heading's rate of change
    slow_turn_deg: float = 90.0            # carrot shrinks to 30 % as the heading error reaches 70 % of this
    # ---- speed-gated turning ("turns"; all off by default = v3 exactly). Gazebo: every VIO divergence
    # began at the first corridor-to-maze corner yawing 0.46-0.8 rad/s WHILE moving 0.4-0.5 m/s; the
    # base's 14 successes peaked at yaw rate x speed 0.105-0.127, the failure 0.130. So turn faster only
    # when slow: cap = min(ceiling, yaw_speed_max / measured horizontal speed), see gated_yaw_cap().
    turn_gate: bool = False
    yaw_speed_max: float = .10             # rad m/s^2: hard bound on commanded yaw rate x measured speed
    yaw_rate_slow_max: float = .50         # rad/s ceiling while moving slowly (nearly stopped)
    yaw_rate_in_place_max: float = .35     # rad/s ceiling turning in place (stop + turn: Gazebo v131 VIO loss)
    speed_unknown_m_s: float = .60         # speed assumed when it cannot be measured (cap = 0.10 / 0.6)
    # the speed the clamp uses is max(measured, predicted): PX4 flies at ~MPC_XY_P (0.95 default) x the
    # setpoint distance, so a long carrot means the drone is about to speed up (measured speed lags)
    speed_per_setpoint_m: float = .95
    # bound the heading setpoint's lead over the measured heading to cap x yaw_lead_s: PX4's yaw loop
    # catches up a lead at its own rate, so an unbounded backlog would keep turning fast after the
    # cap dropped (offline: true rate 0.63 rad/s and yaw x speed 0.14 with a 0.5 rad/s cap, no bound)
    yaw_lead_s: float = .5
    # FlyX v3+view failure at the tunnel approach (escape crawl + look-around spin near walls, then VIO
    # lost): while the camera view (now or at the commanded heading) is shorter than short_view_m,
    # never turn faster than yaw_rate_short_view (sim-time rad/s; v3 flies 0.25 wall = ~0.31 sim in FlyX)
    short_view_m: float = 1.0
    yaw_rate_short_view: float = .25
    near_wall_m: float = .55               # ... and while any believed obstacle is closer than this (FlyX: the
                                           # in-place look-around beside the tube walls lost VIO)
    # exploratory look when slow at a corner: overshoot the route heading toward the turn's side by
    # this much (0 = off), once per corner, while slower than explore_speed_m_s
    explore_look_deg: float = 0.0
    explore_speed_m_s: float = .15
    explore_min_turn_deg: float = 25.0     # only when the route heading is this far from the nose
    explore_hold_s: float = 1.5            # give up the look after this long (lab wall s)
    explore_rearm_m: float = .6            # next look only after moving this far
    # cornerfix "view" port (first-corner-vio): the planner child may lead the heading up to 40 deg into
    # a sharp turn toward a far view (PlannerProcess option corner_view) and tags "corner...-slow" when
    # the camera faces the map closer than 1.5 m; such commands fly a shorter carrot (1.0 = off)
    corner_carrot_scale: float = 1.0
    corner_carrot_min_m: float = .12


def gated_yaw_cap(speed, in_place, cfg, moving_speed=None):
    """Commanded heading-rate cap (rad/s). Deterministic, outside any learned code: with
    cfg.turn_gate it never lets cap x speed exceed cfg.yaw_speed_max, nor the absolute ceilings;
    without it, cfg.yaw_rate_max (v3). ``speed`` (the conservative speed for the bound) None =
    unknown (assume cfg.speed_unknown_m_s). ``moving_speed`` (the measured speed, default
    ``speed``) only chooses the ceiling: nearly stopped or turning in place -> the moderate in-place
    ceiling (a stop followed by a turn in place lost VIO in Gazebo v131), rising linearly to the
    slow-moving ceiling at 0.15 m/s. It can only lower the cap."""
    if not cfg.turn_gate:
        return cfg.yaw_rate_max
    v = cfg.speed_unknown_m_s if speed is None or not math.isfinite(speed) else max(0.0, float(speed))
    m = v if moving_speed is None or not math.isfinite(moving_speed) else max(0.0, float(moving_speed))
    lo, hi = cfg.yaw_rate_in_place_max, cfg.yaw_rate_slow_max
    ceiling = lo if in_place else lo + (hi - lo) * min(1.0, max(0.0, (min(m, v) - .05) / .10))
    return float(min(ceiling, cfg.yaw_speed_max / max(v, 1e-6)))


class Mission:
    def __init__(self, planner, cfg: MissionConfig | None = None):
        self.planner = planner
        self.cfg = cfg or MissionConfig()
        self.state = "WAIT_HEALTH"
        self.frame = None
        self.events = []
        self.target = None                 # latest (course xy, course yaw) from plan()
        self.planner_status = "active"
        self._healthy_since = None
        self._t_state = None
        self._takeoff_xy = None
        self._t_takeoff = None
        self._sim_takeoff = None
        self._land_sent = False
        self._last_command_t = None
        self._turn_xy = None
        self._sp_yaw = None                # last commanded NED yaw (rate limiter state)
        self._last_sp = None               # last NAVIGATE setpoint (held while odometry pauses)
        self._pad_since = None
        self.px4_failsafe_waits = 0        # control ticks spent waiting for PX4 to return to Offboard
        self._sp_yaw_t = None
        self._offboard_lost_since = None
        self.pose_course = None            # (xyz, yaw, rotation) for depth projection
        self._speed_hist = deque(maxlen=20)   # (odometry arrival t, course xy) for the measured speed
        self.speed = None                  # measured horizontal speed, m/s (None = unknown)
        self.yaw_cap = self.cfg.yaw_rate_max    # last applied heading-rate cap
        self.turn_stats = {"look": 0, "cap_max": 0.0, "yv_cmd_max": 0.0}
        self._look = None                  # (target yaw, start t) of an exploratory look
        self._look_from = None             # xy where the last look started
        self._now = None
        self.yaw_rate_cmd = 0.0            # last commanded heading rate, rad/s
        self.speed_for_cap = None          # speed the last cap was computed from (max of measured, predicted)
        self.rtf = 1.0                     # sim seconds per wall second, measured (clipped to [0.5, 1])
        self.view_m = None                 # turns build: how far the camera sees (planner belief map)
        self.wall_m = None                 # turns build: distance to the nearest believed obstacle
        self._clock_hist = deque(maxlen=200)

    # ---------------------------------------------------------------- helpers
    def _go(self, state, now, note=""):
        self.events.append({"t": round(now, 3), "from": self.state, "to": state, "note": note})
        self.state, self._t_state = state, now

    def _fresh(self, t, now, max_age):
        # A report stamped slightly after ``now`` is fresher, not stale: callbacks on other
        # threads can stamp arrivals between the control loop reading the clock and this check.
        return t is not None and now - t <= max_age

    def _unhealthy_reason(self, tel, airborne=False):
        """'' when healthy, else which check failed (logged with the LAND transition)."""
        c, now = self.cfg, tel.now
        age = lambda t: "never" if t is None else f"{now - t:.2f}s old"
        if not self._fresh(tel.odom_t, now, c.odometry_outage_s if airborne else c.pose_max_age_s):
            return f"odometry stale ({age(tel.odom_t)})"
        if tel.foreign:
            return "another program sends PX4 setpoints: " + tel.foreign
        if tel.pose_frame != POSE_FRAME_NED:
            return f"odometry frame {tel.pose_frame} is not NED"
        if not self._fresh(tel.status_t, now, c.slow_lost_s if airborne else c.status_max_age_s):
            return f"vehicle_status stale ({age(tel.status_t)})"
        if not self._fresh(tel.local_t, now, c.slow_lost_s if airborne else c.local_max_age_s):
            return f"local_position stale ({age(tel.local_t)})"
        if not (tel.xy_valid and tel.z_valid):
            return f"estimator invalid (xy_valid={tel.xy_valid}, z_valid={tel.z_valid})"
        return ""

    def _measure_speed(self, tel):
        """Horizontal speed from odometry positions (any contract: PX4 odometry or cuVSLAM pose):
        the larger of the last ~0.15 s and ~0.3 s averages (the conservative one for the clamp).
        None when odometry is stale or there is not enough history."""
        h = self._speed_hist
        if not h or tel.odom_t > h[-1][0]:
            h.append((tel.odom_t, np.asarray(self.pose_course[0][:2], float).copy()))
        if tel.now - tel.odom_t > self.cfg.pose_max_age_s:
            self.speed = None
            return
        t1, p1 = h[-1]
        best = None
        for window in (.15, .3):
            old = [e for e in h if t1 - e[0] >= window]
            if old and t1 - old[-1][0] <= window + .35:
                t0, p0 = old[-1]
                v = float(np.linalg.norm(p1 - p0)) / (t1 - t0)
                best = v if best is None else max(best, v)
        self.speed = best

    def _update_rtf(self, tel):
        """Sim seconds per wall second over the last >= 3 s of ticks, clipped to [0.5, 1.0] (1.0 when
        sim time is unknown; 0.5 = conservative while not yet measured or when sim time jumps)."""
        if tel.sim_now is None:
            self.rtf = 1.0
            return
        h = self._clock_hist
        h.append((tel.now, tel.sim_now))
        w1, s1 = h[-1]
        old = [e for e in h if w1 - e[0] >= 3.0]
        if not old:                         # not measured yet: assume the slowest (conservative)
            self.rtf = .5
            return
        w0, s0 = old[-1]
        r = (s1 - s0) / (w1 - w0)
        self.rtf = float(np.clip(r, .5, 1.0)) if math.isfinite(r) and 0.0 < r < 2.0 else .5

    def _healthy(self, tel):
        return not self._unhealthy_reason(tel)

    def _hold_here(self, tel):
        return tuple(tel.position_ned), yaw_from_q_ned(tel.q_ned_frd)

    def _course_setpoint(self, xy, yaw):
        return course_to_ned_setpoint(self.frame, xy, self.cfg.altitude, yaw)

    # ---------------------------------------------------------------- planning (worker thread)
    def plan(self):
        """Advance the planner from the latest pose; cheap to call, may take ~0.1-0.5 s."""
        from pathplan.ros_node import target_from_command
        if not self.planning_allowed():
            return None
        xyz, yaw, _ = self.pose_course
        cmd = self.planner.update(xyz[:2], yaw)
        self.target = target_from_command(cmd, xyz[:2], yaw, self._carrot_for(cmd, yaw))
        self.target = self._explore(cmd, xyz, yaw, self.target)
        if cmd.mode == "turn":
            # hold the spot where the turning began: re-holding "here" every step let the vehicle
            # drift 11 cm during a look-around in FlyX run 6, toward a wall
            if self._turn_xy is None:
                self._turn_xy = np.asarray(xyz[:2], float).copy()
            self.target = (self._turn_xy, self.target[1])
        else:
            self._turn_xy = None
        self.view_m = getattr(cmd, "view_m", None)       # turns build: camera view on the belief map, m
        self.wall_m = getattr(cmd, "wall_m", None)       # turns build: nearest believed obstacle, m
        self.planner_status = cmd.mode
        return cmd

    def _explore(self, cmd, xyz, yaw, target):
        """Exploratory look (cfg.explore_look_deg > 0): slow, and the route turns away from the
        nose -> face explore_look_deg PAST the route heading, toward the side the route turns to
        (the new passage and its side openings), once per corner. The carrot still follows the
        route; it stays short because the nose is off the route (_carrot_for), so the drone stays
        slow during the look. Released once reached, after explore_hold_s, when faster than
        explore_speed_m_s, or when the planner stops moving. The yaw rate is still gated_yaw_cap."""
        c = self.cfg
        if c.explore_look_deg <= 0 or self.state != "NAVIGATE" or cmd.mode != "move" or cmd.yaw is None:
            self._look = None
            return target
        now = self._now if self._now is not None else 0.0
        xy = np.asarray(xyz[:2], float)
        fast = self.speed is None or self.speed / self.rtf > c.explore_speed_m_s
        if self._look is not None:
            look_yaw, t0 = self._look
            if abs(_wrap(look_yaw - yaw)) < np.deg2rad(5) or now - t0 > c.explore_hold_s or fast:
                self._look = None
                return target
            return (target[0], look_yaw)
        off = _wrap(cmd.yaw - yaw)
        rearmed = self._look_from is None or np.linalg.norm(xy - self._look_from) >= c.explore_rearm_m
        if not fast and rearmed and abs(off) >= np.deg2rad(c.explore_min_turn_deg):
            look_yaw = _wrap(cmd.yaw + np.sign(off) * np.deg2rad(c.explore_look_deg))
            self._look, self._look_from = (look_yaw, now), xy.copy()
            self.turn_stats["look"] += 1
            return (target[0], look_yaw)
        return target

    def _carrot_for(self, cmd, yaw):
        """Setpoint distance: shorter while the nose is still turning toward the route, so the
        drone slows through corners instead of swinging wide (0.5 m aligned, 0.15 m at 60 deg)."""
        c = self.cfg
        if cmd.mode == "move" and cmd.reason == "back out":
            return min(c.carrot_m, .25)                 # turns build: back out slowly, heading held
        if cmd.mode != "move" or cmd.yaw is None:
            return c.carrot_m
        off = abs(float(np.arctan2(np.sin(cmd.yaw - yaw), np.cos(cmd.yaw - yaw))))
        carrot = c.carrot_m * float(np.clip(1.0 - off / np.deg2rad(c.slow_turn_deg), .3, 1.0))
        if c.corner_carrot_scale < 1.0 and str(cmd.reason).startswith("corner") and str(cmd.reason).endswith("-slow"):
            carrot = max(c.corner_carrot_min_m, c.corner_carrot_scale * carrot)     # cornerfix port
        return carrot

    def planning_allowed(self):
        """Map and plan only in flight (or in shadow): on the ground the camera sits 5 cm up and
        each "look around" call spent turning budget the vehicle never used."""
        if self.pose_course is None:
            return False
        if self.state in ("NAVIGATE", "SHADOW"):
            return True
        # [v5 plan_in_takeoff=False] Gazebo v909 (v4c): depth taken while climbing in TAKEOFF made 7
        # phantom cells in the first checkpoint's goal pocket ("no goal cell"), so the drone forgot its map
        # and looked around in place at the launch mark right after takeoff - FlyX run 11's opening too.
        return self.cfg.plan_in_takeoff and self.state == "TAKEOFF" and self.pose_course[0][2] > self.cfg.plan_above_m

    def _resend(self, now):
        """True once every resend_s: VehicleCommand goes over a best-effort link and may be lost."""
        if self._last_command_t is None or now - self._last_command_t >= self.cfg.resend_s:
            self._last_command_t = now
            return True
        return False

    def observe(self, free, hit, behind):
        if self.frame is not None:
            self.planner.observe(free, hit, behind)

    # ---------------------------------------------------------------- control (fast loop)
    def tick(self, tel: Telemetry) -> Output:
        c, now = self.cfg, tel.now
        if self.frame is not None and tel.odom_t is not None:
            self.pose_course = odom_to_course(self.frame, tel.position_ned, tel.q_ned_frd)
            self._measure_speed(tel)
        self._now = now
        self._update_rtf(tel)
        healthy = self._healthy(tel)

        if self.state == "WAIT_HEALTH":
            if not (healthy and tel.landed and self._fresh(tel.landed_t, now, c.status_max_age_s)
                    and tel.arming_state == ARMING_STATE_DISARMED):
                self._healthy_since = None
                return Output(self.state, note="waiting for fresh pose, valid estimator, landed + disarmed")
            self._healthy_since = self._healthy_since if self._healthy_since is not None else now
            if now - self._healthy_since < c.healthy_for_s:
                return Output(self.state, note="healthy, confirming")
            from flyx.px4_link import CourseFrame
            self.frame = CourseFrame.from_launch(
                position_ned=list(tel.position_ned), heading_ned=yaw_from_q_ned(tel.q_ned_frd),
                launch_xy=list(c.launch_xy), body_agl=c.launch_body_agl,
                physical_heading_in_course=c.launch_heading, placement_reviewed=True)
            self.pose_course = odom_to_course(self.frame, tel.position_ned, tel.q_ned_frd)
            self._go("READY", now, "course frame from the launch pose")

        if self.state == "READY":
            self._go("STREAM" if c.execute else "SHADOW", now)

        if self.state == "SHADOW":
            return Output(self.state, note=f"shadow: planner {self.planner_status}, nothing sent to PX4")

        if self.state == "STREAM":
            if not healthy:
                self._go("FAILED", now, "unhealthy before Offboard: " + self._unhealthy_reason(tel))
                return Output(self.state)
            if now - self._t_state >= c.stream_before_offboard_s:
                self._go("WAIT_OFFBOARD", now)
                self._last_command_t = now
                return Output(self.state, self._hold_here(tel), Command.offboard(), "requesting Offboard")
            return Output(self.state, self._hold_here(tel), note="streaming hold setpoints")

        if self.state == "WAIT_OFFBOARD":
            if tel.nav_state == NAV_STATE_OFFBOARD:
                self._go("WAIT_ARM", now)
                self._last_command_t = now
                return Output(self.state, self._hold_here(tel), Command.arm(), "Offboard active, requesting arm")
            if now - self._t_state > c.ack_timeout_s:
                self._go("FAILED", now, "PX4 did not enter Offboard")
                return Output(self.state)
            return Output(self.state, self._hold_here(tel), Command.offboard() if self._resend(now) else None)

        if self.state == "WAIT_ARM":
            if tel.arming_state == ARMING_STATE_ARMED:
                self._takeoff_xy = self.pose_course[0][:2].copy()
                self._t_takeoff = now
                self._sim_takeoff = tel.sim_now
                self._go("TAKEOFF", now)
            elif now - self._t_state > c.ack_timeout_s:
                self._go("FAILED", now, "PX4 refused to arm")
                return Output(self.state)
            else:
                return Output(self.state, self._hold_here(tel), Command.arm() if self._resend(now) else None)

        if self.state in ("TAKEOFF", "NAVIGATE"):
            guard = self._airborne_guard(tel, healthy)
            if guard:
                return guard
            if self.state == "TAKEOFF":
                yaw = self.pose_course[1]
                sp = self._course_setpoint(self._takeoff_xy, yaw)
                off = abs(self.pose_course[0][2] - c.altitude)
                if off < c.takeoff_tolerance_m:
                    self._go("NAVIGATE", now, "at altitude")
                elif now - self._t_state > c.takeoff_timeout_s and off < c.takeoff_loose_m:
                    self._go("NAVIGATE", now, f"near altitude after {c.takeoff_timeout_s:.0f} s ({off:.2f} m off)")
                return Output(self.state, sp, note="takeoff")
            if not self._fresh(tel.odom_t, now, c.pose_max_age_s) and self._last_sp is not None:
                return Output(self.state, self._last_sp, note="odometry paused: holding")
            if self.planner_status == "done":
                return self._centre_and_land(tel)
            if self.planner_status == "failed":
                return self._land(now, "planner failed: " + (self.planner.events[-1]["event"] if self.planner.events else ""))
            if self.target is None:
                sp = self._limit_yaw(self._course_setpoint(self.pose_course[0][:2], self.pose_course[1]), tel)
            else:
                sp = self._limit_yaw(self._course_setpoint(*self.target), tel)
            self._last_sp = sp
            return Output(self.state, sp, note=f"planner {self.planner_status}")

        if self.state == "LAND":
            if tel.landed and tel.arming_state == ARMING_STATE_DISARMED:
                self._go("DONE", now, "landed and disarmed")
                return Output(self.state)
            if tel.nav_state != NAV_STATE_AUTO_LAND and not tel.landed and self._resend(now):
                return Output(self.state, command=Command.land(), note="repeating Land")
            return Output(self.state)

        return Output(self.state)

    def _failsafe_hold(self):
        """What to stream while PX4 sits in a failsafe it will leave once setpoints flow again."""
        if self._last_sp is not None:
            return self._last_sp
        if self.state == "TAKEOFF" and self._takeoff_xy is not None:
            return self._course_setpoint(self._takeoff_xy, self.pose_course[1])
        return self._course_setpoint(self.pose_course[0][:2], self.pose_course[1])

    def _centre_and_land(self, tel):
        """Pad reached (centre within the planner's pad radius): fly to the pad centre, then Land.
        Landing where "reached" fired put the drone 0.25 m off centre (Suraj's Gazebo v139)."""
        c, now = self.cfg, tel.now
        if c.pad_xy is None:
            return self._land(now, "landing pad reached")
        self._pad_since = self._pad_since if self._pad_since is not None else now
        off = float(np.hypot(*(np.asarray(self.pose_course[0][:2]) - np.asarray(c.pad_xy))))
        if off <= c.pad_centre_m or now - self._pad_since > c.pad_centre_timeout_s:
            return self._land(now, f"landing pad reached ({off:.2f} m from its centre)")
        sp = self._limit_yaw(self._course_setpoint(c.pad_xy, self.pose_course[1]), tel)
        self._last_sp = sp
        return Output(self.state, sp, note=f"centring over the pad ({off:.2f} m)")

    def short_view_cap(self):
        """Turns build: yaw_rate_short_view while the planner reports a short camera view, else inf."""
        c = self.cfg
        if c.turn_gate and ((self.view_m is not None and self.view_m < c.short_view_m)
                            or (self.wall_m is not None and self.wall_m < c.near_wall_m)):
            return c.yaw_rate_short_view
        return float("inf")

    def _limit_yaw(self, setpoint, tel):
        """Rate-limit the commanded heading to yaw_rate_max (starts from the vehicle's yaw)."""
        position, yaw = setpoint
        now = tel.now
        if self._sp_yaw is None:
            self._sp_yaw, self._sp_yaw_t = yaw_from_q_ned(tel.q_ned_frd), now
        c = self.cfg
        dt = max(0.0, min(now - self._sp_yaw_t, .2))
        if c.turn_gate:
            # Physical units: the program's clock is the lab WALL clock, but the airframe moves in sim
            # time (FlyX ~0.8x, GB10 ~0.83x real time), so a wall-clock rate limit yaws 1/rtf faster in
            # sim time and wall-clock speed reads rtf slower. Convert both; rtf = 1 on the drone.
            rtf = self.rtf
            measured = c.speed_unknown_m_s if self.speed is None else self.speed / rtf
            dist = float(np.hypot(position[0] - tel.position_ned[0], position[1] - tel.position_ned[1]))
            speed = max(measured, c.speed_per_setpoint_m * dist)     # PX4 P loop: about to fly this fast
            cap = gated_yaw_cap(speed, self.planner_status == "turn", c, measured)    # rad per SIM second
            cap = min(cap, self.short_view_cap())                     # near walls: no faster than v3
            step = cap * rtf * dt
        else:
            rtf, speed = 1.0, self.speed
            cap = gated_yaw_cap(speed, self.planner_status == "turn", c)
            step = cap * dt
        self.speed_for_cap = speed
        self.yaw_cap = cap
        err = float(np.arctan2(np.sin(yaw - self._sp_yaw), np.cos(yaw - self._sp_yaw)))
        turned = float(np.clip(err, -step, step))
        if c.turn_gate and turned != 0.0:
            # never grow the setpoint's lead over the vehicle's heading beyond cap x yaw_lead_s
            psi = yaw_from_q_ned(tel.q_ned_frd)
            lead = float(np.arctan2(np.sin(self._sp_yaw + turned - psi), np.cos(self._sp_yaw + turned - psi)))
            if abs(lead) > cap * c.yaw_lead_s and np.sign(lead) == np.sign(turned):
                turned = 0.0
        self.yaw_rate_cmd = abs(turned) / (dt * rtf) if dt > 0 else 0.0     # commanded heading rate (sim s)
        st = self.turn_stats
        st["cap_max"] = max(st["cap_max"], cap)
        if speed is not None:
            st["yv_cmd_max"] = max(st["yv_cmd_max"], self.yaw_rate_cmd * speed)
        new = self._sp_yaw + turned
        self._sp_yaw, self._sp_yaw_t = float(np.arctan2(np.sin(new), np.cos(new))), now
        return position, self._sp_yaw

    def _airborne_guard(self, tel, healthy):
        c, now = self.cfg, tel.now
        waiting_for_px4 = False
        if tel.nav_state != NAV_STATE_OFFBOARD:
            self._offboard_lost_since = self._offboard_lost_since if self._offboard_lost_since is not None else now
            # PX4's own failsafe while it still wants Offboard: it comes back once setpoints flow again
            px4_will_return = tel.failsafe and tel.user_intention == NAV_STATE_OFFBOARD
            if now - self._offboard_lost_since > (c.failsafe_resume_s if px4_will_return else c.offboard_loss_s):
                why = ("PX4 failsafe that did not clear" if px4_will_return else "PX4 failsafe" if tel.failsafe else
                       "mode changed by someone else" if tel.user_intention != NAV_STATE_OFFBOARD else "unknown")
                self._go("TAKEN_OVER", now, f"PX4 left Offboard: {why} (nav_state {tel.nav_state}, "
                                            f"requested {tel.user_intention}, failsafe {tel.failsafe}); not commanding")
                return Output(self.state)
            waiting_for_px4 = px4_will_return
        else:
            self._offboard_lost_since = None
        problem = self._unhealthy_reason(tel, airborne=True)
        if problem:
            return self._land(now, "unhealthy: " + problem)
        if waiting_for_px4:
            self.px4_failsafe_waits += 1
            return Output(self.state, self._failsafe_hold(),
                          note="PX4 failsafe while it wants Offboard: streaming until it returns")
        if self._sim_takeoff is not None and tel.sim_now is not None:
            flown = tel.sim_now - self._sim_takeoff               # the run limit is in sim time
        else:
            flown = now - self._t_takeoff
        if flown > c.max_flight_s:
            return self._land(now, "flight time limit")
        return None

    def _land(self, now, why):
        self._go("LAND", now, why)
        self._last_command_t = now
        if not self._land_sent:
            self._land_sent = True
            return Output(self.state, command=Command.land(), note=why)
        return Output(self.state)


def _wrap(a):
    return float(np.arctan2(np.sin(a), np.cos(a)))
