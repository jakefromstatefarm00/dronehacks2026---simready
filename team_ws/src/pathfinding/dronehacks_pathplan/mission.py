"""The Sky Beta mission state machine (pure Python, no ROS): fed telemetry, returns what to send.

This is ``flyx/mission.py`` (the FlyX single-file program's state machine) moved onto the
organisers' contract. The flight logic is the same -- takeoff, Theta* replanning with depth,
carrot setpoints, the 0.25 rad/s heading limit, odometry-pause holds, pad centring, the run
limit -- but everything that talked to PX4 is gone:

    FlyX program (native PX4)               this node (Sky Beta)
    ----------------------------------      -------------------------------------------------
    sent Offboard + arm itself              WAITS: the pilot arms (Altitude), switches to
                                            Position; flight_interface enters Offboard
    /fmu/in/trajectory_setpoint (NED)       /skybeta/setpoint/pose (ENU, odom)
    /fmu/out/* for health                   /skybeta/state + /visual_slam/tracking/odometry
    PX4 Land at the pad / on failure        descend to flight_interface's floor (0.3 m) and
                                            HOLD there; the pilot lands (there is no land
                                            command in Sky Beta, by design)
    land on lost sensors                    /skybeta/hold (freeze) and stop; the pilot takes over

States
------
    WAIT_HEALTH  fresh odometry + /skybeta/state, PX4 link, valid PX4 position, landed, disarmed,
                 no other setpoint publisher -- for healthy_for_s. Publishes NOTHING, so
                 flight_interface cannot enter Offboard while this node is not ready
                 (its auto-offboard needs a team setpoint from the last second).
    READY        streams "hold here" setpoints. This stream is the go-ahead flight_interface
                 needs. The course frame follows the drone while it sits on the ground.
    ARMED        the pilot armed: the course frame is LATCHED here (the drone is on the launch
                 mark, facing up the first corridor). Still "hold here", waiting for Offboard.
    TAKEOFF      Offboard + armed: climb (ramped, climb_rate) to ``altitude``
    NAVIGATE     planner through the checkpoints, carrot setpoints, heading rate-limited
    PAD          checkpoint 5 reached: centre over the pad
    DESCEND      ramp down to the floor (``floor_z``: flight_interface's min altitude)
    LOW_HOLD     hold low (over the pad, or where the mission stopped); ``land_request`` is set:
                 THE PILOT LANDS NOW (PX4 disarms itself after touchdown)
    SHADOW       (shadow:=true) maps and plans from the latched frame; never publishes setpoints
    TAKEN_OVER   PX4 left Offboard (pilot or failsafe) mid-mission: stop commanding. PX4's own
                 failsafe Hold (AUTO_LOITER, COM_FAIL_ACT_T) is ridden out for failsafe_resume_s first
    FAULT        lost odometry / PX4 position / link in the air: ask /skybeta/hold, stop commanding.
                 The hold stays in force (flight_interface then refuses Offboard) until this node's
                 next healthy READY, whose first setpoint clears it; a hold someone else asked for
                 is never cleared by this node
    DONE         disarmed after a flight; back to WAIT_HEALTH after rearm_after_s (next flight)

All ``*_t`` arrival stamps and ``now`` are on a monotonic clock (sim time and WSL wall time
both jump; that made fresh data look stale and landed FlyX run 1). ``flight_clock`` (the ROS
clock, sim time in a simulator) is used only for the run limit, when it is running.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from math import atan2, cos, hypot, sin

import numpy as np

AIRBORNE = ("TAKEOFF", "NAVIGATE", "PAD", "DESCEND", "LOW_HOLD")
FLIGHT_STATES = AIRBORNE + ("ARMED", "TAKEN_OVER", "FAULT")
NAV_STATE_AUTO_LOITER = 4                   # PX4's failsafe Hold (and the Hold mode)
NAV_STATE_NAMES = {0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 4: "AUTO_LOITER", 5: "AUTO_RTL", 12: "DESCEND",
                   14: "OFFBOARD", 15: "STAB", 18: "AUTO_LAND"}


def wrap(a):
    return float(atan2(sin(a), cos(a)))


@dataclass
class Telemetry:
    now: float                              # monotonic seconds
    flight_clock: float | None = None       # ROS clock seconds (sim time in a sim); None = not running
    # /visual_slam/tracking/odometry (ENU, odom)
    odom_t: float | None = None
    position: tuple = (0.0, 0.0, 0.0)
    q_xyzw: tuple = (0.0, 0.0, 0.0, 1.0)
    # /skybeta/state
    state_t: float | None = None
    px4_link: bool = False
    armed: bool = False
    landed: bool | None = None
    nav_state: int = -1
    offboard_active: bool = False
    local_valid: bool = False               # local_enu not null: PX4 has a valid position
    frozen: bool = False
    origin_z: float | None = None           # origin_enu z: flight_interface's altitude reference
    # graph checks
    foreign: str = ""                       # other publishers on our setpoint topics / extra interfaces


@dataclass
class Output:
    state: str
    setpoint: tuple | None = None           # (odom ENU xyz, odom yaw or None = do not command yaw)
    note: str = ""
    hold_request: bool = False              # call /skybeta/hold once
    land_request: bool = False              # the pilot should land now (LOW_HOLD)
    reset_planner: bool = False             # a new flight: fresh planner


@dataclass
class MissionConfig:
    shadow: bool = False
    altitude: float = .9                    # course z (above the floor) to fly at; walls are 1.73 m
    carrot_m: float = .5
    # the carrot shrinks (to 30 % at slow_turn_deg) while the nose is still turning toward the route,
    # so the drone slows through corners instead of swinging wide (from the FlyX-proven upload_to_flyx.py)
    slow_turn_deg: float = 60.0
    slow_turn_min_frac: float = .3
    launch_xy: tuple = (-2.515, .914)       # launch mark, course frame
    launch_heading: float = float(np.pi / 2)  # facing up the first corridor (course +y)
    launch_height: float = .05              # body origin above the floor on the mark
    frame_latch: str = "arm"                # "arm": latch at arming | "slam_origin": SLAM started on the mark
    min_altitude_m: float = .3              # MUST equal flight_interface's min_altitude_m
    floor_margin_m: float = .03             # aim this far above the floor (no clamp warnings)
    healthy_for_s: float = 2.0
    pose_max_age_s: float = .3              # odometry this old: on the ground unhealthy, in the air hold
    # READY keeps streaming "hold here" through odometry pauses up to this long (one 0.3 s pause used
    # to drop the node to WAIT_HEALTH and stop the stream that flight_interface's Offboard needs)
    ready_odom_grace_s: float = 1.5
    # armed while not READY and our last setpoint is this recent (flight_interface accepts a setpoint
    # up to 1 s old): ask /skybeta/hold so it refuses Offboard; the pilot must disarm
    hold_if_armed_within_s: float = 1.5
    odometry_outage_s: float = 3.0          # in the air: odometry this old -> FAULT
    state_max_age_s: float = 2.0            # /skybeta/state is 5 Hz
    state_lost_s: float = 5.0               # in the air: no /skybeta/state this long -> FAULT
    offboard_loss_s: float = .5             # PX4 out of Offboard (pilot) this long -> TAKEN_OVER
    # PX4 v1.16 answers a stalled setpoint stream (COM_OF_LOSS_T) with its failsafe Hold (AUTO_LOITER)
    # for up to COM_FAIL_ACT_T (5 s) and returns to Offboard by itself once the stream is back (FlyX
    # run 7: 8 such blips, all recovered). While PX4 is in that Hold, keep streaming this long.
    failsafe_resume_s: float = 5.0
    # flight_interface reports px4_link false after 2 s without vehicle_status; the lab's slow topics
    # pause 2.4-3.3 s. In the air: hold the last setpoint, FAULT only after this long (the flown
    # program tolerated 10 s)
    px4_link_lost_s: float = 8.0
    climb_rate: float = .4                  # m/s, takeoff ramp
    descend_rate: float = .3                # m/s, final descent ramp
    takeoff_tolerance_m: float = .10
    takeoff_timeout_s: float = 12.0         # then accept within takeoff_loose_m
    takeoff_loose_m: float = .25
    pad_centre_m: float = .08               # centre over the pad to within this before descending
    pad_centre_timeout_s: float = 8.0
    descend_tolerance_m: float = .08
    # PX4 flies on EKF2's height, the mission sees cuVSLAM's (0.15 m apart in FlyX): once the ramp is
    # at the floor setpoint this long, LOW_HOLD (the pilot's land cue) even if odometry z disagrees
    floor_settle_s: float = 3.0
    # ground: odometry z this far from flight_interface's origin_enu z (its floor reference, the first
    # odometry sample) means the stack started somewhere other than the floor: not READY (0 = off)
    max_origin_offset_m: float = .1
    max_flight_s: float = 175.0             # rule 7.9: 3-minute run limit, with margin (every airborne state)
    plan_above_m: float = .5                # no mapping/planning below this (depth sees the floor)
    yaw_rate_max: float = .25               # rad/s: VIO loses tracking on fast turns (30/40 losses > 0.35)
    rearm_after_s: float = 3.0              # DONE -> WAIT_HEALTH (the next flight)


class Mission:
    """``tick(tel)`` at ~20 Hz (control thread); ``plan()`` at ~5 Hz (planning thread)."""

    def __init__(self, planner, cfg: MissionConfig | None = None):
        self.planner = planner
        self.cfg = cfg or MissionConfig()
        self.state = "WAIT_HEALTH"
        self.events = []
        self.frame = None                   # pathplan.ros_node.OdomFrame, odom <-> course
        self.pose_course = None             # (xyz, yaw, rotation)
        self.target = None                  # (course xy, course yaw) from plan()
        self.planner_status = "idle"
        self.result = ""                    # why the last flight ended (for the summary)
        self._reset_flight()
        self._healthy_since = None
        self._skip_arming = False           # armed while we were not READY: not ours to fly
        self._sp_out_t = None               # when tick() last returned a setpoint (the node publishes it)
        self._own_hold_t = None             # when this node last asked for /skybeta/hold (None: not ours)
        self._prev_now = None
        self._dt = 0.0
        self.px4_failsafe_waits = 0         # ticks spent streaming through PX4 failsafe Holds / link gaps

    # ---------------------------------------------------------------- helpers
    def _reset_flight(self):
        self._t_state = None
        self._takeoff = None                # (course xy, start z, t)
        self._t_airborne = None             # (monotonic, flight_clock) at takeoff
        self._descend = None                # (course xy, start z, t, why)
        self._sp_yaw = None
        self._sp_yaw_t = None
        self._last_sp = None
        self._pad_since = None
        self._offboard_lost_since = None
        self._link_lost_since = None
        self._floor_since = None
        self._turn_xy = None
        self._hold_sent = False
        self.target = None
        self.planner_status = "idle"

    def _go(self, state, now, note=""):
        self.events.append({"t": round(now, 3), "from": self.state, "to": state, "note": note})
        self.state, self._t_state = state, now

    @staticmethod
    def _fresh(t, now, max_age):
        return t is not None and now - t <= max_age

    def _ground_problem(self, tel, odom_max_age=None):
        """'' when ready to fly, else the first failed check (shown in ~/status)."""
        c, now = self.cfg, tel.now
        age = lambda t: "never" if t is None else f"{now - t:.2f} s old"
        if not self._fresh(tel.odom_t, now, c.pose_max_age_s if odom_max_age is None else odom_max_age):
            return f"no fresh odometry ({age(tel.odom_t)}) - is cuVSLAM tracking?"
        if not self._fresh(tel.state_t, now, c.state_max_age_s):
            return f"no fresh /skybeta/state ({age(tel.state_t)}) - is flight_interface running?"
        if tel.foreign:
            return "another program is commanding: " + tel.foreign
        if not tel.px4_link:
            return "flight_interface has no PX4 link (vehicle_status not arriving)"
        if not tel.local_valid:
            return "PX4 has no valid local position (vision not reaching EKF2?)"
        if not tel.landed:                  # None (no land detector report yet) counts as not landed
            return "PX4 does not report landed"
        if tel.armed:
            return "armed before this node was ready - disarm first"
        if tel.frozen and self._own_hold_t is None:
            return "/skybeta/hold is in force (not ours: whoever asked for it must clear it)"
        if c.max_origin_offset_m > 0 and tel.origin_z is not None:
            # flight_interface's 0.3 m floor is measured from its origin (the first odometry sample).
            # Started on a box / in a hand, that floor is wrong for a drone now sitting on the mark.
            dz = float(tel.position[2]) - float(tel.origin_z)
            if abs(dz) > c.max_origin_offset_m:
                return (f"odometry z is {dz:+.2f} m from flight_interface's origin (where the stack started): "
                        "its floor would be wrong - restart the stack with the drone on the floor")
        return ""

    def _air_problem(self, tel):
        c, now = self.cfg, tel.now
        if not self._fresh(tel.odom_t, now, c.odometry_outage_s):
            return f"odometry lost for {c.odometry_outage_s:.0f} s"
        if not self._fresh(tel.state_t, now, c.state_lost_s):
            return f"/skybeta/state silent for {c.state_lost_s:.0f} s"
        if not tel.local_valid:
            return "PX4 lost its local position"
        if tel.foreign:
            return "another program is commanding: " + tel.foreign
        return ""

    def _make_frame(self, tel):
        from pathplan.ros_node import OdomFrame
        c = self.cfg
        if c.frame_latch == "slam_origin":
            return OdomFrame(tuple(c.launch_xy), float(c.launch_heading), float(c.launch_height))
        return OdomFrame.latched(tel.position, tel.q_xyzw, c.launch_xy, c.launch_heading, c.launch_height)

    def _update_pose(self, tel):
        if self.frame is not None and tel.odom_t is not None:
            self.pose_course = self.frame.odometry_to_course(tel.position, tel.q_xyzw)

    def floor_z(self, tel):
        """Course z of flight_interface's floor (+ margin): origin_enu z + min_altitude_m."""
        c = self.cfg
        oz = tel.origin_z if tel.origin_z is not None else 0.0
        z_odom = oz + c.min_altitude_m + c.floor_margin_m
        return float(self.frame.to_course((0.0, 0.0, z_odom))[2])

    def _sp(self, xy, z, yaw_course=None):
        """Course target -> (odom ENU xyz, odom yaw or None)."""
        p = self.frame.to_odom((float(xy[0]), float(xy[1]), float(z)))
        yaw = None if yaw_course is None else self.frame.yaw_to_odom(yaw_course)
        return tuple(float(v) for v in p), yaw

    def _hold_here(self, tel):
        return tuple(float(v) for v in tel.position), None

    def _limit_yaw(self, yaw, tel):
        """Rate-limit the commanded course heading to yaw_rate_max (starting from the vehicle's)."""
        now = tel.now
        if self._sp_yaw is None:
            self._sp_yaw, self._sp_yaw_t = self.pose_course[1], now
        step = self.cfg.yaw_rate_max * max(0.0, min(now - self._sp_yaw_t, .1))   # a late tick never jumps
        err = wrap(yaw - self._sp_yaw)
        self._sp_yaw, self._sp_yaw_t = wrap(self._sp_yaw + float(np.clip(err, -step, step))), now
        return self._sp_yaw

    def flight_time(self, tel):
        if self._t_airborne is None:
            return 0.0
        t_mono, t_clock = self._t_airborne
        if t_clock is not None and tel.flight_clock is not None:
            return tel.flight_clock - t_clock         # sim time in a simulator
        return tel.now - t_mono

    # ---------------------------------------------------------------- planning (worker thread)
    def planning_allowed(self):
        if self.pose_course is None or self.frame is None:
            return False
        if self.state in ("NAVIGATE", "SHADOW"):
            return True
        return self.state == "TAKEOFF" and self.pose_course[0][2] > self.cfg.plan_above_m

    def _carrot_for(self, cmd, yaw):
        """Setpoint distance: shorter while the nose is still turning toward the route (0.5 m when
        aligned, 0.15 m at 60 deg). Same rule as the FlyX-proven upload_to_flyx.py."""
        c = self.cfg
        if cmd.mode != "move" or cmd.yaw is None:
            return c.carrot_m
        off = abs(float(np.arctan2(np.sin(cmd.yaw - yaw), np.cos(cmd.yaw - yaw))))
        return c.carrot_m * float(np.clip(1.0 - off / np.deg2rad(c.slow_turn_deg), c.slow_turn_min_frac, 1.0))

    def plan(self):
        """Advance the planner from the latest pose; ~0.05-0.5 s. Returns the Command or None."""
        from pathplan.ros_node import target_from_command
        if not self.planning_allowed():
            return None
        xyz, yaw, _ = self.pose_course
        cmd = self.planner.update(xyz[:2], yaw)
        target = target_from_command(cmd, xyz[:2], yaw, self._carrot_for(cmd, yaw))
        if cmd.mode == "turn":
            # hold the spot where the turning began (re-holding "here" every step drifted 11 cm
            # toward a wall during a look-around in FlyX run 6)
            if self._turn_xy is None:
                self._turn_xy = np.asarray(xyz[:2], float).copy()
            target = (self._turn_xy, target[1])
        else:
            self._turn_xy = None
        self.target = target
        self.planner_status = cmd.mode
        return cmd

    # ---------------------------------------------------------------- control (fast loop)
    def tick(self, tel: Telemetry) -> Output:
        now = tel.now
        self._dt = 0.0 if self._prev_now is None else max(0.0, now - self._prev_now)
        self._prev_now = now
        if (self._own_hold_t is not None and not tel.frozen and tel.state_t is not None
                and tel.state_t - self._own_hold_t > 1.0):
            self._own_hold_t = None             # our hold is gone (cleared by our READY setpoint, or never took)
        out = self._tick(tel)
        if out.setpoint is not None:
            self._sp_out_t = now
        if out.hold_request:
            self._own_hold_t = now
        return out

    def _tick(self, tel: Telemetry) -> Output:
        c, now = self.cfg, tel.now
        self._update_pose(tel)

        # ---- a flight ended: the vehicle is disarmed again
        if self.state in FLIGHT_STATES and not tel.armed and self._fresh(tel.state_t, now, c.state_max_age_s):
            if self.state == "ARMED":
                self._go("WAIT_HEALTH", now, "disarmed before Offboard (pilot, or PX4's preflight timeout)")
                self.frame = None
            else:
                if not self.result:
                    self.result = f"disarmed during {self.state}"
                self._go("DONE", now, self.result)
        if self.state == "DONE":
            if tel.armed or now - self._t_state < c.rearm_after_s:
                return Output(self.state, note=self.result)
            self._go("WAIT_HEALTH", now, "ready for the next flight")
            self.frame, self.pose_course, self.result = None, None, ""
            self._reset_flight()
            self._healthy_since = None
            return Output(self.state, reset_planner=True)

        if self.state == "WAIT_HEALTH":
            if not tel.armed:
                self._skip_arming = False
            problem = self._ground_problem(tel)
            if problem or self._skip_arming:
                self._healthy_since = None
                if tel.armed:
                    return self._armed_while_not_ready(tel, problem)
                return Output(self.state, note=problem)
            self._healthy_since = self._healthy_since if self._healthy_since is not None else now
            if now - self._healthy_since < c.healthy_for_s:
                return Output(self.state, note="healthy, confirming")
            self.frame = self._make_frame(tel)
            self._update_pose(tel)
            self._go("SHADOW" if c.shadow else "READY", now, "course frame from the pose on the launch mark")

        if self.state == "SHADOW":
            return Output(self.state, note=f"shadow: planner {self.planner_status}; no setpoints are sent")

        if self.state == "READY":
            if tel.armed:
                self.frame = self._make_frame(tel)                 # LATCH: on the mark, at arming
                self._update_pose(tel)
                self._go("ARMED", now, "pilot armed; course frame latched. Waiting for Position -> Offboard")
                return Output(self.state, self._hold_here(tel), "armed: waiting for Offboard")
            # Leave READY only for a real problem: a short odometry pause keeps the stream going.
            # Stopping it while the pilot armed let Offboard start with nobody flying (review #3a).
            problem = self._ground_problem(tel, odom_max_age=c.ready_odom_grace_s)
            if problem:
                self._healthy_since = None
                self._go("WAIT_HEALTH", now, problem)
                return Output(self.state, note=problem)
            note = "READY: pilot may arm (Altitude), then switch to Position"
            if not self._fresh(tel.odom_t, now, c.pose_max_age_s):
                note = (f"READY, odometry paused {now - tel.odom_t:.1f} s (leaves READY after "
                        f"{c.ready_odom_grace_s:.1f} s)")
            else:
                self.frame = self._make_frame(tel)                 # follows the drone until arming
                self._update_pose(tel)
            if tel.frozen and self._own_hold_t is not None:
                note += "; this setpoint clears the /skybeta/hold this node asked for earlier"
            return Output(self.state, self._hold_here(tel), note)

        if self.state == "ARMED":
            if tel.offboard_active:
                xyz, yaw, _ = self.pose_course
                self._takeoff = (np.asarray(xyz[:2], float).copy(), float(xyz[2]), now)
                self._t_airborne = (now, tel.flight_clock)
                self.result = ""
                self._go("TAKEOFF", now, "Offboard active")
            else:
                return Output(self.state, self._hold_here(tel), "armed: waiting for the switch to Position (Offboard)")

        if self.state in AIRBORNE:
            guard = self._airborne_guard(tel)
            if guard is not None:
                return guard

        if self.state == "TAKEOFF":
            xy0, z0, t0 = self._takeoff
            z_cmd = min(c.altitude, z0 + c.climb_rate * (now - t0))
            z_cmd = max(z_cmd, self.floor_z(tel))
            off = abs(self.pose_course[0][2] - c.altitude)
            if z_cmd >= c.altitude and off < c.takeoff_tolerance_m:
                self._go("NAVIGATE", now, "at altitude")
            elif now - self._t_state > c.takeoff_timeout_s and off < c.takeoff_loose_m:
                self._go("NAVIGATE", now, f"near altitude after {c.takeoff_timeout_s:.0f} s ({off:.2f} m off)")
            else:
                yaw = self._limit_yaw(self.pose_course[1], tel)
                sp = self._sp(xy0, z_cmd, yaw)
                self._last_sp = sp
                return Output(self.state, sp, f"climbing to {c.altitude:.2f} m ({self.pose_course[0][2]:.2f})")

        if self.state == "NAVIGATE":
            if not self._fresh(tel.odom_t, now, c.pose_max_age_s) and self._last_sp is not None:
                self._sp_yaw_t = now                    # the heading limiter resumes from here
                return Output(self.state, self._last_sp, "odometry paused: holding the last setpoint")
            if self.planner_status == "done":
                self._pad_since = now
                self._go("PAD", now, "landing pad reached")
            elif self.planner_status == "failed":
                why = self.planner.events[-1].get("event", "?") if self.planner.events else "?"
                return self._start_descend(tel, None, f"planner failed: {why}")
            else:
                if self.target is None:
                    xy, yaw = self.pose_course[0][:2], self.pose_course[1]
                else:
                    xy, yaw = self.target
                sp = self._sp(xy, c.altitude, self._limit_yaw(yaw, tel))
                self._last_sp = sp
                return Output(self.state, sp, f"planner {self.planner_status}")

        if self.state == "PAD":
            pad = np.asarray(self.planner.pad_xy, float)
            off = float(hypot(*(np.asarray(self.pose_course[0][:2]) - pad)))
            if off <= c.pad_centre_m or now - self._pad_since > c.pad_centre_timeout_s:
                return self._start_descend(tel, pad, f"over the pad ({off:.2f} m from its centre)")
            sp = self._sp(pad, c.altitude, self._limit_yaw(self.pose_course[1], tel))
            self._last_sp = sp
            return Output(self.state, sp, f"centring over the pad ({off:.2f} m)")

        if self.state == "DESCEND":
            xy, z0, t0, why = self._descend
            floor = self.floor_z(tel)
            z_cmd = max(floor, z0 - c.descend_rate * (now - t0))
            if z_cmd <= floor and self._floor_since is None:
                self._floor_since = now
            z = float(self.pose_course[0][2])
            at_floor = abs(z - floor) < c.descend_tolerance_m
            # PX4 holds EKF2's height, not cuVSLAM's: when they disagree the vehicle never reaches the
            # floor in our z. The ramp sitting at the floor for floor_settle_s is enough (review #5).
            settled = self._floor_since is not None and now - self._floor_since >= c.floor_settle_s
            if z_cmd <= floor and (at_floor or settled):
                self.result = why
                extra = "" if at_floor else f" (odometry z {z - floor:+.2f} m from the floor setpoint)"
                self._go("LOW_HOLD", now, why + extra + " - PILOT: LAND NOW")
            else:
                sp = self._sp(xy, z_cmd, self._limit_yaw(self.pose_course[1], tel))
                self._last_sp = sp
                return Output(self.state, sp, f"descending ({why})")

        if self.state == "LOW_HOLD":
            return self._low_hold(tel)

        if self.state in ("TAKEN_OVER", "FAULT"):
            out = Output(self.state, None, self.result, hold_request=(self.state == "FAULT" and not self._hold_sent))
            self._hold_sent = True
            return out

        return Output(self.state)

    def _low_hold(self, tel):
        """Hold at the floor (still ramping down when LOW_HOLD came early, from the run limit)."""
        xy, z0, t0, why = self._descend
        z_cmd = max(self.floor_z(tel), z0 - self.cfg.descend_rate * (tel.now - t0))
        sp = self._sp(xy, z_cmd, self._limit_yaw(self.pose_course[1], tel))
        self._last_sp = sp
        return Output(self.state, sp, f"{why}: holding low - pilot lands", land_request=True)

    def _armed_while_not_ready(self, tel, problem):
        """The pilot armed while we were not READY: never fly this arming. If our last setpoint is
        recent, flight_interface could still grant Offboard (it accepts a setpoint up to 1 s old)
        and the vehicle would lift to its 0.3 m floor with nobody flying it: ask /skybeta/hold,
        which makes it refuse Offboard (review #3b). Once per arming."""
        c, now = self.cfg, tel.now
        first = not self._skip_arming
        self._skip_arming = True
        hold = False
        if first:
            age = None if self._sp_out_t is None else now - self._sp_out_t
            hold = (age is not None and age <= c.hold_if_armed_within_s) or tel.offboard_active
            self._armed_hold_note = ""
            if hold:
                self._armed_hold_note = ("requested /skybeta/hold so flight_interface refuses Offboard"
                                         + ("" if age is None else f" (our last setpoint was {age:.1f} s old)"))
                self.events.append({"t": round(now, 3), "from": self.state, "to": self.state,
                                    "note": "armed while not READY: " + self._armed_hold_note})
        why = f" ({problem})" if problem else ""
        act = "TAKE OVER AND LAND" if tel.offboard_active else "DISARM"
        note = f"PILOT: {act} - armed while this node was not ready{why}; not flying this arming"
        if self._armed_hold_note:
            note += "; " + self._armed_hold_note
        return Output(self.state, note=note, hold_request=hold)

    def _start_descend(self, tel, xy, why):
        xy = np.asarray(self.pose_course[0][:2] if xy is None else xy, float).copy()
        self._descend = (xy, float(self.pose_course[0][2]), tel.now, why)
        self._floor_since = None
        self._go("DESCEND", tel.now, why)
        sp = self._sp(xy, self.pose_course[0][2], self._limit_yaw(self.pose_course[1], tel))
        self._last_sp = sp
        return Output(self.state, sp, f"descending ({why})")

    def _hold_last(self, tel, note):
        """Keep streaming the last setpoint without advancing the mission (PX4 failsafe Hold, PX4
        link gap). The takeoff / descent ramps pause too, so they resume where they were."""
        now, dt = tel.now, self._dt
        self._sp_yaw_t = now                            # the heading limiter resumes from here
        if self._takeoff is not None:
            xy0, z0, t0 = self._takeoff
            self._takeoff = (xy0, z0, t0 + dt)
        if self._descend is not None:
            xy, z0, t0, why = self._descend
            self._descend = (xy, z0, t0 + dt, why)
        self.px4_failsafe_waits += 1
        sp = self._last_sp
        if sp is None:                                  # nothing sent in the air yet: here, above the floor
            xyz = self.pose_course[0]
            sp = self._sp(xyz[:2], max(float(xyz[2]), self.floor_z(tel)), None)
        return Output(self.state, sp, note)

    def _airborne_guard(self, tel):
        c, now = self.cfg, tel.now
        waiting = ""
        if not tel.offboard_active:
            self._offboard_lost_since = self._offboard_lost_since if self._offboard_lost_since is not None else now
            lost = now - self._offboard_lost_since
            mode = NAV_STATE_NAMES.get(tel.nav_state, str(tel.nav_state))
            # PX4's failsafe Hold after a stream stall returns to Offboard by itself (review #2).
            # Any other mode is the pilot (or PX4's final failsafe action): never fight it.
            px4_hold = tel.nav_state == NAV_STATE_AUTO_LOITER
            if lost > (c.failsafe_resume_s if px4_hold else c.offboard_loss_s):
                if self.state == "LOW_HOLD":
                    self.result = self.result or "pilot landing"
                    why = f"pilot took over to land ({mode})"
                elif px4_hold:
                    why = (f"PX4 stayed in its failsafe Hold for {c.failsafe_resume_s:.0f} s in {self.state}: "
                           "not commanding")
                    self.result = why
                else:
                    why = f"PX4 left Offboard in {self.state} (now {mode}): pilot or failsafe; not commanding"
                    self.result = why
                self._go("TAKEN_OVER", now, why)
                return Output(self.state, None, why)
            if px4_hold:
                waiting = f"PX4 failsafe Hold for {lost:.1f} s: streaming the last setpoint until it returns to Offboard"
        else:
            self._offboard_lost_since = None
        # PX4 link: flight_interface drops it after 2 s without vehicle_status (review #1)
        if not tel.px4_link:
            self._link_lost_since = self._link_lost_since if self._link_lost_since is not None else now
            silent = now - self._link_lost_since
            if silent > c.px4_link_lost_s:
                return self._fault(now, f"PX4 link lost for {c.px4_link_lost_s:.0f} s")
            waiting = waiting or f"PX4 link down for {silent:.1f} s: holding the last setpoint"
        else:
            self._link_lost_since = None
        problem = self._air_problem(tel)
        if problem:
            return self._fault(now, problem)
        if waiting:
            return self._hold_last(tel, waiting)
        # the run limit, in every airborne state (TAKEOFF could hang past it, review #5)
        if self.state in ("TAKEOFF", "NAVIGATE", "PAD", "DESCEND") and self.flight_time(tel) > c.max_flight_s:
            why = f"run limit ({c.max_flight_s:.0f} s)"
            if self.state == "DESCEND":                 # keep its ramp; LOW_HOLD continues it
                xy, z0, t0, was = self._descend
                why = f"{why} during the descent ({was})"
                self._descend = (xy, z0, t0, why)
            else:
                xyz = self.pose_course[0]
                self._descend = (np.asarray(xyz[:2], float).copy(), float(xyz[2]), now, why)
            self.result = why
            self._go("LOW_HOLD", now, why + " - PILOT: LAND NOW")
            return self._low_hold(tel)
        return None

    def _fault(self, now, problem):
        self.result = "FAULT: " + problem
        self._go("FAULT", now, problem + " - requested /skybeta/hold; PILOT: TAKE OVER")
        self._hold_sent = True
        return Output(self.state, None, self.result, hold_request=True)
