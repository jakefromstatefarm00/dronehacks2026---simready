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

from dataclasses import dataclass, field

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
    pose_max_age_s: float = .3
    status_max_age_s: float = 3.0          # vehicle_status ~1.7 Hz and land_detected ~0.9 Hz in FlyX
    # vehicle_local_position: only its xy/z validity flags are used (the pose comes from odometry,
    # held to pose_max_age_s). FlyX run 3 landed in TAKEOFF on a 0.69 s gap while odometry was fresh.
    local_max_age_s: float = 2.0
    offboard_loss_s: float = .5
    launch_xy: tuple = (-2.515, .914)      # launch mark, course frame
    launch_heading: float = float(np.pi / 2)   # vehicle heading on the mark, course frame (facing +y)
    launch_body_agl: float = .05           # body origin above the floor when sitting on the mark
    resend_s: float = 1.0                  # repeat Offboard / arm / Land until PX4 shows it (lossy link)
    takeoff_timeout_s: float = 10.0        # then accept being within takeoff_loose_m of the altitude
    takeoff_loose_m: float = .25
    plan_above_m: float = .5               # no mapping/planning below this (depth from the floor)


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
        self._offboard_lost_since = None
        self.pose_course = None            # (xyz, yaw, rotation) for depth projection

    # ---------------------------------------------------------------- helpers
    def _go(self, state, now, note=""):
        self.events.append({"t": round(now, 3), "from": self.state, "to": state, "note": note})
        self.state, self._t_state = state, now

    def _fresh(self, t, now, max_age):
        # A report stamped slightly after ``now`` is fresher, not stale: callbacks on other
        # threads can stamp arrivals between the control loop reading the clock and this check.
        return t is not None and now - t <= max_age

    def _unhealthy_reason(self, tel):
        """'' when healthy, else which check failed (logged with the LAND transition)."""
        c, now = self.cfg, tel.now
        age = lambda t: "never" if t is None else f"{now - t:.2f}s old"
        if not self._fresh(tel.odom_t, now, c.pose_max_age_s):
            return f"odometry stale ({age(tel.odom_t)})"
        if tel.pose_frame != POSE_FRAME_NED:
            return f"odometry frame {tel.pose_frame} is not NED"
        if not self._fresh(tel.status_t, now, c.status_max_age_s):
            return f"vehicle_status stale ({age(tel.status_t)})"
        if not self._fresh(tel.local_t, now, c.local_max_age_s):
            return f"local_position stale ({age(tel.local_t)})"
        if not (tel.xy_valid and tel.z_valid):
            return f"estimator invalid (xy_valid={tel.xy_valid}, z_valid={tel.z_valid})"
        return ""

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
        self.target = target_from_command(cmd, xyz[:2], yaw, self.cfg.carrot_m)
        if cmd.mode == "turn":
            # hold the spot where the turning began: re-holding "here" every step let the vehicle
            # drift 11 cm during a look-around in FlyX run 6, toward a wall
            if self._turn_xy is None:
                self._turn_xy = np.asarray(xyz[:2], float).copy()
            self.target = (self._turn_xy, self.target[1])
        else:
            self._turn_xy = None
        self.planner_status = cmd.mode
        return cmd

    def planning_allowed(self):
        """Map and plan only in flight (or in shadow): on the ground the camera sits 5 cm up and
        each "look around" call spent turning budget the vehicle never used."""
        if self.pose_course is None:
            return False
        if self.state in ("NAVIGATE", "SHADOW"):
            return True
        return self.state == "TAKEOFF" and self.pose_course[0][2] > self.cfg.plan_above_m

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
            if self.planner_status == "done":
                return self._land(now, "landing pad reached")
            if self.planner_status == "failed":
                return self._land(now, "planner failed: " + (self.planner.events[-1]["event"] if self.planner.events else ""))
            if self.target is None:
                return Output(self.state, self._course_setpoint(self.pose_course[0][:2], self.pose_course[1]))
            return Output(self.state, self._course_setpoint(*self.target), note=f"planner {self.planner_status}")

        if self.state == "LAND":
            if tel.landed and tel.arming_state == ARMING_STATE_DISARMED:
                self._go("DONE", now, "landed and disarmed")
                return Output(self.state)
            if tel.nav_state != NAV_STATE_AUTO_LAND and not tel.landed and self._resend(now):
                return Output(self.state, command=Command.land(), note="repeating Land")
            return Output(self.state)

        return Output(self.state)

    def _airborne_guard(self, tel, healthy):
        c, now = self.cfg, tel.now
        if tel.nav_state != NAV_STATE_OFFBOARD:
            self._offboard_lost_since = self._offboard_lost_since if self._offboard_lost_since is not None else now
            if now - self._offboard_lost_since > c.offboard_loss_s:
                why = ("PX4 failsafe" if tel.failsafe else
                       "mode changed by someone else" if tel.user_intention != NAV_STATE_OFFBOARD else "unknown")
                self._go("TAKEN_OVER", now, f"PX4 left Offboard: {why} (nav_state {tel.nav_state}, "
                                            f"requested {tel.user_intention}, failsafe {tel.failsafe}); not commanding")
                return Output(self.state)
        else:
            self._offboard_lost_since = None
        if not healthy:
            return self._land(now, "unhealthy: " + self._unhealthy_reason(tel))
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
