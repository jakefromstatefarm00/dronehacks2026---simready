"""Offline tests of fly_course.py's CourseMission against a kinematic stand-in for PX4 (no ROS).

    python3 -m pytest -q flyx_course/test_fly_course.py
"""
from math import cos, sin
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import fly_course as m  # noqa: E402

# Arbitrary PX4 local frame: EKF origin elsewhere, vehicle facing course +y at some NED yaw.
START_NED, START_YAW = (0.4, -0.3, -0.02), 0.7


class FakePX4:
    """Tracks position + velocity feedforward (P gain k, 1.5 m/s cap), honours Offboard/arm/land."""

    def __init__(self, start=START_NED, yaw=START_YAW, refuse_arm=False, lag_gain=0.95, tau=0.35, a_max=2.0):
        self.tel = m.Telemetry()
        self.tel.position, self.tel.q = start, (cos(yaw / 2), 0.0, 0.0, sin(yaw / 2))
        self.tel.landed, self.tel.xy_valid, self.tel.z_valid = True, True, True
        self.refuse_arm, self.landing, self.ground_z, self.k = refuse_arm, False, start[2], lag_gain
        self.trace = []
        self.v, self.tau, self.a_max = [0.0, 0.0, 0.0], tau, a_max

    def step(self, t, sp, cmd, dt=.05, stale_odom=False):
        tel = self.tel
        if cmd:
            if cmd[0] == m.CMD_DO_SET_MODE:
                tel.nav_state = m.NAV_OFFBOARD
            elif cmd[0] == m.CMD_ARM_DISARM and not self.refuse_arm:
                tel.arming_state = m.ARMED
            elif cmd[0] == m.CMD_NAV_LAND:
                self.landing, tel.nav_state = True, 18
        p = list(tel.position)
        if tel.arming_state == m.ARMED:
            if self.landing:
                p[2] = min(self.ground_z, p[2] + .5 * dt)
                if p[2] >= self.ground_z:
                    tel.landed, tel.arming_state = True, m.DISARMED
            elif sp is not None:
                tgt, ff = sp[0], sp[1] or (0.0, 0.0, 0.0)
                v = [ff[i] + self.k * (tgt[i] - p[i]) for i in range(3)]
                n = sum(x * x for x in v) ** .5
                v = [x * min(1.0, 1.5 / n) for x in v] if n else v
                # vehicle inertia: velocity follows PX4's demand with a lag and an acceleration cap
                dv = [(v[i] - self.v[i]) * min(1.0, dt / self.tau) for i in range(3)]
                na = sum(x * x for x in dv) ** .5
                dv = [x * min(1.0, self.a_max * dt / na) for x in dv] if na else dv
                self.v = [self.v[i] + dv[i] for i in range(3)]
                p = [p[i] + self.v[i] * dt for i in range(3)]
                if p[2] < self.ground_z - .05:
                    tel.landed = False
                tel.q = (cos(sp[2] / 2), 0.0, 0.0, sin(sp[2] / 2))
        tel.position = tuple(p)
        tel.velocity = tuple(self.v)
        tel.now = tel.sim_now = t
        tel.local_t = tel.status_t = tel.landed_t = t
        if not stale_odom:
            tel.odom_t = t
        self.trace.append(tuple(p))


def _inside(rel, spans):
    return rel is not None and any(a <= rel < a + d for a, d in spans)


def fly(px4=None, mission=None, seconds=260, odom=(), local=(), control=(), slow=None, hook=None):
    """odom / local / control: (start s after arming, length s) spans; slow: (start, end) in which
    status + land + local are all silent."""
    px4, mission = px4 or FakePX4(), mission or m.CourseMission(settle=2.0)
    sp = cmd = None
    t, t_arm = 0.0, None
    while t < seconds and mission.state not in ("DONE", "FAILED", "TAKEN_OVER"):
        rel = None if t_arm is None else t - t_arm
        px4.step(t, sp, cmd, stale_odom=_inside(rel, odom))
        if _inside(rel, local):
            px4.tel.local_t = t_arm + max(a for a, d in local if a <= rel)
        if slow and rel is not None and slow[0] <= rel < slow[1]:
            px4.tel.status_t = px4.tel.local_t = px4.tel.landed_t = t_arm + slow[0]
        if hook:
            hook(t, mission, px4)
        if not _inside(rel, control):
            _, sp, cmd, _ = mission.tick(px4.tel)
        else:
            cmd = None                                  # program stalled: nothing new is sent
        if t_arm is None and mission.state == "FLY":
            t_arm = t
        t += .05
    return px4, mission, (t - t_arm if t_arm is not None else None)


def completed(mission):
    land = [e for e in mission.events if e.get("to") == "LAND"]
    return mission.state == "DONE" and land and "course complete" in land[0]["note"]


def course_trace(px4, mission):
    return [mission.frame.to_course(p) for p in px4.trace]


def seg_dist(p, a, b):
    ab = [b[i] - a[i] for i in range(3)]
    L = sum(x * x for x in ab)
    s = 0.0 if not L else max(0.0, min(1.0, sum((p[i] - a[i]) * ab[i] for i in range(3)) / L))
    return sum((p[i] - a[i] - s * ab[i]) ** 2 for i in range(3)) ** .5


def max_off_route(px4, mission):
    """Largest distance of the flown path (above 0.3 m) from the friend's route polyline."""
    pts = [tuple(m.ROUTE["start"])] + [w["position"] for w in mission.ctrl.waypoints]
    h = mission.ctrl.handoff_target
    pts.append((h[0], h[1], 0.0))                        # native Land: straight down from the handoff
    legs = list(zip(pts, pts[1:]))
    return max(min(seg_dist(p, a, b) for a, b in legs) for p in course_trace(px4, mission) if p[2] > 0.3)


def test_frame_round_trip_and_launch_mark():
    f = m.CourseFrame(START_NED, START_YAW)
    c = f.to_course(START_NED)
    assert abs(c[0] - m.LAUNCH_XY[0]) < 1e-9 and abs(c[1] - m.LAUNCH_XY[1]) < 1e-9 and abs(c[2]) < 1e-9
    p = (1.2, -0.7, -0.9)
    assert max(abs(a - b) for a, b in zip(f.to_ned(f.to_course(p)), p)) < 1e-9
    assert abs(f.yaw_to_course(START_YAW) - m.LAUNCH_HEADING) < 1e-9
    fwd = (START_NED[0] + cos(START_YAW), START_NED[1] + sin(START_YAW), START_NED[2])
    d = [a - b for a, b in zip(f.to_course(fwd), c)]
    assert abs(d[0]) < 1e-9 and abs(d[1] - 1) < 1e-9                   # vehicle forward = course +y
    assert f.to_course((0, 0, START_NED[2] - 1))[2] > 0.99               # NED up = course up


def test_full_course_visits_every_waypoint_and_lands_on_the_pad():
    px4, mission, t = fly()
    assert completed(mission), mission.events[-4:]
    wps = [e["waypoint"] for e in mission.events if "waypoint" in e]
    assert wps == list(range(1, len(mission.ctrl.waypoints) + 1))
    land = [e for e in mission.events if e.get("to") == "LAND"][0]
    assert "course complete" in land["note"]
    end = mission.frame.to_course(px4.tel.position)
    lx, ly = m.ROUTE["landing"]["position"][:2]
    assert ((end[0] - lx) ** 2 + (end[1] - ly) ** 2) ** .5 < .15
    # kinematic stand-in only (route keeps >= 0.405 m from walls; Sky Beta proxy radius 0.28 m).
    # The real check is Gazebo truth: v132 kept >= 0.302 m from every wall.
    assert max_off_route(px4, mission) < .13, "left the route corridor"
    assert 40 < t < 130, t


def test_window_passed_inside_the_aperture():
    px4, mission, _ = fly()
    w = m.ROUTE["window"]
    c, n, h = w["center"], w["normal_toward_entry"], w["horizontal_axis"]
    tr = course_trace(px4, mission)
    side = [sum((p[i] - c[i]) * n[i] for i in range(2)) for p in tr]
    hits = [i for i in range(1, len(tr)) if side[i - 1] > 0 >= side[i]
            and abs(sum((tr[i][j] - c[j]) * h[j] for j in range(2))) < w["width"] / 2 - m.ROUTE["footprint_radius"]]
    assert hits, "never crossed the window plane inside the aperture"
    assert abs(tr[hits[0]][2] - c[2]) < w["height"] / 2 - m.ROUTE["footprint_radius"]


def test_refused_arm_fails_on_the_ground():
    px4, mission, _ = fly(FakePX4(refuse_arm=True))
    assert mission.state == "FAILED" and "arm" in mission.events[-1]["note"]
    assert all(p == START_NED for p in px4.trace)


def test_sluggish_px4_is_leashed_and_stays_on_route():
    px4, mission, _ = fly(FakePX4(lag_gain=0.6, tau=1.0))
    assert completed(mission), mission.events[-4:]
    assert max_off_route(px4, mission) < .17


# FlyX square run 2 (2026-11-07 journal): measured pauses (start s after arming, length s)
LAB_RUN2_ODOM = [(1.4, 0.31), (2.0, 0.45), (2.5, 0.62), (3.3, 0.43), (13.6, 0.41), (14.2, 0.41), (14.9, 1.41)]
LAB_RUN2_LOCAL = [(2.0, 0.70), (3.4, 1.49), (13.6, 0.43), (14.9, 2.37)]
LAB_RUN2_CONTROL = [(1.7, 0.70), (2.6, 0.62), (14.6, 1.22), (16.4, 1.75)]


def _repeat(spans, period=20.0, until=160.0):
    return [(a + k * period, d) for k in range(int(until // period)) for a, d in spans]


def test_lab_pause_pattern_throughout_the_course_completes_on_route():
    """Run 2's odometry, local-position and program stalls, repeated every 20 s for the whole course."""
    px4, mission, t = fly(odom=_repeat(LAB_RUN2_ODOM), local=_repeat(LAB_RUN2_LOCAL),
                          control=_repeat(LAB_RUN2_CONTROL))
    assert completed(mission), mission.events[-4:]
    assert any("hold" in e for e in mission.events) and any("resume_after_s" in e for e in mission.events)
    assert max_off_route(px4, mission) < .15
    assert t < m.MAX_FLIGHT_S


def test_lab_run3_slow_topics_silent_6s_completes():
    _, mission, _ = fly(slow=(11.0, 17.0))
    assert completed(mission), mission.events[-4:]


def test_odometry_outage_lands():
    _, mission, _ = fly(odom=[(20.0, 300.0)])
    land = [e for e in mission.events if e.get("to") == "LAND"]
    assert land and "odometry lost" in land[0]["note"]


def test_fresh_invalid_estimate_lands():
    def hook(t, mission, px4):
        if mission.state == "FLY" and t > 25:
            px4.tel.xy_valid = False
    _, mission, _ = fly(hook=hook)
    land = [e for e in mission.events if e.get("to") == "LAND"]
    assert land and "estimator invalid" in land[0]["note"]


def test_leaving_offboard_stops_commanding():
    def hook(t, mission, px4):
        if mission.state == "FLY" and t > 25:
            px4.tel.nav_state = 2                       # pilot / failsafe took over
    _, mission, _ = fly(hook=hook)
    assert mission.state == "TAKEN_OVER"


def test_setpoint_frozen_during_odometry_pause():
    sps = []

    def hook(t, mission, px4):
        if mission.state == "FLY" and 30.5 < t - mission.t_takeoff < 31.3:
            sps.append(mission.v_pos)
    fly(odom=[(30.0, 1.4)], hook=hook)
    assert sps and all(s == sps[0] for s in sps)


def test_program_stall_anywhere_stays_on_route():
    """A 1.8 s stall of this program (FlyX measured up to 1.75 s) at any point of the course: PX4
    keeps the last target, which must keep the drone within the route clearance (0.40 m)."""
    worst = 0.0
    for k in range(2, 60):
        px4, mission, _ = fly(control=[(k * 1.0, 1.8)])
        assert completed(mission), (k, mission.events[-3:])
        worst = max(worst, max_off_route(px4, mission))
    assert worst < .2, worst


def test_other_setpoint_publisher_on_the_ground_never_arms():
    """skybeta_flight_interface (or anything else) already publishing /fmu/in setpoints: do not fight it."""
    def hook(t, mission, px4):
        px4.tel.foreign = "1 other publisher(s) on /fmu/in/offboard_control_mode"
    px4, mission, _ = fly(hook=hook, seconds=40)
    assert mission.state == "WAIT_HEALTH" and px4.tel.arming_state == m.DISARMED
    assert all(p == START_NED for p in px4.trace)


def test_other_setpoint_publisher_in_flight_lands():
    def hook(t, mission, px4):
        if mission.state == "FLY" and t > 25:
            px4.tel.foreign = "1 other publisher(s) on /fmu/in/trajectory_setpoint"
    _, mission, _ = fly(hook=hook)
    land = [e for e in mission.events if e.get("to") == "LAND"]
    assert land and "another program commands PX4" in land[0]["note"]


def test_estimate_divergence_lands():
    """Gazebo v131: visual odometry diverged in the maze and the estimate ran away; the drone must
    land as soon as it is far from the route reference, not wait for PX4 to flag the estimate."""
    def hook(t, mission, px4):
        if mission.state == "FLY" and t > 30:
            p = list(px4.tel.position)
            p[0] += 0.05                                   # estimate drifting 1 m/s
            px4.tel.position = tuple(p)
    _, mission, _ = fly(hook=hook)
    land = [e for e in mission.events if e.get("to") == "LAND"]
    assert land and "diverged" in land[0]["note"]
    assert land[0]["t"] < 33, land[0]


# ------------------------------------------------------------------ Sky Beta contract (default mode)
ODOM0, ODOM_YAW0 = (0.02, -0.01, 0.0), 0.35      # cuVSLAM odom: start-up pose, arbitrary heading


def fly_skybeta(seconds=260, odom=(), hook=None, pilot_delay=2.0):
    """Fake pilot: arms + Offboard once the program streams; disarms (lands) when it reports HOLD."""
    px4, mission = FakePX4(start=ODOM0, yaw=ODOM_YAW0), m.SkybetaMission(settle=2.0)
    sp = None
    t, t_arm, t_stream = 0.0, None, None
    while t < seconds and mission.state not in ("DONE", "TAKEN_OVER"):
        rel = None if t_arm is None else t - t_arm
        px4.step(t, sp, None, stale_odom=_inside(rel, odom))
        if mission.state == "STREAM" and t_stream is None:
            t_stream = t
        if t_stream is not None and t_arm is None and t - t_stream > pilot_delay:
            px4.tel.arming_state, px4.tel.nav_state, t_arm = m.ARMED, m.NAV_OFFBOARD, t
        if mission.state == "HOLD" and px4.tel.arming_state == m.ARMED:
            px4.tel.arming_state, px4.tel.nav_state = m.DISARMED, 0      # the pilot lands
        if hook:
            hook(t, mission, px4)
        _, sp, cmd, _ = mission.tick(px4.tel)
        assert cmd is None, "the Sky Beta contract never commands PX4"
        t += .05
    return px4, mission


def test_enu_frame_round_trip_and_launch_mark():
    f = m.EnuFrame(ODOM0, ODOM_YAW0)
    c = f.to_course(ODOM0)
    assert abs(c[0] - m.LAUNCH_XY[0]) < 1e-9 and abs(c[1] - m.LAUNCH_XY[1]) < 1e-9 and abs(c[2]) < 1e-9
    p = (1.2, -0.7, 0.9)
    assert max(abs(a - b) for a, b in zip(f.to_ned(f.to_course(p)), p)) < 1e-9
    fwd = (ODOM0[0] + cos(ODOM_YAW0), ODOM0[1] + sin(ODOM_YAW0), 0.0)      # 1 m along the start heading
    d = [a - b for a, b in zip(f.to_course(fwd), c)]
    assert abs(d[0]) < 1e-9 and abs(d[1] - 1) < 1e-9                       # = course +y (up the corridor)
    assert abs(f.yaw_to_course(ODOM_YAW0) - m.LAUNCH_HEADING) < 1e-9
    assert f.to_course((0, 0, 1.0))[2] > .99                                 # ENU up = course up


def test_skybeta_full_course_holds_over_the_pad_for_the_pilot():
    px4, mission = fly_skybeta()
    assert mission.state == "DONE", mission.events[-3:]
    hold = [e for e in mission.events if e.get("to") == "HOLD"][0]
    assert "course complete" in hold["note"]
    wps = [e["waypoint"] for e in mission.events if "waypoint" in e]
    assert wps == list(range(1, len(mission.ctrl.waypoints) + 1))
    end = mission.frame.to_course(px4.tel.position)
    lx, ly = m.ROUTE["landing"]["position"][:2]
    assert ((end[0] - lx) ** 2 + (end[1] - ly) ** 2) ** .5 < .15
    assert max_off_route(px4, mission) < .13


def test_skybeta_does_not_fly_before_the_pilot():
    px4, mission = fly_skybeta(seconds=30, pilot_delay=1e9)
    assert mission.state == "STREAM" and all(p == ODOM0 for p in px4.trace)


def test_skybeta_odometry_outage_holds_for_the_pilot():
    _, mission = fly_skybeta(odom=[(20.0, 300.0)])
    hold = [e for e in mission.events if e.get("to") == "HOLD"]
    assert hold and "odometry lost" in hold[0]["note"]


def test_skybeta_pilot_takeover_stops_commanding():
    def hook(t, mission, px4):
        if mission.state == "FLY" and t > 25:
            px4.tel.nav_state = 2
    _, mission = fly_skybeta(hook=hook)
    assert mission.state == "TAKEN_OVER"
