"""Offline tests of suraj_course.py's CourseMission (Sky Beta contract) against a kinematic stand-in
for PX4 + the flight interface + the safety pilot (no ROS).

    python3 -m pytest -q suraj_course/test_suraj_course.py
"""
from math import cos, sin
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import suraj_course as m  # noqa: E402

# cuVSLAM odom at start-up: arbitrary origin and heading (the vehicle faces up the first corridor)
ODOM0, ODOM_YAW0 = (0.02, -0.01, 0.0), 0.35


class FakeVehicle:
    """Tracks pose setpoints (P gain k, 1.5 m/s cap, velocity lag tau, accel cap) once armed and in
    Offboard; reports what /visual_slam/tracking/odometry and /skybeta/state would."""

    def __init__(self, start=ODOM0, yaw=ODOM_YAW0, lag_gain=0.95, tau=0.35, a_max=2.0):
        self.tel = m.Telemetry()
        self.tel.position, self.tel.q = start, (cos(yaw / 2), 0.0, 0.0, sin(yaw / 2))
        self.tel.xy_valid = self.tel.z_valid = True
        self.k, self.tau, self.a_max = lag_gain, tau, a_max
        self.v, self.trace, self.ground_z, self.landing = [0.0, 0.0, 0.0], [], start[2], False

    def step(self, t, sp, dt=.05, stale_odom=False):
        tel = self.tel
        p = list(tel.position)
        flying = tel.arming_state == m.ARMED and tel.nav_state == m.NAV_OFFBOARD
        if tel.arming_state == m.ARMED and self.landing:
            p[2] = max(self.ground_z, p[2] - .5 * dt)
            self.v = [0.0, 0.0, 0.0]
        elif flying and sp is not None:
            tgt = list(sp[0])
            tgt[2] = max(tgt[2], self.ground_z + 0.3)               # the interface's min_altitude floor
            v = [self.k * (tgt[i] - p[i]) for i in range(3)]
            n = sum(x * x for x in v) ** .5
            v = [x * min(1.0, 1.5 / n) for x in v] if n else v
            dv = [(v[i] - self.v[i]) * min(1.0, dt / self.tau) for i in range(3)]
            na = sum(x * x for x in dv) ** .5
            dv = [x * min(1.0, self.a_max * dt / na) for x in dv] if na else dv
            self.v = [self.v[i] + dv[i] for i in range(3)]
            p = [p[i] + self.v[i] * dt for i in range(3)]
            tel.q = (cos(sp[2] / 2), 0.0, 0.0, sin(sp[2] / 2))
        tel.position, tel.velocity = tuple(p), tuple(self.v)
        tel.now = t
        tel.status_t = tel.local_t = t
        if not stale_odom:
            tel.odom_t = t
        self.trace.append(tuple(p))


def _inside(rel, spans):
    return rel is not None and any(a <= rel < a + d for a, d in spans)


def fly(vehicle=None, mission=None, seconds=260, odom=(), state=(), control=(), slow=None, hook=None,
        pilot_delay=2.0):
    """The fake safety pilot arms (-> Offboard) once the program has streamed pilot_delay s, and lands +
    disarms when it reports HOLD. odom / state / control: (start s after arming, length s) spans of
    stale odometry / stale /skybeta/state / a stalled program; slow: (start, end) /skybeta/state silent."""
    veh, mission = vehicle or FakeVehicle(), mission or m.CourseMission(settle=2.0)
    sp = None
    t, t_arm, t_stream = 0.0, None, None
    while t < seconds and mission.state not in ("DONE", "TAKEN_OVER"):
        rel = None if t_arm is None else t - t_arm
        veh.step(t, sp, stale_odom=_inside(rel, odom))
        if _inside(rel, state):
            veh.tel.status_t = veh.tel.local_t = t_arm + max(a for a, d in state if a <= rel)
        if slow and rel is not None and slow[0] <= rel < slow[1]:
            veh.tel.status_t = veh.tel.local_t = t_arm + slow[0]
        if mission.state == "STREAM" and t_stream is None:
            t_stream = t
        if t_stream is not None and t_arm is None and t - t_stream > pilot_delay:
            veh.tel.arming_state, veh.tel.nav_state, t_arm = m.ARMED, m.NAV_OFFBOARD, t
        if mission.state == "HOLD" and not veh.landing:
            veh.tel.nav_state, veh.landing = 0, True                 # pilot: Land
        if veh.landing and veh.tel.position[2] <= veh.ground_z + 1e-6:
            veh.tel.arming_state = m.DISARMED
        if hook:
            hook(t, mission, veh)
        if not _inside(rel, control):
            _, sp, cmd, _ = mission.tick(veh.tel)
            assert cmd is None, "the Sky Beta contract never commands PX4"
        t += .05
    return veh, mission, (t - t_arm if t_arm is not None else None)


def hold_note(mission):
    h = [e for e in mission.events if e.get("to") == "HOLD"]
    return h[0]["note"] if h else ""


def completed(mission):
    return mission.state == "DONE" and hold_note(mission).startswith("course complete")


def course_trace(veh, mission):
    return [mission.frame.to_course(p) for p in veh.trace]


def seg_dist(p, a, b):
    ab = [b[i] - a[i] for i in range(3)]
    L = sum(x * x for x in ab)
    s = 0.0 if not L else max(0.0, min(1.0, sum((p[i] - a[i]) * ab[i] for i in range(3)) / L))
    return sum((p[i] - a[i] - s * ab[i]) ** 2 for i in range(3)) ** .5


def max_off_route(veh, mission):
    """Largest distance of the flown path (above 0.3 m) from the route polyline."""
    pts = [tuple(m.ROUTE["start"])] + [w["position"] for w in mission.ctrl.waypoints]
    h = mission.ctrl.handoff_target
    pts.append((h[0], h[1], 0.0))                        # the pilot lands straight down
    legs = list(zip(pts, pts[1:]))
    return max(min(seg_dist(p, a, b) for a, b in legs) for p in course_trace(veh, mission) if p[2] > 0.3)


def test_no_fmu_topics_in_the_program():
    src = (Path(__file__).resolve().parent / "suraj_course.py").read_text(encoding="utf-8")
    body = src.split('"""', 2)[2]                        # skip the docstring that says so
    assert "/fmu" not in body and "VehicleCommand" not in body and "px4_msgs" not in body


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


def test_full_course_visits_every_waypoint_and_holds_over_the_pad():
    veh, mission, t = fly()
    assert completed(mission), mission.events[-4:]
    wps = [e["waypoint"] for e in mission.events if "waypoint" in e]
    assert wps == list(range(1, len(mission.ctrl.waypoints) + 1))
    end = mission.frame.to_course(veh.tel.position)
    lx, ly = m.ROUTE["landing"]["position"][:2]
    assert ((end[0] - lx) ** 2 + (end[1] - ly) ** 2) ** .5 < .15
    # kinematic stand-in only (route keeps >= 0.405 m from walls; Sky Beta proxy radius 0.28 m).
    # The real check is Gazebo truth: v140/v141 kept >= 0.317 m from every wall.
    assert max_off_route(veh, mission) < .13, "left the route corridor"
    assert 40 < t < 130, t


def test_window_passed_inside_the_aperture():
    veh, mission, _ = fly()
    w = m.ROUTE["window"]
    c, n, h = w["center"], w["normal_toward_entry"], w["horizontal_axis"]
    tr = course_trace(veh, mission)
    side = [sum((p[i] - c[i]) * n[i] for i in range(2)) for p in tr]
    hits = [i for i in range(1, len(tr)) if side[i - 1] > 0 >= side[i]
            and abs(sum((tr[i][j] - c[j]) * h[j] for j in range(2))) < w["width"] / 2 - m.ROUTE["footprint_radius"]]
    assert hits, "never crossed the window plane inside the aperture"
    assert abs(tr[hits[0]][2] - c[2]) < w["height"] / 2 - m.ROUTE["footprint_radius"]


def test_never_moves_before_the_pilot():
    veh, mission, _ = fly(seconds=30, pilot_delay=1e9)
    assert mission.state == "STREAM" and all(p == ODOM0 for p in veh.trace)


def test_waits_for_valid_state_before_streaming():
    def hook(t, mission, veh):
        veh.tel.xy_valid = veh.tel.z_valid = t > 8.0          # no valid local position for 8 s
    _, mission, _ = fly(seconds=12, pilot_delay=1e9, hook=hook)
    stream = [e for e in mission.events if e.get("to") == "STREAM"]
    assert stream and stream[0]["t"] > 8.0 + 2.0             # the settle starts only once valid


def test_sluggish_vehicle_is_leashed_and_stays_on_route():
    veh, mission, _ = fly(FakeVehicle(lag_gain=0.6, tau=1.0))
    assert completed(mission), mission.events[-4:]
    assert max_off_route(veh, mission) < .17


# FlyX square run 2 (2026-11-07 journal): measured pauses (start s after arming, length s)
LAB_RUN2_ODOM = [(1.4, 0.31), (2.0, 0.45), (2.5, 0.62), (3.3, 0.43), (13.6, 0.41), (14.2, 0.41), (14.9, 1.41)]
LAB_RUN2_STATE = [(2.0, 0.70), (3.4, 1.49), (13.6, 0.43), (14.9, 2.37)]
LAB_RUN2_CONTROL = [(1.7, 0.70), (2.6, 0.62), (14.6, 1.22), (16.4, 1.75)]


def _repeat(spans, period=20.0, until=160.0):
    return [(a + k * period, d) for k in range(int(until // period)) for a, d in spans]


def test_lab_pause_pattern_throughout_the_course_completes_on_route():
    """Run 2's odometry, state and program stalls, repeated every 20 s for the whole course."""
    veh, mission, t = fly(odom=_repeat(LAB_RUN2_ODOM), state=_repeat(LAB_RUN2_STATE),
                          control=_repeat(LAB_RUN2_CONTROL))
    assert completed(mission), mission.events[-4:]
    assert any("hold" in e for e in mission.events) and any("resume_after_s" in e for e in mission.events)
    assert max_off_route(veh, mission) < .15
    assert t < m.MAX_FLIGHT_S


def test_lab_run3_state_silent_6s_completes():
    _, mission, _ = fly(slow=(11.0, 17.0))
    assert completed(mission), mission.events[-4:]


def test_odometry_outage_holds_for_the_pilot():
    _, mission, _ = fly(odom=[(20.0, 300.0)])
    assert mission.state == "DONE" and "odometry lost" in hold_note(mission)


def test_invalid_local_position_holds_for_the_pilot():
    def hook(t, mission, veh):
        if mission.state == "FLY" and t > 25:
            veh.tel.xy_valid = False
    _, mission, _ = fly(hook=hook)
    assert "estimator invalid" in hold_note(mission)


def test_pilot_takeover_stops_commanding():
    def hook(t, mission, veh):
        if mission.state == "FLY" and t > 25:
            veh.tel.nav_state = 2                       # pilot switched out of Offboard
    _, mission, _ = fly(hook=hook)
    assert mission.state == "TAKEN_OVER"


def test_setpoint_frozen_during_odometry_pause():
    sps = []

    def hook(t, mission, veh):
        if mission.state == "FLY" and 30.5 < t - mission.t_takeoff < 31.3:
            sps.append(mission.v_pos)
    fly(odom=[(30.0, 1.4)], hook=hook)
    assert sps and all(s == sps[0] for s in sps)


def test_program_stall_anywhere_stays_on_route():
    """A 1.8 s stall of this program (FlyX measured up to 1.75 s) at any point of the course: the
    interface keeps the last pose, which must keep the drone within the route clearance (0.40 m)."""
    worst = 0.0
    for k in range(2, 60):
        veh, mission, _ = fly(control=[(k * 1.0, 1.8)])
        assert completed(mission), (k, mission.events[-3:])
        worst = max(worst, max_off_route(veh, mission))
    assert worst < .2, worst


def test_estimate_divergence_holds_for_the_pilot():
    """Gazebo v131: visual odometry diverged in the maze and the estimate ran away; hold (pilot lands)
    as soon as the drone is far from the reference."""
    def hook(t, mission, veh):
        if mission.state == "FLY" and t > 30:
            p = list(veh.tel.position)
            p[0] += 0.05                                   # estimate drifting 1 m/s
            veh.tel.position = tuple(p)
    _, mission, _ = fly(hook=hook)
    h = [e for e in mission.events if e.get("to") == "HOLD"]
    assert h and "diverged" in h[0]["note"] and h[0]["t"] < 33, h
