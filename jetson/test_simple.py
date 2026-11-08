"""Closed-loop test of simple_v4c_skybeta.py with a stub rclpy (no ROS): python -m pytest -q jetson/test_simple.py"""
import importlib
import json
import math
import os
import sys
import types

HERE = os.path.dirname(__file__)


class _Msg:
    def __init__(self, **kw):
        self.__dict__.update(kw)


def _stub_ros():
    pubs = {}

    class Node:
        def __init__(self, name):
            pass

        def create_publisher(self, typ, topic, q):
            pubs[topic] = []
            return types.SimpleNamespace(publish=lambda m, t=topic: pubs[t].append(m))

        def create_subscription(self, *a, **k):
            return None

        def create_timer(self, *a, **k):
            return None

        def get_logger(self):
            return types.SimpleNamespace(info=lambda s: None, warn=lambda s: None)

        def get_clock(self):
            return types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: 0))

    def mk(name):
        class M:
            def __init__(self, data=None):
                self.data = data
                self.header = types.SimpleNamespace(stamp=0, frame_id="")
                self.pose = types.SimpleNamespace(position=types.SimpleNamespace(x=0, y=0, z=0),
                                                  orientation=types.SimpleNamespace(x=0, y=0, z=0, w=1))
                self.twist = types.SimpleNamespace(linear=types.SimpleNamespace(x=0, y=0, z=0))
        M.__name__ = name
        return M

    mods = {
        "rclpy": types.SimpleNamespace(init=lambda: None, spin=lambda n: None, ok=lambda: False, shutdown=lambda: None),
        "rclpy.node": types.SimpleNamespace(Node=Node),
        "rclpy.qos": types.SimpleNamespace(qos_profile_sensor_data=None),
        "geometry_msgs.msg": types.SimpleNamespace(PoseStamped=mk("PoseStamped"), TwistStamped=mk("TwistStamped")),
        "nav_msgs.msg": types.SimpleNamespace(Odometry=mk("Odometry")),
        "std_msgs.msg": types.SimpleNamespace(String=mk("String")),
    }
    for k in ("geometry_msgs", "nav_msgs", "std_msgs"):
        mods[k] = types.ModuleType(k)
    sys.modules.update(mods)
    return pubs


def run(odom_gap=None, pilot_at=None, seconds=80.0, tau=0.8):
    pubs = _stub_ros()
    sys.path.insert(0, HERE)
    st = importlib.import_module("simple_v4c_skybeta")
    importlib.reload(st)
    clock = [0.0]
    st.time.monotonic = lambda: clock[0]
    n = st.SimpleTest()
    z, ground, armed, landed_t = 0.02, 0.02, True, None
    hold_pose_z = None
    log, dt = [], 0.1
    while clock[0] < seconds:
        clock[0] += dt
        t = clock[0]
        off = armed and t > 2.0 and not (pilot_at and t >= pilot_at)
        landed = z <= ground + 0.01
        gap = odom_gap and odom_gap[0] <= t < odom_gap[1]
        if not gap:
            o = _Msg(pose=_Msg(pose=_Msg(position=_Msg(x=1.0, y=2.0, z=z), orientation=_Msg(x=0, y=0, z=0, w=1))))
            n.on_odom(o)
        n.on_state(_Msg(data=json.dumps({"armed": armed, "offboard_active": off, "landed": landed})))
        np_, nt_ = len(pubs["/skybeta/setpoint/pose"]), len(pubs["/skybeta/setpoint/twist"])
        n.tick()
        pose_new = pubs["/skybeta/setpoint/pose"][np_:]
        tw_new = pubs["/skybeta/setpoint/twist"][nt_:]
        if off:   # vehicle: flight_interface clamps pose z to >= start + 0.3; twist moves at the commanded rate
            if pose_new:
                hold_pose_z = max(pose_new[-1].pose.position.z, ground + 0.3)
                z += (hold_pose_z - z) * dt / tau
            elif tw_new:
                z = max(ground, z + tw_new[-1].twist.linear.z * dt)
            elif hold_pose_z is not None:   # twist expired -> flight_interface holds the last pose
                z += (hold_pose_z - z) * dt / tau
        if landed and n.phase == "LANDED":
            landed_t = landed_t or t
            if t - landed_t > 2.0:
                armed = False                 # PX4 auto-disarm (COM_DISARM_LAND)
        log.append((t, n.phase, z, bool(tw_new), tw_new[-1].twist.linear.z if tw_new else None))
    return log, pubs


def test_profile():
    log, pubs = run()
    phases = []
    for _, p, *_ in log:
        if not phases or phases[-1] != p:
            phases.append(p)
    assert phases == ["WAIT", "CLIMB", "HOVER", "DESCEND", "TOUCHDOWN", "LANDED", "DONE"], phases
    hover = [t for t, p, *_ in log if p == "HOVER"]
    assert 4.9 <= hover[-1] - hover[0] + 0.1 <= 5.3
    assert max(z for _, _, z, *_ in log) < 1.0 + 0.02 + 0.05      # ~1.0 m above start, no overshoot
    assert log[-1][2] <= 0.03                                       # on the floor
    downs = [v for *_, tw, v in log if tw]
    assert downs and all(abs(v + 0.10) < 1e-9 for v in downs)       # touchdown exactly 0.10 m/s down
    xs = [m.pose.position for m in pubs["/skybeta/setpoint/pose"][30:]]
    assert all(abs(p.x - 1.0) < 1e-9 and abs(p.y - 2.0) < 1e-9 for p in xs)   # purely vertical


def test_descent_and_climb_rates():
    log, pubs = run()
    zs = [m.pose.position.z for m in pubs["/skybeta/setpoint/pose"]]
    steps = [b - a for a, b in zip(zs, zs[1:])]
    assert max(steps) <= 0.25 * 0.1 + 1e-9 and min(steps) >= -0.15 * 0.1 - 1e-9


def test_odom_gap_during_touchdown_keeps_descending():
    log, _ = run()
    t_td = next(t for t, p, *_ in log if p == "TOUCHDOWN")
    log2, _ = run(odom_gap=(t_td + 0.5, t_td + 3.0))
    gap = [(t, z, tw) for t, p, z, tw, _ in log2 if t_td + 0.6 < t < t_td + 2.9]
    assert all(tw for _, _, tw in gap)                              # still commanding down
    assert gap[-1][1] < gap[0][1]                                   # and still going down, not climbing
    assert log2[-1][1] == "DONE"


def test_odom_gap_in_hover_stops_sending():
    log, _ = run()
    t_h = next(t for t, p, *_ in log if p == "HOVER")
    log2, pubs = run(odom_gap=(t_h + 1.0, t_h + 2.0))
    assert log2[-1][1] == "DONE"


def test_pilot_takeover_stops_for_good():
    log, pubs = run(pilot_at=6.0)
    assert log[-1][1] == "PILOT"
    assert not any(tw for t, p, z, tw, _ in log if t > 6.1)
