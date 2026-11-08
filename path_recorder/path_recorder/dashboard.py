"""Live terminal dashboard for path_recorder (read-only: subscribes to /path_recorder/status only).

    ros2 run path_recorder dashboard            # inside the stack: cd ~/skybeta && host/run.sh shell

Refreshes once a second. Ctrl-C to quit (the recorder keeps running).
"""
from __future__ import annotations

import json
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

G, Y, R, B, X = "\033[32m", "\033[33m", "\033[31m", "\033[1m", "\033[0m"


def fmt(v, unit="", nd=2):
    return "  -  " if v is None else f"{v:.{nd}f}{unit}"


def ok(cond, text_ok, text_bad):
    return f"{G}{text_ok}{X}" if cond else f"{R}{text_bad}{X}"


def render(s, age):
    out = []
    if s is None:
        return [f"{R}No /path_recorder/status yet.{X} Is the recorder running?",
                "  ros2 node list | grep path_recorder        (should list /path_recorder)",
                "  ros2 launch path_recorder path_recorder.launch.py"]
    stale = age > 2.0
    rec = s["recording"]
    head = (f"{B}{R}* RECORDING{X}{B}  {s['folder'].rsplit('/', 1)[-1]}{X}" if rec
            else f"{B}idle{X} - arm the drone to start a recording")
    out.append(head + (f"   {R}(status {age:.0f} s old: recorder stopped?){X}" if stale else ""))
    out.append("")
    out.append(f"drone    armed {ok(not s['armed'], 'no', 'YES')}   landed {s['landed']}   mode {B}{s['mode']}{X}   "
               f"offboard {ok(not s['offboard'], 'no', 'YES (should never be!)')}")
    out.append(f"health   PX4 link {ok(s['px4_link'], 'ok', 'DOWN')}   PX4 position "
               f"{ok(s['px4_position_valid'], 'valid', 'INVALID')}   odometry {fmt(s['odom_hz'], ' Hz', 0)} "
               f"(last {fmt(s['odom_age_s'], ' s')})   depth {s['depth_topic'] or 'none'} {fmt(s['depth_hz'], ' Hz', 0)}")
    c = s["clearance"]
    out.append("clear    " + ("no depth" if c is None else
                              "   ".join(f"{n} {fmt(v, ' m')}" for n, v in zip(("left", "centre", "right"), c))))
    out.append("")
    if rec:
        p = s["rel_pose"]
        out.append(f"points   {B}{s['points']}{X}   path {B}{s['length_m']:.2f} m{X}   time {s['duration_s']:.0f} s   "
                   f"airborne {s['airborne_s']:.0f} s   max height {s['max_height_m']:.2f} m")
        if p:
            out.append(f"pose     x {p[0]:+.2f} fwd   y {p[1]:+.2f} left   z {p[2]:+.2f} up   yaw {p[3]:+.0f} deg"
                       "   (relative to where it was armed)")
        fl = s["flags"]
        out.append("flags    " + (", ".join(f"{Y}{k} x{v}{X}" for k, v in fl.items()) if fl else f"{G}none{X}"))
        out.append("")
        out.append("last points:  idx     t      x      y      z    yaw  speed  clr_c  reason    flags")
        for r in s["last_points"]:
            out.append(f"            {r['idx']:>5} {r['t']:>6} {r['x']:>6} {r['y']:>6} {r['z']:>6} {r['yaw']:>6} "
                       f"{r['speed']:>6} {r['clear_center'] if r['clear_center'] != '' else '-':>6}  {r['reason']:<9} "
                       f"{r['flags']}")
    if s["finished_this_boot"]:
        out.append("")
        out.append("saved this boot:")
        for d in s["finished_this_boot"]:
            out.append(f"  {d['name']}  {d['points']} pts  {d['length_m']:.2f} m  {d['duration_s']:.0f} s"
                       + (f"  {Y}(short / aborted?){X}" if d["aborted_or_short"] else ""))
    out.append("")
    out.append(f"saving to {s['output_dir']}")
    for w in s["warnings"]:
        out.append(f"{Y}! {w}{X}")
    return out


class Dashboard(Node):
    def __init__(self):
        super().__init__("path_recorder_dashboard")
        self.status, self.t = None, None
        self.create_subscription(String, "/path_recorder/status", self.on_status, 10)
        self.create_timer(1.0, self.draw)

    def on_status(self, msg):
        try:
            self.status, self.t = json.loads(msg.data), time.monotonic()
        except ValueError:
            pass

    def draw(self):
        age = 0.0 if self.t is None else time.monotonic() - self.t
        lines = render(self.status, age)
        sys.stdout.write("\033[2J\033[H" + time.strftime("%H:%M:%S  path_recorder dashboard (Ctrl-C quits)\n\n")
                         + "\n".join(lines) + "\n")
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = Dashboard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
