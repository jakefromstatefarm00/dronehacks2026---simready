"""Add a scripted "stay left" start to a v4c upload file (FlyX or Sky Beta).

    python3 jolie_course/patch_left_start.py upload_to_flyx_v4c.py upload_to_flyx_v4c_left.py
    python3 jolie_course/patch_left_start.py upload_to_flyx_v4c_skybeta_takeoff.py upload_to_flyx_v4c_skybeta_left.py

Why: on the real course the right-hand wall of the first corridor leans ~30 deg into it (the sim's
is upright). Straight up from the launch mark, the drone's right arms reach that wall at about
0.45 m. The 2-D planner at 0.9 m sees no gap wide enough there (about 0.48 m for a 0.56 m drone).

So, before the planner takes over:
  1. take off straight up to 0.35 m (instead of 0.9 m);
  2. slide 0.1 m left (course x -2.62: 0.38 m from the left wall, 10 cm beyond the drone's radius);
  3. fly up the corridor hugging the left wall, low (0.40 -> 0.45 m), until past the leaning
     wall's far end (y 2.45; the wall ends at 2.15);
  4. climb to the normal 0.9 m, then NAVIGATE: the unchanged v4c planner from there.
The setpoint moves along this path at 0.30 m/s and never more than 0.35 m ahead of the drone.
No planning or depth mapping happens during the script (the leaning wall never enters the map).
In the sim the walls are upright, so the path is free space there too. --start-script none
gives the original behaviour.
"""
import sys
from pathlib import Path

START = '''# Scripted start: course (x, y, z) waypoints after the takeoff climb (patch_left_start.py)
START_SCRIPTS = {
    "left": ((-2.62, 1.10, .40), (-2.62, 2.45, .45), (-2.62, 2.45, .90)),
    "none": (),
}


'''


def build(src, out):
    s = Path(src).read_text(encoding="utf-8").replace("\r\n", "\n")
    sky = "class SkybetaNode(Node)" in s

    def rep(a, b, label, count=1):
        nonlocal s
        n = s.count(a)
        assert n == count, f"{label}: expected {count} match(es), found {n}"
        s = s.replace(a, b)

    # ---------------------------------------------------------------- docstring
    assert s.startswith('"""'), "expected the module docstring first"
    s = ('"""STAY-LEFT START (jolie_course/patch_left_start.py): after takeoff it climbs only to 0.35 m,\n'
         'hugs the left wall of the first corridor low (0.40-0.45 m) past the leaning right wall, climbs\n'
         'to 0.9 m and then hands over to the unchanged planner. --start-script none = original.\n\n' + s[3:])

    # ---------------------------------------------------------------- mission config
    rep('''    slow_turn_deg: float = 90.0            # carrot shrinks to 30 % as the heading error reaches 70 % of this
''', '''    slow_turn_deg: float = 90.0            # carrot shrinks to 30 % as the heading error reaches 70 % of this
    # Scripted start (real course: the first corridor's right wall leans ~30 deg into it). After a
    # climb to start_script_takeoff_m, follow these course (x, y, z) waypoints, then NAVIGATE.
    start_script: tuple = ()
    start_script_takeoff_m: float = .35
    start_script_speed: float = .30        # m/s along the path
    start_script_lead_m: float = .35       # the setpoint never runs further ahead of the vehicle
    start_script_timeout_s: float = 45.0
    start_script_no_yaw: bool = False      # Sky Beta: no heading command until NAVIGATE
''', "config")
    rep('''        self.pose_course = None            # (xyz, yaw, rotation) for depth projection
''', '''        self.pose_course = None            # (xyz, yaw, rotation) for depth projection
        self._script_path = None           # scripted start: (N, 3) course path, progress, heading
        self._script_s = self._script_t = self._script_t0 = self._script_yaw = None
''', "init")

    # ---------------------------------------------------------------- mission helpers
    rep('''    def _failsafe_hold(self):
''', '''    def _takeoff_alt(self):
        return self.cfg.start_script_takeoff_m if self.cfg.start_script else self.cfg.altitude

    def _after_takeoff(self):
        return "SCRIPT" if self.cfg.start_script else "NAVIGATE"

    def _script_step(self, tel, now):
        """Scripted start: move the setpoint along start_script at start_script_speed, at most
        start_script_lead_m ahead of the vehicle, holding the takeoff heading; then NAVIGATE."""
        c = self.cfg
        if self._script_path is None:
            first = np.array([*self._takeoff_xy, self._takeoff_alt()], float)
            self._script_path = np.vstack([first, np.asarray(c.start_script, float)])
            self._script_s, self._script_t, self._script_t0 = 0.0, now, now
            self._script_yaw = float(self.pose_course[1])
        path = self._script_path
        seg = np.diff(path, axis=0)
        lengths = np.linalg.norm(seg, axis=1)
        total = float(lengths.sum())

        def at(s):
            for p0, d, n in zip(path[:-1], seg, lengths):
                if s <= n:
                    return p0 + d * (s / n if n > 0 else 0.0)
                s -= n
            return path[-1].copy()

        here = np.asarray(self.pose_course[0], float)
        dt = max(0.0, min(now - self._script_t, .2))
        self._script_t = now
        s_try = min(total, self._script_s + c.start_script_speed * dt)
        if np.linalg.norm(at(s_try) - here) <= c.start_script_lead_m:
            self._script_s = s_try                     # else wait for the vehicle to catch up
        target = at(self._script_s)
        if self._script_s >= total and np.linalg.norm(here - path[-1]) < .12:
            self._go("NAVIGATE", now, "start script done: the planner takes over")
        elif now - self._script_t0 > c.start_script_timeout_s:
            return self._land(now, f"start script stuck at {np.round(here, 2).tolist()} "
                                   f"(setpoint {np.round(target, 2).tolist()})")
        sp = course_to_ned_setpoint(self.frame, target[:2], float(target[2]), self._script_yaw)
        sp = (sp[0], float("nan")) if c.start_script_no_yaw else self._limit_yaw(sp, tel)
        self._last_sp = sp
        return Output(self.state, sp, note=f"start script {self._script_s:.2f}/{total:.2f} m")

    def _failsafe_hold(self):
''', "helpers")

    # ---------------------------------------------------------------- mission tick
    rep('''        if self.state in ("TAKEOFF", "NAVIGATE"):
            guard = self._airborne_guard(tel, healthy)
            if guard:
                return guard
''', '''        if self.state in ("TAKEOFF", "SCRIPT", "NAVIGATE"):
            guard = self._airborne_guard(tel, healthy)
            if guard:
                return guard
            if self.state == "SCRIPT":
                return self._script_step(tel, now)
''', "tick")
    rep("off = abs(self.pose_course[0][2] - c.altitude)", "off = abs(self.pose_course[0][2] - self._takeoff_alt())",
        "takeoff check")
    rep('self._go("NAVIGATE", now, "at altitude")', 'self._go(self._after_takeoff(), now, "at altitude")', "at alt")
    rep('self._go("NAVIGATE", now, f"near altitude after', 'self._go(self._after_takeoff(), now, f"near altitude after',
        "near alt")
    if sky:
        rep("alt = min(c.altitude, self._takeoff_z0", "alt = min(self._takeoff_alt(), self._takeoff_z0", "ramp")
        rep('mission.state in ("TAKEOFF", "NAVIGATE") and not self.vision_lost',
            'mission.state in ("TAKEOFF", "SCRIPT", "NAVIGATE") and not self.vision_lost', "jump check")
        rep('flying = st in ("TAKEOFF", "NAVIGATE")', 'flying = st in ("TAKEOFF", "SCRIPT", "NAVIGATE")', "vision guard")
        rep('''        ack_timeout_s=1e9))''', '''        ack_timeout_s=1e9, start_script=START_SCRIPTS[a.start_script], start_script_no_yaw=True))''',
            "mission config call")
    else:
        rep("sp = self._course_setpoint(self._takeoff_xy, yaw)",
            "sp = course_to_ned_setpoint(self.frame, self._takeoff_xy, self._takeoff_alt(), yaw)", "takeoff sp")
        rep('mission.state in ("WAIT_ARM", "TAKEOFF", "NAVIGATE")', 'mission.state in ("WAIT_ARM", "TAKEOFF", "SCRIPT", "NAVIGATE")',
            "land if airborne")
        rep('''launch_body_agl=a.launch_body_agl, pad_xy=tuple(planner.pad_xy)))''',
            '''launch_body_agl=a.launch_body_agl, pad_xy=tuple(planner.pad_xy),
        start_script=START_SCRIPTS[a.start_script]))''', "mission config call")

    # ---------------------------------------------------------------- command line
    rep('''    ap.add_argument("--launch-body-agl", type=float, default=.05)
''', '''    ap.add_argument("--launch-body-agl", type=float, default=.05)
    ap.add_argument("--start-script", default="left", choices=sorted(START_SCRIPTS),
                    help="left: stay left low past the leaning wall, then the planner; none: original")
''', "cli")
    rep("\ndef parse_args(argv=None):", "\n" + START + "def parse_args(argv=None):", "start scripts")

    compile(s, out, "exec")
    Path(out).write_text(s, encoding="utf-8", newline="\n")
    print(f"wrote {out} ({'Sky Beta' if sky else 'FlyX'})")


if __name__ == "__main__":
    here = Path(__file__).resolve().parent
    build(here / sys.argv[1], here / sys.argv[2])
