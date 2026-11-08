"""Build upload_to_flyx_v4c_skybeta_takeoff.py: the v4c Sky Beta copy with the real-drone takeoff fixes.

    python3 jolie_course/patch_skybeta_takeoff.py

Reads upload_to_flyx_v4c_skybeta.py (never written) and applies exact, asserted text edits.

Why: the v4c FlyX file flew 3/3 in the sim, but its Sky Beta copy lifted off, spun and hit the net on
the real drone. The planner, mission and frame maths are byte-identical and correct (checked: our
ENU/NED quaternions match the organizer's bridge to 1e-13, and a "hold here" setpoint reaches PX4 as
exactly the drone's own position and heading). What differs is where the feedback comes from. In
FlyX the pose that v4c re-commanded every tick ("hold here", "face your current heading") was PX4's
OWN estimate, so re-commanding it was a no-op. On the drone it is cuVSLAM's, while PX4 steers by
EKF2. Any difference between them is commanded again every tick, and the drone chases it: a steady
spin, plus a slide for any position offset (the interface's anchor_setpoints is off).

Real flight log_31 (PX4 ULog): the frames DID agree at takeoff. cuVSLAM stopped 0.4 s after liftoff,
EKF2 fell back to the magnetometer for heading (a -126 deg reset), and PX4 spun the drone at its
200 deg/s limit toward the heading it had been commanded. So the takeoff now sends no heading at all.

Fixes (each a no-op when the two estimates agree, so sim behaviour is unchanged):
  1. Mission, on the ground: the hold setpoint is re-measured only while disarmed and FROZEN from
     arming on (Sky Beta: the pilot arms before Offboard starts).
  2. Mission, takeoff: straight up at the frozen ground spot, with NO heading command (yaw NaN, also
     on the ground: a heading reset can no longer become a spin). The altitude setpoint
     rises at 0.30 m/s (it used to jump to 0.9 m at once). The NAVIGATE yaw limiter continues from
     the commanded heading, not the measured one.
  3. Mission, takeoff guard: more than 0.5 m from the takeoff spot, or turned more than 45 deg ->
     stop (LAND: no new setpoints, the interface hovers, the pilot takes over). This stops the drone
     flying the course from a broken pose.
  4. Mission, holds in NAVIGATE (no plan yet, pad centring): keep the last commanded heading,
     not the measured one.
  5. Node: measure EKF2's position minus cuVSLAM's, from /skybeta/state's local_enu and odom_enu,
     while disarmed. Freeze it at arming and add it to every outgoing pose: the setpoint then means
     the same spot to PX4. If they disagree by more than 1 m, publish nothing (the interface then
     never starts Offboard, so the drone stays on the ground) and say why.
  7. Node, vision guard: no setpoint at all (so no Offboard, no takeoff) until cuVSLAM has run
     steadily for 5 s (>= 15 Hz, gaps <= 0.25 s). In the air, no cuVSLAM for 0.5 s, a pose jump
     (> 0.25 m or 25 deg between frames), or the takeoff guard -> straight down at 0.3 m/s on a
     velocity setpoint (rangefinder height), x/y 0, no heading, until disarmed.
  6. Node: journal "diag" lines at 2 Hz (state, cuVSLAM vs EKF2 position, sent setpoint, cuVSLAM vs
     PX4 heading if px4_msgs is installed - read-only), plus takeoff lines in the end-of-run summary.
"""
from pathlib import Path

HERE = Path(__file__).resolve().parent
SRC, OUT = HERE / "upload_to_flyx_v4c_skybeta.py", HERE / "upload_to_flyx_v4c_skybeta_takeoff.py"


def build():
    s = SRC.read_text(encoding="utf-8").replace("\r\n", "\n")

    def rep(a, b, label, count=1):
        nonlocal s
        n = s.count(a)
        assert n == count, f"{label}: expected {count} match(es), found {n}"
        s = s.replace(a, b)

    # ------------------------------------------------------------------ docstring
    rep('''"""Jolie's FlyX-proven planner (upload_to_flyx.py) on the Sky Beta contract - the drone's interface.
''', '''"""Jolie's FlyX-proven planner (upload_to_flyx.py) on the Sky Beta contract - the drone's interface.

REAL-DRONE TAKEOFF VERSION: built by jolie_course/patch_skybeta_takeoff.py from
upload_to_flyx_v4c_skybeta.py (see that script for what changed and why). On the ground it holds the
spot and heading frozen at arming. It climbs straight up at 0.30 m/s, stops if takeoff goes wrong,
and anchors setpoints on EKF2's position.
''', "docstring")

    # ------------------------------------------------------------------ mission: config
    rep('''    takeoff_loose_m: float = .25
''', '''    takeoff_loose_m: float = .25
    # Real drone (Sky Beta): straight up from the spot and heading frozen at arming, setpoint rising
    # at takeoff_climb_mps from takeoff_start_m (the interface floors every pose at 0.3 m above the
    # start). Drifting farther than takeoff_drift_m or turning more than takeoff_yaw_dev_deg during
    # takeoff stops the mission (no new setpoints: the interface hovers; the pilot takes over).
    takeoff_climb_mps: float = .30
    takeoff_start_m: float = .35
    takeoff_drift_m: float = .50
    takeoff_yaw_dev_deg: float = 45.0
''', "config")

    # ------------------------------------------------------------------ mission: state
    rep('''        self._takeoff_xy = None
        self._t_takeoff = None
''', '''        self._takeoff_xy = None
        self._takeoff_yaw = None           # course heading frozen at arming
        self._takeoff_z0 = None            # course altitude the climb ramp starts from
        self._ground_hold = None           # (NED setpoint, course xyz, course yaw): re-measured until armed
        self.takeoff_report = {}           # worst drift / turn during takeoff (end-of-run summary)
        self._t_takeoff = None
''', "init")

    # ------------------------------------------------------------------ mission: ground hold
    rep("self._hold_here(tel)", "self._ground_hold_sp(tel)", "ground holds", count=5)
    rep('''    def _course_setpoint(self, xy, yaw):
''', '''    def _ground_hold_sp(self, tel):
        """On the ground before takeoff: hold where the vehicle sits. Re-measured while disarmed and
        frozen from arming on. On Sky Beta the pilot arms first, and the interface lifts to its 0.3 m
        floor the moment Offboard starts. Re-holding "here" from cuVSLAM then turned any cuVSLAM /
        EKF2 difference into a target that moved with the vehicle."""
        if self._ground_hold is None or tel.arming_state != ARMING_STATE_ARMED:
            self._ground_hold = (self._hold_here(tel), np.asarray(self.pose_course[0], float).copy(),
                                 float(self.pose_course[1]))
        return (self._ground_hold[0][0], float("nan"))     # no heading command on the ground either

    def _held_yaw_course(self):
        """Heading to keep when nothing asks for a turn: the last COMMANDED one (course frame).
        Re-commanding the measured heading chases any cuVSLAM / EKF2 heading difference."""
        if self._sp_yaw is None:
            return self.pose_course[1]
        return float(self.frame.north_heading_in_course - self._sp_yaw)

    def _course_setpoint(self, xy, yaw):
''', "helpers")

    # ------------------------------------------------------------------ mission: takeoff
    rep('''                self._takeoff_xy = self.pose_course[0][:2].copy()
''', '''                if self._ground_hold is None:
                    self._ground_hold = (self._hold_here(tel), np.asarray(self.pose_course[0], float).copy(),
                                         float(self.pose_course[1]))
                _, ground_xyz, ground_yaw = self._ground_hold
                self._takeoff_xy = ground_xyz[:2].copy()
                self._takeoff_yaw = ground_yaw
                self._takeoff_z0 = max(float(ground_xyz[2]), c.takeoff_start_m)
                self.takeoff_report = {"xy": [round(float(v), 3) for v in self._takeoff_xy],
                                       "yaw_deg": round(float(np.rad2deg(ground_yaw)), 1),
                                       "max_drift_m": 0.0, "max_turn_deg": 0.0}
''', "takeoff latch")
    rep('''            if self.state == "TAKEOFF":
                yaw = self.pose_course[1]
                sp = self._course_setpoint(self._takeoff_xy, yaw)
                off = abs(self.pose_course[0][2] - c.altitude)
''', '''            if self.state == "TAKEOFF":
                # Straight up at the spot and heading frozen at arming, the altitude setpoint rising at
                # takeoff_climb_mps. (Was: the measured heading, re-commanded every tick, and the full
                # altitude at once. In FlyX that heading was PX4's own estimate; on the drone it is
                # cuVSLAM's, and its difference from EKF2's was commanded again every tick.)
                alt = min(c.altitude, self._takeoff_z0 + c.takeoff_climb_mps * (now - self._t_state))
                sp = course_to_ned_setpoint(self.frame, self._takeoff_xy, alt, self._takeoff_yaw)
                self._sp_yaw, self._sp_yaw_t = sp[1], now          # NAVIGATE's yaw limiter starts here
                # No heading command while taking off (yaw NaN: PX4 keeps whatever heading it has).
                # Real drone, log_31: cuVSLAM stopped 0.4 s after liftoff, EKF2 re-aligned its heading to
                # the magnetometer (-126 deg jump) and PX4 spun at 200 deg/s to the commanded heading.
                sp = (sp[0], float("nan"))
                self._last_sp = sp
                drift = float(np.hypot(*(np.asarray(self.pose_course[0][:2]) - self._takeoff_xy)))
                dyaw = self.pose_course[1] - self._takeoff_yaw
                turned = abs(float(np.arctan2(np.sin(dyaw), np.cos(dyaw))))
                rep = self.takeoff_report
                rep["max_drift_m"] = round(max(rep.get("max_drift_m", 0.0), drift), 3)
                rep["max_turn_deg"] = round(max(rep.get("max_turn_deg", 0.0), float(np.rad2deg(turned))), 1)
                if drift > c.takeoff_drift_m or turned > np.deg2rad(c.takeoff_yaw_dev_deg):
                    return self._land(now, f"takeoff unstable: {drift:.2f} m from the takeoff spot, turned "
                                           f"{np.rad2deg(turned):.0f} deg; holding, pilot: take over")
                off = abs(self.pose_course[0][2] - c.altitude)
''', "takeoff")
    rep('''        if self.state == "TAKEOFF" and self._takeoff_xy is not None:
            return self._course_setpoint(self._takeoff_xy, self.pose_course[1])
''', '''        if self.state == "TAKEOFF" and self._takeoff_xy is not None:
            return self._course_setpoint(self._takeoff_xy, self._takeoff_yaw)
''', "failsafe hold")

    # ------------------------------------------------------------------ mission: NAVIGATE holds
    rep('''                sp = self._limit_yaw(self._course_setpoint(self.pose_course[0][:2], self.pose_course[1]), tel)
''', '''                sp = self._limit_yaw(self._course_setpoint(self.pose_course[0][:2], self._held_yaw_course()), tel)
''', "no-plan hold")
    rep('''        sp = self._limit_yaw(self._course_setpoint(c.pad_xy, self.pose_course[1]), tel)
''', '''        sp = self._limit_yaw(self._course_setpoint(c.pad_xy, self._held_yaw_course()), tel)
''', "pad centring")

    # ------------------------------------------------------------------ node: state
    rep('''            self.last_sky = {}
''', '''            self.last_sky = {}
            # EKF2 (PX4 local, /skybeta/state local_enu) minus cuVSLAM (odom_enu), measured on the
            # ground and frozen at arming; added to every outgoing pose (the interface's
            # anchor_setpoints is off, so it sends our cuVSLAM-frame pose to PX4 unchanged).
            self.ekf_offset, self.offset_n, self.offset_frozen, self.block = np.zeros(3), 0, False, ""
            self.offset_now = None                  # latest local_enu - odom_enu (diagnostic)
            self.offset_max_dev = 0.0               # in flight: worst change from the frozen offset
            self.ekf_yaw, self.yaw_gap_max = None, 0.0   # PX4 heading (NED), worst |PX4 - cuVSLAM|
            self.yaw_gap_at_arm = None
            self.last_sent, self.diag_t = None, 0.0
''', "node init")
    rep('''            self.create_subscription(String, "/skybeta/state", self.on_state, 10, callback_group=sensors)
''', '''            self.create_subscription(String, "/skybeta/state", self.on_state, 10, callback_group=sensors)
            try:                                # read-only diagnostic: PX4's own heading, if available
                from px4_msgs.msg import VehicleAttitude
                self.create_subscription(VehicleAttitude, "/fmu/out/vehicle_attitude", self.on_attitude,
                                         qos_profile_sensor_data, callback_group=sensors)
            except ImportError:
                self.get_logger().info("px4_msgs not installed: PX4 heading is not logged (optional)")
''', "attitude sub")

    rep('''            if st.get("clamped") and st.get("clamped") != self.last_sky.get("clamped"):
''', '''            odom, local = st.get("odom_enu"), st.get("local_enu")
            if odom is not None and local is not None:
                d = np.asarray(local, float) - np.asarray(odom, float)
                if np.isfinite(d).all():
                    self.offset_now = d
                    if not armed:                   # on the ground: average it (it is constant there)
                        self.ekf_offset = d if self.offset_n == 0 else .8 * self.ekf_offset + .2 * d
                        self.offset_n += 1
                    elif self.offset_frozen:
                        self.offset_max_dev = max(self.offset_max_dev, float(np.linalg.norm(d - self.ekf_offset)))
            if armed and not self.offset_frozen:
                self.offset_frozen = True
                n = float(np.linalg.norm(self.ekf_offset))
                self.yaw_gap_at_arm = self.yaw_gap()
                log({"kind": "frame_offset", "t": self.now(), "ekf_minus_vslam_enu": list(self.ekf_offset),
                     "samples": self.offset_n, "norm_m": n, "yaw_gap_deg": self.yaw_gap_at_arm})
                msg_ = (f"armed: EKF2 - cuVSLAM position offset {np.round(self.ekf_offset, 3).tolist()} m "
                        f"({self.offset_n} samples), applied to every setpoint")
                if self.yaw_gap_at_arm is not None:
                    msg_ += f"; PX4 vs cuVSLAM heading {self.yaw_gap_at_arm:+.1f} deg"
                if self.offset_n == 0:
                    self.get_logger().warn("no local_enu / odom_enu from the interface while disarmed: "
                                           "setpoints are NOT anchored on EKF2 (the old behaviour)")
                elif n > 1.0:
                    self.block = f"EKF2 and cuVSLAM disagree by {n:.2f} m"
                if self.block:
                    self.get_logger().error(f"NOT FLYING: {self.block}. Disarm; check VIO / EKF2 and restart.")
                    log({"kind": "blocked", "t": self.now(), "why": self.block})
                if self.block or self.offset_n == 0:
                    pass
                elif n > .15 or (self.yaw_gap_at_arm is not None and abs(self.yaw_gap_at_arm) > 10):
                    self.get_logger().warn(msg_ + "  <- larger than expected")
                else:
                    self.get_logger().info(msg_)
            if not armed and self.offset_frozen:    # disarmed again: measure afresh for the next flight
                self.offset_frozen, self.block, self.offset_max_dev = False, "", 0.0
            if st.get("clamped") and st.get("clamped") != self.last_sky.get("clamped"):
''', "offset")

    rep('''        def on_info(self, msg):
            if self.intr is None:
                self.intr = list(msg.k)
''', '''        def on_attitude(self, msg):
            try:
                self.ekf_yaw = yaw_from_q_ned(tuple(float(v) for v in msg.q))
            except (TypeError, ValueError):
                return
            gap = self.yaw_gap()
            if gap is not None and self.offset_frozen:
                self.yaw_gap_max = max(self.yaw_gap_max, abs(gap))

        def yaw_gap(self):
            """PX4 heading minus cuVSLAM heading, deg (None without px4_msgs / odometry)."""
            if self.ekf_yaw is None or self.tel.odom_t is None:
                return None
            d = self.ekf_yaw - yaw_from_q_ned(self.tel.q_ned_frd)
            return round(math.degrees(math.atan2(math.sin(d), math.cos(d))), 1)

        def on_info(self, msg):
            if self.intr is None:
                self.intr = list(msg.k)
''', "attitude cb")

    # ------------------------------------------------------------------ node: publish
    rep('''            if out.setpoint is None or self.sp_pub is None:
                return                                         # pose setpoints persist: the interface holds
            pos, yaw = out.setpoint
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "odom"
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = enu_to_ned(pos)   # same map back
            yaw_enu = math.pi / 2 - float(yaw)
''', '''            if mono - self.diag_t >= .5:                       # 2 Hz: is PX4 where cuVSLAM says?
                self.diag_t = mono
                p = self.tel.position_ned
                log({"kind": "diag", "t": sim, "state": out.state,
                     "vslam_enu": [round(p[1], 3), round(p[0], 3), round(-p[2], 3)],
                     "local_enu": self.last_sky.get("local_enu"),
                     "ekf_minus_vslam": None if self.offset_now is None else [round(float(v), 3) for v in self.offset_now],
                     "offset_applied": [round(float(v), 3) for v in self.ekf_offset],
                     "sent_enu": self.last_sent, "yaw_gap_deg": self.yaw_gap(),
                     "nav": self.last_sky.get("nav_state_name"), "clamped": self.last_sky.get("clamped")})
            if out.setpoint is None or self.sp_pub is None:
                return                                         # pose setpoints persist: the interface holds
            if self.block:
                return                                         # never send: no Offboard, stays on the ground
            pos, yaw = out.setpoint
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "odom"
            enu = np.asarray(enu_to_ned(pos), float) + self.ekf_offset   # same map back, onto EKF2's frame
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (float(v) for v in enu)
            self.last_sent = [round(float(v), 3) for v in enu]
            yaw_enu = math.pi / 2 - float(yaw)
''', "publish")

    rep('''            msg.pose.orientation.w, msg.pose.orientation.z = math.cos(yaw_enu / 2), math.sin(yaw_enu / 2)
''', '''            if math.isfinite(yaw_enu):
                msg.pose.orientation.w, msg.pose.orientation.z = math.cos(yaw_enu / 2), math.sin(yaw_enu / 2)
            else:                       # all-zero quaternion: the interface sends yaw NaN = keep heading
                msg.pose.orientation.w = msg.pose.orientation.z = 0.0
''', "no-yaw quaternion")

    # ------------------------------------------------------------------ summary
    rep('''    if node.stalls:
        lines.append(f"program stalls > 0.15 s: {len(node.stalls)}, worst {max(node.stalls):.2f} s")
''', '''    if node.stalls:
        lines.append(f"program stalls > 0.15 s: {len(node.stalls)}, worst {max(node.stalls):.2f} s")
    if mission.takeoff_report:
        r = mission.takeoff_report
        lines.append(f"takeoff: worst drift {r['max_drift_m']:.2f} m from the spot, worst turn {r['max_turn_deg']:.0f} deg "
                     f"(stops above {mission.cfg.takeoff_drift_m} m / {mission.cfg.takeoff_yaw_dev_deg:.0f} deg)")
    if node.offset_frozen or node.offset_n:
        lines.append(f"EKF2 - cuVSLAM offset applied {np.round(node.ekf_offset, 3).tolist()} m; "
                     f"in flight it changed by up to {node.offset_max_dev:.2f} m")
    if node.yaw_gap_at_arm is not None or node.yaw_gap_max:
        lines.append(f"PX4 vs cuVSLAM heading: {node.yaw_gap_at_arm} deg at arming, worst {node.yaw_gap_max:.1f} deg in flight")
    if node.block:
        lines.append(f"NOT FLOWN: {node.block}")
''', "summary")


    # ------------------------------------------------------------------ vision guard (log_31)
    # Real flight log_31: vibration jumped from 0.5 to 9 m/s2 as the motors spun up, cuVSLAM stopped
    # 0.4 s after liftoff, EKF2 switched its heading to the compass (-126 deg) and dead-reckoned.
    # Holding a position in a dead-reckoned frame slid the drone into the net. So: no takeoff until
    # cuVSLAM is steady, and in the air a lost / jumping cuVSLAM -> straight down on a velocity
    # setpoint (height comes from the rangefinder, not vision) with no heading control.
    rep("""ARMED, DISARMED, NAV_OFFBOARD = 2, 1, 14
""", """ARMED, DISARMED, NAV_OFFBOARD = 2, 1, 14
VISION_MIN_HZ, VISION_MAX_GAP_S = 15.0, .25     # before takeoff: cuVSLAM this steady for 5 s, else no setpoints
VISION_LOST_S = .5                               # in the air: no cuVSLAM odometry this long -> descend
VISION_JUMP_M, VISION_JUMP_DEG = .25, 25.0       # or a pose jump this big between two frames -> descend
DESCEND_MPS, DESCEND_SLOW_MPS = .30, .15         # straight down (after the interface reports landed: slow)
""", "vision constants")
    rep("""    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String

    # Same mission as the FlyX file.""", """    from geometry_msgs.msg import PoseStamped, TwistStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String

    # Same mission as the FlyX file.""", "twist import")
    rep("""            self.last_sent, self.diag_t = None, 0.0
""", """            self.last_sent, self.diag_t = None, 0.0
            self.odom_times = deque(maxlen=600)     # cuVSLAM arrival times (steadiness before takeoff)
            self.prev_odom = None                   # (t, ENU position, NED yaw): pose-jump check
            self.vision_lost, self.descending, self.stale_ticks, self.vision_wait_t = "", False, 0, -1e9
            self.twist_pub = (self.create_publisher(TwistStamped, "/skybeta/setpoint/twist", 10)
                              if a.execute else None)
""", "vision init")
    rep("""            tel.odom_t, tel.position_ned, tel.q_ned_frd = t, enu_to_ned((p.x, p.y, p.z)), q
            stamp = msg.header.stamp.sec""", """            tel.odom_t, tel.position_ned, tel.q_ned_frd = t, enu_to_ned((p.x, p.y, p.z)), q
            yaw_now, here = yaw_from_q_ned(q), (float(p.x), float(p.y), float(p.z))
            if self.prev_odom is not None and mission.state in ("TAKEOFF", "NAVIGATE") and not self.vision_lost:
                pt, pp, py = self.prev_odom
                jump = math.dist(pp, here)
                dyaw = abs(math.degrees(math.atan2(math.sin(yaw_now - py), math.cos(yaw_now - py))))
                if t - pt < .25 and (jump > VISION_JUMP_M or dyaw > VISION_JUMP_DEG):
                    self.vision_lost = f"cuVSLAM pose jumped {jump:.2f} m / {dyaw:.0f} deg in {t - pt:.2f} s"
            self.prev_odom = (t, here, yaw_now)
            self.odom_times.append(t)
            stamp = msg.header.stamp.sec""", "pose jump")
    rep("""        def on_attitude(self, msg):
""", """        def vision_steady(self, now):
            \"\"\"(ok, rate Hz, worst gap s) of cuVSLAM odometry over the last 5 s.\"\"\"
            ts = [x for x in self.odom_times if now - x <= 5.0]
            if len(ts) < 2 or now - ts[0] < 4.5 or now - ts[-1] > .15:
                return False, len(ts) / 5.0, None
            gap = max(b - a for a, b in zip(ts, ts[1:]))
            rate = len(ts) / 5.0
            return rate >= VISION_MIN_HZ and gap <= VISION_MAX_GAP_S, rate, gap

        def vision_guard(self, out, now):
            \"\"\"Before takeoff: publish nothing until cuVSLAM is steady (no setpoint -> the interface
            cannot start Offboard -> no takeoff). In the air: cuVSLAM lost or jumping (or the takeoff
            guard tripped) -> straight down on a velocity setpoint, no heading control, until disarmed.\"\"\"
            st = out.state
            if st in ("STREAM", "WAIT_OFFBOARD", "WAIT_ARM") and out.setpoint is not None:
                ok, rate, gap = self.vision_steady(now)
                if ok:
                    return out
                if now - self.vision_wait_t > 3.0 and self.odom_times and now - self.odom_times[0] >= 4.5:
                    self.vision_wait_t = now
                    why = f"cuVSLAM {rate:.0f} Hz" + ("" if gap is None else f", worst gap {gap:.2f} s")
                    self.get_logger().warn(f"NOT READY TO TAKE OFF: vision not steady ({why}); sending nothing, so "
                                           f"Offboard cannot start. Needs >= {VISION_MIN_HZ:.0f} Hz with gaps <= "
                                           f"{VISION_MAX_GAP_S} s for 5 s (if the pilot already switched: disarm, re-arm).")
                    log({"kind": "vision_wait", "t": self.now(), "rate_hz": rate, "worst_gap_s": gap})
                return type(out)(st, None, out.command, "waiting for steady vision")
            flying = st in ("TAKEOFF", "NAVIGATE")
            age = now - self.tel.odom_t if self.tel.odom_t is not None else 1e9
            self.stale_ticks = self.stale_ticks + 1 if (flying and age > VISION_LOST_S) else 0
            if flying and not self.vision_lost and self.stale_ticks >= 5:      # 5 live ticks: not our own stall
                self.vision_lost = f"no cuVSLAM odometry for {age:.2f} s"
            if flying and self.vision_lost:
                mission._land(now, f"vision lost ({self.vision_lost}): descending straight down")
                st = mission.state
            if (st == "LAND" and not self.descending and mission.events
                    and (self.vision_lost or mission.events[-1]["note"].startswith("takeoff unstable"))):
                self.descending = True
                why = self.vision_lost or mission.events[-1]["note"]
                self.get_logger().error(f"{why}: DESCENDING straight down at {DESCEND_MPS} m/s, no heading "
                                        "control. Pilot: disarm on touchdown (or take over).")
                log({"kind": "descend", "t": self.now(), "why": why})
            if self.descending:
                if self.tel.arming_state == ARMED and self.twist_pub is not None and not self.block:
                    m = TwistStamped()
                    m.header.stamp = self.get_clock().now().to_msg()
                    m.header.frame_id = "odom"
                    m.twist.linear.z = -(DESCEND_SLOW_MPS if self.last_sky.get("landed") else DESCEND_MPS)
                    self.twist_pub.publish(m)              # x, y = 0, yaw rate 0: no heading command
                return type(out)(mission.state, None, None, "descending: vision lost")
            return out

        def on_attitude(self, msg):
""", "vision guard")
    rep("""            out = mission.tick(self.tel)
""", """            out = self.vision_guard(mission.tick(self.tel), mono)
""", "guard call")
    rep("""    if node.block:
        lines.append(f"NOT FLOWN: {node.block}")
""", """    if node.block:
        lines.append(f"NOT FLOWN: {node.block}")
    if node.descending:
        lines.append(f"DESCENDED (vision guard): {node.vision_lost or 'takeoff unstable'}")
""", "summary vision")

    compile(s, str(OUT), "exec")
    OUT.write_text(s, encoding="utf-8", newline="\n")
    print(f"wrote {OUT.name} ({len(s) / 1024:.0f} KB) from {SRC.name}")


if __name__ == "__main__":
    build()
