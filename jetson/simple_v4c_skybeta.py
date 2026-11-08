"""simple_v4c_skybeta.py -- physical autonomy test: take off vertically, hover 5 s, land slowly.

Sky Beta contract only (no /fmu): reads /visual_slam/tracking/odometry and /skybeta/state,
writes /skybeta/setpoint/pose and /skybeta/setpoint/twist. The organizer's flight_interface does
Offboard; the safety pilot arms. It never arms, never changes mode.

    pilot: arm with the mode switch NOT on Position, then flip to Position on the ground
    CLIMB     straight up at 0.25 m/s to 1.0 m above the start (x, y, heading held)
    HOVER     5 s at 1.0 m
    DESCEND   straight down at 0.15 m/s to 0.40 m
    TOUCHDOWN straight down at 0.10 m/s (velocity setpoint) until PX4 reports landed,
              then keeps a gentle 0.10 m/s down so PX4 idles the motors and auto-disarms;
              the pilot disarms if PX4 does not within 15 s
    Before touchdown: odometry or /skybeta/state stale -> stop sending (the flight interface holds).
    During touchdown: keep the slow down command regardless (stopping would make the flight
              interface climb back to the last pose setpoint, 0.40 m).
    Any time: the pilot takes over with the mode switch -> stop for good.
    After touchdown the pilot should disarm promptly if PX4 has not auto-disarmed: if this program
    is killed while still armed on the ground, the flight interface falls back to the last pose
    setpoint (0.40 m) and would lift off again.

    python3 simple_v4c_skybeta.py            # run in the competitor window, nothing else running
    python3 simple_v4c_skybeta.py --shadow   # bench check: prints what it would do, sends nothing
"""
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

HOVER_ALT, HOVER_S = 1.0, 5.0              # m above the start, s
CLIMB_V, DESCEND_V, TOUCH_V = 0.25, 0.15, 0.10   # m/s
TOUCH_FROM = 0.40                          # m above the start: switch to the velocity touchdown
AT = 0.15                                  # m: "at altitude" tolerance
ODOM_STALE, STATE_STALE = 0.5, 1.0         # s
DISARM_WAIT = 15.0                         # s after landed before asking the pilot to disarm
SHADOW = "--shadow" in sys.argv


class SimpleTest(Node):
    def __init__(self):
        super().__init__("simple_test")
        self.pose_pub = self.create_publisher(PoseStamped, "/skybeta/setpoint/pose", 10)
        self.twist_pub = self.create_publisher(TwistStamped, "/skybeta/setpoint/twist", 10)
        self.create_subscription(Odometry, "/visual_slam/tracking/odometry", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(String, "/skybeta/state", self.on_state, 10)
        self.odom = self.odom_t = self.state = self.state_t = None
        self.phase, self.start, self.target_z, self.t_phase = "WAIT", None, 0.0, time.monotonic()
        self.was_offboard = False
        self.create_timer(0.1, self.tick)       # 10 Hz
        self.say(f"up ({'SHADOW: sends nothing' if SHADOW else 'LIVE'}). Pilot: arm (not on Position), then Position.")

    def say(self, msg):
        self.get_logger().info(f"[{self.phase}] {msg}")

    def on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.odom, self.odom_t = (p.x, p.y, p.z, yaw), time.monotonic()

    def on_state(self, m):
        try:
            self.state, self.state_t = json.loads(m.data), time.monotonic()
        except ValueError:
            pass

    def go(self, phase, msg):
        self.phase, self.t_phase = phase, time.monotonic()
        self.say(msg)

    def send_pose(self, x, y, z, yaw):
        if SHADOW:
            return
        m = PoseStamped()
        m.header.stamp, m.header.frame_id = self.get_clock().now().to_msg(), "odom"
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.pose_pub.publish(m)

    def send_down(self, v):
        if SHADOW:
            return
        m = TwistStamped()
        m.header.stamp, m.header.frame_id = self.get_clock().now().to_msg(), "odom"
        m.twist.linear.z = -abs(v)            # straight down; x, y = 0; no yaw rate
        self.twist_pub.publish(m)

    def tick(self):
        now = time.monotonic()
        if self.phase in ("DONE", "PILOT"):
            return
        fresh = (self.odom_t is not None and now - self.odom_t < ODOM_STALE and
                 self.state_t is not None and now - self.state_t < STATE_STALE)
        st = self.state or {}
        armed, offboard, landed = bool(st.get("armed")), bool(st.get("offboard_active")), bool(st.get("landed"))

        if self.was_offboard and not offboard:            # pilot switched mode, or PX4 disarmed
            if armed:
                self.go("PILOT", "Offboard lost: the pilot has the drone. Stopping for good.")
            else:
                self.go("DONE", "disarmed. Test complete.")
            return
        if self.phase in ("TOUCHDOWN", "LANDED"):
            # Keep the slow down command going even if odometry/state stall: PX4 flies velocity on its
            # own estimate, and stopping here would make the flight interface fall back to the last pose
            # setpoint (0.40 m) -- climbing back up, or lifting off again after touchdown.
            self.send_down(TOUCH_V)
            if self.phase == "TOUCHDOWN" and fresh and landed:
                self.go("LANDED", "PX4 reports landed. Holding gentle down command until disarm.")
            elif self.phase == "LANDED" and now - self.t_phase > DISARM_WAIT:
                self.say("still armed: PILOT, DISARM NOW.")
                self.t_phase = now
            return
        if not fresh:
            if self.phase != "WAIT":
                self.say("odometry/state stale: sending nothing (the flight interface holds). Pilot: be ready.")
            return

        x, y, z, yaw = self.odom
        if self.phase == "WAIT":
            self.send_pose(x, y, z, yaw)                  # hold where we are; lets Offboard start
            if offboard:
                self.was_offboard, self.start, self.target_z = True, (x, y, z, yaw), 0.0
                self.go("CLIMB", f"Offboard active. Climbing to {HOVER_ALT} m.")
            return

        x0, y0, z0, yaw0 = self.start
        h = z - z0                                        # height above the start
        if self.phase == "CLIMB":
            self.target_z = min(HOVER_ALT, self.target_z + CLIMB_V * 0.1)
            self.send_pose(x0, y0, z0 + self.target_z, yaw0)
            if self.target_z >= HOVER_ALT and abs(h - HOVER_ALT) < AT:
                self.go("HOVER", f"at {h:.2f} m. Hovering {HOVER_S:.0f} s.")
        elif self.phase == "HOVER":
            self.send_pose(x0, y0, z0 + HOVER_ALT, yaw0)
            if now - self.t_phase >= HOVER_S:
                self.go("DESCEND", "hover done. Descending slowly.")
        elif self.phase == "DESCEND":
            self.target_z = max(TOUCH_FROM, self.target_z - DESCEND_V * 0.1)
            self.send_pose(x0, y0, z0 + self.target_z, yaw0)
            if self.target_z <= TOUCH_FROM and h < TOUCH_FROM + AT:
                self.go("TOUCHDOWN", f"at {h:.2f} m. Touching down at {TOUCH_V} m/s.")


def main():
    rclpy.init()
    n = SimpleTest()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
