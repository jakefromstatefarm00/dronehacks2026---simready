"""TEST ONLY - fake PX4 for wiring tests of flyx/fly_pathplan.py (ROS 2 + px4_msgs, no simulator).

    python3 flyx/fake_px4.py /tmp/fake_px4.json &
    python3 -m flyx.fly_pathplan --execute --wall-clock    # never point this at a real PX4

Publishes /fmu/out/vehicle_odometry and vehicle_local_position_v1 at 50 Hz, and
vehicle_status_v1 / vehicle_land_detected at the slow rates measured in FlyX (1.7 / 0.9 Hz,
plus immediately on change, like PX4); obeys VehicleCommand (Offboard, arm, land) and moves
a point toward the latest TrajectorySetpoint at <= 1 m/s (yaw follows the setpoint).
Writes the commands it received and the final state to a JSON file. Not a flight model.
"""
import json
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import px4_msgs.msg as px4

OUT = sys.argv[1] if len(sys.argv) > 1 else "/tmp/fake_px4.json"


class FakePX4(Node):
    def __init__(self):
        super().__init__("fake_px4")
        self.pos = np.zeros(3)
        self.yaw = 0.0
        self.arming, self.nav, self.landed = 1, 4, True          # disarmed, POSCTL, on the ground
        self.sp = None
        self.last_offboard = -1e9
        self.log = {"commands": [], "setpoints": 0, "max_alt": 0.0, "path": []}
        q = qos_profile_sensor_data
        self.p = {k: self.create_publisher(t, n, q) for k, t, n in [
            ("odom", px4.VehicleOdometry, "/fmu/out/vehicle_odometry"),
            ("status", px4.VehicleStatus, "/fmu/out/vehicle_status_v1"),
            ("land", px4.VehicleLandDetected, "/fmu/out/vehicle_land_detected"),
            ("local", px4.VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1")]}
        self.create_subscription(px4.VehicleCommand, "/fmu/in/vehicle_command", self.on_cmd, q)
        self.create_subscription(px4.TrajectorySetpoint, "/fmu/in/trajectory_setpoint", self.on_sp, q)
        self.create_subscription(px4.OffboardControlMode, "/fmu/in/offboard_control_mode", self.on_mode, q)
        self.dt = .02
        self.slow = {"status": [1 / 1.7, -1e9, None], "land": [1 / .9, -1e9, None]}   # period, last t, last value
        self.create_timer(self.dt, self.step)

    def t(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_mode(self, msg):
        self.last_offboard = time.monotonic()        # loss check on a clock that never jumps

    def on_sp(self, msg):
        self.sp = (np.array(msg.position, float), float(msg.yaw))
        self.log["setpoints"] += 1

    def on_cmd(self, msg):
        self.log["commands"].append({"command": int(msg.command), "p1": msg.param1, "p2": msg.param2,
                                     "nav": self.nav, "arming": self.arming})
        if msg.command == 176 and msg.param2 == 6.0 and time.monotonic() - self.last_offboard < .5:
            self.nav = 14                                       # OFFBOARD (only with a live heartbeat)
        elif msg.command == 400 and msg.param1 == 1.0 and self.nav == 14:
            self.arming, self.landed = 2, False
        elif msg.command == 21:
            self.nav = 18                                       # AUTO_LAND

    def step(self):
        now = self.t()
        if self.nav == 14 and time.monotonic() - self.last_offboard > .5:
            self.nav = 5                                         # offboard loss failsafe -> HOLD-ish
        if self.arming == 2 and self.nav == 14 and self.sp is not None:
            d = self.sp[0] - self.pos
            n = np.linalg.norm(d)
            self.pos += d if n < 1.0 * self.dt else d / n * 1.0 * self.dt
            self.yaw = self.sp[1]
        elif self.nav == 18:
            self.pos[2] = min(0.0, self.pos[2] + .5 * self.dt)   # NED z up to 0 = descending
            if self.pos[2] >= 0.0 and self.arming == 2:
                self.arming, self.landed = 1, True
        self.log["max_alt"] = max(self.log["max_alt"], -self.pos[2])
        if int(now / self.dt) % 25 == 0:
            self.log["path"].append([round(float(v), 3) for v in self.pos])
        us = int(now * 1e6)
        o = px4.VehicleOdometry()
        o.timestamp = o.timestamp_sample = us
        o.pose_frame = 1
        o.position = [float(v) for v in self.pos]
        o.q = [math.cos(self.yaw / 2), 0.0, 0.0, math.sin(self.yaw / 2)]
        self.p["odom"].publish(o)
        if self.due("status", now, (self.arming, self.nav)):
            s = px4.VehicleStatus()
            s.timestamp, s.arming_state, s.nav_state = us, self.arming, self.nav
            self.p["status"].publish(s)
        if self.due("land", now, self.landed):
            ld = px4.VehicleLandDetected()
            ld.timestamp, ld.landed = us, self.landed
            self.p["land"].publish(ld)
        lp = px4.VehicleLocalPosition()
        lp.timestamp, lp.xy_valid, lp.z_valid = us, True, True
        lp.x, lp.y, lp.z, lp.heading = float(self.pos[0]), float(self.pos[1]), float(self.pos[2]), self.yaw
        self.p["local"].publish(lp)

    def due(self, key, now, value):
        period, last, old = self.slow[key]
        if now - last >= period or value != old:
            self.slow[key][1:] = [now, value]
            return True
        return False


def main():
    rclpy.init()
    node = FakePX4()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.log.update(final={"pos": node.pos.tolist(), "arming": node.arming, "nav": node.nav, "landed": node.landed})
        json.dump(node.log, open(OUT, "w"), indent=1)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
