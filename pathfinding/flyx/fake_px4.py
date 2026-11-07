"""TEST ONLY - fake PX4 for wiring tests of flyx/fly_pathplan.py (ROS 2 + px4_msgs, no simulator).

    python3 flyx/fake_px4.py /tmp/fake_px4.json &
    python3 -m flyx.fly_pathplan --execute --wall-clock    # never point this at a real PX4

Publishes /fmu/out/vehicle_odometry and vehicle_local_position_v1 at 50 Hz, and
vehicle_status_v1 / vehicle_land_detected at the slow rates measured in FlyX (1.7 / 0.9 Hz,
plus immediately on change, like PX4); obeys VehicleCommand (Offboard, arm, land) and moves
a point toward the latest TrajectorySetpoint at <= 1 m/s and yaw at <= 1 rad/s.
Writes the commands it received and the final state to a JSON file. Not a flight model.

FAKE_DEPTH=1 also publishes /camera/depth/image_raw (16UC1 640x400, 13 Hz, the FlyX lab's
OAK-D intrinsics) + camera_info, ray-cast from the published course at the vehicle's pose
(launch mark facing up the first corridor), to load-test the node with realistic depth.
"""
import json
import math
import os
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import px4_msgs.msg as px4

OUT = sys.argv[1] if len(sys.argv) > 1 else "/tmp/fake_px4.json"
YAW_RATE = 1.0                      # rad/s


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
        if os.environ.get("FAKE_DEPTH") == "1":
            self.setup_depth()
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
            turn = math.atan2(math.sin(self.sp[1] - self.yaw), math.cos(self.sp[1] - self.yaw))
            self.yaw += max(-YAW_RATE * self.dt, min(YAW_RATE * self.dt, turn))   # PX4 limits yaw rate too
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

    # ------------------------------------------------------------------ fake depth camera
    K = (421.016, 0.0, 319.25, 0.0, 421.016, 200.66, 0.0, 0.0, 1.0)     # FlyX lab CameraInfo
    W, H = 640, 400

    def setup_depth(self):
        from sensor_msgs.msg import CameraInfo, Image
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
        from pathplan.harness import load_published
        from pathplan.world import GridSpec, occupancy
        course, _ = load_published()
        self.spec = GridSpec()
        self.occ = occupancy(course, self.spec, altitude=.9, half_height=.05)
        self.Image, self.CameraInfo = Image, CameraInfo
        self.depth_pub = self.create_publisher(Image, "/camera/depth/image_raw", qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, "/camera/depth/camera_info", qos_profile_sensor_data)
        self.create_timer(1 / 13, self.publish_depth)

    def publish_depth(self):
        stamp = self.get_clock().now().to_msg()      # capture time, like a real camera (before rendering)
        depth = render_depth(self.occ, self.spec, self.pos, self.yaw, self.K, self.W, self.H)
        mm = np.where(np.isfinite(depth) & (depth < 10), depth * 1000, 0).astype("<u2")
        img = self.Image(height=self.H, width=self.W, encoding="16UC1", step=self.W * 2, data=mm.tobytes())
        img.header.stamp, img.header.frame_id = stamp, "oak_left"
        self.depth_pub.publish(img)
        info = self.CameraInfo(height=self.H, width=self.W, k=list(self.K))
        info.header.stamp, info.header.frame_id = stamp, "oak_left"
        self.info_pub.publish(info)


def render_depth(occ, spec, pos_ned, yaw_ned, K, W, H):
    """Depth image (m, inf = no return) of the course walls (occupancy at flight height,
    1.73 m tall) and the floor, from a vehicle at NED pos/yaw (origin = launch mark facing north)."""
    fx, cx, fy, cy = K[0], K[2], K[4], K[5]
    # NED -> course frame (launch mark (-2.515, 0.914), facing course +y)
    x, y, z = pos_ned[1] - 2.515, pos_ned[0] + .914, .05 - pos_ned[2]
    yaw = math.pi / 2 - yaw_ned
    cam = np.array([x, y]) + .115 * np.array([math.cos(yaw), math.sin(yaw)])
    ang = np.arctan((np.arange(W) - cx) / fx)                            # + = right of the heading
    dirs = np.stack([np.cos(yaw - ang), np.sin(yaw - ang)], 1)
    r = np.arange(.05, 4.0, .025)
    rc = spec.to_cell(cam[None, None] + r[None, :, None] * dirs[:, None])
    hit = occ[rc[..., 0], rc[..., 1]]
    wall = np.where(hit.any(1), r[np.argmax(hit, 1)], np.inf)              # horizontal range per column
    slope = (np.arange(H) - cy)[:, None] / fy                              # + = below the horizon
    zc = z + .029
    floor = np.where(slope > 1e-6, zc / np.maximum(slope, 1e-6), np.inf)
    top_ok = zc - slope * wall[None] <= 1.73                               # ray meets the wall below its top
    horiz = np.where(floor < wall[None], floor, np.where(top_ok, wall[None], np.inf))
    return horiz * np.cos(ang)[None]                                       # optical z


def main():
    import signal
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt))   # save the log on TERM
    rclpy.init(signal_handler_options=__import__("rclpy.signals", fromlist=["x"]).SignalHandlerOptions.NO)
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
