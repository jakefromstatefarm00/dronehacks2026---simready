"""Read-only FlyX lab interface check.  Publishes nothing, commands nothing.

    cd <repo>/pathfinding
    python3 -m flyx.check_lab              # ~20 s; prints a report and the fly_pathplan arguments

Finds the PX4 output topics (with their _vN suffix), measures message rates, reads one
sample of each (pose frame, estimator validity, landed/armed state), finds depth image
and CameraInfo topics (encoding, size, intrinsics, frame) and checks the px4_msgs
generation.  Writes the report to runs/flyx-check-<time>.json.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from flyx.px4_link import find_topic   # noqa: E402

PX4_OUT = {"odom": ("vehicle_odometry", "VehicleOdometry"), "status": ("vehicle_status", "VehicleStatus"),
           "land": ("vehicle_land_detected", "VehicleLandDetected"),
           "local": ("vehicle_local_position", "VehicleLocalPosition")}
PX4_IN = ["offboard_control_mode", "trajectory_setpoint", "vehicle_command"]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=20.0)
    a = ap.parse_args(argv)
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    import px4_msgs.msg as px4
    from sensor_msgs.msg import CameraInfo, Image

    rclpy.init()
    node = Node("flyx_check_lab")
    node.set_parameters([Parameter("use_sim_time", value=True)])
    stats, samples = {}, {}

    def counter(name, keep=None):
        stats[name] = {"n": 0, "first_wall": None, "last_wall": None}

        def cb(msg):
            s = stats[name]
            s["n"] += 1
            w = time.monotonic()
            s["first_wall"] = s["first_wall"] or w
            s["last_wall"] = w
            if keep and name not in samples:
                samples[name] = keep(msg)
        return cb

    end = time.monotonic() + 3.0
    while time.monotonic() < end:                       # let discovery settle
        rclpy.spin_once(node, timeout_sec=.1)
    graph = node.get_topic_names_and_types()
    report = {"ros_distro": __import__("os").environ.get("ROS_DISTRO"), "topics": {}, "px4_in": {}, "images": {}}
    keepers = {
        "odom": lambda m: {"pose_frame": m.pose_frame, "position": list(m.position), "q": list(m.q),
                           "timestamp_sample_us": int(m.timestamp_sample)},
        "status": lambda m: {"arming_state": m.arming_state, "nav_state": m.nav_state},
        "land": lambda m: {"landed": bool(m.landed)},
        "local": lambda m: {"xy_valid": bool(m.xy_valid), "z_valid": bool(m.z_valid), "heading": float(m.heading)},
    }
    for key, (base, typ) in PX4_OUT.items():
        topic = find_topic(graph, base, f"px4_msgs/msg/{typ}")
        cls = getattr(px4, typ)
        report["topics"][key] = {"topic": topic, "type": typ,
                                 "installed_message_version": getattr(cls, "MESSAGE_VERSION", None)}
        if topic:
            node.create_subscription(cls, topic, counter(key, keepers[key]), qos_profile_sensor_data)
    for base in PX4_IN:
        report["px4_in"][base] = [t for t, ty in graph if t.startswith(f"/fmu/in/{base}")]
    for t, types in graph:
        if "sensor_msgs/msg/Image" in types and any(k in t.lower() for k in ("depth", "stereo", "left", "right")):
            report["images"][t] = {}
            node.create_subscription(Image, t, counter(t, lambda m: {"encoding": m.encoding, "width": m.width,
                                     "height": m.height, "frame_id": m.header.frame_id}), qos_profile_sensor_data)
        if "sensor_msgs/msg/CameraInfo" in types:
            report["images"].setdefault(t, {})
            node.create_subscription(CameraInfo, t, counter(t, lambda m: {"k": list(m.k), "width": m.width,
                                     "height": m.height, "frame_id": m.header.frame_id}), qos_profile_sensor_data)
    t0 = node.get_clock().now().nanoseconds * 1e-9
    end = time.monotonic() + a.seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.05)
    sim_span = node.get_clock().now().nanoseconds * 1e-9 - t0

    def rate(name):
        s = stats.get(name)
        if not s or s["n"] < 2:
            return 0.0
        return round((s["n"] - 1) / max(s["last_wall"] - s["first_wall"], 1e-9), 1)

    for key in PX4_OUT:
        report["topics"][key].update(rate_hz_wall=rate(key), sample=samples.get(key))
    for t in report["images"]:
        report["images"][t].update(rate_hz_wall=rate(t), sample=samples.get(t))
    report["sim_seconds_per_wall_second"] = round(sim_span / a.seconds, 3)

    problems, depth = [], None
    for key, info in report["topics"].items():
        if not info["topic"]:
            problems.append(f"no unique /fmu/out/{PX4_OUT[key][0]}[_vN] topic of type {PX4_OUT[key][1]}")
        elif not info["rate_hz_wall"]:
            problems.append(f"{info['topic']} discovered but no messages (QoS or px4_msgs mismatch?)")
    if (report["topics"]["odom"].get("sample") or {}).get("pose_frame") not in (1, None):
        problems.append("odometry pose_frame is not NED (1)")
    depths = [t for t, i in report["images"].items() if "depth" in t.lower() and (i.get("sample") or {}).get("encoding")]
    if depths:
        depth = depths[0]
        enc = report["images"][depth]["sample"]["encoding"]
        if enc not in ("16UC1", "32FC1", "mono16"):
            problems.append(f"depth encoding {enc} not supported by depth_projection (16UC1/32FC1)")
    else:
        problems.append("no depth image with data found; the planner would fly the published map only")
    if report["sim_seconds_per_wall_second"] < .5:
        problems.append(f"sim runs at {report['sim_seconds_per_wall_second']}x real time; wall-clock guards may trip")
    for base, ts in report["px4_in"].items():
        if f"/fmu/in/{base}" not in ts:
            problems.append(f"/fmu/in/{base} has no subscriber yet (is the XRCE agent / PX4 up?)")
    report["problems"] = problems
    report["suggested_command"] = ("python3 -m flyx.fly_pathplan" + (f" --depth-topic {depth}" if depth else "")
                                   + "    # shadow first; add --execute only after review")
    out = ROOT / "runs" / f"flyx-check-{time.strftime('%Y%m%d-%H%M%S')}.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2, default=float))
    print(json.dumps(report, indent=2, default=float))
    print(f"\nreport: {out}\n{'OK' if not problems else 'CHECK: ' + '; '.join(problems)}")
    print(f"\nnext (copy this):\n    {report['suggested_command']}")
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
