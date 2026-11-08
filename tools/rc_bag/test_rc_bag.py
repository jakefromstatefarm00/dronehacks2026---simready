"""Synthetic end-to-end test: a fake bag written with rosbags' Writer.

The drone climbs from the ground to 1 m, then flies 2 m sideways (+y) facing +x; the depth camera
sees a flat wall 1.5 m in front of it the whole time, i.e. at odom x = 0.1053 (mount) + 1.5.

    python -I -m pytest tools/rc_bag/test_rc_bag.py -q
"""
from __future__ import annotations

import csv
import shutil
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from rosbags.rosbag2 import StoragePlugin, Writer  # noqa: E402
from rosbags.typesys import Stores, get_typestore  # noqa: E402

import depth_map  # noqa: E402
import summarize  # noqa: E402
import trajectory  # noqa: E402
from course_geom import LAUNCH_XY  # noqa: E402
from read_bag import CAM_OFFSET  # noqa: E402

WALL_DEPTH = 1.5
WALL_X = CAM_OFFSET[0] + WALL_DEPTH
T0 = 1_794_000_000_000_000_000
W, H, FX = 640, 400, 398.8


def _write_bag(path: Path, with_odom: bool = True, storage=StoragePlugin.SQLITE3):
    ts = get_typestore(Stores.ROS2_HUMBLE)
    T = ts.types
    Time, Header = T["builtin_interfaces/msg/Time"], T["std_msgs/msg/Header"]

    def hdr(tn, frame):
        return Header(stamp=Time(sec=tn // 10**9, nanosec=tn % 10**9), frame_id=frame)

    with Writer(path, version=8, storage_plugin=storage) as w:
        c_depth = w.add_connection("/camera/depth/image_raw", "sensor_msgs/msg/Image", typestore=ts)
        c_info = w.add_connection("/camera/depth/camera_info", "sensor_msgs/msg/CameraInfo", typestore=ts)
        if with_odom:
            c_odom = w.add_connection("/visual_slam/tracking/odometry", "nav_msgs/msg/Odometry", typestore=ts)
            c_state = w.add_connection("/skybeta/state", "std_msgs/msg/String", typestore=ts)
        depth = np.full((H, W), int(WALL_DEPTH * 1000), np.uint16)
        depth[:4] = 0                                    # a few "no data" rows
        data = np.frombuffer(depth.astype("<u2").tobytes(), np.uint8)
        for i in range(150):                             # 5 s at 30 Hz
            tn = T0 + int(i * 1e9 / 30)
            s = i / 30
            if with_odom:
                z = min(s, 1.0)
                y = 0.0 if s < 1 else 0.5 * (s - 1)
                pose = T["geometry_msgs/msg/Pose"](
                    position=T["geometry_msgs/msg/Point"](x=0.0, y=y, z=z),
                    orientation=T["geometry_msgs/msg/Quaternion"](x=0.0, y=0.0, z=0.0, w=1.0))
                cov = np.zeros(36)
                cov[[0, 7, 14, 35]] = 1e-4
                V3 = T["geometry_msgs/msg/Vector3"]
                odom = T["nav_msgs/msg/Odometry"](
                    header=hdr(tn, "odom"), child_frame_id="base_link",
                    pose=T["geometry_msgs/msg/PoseWithCovariance"](pose=pose, covariance=cov),
                    twist=T["geometry_msgs/msg/TwistWithCovariance"](
                        twist=T["geometry_msgs/msg/Twist"](linear=V3(x=0.0, y=0.0, z=0.0),
                                                           angular=V3(x=0.0, y=0.0, z=0.0)),
                        covariance=np.zeros(36)))
                w.write(c_odom, tn, ts.serialize_cdr(odom, "nav_msgs/msg/Odometry"))
                if i % 3 == 0:
                    st = (f'{{"armed": {"true" if s > 0.2 else "false"}, "landed": {"true" if z < 0.1 else "false"}, '
                          f'"offboard_active": false, "nav_state": 2, "px4_link": true, '
                          f'"local_enu": [0.0, {y}, {z}], "origin_enu": [0, 0, 0]}}')
                    w.write(c_state, tn, ts.serialize_cdr(T["std_msgs/msg/String"](data=st), "std_msgs/msg/String"))
            img = T["sensor_msgs/msg/Image"](header=hdr(tn, "camera_optical"), height=H, width=W,
                                             encoding="16UC1", is_bigendian=0, step=W * 2, data=data)
            w.write(c_depth, tn, ts.serialize_cdr(img, "sensor_msgs/msg/Image"))
            if i % 10 == 0:
                info = T["sensor_msgs/msg/CameraInfo"](
                    header=hdr(tn, "camera_optical"), height=H, width=W, distortion_model="plumb_bob",
                    d=np.zeros(5), k=np.array([FX, 0, (W - 1) / 2, 0, FX, (H - 1) / 2, 0, 0, 1.0]),
                    r=np.eye(3).ravel(), p=np.array([FX, 0, (W - 1) / 2, 0, 0, FX, (H - 1) / 2, 0, 0, 0, 1.0, 0]),
                    binning_x=0, binning_y=0,
                    roi=T["sensor_msgs/msg/RegionOfInterest"](x_offset=0, y_offset=0, height=0, width=0,
                                                              do_rectify=False))
                w.write(c_info, tn, ts.serialize_cdr(info, "sensor_msgs/msg/CameraInfo"))
    return path


@pytest.fixture(scope="module")
def bag(tmp_path_factory):
    return _write_bag(tmp_path_factory.mktemp("bags") / "fake_rc")


def test_trajectory_csv(bag, tmp_path):
    r = trajectory.run(bag, tmp_path, align="none")
    assert r["status"] == "ok"
    rows = list(csv.DictReader(open(tmp_path / "trajectory_vio.csv")))
    assert len(rows) == 150
    end = rows[-1]
    assert abs(float(end["x"])) < 1e-3 and abs(float(end["y"]) - 0.5 * (149 / 30 - 1)) < 1e-3
    assert abs(float(end["z"]) - 1.0) < 1e-3
    assert (tmp_path / "trajectory_topdown.png").exists() and (tmp_path / "trajectory_height.png").exists()


def test_trajectory_first_pose_alignment(bag, tmp_path):
    trajectory.run(bag, tmp_path, align="first-pose")
    rows = list(csv.DictReader(open(tmp_path / "trajectory_vio.csv")))
    # start on the launch mark; odom +y (sideways left) becomes course -x when facing +y
    assert abs(float(rows[0]["cx"]) - LAUNCH_XY[0]) < 1e-3 and abs(float(rows[0]["cy"]) - LAUNCH_XY[1]) < 1e-3
    assert float(rows[-1]["cx"]) < LAUNCH_XY[0] - 1.9


def test_depth_map_recovers_wall_odom_frame(bag, tmp_path):
    r = depth_map.run(bag, tmp_path, align="none")
    assert r["status"] == "ok" and r["occupied_cells"] > 20
    xy = r["occupied_xy"]
    err = np.abs(xy[:, 0] - WALL_X)
    assert np.median(err) < 0.10, np.median(err)
    assert (err < 0.10).mean() > 0.95
    assert xy[:, 1].min() < 0.0 and xy[:, 1].max() > 2.0         # the wall spans the flown 2 m (+ FOV)
    occ = np.load(tmp_path / "depth_occupied.npy")
    assert occ.sum() == r["occupied_cells"]


def test_depth_map_recovers_wall_course_frame(bag, tmp_path):
    r = depth_map.run(bag, tmp_path, align="first-pose")
    xy = r["occupied_xy"]
    inside = (np.abs(xy) < 3.0).all(1)
    err = np.abs(xy[inside, 1] - (LAUNCH_XY[1] + WALL_X))          # facing +y: wall at y = launch + 1.6053
    assert np.median(err) < 0.10 and (err < 0.10).mean() > 0.95
    assert "wall_distance" in r and r["wall_distance"]["median_m"] is not None
    assert (tmp_path / "depth_map.png").exists() and (tmp_path / "depth_map.md").exists()


def test_summary_and_missing_metadata(bag, tmp_path):
    txt = summarize.run(bag, tmp_path)
    assert "/visual_slam/tracking/odometry" in txt and "position jumps > 0.3 m between poses: 0" in txt
    assert "`armed`: False" in txt
    nometa = tmp_path / "nometa"
    shutil.copytree(bag, nometa)
    (nometa / "metadata.yaml").unlink()
    txt2 = summarize.run(nometa, tmp_path / "o2")
    assert "metadata.yaml missing" in txt2 and "| 150 |" in txt2
    r = depth_map.run(nometa, tmp_path / "o3", align="none")
    assert np.median(np.abs(r["occupied_xy"][:, 0] - WALL_X)) < 0.10


def test_depth_only_bag(tmp_path):
    b = _write_bag(tmp_path / "depth_only", with_odom=False)
    r = depth_map.run(b, tmp_path / "o", align="first-pose")
    assert r["status"] == "no odometry" and r["depth"]["frames"] == 150
    assert abs(r["depth"]["range_pct"][50] - WALL_DEPTH) < 0.01
    assert trajectory.run(b, tmp_path / "o")["status"] == "no odometry"
    assert "no odometry" in summarize.run(b, tmp_path / "o")


def test_mcap_without_metadata(tmp_path):
    b = _write_bag(tmp_path / "mcap_bag", storage=StoragePlugin.MCAP)
    (b / "metadata.yaml").unlink()
    r = depth_map.run(b, tmp_path / "o", align="none")
    assert np.median(np.abs(r["occupied_xy"][:, 0] - WALL_X)) < 0.10
