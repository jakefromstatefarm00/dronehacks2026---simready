#!/usr/bin/env python3
"""Read a rosbag2 bag from an RC run without a ROS install (pure Python: rosbags + numpy).

    python -I tools/rc_bag/read_bag.py BAG            # list topics, types, counts
    python -I tools/rc_bag/read_bag.py BAG --echo /skybeta/state -n 3

BAG can be
  * a bag folder with metadata.yaml (sqlite3 .db3 or mcap, split files are handled), or
  * a bag folder WITHOUT metadata.yaml (recorder killed): every *.db3 / *.mcap in it is read
    directly, in split order, or
  * a single .db3 / .mcap file.

Bags are opened read-only (rosbags uses a read-only sqlite VFS); nothing is written next to them.

Message types: the standard ROS 2 Humble types are built in. px4_msgs types are registered from
.msg files when they can be found (--px4-msgs DIR, $RC_BAG_PX4_MSGS, or a px4_msgs/msg folder in
this repo); bags that embed their message definitions (Jazzy mcap / Iron+ sqlite) register
themselves. Topics whose type is unknown are listed but skipped ("undecodable"), never fatal.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

REPO_ROOT = Path(__file__).resolve().parents[2]
# Camera mount on the Sky Beta drone: base_link -> camera, no rotation (m).
CAM_OFFSET = np.array([0.1053, 0.0, 0.0304])
DEFAULT_FX = 398.8


@dataclass
class TopicInfo:
    name: str
    msgtype: str
    count: int
    decodable: bool


def _find_px4_msg_dirs(extra: str | None) -> list[Path]:
    cands = []
    for p in (extra, os.environ.get("RC_BAG_PX4_MSGS")):
        if p:
            cands.append(Path(p))
    # search the repo (shallowly; skip heavy folders) for .../px4_msgs/msg/*.msg
    skip = {".git", "node_modules", "build", "install", "log", "__pycache__", "checkpoints", "runs"}
    for root, dirs, files in os.walk(REPO_ROOT):
        dirs[:] = [d for d in dirs if d not in skip and not d.startswith(".")]
        if Path(root).name == "msg" and Path(root).parent.name == "px4_msgs":
            cands.append(Path(root))
            dirs[:] = []
        if len(Path(root).relative_to(REPO_ROOT).parts) >= 6:
            dirs[:] = []
    out = []
    for c in cands:
        if c.is_dir():
            out.append(c / "msg" if (c / "msg").is_dir() and not list(c.glob("*.msg")) else c)
    return out


def _register_msg_dir(typestore, d: Path, pkg: str = "px4_msgs") -> int:
    types = {}
    for f in sorted(d.glob("*.msg")):
        try:
            types.update(get_types_from_msg(f.read_text(encoding="utf-8", errors="replace"),
                                            f"{pkg}/msg/{f.stem}"))
        except Exception as err:  # one odd file must not stop the rest
            print(f"[read_bag] skip {f.name}: {err}", file=sys.stderr)
    typestore.register(types)
    return len(types)


def _storage_files(path: Path) -> list[Path]:
    def split_key(p: Path):
        m = re.search(r"_(\d+)\.(db3|mcap)$", p.name)
        return (int(m.group(1)) if m else 0, p.name)
    return sorted([*path.glob("*.db3"), *path.glob("*.mcap")], key=split_key)


class Bag:
    """Uniform, read-only access to one bag (folder or file)."""

    def __init__(self, path: str | Path, px4_msgs: str | None = None, quiet: bool = False):
        self.path = Path(path)
        if not self.path.exists():
            raise FileNotFoundError(f"bag not found: {self.path}")
        self.notes: list[str] = []
        if self.path.is_dir() and (self.path / "metadata.yaml").exists():
            self.files = [self.path]
            self.readers = [Reader(self.path)]
        elif self.path.is_dir():
            self.files = _storage_files(self.path)
            if not self.files:
                raise FileNotFoundError(f"no metadata.yaml and no .db3/.mcap files in {self.path}")
            self.notes.append("metadata.yaml missing: read the storage files directly "
                              f"({', '.join(f.name for f in self.files)})")
            self.readers = [Reader(f) for f in self.files]
        else:
            self.files = [self.path]
            self.readers = [Reader(self.path)]
        for r in self.readers:
            r.open()

        self.typestore = get_typestore(Stores.ROS2_HUMBLE)
        n = 0
        for d in _find_px4_msg_dirs(px4_msgs):
            n += _register_msg_dir(self.typestore, d)
            self.notes.append(f"px4_msgs: registered {n} types from {d}")
        # types embedded in the bag (Jazzy mcap / Iron+ sqlite)
        for c in self.connections:
            if c.msgtype in self.typestore.fielddefs:
                continue
            fmt = getattr(getattr(c, "msgdef", None), "format", None)
            if fmt is not None and fmt.name == "MSG" and c.msgdef.data:
                try:
                    self.typestore.register(get_types_from_msg(c.msgdef.data, c.msgtype))
                except Exception as err:
                    self.notes.append(f"could not register embedded type {c.msgtype}: {err}")

        self.topics: dict[str, TopicInfo] = {}
        for c in self.connections:
            ok = c.msgtype in self.typestore.fielddefs
            t = self.topics.get(c.topic)
            if t:
                t.count += c.msgcount
            else:
                self.topics[c.topic] = TopicInfo(c.topic, c.msgtype, c.msgcount, ok)
        bad = sorted(t.name for t in self.topics.values() if not t.decodable)
        if bad:
            types = sorted({self.topics[b].msgtype for b in bad})
            self.notes.append(f"{len(bad)} topics skipped, message type unknown ({', '.join(types[:6])}"
                              f"{' ...' if len(types) > 6 else ''}); give --px4-msgs DIR to decode them")
        if not quiet:
            for n_ in self.notes:
                print(f"[read_bag] {n_}", file=sys.stderr)

    # ------------------------------------------------------------------ access
    @property
    def connections(self):
        return [c for r in self.readers for c in r.connections]

    def close(self):
        for r in self.readers:
            r.close()

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.close()
        return False

    def has(self, topic: str) -> bool:
        t = self.topics.get(topic)
        return bool(t and t.count > 0 and t.decodable)

    def first_topic(self, *names: str) -> str | None:
        for n in names:
            if self.has(n):
                return n
        return None

    def raw(self, topics=None):
        """Yield (topic, bag_time_ns, rawbytes, msgtype) in time order per storage file."""
        want = None if topics is None else set([topics] if isinstance(topics, str) else topics)
        for r in self.readers:
            conns = [c for c in r.connections if want is None or c.topic in want]
            if not conns:
                continue
            for c, t, data in r.messages(connections=conns):
                yield c.topic, t, data, c.msgtype

    def messages(self, topics=None, stride: int = 1):
        """Yield (topic, bag_time_ns, msg) for decodable topics; stride>1 decodes every Nth per topic."""
        seen: dict[str, int] = {}
        for topic, t, data, mt in self.raw(topics):
            if mt not in self.typestore.fielddefs:
                continue
            k = seen.get(topic, 0)
            seen[topic] = k + 1
            if k % stride:
                continue
            try:
                yield topic, t, self.typestore.deserialize_cdr(data, mt)
            except Exception as err:
                self.notes.append(f"decode error on {topic}: {err}")

    def times(self, topic: str) -> np.ndarray:
        """Bag receive times (s, absolute) of every message on a topic, without decoding."""
        cache = self.__dict__.setdefault("_times", {})
        if topic not in cache:
            cache[topic] = np.array([t for _, t, _, _ in self.raw(topic)], dtype=np.int64) / 1e9
        return cache[topic]


# ---------------------------------------------------------------------- message helpers
def stamp_s(msg) -> float | None:
    h = getattr(msg, "header", None)
    if h is None:
        return None
    s = h.stamp.sec + h.stamp.nanosec * 1e-9
    return s if s > 0 else None


def quat_to_R(q) -> np.ndarray:
    """q = (x, y, z, w) -> 3x3 rotation matrix (batched: (N,4) -> (N,3,3))."""
    q = np.asarray(q, float)
    q = q / np.linalg.norm(q, axis=-1, keepdims=True)
    x, y, z, w = np.moveaxis(q, -1, 0)
    R = np.stack([
        1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
        2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
        2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1)
    return R.reshape(q.shape[:-1] + (3, 3))


def yaw_of(q) -> np.ndarray:
    q = np.asarray(q, float)
    x, y, z, w = np.moveaxis(q, -1, 0)
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def load_odometry(bag: Bag, topic: str = "/visual_slam/tracking/odometry") -> dict | None:
    """nav_msgs/Odometry -> arrays. t = header stamp (falls back to bag time), tb = bag time."""
    if not bag.has(topic):
        return None
    t, tb, p, q, cov, v = [], [], [], [], [], []
    for _, tn, m in bag.messages(topic):
        hs = stamp_s(m)
        tb.append(tn / 1e9)
        t.append(hs if hs is not None else tn / 1e9)
        pp, oo = m.pose.pose.position, m.pose.pose.orientation
        p.append([pp.x, pp.y, pp.z])
        q.append([oo.x, oo.y, oo.z, oo.w])
        c = np.asarray(m.pose.covariance, float).reshape(6, 6)
        cov.append(np.diag(c))
        tw = m.twist.twist.linear
        v.append([tw.x, tw.y, tw.z])
    if not t:
        return None
    return dict(topic=topic, t=np.array(t), tb=np.array(tb), pos=np.array(p), quat=np.array(q),
                cov=np.array(cov), vel=np.array(v), yaw=yaw_of(np.array(q)))


def load_px4_local(bag: Bag) -> dict | None:
    """px4_msgs/VehicleLocalPosition (NED) -> ENU arrays (bag time)."""
    topic = bag.first_topic("/fmu/out/vehicle_local_position", "/fmu/out/vehicle_local_position_v1")
    if topic is None:
        return None
    t, p, hd, valid = [], [], [], []
    for _, tn, m in bag.messages(topic):
        t.append(tn / 1e9)
        p.append([m.y, m.x, -m.z])                       # NED -> ENU
        hd.append(np.pi / 2 - float(getattr(m, "heading", 0.0)))   # NED heading -> ENU yaw
        valid.append(bool(getattr(m, "xy_valid", True)) and bool(getattr(m, "z_valid", True)))
    if not t:
        return None
    return dict(topic=topic, t=np.array(t), tb=np.array(t), pos=np.array(p), yaw=np.array(hd),
                valid=np.array(valid))


def load_px4_odometry(bag: Bag) -> dict | None:
    """px4_msgs/VehicleOdometry (NED / FRD) -> ENU / FLU pose arrays (bag time)."""
    topic = bag.first_topic("/fmu/out/vehicle_odometry")
    if topic is None:
        return None
    t, p, q = [], [], []
    # q_ned(FRD->NED) -> q_enu(FLU->ENU): R_enu = A @ R_ned @ B, A = NED->ENU, B = FLU->FRD
    A = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], float)
    B = np.diag([1.0, -1.0, -1.0])
    for _, tn, m in bag.messages(topic):
        w, x, y, z = (float(a) for a in m.q)
        Rn = quat_to_R([x, y, z, w])
        Re = A @ Rn @ B
        t.append(tn / 1e9)
        p.append([m.position[1], m.position[0], -m.position[2]])
        q.append(_R_to_quat(Re))
    if not t:
        return None
    q = np.array(q)
    return dict(topic=topic, t=np.array(t), tb=np.array(t), pos=np.array(p), quat=q, yaw=yaw_of(q))


def _R_to_quat(R) -> list:
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = 2 * np.sqrt(tr + 1)
        return [(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, s / 4]
    i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2 * np.sqrt(1 + R[i, i] - R[j, j] - R[k, k])
    q = [0.0, 0.0, 0.0, (R[k, j] - R[j, k]) / s]
    q[i] = s / 4
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    return q


def load_skybeta_state(bag: Bag, topic: str = "/skybeta/state") -> list[tuple[float, dict]]:
    out = []
    if not bag.has(topic):
        return out
    for _, tn, m in bag.messages(topic):
        try:
            out.append((tn / 1e9, json.loads(m.data)))
        except (ValueError, TypeError):
            pass
    return out


def depth_to_metres(msg) -> np.ndarray:
    """sensor_msgs/Image 16UC1 (mm) or 32FC1 (m) -> float32 metres (H, W); 0 = no data."""
    h, w, step = msg.height, msg.width, msg.step
    buf = np.asarray(msg.data, dtype=np.uint8)
    enc = msg.encoding.lower()
    if enc in ("16uc1", "mono16"):
        dt = np.dtype(">u2" if msg.is_bigendian else "<u2")
        img = buf[: h * step].reshape(h, step)[:, : w * 2].copy().view(dt).astype(np.float32) / 1000.0
    elif enc == "32fc1":
        dt = np.dtype(">f4" if msg.is_bigendian else "<f4")
        img = buf[: h * step].reshape(h, step)[:, : w * 4].copy().view(dt).astype(np.float32)
        img[~np.isfinite(img)] = 0
    else:
        raise ValueError(f"unsupported depth encoding {msg.encoding}")
    return img


def camera_intrinsics(bag: Bag, width: int, height: int, topic: str = "/camera/depth/camera_info"):
    """(fx, fy, cx, cy) from camera_info if recorded, else the Sky Beta default (fx ~ 398.8)."""
    if bag.has(topic):
        for _, _, m in bag.messages(topic):
            k = np.asarray(m.k, float)
            if k[0] > 0:
                sx = width / m.width if m.width else 1.0
                sy = height / m.height if m.height else 1.0
                return k[0] * sx, k[4] * sy, k[2] * sx, k[5] * sy, "camera_info"
            break
    s = width / 640.0
    return DEFAULT_FX * s, DEFAULT_FX * s, (width - 1) / 2, (height - 1) / 2, "default fx=398.8"


def default_out_dir(bag_path: str | Path, out: str | None) -> Path:
    p = Path(bag_path)
    name = p.stem if p.is_file() else p.name
    if name == "bag" and p.parent.name:          # record_rc_run.sh layout: rc-<label>-<time>/bag
        name = p.parent.name
    d = Path(out) if out else Path.cwd() / "rc_bag_out" / name
    d.mkdir(parents=True, exist_ok=True)
    return d


def find_side_file(bag_path: str | Path, name: str) -> Path | None:
    p = Path(bag_path)
    base = p if p.is_dir() else p.parent
    for d in (base, base.parent):
        if (d / name).exists():
            return d / name
    return None


def main(argv=None):
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(errors="replace")   # Windows consoles: never die on a stray character
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--px4-msgs", help="folder with px4_msgs .msg files")
    ap.add_argument("--echo", help="print messages of this topic")
    ap.add_argument("-n", type=int, default=3)
    a = ap.parse_args(argv)
    with Bag(a.bag, a.px4_msgs) as bag:
        if a.echo:
            for i, (_, t, m) in enumerate(bag.messages(a.echo)):
                if i >= a.n:
                    break
                txt = repr(m)
                print(f"{t / 1e9:.3f}  {txt[:600]}{' ...' if len(txt) > 600 else ''}")
            return 0
        for t in sorted(bag.topics.values(), key=lambda x: x.name):
            print(f"{t.count:9d}  {t.name:55s} {t.msgtype}{'' if t.decodable else '  (undecodable)'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
