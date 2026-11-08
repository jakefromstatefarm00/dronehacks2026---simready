"""Checks of the generated drift files (no ROS needed):  python jolie_course/drift/check_files.py

1. upload_to_flyx_drift.py's embedded align_to_map == align_safe.make_safe_align({}) (the harness's
   "drift" reproduction) and the safe file's == make_safe_align(<its SAFE constants>), on 400 random
   hit clouds around the published walls with random offsets.
2. Each *_skybeta.py has no /fmu topic in its node part and carries apply_alignment + the on_plan call.
"""
import importlib.util
import json
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
JC = HERE.parent

CHILD = r'''
import importlib.util, json, sys
import numpy as np
spec = importlib.util.spec_from_file_location("uploaded", sys.argv[1])
m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
dp = sys.modules["pathplan.depth_projection"]
sys.path.insert(0, sys.argv[2])
from align_safe import make_safe_align
opts = json.loads(sys.argv[3])
ref = make_safe_align(opts)
planner, gspec, env, prior = m._build_planner_embedded()
dist, grad = dp.map_distance_field(prior, gspec)
rng = np.random.default_rng(0)
walls = gspec.to_xy(np.argwhere(np.abs(dist) < .03))
bad = 0; acc = 0
for k in range(400):
    pts = walls[rng.choice(len(walls), 300)] + rng.normal(0, .02, (300, 2))
    pts = pts[np.linalg.norm(pts - pts[0], axis=1) < 2.5]
    pts = pts + rng.normal(0, .08, 2)
    if k % 2:
        pts = pts + rng.normal(0, .05, pts.shape)          # noisy clouds too
    a, _ = dp.align_to_map(pts, dist, grad, gspec)
    b, _ = ref(pts, dist, grad, gspec)
    if (a is None) != (b is None) or (a is not None and np.abs(a - b).max() > 1e-12):
        bad += 1
    acc += a is not None
print(json.dumps({"mismatches": bad, "accepted": acc}))
'''


def check_align(path, opts):
    out = subprocess.run([sys.executable, "-c", CHILD, str(path), str(HERE), json.dumps(opts)],
                         capture_output=True, text=True)
    if out.returncode:
        raise SystemExit(out.stderr[-2000:])
    r = json.loads(out.stdout.strip().splitlines()[-1])
    print(f"{path.name}: align_to_map vs make_safe_align({opts}): {r}")
    assert r["mismatches"] == 0
    return r


def main():
    spec = importlib.util.spec_from_file_location("bdf", HERE / "build_drift_files.py")
    bdf = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(bdf)
    p = bdf.SAFE
    check_align(JC / "upload_to_flyx_drift.py", {})
    check_align(JC / "upload_to_flyx_drift_safe.py", dict(consensus=p["CONSENSUS_M"], robust=p["ROBUST_C"]))
    for name in ("upload_to_flyx_drift_skybeta.py", "upload_to_flyx_drift_safe_skybeta.py"):
        s = (JC / name).read_text(encoding="utf-8")
        node = s[s.rindex("def main("):]
        assert "/fmu" not in node, name
        assert "def apply_alignment(self, shift):" in node and "self.apply_alignment(planner.align)" in node, name
        compile(s, name, "exec")
        print(f"{name}: Sky Beta node has no /fmu, applies the alignment, compiles")
    print("all checks passed")


if __name__ == "__main__":
    main()
