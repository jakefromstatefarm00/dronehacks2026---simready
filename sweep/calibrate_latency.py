"""Measure the flown planner step's compute time (original project_depth, no fast path) on this
CPU, split by (depth frame given, replans in the step). Used to set sim.LAT_* (see README)."""
import json
import sys
import numpy as np
import sim

log = []
for seed in range(int(sys.argv[1]) if len(sys.argv) > 1 else 2):
    sim.run({}, "L0", seed, cache_dir=sim.HERE / "cache", fast_depth=False, timing_log=log)
a = np.array(log, float)
out = {}
for gave in (0, 1):
    for rp in (0, 1):
        m = (a[:, 0] == gave) & (np.minimum(a[:, 1], 1) == rp)
        if m.any():
            out[f"frame{gave}_replan{rp}"] = dict(n=int(m.sum()), median_ms=round(float(np.median(a[m, 2])), 1),
                                                   p90_ms=round(float(np.percentile(a[m, 2], 90)), 1))
print(json.dumps(out, indent=1))
