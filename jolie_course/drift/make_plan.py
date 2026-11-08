"""The A/B grid (Case dicts), split into shards:  python make_plan.py <out_prefix> [--arms base drift] [--shards 2]

Cells (each run for every arm on the same seeds = common random numbers):
  main     drift 0/1/2/4/6/8 cm/m x variant nominal/walls/clutter, clean depth
  heading  drift 2 cm/m + 0.1 deg/m heading drift, nominal/walls
  noise    noisy depth at drift 0 and 4, nominal/walls
  extras   walls_big (+-0.30 m) at 0 and 2; camera mount error (+5 cm fwd, 2 deg yaw) at 0 and 4;
           frame-pose lag 50 ms at 0; VIO jumps (1 per 10 m) at 2; bad depth at 0
Nominal + clean + zero drift is deterministic, so it gets 3 seeds.
"""
import json
import sys


def cells():
    for d in (0.0, .01, .02, .04, .06, .08):
        for v in ("nominal", "walls", "clutter"):
            n = 30 if d in (0.0, .02, .04) else 12
            if d == 0.0 and v == "nominal":
                n = 3
            yield "main", dict(drift=d, variant=v), n
    for v in ("nominal", "walls"):
        yield "heading", dict(drift=.02, head=.1, variant=v), 15
    for d in (0.0, .04):
        for v in ("nominal", "walls"):
            yield "noise", dict(drift=d, variant=v, noise="noisy"), 15
    yield "extra", dict(drift=0.0, variant="walls_big"), 15
    yield "extra", dict(drift=.02, variant="walls_big"), 15
    for d in (0.0, .04):
        yield "extra", dict(drift=d, mount_dx=.05), 10
        yield "extra", dict(drift=d, mount_yaw_deg=2.0), 10
    yield "extra", dict(drift=0.0, pose_lag_s=.05), 10
    yield "extra", dict(drift=.02, jumps=1.0), 15
    yield "extra", dict(drift=0.0, noise="bad"), 10


def plan(arms, extra=None):
    out = []
    for group, c, n in cells():
        for s in range(n):
            for arm in arms:
                case = dict(c, arm=arm, seed=s)
                if extra and arm in extra:
                    case.update(extra[arm])
                out.append(case)
    return out


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("prefix")
    ap.add_argument("--arms", nargs="+", default=["base", "drift"])
    ap.add_argument("--shards", type=int, default=1)
    ap.add_argument("--extra", default="", help='JSON {arm: {field: value}} e.g. safe parameters')
    a = ap.parse_args()
    p = plan(a.arms, json.loads(a.extra) if a.extra else None)
    for i in range(a.shards):
        part = p[i::a.shards]
        json.dump(part, open(f"{a.prefix}{i}.json", "w"))
        print(f"{a.prefix}{i}.json: {len(part)} cases")
