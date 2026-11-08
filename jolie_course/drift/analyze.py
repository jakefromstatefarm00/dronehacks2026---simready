"""Summarise A/B results:  python analyze.py results1.jsonl [results2.jsonl ...] [--arms base drift safe] [--md]

Per cell (drift, heading, variant, noise, extras) and arm: runs, completion %, runs with any contact,
mean contact steps (50 Hz, TRUE geometry, clearance < 0.28 m), mean / worst true min clearance,
mean course time, correction applied vs drift at the end, residual pose error (end / max).
Paired by seed against the first arm: seeds where the arm is WORSE (more contact steps, or
loses completion) and BETTER.
"""
from __future__ import annotations

import argparse
import json
from collections import defaultdict

import numpy as np

CELL = ("drift", "head", "jumps", "variant", "noise", "mount_dx", "mount_dy", "mount_yaw_deg", "pose_lag_s", "dynamics")


def load(paths):
    rows = {}
    for p in paths:
        import gzip
        for line in (gzip.open(p, "rt") if str(p).endswith(".gz") else open(p)):
            try:
                r = json.loads(line)
            except ValueError:
                continue
            if "error" in r or "complete" not in r:
                continue
            if r["arm"] == "safe":                      # label safe variants by their name
                name = json.loads(r["key"]).get("safe", {}).get("name")
                if name:
                    r["arm"] = name
            rows[r["key"]] = r
    return list(rows.values())


def cell_name(c):
    d = dict(zip(CELL, c))
    s = f"{d['drift'] * 100:.0f} cm/m"
    if d["head"]:
        s += f" +{d['head']} deg/m"
    if d["jumps"]:
        s += f" +jumps {d['jumps']}/10m"
    s += f", {d['variant']}"
    if d["noise"] != "clean":
        s += f", {d['noise']} depth"
    if d["mount_dx"]:
        s += f", mount {d['mount_dx'] * 100:+.0f} cm"
    if d["mount_yaw_deg"]:
        s += f", mount yaw {d['mount_yaw_deg']:+.0f} deg"
    if d["pose_lag_s"]:
        s += f", pose lag {d['pose_lag_s'] * 1000:.0f} ms"
    if d["dynamics"] != "px4_like":
        s += f", {d['dynamics']}"
    return s


def summarise(rows, arms):
    by = defaultdict(lambda: defaultdict(dict))
    for r in rows:
        by[tuple(r.get(k) for k in CELL)][r["arm"]][r["seed"]] = r
    out = []
    for c in sorted(by, key=lambda c: (c[3] != "nominal", c[4] != "clean", c[5] or c[7] or c[8] or 0, c[1] or 0, c[2] or 0, c[0], c[3])):
        arms_here = by[c]
        ref = arms[0]
        seeds = set.intersection(*[set(arms_here[a]) for a in arms if a in arms_here]) if all(a in arms_here for a in arms) else set()
        if not seeds:
            continue
        line = {"cell": cell_name(c), "n": len(seeds), "arms": {}}
        for a in arms:
            rs = [arms_here[a][s] for s in sorted(seeds)]
            cs = np.array([r["contact_steps"] for r in rs])
            line["arms"][a] = dict(
                complete=100 * np.mean([r["complete"] for r in rs]),
                any_contact=int((cs > 0).sum()), contact_steps=float(cs.mean()),
                min_clear=float(np.mean([r["min_clear"] for r in rs])), worst_clear=float(min(r["min_clear"] for r in rs)),
                time=float(np.mean([r["course_s"] for r in rs if r["complete"]])) if any(r["complete"] for r in rs) else float("nan"),
                align=float(np.mean([r["align_total"] for r in rs])), align_max=float(max(r["max_align"] for r in rs)),
                drift_end=float(np.mean([r["drift_end"] for r in rs])),
                resid_end=float(np.mean([r["resid_end"] for r in rs])), resid_max=float(max(r["resid_max"] for r in rs)),
                capped=int(sum(r["align_capped"] for r in rs)))
            if a != ref:
                worse = better = 0
                for s in seeds:
                    x, b = arms_here[a][s], arms_here[ref][s]
                    w = (x["contact_steps"] > b["contact_steps"]) or (b["complete"] and not x["complete"])
                    bt = (x["contact_steps"] < b["contact_steps"]) or (x["complete"] and not b["complete"])
                    worse += int(w and not bt)
                    better += int(bt and not w)
                line["arms"][a].update(worse=worse, better=better)
        out.append(line)
    return out


def print_table(out, arms, md=False):
    if md:
        hdr = "| cell | n | " + " | ".join(f"{a}: complete / contact runs (mean steps) / min clear (worst) / time" for a in arms) \
            + " | " + " | ".join(f"{a} vs {arms[0]}: worse / better seeds" for a in arms[1:]) + " | correction / drift / residual (" + arms[1] + ") |"
        print(hdr)
        print("|" + "---|" * (2 + len(arms) + len(arms) - 1 + 1))
    for l in out:
        cells = []
        for a in arms:
            s = l["arms"][a]
            cells.append(f"{s['complete']:.0f}% / {s['any_contact']} ({s['contact_steps']:.0f}) / "
                         f"{s['min_clear']:.3f} ({s['worst_clear']:.3f}) / {s['time']:.1f}")
        pair = [f"{l['arms'][a]['worse']} / {l['arms'][a]['better']}" for a in arms[1:]]
        s1 = l["arms"][arms[1]] if len(arms) > 1 else l["arms"][arms[0]]
        corr = (f"{s1['align']:.3f} (max {s1['align_max']:.2f}{', capped ' + str(s1['capped']) if s1['capped'] else ''}) / "
                f"{s1['drift_end']:.3f} / {s1['resid_end']:.3f} (max {s1['resid_max']:.2f}); base resid "
                f"{l['arms'][arms[0]]['resid_end']:.3f}")
        if md:
            print(f"| {l['cell']} | {l['n']} | " + " | ".join(cells) + " | " + " | ".join(pair) + f" | {corr} |")
        else:
            print(f"{l['cell']:40s} n={l['n']:3d}  " + "  ||  ".join(f"{a}: {c}" for a, c in zip(arms, cells))
                  + "  || worse/better " + ", ".join(pair) + f"  || {corr}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs="+")
    ap.add_argument("--arms", nargs="+", default=["base", "drift"])
    ap.add_argument("--md", action="store_true")
    ap.add_argument("--json", default="")
    a = ap.parse_args()
    out = summarise(load(a.files), a.arms)
    print_table(out, a.arms, a.md)
    if a.json:
        json.dump(out, open(a.json, "w"), indent=1)
