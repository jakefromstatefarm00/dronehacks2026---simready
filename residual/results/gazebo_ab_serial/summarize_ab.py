"""Per-arm summary of residual.score_gazebo rows.

    python3 summarize_ab.py <artifacts dir> <label>=<spec>[,<spec>] ...
e.g.  python3 summarize_ab.py artifacts "serial base"=v600,v602,v604,v606,v608 "serial residual"=v601,v603,...

Prints the per-run markdown table (scorer output) and a per-arm table: completed n/N, NAVIGATE
time mean +/- sd (completed runs), checkpoints, land reasons, corner yaw-rate x speed peak, yv peak.
"""
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path.home() / "dronehacks-2026-gb10"))
from residual.score_gazebo import score, expand  # noqa: E402


def ms(xs):
    xs = [x for x in xs if x is not None]
    if not xs:
        return "-"
    if len(xs) == 1:
        return f"{xs[0]:.1f} (n=1)"
    return f"{statistics.mean(xs):.1f} +/- {statistics.stdev(xs):.1f}"


def ms3(xs):
    xs = [x for x in xs if x is not None]
    if not xs:
        return "-"
    if len(xs) == 1:
        return f"{xs[0]:.3f}"
    return f"{statistics.mean(xs):.3f} +/- {statistics.stdev(xs):.3f}"


def main():
    root = Path(sys.argv[1])
    arms = []
    for a in sys.argv[2:]:
        label, spec = a.split("=", 1)
        runs = [v for s in spec.split(",") for v in expand(s)]
        arms.append((label, runs))
    cols = ["run", "arm", "residual", "complete", "checkpoints", "navigate_s", "land_reason", "diverged_at_s",
            "corner_yv_peak", "yv_peak", "yv_above_0p10_s", "yaw_rate_peak", "speed_mean"]
    per_run = {}
    print("| " + " | ".join(cols) + " |\n|" + "---|" * len(cols))
    for label, runs in arms:
        for v in runs:
            run = root / f"gb10-sky-beta-course-{v}"
            if not (run / "native-program.log").exists():
                per_run.setdefault(label, []).append({"run": v, "missing": True})
                print(f"| {v} | {label} | (no native-program.log) |" + " |" * (len(cols) - 3))
                continue
            if not (run / "truth-poses.csv").exists() or "RESULT:" not in (run / "native-program.log").read_text(errors="replace"):
                per_run.setdefault(label, []).append({"run": v, "missing": True})
                print(f"| {v} | {label} | (no RESULT / truth-poses.csv) |" + " |" * (len(cols) - 3))
                continue
            r = score(run)
            r["arm"] = label
            per_run.setdefault(label, []).append(r)
            print("| " + " | ".join(str(r.get(c, "")) for c in cols) + " |")
    print()
    hdr = ["arm", "runs", "completed", "NAVIGATE s (completed)", "checkpoints", "land reasons", "corner yv peak", "yv peak (all)",
           "yv>0.10 s", "diverged_at_s"]
    print("| " + " | ".join(hdr) + " |\n|" + "---|" * len(hdr))
    for label, runs in arms:
        rs = [r for r in per_run.get(label, []) if not r.get("missing")]
        n = len(per_run.get(label, []))
        comp = [r for r in rs if r["complete"]]
        reasons = {}
        for r in rs:
            k = (r["land_reason"] or r["result"])[:45]
            reasons[k] = reasons.get(k, 0) + 1
        cps = " ".join(str(r["checkpoints"]) for r in rs)
        print("| " + " | ".join([
            label, f"{runs[0]}..{runs[-1]} (n={n})", f"{len(comp)}/{n}",
            ms([r["navigate_s"] for r in comp]), cps,
            "; ".join(f"{k} x{c}" for k, c in reasons.items()),
            ms3([r["corner_yv_peak"] for r in rs]), ms3([r["yv_peak"] for r in rs]),
            ms([r["yv_above_0p10_s"] for r in rs]),
            " ".join(str(r["diverged_at_s"]) for r in rs if r["diverged_at_s"] is not None) or "-",
        ]) + " |")


if __name__ == "__main__":
    main()
