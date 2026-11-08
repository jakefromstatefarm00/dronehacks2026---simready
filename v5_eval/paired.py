"""Paired (same level, same seed) comparison of arms against v4c.

    python residual/paired.py ALL.jsonl ARM [ARM ...]
Per level: n paired, completion base/arm, only-base-completed / only-arm-completed (discordant pairs,
exact two-sided sign-test p), contacts base/arm, newly-contacting seeds, mean NAVIGATE delta on
seeds both completed, VIO events, look-arounds, in-place yaw near walls.
"""
import json
import math
import sys
from pathlib import Path


def sign_p(a, b):
    n = a + b
    if n == 0:
        return 1.0
    k = min(a, b)
    return min(1.0, 2 * sum(math.comb(n, i) for i in range(k + 1)) / 2 ** n)


rows = [json.loads(l) for l in Path(sys.argv[1]).read_text().splitlines() if l.strip()]
idx = {(r["arm"], r["level"], r["seed"]): r for r in rows}
levels = list(dict.fromkeys(r["level"] for r in rows))
print("| level | arm | n | done base/arm | only base / only arm (p) | contacts base/arm | new contact seeds | "
      "NAV delta s (both done) | VIO ev base/arm | look-arounds base/arm | in-place wall yaw rad base/arm |")
print("|" + "---|" * 11)
for arm in sys.argv[2:]:
    for lv in levels:
        pairs = [(idx[("v4c", lv, s)], idx[(arm, lv, s)]) for (a, l, s) in idx if a == arm and l == lv and ("v4c", lv, s) in idx]
        if not pairs:
            continue
        n = len(pairs)
        cb, ca = sum(b["complete"] for b, _ in pairs), sum(x["complete"] for _, x in pairs)
        ob = sum(b["complete"] and not x["complete"] for b, x in pairs)
        oa = sum(x["complete"] and not b["complete"] for b, x in pairs)
        kb, ka = sum(b["contact"] for b, _ in pairs), sum(x["contact"] for _, x in pairs)
        newc = [x["seed"] for b, x in pairs if x["contact"] and not b["contact"]]
        both = [(x["course_time"] - b["course_time"]) for b, x in pairs if b["complete"] and x["complete"]]
        la = lambda r: sum(1 for e in r["events"] if e[1] == "look around")
        print(f"| {lv} | {arm} | {n} | {cb}/{ca} | {ob} / {oa} (p={sign_p(ob, oa):.2f}) | {kb}/{ka} | {newc or '-'} | "
              f"{(sum(both) / len(both)) if both else float('nan'):+.1f} | {sum(b['vio_events'] for b, _ in pairs)}/"
              f"{sum(x['vio_events'] for _, x in pairs)} | {sum(la(b) for b, _ in pairs)}/{sum(la(x) for _, x in pairs)} | "
              f"{sum(b['wall_yaw_rad'] for b, _ in pairs):.1f}/{sum(x['wall_yaw_rad'] for _, x in pairs):.1f} |")
