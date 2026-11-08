# FlyX scripts that have flown successfully

Each file is a whole, self-contained program: paste it into the FlyX Workbench editor, save, press
**Reset world**, then **Run saved file** (or run `python3 <file>` in the terminal).

All of these are native PX4: they publish `/fmu/in` themselves. They are for the FlyX sim only, not
for the Sky Beta drone.

| File | What it is | Source | FlyX result | sha256 (first 12) |
|---|---|---|---|---|
| `upload_to_flyx.py` | Jolie's proven planner | `origin/main:jolie_course/upload_to_flyx.py` | Completed the whole course, 2026-11-07. NAVIGATE 120.9 s, arm→disarm 137.1 s, pad error 0.07 m | a97f94e9ef36 |
| `upload_to_flyx_v3.py` | Jolie's v3: centres the route in narrow channels, recovers instead of landing | `origin/jolie-flyx-v2:pathfinding/flyx/upload_to_flyx.py` (71b2890) | The team judged it better than the proven file in FlyX, 2026-11-07 | 97c193b6b9b6 |
| `upload_to_flyx_cornerfix_view.py` | The proven file plus the first-corner "view" rule: at sharp turns, points the nose up to 40° further into the turn to keep the camera on a far surface | `origin/first-corner-vio:jolie_course/upload_to_flyx_cornerfix_view.py` (9804076) | Successful FlyX run, 2026-11-07 | a83dd1889015 |
| `upload_to_flyx_704.py` | Jolie's v4c: no turning near walls, forgets phantom obstacles first, drift-aware phantom filter, turns earlier, steps out of the keep-out zone before replanning, backs out along the flown track (not in a straight line through walls), recoveries per checkpoint | `origin/jolie-flyx-v2` at **1d44162**, `pathfinding/flyx/upload_to_flyx.py` (same content; line endings differ). The file's header says b142eb5 only because it was built just before that commit | Completed, 2026-11-08 run flyx-20261108-025844: 5/5 checkpoints, NAVIGATE 119.9 s, arm→disarm 136.2 s, pad error 0.08 m; depth 1534 frames, 99 % of hits on published walls | feb73d6434da |

Add a file here only after it has had a successful FlyX run. Copy it from Git rather than editing
it here, and record the run in the table above.

Not here yet, because they have not flown in FlyX: `../flyx_best/upload_to_flyx_v3_cornerfix_view.py`
and `upload_to_flyx_v3_cornerfix_slow.py`, which apply the same corner rules to v3.
