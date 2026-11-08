# cornerfix: the first corridor-to-maze corner and VIO divergence

Goal: stop OpenVINS from diverging at the first corner (top of the first corridor, where the
route turns from north to east and then south around maze divider M1). We made targeted changes
to Jolie's FlyX-proven `upload_to_flyx.py` and A/B-tested them in the GB10 Gazebo rig (real PX4
SITL + OpenVINS).

| File | What |
|---|---|
| `build_cornerfix.py` | Builder. Reads `../upload_to_flyx.py` (never writes it), makes marked insertions, and writes the variants and their Sky Beta copies. The Sky Beta copies come from `../build_skybeta_copy.py`'s own `build()`. |
| `../upload_to_flyx_cornerfix_slow.py` (+ `_skybeta`) | Variant **slow** |
| `../upload_to_flyx_cornerfix_view.py` (+ `_skybeta`) | Variant **view** |

```
python3 jolie_course/cornerfix/build_cornerfix.py      # regenerates all four
```

Both variants leave everything else in her file unchanged: the yaw cap of 0.25 rad/s, turning in
place beyond 60 deg, the carrot that shrinks while turning, pad centring and depth handling. Each
embedded module stays byte-identical apart from the marked `cornerfix` insertions:
- a helper block and one `corner_rule(...)` call in `pathplan.planner_process`;
- two `MissionConfig` fields and three lines in `Mission._carrot_for` in `flyx.mission`.

Each plan is journalled with `reason` = `corner-slow` / `corner-view` / `corner-view-slow`.

## 1. Analysis of the existing Gazebo flights

__ANALYSIS__

## 2. Variants (stated as rules, no coordinates)

**Corner mode** is on when either holds:
- the planned route turns by more than 45 deg within 1.2 m ahead (direction measured over 0.3 m);
- the nose is still more than 20 deg off the planner's heading.

Corner mode stays on while the route ends within 1.2 m. That end is a checkpoint goal, and the
planner cannot yet see the next leg past it.

**View distance** is measured on the planner's own occupancy map (the published walls plus the
border net): how far the forward camera sees along a heading, capped at 2 m.

- **slow:** in corner mode, if the camera sees less than 1.5 m (along its current or its
  commanded heading), the carrot is halved (floor 0.12 m), so the drone flies at about half
  speed. Heading is unchanged.
- **view (perception-aware heading):** in corner mode, while the route turns ahead (not merely
  while the nose is still turning), the commanded heading may move up to 40 deg further into the
  turn, in 10 deg steps. It takes the first heading that sees at least 1.5 m, or else the heading
  that sees farthest (each extra step must gain more than 0.1 m). The 40 deg limit keeps it inside
  her 60 deg turn-in-place margin. It then slows down as **slow** does if the view is still short.
  In effect, the nose turns toward the next leg before the corner, while the drone moves slowly,
  and keeps the camera on a surface at least 1.1-1.5 m away rather than on the net at 0.75 m.

Offline kinematic check (point mass, no sensing; `kinsim.py` in the work notes) over the first
corner, from y = 1.5 m to 3 s after checkpoint 1:

| program | time with the camera < 1.0 m from a surface while turning | min view while turning | whole course (kinematic) |
|---|---|---|---|
| base | 3.9 s | 0.80 m | 70 s |
| slow | 2.2 s | 0.77 m | 84 s |
| view | 0.0 s | 1.10 m | 83 s |

The view rule also adds some yaw: over the whole course, 10 yaw-rate reversals instead of 2, and
13 % more total yaw, all still at or below the cap.

## 3. A/B in Gazebo

__AB__

## 4. Recommendation

__REC__
