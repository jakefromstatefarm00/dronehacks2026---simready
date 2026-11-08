# Map alignment (VIO drift correction): offline A/B and decision

Branch `drift-eval`. This compares Jolie's FlyX-proven planner (`jolie-flyx-sources` e1ac88d,
= `jolie_course/upload_to_flyx.py`) with her map alignment on top of it (`jolie-drift-correction`
f7b97d3). Everything here is new. Her branches and the frozen files were not touched.

## How the alignment works

Each depth frame (13 Hz) is projected from the reported pose. The hit points within 0.35 m of a
published wall then go through a Gauss-Newton fit on the signed distance field of the published
walls, sampled bilinearly, for at most 5 iterations. The fit corrects a direction only if the walls
constrain it (Hessian eigenvalue > max(10, 2 % of the hits)), so a straight corridor corrects only
sideways. It refuses frames with too few hits (< 40), no rms gain (< 3 mm), or a shift > 0.3 m.
At each 5 Hz planning step, `fly_pathplan` moves the course frame's origin by 0.6 x the measured
shift. Each step moves it at most 4 cm, and the total stops at 1.5 m. Planning, setpoints and the
next frames' projection all use the moved frame. The method is a proportional observer that pulls
the frame until depth sits on the published walls.

Failure modes, by construction:
* **Moved / unmapped geometry:** it aligns to the published map. If the walls in view moved
  coherently (or one wall is all it sees), a moved wall looks exactly like drift. The frame then
  takes on that wall's offset (up to the wall shift) until other walls pull it back. The planner's
  `drop_misregistered` already ignores depth within 0.25 m of a published wall, so neither arm
  sees a wall that moved < 25 cm.
* **Noisy depth:** single-frame shifts are noisy. The 4 cm / step clip limits each kick, but kicks
  that take the error past the capture range (0.35 m band, 0.3 m refusal) lock onto the wrong wall.
  The error then grows to the 1.5 m cap.
* **Degenerate views:** in a long corridor the along-corridor direction is never corrected. That is
  correct, but the error there keeps growing until a cross wall is seen. Grazing side walls give
  little evidence.
* **Mount offset / mount yaw / heading drift:** a wrong camera offset makes a constant bias, which
  the frame absorbs (a few cm). Heading error is not modelled: it shows up as a translation that
  changes as the view changes.

## The offline A/B (`ab_sim.py`)

* Runs each branch's own `flyx.mission.Mission` (state machine, carrot, yaw cap, pad centring) and
  `pathplan.planner_process.planner_process` (depth projection, wall snap, votes, theta*, and
  `align_to_map` on the drift branch), on a thread with an in-memory pipe.
* `fly_pathplan`'s glue (pose at the frame stamp, turn-rate filter, 5 Hz planning, 20 Hz control,
  `apply_alignment`) is re-implemented line for line. The alignment lands 0.1 s after the plan
  starts, the way the real planning thread does.
* PX4: a P position loop with a velocity lag (0.3 s), 1.5 m/s and 3 m/s², flying the estimated
  pose.
* VIO drift (estimate - truth), as a function of distance flown with common random numbers:
  * a bias of `drift` m/m in a slowly wandering direction, plus white noise;
  * optional heading drift (deg/m) and jumps (N(0, 8 cm) per axis, 1 per 10 m).
* Depth: `fake_px4.render_depth` (640x400, FlyX intrinsics), cast from the TRUE pose on the TRUE
  (perturbed) course. Noise options:
  * noisy: σ = 1 cm + 1.5 %·z², in 8x8-pixel blocks, 5 % dropout, 0.5 % outliers;
  * bad: σ = 2 cm + 3 %·z², 15 % dropout, 2 % outliers.
* Course variants (`pathplan/variants.py`): walls (maze, chicane and approach walls moved ±15 cm),
  walls_big (±30 cm), clutter (1-3 unmapped posts by the route).
* Metrics are on TRUE geometry:
  * contact steps (50 Hz, clearance < 0.28 m);
  * min clearance;
  * completion: landed through the planner's pad logic, all 4 lines crossed by the true track, and
    landed inside the ring;
  * course time;
  * correction applied vs drift, and the residual pose error.

The fake-PX4 kinematics are faster than FlyX: courses here take ~65 s, where FlyX took 120.9 s.

__RESULTS__
