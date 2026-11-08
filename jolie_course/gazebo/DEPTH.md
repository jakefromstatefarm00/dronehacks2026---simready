# Depth camera in the GB10 Gazebo rig (sim only, opt-in)

The FlyX lab gives the planner `/camera/depth/image_raw` + `/camera/depth/camera_info`.
The GB10 rig renders an ideal depth camera but, by default, only bridges it as
`/camera/ideal_depth/image` with no CameraInfo, so `upload_to_flyx.py` flew with
"depth: no frames used" (baseline v200-v214). `rig/with_depth.py` adds the lab topics
for one run. Default runs are unchanged. No flight file and no PX4 parameter is touched.

## Enable

```bash
# on GB10, from ~/dronehacks-2026-gb10 (deployed copies live in jolie_flyx/)
DEPTH=1 jolie_flyx/launch.sh NNN jolie_flyx/upload_to_flyx.py [program args]
# or a queue line (jolie_flyx/queue.txt):
NNN jolie_flyx/with_depth.py jolie_flyx/upload_to_flyx.py [program args]
# or by hand:
NATIVE_PROGRAM="$HOME/dronehacks-2026-gb10/jolie_flyx/with_depth.py \
  $HOME/dronehacks-2026-gb10/jolie_flyx/upload_to_flyx.py --output $HOME/dronehacks-2026-gb10/artifacts/gb10-sky-beta-course-vNNN/journal" \
  nohup sim/runtime/gb10_slot_run.sh artifacts/published-course-continuous-k12/trajectory.csv \
  configs/sky_beta_course_continuous_k12_ff.json artifacts/gb10-sky-beta-course-vNNN
```

You don't need any program flags. `upload_to_flyx.py` already defaults to
`--depth-topic /camera/depth/image_raw`, the sibling camera_info, and
`--camera-offset 0.115 0 0.029 --camera-pitch-down-deg 0`, which is the rig's mount.

## What it is

| | |
|---|---|
| Sensor | `ideal_depth`, gz `depth_camera` on `sky_beta_estimated/base_link` (world SDF built by `sim/sky_beta_flight_model.py`) |
| Mount | (0.11466, -0.00015, 0.02891) m in base_link FLU, no rotation. That is the planner default, to within 0.2 mm |
| Image | 640x400, 32FC1 metres (planar z-depth), clip 0.1-10 m, +inf = no return; no noise |
| Intrinsics | gz CameraInfo: fx = fy = 381.361, cx = 320, cy = 200, no distortion (= 320 / tan(hfov/2), hfov 1.3962634 rad = 80 deg) |
| Stamps | Gazebo sim time (same clock as /clock and PX4 SITL). Receive lag is about one sim frame (33 ms) |
| Rate | 15 Hz requested. In practice 3-6 Hz sim time, because the rig renders 6 cameras below real time. Depth-on flights used 310-700 frames. Rendering load does not change: the rig already subscribes the depth camera |

In the run's namespace, `with_depth.py` starts (default `--depth-source relay`):

* `image_transport republish` (C++), which republishes the rig's own `/camera/ideal_depth/image`
  as `/camera/depth/image_raw`. The header (sim stamp) is kept, and Gazebo gets no second
  subscriber.
* a tiny `ros_gz_bridge` for `/dronehacks/camera_info -> /camera/depth/camera_info`.
  gz publishes the depth CameraInfo there, not on the SDF's `camera_info_topic`.

`--depth-source bridge` (v402-v419) bridges `/dronehacks/ideal_depth` a second time instead.
Then the wrapper runs the program in its own session and forwards SIGINT/SIGTERM. Topic names
come from the rig's `gazebo-topics.txt`, not from `gz topic -l`. The Ruby CLI's CPU burst
lands exactly at the bootstrap-fixture drop.

Differences from the lab: the lab uses 16UC1 mm with OAK-D stereo noise and holes. This
depth is perfect and float. The planner decodes both encodings.

## Record and validate

With `--depth-record-period 1` (off by default: the recorder is a second 1 MB/frame
subscriber), `with_depth.py` records one [::4, ::4] frame per sim second to `<run>/depth/`
(`depth-frames.npz`, `depth-info.json`, recorder log). It starts 20 s after the program.
The bridge config and log are always written there. After
`sim/runtime/post_flight.sh NNN`, fetch the folder and the truth file:

```bash
ssh promaxgb10-6c88 'cd ~/dronehacks-2026-gb10/artifacts && tar czf - gb10-sky-beta-course-vNNN/truth-poses.csv gb10-sky-beta-course-vNNN/depth' | tar xzf -
python jolie_course/gazebo/validate_depth.py gb10-sky-beta-course-vNNN [--airborne] [--sweep]
```

The script projects each frame with the flown file's own `pathplan.depth_projection` at
the truth pose. It prints the planner's "% of slab hits on published walls" and a 3-D check:
the distance of every depth point to the published course OBJ. Results are in
`results/depth-validation-*.txt`; flights are scored with `depth_table.py`.

## Results (Nov 8)

**Geometry.** Checked at the truth pose with the flown file's `project_depth`:

* v401, on the ground: 100 % of slab hits are on published walls, and 100 % of 3-D
  points lie within 5 cm of the OBJ.
* v406, 33 airborne frames: 100 % of slab hits are on published walls, and 100 % of
  14.5 k points lie within 5 cm of the OBJ (median 0.0 cm).

A 5 deg yaw error drops the within-5-cm share to 64 %, and +10 cm in z drops it to 89 %.
So the default mount is right, and no frame fix is needed. Stamps match odometry:
280 of 282 frames matched within 50 ms.

**A/B.** Unchanged `upload_to_flyx.py`, interleaved on/off runs, one slot, v407-v424 (load
gate < 20 at launch; `rig/ab_series.sh`). See `results/depth-ab-table.txt`.

* Several runs in both arms lost VIO on the ground or at takeoff and never flew the course:
  off v409, v411, v421 (3/8); on v408, v410, v412, v417 (4/11). Other agents' runs failed
  the same way that night, including runs without depth. Baseline v200-v214 had none.
* Every run that got past takeoff completed: off 5/5, on 6/6. NAVIGATE mean was
  64.8 s off and 63.9 s on.
* Minimum truth clearance was >= 0.29 m on 4/5 off runs (v424 0.214) and 3/6 on runs
  (worst 0.276).
* In healthy depth-on runs the planner reported 100 % of hits on published walls and
  0 new obstacles.
* New obstacles (129-414, wall share 71-90 %) appeared only after VIO had already
  diverged. They are phantom walls from a wrong pose. In v410 they kept the drone in
  "escape" at the start until the flight time limit.

**Drift-correction build** (jolie-drift-correction f7b97d3, depth on, v425-v427): 3/3
completed, NAVIGATE mean 69.2 s. Min clearance was 0.24-0.27 m, so all 3 runs were below
0.28 m. The total map alignment was 0.09-0.11 m per flight, while truth VIO error in healthy
runs here is about 0.05 m. The correction may be adding error, not removing it.
