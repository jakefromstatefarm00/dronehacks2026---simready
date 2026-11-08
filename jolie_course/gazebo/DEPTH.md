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
| Rate | 15 Hz requested. In practice about 3 Hz sim time, because the rig renders 6 cameras at about 0.5x real time. Rendering load does not change: the rig already subscribes the depth camera |

`with_depth.py` starts a second `ros_gz_bridge` in the run's namespace:
`/dronehacks/ideal_depth -> /camera/depth/image_raw` and
`/dronehacks/camera_info -> /camera/depth/camera_info`. gz publishes the depth CameraInfo
there, not on the SDF's `camera_info_topic`. Then it runs the program.

Differences from the lab: the lab uses 16UC1 mm with OAK-D stereo noise and holes. This
depth is perfect and float. The planner decodes both encodings.

## Record and validate

`with_depth.py` also records one [::4, ::4] frame per sim second to `<run>/depth/`
(`depth-frames.npz`, `depth-info.json`, bridge and recorder logs). After
`sim/runtime/post_flight.sh NNN`, fetch the folder and the truth file:

```bash
ssh promaxgb10-6c88 'cd ~/dronehacks-2026-gb10/artifacts && tar czf - gb10-sky-beta-course-vNNN/truth-poses.csv gb10-sky-beta-course-vNNN/depth' | tar xzf -
python jolie_course/gazebo/validate_depth.py gb10-sky-beta-course-vNNN [--airborne] [--sweep]
```

The script projects each frame with the flown file's own `pathplan.depth_projection` at
the truth pose. It prints the planner's "% of slab hits on published walls" and a 3-D check:
the distance of every depth point to the published course OBJ. Results are in
`results/depth-table.txt`.
