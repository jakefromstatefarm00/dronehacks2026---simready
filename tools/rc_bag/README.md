# rc_bag: laptop tools for RC-run bags

Turn the bags recorded by `jetson/record_rc_run.sh` (hand-flown runs of the real course) into a
summary, trajectories and a depth-derived map of the **real** course compared with the published
one (`pathfinding/course/dronehacks-course-v3.obj`). These tools only read bags. They need no ROS:
just Python 3.10+ with numpy, matplotlib and [rosbags](https://pypi.org/project/rosbags/) (tested
with rosbags 0.11.9).

```
pip install rosbags numpy matplotlib pytest
scp -r dronehacks@skybeta-06.local:~/team_ws/runs/rc-run1-20261107-101500 .
python -I tools/rc_bag/summarize.py  rc-run1-20261107-101500/bag      # -> summary.md
python -I tools/rc_bag/trajectory.py rc-run1-20261107-101500/bag      # -> CSV + PNGs
python -I tools/rc_bag/depth_map.py  rc-run1-20261107-101500/bag      # -> occupancy grid + wall report
```

Use `python -I` (isolated mode) for anything that reads bag data. The scripts add their own folder
to `sys.path`. Outputs go to `./rc_bag_out/<run name>/`, or to the folder you pass with `--out DIR`.

`BAG` can be any of these:
- a bag folder with `metadata.yaml`. sqlite3 `.db3` (Humble, the drone) and `.mcap` (Jazzy) both work, and so do bags split every 2 GB.
- a bag folder **without** `metadata.yaml`, left behind when the recorder was killed. Every
  `*.db3` / `*.mcap` in the folder is read directly, in split order.
- a single `.db3` / `.mcap` file.

## Scripts

| script | what it does | outputs |
|---|---|---|
| `read_bag.py BAG [--echo TOPIC -n 3]` | Library plus CLI. Lists topics, types and counts, or prints messages. | stdout |
| `summarize.py BAG` | Per-topic count, rate, max gap and gaps; flight timeline from `/skybeta/state` (armed, landed, offboard, nav_state, px4_link) and `vehicle_status_v1` (arming, nav_state, failsafe, armed time); `failsafe_flags`; RC sticks from `manual_control_setpoint`; battery; VIO health (gaps > 100 ms, jumps > 0.3 m, speed, covariance); depth stream stats; `tegrastats.log` CPU/GPU/RAM/temperature/power; `notes.txt` and `check.txt` when they are next to the bag | `summary.md` |
| `trajectory.py BAG [--align first-pose\|odom-origin\|none]` | Exports the cuVSLAM odometry, the PX4 `vehicle_local_position` (NED converted to ENU; falls back to `vehicle_odometry`) and `/skybeta/state local_enu` | `trajectory_{vio,px4,skybeta}.csv`, `trajectory_topdown.png` (over the published course), `trajectory_height.png` |
| `depth_map.py BAG [--align ...] [--stride 3] [--res 0.05] [--band 0.5] [--pose auto\|vio\|px4]` | Projects sampled depth frames through the time-matched pose into a 2-D occupancy grid at flight height. Compares the result with the published course. | `depth_map.png`, `depth_occupancy.npy` (frames per cell, uint16, row = y), `depth_occupied.npy` (bool), `depth_map.json`, `depth_map.md` |
| `course_geom.py` | Loads the published course through `pathfinding/drl/course.py`, cuts it at a height band into 2-D footprints, and holds the odom-to-course alignment | (library) |
| `test_rc_bag.py` | Synthetic bag test (see below) | |

### Alignment (`--align`)
The course frame is metres, Z up, origin at the centre of the floor. The drone starts on the launch
mark (-2.515, 0.914) with heading pi/2 (+y).
- `first-pose` (default): each trajectory's own first pose is placed on the launch mark, facing +y.
  This assumes the recording started with the drone sitting on the launch mark. `record_rc_run.sh`
  refuses to start while the drone is armed.
- `odom-origin`: the odom origin is taken as the launch mark (the flight code's `OdomFrame`
  convention). Use it when the bag started after VIO had already moved.
- `none`: raw odom frame. Nothing is compared with the course.

### What depth_map.py does
1. It reads every 3rd depth frame (`/camera/depth/image_raw`, 16UC1 mm) and every 4th pixel. It
   keeps ranges from 0.35 to 4.0 m and drops 0 and values ≤ 0.314 m (the OAK's "too close"
   reading).
2. It projects each pixel with fx/fy/cx/cy from `/camera/depth/camera_info`, or fx = 398.8 when
   that topic is missing. Each pixel goes from the optical frame (z forward, x right, y down) to
   base_link (camera at x 0.1053, z 0.0304, no rotation), then to odom using the nearest
   cuVSLAM pose (header stamps, within 0.1 s; position interpolated).
3. It removes points below floor + 0.15 m. It keeps points within ±0.5 m of the flight height,
   which is the median height while the drone is more than 0.3 m above the start.
4. It builds a 5 cm grid that counts the distinct frames seeing each cell. A cell is occupied
   when at least 2 frames saw it.
5. It compares the grid with the published course cut at the same height band (walls, window wall,
   boulders, tube shell, net):
   - the **median and 90th-percentile distance** from occupied cells inside the net to the nearest
     published surface;
   - **new obstacles**: connected groups of ≥ 6 occupied cells more than 0.25 m from anything
     published;
   - **published walls confirmed by depth**: for each wall, the share of its outline within range of
     the path that has an occupied cell within 15 cm. A low value next to the flown path points to
     a wall that moved or is missing.

A bag with depth but no pose (no VIO, no decodable PX4 pose) reports **"no odometry"**. It still
summarises the depth stream (`depth_stats.png`, `depth_map.md`) and does not crash.

## px4_msgs
The `/fmu/out/*` topics use px4_msgs types, which aren't built into rosbags. The tools register them
from `.msg` files found in the first of these places:
1. `--px4-msgs DIR`;
2. `$RC_BAG_PX4_MSGS`;
3. a `px4_msgs/msg` folder inside this repo.

Bags that embed their message definitions register their types automatically. That covers Jazzy
mcap and Iron+ sqlite, but **not** Humble sqlite, which is what the drone records. Without the
`.msg` files, the `/fmu` topics are still counted in the topic table, marked "(undecodable)" and
skipped. Use the same px4_msgs version as the drone, or the CDR decoding will be wrong. On the
Jetson it is under the container's workspace (`.../src/px4_msgs/msg`):

```
docker cp skybeta-stack:/root/<ws>/src/px4_msgs/msg ./px4_msgs_msg     # on the Jetson host, then scp
python -I tools/rc_bag/summarize.py BAG --px4-msgs ./px4_msgs_msg
```

## Test

```
python -I -m pytest tools/rc_bag/test_rc_bag.py -q
```

The test builds a fake bag with rosbags' `Writer`. The drone climbs to 1 m, then flies 2 m sideways
while depth sees a flat wall 1.5 m ahead. The test checks that:
- `trajectory.py` reproduces the path;
- `depth_map.py` puts the wall within 10 cm of where it is, in both the odom frame and the course
  frame;
- the bag still reads when `metadata.yaml` is deleted, for sqlite3 and mcap;
- a depth-only bag reports "no odometry" without crashing.

## Limits
- Occupancy counts hits only, with no free-space ray casting. Moving people and the net show up as
  obstacles. Cells outside the net are left out of the comparison.
- The accuracy of the comparison depends on VIO drift and on the start pose being on the launch
  mark. Look at `trajectory_topdown.png` first. If the walls are seen consistently offset, suspect
  the alignment before you conclude the course moved.
- PX4 and VIO trajectories are aligned separately, each by its own first pose. Their difference
  shows drift, not a frame offset.
