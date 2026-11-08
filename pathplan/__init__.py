"""Online path planning on a map seeded by the published layout and corrected by
what the drone senses.  Planning never reads simulator truth.  The only module
that can command the vehicle is ``ros_node`` (position setpoints, and only with
``publish_setpoints:=true``; it never arms, switches mode or lands).

Layers (what the drone would run)
---------------------------------
world        GridSpec / Envelope, course -> occupancy + cost grids, checkpoints.
belief       BeliefMap: the drone's map; integrates observations, gives the cost grid.
grid_search  Theta* (default) and A* route search, shared line/edge cost model.
online       OnlinePlanner: the drone-side brain (observe depth cells, update pose -> command).
depth_projection  Real depth image + intrinsics + pose -> map cells.
ros_node     ROS 2 wrapper (shadow mode by default; not yet run).

Test bench (stands in for the world and for comparisons)
--------------------------------------------------------
mission      Simulated world around OnlinePlanner: kinematic motion + contact check.
sensor       Depth-camera model that reveals the true course in a view fan.
variants     Perturbed courses for generality testing.
harness      Batch evaluation of planners over variants: runs.csv, summary.json, PNGs.
dstar_lite   D* Lite incremental replanning (experimental, off by default).
"""
