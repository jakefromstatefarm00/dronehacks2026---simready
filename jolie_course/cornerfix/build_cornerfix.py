"""Build the first-corner VIO variants of Jolie's FlyX-proven upload_to_flyx.py, and their Sky Beta copies.

    python3 jolie_course/cornerfix/build_cornerfix.py

upload_to_flyx.py is read, never written. Each variant keeps every embedded module byte for byte
except two small, marked insertions ("cornerfix"):

  * pathplan.planner_process (planner child): before the carrot trims the route, ``corner_rule``
    looks for corner mode: the route turns by more than CORNER_TURN_DEG within CORNER_WITHIN_M
    ahead, or the nose is still more than CORNER_TURNING_DEG off the route. In corner mode the
    "view" variant may move the commanded heading up to VIEW_MAX_DEV_DEG further into the turn,
    to the first heading whose forward view on the known map (published walls, net) reaches
    VIEW_ENOUGH_M (else the one that sees farthest). Then, if the camera faces the map closer
    than VIEW_ENOUGH_M (at the current or the commanded heading), the command is tagged
    "corner[-view]-slow".
  * flyx.mission: ``_carrot_for`` scales the carrot of a "...-slow" command by corner_carrot_scale
    (floor corner_carrot_min_m): the drone flies those stretches at about half speed.

The yaw-rate cap (0.25 rad/s), turn-in-place at 60 deg and the turn slow-down are unchanged.

Variants (see VARIANTS):
  slow : half speed in corner mode while the camera faces a near (< 1.5 m) surface. Heading unchanged.
  view : the perception-aware heading above, then slow as "slow" does if the view is still near.

The Sky Beta copies are made with jolie_course/build_skybeta_copy.py's own build(), pointed at the
variant (same PX4-I/O swap, byte-identical embedded modules).
"""
from pathlib import Path
import importlib.util
import sys

HERE = Path(__file__).resolve().parent
JC = HERE.parent
SRC = JC / "upload_to_flyx.py"

CORNER_HELPERS = '''

# ------------------------------------------------------------------------------------------------
# cornerfix (jolie_course/cornerfix/build_cornerfix.py, variant {name!r}). Gazebo + OpenVINS: VIO
# diverged at the first corridor-to-maze corner (v209), where the nose sweeps across the net at
# 0.75-0.95 m while the drone moves and turns; OpenVINS's SLAM features fall from ~75 to ~0-20.
CORNER_TURN_DEG = 45.0     # a route turn sharper than this ...
CORNER_WITHIN_M = 1.2      # ... this far ahead (or the nose still turning, below) = corner mode
CORNER_TURNING_DEG = 20.0  # nose still this far off the route direction = corner mode
CORNER_SEG_M = .3          # the route's direction is measured over this length
CORNER_VIEW = {view}       # also pick a heading that keeps the camera on a far surface
VIEW_ENOUGH_M = 1.5        # forward view on the known map that counts as far enough (else: slow)
VIEW_MAX_DEV_DEG = 40      # never more than this beyond the planner's heading (turn-in-place is 60)
VIEW_STEP_DEG = 10


def route_turn_ahead(path, within=CORNER_WITHIN_M, seg=CORNER_SEG_M):
    """Signed largest change of the route's direction within ``within`` m ahead (rad, + = left)."""
    from .online import _point_along, _wrap
    path = np.asarray(path, float)
    p0, p1 = _point_along(path, 0.0), _point_along(path, seg)
    if np.linalg.norm(p1 - p0) < 1e-6:
        return 0.0
    h0 = float(np.arctan2(p1[1] - p0[1], p1[0] - p0[0]))
    best = 0.0
    for s in np.arange(.1, within + 1e-9, .1):
        a, b = _point_along(path, s), _point_along(path, s + seg)
        if np.linalg.norm(b - a) < .05:                        # the route ends here
            break
        d = _wrap(float(np.arctan2(b[1] - a[1], b[0] - a[0])) - h0)
        if abs(d) > abs(best):
            best = d
    return best


def view_distance(occ, spec, xy, heading, cap=2.0, step=.05):
    """How far the forward camera sees along ``heading`` on the occupancy map (m, capped)."""
    d = np.arange(step, cap + 1e-9, step)
    pts = np.asarray(xy, float) + d[:, None] * np.array([np.cos(heading), np.sin(heading)])
    rc = spec.to_cell(pts)                                     # clipped: the border (net) is occupied
    hit = occ[rc[:, 0], rc[:, 1]]
    return float(d[int(np.argmax(hit))]) if hit.any() else cap


def view_heading(occ, spec, xy, want, sign):
    """The planner's heading, or up to VIEW_MAX_DEV_DEG further into the turn: the first that sees
    VIEW_ENOUGH_M, else the one that sees farthest (a deviation must gain > 0.1 m)."""
    best, best_d = want, view_distance(occ, spec, xy, want)
    if best_d >= VIEW_ENOUGH_M or sign == 0:
        return want
    for k in range(VIEW_STEP_DEG, VIEW_MAX_DEV_DEG + 1, VIEW_STEP_DEG):
        h = want + sign * np.deg2rad(k)
        d = view_distance(occ, spec, xy, h)
        if d >= VIEW_ENOUGH_M:
            return float(np.arctan2(np.sin(h), np.cos(h)))
        if d > best_d + .1:
            best, best_d = h, d
    return float(np.arctan2(np.sin(best), np.cos(best)))


def corner_rule(planner, cmd, xy, yaw):
    """Near a sharp route turn: in the view variant steer the heading toward a far surface; then,
    if the camera (now, or at the commanded heading) faces the known map closer than VIEW_ENOUGH_M,
    tag the command "...-slow" so the mission flies it at half speed. Other commands are untouched.
    reason: corner-slow | corner-view | corner-view-slow (journalled).

    Corner mode is sticky while the route ends within CORNER_WITHIN_M (a checkpoint goal, beyond
    which the planner does not see the next leg yet): without that the nose swung back toward the
    corner just before checkpoint 1 (offline kinematic check)."""
    from .online import _wrap
    if cmd.mode != "move" or cmd.reason or cmd.yaw is None or cmd.path is None or len(cmd.path) < 2:
        return
    path = np.asarray(cmd.path, float)
    turn = route_turn_ahead(path, CORNER_WITHIN_M, CORNER_SEG_M)
    err = _wrap(cmd.yaw - yaw)
    sharp = abs(turn) > np.deg2rad(CORNER_TURN_DEG)
    turning = abs(err) >= np.deg2rad(CORNER_TURNING_DEG)
    sticky = getattr(planner, "_corner_sign", 0.0)
    route_ends = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum()) < CORNER_WITHIN_M
    if not (sharp or turning or (sticky and route_ends)):
        planner._corner_sign = 0.0
        return
    # turn direction: the route's; else the corner we are still in (the nose may be ahead of the
    # planner's heading because of the view rule); else the nose's
    sign = float(np.sign(turn)) if sharp else sticky if sticky else float(np.sign(err))
    planner._corner_sign = sign
    occ, reason = planner.belief.occ, "corner"
    if CORNER_VIEW and (sharp or (sticky and route_ends)):     # never while merely turning: bounded
        h = view_heading(occ, planner.spec, xy, float(cmd.yaw), sign)
        if abs(_wrap(h - cmd.yaw)) > 1e-6:
            cmd.yaw, reason = h, "corner-view"
    near = min(view_distance(occ, planner.spec, xy, yaw), view_distance(occ, planner.spec, xy, cmd.yaw))
    if near < VIEW_ENOUGH_M:
        reason += "-slow"
    if reason != "corner":
        cmd.reason = reason
'''

VARIANTS = {
    "slow": dict(view=False, rule="within 1.2 m of a >45 deg route turn (or while the nose is >20 deg off the "
                                   "route), if the camera faces the known map closer than 1.5 m (now or at the "
                                   "commanded heading): half-speed carrot (floor 0.12 m). Heading unchanged."),
    "view": dict(view=True, rule="in the same corner mode, first turn the commanded heading up to 40 deg further "
                                  "into the turn (10 deg steps) to the first heading whose forward view on the "
                                  "known map reaches 1.5 m (else the farthest-seeing one); then slow as 'slow' "
                                  "does if the view is still < 1.5 m."),
}


def _rep(s, a, b, label, src_name="upload_to_flyx.py"):
    assert s.count(a) == 1, f"{label}: expected exactly one match in {src_name}"
    return s.replace(a, b)


def build_variant(name):
    v = VARIANTS[name]
    s = SRC.read_text(encoding="utf-8")
    out = JC / f"upload_to_flyx_cornerfix_{name}.py"
    head = (f'"""CORNERFIX VARIANT {name!r} of Jolie\'s FlyX-proven upload_to_flyx.py (first-corner VIO divergence).\n\n'
            f"GENERATED by jolie_course/cornerfix/build_cornerfix.py from upload_to_flyx.py -- do not edit by hand.\n"
            f"Rule: {v['rule']}\n"
            f"Everything else is upload_to_flyx.py unchanged (yaw cap 0.25 rad/s, turn-in-place at 60 deg, turn\n"
            f"slow-down). Plans are journalled with reason corner-slow / corner-view / corner-view-slow. See jolie_course/cornerfix/README.md.\n\n"
            f"--- upload_to_flyx.py documentation ---\n")
    s = _rep(s, '"""UPLOAD THIS FILE TO FLYX:', head + 'UPLOAD THIS FILE TO FLYX:', "docstring")
    # planner child: helpers after TURN_IN_PLACE_DEG, rule before the carrot trims the route
    s = _rep(s, "TURN_IN_PLACE_DEG = 60.0\n", "TURN_IN_PLACE_DEG = 60.0\n" +
             CORNER_HELPERS.format(name=name, view=v["view"]), "TURN_IN_PLACE_DEG")
    s = _rep(s, '''        if cmd.mode == "move" and cmd.path is not None and len(cmd.path) >= 2:
            cmd.path = safe_carrot_path(planner.belief.cost, spec, np.asarray(xy, float), cmd.path, carrot_m)''',
             '''        if cmd.mode == "move" and cmd.path is not None and len(cmd.path) >= 2:
            corner_rule(planner, cmd, np.asarray(xy, float), float(yaw))     # cornerfix
            cmd.path = safe_carrot_path(planner.belief.cost, spec, np.asarray(xy, float), cmd.path, carrot_m)''',
             "planner_process carrot")
    # mission: half-speed carrot on "corner" commands
    s = _rep(s, "    slow_turn_deg: float = 60.0            # carrot shrinks to 30 % as the heading error reaches this\n",
             "    slow_turn_deg: float = 60.0            # carrot shrinks to 30 % as the heading error reaches this\n"
             "    corner_carrot_scale: float = .5        # cornerfix: carrot x this on 'corner...-slow' commands\n"
             "    corner_carrot_min_m: float = .12       # cornerfix: but never shorter than this\n",
             "MissionConfig.slow_turn_deg")
    s = _rep(s, "        return c.carrot_m * float(np.clip(1.0 - off / np.deg2rad(c.slow_turn_deg), .3, 1.0))\n",
             "        carrot = c.carrot_m * float(np.clip(1.0 - off / np.deg2rad(c.slow_turn_deg), .3, 1.0))\n"
             "        if str(cmd.reason).startswith(\"corner\") and str(cmd.reason).endswith(\"-slow\"):   # cornerfix\n"
             "            carrot = max(c.corner_carrot_min_m, c.corner_carrot_scale * carrot)\n"
             "        return carrot\n", "_carrot_for")
    compile(s, str(out), "exec")
    out.write_text(s, encoding="utf-8", newline="\n")
    print(f"wrote {out} ({len(s) / 1024:.0f} KB)")
    return out


def build_skybeta(name, variant_path):
    spec = importlib.util.spec_from_file_location("build_skybeta_copy", JC / "build_skybeta_copy.py")
    b = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(b)
    b.SRC = variant_path
    b.OUT = JC / f"upload_to_flyx_cornerfix_{name}_skybeta.py"
    b.DOC = b.DOC.replace(
        '"""Jolie\'s FlyX-proven planner (upload_to_flyx.py) on the Sky Beta contract - the drone\'s interface.',
        f'"""CORNERFIX VARIANT {name!r} (upload_to_flyx_cornerfix_{name}.py) on the Sky Beta contract - the drone\'s interface.\n\n'
        f"Rule: {VARIANTS[name]['rule']}\n"
        f"GENERATED by jolie_course/cornerfix/build_cornerfix.py with build_skybeta_copy.py's build(); below is that\n"
        f"builder's description, which applies to this variant with upload_to_flyx_cornerfix_{name}.py in place of\n"
        f"upload_to_flyx.py.\n")
    b.DOC = b.DOC.replace("python3 jolie_course/upload_to_flyx_skybeta.py",
                          f"python3 jolie_course/upload_to_flyx_cornerfix_{name}_skybeta.py")
    assert b.DOC.startswith('"""CORNERFIX'), "build_skybeta_copy.DOC changed; update build_cornerfix.py"
    b.build()


if __name__ == "__main__":
    for n in sys.argv[1:] or list(VARIANTS):
        build_skybeta(n, build_variant(n))
