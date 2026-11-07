"""Jolie's FlyX-proven modules, read from jolie_course/upload_to_flyx.py, with knobs applied as
source edits.

The proven file is parsed (ast, never executed, never written). Every knob is an exact text
substitution of one constant in one embedded module (or in the file's main()), so the sweep
runs exactly the code a candidate file would fly: build_candidates.py applies the same table.
"""
from __future__ import annotations

import ast
import base64
import hashlib
import sys
import types
import zlib
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
PROVEN = REPO / "jolie_course" / "upload_to_flyx.py"

# knob -> (where, exact text in the proven file, template for the new value, proven value)
# where: an embedded module name, or "main" (the file outside _SOURCES: argparse defaults).
KNOBS = {
    "carrot_m": [("flyx.mission", "    carrot_m: float = .5 ", "    carrot_m: float = {v!r} "),
                 ("main", 'ap.add_argument("--carrot", type=float, default=.5)',
                  'ap.add_argument("--carrot", type=float, default={v!r})')],
    "slow_turn_deg": [("flyx.mission", "    slow_turn_deg: float = 60.0 ", "    slow_turn_deg: float = {v!r} ")],
    "min_carrot_frac": [("flyx.mission", "np.deg2rad(c.slow_turn_deg), .3, 1.0))",
                         "np.deg2rad(c.slow_turn_deg), {v!r}, 1.0))")],
    "yaw_rate_max": [("flyx.mission", "    yaw_rate_max: float = .25 ", "    yaw_rate_max: float = {v!r} ")],
    "pad_centre_m": [("flyx.mission", "    pad_centre_m: float = .08 ", "    pad_centre_m: float = {v!r} ")],
    "turn_in_place_deg": [("pathplan.planner_process", "TURN_IN_PLACE_DEG = 60.0\n", "TURN_IN_PLACE_DEG = {v!r}\n")],
    "wall_snap_m": [("pathplan.planner_process", "WALL_SNAP_M = .25 ", "WALL_SNAP_M = {v!r} ")],
    "hit_confirm": [("pathplan.planner_process", "HIT_CONFIRM = 3 ", "HIT_CONFIRM = {v!r} ")],
    # main() imports MAX_TURN_RATE (depth frames dropped while turning faster) from here
    "max_turn_rate": [("pathplan.planner_process", "MAX_TURN_RATE = .5 ", "MAX_TURN_RATE = {v!r} ")],
    "lookahead_m": [("pathplan.online", "    lookahead_m: float = .7 ", "    lookahead_m: float = {v!r} ")],
    "commit_margin": [("pathplan.online", "    commit_margin: float = .10 ", "    commit_margin: float = {v!r} ")],
    "replan_every_m": [("pathplan.online", "    replan_every_m: float = .5\n", "    replan_every_m: float = {v!r}\n")],
    "turn_step_deg": [("pathplan.online", "    turn_step_deg: float = 15.\n", "    turn_step_deg: float = {v!r}\n")],
    "suspect_penalty": [("pathplan.online", "    suspect_penalty: float = 8.0 ", "    suspect_penalty: float = {v!r} ")],
    "tracking_margin": [("pathplan.world", "    tracking_margin: float = .08 ", "    tracking_margin: float = {v!r} ")],
    "soft_margin": [("pathplan.world", "    soft_margin: float = .15 ", "    soft_margin: float = {v!r} ")],
    "soft_weight": [("pathplan.world", "    soft_weight: float = 4.0 ", "    soft_weight: float = {v!r} ")],
}
BASE = dict(carrot_m=.5, slow_turn_deg=60.0, min_carrot_frac=.3, yaw_rate_max=.25, pad_centre_m=.08,
            turn_in_place_deg=60.0, wall_snap_m=.25, hit_confirm=3, max_turn_rate=.5, lookahead_m=.7,
            commit_margin=.10, replan_every_m=.5, turn_step_deg=15., suspect_penalty=8.0,
            tracking_margin=.08, soft_margin=.15, soft_weight=4.0)
INT_KNOBS = {"hit_confirm"}


def _literal(tree, name):
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(getattr(t, "id", None) == name for t in node.targets):
            return ast.literal_eval(node.value)
    raise KeyError(name)


_CACHE = {}


def proven():
    """(text of upload_to_flyx.py, its _SOURCES, _PACKAGES, map literals)."""
    if not _CACHE:
        text = PROVEN.read_text(encoding="utf-8")
        tree = ast.parse(text)
        _CACHE.update(text=text, sources=_literal(tree, "_SOURCES"), packages=_literal(tree, "_PACKAGES"),
                      prior=_literal(tree, "_PRIOR"), prior_shape=_literal(tree, "_PRIOR_SHAPE"),
                      prior_sha=_literal(tree, "_PRIOR_SHA256"), checkpoints=_literal(tree, "_CHECKPOINTS"),
                      altitude=_literal(tree, "_MAP_ALTITUDE"))
    return _CACHE


def normalise(knobs):
    out = dict(BASE)
    for k, v in (knobs or {}).items():
        if k not in BASE:
            raise KeyError(f"unknown knob {k!r}")
        out[k] = int(round(v)) if k in INT_KNOBS else round(float(v), 4)
    return out


def edits(knobs):
    """[(where, old, new)] for the knobs that differ from the proven file."""
    k = normalise(knobs)
    out = []
    for name, value in k.items():
        if value == BASE[name]:
            continue
        for where, old, tmpl in KNOBS[name]:
            out.append((where, old, tmpl.format(v=value)))
    return out


def _sub(text, old, new, label):
    n = text.count(old)
    if n != 1:
        raise AssertionError(f"{label}: expected exactly one {old!r}, found {n}")
    return text.replace(old, new)


def sources_for(knobs):
    src = dict(proven()["sources"])
    for where, old, new in edits(knobs):
        if where != "main":
            src[where] = _sub(src[where], old, new, where)
    return src


def file_for(knobs):
    """The whole proven file with the knob edits applied (for candidate files)."""
    p = proven()
    text = p["text"]
    current = dict(p["sources"])
    for where, old, new in edits(knobs):
        if where == "main":
            head, sep, tail = text.partition("\n_SOURCES = {")
            end = tail.index("\n}\n")
            text = head + sep + tail[:end] + _sub(tail[end:], old, new, "main")
        else:
            # the module source sits inside r'''...''' in the file, byte for byte
            s0 = current[where]
            assert text.count(s0) == 1, where
            current[where] = _sub(s0, old, new, where)
            text = text.replace(s0, current[where])
    return text


def install(knobs=None):
    """Register the (edited) embedded modules as the packages pathplan / flyx, replacing any
    previous install (the modules import each other lazily, so sys.modules must be current)."""
    src = sources_for(knobs)
    p = proven()
    mods = {}
    for pkg_name, names in p["packages"].items():
        pkg = types.ModuleType(pkg_name)
        pkg.__path__, pkg._embedded = [], True
        sys.modules[pkg_name] = pkg
        for name in names:
            full = pkg_name + "." + name
            mod = types.ModuleType(full)
            mod.__package__ = pkg_name
            mod.__file__ = "<embedded " + pkg_name + "/" + name + ".py>"
            sys.modules[full] = mod
            exec(compile(src[full], mod.__file__, "exec"), mod.__dict__)
            setattr(pkg, name, mod)
            mods[full] = mod
    return mods


def prior_and_checkpoints(mods):
    p = proven()
    n = p["prior_shape"][0] * p["prior_shape"][1]
    bits = np.frombuffer(zlib.decompress(base64.b64decode(p["prior"])), np.uint8)
    prior = np.unpackbits(bits)[:n].astype(bool).reshape(p["prior_shape"])
    assert hashlib.sha256(prior.tobytes()).hexdigest() == p["prior_sha"]
    W = mods["pathplan.world"]
    cps = [W.Checkpoint(c["name"], np.array(c["centre"]), np.array(c["normal"]), c["half_width"], c["points"],
                        kind=c["kind"], other_route=c["other_route"]) for c in p["checkpoints"]]
    return prior, cps


def build_planner_factory(mods):
    """The same as upload_to_flyx._build_planner_embedded, on the installed modules."""
    def build(planner_name="theta", altitude=.9, chicane="window"):
        O, W = mods["pathplan.online"], mods["pathplan.world"]
        prior, cps = prior_and_checkpoints(mods)
        spec, env = W.GridSpec(), W.Envelope()
        return O.OnlinePlanner(prior, spec, env, cps, O.PlannerConfig(planner=planner_name)), spec, env, prior
    return build
