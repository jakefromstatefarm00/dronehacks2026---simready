"""The two turn rules of jolie_course/upload_to_flyx.py, the version that completed the whole FlyX
course. They must stay in this package: without them the drone stops and turns in place 21-26 times
per flight (30 deg default) and swings wide at corners."""
from math import radians

import numpy as np

from dronehacks_pathplan.mission import Mission, MissionConfig
from pathplan.online import Command


def test_turn_in_place_only_beyond_60_deg():
    from dronehacks_pathplan.planner_core import TURN_IN_PLACE_DEG, PlannerCore
    assert TURN_IN_PLACE_DEG == 60.0
    core = PlannerCore()
    assert core.planner.cfg.turn_in_place_deg == 60.0
    core.reset()                                          # a new flight keeps the rule
    assert core.planner.cfg.turn_in_place_deg == 60.0


def test_carrot_shrinks_while_the_nose_is_still_turning():
    m = Mission(planner=None, cfg=MissionConfig())
    path = np.array([[0.0, 0.0], [1.0, 0.0]])
    move = lambda off_deg: Command("move", path, radians(off_deg))
    assert m._carrot_for(move(0), 0.0) == .5                              # aligned: full 0.5 m
    assert abs(m._carrot_for(move(30), 0.0) - .25) < 1e-9                 # halfway to 60 deg: half
    assert abs(m._carrot_for(move(60), 0.0) - .15) < 1e-9                 # floor: 30 %
    assert abs(m._carrot_for(move(120), 0.0) - .15) < 1e-9
    assert abs(m._carrot_for(move(-30), 0.0) - .25) < 1e-9                # either side
    assert m._carrot_for(Command("turn", yaw=1.0), 0.0) == .5             # only moves are shortened
    assert m._carrot_for(Command("move", path, None), 0.0) == .5          # no heading command: full


def test_same_rule_as_the_flown_file():
    """Compare with the embedded mission source of jolie_course/upload_to_flyx.py when available."""
    import ast
    from pathlib import Path
    flown = Path(__file__).resolve().parents[4] / "jolie_course" / "upload_to_flyx.py"
    if not flown.exists():
        return
    t = ast.parse(flown.read_text(encoding="utf-8"))
    src = next(ast.literal_eval(n.value) for n in t.body
               if isinstance(n, ast.Assign) and getattr(n.targets[0], "id", "") == "_SOURCES")
    assert "slow_turn_deg: float = 60.0" in src["flyx.mission"]
    assert "TURN_IN_PLACE_DEG = 60.0" in src["pathplan.planner_process"]
