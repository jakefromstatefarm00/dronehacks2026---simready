"""Review 2 findings 4 (clocks) and 7 (vision bridge), and run.sh team mode: the launch/config text
(no ROS here, so the files are read, not launched)."""
import ast
from pathlib import Path
import re

from dronehacks_pathplan.runtime import VISION_TOPIC, foreign_publishers

PKG = Path(__file__).resolve().parents[1]
REPO = PKG.parents[2]


def _launch_args(path, table):
    """{name: default} from a launch file's argument table (a module-level list of tuples)."""
    tree = ast.parse(path.read_text(encoding="utf-8"))
    for node in tree.body:
        if isinstance(node, ast.Assign) and getattr(node.targets[0], "id", "") == table:
            return {ast.literal_eval(t.elts[0]): ast.literal_eval(t.elts[1]) for t in node.value.elts}
    raise AssertionError(f"{table} not in {path}")


# ---------------------------------------------------------------- 7. never two vision bridges
def test_two_vision_publishers_are_foreign():
    one = {VISION_TOPIC: 1, "/skybeta/state": 1, "/skybeta/setpoint/pose": 1}
    assert foreign_publishers(one, "/skybeta/setpoint/pose") == ""
    two = {**one, VISION_TOPIC: 2}
    assert "2 vision bridges" in foreign_publishers(two, "/skybeta/setpoint/pose")


def test_sim_launch_starts_no_vision_bridge_by_default():
    src = (PKG / "launch" / "sim.launch.py").read_text(encoding="utf-8")
    m = re.search(r"DeclareLaunchArgument\('vision_bridge',\s*default_value='(\w+)'", src)
    assert m and m.group(1) == "false"


# ---------------------------------------------------------------- 4. flight_interface on the wall clock
def test_flight_interface_is_on_the_wall_clock_in_the_sim_and_on_the_drone():
    sim = _launch_args(PKG / "launch" / "sim.launch.py", "PASS_THROUGH")
    drone = _launch_args(PKG / "launch" / "pathplan.launch.py", "NODE_ARGS")
    assert sim["use_sim_time"] == "true" and sim["interface_use_sim_time"] == "false"
    assert drone["use_sim_time"] == "false" and drone["interface_use_sim_time"] == "false"
    src = (PKG / "launch" / "pathplan.launch.py").read_text(encoding="utf-8")
    assert "'use_sim_time': LaunchConfiguration('interface_use_sim_time')" in src


def test_run_sh_team_mode():
    src = (REPO / "flyx_skybeta" / "run.sh").read_text(encoding="utf-8")
    team = src[src.index("  team)"):src.index(";;", src.index("  team)"))]
    assert "FI_EXTRA=(-p use_sim_time:=false)" in team             # interface: wall clock
    assert "-p use_sim_time:=\"$SIMT\"" in team                     # our node: sim time (run limit)
    assert "-p journal_dir:=\"$OUT\"" in team
    assert "land_states:=LOW_HOLD,FAULT,TAKEN_OVER" in team
    assert "TEAM_PILOT" not in src and "dronehacks_pathplan.sim_pilot" not in src
    assert "from team_mission.sim.sim_pilot import main" in src
    assert src.index('OUT="$ROOT/flyx_skybeta/runs') < src.index("  team)")   # $OUT set before it is used


def test_jakes_sim_pilot_treats_ready_as_streaming():
    src = (REPO / "team_ws" / "src" / "team_mission" / "team_mission" / "sim" / "sim_pilot.py").read_text(
        encoding="utf-8")
    block = src[src.index("if self.require_mission:"):src.index("if ready and dt > self.wait_s")]
    assert "'READY'" in block and "'STREAM'" in block
