"""The organizer's sim/pegasus_skybeta.py with two documented insertions -- nothing else.

Runs the organizer file's own source with:

1. Course (optional): right after it loads its SKYBETA_ENV environment, the USD in
   SKYBETA_COURSE_USD (made by convert_course.py) is referenced at /World/course.
   Place the Iris on the launch mark with the script's own variables:
       SKYBETA_ENV="Default Environment" SKYBETA_SPAWN_X=-2.515 SKYBETA_SPAWN_Y=0.914
       SKYBETA_SPAWN_YAW=1.5707963 SKYBETA_COURSE_USD=<course.usd>
2. Camera view roll (SKYBETA_CAMERA_VIEW_ROLL_DEG, run_all.sh sets 180): Pegasus v5.1.0
   MonocularCamera.start() passes scipy's Rotation.as_quat() -- (x, y, z, w) -- to
   Isaac's set_local_pose(), which reads (w, x, y, z). The organizer's orientation
   [0, 0, 0] therefore becomes a 180 deg turn and the cameras look backwards at the Iris
   (verified on GB10, runs square2/camcheck1); Pegasus's own default [0, 0, 180] is the
   one that comes out forward and upright (camcheck4). Their README names exactly this
   value. SKYBETA_CAMERA_ROLL cannot be used for it: it also rotates the stereo baseline
   offset (swapping left and right) and must match the launch's camera_roll TF. This
   adds the angle to the camera *view* only; positions and the cuVSLAM TFs in
   skybeta_sim.launch.py stay as the organizer wrote them.

Vehicle, PX4 link, ROS 2 backend, intrinsics: the organizer's code, untouched.
"""
import os
import sys
from pathlib import Path

ORGANIZER = Path(os.environ.get(
    'SKYBETA_ORGANIZER_SCRIPT', Path(__file__).resolve().parents[2]/'skybeta/sim/pegasus_skybeta.py'))
source = ORGANIZER.read_text()


def insert(old, new):
    global source
    if source.count(old) != 1:
        sys.exit(f'organizer script changed: cannot find {old.strip()[:60]!r} in {ORGANIZER}')
    source = source.replace(old, new)


env_line = '        self.pg.load_environment(SIMULATION_ENVIRONMENTS[ENVIRONMENT])\n'
insert(env_line, env_line + (
    '        _course = os.environ.get("SKYBETA_COURSE_USD")\n'
    '        if _course:\n'
    '            from omni.usd import get_context as _usd_context\n'
    '            _p = _usd_context().get_stage().DefinePrim("/World/course", "Xform")\n'
    '            _p.GetReferences().AddReference(_course)\n'
    '            carb.log_warn(f"[skybeta] course referenced at /World/course: {_course}")\n'))
insert('            np.rad2deg(CAMERA_ROLL),\n',
       '            np.rad2deg(CAMERA_ROLL) + float(os.environ.get("SKYBETA_CAMERA_VIEW_ROLL_DEG", "0")),\n')
code = compile(source, str(ORGANIZER), 'exec')
exec(code, {'__name__': '__main__', '__file__': str(ORGANIZER)})
