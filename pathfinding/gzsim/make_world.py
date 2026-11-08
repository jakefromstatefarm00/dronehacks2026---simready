"""Generate a test world for fly_planner.py from the dronehacks-sim course world.

    python3 make_world.py --sim-root <dronehacks-sim checkout> --output /out/worlds/clutter-00 \
        --clutter --seed 3

Writes <output>/course.sdf and <output>/models/course_x3_depth/ (their X3 model plus a
forward depth camera). Their files are read, never modified. Adds:

* the Sensors system (ogre2) so the depth camera renders;
* a forward depth camera on the X3: 80 deg HFOV (OAK-D S2 mono), 320x200, 10 Hz, 0.2-6 m;
* with --clutter: 1-3 unmapped vertical cylinders near the nominal route, drawn by the same
  generator as the offline benchmark (pathplan.variants, kind "clutter") and kept only if an
  X3-sized route still exists.  Their positions go to <output>/world.json.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

DEPTH_SENSOR = """<sensor name="depth" type="depth_camera">
  <pose>0.25 0 0 0 0 0</pose>
  <always_on>1</always_on>
  <update_rate>10</update_rate>
  <topic>/course_quad/depth</topic>
  <camera>
    <camera_info_topic>/course_quad/depth/camera_info</camera_info_topic>
    <horizontal_fov>1.3962634</horizontal_fov>
    <image><width>320</width><height>200</height><format>R_FLOAT32</format></image>
    <clip><near>0.2</near><far>6.0</far></clip>
  </camera>
</sensor>"""

CYLINDER = """<model name="{name}">
  <static>true</static>
  <pose>{x} {y} {zc} 0 0 0</pose>
  <link name="link">
    <collision name="collision"><geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry></collision>
    <visual name="visual"><geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>
      <material><ambient>0.15 0.35 0.8 1</ambient><diffuse>0.15 0.35 0.8 1</diffuse></material></visual>
  </link>
</model>"""


def clutter_positions(seed, radius, margin):
    """Benchmark clutter generator, kept only if an X3-sized route through all checkpoints exists."""
    from pathplan.harness import START_XY, load_published, static_route
    from pathplan.variants import perturb
    from pathplan.world import Envelope, GridSpec, checkpoints_from_route, cost_grid, occupancy
    course, route = load_published()
    spec, env = GridSpec(resolution=.025), Envelope(radius=radius, tracking_margin=margin)
    slab = env.half_height + env.tracking_margin
    cps = checkpoints_from_route(route, course)
    cps[-1].half_width = .762 - radius - margin
    prior_cost = cost_grid(occupancy(course, spec, altitude=.9, half_height=slab), spec, env)
    # the launch mark is inside the X3's keep-out in the narrow first corridor (the planner steps
    # out at takeoff); route from the nearest planable cell instead
    free = np.argwhere(np.isfinite(prior_cost))
    start = spec.to_xy(free[np.argmin(np.linalg.norm(spec.to_xy(free) - START_XY, axis=1))])
    nominal = static_route(prior_cost, spec, env, cps, start)
    if nominal is None:
        raise SystemExit('no X3-sized route on the published course')
    keep = [(START_XY, .8), (np.asarray(route.centre[route.names.index('pad_hover')][:2]), .9)]
    keep += [(c.centre, .5) for c in cps]
    rng = np.random.default_rng(seed)
    for attempt in range(50):
        variant, info = perturb(course, 'clutter', rng, route_xy=nominal, keep_clear=keep)
        truth = cost_grid(occupancy(variant, spec, altitude=.9, half_height=slab), spec, env)
        if static_route(truth, spec, env, cps, start) is not None:
            n = len(variant.cyl_xy) - len(course.cyl_xy)
            return [dict(x=float(x), y=float(y), r=float(r)) for (x, y), r in
                    zip(variant.cyl_xy[-n:], variant.cyl_r[-n:])], attempt
    raise SystemExit('no feasible clutter layout found')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--sim-root', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--clutter', action='store_true')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--radius', type=float, default=.36)
    ap.add_argument('--tracking-margin', type=float, default=.05)
    a = ap.parse_args()

    out = a.output
    models = out / 'models'
    if out.exists():
        shutil.rmtree(out)
    shutil.copytree(a.sim_root / 'models' / 'course_x3', models / 'course_x3_depth')
    model_sdf = models / 'course_x3_depth' / 'model.sdf'
    tree = ET.parse(model_sdf)
    link = next(l for l in tree.getroot().iter('link') if l.get('name') == 'base_link')
    link.append(ET.fromstring(DEPTH_SENSOR))
    tree.write(model_sdf, encoding='utf-8', xml_declaration=True)
    cfg = models / 'course_x3_depth' / 'model.config'
    cfg.write_text(cfg.read_text().replace('<name>', '<name>depth ', 1))

    world = ET.parse(a.sim_root / 'worlds' / 'course.sdf')
    w = world.getroot().find('world')
    for inc in w.iter('include'):
        uri = inc.find('uri')
        if uri is not None and uri.text.strip() == 'model://course_x3':
            uri.text = 'model://course_x3_depth'
    if not any(p.get('filename') == 'gz-sim-sensors-system' for p in w.findall('plugin')):
        plugin = ET.SubElement(w, 'plugin', filename='gz-sim-sensors-system', name='gz::sim::systems::Sensors')
        ET.SubElement(plugin, 'render_engine').text = 'ogre2'
    info = {'sensors': 'X3 + forward depth camera 320x200@10Hz, 80 deg', 'clutter': []}
    if a.clutter:
        cyl, attempts = clutter_positions(a.seed, a.radius, a.tracking_margin)
        for i, c in enumerate(cyl):
            w.append(ET.fromstring(CYLINDER.format(name=f'clutter_{i}', x=c['x'], y=c['y'], zc=.8, r=c['r'], h=1.6)))
        info.update(clutter=cyl, seed=a.seed, draws_rejected=attempts)
    world.write(out / 'course.sdf', encoding='utf-8', xml_declaration=True)
    (out / 'world.json').write_text(json.dumps(info, indent=2))
    print(json.dumps(info))


if __name__ == '__main__':
    main()
