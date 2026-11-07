"""Published DroneHacks course OBJ -> USD with static colliders, for Isaac Sim.

    isaac_run sim/organizer/isaac/convert_course.py <course.obj> <out.usd>

The OBJ (Onshape export, metres, z-up) is already in the course frame the planner
uses (+-3.05 m, launch mark (-2.515, 0.914)), so it is referenced at the origin
unchanged. Materials come from the published MTL as-is. Every mesh gets a
triangle-mesh collider so contacts are physical (the Iris can hit walls and nets).
"""
import sys
from pathlib import Path

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import asyncio                                       # noqa: E402
import omni.kit.asset_converter as converter         # noqa: E402
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema  # noqa: E402

src, dst = Path(sys.argv[1]).resolve(), Path(sys.argv[2]).resolve()
raw = dst.with_name(dst.stem + '_raw.usd')


async def convert():
    ctx = converter.AssetConverterContext()
    ctx.ignore_animations = True
    ctx.ignore_camera = True
    ctx.ignore_light = True
    ctx.single_mesh = False
    ctx.use_meter_as_world_unit = True
    task = converter.get_instance().create_converter_task(str(src), str(raw), None, ctx)
    ok = await task.wait_until_finished()
    if not ok:
        raise RuntimeError(f'conversion failed: {task.get_status()} {task.get_error_message()}')


asyncio.get_event_loop().run_until_complete(convert())

stage = Usd.Stage.CreateNew(str(dst))
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
root = UsdGeom.Xform.Define(stage, '/Course')
stage.SetDefaultPrim(root.GetPrim())
root.GetPrim().GetReferences().AddReference(str(raw))
n = 0
for prim in stage.Traverse():
    if prim.IsA(UsdGeom.Mesh):
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr().Set('none')
        n += 1
bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default']).ComputeWorldBound(root.GetPrim()).GetRange()
stage.GetRootLayer().Save()
print(f'[convert_course] {dst}: {n} collider meshes, bounds {bbox.GetMin()} .. {bbox.GetMax()}', flush=True)
app.close()
