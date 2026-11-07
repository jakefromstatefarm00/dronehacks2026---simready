"""Textured variant of the course USD (ESTIMATED appearance, not the venue, not FlyX).

    isaac_run sim/organizer/isaac/texture_course.py <course.usd> <out.usd> <course-textures-dir> <arena-textures-dir>

Why: the published OBJ/MTL renders every wall flat grey; stereo cuVSLAM loses track as
soon as the view is all wall (Isaac run course-pathplan1). This applies the same
procedural "estimated concrete/plaster" textures our Gazebo arena used
(artifacts/dronehacks_course_surface_*seed2026-v1) as world-projected OmniPBR materials
(the OBJ has no UVs), adds the arena's exterior walls 1.45 m outside the nets (3.5 m tall)
and its floor, and two lights. Geometry and colliders are the published course, unchanged.
"""
import sys
from pathlib import Path

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt  # noqa: E402

src, dst, piece_dir, arena_dir = sys.argv[1], Path(sys.argv[2]), Path(sys.argv[3]), Path(sys.argv[4])
stage = Usd.Stage.CreateNew(str(dst))
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
root = UsdGeom.Xform.Define(stage, '/Course')
stage.SetDefaultPrim(root.GetPrim())
root.GetPrim().GetReferences().AddReference(src)


def omnipbr(name, texture, scale):
    mat = UsdShade.Material.Define(stage, f'/Course/Looks/{name}')
    sh = UsdShade.Shader.Define(stage, f'/Course/Looks/{name}/Shader')
    sh.SetSourceAsset(Sdf.AssetPath('OmniPBR.mdl'), 'mdl')
    sh.SetSourceAssetSubIdentifier('OmniPBR', 'mdl')
    sh.CreateInput('diffuse_texture', Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(str(texture)))
    sh.CreateInput('project_uvw', Sdf.ValueTypeNames.Bool).Set(True)
    sh.CreateInput('world_or_object', Sdf.ValueTypeNames.Bool).Set(True)
    sh.CreateInput('texture_scale', Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(scale, scale))
    sh.CreateInput('reflection_roughness_constant', Sdf.ValueTypeNames.Float).Set(0.8)
    out = sh.CreateOutput('out', Sdf.ValueTypeNames.Token)
    for k in ('surface', 'volume', 'displacement'):
        getattr(mat, f'Create{k.capitalize()}Output')('mdl').ConnectToSource(out)
    return mat


pieces = sorted(piece_dir.glob('*.png'))
piece_mats = [omnipbr(f'piece_{i:02d}', p, 1.0) for i, p in enumerate(pieces)]
n = 0
for prim in stage.Traverse():
    if prim.IsA(UsdGeom.Mesh) and not str(prim.GetPath()).startswith('/Course/Arena'):
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(piece_mats[n % len(piece_mats)],
                                                      UsdShade.Tokens.strongerThanDescendants)
        n += 1


def quad(path, corners, mat):
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*c) for c in corners]))
    m.CreateFaceVertexCountsAttr([4])
    m.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    UsdPhysics.CollisionAPI.Apply(m.GetPrim())
    UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)


half, off, h = 3.04927, 1.45, 3.5
e = half+off
walls = {'north': [(-e, e, 0), (e, e, 0), (e, e, h), (-e, e, h)],
         'south': [(e, -e, 0), (-e, -e, 0), (-e, -e, h), (e, -e, h)],
         'east': [(e, e, 0), (e, -e, 0), (e, -e, h), (e, e, h)],
         'west': [(-e, -e, 0), (-e, e, 0), (-e, e, h), (-e, -e, h)]}
for side, corners in walls.items():
    quad(f'/Course/Arena/{side}', corners, omnipbr(f'exterior_{side}', arena_dir/f'estimated_exterior_{side}.png', 0.25))
quad('/Course/Arena/floor', [(-e, -e, -0.007), (e, -e, -0.007), (e, e, -0.007), (-e, e, -0.007)],
     omnipbr('exterior_floor', arena_dir/'estimated_exterior_floor.png', 0.25))
sun = UsdLux.DistantLight.Define(stage, '/Course/Lights/key')
sun.CreateIntensityAttr(2500.0)
UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(35, 20, 30))
UsdLux.DomeLight.Define(stage, '/Course/Lights/fill').CreateIntensityAttr(800.0)
stage.GetRootLayer().Save()
print(f'[texture_course] {dst}: {n} course meshes textured with {len(pieces)} estimated textures; arena walls+floor+lights added', flush=True)
app.close()
