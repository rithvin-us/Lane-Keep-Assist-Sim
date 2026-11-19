"""
build_lighting_track.py - Blender: lighting test track (JSON from MATLAB) -> USD scene for Unreal.

    blender -b --factory-startup --python tools/blender/build_lighting_track.py -- \
        data/scenes/LightingTrack/track.json data/scenes/LightingTrack/LightingTrack.usd

The JSON comes from data/scenarios/lka_lighting_track.m (same geometry the MATLAB bench
scores against). Built like the RoadRunner export the bench already loads (LKA.usd):
Z up, metres, one road mesh with one GeomSubset per material, UsdPreviewSurface materials.

  asphalt   textured strip, lane + shoulders (Asphalt1_Diff.png from data/scenes/LKA)
  lines     white 0.12 m strips on both lane edges, raised 10 mm
  patches   lighting patches painted on the asphalt (5 mm) with the lines inside them
            re-coloured to match:  dark (tree / bar shadow), bright (sun), glare (washed out)
  ground    grass plane under everything

Also renders preview_top.png (whole track) and preview_driver.png (camera view at a patch)
next to the USD, with Blender's Workbench engine, so the scene can be checked without Unreal.
"""
import json
import math
import sys
from pathlib import Path

import bpy

REPO = Path(__file__).resolve().parents[2]
TEX = REPO / "data" / "scenes" / "LKA" / "Asphalt1_Diff.png"
T = {}                                            # track JSON, set by load()


def load(json_file):
    T.clear()
    T.update(json.loads(Path(json_file).read_text()))
    return T

# grey levels (linear base colour) of asphalt / paint per lighting condition
LOOK = {
    "normal": (None, 0.80),        # asphalt = texture
    "dark":   (0.018, 0.10),       # deep shadow: paint barely brighter than sunlit asphalt
    "bright": (0.40, 0.95),        # strong sun: asphalt close to paint
    "glare":  (0.72, 1.00),        # washed out: almost no contrast left
}
Z = {"ground": -0.03, "asphalt": 0.0, "patch": 0.005, "line": 0.010}


def reset():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    sc = bpy.context.scene
    sc.unit_settings.system = "METRIC"
    sc.unit_settings.scale_length = 1.0


def material(name, grey=None, rgb=None, image=None, rough=0.9):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    bsdf = m.node_tree.nodes["Principled BSDF"]
    bsdf.inputs["Roughness"].default_value = rough
    if image is not None:
        tex = m.node_tree.nodes.new("ShaderNodeTexImage")
        tex.image = bpy.data.images.load(str(image))
        m.node_tree.links.new(tex.outputs["Color"], bsdf.inputs["Base Color"])
    else:
        c = rgb if rgb is not None else (grey, grey, grey)
        bsdf.inputs["Base Color"].default_value = (*c, 1.0)
        m.diffuse_color = (*c, 1.0)
    return m


def strip_mesh(name, quads, mat_names, mats, z):
    """quads: list of ((p0, p1, p2, p3), material name, uv quad or None)."""
    verts, faces, fmat, uvs = [], [], [], []
    for (q, mname, uv) in quads:
        i = len(verts)
        verts += [(p[0], p[1], z) for p in q]
        faces.append((i, i + 1, i + 2, i + 3))
        fmat.append(mat_names.index(mname))
        uvs.append(uv or [(0, 0), (1, 0), (1, 1), (0, 1)])
    me = bpy.data.meshes.new(name)
    me.from_pydata(verts, [], faces)
    me.uv_layers.new(name="st")
    for poly, uvq in zip(me.polygons, uvs):
        poly.material_index = fmat[poly.index]
        for li, uv in zip(poly.loop_indices, uvq):
            me.uv_layers["st"].data[li].uv = uv
    for mn in mat_names:
        me.materials.append(mats[mn])
    me.update()
    ob = bpy.data.objects.new(name, me)
    bpy.context.scene.collection.objects.link(ob)
    return ob


def lateral(i, d):
    """Point d metres left of the lane centre at sample i."""
    x, y, psi = T["x"][i], T["y"][i], T["psi"][i]
    return (x - d * math.sin(psi), y + d * math.cos(psi))


def patch_at(s, d):
    for p in T["patches"]:
        if p["S"][0] <= s <= p["S"][1] and p["D"][0] <= d <= p["D"][1]:
            return p["Type"]
    return "normal"


def build():
    reset()
    mats = {"asphalt": material("Asphalt", image=TEX, rough=0.85),
            "grass": material("Grass", rgb=(0.05, 0.12, 0.03), rough=1.0)}
    for kind, (ga, gl) in LOOK.items():
        if ga is not None:
            mats[f"asphalt_{kind}"] = material(f"Asphalt_{kind}", grey=ga, rough=0.3 if kind == "glare" else 0.85)
        mats[f"line_{kind}"] = material(f"LaneMarking_{kind}", grey=gl, rough=0.6)

    n, half = len(T["s"]), T["laneWidth"] / 2 + T["shoulder"]
    s = T["s"]
    # asphalt: one quad per sample step, UV in metres / 3.5 so the texture keeps its scale
    q = []
    for i in range(n - 1):
        a, b = lateral(i, -half), lateral(i, half)
        c, d = lateral(i + 1, half), lateral(i + 1, -half)
        u0, u1 = s[i] / 3.5, s[i + 1] / 3.5
        q.append(((a, d, c, b), "asphalt", [(u0, 0), (u1, 0), (u1, 2 * half / 3.5), (u0, 2 * half / 3.5)]))
    names = ["asphalt"]
    road = strip_mesh("Asphalt", q, names, mats, Z["asphalt"])

    # patches: strips over the asphalt between their lateral limits
    q, pnames = [], []
    for p in T["patches"]:
        mname = f"asphalt_{p['Type']}"
        pnames.append(mname)
        idx = [i for i in range(n) if p["S"][0] <= s[i] <= p["S"][1]]
        for i in idx[:-1]:
            a, b = lateral(i, p["D"][0]), lateral(i, p["D"][1])
            c, d = lateral(i + 1, p["D"][1]), lateral(i + 1, p["D"][0])
            q.append(((a, d, c, b), mname, None))
    patches = strip_mesh("LightingPatches", q, sorted(set(pnames)), mats, Z["patch"])

    # lines: both lane edges, each 0.25 m step coloured by the patch it lies in
    q, lnames = [], set()
    lw = T["lineWidth"] / 2
    for side in (-1, 1):
        dc = side * T["laneWidth"] / 2
        for i in range(n - 1):
            mname = "line_" + patch_at(0.5 * (s[i] + s[i + 1]), dc)
            lnames.add(mname)
            a, b = lateral(i, dc - lw), lateral(i, dc + lw)
            c, d = lateral(i + 1, dc + lw), lateral(i + 1, dc - lw)
            q.append(((a, d, c, b), mname, None))
    lines = strip_mesh("LaneMarkings", q, sorted(lnames), mats, Z["line"])

    # ground: grass plane covering the track with 30 m margin
    xs, ys = T["x"], T["y"]
    x0, x1, y0, y1 = min(xs) - 30, max(xs) + 30, min(ys) - 30, max(ys) + 30
    ground = strip_mesh("Ground", [(((x0, y0), (x1, y0), (x1, y1), (x0, y1)), "grass", None)], ["grass"], mats, Z["ground"])
    return road, patches, lines, ground


def export(objs, OUT):
    OUT.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.object.select_all(action="DESELECT")
    for o in objs:
        o.select_set(True)
    kw = dict(filepath=str(OUT), selected_objects_only=True, export_materials=True,
              generate_preview_surface=True, export_textures=True, overwrite_textures=True,
              relative_paths=True, export_uvmaps=True, export_normals=True,
              root_prim_path="/" + OUT.stem, evaluation_mode="RENDER")
    bpy.ops.wm.usd_export(**kw)


def render(cam_xyz, look_at, ortho_scale, path, lens=24.0, size=(1280, 720)):  # quick Workbench preview
    sc = bpy.context.scene
    sc.render.engine = "BLENDER_WORKBENCH"
    sc.display.shading.light = "STUDIO"
    sc.display.shading.color_type = "TEXTURE"
    sc.render.resolution_x, sc.render.resolution_y = size
    sc.world = sc.world or bpy.data.worlds.new("World")
    cam_data = bpy.data.cameras.new("Cam")
    if ortho_scale:
        cam_data.type = "ORTHO"
        cam_data.ortho_scale = ortho_scale
    else:
        cam_data.lens = lens
    cam_data.clip_end = 1000
    cam = bpy.data.objects.new("Cam", cam_data)
    sc.collection.objects.link(cam)
    cam.location = cam_xyz
    d = [look_at[k] - cam_xyz[k] for k in range(3)]
    yaw = 0.0 if ortho_scale else math.atan2(d[1], d[0]) - math.pi / 2     # top view: +x to the right
    cam.rotation_euler = (math.atan2(math.hypot(d[0], d[1]), -d[2]), 0, yaw)
    sc.camera = cam
    sc.render.filepath = str(path)
    bpy.ops.render.render(write_still=True)
    bpy.data.objects.remove(cam)


if __name__ == "__main__":
    argv = sys.argv[sys.argv.index("--") + 1:]
    load(argv[0])
    OUT = Path(argv[1]).resolve()
    objs = build()
    export(objs, OUT)
    xs, ys = T["x"], T["y"]
    cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
    span = max(max(xs) - min(xs), (max(ys) - min(ys)) * 16 / 9) + 20
    render((cx, cy, 200), (cx, cy, 0), span, OUT.parent / "preview_top.png", size=(1600, 900))
    # driver view at the dappled shadow after the first bend (camera 1.2 m high, 1.6 m ahead of the axle)
    i = min(range(len(T["s"])), key=lambda k: abs(T["s"][k] - (T["sections"][1]["s1"] - 4)))
    x, y, psi = T["x"][i], T["y"][i], T["psi"][i]
    render((x, y, 1.2), (x + 10 * math.cos(psi), y + 10 * math.sin(psi), 0), None, OUT.parent / "preview_driver.png", lens=18)
    print(f"wrote {OUT} and previews ({len(T['patches'])} lighting patches, track {T['s'][-1]:.0f} m)")
