"""
render_drive.py - Blender: drive a camera along the lighting track, render the frames, write ground truth.

    blender -b --factory-startup --python tools/blender/render_drive.py -- \
        data/scenes/LightingTrack/track.json results/blender_drive [--frames N] [--samples 24]

Why this exists: MathWorks' Unreal build cannot load a USD scene at run time on Linux
("Run-time import of Universal Scene Description files is not supported by Unreal Engine
on this operating system"), so on this machine the lighting track is rendered by Blender
itself (Cycles, GPU when available, sun + sky) and the video goes through run_lka_on_video.

Camera = the bench camera (LKA.Vision.Camera, profile 'baja'): 1280 x 720, f = 1109 px
(60 deg horizontal), 1.20 m high, 8 deg nose-down, 1.60 m ahead of the rear axle.
Drive: 15 km/h along the lane centre with a +-0.30 m weave (period 8 s), so the lane
offset the LKA should measure is known for every frame.

Writes <out>/frames/f00001.jpg ..., <out>/ground_truth.csv
(frame, t_s, s_m, vehicle_offset_m [+ left of the lane centre], heading_error_deg,
 left_line_m, right_line_m [line offsets at the rear axle, vehicle frame, + left]).
Encode: ffmpeg -framerate 20 -i <out>/frames/f%05d.jpg -c:v libx264 -pix_fmt yuv420p -crf 18 drive.mp4
"""
import argparse
import math
import sys
from pathlib import Path

import bpy

sys.path.insert(0, str(Path(__file__).resolve().parent))
import build_lighting_track as track  # noqa: E402

FPS, SPEED = 20, 15 / 3.6                     # LKA rate [Hz], [m/s]
WEAVE_A, WEAVE_T = 0.30, 8.0                  # lateral weave amplitude [m], period [s]
CAM_X, CAM_H, PITCH_DEG, F_PX, W, H = 1.60, 1.20, 8.0, 1109.0, 1280, 720


def interp(xs, ys, x):
    """Linear interpolation on the (monotonic) station grid."""
    lo, hi = 0, len(xs) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        lo, hi = (mid, hi) if xs[mid] <= x else (lo, mid)
    a = (x - xs[lo]) / (xs[hi] - xs[lo])
    return ys[lo] + a * (ys[hi] - ys[lo])


def pose(t, s0):
    """Rear-axle pose and the ground truth the detector should report."""
    T = track.T
    s = s0 + SPEED * t
    e = WEAVE_A * math.sin(2 * math.pi * t / WEAVE_T)                 # + left of centre
    de_ds = WEAVE_A * 2 * math.pi / WEAVE_T * math.cos(2 * math.pi * t / WEAVE_T) / SPEED
    x, y = interp(T["s"], T["x"], s), interp(T["s"], T["y"], s)
    psi_r = interp(T["s"], T["psi"], s)
    xr, yr = x - e * math.sin(psi_r), y + e * math.cos(psi_r)
    dpsi = math.atan(de_ds)                                           # vehicle heading - lane heading
    w = T["laneWidth"] / 2
    return dict(s=s, e=e, dpsi=dpsi, xr=xr, yr=yr, psi=psi_r + dpsi,
                left=(w - e) * math.cos(dpsi), right=(-w - e) * math.cos(dpsi))


def setup_render(samples, width=W, height=H):
    sc = bpy.context.scene
    sc.render.engine = "CYCLES"
    sc.cycles.samples = samples
    sc.cycles.use_denoising = True
    sc.render.resolution_x, sc.render.resolution_y = width, height
    sc.render.image_settings.file_format = "JPEG"
    sc.render.image_settings.quality = 92
    sc.view_settings.view_transform = "Standard"         # camera-like tone, no filmic wash-out
    dev = "CPU"
    prefs = bpy.context.preferences.addons["cycles"].preferences
    for kind in ("OPTIX", "CUDA"):
        try:
            prefs.compute_device_type = kind
            prefs.get_devices()
            gpus = [d for d in prefs.devices if d.type == kind]
            if gpus:
                for d in prefs.devices:
                    d.use = d.type == kind
                dev = kind
                break
        except TypeError:
            continue
    sc.cycles.device = "GPU" if dev != "CPU" else "CPU"
    if dev == "OPTIX":
        sc.cycles.denoiser = "OPTIX"
    # sky + sun. Calibrated, not physical: a white line in full sun renders near 1.0, sunlit
    # asphalt ~0.4 (sRGB), so the painted bright / dark / glare patches set the contrast
    world = bpy.data.worlds.new("Sky")
    world.use_nodes = True
    bg = world.node_tree.nodes["Background"]
    bg.inputs["Color"].default_value = (0.55, 0.68, 0.90, 1.0)
    bg.inputs["Strength"].default_value = 0.5        # sky fill: sets how dark a shadow gets
    sc.world = world
    sun = bpy.data.objects.new("Sun", bpy.data.lights.new("Sun", "SUN"))
    sun.data.energy = 3.0
    sun.data.angle = math.radians(1.0)
    sun.rotation_euler = SUN_ROT
    sc.collection.objects.link(sun)
    return dev


SUN_ROT = (math.radians(50), 0.0, math.radians(120))   # sun lamp orientation (XYZ euler)


def shadow_casters(height=4.0):
    """Real shadows for the 'dark' patches: an invisible plane above each patch, shifted
    towards the sun so its shadow lands on the patch footprint (asphalt and paint both
    darken, with the soft edge of a 1 deg sun)."""
    T = track.T
    ax, az = SUN_ROT[0], SUN_ROT[2]
    d = (-math.sin(az) * math.sin(ax), math.cos(az) * math.sin(ax), -math.cos(ax))   # light travel
    ox, oy = height * d[0] / d[2], height * d[1] / d[2]       # ground -> caster: back along the ray (d[2] < 0)
    verts, faces = [], []
    for p in T["patches"]:
        if p["Type"] != "dark":
            continue
        idx = [i for i, s in enumerate(T["s"]) if p["S"][0] <= s <= p["S"][1]]
        for i in idx[:-1]:
            k = len(verts)
            for (ii, dd) in ((i, p["D"][0]), (i + 1, p["D"][0]), (i + 1, p["D"][1]), (i, p["D"][1])):
                x, y = track.lateral(ii, dd)
                verts.append((x + ox, y + oy, height))
            faces.append((k, k + 1, k + 2, k + 3))
    me = bpy.data.meshes.new("ShadowCasters")
    me.from_pydata(verts, [], faces)
    ob = bpy.data.objects.new("ShadowCasters", me)
    ob.visible_camera = False
    ob.visible_glossy = False
    ob.visible_transmission = False
    bpy.context.scene.collection.objects.link(ob)


def camera(f_px=F_PX, width=W, cx=None, cy=None, height=H):
    """Pinhole camera matching LKA.Vision.Camera (focal length and principal point in px)."""
    sc = bpy.context.scene
    cd = bpy.data.cameras.new("LkaCamera")
    cd.sensor_fit = "HORIZONTAL"
    cd.sensor_width = 36.0
    cd.lens = 36.0 * f_px / width                                   # e.g. 1109 px at 1280 px
    if cx is not None:                                              # principal point off-centre
        cd.shift_x = (width / 2 - cx) / width
        cd.shift_y = (cy - height / 2) / width
    cd.clip_start, cd.clip_end = 0.1, 500
    cam = bpy.data.objects.new("LkaCamera", cd)
    sc.collection.objects.link(cam)
    sc.camera = cam
    return cam


def build_scene(json_file, samples, width=W, height=H):
    """Track + real shadows + sky/sun + render settings; returns the Cycles device used."""
    track.load(json_file)
    # dark patches get their darkness from real shadows here, not from dark paint
    every = track.T["patches"]
    track.T["patches"] = [p for p in every if p["Type"] != "dark"]
    track.build()
    track.T["patches"] = every
    shadow_casters()
    return setup_render(samples, width, height)


def place(cam, xr, yr, psi, cam_x=CAM_X, cam_h=CAM_H, pitch_deg=PITCH_DEG):
    """Camera pose from the rear-axle pose (x, y, heading)."""
    cam.location = (xr + cam_x * math.cos(psi), yr + cam_x * math.sin(psi), cam_h)
    # Blender camera looks down -Z: tilt up 90 deg minus the pitch, then turn to the heading
    cam.rotation_euler = (math.radians(90 - pitch_deg), 0, psi - math.pi / 2)


def main():
    argv = sys.argv[sys.argv.index("--") + 1:]
    ap = argparse.ArgumentParser()
    ap.add_argument("json")
    ap.add_argument("out")
    ap.add_argument("--frames", type=int, default=0, help="0 = the whole track")
    ap.add_argument("--samples", type=int, default=24)
    ap.add_argument("--start", type=float, default=3.0, help="start station [m]")
    a = ap.parse_args(argv)

    dev = build_scene(a.json, a.samples)
    cam = camera()
    out = Path(a.out)
    (out / "frames").mkdir(parents=True, exist_ok=True)
    n_max = int((track.T["s"][-1] - a.start - 12) / SPEED * FPS)    # stop before the road ends
    n = min(a.frames, n_max) if a.frames else n_max
    print(f"rendering {n} frames on {dev}, {a.samples} samples")
    rows = ["frame,t_s,s_m,vehicle_offset_m,heading_error_deg,left_line_m,right_line_m"]
    for k in range(n):
        t = k / FPS
        p = pose(t, a.start)
        place(cam, p["xr"], p["yr"], p["psi"])
        bpy.context.scene.render.filepath = str(out / "frames" / f"f{k + 1:05d}.jpg")
        bpy.ops.render.render(write_still=True)
        rows.append(f"{k + 1},{t:.3f},{p['s']:.3f},{p['e']:.4f},{math.degrees(p['dpsi']):.3f},"
                    f"{p['left']:.4f},{p['right']:.4f}")
        if (k + 1) % 50 == 0:
            print(f"  {k + 1}/{n} frames", flush=True)
            (out / "ground_truth.csv").write_text("\n".join(rows) + "\n")
    (out / "ground_truth.csv").write_text("\n".join(rows) + "\n")
    print(f"done: {n} frames in {out / 'frames'}")


if __name__ == "__main__":
    main()
