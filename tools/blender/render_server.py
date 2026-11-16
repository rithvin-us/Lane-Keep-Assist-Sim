"""
render_server.py - Blender as the camera of a closed-loop MATLAB simulation.

    blender -b --factory-startup --python tools/blender/render_server.py -- \
        data/scenes/LightingTrack/track.json --port 5560 --width 640 --height 360 --samples 12

Loads the lighting track once (same scene as render_drive.py: real shadows, sun + sky), then
answers one JSON request per line on 127.0.0.1:<port>:

    {"cmd": "camera", "f": 554.5, "cx": 320, "cy": 180, "x": 1.6, "h": 1.2, "pitch": 8}
        -> {"ok": true, "width": 640, "height": 360, "device": "OPTIX"}
    {"cmd": "render", "x": 12.3, "y": -0.4, "psi": 0.02, "out": "/dev/shm/lka_frame.jpg"}
        -> {"ok": true, "ms": 180.2}            x, y, psi: rear-axle pose in the track frame
    {"cmd": "quit"}

MATLAB side: src/vision/LkaBlenderCamera.m (MATLAB System block in LKA_Blender_TestBench).
The vehicle pose comes from the Simulink plant, so the path is whatever the controller
drives, not a recorded one.
"""
import argparse
import json
import socket
import sys
import time
from pathlib import Path

import bpy

sys.path.insert(0, str(Path(__file__).resolve().parent))
import render_drive as rd  # noqa: E402


def main():
    argv = sys.argv[sys.argv.index("--") + 1:]
    ap = argparse.ArgumentParser()
    ap.add_argument("json")
    ap.add_argument("--port", type=int, default=5560)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=360)
    ap.add_argument("--samples", type=int, default=12)
    a = ap.parse_args(argv)

    dev = rd.build_scene(a.json, a.samples, a.width, a.height)
    sc = bpy.context.scene
    sc.render.image_settings.file_format = "JPEG"
    sc.render.image_settings.quality = 95
    cam = rd.camera(rd.F_PX * a.width / rd.W, a.width, height=a.height)
    mount = dict(cam_x=rd.CAM_X, cam_h=rd.CAM_H, pitch_deg=rd.PITCH_DEG)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", a.port))
    srv.listen(1)
    print(f"render server ready on 127.0.0.1:{a.port} ({a.width}x{a.height}, {dev}, {a.samples} samples)", flush=True)
    while True:
        conn, _ = srv.accept()
        f = conn.makefile("rwb")
        try:
            for line in f:
                req = json.loads(line)
                cmd = req.get("cmd")
                if cmd == "quit":
                    f.write(b'{"ok": true}\n'); f.flush()
                    return
                if cmd == "camera":
                    bpy.data.objects.remove(cam)
                    cam = rd.camera(req["f"], a.width, req.get("cx"), req.get("cy"), a.height)
                    mount = dict(cam_x=req.get("x", rd.CAM_X), cam_h=req.get("h", rd.CAM_H),
                                 pitch_deg=req.get("pitch", rd.PITCH_DEG))
                    rep = {"ok": True, "width": a.width, "height": a.height, "device": dev}
                elif cmd == "render":
                    t0 = time.perf_counter()
                    rd.place(cam, req["x"], req["y"], req["psi"], **mount)
                    sc.render.filepath = req["out"]
                    bpy.ops.render.render(write_still=True)
                    rep = {"ok": True, "ms": 1000 * (time.perf_counter() - t0)}
                else:
                    rep = {"ok": False, "error": f"unknown cmd {cmd!r}"}
                f.write((json.dumps(rep) + "\n").encode())
                f.flush()
        except (ConnectionError, json.JSONDecodeError) as e:
            print("client gone:", e, flush=True)
        finally:
            conn.close()


if __name__ == "__main__":
    main()
