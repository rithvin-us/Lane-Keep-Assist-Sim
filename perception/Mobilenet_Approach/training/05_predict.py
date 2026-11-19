"""
05_predict.py - run the trained network.

A) on a video: writes a side-by-side video (left = camera, right = what the network sees)
     python 05_predict.py --weights runs/run1/best.pt --video videos/clip_A.mp4 --out result_A.mp4
     python 05_predict.py --weights runs/run1/best.pt --video new_clip.mp4 --zed-left

B) on a folder of frames: ranks the frames the network is least sure about and writes draft labels for CVAT
     python 05_predict.py --weights runs/run1/best.pt --frames frames/all --skip-labelled dataset
   writes round2/suggest.csv           frames sorted from least sure to most sure
          round2/frames/               the --top least-sure frames, ready to upload to CVAT
          round2/prelabels.xml         draft polygons for those frames (CVAT format "CVAT for images 1.1")
"""
import argparse
import csv
import shutil
import time
from pathlib import Path
from xml.sax.saxutils import quoteattr

import cv2
import numpy as np
import torch

from lane_common import IMG_EXT, content_box_from_video, load_model, overlay, pick_device, prepare_frame, to_tensor_image


@torch.no_grad()
def infer(model, bgr, cfg, dev):
    """Returns (class mask at the picture's own size, mean confidence 0..1)."""
    x = torch.from_numpy(to_tensor_image(bgr, cfg["width"], cfg["height"]))[None].to(dev)
    prob = model(x).softmax(1)[0]
    conf, pred = prob.max(0)
    mask = cv2.resize(pred.cpu().numpy().astype(np.uint8), (bgr.shape[1], bgr.shape[0]), interpolation=cv2.INTER_NEAREST)
    return mask, float(conf.mean())


def run_video(a, model, cfg, dev):
    cap = cv2.VideoCapture(a.video)
    if not cap.isOpened():
        raise SystemExit(f"cannot open {a.video}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    box = None
    if not a.no_autocrop:
        y0, y1, x0, x1 = content_box_from_video(cap)
        if a.zed_left:
            half = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) // 2
            x0, x1 = min(x0, half), min(x1, half)
        box = (y0, y1, x0, x1)
    writer, n, t_net = None, 0, 0.0
    while True:
        ok, frame = cap.read()
        if not ok or (a.max_frames and n >= a.max_frames):
            break
        f = frame[:, : frame.shape[1] // 2] if a.zed_left else frame
        f = prepare_frame(f, box=box, max_width=a.max_width)
        t0 = time.time()
        mask, conf = infer(model, f, cfg, dev)
        if dev == "cuda":
            torch.cuda.synchronize()
        t_net += time.time() - t0
        view = np.hstack([f, overlay(f, mask, cfg["classes"])])
        cv2.putText(view, f"frame {n}  confidence {conf:.2f}", (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        if view.shape[1] > 1920:
            s = 1920 / view.shape[1]
            view = cv2.resize(view, None, fx=s, fy=s)
        view = view[: view.shape[0] // 2 * 2, : view.shape[1] // 2 * 2]
        if writer is None:
            writer = cv2.VideoWriter(a.out, cv2.VideoWriter_fourcc(*"mp4v"), fps, (view.shape[1], view.shape[0]))
        writer.write(view)
        n += 1
        if n % 100 == 0:
            print(f"  {n} frames done", flush=True)
    cap.release()
    if writer:
        writer.release()
    if n:
        print(f"wrote {a.out}: {n} frames, network {1000 * t_net / n:.1f} ms/frame on {dev} "
              f"(resize + network + upscale; this is NOT the speed on the Jetson)")


def mask_to_polygons(mask, class_id, min_area, eps_px):
    cs, _ = cv2.findContours((mask == class_id).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for c in cs:
        if cv2.contourArea(c) < min_area:
            continue
        p = cv2.approxPolyDP(c, eps_px, True).reshape(-1, 2)
        if len(p) >= 3:
            out.append(p)
    return out


def run_frames(a, model, cfg, dev):
    classes = cfg["classes"]
    done = set()
    if a.skip_labelled:
        done = {p.stem for p in (Path(a.skip_labelled) / "masks").glob("*.png")}
    files = sorted(p for p in Path(a.frames).iterdir() if p.suffix.lower() in IMG_EXT and p.stem not in done)
    if not files:
        raise SystemExit("no frames left to look at")
    out = Path(a.out_dir)
    (out / "frames").mkdir(parents=True, exist_ok=True)
    rows, masks = [], {}
    for i, p in enumerate(files):
        img = cv2.imread(str(p))
        mask, conf = infer(model, img, cfg, dev)
        rows.append((p.name, conf))
        masks[p.name] = (mask, img.shape[1], img.shape[0])
        if (i + 1) % 100 == 0:
            print(f"  {i + 1}/{len(files)} frames", flush=True)
    rows.sort(key=lambda r: r[1])
    with open(out / "suggest.csv", "w", newline="") as f:
        w = csv.writer(f); w.writerow(["frame", "mean_confidence"]); w.writerows((n, f"{c:.4f}") for n, c in rows)

    # pick the least-sure frames, but keep them at least --min-gap candidates apart so they are not near-duplicates
    order = {p.name: i for i, p in enumerate(files)}
    picked = []
    for name, _ in rows:
        if len(picked) >= a.top:
            break
        if all(abs(order[name] - order[q]) >= a.min_gap or name.split("_")[0] != q.split("_")[0] for q in picked):
            picked.append(name)
    picked.sort()
    lines = ['<?xml version="1.0" encoding="utf-8"?>', "<annotations>", "  <version>1.1</version>"]
    for i, name in enumerate(picked):
        shutil.copy(Path(a.frames) / name, out / "frames" / name)
        mask, w_, h_ = masks[name]
        lines.append(f'  <image id="{i}" name={quoteattr(name)} width="{w_}" height="{h_}">')
        for cid in range(1, len(classes)):                      # road first (z 1), paint on top (z 2, 3)
            for poly in mask_to_polygons(mask, cid, a.min_area, a.simplify):
                pts = ";".join(f"{x:.2f},{y:.2f}" for x, y in poly)
                lines.append(f'    <polygon label={quoteattr(classes[cid])} source="auto" occluded="0" points="{pts}" z_order="{cid}">')
                lines.append("    </polygon>")
        lines.append("  </image>")
    lines.append("</annotations>")
    (out / "prelabels.xml").write_text("\n".join(lines) + "\n")
    print(f"looked at {len(files)} frames. Least sure: {rows[0][0]} ({rows[0][1]:.2f}); most sure: {rows[-1][0]} ({rows[-1][1]:.2f})")
    print(f"wrote {out / 'suggest.csv'}, {len(picked)} frames in {out / 'frames'}, draft polygons in {out / 'prelabels.xml'}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--weights", required=True)
    ap.add_argument("--video"); ap.add_argument("--out", default="result.mp4")
    ap.add_argument("--zed-left", action="store_true"); ap.add_argument("--no-autocrop", action="store_true")
    ap.add_argument("--max-width", type=int, default=1280); ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--frames"); ap.add_argument("--out-dir", default="round2")
    ap.add_argument("--skip-labelled", help="dataset folder: frames already labelled there are left out")
    ap.add_argument("--top", type=int, default=60, help="how many least-sure frames to prepare for labelling")
    ap.add_argument("--min-gap", type=int, default=3, help="keep picked frames this many candidates apart")
    ap.add_argument("--min-area", type=float, default=150.0); ap.add_argument("--simplify", type=float, default=2.0)
    a = ap.parse_args()
    if bool(a.video) == bool(a.frames):
        ap.error("give exactly one of --video or --frames")
    dev = pick_device()
    model, cfg = load_model(a.weights, dev)
    print(f"model {cfg['encoder']} {cfg['width']}x{cfg['height']} classes {cfg['classes']} on {dev}")
    (run_video if a.video else run_frames)(a, model, cfg, dev)


if __name__ == "__main__":
    main()
