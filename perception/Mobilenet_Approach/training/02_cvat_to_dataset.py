"""
02_cvat_to_dataset.py - convert a CVAT "CVAT for images 1.1" export into a training dataset (2 classes: road, lane).

  python 02_cvat_to_dataset.py --export round1.zip --frames frames/round1
  python 02_cvat_to_dataset.py --export round2.zip --frames round2/frames       (adds to the same dataset)

--export is the .zip you download from CVAT (or the unzipped folder, or annotations.xml itself).
In CVAT use:  Export task dataset -> format "CVAT for images 1.1"  (untick "Save images").

How your shapes become pixels
  road   polygon   -> filled, class 1
  lane   polyline  -> a line along the paint, class 2. A polyline has no width, so it is drawn with a thickness that
                      grows towards the bottom of the picture (paint looks wider when it is close to the camera).
  lane   polygon   -> filled, class 2 (this is what the draft labels from 05_predict.py use)
  everything you leave unlabelled is background, class 0.
  Lane is drawn on top of road.

Writes
  dataset/images/<name>.jpg      the picture
  dataset/masks/<name>.png       one number per pixel: 0 background, 1 road, 2 lane
  dataset/preview/<name>.jpg     picture with the labels drawn on top  <- LOOK AT THESE before training
  dataset/classes.json           the class list the trainer will use

If the lane lines in dataset/preview look too thick or too thin, change --lane-bottom / --lane-top and run again.
"""
import argparse
import json
import shutil
import zipfile
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path

import cv2
import numpy as np

from lane_common import CLASSES, IMG_EXT, overlay

ALIASES = {"background": 0, "road": 1, "lane": 2, "lanes": 2, "lane_paint": 2, "lanepaint": 2,
           "lane paint": 2, "paint": 2}


def load_xml(path):
    p = Path(path)
    if p.is_file() and p.suffix.lower() == ".zip":
        with zipfile.ZipFile(p) as z:
            names = z.namelist()
            xmls = [n for n in names if n.lower().endswith("annotations.xml")] or [n for n in names if n.lower().endswith(".xml")]
            if not xmls:
                hint = " (this looks like a 'Segmentation mask' export)" if any("SegmentationClass" in n for n in names) else ""
                raise SystemExit(f"no annotations.xml inside {p}{hint}.\n"
                                 f"In CVAT export with the format 'CVAT for images 1.1'.")
            return ET.fromstring(z.read(xmls[0]))
    if p.is_dir():
        f = next(iter(p.rglob("annotations.xml")), None)
        if f is None:
            raise SystemExit(f"no annotations.xml found under {p}")
        return ET.parse(f).getroot()
    if p.is_file():
        return ET.parse(p).getroot()
    raise SystemExit(f"{p} does not exist")


def read_points(shape):
    pts = [tuple(float(v) for v in q.split(",")) for q in shape.get("points", "").split(";") if q.strip()]
    return np.array(pts, np.float32).reshape(-1, 2)


def lane_width(y, h, a):
    """Paint width in pixels at image row y. Zero at the horizon, growing linearly towards the bottom (flat ground).
    --lane-bottom and --lane-top are given for a 720-pixel-high picture and scaled to the real height."""
    s = h / 720.0
    yh = a.horizon * h
    k = (a.lane_bottom * s) / max(h - yh, 1.0)
    return max(a.lane_top * s, k * (y - yh))


def draw_lane_line(mask, pts, value, h, a):
    for i in range(len(pts) - 1):
        (x0, y0), (x1, y1) = pts[i], pts[i + 1]
        t = max(1, int(round(lane_width((y0 + y1) / 2, h, a))))
        cv2.line(mask, (int(round(x0)), int(round(y0))), (int(round(x1)), int(round(y1))), value, thickness=t)
    for x, y in pts:                                                # round the joints so thick segments do not leave gaps
        cv2.circle(mask, (int(round(x)), int(round(y))), max(1, int(round(lane_width(y, h, a) / 2))), value, -1)


def rasterise(image_el, w, h, a, skipped):
    """One <image> element -> class-index mask (h, w) uint8."""
    items = []
    for s in image_el:
        if s.tag not in ("polygon", "polyline"):
            if s.tag in ("box", "points", "ellipse", "mask", "cuboid", "skeleton"):
                skipped[f"{s.get('label')} {s.tag}"] += 1
            continue
        name = (s.get("label") or "").strip().lower()
        if name not in ALIASES:
            raise SystemExit(f"CVAT has shapes with the label '{s.get('label')}' (frame {image_el.get('name')}), "
                             f"which this script does not know.\nExpected: road, lane. Delete those shapes in CVAT and export again.")
        items.append((ALIASES[name], s))
    mask = np.zeros((h, w), np.uint8)
    for k, s in sorted(items, key=lambda t: t[0]):                  # road (1) first, lane (2) on top
        if k == 0:
            continue
        pts = read_points(s)
        if s.tag == "polygon" and len(pts) >= 3:
            cv2.fillPoly(mask, [np.round(pts).astype(np.int32)], k)
        elif s.tag == "polyline" and k == 2 and len(pts) >= 2:
            draw_lane_line(mask, pts, k, h, a)
        else:
            skipped[f"{CLASSES[k]} {s.tag} (not usable)"] += 1
    return mask


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--export", required=True, help="the .zip downloaded from CVAT, the unzipped folder, or annotations.xml")
    ap.add_argument("--frames", required=True, help="folder holding the pictures that were uploaded to CVAT")
    ap.add_argument("--out", default="dataset")
    ap.add_argument("--keep-empty", action="store_true", help="keep frames with nothing labelled (default: skip them)")
    ap.add_argument("--lane-bottom", type=float, default=34.0,
                    help="lane paint width in pixels at the bottom of a 720-high picture (default 34)")
    ap.add_argument("--lane-top", type=float, default=3.0, help="thinnest lane width in pixels, far away (default 3)")
    ap.add_argument("--horizon", type=float, default=0.30,
                    help="where the horizon sits, as a fraction of the picture height from the top (default 0.30)")
    a = ap.parse_args()

    root = load_xml(a.export)
    classes = list(CLASSES)
    out = Path(a.out)
    for d in ("images", "masks", "preview"):
        (out / d).mkdir(parents=True, exist_ok=True)
    cj = out / "classes.json"
    if cj.exists() and json.loads(cj.read_text()) != classes:
        raise SystemExit(f"{cj} was made with a different class list. Delete the '{out}' folder and convert all exports again.")
    cj.write_text(json.dumps(classes))

    frames = {p.stem: p for p in Path(a.frames).iterdir() if p.suffix.lower() in IMG_EXT}
    px = np.zeros(len(classes), np.int64)
    present = np.zeros(len(classes), np.int64)
    done, empty, missing = 0, [], []
    skipped = Counter()
    for im in root.findall("image"):
        stem = Path(im.get("name")).stem
        w, h = int(im.get("width")), int(im.get("height"))
        idx = rasterise(im, w, h, a, skipped)
        if not idx.any() and not a.keep_empty:
            empty.append(stem)
            continue
        if stem not in frames:
            missing.append(stem)
            continue
        img = cv2.imread(str(frames[stem]))
        if img is None or img.shape[:2] != (h, w):
            got = "unreadable" if img is None else f"{img.shape[1]}x{img.shape[0]}"
            raise SystemExit(f"{stem}: CVAT says {w}x{h} but the picture in {a.frames} is {got} - wrong --frames folder?")
        if frames[stem].suffix.lower() in (".jpg", ".jpeg"):
            shutil.copy(frames[stem], out / "images" / (stem + ".jpg"))
        else:
            cv2.imwrite(str(out / "images" / (stem + ".jpg")), img, [cv2.IMWRITE_JPEG_QUALITY, 95])
        cv2.imwrite(str(out / "masks" / (stem + ".png")), idx)
        cv2.imwrite(str(out / "preview" / (stem + ".jpg")), overlay(img, idx, classes), [cv2.IMWRITE_JPEG_QUALITY, 85])
        cnt = np.bincount(idx.ravel(), minlength=len(classes))[: len(classes)]
        px += cnt
        present += cnt > 0
        done += 1

    total = len(list((out / "masks").glob("*.png")))
    print(f"converted {done} labelled frames from this export  (dataset now holds {total})")
    if empty:
        print(f"skipped {len(empty)} frames with nothing labelled (first: {empty[:3]})")
    if missing:
        print(f"WARNING: {len(missing)} frames in the export have no picture in {a.frames} (first: {missing[:3]})")
    if skipped:
        print("note: shapes that were not used: " + ", ".join(f"{n} x{c}" for n, c in skipped.items()))
    if done:
        print("\nclass            % of pixels   frames containing it")
        for i, n in enumerate(classes):
            print(f"  {n:14s} {100 * px[i] / px.sum():9.2f}   {present[i]:5d} / {done}")
        if present[2] < 20:
            print("\nWARNING: lane appears in fewer than 20 frames. Label more frames that show lane lines.")
        if present[1] < done:
            print(f"\nnote: {done - present[1]} frames have no 'road' pixels - check their previews.")
    print(f"\nNow open {out / 'preview'} and check every picture before training.")


if __name__ == "__main__":
    main()
