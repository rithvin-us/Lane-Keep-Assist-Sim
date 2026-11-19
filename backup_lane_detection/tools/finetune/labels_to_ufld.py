"""Turn labelled college-track frames into UFLDv2 (TuSimple config) training data.

Why not the UFLDv2 converter (scripts/convert_tusimple.py): it decides which of the
4 lane slots a line belongs to from the sign of its overall image slope (a line
whose top is right of its bottom = left of the car). In a sharp left bend of the
snake track the ego-left line's top is left of its bottom, so it is taken for a
right-hand line: it is trained as ego-right, and the real ego-right as right-right.
Here every polyline carries its slot as its label, and the converter writes the
three files the UFLDv2 Tusimple loader reads directly.

Labels (CVAT "CVAT for images 1.1" XML, or labelme JSON files): one polyline per
painted line, label LL, L, R or RR (left-left, ego-left, ego-right, right-right,
the 4 slots the CNN outputs; ego = the lane the vehicle drives in). A line that
bends back up the image is used up to the point where it turns (the near part,
which the row anchors can represent).

Usage (the frames come from tools/finetune/extract_frames.py, 1280x720):
    python tools/finetune/labels_to_ufld.py --frames college_frames \\
        --labels annotations.xml --root /content/tusimple

--root is the UFLDv2 data_root (the TuSimple folder with train_gt.txt and
tusimple_anno_cache.json from convert_tusimple.py). The script
  * copies the labelled frames to <root>/college/ with a segmentation PNG beside each,
  * keeps the TuSimple originals as train_gt_tusimple.txt / tusimple_anno_cache_tusimple.json
    (made on the first run; later runs start from them, so a re-run does not add twice),
  * writes train_gt.txt and tusimple_anno_cache.json = TuSimple + college training
    frames (repeated --repeat times so they are about a quarter of each epoch),
  * holds back --val of the college frames, in blocks of --block consecutive frames
    (neighbouring frames are near copies), and with --eval college (default) makes
    them the test set (test.txt, test_label.json; TuSimple's kept as *_tusimple.*),
    so train.py keeps the checkpoint that is best on the college track.
    --eval tusimple puts the TuSimple test set back (to check highways afterwards).
Without TuSimple files in --root it writes a college-only data root.

--preview DIR draws every converted label (slot colours, sampled anchor points) for
a visual check before training.
"""
import argparse
import glob
import json
import os
import random
import re
import shutil
import sys
import xml.etree.ElementTree as ET

import cv2
import numpy as np

W, H = 1280, 720
ROWS = np.arange(160, 711, 10)          # TuSimple h_samples = UFLDv2 row anchors (56)
SLOTS = {'ll': 0, 'l': 1, 'r': 2, 'rr': 3}
ALIASES = {
    '1': 'll', 'left_left': 'll', 'leftleft': 'll', 'left2': 'll',
    '2': 'l', 'ego_left': 'l', 'egoleft': 'l', 'left': 'l',
    '3': 'r', 'ego_right': 'r', 'egoright': 'r', 'right': 'r',
    '4': 'rr', 'right_right': 'rr', 'rightright': 'rr', 'right2': 'rr',
}
COLOURS = [(255, 0, 255), (0, 0, 255), (0, 255, 0), (255, 255, 0)]   # BGR per slot
NO_POINT = -99999.0                     # "no lane at this row" in the UFLDv2 cache


def slot_of(label):
    key = re.sub(r'[\s-]+', '_', label.strip().lower())
    key = ALIASES.get(key, key)
    if key not in SLOTS:
        raise ValueError(f'unknown lane label "{label}" (use LL, L, R, RR)')
    return SLOTS[key]


# ------------------------------------------------------------------ reading labels
def read_cvat(path):
    """CVAT for images 1.1: {image name: [(label, Nx2 points)]}."""
    out = {}
    for img in ET.parse(path).getroot().iter('image'):
        lines = []
        for pl in img.iter('polyline'):
            pts = [tuple(map(float, p.split(','))) for p in pl.get('points').split(';')]
            lines.append((pl.get('label'), np.array(pts)))
        out[img.get('name')] = lines
    return out


def read_labelme(folder):
    """labelme JSON files: {image path: [(label, Nx2 points)]}."""
    out = {}
    for path in glob.glob(os.path.join(folder, '**', '*.json'), recursive=True):
        with open(path) as f:
            d = json.load(f)
        if 'shapes' not in d:
            continue
        lines = [(s['label'], np.array(s['points'], float)) for s in d['shapes']
                 if s.get('shape_type', 'linestrip') in ('linestrip', 'line', 'polyline')]
        out[os.path.normpath(os.path.join(os.path.dirname(path), d['imagePath']))] = lines
    return out


def resolve(name, frames):
    """Labelled image name -> path relative to the frames folder."""
    if os.path.isabs(name) or os.path.exists(name):
        rel = os.path.relpath(os.path.abspath(name), os.path.abspath(frames))
        if not rel.startswith('..'):
            return rel.replace(os.sep, '/')
    if os.path.exists(os.path.join(frames, name)):
        return name.replace(os.sep, '/')
    hits = glob.glob(os.path.join(frames, '**', os.path.basename(name)), recursive=True)
    if len(hits) != 1:
        raise FileNotFoundError(f'{name}: {len(hits)} matches under {frames}')
    return os.path.relpath(hits[0], frames).replace(os.sep, '/')


# ------------------------------------------------------------------ geometry
def sample_rows(pts):
    """x of the polyline at each row anchor (-2 = no point) and whether it was cut."""
    pts = np.asarray(pts, float)
    if pts[0, 1] < pts[-1, 1]:
        pts = pts[::-1]                 # start at the bottom of the image
    dy = np.diff(pts[:, 1])
    cut = bool(np.any(dy > 0))
    if cut:                             # line turns back down the image: keep the near part
        pts = pts[:np.argmax(dy > 0) + 1]
    y, keep = np.unique(pts[::-1, 1], return_index=True)
    x = pts[::-1, 0][keep]
    xs = np.full(len(ROWS), -2.0)
    inside = (ROWS >= y[0]) & (ROWS <= y[-1])
    if len(y) >= 2:
        xs[inside] = np.interp(ROWS[inside], y, x)
    xs[(xs < 0) | (xs >= W)] = -2.0
    return xs, cut


def convert_frame(lines, name, warn):
    """4x56 x positions in slot order (-2 = none)."""
    lanes = np.full((4, len(ROWS)), -2.0)
    for label, pts in lines:
        k = slot_of(label)
        xs, cut = sample_rows(pts)
        if cut:
            warn(f'{name}: {label} turns back up the image, used up to the turn')
        if np.any(lanes[k] >= 0):
            warn(f'{name}: two lines labelled {label}, keeping the longer one')
            if np.sum(xs >= 0) <= np.sum(lanes[k] >= 0):
                continue
        lanes[k] = xs
    for i in range(len(ROWS) - 1, -1, -1):  # left-to-right order on the lowest shared row
        present = lanes[:, i] >= 0
        if present.sum() >= 2:
            xs = lanes[present, i]
            if np.any(np.diff(xs) <= 0):
                warn(f'{name}: slots out of left-to-right order at row {ROWS[i]} (labels swapped?)')
            break
    return lanes


def seg_label(lanes):
    """UFLDv2 segmentation PNG: lane k drawn with value k+1, 16 px wide (convert_tusimple draw())."""
    im = np.zeros((H, W), np.uint8)
    for k in range(4):
        pts = [(int(x), int(y)) for x, y in zip(lanes[k], ROWS) if x >= 0]
        for p, q in zip(pts[:-1], pts[1:]):
            cv2.line(im, p, q, k + 1, thickness=16)
    return im


def cache_entry(lanes):
    pts = np.zeros((4, len(ROWS), 2))
    pts[:, :, 0] = np.where(lanes >= 0, lanes, NO_POINT)
    pts[:, :, 1] = ROWS
    return pts.tolist()


def tusimple_line(name, lanes):
    """TuSimple label line; empty slots left out (LaneEval counts every listed lane)."""
    return json.dumps({'lanes': [[int(round(x)) for x in l] for l in lanes if np.any(l >= 0)],
                       'h_samples': ROWS.tolist(), 'raw_file': name})


# ------------------------------------------------------------------ split
def split(names, val, block, seed):
    """Validation = whole blocks of consecutive frames per video."""
    def key(n):
        d = re.findall(r'\d+', os.path.basename(n))
        return (os.path.dirname(n), int(d[-1]) if d else 0, n)
    blocks = {}
    for n in sorted(names, key=key):
        video = os.path.dirname(n)
        bl = blocks.setdefault(video, [[]])
        if len(bl[-1]) >= block:
            bl.append([])
        bl[-1].append(n)
    flat = [b for v in sorted(blocks) for b in blocks[v]]
    random.Random(seed).shuffle(flat)
    val_set, target = set(), val * len(names)
    for b in flat:
        if len(val_set) >= target:
            break
        val_set.update(b)
    return [n for n in names if n not in val_set], sorted(val_set, key=key)


# ------------------------------------------------------------------ main
def backup(root, name, orig):
    """Original TuSimple file kept as <orig>; returns its path or None."""
    src, bak = os.path.join(root, name), os.path.join(root, orig)
    if not os.path.exists(bak) and os.path.exists(src):
        shutil.copy2(src, bak)
    return bak if os.path.exists(bak) else None


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--frames', required=True, help='folder of extract_frames.py')
    p.add_argument('--labels', required=True, help='CVAT XML file, or folder of labelme JSON')
    p.add_argument('--root', required=True, help='UFLDv2 data_root (TuSimple folder)')
    p.add_argument('--val', type=float, default=0.2, help='share of college frames held back (0.2)')
    p.add_argument('--block', type=int, default=10, help='consecutive frames per split block (10)')
    p.add_argument('--repeat', type=int, default=0, help='times each college frame is listed (0 = auto)')
    p.add_argument('--eval', choices=['college', 'tusimple'], default='college')
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--preview', help='folder for label overlay images')
    a = p.parse_args()

    raw = read_cvat(a.labels) if a.labels.lower().endswith('.xml') else read_labelme(a.labels)
    warnings = []
    frames = {}
    for name, lines in raw.items():
        rel = 'college/' + resolve(name, a.frames)
        src = os.path.join(a.frames, rel[len('college/'):])
        im = cv2.imread(src)
        if im is None or im.shape[:2] != (H, W):
            raise ValueError(f'{src}: must be a {W}x{H} image (use extract_frames.py)')
        frames[rel] = (src, convert_frame(lines, rel, warnings.append))
    if not frames:
        sys.exit('no labelled frames found')

    train, val = split(list(frames), a.val, a.block, a.seed)
    tus_gt = backup(a.root, 'train_gt.txt', 'train_gt_tusimple.txt')
    tus_cache = backup(a.root, 'tusimple_anno_cache.json', 'tusimple_anno_cache_tusimple.json')
    tus_test = backup(a.root, 'test.txt', 'test_tusimple.txt')
    tus_label = backup(a.root, 'test_label.json', 'test_label_tusimple.json')
    gt_lines = open(tus_gt).read().splitlines() if tus_gt else []
    cache = json.load(open(tus_cache)) if tus_cache else {}
    repeat = a.repeat or max(1, round(len(gt_lines) / 3 / max(len(train), 1)))

    for rel, (src, lanes) in frames.items():
        dst = os.path.join(a.root, rel)
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        shutil.copy2(src, dst)
        cv2.imwrite(dst[:-4] + '.png', seg_label(lanes))
        cache[rel] = cache_entry(lanes)
        if a.preview:
            out = os.path.join(a.preview, os.path.basename(rel))
            os.makedirs(a.preview, exist_ok=True)
            im = cv2.imread(src)
            for k in range(4):
                for x, y in zip(lanes[k], ROWS):
                    if x >= 0:
                        cv2.circle(im, (int(x), int(y)), 4, COLOURS[k], -1)
            cv2.putText(im, 'LL L R RR = magenta red green cyan', (20, 40), cv2.FONT_HERSHEY_SIMPLEX,
                        1, (255, 255, 255), 2)
            cv2.imwrite(out, im)

    for rel in train:
        lanes = frames[rel][1]
        bits = ' '.join('1' if np.any(l >= 0) else '0' for l in lanes)
        gt_lines += [f'{rel} {rel[:-4]}.png {bits}'] * repeat
    with open(os.path.join(a.root, 'train_gt.txt'), 'w') as f:
        f.write('\n'.join(gt_lines) + '\n')
    with open(os.path.join(a.root, 'tusimple_anno_cache.json'), 'w') as f:
        json.dump(cache, f)

    val_lab = [rel for rel in val if np.any(frames[rel][1] >= 0)]
    with open(os.path.join(a.root, 'college_val.txt'), 'w') as f:
        f.write(''.join(r + '\n' for r in val_lab))
    with open(os.path.join(a.root, 'college_val_label.json'), 'w') as f:
        f.write(''.join(tusimple_line(r, frames[r][1]) + '\n' for r in val_lab))
    if a.eval == 'college' or not tus_test:
        shutil.copy2(os.path.join(a.root, 'college_val.txt'), os.path.join(a.root, 'test.txt'))
        shutil.copy2(os.path.join(a.root, 'college_val_label.json'), os.path.join(a.root, 'test_label.json'))
    else:
        shutil.copy2(tus_test, os.path.join(a.root, 'test.txt'))
        shutil.copy2(tus_label, os.path.join(a.root, 'test_label.json'))

    with open(os.path.join(a.root, 'college_split.txt'), 'w') as f:
        f.write(''.join(f'train {r}\n' for r in train) + ''.join(f'val {r}\n' for r in val))
    for w in warnings:
        print('warning:', w)
    slots = np.array([[np.any(l >= 0) for l in frames[r][1]] for r in frames])
    print(f'{len(frames)} college frames: {len(train)} train (x{repeat}), {len(val)} val; '
          f'lines per slot LL/L/R/RR = {"/".join(map(str, slots.sum(0)))}; '
          f'both ego lines in {slots[:, 1:3].all(1).mean():.0%}')
    print(f'train_gt.txt: {len(gt_lines)} lines ({len(gt_lines) - len(train) * repeat} TuSimple); '
          f'test set = {"college val" if a.eval == "college" or not tus_test else "TuSimple test"}')


if __name__ == '__main__':
    main()
