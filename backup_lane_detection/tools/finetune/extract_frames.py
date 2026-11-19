"""Extract training frames for the team UFLDv2 CNN from track videos.

Saves one frame every --every seconds (and the frames listed in --times) as
1280x720 JPEG, the TuSimple image size the CNN is trained on: the video is
centre-cropped to 16:9 first, so nothing is stretched. Time stamps come from the
decoder, not from the nominal frame rate (WhatsApp videos report a wrong one).

Usage:
    python tools/finetune/extract_frames.py vids/*.mp4 --out college_frames
    python tools/finetune/extract_frames.py vids/*.mp4 --out college_frames --times wrong_frames.csv

--times: CSV with columns video,time_s (the wrong-frame list of docs/team_actions.md
item 7; video = file name with or without extension).

Output: <out>/<video name>/<video name>_f<frame index>.jpg and <out>/frames.csv
(video, frame, time_s, file). Label the images with CVAT or labelme, then run
tools/finetune/labels_to_ufld.py.
"""
import argparse
import csv
import os
import re
import sys

import cv2

W, H = 1280, 720


def safe_name(path):
    stem = os.path.splitext(os.path.basename(path))[0]
    return re.sub(r'[^A-Za-z0-9.-]+', '_', stem).strip('_')


def to_tusimple_size(frame):
    h, w = frame.shape[:2]
    if w * H > h * W:                       # wider than 16:9: cut the sides
        cw = round(h * W / H)
        frame = frame[:, (w - cw) // 2:(w - cw) // 2 + cw]
    elif w * H < h * W:                     # taller: cut top and bottom
        ch = round(w * H / W)
        frame = frame[(h - ch) // 2:(h - ch) // 2 + ch]
    return cv2.resize(frame, (W, H), interpolation=cv2.INTER_AREA)


def read_times(path):
    times = {}
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            key = safe_name(row['video'])
            times.setdefault(key, []).append(float(row['time_s']))
    return times


def extract(video, out_dir, every, extra_times):
    cap = cv2.VideoCapture(video)
    if not cap.isOpened():
        raise RuntimeError(f'cannot open {video}')
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    name = safe_name(video)
    os.makedirs(os.path.join(out_dir, name), exist_ok=True)
    pending = sorted(extra_times)
    rows, next_t, k = [], 0.0, 0
    warned = False
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        t = cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
        if t <= 0 and k > 0:
            t = k / fps                     # decoder gave no time stamp
        if not warned and frame.shape[0] > frame.shape[1]:
            print(f'warning: {video} is portrait; the 16:9 crop keeps only the middle', file=sys.stderr)
            warned = True
        take = t >= next_t - 1e-6
        while pending and pending[0] <= t + 0.5 / fps:
            pending.pop(0)
            take = True
        if take:
            rel = f'{name}/{name}_f{k:06d}.jpg'   # unique base names (CVAT may drop folders)
            cv2.imwrite(os.path.join(out_dir, rel), to_tusimple_size(frame), [cv2.IMWRITE_JPEG_QUALITY, 95])
            rows.append({'video': name, 'frame': k, 'time_s': round(t, 3), 'file': rel})
            while next_t <= t + 1e-6:
                next_t += every
        k += 1
    cap.release()
    return rows


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('videos', nargs='+')
    p.add_argument('--out', required=True)
    p.add_argument('--every', type=float, default=0.5, help='seconds between frames (default 0.5)')
    p.add_argument('--times', help='CSV video,time_s of extra frames to include')
    a = p.parse_args()

    extra = read_times(a.times) if a.times else {}
    all_rows = []
    for v in a.videos:
        rows = extract(v, a.out, a.every, extra.get(safe_name(v), []))
        print(f'{v}: {len(rows)} frames')
        all_rows += rows
    with open(os.path.join(a.out, 'frames.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['video', 'frame', 'time_s', 'file'])
        w.writeheader()
        w.writerows(all_rows)
    print(f'{len(all_rows)} frames in {a.out}')


if __name__ == '__main__':
    main()
