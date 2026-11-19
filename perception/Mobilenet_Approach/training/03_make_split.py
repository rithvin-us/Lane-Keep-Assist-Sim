"""
03_make_split.py - split the dataset into training and validation frames.

  python 03_make_split.py                                  (default: blocks of 8 frames, every 5th block = validation)
  python 03_make_split.py --val-prefix B                   (train on video A, validate on all of video B)

Frames next to each other in a video look almost the same. If neighbours were split at random, the validation
score would look far better than the model really is. So frames are cut into blocks of consecutive frames and
whole blocks go to validation.
"""
import argparse
from collections import defaultdict
from pathlib import Path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default="dataset")
    ap.add_argument("--block", type=int, default=8, help="consecutive frames per block")
    ap.add_argument("--val-every", type=int, default=5, help="every Nth block becomes validation")
    ap.add_argument("--val-prefix", help="put every frame of this video (e.g. B) in validation instead")
    a = ap.parse_args()

    names = sorted(p.stem for p in (Path(a.data) / "masks").glob("*.png"))
    if not names:
        raise SystemExit(f"no masks in {a.data}/masks - run 02_cvat_to_dataset.py first")
    groups = defaultdict(list)
    for n in names:
        groups[n.rsplit("_", 1)[0]].append(n)

    train, val = [], []
    for prefix, items in sorted(groups.items()):
        if a.val_prefix:
            (val if prefix == a.val_prefix else train).extend(items)
            continue
        blocks = [items[i:i + a.block] for i in range(0, len(items), a.block)]
        for bi, b in enumerate(blocks):
            # offset so the validation blocks sit in the middle of the video, not always at its start
            (val if (bi + a.val_every // 2) % a.val_every == 0 else train).extend(b)
    if not train or not val:
        raise SystemExit("split left one side empty - label more frames or change --block / --val-every")

    (Path(a.data) / "train.txt").write_text("\n".join(train) + "\n")
    (Path(a.data) / "val.txt").write_text("\n".join(val) + "\n")
    print(f"{len(names)} frames -> {len(train)} train, {len(val)} validation")
    for prefix, items in sorted(groups.items()):
        nv = sum(1 for n in items if n in set(val))
        print(f"  video {prefix}: {len(items) - nv} train, {nv} validation")


if __name__ == "__main__":
    main()
