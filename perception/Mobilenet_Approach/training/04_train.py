"""
04_train.py - train the lane segmentation network (U-Net with a MobileNetV2 encoder).

  python 04_train.py                                   (defaults: 480x288 input, 80 epochs, batch 8)
  python 04_train.py --epochs 120 --name run2
  python 04_train.py --width 320 --height 192          (smaller and faster, thin lines get harder)

Reads   dataset/images, dataset/masks, dataset/train.txt, dataset/val.txt, dataset/classes.json
Writes  runs/<name>/best.pt        weights with the best validation score
        runs/<name>/last.pt        weights after the last epoch
        runs/<name>/log.csv        one line per epoch
        runs/<name>/val_preview/   validation pictures: left = your labels, right = the network's answer
"""
import argparse
import csv
import json
import random
import time
from pathlib import Path

import cv2
import numpy as np
import torch
import torch.nn.functional as F
from torch.utils.data import DataLoader, Dataset

from lane_common import MEAN, STD, build_model, overlay, pick_device, read_classes


# ----------------------------------------------------------------------------- augmentation
def add_shadow(img, rng):
    """Darken 1-3 random polygons, with a bluish tint and a soft edge, like tree or building shade."""
    h, w = img.shape[:2]
    out = img.astype(np.float32)
    for _ in range(rng.randint(1, 3)):
        k = rng.randint(3, 7)
        cx, cy = rng.uniform(0, w), rng.uniform(0.3 * h, h)
        rad = rng.uniform(0.15, 0.6) * w
        ang = np.sort(np.array([rng.uniform(0, 2 * np.pi) for _ in range(k)]))
        pts = np.stack([cx + rad * np.cos(ang) * rng.uniform(0.5, 1.0),
                        cy + rad * np.sin(ang) * rng.uniform(0.5, 1.0)], 1).astype(np.int32)
        m = np.zeros((h, w), np.float32)
        cv2.fillPoly(m, [pts], 1.0)
        m = cv2.GaussianBlur(m, (0, 0), rng.uniform(1.0, 6.0))[..., None]
        s = rng.uniform(0.25, 0.75)                                    # how dark the shade is
        gain = np.array([s * rng.uniform(1.0, 1.25), s, s * rng.uniform(0.8, 1.0)], np.float32)   # B, G, R
        out = out * (1 - m) + out * gain * m
    return np.clip(out, 0, 255).astype(np.uint8)


def augment(img, mask, rng):
    h, w = img.shape[:2]
    if rng.random() < 0.5:                                             # mirror
        img, mask = img[:, ::-1], mask[:, ::-1]
    if rng.random() < 0.7:                                             # zoom, shift, small rotation
        sc, ang = rng.uniform(0.85, 1.25), rng.uniform(-6, 6)
        M = cv2.getRotationMatrix2D((w / 2, h / 2), ang, sc)
        M[:, 2] += (rng.uniform(-0.08, 0.08) * w, rng.uniform(-0.08, 0.08) * h)
        img = cv2.warpAffine(img, M, (w, h), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT_101)
        mask = cv2.warpAffine(mask, M, (w, h), flags=cv2.INTER_NEAREST, borderMode=cv2.BORDER_REFLECT_101)
    img = np.ascontiguousarray(img)
    if rng.random() < 0.6:
        img = add_shadow(img, rng)
    if rng.random() < 0.8:                                             # exposure, contrast, colour cast
        f = img.astype(np.float32) / 255.0
        f = f ** rng.uniform(0.6, 1.6)
        f = (f - 0.5) * rng.uniform(0.75, 1.3) + 0.5 + rng.uniform(-0.08, 0.08)
        f = f * np.array([rng.uniform(0.9, 1.1) for _ in range(3)], np.float32)
        img = np.clip(f * 255, 0, 255).astype(np.uint8)
    if rng.random() < 0.25:
        img = cv2.GaussianBlur(img, (0, 0), rng.uniform(0.5, 1.5))
    if rng.random() < 0.3:                                             # video-compression artefacts
        ok, enc = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, rng.randint(30, 75)])
        if ok:
            img = cv2.imdecode(enc, cv2.IMREAD_COLOR)
    return img, np.ascontiguousarray(mask)


class LaneData(Dataset):
    def __init__(self, root, names, width, height, train):
        self.root, self.names, self.w, self.h, self.train = Path(root), names, width, height, train
        self.rng = random.Random()

    def __len__(self):
        return len(self.names)

    def __getitem__(self, i):
        n = self.names[i]
        img = cv2.imread(str(self.root / "images" / (n + ".jpg")))
        mask = cv2.imread(str(self.root / "masks" / (n + ".png")), cv2.IMREAD_GRAYSCALE)
        img = cv2.resize(img, (self.w, self.h), interpolation=cv2.INTER_AREA)
        mask = cv2.resize(mask, (self.w, self.h), interpolation=cv2.INTER_NEAREST)
        if self.train:
            img, mask = augment(img, mask, self.rng)
        x = (cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0 - np.array(MEAN, np.float32)) / np.array(STD, np.float32)
        return torch.from_numpy(np.ascontiguousarray(x.transpose(2, 0, 1))), torch.from_numpy(mask.astype(np.int64)), n


def seed_worker(worker_id):
    info = torch.utils.data.get_worker_info()
    info.dataset.rng = random.Random(torch.initial_seed() % 2 ** 32)   # different augmentation in every worker/epoch


# ----------------------------------------------------------------------------- loss and score
def dice_loss(logits, target, n_classes, eps=1.0):
    p = logits.softmax(1)
    t = F.one_hot(target, n_classes).permute(0, 3, 1, 2).float()
    inter = (p * t).sum((0, 2, 3))
    den = p.sum((0, 2, 3)) + t.sum((0, 2, 3))
    present = t.sum((0, 2, 3)) > 0                                     # ignore classes missing from this batch
    d = 1 - (2 * inter + eps) / (den + eps)
    return d[present].mean() if present.any() else d.mean() * 0


def class_weights(root, names, n_classes):
    cnt = np.zeros(n_classes, np.float64)
    for n in names:
        m = cv2.imread(str(Path(root) / "masks" / (n + ".png")), cv2.IMREAD_GRAYSCALE)
        cnt += np.bincount(m.ravel(), minlength=n_classes)[:n_classes]
    freq = cnt / cnt.sum()
    w = np.where(freq > 0, np.median(freq[freq > 0]) / np.maximum(freq, 1e-9), 0.0)
    return np.clip(w, 0.25, 8.0) * (freq > 0), freq


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default="dataset")
    ap.add_argument("--name", default="run1")
    ap.add_argument("--width", type=int, default=480)
    ap.add_argument("--height", type=int, default=288)
    ap.add_argument("--epochs", type=int, default=80)
    ap.add_argument("--batch", type=int, default=8)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--encoder", default="mobilenet_v2")
    ap.add_argument("--workers", type=int, default=2)
    ap.add_argument("--no-pretrained", action="store_true", help="do not download ImageNet weights (worse results)")
    ap.add_argument("--resume", help="start from these weights, e.g. runs/run1/best.pt")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    if a.width % 32 or a.height % 32:
        ap.error("--width and --height must be multiples of 32")

    torch.manual_seed(a.seed); random.seed(a.seed); np.random.seed(a.seed)
    root = Path(a.data)
    classes = read_classes(root)
    nc = len(classes)
    train_names = (root / "train.txt").read_text().split()
    val_names = (root / "val.txt").read_text().split()
    dev = pick_device()
    out = Path("runs") / a.name
    (out / "val_preview").mkdir(parents=True, exist_ok=True)
    print(f"device: {dev} | {len(train_names)} train / {len(val_names)} val frames | classes: {classes}")
    if dev == "cpu":
        print("no GPU found: training will be slow. Google Colab with a GPU runtime is the usual way round this.")

    w, freq = class_weights(root, train_names, nc)
    print("pixel share per class:", {c: f"{100 * f:.2f}%" for c, f in zip(classes, freq)})
    print("loss weight per class:", {c: round(float(x), 2) for c, x in zip(classes, w)})

    model = build_model(a.encoder, nc, pretrained=not a.no_pretrained).to(dev)
    if a.resume:
        model.load_state_dict(torch.load(a.resume, map_location="cpu", weights_only=False)["state_dict"])
    cfg = dict(encoder=a.encoder, width=a.width, height=a.height, classes=classes)
    (out / "config.json").write_text(json.dumps(dict(vars(a), classes=classes), indent=2))

    g = torch.Generator(); g.manual_seed(a.seed)
    tl = DataLoader(LaneData(root, train_names, a.width, a.height, True), batch_size=a.batch, shuffle=True,
                    num_workers=a.workers, drop_last=len(train_names) > a.batch, worker_init_fn=seed_worker, generator=g)
    vl = DataLoader(LaneData(root, val_names, a.width, a.height, False), batch_size=a.batch, shuffle=False,
                    num_workers=a.workers)
    opt = torch.optim.AdamW(model.parameters(), lr=a.lr, weight_decay=1e-4)
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, max_lr=a.lr, total_steps=a.epochs * len(tl), pct_start=0.1)
    ce_w = torch.tensor(w, dtype=torch.float32, device=dev)
    use_amp = dev == "cuda"
    scaler = torch.amp.GradScaler("cuda", enabled=use_amp)

    best, log_rows = -1.0, []
    for ep in range(1, a.epochs + 1):
        model.train(); t0 = time.time(); run = 0.0
        for x, y, _ in tl:
            x, y = x.to(dev), y.to(dev)
            with torch.autocast("cuda", enabled=use_amp):
                lg = model(x)
                loss = F.cross_entropy(lg.float(), y, weight=ce_w) + dice_loss(lg.float(), y, nc)
            opt.zero_grad(set_to_none=True)
            scaler.scale(loss).backward()
            scaler.step(opt); scaler.update(); sched.step()
            run += loss.item()

        model.eval()
        inter, union = np.zeros(nc), np.zeros(nc)
        shown = 0
        with torch.no_grad():
            for x, y, names in vl:
                pred = model(x.to(dev)).argmax(1).cpu().numpy()
                yy = y.numpy()
                for c in range(nc):
                    inter[c] += ((pred == c) & (yy == c)).sum()
                    union[c] += ((pred == c) | (yy == c)).sum()
                if (ep % 10 == 0 or ep == a.epochs) and shown < 8:
                    for k in range(len(names)):
                        if shown >= 8:
                            break
                        img = cv2.resize(cv2.imread(str(root / "images" / (names[k] + ".jpg"))), (a.width, a.height))
                        pair = np.hstack([overlay(img, yy[k].astype(np.uint8), classes), overlay(img, pred[k].astype(np.uint8), classes)])
                        cv2.imwrite(str(out / "val_preview" / f"ep{ep:03d}_{names[k]}.jpg"), pair)
                        shown += 1
        iou = np.where(union > 0, inter / np.maximum(union, 1), np.nan)
        miou = float(np.nanmean(iou))
        row = dict(epoch=ep, train_loss=round(run / max(len(tl), 1), 4), mIoU=round(miou, 4),
                   **{f"IoU_{c}": (round(float(v), 4) if v == v else "") for c, v in zip(classes, iou)})
        log_rows.append(row)
        with open(out / "log.csv", "w", newline="") as f:
            wr = csv.DictWriter(f, fieldnames=list(row)); wr.writeheader(); wr.writerows(log_rows)
        ck = dict(state_dict=model.state_dict(), cfg=cfg, epoch=ep, mIoU=miou)
        torch.save(ck, out / "last.pt")
        flag = ""
        if miou > best:
            best = miou; torch.save(ck, out / "best.pt"); flag = "  <- best"
        print(f"epoch {ep:3d}/{a.epochs}  loss {row['train_loss']:.3f}  mIoU {miou:.3f}  "
              + "  ".join(f"{c} {v:.2f}" if v == v else f"{c} n/a" for c, v in zip(classes, iou))
              + f"  ({time.time() - t0:.0f}s){flag}")
    print(f"\nbest validation mIoU {best:.3f}  ->  {out / 'best.pt'}")
    print(f"look at {out / 'val_preview'}: left = your labels, right = the network")


if __name__ == "__main__":
    main()
