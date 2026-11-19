"""
lane_common.py - shared helpers for the lane-segmentation scripts. Keep this file next to the numbered scripts.
"""
import json
from pathlib import Path

import cv2
import numpy as np

# class index -> name. Index 0 is always background.
CLASSES = ["background", "road", "lane"]
# display colours (BGR) used in previews and overlay videos
COLORS_BGR = {"background": (0, 0, 0), "road": (0, 200, 0), "lane": (0, 212, 255)}
MEAN = (0.485, 0.456, 0.406)      # ImageNet statistics (RGB), needed by the pretrained encoder
STD = (0.229, 0.224, 0.225)
IMG_EXT = (".jpg", ".jpeg", ".png", ".bmp")


def content_box_from_video(cap, samples=12, thr=10):
    """Find the picture area inside black letterbox bars. Returns (y0, y1, x0, x1). Rewinds the capture."""
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or 1
    acc = None
    for i in np.linspace(0, max(n - 1, 0), samples).astype(int):
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(i))
        ok, f = cap.read()
        if not ok:
            continue
        g = f.max(axis=2)
        acc = g if acc is None else np.maximum(acc, g)
    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
    if acc is None:
        raise RuntimeError("could not read any frame from the video")
    rows = np.where(acc.mean(1) > thr)[0]
    cols = np.where(acc.mean(0) > thr)[0]
    if len(rows) == 0 or len(cols) == 0:
        return 0, acc.shape[0], 0, acc.shape[1]
    return int(rows[0]), int(rows[-1]) + 1, int(cols[0]), int(cols[-1]) + 1


def prepare_frame(frame, box=None, zed_left=False, max_width=0):
    """Same geometry for training frames and for inference: left eye -> crop letterbox -> optional downscale."""
    if zed_left:                                    # ZED side-by-side image: keep the left camera only
        frame = frame[:, : frame.shape[1] // 2]
    if box is not None:
        y0, y1, x0, x1 = box
        frame = frame[y0:y1, x0:x1]
    if max_width and frame.shape[1] > max_width:
        s = max_width / frame.shape[1]
        frame = cv2.resize(frame, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
    return frame


def to_tensor_image(bgr, width, height):
    """BGR uint8 -> float32 CHW, resized and normalised the way the network expects."""
    rgb = cv2.cvtColor(cv2.resize(bgr, (width, height), interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2RGB)
    x = (rgb.astype(np.float32) / 255.0 - np.array(MEAN, np.float32)) / np.array(STD, np.float32)
    return np.ascontiguousarray(x.transpose(2, 0, 1))


def colorize(mask, class_names):
    out = np.zeros(mask.shape + (3,), np.uint8)
    for i, name in enumerate(class_names):
        out[mask == i] = COLORS_BGR.get(name, (255, 0, 255))
    return out


def overlay(bgr, mask, class_names, alpha=0.45):
    """Blend the class colours over the picture (background stays untouched)."""
    col = colorize(mask, class_names)
    out = bgr.copy()
    m = mask > 0
    out[m] = (alpha * col[m] + (1 - alpha) * bgr[m]).astype(np.uint8)
    return out


def read_classes(data_dir):
    p = Path(data_dir) / "classes.json"
    return json.loads(p.read_text()) if p.exists() else list(CLASSES)


def build_model(encoder, n_classes, pretrained):
    import segmentation_models_pytorch as smp
    return smp.Unet(encoder_name=encoder, encoder_weights="imagenet" if pretrained else None,
                    in_channels=3, classes=n_classes)


def load_model(weights, device):
    """Returns (model in eval mode, cfg dict with encoder/width/height/classes)."""
    import torch
    ck = torch.load(weights, map_location="cpu", weights_only=False)
    cfg = ck["cfg"]
    model = build_model(cfg["encoder"], len(cfg["classes"]), pretrained=False)
    model.load_state_dict(ck["state_dict"])
    return model.to(device).eval(), cfg


def pick_device():
    import torch
    if torch.cuda.is_available():
        return "cuda"
    if getattr(torch.backends, "mps", None) is not None and torch.backends.mps.is_available():
        return "mps"
    return "cpu"
