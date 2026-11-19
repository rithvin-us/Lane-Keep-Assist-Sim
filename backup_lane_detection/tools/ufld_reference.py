"""Reference UFLDv2 inference, exactly as in the team's Colab / cfzd demo.py.

    python tools/ufld_reference.py <team_cnn.onnx> <out.json> <image1.png> [<image2.png> ...]

Preprocessing with PIL and torchvision (Resize((400, 800)) -> ToTensor -> Normalize with
the ImageNet mean / std -> keep the bottom 320 rows), network through onnxruntime, decoding
with pred2coords from cfzd/Ultra-Fast-Lane-Detection-v2 (demo.py, MIT licence), restated
with one change: positions are returned as floats (demo.py truncates them with int()) and
each point carries its lane slot and anchor index, so the MATLAB port (lka_detector_ufld,
lka_ufld_decode) can be compared point by point (tests/unit_tests/test_ufld_reference_parity.m).

Output JSON: [{"image": path, "height": h, "width": w,
               "lanes": [{"slot": 1-based lane slot, "row": true|false,
                          "points": [[u, v, anchor], ...]}]}]
u, v are 0-based continuous pixel coordinates (pred2coords convention), anchor 1-based.
"""
import json
import sys

import numpy as np
import onnxruntime as ort
import torch
import torchvision.transforms as transforms
from PIL import Image

# configs/tusimple_res18.py
TRAIN_H, TRAIN_W, CROP_RATIO = 320, 800, 0.8
NUM_GRID_ROW, NUM_CLS_ROW, NUM_GRID_COL, NUM_CLS_COL = 100, 56, 100, 41
ROW_ANCHOR = np.linspace(160, 710, NUM_CLS_ROW) / 720
COL_ANCHOR = np.linspace(0, 1, NUM_CLS_COL)


def pred2coords(pred, row_anchor, col_anchor, local_width=1, original_image_width=1640,
                original_image_height=590):
    _, num_grid_row, num_cls_row, _ = pred["loc_row"].shape
    _, num_grid_col, num_cls_col, _ = pred["loc_col"].shape
    max_indices_row = pred["loc_row"].argmax(1).cpu()
    valid_row = pred["exist_row"].argmax(1).cpu()
    max_indices_col = pred["loc_col"].argmax(1).cpu()
    valid_col = pred["exist_col"].argmax(1).cpu()
    pred["loc_row"] = pred["loc_row"].cpu()
    pred["loc_col"] = pred["loc_col"].cpu()
    coords = []
    row_lane_idx = [1, 2]
    col_lane_idx = [0, 3]
    for i in row_lane_idx:
        tmp = []
        if valid_row[0, :, i].sum() > num_cls_row / 2:
            for k in range(valid_row.shape[1]):
                if valid_row[0, k, i]:
                    all_ind = torch.tensor(list(range(max(0, max_indices_row[0, k, i] - local_width),
                                                      min(num_grid_row - 1, max_indices_row[0, k, i] + local_width) + 1)))
                    out_tmp = (pred["loc_row"][0, all_ind, k, i].softmax(0) * all_ind.float()).sum() + 0.5
                    out_tmp = out_tmp / (num_grid_row - 1) * original_image_width
                    tmp.append([float(out_tmp), float(row_anchor[k] * original_image_height), k + 1])
            coords.append({"slot": i + 1, "row": True, "points": tmp})
    for i in col_lane_idx:
        tmp = []
        if valid_col[0, :, i].sum() > num_cls_col / 4:
            for k in range(valid_col.shape[1]):
                if valid_col[0, k, i]:
                    all_ind = torch.tensor(list(range(max(0, max_indices_col[0, k, i] - local_width),
                                                      min(num_grid_col - 1, max_indices_col[0, k, i] + local_width) + 1)))
                    out_tmp = (pred["loc_col"][0, all_ind, k, i].softmax(0) * all_ind.float()).sum() + 0.5
                    out_tmp = out_tmp / (num_grid_col - 1) * original_image_height
                    tmp.append([float(col_anchor[k] * original_image_width), float(out_tmp), k + 1])
            coords.append({"slot": i + 1, "row": False, "points": tmp})
    return coords


def main(argv):
    if len(argv) < 4:
        print(__doc__)
        return 2
    model, out_json, images = argv[1], argv[2], argv[3:]
    sess = ort.InferenceSession(model, providers=["CPUExecutionProvider"])
    names = [o.name for o in sess.get_outputs()]
    tf = transforms.Compose([
        transforms.Resize((int(TRAIN_H / CROP_RATIO), TRAIN_W)),
        transforms.ToTensor(),
        transforms.Normalize((0.485, 0.456, 0.406), (0.229, 0.224, 0.225))])
    results = []
    for path in images:
        img = Image.open(path).convert("RGB")
        w, h = img.size
        x = tf(img)[:, -TRAIN_H:, :][None].numpy()
        outs = sess.run(None, {sess.get_inputs()[0].name: x})
        pred = {}
        for name, arr in zip(names, outs):
            key = next(k for k in ("loc_row", "loc_col", "exist_row", "exist_col") if k in name)
            pred[key] = torch.from_numpy(arr)
        lanes = pred2coords(pred, ROW_ANCHOR, COL_ANCHOR, original_image_width=w, original_image_height=h)
        results.append({"image": path, "height": h, "width": w, "lanes": lanes})
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump(results, f)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
