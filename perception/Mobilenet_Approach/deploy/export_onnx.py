"""
export_onnx.py - turn the trained PyTorch weights into the ONNX file MATLAB runs.

    python deploy/export_onnx.py                                  (weights/best.pt -> deploy/mobilenet_unet_288x480.onnx)
    python deploy/export_onnx.py --weights runs/run2/best.pt

Run it from perception/Mobilenet_Approach/ after every retraining, then restart the MATLAB
detector (the imported network is cached next to the ONNX as *_dlnetwork.mat and rebuilt
automatically when the ONNX is newer).

The ONNX takes 1 x 3 x H x W float32 (RGB, /255, ImageNet mean/std, stretched to H x W)
and returns 1 x C x H x W class scores (logits). deploy/mobilenet.json tells MATLAB how to
prepare the image and which channel is the lane line. The script checks that onnxruntime
gives the same answer as PyTorch before it writes the sidecar.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import torch

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "training"))
from lane_common import MEAN, STD, load_model  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--weights", default=str(ROOT / "weights" / "best.pt"))
    ap.add_argument("--opset", type=int, default=17)
    a = ap.parse_args()

    model, cfg = load_model(a.weights, "cpu")
    h, w = cfg["height"], cfg["width"]
    name = f"mobilenet_unet_{h}x{w}.onnx"
    onnx_path = HERE / name
    x = torch.randn(1, 3, h, w)
    kw = dict(input_names=["image"], output_names=["logits"], opset_version=a.opset)
    try:
        torch.onnx.export(model, x, str(onnx_path), dynamo=False, **kw)
    except TypeError:                                   # older torch without the dynamo switch
        torch.onnx.export(model, x, str(onnx_path), **kw)

    import onnxruntime as ort
    sess = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    with torch.inference_mode():
        ref = model(x).numpy()
    err = float(np.abs(sess.run(None, {"image": x.numpy()})[0] - ref).max())
    if err > 1e-3:
        raise SystemExit(f"onnxruntime differs from PyTorch by {err:g}: export is broken")

    classes = list(cfg["classes"])
    sidecar = {
        "type": "seg",
        "name": f"Mobilenet Approach: U-Net + {cfg['encoder']} ({Path(a.weights).name}, {w}x{h})",
        "onnx": name,
        "input_size": [h, w],
        "channel_order": "RGB",
        "scale": 255,
        "mean": list(MEAN),
        "std": list(STD),
        "resize": "stretch",
        "output": "softmax",
        "lane_channel": classes.index("lane") + 1,      # 1-based for MATLAB
        "road_channel": classes.index("road") + 1,
        "classes": classes,
        "max_range_m": 20,
    }
    (HERE / "mobilenet.json").write_text(json.dumps(sidecar, indent=2) + "\n")
    print(f"wrote {onnx_path} ({onnx_path.stat().st_size / 1e6:.1f} MB, max |ort - torch| = {err:.1e}) "
          f"and {HERE / 'mobilenet.json'}")


if __name__ == "__main__":
    main()
