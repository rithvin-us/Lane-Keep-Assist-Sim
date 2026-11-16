"""Export the team UFLDv2 (ResNet-18, TuSimple) checkpoint to ONNX for the MATLAB LKA.

Self-contained re-statement of cfzd/Ultra-Fast-Lane-Detection-v2 `parsingNet`
(model/model_culane.py, MIT licence) for the TuSimple ResNet-18 config, so no code
from the UFLDv2 repository is executed. Layer names match the original, and the
checkpoint is loaded with strict=True (the upstream deploy/pt2onnx.py uses
strict=False, which hides missing or unexpected keys) and weights_only=True (no
pickle code runs).

Usage:
    python tools/export_ufld_onnx.py <model_best.pth> <out.onnx>

Output: ONNX opset 17, input "input" [1,3,320,800] float32 (RGB, ImageNet-normalised,
resized to 400x800 then bottom 320 rows), outputs (raw logits)
    loc_row [1,100,56,4], loc_col [1,100,41,4], exist_row [1,2,56,4], exist_col [1,2,41,4].
The script checks onnxruntime against PyTorch on a random input before returning.
"""
import sys

import numpy as np
import torch
import torchvision

# configs/tusimple_res18.py
NUM_GRID_ROW, NUM_CLS_ROW = 100, 56
NUM_GRID_COL, NUM_CLS_COL = 100, 41
NUM_LANES = 4
H, W = 320, 800


class Backbone(torch.nn.Module):
    """model/backbone.py resnet('18'): conv1 ... layer4, returns layer4 features."""

    def __init__(self):
        super().__init__()
        r = torchvision.models.resnet18(weights=None)
        self.conv1, self.bn1, self.relu, self.maxpool = r.conv1, r.bn1, r.relu, r.maxpool
        self.layer1, self.layer2, self.layer3, self.layer4 = r.layer1, r.layer2, r.layer3, r.layer4

    def forward(self, x):
        x = self.maxpool(self.relu(self.bn1(self.conv1(x))))
        return self.layer4(self.layer3(self.layer2(self.layer1(x))))


class ParsingNet(torch.nn.Module):
    """model/model_culane.py parsingNet with use_aux=False, fc_norm=False."""

    def __init__(self):
        super().__init__()
        self.dim1 = NUM_GRID_ROW * NUM_CLS_ROW * NUM_LANES
        self.dim2 = NUM_GRID_COL * NUM_CLS_COL * NUM_LANES
        self.dim3 = 2 * NUM_CLS_ROW * NUM_LANES
        self.dim4 = 2 * NUM_CLS_COL * NUM_LANES
        self.input_dim = H // 32 * W // 32 * 8
        self.model = Backbone()
        self.cls = torch.nn.Sequential(
            torch.nn.Identity(),
            torch.nn.Linear(self.input_dim, 2048),
            torch.nn.ReLU(),
            torch.nn.Linear(2048, self.dim1 + self.dim2 + self.dim3 + self.dim4),
        )
        self.pool = torch.nn.Conv2d(512, 8, 1)

    def forward(self, x):
        fea = self.pool(self.model(x)).view(-1, self.input_dim)
        out = self.cls(fea)
        d1, d2, d3 = self.dim1, self.dim1 + self.dim2, self.dim1 + self.dim2 + self.dim3
        return (out[:, :d1].view(-1, NUM_GRID_ROW, NUM_CLS_ROW, NUM_LANES),
                out[:, d1:d2].view(-1, NUM_GRID_COL, NUM_CLS_COL, NUM_LANES),
                out[:, d2:d3].view(-1, 2, NUM_CLS_ROW, NUM_LANES),
                out[:, d3:].view(-1, 2, NUM_CLS_COL, NUM_LANES))


def main(ckpt_path, onnx_path):
    ckpt = torch.load(ckpt_path, map_location="cpu", weights_only=True)
    state = ckpt["model"] if "model" in ckpt else ckpt
    state = {(k[7:] if k.startswith("module.") else k): v for k, v in state.items()}
    net = ParsingNet().eval()
    net.load_state_dict(state, strict=True)
    print("checkpoint keys:", list(ckpt.keys()) if isinstance(ckpt, dict) else type(ckpt))

    x = torch.zeros(1, 3, H, W)
    torch.onnx.export(net, x, onnx_path, input_names=["input"],
                      output_names=["loc_row", "loc_col", "exist_row", "exist_col"],
                      opset_version=17, do_constant_folding=True, dynamo=False)

    import onnxruntime as ort
    sess = ort.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])
    xr = torch.randn(1, 3, H, W)
    with torch.no_grad():
        ref = [t.numpy() for t in net(xr)]
    got = sess.run(None, {"input": xr.numpy()})
    err = max(float(np.max(np.abs(a - b))) for a, b in zip(ref, got))
    print("outputs:", [(o.name, o.shape) for o in sess.get_outputs()])
    print("max |onnxruntime - torch| = %.2e" % err)
    if err > 1e-3:
        raise SystemExit("ONNX export does not match PyTorch")


if __name__ == "__main__":
    main(*sys.argv[1:3])
