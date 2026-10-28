# Lane detection models

Every lane detector produces the same thing: **camera image in → per-pixel lane-line
probability map out** (`det.Detect(rgb)`). The rest of the chain (bird's-eye view, search,
fit, tracker) is shared. A neural model is an ONNX file plus a JSON sidecar next to it; the
sidecar `type` selects the adapter. Select the detector with `LKA.Vision.Detector`.

| Detector | Sidecar | Adapter | Model |
|---|---|---|---|
| `yolop` | `yolop.json` (`seg`) | `lka_detector_onnx_seg.m` | `yolop_ll_384x640.onnx`: YOLOP lane-line head, exported from ADAS_2026 (`export_yolop_onnx.py`) |
| `cnn` | `cnn.json` (`ufld_v2`) | `lka_detector_ufld.m` | `team_cnn.onnx`: team UFLDv2 ResNet-18 (TuSimple), **not in git** (> 100 MB), copy from the team Drive |
| `classic` | none | `lka_detector_classic.m` | white/yellow local-contrast reference detector |
| `laneassist` | none | `lka_detector_laneassist.m` | ported tsiggi/LaneAssist square-pulse row histograms, two pages (left / right); parameters in `LKA.Vision.LaneAssist`, comparison in `docs/laneassist_comparison.md` |

## Exporting the team UFLDv2 model (Colab, `UFLDv2_Lane_Detection_Colab.ipynb`)

1. Run cells 1–4 (GPU, Drive, clone, `my_interp` build).
2. Find the checkpoint: `!find /content/drive/MyDrive/lane_logs/tusimple_res18 -name model_best.pth`.
3. Set `TEST_MODEL` in cell 25 to that exact path (it is still the `<run_folder>` placeholder) and run
   cell 25 once. Record the printed TuSimple test accuracy / F1 here.
4. Run cell 27: `!python deploy/pt2onnx.py --config_path configs/tusimple_res18.py --model_path {TEST_MODEL}`.
5. Check the export:
   ```python
   import onnx, glob, os
   f = sorted(glob.glob(os.path.dirname(TEST_MODEL) + '/*.onnx') + glob.glob('*.onnx'))[-1]
   m = onnx.load(f); print(f)
   print([(o.name, [d.dim_value for d in o.type.tensor_type.shape.dim]) for o in m.graph.output])
   ```
   Expected: 4 outputs with shapes `[1,100,56,4]` (loc_row), `[1,100,41,4]` (loc_col),
   `[1,2,56,4]` (exist_row), `[1,2,41,4]` (exist_col), any order. Input `[1,3,320,800]`.
6. Put the `.onnx` on the team Drive, then copy it to `models/perception/team_cnn.onnx` on the
   simulation PC.

## Contract for other models

- `type: "seg"`: ONNX input `1 x 3 x H x W` float32; output `1 x C x H x W` (same H, W).
  `C = 1` gives a lane logit (`"output": "sigmoid"`) or a probability (`"output": "prob"`).
  `C ≥ 2` gives class scores with the lane class at `lane_channel` (1-based, `"output": "softmax"`).
  The preprocessing is described by `channel_order`, `scale`, `mean`/`std`, and `resize`
  (`letterbox` with `pad_value`, or `stretch`).
- `type: "ufld_v2"`: UFLDv2 row/column-anchor heads as above. A different dataset config
  (CULane, CurveLanes) only changes the sidecar numbers.
- A model with any other output needs its own adapter returning the same probability map.

## How a model is judged

`scripts/run_detector_benchmark.m` scores every detector on the same frames:
- tolerant precision / recall / F1 (3 px) and IoU;
- **pixel accuracy**, reported only to show its background bias: an empty mask already scores about 96%;
- detection rate per side;
- lateral error of the fitted boundary at 5 / 10 / 20 m, and heading error;
- ms per frame;
- closed-loop lane-keeping KPIs.

## Team CNN notebook review (2026-10-16)

- Training: UFLDv2 ResNet-18, TuSimple train split (3 626 images), 100 epochs, SGD lr 0.05.
  Evaluated every epoch on the TuSimple test split (2 782 images, 348 batches).
- The "96 %" is the TuSimple **point accuracy** (0.9596–0.9600, F1 0.962–0.964 in the last
  epochs). It is a standard lane metric, not pixel accuracy, so it is meaningful.
- `model_best.pth` is the best epoch **on the test split**, so the number is slightly optimistic
  (no separate validation split).
- Domain: TuSimple is US highways in daylight (1280×720, same as the simulator camera). The
  official LKA track and the Unreal renders are different; the benchmark measures that gap.
- The checkpoint file (771 MB) is much larger than the network (ResNet-18 UFLDv2); it probably
  also stores optimizer state. Only the ONNX export is needed for simulation.

## Provenance and licence

YOLOP: https://github.com/hustvl/YOLOP (licence: see `YOLOP_LICENSE.txt`), weights
`End-to-end.pth` trained on BDD100K (BDD100K licence: research use).
UFLDv2: https://github.com/cfzd/Ultra-Fast-Lane-Detection-v2, trained by the team on TuSimple.
