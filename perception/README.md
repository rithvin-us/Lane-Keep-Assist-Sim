# perception/: lane detectors

One folder per approach. Exactly one is the default (`LKA.Vision.Detector` in
`src/vision/lka_vision_params.m`); today that is **`mobilenet`** →
`Mobilenet_Approach/`. Detectors that were replaced live in `../backup_lane_detection/`.

## The contract (all detectors)

```
RGB image (H x W x 3 uint8)  ──►  det.Detect(rgb)  ──►  lane-line probability map (H x W single, 0..1)
```

Everything after the map is shared and does not change when the detector changes:
bird's-eye warp → lane search + fit → `LaneSensorBus` → `LKA_Controller`.

An approach folder provides:

```
perception/<Name>_Approach/
├── deploy/<name>.json      sidecar: what MATLAB needs (below)
├── deploy/<model>.onnx     the network: input 1x3xHxW float32, output 1xCxHxW
├── realtime/               optional: live node (same TCP protocol as Mobilenet_Approach/realtime/lane_node.py)
├── training/  weights/     however the approach is trained
└── README.md
```

Sidecar fields (`type: "seg"`, read by `src/vision/lka_detector_onnx_seg.m`):
`onnx`, `input_size` [H W], `channel_order` (RGB/BGR), `scale`, `mean`, `std`,
`resize` (`stretch` or `letterbox` + `pad_value`), `output` (`softmax` / `sigmoid` / `prob`),
`lane_channel` (1-based), optional `max_range_m` (trusted look-ahead).

## Trying a new model (no other code changes)

1. Copy `Mobilenet_Approach/` as `<Name>_Approach/`, train, and export the ONNX and the
   sidecar `deploy/<name>.json`.
2. `lka_detector_create('list')` shows `<name>`.
3. Compare it on the same clips, with the same metric script:
   ```matlab
   run_lka_on_test_videos(Detector='<name>')                         % -> test_res/<name>/
   run_lka_on_video(fullfile(getenv('HOME'),'Downloads','college_lane.mp4'), Detector='<name>', ...
       Camera='auto', OutDir=fullfile(pwd,'test_res','<name>','college_lane'))
   ```
   ```bash
   python3 tools/clip_metrics.py test_res/mobilenet test_res/<name>
   ```
4. Add its row to the table below. Make it the default only when it wins: change
   `V.Detector` in `lka_vision_params.m`, and archive the old default in `backup_lane_detection/`.

## Comparison (measured 2026-11-16, this laptop, CPU only)

### Recorded college clips: same lane fit, same controller, only the detector differs

Scored by `tools/clip_metrics.py` from each run's `lka_video_results.csv`. "jumps" and
"swaps" count frame-to-frame steps of a reported line of more than 0.3 m and 1 m: jumps
are visible flicker, and a swap means the fit moved to another line. Lower is better.

| Clip | Detector | both lines | any line | jumps | swaps | detector ms (MATLAB, CPU) |
|---|---|---|---|---|---|---|
| college_lane (60 s) | **mobilenet** | **53.2 %** | 87.3 % | **477** | **111** | 110 |
| | classic 960x540 (archived) | 28.3 % | 95.2 % | 781 | 527 | 7 |
| | classic 1280x720 (archived) | 40.6 % | 98.8 % | 812 | 396 | 14 |
| | yolop (archived) | 1.3 % | 53.1 % | 198 | 25 | 87 |
| 12.23.27 | **mobilenet** | 87.2 % | 94.2 % | 185 | **12** | 103 |
| | cnn UFLDv2, GPU (archived) | **90.9 %** | 100 % | 202 | 45 | 13 |
| | classic (archived)¹ | 95.5–99.0 % | – | – | 3–38 | 4–5 |
| 12.23.27 (1) | **mobilenet** | 75.1 % | 87.6 % | 153 | **8** | 105 |
| | cnn (archived) | 90.3 % | 99.1 % | 107 | 21 | 13 |
| | classic (archived)¹ | 95.7–96.7 % | – | – | 14–29 | 4–5 |
| 12.23.28 | **mobilenet** | 80.6 % | 86.0 % | 242 | 56 | 103 |
| | cnn (archived) | 59.5 % | 99.2 % | 287 | 83 | 12 |
| | yolop (archived)¹ | 50.1 % | – | – | – | 94–96 |
| | classic (archived)¹ | 67.1–86.8 % | – | – | 20–147 | 4–5 |

¹ From `CONTEXT.md` (2026-10-28, other metric script, with and without `AdaptiveRange`); no
per-frame CSV kept.

Fitted line vs painted line, MobileNet (median / 90 %): 0.04–0.05 m / 0.08–0.11 m on the three
WhatsApp clips, 0.10 m / 0.37 m on college_lane.

How to read it:
- **college_lane is very probably in the MobileNet training data**: the shadow example in
  `Mobilenet_Approach/README.md` is a frame of it. Its win there (53 % vs 28–41 % both
  lines, 4x fewer swaps) is optimistic. The WhatsApp clips are the fair test.
- On the WhatsApp clips MobileNet finds both lines **less often than classic** (75–87 % vs
  96–99 % on the two easy clips). It swaps lines least of all detectors (8–12), and on the hard
  clip 12.23.28 it beats the CNN and YOLOP by 21–30 points.
- Its lane width estimate is noisier (width CV 0.33–0.84 vs 0.24–0.44 for the CNN). The thin
  lane class (IoU 0.47 in training) gives broken masks in the far range.

### Rendered frames (60 synthetic frames, proxy for the Unreal / competition simulator)

| Detector | F1 | IoU | detection L / R | MaeY10 | MaeY20 |
|---|---|---|---|---|---|
| **mobilenet** | 0.597 | 0.352 | 0.67 / 0.27 | 0.015 m | 0.031 m |
| classic (archived) | 0.996 | 0.983 | 0.92 / 1.00 | 0.0006 m | 0.0009 m |
| yolop (archived) | 0.427 | 0.190 | 0.28 / 0.18 | 0.013 m | 0.357 m |

**Risk:** MobileNet has never seen a rendered lane. In the Unreal camera bench and the
competition co-simulation it finds the left line in 2 of 3 frames and the right-hand line in about 1 of 4. Fix: label
about 50 Unreal frames (`run_lka_vision(Dataset=true)`) and retrain.

### Blender lighting track (rendered drive, exact ground truth)

`data/scenarios/lka_lighting_track.m` → Blender (`tools/blender/`) → 1,050 frames at 15 km/h
with a ±0.3 m weave → `run_lka_on_video` with the exact camera → `score_lighting_track`.
Line error = |reported − true| line offset 8 m ahead (where the camera sees it).
Result: `test_res/mobilenet/lighting_track/` (annotated video, `lighting_track_score.csv`).

| Condition / section | frames | both lines | error L / R at 8 m |
|---|---|---|---|
| all | 1050 | 84.4 % | 0.26 / 0.28 m |
| normal | 349 | 90.8 % | 0.29 / 0.28 m |
| sun (bright) | 153 | 88.2 % | 0.09 / 0.41 m |
| real shadow (dark) | 472 | 82.8 % | 0.24 / 0.25 m |
| **glare** | 76 | **56.6 %** | **0.78** / 0.26 m |
| straight | 623 | 91.0 % | 0.17 / 0.20 m |
| 30° bend, R 25 m | 125 | 93.6 % | 0.32 / 0.31 m |
| S-bend 45°, R 20 m | 151 | 77.5 % | 0.47 / 0.54 m |
| **60° bend, R 15 m** | 151 | **56.3 %** | 0.44 / 0.34 m |

LKA active 99.8 % of the frames. On the straight the reported offsets at the rear axle
match the truth within 2–5 cm. Shadows cost about 8 points; glare (asphalt almost as
bright as paint) and the 60° bends are where it fails. Training frames with glare and
tight bends are the next data to label.

### Latency

| Where | Runtime | Per frame |
|---|---|---|
| Live node (`realtime/lane_node.py`) | onnxruntime CPU, 8 threads, idle CPU | 25.5 ms p50 / 27.9 ms p95 |
| Live node, MATLAB running alongside | onnxruntime CPU, 6 threads | 32–40 ms |
| Live end to end (node + MATLAB warp, fit, controller) | measured with `run_lka_live` | 37.5 + 10.8 ms median: inside the 50 ms budget at 20 Hz |
| Offline in MATLAB | `dlnetwork`, CPU (no Parallel Computing Toolbox) | 103–115 ms: fine for recorded clips, not for live |
| PyTorch | CPU fp32, 8 / 4 / 1 threads | 49 / 72 / 220 ms |
| Jetson Orin Nano, TensorRT fp16 | not measured | estimate 8–15 ms from FLOPs |

Model: 6.63 M parameters, 14.2 GFLOPs per 480x288 frame. 88 % of the FLOPs are in the U-Net
decoder (the MobileNetV2 encoder is 1.65 GFLOPs). A faster model should get a lighter decoder
(smaller `decoder_channels`) before a smaller encoder or a lower resolution, which would cost
the thin lane class.
