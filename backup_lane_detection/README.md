# backup_lane_detection — archived lane detectors

These lane detectors were active until 2026-11-16. They were replaced by the
**Mobilenet Approach** (`perception/Mobilenet_Approach/`). Nothing in this folder is on the
MATLAB path or is called by the active code. It is kept so that any of them can be
compared against or brought back.

## What is here

| Folder | Contents |
|---|---|
| `matlab/` | detector code: `lka_detector_classic.m` + `lka_paint_probability.m` (classic white/yellow contrast), `lka_detector_laneassist.m` + `lka_laneassist_*.m` (port of tsiggi/LaneAssist), `lka_detector_ufld.m` + `lka_ufld_*.m` + `lka_rasterize_lanes.m` (team UFLDv2 "cnn"), `lka_dashed_detect.m` / `lka_horizontal_detect.m` (lane-marking reports nothing called), `backup_detector_params.m` (their `V.Classic`, `V.LaneAssist`, `V.Marking` parameters) |
| `models/perception/` | YOLOP lane head (`yolop_ll_384x640.onnx`, `yolop.json`), team CNN sidecar `cnn.json` (its ONNX was never in git), their README and licence |
| `tools/` | UFLDv2 export and reference scripts, ONNX parity fixtures, the UFLDv2 fine-tune kit |
| `tests/` | the tests of those detectors |
| `docs/laneassist_comparison.md` | the full classic vs LaneAssist vs YOLOP comparison (findings L1–L11) |
| `results/` | detector benchmark tables, Unreal closed-loop runs with those detectors |
| `test_res/<detector>/` | their runs on the recorded college clips |
| `reference/LaneAssist/` | the read-only tsiggi/LaneAssist checkout (git-ignored, 322 MB) |
| `LaneAssist_LICENSE.txt` | its MIT licence |

## Last measured numbers (from `CONTEXT.md`, 2026-10-28)

Both-lines ratio on the recorded college clips, 20 Hz, CPU:

| Detector | 12.23.27 (1) | 12.23.27 | 12.23.28 | college_lane | detect ms |
|---|---|---|---|---|---|
| classic | 95.7 % | 95.5 % | 86.8 % | 28.3 % (960x540), 40.6 % (1280x720) | 4–14 |
| laneassist re-tuned | 52.3 % | 44.6 % | 6.4 % | – | 7–10 |
| yolop | 86.7 % | 88.6 % | 50.1 % | 1.3 % | 87–96 |
| cnn (UFLDv2, GPU) | 90.3 % | 90.9 % | 59.5 % | – | 12–13 |

The new detector's numbers are in `perception/README.md`.

## How to bring one back (for a comparison run)

The active code finds a detector only through `perception/<Approach>/deploy/<name>.json`.
The classic and LaneAssist detectors have no ONNX file, so the simplest way is a temporary
session without changing the repository:

```matlab
run('config/startup.m')
addpath(fullfile(pwd, 'backup_lane_detection', 'matlab'))
evalin('base', 'lkaProfile = ''baja''; lka_controller_params;'); LKA = evalin('base', 'LKA');
V = backup_detector_params(LKA.Vision);      % adds V.Classic, V.LaneAssist, V.Marking
det = lka_detector_classic(V);               % or lka_detector_laneassist(V)
prob = det.Detect(rgb);                      % same contract as the active detector
```

YOLOP is an ONNX "seg" model. To compare it in the normal tools, copy it into an approach
folder: `perception/YOLOP_Approach/deploy/` holding `yolop.json` and `yolop_ll_384x640.onnx`.
Then use `Detector='yolop'`.

To restore one permanently, `git mv` its files back to `src/vision/`. Then add its `case`
back to `lka_detector_create.m`; the history shows the old version (`git log --follow
src/vision/lka_detector_create.m`).
