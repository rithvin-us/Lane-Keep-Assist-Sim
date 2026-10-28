# LaneAssist (tsiggi) vs our camera LKA

Reference repository: <https://github.com/tsiggi/LaneAssist> — "Lane Detection and Lane Keeping
algorithm for 1:10 scale autonomous cars in simulated cities", written by
Christos-Alexandros Tsingiropoulos for the Vroom team at the 2023 Bosch Future Mobility
Challenge. Python + OpenCV, MIT licence, 53 commits.

The checkout lives in `LaneAssist/` at the repository root. It is **read-only reference
material**: it is git-ignored (`.gitignore`), nothing in it is edited, and nothing in our build
depends on it. Its licence is copied to `third_party/LaneAssist_LICENSE.txt`; the ported files
carry the attribution the README asks for.

Two facts shape this whole document:

1. **They publish no performance numbers.** No FPS, no accuracy, no IoU, no latency, in the
   README or anywhere in the repo — only GIFs and sample images. `main.py` times
   `lanes_detection()` alone and prints a mean seconds-per-frame; it never computes a frame
   rate, and it processes every 10th frame of the video (`skipped_frames = 10`). Every
   comparison figure here is one we measured.
2. **Their output is not metric.** They fit `x = a·y² + b·y + c` in *image pixels* and steer on
   `angle = 90 − degrees(atan2(H_px, error_px))`. There is no camera calibration, no homography
   and no metre in the repository. Their lanes cannot be scored against ground truth in metres
   until they are put behind a camera model — which is what the port below does.

---

## 1. The two pipelines side by side

| Stage | LaneAssist | Ours |
|---|---|---|
| Colour | `cvtColor(BGR2GRAY)` only | `rgb2hsv` white/yellow + local contrast (`src/vision/lka_paint_probability.m`), or ONNX nets (YOLOP, team UFLDv2) |
| Preprocess | **none**: no blur, no binarisation, no Canny, no ROI mask, no resize | none either (no Canny or Hough anywhere in this repo); effective ROI is the BEV extent |
| Features | 20 image rows over the bottom 50 % of the frame, bottom to top. A lane point is a "square pulse": `p[i] > 170` **and** a rise `p[i] − p[i−3] > 60`, closed by the matching fall, width inside `[min, max + 10]` where `max` tapers linearly with image height | per-pixel lane probability over the whole frame, then 1 m slices in the metric bird's-eye view |
| Grouping | one-to-one point↔lane matching per slice, cost `0.3·slope distance + 0.7·extrapolation error`, gate `0.15·width`; unmatched points open new lanes, so **N lanes** are tracked | per side a sliding window keeps the run **closest to the window centre**; exactly 2 lanes (`src/vision/lka_search_boundary.m`) |
| Geometry | **none** — image pixels throughout | `monoCamera` + `birdsEyeView` inverse perspective, flat road (`src/vision/lka_camera_model.m`), metres |
| Fit | `np.linalg.lstsq` on a Vandermonde of `y`; degree 2 above `0.3·slices` points, else degree 1; rejects `|a| > 0.1` | Huber IRLS, ridge-penalised, degree 1/2/3 by longitudinal span, clothoid form of `LaneBoundaryBus` (`src/vision/lka_fit_boundary.m`) |
| Confidence | `certainty = 0.5·(points/slices·100) + 0.5·max(0, 100 − rms(Δcoeffs vs previous frame))`; the weaker side is distrusted when the gap exceeds 20; lane keeping needs one side above 80 or both above 60, with one frame of hysteresis | `Strength = min(1, span/12) · min(1, mean(prob)/0.7)` — span and brightness, **no temporal term** |
| Adaptivity | pulse threshold moved towards (darkest accepted lane pixel − 20) each frame; `bottom_width`/`top_width` re-estimated from the fitted coefficients each frame | fixed `0.12 / 0.03 / 41 px`; lane width tracked (`state.Width`) but not fed into the search window |
| One line only | desired lane = the visible line offset by a width that tapers `bottom_width → top_width`, with `top_width × 2` on a sharp bend | constant `W/2` normal shift (`src/perception/lka_estimate_lane_center.m`): no taper, no bend widening |
| Extras | dashed-line classifier and stop-line/intersection detector — both written, **both dead** | neither existed before this work |
| State estimation | none | Kalman filter on `[y ψ κ κ′]` with yaw-rate input, χ² gating, latency compensation (`src/perception/lka_lane_tracker_step.m`) |
| Control | `atan(error/H)` on a top-weighted mean, rolling mean of `median_constant = 2` samples, clamp ±23°. **No PID**, although the README claims one and `config.ini` carries `kp/ki/kd` | pure pursuit with a speed look-ahead table + curvature integral, Stanley 2025, LTV MPC, curvature/jerk limiter (UNECE R79), actuator model, supervisor FSM, LDW |
| Platform | Python/OpenCV, 848×480, `resize_img 0.5`, `fps 15` in config | MATLAB/Simulink, 1280×720, 20 Hz (`LKA.Ts = 0.05`), GPU for detector + BEV warp |
| Verification | none in the repo | 21 test files, ISO 11270 case, MIL bench of 5 scenarios × 3 controllers |

## 2. Parameter mapping

Their `config.ini` is tuned for a 1:10 car and an 848×480 camera "455". The port rescales to our
1280×720: column counts ×1.51, row counts ×1.5, and the second-degree limit ×(1.51/1.5²) = ×0.671,
because `a` in `x = a·y²` has units of 1/px. **This rescale is an assumption, not a
measurement** — their own README names `square_pulses_min_height`, `square_pulses_pix_dif`,
`square_pulses_min_height_dif`, `bottom_width` and `top_width` as the parameters to re-tune for a
new camera.

| `config.ini` | `LKA.Vision.LaneAssist` | theirs (848×480) | ported (1280×720) |
|---|---|---|---|
| `slices` | `Slices` | 20 | 20 |
| `bottom_perc_455` | `BottomPerc` | 0.5 | 0.5 |
| `bottom_offset` | `BottomOffset` | 10 | 15 |
| `peaks_min_width_455` | `PeaksMinWidth` | 3 | 5 |
| `peaks_max_width_455` | `PeaksMaxWidth` | 20 | 30 |
| `square_pulses_allowed_peaks_width_error` | `WidthError` | 10 | 15 |
| `square_pulses_pix_dif` | `PixDif` | 3 | 5 |
| `square_pulses_min_height` | `MinHeight` | 170 | 170 |
| `square_pulses_min_height_dif` | `MinHeightDif` | 60 | 60 |
| `max_allowed_width_perc` | `MaxAllowedWidthPerc` | 0.15 | 0.15 |
| `weight_for_width_distance` | `WidthWeight` | 0.3 | 0.3 |
| `weight_for_expected_value_distance` | `ExpectedWeight` | 0.7 | 0.7 |
| `min_peaks_for_lane` | `MinPeaksForLane` | 3 | 3 |
| `optimal_peak_perc` | `OptimalPeakPerc` | 0.3 | 0.3 |
| `extreme_coef_second_deg` | `ExtremeCoef2` | 0.1 | 0.067 |
| `extreme_coef_first_deg` | `ExtremeCoef1` | 3 | 3 |
| `certainty_perc_from_peaks` | `CertaintyFromPeaks` | 0.5 | 0.5 |
| `allowed_certainty_perc_dif` | `AllowedCertaintyDif` | 20 | 20 |
| `min_single_lane_certainty` | `MinSingleCertainty` | 80 | 80 |
| `min_dual_lane_certainty` | `MinDualCertainty` | 60 | 60 |

`bottom_width_455 = 190` / `top_width_455 = 85` have no port: they are half lane widths in
pixels for their single-line fallback, and our equivalent is the metric `LKA.Lane.Width`
(3.5 m nominal) with the tracked `state.Width`.

## 3. The port

Three arms, so the comparison cannot be slanted by picking one:

| Arm | Files | What it measures |
|---|---|---|
| Detector plugin, `V.Detector = 'laneassist'` | `src/vision/lka_laneassist_peaks.m`, `lka_laneassist_cluster.m`, `lka_detector_laneassist.m` | **their feature extraction** under our BEV, search, fit and tracker. Pages: 1 = left line, 2 = right line |
| Full stack | `src/vision/lka_laneassist_sensor.m` | **their perception stack**: their chosen polynomials projected to the ground through `imageToVehicle` and nothing else of ours |
| Their control law | not ported | compared on the code only (section 6); their angle law has no speed, no look-ahead and no lane model, so a closed-loop port would measure our plant, not their algorithm |

Registered in `src/vision/lka_detector_create.m`, `scripts/run_detector_benchmark.m`,
`scripts/measure_lka_timing.m` and `models/perception/README.md`. Tests:
`tests/unit_tests/test_detector_laneassist.m` (7 toolbox-free tests pass; 4 need the Computer
Vision / Automated Driving Toolbox and are filtered where those are absent).

Deliberate deviations from the original, each behind a parameter:

- `ResetWidthOnExit` (default `false` = original). Their pulse loop leaves `pix_num` standing
  when a pulse dies because the level fell below `min_height`, so the next pulse in that row
  starts with a stale width and can be rejected as too wide.
- `AdaptiveFloor` (new). Their adaptation is guarded by `tmp >= self.minimum` where
  `self.minimum` **is** the configured threshold, so the branch can only fire when `tmp` equals
  the threshold exactly: as shipped it never lowers anything, while the reset branch (fewer than
  10 chosen points) raises the threshold back to 170 and destroys the lower value their own
  `real_world_example` sets after construction (`main.py:89`). The idea is sound, their
  implementation is inert; the floor makes it measurable.

## 4. Measured results

### 4.1 Environment of every number below

The measurements in 4.2 to 4.5 were taken while the install was still incomplete, on MATLAB
R2026b with **MATLAB, Simulink, Automated Driving, Computer Vision, Image Processing** only.
They are all **CPU** numbers, and every detector figure is our own measurement: their repository
publishes none.

Since then **Model Predictive Control + Control System** and **Signal Processing** were
installed, which changed the following and nothing else:

- The Simulink models rebuild and compile again (`build_lka_models`, `build_lka_vision_bench`),
  and the unit suite went from 122 passed / 6 failed to **126 passed / 2 failed / 11 filtered**.
  The two remaining failures are `VideoWriter` under `matlab -batch`, not code.
- Signal Processing turned out not to be needed: the one call that wanted it
  (`hann` in `lka_bev_odometry.m`) is now a local `hannWindow`. See finding L10.

Still missing, so still pending: **Deep Learning Toolbox + the ONNX converter** (no `cnn`, no
`yolop`, so this comparison is `classic` against `laneassist`) and **Parallel Computing Toolbox**
(no GPU numbers; `gpuDeviceCount` is undefined). `requirements/matlab_toolboxes.md` lists what
each one unblocks, and `lka_check_requirements` prints the current state.

One environment detail that shaped the method: `VideoReader` and `VideoWriter` fail in
`matlab -batch` on this Linux install ("Unable to initialize the video properties", "The
specified profile is not valid"), so the three clips were decoded with
`ffmpeg -vf fps=20 -q:v 2` and the perception chain was run frame by frame. The annotated videos
were produced the same way (`requirements/system.md`).

### 4.2 Detector accuracy, 60 labelled synthetic frames

`make_synthetic_vision_dataset` with exact ground truth, scored by `lka_detector_benchmark`
(`results/benchmark/synthetic_laneassist.csv`):

| Variant | P | R | F1 | IoU | DetRate L/R | MaeY5 | MaeY10 | MaeY20 | Heading | ms |
|---|---|---|---|---|---|---|---|---|---|---|
| `classic` (ROI on, the default) | 1.000 | 0.991 | **0.996** | **0.983** | 0.917 / 1.000 | 0.0044 m | **0.00062 m** | **0.00091 m** | **0.16°** | **16.84** |
| `laneassist` (their config, rescaled) | 0.917 | 0.424 | 0.578 | 0.249 | 0.633 / 0.667 | 0.0244 m | 0.0204 m | 0.704 m | 4.10° | 10.45 |
| `laneassist` re-tuned to our mount | 0.993 | 0.610 | 0.751 | 0.377 | 0.667 / 0.750 | 0.0333 m | 0.0214 m | 0.267 m | 2.82° | 11.97 |
| `yolop` (ONNX, CPU) | 0.954 | 0.295 | 0.427 | 0.190 | 0.283 / 0.183 | 0.0590 m | 0.0127 m | 0.357 m | 3.42° | 117.90 |

`yolop` scores badly **here for a reason that is not about YOLOP**: it is a network trained on
real road photographs, and this dataset is a synthetic renderer. Its clip numbers below are the
ones to judge it by. `cnn` is still absent because `models/perception/team_cnn.onnx` is not in
git.

Two caveats that keep this honest:

1. **The segmentation columns are unkind to them by construction.** Their detector outputs a
   thin fitted polyline; the ground-truth label is the filled paint stripe. Recall 0.42 is
   mostly that mismatch, not blindness. `DetRate`, `MaeY*` and `MaeHeading` are the fair
   columns, and they say the same thing: 0.63/0.67 of frames against 0.92/1.00, and 20 m error
   of 0.70 m against 0.0009 m.
2. **`classic` looks unrealistically perfect** here because the synthetic renderer paints exactly
   the white stripe its colour model was written for. The clip numbers in 4.4 are the realistic
   ones.

**Why their untuned row is so weak, and it is their design, not the port.** Their scan covers
the bottom `bottom_perc = 0.5` of the frame. On a 1:10 car that is the whole useful track; on our
mount it is 4 m to 7.5 m of road, so the support spans 3.45 m, below `V.Fit.MinSpan = 4 m`, and
the fit is rejected outright. Re-tuning the two parameters their README tells you to re-tune
(`BottomPerc` 0.5 → 0.70 to reach our horizon, `Slices` 20 → 40) cut `MaeY20` from 0.704 m to
0.267 m. It is still 300× the `classic` error at 10 m.

### 4.3 Detector time

Interleaved A/B, 40 alternating repetitions on the real Unreal frame
`results/vision/calibration_raw.png` (1280×720):

| `classic` | median | p95 |
|---|---|---|
| full frame | 15.40 ms | 18.24 ms |
| BEV row ROI (537 of 720 rows) | **13.97 ms (−9.3 %)** | 15.80 ms |

Thresholded map identical (39 935 pixels above 0.5 either way) and the lane offsets identical to
four decimals (L 1.8767 m, R −1.5964 m). The bird's-eye arrays are not bit-identical, because the
41×41 box mean sees replicated rows at the crop edge instead of real ones; `V.Classic.RoiMargin = 20`
keeps that away from the road, and it does not reach the output. A 20-repetition non-interleaved
loop had reported the ROI as *slower* on this frame; that was measurement noise, which is why the
A/B above is interleaved. On the clips the same change is worth far more (4.4), because their
horizon sits higher in frame.

`laneassist` on the same frame, 20-repetition medians: 8.75 ms untuned, 12.98 ms re-tuned
(`results/timing/laneassist_detector_timing.csv`). Their core alone (row scan + clustering + fit
+ certainty, no rasterisation) is 5.99 ms and reads 25 600 of 921 600 pixels — **2.8 %**. That
2.8 % is the finding worth keeping; the 5.99 ms itself compares algorithms, not implementations,
since this is MATLAB with an explicit pixel loop while their Python skips 9 of every 10 frames.

### 4.4 The recorded video feed — the real comparison

Three college-track clips, decoded at 20 Hz, 832×464. The camera pitch and lane width are
auto-calibrated **once per clip with `classic`** and then reused for every variant, so the
geometry is identical and only the detector changes
(`results/benchmark/laneassist_video_comparison.csv`):

| Clip | Variant | Frames | Both lines | Any line | Detect | Lane step | Pipeline | Width |
|---|---|---|---|---|---|---|---|---|
| 12.23.27 (1) | `classic` | 579 | **95.7 %** | 100 % | 4.89 ms | 4.57 ms | 105.7 Hz | 1.81 m |
| pitch −1.6°, 1.77 m | `classic` + ROI | 579 | **95.7 %** | 100 % | **3.03 ms** | 4.50 ms | **133.2 Hz** | 1.81 m |
| | `laneassist` | 579 | 13.6 % | 59.9 % | 5.13 ms | 4.36 ms | 105.0 Hz | 1.27 m |
| | `laneassist` re-tuned | 579 | 52.3 % | 88.6 % | 7.99 ms | 4.88 ms | 78.3 Hz | 0.68 m |
| 12.23.27 | `classic` | 624 | **95.5 %** | 100 % | 4.85 ms | 4.75 ms | 104.2 Hz | 2.10 m |
| pitch 7.6°, 2.22 m | `classic` + ROI | 624 | **95.5 %** | 100 % | **3.86 ms** | 4.75 ms | **116.5 Hz** | 2.10 m |
| | `laneassist` | 624 | 3.7 % | 35.3 % | 5.19 ms | 4.08 ms | 108.6 Hz | 0.80 m |
| | `laneassist` re-tuned | 624 | 44.6 % | 93.3 % | 7.83 ms | 4.80 ms | 79.1 Hz | 0.64 m |
| 12.23.28 | `classic` | 621 | **86.8 %** | 99.8 % | 4.95 ms | 4.56 ms | 105.2 Hz | 1.98 m |
| pitch 0.5°, 1.76 m | `classic` + ROI | 621 | **86.8 %** | 99.8 % | **3.18 ms** | 4.54 ms | **129.8 Hz** | 1.98 m |
| | `laneassist` | 621 | 2.4 % | 17.1 % | 4.02 ms | 3.76 ms | 132.2 Hz | 1.34 m |
| | `laneassist` re-tuned | 621 | 6.4 % | 43.2 % | 5.88 ms | 3.88 ms | 102.8 Hz | −0.17 m |

`yolop` was added once the Deep Learning Toolbox and the ONNX converter were installed, on the
same frames and the same calibration (`results/benchmark/laneassist_video_comparison.csv`):

| Clip | `classic` | `laneassist` | `laneassist` re-tuned | `yolop` |
|---|---|---|---|---|
| 12.23.27 (1) | **95.7 %** @ 3.9 ms | 13.6 % @ 6.1 ms | 52.3 % @ 9.5 ms | 86.7 % @ 95.6 ms |
| 12.23.27 | **95.5 %** @ 4.8 ms | 3.7 % @ 6.3 ms | 44.6 % @ 9.4 ms | 88.6 % @ 95.4 ms |
| 12.23.28 | **86.8 %** @ 3.6 ms | 2.4 % @ 4.9 ms | 6.4 % @ 7.1 ms | 50.1 % @ 94.0 ms |

Both-lines ratio @ median detector time, CPU, measured at the shipped defaults
(`RoiFromBev` on, `ConsistencyWeight` 0). `yolop` is about 20x slower than `classic`
(10.5-10.6 Hz against 208-275 Hz, against a 20 Hz loop budget) and behind it on every clip. It
does beat the re-tuned `laneassist` everywhere, which is the fair reading of a CNN against a
hand-tuned classical detector.

Read-outs:

- **Their detector collapses on real footage**: both lines in 2.4–13.6 % of frames, 6.4–52.3 %
  after the re-tune, against 86.8–95.7 % for `classic`. Their re-tuned lane-width estimate
  (0.68 m, 0.64 m, −0.17 m against a measured 1.76–2.22 m) shows why: it locks the two sides onto
  the wrong pair of edges, and a negative width means it put "left" to the right of "right".
- **The ROI is free speed**: identical both-lines, any-line, width and offset on all three clips,
  with 20–38 % off the detector and 11–26 % on the whole pipeline.
- `classic` on these clips also beats the figure the team CNN recorded in `test_res/summary.csv`
  (90.9 / 90.3 / 59.5 % both-lines), but that run had its own auto-calibration and its own pitch
  estimate, so treat it as an indication, not a head-to-head.
- The lateral offsets (`OffsetRms` 0.79–1.49 m) are **not** an accuracy claim about anything: the
  phone was not centred in the lane and the pitch auto-calibration disagrees between runs
  (−1.6 / 7.6 / 0.5° here against 8.3 / −2.1 / −2.9° recorded earlier). Both-lines ratio is the
  geometry-insensitive number, which is why the verdicts rest on it.

### 4.5 Each adopted change, measured on its own

`results/benchmark/flags_video_split.csv` and `flags_synthetic_split.csv`. Both-lines ratio,
`classic`, same calibration per clip:

| Change | clip 12.23.27 (1) | clip 12.23.28 | synthetic | time | verdict |
|---|---|---|---|---|---|
| baseline (ROI on) | 95.68 % | 86.80 % | F1 0.996, MaeY10 0.00062 m | — | — |
| **A** adaptive paint offset | 95.68 % (=) | 86.80 % (=) | identical | **+1.8 ms/frame** | **rejected**: pays, returns nothing |
| **B1** `ConsistencyWeight = 0.3` | **96.55 % (+0.86)** | 86.80 % (=) | identical | none | **adopted, now default** |
| **B2** `TrustFrames = 2` | 95.16 % (−0.52) | 83.74 % (−3.06) | DetRate → 0 in the stateless bench | none | rejected for now |
| **D1** `TaperExp = 1` | 95.68 % (=) | 83.25 % (−3.54) | MaeY20 −0.00003 m | none | **rejected** |
| **D1b** `TaperExp = 0.5` | 95.68 % (=) | 78.58 % (−8.21) | MaeY20 −0.00003 m | none | **rejected** |
| **D2** cold window from tracked width | 96.03 % (+0.35) | 84.54 % (−2.25) | identical | none | off, mixed |
| **D3** seed nearest line while unseeded | 95.68 % (=) | 86.80 % (=) | MaeY5 −0.0001 m | none | off, no effect here |

Three of these deserve a sentence, because the measurement contradicted the expectation:

- **A did nothing.** The adapted offset is clamped to at most the fixed 0.12, and the 99.9th
  percentile of paint-coloured contrast × 0.6 lands above 0.12 on these frames, so it clamps to
  the old value — while still paying for the sort. Their version works because it adapts to *the
  pixels that a chosen lane actually used*, a geometric feedback our detector does not have.
  Reproducing the idea properly needs that feedback, not a percentile.
- **B2 cannot be judged by the per-frame benchmark at all**: `lka_detector_benchmark` resets the
  tracker every frame, so `Hits` never reaches 2 and `DetRate` goes to 0. That is a property of
  the harness, and it is also the honest answer: a "must agree with itself for N frames" rule is
  only measurable on a sequence, and on these sequences it costs availability and buys nothing,
  because the detector it is meant to catch (`yolop`) cannot run here.
- **The range taper backfired**, exactly as the risk note predicted: looser far-range mass admits
  noise, and the noisiest clip lost 3.5 points at exponent 1 and 8.2 at 0.5.

### 4.6 Still pending, and why

| Pending | Blocked by | Command once unblocked |
|---|---|---|
| `cnn` / `yolop` rows, ONNX parity | Deep Learning Toolbox + ONNX converter, `team_cnn.onnx` | `run_detector_benchmark(Detectors={'classic','laneassist','yolop','cnn'})` |
| Camera-in-the-loop KPIs for `laneassist`, official 241-frame Unreal dataset | an Unreal run (the MPC block compiles again) | `run_lka_vision(Scenarios={'S1_Nominal','S5_InitialOffset'}, Detectors={'laneassist'})`, `run_detector_benchmark()` |
| GPU timings | Parallel Computing Toolbox | `measure_lka_timing` |
| `run_lka_on_test_videos` end to end | `VideoReader` under `matlab -batch` | run it from the MATLAB desktop, or use the ffmpeg path in `requirements/system.md` |
| Dashed / stop-line results reaching the controller | new `LkaInfoBus` fields + model regeneration | wire in `create_lka_buses.m` and `build_lka_models.m`, then `build_lka_models()` |

## 6. Where we are ahead

- **Metric output and traceability.** Lateral offset in metres, curvature in 1/m, heading in rad,
  against OpenDRIVE ground truth; ISO 11270 compliance case and a 26-column KPI table
  (`src/evaluation/lka_compute_kpis.m`). Nothing in their repo can produce any of it.
- **State estimation.** Kalman filter with yaw-rate input, χ² gating, Joseph-form update, latency
  compensation, coasting and take-over requests. They carry the previous polynomial and a boolean.
- **Control.** Pure pursuit with a speed look-ahead table and curvature integral, Stanley 2025,
  LTV MPC with an active-set QP, curvature and jerk limiting to UNECE R79, a 0.5 s authority ramp
  and a 5 Hz actuator filter. Theirs is `atan(error/H)` with a one-sample average, no speed term
  at all.
- **Range.** 30 m BEV with range-dependent fit degree against their bottom 50 % of a 480-row
  frame.
- **Verification.** 21 MATLAB test files, a replay harness, a frame logger, a synthetic renderer
  with exact ground truth, an ONNX parity test against the Python reference. Their repo has two
  ad-hoc scripts in `testing/`.
- **Engineering hygiene.** No dead branches in the perception path; theirs ships four.

What they are genuinely better at, and we should copy rather than dismiss: relative instead of
absolute thresholds, a confidence that includes time, N-lane candidate tracking, and reading only
the pixels they need.

## 7. What was adopted, measured, and what is now default

| # | Parameter | Default now | Measured outcome |
|---|---|---|---|
| **ROI** | `V.Classic.RoiFromBev` | **true** | −9.3 % detector time on the Unreal frame, −20 to −38 % on the clips, detection identical (4.3, 4.4) |
| **B1** | `V.Fit.ConsistencyWeight` | **0** (was briefly 0.3) | +0.87 points on clip 12.23.27 (1), but **−19.2 on clip 12.23.28** (86.8 → 67.6 %) once the prior-staleness fix landed: a re-acquired side scores 0 on the term and drops under `StrengthMin`. Reverted in `8ede4fa` |
| **C** | `lka_dashed_detect`, `lka_horizontal_detect` | new, nothing calls them | 6 unit tests pass; dash 3.0 m / gap 6.0 m recovered within 0.3 m, stop line at 12 m located within 0.2 m |
| L10 | `hannWindow` in `lka_bev_odometry` | replaces `hann` | 4 odometry tests went from erroring to passing |
| A | `V.Classic.Adaptive` | false | no detection change, +1.8 ms/frame — rejected |
| B2 | `V.Track.TrustFrames` | 1 | 2 costs 0.5 and 3.1 points of both-lines, gains nothing measurable here |
| D1 | `V.Search.TaperExp` | 0 | 1.0 costs 3.5 points on the noisy clip, 0.5 costs 8.2 — rejected |
| D2 | `V.Search.ColdHalfWidthFromWidth` | false | +0.35 / −2.25 points, mixed |
| D3 | `V.Search.SeedFirstWhenUnseeded` | false | neutral on these clips |

**Reverting either new default is one parameter.** `V.Classic.RoiFromBev = false` restores the
full-frame detector; `V.Fit.ConsistencyWeight = 0` restores the purely geometric `Strength`.

**What is not verified about the two new defaults.** Both were measured open loop: three clips,
60 labelled synthetic frames, 136 unit tests (116 pass, the rest blocked by missing toolboxes).
Neither has been through `run_lka_mil` or `run_lka_vision`, because the MPC block cannot compile
without the MPC Toolbox. `ConsistencyWeight` feeds `Strength`, which feeds
`LKA.Validation.StrengthMin` and the strength-weighted centre fusion, so the closed-loop KPIs are
the gate it has not yet passed. Run `run_lka_mil()` and `run_lka_vision()` once the toolbox is in,
and compare against the committed `results/mil/lka_mil_kpis.csv`.

**Calibration note on B1.** At weight 0.3 a fit that jumps 0.5 m between frames drops from
`Strength` 1.00 to 0.70, still above `LKA.Validation.StrengthMin = 0.30`: the term reorders
confidence, it does not by itself reject anything. Rejection needs either a weight above 0.7 or a
higher `StrengthMin`, and that trade belongs to the closed-loop run.

**What C does not do.** The dashed and stop-line results are returned but not wired into
`LkaInfoBus`: that needs new bus fields in `data/bus_definitions/create_lka_buses.m` and a new
port in `scripts/build_lka_models.m`, which regenerates the `.slx` models — and a regenerated
model cannot be compiled here while the MPC block is unbuildable. Deliberately left out rather
than committed unverified. (A test run of `build_lka_vision_bench` during this work did overwrite
`models/LKA_Vision_TestBench.slx` with a half-built model; it was restored from git.)

## 7a. Verdict: does their algorithm belong in our model?

**No.** On identical frames with identical geometry, `classic` holds both lane lines in
86.8–95.7 % of real frames against their 2.4–13.6 % (6.4–52.3 % after re-tuning), and on labelled
synthetic frames it is F1 0.996 / `MaeY10` 0.00062 m against 0.751 / 0.0214 m. Their re-tuned
width estimate is wrong by a factor of three and occasionally negative, which means wrong-pair
locking, the one failure mode a lane keeper must not have. `laneassist` stays in the repository as
a **benchmark baseline and reference implementation**, selectable with
`LKA.Vision.Detector = 'laneassist'`, and is not proposed for the competition model.

What we did take from them, with the measured gain: read only the pixels that can matter (−9 to
−38 % detector time), and put time into the confidence (+0.86 points of both-lines). What we
deliberately did not take: their absolute-threshold adaptation (no effect, a real cost), their
range taper (actively harmful on noisy footage), and their control law (no speed, no look-ahead,
no PID despite the README).

## 8. Reproducing this comparison

```bash
git clone https://github.com/tsiggi/LaneAssist.git LaneAssist   # already git-ignored
```

Their code to read, in the order it runs: `main.py` → `src/LaneDetection/detect.py`
(`lanes_detection` → `peaks_detection` → `find_lane_peaks` → `peaks_clustering` →
`find_best_qualified_points` → `verify_with_expected_value` → `choose_correct_lanes` →
`fit_polyfit` → `lanes_post_processing`) → `src/LaneKeeping/lanekeeping.py` (`lane_keeping` →
`desired_lane` → `pid_controller` → `fix_angle`).

Ours, same order: `src/vision/lka_detector_create.m` → the chosen detector →
`src/vision/lka_bev_warp.m` → `src/vision/lka_vision_lane_step.m` →
`src/vision/lka_search_boundary.m` → `src/vision/lka_fit_boundary.m` →
`src/perception/lka_validate_boundaries.m` → `lka_estimate_lane_center.m` →
`lka_lane_tracker_step.m` → `src/controllers/lka_pure_pursuit.m` → `lka_curvature_to_steer.m`.
