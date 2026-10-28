# CONTEXT.md — session handoff for other agents

Written 2026-10-28. Everything below was measured or observed in this repository on this
machine, not assumed. If a number here disagrees with a CSV in `results/`, trust the CSV and
fix this file.

Read this with `README.md` (what the project is), `docs/lka_controller.md` (how the LKA works)
and `docs/laneassist_comparison.md` (the detector comparison and its findings L1–L11).

---

## 1. What this session did

Task: clone `https://github.com/tsiggi/LaneAssist` as a read-only reference, compare its lane
detection with ours on accuracy and FPS, and adopt what is worth adopting.

Outcome in one line: **their algorithm is not more accurate, so it is a benchmark baseline, not
a candidate for the competition model**; two of their ideas were adopted after measurement, four
were implemented and rejected on their own numbers.

### Merged to `main`

| Commit | What |
|---|---|
| `281ba30` | the LaneAssist port (4 new `src/vision` files), dashed/stop-line detection, the `hann` fix, 13 tests, `docs/laneassist_comparison.md` |
| `4b8792e` | `requirements/`, `lka_check_requirements`, README §3 rewrite, rebuilt `models/*.slx`, the MIL table |
| `8ede4fa` | reverted `V.Fit.ConsistencyWeight` to 0 after it regressed the hard clip |

`f3cd490` (PR #1, `ccr-586dff79-exyk2l`) was merged by someone else mid-session and brought
`V.Fit.AdaptiveRange` / `MaxResidual` / `MinRange`, a CNN fine-tune kit and tight-bend fit fixes.
The only conflict was in `src/vision/lka_vision_params.m` and was resolved keeping **both** sides.
Verified after that rebase: **137 passed, 2 failed, 10 filtered of 147**, both failures being the
headless `VideoWriter` ones below.

---

## 2. The reference repository

`LaneAssist/` at the repo root: **git-ignored, read-only, never edited**. MIT licence copied to
`third_party/LaneAssist_LICENSE.txt`; every ported file carries the attribution their README asks
for. Same pattern as the existing `ADAS_2026/` reference.

Their algorithm, for anyone who needs it without reading their source: grayscale only — no blur,
no threshold image, no Canny, no ROI mask, no perspective transform, no metres anywhere. 20 image
rows over the bottom 50 % of the frame; a lane point is a "square pulse" (`p[i] > 170` **and** a
rise `p[i] − p[i−3] > 60`, closed by the matching fall, width inside a height-tapered window);
points are matched to lanes one-to-one with cost `0.3·slope + 0.7·extrapolation`; each lane is
fitted as `x = a·y² + b·y + c` **in pixels**; steering is `atan(error_px / height_px)` with a
one-sample "median" filter and no PID, despite their README and the `kp/ki/kd` in their
`config.ini`.

Defects found in their code (relevant because we nearly copied the ideas): their adaptive
threshold cannot lower anything as shipped (`tmp >= self.minimum` where `self.minimum` *is* the
threshold) and its reset branch destroys the lower value their own real-world example sets; the
pulse width counter is not cleared on a sub-threshold exit; `certainty` can exceed 100;
`is_dashed` references an undefined `self.log`; `main.py` reassigns `cap` so `highway.mp4` is
never read.

---

## 3. Measured results

All CPU, Parallel Computing Toolbox not installed. Sources: `results/benchmark/*.csv`,
`results/timing/*.csv`.

### 60 labelled synthetic frames (`make_synthetic_vision_dataset`)

| Detector | F1 | IoU | DetRate L/R | MaeY10 | MaeY20 | Heading | ms |
|---|---|---|---|---|---|---|---|
| `classic` | **0.996** | **0.983** | 0.917 / 1.000 | **0.0006 m** | **0.0009 m** | **0.16°** | 16.8 |
| `laneassist` | 0.578 | 0.249 | 0.633 / 0.667 | 0.0204 m | 0.704 m | 4.10° | 10.4 |
| `laneassist` re-tuned | 0.751 | 0.377 | 0.667 / 0.750 | 0.0214 m | 0.267 m | 2.82° | 12.0 |
| `yolop` | 0.427 | 0.190 | 0.283 / 0.183 | 0.0127 m | 0.357 m | 3.42° | 117.9 |

Two caveats, both important: `classic` is flattered because the renderer paints exactly the white
stripe its colour model expects, and `yolop` is punished because a CNN trained on real roads does
not fire reliably on a synthetic scene. The clip table below is the realistic one.

### Three recorded college-track clips, 20 Hz

| Detector | 12.23.27 (1) | 12.23.27 | 12.23.28 | detect | rate |
|---|---|---|---|---|---|
| `classic` | **95.7 %** | **95.5 %** | **86.8 %** | 3.6–4.9 ms | 208–275 Hz |
| `laneassist` | 13.6 % | 3.7 % | 2.4 % | 4.9–6.3 ms | 158–206 Hz |
| `laneassist` re-tuned | 52.3 % | 44.6 % | 6.4 % | 7.1–9.5 ms | 106–140 Hz |
| `yolop` | 86.7 % | 88.6 % | 50.1 % | 94–96 ms | 10.5–10.6 Hz |

Column is both-lines ratio, all measured at the shipped defaults (`RoiFromBev` on,
`ConsistencyWeight` 0). `yolop` is ~20x slower than `classic` against a 20 Hz loop budget and
behind it on every clip, but ahead of the re-tuned `laneassist` everywhere.

Why both-lines and not lateral offset: the camera is unmeasured. Pitch auto-calibration returned
−1.6 / 7.6 / 0.5° in one run and 8.3 / −2.1 / −2.9° in another **on the same footage**, and lane
width comes out 1.77–2.22 m against the 3.5 m nominal. Offsets on these clips are therefore not
an accuracy claim about anything; both-lines ratio is geometry-insensitive.

### MIL, after rebuilding the models

All five previously committed Pure Pursuit rows reproduce **bit-identical**. With MPC installed
the table gained MPC and Stanley 2025 rows, and they show an open defect: **Stanley 2025 fails
every scenario** (`RulebookPass 0`, wheels out on S1, S2, S3, S5, in-lane down to 73.0 % on S3).
MPC passes everywhere except S4, which Pure Pursuit also fails.

---

## 4. What changed in our code, and why

| Change | State | Evidence |
|---|---|---|
| `V.Classic.RoiFromBev` — compute the paint map only on rows below the horizon | **default on** | −9.3 % detector time on the Unreal frame (interleaved A/B), −20 to −38 % on the clips, detection identical everywhere measured |
| `lka_bev_odometry` no longer calls `hann` | **fixed** | Signal Processing Toolbox dependency killed the odometry and video path; 4 tests went from erroring to passing |
| `V.Fit.ConsistencyWeight` — temporal term in `Strength` | implemented, **default 0** | +0.9 points on clip 1, **−19.2 on clip 3** once the prior-staleness fix landed |
| `V.Classic.Adaptive` — per-frame paint offset | implemented, default off | no detection change on any clip, +1.8 ms/frame |
| `V.Track.TrustFrames` | implemented, default 1 | 2 costs 0.5 and 3.1 points of both-lines, gains nothing measurable |
| `V.Search.TaperExp` — range-tapered slice thresholds | implemented, default 0 | 1.0 costs 3.5 points on the noisy clip, 0.5 costs 8.2 |
| `V.Search.ColdHalfWidthFromWidth`, `SeedFirstWhenUnseeded` | implemented, default off | +0.35 / −2.25, and neutral |
| `lka_dashed_detect`, `lka_horizontal_detect` | new, nothing calls them | 6 tests pass; dash 3.0 m / gap 6.0 m within 0.3 m, stop line at 12 m within 0.2 m |

New files: `src/vision/lka_laneassist_{peaks,cluster,sensor}.m`, `lka_detector_laneassist.m`,
`lka_dashed_detect.m`, `lka_horizontal_detect.m`, `src/utilities/lka_check_requirements.m`,
`tests/unit_tests/test_detector_laneassist.m`, `test_vision_lane_markings.m`.

The detector plugs into the existing contract (`det.Detect(rgb) → prob`), so
`LKA.Vision.Detector = 'laneassist'` works everywhere `classic` does. Parameters live in
`V.LaneAssist`, rescaled from their 848×480 config to our 1280×720 (columns ×1.51, rows ×1.5,
second-degree limit ×0.671) — **an assumption, not a measurement**.

---

## 5. Environment traps that cost time here

1. **Toolbox installed into a running MATLAB is only half-installed.** Built-in MEX functions
   register at startup. Symptom: `Invalid function name '_visionIsObject'` from `insertShape`.
   Fix: restart MATLAB. The search path can be refreshed in place by adding the entries
   `pathdef()` has and `path` does not.
2. **`VideoReader` / `VideoWriter` do not work under `matlab -batch`** on this box ("Unable to
   initialize the video properties", "The specified profile is not valid"). Two tests fail for
   this reason alone. Use the MATLAB desktop, or decode and mux with ffmpeg
   (`requirements/system.md`).
3. **`test_vision_bench_build` regenerates `models/LKA_Vision_TestBench.slx`** and leaves a
   half-built 164 KB file if the compile fails. It happened three times. Check
   `git status models/` after a suite run.
4. **`lka_controller_params` is a script, not a function**: it writes into the *caller's*
   workspace. A local variable named `sensor` (e.g. a `LaneSensorBus` from a loop) shadows the
   params struct and the next call dies with `Unrecognized field name "lateralNoiseStd"`.
5. **MPC Toolbox gates everything Simulink.** Without it, `LKA_Controller/Lateral
   Controllers/MPC Lane Keeping/Curvature MPC (active-set QP)` cannot resolve its output sizes,
   so even a pure-pursuit-only run will not compile.

---

## 6. How to reproduce anything here

```matlab
run('config/startup.m')
lka_check_requirements                     % what is installed, what each missing item costs
runtests('tests/unit_tests')               % 137 pass; the 2 failures are VideoWriter, headless only

% accuracy with real labels (no Unreal needed)
V = lka_vision_params('baja'); V.Ts = 0.05; V.UseGpu = false;
make_synthetic_vision_dataset('data/vision_datasets/synthetic_la', 60, V);
lka_detector_benchmark('data/vision_datasets/synthetic_la', {'classic','laneassist','yolop'}, V)

% the official tables
run_detector_benchmark(Detectors={'classic','laneassist','yolop','cnn'})   % needs Unreal
run_lka_mil()                                                              % 5 scenarios x 3 controllers
measure_lka_timing                                                         % per-stage ms and Hz
run_lka_on_test_videos()                                                   % desktop MATLAB only
```

Annotated video without `VideoWriter`: render frames with `lka_annotate_frame` and mux with
ffmpeg. Four examples exist at `test_res/<clip>/lka_annotated_<detector>.mp4` — on disk, **not in
git** (51 MB, ignored by `.gitignore:73`).

---

## 7. Open work, highest value first

1. **Stanley 2025 fails every MIL scenario.** Wheels out on four of five. `src/controllers/lka_stanley_2025.m`.
2. **Measured vehicle and camera numbers.** The largest error source in every clip result: lens
   height, pitch, mount X, phone FOV, the track's true lane width, wheelbase, understeer
   gradient, steer limits, loop delay. Sheet: `docs/vehicle_measurements.md`; they land in
   `data/parameters/vehicle_params.m`, `sensor_params.m`, `V.Camera`.
3. **`team_cnn.onnx` is missing** (team Drive, >100 MB), so the comparison is 3-way not 4-way.
4. **Parallel Computing Toolbox** for any GPU number; all timings here are CPU.
5. **Wire dashed/stop-line into `LkaInfoBus`** (`create_lka_buses.m` + `build_lka_models.m`, then
   rebuild).
6. Findings still open in `docs/laneassist_comparison.md`: **L4** single-line fallback has no
   width taper or bend widening; **L6** the search keeps the run nearest the window centre, so an
   arrow can capture it; **L2** a real adaptive threshold needs support-pixel feedback from the
   fit, not a percentile.

---

## 8. Conventions worth keeping

- Models are **generated**: edit `scripts/build_lka_models.m`, never the `.slx` by hand.
- Every behaviour change to the shared pipeline arrives **behind a parameter, defaulted off**,
  and is switched on only with a measurement next to it. Three of the four changes proposed this
  session were rejected by their own numbers — that is the process working.
- Reference checkouts (`ADAS_2026/`, `LaneAssist/`) are git-ignored and never edited.
- Do not reintroduce a Signal Processing Toolbox dependency; the competition machine will not
  have it.
