# Bug and error report

Every defect found while swapping in the Mobilenet detector and building the live, Blender
and closed-loop benches (2026-11-16), with cause, fix and evidence. Open items at the end.
Pre-existing findings from earlier sessions stay in `CONTEXT.md` §7.

## Fixed

| # | Symptom | Cause | Fix | Evidence |
|---|---|---|---|---|
| B1 | `Render server makes 640x360, the camera expects 1280x720.` | `run_lka_blender_sim` built the bench mid-run; the build reloads `lka_controller_params` and overwrote the run's `LKA` | build before `LKA` is set | smoke run then passed (10 s, in lane 100 %) |
| B2 | Unreal starts, never connects; `3D Simulation engine interface read error.` | MathWorks: "Run-time import of Universal Scene Description files is not supported by Unreal Engine on this operating system" (Linux). Affects **every** USD scene, `LKA.usd` included | Blender closed-loop bench (`LKA_Blender_TestBench`); Unreal scene kept for Windows (`S6_LightingTrack`) | `results/blender/` |
| B3 | `The specified profile is not valid.` from `VideoWriter('…','MPEG-4')`, headless and desktop | no MPEG-4 profile in this Linux MATLAB | `src/utilities/LkaVideoWriter.m`: Motion JPEG AVI, then ffmpeg to the same `.mp4` path; used by `LkaVisionRecorder`, `run_lka_on_video`, tests | 2 always-failing tests now pass |
| B4 | 29 of 30 frames read back | H.264 B-frames shift timestamps by 0.1 s; last frame time > `Duration`, so the reader stops | `-bf 0` | `test_lka_video_tools` passes |
| B5 | empty, unreadable `.avi` left after a failed run | writer closed with zero frames | delete the AVI and the MP4 when no frame was written | `test_vision_assets/testVideoWriterFallsBackToFfmpeg` |
| B6 | `Array indices must be positive integers…` in `run_lka_on_video` pass 2 | second reader still got the old eye argument after the `Crop` change | pass the same `prep` function | college clips rendered |
| B7 | college_lane results not reproducible | the clip has black bars (960x818, picture 960x538); earlier runs used a hand-cropped copy | `Crop='auto'` (content box over 12 frames, as in training) | `[LKA video] letterbox: using rows 163-704…` |
| B8 | `LaneWidth` ignored when a camera struct is given | only applied in the auto-calibration branch | applied in both | lighting track scored with 3.0 m |
| B9 | `Attempt to add "projectRoot" to a static workspace.` | `run_lka_live` has a nested function; `startup.m` is a script | run startup in the base workspace | live test passed |
| B10 | `Attempt to read … already-destroyed variable 'n'` at the end of a live run | `onCleanup` ran after nested-function variables were destroyed | session snapshot in a handle (`containers.Map`) read by a subfunction | 405-frame live test saved and replayed |
| B11 | node core dump `terminate called without an active exception` | port already in use; exception left the camera thread running | bind the port first, clear `port … is busy` message | `lane_node.py` |
| B12 | `Method 'predict' is not defined for class 'lka_onnx_…'` | the ONNX importer writes layer code next to the model; folder not on the path | `config/startup.m` adds every `perception/*/deploy` | `test_mobilenet_detector` |
| B13 | `V.Track.CurvatureMax` / `CheckAtNear` assigned twice | copy-paste | duplicate removed | — |
| B14 | `test_vision_benchmark` fails after the swap | thresholds were classic's quality on rendered frames | regression floor at the measured Mobilenet level, documented as such | test passes |
| B15 | dashboard lane chart unreadable; "Live camera · live camera" | one-frame outliers set the scale; run name duplicated the kind | chart clamped to ±3 m; source name used | screenshot check |
| B16 | lane chart showed 60 s of a 2-minute window | telemetry keeps the last 1200 samples (60 s at 20 Hz) | documented; path view keeps the whole run (20000 points) | — |
| B18 | dashboard called the Blender run "Unreal closed loop" | one label for every `sim` run | "Closed-loop simulation" | screenshot |
| B19 | dashboard latency chart and heading tile empty during closed-loop runs | the recorder publishes no timing; the detector block never published | `LkaVisionLaneDetector` publishes detector / warp / fit ms through the new `lka_monitor_publish('extra', …)`; recorder adds the heading | live.json shows `DetectMs 153.3, WarpMs 2.2, FitMs 5.1` |
| B17 | dead code | `pure_pursuit_steer.m` (no callers), lane-marking reports nobody called | removed / archived in `backup_lane_detection/` | dead-code scan incl. `.slx` contents |

## Open

| # | Issue | Impact | Next step |
|---|---|---|---|
| O1 | Mobilenet on rendered frames: F1 0.60 (classic 0.996) | Unreal / competition co-simulation | label ~50 simulator frames, retrain |
| O2 | glare (56.6 % both lines) and 60° R15 bends (56.3 %) | lighting track, likely the real track at noon | label glare + tight-bend frames |
| O3 | network in MATLAB: 103–132 ms on the CPU | offline only; live uses the node | Parallel Computing Toolbox, or keep the node |
| O4 | camera mount never measured | every metric result on the recorded clips | `docs/vehicle_measurements.md` |
| O5 | lane width estimate noisy on real clips (CV 0.33–0.84) | single-line frames move the centre | width prior from the measured track |
| O6 | `run_lka_live` steering is a preview (no Stateflow supervisor) | live display only; the full controller runs at the end | generated controller on the Jetson |
| O7 | Stanley 2025 fails every MIL scenario (pre-existing) | only if selected | `src/controllers/lka_stanley_2025.m` |
| O8 | MATLAB does not run on the Jetson | deployment | code generation / port (`docs/algorithm_review.md` §5) |
| O9 | TensorRT latency on the Jetson not measured | latency budget | `trtexec` (`docs/algorithm_review.md` §5.2) |
| O10 (B20) | lane-centre estimate at the rear axle is wrong by 0.3–0.9 m RMS (2.4 m peak) at bend entries and in the S-bend, although both lines are found | closed loop on the lighting track: the car stays within 0.49 m because pure pursuit aims Ld ahead, where the fit is right, but in the S-bend (curvature flips inside the fit window) the car drifts 0.44 m | anchor the estimate at the first visible row (x ≈ 5.5 m) and let the tracker carry it to the axle, or fit with a clothoid prior from the previous frame; measure with `run_lka_blender_sim` |
| O11 (B21) | 27 lane-departure warnings in a run that never left the lane | LDW uses the same axle offset as O10 | fixed by O10; until then do not trust LDW on tight tracks |
