# CLAUDE.md: Lane-Keep-Assist-Sim

Read `CONTEXT.md` §0 first (current state), then `README.md` (layout, commands).

## Layout in one breath
- `perception/<Name>_Approach/`: lane detectors; the active one is `Mobilenet_Approach` (`LKA.Vision.Detector = 'mobilenet'`).
  MATLAB finds a detector through `perception/*/deploy/<name>.json` (`lka_detector_create`).
- `src/` is MATLAB algorithm code. `models/*.slx` are **generated** by `scripts/build_lka_models.m` (and the
  `build_lka_*_bench.m` scripts): edit the scripts, never the `.slx`.
- `backup_lane_detection/` holds archived detectors. It is off the path; never call it from active code.
- `tools/blender/` builds the lighting test track and serves the camera for the closed loop (`run_lka_blender_sim`).

## Environment (this machine)
- MATLAB R2026b through the MATLAB MCP (desktop session). No Parallel Computing Toolbox, so MATLAB inference runs on the CPU.
- `VideoReader` / `VideoWriter` MPEG-4 fail headless and have no MPEG-4 profile here: write video with `LkaVideoWriter`.
- Unreal (Automated Driving Toolbox) **cannot load USD scenes on Linux**. Use the Blender bench here; the Unreal bench runs on Windows only.
- Python for the detector: `perception/Mobilenet_Approach/.venv` (3.12, CPU torch, onnxruntime). System Python has none of it.
- Blender 4.2 LTS portable: `~/.local/bin/blender`.
- `startup.m` is a script: functions with nested functions must call it with `evalin('base', ...)`.
- `pkill -f <pattern>` kills your own shell when the pattern is in the command line. Use `pidof`, or a pattern the command line does not contain.

## Conventions
- A behaviour change to the shared pipeline arrives behind a parameter, defaulted off, with a measurement next to it.
- A new detector gets its own approach folder, and its numbers go into `perception/README.md` before it becomes the default.
- Score clips with `tools/clip_metrics.py` and the lighting track with `score_lighting_track`, so numbers stay comparable.
- Keep one annotated video per detector (clip 12.23.28) plus the current test videos. `test_res/**/*.mp4` is git-ignored.
- Tests: `runtests('tests/unit_tests')` must pass 113 of 113. Check `git status models/` afterwards: `test_vision_bench_build` regenerates a `.slx`.
- Bugs found go into `docs/bug_report.md` (fixed or open).
- Commit / push only when asked; work on a branch, not `main`.
