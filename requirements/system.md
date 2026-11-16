# System software, assets and files kept outside git

## GPU

NVIDIA driver with CUDA support, plus the Parallel Computing Toolbox (see
[`matlab_toolboxes.md`](matlab_toolboxes.md)). The development machine is an RTX 4050 with 6 GB,
shared with Unreal: `scripts/run_lka_vision.m` warns below 2.5 GB free, and Unreal aborts with
"error code: 4" when it loses the rest. Close the detector session before a long Unreal run.

Check from MATLAB:

```matlab
canUseGPU, gpuDeviceCount, gpuDevice
```

## ffmpeg

Needed only for the annotated-video path that does not go through MATLAB's `VideoWriter`:

```bash
sudo pacman -S ffmpeg        # Arch / Omarchy
```

Why it exists: `VideoReader` and `VideoWriter` fail inside `matlab -batch` on this Linux install
("Unable to initialize the video properties", "The specified profile is not valid"), because the
GStreamer/EGL stack has no display. From the MATLAB **desktop** both work and
`run_lka_on_test_videos` writes its own MP4s. From a headless batch job, decode and mux with
ffmpeg instead:

```bash
ffmpeg -v error -i clip.mp4 -vf fps=20 -q:v 2 frames/f%05d.jpg      # decode at the LKA rate
ffmpeg -v error -framerate 20 -i annot/a%05d.jpg -c:v libx264 -pix_fmt yuv420p -crf 20 out.mp4
```

## Blender (test-track scenes)

Only for rebuilding `data/scenes/LightingTrack/` (`tools/blender/build_lighting_track.py`).
Blender 4.2 LTS, portable, no root needed:

```bash
mkdir -p ~/.local/opt && cd ~/.local/opt
curl -LO https://download.blender.org/release/Blender4.2/blender-4.2.9-linux-x64.tar.xz
tar xf blender-4.2.9-linux-x64.tar.xz && ln -sf ~/.local/opt/blender-4.2.9-linux-x64/blender ~/.local/bin/blender
```

The generated USD, its texture and the previews are in git, so running the bench needs no
Blender.

## Unreal scene

`data/scenes/LKA/LKA.usd` and its textures are in git (11 MB total). Unreal itself comes with the
Automated Driving Toolbox; no separate engine install is needed for this scene. Unreal writes
`data/scenes/LKA/sim3dSemanticTypes.xml` next to the scene on every run — it is git-ignored.

Known gap: the RoadRunner USD export carries no lane-marking semantic class (ids 2 and 57 only),
so `Precision/Recall/F1/IoU` come out `NaN` on Unreal datasets. Use the synthetic dataset
(`make_synthetic_vision_dataset`, label id 1) when those columns are needed.

## Files deliberately not in git

| Path | What | Where to get it |
|---|---|---|
| `perception/Mobilenet_Approach/.venv/` | Python environment of the lane node | `perception/Mobilenet_Approach/requirements.txt` |
| `ADAS_2026/` | the 2025 Python stack, read-only reference | `RatheeswarS/ADAS_2026` |
| `data/vision_datasets/` | recorded and synthetic frame datasets | regenerate with `run_lka_vision(Dataset=true)` or `make_synthetic_vision_dataset` |
| `results/monitor/` | live dashboard telemetry | written at run time |

Reference checkouts (`ADAS_2026/`, `backup_lane_detection/reference/LaneAssist/`) are
read-only: nothing in the build depends on them, and neither is edited. The archived
detectors are described in `backup_lane_detection/README.md`.
