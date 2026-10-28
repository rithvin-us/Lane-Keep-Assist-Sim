# MathWorks products

MATLAB **R2026a or newer** (the models are saved in R2026a; the competition skeleton needs
R2025b or newer). Verified working on R2026b.

Check what is installed with `ver`, or run `lka_check_requirements`.

## Required

| Product | Used by | Without it |
|---|---|---|
| MATLAB | everything | — |
| Simulink | `models/*.slx`, every test bench | no model runs |
| **Automated Driving Toolbox** | `monoCamera`, `birdsEyeView`, `imageToVehicle`/`vehicleToImage` (`src/vision/lka_camera_model.m`), the Unreal co-simulation blocks (`drivingsim3d`) | `Undefined function 'cameraIntrinsics'`; and before it was installed, `build_lka_vision_bench` failed with `There is no block named 'drivingsim3d/Simulation 3D Scene Configuration'` |
| **Computer Vision Toolbox** | `insertShape` (lane rasterisation `src/vision/lka_rasterize_lanes.m`, annotation `lka_annotate_frame.m`) | `Invalid function name '_visionIsObject'` from `insertShape`. Note: installing it into a *running* MATLAB is not enough — its built-ins register at startup, so MATLAB must be restarted |
| **Image Processing Toolbox** | `imboxfilt`, `strel`, `imresize`, `bwlabel` (`lka_paint_probability.m`, `lka_seg_metrics.m`, `lka_search_boundary.m`) | `Undefined function 'imboxfilt'` / `'strel'`; the classic detector and the segmentation metrics stop |
| **Model Predictive Control Toolbox** | `mpcActiveSetSolver` / `mpcActiveSetOptions` in `src/controllers/lka_mpc_curvature.m` | `Simulink is unable to determine sizes and/or types of the outputs for block 'LKA_Controller/Lateral Controllers/MPC Lane Keeping/Curvature MPC (active-set QP)'`. This blocks **every** Simulink run, including the pure-pursuit and Stanley scenarios, because the whole controller model has to compile |
| **Control System Toolbox** | prerequisite of the MPC Toolbox | installs with it |

## Required for the neural detectors

| Product | Used by | Without it |
|---|---|---|
| **Deep Learning Toolbox** | `dlnetwork`, `predict` (`src/vision/lka_onnx_backend.m`) | `cnn` and `yolop` cannot be created |
| **Deep Learning Toolbox Converter for ONNX Model Format** (support package) | `importNetworkFromONNX` (`src/vision/lka_import_onnx_detector.m`) | the ONNX models cannot be imported; `lka_onnx_runtime_available` falls back to the Python `onnxruntime` path, which is CPU-only |

`classic` and `laneassist` need neither, which is why they are the fallbacks.

## Required for the GPU paths

| Product | Used by | Without it |
|---|---|---|
| **Parallel Computing Toolbox** | `gpuArray`, `gpuDeviceCount`, `canUseGPU` (detector batching, `lka_bev_warp`, `lka_bev_odometry`) | `Undefined function 'gpuDeviceCount'`; everything still runs on the CPU, but `V.UseGpu` has no effect and the GPU rows of `results/timing/lka_timing.csv` cannot be reproduced |

## Explicitly not required

**Signal Processing Toolbox.** `src/vision/lka_bev_odometry.m` called `hann(n)` for the
correlation window, which killed the whole visual-odometry path with
`Undefined function 'hann'` on any install without the toolbox, taking four tests and every
`run_lka_on_video` run with it. It is now a local `hannWindow` with identical values. Keep it
that way: a window function is three lines of arithmetic and the competition machine will not
have the toolbox.

## Installing into a running session

Toolboxes installed while MATLAB is open are only partly usable: the search path can be
refreshed, but built-in MEX functions register at startup. If a freshly installed toolbox throws
`Invalid function name '_...'`, restart MATLAB.
