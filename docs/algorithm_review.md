# Algorithm review: lane detection + LKA, in MATLAB and on the Jetson

Status 2026-11-16. Covers what the code does today, where time goes, what is weak, and
what changes when it moves from this laptop (MATLAB + Python) to the target computer, an
NVIDIA **Jetson Orin Nano / Orin NX**. Numbers marked *measured* were measured in this
repository; numbers marked *estimate* were not and say how they were derived.

---

## 1. The chain, end to end

```
camera frame (RGB)
  │  perception/Mobilenet_Approach            "where is lane paint?"
  ├─► U-Net + MobileNetV2, 480 x 288 input, 3 classes (background, road, lane)
  │     softmax → lane-class probability map   (lka_detector_onnx_seg + mobilenet.json)
  │
  │  src/vision                                "where are the two lane lines, in metres?"
  ├─► bird's-eye warp: probability map → metric top view, 0.05 m/px, 30 m ahead, ±6 m
  │     (lka_camera_model + lka_bev_warp; needs camera height, pitch, focal length)
  ├─► per side: sliding-window search (warm around last frame's fit, cold around the
  │     nominal lane / nearest peak) → robust Huber fit of a clothoid
  │     y(x) = c0 + c1 x + c2 x² + c3 x³ → plausibility (heading, curvature, side, pair
  │     width vs tracked width)                                       (lka_vision_lane_step)
  │   output: LaneSensorBus = left / right {offset, heading, curvature, curvature rate,
  │           strength 0..1, view range}
  │
  │  models/LKA_Controller.slx (generated)     "how should the car steer?"
  ├─► boundary validation (strength, side, jump, width)        lka_validate_boundaries
  ├─► lane-centre fusion, strength-weighted                    lka_estimate_lane_center
  ├─► Kalman tracker of the lane centre [offset, heading, curvature, curvature rate],
  │     predicted with speed and yaw rate (bridges dropouts and the 1-frame latency)
  ├─► supervisor (Stateflow): OFF / STANDBY / ACTIVE / DEGRADED / TAKEOVER
  ├─► lateral controller: pure pursuit (default) | Stanley 2025 | curvature MPC (QP)
  ├─► curvature limiter: |a_y| ≤ 3 m/s², jerk ≤ 5 m/s³ (UNECE R79 B1), steering limits
  └─► road-wheel angle: δ = atan(κ (L + K_us v²)), actuator limits 32°, 60°/s
```

What each stage contributes to an error:

| Stage | Typical failure | Seen where |
|---|---|---|
| Detector | paint missed under glare, broken thin masks far away, wrong paint (other line) | lighting track: glare 56.6 % both lines; WhatsApp clips 75–87 % |
| Bird's-eye | wrong camera height / pitch → wrong metres, lines not parallel | all recorded clips (camera never measured) |
| Search + fit | tight bend fitted too straight, extrapolation back to the axle | 60° R15 bends: line error 0.34–0.44 m at 8 m |
| Validation / tracker | a good line rejected (jump gate) or a bad one accepted | width CV 0.33–0.84 on the WhatsApp clips |
| Controller | late reaction, overshoot, corner cutting | closed-loop runs (`run_lka_blender_sim`, §4) |

## 2. The detector

- **Model:** `smp.Unet(encoder="mobilenet_v2")`, 6.63 M parameters, 14.2 GFLOPs per frame at
  480 x 288. The decoder costs 88 % of that (*measured*, `torch.utils.flop_counter`).
- **Training:** 31 labelled frames from one campus video (+ 8 validation). Lane IoU 0.47,
  road 0.71, background 0.93. Augmentation: shadows, exposure, blur, JPEG.
- **Contract with MATLAB:** `deploy/mobilenet.json` (RGB, /255, ImageNet mean/std, stretch to
  480 x 288, softmax, lane channel 3). ONNX vs PyTorch max |Δ| 1.1e-5 (*measured*).
- **Strengths:** few line swaps (8–12 on the WhatsApp clips vs 21–45 for the CNN), robust to
  shadow (82.8 % both lines in real shadows on the lighting track).
- **Weaknesses:** glare and washed-out asphalt; tight bends; rendered frames (synthetic
  F1 0.60); trained on one place. Its `road` class is unused today.

## 3. Latency

| Where | Runtime | ms / frame |
|---|---|---|
| Laptop, live node | onnxruntime CPU (Ryzen 7 7735HS), 8 threads | 25.5 p50 / 27.9 p95 (*measured*) |
| Laptop, live end to end | node + MATLAB warp, fit, controller | 37.5 + 10.8 median (*measured*) |
| Laptop, inside MATLAB | `dlnetwork` CPU (no Parallel Computing Toolbox) | 103–132 (*measured*); 192 with Blender rendering alongside |
| Jetson Orin Nano 8 GB | TensorRT FP16 | 8–15 (*estimate*: 14.2 GFLOPs at 1–2 effective TFLOPS) |
| Jetson Orin NX 16 GB | TensorRT FP16 | 5–10 (*estimate*: about 1.5–2x the Nano) |

The 20 Hz loop has 50 ms. On the Jetson the network is no longer the bottleneck. The
remaining time goes to camera capture, the bird's-eye warp, and the lane fit; these need to
leave Python and MATLAB (§5).

## 4. How the MATLAB side is tested

| Bench | What it shows | Command |
|---|---|---|
| MIL (emulated sensor) | controller alone, 6 scenarios × 3 controllers | `run_lka_mil` |
| Recorded clips, open loop | detector + lane fit on real video; the steering is what the LKA *would* command | `run_lka_on_test_videos` |
| Blender drive, open loop | the same on rendered video with exact ground truth per condition | README "Lighting test track" |
| **Blender closed loop** | detector + controller + Baja plant together: the camera sees where the controller drove | `run_lka_blender_sim` |
| Unreal closed loop | same idea in Unreal; **Windows only** (USD runtime import) | `run_lka_vision(Scenarios={'S6_LightingTrack'})` |
| Live camera | node + MATLAB on real hardware, preview steering + full replay | `run_lka_live` |

**Open loop vs closed loop.** In a recorded or rendered video the car does not react to the
steering. The magenta arc in the annotated video is the path the commanded curvature would
drive. If the car in the video keeps drifting (the rendered drive weaves ±0.3 m on
purpose), the arc keeps pointing back to the centre. It is not driving anywhere, so the
"error" never closes. Only the closed-loop benches answer "does the LKA stay in the lane".

## 5. Running on the Jetson Orin Nano / NX

### 5.1 What cannot move as is

- **MATLAB does not run on the Jetson** (no aarch64 host). Everything in `src/` and
  `LKA_Controller.slx` reaches the Jetson only as generated code or as a port.
- **The live path on the laptop uses TCP between two processes.** On the Jetson both ends
  should be one process, or a ROS 2 graph with zero-copy transport.

### 5.2 Recommended target architecture

```
ZED 2i ── ZED SDK (rectified left eye, GPU memory, HD720@60) ──┐
                                                              ▼
            TensorRT FP16 engine (mobilenet_unet_288x480)  ── lane prob 288 x 480 (GPU)
                                                              ▼
            bird's-eye LUT remap (CUDA / cv2.cuda.remap, LUT from lka_camera_model)
                                                              ▼
            lane search + fit + validation + tracker + controller
              (MATLAB Coder / Embedded Coder C++ from src/ and LKA_Controller,
               or a C++ port; 20 Hz, CPU A78AE)
                                                              ▼
            steering command ── CAN / PWM to the steering actuator
```

| Step | How | Notes |
|---|---|---|
| Network | `trtexec --onnx=deploy/mobilenet_unet_288x480.onnx --fp16 --saveEngine=mobilenet_fp16.engine` | then measure: `trtexec --loadEngine=… --iterations=500`; INT8 needs a calibration set (100–500 college frames) |
| Camera | ZED SDK on JetPack 6 | rectified images (no `--calib` step), timestamps, IMU for yaw rate |
| Pre-processing | resize + normalise on the GPU | CPU resize of 1280x720 costs 1–3 ms on an A78 |
| Bird's-eye | precomputed LUT (`cam.Sampler`) + remap | the LUT depends only on the camera mount: compute once |
| Lane fit + controller | MATLAB Coder / Embedded Coder | `lka_vision_lane_step`, `lka_fit_boundary`, `lka_search_boundary` and the controller MATLAB Function blocks are plain MATLAB; `bwlabel` and `mpcActiveSetSolver` support code generation. Check each with `coder.screener` first |
| Supervisor + limits | Embedded Coder of `LKA_Controller.slx` | Stateflow and MATLAB Function blocks generate C |

Power: benchmark in the mode you will race in (`sudo nvpmodel -q`, then `-m 0`;
`sudo jetson_clocks` for repeatable numbers) and log `tegrastats` alongside.

### 5.3 Jetson risks

| Risk | Why | Mitigation |
|---|---|---|
| Codegen gaps | some vision helpers use `monoCamera` / `birdsEyeView` objects | build the LUT offline, generate code only for the per-frame math |
| Thermal throttling in the buggy | small heatsink, sun | fan, measure `tegrastats` during a 10-minute run |
| Unified memory contention | CPU and GPU share LPDDR5 | keep frames on the GPU; avoid host copies of the 1280x720 image |
| Different numerics | FP16 / INT8 | compare engine output against `deploy/export_onnx.py` output on 50 college frames before driving |

## 6. Verdict

The chain works end to end. On real campus clips it finds both lines in 75–87 % of the
frames (not seen in training) and swaps lines least of all detectors tried. Its limits are
the data, not the architecture: glare, tight bends and simulator frames. Before the event:
1. Measure the camera mount (drives every metric number).
2. Label glare, tight-bend and ZED footage, then retrain.
3. Build the TensorRT engine and measure it on the Jetson.
4. Generate code for the lane fit and the controller, then run the closed loop on the vehicle.
