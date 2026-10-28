# Lane Keeping Assist (LKA) - MATLAB & Simulink
## SAE / aBAJA (Autonomous Baja) Competition

This repository contains the simulation, control algorithms, vehicle dynamics modeling, and verification framework for the **Lane Keeping Assist (LKA)** system developed for the SAE / aBAJA autonomous challenge.

The repository follows **MathWorks Automotive Advisory Board (MAAB)** guidelines and the **ISO 11270** standard for LKAS verification.

---

## 1. Quickstart

1. Launch MATLAB **R2026a** (the models are saved in R2026a; the competition skeleton needs R2025b or newer).
2. Set your MATLAB working directory to the root of this repository (the cloned `LKA---Simulink-/` folder).
3. Initialise paths, parameters and bus objects:
   ```matlab
   run('config/startup.m')
   ```
4. Run the LKA MIL test bench: 5 scenarios × 3 controllers. Figures and the KPI table are written to `results/mil/`:
   ```matlab
   results = run_lka_mil();                                   % everything (~15 min)
   results = run_lka_mil(Scenarios={'S1_Nominal'}, Controllers=1);   % one run
   ```
5. Run the tests:
   ```matlab
   runtests('tests')
   ```
6. Put the LKA into the competition model for RoadRunner co-simulation (see `docs/lka_controller.md` §5):
   ```matlab
   integrate_lka_into_skeleton()
   ```
7. Camera lane detection (rule I.2.3: no ground-truth lane input in the submission).
   The official LKA track runs in Unreal from `data/scenes/LKA/LKA.usd` (no RoadRunner needed):
   ```matlab
   report = calibrate_vision_bench();                   % once: frame map, camera height check
   run_lka_vision(Detectors={'classic','cnn','yolop'}); % camera-in-the-loop, video in results/vision
   run_detector_benchmark();                            % YOLOP vs team UFLDv2 vs classic
   ```
   Detector selection: `LKA.Vision.Detector`; model contract: `models/perception/README.md`.
   The networks run on the GPU through the ONNX importer add-on (onnxruntime CPU fallback).
   Audit of the 2025 Python stack: `docs/adas2026_audit.md`.
   Comparison against the `tsiggi/LaneAssist` reference detector (ported as
   `LKA.Vision.Detector = 'laneassist'`): `docs/laneassist_comparison.md`.
8. Recorded track videos (lane, LKA steering, turning radius, speed, yaw) and the live dashboard:
   ```matlab
   start_lka_dashboard();        % http://127.0.0.1:8050 (live run, GPU, results)
   run_lka_on_test_videos();     % every vids/*.mp4 -> test_res/<name>/ (MP4, PNG, CSV) + summary.csv
   run_lka_pick_video();         % choose one video in a dialog, results open by themselves
   watch_lka_videos();           % process every video copied into vids/ automatically
   ```
   The video tools use the **tight-track profile** by default (2 m college lane, bends down to
   R 3 m): with the official-track settings that lane was fitted almost straight
   (`docs/lka_controller.md` §9.1). For a video of the official track pass `TrackProfile='official'`.
   For a live test on the college track set `lkaLaneWidth = 2.0; lkaTrackProfile = 'tight';`
   before `run('config/startup.m')`. See `docs/lka_controller.md` §9-10.
   Annotated video without MATLAB's VideoWriter (needed on a headless machine, see
   `requirements/system.md`): render the frames, then mux with ffmpeg. Examples from the
   college-track clips are in `test_res/<clip>/lka_annotated_<detector>.mp4`.
   See `docs/lka_controller.md` §9-10.
9. One command to check everything after a change (also from a terminal: `tools/run_lka.bat checks`):
   ```matlab
   run_lka_checks();                 % all unit tests, report in results/checks/
   run_lka_checks(Level='full');     % + closed-loop MIL tests + every video (straight-lane alarm)
   ```
10. When finished:
   ```matlab
   run('config/shutdown.m')
   ```

How the LKA works (lane perception, supervisor, pure pursuit / Stanley 2025 / MPC,
limits, turning radius): **[`docs/lka_controller.md`](docs/lka_controller.md)**.
New to MATLAB or to the project: open **`docs/LKA_Learning_Lab.html`** in a browser (interactive
lane-fit and steering labs, code reader, command builder, quiz, glossary) and
`docs/LKA_Team_Guide.html`.
What the team still has to measure, record, train or install:
**[`docs/team_actions.md`](docs/team_actions.md)**.
The stages after that, with owners and pass/fail gates: **[`docs/next_plan.md`](docs/next_plan.md)**.

---

## 2. Repository Layout

```plaintext
LKA---Simulink-/
├── config/              startup.m (paths, params, buses), shutdown.m
├── data/
│   ├── bus_definitions/ create_lka_buses.m        LaneSensorBus, LaneEstimateBus, LkaInfoBus (+ legacy buses)
│   ├── parameters/      vehicle_params.m          Baja mass, geometry, tyres, steering 32 deg / 60 deg/s / 0.1 s
│   │                    sensor_params.m           emulated detector: 20 Hz, 50 ms, noise, dropouts
│   │                    lka_controller_params.m   builds LKA (lka_build_params.m) + legacy `controller`
│   └── scenarios/       generate_lka_scenarios.m  S1..S5 MIL scenarios; tracks/LKA_IPG.xodr (official track)
│                        generate_test_scenarios.m legacy script scenarios
├── models/              (generated by scripts/build_lka_models.m)
│   ├── LKA_Controller.slx       complete LKA (referenced model)
│   ├── VirtualLaneSensor.slx    emulated lane detector (subsystem reference)
│   └── LKA_MIL_TestBench.slx    closed-loop Baja test bench
├── src/
│   ├── perception/      lane geometry, boundary validation, centre fusion, Kalman tracker, LDW
│   ├── controllers/     pure pursuit, Stanley 2025 port, curvature MPC, steering geometry, limits
│   │                    (+ legacy script controllers stanley_steer_control.m, pure_pursuit_steer.m)
│   ├── plant/           nonlinear single-track Baja model
│   ├── evaluation/      result extraction, KPIs, ground-truth wheel-in-lane metrics
│   └── utilities/       parameter builder, OpenDRIVE reader, road lookup
├── scripts/             build_lka_models, run_lka_mil, plot_lka_results, plot_lka_comparison,
│                        integrate_lka_into_skeleton, run_baseline_simulation (legacy), evaluate_kpis (legacy)
│                        run_lka_on_video, run_lka_on_test_videos, run_lka_pick_video, watch_lka_videos, run_lka_checks
├── tools/               dashboard server, ONNX export / parity tools, run_lka.bat / run_lka.sh (matlab -batch),
│                        finetune/ (college frames + labels → UFLDv2 training data, docs/team_actions.md item 8)
├── tests/               unit_tests/test_lka_core.m, scenario_tests/test_lka_mil_scenarios.m (+ legacy tests)
├── docs/                lka_controller.md (design), mathematical_models.md, references/
├── requirements/        MathWorks products, Python packages, system software (+ lka_check_requirements)
├── results/mil/         figures and lka_mil_kpis.csv from run_lka_mil
├── results/benchmark/   detector comparison: classic vs laneassist vs cnn vs yolop
├── third_party/         licences of the read-only reference checkouts
└── SoftwareVirtualWorldSimulation-master/   aBaja 2026 competition skeleton (aBaja2026.slx + RoadRunner co-sim)
```

---

## 3. Requirements

Full list, with the exact error each missing piece produces: **[`requirements/`](requirements/)**
(`matlab_toolboxes.md`, `python-requirements.txt`, `system.md`). From MATLAB:

```matlab
run('config/startup.m')
lka_check_requirements        % every product, package and file, and what is missing here
```

MATLAB **R2026a or newer**. Required for the MIL bench and the camera chain:

The models and scripts in this repository use:
- **MATLAB & Simulink**, **Stateflow** (LKA mode logic)
- **Automated Driving Toolbox** (`monoCamera`, `birdsEyeView`, Simulation 3D / Unreal blocks)
- **Control System Toolbox** (required by the MPC controller build chain)
- **Model Predictive Control Toolbox** (`mpcActiveSetSolver` in the MPC controller)
- **Computer Vision Toolbox** and **Image Processing Toolbox** (lane detector, annotated video)
- **Deep Learning Toolbox** with the add-on **Deep Learning Toolbox Converter for ONNX Model Format**
  (team CNN and YOLOP; Python onnxruntime is the CPU fallback)
- **Parallel Computing Toolbox** (GPU, optional)
- **RoadRunner** and **RoadRunner Scenario**: only for the competition co-simulation in `aBaja2026.slx`

The LKA's own vehicle model is a MATLAB function (`src/plant`), so Vehicle Dynamics Blockset and
Simscape are not needed for it; the organisers' `aBaja2026.slx` may need further products.

---

## 4. Key Sources and References

1. **Rajamani, Rajesh (2012)**. *Vehicle Dynamics and Control*. 2nd Edition, Springer. *(Used for 2-DOF bicycle model and lateral deviation kinematics).*
2. **Snider, Jarrod M. (2009)**. *Automatic Steering Methods for Autonomous Automobile Path Tracking*. Carnegie Mellon University, CMU-RI-TR-09-08. *(Used for the Stanley and Pure Pursuit steering laws).*
3. **ISO 11270:2014**. *Intelligent transport systems — Lane keeping assistance systems (LKAS) — Performance requirements and test procedures*. *(Used for pass/fail KPI validation).*
4. **MathWorks Automotive Advisory Board (MAAB)**. *Control Algorithm Modeling Guidelines Using MATLAB, Simulink, and Stateflow*.

For complete BibTeX citations, see [`docs/references/references.bib`](docs/references/references.bib).
