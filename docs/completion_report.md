# Completion report: Mobilenet lane detector + LKA (2026-11-16)

## Delivered

| Item | Where | State |
|---|---|---|
| Mobilenet Approach as the only active detector; old detectors archived | `perception/`, `backup_lane_detection/` | done, tests 113 of 113 |
| Live camera path (ZED 2i / USB → node → MATLAB → dashboard) | `realtime/lane_node.py`, `run_lka_live`, `docs/real_time_guide.md` | tested with a recorded clip as the camera; not yet with a real ZED |
| Lighting test track: one definition → Blender scene / USD | `lka_lighting_track`, `tools/blender/` | done; Unreal use is Windows only |
| Open-loop scoring on the rendered drive | `score_lighting_track` | done (`perception/README.md`) |
| **Closed loop with Blender camera** (plant + Mobilenet + `LKA_Controller`) | `run_lka_blender_sim`, `LKA_Blender_TestBench` | done, 2 full runs |
| Dashboard: path view, latency, CPU/GPU, closed-loop runs | `tools/dashboard/` | done |
| Algorithm review, MATLAB and Jetson Orin Nano / NX | `docs/algorithm_review.md` | done |
| Bug report (21 fixed / open items) | `docs/bug_report.md` | living document |

## Closed-loop results (lighting track, 0 → 8 km/h at 0.5 m/s², 101 s, nothing scripted)

| Controller | max lateral offset | RMS | in lane | both lines | worst section |
|---|---|---|---|---|---|
| Pure pursuit | 0.49 m | 0.14 m | 100 % | 80.5 % | 30° bend into the S-bend (0.47–0.49 m) |
| MPC | **0.37 m** | **0.13 m** | 100 % | 89.2 % | (see `results/blender/lighting_track_8kmh_mpc/`) |

The 3.0 m lane leaves about 0.75 m per side for a 1.5 m car, so both controllers keep
every wheel in. The 60° R15 bends were not the hard part (0.14 m with pure pursuit). The S-bend
was, because of the lane estimate at the rear axle (bug report O10).

## What is left (priority order)

1. Measure the camera mount on the vehicle (`docs/vehicle_measurements.md`).
2. O10: anchor the lane estimate at the first visible row; this also removes the false LDW warnings (O11).
3. Label glare, tight-bend, simulator and ZED frames; retrain; re-run `run_lka_blender_sim` and `score_lighting_track`.
4. Jetson: TensorRT engine and `trtexec` latency; code generation for the lane fit and controller (`docs/algorithm_review.md` §5).
5. A short `run_lka_blender_sim` regression run in the test suite (telemetry actions are now covered by `test_monitor_publish`).
