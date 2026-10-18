# Next plan (from 2026-10-18)

Where we are: the controller, simulation benches, camera lane detection, video tools, the
tight-track fix for the college videos and the CNN fine-tune kit are on branch
`ccr-586dff79-exyk2l`. What is missing is **proof in MATLAB, real measurements, training data
and RoadRunner**. Each stage below has an owner, a size and a gate (the check that says it is
done). Item numbers refer to [`team_actions.md`](team_actions.md).

```
Stage 1 Prove fix in MATLAB ──► Stage 3 Measure ──► Stage 4 Fine-tune CNN ──► Stage 5 RoadRunner ──► Stage 6 Virtual round
Stage 2 Snake track in simulation (starts now, in parallel) ─────────────┘                                   │
                                                                                         Stage 7 Real vehicle ◄┘
```

---

## Stage 1: prove the tight-track fix in MATLAB (team, half a day, now)

The fix was checked with a Python copy of the camera chain and in Octave, never in MATLAB with
the team CNN.

- Do: item 0 (`run_lka_checks()`, `run_lka_on_test_videos()`, the S1 check).
- Gate: all tests pass; `StraightAlarm` false and `ShapeErrorP90_m` < 0.25 m on every clip in
  `test_res/summary.csv`; S1 still ≤ 0.05 m.
- If a gate fails: send `results/checks/lka_checks_*.md` and `summary.csv`; the fix comes back
  the same day.

## Stage 2: drive the snake track in closed-loop simulation (Claude, 1-2 days, starts now)

Detecting a bend is half the job: the vehicle must also be able to drive it. Numbers from the
current parameters (`lka_build_params.m`, `vehicle_params.m`):

| Limit | Value | Top speed on an R 3 m bend | On the R 4.6 m S-bend of the tests |
|---|---|---|---|
| Steering angle 32°, wheelbase 1.45 m | tightest circle R 2.3 m (2.6 m with the 5 % margin) | OK | OK |
| Lateral acceleration `Lim.AyMax` 3 m/s² (road-car comfort rule UNECE R79) | v ≤ √(3·R) | **10.8 km/h** | 13.3 km/h |
| Lateral jerk `Lim.JerkMax` 5 m/s³ (same rule) | limits how fast the curvature can change | (sudden bend start: corner cut) | **12.7 km/h** |
| Steering rate 60°/s | | OK | OK up to 23 km/h |
| Lane 2.0 m, vehicle 1.5 m over the tyres | **0.25 m** to each line | | |
| Controller 2 (Stanley 2025) steering limit 12° | cannot drive below R 6.8 m | **not usable** | not usable |

So on the snake section the LKA, as set today, holds about **10 km/h** with 0.25 m to spare.

- Do: add scenario `S6_CollegeSnake` (2 m lane, R 3-8 m S-bends, Baja plant, tight-track
  settings) to `data/scenarios/generate_lka_scenarios.m`; run pure pursuit and MPC; find the
  highest speed that keeps the tyres inside the lines; if needed, a `'tight'` controller
  setting (shorter look-ahead, low-speed a_y / jerk limits) in `lka_set_track_profile`.
- Gate: no tyre on the line at the chosen speed; official-track scenarios S1-S5 unchanged.
- Uses today's guesses for the bend radius and steering limits; Stage 3 replaces them.

## Stage 3: measurements (team, 1-2 days, can run alongside Stage 1)

| What | Item | Unlocks |
|---|---|---|
| Camera: phone, video mode, field of view or checkerboard, lens height, position | 1 | metric distances ±5 % instead of ±20 % |
| Track: lane width at 5 places, line width, **tightest bend radius** | 2 | `LaneWidth`, `MinRadius`; Stage 2 speed |
| Speed log with clap sync | 3 | speed / yaw calibration |
| Original 1080p / 720p videos, 2 s standing still at the start, snake both ways | 4 | detection range; training frames |
| Vehicle sheet, at least wheelbase, max wheel angle, steering speed and delay | 6 | Stage 2 numbers; real-vehicle tuning |
| `vids/wrong_frames.csv` | 7 | first frames to label |

- Gate: `vids/camera.txt`, `vids/track.txt`, the vehicle sheet filled, new videos in `vids/`;
  Claude re-runs the videos and Stage 2 with them.

## Stage 4: fine-tune the team CNN on the college track (team labels, about 1 week)

Today the CNN finds both lines in only 60 % of the snake-section frames (clip 12.23.28).

- Do: item 8 (`tools/finetune/`): extract 300-500 frames, label in CVAT (split across 4 people
  this is about 1-2 hours each), convert, train in Colab, export.
- Claude: checks the converter warnings and previews, compares before / after.
- Optional (Claude, 1 day, say if wanted): draft labels drawn automatically from the paint
  tracer, loaded into CVAT so people only correct them; roughly halves the labelling time.
- Gate: both lines found in ≥ 90 % of the 12.23.28 frames; college test accuracy up; TuSimple
  accuracy ≥ 0.95; `test_ufld_reference_parity` passes; S1 ≤ 0.05 m.

## Stage 5: RoadRunner co-simulation (team installs, then Claude; when the licence is active)

- Team: check the activation key; install RoadRunner R2026a (item 9).
- Claude: plan phase 5 in [`superpowers/plans/2026-10-16-lka-vision-integration.md`](superpowers/plans/2026-10-16-lka-vision-integration.md):
  camera position on the competition car (U4) and yaw-rate sign (U10).
- Gate: official track driven in RoadRunner with the CNN in the loop; lateral error within
  the MIL result ± 0.05 m.

## Stage 6: virtual round (team lead)

- Freeze the parameters (git tag), run `run_lka_checks('full')`, record the result videos.
- Needed from the team lead: round date, submission format, rulebook speed and scoring.

## Stage 7: real vehicle (after the virtual round)

- Items 13-14: fix the 2025 Jetson stack in the order of the audit (C1-C2 steering signs and
  C3-C5 firmware watchdog first), wheels-up bench test, then walking speed on the track, then
  up to the Stage 2 speed.
- Gate: one full lap of the college track hands-off at the Stage 2 speed, with a person at the
  kill switch.

---

## Risks

| Risk | Effect | What we do |
|---|---|---|
| Tightest bend below R 3 m, or real wheel angle < 32° | vehicle cannot follow at any speed | Stage 3 measures it first; Stage 2 sets the speed |
| Only 0.25 m between tyre and line | small errors become line touches | Stage 2 speed; camera calibration (Stage 3) |
| RoadRunner key inactive | no co-simulation for the virtual round | ask the organisers or lab admin now; Stages 1-4 do not depend on it |
| Labelling takes longer than planned | CNN stays weak on the snake track | split the work, draft labels (Stage 4 option); the classic detector stays as fallback |
| Phone camera ≠ competition camera | tuning on videos does not carry over | ask the organisers for their camera data (item 5) |

## What Claude does without waiting for anyone

1. Stage 2 (snake-track closed-loop simulation and speed limit).
2. Stage 4 option (draft labels), if the team wants it.
3. Same-day fixes for anything Stage 1 turns up.
