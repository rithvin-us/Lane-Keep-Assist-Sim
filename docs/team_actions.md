# What the team still has to do (LKA, status 2026-10-18)

The software side is complete for the virtual round: controller, simulation benches, camera
lane detection (team CNN on the GPU), track-video tool and live dashboard are in `main` and
tested. The items below need **people, measurements, data or installs** that cannot be done
from the code. They are sorted by priority. Each item says why it matters, how to do it, and
what to hand back.

---

## P1 — before the virtual round

### 0. Re-run the college videos with the tight-track fix (new, 2026-10-16)

**Why.** The lane in the annotated college videos stayed almost straight while the painted lane
bends at R 3-8 m: every camera setting was tuned for the official track (`docs/lka_controller.md`
§9.1). The fix is in the code, checked with a Python replica of the camera chain and in Octave, but
it has not run in MATLAB with the team CNN yet.

**How.**
1. `run_lka_checks()`: all unit tests must pass, including the new `test_vision_tight_track`.
2. `run_lka_on_test_videos()` (the tight profile is the default now).
3. Open `test_res/summary.csv`: `StraightAlarm` must be false and `ShapeErrorP90_m` below 0.25 m
   for every clip. Watch each `lka_video_annotated.mp4`: the green and blue lines must sit on the
   painted lines through the bends.
4. Check the official track is unchanged: `run_lka_mil(Scenarios={'S1_Nominal'}, Controllers=1)` and
   `run_lka_vision(Detectors={'cnn'})` must match `docs/lka_controller.md` §6 and §8.

**Give back.** The new `test_res/summary.csv` and the `results/checks/lka_checks_*.md` report.

### 1. Measure the camera that films the track videos

**Why.** A single camera cannot tell a narrow lane seen from low down from a wide lane seen
from high up. The video tool therefore assumes a 1.2 m lens height and a 70° field of view.
Every distance, speed and radius from the college videos is only as good as these two numbers
(today about ±20 %).

**How.**
1. Write down the phone model and the video mode used (resolution, frame rate, which lens: main
   1x, not ultra-wide).
2. Field of view: look up the main-camera spec (for example "26 mm equivalent"), or better,
   calibrate the camera:
   - print a checkerboard (MATLAB: `open checkerboardPattern.pdf`), tape it to a flat board;
   - film or photograph it from 15-20 angles in the same video mode;
   - MATLAB app **Camera Calibrator**, export `cameraParams`, save it as
     `data/camera/phone_cameraParams.mat`.
3. Mount: with the phone in its holder on the vehicle, measure with a tape:
   - lens height above the road [m];
   - distance of the lens ahead of the rear axle [m];
   - sideways offset from the vehicle centre line [m] (+ left).
4. Mount the phone rigidly (no hand-held clips), in landscape, level.

**Give back.** A short text file `vids/camera.txt` with the numbers above (and the `.mat` if
calibrated). Then the videos are re-run with
`run_lka_on_test_videos(HFovDeg=<deg>, CameraHeight=<m>)`.

### 2. Measure the college track lanes

**Why.** The controller and the lane search check that the two detected lines are a
plausible lane-width apart. The official track has 3.5 m lanes; the college lane measured from
the videos is about 1.9-2.1 m, but that number depends on item 1.

**How.** At 5 places (two straights, the start of a bend, the narrowest point of the snake
section, one wide point), measure with a tape:
- the distance between the **inner edges** of the two painted lines [m];
- the width of one painted line [m].
Take one photo of each place with the tape visible.

Also measure the **tightest bend** (the tight-track profile is set for radius 3 m): stretch the
tape straight across the inside line of the bend like a bow string, length `c` [m] (about 3 m),
and measure the largest gap `s` [m] between the tape and the painted line. Radius
`R = c^2 / (8 s) + s / 2`.

**Give back.** The numbers in `vids/track.txt`. Use them as `LaneWidth=<m>` and
`MinRadius=<R>` in the video tool (`run_lka_on_test_videos(LaneWidth=2.0, MinRadius=3)`).
For a live test on that track, set `lkaLaneWidth = <m>;` before `run('config/startup.m')`.

### 3. Record a speed reference together with a video

**Why.** Speed and yaw rate from the videos come from the road texture (visual odometry): about
±20 % on 832x464 WhatsApp clips. One run with a real speed log calibrates the method, and the
tool can then use the log directly.

**How.**
1. Install a GPS logger app that exports CSV with time and speed (1 Hz or faster), or log wheel
   speed from the vehicle if available. The phone gyro (yaw rate) is a bonus.
2. Start the video and the logger, then clap once in front of the camera while pressing a mark
   in the logger (sync point).
3. Drive one lap at 10-30 km/h, staying in the lane centre.
4. Convert to CSV columns `time_s, speed_mps[, yaw_rate_radps]`, with time 0 = the clap.

**Give back.** `vids/<clip>.mp4` and `vids/<clip>_speed.csv`. Run:
`run_lka_on_video('vids/<clip>.mp4', Camera='auto', SpeedLog='vids/<clip>_speed.csv')`.

### 4. Record better videos (and do not send them through WhatsApp)

**Why.** WhatsApp shrinks the video to 832x464 and compresses it heavily; far lines and the
road texture smear, which costs detection range and odometry accuracy.

**How.**
- 1920x1080 (or 1280x720) at 30 fps, main lens, phone mounted as in item 1.
- Start every clip with the vehicle standing still for 2 s on a straight, in the lane centre
  (the camera pitch is calibrated from the first second).
- Share the original files (Google Drive / USB), not through a messenger.
- Record day and dusk, and the snake section slowly.

**Give back.** Files in `vids/`; then `run_lka_on_test_videos()` writes `test_res/`.

### 5. Get the organisers' official track video, if they publish one

**Why.** It is the best preview of the competition camera view.

**How.** Put the file in `vids/` and run `run_lka_on_test_videos()`. For the competition
camera (not a phone), give its field of view and mount (item 1) if known.

### 6. Confirm the Baja vehicle parameters

**Why.** The controller converts the commanded curvature into a road-wheel angle with the
wheelbase, and the test bench simulates the vehicle with mass, CG and tyre data. Our file says
wheelbase 1.45 m and 240 kg; the 2025 CarMaker Baja had 1.341 m and about 347 kg.

**How.** Fill in the sheet **[`vehicle_measurements.md`](vehicle_measurements.md)**
(31 items with the current values and how to measure each one). The most important:
- wheelbase;
- maximum road-wheel angle left and right;
- steering actuator speed, delay and centre offset;
- camera position;
- mass and axle loads.

**Give back.** The values, or edit `data/parameters/vehicle_params.m` directly. Then run
`run_lka_mil()` and `run_lka_vision(Detectors={'cnn'})` and compare with
`docs/lka_controller.md` §6 and §8.

### 7. Watch the result videos and report wrong frames

**Why.** Automatic numbers (lines found 90 %) do not tell whether the found line is the right
one. A person catches wrong lines in seconds.

**How.** `start_lka_dashboard()` → Results → play each video, or open
`test_res/<clip>/lka_video_annotated.mp4`. The green area must be the lane the vehicle is
driving in, green / blue lines on the painted lines, magenta = where the LKA would steer.

**Give back.** `vids/wrong_frames.csv` with the header `video,time_s,what` (for example
`WhatsApp Video 2026-10-16 at 12.23.28,18.0,left line on the kerb`). These frames are also the
first ones to label in item 8 (`extract_frames.py --times` picks them up).

---

## P2 — improves the team CNN on the college track

### 8. Fine-tune the team CNN (UFLDv2) on college-track frames

**Why.** The CNN was trained on TuSimple highway images. On the college snake section
(clip 12.23.28) it takes road edges and kerbs for lane lines (both lines found in only 60 % of
the frames). MATLAB runs the model exactly like Colab (checked to 0.06 px), so the only way to
make it see this track is more training data. Code-side tricks were tested and did not help.

**Two traps in the UFLDv2 code** (both avoided by `tools/finetune/`):
- `scripts/convert_tusimple.py` sorts the lines into the 4 slots by the sign of their slope in
  the image. In a sharp left bend the ego-left line leans like a right-hand line, so it is
  trained as ego-right (and the real ego-right as right-right). `labels_to_ufld.py` takes the
  slot from the label instead (checked: same cache as the UFLDv2 converter on straight lanes,
  correct slots in the bend where the UFLDv2 converter swaps them).
- `train.py --finetune` loads only the ResNet backbone (keys containing `model`); the lane head
  would start from random. `--resume` loads the whole network, but only from a file named
  `epNNN.pth`; step 4 makes one.

**How.**
1. **Frames.** Original-quality videos (item 4), then
   `python tools/finetune/extract_frames.py vids/*.mp4 --out college_frames --times vids/wrong_frames.csv`.
   One frame every 0.5 s at 1280x720; 300-500 frames are 2.5-4 minutes of driving. Include
   the snake section in both directions, straights, shadows, and frames with no line in view.
2. **Label** in CVAT (free, cvat.ai): new task from the `college_frames` images, labels `LL`,
   `L`, `R`, `RR` (polyline). `L` / `R` are the two lines of the lane the vehicle drives in, `LL`
   / `RR` the next lines out (often none on the college track). Draw each line from the bottom
   of the image up to where it is still visible; follow the paint around the bend. Leave a
   frame empty if no line is painted. Tip: use *track* mode, draw every 5th frame and let CVAT
   interpolate, then correct. Export **CVAT for images 1.1** (`annotations.xml`). About 1 hour
   per 100 frames.
3. **Convert** (Colab, after the TuSimple data is set up as for the original training; copy
   `tools/finetune/` to the Drive):
   ```
   !python /content/drive/MyDrive/lka/finetune/labels_to_ufld.py \
       --frames /content/drive/MyDrive/lka/college_frames \
       --labels /content/drive/MyDrive/lka/annotations.xml \
       --root /content/tusimple --preview /content/drive/MyDrive/lka/preview
   ```
   Read the warnings (swapped labels, lines that turn back) and look at a few `preview` images:
   magenta / red / green / cyan dots = LL / L / R / RR. 20 % of the college frames are held back
   (in 5 s blocks) and become the test set, so training keeps the checkpoint that is best on the
   college track; each training frame is listed several times so the college frames are about a
   quarter of each epoch.
4. **Baseline and start checkpoint.** First score the current model on the college test set:
   `!python test.py configs/tusimple_res18.py --data_root /content/tusimple --test_model <model_best.pth> --test_work_dir /content/tmp`.
   Then:
   ```
   import torch, os
   ck = torch.load('<model_best.pth>', map_location='cpu')
   os.makedirs('/content/ft_init', exist_ok=True)
   torch.save({'model': ck['model']}, '/content/ft_init/ep000.pth')
   ```
5. **Train** (1/10 of the original learning rate, 20 epochs; with `--resume` the warm-up is
   skipped, which is fine at this rate; about 30-60 min on a T4):
   ```
   !python train.py configs/tusimple_res18.py --data_root /content/tusimple \
       --resume /content/ft_init/ep000.pth --epoch 21 --learning_rate 0.005 --steps 12 17 \
       --log_path /content/drive/MyDrive/lane_logs/college_ft
   ```
6. **Highway check.** Re-run step 3 with `--eval tusimple`, then `test.py` (as in step 4) with
   the new `model_best.pth`: TuSimple accuracy must stay above about 0.95 (it was 0.96).
7. **Export**: `python tools/export_ufld_onnx.py <new.pth> team_cnn.onnx`. Put the new
   `team_cnn.onnx` on the team Drive and in `models/perception/` (the file is too big for
   GitHub). Delete `models/perception/team_cnn_dlnetwork.mat` so MATLAB re-imports it.
8. **Check in MATLAB**: `runtests('tests/unit_tests/test_ufld_reference_parity.m')`,
   `run_lka_on_test_videos()`, `run_lka_vision(Detectors={'cnn'})` (S1 must stay ≤ 0.05 m).

**Give back.** The new `.pth` and `.onnx` (Drive); the college-test accuracy before (step 4) and
after (step 5) and the TuSimple accuracy after (step 6); the before / after
`test_res/summary.csv`. Keep `annotations.xml`: every new video adds frames to the same set.

---

## P3 — tools and installs (decision by the team lead / lab admin)

### 9. RoadRunner R2026a (competition co-simulation, plan phase 5)

**Why.** The final simulation runs in RoadRunner with the competition skeleton
(`aBaja2026.slx`). The LKA is wired in (`integrate_lka_into_skeleton`) but has never run against
RoadRunner because it is not installed.

**How.** MathWorks account → Downloads → R2026a → install **RoadRunner** and **RoadRunner
Scenario** (licence from the aBaja student offer or campus). In MATLAB: `rrApp = roadrunnerSetup;`
once. Then follow `docs/lka_controller.md` §5. The first run must check the sign of the start
pose and yaw (`LKA.Cosim.YawOffsetDeg`).

### 10. CUDA Toolkit + cuDNN + TensorRT (optional, faster CNN)

**Why.** The CNN now takes about 32 ms per frame on the RTX 4050 through MATLAB. GPU Coder with
TensorRT would bring it to a few milliseconds and is the same path as on the Jetson.

**How.** Install the CUDA Toolkit, cuDNN and TensorRT versions listed for MATLAB R2026a GPU Coder,
set `CUDA_PATH`, `NVIDIA_CUDNN`, `NVIDIA_TENSORRT`, then check with `coder.checkGpuInstall`.

### 11. NVIDIA power setting (optional, one minute)

NVIDIA Control Panel → Manage 3D settings → Program settings → MATLAB → Power management mode:
**Prefer maximum performance**. The laptop GPU then stops dropping its clock between frames.

### 12. Every team member's PC

- MATLAB R2026a with the toolboxes listed in `README.md`, plus the add-on **Deep Learning
  Toolbox Converter for ONNX Model Format**.
- `git pull` the `main` branch; copy `models/perception/team_cnn.onnx` from the team Drive.
- Do not save the Simulink models in an older MATLAB release.

---

## P4 — after the virtual round (Jetson / real vehicle)

### 13. Fix the 2025 Python stack before it drives the real vehicle

`docs/adas2026_audit.md` lists the findings with evidence. Order:
1. **C1 + C2** steering and yaw-damping signs (the 2025 law steers the wrong way);
2. **C3-C5** firmware: strict serial parser, steering watchdog, brake latch; bench test wheels-up;
3. **H1** calibrate the ZED and use a metric bird's-eye view (port of `lka_camera_model.m`);
4. **H2, H3, M5** lane width 3.5 m, per-frame command (no 1 s mean), measured speed;
5. **H4, H6, H7** gyro sign, NaN guards, YOLOP fed RGB;
6. the remaining M and L items.

### 14. Bench and track tests on the real vehicle

Wheels-up test of steering direction and limits first, then low speed on a straight, then the
track at increasing speed, with the LKA able to be switched off instantly by the driver.

---

Questions about any item: the technical details are in `docs/lka_controller.md`, the plain
introduction in `docs/LKA_Team_Guide.html`.
