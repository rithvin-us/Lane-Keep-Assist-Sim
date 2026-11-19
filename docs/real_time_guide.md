# Real-time guide: Mobilenet lane detector + MATLAB LKA with the ZED 2i

How to run the lane keeping assist live from a ZED 2i stereo camera on the laptop:

```
ZED 2i (USB 3, left eye) ──► lane_node.py (Python, onnxruntime CPU)
                               │  lane probability map + frame, 480 x 288, TCP 127.0.0.1:5555
                               ▼
                             run_lka_live.m (MATLAB)
                               bird's-eye ─► lane fit ─► LKA controller ─► steering command
                               │
                               ▼
                             dashboard  http://127.0.0.1:8050
```

The car is **not** steered by this setup. MATLAB computes the steering the LKA *would*
command (open loop), live and again at the end through the full `LKA_Controller` model.

---

## 0. Measured latency on this laptop (Ryzen 7 7735HS, CPU only)

| Stage | Time per frame |
|---|---|
| Node: crop + resize to 480 x 288 | 1.2 ms |
| Node: MobileNet U-Net (onnxruntime, 6 threads, MATLAB busy alongside) | 32 ms (25 ms with 8 threads on an idle CPU) |
| Node: camera capture → frame sent to MATLAB | 46–51 ms median, 56–60 ms p95 (20 Hz test on a recorded clip) |
| MATLAB: bird's-eye + lane fit + controller | measure on the day; about 15 ms on the recorded clips |
| For comparison: the same network inside MATLAB (`dlnetwork`, CPU) | 111–132 ms: too slow for live use, fine offline |

The LKA loop runs at 20 Hz (50 ms per frame). The dashboard tile **Processing per frame**
turns red above 50 ms. The tile **Camera → command** turns red above 100 ms.

---

## 1. Before you leave (tonight)

1. **Rehearse without the camera.** This runs the whole chain on a recorded clip:
   ```bash
   cd ~/Projects/Lane-Keep-Assist-Sim/perception/Mobilenet_Approach
   .venv/bin/python realtime/lane_node.py --video ~/Downloads/college_lane.mp4 --hfov 70
   ```
   Then in MATLAB:
   ```matlab
   cd ~/Projects/Lane-Keep-Assist-Sim
   start_lka_dashboard();                      % opens http://127.0.0.1:8050
   run_lka_live(CameraHeight=1.2, PitchDeg=8, LaneWidth=2.0, Duration=60);
   ```
   You should see the annotated frame and the steering, lane and latency charts move.
   After 60 s the full controller replay runs and writes `results/live/<date_time>/`.

2. **Get the ZED calibration file.** Without the ZED SDK the USB image is *not*
   undistorted, and the bird's-eye geometry is only as good as the focal length.
   - Find the serial number on the camera label (or run `v4l2-ctl --list-devices`).
   - Download `SN<serial>.conf` from `https://calib.stereolabs.com/?SN=<serial>`. On a
     machine with the ZED SDK it is also in `/usr/local/zed/settings/`.
   - Copy it to `perception/Mobilenet_Approach/realtime/`.
   Without it, pass `--hfov`. The ZED 2i is about 102° unrectified at HD720, but treat that
   as a guess.

3. **Pack:** the ZED 2i, its USB-C cable (USB 3), a tape measure, a phone with an
   inclinometer app, and the laptop charger. CPU inference is slower on battery, so keep the
   laptop plugged in.

---

## 2. Mount and measure (on site, 10 minutes)

These three numbers set the metric bird's-eye view. Wrong values give wrong lane offsets
and steering. Write them in `docs/vehicle_measurements.md`.

| Value | How | `run_lka_live` option |
|---|---|---|
| Lens height above the ground | tape, to the centre of the **left** lens | `CameraHeight` [m] |
| Pitch, nose down | phone inclinometer on the flat top of the camera | `PitchDeg` [deg] |
| Distance ahead of the rear axle | tape, along the ground from the rear axle centre to below the lens | `CameraX` [m] |
| Lane width of the track | tape, inner edge to inner edge of the painted lines | `LaneWidth` [m] |

Mount the camera on the vehicle centre line, level side to side, looking straight ahead.
The left eye is used, so a small offset to the left is fine.

---

## 3. Start (3 terminals)

**Terminal 1: find the camera.**
```bash
v4l2-ctl --list-devices                      # the ZED shows as "ZED 2i": note its /dev/videoN
v4l2-ctl -d /dev/video0 --list-formats-ext   # must list 2560x720: otherwise the port is USB 2
```

**Terminal 1: lane node.** Use `--camera N` for `/dev/videoN`.
```bash
cd ~/Projects/Lane-Keep-Assist-Sim/perception/Mobilenet_Approach
.venv/bin/python realtime/lane_node.py --camera 0 --zed --calib realtime/SN<serial>.conf --show --record runs/live
```
- `--zed` keeps the left half of the side-by-side image.
- `--show` opens a preview window with the network's mask (press `q` to quit).
- `--record runs/live` saves the raw left-eye video and frame times for replay later.
- Every 2 s it prints: frames/s, capture → sent latency, network ms, MATLAB connected or not.

**Terminal 2: dashboard** (or `start_lka_dashboard()` from MATLAB):
```bash
cd ~/Projects/Lane-Keep-Assist-Sim && python3 tools/lka_dashboard.py
```
Open `http://127.0.0.1:8050`.

**MATLAB: the LKA**, with your measured numbers:
```matlab
cd ~/Projects/Lane-Keep-Assist-Sim
out = run_lka_live(CameraHeight=0.95, PitchDeg=10, CameraX=1.2, LaneWidth=2.0);
```
- Stop with **Ctrl+C**. The session is still saved and replayed.
- Add `Speed=3` (m/s) if visual odometry reads nonsense; this happens when the camera
  stands still or the ground has no texture.
- Add `Duration=120` for a fixed-length run.

---

## 4. What to check, in this order

1. **Camera → command latency below 100 ms, frame rate about 20 Hz** (dashboard tiles).
   If it is slower, see §6.
2. **Lines found: L R** on a straight. If only one side is found, check the camera pitch and
   lane width first; a wrong pitch bends the bird's-eye view.
3. **Lane centre offset follows reality.** Stand the vehicle in the middle of the lane: the
   offset should read about 0 m. Move it 0.3 m left: it should read about −0.3 m (the lane
   centre is then right of the vehicle). If the sign or size is wrong, `CameraHeight` or
   `PitchDeg` is wrong.
4. **Steering sign.** On a left bend the command is positive (+ left).
5. **Recording.** Drive or push the vehicle through the bends once with `--record` on.
   This clip is the most valuable thing you bring back (§5).

---

## 5. After the run

- `results/live/<date_time>/`
  - `live_frames.csv`: every frame (lines, live steering preview, node and MATLAB ms)
  - `live_results.csv`: the full `LKA_Controller` replay (steering, turning radius, LKA state)
  - `live_overview.png`: steering (controller vs preview), lane lines, latency vs budget
- `perception/Mobilenet_Approach/runs/live/live_<time>.mp4`: raw left-eye video. Replay it
  offline with exactly the same pipeline:
  ```matlab
  run_lka_on_video('perception/Mobilenet_Approach/runs/live/live_<time>.mp4', ...
      Camera='auto', HFovDeg=102, CameraHeight=0.95, LaneWidth=2.0)
  ```
- Frames from the recording are the next training data. Unseen ZED footage is where the
  model is weakest (see `perception/README.md`). Steps: `training/01_extract_frames.py`,
  label in CVAT, retrain, `deploy/export_onnx.py`.

---

## 6. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `cannot open camera 0` | wrong index (`v4l2-ctl --list-devices`), or another program has the camera |
| node warns `the ZED gives 1344x376` | USB 2 port or cable: use a blue USB 3 port, or the cable that came with the camera |
| network > 40 ms | laptop on battery or MATLAB busy: plug in, try `--threads 8`, close the dashboard tab |
| MATLAB `connecting ... Timeout` | start the node first; check `--port` matches (`run_lka_live(Port=...)`) |
| lines found but offset wrong by a factor | `CameraHeight` / `LaneWidth` wrong, or no `--calib` (focal length guessed) |
| lines bend away in the bird's-eye view | `PitchDeg` wrong: re-measure. On a straight the two lines must be parallel |
| one side never found | the lane is wider than `LaneWidth` × 2: set the real width |
| speed reads 0 or jumps | visual odometry needs road texture and motion: pass `Speed=` |
| `Unknown lane detector` | MATLAB was started before the repository moved: `clear functions; run('config/startup.m')` |

---

## 7. Reference

- Node options: `perception/Mobilenet_Approach/realtime/lane_node.py --help`
- MATLAB options: `help run_lka_live`
- Detector comparison and limits: `perception/README.md`
- Controller design: `docs/lka_controller.md`
