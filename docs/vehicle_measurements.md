# Vehicle measurement sheet (LKA)

Fill in the "Measured" column and send it back, or edit `data/parameters/vehicle_params.m`
(vehicle) and `src/vision/lka_vision_params.m` (camera, profile `baja`) directly. After that:
`run_lka_mil()` and `run_lka_vision(Detectors={'mobilenet'})`.

Priority: **A** the controller uses it directly (steering geometry, limits, camera position);
**B** it changes the simulated vehicle (test bench accuracy); **C** fine tuning, estimate if
hard to measure.

## 1. Geometry and mass

| # | Quantity | Code name | Now | Measured | How to measure | Prio |
|---|---|---|---|---|---|---|
| 1 | Wheelbase | `vehicle.wheelbase` | 1.45 m | | Wheels straight. Tape from the front hub centre to the rear hub centre, left and right side, average. | A |
| 2 | Front track width | `vehicle.trackWidth` | 1.30 m | | Centre of the left tyre contact patch to the centre of the right one. | B |
| 3 | Rear track width | (same field) | 1.30 m | | As above, rear axle. | B |
| 4 | Mass with driver | `vehicle.mass` | 240 kg | | Four scales (or four bathroom scales on boards), one under each wheel, driver seated. Sum. | B |
| 5 | Load on the front / rear axle | gives `vehicle.a` | a = 0.75 m | | From the four scales: a = wheelbase × (rear load / total load). | B |
| 6 | Overall width (outside of the tyres) | `vehicle.tireWidth` + track | 1.30 + 0.20 m | | Tape across the outer tyre walls. The rulebook "wheel on the line" check uses the tyre edges. | A |
| 7 | Tyre size and width | `vehicle.tireWidth` | 0.20 m | | Read the sidewall (for example 23x7-10), measure the tread width. | B |
| 8 | Loaded wheel radius | `vehicle.wheelRadius` | 0.29 m | | Driver seated: ground to the wheel centre. | B |
| 9 | CG height | `vehicle.cgHeight` | 0.40 m | | Tilt test: lift the rear axle ~0.3 m, read the front load change. Or estimate from CAD. | C |
| 10 | Yaw inertia | `vehicle.Iz` | 180 kg·m² | | CAD, or bifilar pendulum. Estimate: about 0.1 × mass × (length² + width²). | C |

## 2. Steering (most important for the LKA)

| # | Quantity | Code name | Now | Measured | How to measure | Prio |
|---|---|---|---|---|---|---|
| 11 | Maximum road-wheel angle, left | `vehicle.maxSteerAngle` | 32° | | Full lock, angle of the front wheel against straight ahead (phone angle app on the rim or a protractor on the floor). Or drive a full-lock circle slowly: angle = atan(wheelbase / radius of the rear-axle circle). | A |
| 12 | Maximum road-wheel angle, right | (same) | 32° | | As above. Report both; the code then uses the smaller one. | A |
| 13 | Steering actuator speed | `vehicle.maxSteerRate` | 60°/s | | Command lock-to-lock with the actuator, film the wheel at 60 fps (or log the steering sensor): angle travelled / time. | A |
| 14 | Actuator delay and lag | `vehicle.steerLagTau` | 0.10 s | | Command a 10° step. Delay = time to the first movement, lag = time from the start of the movement to 63 % of the step. | A |
| 15 | Steering free play (backlash) | (not modelled yet) | – | | Wiggle the wheel by hand with the actuator holding: road-wheel angle of the play. | B |
| 16 | Straight-ahead centre offset | (actuator zero) | – | | Actuator command that drives a straight line (drive 50 m hands-off, adjust). | A |
| 17 | Command scaling | (actuator interface) | – | | Actuator counts / volts per degree of road-wheel angle, measured at 0°, ±10°, ±20°, ±lock. | A |
| 18 | Handwheel-to-road-wheel ratio | (if the actuator turns the column) | – | | Handwheel angle / road-wheel angle at ±10°. | B |

## 3. Tyre / handling (fine tuning)

| # | Quantity | Code name | Now | Measured | How to measure | Prio |
|---|---|---|---|---|---|---|
| 19 | Understeer gradient | from `vehicle.Cf`, `vehicle.Cr` | Cf 22 000, Cr 26 000 N/rad | | Constant-radius test: drive a 15-20 m circle at 3-4 speeds (10-30 km/h), note the road-wheel angle and the speed. The change of steering angle with lateral acceleration (v²/R) gives the understeer gradient; I convert it to Cf/Cr. | C |
| 20 | Tyre-road friction on the track | `vehicle.mu` | 0.8 | | Optional: highest steady circle speed before sliding, μ ≈ v²/(g·R). | C |

## 4. Camera mount

| # | Quantity | Code name | Now | Measured | How to measure | Prio |
|---|---|---|---|---|---|---|
| 21 | Camera model, lens, video mode | `V.Camera.FocalLength`, `PrincipalPoint`, `ImageSize` | 1109 px, 1280x720 | | Model and resolution. Best: calibrate with a checkerboard (MATLAB Camera Calibrator) and send `cameraParams`. ZED 2i: which eye and resolution. | A |
| 22 | Lens distance ahead of the rear axle | `V.Camera.X` | 1.60 m | | Tape along the ground from below the rear-axle centre to below the lens. | A |
| 23 | Lens sideways offset | `V.Camera.Y` | 0 m | | From the vehicle centre line (+ left). | A |
| 24 | Lens height above the road | `V.Camera.Height` | 1.20 m | | Driver seated, tyres at normal pressure. | A |
| 25 | Camera pitch (down) | `V.Camera.PitchDeg` | 8° | | Phone angle app on the camera body, or let the auto-calibration estimate it from a straight-lane video. | A |
| 26 | Camera roll | (assumed 0) | 0° | | Angle app; keep within ±1°. | B |

## 5. Speed and yaw-rate sensors

| # | Quantity | Code name | Now | Measured | How to measure | Prio |
|---|---|---|---|---|---|---|
| 27 | Speed sensor type and resolution | `LKA.EgoSen.SpeedQuant`, `SpeedScaleErr` | model values | | Pulses per wheel revolution and the rolling circumference; check against GPS on a 100 m straight at 20 km/h. | B |
| 28 | Speed update rate and delay | (ego sensor model) | 20 Hz | | From the sensor / firmware. | B |
| 29 | IMU position and orientation | (ego sensor model) | gyro z up, + left | | Where it is mounted and which axis points up. | B |
| 30 | Gyro bias and noise at rest | `YawRateBias`, `YawRateNoiseStd` | model values | | Log the gyro for 60 s with the vehicle standing still: mean = bias, standard deviation = noise. | B |

## 6. Whole-loop timing (Jetson phase)

| # | Quantity | Now | Measured | How to measure | Prio |
|---|---|---|---|---|---|
| 31 | Camera-to-steering delay of the real system | 50 ms (one frame) | | Film the camera view and the front wheel together while a line moves in front of the camera; count frames between the image change and the wheel moving. | A |

Hand-back: this file filled in (copy it, keep the units), plus photos of the measurement set-ups
for items 1, 11, 22-25.
