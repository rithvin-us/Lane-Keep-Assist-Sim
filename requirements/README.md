# Requirements

What has to be installed before this repository runs, and what exactly stops working when a
piece is missing. Every "without it" line below was observed on this machine while bringing a
bare MATLAB + Simulink install up to a working one, so they are failures we actually hit, not a
list copied from documentation.

| File | Contents |
|---|---|
| [`matlab_toolboxes.md`](matlab_toolboxes.md) | MathWorks products and support packages, with the exact error each missing one produces |
| [`python-requirements.txt`](python-requirements.txt) | Python packages for `tools/` (ONNX export, the reference implementations, the dashboard) |
| [`system.md`](system.md) | Non-MathWorks system software: GPU driver, ffmpeg, Unreal scene assets, the model files kept outside git |

## Quick check

```matlab
run('config/startup.m')
lka_check_requirements        % prints every product and tool, and what is missing
```

`lka_check_requirements` (in `src/utilities/`) is the executable version of these documents: it
reports each product, each support package and each file this repository needs, and names the
feature that is unavailable without it.

## The short version

- **Minimum to run the MIL test bench and the camera chain:** MATLAB, Simulink, Automated
  Driving Toolbox, Computer Vision Toolbox, Image Processing Toolbox, Model Predictive Control
  Toolbox, Control System Toolbox.
- **Additionally for the neural detectors (`cnn`, `yolop`):** Deep Learning Toolbox and the
  Deep Learning Toolbox Converter for ONNX Model Format, plus `models/perception/team_cnn.onnx`
  from the team Drive (it is over 100 MB and is not in git).
- **Additionally for the GPU paths and the GPU timing table:** Parallel Computing Toolbox.
- **Not required:** Signal Processing Toolbox. It used to be, through one `hann` call in
  `src/vision/lka_bev_odometry.m`; that call is now a local `hannWindow`, so the camera path runs
  without the toolbox. Do not reintroduce a dependency on it — the competition machine will not
  have it.
