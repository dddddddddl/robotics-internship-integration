# Offline kinematics

This supplied MATLAB-to-Python model provides homogeneous transforms for the body, camera, lidar, and two arms. The main numerical module is [jibot_kinematics.py](jibot_kinematics.py); [run_jibot.py](run_jibot.py) exposes a command-line interface. NumPy performs the calculations; Matplotlib supports optional visualization.

Run from the repository root after installing `requirements.txt`:

```bash
python offline_kinematics/run_jibot.py --help
python offline_kinematics/run_jibot.py camera-ground
python offline_kinematics/run_jibot.py object-ground --object-xyz-cam -0.16 0.02 0.98
python offline_kinematics/run_jibot.py left-arm
python offline_kinematics/jibot_all_fream.py
```

The last command opens the supplied model visualization. The historical `fream` spelling remains to preserve imports and script identity. `matlab_style_scripts.py` and the small `jibot_*.py` wrappers provide the original demonstration entry points. There are no MATLAB `.m` files in the supplied numerical-model folder; this repository does not claim a cross-language equivalence check.

Model translations are in meters. CLI joint inputs default to degrees; commands accepting `--unit` can use radians. When calling the Python API directly, specify `unit` explicitly: `camera_to_ground` and `object_to_ground` default to radians, while some component helpers default to degrees. Internal frame conventions and model geometry are inherited from the supplied source and have not been recalibrated against physical metrology during preparation.

`measurement_processing.py` contains helpers for fitting a plane and cylinder from metrology workbooks. Chinese workbook column identifiers are intentionally retained for input compatibility. The input measurement workbook was not supplied. The head-processing wrapper now accepts an explicit file path instead of a stale Windows path:

```bash
python offline_kinematics/head1_dataprocess.py /path/to/head_measurements.xlsx --sheet Sheet1
```

The original model README contained unreadable binary data. This guide replaces it; the local original has been retained. Numerical model equations were not changed during preparation.
