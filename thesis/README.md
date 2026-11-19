# Analysis

Everything offline lives here. It all reads the recorded runs in `logs/` (not in git) and never writes into them.

| Folder | What | Start with |
|---|---|---|
| [`data_analysis/`](data_analysis/README.md) | The analyses behind the report: one numbered folder per result (01–20) with `analysis.py`, `figures/` and a generated `RESULTS.md`, plus shared code in `common/` | `data_analysis/README.md` |
| [`run_tools/`](run_tools) | Checks for a single run or a set of runs | see below |
| [`earlier/`](earlier) | Earlier analyses that `data_analysis/` replaced, kept because the lab log cites them | `earlier/plotting/README.md`, `earlier/E0_calibrated_kinematics/RESULTS.md` |

## Running the report analyses

```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
cd thesis/data_analysis
python3 -m common.selftest              # must print PASS on every line
python3 05_output_mode_benchmark/analysis.py
```

The run order and the rules (no inherited numbers, `logs/` read-only) are in
[`data_analysis/README.md`](data_analysis/README.md). The first run caches the bags in `common/cache/`,
which takes several minutes.

## Run tools

Usage examples are in each script's docstring.

| Script | Use |
|---|---|
| `plot_paths.py [run]` | Touch target vs tool0 path for a run (default `logs/NEW_RUN`), with lag and jitter metrics |
| `kinematics_check.py <run>` | How well a kinematics file reproduces the controller's own TCP, from a run's bag |
| `rcm_offset_check.py <run>...` | Where the teleop latched the RCM, against where the arm actually pivoted |
| `latch_line_fit.py fit\|report` | Fit the shaft line the controller held and build a corrected tool (V9c latch test) |

## earlier/

| Folder | What |
|---|---|
| `E0_calibrated_kinematics/` | Offline prediction of the calibrated-kinematics RCM error (ruled out kinematics as the cause of the latch offset) |
| `E6_singularity/` | Kinematic simulation and run metrics for the singularity guard |
| `exchange/` | First quick-change repeatability scripts (folder 13 supersedes them) |
| `plotting/` | First cross-condition comparison figures (V7–V9) |
