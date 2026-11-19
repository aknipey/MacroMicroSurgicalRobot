# Data analysis for the thesis report

One folder per result. Each folder starts as an `ANALYSIS.md` (question, data, extraction, method, pseudocode,
statistics, figures) and eventually holds its own `analysis.py`, `figures/` and a short `RESULTS.md` with the
numbers the report quotes. Written 2026-11-13, after the Chapter 4 first draft.

## Ground rules

1. **`logs/` is read-only.** Nothing here writes into a run folder, renames, moves or "fixes" a raw file. Every
   output lands in this tree (`<folder>/figures/`, `<folder>/RESULTS.md`, `<folder>/cache/`).
2. **No inherited numbers.** Numbers in `docs/VERSIONS.md`, `REPLAYS_*.md`, the planning files, METRICS.yaml,
   NDI_META.yaml and the Chapter 4 draft are **claims to test, never inputs**. Each analysis recomputes from the
   rawest file available (bag messages and the per-sample CSVs) and lists, in its RESULTS.md, every prior claim it
   confirms or contradicts. Where a recorder-derived summary (e.g. `NDI_META.yaml error_rms_mm`) is used as a
   shortcut, it is first reproduced from `NDI_PATH.csv` on at least one run.
3. **Exclusions are re-derived, not copied.** A run is excluded only by a check that this code runs and prints
   (gap in a stream, camera moved, wrong kinematics), with the reason in RESULTS.md.
4. **Every number in the report traces to one script and one line of its output.** `RESULTS.md` lists them.

## Where numbers come from (audit, 2026-11-13)

Every number in a `RESULTS.md` is written by that folder's `analysis.py` from `logs/` (or from `config/` files
it names in the output). Nothing is typed in by hand. The only constants in the code are **method parameters**
(choices, stated in each RESULTS.md), never measured values:

| Parameter | Value | Folder |
|---|---|---|
| Gap that excludes a run (inside a stroke) | 50 ms | 05 |
| Camera/wand-moved: persistent step between 4 s medians of the FK-vs-NDI residual | 3 mm | 09 |
| Exchange: "left / back at the seat" | 1 mm; settling window 0.5 s; bump-test trim 1 s | 13 |
| Vibration: Welch segment 1 s, bands 8–12 / 15–25 / 30–60 Hz, 3 Hz motion removal | – | 01 |
| Sample grid | 500 Hz (= the measured /joint_states rate) | common, 01 |

Values that *look* like data but are read from files: tool lever arms (`config/tools.yaml`, 13), kinematics
(`config/calibrations/…`, ur_description), Servo/guard thresholds (META.yaml, 07). Two inherited values were
found and removed on 2026-11-13: hand-typed lever arms (325 / 436.3 mm; the 325 mm was the wrong holder tool for
the swap dates) and an assumed 12 ms NDI delay (now fitted per run).

## Facts about the raw data (checked 2026-11-13, re-check in `common/` on load)

| Source | What it holds | Notes |
|---|---|---|
| `UR_PATH.csv` | `t, stamp, source, frame, x, y, z, qx, qy, qz, qw` (m, base_link) | `t` = host receive time since recording start, the **same clock for every file in a run** (META notes). `source` rtde = pendant TCP moved back to tool0 by `tcp_offset`; `tf` = fallback |
| `TOUCH_PATH.csv` | teleop target for tool0 in base_link | **only while WHITE is held** (gaps = clutch released) |
| `STYLUS_RAW.csv` | raw Touch pose (mm, device frame), `vx vy vz`, `white`, `grey` | ~1 kHz publish, real updates every ~4 ms (repeated poses in between) |
| `TRAJECTORY.csv/.yaml` | joint positions + ee pose of the recorded input | `.yaml` for trajectory-player runs (PoseOffset, SingularityInput) |
| `META.yaml` | code version, kinematics model + hash, tool, `tcp_offset.check_max_mm`, **every teleop setting** (`TOUCH_COMMAND_FILTER`, gains, scale...) | the per-run settings make the version history a natural experiment |
| `REPLAY.yaml` | which original a replay reproduces | |
| `NDI_PATH.csv` | 60 Hz: `tip_*`, `axis_*`, marker origin `x y z`, quaternion `q0..qz`, `fit_error`, `rcm_*` | camera frame, mm |
| `rosbag/` (mcap) | `/joint_states` 500 Hz (header stamps 2.0 ms, no duplicates) with **position, velocity, effort = motor current (A)**; `/servo_node/delta_twist_cmds`; teleop target; `/tcp_pose_broadcaster/pose`; mode-specific command topics; `/phantom/state`, `/phantom/button`; condition number and RCM topics from V7 on | read with `rosbag2_py` (`source /opt/ros/jazzy/setup.bash`); the `mcap` pip package is not installed. **Joint order in `/joint_states` is not the URDF order** (elbow first): always index by name. Topic prefix `ur5e_touch_teleop` up to V7, `ur16e_touch_teleop` after the rename |

Command topic by output mode: trajectory → `/scaled_joint_trajectory_controller/joint_trajectory`; velocity →
`/forward_velocity_controller/commands` (+ `/joint_velocity_limiter/commands` from V5); position →
`/joint_velocity_limiter/commands` (V1: `/forward_position_controller/commands`).

**Bag vs CSV time span (found by `common/selftest.py`):** the rosbag starts a few seconds after the CSV
recorder (3.8 s on V9b/NoTool Position LongBenchmark R1). Bag-based analyses use only the overlap; the CSV
`t` clock is the reference, and `common.bags.to_run_clock` maps bag times onto it (checked: FK of
`/joint_states` reproduces `UR_PATH.csv` to < 0.5 mm on the overlap). `/tcp_pose_broadcaster/pose` is in the
controller's `base` frame (base_link rotated by π about z), not base_link.

## Shared code (`common/`, to write first)

- `bags.py`: `read_topics(run, topics) -> {topic: (t_header, t_receive, array)}`; joint reorder by name;
  cache to `common/cache/<run-hash>.npz` keyed on the bag's mtime and size (bags are big; read once).
- `clock.py`: map bag time to the CSV `t` clock through the `/tcp_pose_broadcaster/pose` messages, which are the
  same RTDE packets as `UR_PATH.csv` rows with `source = rtde` (match by stamp, fit offset, report residual).
- `runs.py`: run discovery and a run table (version, mode, trajectory, replay n, scale, filter cutoff,
  kinematics, camera day) built **from META/REPLAY files**, never from folder names alone.
- `style.py`: one colour + marker per condition, shared by every figure (the categorical slots of
  `thesis/earlier/plotting/common.py`; run the dataviz palette validator once on the final set), figure widths
  for the report (full 15 cm, half 7.3 cm), vector PDF + PNG preview.
- `checks.py`: stream gaps (> 3× median dt), NDI visibility, `tcp_offset.check_max_mm` > 5 mm (wrong
  kinematics), camera-moved test (robot-vs-NDI residual step), each returning a reason string.

## Results

| # | Folder | Report section | Claim it tests |
|---|---|---|---|
| 01 | [01_vibration_threshold_i](01_vibration_threshold_i/ANALYSIS.md) | 4.1.3–4.1.4, scorecard (i) | ≥ 30 % peak-band vibration reduction vs an unfiltered baseline |
| 02 | [02_closed_loop_latency_model](02_closed_loop_latency_model/ANALYSIS.md) | 4.1.3, 4.5.1 | The filter costs far less lag inside the loop than its open-loop group delay; what would meet 50 ms |
| 03 | [03_buzz_large_vibrations](03_buzz_large_vibrations/ANALYSIS.md) | 4.1.4 (residuals) | Where and in which modes the > 80 Hz buzz appears |
| 04 | [04_displacement_delivery_ratio](04_displacement_delivery_ratio/ANALYSIS.md) | 4.1.1 | Trajectory route delivers ~60 % of Servo's commanded displacement; position route does not |
| 05 | [05_output_mode_benchmark](05_output_mode_benchmark/ANALYSIS.md) | 4.1.4, T4.1, F5.5 | Same-day lag / tracking / smoothness of the output modes, paired by trajectory |
| 06 | [06_lag_by_version](06_lag_by_version/ANALYSIS.md) | 4.1 opener, F5.1 | The V1 → V9b chain of diagnoses in one figure |
| 07 | [07_singularity_guard](07_singularity_guard/ANALYSIS.md) | 4.2.1, F6.1 | Guard on returns, guard off stays stuck |
| 08 | [08_depth_invariant_rcm_law](08_depth_invariant_rcm_law/ANALYSIS.md) | 4.3.1, F7.2 | The law keeps flange motion per mm of stylus constant across depth |
| 09 | [09_rcm_ndi_validation](09_rcm_ndi_validation/ANALYSIS.md) | 4.3.2, T4.2 | Independently measured RCM error by trajectory, mode, day |
| 10 | [10_rcm_error_budget](10_rcm_error_budget/ANALYSIS.md) | 4.3.3, F7.6 | Controller vs kinematics vs measurement chain; variance decomposition |
| 11 | [11_pose_offset_camera_test](11_pose_offset_camera_test/ANALYSIS.md) | 4.3.3 | Does the latch-pose offset follow the camera (chain) or the pose (arm)? |
| 12 | [12_replay_repeatability](12_replay_repeatability/ANALYSIS.md) | 3.4 | How identical replays of the same input are |
| 13 | [13_exchange_repeatability](13_exchange_repeatability/ANALYSIS.md) | 4.4.1, scorecard (ii) | Re-seat repeatability and exchange time, per coupling version (V1, V2, V3) |
| 14 | [14_early_versions_tracking](14_early_versions_tracking/analysis.py) | 4.1.1 | V1–V3 lag and error with the folder-05 method; emulator lookahead / pose-source runs |
| 15 | [15_velocity_mode_faults](15_velocity_mode_faults/analysis.py) | 4.1.2, 4.1.3 | Velocity-output vibration, faults (frozen /joint_states), chatter; what the 500 Hz chain + limiter changed |
| 16 | [16_stylus_and_filters_offline](16_stylus_and_filters_offline/analysis.py) | 4.1.2, 4.1.3 | Stylus update steps and tremor; Kalman, One Euro and Butterworth offline with the node's code |
| 17 | [17_touch_stick_slip](17_touch_stick_slip/analysis.py) | 4.1.4 | Slow-speed catches vs the Touch encoders |
| 18 | [18_misc_backing](18_misc_backing/analysis.py) | 4.2.1, 4.4.2 | Singularity amplification, demo counts, micro-tool calibration |
| 19 | [19_benchmark_inputs](19_benchmark_inputs/analysis.py) | 3.4.4, T3.7 | What each recorded input asks of the arm (target speed, extent), vs laparoscopic tip speeds |
| 20 | [20_v9c_mode_filter_grid](20_v9c_mode_filter_grid/) | 4.1.3–4.1.4, Fig 4.6 | Session D output mode × filter grid |
| 21 | [21_latch_line_correction](21_latch_line_correction/ANALYSIS.md) | 4.3.1 (T4.2 session D), 4.3.4, Fig 4.11, scorecard (iii) | Cause of the latch offset (tool line from the hand–eye fit) and the measured correction (latch test) |

Folders 14–18 were added on 2026-11-13 to back claims in the draft that no RESULTS.md covered; 08 gained a
scripted-input section (alternative laws). They have no ANALYSIS.md: the method is in each `analysis.py` docstring.

**Running it:** `source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && cd thesis/data_analysis`,
then `python3 -m common.selftest` (must print PASS on every line), then `python3 <folder>/analysis.py` in this order:
12 → 05 → 01 → 04 → 09 → 10 (needs 09's cache) → 11 → 07 → 08 → 13 → 06 → 03 → 02 → 14 → 15 → 16 → 17 → 18. Bags are cached in
`common/cache/` on first read (several minutes); later runs are fast. Each folder's `RESULTS.md` is regenerated
on every run, so it can never drift from the code.

**Order of work:** `common/` → 12 (it sets the noise floor every other comparison is judged against) → 01, 05 →
04, 02 → 09, 10, 11 → 07, 08, 13 → 06, 03.

## Figure intent (all folders)

Every figure answers one question that is written as its title in the ANALYSIS.md. Thresholds and noise floors
are drawn **on** the figure (a 1 mm line, a 30 % line, a tracker-floor band), so the reader can see the verdict
without the text. Replays are shown as individual marks, never only as a mean. One colour per condition across
the whole report; markers carry identity for print and colour-blind readers. No dual y-axes.
