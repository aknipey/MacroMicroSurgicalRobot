# Teleoperation Stack Versions

Each version is the state of the Touch -> UR16e teleoperation code for a set of
recorded runs. Every version has a git tag, so its code can be checked out or
browsed exactly:

```bash
git show v2 --stat            # what the version is
git diff v1 v2 -- src/ ls_thesis/ macro_micro_robot/ launch/   # what changed between two versions (ls_thesis/ before 2026-11-02)
git worktree add /tmp/v2 v2   # a copy of the code as it ran
```

Run data lives in `logs/V<n>/` (git-ignored, on the lab PC). Each run's
`META.yaml` holds every `teleop_settings.py` value it ran with and its
`code_version`.

| Version | Tag | Date | Change | Why | Lag (pos / angle) |
|---------|-----|------|--------|-----|-------------------|
| V1 | `v1`, `v1-90deg` | 2026-10-12 | Baseline: Servo positions -> `forward_position_controller` | First working Touch -> UR16e teleop, to measure against | 640-760 ms / > 1.5 s |
| V2 | `v2` | 2026-10-12 | Servo `JointTrajectory` -> scaled trajectory controller | V1 lagged badly: Servo re-based every step on the lagging measured state, so the arm ran at ~10% of the commanded speed | - / 128-158 ms (302 ms twist) |
| V3 | `v3` | 2026-10-12 | Selectable motion scaling (1:1 / 1:5 / 1:20) | Finer control: hand motion scaled down 5× or 20× | ~140 ms / ~140 ms |
| V4 | `v4` | 2026-10-14 | Velocity output option, UR emulator, latency study | V3's ~140 ms lag is mostly trajectory output delivering only 60% of the commanded motion | sim: ~30 ms / ~34 ms (velocity mode) |
| V5 | `v5` | 2026-10-15 | `TOUCH_COMMAND_FILTER` low-pass on the command + `TOUCH_KALMAN` filter on the stylus + 500 Hz `joint_states` robot pose + 500 Hz command chain + `joint_velocity_limiter` | Velocity mode vibrated on the real arm: Servo does not smooth its velocity output, so stylus tremor/jitter (amplified by the feedforward), the Touch's 4 ms update steps and the 20 Hz TF pose all reached the joints | 58 ms / 72 ms (real arm, by hand, 500 Hz + limiter, no faults); earlier V5 tests and the V4 velocity benchmarks faulted the robot (wrist current chatter) |
| V6 | `v6` | 2026-10-19 | `SingularityGuard` on the Touch twist (damped least squares + block motion into the singularity + back-off), Servo's own singularity check moved out of the way while it runs; RCM mode: hold WHITE + click GREY latches the tool tip as a pivot point, the stylus then drives the tip with the shaft held through the point (`RcmTool`, `TOUCH_RCM_*`) | Strokes hit singularities: Servo's check scales the whole twist to zero and, past its hard stop, halts every command (even one leading out), so the arm got stuck. RCM: laparoscopic-style work, the shaft has to pivot about a port | Mock arm: shaft ≤ 1.35 mm off the point (mean 0.39) over 21° pivots. Real arm RCM runs in `logs/V6/` (BasicInRCM, ScalingInRCM, ExitingRCM): near the point the arm has to accelerate hard for small tip moves |
| V7 | `v7` | 2026-10-19 – 10-26 | RCM sideways scaling: a sideways stylus move moves tool0 by the same amount at every depth (pivot slope k / r(d), `TOUCH_RCM_TOOL0_GAIN` = 0.5, to 130 mm), replacing V6's (depth / 10 mm)² fade; pivot continuous through 0 mm; tip can retract 5 mm past the point; RCM tool = NDI wand (hand-eye calibration). **Part 2:** haptic cues on the Touch (`TOUCH_HAPTIC_*`) through a new driver (`touch_server` + `touch_bridge`) replacing `omni_state`. **Tooling:** NDI Vega tracking of every run, recording refused without the tracker, replays with RCM chords / `--rcm`, UR16e hard-coded. Overview: section *V7 — summary of what changed from V6* | V6 real arm runs: near the RCM point the arm moved r(d) / d times the tip, so it accelerated hugely for small moves; fades tried first swung the arm when moving in and out. Part 2: feel limits and mode changes instead of watching for them. NDI: measure the RCM error independently of the robot's kinematics | Offline: tool0 0.50 mm per mm of stylus at 0-130 mm. Real arm, 14 NDI replays (2026-10-26): NDI RCM error 0.4-2.9 mm rms, teleop's own 0.05-0.23 mm (gap = UR16e factory kinematics, `logs/V7/REPLAYS_2026-10-26.md`). Haptics: OpenHaptics renders no force in a ROS process (fixed by the new driver); this Touch's force output is faulty |
| V8 | `v8` | 2026-10-27 – 10-30 | GREY-hold macro/micro switch; numbered tool library switched from the launch terminal; smoother `speedj` on the robot (`SERVO_SMOOTH_SPEEDJ`); micro module made to work (startup deadlock, lost handshake, rad/deg motor maths, stylus position control, responsive damping); tool tip from one NDI ball, micro module = tool 2. Overview: section *V8 — summary of what changed from V7* | One stylus for arm and micro; tools change with the quick-change coupling; the arm buzzed / ground on accelerations; the Touch never moved the micro module | Sim / fakes: micro tip step settles in 69 ms (was 1.1 s), tracks within ~0.1 mm; speedj script loads (real-arm effect not yet measured). Real: tool switching; micro tip at (−8.7, 17.0, 436.0) mm ±1.5 mm |
| (after V8) | `1b9b502` | 2026-10-30 | Rotation scaled the same as translation at every motion scale (`TOUCH_SCALE_ROTATION_EXPONENT` 0.5 → 1: 1:5 / 1:20 for both) instead of the square root (1:2.24 / 1:4.47) | User's choice: one scale for the whole motion | Test node: double-clicks cycle 1:1 / 1:5 / 1:20 with rotation 1:1 / 1:5 / 1:20 |
| (after V8) | `1b9b502` | 2026-10-30 | NDI tracker no longer required to record: `LOG_REQUIRE_NDI` True → False (GREY click / replay record without it, with a warning and `ndi.tracker_at_start: false`) | User's choice: record runs without the Vega | - |
| (after V8) | `1b9b502` | 2026-10-30 | `launch_system.sh`: each command in its own process group, shutdown stops the whole group (SIGINT → SIGTERM 15 s → SIGKILL 5 s); stops orphaned `ros2_control_node`s on the same ROS domain at start-up and refuses while a live session's driver runs | Orphaned `ros2_control_node`s from earlier sessions published stale `/joint_states` (poses 1-1.5 h old) into the live one | Dummy tree ignoring SIGINT: removed at 15 s; fake driver on the same domain refused, other domain allowed |
| (after V8) | `1b9b502` | 2026-10-30 | `SERVO_OUTPUT = "position"` (new, now the default): Servo's joint velocities go through `joint_velocity_limiter`, which integrates them into its own joint positions and sends them `SERVO_POSITION_LEAD` (30 ms) ahead to `forward_position_controller`, so the UR runs `servoj` instead of `speedj` | The buzz / grind is `speedj`-only and pose-dependent (`LargeVibrations` 1.81 A vs `NEW_RUN` 0.22 A, same session, settings and command roughness); `servoj` never buzzed. Trajectory mode's ~140 ms lag came from Servo's re-basing, not `servoj` | Sim (emulator, `V3/CyclingScaling`): 58 / 62 ms, 4.9 mm (velocity 62 / 66 ms, 5.3 mm). Real arm: not yet run |
| (after V8) | `9f4f68e` | 2026-11-02 | Renames only, no behaviour change: package `ls_thesis` → `macro_micro_robot` (Python folder, `include/` folder, every `ros2 launch/run`), `ur5e_*` → `ur16e_*` (files, nodes, topics, launch_system.sh modes, identifiers, docs). Overview: section *Rename: ls_thesis → macro_micro_robot, ur5e → ur16e* | The arm has always been the lab UR16e; the names came from the original UR5e project and made the code read as if it drove a UR5e | Clean mapped diff (only the listed exceptions differ), package builds, mock launch: `/ur16e_touch_teleop` up with its topics |

**V7 SimpleHolder replays (2026-10-26, with NDI):** see `logs/V7/REPLAYS_2026-10-26.md`:
results, original → replay map, and why the NDI error is mostly factory-kinematics error.

Lag is the time shift that best lines the robot's tool0 path up with the stylus
target path (`thesis/run_tools/plot_paths.py`). Each version's section below has the full
detail: code state, what was implemented and why, runs, and results.

## Runs and comparing versions

`logs/` is git-ignored (run data stays on the lab PC). `logs/RUN_VERSIONS.md` is a
link to this file, so the version history is readable next to the runs and
tracked in git.

Runs are written to `logs/NEW_RUN/`, which the next recording deletes, so move each
one by hand into `logs/V<n>/`. The exact settings for any run are in its `META.yaml`.

**Comparing versions:** replay a reference run's stylus into the new version
(`ros2 run macro_micro_robot stylus_replay.py V3/CyclingScaling` with the stack on
`input:=replay ur_type:=ur16e`), then `python3 thesis/run_tools/plot_paths.py logs/V<n>`
writes `METRICS.yaml` per run and `METRICS_SUMMARY.csv` per folder. Only replays
of the same run have identical input, so compare those, not two hand-made runs.
A replayed run has a `REPLAY.yaml` naming its source.

## How the code states were recovered

V1, V2 and most V3 runs were recorded on uncommitted code (`code_version:
3fe9421-dirty`). Their exact code was rebuilt by replaying every file edit from
the development sessions in time order on top of commit `3fe9421`, together with
the editor's saved versions of `teleop_settings.py`. Two checks:

1. Replaying up to 2026-10-12 13:45 reproduces commit `af2a1e2` byte for byte
   (all 39 tracked files).
2. Every V1-V3 run's `META.yaml` settings match the rebuilt `teleop_settings.py`
   at that run's time.

`v1`, `v1-90deg` and `v2` are those rebuilt states, committed on top of `3fe9421`
(not on a branch). `v3` is the original commit `af2a1e2`. `v4` is an ordinary
commit.

---

## Common structure (all versions)

```
Touch stylus --omni_state (1 kHz)--> ur5e_touch_teleop --TwistStamped (100 Hz)-->
MoveIt Servo (250 Hz) --> UR joint controller --> UR16e
                ^                                          |
                +------ tool0 pose (TF or joint_states) ----+
```

- **Clutch:** holding the WHITE button engages. The stylus pose and the robot's
  tool0 pose at that moment become the origins, and releasing freezes the robot
  so the stylus can be re-centred.
- **Target:** `p_target = p_robot0 + s · A · (p_stylus - p_stylus0)`. `A` is the
  "toward_base" axis mapping frozen at engage, and `s` is the translation scale.
  Pen tilt (swing) and pen twist come from a swing-twist decomposition of the
  stylus rotation since engage. Tilt is mapped through the same frame onto the
  tool, and twist turns tool0 z (wrist 3).
- **Control law** (`ur5e_touch_teleop.update`):
  `v = k_ff · v_target + K_p · (p_target - p_robot)` and
  `ω = k_ff · ω_target + K_r · rotvec(R_target · R_robot⁻¹)`, each clamped to a
  maximum speed. `v_target` is the low-pass filtered derivative of the target
  (velocity feedforward). The result is sent to MoveIt Servo as a Cartesian twist.
- **Logging** (`trajectory_recorder`, GREY button): `TOUCH_PATH.csv` (target),
  `STYLUS_RAW.csv`, `UR_PATH.csv` (UR RTDE TCP pose), `TRAJECTORY.yaml/csv`,
  a rosbag, and `META.yaml`.

---

## V1 — Baseline

**Code:** tag `v1` (10 cm runs) and `v1-90deg` (identical except
`TOUCH_ROTATION_GAIN 20`). Both are rebuilt from `3fe9421-dirty`.

**Implemented on top of `3fe9421`:**
- Velocity feedforward (`TOUCH_FEEDFORWARD 1.0`, low-pass `TOUCH_FEEDFORWARD_FILTER
  0.02` s) added to the P controller.
- Pen twist via swing-twist decomposition, mapped to wrist 3 (`TOUCH_ENABLE_TWIST`).
- Teleop publishes its target on `~/target_pose`. The recorder writes the path
  logs and a rosbag (`ls_thesis/run_logger.py`), and `thesis/run_tools/plot_paths.py`
  plots the logs and computes the metrics.
- Quieter launch output (`verbose:=true` restores it).

**Signal chain:** MoveIt Servo (started by `ur_moveit.launch.py`) ->
`Float64MultiArray` joint positions -> `forward_position_controller`.

**Settings:** `TOUCH_POSITION_GAIN 10`, `TOUCH_ROTATION_GAIN 10` (20 for the 90° runs),
`TOUCH_MAX_LINEAR_SPEED 2.0` m/s, `TOUCH_MAX_ANGULAR_SPEED 40` rad/s.

**Runs** (`logs/V1`): `V1_Sideways_10CM`, `V1_Vertical_10CM`, `V1_Forwards_10CM`
(`v1`); `V1_90_Degrees_Forwards`, `V1_90_Degrees_Left`, `V1_90_Degrees_Twist`
(`v1-90deg`).

**Result:** translation lag 640-760 ms, and rotation lag above the 1.5 s search
limit.

**Problem found:** in position-output mode, Servo re-bases every step on the
*measured* joint state. The real arm takes about 40 ms to reach a command, so
each step only moves it a fraction of the way, and the arm ran at about 10% of
the commanded speed (0.32 m/s commanded -> 0.04 m/s actual). Mock hardware
hides this because it reaches every command instantly.

---

## V2 — Servo trajectory output

**Code:** tag `v2`, rebuilt from `3fe9421-dirty`.

**Implemented:**
- MoveIt Servo is started by `launch/ur5e_teleop.launch.py` itself, with output
  `trajectory_msgs/JointTrajectory` on
  `/scaled_joint_trajectory_controller/joint_trajectory` and
  `max_expected_latency = SERVO_LOOKAHEAD 0.05` s. Servo now builds each step on
  its own previous (future) command, so it no longer re-bases on the lagging
  measured state. Joint velocities are not published, because the controller
  rejects a non-zero end velocity.
- `TOUCH_LOOP_DELAY 0.05` s: the P term compares the robot with the target as it
  was `loop_delay` ago, so it doesn't keep pushing for motion that is already on
  its way. The feedforward term carries the motion since then.
- Teleop and trajectory playback share `scaled_joint_trajectory_controller`, and
  the player pauses Servo during playback.
- Retuned: gains 10/10-20 -> 8/8; speed caps 2.0 m/s / 40 rad/s -> 1.2 m/s / 6 rad/s.

**Runs** (`logs/V2`): `V2_90_Degrees_Forwards`, `V2_90_Degrees_Left`,
`V2_90_Degrees_Twist` (`v2`).

The other five runs in `logs/V2` (`Right_10CM`, `Up_10CM`, `IN_10CM`,
`General_Motion_Fast`, `General_Motion_Slow`, 13:25-13:32) were recorded after
the V3 motion-scaling code was in place, at scale 1:1. Their robot-side code
equals `v3`; only `plot_paths.py` differed.

**Result** (90° rotation runs): angle lag 128-158 ms (302 ms on the pen-twist
run), against more than 1.5 s in V1. RMS angle error dropped from 27-36° to
6-24°. The five later runs (V3 code at 1:1) show 106-128 ms translation lag on
10 cm moves.

---

## V3 — Motion scaling

**Code:** tag `v3` = commit `af2a1e2` (exact: the runs from 13:39 used this tree).

**Implemented:**
- Selectable motion scale, cycled by double-clicking WHITE (`TOUCH_SCALE_STEPS
  [1.0, 0.2, 0.05]` = 1:1, 1:5, 1:20, with `TOUCH_DOUBLE_CLICK_TIME 0.3` s).
  Nothing waits to see whether a press becomes a double click. Both presses
  engage straight away, the second re-captures the origins so the robot doesn't
  jump, and the new scale applies from that press.
- Rotation is scaled less than translation: `step ** TOUCH_SCALE_ROTATION_EXPONENT`
  (0.5, so 1:5 -> 1:2.24 and 1:20 -> 1:4.47). The deadband shrinks with the scale.
- The current scale is published latched on `/ur5e_touch_teleop/motion_scale`.
- The recorder tries to convert the pendant TCP back to tool0 using an offset
  measured against TF. This never succeeded in V3 (`tcp_offset: null`); fixed in V4.

**Runs** (`logs/V3`): `Scaled5_Slow`, `Scaled5_Fast`, `Scaled20_Slow`,
`Scaled20_Fast`, `CyclingScaling`, `NoMotion`, all with `ur_type:=ur16e`.

**Result:** about 140 ms lag for both translation and angles at every scale. At
1:5 and 1:20 the position only *looks* lag-free, because translation is scaled
down far more than rotation, so the same time delay leaves a gap well under 1 mm.
Static jitter (`NoMotion`) is 0.016 mm.

**Invalid runs:** `CyclingScaling1`-`3` (replays of `CyclingScaling`) were
launched with the default `ur_type:=ur5e` on the UR16e (TF upper_arm->forearm
0.425 m instead of 0.4784 m). Their ~31 mm translation error is real motion on
the wrong kinematics.

---

## V4 — Latency: velocity output option, UR emulator

**Code:** tag `v4`. It also includes `d11bf25` (stylus replay) and `82de072`
(recorder TCP-offset fix) from after V3.

**Why:** an analysis of the V3 logs showed the ~140 ms lag is mostly *not* delay.
With Servo's `JointTrajectory` output, the arm covers only **about 60% of the
commanded twist displacement**. The ratio is 0.60 on the real UR16e and 0.60 in
the emulator, whatever the lookahead. The P loop makes up the remaining 40%, and
that shows as lag. With joint-velocity output the ratio is 1.04.

**Implemented:**
- `stylus_replay.py` (from `d11bf25`): replays a logged run's stylus into the
  stack with the original timing, after moving the robot to the run's start
  pose and setting the same motion scale. This gives identical input for
  comparing versions.
- Recorder fixes (from `82de072`):
  - The pendant TCP offset is now actually measured. RTDE samples arrive at
    500 Hz, ahead of the 20 Hz TF, so they wait until TF catches up.
  - The offset is re-checked throughout the run and saved as
    `tcp_offset.check_max_mm` in `META.yaml`. More than 5 mm means TF uses the
    wrong arm kinematics, and `plot_paths.py` warns about it.
- `SERVO_OUTPUT` (`"trajectory"` by default, or `"velocity"`):
  - `"velocity"` sends Servo's joint velocities (`Float64MultiArray`) to
    `forward_velocity_controller` (UR `speedj`).
  - All nodes now activate whichever controller Servo feeds
    (`ur5e_common.servo_controller`).
  - Trajectory playback still uses the trajectory controller.
- `VelocityWatchdog` (in both teleop nodes):
  - `forward_velocity_controller` keeps its last command indefinitely, so if
    Servo goes quiet for 50 ms mid-motion, the watchdog publishes zeros.
  - In the sim it stopped a runaway wrist command after 0.64°.
- `TOUCH_POSE_SOURCE` (`"tf"` by default, or `"joint_states"`): the robot pose
  for the control loop is computed by forward kinematics from each
  `/joint_states` message (500 Hz, URDF from `/robot_description`), instead of
  the 20 Hz TF.
- `src/ur_emulator.py` + `emulate_ur:=true`: mock hardware with the real arm's
  response, for latency testing without the robot.
  - Trajectory mode = sample the trajectory like the controller, then 16 ms dead
    time + a 15 ms first-order lag (fitted to the V3 logs, 0.1-0.4 mrad RMS).
  - Velocity mode = 16 ms dead time + a 40 rad/s² ramp, taken from the driver's
    `speedj` call. This is an estimate, since no real velocity-mode data exists
    yet.
  - Validated on `V3/CyclingScaling`: sim 144 ms / 12.2 mm vs real 140 ms / 11.9 mm.
- `SERVO_LOOKAHEAD` floor documented: Servo refuses to start below 3× its 4 ms
  publish period (0.012 s).

**Defaults are unchanged:** with default settings, V4 controls the robot exactly
like V3.

**Sim results** (mock UR16e + emulator, replaying `V3/CyclingScaling`, scored on
forward kinematics of `/joint_states`; runs in `logs/SIM_latency/`):

| Configuration | Lag pos / angle | RMS error | Jitter |
|---|---|---|---|
| V3 settings | 144 / 144 ms | 12.2 mm | 0.07 mm |
| `SERVO_LOOKAHEAD 0.02` | 126 / 124 ms | 10.4 mm | 0.07 mm |
| + feedforward filter 0.01 s, 250 Hz teleop | 122 / 118 ms | 10.0 mm | 0.08 mm |
| `SERVO_LOOKAHEAD 0.013` (minimum) + the above | 114 / 112 ms | 9.5 mm | 0.08 mm |
| + `TOUCH_POSE_SOURCE "joint_states"` (trajectory mode) | 130 / 128 ms | 10.7 mm | 0.08 mm |
| **Velocity output** + filter 0.01, 250 Hz, joint_states, loop delay 0.03 | **30 / 34 ms** | **2.5 mm** | 0.25 mm |
| Same, pessimistic arm (30 ms dead time, 20 ms lag, 20 rad/s²) | 38 / 46 ms | 4.3 mm | 0.20 mm |

In trajectory mode, the fresh joint_states pose made tracking worse. The stale
TF had been pushing the P loop harder, which partly hid the 60% shortfall.

**Settings to try on the real arm** (not yet tested there): `SERVO_OUTPUT =
"velocity"`, `TOUCH_FEEDFORWARD_FILTER = 0.01`, `TOUCH_PUBLISH_RATE = 250.0`,
`TOUCH_POSE_SOURCE = "joint_states"`, `TOUCH_LOOP_DELAY = 0.03`.

**Runs on the real arm** (2026-10-15, all `code_version: ccb0ee8-dirty`, which is
`v4` plus the V5 filter work; `SERVO_OUTPUT = "velocity"` in each):

- First velocity run, by hand: only `SERVO_OUTPUT` changed, everything else at the
  V4 defaults (100 Hz publish, TF pose, feedforward filter 0.02 s). The arm
  vibrated badly. Not kept (`NEW_RUN` was overwritten later); analysed below.
- `V4/LongBenchmark_failed`, `V4/LongBenchmark_failed2`, `V4/ShortBenchmark_failed`:
  replays of `V3/LongBenchmark` and `V3/ShortBenchmark` at 1:1, with the V5
  command filter at 8 Hz. **Each one faulted the robot** 4-5 s in. In
  `LongBenchmark_failed2` the rosbag is unreadable and the path logs were not
  written.

**Launch:** always pass `ur_type:=ur16e` for the lab UR16e (the default is ur5e).

### Why velocity mode vibrated

Compared with the V1/V2 trajectory-mode runs:

- **Size:** the tool vibrated about 5× more at 15-25 Hz (main peak ~19 Hz) and
  about 7× more at 30-60 Hz, about 0.6 mm peak to peak. Servo reported no warnings
  and speed scaling stayed above 92%, so the robot was following its commands.
- **The command itself was shaky:** the twist sent to Servo already had the same
  19 Hz content, worst on rotation. The feedforward term passes stylus noise
  through, and its 20 ms filter only weakens 19 Hz by about 2.5×.
- **Where the noise comes from:** `omni_state` publishes at 1 kHz, but the pose only
  changes every ~4 ms (sometimes 12 ms); the samples in between repeat the last
  one. The teleop node differences the target every 10 ms, so each window holds
  2 or 3 real updates at random. That puts up to ±33% noise on the feedforward
  velocity, and it grows with speed, which is why the vibration appeared during
  motion.
- **Why trajectory mode was fine:** Servo's smoothing filter only smooths joint
  *positions*. In velocity mode Servo computes the velocities before smoothing and
  never recomputes them afterwards, so the command reaches the arm unfiltered
  (MoveIt Servo 2.12.4, `servo.cpp`, `getNextJointState`). Trajectory mode also
  has the 50 ms lookahead and the UR's own position tracking to soften it.
- **Possible feedback loop:** the stylus had about 3× more 19 Hz jitter than in
  the earlier runs (16 µm vs about 5 µm RMS). The arm's vibration may be getting
  back into the Touch through the bench or the operator's hand.

### Why the velocity-mode benchmarks faulted the robot

**What the robot reported** (driver log, the same sequence in all three runs):
`C306A0 Joint: Critical error`, then `C281A3 joint entered the Fault State`
(plus `C740A0 Hardware monitoring: Critical error` in the third). The program
stopped, and the arm had to be restarted from the pendant.

**What the joints were doing:** in both readable runs a wrist joint was at its
rated top speed at the moment of the fault:

| Run | Joint at the fault | Peak speed | Peak acceleration |
|-----|--------------------|------------|-------------------|
| `LongBenchmark_failed` (velocity) | wrist 1 | 3.12 rad/s | 43 rad/s² |
| `ShortBenchmark_failed` (velocity) | wrist 3 | 3.17 rad/s | 36 rad/s² |
| `NEW_RUN` = `LongBenchmark` replay, trajectory mode | - (no fault) | 2.01 rad/s | 42 rad/s² |
| `V3/LongBenchmark`, `V3/ShortBenchmark` (trajectory) | - (no fault) | 1.92-1.95 rad/s | 42-117 rad/s² |

The UR16e's wrist limit is 180°/s = **3.14 rad/s** (`ur_description` UR16e
`joint_limits.yaml`). Peak accelerations were no higher than in trajectory runs
that did not fault, so speed is the difference, not acceleration.

**Why the wrist got that fast:**

1. The benchmarks twist the pen quickly: the stylus target rotates at up to
   3.3-3.6 rad/s just before each fault.
2. Trajectory mode delivers only ~60% of the commanded motion (see V4 "Why"), so
   in V3 the wrists peaked around 2 rad/s and the shortfall showed up as lag
   instead. Velocity mode delivers ~100%, so the wrist was asked for the full
   speed.
3. Servo scales joint velocities down to the URDF limit, which is *exactly*
   3.14 rad/s with no margin. The arm's own tracking overshoots the command
   slightly, which takes the wrist past its rated speed, and the joint faults.
4. `TOUCH_MAX_ANGULAR_SPEED` (6 rad/s) is above the wrist limit, so it never
   capped anything.

This is inferred from the timing and the joint speeds. The pendant's log (Log
tab, at each fault) would name the exact joint and fault reason. Neither V5
filter fixes it, because they smooth noise and do not limit speed. **Do not
re-run the velocity-mode benchmarks at 1:1 until there is a joint speed margin**
(e.g. Servo limited to ~80% of the URDF joint speeds, or
`TOUCH_MAX_ANGULAR_SPEED` well below 3.14 rad/s).

---

## V5 — Smoothing for velocity mode

**Code:** tag `v5` (commit `5a7b1af` holds the code; the tag sits on the following
docs-only commit). The V5 runs were recorded before the commit, so their META.yaml
says `code_version: ccb0ee8-dirty`. The final benchmark runs
(`V5/LongBenchmarkTuned`, `V5/ShortBenchmarkTuned`, 12:20-12:21) ran on exactly the
code and settings in `v5`: no code or settings file changed between those runs and
the commit, only docs. Earlier V5 runs (10:18-12:15) used intermediate states,
described in the test sections below together with their settings.

**Why:** velocity mode vibrated on the real arm, because the command reaching the
arm is unfiltered in that mode and it was noisy (see V4 "Why velocity mode
vibrated"). Two new filters in `ur5e_touch_teleop.py`, each with its own toggle
in `teleop_settings.py`, plus setting changes found on the real arm (table
below). All settings are recorded in each run's `META.yaml` automatically.

**Implemented:**

- `TOUCH_COMMAND_FILTER` (Hz, 0 = off, default 8.0): a 2nd-order Butterworth
  low-pass on all six values of the twist sent to Servo (feedforward + gains),
  applied after the speed caps and clamped again afterwards. It restarts from
  rest on every WHITE press. Releasing WHITE still sends an unfiltered zero, so
  the arm stops at once rather than coasting. The node refuses a cutoff at or
  above half `TOUCH_PUBLISH_RATE`. This treats the symptom: it removes most of
  the vibration but adds lag.
- `TOUCH_KALMAN` (True/False, default True): a constant-velocity Kalman filter on
  the raw stylus pose, with an extended (error-state) version for the
  orientation. This treats the cause. It is fed only the real Touch updates (or a
  repeat after 8 ms, which means the stylus has stopped) and predicts in between,
  so the target moves smoothly and the feedforward velocity no longer jumps with
  how many updates fell in each window. It restarts at rest after a gap in the
  Touch data. Settings: `TOUCH_KALMAN_ACCEL_NOISE 2.0`,
  `TOUCH_KALMAN_POSITION_NOISE 5e-4` m, `TOUCH_KALMAN_ANGULAR_ACCEL_NOISE 30.0`,
  `TOUCH_KALMAN_ROTATION_NOISE 0.01` rad.
- **Why a Kalman filter rather than a One Euro filter:** both were tried on the V3
  benchmark stylus data. A One Euro filter raises its cutoff with speed, so it
  lets the speed-dependent noise above through. The settings that did cut the
  noise added 30-50 ms of feedforward lag and 2-6 mm of path lag. The Kalman
  filter cut the noise with no added lag.

**Settings changed from `v4`** (the full V5 set, current as of 2026-10-15 10:35;
each one is explained in the sections below):

| Setting | V4 | V5 | Why |
|---------|----|----|-----|
| `SERVO_OUTPUT` | `"trajectory"` | `"velocity"` | ~140 ms -> 56-60 ms lag on the real arm (trajectory mode delivers only 60% of the motion) |
| `TOUCH_KALMAN` (+ 4 noise values) | - | `True` (2.0, 5e-4 m, 30.0, 0.01 rad) | Removes the noise from the Touch's 4 ms update steps without adding lag |
| `TOUCH_COMMAND_FILTER` | - | 5.0 Hz | Velocity mode has no smoothing; removes stylus tremor/jitter amplified by the feedforward (8 Hz first; 6 Hz cut tool jerk by a third, see "500 Hz + limiter" test) |
| `TOUCH_MAX_CORRECTION_SPEED` / `_ANGULAR_SPEED` (new) | - | 0.05 m/s / 0.5 rad/s | Gradual catch-up after the robot was held back by a cap, instead of a surge |
| `TOUCH_POSE_SOURCE` | `"tf"` | `"joint_states"` | TF is 20 Hz; its staircase put a 20 Hz sawtooth into the correction term |
| `TOUCH_PUBLISH_RATE` | 100 Hz | 500 Hz | Matches Servo's cycle; at 100 Hz the 10 ms velocity steps caused a ~100 Hz buzz, at 250 Hz a 250 Hz chatter |
| `SERVO_VELOCITY_RATE` (new) | (250 Hz) | 500 Hz | Velocity mode only: a new joint speed every UR control cycle |
| `VELOCITY_LIMITER` (new, + speed fraction, accelerations, smoothing) | - | `True` (0.8, [8, 8, 15, 25, 25, 25] rad/s², 2 × 3 ms after the caps) | Velocity mode only: joint speed/acceleration caps and smoothing; every velocity-mode fault came with peak chatter in the faulting wrist |
| `TOUCH_MAX_ANGULAR_SPEED` | 6.0 rad/s | 3.0 rad/s | Was set to 2.0 by hand after the benchmark faults. Raised to 3.0 once `joint_velocity_limiter` capped the joints: 2.0 clipped 16-26% of the benchmarks |
| `TOUCH_STARTUP_DELAY` | 8.0 s | 8.0 s (was briefly 0) | At 0 the velocity controller was requested before it existed, so the arm never moved |

**Offline check, command filter** (the vibrating run's recorded 100 Hz command run
through the filter):

| Cutoff | 15-25 Hz left | 30-50 Hz left | Added lag |
|--------|---------------|---------------|-----------|
| 10 Hz | 23% | 2% | 22 ms |
| **8 Hz (default)** | **14%** | **1%** | **28 ms** |
| 6 Hz | 8% | 1% | 37 ms |

**Offline check, Kalman filter** (recorded stylus fed through the node's filter at
1 kHz, sampled at 100 Hz as the node does; noise = feedforward velocity above
10 Hz; lag against a smoothed true stylus velocity):

| Run | Linear noise (mm/s) | Linear lag (ms) | Angular noise (rad/s) | Angular lag (ms) |
|-----|---------------------|-----------------|-----------------------|------------------|
| `V3/LongBenchmark` | 11.0 -> 7.2 (-35%) | 25.0 -> 23.5 | 0.153 -> 0.106 (-30%) | 20.0 -> 19.5 |
| `V3/ShortBenchmark` | 11.6 -> 9.0 (-23%) | 25.5 -> 24.0 | 0.148 -> 0.119 (-20%) | 20.0 -> 19.5 |
| `V3/Scaled5_Slow` | 3.3 -> 1.9 (-42%) | 26.5 -> 25.0 | 0.041 -> 0.034 (-17%) | 19.0 -> 19.5 |
| `V2/V2_90_Degrees_Twist` | 0.9 -> 1.0 | 26.0 -> 24.0 | 0.063 -> 0.040 (-37%) | 21.0 -> 20.0 |

**Trade-off chosen:** a constant-velocity filter runs on briefly when the hand
stops suddenly. Smoother settings cut more noise (up to ~40%) but overshoot more:
on a 50 ms stop from 200 mm/s at 1:1, 1.2 mm with accel noise 0.1, against
0.13 mm with the chosen 2.0 (0.33 mm from 500 mm/s). For rotation, 0.6° from
3 rad/s with the chosen 30, against 1.0° with 10. The overshoot shrinks with the
motion scale (5× at 1:5, 20× at 1:20).

### Real-arm test: velocity mode + Kalman filter, command filter off (still vibrated)

**Run:** 2026-10-15 10:18, by hand at 1:1, about 2 s engaged (`logs/NEW_RUN` at
the time). Settings: `SERVO_OUTPUT "velocity"`, `TOUCH_KALMAN True`,
`TOUCH_COMMAND_FILTER 0`, `TOUCH_POSE_SOURCE "tf"`, `TOUCH_MAX_ANGULAR_SPEED 2.0`.
The rosbag has a 335 ms dropout at 0.49 s (all topics), so the analysis uses the
clean 1.6 s after it.

**Startup problem first:** the arm did not move at all at first.
`TOUCH_STARTUP_DELAY` had been set to 0, so the teleop node asked for
`forward_velocity_controller` before the driver had loaded any controllers. The
switch failed and nothing retried, so no motion controller was active.
Trajectory mode never showed this, because there the driver starts the
trajectory controller itself. The controller was then switched on by hand. Fix:
`TOUCH_STARTUP_DELAY` back to 8.0.

**What the logs show:**

- **The vibration is commanded.** The arm's measured tool rotation speed matches
  the commanded twist band by band up to 25 Hz (e.g. wz 8-15 Hz: 0.018 rad/s
  commanded, 0.018 measured). The tool vibrates at about 0.05-0.08 mm RMS
  (8-60 Hz), peaking at 19.5 Hz with a harmonic at 39 Hz.
- **The Kalman filter was working, but this noise is not the staircase noise.**
  The target changed on every sample (against ~25% raw), yet the command's
  8-40 Hz content was as large as in the first, unfiltered run (wz 0.023 vs
  0.024 rad/s), during a ~3× gentler motion.
- **Main source: the stylus's own small fast motions, amplified by the
  feedforward.** The stylus moved about 0.025° RMS at 7-13 Hz (physiological hand
  tremor) and 0.01° at 15-25 Hz. The feedforward differentiates the target,
  multiplying a wiggle by 2πf (×60 at 10 Hz, ×125 at 20 Hz), so these become
  ~0.02 rad/s, exactly the command's content at 1:1. Most of it (wy 0.0115 of
  0.0126 rad/s, wz 0.021 of 0.025) is in the feedforward term. A constant-velocity
  Kalman filter passes this because it looks like real motion; a low-pass is what
  removes it. In trajectory mode Servo's position smoothing did that job.
- **Smaller source: the 20 Hz robot pose.** `robot_state_publisher` publishes TF at
  20 Hz (`publish_frequency 20.0`). With `TOUCH_POSE_SOURCE "tf"` the correction
  term sees the robot pose as a staircase that steps every 50 ms, which is a
  20 Hz sawtooth (+ harmonics) in the command. That matches the 19.5/39 Hz peaks,
  but by itself it adds only ~0.6 mm/s.

**Changes (V5):**

- `TOUCH_POSE_SOURCE = "joint_states"`: the robot pose is computed from each
  500 Hz `/joint_states`. Checked on this run: it matches the controller's own
  TCP pose to within 0.7 mm (the TCP offset held constant), so the UR16e
  kinematics are right.
- `TOUCH_COMMAND_FILTER = 8.0` again: it leaves 53% at 10 Hz, 12% at 20 Hz and 1%
  at 40 Hz, for 28 ms of lag. If the tremor at ~10 Hz is still felt, 6 Hz leaves
  33% / 7% for 37 ms. Surgical teleoperation usually filters tremor at about this
  cutoff anyway.
- `TOUCH_STARTUP_DELAY = 8.0` (see above).
- The Kalman filter stays on. It still removes the staircase noise and lag
  costs nothing.

### Real-arm test: + joint_states pose + command filter 8 Hz (less, but still vibrating)

**Run:** 2026-10-15 10:29, by hand at 1:1, 5.6 s of clean data (`logs/NEW_RUN` at
the time; the rosbag again has a 334 ms dropout near the start). Settings as
above: velocity mode, Kalman on, command filter 8 Hz, `joint_states` pose,
publish rate 100 Hz.

**Lag** (`plot_paths.py`): **56 ms translation, 60 ms angles**, against ~140 ms in
V3. That matches the prediction of the V4 sim's ~30 ms plus the command filter's
28 ms. With the lag removed, the path error is 0.72 mm RMS / 0.35°. Robot jitter
above 5 Hz is 0.14 mm (the target's own is 0.20 mm).

**What changed in the vibration:**

- The 20 Hz buzz is mostly gone: the measured tool rotation at 15-25 Hz fell
  from 0.0155 to 0.0036 rad/s (wz), and the 19.5/39 Hz peaks disappeared. Tool
  vibration (8-60 Hz) fell from 0.03-0.08 to 0.03-0.04 mm RMS.
- **What is left peaks at ~100 Hz**, and it is not in the command (45-120 Hz:
  wy 0.0019 rad/s commanded, 0.054 measured). Both velocity-mode runs show a
  97-102 Hz joint peak, with 3-10× more 45-140 Hz content than the trajectory-mode
  benchmark. Cause: the teleop node publishes at 100 Hz and Servo runs at 250 Hz,
  so Servo holds each command for 2 or 3 cycles. `speedj` sees a velocity
  staircase that steps every 10 ms, and the arm feels each step as a small
  kick. Trajectory mode hid it because the robot interpolates positions.
- Some ~8-10 Hz motion remains too (peaks at 7.8 Hz). That is hand tremor,
  which the 8 Hz filter only halves at 10 Hz.

**Change:** `TOUCH_PUBLISH_RATE = 250.0`, so every Servo cycle gets a fresh command.
The steps are 2.5× smaller and land at 250 Hz, which the arm passes far less
of. The V4 sim ran at 250 Hz. The command filter's coefficients follow the
publish rate automatically. If the tremor is still felt, lower
`TOUCH_COMMAND_FILTER` to 6 Hz (37 ms instead of 28 ms).

### Real-arm test: 250 Hz publish (noise remains, two more joint faults)

**Run:** 2026-10-15 10:35:54, by hand at 1:1, 10 s (`logs/NEW_RUN` at the time).
Full V5 settings including `TOUCH_PUBLISH_RATE 250`. The operator reported "weird
noises" from the arm.

**Two more `C306A0 Joint: Critical error` faults**, at 10:28:14 and 10:34:35. The
sequence was the same as the benchmark faults: `C281A3` joint entered the Fault State,
`C271A1` low-level runtime behind, `C740A0` hardware monitoring critical error.
**Neither was recorded:** both happened between recordings (the bag only runs
while GREY recording is on), so the joint speeds and torques at the moment of
these faults are unknown. In the recorded run the fastest wrist was 2.25 rad/s
(72% of its limit).

**The noise is torque chatter in the wrist motors, at the command update rate:**

| Run | Mode / teleop rate | Wrist torque, 45-250 Hz RMS | Torque peaks |
|-----|--------------------|-----------------------------|--------------|
| `V3/LongBenchmark`, `V3/ShortBenchmark`, `V2` twist | trajectory / 100 Hz | 0.01-0.05 Nm | none consistent |
| `V4/LongBenchmark_failed`, `ShortBenchmark_failed` | velocity / 100 Hz | 0.34-0.85 Nm | 100, 150 Hz |
| this run | velocity / 250 Hz | 0.54-0.98 Nm | 246-250 Hz |

The chatter appears only while engaged and moving (0.01 Nm when not engaged), and
its frequency follows the update pattern. At 100 Hz the teleop and Servo rates
beat every 20 ms, giving 50 Hz harmonics. At 250 Hz Servo's own cycle remains,
feeding the robot's 500 Hz loop. The velocity setpoint therefore changes only
every 2 or more robot cycles, and each step is a small velocity jump that the
joint's velocity loop answers with a torque kick. In trajectory mode the robot
interpolates its position setpoint every cycle, so there are no steps. Raising
the teleop rate to 250 Hz did not reduce the chatter; it only moved it to
250 Hz.

**Fault cause, not confirmed:** two candidates. (1) Wrist overspeed, as in the
recorded benchmark faults: `TOUCH_MAX_ANGULAR_SPEED 2.0` left only ~3% wrist
headroom in that data. (2) This torque chatter, 15-20× above trajectory mode,
in the joints that faulted before. The pendant's log entry for each fault (joint
and detail) and a recording that runs continuously would settle it.

### Recorded joint fault (wrist 3) and the fix: 500 Hz chain + joint velocity limiter

**Run:** 2026-10-15 10:44:58, by hand at 1:1 (`logs/NEW_RUN` at the time). Full V5
settings with `TOUCH_PUBLISH_RATE 250`. The arm faulted 8.3 s in (`C306A0`, wrist
3), and this time the fault is inside the recording.

**Not overspeed:** wrist 3 was at **1.31 rad/s** (41% of its 3.14 rad/s limit).
In the last 3 s it swung back and forth between about ±1.3-1.9 rad/s (pen
twisting), reaching up to 28 rad/s². The current reported for it was -4.2 at the
fault (range -4.4 to 3.5 over the run).

**What every recorded fault has in common:** in the 0.1 s before each of the three
recorded velocity-mode faults, the joint that faulted had its **highest
high-frequency current chatter of the whole run**:

| Run | Joint | Chatter before fault | Run median | Speed at fault |
|-----|-------|----------------------|------------|----------------|
| this run | wrist 3 | 1.39 (top 0.1%) | 0.61 | 1.31 rad/s |
| `V4/LongBenchmark_failed` | wrist 1 | 2.02 (top 0.1%) | 0.30 | 2.38 rad/s |
| `V4/ShortBenchmark_failed` | wrist 1 | 2.04 (top 1.5%) | 0.65 | 1.85 rad/s |

Speed at the fault was not consistent: earlier I put the benchmark faults down
to overspeed, but only one joint was at its limit there. Acceleration *as such*
is not the cause either. The V3 trajectory-mode benchmarks needed the same wrist
accelerations (99th percentile 25-33 rad/s², peaks up to 42) and never faulted.

**Mechanism (most likely; the UR's own log would confirm):** the driver runs each
velocity command as `speedj(qd, 40 rad/s², 2 ms)`
(`ur_client_library/resources/external_control.urscript`). With Servo at 250 Hz
the set point changes every 2 robot cycles, and each change is a step of size
acceleration × period. During hard reversals the steps are largest and the joint
ramps at the 40 rad/s² cap between set points, which is the chatter that peaks
right before each fault. Trajectory mode interpolates a smooth position set point
every cycle, so it never chatters, whatever the acceleration.

**Changes:**

- **500 Hz command chain** (velocity mode only): `SERVO_VELOCITY_RATE = 500` (Servo's
  `publish_period` 2 ms, the UR's own cycle) and `TOUCH_PUBLISH_RATE = 500`. The
  joint speed set point now changes every robot cycle in steps half the size.
  Trajectory mode keeps Servo at 250 Hz.
- **`joint_velocity_limiter.py`** between Servo and `forward_velocity_controller` (or
  `ur_emulator`), started by the launch file in velocity mode when
  `VELOCITY_LIMITER = True`. Per Servo command it:
  1. low-pass filters each joint (`VELOCITY_LIMITER_FILTER 40` Hz, removes steps
     from late commands);
  2. caps joint speed at `VELOCITY_LIMITER_SPEED_FRACTION 0.8` of the URDF limit
     (1.68 rad/s base/shoulder, 2.51 rad/s elbow/wrists on the UR16e);
  3. caps joint acceleration at `VELOCITY_LIMITER_MAX_ACCELERATION [8, 8, 15, 25, 25, 25]`
     rad/s². That is below speedj's 40, so the joint is never bang-banging; the
     wrist cap is near the benchmarks' 99th percentile.

  Both caps scale all joints together, so the tool keeps its direction and just
  slows. If Servo goes quiet, the limiter repeats its last command through the
  filter and limits, and switches to zero after 50 ms. `VelocityWatchdog` now
  watches the limiter's output. The setting `TOUCH_MAX_ANGULAR_SPEED` caps the
  *tool*, which cannot protect the joints (see below).
- `/joint_velocity_limiter/commands` and `/forward_velocity_controller/commands`
  added to `LOG_BAG_TOPICS`, so the joint speed set points are in every run.

**Sim check** (mock UR16e + `ur_emulator`, domain 88, replay of `V3/ShortBenchmark`,
the benchmark that faulted the real arm):

| | Pass-through (no limiter, 500 Hz) | With limiter |
|-|-----------------------------------|--------------|
| Twist / Servo / limiter rates | 500 / 500 / 500 Hz | 500 / 500 / 500 Hz (teleop p99 gap 2.8 ms) |
| Fastest wrist | 2.82 rad/s | 2.51 rad/s (cap) |
| Largest step-to-step set point acceleration | 109 rad/s² (wrist 1) | 25 rad/s² (cap) |
| Set point 45-250 Hz content (wrist 1) | 0.0060 rad/s | 0.0017 rad/s (4-5× less) |
| Translation lag | 55 ms | 58 ms |
| Path error, lag removed | 3.9 mm | 4.7 mm (the limiter slowed ~10% of this violent run) |

Servo commanded the wrists up to 2.83 rad/s although `TOUCH_MAX_ANGULAR_SPEED` was
2.0 rad/s, confirming that a tool-speed cap does not bound joint speeds. Not yet
run on the real arm.

### Real-arm test: 500 Hz + limiter (no faults; still a bit jerky) and smoothing

**Run:** 2026-10-15 11:03:31, by hand at 1:1, 12 s engaged (`logs/NEW_RUN` at the
time). First run with the 500 Hz chain and `joint_velocity_limiter`. **No joint
faults.** The chain held 500 Hz throughout (one 8 ms gap) and Servo reported no
warnings. The operator found the motion "still a bit jerky".

**Lag** (`plot_paths.py`): **58 ms translation, 72 ms angles**. With the lag
removed, the error is 1.34 mm / 1.94° (max 9.7°). Robot jitter above 5 Hz is
0.12 mm, against the target's own 0.18 mm.

**Why it was jerky: clipping, then catching up.**

- The tool angular cap (`TOUCH_MAX_ANGULAR_SPEED 2.0`) clamped the command for 6.5%
  of the engaged time. The limiter changed the command for 3% (mostly wrist 1 at
  its 2.51 rad/s speed cap, cutting up to 1.7 rad/s), and was at an acceleration
  cap for 1.7%. A wrist cannot twist as fast as a hand at 1:1 (the target reached
  2-3 rad/s), so the robot has to fall behind on fast pen twists.
- 90% of the jerkiest 5% of moments (tool angular jerk) were within 0.15 s of a
  clamp. Example: at 7.47 s the target turned at 2.1 rad/s while the limiter held
  the robot to 0.76 rad/s through a reversal, and the catch-up 0.1 s later had the
  run's largest jerk (1267 rad/s³).
- The catch-up is the gain term. The angular correction (8 × error) is below
  0.11 rad/s 90% of the time, but its top 5% is 1.0-1.7 rad/s: after a clamp the
  robot is 6-12° behind and the gain asks for all of it at once.
- The robot is already ~6× smoother than the hand: tool angular jerk p95 ~470 vs
  3000 rad/s³ for the target, both smoothed at 30 Hz. But it is rougher than the
  8 Hz-filtered target (281), and that extra comes from the clamps and catch-up.

**Changes:**

- `TOUCH_MAX_CORRECTION_SPEED = 0.05` m/s and `TOUCH_MAX_CORRECTION_ANGULAR_SPEED = 0.5`
  rad/s: caps on what the gain terms alone may add, so the robot catches up
  gradually after it was held back. Normal tracking needs far less (99% of linear
  corrections < 0.015 m/s, 90% of angular < 0.11 rad/s).
- Limiter smoothing moved **after** the caps and made critically damped:
  `VELOCITY_LIMITER_SMOOTHING = 0.003` s, two first-order stages (~6 ms lag),
  replacing the 40 Hz Butterworth before the caps (`VELOCITY_LIMITER_FILTER`). It
  rounds the corners the caps make, and its never-negative impulse response means
  the output cannot exceed the speed or acceleration caps (unit test: 2.513 rad/s
  and 25.0 rad/s² exactly).
- `TOUCH_COMMAND_FILTER = 6.0` (was 8.0).

**Sim comparison** (mock UR16e + `ur_emulator`, replaying this run's stylus; tool jerk
from forward kinematics of the mock joints, smoothed at 30 Hz):

| Variant | Angular jerk p95 / p99 (rad/s³) | Linear jerk p95 / p99 (m/s³) | Lag pos / ang (ms) | Error, lag removed (mm / °) |
|---------|------|------|------|------|
| V0: limiter smoothing only | 513 / 796 | 24.0 / 40.3 | 61 / 84 | 1.05 / 2.60 |
| V1: + correction caps 0.05 / 0.5 | 467 / 714 | 24.2 / 36.5 | 59 / 80 | 1.06 / 2.74 |
| **V3: V1 + command filter 6 Hz (chosen)** | **343 / 541** | **18.3 / 33.5** | **63 / 86** | **1.34 / 3.16** |
| V6: V1 + command filter 5 Hz | 265 / 440 | 15.4 / 24.7 | 66 / 90 | 1.48 / 3.33 |
| V7: angular cap 0.3 + filter 6 Hz | 335 / 551 | 17.4 / 27.8 | 63 / 84 | 1.26 / 3.27 |
| V4: V1 + `TOUCH_LOOP_DELAY` 0.08 | 529 / 817 | 26.2 / 38.8 | 80 / 94 | 0.73 / 1.53 |

V3 cuts angular jerk by a third against V0 for ~2 ms more lag. V6 (5 Hz) halves it
for ~5-6 ms, the next step if more smoothing is wanted. A longer loop delay
(V4) tracked the path better but was jerkier and 14-21 ms slower, so it was not
used. Not yet run on the real arm.

### Real-arm test: smoothing applied ("very smooth", but catches at points)

**Run:** 2026-10-15 11:21:54, by hand at 1:1, mostly pen rotation with the tip almost
still (`logs/NEW_RUN` at the time). Settings as in the previous section (command
filter 6 Hz, correction caps, limiter smoothing after the caps). The operator found
it very smooth, but felt it "catching" at individual points.

**Lag** (`plot_paths.py`): **54 ms translation, 56 ms angles**. With the lag removed,
the error is 0.57 mm / 0.31° (max 1.16 mm / 0.97°). Jitter above 5 Hz: robot
0.04 mm / 0.06°, target 0.06 mm / 0.13°. Best tracking of any run so far.

**What the catches are not:**

- Servo: no warnings all run. Not a singularity: wrist 2 stayed at -84° to -88°,
  far from 0°/180°. No collision slow-down, no joint limit.
- Clipping: the tool angular cap was never reached, no joint acceleration cap
  was hit, and the fastest joint was 1.16 rad/s (cap 2.51).
- The UR: speed scaling stayed at 100% (no safety slow-down). Every joint followed
  its commanded speed within ~5 ms, with gain 0.96-1.00 on the moving joints. No
  joint stuck when passing through zero speed (no time near zero beyond what was
  commanded), so not gearbox stiction.
- The Kalman filter's "stopped" rule (a repeated Touch sample after 8 ms is a
  measurement): it fired only 4 times while turning, and longer hold times changed
  nothing offline.

**What they are:** 14 dips in the robot's tool speed (below 50% of the local peak,
under 0.3 s). 11 are mid-motion (direction change under 60°), 3 are reversals. In
almost every one **the raw stylus reading dips just as much** (e.g. 4.14 s: stylus
12%, target 13%, robot 20% of the speed before). The robot follows the target, and
the target follows the stylus. The hesitations are already in the Touch signal.
They are not at one place: the tip stayed within a few mm, and the hesitations
were spread over ±10-12° of orientation.

**Hand or device?** Split into pen-frame axes, some hesitations are a single axis
stalling while the others keep moving (the main tilt axis at 1.76, 3.02, 3.35 and
4.14 s), which would fit a sticking gimbal joint. Others stop all axes together
(6.88, 7.33 s), which looks like the hand. Slow manual motion is naturally made of
sub-movements a few times per second, and now that the arm follows within ~55 ms
with nothing clipped, those show up on the robot. The pose alone cannot separate
the two.

**Change:** `/phantom/joint_states` (waist, shoulder, elbow, yaw, pitch, roll encoder
angles) added to `LOG_BAG_TOPICS`, so the next run shows directly whether one
Touch joint sticks or skips. (At rest the shoulder reads 0.2693 rad, the same as the
stuck value noted on 2026-10-08 for SN 19039000821, but that is likely just the
docked pose. It needs checking while moving.)

### Real-arm test: the "catches" are stick-slip in the Touch's pitch gimbal

**Run:** 2026-10-15 11:33:02, by hand at 1:1, about 6.6 s engaged. The motion was almost
pure stylus pitch (rocking the pen), with `/phantom/joint_states` recorded for the
first time. The operator felt the hand motion was smooth but the robot still caught
at points. Touch in use: **SN 22129000559** (its `~/.3dsystems/config` temperature
file was written at 11:35; `Default Device.config` still names 19039000821).

**Touch encoders while engaged:** shoulder moved 15.6-17.9° (not stuck on this
unit). Pitch carried the motion (-167.8° to -110.7°, 0.0732° per count, an update
every ~4 ms while moving). Yaw and roll moved ~10° each, and the arm joints barely
moved.

**At the biggest robot catches the pitch joint stops dead and then jumps** (Touch
joint speeds smoothed at 15 Hz, shifted by the ~55 ms robot lag):

| Robot catch | Robot speed at the dip | Pitch before -> during -> after (°/s) | Yaw / roll |
|-------------|------------------------|-------------------------------------|------------|
| 3.94 s | 7% | 16.6 -> **0.0** -> 46.5 | < 2 |
| 1.64 s | 10% | 28.2 -> **0.0** -> 25.2 | < 6 |
| 1.93 s | 17% | 34.3 -> **0.1** -> 38.0 | < 3 |
| 0.48 s | 20% | 10.4 -> **0.0** -> 21.7 | < 8 |

A full stop in one gimbal axis mid-motion, followed by a burst at up to 2-3× the
speed before, is the signature of **stick-slip friction**. The joint sticks while the
hand keeps pushing (pen and fingers flex), then breaks free and catches up. Single
pitch-count stalls (15-66 ms with no count, then exactly one count, then ~4 ms per
count) appeared only at slow pitch speeds (8-24°/s). Faster passes through the
same angles (up to 47°/s) never stalled, and the stalls were not tied to one
pitch angle. Yaw and roll never stalled. The encoder is not skipping counts: the
angle stays continuous, so the joint itself is stopping.

**Robot side:** no clipping, no Servo warnings, speed scaling 100%, joints follow
their commands within ~5 ms. The robot reproduces the Touch's stop-go faithfully,
because it now follows within ~55 ms with nothing smoothing at the 50-200 ms scale
of these stalls. Filtering them out would need a ~2-3 Hz low-pass (~70 ms more lag).

**Next:** check the Touch's pitch gimbal joint (the stylus pivot in the gimbal yoke)
for friction, or compare with the other unit (SN 19039000821, whose shoulder
encoder was stuck on 2026-10-08).

### Benchmarks: velocity mode (V5) against trajectory mode (V3), and tuning options

**Runs** (`logs/V5`, 2026-10-15 11:56-11:58): `LongBenchmark` and `ShortBenchmark`,
replays of `V3/LongBenchmark` and `V3/ShortBenchmark` at 1:1 with the full V5
settings. **No joint faults** (the same replays faulted the arm three times in V4).
Compared with the trajectory-mode replays of the same sources: `V3/LongBenchmark1`/`2`
and `V3/ShortBenchmark2`/`3`. (`V3/ShortBenchmark1` is a replay of
*LongBenchmark* despite its name.)

| Metric (long / short) | Trajectory (V3 replays) | Velocity (V5) |
|-----------------------|-------------------------|---------------|
| Lag, translation | 160 / 150 ms | **62 / 58 ms** |
| Lag, angles | 224 / 242 ms | **114 / 80 ms** |
| RMS position error, same time | 22.4 / 36.9 mm | **9.7 / 15.3 mm** |
| RMS position error, lag removed | 5.9 / 7.1 mm | **2.6 / 3.7 mm** |
| RMS angle error, same time | 16.7 / 24.3° | **14.5 / 16.9°** |
| RMS angle error, lag removed | **8.6 / 11.5°** | 12.0 / 15.1° |
| Position jitter > 5 Hz (robot) | **0.17 / 0.24 mm** | 0.34 / 0.42 mm |
| Angle jitter > 5 Hz (robot) | **0.10 / 0.12°** | 0.14 / 0.19° |
| Position jitter 5-15 / 15-40 / 40-120 Hz | **0.13 / 0.02 / 0.01** / **0.18 / 0.04 / 0.01** mm | 0.25 / 0.02 / 0.01 / 0.32 / 0.04 / 0.01 mm |
| Wrist current, 45-250 Hz | 0.03 / 0.04 | 0.04 / 0.08 |
| Tool angular jerk p95 | **334 / 485** rad/s³ | 470 / 568 rad/s³ |
| Angular command at the tool cap | 7 / 24% (cap 6.0) | 16 / 26% (cap 2.0) |
| Limiter changing the command | - | 11 / 11% |

**Reading:** velocity mode tracks 2.5× sooner and with less than half the
position error. The wrist current chatter is gone (as quiet as trajectory mode).
It is **not yet as smooth as trajectory mode**: about twice the jitter, all of it in
the 5-15 Hz band (above 15 Hz the two are identical). That is hand motion/tremor
and catch-ups that trajectory mode hides behind its lag. Angle accuracy with the lag
removed is worse because the 2.0 rad/s tool cap clipped 16-26% of these fast
benchmarks.

**Sim tuning** (mock UR16e + `ur_emulator`, replaying the same two benchmarks). The sim
reproduces the real V5 runs closely: 5-15 Hz jitter 0.333 / 0.254 mm (real 0.323 /
0.253), angle error lag removed 15.8 / 12.0° (real 15.1 / 12.0°).

| Variant (short / long) | Angular jerk p95 | Position jitter 5-15 Hz | Lag pos / ang | Error lag removed, pos / ang |
|------------------------|------------------|-------------------------|---------------|------------------------------|
| B0 current: tool cap 2.0, filter 6 Hz | 640 / 486 | 0.333 / 0.254 mm | 68 / 90, 76 / 128 ms | 6.6 / 15.8, 4.0 / 12.0 |
| B1 tool cap 3.0, filter 6 Hz | 752 / 664 | 0.324 / 0.225 mm | 72 / 104, 78 / 124 ms | 8.0 / 9.6, 4.5 / 11.5 |
| B2 tool cap 2.0, filter 5 Hz | 604 / 458 | 0.289 / 0.221 mm | 76 / 94, 84 / 134 ms | 7.9 / 16.0, 4.4 / 11.9 |
| B3 tool cap 3.0, filter 5 Hz | 719 / 515 | 0.289 / 0.198 mm | 78 / 110, 86 / 130 ms | 9.5 / 9.6, 4.7 / 11.2 |

- Raising `TOUCH_MAX_ANGULAR_SPEED` to 3.0 (the joint limiter still caps the wrists at
  2.51 rad/s) cuts the short benchmark's angle error with the lag removed from 15.8°
  to 9.6°, better than trajectory mode (11.5°). The cost is more angular jerk
  (+17-37%), because the limiter's joint caps clip more sharply than the tool cap
  followed by the command filter.
- `TOUCH_COMMAND_FILTER` 6 -> 5 Hz cuts the 5-15 Hz jitter by 13% for 4-8 ms more lag.
- Matching trajectory mode's smoothness completely would need a ~2-3 Hz filter, which
  would bring the lag back towards trajectory mode's. The trade-off is inherent.

**Applied (B3):** `TOUCH_MAX_ANGULAR_SPEED = 3.0`, `TOUCH_COMMAND_FILTER = 5.0`. Expected
on the arm, from the sim: about 80 ms position lag (still about half of trajectory
mode), the lowest 5-15 Hz jitter of the variants, and angle error with the lag
removed of ~10-11°, against 12-15° before.

**Repeatability check** (2026-10-15 12:14-12:15): the first "tuned" replays had run on a
stack launched before the settings change (META: 2.0 rad/s, 6 Hz), so they repeated
the 11:56 runs. They matched within 1%: lag 62 / 116 and 58 / 80 ms (was 62 / 114 and
58 / 80); 5-15 Hz jitter 0.252 / 0.322 mm (was 0.253 / 0.323); angle error with the lag
removed 11.92 / 15.12° (was 11.95 / 15.13°). Benchmark replays are repeatable to
better than 1%, so tuning steps above that are real. Those runs were overwritten by
the tuned ones below.

**Tuned runs on the arm** (`V5/LongBenchmarkTuned`, `V5/ShortBenchmarkTuned`, 12:20-12:21,
META 3.0 rad/s, 5 Hz; no faults). Long / short:

| Metric | Trajectory (V3) | V5 before | **V5 tuned** | Sim predicted (B3) |
|--------|-----------------|-----------|--------------|--------------------|
| Lag, translation | 160 / 150 ms | 62 / 58 ms | **74 / 66 ms** | 86 / 78 ms |
| Lag, angles | 224 / 242 ms | 114 / 80 ms | 118 / 98 ms | 130 / 110 ms |
| RMS angle error, same time | 16.7 / 24.3° | 14.5 / 16.9° | **14.2 / 13.7°** | - |
| RMS angle error, lag removed | 8.6 / 11.5° | 12.0 / 15.1° | **11.2 / 9.2°** | 11.2 / 9.6° |
| Position jitter 5-15 Hz | 0.131 / 0.179 mm | 0.253 / 0.323 mm | **0.197 / 0.278 mm** | 0.198 / 0.289 mm |
| RMS position error, same time | 22.4 / 36.9 mm | 9.7 / 15.3 mm | 11.6 / 17.9 mm | - |
| RMS position error, lag removed | 5.9 / 7.1 mm | 2.6 / 3.7 mm | 3.7 / 6.2 mm | 4.7 / 9.5 mm |
| Tool angular jerk p95 | 334 / 485 | 470 / 568 | 464 / 607 rad/s³ | 515 / 719 |
| Wrist current 45-250 Hz | 0.03 / 0.04 | 0.04 / 0.08 | 0.04 / 0.03 | - |
| Angular command at the tool cap | 7 / 24% | 16 / 26% | 5 / 13% | - |
| Limiter changing the command | - | 11 / 11% | 21 / 29% | - |
| Fastest wrist / acceleration | - | - | 2.53 rad/s / 28 rad/s² | - |

**Reading:** the sim predicted the angle accuracy and jitter almost exactly; the arm
did 12 ms better on lag than the sim. Rotation now matches trajectory mode on the
short benchmark (9.2 vs 11.5°) and is close on the long one. The 5-15 Hz jitter fell
14-22%, leaving it ~1.5x trajectory mode's (was ~1.9x). The cost is 8-12 ms more
position lag and more position error in the fastest twists. With the tool cap
raised, the wrists now reach the limiter's 2.51 rad/s joint cap more often (limiter
active 21-29%). The limiter scales all joints together, so while a wrist is capped
the tool's translation slows too.

**Before the next real-arm try:**

- ~~Fix the joint speed margin first~~: done by `joint_velocity_limiter.py` (speed
  and acceleration caps per joint, see above). Relaunch so it starts.
- ~~Try the Kalman filter alone first~~: done, not enough (see "Real-arm test"
  above). Run with `TOUCH_POSE_SOURCE "joint_states"` and the command filter at
  8 Hz.
- The command filter also applies in trajectory mode and adds the same lag there.
  Set it to 0 for trajectory runs. The Kalman filter can stay on in both modes.
- The command filter delays the P loop's correction. If the arm overshoots, raise
  `TOUCH_LOOP_DELAY` from 0.05 to about 0.08.
- Start at 1:20 scale with slow strokes and the e-stop in reach.
- ~~Add the other V4 suggestions (250 Hz publish, `joint_states` pose)~~: done,
  see the real-arm tests above. `TOUCH_LOOP_DELAY` is still 0.05 (the V4 sim's
  best with a fresh pose was 0.03, but the command filter adds ~28 ms).

---

## V6 (part 1) — Singularity guard

**Code:** tag `v6`. Only the Touch node uses the guard; keyboard,
player and stylus replay are unchanged apart from the keyboard node resetting Servo's
thresholds.

**Why:** strokes ran the arm into singularities (wrist 2 through 0, elbow straight).
MoveIt Servo's own check (`ur_servo.yaml`: slow from condition number 100, halt at
200) has two problems:

1. It scales the *whole* twist, so a stroke with any small component towards the
   singularity stalls completely.
2. Above the hard stop it halts every command, even one leading straight out. On
   the mock arm (UR16e, emulator, velocity mode): driven outward with the guard off,
   Servo stopped with "Very close to a singularity, emergency stop" at c = 554 and
   stayed there whatever was commanded. Raising
   `leaving_singularity_threshold_multiplier` did not release it.

Before it halts, J⁻¹ also turns a small tool speed into a large wrist speed: at
c = 100, a joint moves ~50x faster than the tool's speed suggests, the pattern
behind the V4/V5 wrist overspeed faults.

**Implemented** (`SingularityGuard` in `ls_thesis/ur5e_common.py`, the last stage of
`ur5e_touch_teleop.update`, after the command filter):

- Each cycle the node computes the arm's geometric Jacobian from `/joint_states`
  (`UrdfChain.jacobian`, plain numpy). This is the same matrix Servo checks,
  ~8 at HOME. The guard works in joint space, q' = J⁻¹ v:
  - below `TOUCH_SINGULARITY_SLOW` (30) nothing changes;
  - from 30 to `TOUCH_SINGULARITY_STOP` (60), damped least squares (damping
    rising to `TOUCH_SINGULARITY_DAMPING` 0.05) bounds joint speeds, and the part
    of q' that heads into the singularity is scaled down, all of it gone at 60.
    Motion along or away from it is kept, so the tool slides along the edge
    instead of stalling;
  - above 60, an escape joint velocity straight out (≤ 0.2 rad/s) is added, so
    the arm settles at 60 and backs out if pushed or started past it. It only
    acts while WHITE is held.
- "Into the singularity" is the gradient of m = (Σ σᵢ⁻²)^-½ (≈ σ_min), computed
  analytically from J. The gradient of σ_min itself flips between the two modes
  every cycle where two singularities meet: in a sim this made a 1.36 per-cycle
  twist jump, versus 0.026 with m. At an exactly singular pose (mock hardware's
  all-zero start) the analytic gradient cancels to 0, so the guard then probes
  each joint 0.02 rad one way, scored by Σ log σᵢ.
- The twist goes back to Servo as v = J q', which Servo turns into exactly that q'.
- While the guard runs, the Touch node raises Servo's thresholds to 1e4 / 1e6
  (`SERVO_SINGULARITY_*_GUARDED`), so Servo no longer halts the arm the guard is
  backing out. `joint_velocity_limiter` still caps every joint. The keyboard
  node sets them back to 100 / 200 when it starts.
- Log messages when it engages ("Near the wrist singularity ... slowed", "At the
  elbow singularity limit ... backing out"), with 10% hysteresis. The condition
  number is published on `~/condition_number` and added to `LOG_BAG_TOPICS`.
- Cost: ~0.12 ms per cycle when clear, ~0.2 ms when active.

**Tests:**

- **Kinematic sim** (UR16e URDF, Servo modelled as q' = J⁺v, 500 Hz):
  - Tool driven outward at 0.1 m/s. Unguarded: c reached 29 578 and 1 050 rad/s.
    Guarded: held at c = 60, ≤ 1 rad/s, still moving along the edge.
  - Started at c = 821 with no input: backed out to 60 at 0.2 rad/s.
  - 40 random strokes: c ≤ 67, per-cycle twist change ≤ 0.026, never stuck.
- **Full stack on mock hardware** (`emulate_ur:=true`, velocity mode, private
  ROS domain):
  - Dragged outward from HOME: c went 8.5 → 36 → 60.3 and held while the stylus
    kept pulling; Servo status "No warnings" throughout.
  - From Servo's emergency stop at c = 554: holding WHITE backed out to c = 60
    within ~1 s, then the arm followed the stylus along the edge until Servo's
    collision check slowed it near the table.

**Notes for the real arm:**

- At the reach limit the tool slides along the edge of the workspace: pulling
  further out moves it round the reach sphere, e.g. downwards. Release and
  re-engage to reset the target.
- To compare with Servo's behaviour alone: `TOUCH_SINGULARITY_GUARD = False`
  (the Touch node then sets Servo back to 100 / 200).
- If the guard feels too restrictive, raise `TOUCH_SINGULARITY_SLOW` / `_STOP`
  (e.g. 40 / 80 lets wrist 2 get to ~6° from 0), but keep STOP well below where
  joint speeds blow up (~100+).


## V6 (part 2) — RCM (remote centre of motion) mode

**Code:** tag `v6`, together with the singularity guard. Touch node only (`ur5e_touch_teleop.py`, and
`RcmTool` in `ls_thesis/ur5e_common.py`). Free-motion teleop is unchanged; RCM is a
mode that is switched on and off with the buttons.

**Why:** with the micro module out of action, laparoscopic-style work has to come
from the UR alone. The tool's skinny shaft must pivot about a fixed entry point (a
port), so the arm gives up 2 of its 6 DOF. The tool can only pivot (2), insert, and
roll about the shaft.

**Use:**
- Put the tool tip at the port and hold WHITE + click GREY: the tip becomes the RCM point.
- Hold WHITE + click GREY again to leave. This only works once the tool is pulled back
  to `TOUCH_RCM_EXIT_DEPTH` (3 mm), so the shaft cannot be swept sideways while it is
  inserted.
- GREY on its own still toggles recording. While WHITE is held, GREY is the chord and
  never toggles recording, so nothing has to wait to tell a click from a chord.

**Tool geometry** (`TOUCH_RCM_SHAFT_*`, in the tool0 frame):
- The shaft starts at `SHAFT_START`, runs along `SHAFT_DIRECTION` (any direction, for
  a tilted mount) and ends at the tip `SHAFT_LENGTH` further on.
- The deepest insertion is the length minus `TOUCH_RCM_DEPTH_MARGIN`, so the holder
  never reaches the port.
- The tool lives only in these settings. The pendant TCP stays at zero, so
  `tcp_pose_broadcaster` and the recorder's TCP/TF drift check keep meaning tool0.
- The defaults are placeholders.

**Mapping: the stylus drives the tool tip** (pen twist → roll, pen tilt ignored):
- The tip target is the tip at engage plus the scaled stylus motion. The shaft is aimed
  from the RCM point at that target, and the insertion depth is their distance.
- This is the usual laparoscopic / da Vinci arrangement: the user steers the tip, and
  the fulcrum reversal outside is the arm's problem.
- The catch: right after latching, the tip *is* at the RCM point, so a small sideways
  move would need an unbounded pivot. Below `TOUCH_RCM_MIN_PIVOT_DEPTH` (10 mm; replaced in V7) the
  sideways part is faded out by (depth / 10 mm)². The pivot angle then starts at 0
  and grows smoothly (the largest step in a 0-30 mm sweep was 0.03°), and the tool
  first goes in straight.
- The alternative (pen tilt pivots the shaft) has no singularity, but the tip moves
  against the hand. It was rejected for that reason.

**Control: the twist is built about the RCM point, not tracked per axis.**
- Free mode tracks tool0 position and orientation separately, so any lag would move
  the shaft sideways off the point.
- In RCM mode, the angular velocity ω (feedforward + `rotation_gain` correction to the
  target orientation) and an insertion speed (depth feedforward + `position_gain`
  correction) are turned into tool0's velocity `v = insertion·axis + ω × (p − q)`.
  Here q is the shaft point at the RCM point. A rigid motion like this keeps the shaft
  through the point to first order, however it is scaled.
- Drift is pulled back across the shaft at `TOUCH_RCM_GAIN` (8 /s, capped at
  `TOUCH_MAX_CORRECTION_SPEED`).

Three details mattered (offline kinematic sim, 12 s of fast pivots + insertion + roll,
peak 169°/s):
1. **Speed caps scale linear and angular together** (`cap_together`). Clamping them
   separately, as free mode does, changes the twist's shape and breaks the pivot.
2. **The command filter runs on (insertion, ω), and the lever arm is formed after it
   from the newest pose.** Filtering the finished tool0 twist delayed it by the
   filter's ~45 ms group delay, and by then the lever arm had turned. Result: 5.8 mm
   off the point (centripetal error ~ |ω|² |lever| delay).
3. **The twist is built for the pose `TOUCH_LOOP_DELAY` ahead** (turned about the
   pivot and slid along the shaft), as that is when the arm carries it out. With the
   lever only: 2.6 / 4.2 mm at 30 / 50 ms arm delay. With the full pose prediction:
   0.45 mm when the delay matches `TOUCH_LOOP_DELAY` (50 ms), 1.3 mm at 30 ms and
   2.8 mm at 80 ms. **`TOUCH_LOOP_DELAY` matching the real arm matters more here
   than in free mode.**

**Interaction with other stages:**
- The singularity guard reshapes the twist in joint space and adds a back-off, which
  would move the shaft off the point. In RCM mode only the fraction it lets through
  (`command · guarded / |command|²`, clipped to 0-1) is kept, applied to the whole
  twist. Motion into the singularity is slowed and motion away is free, but the arm
  does not back out on its own. The log says so.
- `joint_velocity_limiter` scales all joints together, so it already keeps the pivot.

**Safety:** if the shaft is ever more than `TOUCH_RCM_MAX_ERROR` (5 mm) off the point,
the robot holds still and RCM mode may be left at any depth.

**Logging** (all added to `LOG_BAG_TOPICS`): `~/rcm_active` and `~/rcm_point`
(latched), `~/rcm_error` (shaft line → point, the accuracy metric),
`~/insertion_depth` and `~/tool_tip` (z = shaft axis, for RViz). Recorded
trajectories are joint paths, so replays reproduce the RCM motion unchanged.

**Tests:**
- **Offline:** tool geometry, `align`, the depth cap, continuity of the min-pivot
  fade, and the closed loop above.
- **Full stack on mock hardware** (UR16e, `emulate_ur:=true`, velocity mode, private
  ROS domain), with a scripted stylus from HOME:

| Phase | Shaft off point (max / mean) | Depth |
|---|---|---|
| Chord on, insert 80 mm | 0.019 / 0.013 mm | 0 → 78 mm |
| 0.5 Hz circles up to 30 mm + ±30° twist | 1.35 / 0.39 mm | 78-86 mm, shaft tilted up to 21° |
| Exit chord at 80 mm | refused ("Retract the tool ...") | - |
| Retract, exit chord | 0.021 mm; exit accepted at 0.1 mm | 80 → 0 mm (1.8 mm overshoot out of the port) |

  The chord never toggled the (running) recorder. There were no Servo or singularity
  warnings.

**Not in this version:**
- Haptic walls. The Touch driver already listens on `/phantom/force_feedback` (N; it
  can only push the tip in x/y/z, ~0.9 N continuous). This is left for its own version
  so its effect can be compared in the logs.
- Calibrating the shaft offset and direction (e.g. a pivot calibration).
- RCM for the keyboard node.

**Notes for the real arm:** measure the tool and set `TOUCH_RCM_SHAFT_*` first. Start
at 1:5 scale with low speed caps and no tool contact, and watch `~/rcm_error`. If it
grows on fast pivots, `TOUCH_LOOP_DELAY` is off from the arm's real delay. Pivoting
about a fixed point uses much more arm motion than free teleop and reaches wrist
singularities sooner, so choose the port position with that in mind.


## V7 — summary of what changed from V6

**Code:** tag `v7` (commit `a43f58e`; builds on `v6`). Dates 2026-10-19 to 2026-10-26.
The sections below have the full detail, measurements and reasoning; this is the overview.

### 1. RCM motion (part 1)

| Change | Why | Detail |
|---|---|---|
| Sideways stylus motion moves tool0 by the same amount at every depth, 0-130 mm (pivot slope k / r(d), `TOUCH_RCM_TOOL0_GAIN` = 0.5); this replaces V6's (depth / 10 mm)² fade | In V6 the arm moved r(d) / d times the tip (~30× at 10 mm deep), so it accelerated hugely for small moves near the RCM point (`V6/BasicInRCM`, `ScalingInRCM`, `ExitingRCM`). Fades were tried first and swung the arm when going in and out | [sideways scaling](#the-sideways-scaling-function), [what was tried](#how-it-was-chosen-what-was-tried-first) |
| The pivot is continuous through 0 mm | A separate branch for targets at or behind the point snapped tool0 back (1-5 m/s jumps on the arm) | [continuous through 0 mm](#continuous-through-0-mm) |
| The tip can retract 5 mm past the point (`TOUCH_RCM_RETRACT_DEPTH`) | So the tool can leave the port before RCM mode is switched off | [retract](#retract-past-the-point) |
| RCM tool = the NDI wand: `TOUCH_RCM_SHAFT_*` from a hand-eye calibration (2026-10-22, redone 2026-10-26 after re-seating the wand) | The placeholder tip was 53 mm short, so the RCM would pivot inside the shaft | [2026-10-22](#tool-calibration-for-the-ndi-wand-2026-10-22), [2026-10-26](#rcm-tool-recalibrated-wand-re-seated-in-the-holder-2026-10-26) |

### 2. Haptic cues on the Touch (part 2)

| Change | Why | Detail |
|---|---|---|
| New Touch driver, `touch_driver.launch.py` = `touch_server` (OpenHaptics only) + `touch_bridge` (ROS only), over UDP; replaces `omni_common`'s `omni_state` | OpenHaptics silently renders no force in any process that loads liblttng-ust / libnuma, which every ROS 2 node does. Position and buttons were unaffected, which is why teleop always worked | [no force in ROS](#no-force-on-the-real-touch-openhaptics-renders-nothing-in-a-ros-process) |
| Cues (`TOUCH_HAPTIC_*`, `ls_thesis/haptics.py`): a tap on RCM on / off, a click crossing the RCM point, a click at the depth limits, and a singularity push from the guard's gradient | So the operator can feel limits and mode changes instead of watching for them; V6's singularity guard silently removed motion | [design](#what-you-feel-while-white-is-held) |
| Iterated on the real Touch: driver damping removed; walls and detent rendered in the Touch's own 1 kHz loop; detent spring → click; spring walls → limit click; strengths cut to 0.002-0.01 N | Shaking and "violent" forces in successive runs | [first run](#first-real-touch-run-too-strong-and-shaking-driver-damping) onwards |
| **Status:** this Touch (SN 22129000559) has faulty force output: any command above ~0.0008 N gives the same fixed kick (staircase test). Cues are on (`TOUCH_HAPTICS = True`) at the tested-range strengths at the user's request; spring walls are off | Hardware, not code. Vendor check: TouchCheckup's Forces page | [staircase test](#run-logsnew_run-1649-clicks-and-walls-feel-maxed-out-and-vibrate), [HAPTICS_HANDOFF.md](HAPTICS_HANDOFF.md) |

The robot's motion is unchanged by part 2: it only adds forces on the stylus.

### 3. Measurement and run tooling

| Change | Why | Detail |
|---|---|---|
| NDI Polaris Vega tracking (`src/ndi_tracking`, standalone): the recorder publishes its state latched on `/trajectory_recorder/recording`, and the tracker records `NDI_*` files into the same run folder and latches its own RCM point with the teleop's | An independent measurement of the RCM error, not from the robot's kinematics | [NDI hook](#ndi-tracking-hook-2026-10-20-alongside-v7) |
| Recording refused unless the NDI tracker is following (`LOG_REQUIRE_NDI`); a dropout mid-run and a missing `NDI_PATH.csv` at stop are logged as errors; `META.yaml` `ndi:` | On 2026-10-22 the tracker stopped unnoticed and 2 originals + 12 replays had no NDI data | [NDI guard](#recording-refused-without-the-ndi-tracker-2026-10-26) |
| `stylus_replay.py` replays the RCM chord (WHITE + GREY), `--rcm` for runs recorded with RCM already on, back-to-back RCM replays, and waits for the tracker | The SimpleHolder runs are RCM runs; before, a replay moved the tool freely | [replays enter RCM](#replays-enter-rcm-mode-2026-10-22) |
| UR16e hard-coded (`ur5e_teleop.launch.py` default, `launch_system.sh`); the default kinematics are UR16e factory values, not the old UR5e calibration file | The lab arm is a UR16e; `launch_system.sh` still passed `ur5e` | [UR16e](#robot-type-hard-coded-to-ur16e-2026-10-26) |
| `plot_paths.py`: mouse-wheel zoom on the 3D path | Close-up checks of small deviations | [NDI hook](#ndi-tracking-hook-2026-10-20-alongside-v7) |
| `thesis/run_tools/rcm_offset_check.py`: teleop vs NDI RCM points and pivots in base_link | To find where the NDI-measured RCM error comes from | `logs/V7/REPLAYS_2026-10-26.md` |

### V7 runs (`logs/V7/`)

| Runs | Date | Notes |
|---|---|---|
| `WeirdJumpsRCM`, `WeirdVibrationSounds` | 2026-10-19 | Before NDI. The jumps came from `ros2 bag record` starting with each recording, not from the RCM mapping ([jumps](#jumps-in-v7weirdjumpsrcm-recording-start-not-the-mapping)) |
| `FastCheckingRCM`, `SlowCheckingRCM`, `RCMOnly`, `WholeRunDrinkBottle` `SimpleHolder` (originals) | 2026-10-22 | Only Fast and WholeRun have NDI data; wand seat / calibration of 2026-10-22 |
| 14 replays of those (`_Replay1-5` / `_Replay1-3`) | 2026-10-26 | All with NDI. NDI RCM error 0.4-2.9 mm rms, while the teleop holds its own point to 0.05-0.23 mm: the gap is mostly UR16e **factory kinematics** error. See `logs/V7/REPLAYS_2026-10-26.md` |

### Open / not done in V7

- The arm's own kinematic calibration is not loaded (`ur_calibration` failed to connect); this is the likely fix for most of the NDI-measured RCM error.
- The bag recorder still starts with each recording (the cause of the `WeirdJumpsRCM` jumps). Planned: start it once, paused.
- "Not run on the real arm yet" in parts 1 and 2 was written on 2026-10-19. Since then the 2026-10-22 / 26 SimpleHolder runs have used the V7 RCM code, but the retract past the point has had no dedicated real-arm test.
- The haptics need a working Touch unit to tune.

## V7 (part 1) — RCM: sideways scaling and entry / exit

**Code:** tag `v7`. Builds on `v6`. Touch node only:
`RcmTool.lever`, `pivot_slope`, `pivot_fade` and `target_shaft` in
`ls_thesis/ur5e_common.py`, `rcm_target` / `engage` in `src/ur5e_touch_teleop.py`,
and the settings below. Free-motion teleop is unchanged.

**Summary of the changes:**
1. **Sideways scaling:** in RCM mode a sideways stylus move now moves tool0 by the
   same amount at every depth (from 0 to 130 mm), instead of the tip following the
   stylus 1:1. This replaces V6's (depth / 10 mm)² fade.
2. **Continuous through 0 mm:** the pivot no longer snaps straight when the target
   depth reaches the RCM point.
3. **Retract past the point:** the tip can be pulled up to 5 mm back out past the RCM
   point, so the tool can leave the port before RCM mode is switched off.

### Why: the lever problem near the RCM point

In RCM mode the shaft pivots about the fixed point c. For a pivot by a small angle θ:
- the tip, `d` past the point (the insertion depth), moves θ · d sideways;
- tool0, on the other side of the point, moves θ · r(d) the other way, where
  r(d) = |tool0 → c| (`RcmTool.lever`; ≈ 0.312 m − d for the 0.262 m shaft that
  starts 50 mm out of the flange).

So the arm moves r(d) / d times as far as the tip: ≈ 30× at 10 mm deep, 9× at 30 mm,
1.4× at 130 mm. In V6 the stylus drove the tip 1:1, so the arm's sideways motion
and acceleration grew as 1 / d near the point, without bound at the point itself.
V6 faded the sideways stylus motion by (d / 10 mm)² below 10 mm, which hid the worst
of it. But the pivot per mm of stylus still peaked at 10 mm (5.7°/mm, as much as with
no fade), and the fade's kink at 10 mm was a step in acceleration. In the V6 real arm
runs (`logs/V6/BasicInRCM`, `ScalingInRCM`, `ExitingRCM`) the arm accelerated hugely
for small moves near the point.

### The sideways scaling function

The stylus still sets a tip target, `tip_origin + translation_scale · Δstylus`
(axis-mapped as in free mode). It is split, relative to the shaft axis `a₀` at
engage, into an axial part (depth along `a₀`) and a sideways part `s` across it.
The shaft is then tilted from `a₀` towards `s` by

  tan θ = p(d) · |s|,  with p(d) = k / r(d)   (`RcmTool.pivot_slope`)

and slid so that the tip sits at the target's axial depth (`target_shaft`). Since
tool0 swings θ · r(d) ≈ k · |s|, **a sideways stylus move of 1 mm moves tool0 by k mm
at every depth**, the opposite way to the tip. Equivalently, the tip only gets a
share k · d / r(d) of the sideways stylus motion:

| Depth | 0 | 10 mm | 30 mm | 70 mm | 130 mm | 160 mm | 200 mm | 240 mm |
|---|---|---|---|---|---|---|---|---|
| tool0 per mm of stylus (k = 0.5) | 0.50 | 0.50 | 0.50 | 0.50 | 0.50 | 0.34 | 0.20 | 0.11 |
| tip per mm of stylus | 0 | 0.017 | 0.053 | 0.15 | 0.36 | 0.36 | 0.36 | 0.36 |
| pivot per mm of stylus | 0.092° | 0.095° | 0.10° | 0.12° | 0.16° | 0.13° | 0.10° | 0.09° |

Properties that made it the choice:
- **No blow-up at the point.** p(d) = k / r(d) is finite at d = 0 (r(0) = 0.312 m),
  so the pivot per mm of stylus stays between 0.09° and 0.16° over the whole range.
  No separate near-point fade is needed: the tip's share k · d / r(d) goes to 0 at
  the point by itself.
- **Going in or out does not swing the arm.** With the stylus held to one side and
  only the depth changing, tool0's sideways offset k · |s| does not depend on d. So
  insertion with a sideways offset moves tool0 along the shaft only (offline: target
  5 mm to the side, tool0 stayed 5.00 mm off from 30 to 130 mm). The fade versions
  below failed exactly here.
- **One smooth function of depth,** with no thresholds or kinks inside 0-130 mm.
- **The stylus target is absolute,** as in free mode: no drift between pen and tool
  builds up, and releasing / re-engaging WHITE starts from the current shaft axis
  (`a₀` is reset at engage, so `s` is 0 and nothing moves).

**Past 130 mm** (`TOUCH_RCM_TOOL0_GAIN_DEPTH`) the tip keeps the share it had at
130 mm (p = share / d), so tool0 moves less than k there. Keeping tool0 at k all the
way would make the pivot per mm grow ~4× by the deepest insertion (242 mm), as the
lever gets short (r = 70 mm). 130 mm was the target range for constant motion, with
some loss accepted past 100 mm.

**k = `TOUCH_RCM_TOOL0_GAIN` = 0.5:** tool0 moves half the (scaled) stylus motion.
k = 1 (1:1) felt too large on the real arm at 1:1 motion scale (tool0 at 100-150
mm/s). Raising k gives the tip more sideways motion at every depth, and keeps
tool0's motion the same at every depth. Insertion (along the shaft) is not scaled by
k: it stays 1:1 with the stylus times the motion scale. k = 0 switches the law off
(the tip then follows the stylus 1:1, with the fade below if set).

### How it was chosen (what was tried first)

| Try | Function of depth d (sideways share of the tip) | Result | Why rejected |
|---|---|---|---|
| V6 | (d / 10 mm)² below 10 mm, then 1 | Big accelerations near the point (real arm, V6 runs) | Pivot per mm peaks at 1 / 10 mm (5.7°/mm); kink at 10 mm (offline: 124 m/s² inserting past a 10 mm side offset) |
| 1 | Smootherstep 10t³ − 15t⁴ + 6t⁵ of t = d / 30 mm | Offline: 5 mm circle at 10 mm depth 4.4 → 1.0 m/s²; the kink case 124 → 0.8 m/s² | Still "moves a lot" near the point (real arm) |
| 2 | Smootherstep³ (`TOUCH_RCM_PIVOT_FADE_STRENGTH` 3) | Keeps 1 / 12 / 49 / 90 % at 10 / 15 / 20 / 25 mm; 0.05 m/s² at 10 mm | Past 30 mm still the plain mapping: arm motion = 9× the tip at 30 mm, falling with depth, so the same stylus motion felt very different at different depths |
| 3 | fade³ × k · d / r(d), k = 1 (constant tool0 past 30 mm) | Offline: tool0 exactly 1.00 mm per mm from 30 to 130 mm | Real arm run (13:39): rough around 30 mm, large past 30 mm (see below) |
| 4 | k · d / r(d), no fade, k = 0.5 | Real arm run (13:46): smooth, except jumps at 0 mm | Jumps from a separate code path at d ≤ 0, fixed below; this is the final function |

The fades all had the same flaw, which try 3 showed on the arm. The stylus target is
absolute, so the sideways motion the fade holds back while shallow comes back as the
tool goes deeper. Moving in and out across the fade then swings tool0 sideways even
with the hand still sideways. In the 13:39 run (depth going in and out between 9 and
47 mm, stylus at 100-160 mm/s), the tool0 target swung at 4-6× the stylus speed at
15-25 mm. Offline, the same kind of motion (10 ↔ 45 mm at 0.7 Hz with a 30 mm
sideways sweep) gave 100 mm/s of tool0 sideways motion for a 24 mm/s stylus with the
fade, and 24 mm/s (k = 1) or 12 mm/s (k = 0.5) without it. Peak acceleration went
from 1.34 to 0.34 m/s², and what is left is the 1:1 insertion.

The fade code is kept as an option (`TOUCH_RCM_PIVOT_FADE_DEPTH` > 0, shape
smootherstep ^ `TOUCH_RCM_PIVOT_FADE_STRENGTH`). It multiplies the slope, so it can
make entry straighter, at the cost of the in/out swing above.

### Continuous through 0 mm

In the 13:46 run (k = 0.5, no fade) the motion was smooth except at 0 mm. The tool0
target leapt at 1-5 m/s (2-15 rad/s) for one or two samples each time the target
depth crossed 0 (t = 6.3, 7.4, 7.5, 10.9, 11.6, 12.1, 13.4, 22.8, 23.4 s), and stayed
below 0.25 m/s everywhere else. With the k / r(d) law the pivot angle tends to a
finite value as d → 0 (tan θ = k · |s| / r(0)). But a target at or behind the point
took a separate branch that put the shaft straight back on `a₀`, which snapped tool0
back by k · |s| in one step (5 mm for a 10 mm sideways offset).

Fix: `target_shaft` now always tilts the axis by `pivot_slope`, which is held at its
d = 0 value for targets behind the point, so the pivot is continuous through 0. For
targets past the point it is the same mapping as before (checked numerically).
Offline, with the stylus 20 mm to the side and the depth going ±4 mm about the point,
the largest tool0 step went from 11 mm in one 2 ms sample (5.5 m/s, as on the arm) to
0.03 mm. Side effect: retracted to the port, the shaft stays tilted by the sideways
pen offset (3.7° for 40 mm at k = 0.5) instead of snapping straight.

### Retract past the point

The depth used to stop at 0, so the tip could only come back to the RCM point, not out
of the port, before leaving RCM mode. It may now be pulled up to
`TOUCH_RCM_RETRACT_DEPTH` (5 mm) back out past the point (`target_shaft`'s
`min_depth`, also the lower limit when re-engaging). The shaft line still runs
through the point out there. Leaving still needs depth ≤ `TOUCH_RCM_EXIT_DEPTH`
(3 mm), which any retracted depth meets.

### Settings

| Setting (parameter) | Value | Meaning |
|---|---|---|
| `TOUCH_RCM_TOOL0_GAIN` (`rcm_tool0_gain`) | 0.5 | tool0 sideways motion per unit of scaled sideways stylus motion; 0 = off |
| `TOUCH_RCM_TOOL0_GAIN_DEPTH` (`rcm_tool0_gain_depth`) | 0.13 m | deepest the constant-tool0 law holds to |
| `TOUCH_RCM_PIVOT_FADE_DEPTH` (`rcm_pivot_fade_depth`) | 0 (off) | optional extra fade near the point; replaces V6's `TOUCH_RCM_MIN_PIVOT_DEPTH` |
| `TOUCH_RCM_PIVOT_FADE_STRENGTH` (`rcm_pivot_fade_strength`) | 1 | power on the fade's smootherstep (≥ 1) |
| `TOUCH_RCM_RETRACT_DEPTH` (`rcm_retract_depth`) | 0.005 m | how far the tip may come back out past the point |

### Tests

- **Offline** (`RcmTool` with the 0.262 m shaft): the tables above; the law gives 1.00
  (k = 1) / 0.50 (k = 0.5) mm of tool0 per mm at 0-130 mm; same mapping as before past
  the point; the 0 mm crossing; the node's own `rcm_target` called directly (2.00 mm
  of tool0 for a 2 mm stylus move at 70 mm depth, k = 1; smooth pull-out through 0).
- **Mock arm** (UR16e, `emulate_ur:=true`, from the `V7/WeirdJumpsRCM` start pose,
  scripted stylus): RCM on, insert 40 mm (depth 40.3 mm), 15 mm sideways (depth
  unchanged), stylus pulled ~10 mm past the point: depth stopped at −5.2 mm (−5.7 mm at
  most), exit chord accepted. Shaft line to the point: 0.01 mm at most.
- **Real arm:** the 13:39 and 13:46 runs above, recorded to `logs/NEW_RUN` and not
  kept (overwritten by later runs), and `logs/V7/WeirdJumpsRCM`. The final code
  (continuity fix, retract) has not been run on the real arm yet.

### Jumps in `V7/WeirdJumpsRCM`: recording start, not the mapping

About 1.73 s into the recording, no joint velocity command reached the arm for over
50 ms. The Touch node's watchdog zeroed the command ("MoveIt Servo stopped sending
joint velocities mid-motion"), so the arm braked (6.5 m/s², 100 → 16 mm/s) while the
target ran on. Once commands flowed again it was 37 mm behind and lurched to catch up
(6.8 m/s², with the velocity limiter clipping). The target itself was smooth (depth
59-99 mm). The session log has this error 1.8-2.9 s after 3 of 5 recording starts and
never outside a recording, while `ros2 bag record` (started fresh by each GREY press)
is still subscribing to its ~20 topics. The laptop has no real-time scheduling, so
that start-up competes with Servo and the limiter. Also running then, on another ROS
domain: leftover mock-test Servo / ros2_control / move_group processes (since
stopped), which added CPU load. Planned fix: start the bag recorder once, paused, and
have GREY pause / resume it. Until then, hold still for ~3 s after starting a
recording.

### Tool calibration for the NDI wand (2026-10-22)

**What:** `TOUCH_RCM_SHAFT_START / _DIRECTION / _LENGTH` set to the NDI wand (probe
8700340) mounted on the flange as the tool: START (−0.6, 6.6, 206.2) mm, DIRECTION
(−0.0061, 0.0148, 0.9999) (0.92° off tool0 z), LENGTH 158.6 mm, so the tip is at
(−1.56, 8.97, 364.85) mm in tool0. Was the placeholder [0, 0, 0.05] / [0, 0, 1] / 0.262
(tip at 312 mm).

**Why:** the placeholder tip was 53 mm short of the real one (and 9 mm off-axis), so RCM
mode would latch and pivot about a point inside the shaft instead of the tip, and the NDI
would see that as RCM error.

**How:** no divot could be used, so `ndi_tracking robot-tcp --wand-is-tool` was only used
to record (wrist 1/2/3 jogged one at a time, 90 s, `calibrations/raw/robot_tcp_20261022_092412.npz`);
its printed result assumes a divot and is invalid. A hand-eye fit alone gives tool0 → wand,
and the wand's own markers → tip calibration (2026-10-20, 0.93 mm pivot rms, kept in
`calibrations/wand_only_8700340_2026-10-20/`) maps the tip and axis into tool0. Check: the
tip through the robot's kinematics vs. the tip seen by the camera agree to 1.05 mm rms
(max 2.2 mm) over 0.5 m of travel; the two halves of the recording give tips 0.5 / 0.95 mm
from the full fit. The hand-eye "camera pose spread" (9.6 mm) is 0.5° of rotation noise
times the ~1.4 m camera distance, not tip error. LENGTH is the wand's tip → first-marker
distance along the shaft (not measured with a ruler); deepest insertion = 158.6 − 20 mm
margin = 139 mm. Result in `calibrations/robot_tcp_handeye_2026-10-22.json`.

### Replays enter RCM mode (2026-10-22)

**What:** `stylus_replay.py` replays the RCM chord: a GREY press that starts while
WHITE is held is sent (from the run's `STYLUS_RAW.csv` `grey` column), until GREY is
released. GREY on its own (recording start / stop) is still masked. The replay also
refuses to start while the teleop's latched `rcm_active` is on, and warns if RCM is
still on at the end (an exit chord refused).

**Why:** the 2026-10-22 runs (`V7/*SimpleHolder`) are RCM runs. Before, a replay never
entered RCM mode, so the same stylus motion moved the tool freely instead of about the
port: not a repeat of the run, and with the tool in the holder a likely collision.

**Tests:** offline, the chords come out at 35.34 / 92.65 s
(`WholeRunDrinkBottleSimpleHolder`, NDI rcm_set / rcm_clear at 35.37 / 92.67 s) and
2.89 / 31.69 s (`FastCheckingRCMSimpleHolder`); no GREY without WHITE is sent. Mock arm
(UR16e, `emulate_ur:=true`, domain 77, `--no-record`), replay of
`FastCheckingRCMSimpleHolder`: RCM ON then OFF 28.80 s apart (original 28.80 s), exit
accepted, no warnings beyond the usual controller switching. Not yet run on the real arm.

**Runs recorded with RCM mode already on (`--rcm`, 2026-10-22):** `V7/RCMOnlySimpleHolder`
was recorded with RCM already on (latched before GREY), so it has no chord to replay; its
`_Replay1` ran free. `stylus_replay.py --rcm` turns RCM on at the start pose (WHITE + GREY
with the stylus still, before the recorder starts), so the RCM point is the tip there. For
this run the tip was 0.56 mm past its RCM point at the start (shaft 0.01 mm off it), so
the replay pivots within ~0.5 mm of the original point (mock: (0.6447, 0.2262, 0.2443) m vs
(0.6446, 0.2261, 0.2448) m). The replay warns when the bag's first `rcm_active` and `--rcm`
disagree.

The run ends with the tool 100 mm in, so the teleop (correctly) refuses to leave RCM mode
after it, and with no Touch in replay mode only a stack restart cleared it. Replays can now
run back to back: at the end the replay tries to leave RCM mode (works if the tool is
out); if it is still on at the next replay's start, the robot moves straight to the start
pose (joint move, tool still in - fine for the open simple holder) and leaves RCM mode
there, where the tip is back at the latched point. A rewind of the last recording (to
retract along the shaft first) was tried and dropped: not needed for this holder, and it
stopped 3.4 mm deep against the 3 mm exit limit because the latched point differs from the
original's by 0.5 mm. Mock: three `--rcm` replays of `RCMOnlySimpleHolder` back to back, each
left RCM at the start pose and latched the same point.

### Not in this version (yet)

- The paused bag recorder above.

## V7 (part 2) — Haptic cues on the Touch

**Code:** tag `v7`, part of V7 (with part 1). Current state and next steps:
[HAPTICS_HANDOFF.md](HAPTICS_HANDOFF.md); this section is the history, in order (the first
design below was changed several times, see the later subsections). New
`ls_thesis/haptics.py`; `SingularityGuard.twist_gradient` in `ls_thesis/ur5e_common.py`;
`update_haptics` and helpers in `src/ur5e_touch_teleop.py`; `TOUCH_HAPTIC_*` settings.
**The robot's motion is unchanged**: these only add forces on the stylus.

### What you feel (while WHITE is held)

| Cue | When | Force |
|---|---|---|
| Depth walls | RCM mode, the stylus target past the deepest insertion (shaft length − `TOUCH_RCM_DEPTH_MARGIN`) or past the retract limit (`TOUCH_RCM_RETRACT_DEPTH`) | One-sided spring-damper along the stylus direction that drives the depth: 250 N/m of stylus penetration + 2 N s/m moving in; never pulls |
| Detent | RCM mode, the tip target crossing the RCM point | Slope of a Gaussian well, pulling towards depth 0: 0.3 N at most, at ±2 mm of stylus; nothing past ~8 mm |
| Buzz | RCM on (one 120 ms), off (two 60 ms), refused because the tool is still in (three 40 ms) | 0.4 N, 40 Hz, up / down, Hann-windowed so it starts and ends at 0 |
| Singularity push | Condition number above `TOUCH_SINGULARITY_SLOW` | Along the stylus direction that moves the arm out: 0.8 N × the guard's `limiting` (0 at slow, 1 at stop), + 3 N s/m × limiting on stylus motion heading in; 30 ms low-pass |

The sum is capped at 1.5 N (the Touch gives 3.3 N peak, 0.88 N continuous) and slew-limited
to 200 N/s, because a step in force can trip the Touch's max-force-impulse check.
Wall and detent distances are in stylus metres (penetration / motion scale), so they feel
the same at 1:1, 1:5 and 1:20.

### Why these designs

- **Walls and detent are placed on the stylus target, not the robot.** The target's depth
  is computed without the limits (`target_shaft` with ±inf), so pushing past a limit,
  where the target stops, presses into the wall. Being on the hand side, they feel the same
  however far the arm lags and form no loop through the arm, so they cannot make it
  oscillate.
- **Singularity direction: the guard's own gradient, mapped to the stylus.** The guard
  already computes g, the joint-space gradient of m ≈ s_min. Through its damped inverse,
  dm/dt = g · J_dls⁺ v = ((J_dls⁺)ᵀ g) · v, so `twist_gradient` = (J_dls⁺)ᵀ g is the twist
  direction that leads out. That is mapped to the stylus with the 6 × 3 stylus → tool0 twist
  map: [scale · axis_matrix; 0] in free mode, and numerically through `rcm_target` in RCM
  mode. Only the translation is used: the Touch has no torque output.
- **A field, not just damping:** the push acts even when the pen is still, so at the edge
  the hand is held back with a force that grows as it goes in, like a soft wall. The
  damping term only resists motion heading in.
- **The driver keeps the last force it was sent** (`omni_state.cpp`, `force_callback`), so
  the node sends zero at start (clearing a force left by a node that died) and on
  Ctrl-C / SIGTERM. rclpy's own signal handler is off so the context is still up to send it.
  A `kill -9` still leaves the last force on, until the next teleop start.

### Tests

- **Offline** (UR16e URDF): 150 random poses with 32 < c < 120 (55 elbow, 56 shoulder,
  39 wrist). In all 150, a 1 mm tool move along the push lowered c and one against it
  raised it, and the push beat 97-99% of random directions. Shapes: wall 0.5 N at 2 mm,
  0 moving out; detent ±0.30 N at ±2 mm; buzzes peak 0.39 N, ≤ 0.2 N per 2 ms.
- **Mock arm** (UR16e, `emulate_ur:=true`, scripted stylus, 1:1):
  - RCM: on → one buzz; stylus 20 mm out → tip stopped at −5.3 mm and the wall pushed
    back in (1.5 N cap); back in through the point → the detent pulls towards 0 from both
    sides (±0.29 N); exit at 29 mm → refused + three buzzes; at the point → off + two
    buzzes.
  - Singularity: stylus 700 mm away from the base (elbow) → push towards the base from
    c = 30, 0.8 N + damping (1.06 N while still pushing in), 0.6 N held still at c ≈ 54,
    0 on release.
  - Ctrl-C of the node while pushing 1.12 N → its last message was zero.
  - Largest step between messages 0.38 N (limit 0.4).
- **Real Touch:** not run yet. Stiffness / damping are first guesses for the ~2-4 ms ROS
  delay; if a wall buzzes, lower `TOUCH_HAPTIC_WALL_STIFFNESS` or raise its damping.

### No force on the real Touch: OpenHaptics renders nothing in a ROS process

On the real Touch (SN 22129000559) no cue was felt. A recording showed the walls being
sent correctly (0.9 N back towards the port at the −5 mm limit), and `omni_state` passed
them on: under gdb its `force_callback` got them and `hdSetDoublev(HD_CURRENT_FORCE)` was
called with them (0, 1.000, 0 N) at 1 kHz, with HD_FORCE_OUTPUT enabled, calibration OK
and no HD errors. The pen still did not move. Lift test: the pen resting free, a ramped
1 N upward force, rise measured from the encoders:

| Program | Rise |
|---|---|
| Plain OpenHaptics program (HelloHapticDevice's call order) | 148-165 mm |
| Same, with `omni_state`'s exact HD calls, order and calibration | 151-154 mm |
| Same, with the force applied 3 s after start | 154 mm |
| `omni_state`, force over ROS | 0 mm |
| `omni_state` copy with a fixed force, ROS never initialised | 0 mm |
| Plain program **linked** against librclcpp / librcl (never called) | 0 mm |
| … against librcutils, librmw_implementation, spdlog, yaml, fmt | 155-160 mm |
| … against liblttng-ust, libtracetools or libnuma | 0 mm |
| Plain program with only libnuma `LD_PRELOAD`ed | 0 mm |

So OpenHaptics (3.4, driver Dev_Dec 10 2025) silently stops rendering forces in any
process that has liblttng-ust or libnuma loaded, and every ROS 2 node loads both
(rcl → tracetools → lttng-ust → libnuma). The servo loop still runs at 1 kHz and every
query looks normal. The position and buttons were never affected, which is why
teleoperation always worked. No workaround inside one process was found: a stub
libnuma (no constructor) still failed through lttng-ust, and preloading the Touch
libraries first did not help.

**Fix:** the device runs in a process without ROS. `touch_server` (`src/touch_server.cpp`,
links only HD / HDU) runs `omni_state`'s servo callback unchanged: same axes, mm, velocity
filter, damping and calibration. `touch_bridge` (`src/touch_bridge.cpp`) publishes
`omni_state`'s topics and messages and forwards `/phantom/force_feedback`. They talk over
UDP on localhost (`include/ls_thesis/touch_packets.hpp`). `launch/touch_driver.launch.py`
starts both, and `ur5e_teleop.launch.py` (input:=touch) uses it instead of `omni_state`.
Through it: `/phantom/state`, `/joint_states` and `/pose` at 1022 Hz, and the 1 N lift over
ROS rose 168-177 mm. One change from `omni_state`: a force not refreshed for 0.25 s drops
to zero (omni_state kept the last one, so a crashed sender left it on). The teleop sends
every cycle while a cue is active, so this does not cut cues short.

Before this was found, the Touch config files were also replaced: `~/.3dsystems/config`
had been written for the other unit (SN 19039000821). `Touch_HeadlessSetup` was rerun for
SN 22129000559 (the old files are in `~/.3dsystems/config.backup-SN19039000821-2026-10-19`).
After that and a power cycle, TouchCheckup's Box test felt right. Whether the config
mismatch did anything is not known: the ROS-process problem above blocked forces either way.

### First real-Touch run: too strong and shaking (driver damping)

The first run with `touch_server` (16:19, motion scale 1:40) felt "way too strong and
vibratory". The teleop only ever sent the RCM detent (≤ 0.30 N, zero most of the time).
But while any force was fresh, `touch_server` also subtracted `omni_state`'s damping,
0.001 N per mm/s of its 20 Hz-filtered velocity: up to 2.0 N (median 0.5 N). Its filter
lags so much near 25 Hz that it pushed along the motion. The pen's 15-40 Hz shake was ~70×
higher while forces were fresh than before or after, at up to 1.7 m/s (vs 0.37 m/s).
`omni_state` always had this term; it was never felt only because its forces never
reached the motors.

Changes: the damping is removed from `touch_server` (the force sent is the force
rendered). The teleop's own wall / singularity damping now uses the Kalman pen
velocity (no lag) instead of the driver's. Defaults are softened: cap 1.5 → 1.0 N, wall
250 → 150 N/m and 2 → 1 N s/m, detent 0.3 → 0.15 N, buzz 0.4 → 0.3 N, singularity
0.8 → 0.5 N and 3 → 1.5 N s/m.

The RCM on / off buzz (0.4 N, 40 Hz, up to three bursts) was still "way too big and too
much": it is now a single 60 ms Hann-shaped push of 0.1 N (two when leaving is refused),
`TOUCH_HAPTIC_PULSE_FREQUENCY` = 0 (> 0 brings the buzz back).

### Walls and detent rendered in the Touch's own loop

Next run (16:25): "violent, pulls it out of my hand". Right after RCM on, the detent flipped
sign every 20-40 ms while the pen jumped 10-20 mm per sample (up to ~7 m/s), and the walls
hit the 1.0 N cap as it swung past the limits. These cues are springs on the pen's position,
and they went round Touch → bridge → teleop (Python, 500 Hz) → bridge → server. That trip
is tens of ms, long enough for even a 0.15 N detent to push at the wrong moment and grow.

Fix: the teleop now sends only the shape (`TouchScenePacket`, UDP straight to
`touch_server`): the insertion direction in stylus axes, the stylus position at engage, the
detent centre and the wall limits along it (stylus mm), stiffness, detent size and the
force cap. `touch_server` renders walls and detent from the current position on every 1 kHz
tick, with no delay, adds the slow cues still sent over `/phantom/force_feedback`
(singularity push, taps), then caps the total (≤ 2 N hard) and slew-limits it (0.2 N per
tick). The scene is dropped 0.25 s after the last packet. The wall damping is gone (its
velocity was the delayed part). `/phantom/applied_force` now logs what was actually rendered.
Mock arm, with a stand-in renderer on a test port (the scene goes over UDP, which ROS
domains do not isolate: tests must set `haptic_scene_port` away from 47102): pen 20 mm out →
wall from −5 mm, pushing back in up to the 1.0 N cap; detent ±0.15 N at ±2 mm, pulling
towards the port; no scene after RCM off.

### Detent spring replaced by a click

It still shook at the port (16:34 run: ±0.15 N rendered by `touch_server`, pen oscillating
~±8 mm at 20-30 Hz in the detent zone only, calm elsewhere). Comparison by hand: the
OpenHaptics HelloHapticDevice gravity well (an attracting spring, 0.075 N/mm) also shook,
while TouchCheckup's Box and Sphere (one-sided walls: free inside, pushed back only when
pressing into one) were smooth on the same device. So the device and `touch_server` are
fine. An attracting spring held loosely in the hand, with nothing to damp it, keeps
overshooting its centre, and the detent was such a spring.

The detent is no longer sent (`touch_server` can still render one). Instead the teleop
plays a 40 ms, 0.1 N tap (`TOUCH_HAPTIC_CROSSING_FORCE`) against the direction of travel
when the tip target crosses the RCM point, with ±1 mm of stylus hysteresis
(`TOUCH_HAPTIC_CROSSING_HYSTERESIS`). Taps are open loop: they don't depend on where the pen
is, so they cannot oscillate. The walls stay as they are (Box-like). Mock arm: a click
(+z, against the downward push) on crossing inwards; 0 N within ±5 mm of the point; walls
up to the 1.0 N cap. Leaving the point right after RCM on does not click (the target starts
at the point and has not crossed it).

### Run `logs/NEW_RUN` (16:49): clicks and walls feel maxed out and vibrate

The first run with `/phantom/applied_force` in the bag. `touch_server` rendered what was asked:
0.1 N clicks and walls up to the 1.0 N cap. But every 0.1 N, 40 ms click (1.33, 3.95, 4.55 s)
set the pen shaking ~±10 mm at 20-45 Hz, up to 2 m/s, mostly through the elbow joint (±3°).
Before each click it moved smoothly (0.2 mm high-pass). It felt "maxed out". The tests
below show the device is fine: the Touch arm is light and nearly balanced, so a loosely held
pen follows even small forces, and these cues were much too strong for it. Two things kept the shake going:

- **Clicks retriggered themselves.** The shake carried the target back across the ±1 mm
  hysteresis, and each pass fired another click 40-60 ms later (1.33-1.68 s, 3.1-3.3 s).
  Fix: `TOUCH_HAPTIC_CROSSING_HOLDOFF` (0.3 s) between clicks; the side is still tracked.
- **The retract wall was within reach of the shake.** At scale 0.5 the −5 mm limit is
  10 mm of stylus from the point, so the shake hit the wall: 0 ↔ 1 N at ~22 Hz from 1.6 to
  2.9 s and 4.0 to 4.25 s.

Not the cause: the missing `Default Device.config`. For this HID Touch, `Touch_HeadlessSetup`
deletes `Default Device.config` and `Device Calibration` itself (strace) and keeps the config
on the device. OpenHaptics reports Touch, SN 22129000559, 3.3 N max (0.88 N continuous),
0.5 N/mm max stiffness, servo at 1020 Hz: all as expected.

Bare-device taps (`touch_server` stopped; a plain OpenHaptics program, 40 ms sin² pulses straight
up, pen held loosely, 1 s apart). The pen started moving 15-29 ms after each pulse (too soon for
the hand), 40-100° off vertical, mostly towards the operator:

| Pulse | Felt | Pen motion |
|---|---|---|
| 0.1 N | "a huge amount of force" | 15-19 mm, ringing ~37 Hz (held firmer) |
| 0.01 N | "much better" (held less tightly) | 43-59 mm, smooth |
| 0.001 N | "even softer", odd direction | 12-33 mm |

Repeats were inconsistent: 0.001 N moved the pen 0-2 mm the second time, and 0.01 N moved it
14-52 mm. The motion depends mostly on the grip, so feel is the better guide: 0.0005 N and
0.0001 N were not felt. Hands-off force check: a slow upward ramp from the inkwell lifted the
pen at 0.20 N, a normal value (an earlier run started outside the inkwell, where the arm was
nearly balanced, and lifted at 0.0009 N; that run is invalid). The docked calibration check
was fine too: status OK, inkwell at (0, -65.5, -88.1) mm. Then, pen lying on the table, the same ramp
lifted it at 0.0007 N: no motion at or below 0.0005 N, then within ~15 ms ~0.5 m/s, coasting 44 mm,
mostly towards the operator, after the force was back at 0 (it needs ~1-2 N). So a command
just past a tiny threshold gives a large kick in an odd direction, in bare OpenHaptics. Suspected
cause, not proven: a big fixed offset per motor once the command passes the threshold (like a
friction or deadband compensation set far too high). That would also explain the wall chatter.
All strengths were cut ~10x: cap 1.0 → 0.1 N, walls 150 → 15 N/m, crossing click and RCM tap
0.1 → 0.01 N, singularity push 0.5 → 0.05 N. The off-vertical motion (mostly towards the operator)
matches the table run, so it is likely the device, not the hand. Next: a staircase of constant
forces on the table, then Touch_AdvancedConfig / the driver / 3D Systems support.

Staircase test, pen on the table, hands off, bare OpenHaptics, force straight up. A slow ramp
at 0.0001 N/s first moved the pen at 0.00084 N. Then steps, each from rest:

| Step | 10 mm after | Path |
|---|---|---|
| 0.001 N | 21 ms | up + towards the operator |
| 0.002 N | 21 ms | same |
| 0.005 N | 22 ms | same |
| 0.01 N | 23 ms | same |
| 0.03 N | 20 ms | reversed: down + away |

Over 10x of commanded force the kick is the same, and at 0.03 N it reverses. So this Touch's
force output is faulty: any command above ~0.0008 N gives a fixed, violent kick that does not
follow the command. No setting in our code can fix it, and every wall or click passes through
the threshold. TouchCheckup also sets forces with hdSetDoublev, and its Forces page (sliders
that set the force directly) is the vendor-side check. Haptics should stay off
(`TOUCH_HAPTICS = False`) until the unit is repaired or replaced.

**Strengths set to the tested range (user's request).** The user wanted the cues to work at the
forces found in testing: nothing below ~0.0008 N, 0.001 N soft but felt, 0.01 N about as much as
is comfortable. Cap 0.1 → 0.01 N, walls 15 → 2 N/m (the cap 5 mm of stylus into the wall),
crossing click and RCM tap 0.01 → 0.002 N, singularity push 0.05 → 0.005 N with damping 1.5 →
0.015 N s/m, slew 200 → 2 N/s (0.01 N in 5 ms). Caveat: the staircase test showed the same kick
for every step from 0.001 to 0.01 N, so on this unit the levels may feel alike. The walls and
clicks still pass through the ~0.0008 N threshold.

**Run 17:28: walls replaced by a limit click.** With the tested-range strengths the shake dropped
from 10-20 mm to ~2 mm. What was left was all at the retract wall, while its force sat at
0.0005-0.0015 N: across the Touch's ~0.0008 N kick threshold, so the kick flicked on and off.
Pressing further in, to the 0.01 N cap, shook 12-16 mm. Any spring wall starts from 0 N, so on
this unit it always buzzes at its edge. The walls are now off (`TOUCH_HAPTIC_WALL_STIFFNESS = 0`),
and `check_limits` plays one 40 ms, 0.002 N click back out of the limit
(`TOUCH_HAPTIC_LIMIT_FORCE`) when the tip target reaches the deepest insertion or the retract
limit. It uses the crossing click's hysteresis (1 mm of stylus) and 0.3 s hold-off. The arm already
stops at the limits by itself. Offline check: one click entering each limit, pushing back the right way;
none from ±0.5 mm jitter at the edge. The META.yaml of that run lists the old strengths: the
recorder (terminal 1) had loaded the settings before they changed. The teleop used the new ones
(max sent 0.002 N, max rendered 0.01 N).

## NDI tracking hook (2026-10-20, alongside V7)

**What:** `trajectory_recorder` publishes its recording state latched on
`/trajectory_recorder/recording` (`std_msgs/String`, JSON `{"recording", "run_dir",
"start_unix"}`): once when it starts up, once on start, once on stop. Stop is published
*before* `close_logs` (bag shutdown, `LOG_KEEP_ALL_RUNS` copy).

**Why:** the Polaris Vega VT (NDI optical tracker) measures the tool independently of the
UR's kinematics, to check that the shaft stays on the RCM point. The tracker
(`src/ndi_tracking`, standalone, to become its own repo) follows this topic to record
`NDI_PATH.csv` / `NDI_EVENTS.csv` / `NDI_META.yaml` (RCM error metrics) / `NDI_PLOT.png`
into the same `NEW_RUN` folder, with `t` on the recorder's clock. It also follows the
existing `/ur5e_touch_teleop/rcm_active`: RCM on latches the NDI-measured tool tip as the
RCM point, the same moment the teleop latches its own. Stop goes out early so the NDI
files are finished before the folder is copied.

**Teleop behaviour:** unchanged (one extra latched publisher). Tested on a private ROS
domain: GREY start/stop and RCM on/off reached the tracker, NDI files landed in NEW_RUN
next to the recorder's own.

**Plots (2026-10-20):** `thesis/run_tools/plot_paths.py` 3D path figure zooms with the mouse wheel
(`scroll_zoom_3d`, about the box centre, all axes equally so the scale stays equal).
Matplotlib's 3D axes only zoom with a right-drag, which was awkward for close-up checks of
small deviations. Plotting only; no effect on runs.

## Robot type hard-coded to UR16e (2026-10-26)

**What:** `launch/ur5e_teleop.launch.py` now defaults `ur_type` to `ur16e` (was `ur5e`).
`scripts/launch_system.sh` sets `UR_TYPE="ur16e"` and passes it in every robot mode
(`ur5e`, `ur5e-keyboard`, `ur5e-touch`, `full`); its default `--calibration-file` is now
empty, so the UR16e factory kinematics are used unless a calibration file is given.

**Why:** the lab arm is a UR16e (dashboard: UR16, SN 20225600118, PolyScope 5.12.1), but
`launch_system.sh` hard-coded `ur_type:=ur5e` and the old UR5e's
`lab_ur5e_1_calibration.yaml`. That is the same mismatch behind the V3 replays running
UR5e kinematics on the UR16e. Passing `ur_type:=ur5e` explicitly still works for the old arm.

## Recording refused without the NDI tracker (2026-10-26)

**What:** `LOG_REQUIRE_NDI = True` (teleop_settings.py). `trajectory_recorder` refuses to
start a recording (GREY button or replay) unless the `ndi_tracking` node is subscribed
to `/trajectory_recorder/recording`. During a recording it checks every second and logs an
error the moment the tracker drops out. At stop it checks for `NDI_PATH.csv` in the run
folder and adds a WARNING to the stop message if the file is missing or the tracker
stopped part-way through. `META.yaml` gets `ndi: {tracker_at_start, lost_at, files}`.
`stylus_replay.py` makes the same check before the arm moves, so a replay without the
tracker exits straight away rather than after the approach move. With
`LOG_REQUIRE_NDI = False` it records anyway, with errors at start and stop.

**Why:** on 2026-10-22 the tracker stopped following at ~09:51 and nothing said so: the
SlowChecking and RCMOnly originals and all 12 replays (Fast x5, Slow x3, WholeRun x3,
RCMOnly x1) had no NDI data. The 12 replays were moved to the Trash at the user's request
(originals kept) and are to be redone with the tracker running.

**Tests:** private ROS domain, recorder logging to a scratch folder, a stand-in
`ndi_tracking` node: no tracker → refused; tracker up → recorded, no warning; tracker
killed mid-run → error at 2.1 s, stop warning, `META.yaml ndi.lost_at: 2.1`.

## RCM tool recalibrated: wand re-seated in the holder (2026-10-26)

**What:** `TOUCH_RCM_SHAFT_START` [-0.0006, 0.00662, 0.20624] → [0.00008, 0.00699, 0.20827] m,
`TOUCH_RCM_SHAFT_DIRECTION` [-0.00607, 0.01481, 0.99987] → [-0.00309, 0.01798, 0.99983];
LENGTH unchanged (0.1586 m, wand geometry). Tip in tool0 (-0.41, 9.84, 366.87) mm, 2.5 mm
from the 2026-10-22 tip (mostly +2.0 mm along the shaft); shaft 1.05° off tool0 z (was 0.92°).

**Why:** the same wand went back in the same holder for the replay re-runs, seated slightly
differently. Same method as 2026-10-22: `robot-tcp --wand-is-tool -d 60`, wrist 1/2/3 jogged
one at a time, no divot; tip and axis from the wand's own calibration through the hand-eye
fit (the robot-pivot numbers robot-tcp prints are invalid without a divot). Fit quality:
rotation mismatch 0.42° rms (was 0.49°), camera-pose spread 8.1 mm / 0.34° (was 9.6 / 0.47),
tip check against the camera 0.82 mm rms / 1.95 max (2026-10-22 data, same check: 1.15 / 2.61).
Recomputing 2026-10-22's raw data with the same script reproduced its settings exactly.
Record: `src/ndi_tracking/calibrations/robot_tcp_handeye_2026-10-26.json`.

**Fix (same day):** the replay's tracker check ran ~1 s after its node started, before DDS
discovery had found the `ndi_tracking` node (1.0-2.6 s on the live stack), so replays were
refused with the tracker running. `stylus_replay.py` now waits up to 5 s for it
(`wait_for_ndi_tracker`). The recorder's own check is unaffected: it is a long-lived node.

## 2026-10-26 replay set: results and the factory-kinematics caveat

The 14 SlowChecking/FastChecking/RCMOnly/WholeRun SimpleHolder replays were re-recorded with
NDI data. Results, the original-to-replay map and the analysis are in
**`logs/V7/REPLAYS_2026-10-26.md`**. In short: the teleop holds its own RCM point to
0.05-0.23 mm, but the NDI-measured error (0.4-2.9 mm rms) is mostly pose-dependent error in
the UR16e factory kinematics (TF vs the controller's calibrated tool0: 0.1-0.25°). Loading
the arm's own calibration is the untested next step; `ur_calibration` failed to connect.
Analysis script: `thesis/run_tools/rcm_offset_check.py`.

## V8 — summary of what changed from V7

**Code:** tag `v8` (builds on `v7`). Dates 2026-10-27 to 2026-10-30. The sections below have the full detail, measurements and reasoning; this is the overview.

| Change | Why | Detail |
|---|---|---|
| Hold GREY 1 s (no WHITE) to switch the stylus between the arm and the micro module; WHITE is the deadman for both; `launch_system.sh ur5e-touch --with-micro` | One stylus for both modules, without one GREY press both recording and moving the micro | [macro / micro switch](#macro--micro-switch-on-the-grey-button-2026-10-27) |
| Numbered tool library (`config/tools.yaml`, `tool_library.py`); type a number in the launch terminal to switch; refused in RCM, with WHITE held or while the arm moves; tool logged in `META.yaml` | Several tools on the quick-change coupling, each with its own calibration; editing settings and restarting per swap was slow and error-prone | [tool library](#tool-library-numbered-calibrated-tools-switched-from-the-terminal-2026-10-30) |
| The robot runs a copy of the driver's URScript whose `speedj` ramps over each 2 ms step (`SERVO_SMOOTH_SPEEDJ`) | Buzz / grind: 0.3-1.0 A of 120-250 Hz current on the big joints while accelerating, absent from the command | [buzz](#buzz--grind-speedj-ramps-over-each-2-ms-step-2026-10-30) |
| Micro module: transient-local states + 1 Hz republish, robust Arduino handshake (`Serial_Control` repo), motor command = rest + tendon model, rounding, joint limits from the motor limits | The Touch never moved it (startup deadlock); then the motors moved ~1/1000 of what they should | [micro module](#micro-module-startup-deadlock-lost-handshake-and-motors-barely-moving-2026-10-30) |
| Micro module: stylus position control (0.05×, task weighting), damping and orientation lag cut | Position barely moved the tip; the user wants a responsive Touch | [position](#micro-module-stylus-position-control-and-a-responsive-touch-2026-10-30) |
| `scripts/calibration/tip_ball_calibration.py`: tool tip in tool0 from one NDI ball, no wand; micro module = tool 2 (tip (−8.7, 17.0, 436.0) mm, 200 mm shaft) | The micro module has no wand; `robot-tcp --tip-ball` needs one | [tip ball](#tool-tip-from-a-single-marker-ball-no-wand-scriptstip_ball_calibrationpy-2026-10-30) |

None of these has a dedicated real-arm benchmark yet: the speedj script, the micro fixes and position control are verified in simulation / with fakes; the tool switch and the tip-ball calibration ran on the real setup.

### Open / not done in V8

- `serial_bridge.py`'s handshake / republish fix lives in the separate `Serial_Control` repo.
- No real-arm measurement yet of the `speedj` script's effect on the buzz, or of the micro module's direction, frame and feel.
- The micro tip (tool 2) is only right with the module straight, and is ±1.5 mm (factory kinematics).

## Macro / micro switch on the GREY button (2026-10-27)

**What:** holding GREY for 1 s (`TOUCH_MODE_SWITCH_HOLD_TIME`) without touching WHITE
switches the stylus between the arm and the micro-manipulator. `ur5e_touch_teleop`
owns the mode and publishes it latched on `/teleop_mode` (`macro` at start).
`micro_module_control` subscribes to it. WHITE is now the deadman for **both**
modules (the micro node used GREY). Each node ignores WHITE while the other module is
active: no engage, no double-click scale change, no RCM chord. A GREY click shorter
than 1 s still starts/stops recording, but on **release** now, not press. WHITE
touched at any point during a GREY press cancels both the click and the switch, so
WHITE + GREY stays only the RCM chord. `launch_system.sh ur5e-touch --with-micro`
starts the serial bridge and micro node alongside the arm teleop.

**Why:** both nodes read `/phantom/button`, so with both running a GREY press would
toggle recording and drive the micro module at the same time, and WHITE would move
the arm while you were working the micro. The micro deadman had to move off GREY,
because holding GREY to drive it would switch back to the arm after 1 s. The switch
fires while GREY is still held, so you know it took. Recording has to wait for
release, because only then is it clear the press was a click and not a hold.
Without the arm teleop running, `/teleop_mode` is never published and the micro
node stays active, so the micro-only `touch` mode still works.

**Tests:** private ROS domain, `ur5e_touch_teleop` (`setup_servo:=false`) +
`micro_module_control`, fake stylus/button/motor topics. A 0.4 s GREY click does not
switch. A GREY hold with WHITE pressed partway does not switch. 1.2 s holds go
macro → micro → macro on both nodes. WHITE in macro mode → 0 micro commands. WHITE in
micro mode → 37 micro commands in 0.6 s. Not yet tested on the real arm, Touch or
micro module.

**Also:** removed a stray `ok` typed at the start of `src/touch_bridge.cpp` (an
editor slip, 17:27) that broke the build.

## Tool library: numbered, calibrated tools switched from the terminal (2026-10-30)

**What:** the RCM tool now comes from a numbered library in `config/tools.yaml` (no longer
the single `TOUCH_RCM_SHAFT_*` tool in `teleop_settings.py`). Each entry has a name, the
tip and shaft direction in tool0, the shaft length, and the calibration file and date it
came from. `ls_thesis/tools.py` loads it. `ros2 run ls_thesis tool_library.py
list|add|remove|default` manages it. `add` takes the tip and direction from an ndi_tracking
robot-tcp JSON (`--from`) or as numbers, plus a measured `--length`. The teleop starts on
the library's `default` tool. Setting its `tool` parameter switches tools (0 = default).
In `launch_system.sh ur5e-touch` the main terminal now runs `tool_library.py console`
while the logs are tailed in the background: type a number + Enter to switch, `l` to list.
A switch is **refused**, with the reason printed, in RCM mode, while WHITE drives the arm,
or unless every joint has stayed under `TOOL_SWITCH_MAX_JOINT_SPEED` (0.01 rad/s) for
`TOOL_SWITCH_STILL_TIME` (0.5 s). The node also refuses when there are no fresh
`/joint_states`, or the message has no velocities. The library file is re-read on every
switch, so a tool added while the stack runs can be selected without a restart. The active
tool is published latched on `~/active_tool`, and the recorder writes it to META.yaml as
`tool` (`at_start` + any `changes` during the run). Tool 1 is the NDI wand 8700340 from
`robot_tcp_handeye_2026-10-26.json` with the 158.6 mm shaft: the same shaft start as the
old settings to 0.01 mm, so behaviour is unchanged. `TOUCH_RCM_SHAFT_*` is only used if
the library is empty or missing.

**Why:** several tools are going on the quick-change coupling, each with its own
calibration. Editing settings and restarting the stack for every swap is slow, and it is
easy to run with the wrong tool. A wrong tool puts the RCM point and the insertion depth
in the wrong place. The switch is a parameter so it gets a success/failure reply with no
new interface types, and it also works from any terminal (`ros2 param set
/ur5e_touch_teleop tool N`). It changes only the tool model, not the robot pose, so it is
only safe while nothing depends on the old model: not in RCM mode (the latched point and
the depth limits belong to the old tool), and not while WHITE or anything else is moving
the arm. The recorder logs the tool, because the RCM settings stored in META.yaml no
longer describe the tool actually used.

**Tests:** private ROS domain 77, `ur5e_touch_teleop` alone (`setup_servo:=false`, haptics
off) on a scratch copy of the library with a test tool 2 added, fake `/joint_states`,
stylus and button topics, static TF. All 12 checks passed: startup on tool 1. Refused with
no joint states, while moving, 0.3 s after stopping, with WHITE held, in RCM mode, and for
an unknown number. Accepted when still for 0.5 s, after leaving RCM, and 0 → default.
Console: piped numbers switch tools and print refusals and the active tool. Not yet tested
on the real arm, or through `launch_system.sh` end to end.

## Buzz / grind: speedj ramps over each 2 ms step (2026-10-30)

**What:** with `SERVO_SMOOTH_SPEEDJ = True` (default), `ur5e_teleop.launch.py` passes
`launch/ur_rsp_smooth_speedj.launch.py` as the driver's `description_launchfile`. That
file is a copy of `ur_robot_driver`'s `ur_rsp.launch.py` (driver 32bb4c1); the only
change is `script_filename`, which now points to `config/external_control_smooth_speedj.urscript`.
That is ur_client_library 2.15.2's `external_control.urscript` with one change, in
`speedThread`: the stock `speedj(qd, 40.0, steptime)` becomes `speedj(qd, qdd, steptime)`
with `qdd = max |qd - last qd| / steptime`, clamped to 1-40 rad/s². This is the same
thing the driver's own spline code (`jointSplineStep`) does. Nothing in the driver
package or `/opt/ros` is changed. `False` restores the stock script.

**Why:** the occasional buzzing / grinding sound. Spectra of the V7 rosbags
(`WeirdVibrationSounds`, `FastCheckingRCMSimpleHolder`, `SlowCheckingRCMSimpleHolder`)
at each stage of the pipeline:
- Joint current (`/joint_states` effort) carried 0.3-1.0 A RMS at 120-250 Hz on the
  base / shoulder / elbow in the buzzing and fast runs (peaks at 228-247 Hz and
  119-134 Hz), against 0.04 A in the slow run.
- The command at every stage had almost nothing there: twist, Servo output and
  limiter output were all ≤ 0.0003 rad/s at 150-250 Hz.
- The buzz followed command step size (r = 0.55-0.65) and joint acceleration
  (0.48-0.53), not speed (−0.06 to 0.28).
- Command timing was not the cause. In the 10-22 runs each command lands
  0.4 ms after the robot tick, and missing / doubled cycles did not correlate with
  the buzz (r −0.18 to −0.27).

What does explain it: the stock speed thread reaches each new 2 ms set point at
40 rad/s², so a step of a × 2 ms is done in a fraction of the cycle. That is a torque
pulse every cycle, scaled by how hard the arm accelerates, and the heavy UR16e joints
ring with it. Ramping over the whole step makes the speed piecewise linear instead of a
staircase. The cost is about 1 ms of extra lag on average. The limiter's caps
(≤ 25 rad/s²) keep `qdd` under the stock 40.

**Tests:** both launch files parse. On a private domain, `ur_rsp_smooth_speedj.launch.py`
publishes a `/robot_description` whose `script_filename` is the installed
`ls_thesis/config/external_control_smooth_speedj.urscript`. **Not yet run on the real
arm.** URScript can only be checked by the controller. If External Control stops with a
script error on the pendant, set `SERVO_SMOOTH_SPEEDJ = False`. To check the effect,
record a run with fast moves and compare the 150-250 Hz joint current against the runs
above (analysis scripts were scratch-only; numbers above).

## Micro module: startup deadlock, lost handshake, and motors barely moving (2026-10-30)

**Symptoms (first real run of `ur5e-touch --with-micro`):** (1) the Touch never moved the
micro module; (2) earlier, in keyboard mode, the Arduino ignored every command; (3) once
unstuck by hand, it moved but the motor states hardly changed.

**Causes and changes:**
1. **Deadlock at startup.** After the handshake the Arduino reports its angles only in
   reply to a command. `serial_bridge.py` published the one handshake state, and the
   launcher started `micro_module_control` ~2 s later. That node subscribed with volatile
   QoS (the bridge publishes transient local), so it never got the state, and it sends no
   command until it has one. Fixed on both sides. `micro_module_control` now subscribes to
   the states transient local. The bridge republishes the last state at 1 Hz with the
   limits.
2. **Handshake lost (`Serial_Control/ros2_bridge/serial_bridge.py`).** Opening the port
   resets the Mega. The bridge answered strings left in the buffer from before the reset,
   so its "OK" arrived during the bootloader and was lost. The firmware then waits for
   "OK" forever and treats every command as a failed "OK". The bridge now waits 2 s after
   opening, clears the input, and handshakes. A state arriving > 0.5 s after the last
   command must be the startup string again (lost OK or a reset), so it re-sends "OK".
   The firmware ignores a stray "OK" outside the handshake (not 12 digits).
3. **Motors moved ~1/1000 of what they should (`micro_module_control.cpp`).** The loop
   added each joint delta (rad) straight onto the motor commands (degrees). Lachlan's
   original (`f3df5f3`) added the *absolute* joint → motor mapping every cycle instead,
   which would run away. Now: command = motor angles at startup +
   `GetMotorPositionsFromJointPositions(joints)` (the tendon model). Each joint delta is
   applied only if all four motors stay inside their limits.
   - Commands are rounded to whole degrees, not truncated (89.95 went out as 89).
   - The damped inverse's joint limits were ±0.05 / ±0.025 rad. The tendon model puts
     that at about ±1° of motor travel (~20° of motor per radian), so the damping held
     every motor within a degree of rest. The limits are now found at startup by
     stepping each joint until a motor would hit its limit, and logged.
   - The 60 Hz prints of joint positions, limits, lambda and D are removed (they flooded
     the launcher terminal).

**Tests:** private ROS domain with a pseudo-terminal fake Arduino. It behaves like
`motor_control_absolute.ino`, but drops the first "OK". Stale startup strings were left
buffered before the bridge opened the port.
- The bridge re-sent OK after 1 s and the handshake completed.
- `micro_module_control` started 4 s after the handshake still got the state (the
  deadlock case).
- With a fake Touch in micro mode holding WHITE, ±30° pen tilts drove the motors over
  93-97 / 88-93 / 82-87 / 84-93°, against ≤ 1° before.
- Stylus *position* still barely moves the tip. The ×10 position target added in the
  week-7 commit is metres beyond a 4 mm module's reach, and the damping swamps the
  position rows. Lachlan's original drove orientation only.

Not yet run on the real micro module. Motion direction and gain are unverified.

## Micro module: stylus position control, and a responsive Touch (2026-10-30)

**What (`micro_module_control.cpp`):**
- **Position:** the tip target is origin + `MICRO_POSITION_SCALE` (0.05) × the stylus
  movement, turned into the manipulator frame by the same 45° rotation as the stylus
  orientation. It was 10× the stylus, unrotated: a 40 mm stroke asked for a target
  0.4 m away from a 4 mm module.
- **Task weighting:** position rows of the Jacobian and the position error are divided
  by `POSITION_WEIGHT_LENGTH` (5 mm) before the inverse, so position (m) weighs about as
  much as orientation (rad). Unweighted, the position rows are ~1/200 of the angle rows
  and the damping swamped them. The per-cycle cap now applies to the weighted step.
- **Damping:** the damped least squares had an extra unscaled `D²`:
  `JᵀJ + D² + λ²D²` with λ = 1. Now `JᵀJ + λ²D²` with `LEAST_SQUARES_DAMPING_FACTOR`
  0.3. D still rises from 1 to 2 at a joint limit.
- **Orientation lag:** `MANIPULATOR_ROTATION_SCALE_FACTOR` is a per-cycle slerp share
  toward 1:1 pen tilt, i.e. a lag, not a scale. 0.02 → 0.2: time constant ~0.8 s → ~80 ms.
- A throttled log line (2 Hz) shows the tip target and the tip offset in mm while engaged.

**Why:** stylus position barely moved the tip, and the user wants the Touch to feel
responsive.

**Tests:** private ROS domain, fake Arduino + `serial_bridge.py` + fake Touch in micro
mode.
- During a ±40 mm stylus circle, the tip (tendon model) stayed within ~0.1 mm of its
  2-3 mm target. The motors swept 92-97 / 88-91 / 82-86 / 88-94°.
- 30 mm stylus step: first motor change 29 ms, at the final angles in 69 ms (1.1 s
  before the damping change).
- 20° pen-tilt step: first change 39 ms, settled 0.36 s.

Real servo speed and the 9600-baud link (~13 ms per command) come on top. Not yet run
on the real module, so direction, frame and feel are unverified. Tilting while holding
the stylus still asks the 4-DOF module for both a rotation and a fixed tip, so it
compromises.

## Tool tip from a single marker ball, no wand: `scripts/calibration/tip_ball_calibration.py` (2026-10-30)

**What:** calibrates a tool tip in tool0 from one passive marker ball on the tip, for the
micro module, which has no wand. You freedrive the arm with the ball in view. Vega frames
with exactly one stray marker give the ball's camera position. The point fixed in tool0
and the base → camera pose are fitted together from the robot kinematics alone
(`ndi_tracking.robot_tcp.fit_robot_point`), best over an NDI delay sweep (−50 to
+100 ms), with outliers (> 3× the median residual) dropped once.
- The tip is the ball centre moved `--ball-radius` back toward the holder.
- The direction runs from `(0, 0, --holder-z)` on tool0 z to the tip.
- Output goes to `config/calibrations/<name>_<date>.json`, with `tip_tool0_mm` /
  `direction_tool0` as `tool_library.py add --from` expects.
- `src/ndi_tracking` is used as a library and not changed.

**Why:** `ndi_tracking robot-tcp --tip-ball` needs the wand on the tool, both to pick
the ball out among the strays and for its hand-eye check. The micro module has no wand.

**Tests:** synthetic. Tip (3, −8, 262) mm, base → camera pose, 0.25 mm marker noise,
30 ms NDI delay, 20 sightings 30 mm off. The fit gave (2.98, −8.04, 261.99): 0.04 mm
error, delay 30 ms, all 20 outliers dropped. Not yet run on the robot.

**First real run (micro module tip ball, 14:40,
`config/calibrations/micro_tip_2026-10-30_144004.json`):** ball centre in tool0
(−9.15, 15.93, 436.25) mm, 2363 single-stray frames, swing 136°, residual 1.88 mm rms.
The script then gained `--file` re-fits, and `--holder-z` became optional (default
tool0 origin → ball). Re-analysis of the raw `.npz`:
- TF updated at only ~19 Hz (52 ms steps; External Control was not running). With
  robot poses slerp-interpolated, the result is unchanged: (−9.15, 15.94, 436.41) mm,
  residual 1.53 mm.
- The residual does not grow with speed (still: 1.48 mm, > 100 mm/s: 1.39 mm), so it
  is not timing. It is pose-dependent.
- A gravity-sag model of the module explains only 16 % of it.
- The two halves of the run give tips 1.6 mm apart, mostly along z, so treat the offset
  as ±1.5 mm.

Most likely cause: the UR16e **factory kinematics**. The same 0.1-0.25° TF error seen
in the 10-26 replays comes to ~0.8-1.9 mm at a 436 mm lever. Loading the arm's own
kinematic calibration (`ur_calibration`, still not working) should tighten it.

## Rotation scaled the same as translation (2026-10-30, after V8)

**What:** `TOUCH_SCALE_ROTATION_EXPONENT` 0.5 → 1: tilt and twist are scaled by the same
step as translation.

| Translation | Rotation before (step^0.5) | Rotation now (step^1) |
|---|---|---|
| 1:1 | 1:1 | 1:1 |
| 1:5 | 1:2.24 | 1:5 |
| 1:20 | 1:4.47 | 1:20 |

**Why:** the user's choice, after considering half the ratio, the cube root and step^2/3:
one scale for the whole motion. At 1:20 a 20° pen tilt now turns the tool 1° (was 4.5°).

**Tests:** `ur5e_touch_teleop` on a private domain (`setup_servo:=false`): three WHITE double
clicks logged 1:5 (rotation 1:5), 1:20 (1:20), 1:1 (1:1). Not run on the real arm.

## NDI tracker no longer required to record (2026-10-30, after V8)

**What:** `LOG_REQUIRE_NDI` True → False. A GREY click (or a replay) now records even when
`ndi_tracking run` is not following the recording. The recorder still logs an error at start
("this run has NO NDI data"), the replay no longer waits for the tracker, and `META.yaml`
has `ndi: {tracker_at_start: false}`, so a run without NDI data is still visible afterwards.
Setting it back to True restores the refusal.

**Why:** the user's choice, to record runs without the Vega. (The guard was added on
2026-10-26 after 2 originals + 12 replays were recorded without NDI unnoticed; see
*Recording refused without the NDI tracker*.)

## Orphaned drivers fed stale joint states into live sessions; vibration still there (2026-10-30, after V8)

**Run `logs/NEW_RUN` (16:21, code `eed73bb`-dirty, `SERVO_SMOOTH_SPEEDJ` on, no NDI):** the
user heard the vibration again.

1. **The buzz is not fixed by the speedj ramp.** The live `/robot_description` loaded
   `external_control_smooth_speedj.urscript`, yet the joint current still carries
   0.80 A rms at 150-250 Hz and 0.95 A at 80-150 Hz on the shoulder lift (V7:
   `WeirdVibrationSounds` 0.97 / 0.45 A, `FastCheckingRCM` 0.36 / 0.51 A, slow run 0.04 A).
   It again follows command steps (r = 0.69) and acceleration (0.43). The command itself
   still has ≤ 0.0003 rad/s in that band. The speedj-kick explanation is at best incomplete.
2. **Four orphaned `ros2_control_node`s** (14:34, 15:11, 15:18, 15:25) were alive on domain 0
   next to the live one (16:09). Each ran its own `joint_state_broadcaster`, so `/joint_states`
   had 5 publishers. In the 8.3 s bag, 8 messages jump > 0.05 rad (up to 3.56 rad). They
   carry stamps 722-5563 s old and frozen poses, one the default `[0, −1.309, 1.571,
   −1.833, −1.571, 0]`. The teleop (`joint_states` pose source), Servo and MoveIt all read
   that topic, so each one is a cycle computed for the wrong arm pose. The live node held
   every robot port (50001-50004, 30001-30004), so the orphans never reached the robot. The
   user killed them, and `/joint_states` is back to 1 publisher.
   - Why they were left: `launch_system.sh` started each command as `( cmd ) &` in the
     launcher's own process group and stopped it with one SIGTERM to the top PID. Ctrl+C
     also SIGINTed every node directly, `ros2 launch` exited on the second signal, and the
     UR control node, slow to stop, survived. **Fix:** `run_bg` uses `setsid`, and
     `stop_groups` (cleanup and `stop`) signals each whole group: SIGINT, SIGTERM after
     15 s, SIGKILL after 5 s. `check_no_running_driver` refuses to start a driver while
     another `ros2_control_node` runs on the same `ROS_DOMAIN_ID`. It runs before the PID file is
     replaced, so `launch_system.sh stop` still covers the previous run. (First version ran it
     after: the first refused start deleted the PID file. Hit at 16:28 by the 16:09 session's
     driver, orphaned by the old launcher's shutdown as predicted.)
     Then, at the user's request, orphans are no longer just reported: a `ros2_control_node`
     whose parent is `systemd` / init (its `ros2 launch` is gone) is stopped by the launcher
     (same SIGINT → SIGTERM → SIGKILL steps) and start-up continues. It still refuses for a
     driver whose launch is alive (another terminal's session). Tested with fake drivers on
     domain 77: live parent → refused; orphan → stopped (SIGINT ignored, SIGTERM at 15 s).

**Next:** record the same motion again with no orphans (or replay `NEW_RUN` with
`stylus_replay`), with `SERVO_SMOOTH_SPEEDJ` True and False. That separates the
stale-pose kicks, the speedj ramp and whatever else drives the 120-250 Hz current.

## Buzz is speedj-only and depends on pose; back to servoj without the lag (2026-10-30, after V8)

**Runs (16:36 `LargeVibrations`, 16:43 `NEW_RUN`, same launch session, code `eed73bb`-dirty,
identical settings, `SERVO_SMOOTH_SPEEDJ` on, one `/joint_states` publisher):** the user
moved the arm around until it buzzed (`LargeVibrations`), then moved it out of that area
(`NEW_RUN`, no buzz).

| | LargeVibrations | NEW_RUN |
|---|---|---|
| Big-joint current 80-249 Hz, while moving | 1.81 A | 0.22 A |
| Big-joint acceleration (rms) | 4.6 rad/s² | 6.2 rad/s² |
| Command step per 2 ms | 10.8 mrad/s | 15.3 mrad/s |
| Command 20-80 / 80-249 Hz | 1.35 / 0.21 mrad/s | 1.43 / 0.51 mrad/s |
| Mean pose (pan, lift, elbow, w1, w2, w3) | −0.25 −1.32 2.00 −1.98 −0.99 1.33 | 0.20 −1.51 1.59 −1.85 −1.05 2.41 |

- **The acceleration explanation above is wrong as a cause.** `NEW_RUN` accelerates
  harder with a rougher command and is 8× quieter; `LargeVibrations` buzzes at ~1 A even
  in its calm first seconds (1.5 rad/s²); the V5 Tuned benchmarks had the hardest commands
  of all and 0.10-0.18 A. Within a buzzing run the buzz does grow with acceleration /
  command step (r = 0.94 / 0.96 here), which is what the earlier correlations measured.
  Whether a run buzzes is set by the pose. Gravity load is not it: mean joint currents
  are the same in buzzing and quiet windows (lift −3.4 vs −3.3 A).
- **The tool is not it either** (user: a buzzing area can be found with any tool), and
  **`servoj` never buzzed** (V3 trajectory mode, 0.11 A).
- **Likely mechanism (not proven):** `speedj` takes a new joint speed every 2 ms, so the
  reference acceleration jumps every cycle, also with the ramped script; that is a kick
  at 500 Hz scaled by the step size. In some poses an arm / joint-loop mode sits where
  it rings with that. The logs cannot show it: the command is only sampled once per
  cycle, and anything above 250 Hz in `/joint_states` aliases (a ~370 Hz ring reads as
  ~130 Hz, where the buzz peaks). A phone recording of the sound would tell.

**What:** `SERVO_OUTPUT = "position"`, the new default. Servo is set up exactly as in
velocity mode (joint velocities at 500 Hz into `joint_velocity_limiter`, same caps and
smoothing). The limiter (`output` "position") integrates the limited velocities into
its own joint positions and publishes `q + qd × SERVO_POSITION_LEAD` to
`forward_position_controller`; the driver sends that as `servoj` each cycle (driver
xacro: lookahead 0.03 s, the minimum, gain 2000).
- Integrating its own sum is what avoids V1's crawl (Servo's position output re-bases on
  the measured joints every step). It is integrated over the real time between steps,
  not one period each: commands arrive in bursts (two within 0.01 ms, then a gap), and a
  full step each put a kick into the set point again (emulator: peak joint acceleration
  2980 → 830 rad/s², 50-240 Hz acceleration 2.4× lower).
- `SERVO_POSITION_LEAD` = 0.03 s cancels the UR's servoj response (16 ms dead time +
  15 ms lag, `ur_emulator`'s fit to V3) at steady speed.
- Safety: the position starts from `/joint_states`, again whenever the arm is found
  > 0.01 rad from the last command after being idle (moved by another controller, e.g. a
  replay's approach), and is never more than `SERVO_POSITION_MAX_ERROR` (0.15 rad) ahead
  of the measured joints (warning in the limiter's 5 s report). If Servo or the limiter
  dies, `forward_position_controller` holds the last position, so the arm stops.
- `ur_emulator` takes positions on `/ur_emulator/positions` (same dead time + lag as
  trajectory mode). `VelocityWatchdog` stays velocity-only. `SERVO_SMOOTH_SPEEDJ` does
  nothing in position mode (the script's servo thread is the stock one).

**Tests (mock UR16e + `emulate_ur:=true`, domain 87, `stylus_replay --no-record` of
`V3/CyclingScaling`, scored on forward kinematics of `/joint_states` vs the teleop target):**

| Output | Lag pos / angle | RMS error | With lag removed | Robot behind Servo's output |
|---|---|---|---|---|
| velocity (`speedj`) | 62 / 66 ms | 5.3 mm | 2.06 mm | 34 ms |
| **position, lead 0.03** | **58 / 62 ms** | **4.9 mm** | **1.36 mm** | **22 ms** |
| position, lead 0.05 (first version, per-period integration) | 50 / 54 ms | 4.3 mm | 0.72 mm | 6 ms |

The ~36 ms before Servo's output (teleop node + Servo) is the same in both modes. Lead
0.05 is better on the emulator but tuned to a model; tune it on the real arm by lining up
`/forward_position_controller/commands` with `/joint_states`.

**Not yet run on the real arm.** Check: the buzz in the area found in `LargeVibrations`
(80-249 Hz joint current), and the lag.

## Batch replays and the wand-tool calibration as scripts (2026-10-30, after V8)

**What:**
- `scripts/replay_batch.sh [--dest=V8] [--repeats=3] [run[:--rcm] ...]`: runs
  `stylus_replay.py --yes` for each run (default: the four V7 SimpleHolder originals,
  RCMOnly with `--rcm`, as on 2026-10-26) and moves each `logs/NEW_RUN` to
  `logs/<dest>/<run>_Replay<n>` before the next replay deletes it. It refuses to start
  while `logs/NEW_RUN` exists or the `ndi_tracking` node is not on ROS, and stops at the
  first replay that has no saved recording (`REPLAY.yaml`) or no `NDI_PATH.csv` (that one
  is kept as `..._NO_NDI`).
- `scripts/calibration/wand_tool_calibration.py`: refits the hand-eye from a divot-less `robot-tcp
  --wand-is-tool` raw recording (newest `calibrations/raw/robot_tcp_*.npz`, wrist 1/2/3
  jogged, the sequence twice), with ndi_tracking's own fit. It drops NDI frames whose implied
  camera pose is > 30 mm off (mis-tracked wand), and fits each half on its own as a
  repeatability check. It writes `config/calibrations/wand_handeye_<date_time>.json` with the
  hand-eye wand tip as `tip_tool0_mm`, compares it with the library's tool 1 and prints the
  `tool_library.py add 1 ... --force` command. NOT RECOMMENDED if the halves are > 2 mm
  apart, the rotation mismatch or camera-pose spread is > 1°, or the recording is > 2 h old.

**Why:** the replay set is being redone with position output. Each replay overwrites
`NEW_RUN`, so renaming by hand between runs risks losing one, and `LOG_REQUIRE_NDI` is off,
so nothing else would stop a replay set without NDI ([NDI required for recordings]). The
calibration: `robot_tcp.json` stores the robot-pivot tip as `tip_tool0_mm`, which is
invalid without a divot, so `tool_library.py add --from robot_tcp.json` would take the
wrong tip. On 2026-10-26 the corrected record was written by hand.

**Tests:** the converter on the 2026-10-26 raw recording reproduces tool 1 to 0.00 mm /
0.00° (halves 1.2 mm apart). First recording today (17:25:58, wrists jogged once each): 47
mis-tracked frames at 33-35 s; without them the fit matches 10-26's quality (mismatch
0.38°, spread 7.6 mm / 0.35°), but the halves are 5.2 mm apart (mostly tool0 x), so it was
not applied. Both halves put the tip ~5-7 mm from tool 1 (y +5 to +7, z −3 to −4 mm, shaft
+1.4°): the holder has probably moved since 10-26. The batch script's syntax and its
NEW_RUN refusal were checked; it has not yet run on the arm.

## Rename: ls_thesis → macro_micro_robot, ur5e → ur16e (2026-11-02, after V8)

**What:** names only. The code does exactly what it did at `1b9b502`.

| Before | After |
|---|---|
| package `ls_thesis` (`ros2 launch/run ls_thesis ...`) | `macro_micro_robot` |
| `ls_thesis/` (Python helpers), `include/ls_thesis/` | `macro_micro_robot/`, `include/macro_micro_robot/` |
| `ls_thesis/ur5e_common.py` | `macro_micro_robot/ur16e_common.py` |
| `launch/ur5e_teleop.launch.py`, `rviz/ur5e_teleop.rviz` | `launch/ur16e_teleop.launch.py`, `rviz/ur16e_teleop.rviz` |
| `src/ur5e_touch_teleop.py`, `ur5e_keyboard_teleop.py`, `ur5e_control.cpp` | `src/ur16e_touch_teleop.py`, `ur16e_keyboard_teleop.py`, `ur16e_control.cpp` |
| nodes / topics `/ur5e_touch_teleop/...`, `ur5e_keyboard_teleop`, `ur5e_control` | `/ur16e_touch_teleop/...`, `ur16e_keyboard_teleop`, `ur16e_control` |
| `launch_system.sh` modes `ur5e`, `ur5e-keyboard`, `ur5e-touch` | `ur16e`, `ur16e-keyboard`, `ur16e-touch` |
| `lab_ur5e_1_calibration.yaml` (repo root) | `config/lab_ur5e_1_calibration.yaml` (same installed path) |

**Kept as UR5e on purpose:** `lab_ur5e_1_calibration.yaml` really is a UR5e's calibration
(upper arm 0.4250 m, forearm 0.3922 m, shoulder height 0.1623 m; a UR16e is 0.4784 / 0.3600 /
0.1807 m), from the original project's arm. It is still loaded only for `ur_type:=ur5e`, and
`ur5e` stays in the `ur_type` choices. No recorded run used a UR16e calibration file: every run
used the UR16e factory kinematics, except the V3 `CyclingScaling` replays that ran with
`ur_type:=ur5e` (see *V3*). The 0.6-3.5 mm `tcp_offset.check_max_mm` of the V3-V8 runs fits the
factory model (a UR5e model would be ~30 mm off). The arm's own calibration is still not
extracted (see the V7 replay notes).

**Old recordings:** bags before this date have `/ur5e_touch_teleop/...` topics.
`thesis/run_tools/rcm_offset_check.py` reads both names. Historical sections above keep the old names,
as they describe the code at their tags.

**Outside this repo:** `ndi_tracking` default `--rcm-topic` → `/ur16e_touch_teleop/rcm_active`, and
its `teleop_settings.py` path → `macro_micro_robot/` (`robot_tcp.py`, `__main__.py`,
`ros_link.py`, README).

**Also renamed:** the launch file's private `_ls_quieted` flag → `_quieted`.

**After pulling:** `colcon build --packages-select macro_micro_robot --symlink-install`, then
use the new names. `install/ls_thesis` from earlier builds is stale and can be deleted.

**Why:** the arm has always been the lab UR16e. The names came from the original UR5e project,
so the code read as if it drove a UR5e, and `ls_thesis` said nothing about what the package is.

**Tests:** each changed file was compared with its `1b9b502` version after the same name
mapping, and only the intended exceptions above differ. The package builds (scratch build),
every Python file compiles, and `macro_micro_robot.*` imports. A mock launch
(`use_mock_hardware:=true input:=replay`, private domain) started `/ur16e_touch_teleop`.

## Arm's own kinematic calibration: `ur_calibration_extract` (2026-11-03, after V8)

**What:** a new tool that reads the UR16e's factory calibration from the controller into a
`kinematics_params_file`, and a `kinematics:` block in every run's `META.yaml`. Nothing changes for
existing launches: the default is still the nominal UR16e model until a calibration file is passed.
Full write-up: [UR_CALIBRATION_FIX.md](UR_CALIBRATION_FIX.md).

- `ros2 launch macro_micro_robot ur_calibration_extract.launch.py` →
  `config/calibrations/ur16e_kinematics_<date>.yaml`. Use it with `launch_system.sh --calibration-file=`.
- `src/ur_calibration_extract.cpp`, plus `vendor/ur_calibration/` (byte-identical upstream `jazzy` @ `67578e0`
  `calibration.hpp/.cpp`, BSD-3); `CMakeLists.txt` / `package.xml` (+ `ur_client_library`, `yaml_cpp_vendor`).
- `trajectory_recorder.py` gets a `kinematics_params_file` parameter, passed in by `ur16e_teleop.launch.py`.
  `META.yaml kinematics:` = file, `model` (nominal / calibrated), sha256, calibration hash.

**Why:** every run so far used the nominal model, not this arm's calibration. The driver logged an ERROR
saying so at every connection (125 logs, 10-01 to 10-30). This is the main suspect for the 0.6–1.2 mm
TF/controller drift and the 0.17–3.7 mm NDI pivot offset (see *2026-10-26 replay set*). The driver's own
`ur_calibration` (3.8.0) couldn't produce the file because of two upstream bugs:
1. It never connects. It calls `Pipeline::run()` without `Pipeline::init()`, and with ur_client_library
   2.15.2 only `init()` opens the socket (`Stream is invalid` forever). Fixed upstream `0089621`.
2. Once connected, it writes link rotations in the wrong Euler convention (intrinsic XYZ instead of URDF's
   fixed-axis XYZ). The file loads, passes the driver's hash check, and is still wrong: up to 3.2 mm at
   the flange for a synthetic UR16e calibration. Fixed upstream `81df755`. The 11-03 plan's "one-line
   patch" would have missed this.

Fixed here rather than in `src/Universal_Robots_ROS2_Driver`: that repo is outside the edit scope, a
patch would be lost on pull, and updating the driver to 3.10 mid-experiments brings unrelated changes.
The extractor also refuses to write a file whose URDF reading doesn't reproduce the controller's raw D-H
model (≤1e-9, 10 000 poses), and it records how far the nominal model is from this arm.

**Tests:** offline only (no robot): real UR10e and synthetic UR16e calibrations pass the check at ~1e-13;
the output parses in xacro; an unreachable robot gives exit 1 after 3 tries. See the write-up §5.
**Real robot not yet run.**

**Next:** extract on the arm (driver must then print `Calibration checked successfully.`), redo the wand
hand-eye and the micro tip ball, then the RD1 replays (`docs/lab_plans/07_remaining_experiments.md` §1.3).

**Same day, after the tests above:**
- **Real arm, first extraction (2026-11-03):** `config/calibrations/ur16e_kinematics_2026-11-03.yaml`, hash
  `calib_912036121568766846`. Check passed (max element 2.0e-14). **Nominal UR16e vs this arm: flange
  position max 2.42 mm, RMS 1.44 mm; rotation max 0.42°** (10 000 random poses). Read-only, the arm did
  not move. The controller reported per-joint checksums 0xFFFFFFFF and `calibration_status: 1`. The
  hash is computed from the D-H values, so the checksums don't matter. What status 1 means is unknown.
- **Session A settings (`teleop_settings.py`):** `SERVO_OUTPUT = "velocity"`, `SERVO_SMOOTH_SPEEDJ = False`,
  `TOUCH_SCALE_ROTATION_EXPONENT = 0.5`, the values the 2026-10-26 replays ran with (their `META.settings`;
  smooth speedj didn't exist then). This way the calibrated-kinematics replays differ from that set only in
  the kinematic model and the re-calibrated end effector. Other settings that differ from 10-26 affect
  logging / the tool library, not motion. **Set back to `"position"` / `True` / `1.0` for Session B.**
- An unsaved `logs/NEW_RUN` (2026-10-30 17:18, 136 s Touch run, no NDI) was moved to
  `logs/V8/UNSAVED_2026-10-30_171830` before the next recording could delete it.
- **`scripts/calibration/wrist_jog.py`:** writes the hand-eye wrist jog (wrists 1-2-3 one at a time, ±30°,
  A·sin(2πt/T), 14 s each, twice: 90 s) from the current pose as a `trajectory_player.py` file,
  instead of jogging by hand on the pendant. That needed External Control stopped and never repeated.
  Refuses > 45° or > 30°/s. Mock test (velocity-mode stack, 7 s per wrist): each wrist −30.0…+30.0°,
  other joints 0.0°, back at the start pose, `forward_velocity_controller` active again afterwards.
- **Wand hand-eye attempts on the SimpleHolder (2026-11-03, calibrated kinematics, `wrist_jog.py`):**
  09:26 run: the NDI wand pose moved 111 mm / 3.4° between the rest poses before and after (the robot
  returned to within 0.02 mm), so the wand or the Vega moved during the wrist-3 swings. Halves 51 mm
  apart: discarded. 09:34 run: good quality (mismatch 0.24°, spread 5.3 mm / 0.22°, both better than
  10-26), tip (−0.05, 16.56, 324.99) mm. But the jog started 44 s into the 120 s recording, so the last
  wrist-3 swing was cut off and the halves are 3.0 mm apart: redo.
  Fixes: `wand_tool_calibration.py` splits the halves at the middle of the *motion*, not the recording
  (a still lead-in left the first half with one wrist only), and warns if the arm is still moving when
  the recording ends. `wrist_jog.py` suggests `-d <motion + 60 s>`. The 10-26 recording gives the same
  result as before (halves 1.2 mm).
- **Tool 3 `V2_holder_only` (2026-11-03 09:41), now the default:** wand 8700340 in the SimpleHolder,
  hand-eye with calibrated kinematics (`wrist_jog.py`, 150 s recording, whole jog captured). Mismatch 0.29°,
  spread 6.0 mm / 0.25°, halves 0.4 mm, wand still to 0.04 mm / 0.01° (rest before/after). Tip
  (−1.01, 16.34, 324.99) mm, within ~1 mm of the 09:34 attempt. Tool 1 (10-26, nominal model) is kept for
  comparison.
- **Calibration confirmed on the arm (`thesis/run_tools/kinematics_check.py`, new):** in the first calibrated
  replay (V8/CalibratedKinematics/FastCheckingRCMSimpleHolder_Replay1), tool0 through the calibrated file
  matches the controller's TCP to 0.000 mm spread / 0.000°, against 0.847 mm / 0.374° for the nominal model.
  A 10-26 replay gives 0.000 / 0.000 (calibrated) vs 0.638 / 0.371 (nominal), so the offline
  re-analysis (E0) can use this file directly. `check_max_mm` stayed ~1.1 mm because it measures TF
  timing, not kinematics; the earlier "should fall to ~0" prediction was wrong (docs corrected).
- Noticed: the pendant TCP (a pure translation) was ~119 mm on 10-26 and 600.5 mm on 11-03. ROS
  doesn't use it for motion, but the controller's TCP-speed safety limits do.
- `code_version: unknown` in that replay: today's `colcon build` calls lacked `--symlink-install`, so the
  recorder's git lookup ran in install/ instead of the repo. Fix: clean rebuild with `--symlink-install`.
- **Session A replays done** (12 + 1, all with NDI): results and caveats in
  `logs/V8/CalibratedKinematics/REPLAYS_2026-11-03.md`. SlowChecking 2.87 → 1.30 mm rms, FastChecking
  1.70 → 1.27, but RCMOnly 0.43 → 2.12 and WholeRun 0.90 → 1.26. Not a clean pair: the new tool's tip is
  42 mm closer to the flange, swings are larger, and RCMOnly behaved differently. E0 next.
- **Bug (from the 11-02 rename), fixed in `stylus_replay.py`:** it read the original run's motion scale and
  RCM-at-start only from `/ur16e_touch_teleop/...`. Bags recorded before the rename use
  `/ur5e_touch_teleop/...`, so every older run looked like "no motion_scale = 1:1". The V7 RCMOnly original
  is 1:5, so **the 11-03 RCMOnly replays ran at 1:1: invalid** (tool0 moved ~2× and turned 68° vs 25° on
  10-26). Fast/Slow/WholeRun are 1:1 originals and unaffected (tool0 motion matches 10-26 within a few mm).
  Now reads both names, like `rcm_offset_check.py`. The 10-26 replays ran before the rename, so they're correct.
- RCMOnly redone at 1:5 (95f9eac): 0.47-0.50 mm rms, 100 % within 2 mm (10-26: 0.42-0.44, swing 40° → 47°).
- **V9 = position mode (`SERVO_OUTPUT = "position"`), everything else as Session A** (calibrated kinematics,
  tool 3, stock speedj, rotation exponent 0.5), so V9 vs V8/CalibratedKinematics differs only in the output
  mode. Mock check with today's code: `forward_position_controller` active, RCMOnly replay at 1:5 ran with no
  errors. **First real-arm run of position mode: supervised.**
- `replay_batch.sh` default order: RCMOnly last. At 1:5 it ends with RCM on and the tool in; RCM can only
  be left back at its own RCM point, so the WholeRun replay after it refused to start (V9 batch, nothing
  moved past WholeRun's start pose). Run a different run after RCMOnly only after restarting the stack.
- V9/PositionMode RCM set done (12 replays, no faults). **V9/TrajectoryMode: `SERVO_OUTPUT = "trajectory"`**, all
  else unchanged. LargeVibrations is replayed only in modes that stream positions (trajectory, position):
  it buzzed with speedj, and the user doesn't want it repeated in velocity mode.
- `scripts/set_v9_mode.sh trajectory|position|velocity-unfiltered|velocity-optimised|final`: sets the five
  settings that differ between the V9 sets and commits, so each recording's code_version names its mode.
- **Tool-tip clearance with tool 3 (tip 325 mm from the flange), lowest tip z above the robot base plane:**
  RCM set ≥ 101 mm, NoBuzzArea 191 mm, Scaled5_Slow 48.5 mm (measured in its trajectory replay),
  Scaled5_Fast ~70 mm (estimate), but LongBenchmark 11 mm, ShortBenchmark 4.9 mm (measured) and
  LargeVibrations ~1 mm. Those three were recorded with a shorter tool: not replayed with the wand mounted.
| (after V8) | `f9af572` | 2026-11-03 | Arm's own kinematic calibration loaded (`ur_calibration_extract`, `kinematics_params_file`); tool 3 `V2_holder_only` hand-eye through it (`wrist_jog.py`); META `kinematics` block; `kinematics_check.py` | ROS used the nominal UR16e model (driver ERROR in 125 logs); the driver's ur_calibration 3.8.0 never connects and writes wrong rpy. Docs: `docs/UR_CALIBRATION_FIX.md` | ROS tool0 = controller TCP to 0.000 mm (nominal 0.85 mm). Replays `logs/V8/CalibratedKinematics/REPLAYS_2026-11-03.md`: SlowChecking 2.87 → 1.30 mm rms, others ~1.2-1.3, RCMOnly 0.47-0.50 |
| V9 (part 1) | `0156789`, `977f652` | 2026-11-03 | Same RCM replay set in position and trajectory output (velocity = V8/CalibratedKinematics); `set_v9_mode.sh` | 3-mode comparison, same day / code / kinematics / tool; first real-arm position mode | Position: 12 replays, no faults, same RCM accuracy as velocity. `logs/V9/REPLAYS_2026-11-03.md`. Part 2: `docs/lab_plans/08_next_session_V9b.md` |
- **End of the 11-03 session:** V9 part 1 = RCM set in position (V9/PositionMode) and trajectory
  (V9/TrajectoryMode); results in `logs/V9/REPLAYS_2026-11-03.md`. Remaining NDI runs and the order:
  `docs/lab_plans/08_next_session_V9b.md`. Settings set back to `final` (position, smooth speedj, exponent
  1.0) until then. The failed 09:26 and cut-off 09:34 hand-eye fits are kept in `config/calibrations/` for
  the method chapter.
- **2026-11-06, `scripts/lab/run_v9b.sh` + `scripts/lab/pose_offset_motion.py`:** runs the remaining wand + NDI runs (RCM only):
  bridge, unfiltered RCMOnly, optimised RCM set, static floor, pose-offset test at two camera places. Tracked
  `Scaled5_*` / `NoBuzzArea` runs dropped: at their wrist 3 angle (~135 deg) the markers point ~110 deg away from a camera
  that sees the RCM runs (11-03 `Scaled5_Slow_Replay1`: 0 of 1166 NDI rows visible). `replay_batch.sh` only checked that
  `NDI_PATH.csv` was not empty, so it passed that run: it now needs 95 % visible, and `--no-ndi` is for tools without markers.
- **2026-11-06 RCM runs done:** `logs/V9b/REPLAYS_2026-11-06.md` has setup, purpose, results and the failed attempts. The camera had
  moved 107 mm since 11-03: RCMOnly matches (0.45), SlowChecking does not (1.87 vs 1.29), so compare with today's `PositionBridge`.
- **`run_v9b.sh` pose runs:** an External Control drop left a trajectory waiting, and it jumped ~170 mm in 2.5 s when the program
  restarted. The runner now waits 8 s after the recorder start, stops the player and stack on a drop, and needs "Trajectory
  complete" (`trajectory_player.py` exits 0 on failure). It reads program state from the stack log: subscribing to the
  program-running topic lined up with the 220-330 ms controller stalls behind the drops (not proven).
- **`run_v9b.sh` steps 7-11, no tool and no NDI:** benchmarks, Scaled5, NoBuzzArea in trajectory, position and velocity-optimised,
  LargeVibrations in trajectory and position only, unfiltered Scaled5 at 1:5. Tool 3 stays configured so the paths match the 11-03
  trajectory leg; the flange stays >= 262 mm above the base plane. Results go to `logs/V9b/NoTool/`.
- **2026-11-06, singularity guard run-off fixed (`SingularityGuard.__call__`):** a scripted stroke into the wrist singularity
  (`scripts/lab/singularity_input.py`, repeatable with no Touch) showed the guard holding c = 60 but the tool leaving its target with the
  stylus held past the limit: 609 mm height drop, 155 mm/s, 89 deg/s on the mock. R6.3 only covered the short case. The guard cut q' and mapped
  it back, so its output could oppose the command. It now cuts the twist, so the output is a part of the command: 0.7 mm, 36 mm/s, 4.8 deg/s,
  same input, same c limit. Simulation, 15 elbow strokes: 15 failed before, none after. Evidence: `logs/V9b/REPLAYS_2026-11-06.md`,
  `thesis/earlier/E6_singularity/`. Real arm: `run_v9b.sh --from=12` (guard on x3, off x3; the guard is always restored).
- **2026-11-06, E6 on the real arm (3 replays each, same scripted input):** guard on: c 60.4, returned to 0.6 mm, 4.5 deg/s. Guard off: c 204,
  Servo emergency stop, and the arm could not leave (172 mm from the start, 3 of 3). The mock returned with the guard off, so this needed the arm.
  `logs/V9b/REPLAYS_2026-11-06.md`, `logs/V9b/Singularity/METRICS.csv`.
- **2026-11-06, exchange protocol confirmed by the user:** 30 s still, 20 swaps (wand moved out of view, lever closed fully, 5 s seated),
  30 s still. Matches the 21 still segments in each `logs/Exchange` set, so R8.2 no longer rests on an inferred protocol and RD4 needs
  no redo for coupling repeatability. Written up in `logs/Exchange/PROTOCOL.md`.
- **2026-11-06, micro module recalibrated (`scripts/calibration/micro_tool_calibration.py`, tool 2):** four dots, two opposite pairs, so the shaft axis comes from the
  midpoints and no frame needs every dot. Each dot is fitted as a point fixed in tool0 with base -> camera. Tip (-6.41, 15.56, 436.28) mm, 1.68 deg off tool0 z,
  through the calibrated kinematics; the 10-30 entry used the nominal model. Tip = metal end + 21.8 mm measured, continuum section assumed straight.
  `wrist_jog.py --wrist3-amplitude` added to turn a tool about its shaft. Details: `logs/V9b/REPLAYS_2026-11-06.md`.
- **2026-11-19, `replay_batch.sh` _NO_NDI naming:** a second failed replay of the same run was moved *inside* the first `_NO_NDI` folder
  (as `NEW_RUN`), because both got `<name>_Replay<n>_NO_NDI`. It now takes the next free `_NO_NDI` number. Cause of the failures (V9c step 23):
  SlowChecking replays approach the V7 start joints (TCP z 0.625 m), above the Vega's view (wand lost above ~0.595 m), so the RCM never latched;
  camera moved; fits (tool0 -> wand) kept, since they do not depend on the camera pose.
- **2026-11-19, latch offset = tool line mismatch (offline) + phase L test (`run_v9c.sh --phase=L`, `thesis/run_tools/latch_line_fit.py`):**
  the "latch offset" is not timing (|offset| vs tip speed at the latch r = 0.008, 63 runs) and not the camera (folder 11). The tool slides
  100-150 mm through the RCM, and the line the controller holds is fixed in the wand frame but ~0.3 deg (tool 3) / ~1.05 deg (V9c tools 4-6)
  and ~2 mm off the tracked shaft, so the shaft misses the point by depth x angle. One fixed line per tool cuts the NDI RCM rms from
  1.1-1.6 to 0.14-0.27 mm on held-out V9c RCMOnly / SlowChecking replays. Corrected tools built from tools 4, 5, 6 and the perturbed
  7 (+1 mm lateral) and 8 (+2 mm axial) agree to ~0.15 mm / 0.01 deg, so the correction undoes known errors. It does not carry over
  to FastChecking / WholeRun (extra dynamic part). The direction comes from X * (NDI axis), so a non-zero angle is the hand-eye rotation
  (or arm orientation), not the NDI axis: the robot really did miss, and the old runs stay as measured. Phase L measures the fix:
  fresh fit (tool 10), training replays, corrected tool 11 from those only, then six interleaved A/B blocks of SlowChecking, RCMOnly,
  FastChecking, WholeRun. Prediction before the run: tool 11 <= 0.35 mm rms on RCMOnly and SlowChecking.
- **2026-11-19, phase L result + step 45:** tool 11 (corrected) vs tool 10 (fresh fit, 0.49 deg): RCMOnly 0.62 -> 0.28 mm, SlowChecking
  0.98 -> 0.26 mm (prediction <= 0.35 met; depth slope 0.72 -> 0.03 mm/100 mm), FastChecking 2.12 -> 0.89 mm, WholeRun 0.74 -> 1.17 mm
  (worse: its line angle is 0.12 deg with tool 10, so the 0.49 deg correction over-corrects it). About 0.3-0.4 deg of the line error is
  pose-dependent, which one fixed tool cannot cancel. B FastChecking_Replay2 ran at 1:20 (scale switched 7 s before; TCP path 472 vs
  ~3080 mm), kept as `_WRONG_SCALE`; `latch_line_fit.py` now reads only `<name>_Replay<n>` folders; `--only=45` replays one more.
- **2026-11-19, follow-up chosen after the phase L result (`run_v9c.sh --from=46`):** tool 12 = tool 10 corrected on all four
  trajectories (Train + the first tool-10 fast checking and whole run replays; mount untouched since phase L). Test: new
  replays, blocks 12 12 10 12 (the tool-10 block is a drift check, `Latch/A_tool10_drift`). Prediction written before the run
  (offline, held-out tool-10 replays): 0.31 / 0.22 / 0.50 / 0.63 mm for RCM only / slow / fast / whole run; offline predictions
  were ~0.4 mm optimistic for trajectories not trained on. Target: all four <= 1.0 mm with one tool. Report it as a follow-up,
  not as the pre-registered test.
