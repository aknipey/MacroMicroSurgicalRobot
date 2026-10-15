# Teleoperation Stack Versions

Each version is the state of the Touch -> UR16e teleoperation code for a set of
recorded runs. Every version has a git tag, so its code can be checked out or
browsed exactly:

```bash
git show v2 --stat            # what the version is
git diff v1 v2 -- src/ ls_thesis/ launch/   # what changed between two versions
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

Lag is the time shift that best lines the robot's tool0 path up with the stylus
target path (`scripts/plot_paths.py`). Each version's section below has the full
detail: code state, what was implemented and why, runs, and results.

## Runs and comparing versions

`logs/` is git-ignored (run data stays on the lab PC). `logs/RUN_VERSIONS.md` is a
link to this file, so the version history is readable next to the runs and
tracked in git.

Runs are written to `logs/NEW_RUN/`, which the next recording deletes, so move each
one by hand into `logs/V<n>/`. The exact settings for any run are in its `META.yaml`.

**Comparing versions:** replay a reference run's stylus into the new version
(`ros2 run ls_thesis stylus_replay.py V3/CyclingScaling` with the stack on
`input:=replay ur_type:=ur16e`), then `python3 scripts/plot_paths.py logs/V<n>`
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
  logs and a rosbag (`ls_thesis/run_logger.py`), and `scripts/plot_paths.py`
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
