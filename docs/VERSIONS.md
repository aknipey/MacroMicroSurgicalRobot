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

| Version | Tag | Date | Change | Lag (pos / angle) |
|---------|-----|------|--------|-------------------|
| V1 | `v1`, `v1-90deg` | 2026-10-12 | Baseline: Servo positions -> `forward_position_controller` | 640-760 ms / > 1.5 s |
| V2 | `v2` | 2026-10-12 | Servo `JointTrajectory` -> scaled trajectory controller | - / 128-158 ms (302 ms twist) |
| V3 | `v3` | 2026-10-12 | Selectable motion scaling (1:1 / 1:5 / 1:20) | ~140 ms / ~140 ms |
| V4 | `v4` | 2026-10-14 | Velocity output option, UR emulator, latency study | sim: ~30 ms / ~34 ms (velocity mode) |

Lag is the time shift that best lines the robot's tool0 path up with the stylus
target path (`scripts/plot_paths.py`).

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

**Runs:** none on the real arm yet.

**Launch:** always pass `ur_type:=ur16e` for the lab UR16e (the default is ur5e).
