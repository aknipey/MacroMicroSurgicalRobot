# 8. Next lab session (V9b): the remaining NDI runs, then the markers come off

Written 2026-11-03 at the end of Session A + V9 part 1. Results so far: `logs/V8/CalibratedKinematics/REPLAYS_2026-11-03.md`,
`logs/V9/REPLAYS_2026-11-03.md`. Every step below keeps the tool tip ≥ 48 mm above the robot base plane.

## Before you start
- [ ] **Wand and SimpleHolder untouched since 2026-11-03** (tool 3 `V2_holder_only` is only valid then). If
      either moved: redo the hand-eye first (*Hand-eye* below).
- [ ] Camera may have moved: fine, but **don't move it again during the session** (all the vibration runs must
      share one camera position).
- [ ] Vega on: `cd ~/ros2_ws/src/ndi_tracking && ./setup_vega.sh`
- [ ] Pendant: External Control loaded, speed slider 100 %.
- [ ] `ls ~/ros2_ws/src/MacroMicroSurgicalRobot/logs/NEW_RUN` must not exist (move it into a version folder).

Terminals (as on 11-03): **T1** = `python3 -m ndi_tracking run` (leave running), **T2** = the stack, **T3** =
commands, from `~/ros2_ws/src/MacroMicroSurgicalRobot`.

**Restart the stack** (after every mode switch, and after any batch ending in RCMOnly): T2 Ctrl+C (never
Ctrl+Z), check `pgrep -fa ros2_control_node` prints nothing, then
```bash
cd ~/ros2_ws && source install/setup.bash && ros2 launch macro_micro_robot ur16e_teleop.launch.py input:=replay kinematics_params_file:=$HOME/ros2_ws/src/MacroMicroSurgicalRobot/config/calibrations/ur16e_kinematics_2026-11-03.yaml
```
and press Play. Check the startup shows `Tool 3: V2_holder_only` and no "calibration parameters ... don't match".

## Runs (≈ 45 min)

| # | Mode (T3 first) | Batch (T3) | Purpose |
|---|---|---|---|
| 0 | `./scripts/set_v9_mode.sh position`, restart | `./scripts/replay_batch.sh --dest=V9b/PositionBridge --repeats=2 V7/SlowCheckingRCMSimpleHolder V7/RCMOnlySimpleHolder:--rcm` | **Bridge:** must match 11-03 (SlowChecking ~1.29, RCMOnly ~0.48 mm rms). If not, something moved: redo the hand-eye, then carry on |
| 1 | `./scripts/set_v9_mode.sh trajectory`, restart | `./scripts/replay_batch.sh --dest=V9b/TrajectoryMode --repeats=2 V3/Scaled5_Slow V3/Scaled5_Fast V7/NoBuzzArea` | E2 tip vibration (1:5) + no-buzz motion |
| 2 | `./scripts/set_v9_mode.sh position`, restart | `./scripts/replay_batch.sh --dest=V9b/PositionMode --repeats=2 V3/Scaled5_Slow V3/Scaled5_Fast V7/NoBuzzArea` | same |
| 3 | `./scripts/set_v9_mode.sh velocity-unfiltered`, restart | `./scripts/replay_batch.sh --dest=V9b/VelocityUnfiltered --repeats=2 V3/Scaled5_Slow V3/Scaled5_Fast V7/RCMOnlySimpleHolder:--rcm` | The unfiltered baseline behind the −77 % number (RD2). **1:5 runs only. Ctrl+C if it shakes badly** |
| 4 | `./scripts/set_v9_mode.sh velocity-optimised`, restart | `./scripts/replay_batch.sh --dest=V9b/VelocityOptimised --repeats=2 V3/Scaled5_Slow V3/Scaled5_Fast V7/NoBuzzArea` then `./scripts/replay_batch.sh --dest=V9b/VelocityOptimised --repeats=3` | E2 + the final velocity RCM set (smooth speedj) |
| 5 | (arm still, same stack) | `source ~/ros2_ws/install/setup.bash; ros2 service call /trajectory_recorder/start std_srvs/srv/Trigger; sleep 30; ros2 service call /trajectory_recorder/stop std_srvs/srv/Trigger; mv logs/NEW_RUN logs/V9b/NDIStaticFloor` | NDI noise floor for E2 |
| 6 | `./scripts/set_v9_mode.sh final` | – | Back to the defaults (position, smooth speedj, exponent 1.0). Markers can come off now |

**Never** replay these with the wand mounted: `V3/LongBenchmark`, `V3/ShortBenchmark` (tip 11 / 4.9 mm above
the base plane), `V7/LargeVibrations` (~1 mm, and it buzzes in velocity mode: user's rule, never in velocity).

## After the markers are off
- **E1 benchmarks (no NDI needed):** `V3/LongBenchmark V3/ShortBenchmark` ×2 in `position`, `velocity-optimised`,
  and `trajectory` (trajectory already done 11-03 with the wand: redo for the same tool), plus
  `V7/LargeVibrations` in position and trajectory **only**. Check the tip clearance with the tool fitted first
  (ask Claude to compute it from the 11-03 replays and the new tool length).
- **Micro-module tip calibration (needs one marker ball on the micro tip):** `python3 scripts/tip_ball_calibration.py -d 60 --name micro_tip`,
  freedrive with lots of rotation; then update tool 2 with `tool_library.py add 2 ... --force`.
- **Session D:** micro-module tasks + demo on video.
- **Exchange repeatability (no redo needed if answered):** what V1/V2/V3 in `logs/Exchange/*_QC_V*` mean,
  whether the tool was fully removed each swap, lever cycled, arm still, swaps per session.

## Hand-eye (only if the wand / holder moved)
T2 stack in `ur16e-keyboard` mode (`./src/MacroMicroSurgicalRobot/scripts/launch_system.sh ur16e-keyboard --calibration-file=...`), wand
1.5-2 m from the Vega with markers facing it, then T3 `python3 scripts/wrist_jog.py`, T1 `python3 -m ndi_tracking robot-tcp --wand-is-tool -d 150`,
T3 the printed `trajectory_player.py` line → `python3 scripts/wand_tool_calibration.py` → if OK,
`ros2 run macro_micro_robot tool_library.py add 3 "V2_holder_only" --from <json> --length 158.6 --force` → commit.

## At the desk (Claude)
- E0: re-analyse the 10-26 bags through the calibrated file (kinematics vs tool part of the 11-03 change, the
  ~1.25 mm floor).
- Analyse V9/V9b: RCM per mode, tip PSD 15-25 Hz (E2), lag / jitter / current (E1), buzz current.
- Turn the scratch analyses into scripts (E0c).
