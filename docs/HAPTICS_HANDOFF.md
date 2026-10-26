# Haptics on the Touch: status and handoff (2026-10-19)

Where the haptic-cue work stands, to pick up in a new session. The full reasoning and
measurements for every step are in [VERSIONS.md](VERSIONS.md), section **V7 (part 2)**. This file
is the short version, plus what is still open.

## What was asked for

Force feedback on the Touch stylus while teleoperating the UR16e:
- RCM mode: a soft wall at the deepest insertion and at the −5 mm retract limit.
- A small cue when the tip crosses the RCM point.
- A short cue when RCM mode switches on or off.
- Near a singularity: resistance in the direction that makes it worse.

## Current state

| Cue | How it works now | Where | Tested |
|---|---|---|---|
| RCM on / off | One 60 ms, 0.1 N upward tap (two taps if leaving RCM is refused because the tool is still in) | teleop → `/phantom/force_feedback` | Real Touch: felt; the earlier buzz (0.4 N, 40 Hz) was far too much |
| Crossing the RCM point | One 40 ms, 0.1 N tap against the direction of travel, ±1 mm stylus hysteresis | teleop → `/phantom/force_feedback` | Mock arm only. **Not yet tried on the real Touch** |
| Depth walls | One-sided spring, 150 N/m of stylus, rendered at 1 kHz from the pen's position | teleop sends the shape (UDP) → `touch_server` | Mock arm; on the real Touch only alongside the old detent |
| Singularity push | Up to 0.5 N along the stylus direction that leads out of the singularity, from the guard's gradient; 30 ms low-pass | teleop → `/phantom/force_feedback` | Offline (150/150 poses) + mock arm. **Not yet tried on the real arm/Touch** |

The total force is capped at 1.0 N (`TOUCH_HAPTIC_MAX_FORCE`; hard cap 2 N in `touch_server`) and
slew-limited. All settings are `TOUCH_HAPTIC_*` in `ls_thesis/teleop_settings.py`.
`TOUCH_HAPTICS = False` turns them all off.

## How to run

Three terminals, each first `source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash`:

```bash
# 1 robot (then press Play on the pendant)
ros2 launch ls_thesis ur5e_teleop.launch.py ur_type:=ur16e robot_ip:=192.168.0.100 input:=none
# 2 Touch driver: NOT omni_common's omni_state (its forces never reach the motors)
ros2 launch ls_thesis touch_driver.launch.py
# 3 teleop
ros2 run ls_thesis ur5e_touch_teleop.py --ros-args -p max_linear_speed:=0.05 -p translation_scale:=0.5
```

Only one program can own the Touch at a time: close TouchCheckup and any `omni_state` first.
After C++ changes: `colcon build --packages-select ls_thesis --symlink-install`, then restart
terminal 2. Python and settings changes only need terminal 3 restarted.

## What was found (in order)

1. **OpenHaptics renders no force in any process that loads ROS 2's libraries.** liblttng-ust and
   libnuma, pulled in by rcl → tracetools, each trigger it. There are no errors: the 1 kHz loop
   runs and HD_FORCE_OUTPUT reads enabled. So `omni_state` never produced any force, while
   position and buttons always worked. Proven with the lift test
   (`scripts/touch_diagnostics/`): a plain OpenHaptics program lifts the pen 150-165 mm at 1 N,
   and the same program merely *linked* against librcl lifts it 0 mm.
   **Fix:** the Touch driver is split in two. `src/touch_server.cpp` links only HD/HDU and
   owns the device; `src/touch_bridge.cpp` is a ROS node publishing the same `/phantom/*`
   topics as `omni_state`. They talk over UDP on localhost (`include/ls_thesis/touch_packets.hpp`;
   state → 47101, forces and scene → 47102). They start together with
   `launch/touch_driver.launch.py`, which `ur5e_teleop.launch.py` (input:=touch) and
   `scripts/launch_system.sh` now use. Never link anything ROS into `touch_server`.
2. **omni_state's damping term shook the pen.** It subtracted 1 N s/m of a 20 Hz-filtered velocity,
   which lags enough at ~25 Hz to push along the motion (2 N while the cue asked for 0.3 N).
   It was removed from `touch_server`.
3. **Springs computed in the Python teleop arrived too late.** The round trip Touch → bridge →
   teleop → bridge → server took tens of ms, so the cues shook the pen. The walls are now sent
   as a shape (`TouchScenePacket`) and rendered in `touch_server`'s 1 kHz loop.
4. **An attracting spring shakes a hand-held pen.** This held even at 1 kHz: the vendor's
   HelloHapticDevice gravity well shook too, while TouchCheckup's Box and Sphere (one-sided
   walls) are smooth. So the RCM-point detent was replaced by an open-loop click.
5. Along the way, `~/.3dsystems/config` turned out to be for the *other* Touch (SN 19039000821).
   `Touch_HeadlessSetup` was rerun for SN 22129000559 (connected now); the old config is in
   `~/.3dsystems/config.backup-SN19039000821-2026-10-19`. Rerun the setup whenever the units are
   swapped. Tools: `~/Downloads/TouchDriver_2025_12_10+1/TouchDriver_2025_12_10/bin`.

## Open / next steps

1. **Try the latest version on the real Touch.** Check that the crossing click is felt, the walls are
   smooth (not shaking), and the RCM tap is fine. Record each run (GREY): `/phantom/applied_force` in
   the bag is what the Touch actually rendered, which makes any shake easy to diagnose.
2. **The singularity push has not been felt on the real arm yet.** It is computed in the teleop and
   goes over ROS, and it depends on the robot's pose, so in principle it can shake like the old
   springs did. It is low-passed (30 ms) and a "push away", not a spring towards a point, but watch
   it. If it shakes, lower `TOUCH_HAPTIC_SINGULARITY_FORCE`, or send it as a scene too.
3. If the walls shake: lower `TOUCH_HAPTIC_WALL_STIFFNESS` (150 N/m now). Damping would have to be
   rendered in `touch_server` from a velocity that does not lag, so don't reuse omni_state's.
4. Tune the strengths by feel. The crossing click (`TOUCH_HAPTIC_CROSSING_FORCE`) and the tap
   (`TOUCH_HAPTIC_PULSE_FORCE`) are both 0.1 N.
5. **Commit and tag V7.** Part 1 (RCM sideways scaling) and part 2 (haptics + new Touch driver)
   are both uncommitted on branch `basic-mapping`; VERSIONS.md has V7 marked "not tagged yet".
   Uncommitted: CMakeLists.txt, README.md, docs (RUN_UR16E_TOUCH, VERSIONS, this file),
   launch (ur5e_teleop + new touch_driver), ls_thesis (teleop_settings, ur5e_common, new haptics.py),
   scripts (launch_system.sh, new touch_diagnostics/), src (ur5e_touch_teleop.py, new
   touch_server.cpp, touch_bridge.cpp), include/ls_thesis/touch_packets.hpp.

## Cautions for testing

- **UDP is not isolated by `ROS_DOMAIN_ID`.** A mock-test teleop sends its wall shape to whichever
  `touch_server` listens on 127.0.0.1:47102, which is the user's real Touch. Run test teleops with
  `-p haptic_scene_port:=47999`.
- On the shared laptop, only signal processes whose `/proc/<pid>/environ` has the test
  `ROS_DOMAIN_ID`. A bare `pgrep` once picked (and stopped) the live teleop node.
- The driver keeps the last force only for 0.25 s (`touch_server`'s timeout), so a crashed teleop
  cannot leave a force on.

## Files

| File | What |
|---|---|
| `src/touch_server.cpp` | Device loop (OpenHaptics only): state out, forces in, walls rendered at 1 kHz, cap + slew limit, 0.25 s timeout |
| `src/touch_bridge.cpp` | ROS side: `/phantom/state`, `/button`, `/pose`, `/joint_states`, `/applied_force`; forwards `/phantom/force_feedback` |
| `include/ls_thesis/touch_packets.hpp` | UDP packet layouts and ports (the Python scene format is `"<II3d3d7d"`) |
| `launch/touch_driver.launch.py` | Starts both |
| `ls_thesis/haptics.py` | Taps (`PulsePlayer`), singularity push, force limiting |
| `src/ur5e_touch_teleop.py` | `update_haptics`, `send_scene` (walls), `check_crossing` (click), `singularity_haptic_force` |
| `ls_thesis/ur5e_common.py` | `SingularityGuard.twist_gradient` (direction out of the singularity) |
| `scripts/touch_diagnostics/` | Lift tests that found the OpenHaptics + ROS problem |
