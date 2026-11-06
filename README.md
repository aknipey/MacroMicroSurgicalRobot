# A Low-Cost Teleoperable Surgical Robot with a Macro-Micro Structure and a Continuum Tip

ROS 2 Jazzy package for teleoperating a Universal Robots UR16e (macro) and a tendon-driven
continuum micro-manipulator (micro) with a 3D Systems Touch haptic stylus. The arm is driven
through MoveIt Servo. It supports a remote centre of motion (RCM) mode, singularity handling,
haptic cues, and run recording and replay.

## Install

You need Ubuntu 24.04 and ROS 2 Jazzy, plus a real-time kernel for the real arm
([UR driver docs](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)).

Clone into `~/ros2_ws/src/MacroMicroSurgicalRobot`. That exact path is the default for
`LOG_DIR` and `TOOL_LIBRARY` in `teleop_settings.py`.

These go in the same workspace and are not part of this repo:

| Package | For |
|---|---|
| [Universal_Robots_ROS2_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) | Arm driver, `ur_moveit_config` |
| `omni_msgs` | Touch messages, from a ROS 2 Geomagic Touch driver |
| `motor_angles_msg` | Micro-module messages: a ROS 2 port of [the original](https://github.com/LMBScott/motor_angles_msg/) |
| [Serial_Control](https://github.com/LMBScott/Serial_Control/) | Micro-module Arduino bridge (`ros2_bridge/`) |
| [OpenHaptics SDK](https://support.3dsystems.com/s/article/OpenHaptics-for-Linux-Developer-Edition-v34?language=en_US) | `touch_server`; skipped at build time if absent |

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

There are no automated tests.

## Run

`scripts/launch_system.sh` starts everything for a scenario, in order, and stops it all on
Ctrl+C. Run `scripts/launch_system.sh help` for every mode and flag.

```bash
cd ~/ros2_ws
S=src/MacroMicroSurgicalRobot/scripts/launch_system.sh
$S ur16e-touch --mock --with-rviz    # simulated arm, real Touch
$S ur16e-touch --with-micro --calibration-file=<yaml>   # real arm + micro module
$S ur16e-keyboard --mock --with-rviz # simulated arm, keyboard
$S touch                             # micro module only
$S display                           # RViz model, no hardware
```

On the real arm, start External Control on the teach pendant once the driver is up.

**Stylus controls (arm)**

| Input | Action |
|---|---|
| Hold WHITE | Drive the arm; release to stop and re-centre |
| Double-click WHITE | Cycle motion scale (1:1, 1:5, 1:20) |
| Click GREY | Start or stop recording |
| WHITE + click GREY | Enter or leave RCM mode |
| Hold GREY 1 s | Switch the stylus between arm and micro module |

Replay a recording with `ros2 run macro_micro_robot trajectory_player.py latest --speed 0.5`,
or go to a safe pose with `... trajectory_player.py home`.

Tuning values live in `macro_micro_robot/teleop_settings.py`. Tools live in `config/tools.yaml`.

> Switch off servo power on the micro module before unplugging its control USB cable.

**More detail:**
[step-by-step guide for the real arm](docs/RUN_UR16E_TOUCH.md) ·
[UR16e calibration extraction](docs/UR_CALIBRATION_FIX.md)

## Layout

| Path | Contents |
|---|---|
| `macro_micro_robot/` | Shared Python: settings, Servo helpers, singularity guard, RCM geometry, haptics |
| `src/` | Nodes: Touch teleop, keyboard teleop, recorder, player, velocity limiter, UR emulator, Touch driver, micro-module control |
| `include/` | C++ headers |
| `launch/`, `urdf/`, `meshes/`, `rviz/` | Launch files and the micro-module model |
| `config/` | Tool library, calibrations, custom URScript |
| `scripts/` | Launcher, replay batches, calibration tools, diagnostics |
| `thesis/` | Figures and offline analyses of the recorded runs (`logs/`, not in git) |
| `vendor/ur_calibration/` | Unmodified upstream code; don't edit it |
| `docs/` | Operating guides, parts list, architecture diagram |
| `claude-notes/` | Working notes: version history, lab plans, investigations |

## Licence

MIT, see [LICENSE](LICENSE). `vendor/ur_calibration/` and `launch/ur_rsp_smooth_speedj.launch.py`
are BSD-3-Clause, from Universal Robots' driver. The project builds on Lachlan Scott's 2023 work
in this repository, which was published without a licence.

Original installation notes by Mustafa Sevinc.
