# A Low-Cost Teleoperable Surgical Robot with a Macro-Micro Structure and a Continuum Tip

> Honours thesis overview (summary, results, how the repositories fit together): [Honours-Thesis](https://github.com/aknipey/Honours-Thesis)

ROS 2 Jazzy package that teleoperates a Universal Robots UR16e (the macro stage) and a
tendon-driven continuum micro-manipulator (the micro stage) with a 3D Systems Touch haptic
stylus. The arm is driven through MoveIt Servo. On top of plain teleoperation the package provides:
- a remote centre of motion (RCM) mode for keyhole work;
- a singularity guard;
- haptic cues on the stylus;
- a library of calibrated tools;
- recording of every run, and replay of a run's exact stylus input.

```
Touch ─► touch_server ─UDP─► touch_bridge ─► ur16e_touch_teleop ─► MoveIt Servo ─► joint_velocity_limiter ─► UR16e
                                                   │  (RCM, guard, haptics)                 (position / velocity)
                                                   └─► micro_module_control ─► serial bridge ─► micro module
```

## Repository layout

| Path | Contents |
|---|---|
| [`src/`](src) | Nodes: Touch and keyboard teleop, trajectory recorder and player, stylus replay, joint velocity limiter, UR emulator, Touch driver (`touch_server` + `touch_bridge`), micro-module control, tool library CLI |
| [`macro_micro_robot/`](macro_micro_robot) | Shared Python: **all tuning values** ([`teleop_settings.py`](macro_micro_robot/teleop_settings.py)), Servo and kinematics helpers, singularity guard, tool library, haptics |
| [`include/macro_micro_robot/`](include/macro_micro_robot) | C++ headers |
| [`launch/`](launch), [`urdf/`](urdf), [`meshes/`](meshes), [`rviz/`](rviz) | Launch files and the micro-module model |
| [`config/`](config) | Tool library (`tools.yaml`), arm and tool calibrations, smoothed speedj URScript |
| [`scripts/`](scripts) | `launch_system.sh` (start a session); `calibration/` (kinematics check, hand-eye, tool tip and tool-line calibration); `touch_diagnostics/` |
| [`thesis/results/`](thesis/results/README.md) | The analysis behind every number and data figure in the thesis report, one folder per result, in report order |
| [`data/`](data/README.md) | The recordings the results use (`data/recordings/`; small files in git, CSVs and bags as a separate archive) and their index |
| [`docs/`](docs) | Guides, lab log, report supplement, parts list, architecture diagram |
| [`vendor/ur_calibration/`](vendor/ur_calibration) | Unmodified upstream code from the UR driver (do not edit) |
| `logs/` | The recorder's working folder on the lab computer (not in git) |

## Install

You need Ubuntu 24.04 and ROS 2 Jazzy. The real arm also needs a real-time kernel; see the
[UR driver docs](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver).

Clone into `~/ros2_ws/src/MacroMicroSurgicalRobot`. `LOG_DIR` and `TOOL_LIBRARY` in
`teleop_settings.py` assume that path. These packages go in the same workspace:

| Package | For |
|---|---|
| [Universal_Robots_ROS2_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) | Arm driver, `ur_moveit_config` |
| `cartesian_control_msgs` | Build dependency of the legacy `ur16e_control` node |
| `omni_msgs` (from a ROS 2 Geomagic Touch driver) | Touch messages |
| `motor_angles_msg` | Micro-module messages; a ROS 2 port of [the original](https://github.com/LMBScott/motor_angles_msg/) |
| [Serial_Control](https://github.com/LMBScott/Serial_Control/) | Micro-module Arduino bridge |
| `ndi_tracking` | NDI optical tracking of runs and tool calibration (optional) |
| [OpenHaptics SDK](https://support.3dsystems.com/s/article/OpenHaptics-for-Linux-Developer-Edition-v34?language=en_US) | `touch_server`; skipped at build time if absent |

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to macro_micro_robot
source install/setup.bash
```

With `--symlink-install`, changes to Python files and `teleop_settings.py` take effect the next
time the nodes start, with no rebuild.

## Run

`scripts/launch_system.sh` starts a whole session in order and stops it all on Ctrl+C.
`scripts/launch_system.sh help` lists every mode and flag.

```bash
cd ~/ros2_ws
S=src/MacroMicroSurgicalRobot/scripts/launch_system.sh
$S ur16e-touch --mock --with-rviz                              # simulated arm, real Touch
$S ur16e-touch --calibration-file=<yaml> --with-micro          # real arm + micro module
$S ur16e-keyboard --mock --with-rviz                           # simulated arm, keyboard
$S touch                                                       # micro module only
$S display                                                     # RViz model, no hardware
```

On the real arm, start the External Control program on the teach pendant once the driver is up.
New to the lab setup? Follow [docs/RUN_UR16E_TOUCH.md](docs/RUN_UR16E_TOUCH.md) step by step.

### Stylus controls

| Input | Action |
|---|---|
| Hold WHITE | Drive the arm (release to stop and re-centre) |
| Double-click WHITE | Cycle the motion scale (1:1, 1:5, 1:20) |
| Click GREY | Start or stop recording a run |
| WHITE + click GREY | Enter RCM mode (latches the tool tip as the pivot); again, retracted, to leave |
| Hold GREY 1 s | Switch the stylus between the arm and the micro module |
| Type a number + Enter (launcher terminal) | Switch to that tool from `config/tools.yaml` |

> Switch off servo power on the micro module before unplugging its USB cable.

### Recording and replay

Each recording lands in `logs/NEW_RUN/`. It holds the path CSVs, the raw stylus, a rosbag and
`META.yaml` with the code version and every setting. The next recording replaces it, so rename the
folder to keep the run.

```bash
ros2 run macro_micro_robot trajectory_player.py home                 # safe ready pose
ros2 run macro_micro_robot trajectory_player.py latest --speed 0.5   # replay joint trajectory
ros2 launch macro_micro_robot ur16e_teleop.launch.py input:=replay   # then:
ros2 run macro_micro_robot stylus_replay.py V9b/SomeRun              # replay the same hand input
```

### Calibration

| Task | How |
|---|---|
| Arm kinematics (do once per arm) | `ros2 launch macro_micro_robot ur_calibration_extract.launch.py`, then pass the file with `--calibration-file=` ([why](docs/UR_CALIBRATION_FIX.md)) |
| NDI wand as a tool (hand-eye) | `scripts/calibration/wrist_jog.py` + `wand_tool_calibration.py` |
| Micro-module tip | `scripts/calibration/micro_tool_calibration.py` |
| Add a tool to the library | `ros2 run macro_micro_robot tool_library.py add ...` |

Procedures are in [docs/THESIS_APPENDIX.md §5](docs/THESIS_APPENDIX.md#5-calibration-procedures).

## Documentation

| File | What |
|---|---|
| [docs/RUN_UR16E_TOUCH.md](docs/RUN_UR16E_TOUCH.md) | Step-by-step guide for running the real arm |
| [docs/VERSIONS.md](docs/VERSIONS.md) | Lab log: every stack version (V1–V9c), what changed and why, with its git tag or commit |
| [docs/THESIS_APPENDIX.md](docs/THESIS_APPENDIX.md) | Report supplement: architecture, settings, hardware, calibration, risk, reproducibility |
| [docs/UR_CALIBRATION_FIX.md](docs/UR_CALIBRATION_FIX.md) | Why the driver's calibration tool is replaced |
| [docs/HAPTICS_HANDOFF.md](docs/HAPTICS_HANDOFF.md) | Touch force-feedback findings |
| [thesis/results/README.md](thesis/results/README.md) | The results: index by report section, how to rerun them |
| [data/README.md](data/README.md) | The recordings: what is kept, what is in git, the archive |

## Credits

The micro module and its original ROS 1 control code are Lachlan Scott's work (2023, this
repository). The original installation notes are by Mustafa Sevinc. `vendor/ur_calibration/` and
`launch/ur_rsp_smooth_speedj.launch.py` come from Universal Robots' ROS 2 driver (BSD-3-Clause).
