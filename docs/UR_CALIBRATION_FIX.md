# Loading the UR16e's own kinematic calibration (`ur_calibration_extract`)

Added 2026-11-03. Code: `src/ur_calibration_extract.cpp`, `vendor/ur_calibration/`,
`launch/ur_calibration_extract.launch.py`. Plan context: `docs/lab_plans/07_remaining_experiments.md` §1.

## TL;DR: how to use it

```bash
# 1. Robot powered on (brakes can stay on; no remote control / External Control program needed)
ros2 launch macro_micro_robot ur_calibration_extract.launch.py            # robot_ip defaults to 192.168.0.100
#    -> config/calibrations/ur16e_kinematics_<date>.yaml (in the source tree, commit it)
#    Look for "Check passed" and the "Nominal model vs this arm" line in the output.

# 2. Run with it
scripts/launch_system.sh ur16e-touch --calibration-file=$HOME/ros2_ws/src/MacroMicroSurgicalRobot/config/calibrations/ur16e_kinematics_<date>.yaml
#    Driver startup must now print   "Calibration checked successfully."
#    instead of                      "The calibration parameters of the connected robot don't match ..."
```

Then, before trusting any NDI number: redo the hand-eye and tool-tip calibrations (§6).

## 1. The problem

ROS (TF, MoveIt Servo, the teleop's RCM, the recorder, the hand-eye) computes `tool0` from the URDF. Unless a
`kinematics_params_file` is passed, the URDF uses ur_description's **nominal UR16e** model. Each UR arm is
measured at the factory, and its own D-H corrections are stored **on the controller**. The controller uses
them for its own TCP (`/tcp_pose_broadcaster/pose`, the pendant). Every run so far gave ROS the nominal model.

The driver warned about this on every connection. **125 ROS logs (2026-10-01 to 10-30) contain**

> The calibration parameters of the connected robot don't match the ones from the given kinematics config
> file. Please be aware that this can lead to critical inaccuracies of tcp positions. Use the ur_calibration
> tool ...

and none contain `Calibration checked successfully`. (`grep -rl "don't match" ~/.ros/log | wc -l`)

The other evidence is in `07_remaining_experiments.md` §1.1: TF vs controller tool0 drifting 0.6–1.2 mm
within a run (`META.yaml tcp_offset.check_max_mm`), the 0.17–3.7 mm NDI pivot offset (R7.9), and the
pose-dependent 1.5–1.9 mm micro tip residual (R8.4).

## 2. Why the driver's `ur_calibration` didn't work: two bugs

Versions here: driver `src/Universal_Robots_ROS2_Driver` at 3.8.0-7-g32bb4c1 (built from source), against
apt `ros-jazzy-ur-client-library` 2.15.2. Both bugs are fixed upstream on the `jazzy` branch (3.10.0,
`67578e0`), after the 3.8.0 we have.

### Bug 1: it never connects (upstream `0089621`, PR #1981, backport #1984, 2026-09-15)

`ros2 launch ur_calibration calibration_correction.launch.py` loops forever printing
`Stream is invalid. Connect it before trying to read!`.

- In ur_client_library 2.15.2, the primary-interface socket is opened only in `Pipeline::init()`:
  `Pipeline::init()` → `URProducer::setupProducer()` → `URStream::connect()`
  (`/opt/ros/jazzy/include/ur_client_library/comm/pipeline.h:352`, `producer.h:154`).
- `Pipeline::run()` only starts the reader threads.
- 3.8.0's `ur_calibration/src/calibration_correction.cpp` (`CalibrationCorrection::run()`) calls
  `pipeline.run()` and never `init()`. It reads from a socket that was never opened, which is the warning
  above. Older client libraries connected elsewhere, so this broke silently when the library changed.
- Upstream fix: add `pipeline.init();` before `pipeline.run();`.

### Bug 2: once connected, it writes wrong link rotations (upstream `81df755`, PR #2030, backport #2037, 2026-10-20)

**This one is not in the earlier plan, which said a one-line patch was enough. It isn't.** With only bug 1
fixed, you would get a file that loads without complaint, passes the driver's hash check, and is still wrong.

- `Calibration::toYaml()` wrote each link's rotation as `rot.eulerAngles(0, 1, 2)`. Those are intrinsic
  X-Y-Z angles: R = Rx(roll)·Ry(pitch)·Rz(yaw).
- ur_description puts roll/pitch/yaw straight into URDF `<origin rpy>`, which is fixed-axis X-Y-Z:
  R = Rz(yaw)·Ry(pitch)·Rx(roll).
- The two agree only when at most one angle is non-zero. Every link that combines a nominal quarter-turn
  with a calibration delta (most UR links) comes out slightly rotated.
- The driver's hash check can't catch this: the hash is copied from the controller, not computed from
  the angles.

How big it is depends on the arm. I measured it by re-expressing correct files in the 3.8.0 convention
(`eulerAngles(0,1,2)` ≡ scipy `as_euler('XYZ')`) and comparing flange poses over 5000 random joint
configurations:

| Calibration | 3.8.0 file vs correct: flange position | rotation |
|---|---|---|
| Synthetic UR16e, typical ~1 mm / ~1 mrad deltas (upstream's test set) | max **3.2 mm**, RMS 1.7 mm | max 0.37° |
| Real UR10e controller calibration (upstream test data) | max 0.007 mm | < 0.001° |

On the synthetic UR16e, the error the bug adds is about as large as the whole calibration correction
(5.8 mm, §5). So "the calibration didn't help" could have been this bug.

## 3. Why the fix lives in this package, not the driver repo

- Edit scope: this project changes only `src/MacroMicroSurgicalRobot`. The driver checkout is shared,
  and it already has an unrelated local edit (`ur_robot_driver/launch/ur_control.launch.py`).
- Patching the driver gets overwritten by the next `git pull` / re-clone. Updating the driver to 3.10.0
  would also pull in unrelated driver changes in the middle of the thesis experiments. That's a bigger
  risk than a self-contained tool.
- Keeping it here puts the tool in the thesis repo, versioned with the code that uses its output.

## 4. What was added

| File | What | Changed from upstream? |
|---|---|---|
| `vendor/ur_calibration/include/ur_calibration/calibration.hpp` | D-H → URDF conversion (header) | **No.** Byte-identical to upstream `jazzy` @ `67578e0` |
| `vendor/ur_calibration/src/calibration.cpp` | D-H → URDF conversion, **includes the bug 2 fix** | **No.** Byte-identical to upstream `jazzy` @ `67578e0` |
| `vendor/ur_calibration/LICENSE` | BSD-3 licence of the above | Copy |
| `src/ur_calibration_extract.cpp` | Replaces `calibration_correction.cpp` and `CalibrationConsumer`: connects (**bug 1 fix**: `pipeline.init()`), converts, checks, writes | New, based on upstream |
| `launch/ur_calibration_extract.launch.py` | Defaults for the lab UR16e, output into `config/calibrations/` | New |
| `CMakeLists.txt`, `package.xml` | Build it; depend on `ur_client_library`, `yaml_cpp_vendor` | |
| `src/trajectory_recorder.py`, `launch/ur16e_teleop.launch.py` | `META.yaml` gets a `kinematics:` block (file, nominal/calibrated, sha256, hash) | |

Check the vendored files are still identical to upstream:
```bash
cd ~/ros2_ws/src/Universal_Robots_ROS2_Driver && git fetch origin
for f in include/ur_calibration/calibration.hpp src/calibration.cpp; do
  git show 67578e0:ur_calibration/$f | diff -q - ~/ros2_ws/src/MacroMicroSurgicalRobot/vendor/ur_calibration/$f && echo "$f identical"; done
```
sha256: `calibration.hpp` 1d9ec962…f1d42e, `calibration.cpp` bd8809f3…6f0c72 (full sums in
`vendor/ur_calibration/README.md`).

`CalibrationConsumer` was not vendored. Upstream's consumer converts straight to YAML and throws away the
controller's raw D-H numbers, and the check below needs them. `KinematicsConsumer` in
`ur_calibration_extract.cpp` does the same thing but keeps them.

### What the extractor does beyond upstream

1. **Bounded connect / receive.** 3 connect tries 2 s apart, then up to 10 s for the kinematics package,
   then exit 1. Upstream retries forever. (Against an IP that doesn't answer at all, each try can take
   the OS TCP timeout, up to about 2 minutes.)
2. **An independent check before writing.** It rebuilds the arm from the YAML exactly the way URDF does
   (`Rz(yaw)·Ry(pitch)·Rx(roll)`, then the joint rotation), and from the controller's raw D-H parameters
   (`Rz(θ+q)·Tz(d)·Tx(a)·Rx(α)`, the controller's own model). It compares the two over 10 000 random joint
   configurations, and **refuses to write the file** if any matrix element differs by more than 1e-9.
   Upstream's unit test compares the YAML only with ur_calibration's own corrected chain. This check doesn't
   depend on any ur_calibration code, so it would catch bug 2, or any later regression, on this arm's real
   numbers.
3. **How wrong the nominal model was.** It compares with `ur_description/config/<ur_type>/default_kinematics.yaml`
   and prints, and writes into the file, the flange position/rotation error of the nominal model for this
   arm. That's the kinematic error every earlier run carried, a number for the RCM error budget (F7.6).
4. **Self-describing output.** The file header (YAML comments, ignored by ur_description) records the date,
   source, check result, nominal-vs-calibrated numbers and the controller's **raw D-H parameters**. You can
   re-check it later without the robot: `ur_calibration_extract.launch.py dh_file:=<that file>`.
5. **Exit status 0 only if the file was written and passed the check.**

## 5. Tests (2026-11-03, no robot; on ROS_DOMAIN_ID 77)

| Test | Result |
|---|---|
| Builds in `macro_micro_robot`, no new warnings | ✓ |
| Real UR10e controller calibration (upstream test data), via `dh_file` | Check passed: max element 1.7e-13, max position 1.7e-10 mm. Nominal UR10e vs that arm: max 3.6 mm, RMS 1.9 mm, 0.62° |
| Synthetic UR16e (~1 mm / ~1 mrad deltas) | Check passed: max element 1.4e-15. Nominal UR16e vs that arm: max 5.8 mm, RMS 3.1 mm, 0.35° |
| Re-check its own output file via `dh_file` | Identical output (apart from the date/source lines) |
| `xacro ur.urdf.xacro ur_type:=ur16e kinematics_params:=<output>` | Parses; joint origins match the file |
| Nothing listening on port 30001 (127.0.0.1) | 3 tries, exit 1 after 4 s, no file written |
| `kinematics_summary()` in the recorder: no file / nominal / calibrated / missing file | All four give the expected META block |

**Not tested:** against the real controller (needs the lab). The first lab run is the real test: it should
print `Check passed`, and the driver should then print `Calibration checked successfully.`

**If it doesn't connect on the real robot:** `nc -zv 192.168.0.100 30001`. Check the pendant: PolyScope
Settings → Security → Services. The primary interface (30001) must be enabled; read-only access is enough.

## 6. After loading it: what must be redone

These were all fitted through the nominal model, so they've partly absorbed its error. Mixing them with
the calibrated model can make results look *worse* (07 §1.1):

1. Wand hand-eye: `scripts/calibration/wand_tool_calibration.py` from a fresh wrist-jog recording.
2. Micro tip ball: `scripts/calibration/tip_ball_calibration.py`, then update `config/tools.yaml`.
3. Make the calibrated file the default in `ur16e_teleop.launch.py` (the `default_kinematics` expression)
   only after the RD1 replays show it helps.

What the before/after should show:
- the driver startup line (§1). At the default warn log level only the mismatch ERROR is visible, so
  success = that ERROR is absent;
- `META.yaml kinematics.model: calibrated`, with the hash matching the controller's;
- `thesis/run_tools/kinematics_check.py <run>`: tool0 computed through the file vs the controller's TCP, sample
  by sample from the bag (same RTDE packet). **Measured 2026-11-03:** calibrated spread 0.000 mm /
  0.000°; nominal 0.847 mm / 0.374° (FastChecking replay) and 0.638 mm / 0.371° (a 10-26 replay, which
  the calibrated file also matches exactly: the controller's calibration hasn't changed since).
- **Not** `tcp_offset.check_max_mm`, which this doc first predicted would fall to ~0. It compares
  against TF interpolated from ~20 Hz, so while moving it mostly measures timing: FastChecking gave
  1.24–1.38 mm (10-26, nominal) and 1.08 mm (11-03, calibrated).

## 7. When this can go

Once `src/Universal_Robots_ROS2_Driver` is updated to ≥ 3.10.0 (or any version containing both `0089621`
and `81df755`), the stock `ur_calibration` works. The extractor stays useful for the independent check and
the nominal-vs-calibrated numbers. Files written by the stock 3.10 tool can be checked with
`dh_file:=` only if the raw D-H numbers are known, which that tool doesn't save.

**Old files:** any calibration YAML written by `ur_calibration` before the bug 2 fix has the wrong
rotations. `config/lab_ur5e_1_calibration.yaml` (the old project's UR5e) may be one of them. It isn't used
for the UR16e, so it was left alone.

## 8. Why this matters for the thesis

- Same pattern as the other interface failures (Servo delivering 60 %, OpenHaptics losing forces under
  ROS 2). The off-the-shelf stack defaults to a nominal model. The warning was an ERROR line buried in
  startup output. The tool meant to fix it was broken twice: once by a client-library API change (silent
  failure), once by an Euler-convention mismatch (silently wrong output, invisible to the driver's own
  hash check).
- A constraint the controller reports satisfying is only as good as the model it's expressed in. An
  independent check (here, rebuilding from the raw D-H numbers; for the RCM, NDI) is what catches it.
