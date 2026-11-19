# 7. Testing plan: what is left, what to redo, and the kinematics fix

**Next lab session: `08_next_session_V9b.md`** (exact steps). Session A was done 2026-11-03.

Ordered by how much each test strengthens the thesis per hour of lab time. **Check the submission date first**
and cut from the bottom. Each test names the threshold or claim it settles (R-IDs from `02_results_inventory.md`,
C-IDs from `01_story_and_novelty.md`).

Contents:
1. The factory-kinematics problem and how to fix it (§1). Do this first: every later NDI number depends on it.
2. Tests still to do (§2): E0–E10.
3. Tests to redo, and tests *not* to redo (§3).
4. Lab session plan (§4).
5. Data hygiene (§5).

Before any lab test: commit, so `code_version` in META is not `-dirty`. Copy `logs/NEW_RUN` into a version
folder after every run, because the next recording deletes it. (The 2026-10-30 16:43 run the buzz analysis uses is
already safe as `logs/V7/NoBuzzArea`.)

---

## 1. Factory kinematics: what the problem is and how to fix it

### 1.1 What is actually wrong

- ROS computes `tool0` (TF, and everything the teleop, recorder and hand-eye use) from the **nominal UR16e model**
  in the URDF. No `kinematics_params_file` has been loaded on any run (VERSIONS.md 2026-11-02 rename notes).
- Every UR arm is calibrated at the factory, and the robot-specific DH corrections live **on the controller**. The
  controller uses them for its own TCP (`/tcp_pose_broadcaster/pose`, the pendant's TCP). ROS does not.
- Evidence it matters:
  - TF vs controller tool0 differ by 0.1–0.25° (`META.yaml tcp_offset.rotvec_deg`).
  - That difference changes by up to 0.6–1.2 mm within a run (`check_max_mm`), so the error depends on pose.
  - The NDI-measured pivot is 0.17–3.7 mm off the model's pivot, mostly along base z (R7.9).
  - The micro-module tip-ball fit has a pose-dependent 1.5–1.9 mm residual that does not grow with speed (R8.4).
- **The hand-eye calibration and every tool tip in `config/tools.yaml` were fitted through the same nominal
  model.** They have partly absorbed its error. After the fix they **must be redone**, or the fix can look like it
  made things worse.

### 1.2 Why `ur_calibration` failed, and the fix (found and fixed 2026-11-03)

**Done, in this package:** `ros2 launch macro_micro_robot ur_calibration_extract.launch.py`. Full write-up,
tests and provenance: `docs/UR_CALIBRATION_FIX.md`. The driver repo was not touched.

`ros2 launch ur_calibration calibration_correction.launch.py ...` failed with
`Stream is invalid. Connect it before trying to read!`. Our driver (3.8.0, built from source against apt
ur_client_library 2.15.2) has **two** bugs, both fixed upstream on `jazzy` after 3.8.0:

1. **Never connects** (upstream `0089621`). In client library 2.15.2 only `Pipeline::init()` opens the
   socket; `calibration_correction.cpp` calls `pipeline.run()` without it.
2. **Writes wrong link rotations once connected** (upstream `81df755`). `toYaml()` used intrinsic X-Y-Z
   Euler angles, but URDF `rpy` is fixed-axis X-Y-Z. The file loads and passes the driver's hash check, but
   it is off by up to 3.2 mm at the flange on a synthetic UR16e calibration.
   *The earlier version of this plan proposed only the one-line `init()` patch. That would have given a
   silently wrong calibration.*

The fix: `src/ur_calibration_extract.cpp`, with upstream jazzy's `calibration.hpp/.cpp` vendored
byte-identical in `vendor/ur_calibration/`. It adds:
- bounded connect / receive;
- an independent check: the YAML read as URDF must reproduce the controller's raw D-H model to 1e-9 over
  10 000 poses, or no file is written;
- the nominal-vs-this-arm flange error, a direct number for the error budget (F7.6);
- the raw D-H numbers in the file header, so the file can be re-checked offline (`dh_file:=`).

Extra evidence for the thesis: the driver printed "The calibration parameters of the connected robot
don't match ..." as an ERROR in **125 session logs (10-01 to 10-30)**, and never "Calibration checked
successfully".

- Upstream PRs already exist (#1981, #2030), so there's nothing to upstream. Cite them.
- **If it doesn't connect:** `nc -zv 192.168.0.100 30001`, and check PolyScope Settings → Security → Services
  (primary interface enabled; read-only access is enough). The robot does not need remote control.

### 1.3 Using it

1. Launch with `kinematics_params_file:=<that yaml>` (`launch_system.sh --calibration-file=`). The driver must
   print `Calibration checked successfully.`, and `META.yaml kinematics.model` must say `calibrated`. The launch
   default is still the nominal model. Make the calibrated file the default only after step 4 shows it helps.
2. Confirm with `thesis/run_tools/kinematics_check.py <run>` (tool0 through the file vs the controller's TCP,
   same RTDE packet). Done 11-03: calibrated spread 0.000 mm / 0.000°, nominal 0.85 mm / 0.37°. The
   recorder's `check_max_mm` is not a kinematics measure: it is dominated by TF timing while moving
   (1.08 mm calibrated vs 1.24–1.38 mm nominal on FastChecking).
3. Redo the wand hand-eye (`scripts/calibration/wand_tool_calibration.py` from a fresh wrist-jog `robot-tcp --wand-is-tool`
   recording). Then redo the micro tip ball (`scripts/calibration/tip_ball_calibration.py`) and update `tools.yaml`.
4. Replay a subset of the 2026-10-26 NDI set (see RD1).

### 1.4 Do it offline first (E0, no lab time)

Every bag since V3 already records **`/tcp_pose_broadcaster/pose`**: the controller's own TCP, computed with the
calibrated kinematics, at ~500 Hz, next to `/tf`, `/joint_states` and the NDI wand pose. So the effect of the fix
can be **predicted from data you already have**:

- For each of the 14 NDI replays, fit the hand-eye (base→camera, flange→wand) twice:
  - once with TF tool0 (factory model);
  - once with the controller pose (calibrated model; the fixed pendant TCP is absorbed into the flange→wand fit,
    so its value doesn't matter).
- Compare the fit residuals and the recomputed NDI RCM error and pivot offset for each set.
- Prediction:
  - SlowChecking drops from 2.9 mm towards RCMOnly's level.
  - The RCMOnly ↔ SlowChecking pivot-offset spread (1.33 vs 3.7 mm) collapses.
- Write it as `thesis/earlier/calibrated_kinematics_offline.py`, reusing the math in
  `thesis/run_tools/rcm_offset_check.py`.
- Caveat: `rcm_error` (the controller's own number) was computed online through TF. It stays a "factory model"
  number, and that is fine, because it shows the controller holding *its* point.

**Why E0 matters:** it turns R7.9 from "probably the kinematics" into a measured decomposition even if lab access
disappears. The lab rerun (RD1) then becomes the *confirmation* of a prediction, which is stronger science than
the other way round.

### 1.5 Stretch: your own NDI-based kinematic identification (E0b)

- If `ur_calibration` can't be made to work, or for a richer chapter: identify DH corrections yourself.
- Method: least squares on (joint angles → NDI wand pose) pairs over many poses, with base→camera and flange→wand
  as nuisance parameters. You already have tens of thousands of such pairs.
- Gives a 3-way comparison: nominal vs UR factory calibration vs NDI-identified.
- **Time-box it to a day.** Only do it if the core thesis is written. Identifiability is fiddly: the
  base→camera transform trades off against the first joints, and the flange→wand transform against the wrist.

### 1.6 Why this is interesting in the thesis

- **It fits the main story exactly.** The error sat at an interface: ROS ↔ controller, where the kinematic model
  is the contract between them. The off-the-shelf driver ships the nominal model by default, and the tool meant to
  fix that was silently broken by a client-library API change. Same pattern as Servo ↔ UR (60 % delivery) and SDK
  ↔ ROS runtime (no forces).
- **It makes C7 a quantified result:** the **RCM error budget**.

  | Source | Factory model | Calibrated |
  |---|---|---|
  | Controller (own `rcm_error`) | 0.05–0.23 mm | (unchanged) |
  | Kinematic model | ~0.2–3.7 mm (R7.9) | E0 / RD1 |
  | Hand-eye + NDI measurement floor | ~0.8 mm (R4.6) | redo |
  | Residual (compliance, wand seat) | ? | ? |

  Make it a stacked-bar figure per trajectory set (new **F7.6**). Even if calibration *doesn't* help, the budget
  shows where the error is. That is a result, not a failure.
- **A clear evaluation lesson for software RCM:**
  - A constraint the controller reports satisfying is only as good as the model it is expressed in.
  - Independent measurement is necessary.
  - Robot-specific calibration should be the default in ROS cobot setups.
- **It also tightens R8.4** (micro tip ±1.5 mm) and **E3 robot-frame repeatability**. One fix improves three
  results.

---

## 2. Tests still to do

### Priority 0: offline, start now
- **E0. Calibrated-kinematics prediction from existing bags** (§1.4). ~1 day of analysis.
  Settles C7 / R7.9 / threshold (iii).
- **E0c. Re-create the ⚠ scratch analyses as scripts** (§5). These are the numbers the thesis rests on: delivery
  ratio, spectra, chatter, stick-slip, buzz spectrogram.

### Priority 1: closes the core story

#### E4. Calibrated kinematics on the arm (half a session, now low-risk)
Was "risky" because `ur_calibration` didn't work. With the §1.2 fix it is about an hour:
1. Extract the calibration (`ur_calibration_extract.launch.py`; must print `Check passed`).
2. Relaunch with it and check `check_max_mm`.
3. Wrist-jog hand-eye.
4. Re-fit the micro tip ball.
5. Run RD1.

Settles C7 / R7.9 / threshold (iii) and tightens R8.4.

#### E1. Position mode on the real arm, a 3-mode paired benchmark (~1 session)
**Settles:** C3, R5.18, threshold (iii) latency, and the buzz fix (R5.17). R5.18 is ★ until this is done.

1. With the **final code**, replay `V3/LongBenchmark` and `V3/ShortBenchmark` (1:1) in **all three output
   modes in the same session**: `trajectory`, `velocity` (V5 tuned settings) and `position`. Two replays each.
   - Today's comparison mixes V3 trajectory runs (Oct), V5 velocity runs and sim-only position. A same-day,
     same-code, same-kinematics table is far more defensible.
2. Drive into the `LargeVibrations` pose region by hand in position mode. Compare the 80–249 Hz big-joint current
   against 1.81 A (velocity) and 0.22 A (`V7/NoBuzzArea`).
3. Tune `SERVO_POSITION_LEAD` (0.03 vs 0.05) by aligning `/forward_position_controller/commands` with
   `/joint_states`.
4. Start at 1:20 with slow strokes, e-stop in reach (V4 lesson). Joint limiter on.

**Output:** the final benchmark table (lag, lag-removed error, 5–15 Hz jitter, joint current 45–250 Hz, faults).
This ends Chapter 5. If position mode delivers ~58 ms with no buzz, it is the system's final configuration. If it
doesn't, the velocity-mode V5 tuned result stands and position mode becomes "investigated, not adopted".

#### E2. Independent tip-vibration comparison (same session as E1, with NDI)
**Settles:** threshold (i) measured at the *distal tip* and *independently*, as the Thesis A wording asks.

- Wand on the coupling, NDI recording. Same replay in:
  - (a) **unfiltered** velocity (`TOUCH_COMMAND_FILTER = 0`, `TOUCH_KALMAN = False`). **Only at 1:5 or 1:20 with
    the joint limiter on**: the unfiltered 1:1 runs faulted the arm. This is also RD2 (the baseline run that was
    never kept).
  - (b) V5 tuned.
  - (c) position mode.
- Metric: NDI tip PSD in the 15–25 Hz band (the Vega is 60 Hz, so the Nyquist limit is 30 Hz, which covers the
  19 Hz peak), with the stylus-target path subtracted. Use joint current for 45–250 Hz.
- Check first that the NDI sees anything: the static floor of the Vega on the wand should be well below the
  ~0.6 mm p-p vibration. Record 30 s with the arm still.

**Output:** "X % reduction in the peak band at the tip, measured independently." This is the cleanest answer to
threshold (i).

#### E3. Exchange protocol + robot-frame repeatability (~1–2 h)
**Settles:** threshold (ii) properly (R8.2 is preliminary).

- First, from memory or notes, write down what `logs/Exchange/*_QC_V1/V2/V3` were:
  - coupling version or session number;
  - whether the tool was fully removed and re-seated each swap;
  - whether the lever was cycled;
  - whether the arm was static.
- One more session: 20 swaps with a **phone video** for timing (start = lever opened, end = lever closed), NDI on
  the wand, a **known protocol written before starting**.
- **Robot-frame repeatability:** after each swap, compare the NDI tip with the tip predicted by robot kinematics
  plus the `tools.yaml` entry. This tests the whole chain, not just the coupling, and it's what matters for the
  RCM. Do it **after E4** so it uses the calibrated model.
- Optional: alternate two tools (wand ↔ micro module) with tool-library switching in the loop. That shows the
  modularity claim end to end.

**Output:** median swap time, coupling repeatability, robot-frame repeatability, scatter figure (F8.3).

### Priority 2: turns a "probably" into a result

#### E5. Micro module on real hardware (~1 session)
**Settles:** R8.7, R8.9 (currently sim/fakes only). **The macro–micro integration is in the title, so it needs at
least one real demonstration.**

- Direction and frame check: does a stylus tilt bend the tip the expected way?
- Step response: stylus step → motor angles, timed.
- A 10-DOF demo with the macro/micro switch, **on video**.
- If possible: an NDI ball on the micro tip during bending, comparing tip motion with the tendon model.

**Output:** at least a demonstration figure and step-response numbers; ideally a model-vs-measured tip plot.

#### E6. Singularity guard on the real arm (~30 min)
**Settles:** R6.3, which is mock-only.

- Drive slowly towards a wrist singularity (wrist 2 → 0) at 1:5 with the guard on. Expect: c stays ≤ ~60, the tool
  slides, no Servo halt.
- Then with the guard off for comparison, carefully: Servo will stop the arm, and that is the point.

**Output:** a real-arm condition-number trace for F6.1.

### Priority 3: nice to have
- **E7. RCM stress test with the final code (~30 min):**
  - The continuity fix and retract-past-the-point have had no dedicated real-arm test.
  - Replay RCM runs that cross 0 mm. Expect no tool0 jumps, and check the retract.
  - Fold into the RD1 session.
- **E8. Single-subject usability demo:**
  - Timed point-touch or peg tasks through a port: 1:1 vs 1:5, RCM on vs off.
  - Present it as a demonstration, not a statistic.
  - Ask your supervisor about ethics before involving anyone else.
- **E9. Haptics on a working Touch:** only if a working unit turns up. Otherwise the device characterisation
  (R6.5, R6.6) *is* the result. **Cut first.**
- **E10. Paused bag recorder:**
  - A software change that removes the start-up jumps (R5.19). Log it in VERSIONS.md.
  - Do it **before** E1/E2, so those runs don't start with a 37 mm lurch.

---

## 3. Tests to redo

The rule: redo a test when (a) it was run under conditions that have since changed and are now known to bias it,
(b) the data wasn't kept, or (c) its protocol can't be stated. Don't redo tests that are historical steps in the
story. Those are *supposed* to show the old behaviour.

| ID | Redo | Why | When | Keep the old one? |
|---|---|---|---|---|
| **RD1** | **2026-10-26 NDI RCM replays** with calibrated kinematics + new hand-eye. Minimum: 2 each of SlowChecking (worst, 2.9 mm), RCMOnly (best, 0.42), FastChecking, WholeRun | The factory model biases them by ~0.2–3.7 mm depending on pose (R7.9). The before/after pair *is* the result | E4 session | **Yes.** It's the "factory" condition of the comparison. Don't overwrite or trash it |
| **RD2** | **Unfiltered velocity baseline** (the threshold (i) baseline) | The first velocity run was analysed but not kept (R5.7 ⚠). An examiner can't check −77 % against nothing | E2, at reduced scale, limiter on | n/a |
| **RD3** | **Final benchmark of trajectory and V5 tuned velocity with the final code** | V3 and V5 numbers came from different code versions, sessions and wand seats; the final code (V8+) has never been benchmarked on the arm in trajectory or velocity mode | E1 | Yes. V3/V5 remain the history |
| **RD4** | **Exchange repeatability** with a written protocol + video | Protocol inferred from data (R8.2) | E3 | Yes, as the 60-swap coupling-only data set, if the protocol is confirmed |
| **RD5** | **Wand hand-eye + micro tip-ball calibration** | Fitted through the nominal model; must match whatever kinematics the final runs use | E4 | Keep the files (dated) for the method chapter |
| **RD6** | **Buzz comparison** in the `LargeVibrations` region | Velocity buzz vs position mode needs the same pose region on the same day | E1 step 2 | Yes |
| **RD7** | **Stick-slip evidence**, only if the 11:33 run wasn't kept (F5.6 ⚠) | The figure needs a recorded run. A 2-minute slow-stroke recording is enough | Any session, 5 min | n/a |
| RD8 (optional) | Benchmarks with **a different Touch unit** | Shows R5.16 is device-specific: with a non-sticking pitch gimbal the catches should vanish | Only if a unit is available | – |

**Don't redo:**
- V1/V2 crawl, V4 faults, V6 RCM acceleration, V7 fade swings: these *are* the story, and redoing them risks the
  arm for no gain.
- OpenHaptics lift tests and staircase: solid bench characterisation, keep as is.
- Sim/emulator studies, unless the emulator is changed.
- V3/V5 runs: keep them as historical results; RD3 adds the definitive comparison beside them.
- `V3/CyclingScaling`: the **original** is valid (UR16e model). Only its replays `CyclingScaling1–3` were invalid
  (UR5e model) and they're already excluded. The R4.2 emulator validation uses the original, so it stands.

---

## 4. Lab session plan

The order matters: **kinematics first**, so every later NDI number uses the final model, and the E1/E2 benchmark
uses the same model across all three modes.

| Session | Tests | Length | Before you go |
|---|---|---|---|
| **0 (desk)** | E0, E0c, E10 (`ur_calibration` fix: done 11-03) | 2–3 days | Run E0 so you know what RD1 should show |
| **A** | E4 → RD5 → RD1 (+E7 stress replays) | Half day | Wand in holder; NDI warmed up; `nc -zv 192.168.0.100 30001` |
| **B** | E1 + E2 (+RD2, RD3, RD6) | Full day | Commit; wand on coupling; limiter on; start at 1:20; e-stop |
| **C** | E3 (RD4) + E6 (+RD7) | Half day | Write the exchange protocol; phone for video |
| **D** | E5 micro module demo + photos/video of everything (F1.1, F4.1) | Half day | Micro hardware working on the bench first |

If time runs short: A and B are essential, C is a few hours and settles a threshold, D is needed for the title's
"macro–micro" claim (even a short video), and E8/E9 are cut.

---

## 5. Data hygiene before writing

- [ ] Every run in `02_results_inventory.md` exists in `logs/V*/`. Note which were overwritten (several V5 tests
      were) and use the recorded numbers from VERSIONS.md for those.
- [ ] Re-create the ⚠ scratch analyses as scripts in `scripts/thesis_figures/`: delivery ratio, spectra, chatter,
      stick-slip alignment, exchange repeatability, buzz spectrogram, and E0. Commit them so every number in the
      thesis is reproducible.
- [ ] Re-run `thesis/run_tools/plot_paths.py` on each version folder so METRICS are current.
- [ ] Back up `logs/` off the lab PC (it is git-ignored): external drive + cloud.
- [ ] Photos and videos of the hardware, coupling swaps and the 10-DOF system.
- [ ] Export Onshape renders and drawings of the coupling versions.
- [x] Record the kinematics condition (factory / calibrated + file checksum) in every new run's META, so the two
      conditions can never be mixed up later. Done 11-03: `META.yaml kinematics:` (model, file, sha256, hash).
      Older runs have no block, and all of them are factory.
