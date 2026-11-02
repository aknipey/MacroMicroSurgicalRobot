# 2. Results inventory

Every result worth writing about, with its numbers, where the data is, how strong the evidence is, and which
chapter it goes in. The source of truth is `docs/VERSIONS.md` (tracked) plus the run folders in `logs/`
(lab PC only, git-ignored). **Re-check every number against the run's `META.yaml` / `METRICS.yaml` before it goes
in the thesis.** Some came from analysis scripts that only ever lived in a scratchpad and have to be re-created
(flagged ⚠ script).

**Evidence tags:**
- `[REAL]` real UR16e
- `[REAL-REPLAY]` real arm, identical stylus input replayed
- `[NDI]` independent optical measurement
- `[SIM]` mock UR16e + `ur_emulator`
- `[OFFLINE]` recorded data or a kinematic model processed offline
- `[MOCK]` full stack on mock hardware
- `[BENCH]` device on the bench

**Strength:** ★★★ solid, repeated or replayed · ★★ single real run or validated sim · ★ indicative only, or not
yet on hardware

---

## Chapter 4 — Methods and framework results

| ID | Result | Numbers | Evidence | Source | Strength |
|---|---|---|---|---|---|
| R4.1 | Stylus replay benchmarks are repeatable | Two replay sets of the same runs matched within 1 %: lag 62/116 vs 62/114 ms, 5–15 Hz jitter 0.252/0.322 vs 0.253/0.323 mm, angle error 11.92/15.12° vs 11.95/15.13° | [REAL-REPLAY] | VERSIONS.md V5 "Repeatability check" (runs overwritten; numbers recorded) | ★★★ |
| R4.2 | Emulator reproduces the real arm | `V3/CyclingScaling`: sim 144 ms / 12.2 mm vs real 140 ms / 11.9 mm. Arm model: 16 ms dead time + 15 ms first-order lag, fit 0.1–0.4 mrad RMS | [SIM]+[REAL] | VERSIONS.md V4; `logs/SIM_latency/` | ★★★ |
| R4.3 | Sim predicts tuned real benchmarks | Predicted angle error lag-removed 11.2/9.6° vs real 11.2/9.2°; 5–15 Hz jitter 0.198/0.289 vs 0.197/0.278 mm; arm 12 ms better on lag than predicted | [SIM]+[REAL-REPLAY] | VERSIONS.md V5 "Tuned runs" | ★★★ |
| R4.4 | Recorder detects wrong kinematics | TCP-vs-TF drift check (`tcp_offset.check_max_mm`, > 5 mm = wrong model). V3 CyclingScaling1–3 ran on UR5e kinematics (31 mm error, invalid runs). V3–V8 runs show 0.6–3.5 mm (consistent with UR16e factory model) | [REAL] | META.yaml of every run; VERSIONS.md V3 "Invalid runs" | ★★★ |
| R4.5 | Static jitter floor | Robot jitter with no motion 0.016 mm (`V3/NoMotion`) | [REAL] | `logs/V3/NoMotion` | ★★ |
| R4.6 | Wand hand-eye calibration quality | 2026-10-22: tip robot-vs-camera 1.05 mm rms (max 2.2) over 0.5 m; 2026-10-26: 0.82 mm rms / 1.95 max, rotation mismatch 0.42°, camera-pose spread 8.1 mm / 0.34°; wand pivot calibration 0.93 mm rms | [NDI] | VERSIONS.md "Tool calibration for the NDI wand", "RCM tool recalibrated"; `src/ndi_tracking/calibrations/` | ★★★ |
| R4.7 | Orphaned drivers corrupt state | 4 orphaned `ros2_control_node`s → 5 `/joint_states` publishers; 8 jumps > 0.05 rad (up to 3.56 rad) in 8.3 s, stamps 722–5563 s old. Fixed by process-group launcher + orphan detection | [REAL] | VERSIONS.md "Orphaned drivers" | ★★ (good "lesson learned", short) |

---

## Chapter 5 — Latency and smoothness (the core)

### 5a. From crawl to trajectory mode

| ID | Result | Numbers | Evidence | Source | Strength |
|---|---|---|---|---|---|
| R5.1 | V1 position output crawls | Lag 640–760 ms translation, > 1.5 s rotation; 0.32 m/s commanded → 0.04 m/s actual (~10 %). Cause: Servo re-bases each step on measured joints; arm takes ~40 ms per step. Mock hides it | [REAL] | `logs/V1/*`; VERSIONS.md V1 | ★★★ |
| R5.2 | Trajectory output fixes the crawl | V2 angle lag 128–158 ms (302 ms twist) vs > 1.5 s; RMS angle error 27–36° → 6–24°; 10 cm moves 106–128 ms | [REAL] | `logs/V2/*` | ★★★ |
| R5.3 | Motion scaling | 1:1 / 1:5 / 1:20 by double click, ~140 ms lag at every scale; at small scales position *looks* lag-free (gap < 1 mm) | [REAL] | `logs/V3/Scaled*`, `CyclingScaling` | ★★ |
| R5.4 | **Displacement delivery ratio 0.60** | Trajectory output: arm covers 60 % of commanded twist displacement, real and sim alike, independent of lookahead; velocity output 1.04. Trajectory tweaks floor at ~114 ms (lookahead 0.013 s = Servo's minimum) | [REAL]+[SIM] | VERSIONS.md V4 "Why" + sim table | ★★★ **key diagnosis** |
| R5.5 | Sim latency study table | V3 settings 144/144 ms, 12.2 mm → velocity + ff filter 0.01 + 250 Hz + joint_states + loop delay 0.03: **30/34 ms, 2.5 mm**; pessimistic arm 38/46 ms | [SIM] | VERSIONS.md V4 sim table; `logs/SIM_latency/` | ★★★ (sim) |
| R5.6 | Fresh pose made trajectory mode *worse* | joint_states FK: 114 → 130 ms; stale TF had been pushing the P-loop harder, hiding the 60 % shortfall | [SIM] | VERSIONS.md V4 | ★★ (nice subtle point) |

### 5b. Velocity mode: why it vibrated and faulted

| ID | Result | Numbers | Evidence | Source | Strength |
|---|---|---|---|---|---|
| R5.7 | Velocity mode vibrates | Tool vibration ~5× (15–25 Hz, peak ~19 Hz) and ~7× (30–60 Hz) trajectory mode, ~0.6 mm p-p; speed scaling > 92 % (arm follows command) | [REAL] | first velocity run (analysed, not kept); VERSIONS.md V4 "Why velocity mode vibrated" | ★★ ⚠ run not kept, numbers recorded |
| R5.8 | Causes of commanded vibration | (a) Servo computes velocities before smoothing, never re-smooths (MoveIt Servo 2.12.4 `servo.cpp getNextJointState`); (b) Touch pose updates every ~4 ms (sometimes 12) at 1 kHz publishing → up to ±33 % noise on 10 ms differences; (c) tremor 0.025° RMS at 7–13 Hz × feedforward 2πf (×60 at 10 Hz, ×125 at 20 Hz) = the command's content; (d) 20 Hz TF staircase → 19.5/39 Hz | [REAL]+[OFFLINE] | VERSIONS.md V4 + V5 real-arm tests | ★★★ |
| R5.9 | **Joint faults** | 3 benchmark replays faulted (C306A0 / C281A3 / C740A0) 4–5 s in; wrist 3.12/3.17 rad/s vs 3.14 limit in two; later recorded fault at only 1.31 rad/s (41 % of limit). **In all 3 recorded faults the faulting joint had its highest current chatter of the run** (top 0.1–1.5 %). Torque chatter 45–250 Hz: trajectory 0.01–0.05 Nm vs velocity 0.34–0.98 Nm, at the update-rate frequencies | [REAL] | `logs/V4/*_failed`; VERSIONS.md V4/V5 fault sections | ★★★ |

### 5c. The fixes and their effect

| ID | Result | Numbers | Evidence | Source | Strength |
|---|---|---|---|---|---|
| R5.10 | Kalman vs One Euro on stylus | CV Kalman: linear noise −23 to −42 %, angular −17 to −37 %, **no added lag** (lag slightly reduced); One Euro settings that cut noise added 30–50 ms ff lag, 2–6 mm path lag. Overshoot trade-off: 0.13 mm on 50 ms stop from 200 mm/s (vs 1.2 mm at smoother setting) | [OFFLINE] | VERSIONS.md V5 Kalman table | ★★★ |
| R5.11 | Kalman alone not enough | Real arm: command 8–40 Hz content unchanged (wz 0.023 vs 0.024 rad/s): the noise is tremor, which looks like real motion to a CV model | [REAL] | VERSIONS.md "velocity mode + Kalman filter, command filter off" | ★★ |
| R5.12 | Command low-pass trade-off | 10 Hz: 23 %/2 % left (15–25/30–50 Hz), 22 ms; **8 Hz: 14 %/1 %, 28 ms**; 6 Hz: 8 %/1 %, 37 ms | [OFFLINE] | VERSIONS.md V5 command-filter table | ★★★ |
| R5.13 | Vibration reduction on the arm | 20 Hz buzz: tool rotation 15–25 Hz 0.0155 → 0.0036 rad/s (**−77 %**), 19.5/39 Hz peaks gone; tool vibration 8–60 Hz 0.03–0.08 → 0.03–0.04 mm; lag 56/60 ms, error lag-removed 0.72 mm/0.35° | [REAL] | VERSIONS.md "+ joint_states pose + command filter 8 Hz" | ★★ (supports threshold (i)) |
| R5.14 | 500 Hz chain + limiter: no faults | First run with it: no faults, chain held 500 Hz, lag 58/72 ms. Sim: wrist 2.82 → 2.51 rad/s (cap), set-point accel 109 → 25 rad/s², 45–250 Hz content 4–5× lower. Tool-speed caps proven unable to bound joint speeds (Servo asked 2.83 rad/s with tool cap 2.0) | [REAL]+[SIM] | VERSIONS.md "Recorded joint fault ... the fix" | ★★★ |
| R5.15 | **Benchmark: velocity (V5) vs trajectory (V3)** | Lag translation 160/150 → **62/58 ms** (tuned 74/66); angles 224/242 → 114/80 (tuned 118/98); RMS pos error 22.4/36.9 → 9.7/15.3 mm; lag-removed 5.9/7.1 → 2.6/3.7 mm; wrist chatter 0.03/0.04 → 0.04/0.08 (tuned 0.04/0.03, i.e. **same as trajectory**); jitter 5–15 Hz 0.131/0.179 → 0.197/0.278 mm (tuned; ~1.5× trajectory); no faults (same replays faulted 3× in V4) | [REAL-REPLAY] | `logs/V5/LongBenchmark(Tuned)`, `ShortBenchmark(Tuned)` vs `logs/V3/LongBenchmark1/2`, `ShortBenchmark2/3` | ★★★ **headline** |
| R5.15b | Jerk / smoothing variants (sim) | Correction caps + 6 Hz filter: angular jerk p95 513 → 343 rad/s³ for +2 ms lag; 5 Hz halves it for +5–6 ms; longer loop delay tracks better but jerkier | [SIM] | VERSIONS.md V5 "Sim comparison" | ★★ |
| R5.15c | Best hand run | "Very smooth": lag 54/56 ms, error lag-removed 0.57 mm / 0.31°, robot jitter 0.04 mm / 0.06° < target 0.06 mm / 0.13° | [REAL] | VERSIONS.md "smoothing applied" | ★★ |

### 5d. Residual issues: device, buzz, position mode

| ID | Result | Numbers | Evidence | Source | Strength |
|---|---|---|---|---|---|
| R5.16 | **"Catches" are Touch pitch-gimbal stick-slip** | At robot dips to 7–20 % speed, pitch joint speed → 0.0 then bursts 2–3× (e.g. 16.6 → 0.0 → 46.5 °/s); stalls only at 8–24 °/s, never at 47 °/s; yaw/roll never stall; encoder continuous (not skipping). Robot follows within ~55 ms, so it reproduces the device faithfully; removing would need a 2–3 Hz filter (+70 ms) | [REAL]+[BENCH] | VERSIONS.md "the catches are stick-slip"; `/phantom/joint_states` in bag | ★★★ (great "it wasn't us" result) |
| R5.17 | **Buzz is speedj-only and pose-dependent** | `LargeVibrations` 1.81 A vs `NEW_RUN` 0.22 A (80–249 Hz big-joint current), same session/settings; NEW_RUN *harder* accel (6.2 vs 4.6 rad/s²) and rougher command; servoj never buzzed (0.11 A); not tool, not gravity load. Earlier "follows acceleration" correlations only held *within* a buzzing pose. Ramped speedj script did not fix it | [REAL] | `logs/V7/LargeVibrations`, NEW_RUN (16:43, copy it into a version folder!); VERSIONS.md "Buzz is speedj-only" | ★★ (mechanism unproven, aliasing above 250 Hz) |
| R5.18 | Position (servoj) streaming without the crawl | Emulator `V3/CyclingScaling`: position lead 0.03 → **58/62 ms, 4.9 mm, 1.36 mm lag-removed** vs velocity 62/66 ms, 5.3 mm, 2.06 mm; real-time integration cut peak joint accel 2980 → 830 rad/s² | [SIM] | VERSIONS.md same section | ★ until run on the real arm |
| R5.19 | Bag recorder start-up causes jumps | Watchdog zeroed command when `ros2 bag record` started (3 of 5 recording starts), 37 mm catch-up lurch | [REAL] | `logs/V7/WeirdJumpsRCM` | ★★ (limitation / lesson) |

---

## Chapter 6 — Safety behaviours

| ID | Result | Numbers | Evidence | Source | Strength |
|---|---|---|---|---|---|
| R6.1 | Servo's singularity handling fails teleop | Scales whole twist; above hard stop halts *every* command incl. escaping ones: stuck at c = 554 regardless of input; at c = 100 joints move ~50× tool speed | [MOCK] | VERSIONS.md V6 part 1 | ★★★ |
| R6.2 | Guard in kinematic sim | Unguarded outward drive: c 29 578, 1050 rad/s; guarded: held at c = 60, ≤ 1 rad/s, sliding; from c = 821 backs out at 0.2 rad/s; 40 random strokes c ≤ 67, per-cycle twist change ≤ 0.026 (vs 1.36 using σ_min gradient) | [OFFLINE] | same | ★★★ |
| R6.3 | Guard in full mock stack | c 8.5 → 36 → 60.3 held, Servo "No warnings"; from Servo e-stop at c = 554 backs out to 60 in ~1 s; cost 0.12–0.2 ms/cycle | [MOCK] | same | ★★ (real-arm singularity test would strengthen) |
| R6.4 | Haptic cue design + singularity push direction | Push direction lowered c in 150/150 random near-singular poses and beat 97–99 % of random directions; mock: wall/detent/buzz shapes as designed, 1.5 N cap, slew ≤ 0.4 N/step, zero force on Ctrl-C | [OFFLINE]+[MOCK] | VERSIONS.md V7 part 2 | ★★ |
| R6.5 | **OpenHaptics renders no force in ROS 2 processes** | Lift test (1 N up): plain program 148–165 mm; same calls in omni_state 0 mm; linked (never called) to librclcpp 0 mm; liblttng-ust / libtracetools / libnuma 0 mm; LD_PRELOAD libnuma alone 0 mm; rcutils/rmw/spdlog/yaml/fmt 155–160 mm. Fixed by `touch_server` (HD only) + `touch_bridge` (ROS only) over UDP | [BENCH] | VERSIONS.md "No force on the real Touch" | ★★★ **clean result** |
| R6.6 | Touch force output faulty (SN 22129000559) | Staircase 0.001–0.01 N: identical 10 mm-in-~21 ms kick, at 0.03 N reverses; threshold ~0.0008 N; spring walls always chatter at their edge → replaced by limit clicks | [BENCH] | VERSIONS.md "Run 16:49" + staircase | ★★★ (as characterisation) |
| R6.7 | Haptic cues iterated with real Touch | Shake 10–20 mm → ~2 mm at tested-range strengths; click self-retriggering fixed by 0.3 s hold-off | [REAL] | same | ★ (device faulty) |
| R6.8 | Other safeguards | Velocity watchdog (stopped runaway after 0.64°), joint limiter caps proven (unit test 2.513 rad/s, 25.0 rad/s² exact), RCM max-error hold (5 mm), RCM exit interlock (≤ 3 mm depth), tool-switch interlocks (12/12 checks), orphan-driver guard | [SIM]/[MOCK] | VERSIONS.md various | ★★ |

---

## Chapter 7 — Software RCM

| ID | Result | Numbers | Evidence | Source | Strength |
|---|---|---|---|---|---|
| R7.1 | Twist built about the RCM point | Offline 12 s fast pivots (169°/s peak): filtering the finished twist → 5.8 mm off point; filter on (insertion, ω) then lever → 2.6/4.2 mm at 30/50 ms delay; + pose prediction → **0.45 mm** at matched delay (1.3 mm at 30 ms, 2.8 at 80 ms). Caps must scale linear+angular together | [OFFLINE] | VERSIONS.md V6 part 2 | ★★★ |
| R7.2 | Mock RCM accuracy | Insert 80 mm: 0.019/0.013 mm; 0.5 Hz circles ±30° twist, 21° tilt: **1.35 max / 0.39 mean mm**; exit refused when deep, accepted at 0.1 mm | [MOCK] | same | ★★ |
| R7.3 | V6 real arm problem | Near the point the arm moved r(d)/d × tip (~30× at 10 mm, 9× at 30, 1.4× at 130): huge accelerations | [REAL] | `logs/V6/BasicInRCM`, `ScalingInRCM`, `ExitingRCM` | ★★ |
| R7.4 | **Depth-invariant sideways law** | tan θ = k·\|s\|/r(d), k = 0.5: tool0 0.50 mm per mm of stylus from 0–130 mm (exact offline); pivot per mm 0.09–0.16° over whole range vs 5.7°/mm peak for V6 fade; inserting with 5 mm offset keeps tool0 5.00 mm off (no swing) | [OFFLINE] | VERSIONS.md V7 part 1 tables | ★★★ |
| R7.5 | Alternatives that failed | V6 fade: 124 m/s² kink; smootherstep: 4.4 → 1.0 m/s² but still large; fade³: in/out swing 4–6× stylus speed at 15–25 mm (real arm 13:39); fade vs law offline 100 mm/s vs 24/12 mm/s tool0, peak accel 1.34 → 0.34 m/s² | [OFFLINE]+[REAL] | same "How it was chosen" | ★★★ (shows design process) |
| R7.6 | Continuity through 0 mm | Branch at d ≤ 0 caused 1–5 m/s jumps (real arm 13:46); fix: largest tool0 step 11 mm → 0.03 mm offline | [REAL]+[OFFLINE] | same | ★★ |
| R7.7 | **NDI replay set (14 replays)** | RMS shaft-to-point error: FastChecking 1.58–1.77 mm (p95 3.0–3.3, 77–83 % within 2 mm, 47° swing); SlowChecking 2.80–2.92 (20–22 %, 16°); RCMOnly **0.42–0.44** (100 %, 40°); WholeRun **0.89–0.94** (96–97 %, 70°). Within-set spread 0.1–0.2 mm | [NDI]+[REAL-REPLAY] | `logs/V7/*_Replay*`; `logs/V7/REPLAYS_2026-10-26.md` | ★★★ **headline** |
| R7.8 | Controller holds its own point | Teleop `rcm_error` (TF) 0.05–0.23 mm rms | [REAL] | same | ★★★ |
| R7.9 | **Gap = factory kinematics** | Latch timing ruled out (at rest teleop/NDI points agree 0.2 mm); real pivot off model pivot by 0.17/0.34/1.33/3.7 mm mostly base z (pose-dependent); TF vs controller tool0 0.1–0.25°, = ~1.6 mm at 367 mm tip; same effect in micro tip calibration (1.5–1.9 mm residual at 436 mm, not speed-dependent, gravity sag explains 16 %) | [NDI]+[OFFLINE] | `scripts/rcm_offset_check.py`; REPLAYS md; VERSIONS.md tip-ball section | ★★ (strong circumstantial; calibrated-kinematics test would make it ★★★) |
| R7.10 | Originals vs replays | Originals (old seat): Fast 1.92 mm, WholeRun 1.24 mm | [NDI] | REPLAYS md | ★★ |

---

## Chapter 8 — Modularity and macro–micro

| ID | Result | Numbers | Evidence | Source | Strength |
|---|---|---|---|---|---|
| R8.1 | Quick-change coupling design | Maxwell-type kinematic coupling (3 V-groove seats at 0/105/255°) + rotary face-cam lever (32° swing), T-bar on spring (14 N/mm → ~107 N preload, ~1.2 N·m tipping); ISO 9409-1-50-4-M6 tool face; iterations printed → steel balls/pins → V2 6 mm rods + epoxy-replicated balls + 3-lug cam → V3 99 mm ball PCD + lightened plate | [CAD] | Onshape docs 9fc7577e…, e933569c…; memory notes | ★★ |
| R8.2 | **Exchange repeatability (preliminary)** | 3 sessions × 20 swaps (inferred from 21 still segments each): tip re-seat spread rms **0.06 / 0.02 / 0.05 mm**, max **0.19 / 0.05 / 0.12 mm**; axis ≤ 0.03°; swap gaps 3.3–9.3 s (median ~4.8 s). First swaps of a session slightly larger (bedding-in?) | [NDI] | `logs/Exchange/2026102*_QC_V1/V2/V3/NDI_PATH.csv`; `planning/analysis/exchange_repeatability.py` | ★★ — **confirm protocol** (was the tool fully removed each time? what do V1/V2/V3 mean: coupling version or session? arm static?). Numbers near the Vega's noise floor; report as "≤ 0.2 mm, at the tracker's resolution" |
| R8.3 | Tool calibration without a divot | Wrist-jog hand-eye + wand's own tip calibration: R4.6 numbers | [NDI] | VERSIONS.md | ★★★ |
| R8.4 | Single-ball tip calibration | Synthetic: 0.04 mm error, delay recovered 30 ms, outliers rejected; real micro module: ball (−9.15, 15.93, 436.25) mm, 2363 frames, 136° swing, residual 1.53–1.88 mm (pose-dependent → kinematics), halves 1.6 mm apart → ±1.5 mm | [NDI]+[OFFLINE] | `config/calibrations/micro_tip_2026-10-30_144004.json`; VERSIONS.md | ★★ |
| R8.5 | Tool library + interlocks | Refused in RCM, with WHITE, while moving, < 0.5 s still, unknown id; logged in META; 12/12 checks | [MOCK] | VERSIONS.md "Tool library" | ★★ (real tool switching was used on the setup) |
| R8.6 | Micro module faults found | QoS deadlock (volatile vs transient-local), handshake lost to bootloader reset, rad added to degrees (motors moved ~1/1000), truncation, joint limits ±0.05 rad ≈ ±1° motor | [MOCK]+[OFFLINE] | VERSIONS.md micro sections; Serial_Control repo | ★★ |
| R8.7 | Micro tip control (sim/fakes) | Tip within ~0.1 mm of 2–3 mm target in ±40 mm stylus circle; 30 mm step settles 69 ms (was 1.1 s); 20° tilt step 0.36 s; motors sweep ±5° vs ≤ 1° before | [SIM] | same | ★ until real hardware |
| R8.8 | One-stylus macro/micro switch | GREY 1 s hold switches; click < 1 s records; WHITE deadman for both; 37 micro commands in 0.6 s in micro mode, 0 in macro | [MOCK] | VERSIONS.md "Macro / micro switch" | ★★ |
| R8.9 | Integrated 10-DOF system | Micro module = tool 2 at (−8.7, 17.0, 436.0) mm ±1.5 mm, tool switching on real setup | [REAL] | same | ★ (needs a demonstration run with video) |

---

## Baseline to compare against (Lachlan Scott 2023)

- UR5e, ROS 1, `pose_based_cartesian_traj_controller`; vibration in translational motion was observed but **not
  quantified**; trajectory similarity shown qualitatively; micro module EM error 0.027 / 0.016 mm (average).
- **Your quantitative baseline is therefore your own V1/V3** on the UR16e, not Lachlan's numbers. State this
  clearly: different arm (UR16e, heavier joints), different middleware (ROS 2 Jazzy, MoveIt Servo).
- You *can* compare qualitatively: Lachlan listed vibration, tip exchange and haptics as future work, and this
  thesis addresses all three.

---

## Results NOT to lead with (keep for appendix or limitations)

- V1 rebuilt-from-transcripts provenance: interesting, but put it in Appendix A as a reproducibility note.
- Stray-character build break, rename commit, launcher internals: leave out.
- V7 haptic strength iterations in detail: summarise, since the device was faulty.
- `logs/V2` mixed-version runs: footnote only.
