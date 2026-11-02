# 3. Chapter-by-chapter plan

The LaTeX skeleton (`../latex/chapters/`) uses these exact section headings. For each chapter this file gives
the **job** (what the reader must come away believing), the **content** of each section with result IDs from
`02_results_inventory.md`, the figures and tables, and the **hand-off sentence** into the next chapter, which is
what makes the thesis read as one story.

**Page budget** (main text ≈ 75–90 pages; **check the Thesis C limit in the course outline** and scale):

| Ch | Title | Pages |
|---|---|---|
| 1 | Introduction | 4–5 |
| 2 | Literature Review | 12–15 |
| 3 | Research Questions & Approach | 3–4 |
| 4 | System & Methods | 8–10 |
| 5 | Latency & Smoothness | 12–15 |
| 6 | Safety Behaviours | 6–8 |
| 7 | Software RCM | 10–12 |
| 8 | Modularity & Macro–Micro | 8–10 |
| 9 | Discussion | 6–8 |
| 10 | Conclusion | 2–3 |

Writing order: **4 → 5 → 7 → 8 → 6 → 3 → 9 → 2 (revise) → 1 → 10 → abstract.** Write methods and results while
the data is fresh, then frame them.

---

## Front matter

- **Statement on the use of generative AI:** UNSW requires this. Be specific and honest. Claude Code was used for
  software development, log analysis, Onshape FeatureScript and documentation. All experiments were run by you,
  design decisions and conclusions are yours, and outputs were checked by real-arm tests, replays and independent
  NDI tracking. Check the course outline for the exact required form.
- **Abstract:** write last. One sentence each: context, problem, approach, 4–5 numbers, significance.

---

## Chapter 1 — Introduction  {#chapter-1}

**Job:** the reader understands why low-cost macro–micro MIS robots matter, what Lachlan's prototype could and
couldn't do, and what this thesis contributes, *before* the literature review.

- **1.1 Clinical motivation:** MIS benefits, the cost and access barrier of closed systems (reuse Thesis A
  citations: eckhoff2023costs, raza2024transforming, dagnino2024trends).
- **1.2 Macro–micro systems:** gross positioning + local dexterity, with a figure of your physical setup (UR16e +
  coupling + micro module + Touch + Vega). **Take good photos now.**
- **1.3 Starting point:** scott2023macro. His conclusion lists the vibration, the time-consuming tip exchange and
  haptics as future work. Note the move to the UR16e, ROS 2 Jazzy and MoveIt Servo, and that his ROS 1 Cartesian
  controller does not exist in ROS 2. That is why the teleop stack was rebuilt.
- **1.4 Aims and scope:** three aims (smooth low-latency teleop; safe constrained motion; modular tooling) and one
  enabling aim (repeatable, independent evaluation).
- **1.5 Contributions:** the top five from `01_story_and_novelty.md` §1.3, each with "(Chapter N)".
- **1.6 Outline:** one sentence per chapter.

**Figures:** F1.1 system photo (annotated); F1.2 macro–micro concept sketch (optional).
**Hand-off:** "Chapter 2 reviews the literature behind each aim and identifies the gaps these contributions address."

---

## Chapter 2 — Literature Review  {#chapter-2}

**Job:** show you know the field and set up exactly the gaps your results fill. **Rework your Thesis A review;
don't start from scratch.** It covered MIS/robots, teleoperation/control, modularity/continuum, safety, synthesis.
What's new: the results now demand literature on specific topics your Thesis A review did not cover (see
`06_literature_to_find.md`).

Restructure so that **every section ends with a one-paragraph "gap" that names the chapter answering it.**

- **2.1 MIS and surgical robotics:** commercial systems and their cost; open and low-cost platforms (SnakeRaven,
  ROS-based systems; torielli2023ros is in your bib).
- **2.2 Macro–micro architectures:** continuum tips (burgnerkahrs2015continuum, dupont2022continuum,
  razjigaev2021snakeraven) and industrial arms or cobots as the macro stage. *Gap: the macro stage's dynamics
  and interfaces are rarely characterised.*
- **2.3 Teleoperation of the macro stage:** this section grows the most.
  - Latency, transparency and operator performance: what lag is acceptable (find the classic numbers).
  - Servo-level interfaces: velocity vs position streaming, MoveIt Servo, UR External Control (`speedj`/`servoj`
    are documented in UR's script manual).
  - Tremor and filtering: the physiological tremor band (8–12 Hz), low-pass vs adaptive vs Kalman, One Euro
    (Casiez 2012).
  - Motion scaling and clutching.
  - *Gap: off-the-shelf streaming interfaces are treated as transparent, and their effect on lag and vibration is
    not quantified → Ch 5.*
- **2.4 Constrained motion and safety:**
  - Singularity handling: DLS (Nakamura & Hanafusa; Wampler), manipulability (Yoshikawa), and how MoveIt Servo
    handles it.
  - RCM: mechanical vs software (Taylor's LARS, Aghakhani et al., sadeghian2019constrained in your bib), and
    motion scaling with the fulcrum effect.
  - Haptics: virtual fixtures (Rosenberg; Abbott), device limits.
  - *Gaps: singularity handling that preserves useful motion; RCM scaling near the port; haptic deployment on
    open stacks → Ch 6, 7.*
- **2.5 Accuracy and validation:**
  - Kinematic calibration of industrial arms (UR's factory calibration, DH errors).
  - Independent tracking (optical vs EM; your Vega vs Lachlan's EM).
  - *Gap: software RCM accuracy is usually reported from the controller's own model → Ch 7.*
- **2.6 Modularity and tool exchange:** tool changers and kinematic couplings (Maxwell/Kelvin, Slocum's design
  rules), ko2025tool (15 s automated exchange) and hong2025backlash from your bib, and post-swap recalibration.
  *Gap → Ch 8.*
- **2.7 Synthesis:** the four gaps G1–G4 (`01_story_and_novelty.md` §1.4) in a table: gap → evidence of the gap →
  chapter.

**Figures:** F2.1 taxonomy of RCM approaches (optional); T2.1 gap table.
**Hand-off:** "These gaps motivate the research question and thresholds in Chapter 3."

---

## Chapter 3 — Research Questions, Hypotheses and Approach  {#chapter-3}

**Job:** restate the Thesis A question and thresholds, and frame how the work answers them. Be upfront that the
thresholds were set before the platform was characterised.

- **3.1 Research question:** the Thesis A wording (see `04_research_question_scorecard.md`). Replace "UR5e" with
  UR16e and note why.
- **3.2 Hypotheses and thresholds:** (i) ≥ 30 % peak-band vibration reduction vs unfiltered baseline; (ii) median
  exchange ≤ 20 s with ≤ 1.0 mm post-swap repeatability; (iii) RCM error ≤ 1.0 mm without latency > 50 ms. Define
  precisely what each will be measured with, since Ch 4 defines the metrics.
  - **Decision for you:** keep the thresholds as written and report against them honestly (recommended: it's
    rigorous, and the misses are explained), or revise (iii) with justification. Examiners usually respect
    "the threshold was not met, and here is the quantified reason" far more than a moved goalpost.
- **3.3 Sub-questions:**
  - RQ1: What limits lag and smoothness on an open MoveIt Servo → UR stack, and how far can software reduce it?
  - RQ2: Can singularities and constraint violations be handled without stalling the operator?
  - RQ3: How accurately can a software RCM be held, measured independently?
  - RQ4: Can tools be exchanged quickly and repeatably, and the system recover calibration?
- **3.4 Approach:** versioned iteration (V1–V8). Measure, diagnose, change one thing, then re-measure with
  identical input. A small diagram of this loop works well here.
- **3.5 Scope:** single operator, lab bench, no tissue, no clinical claims; the Touch unit's force fault limits
  haptics; the micro module is secondary.

**Figures:** F3.1 the iterative loop / version timeline (V1–V8 on a time axis with the main change and the lag).

---

## Chapter 4 — System Overview and Experimental Methods  {#chapter-4}

**Job:** everything the result chapters rely on is defined *once* here, especially the metrics.

- **4.1 Hardware:**
  - UR16e: SN 20225600118, PolyScope 5.12.1, wrist limit 3.14 rad/s, factory kinematics, 500 Hz controller.
  - Micro module: Lachlan's design, 4 servos + tendons, Mega 2560 at 9600 baud.
  - Touch: two units (SN 22129000559 HID, used for most runs; SN 19039000821 with a stuck shoulder encoder),
    1 kHz servo loop with pose updates every ~4 ms, 3.3 N peak / 0.88 N continuous force.
  - NDI Vega: 60 Hz, probe 8700340, wand pivot calibration 0.93 mm rms.
- **4.2 Software architecture:** the signal-chain figure (stylus → teleop node → MoveIt Servo → joint limiter →
  controller → UR), and per-node rates (1 kHz / 500 Hz / 500 Hz / 500 Hz).
  - **4.2.1 Control law:** clutch, target = p₀ + s·A·Δp, swing-twist rotation, v = k_ff v_target + K_p (p_target
    − p_robot), speed and correction caps, motion scale steps. (Equations go here; Ch 5 refers back.)
  - **4.2.2 Output modes:** trajectory, velocity (speedj), position (servoj with integrated velocities). A table of
    controller, UR command, Servo rate, and which versions used each.
- **4.3 Experimental methods:**
  - Recording: GREY button; TOUCH_PATH, STYLUS_RAW, UR_PATH (RTDE TCP), rosbag, META with every setting + code
    version. (R4.4)
  - Stylus replay: original timing, start-pose approach, motion scale and RCM chord replayed; repeatability < 1 %
    (R4.1).
  - Emulator: dead time + lag model and its validation (R4.2).
  - Calibration: wand hand-eye without a divot; tip ball (R4.6, R8.3–R8.4).
  - Benchmarks: LongBenchmark / ShortBenchmark / CyclingScaling / SimpleHolder RCM set; describe the motions.
- **4.4 Metrics:** define each one mathematically.
  - Lag: the time shift that best aligns the robot tool0 path with the target path (plot_paths.py).
  - Tracking error: at the same time, and with the lag removed.
  - Jitter: RMS above 5 Hz, and per band 5–15 / 15–40 / 40–120 Hz.
  - Tool angular jerk p95, smoothed at 30 Hz.
  - Joint current in the 45–250 Hz / 80–249 Hz bands (from `/joint_states` effort).
  - RCM error: distance from the shaft line to the latched point, from the controller (TF) and from the NDI.
  - Exchange time and repeatability (define the "still segment" detection).
- **4.5 Safety procedures:** e-stop, starting at 1:20, private ROS domains for tests, mock-first.

**Figures:** F4.1 hardware photo with frames (base, tool0, camera, stylus); F4.2 software architecture (from
`docs/system_architecture.drawio`); F4.3 replay pipeline; T4.1 output modes; T4.2 metric definitions.

---

## Chapter 5 — Low-Latency, Smooth Teleoperation of the Macro Stage  {#chapter-5}

**Job:** the strongest chapter. The reader follows a chain of measured diagnoses and believes each fix because
identical-input replays show it.

- **5.1 Baseline (V1):** the crawl and the re-basing cause (R5.1). Include the 0.32 → 0.04 m/s fact. Mock
  hardware hides it, which is a lesson about sim-only validation.
- **5.2 Trajectory output and scaling (V2–V3):** R5.2, R5.3.
- **5.3 Displacement delivery ratio:** R5.4, R5.5, R5.6.
  - Figure: commanded vs delivered displacement (ratio 0.60 vs 1.04).
  - Table: the sim latency study.
  - This is where C1 lands.
- **5.4 Velocity output:** R5.7–R5.9.
  - 5.4.1: tremor × feedforward, with a spectrum showing stylus tremor at 7–13 Hz and the command content, plus
    the 2πf argument.
  - 5.4.2: update-rate staircases, with the torque-chatter table (trajectory 0.01–0.05 Nm vs velocity 0.34–0.98).
  - 5.4.3: faults, with the table of chatter before each fault.
- **5.5 Filtering and rate design:** R5.10–R5.14.
  - 5.5.1: Kalman vs One Euro.
  - 5.5.2: the low-pass trade-off table and group-delay formula (feeds C4).
  - 5.5.3: 500 Hz chain + limiter, showing that tool caps cannot bound joint speed.
- **5.6 Benchmark comparison:** R5.15, the headline table (trajectory vs velocity vs tuned), plus a Pareto plot
  of lag vs jitter or jerk for every configuration (real + sim). **This plot could be the thesis's signature
  figure.**
- **5.7 Residual hesitations = Touch stick-slip:** R5.16. Figure: robot speed dips aligned with pitch-joint
  stalls.
- **5.8 Buzz and position mode:** R5.17, R5.18. Present it honestly: the mechanism is not proven, servoj never
  buzzed, and position mode is the fix (real-arm result pending, or included if you run it).
- **5.9 Summary:** one table, V1 → final: lag, error, vibration, faults.

**Figures:**
- F5.1 lag by version (bar chart, real vs sim).
- F5.2 delivery ratio.
- F5.3 spectra (stylus / command / tool) before and after.
- F5.4 torque-chatter spectra at 100/250/500 Hz rates.
- F5.5 Pareto lag vs smoothness.
- F5.6 stick-slip alignment.
- F5.7 buzz vs pose.
- T5.1 sim latency study.
- T5.2 fault table.
- T5.3 command-filter table.
- T5.4 benchmark table.

**Hand-off:** "With the arm following the hand within ~60 ms, stroke-level problems appeared: singularities that
stall the arm (Chapter 6), and the need to constrain motion about a port (Chapter 7)."

---

## Chapter 6 — Safety Behaviours  {#chapter-6}

**Job:** the system protects itself and the operator without taking control away, and the operator can *feel*
limits. The haptic part is honest about the device.

- **6.1 Why Servo's singularity handling fails teleop:** R6.1.
- **6.2 Directional guard:**
  - 6.2.1: m = (Σσᵢ⁻²)^(−½) and why σ_min's gradient flips (1.36 vs 0.026 per-cycle jump).
  - 6.2.2: thresholds 30/60, DLS damping, removing the into-singularity component, back-off, applying it in
    joint space, raising Servo's thresholds while the guard runs.
  - 6.2.3: R6.2, R6.3.
- **6.3 Haptic cues:**
  - 6.3.1: design (R6.4): walls and detent on the *stylus target*, not the robot, so there is no loop through
    the arm; singularity push direction (J_dls⁺)ᵀ g.
  - 6.3.2: **the OpenHaptics finding** (R6.5) with the lift-test table and the process-split fix diagram.
  - 6.3.3: device characterisation (R6.6, R6.7) and the staircase plot.
- **6.4 Other safeguards:** R6.8 in a table (safeguard, failure it prevents, how it was tested).
- **6.5 Summary.**

**Figures:** F6.1 condition number vs time, guarded vs unguarded (sim); F6.2 guard geometry sketch; T6.1
lift-test matrix; F6.3 touch_server/touch_bridge architecture; F6.4 force staircase response; T6.2 safeguards.

**Hand-off:** "The guard and cues bound free motion; laparoscopic work also needs the shaft to pivot about a fixed
port, which Chapter 7 adds in software."

---

## Chapter 7 — Software Remote Centre of Motion Teleoperation  {#chapter-7}

**Job:** a clean derivation, a design process that rejected weaker alternatives with evidence, and an independent
validation that is honest about where the residual error comes from.

- **7.1 Requirements:** 2 DOF lost; the tip-steering convention (da Vinci-like), with the alternative (tilt pivots
  the shaft) rejected because the tip moves against the hand.
- **7.2 Constraint-preserving twist:** v = insertion·axis + ω × (p − q); caps scaled together; filter before the
  lever; prediction one loop delay ahead (R7.1, R7.2).
- **7.3 Lever problem:** r(d)/d amplification (≈ 30× at 10 mm) and the V6 real-arm runs (R7.3).
- **7.4 Depth-invariant law:** derivation of tan θ = k|s|/r(d) and the properties (bounded at the port, no swing
  on insertion, one smooth function, absolute target); the depth table; alternatives tried (R7.4, R7.5);
  continuity through 0 (R7.6); behaviour past 130 mm; choice of k = 0.5.
- **7.5 Entry, exit, retraction:** chord, exit interlock at ≤ 3 mm, 5 mm retract, max-error hold.
- **7.6 Independent validation:**
  - 7.6.1: setup (wand in SimpleHolder, hand-eye, replay with chord / `--rcm`, 14 replays).
  - 7.6.2: results table R7.7, plus NDI_PLOT figures.
  - 7.6.3: separating controller from model error (R7.8–R7.10). The latch-timing check, pivot offsets along base z,
    and the TF-vs-controller tool0 angle → mm at the lever. Cross-check: the same ~1.5 mm in the micro-tip
    calibration. **If you run the calibrated-kinematics replay, it goes here and becomes the decisive result.**
- **7.7 Summary.**

**Figures:** F7.1 RCM geometry (tip, point, tool0, lever r(d), θ, s); F7.2 tool0-per-mm and pivot-per-mm vs depth
for V6 / fades / final law; F7.3 tool0 sideways speed during in/out motion, fade vs law; F7.4 NDI RCM error vs
time for one replay per set; T7.1 replay results; F7.5 teleop-latched vs NDI-latched points and pivots in
base_link.

**Hand-off:** "The RCM and its validation both depend on knowing the tool, which Chapter 8 makes exchangeable and
calibrated."

---

## Chapter 8 — Modular Tooling and Macro–Micro Integration  {#chapter-8}

**Job:** Lachlan's "time-consuming exchange" problem is answered with a measured coupling plus a calibration and
software chain, and the micro module works with one stylus.

- **8.1 Quick-change coupling:**
  - 8.1.1: kinematic-coupling design rules (Slocum), the cam preload calculation (107 N, 1.2 N·m), ISO 9409 tool
    face.
  - 8.1.2: iterations (printed domes → steel balls/pins → V2 rods + replicated balls + 3-lug cam → V3 99 PCD +
    lightened plate) and why each one changed.
  - 8.1.3: exchange time and repeatability (R8.2), against Thesis A threshold (ii) and Ko et al.
- **8.2 Tool library:** divot-free calibration (R8.3, R8.4), switching and interlocks (R8.5), with the tool
  recorded in META.
- **8.3 Micro module:** faults found (R8.6), position control and task weighting (R8.7). Be explicit about what
  is sim-only.
- **8.4 One stylus for two stages:** mode switch design (R8.8), the 10-DOF demonstration (R8.9).
- **8.5 Summary.**

**Figures:** F8.1 coupling exploded CAD + photo; F8.2 the cam / preload diagram; F8.3 tip re-seat scatter (20
swaps, 3 sessions) with a 1 mm circle for scale; F8.4 swap time histogram; T8.1 calibration methods and quality;
F8.5 micro module step response (sim); F8.6 mode-switch state machine.

---

## Chapter 9 — Discussion  {#chapter-9}

**Job:** answer the question, argue the novelty, own the limitations. This is where the "interfaces" thesis
statement is made explicitly.

- **9.1 Answering the RQ:** the threshold scorecard table from `04_research_question_scorecard.md`.
- **9.2 Lag–smoothness bound:** the group-delay argument (C4). Prediction (lead, Kalman) partially breaks it at
  steady speed (position mode's 30 ms lead), and a tremor model could go further: future work.
- **9.3 Where the limits lie:** the symptom / obvious suspect / actual cause table (`01_story_and_novelty.md`
  §1.2). What it means for people building on open stacks: measure at every interface, and validate on real
  hardware, because the mock hid V1's crawl.
- **9.4 Novelty vs prior work:** go through C1–C10 against the literature (`01_story_and_novelty.md` §1.5).
- **9.5 Limitations:**
  - one operator, no user study;
  - single sessions for some results;
  - the factory kinematics are uncalibrated;
  - faulty haptic device;
  - micro module and position mode lack real-hardware validation (unless they are run);
  - the Vega's 60 Hz rate cannot see 100–250 Hz vibration (joint current is used instead);
  - `/joint_states` aliasing above 250 Hz;
  - exchange repeatability is at the tracker's noise floor.
- **9.6 Implications for low-cost surgical robotics:** what cost/performance this shows is reachable, and what
  still needs dedicated hardware (a stiffer arm, real-time control, a calibrated model).

---

## Chapter 10 — Conclusion and Future Work

- **Conclusions:** one paragraph per aim with the number.
- **Future work** (ordered):
  1. Calibrated kinematics (decisive for RCM error).
  2. Real-arm position-mode validation and buzz-pose study (microphone / accelerometer above 250 Hz).
  3. Haptics on a working Touch.
  4. Micro module validation with NDI ball tracking.
  5. Predictive tremor-aware filtering to beat the lag bound.
  6. User study.
  7. Closed-loop RCM using the NDI (visual servoing).
  8. Paused bag recorder (no start-up jumps).

---

## Appendices (already in the skeleton)

| App | Content | Source |
|---|---|---|
| A | Version history table V1–V8 + tags | docs/VERSIONS.md summary table |
| B | Software architecture, node graph | docs/system_architecture.drawio |
| C | Final teleop settings | macro_micro_robot/teleop_settings.py (at the final tag) |
| D | Coupling drawings, holder, BOM | Onshape docs; PartsList.pdf |
| E | Calibration procedures | VERSIONS.md calibration sections; scripts |
| F | Per-run tables, extra spectra, all NDI plots | logs/*/METRICS_SUMMARY.csv, NDI_PLOT.png |
| G | Risk assessment | lab RA |
| H | Reproducibility: tags, replays, data | README, VERSIONS.md "Runs and comparing versions" |
