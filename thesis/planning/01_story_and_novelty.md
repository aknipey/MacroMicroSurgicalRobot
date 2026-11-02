# 1. The story and the novelty argument

This is the spine of the thesis. Every chapter, figure and paragraph should serve this story.
The results behind each claim are in `02_results_inventory.md`; IDs like **R5.3** point there.

---

## 1.1 The one-sentence thesis

> **On a low-cost, open-architecture macro–micro surgical robot, teleoperation quality is not limited by the
> high-level controller. It is limited at the *interfaces* between off-the-shelf parts: how MoveIt Servo hands
> commands to the arm, how the UR interpolates them, how the haptic SDK behaves inside ROS 2, how the stylus
> moves mechanically, and how accurate the arm's kinematic model is. Diagnosing each interface with
> identical-input replays and independent optical tracking let simple software and printed hardware turn a
> vibrating proof-of-concept into a measured platform. It now has about 2.5× lower lag, no joint faults, a
> software RCM held to a fraction of a millimetre, and a tool exchange that re-seats within about 0.1 mm in
> about 5 s.**

Everything else hangs off this. The novelty is partly in individual techniques (C3, C5, C6). It is more
convincingly in the **diagnoses**: you found *where* the problems really were, quantified them, and showed
the fix worked under identical input. That kind of contribution survives examiner scrutiny well, because it
rests on measurements rather than on claims of a better algorithm.

---

## 1.2 The narrative arc (how the chapters link)

Think of it as a chain where each fix exposed the next limit. This is the thesis's natural shape, and it
reads far better than "I built feature A, then feature B".

```
Lachlan's prototype (2023): vibrating, monolithic tip, no haptics
   │  moved to UR16e + ROS 2 Jazzy + MoveIt Servo
   ▼
V1  Arm crawls at ~10 % of commanded speed (640–760 ms lag)
   │  WHY? Servo re-bases each step on the lagging measured state  (R5.1)
   ▼
V2/V3  Trajectory output: lag ~140 ms, smooth, scaled motion works
   │  WHY still 140 ms? Built an emulator, validated to 3 %: the arm only
   │  covers 60 % of the commanded displacement                    (R5.2–R5.4)
   ▼
V4  Velocity output: sim says 30 ms. Real arm VIBRATES and FAULTS
   │  WHY? (a) Servo never smooths velocity output; (b) feedforward
   │  differentiates tremor (×2πf); (c) 4 ms stylus quantisation;
   │  (d) update-rate staircases → torque chatter → joint faults  (R5.5–R5.9)
   ▼
V5  Kalman on stylus + command low-pass + 500 Hz chain + joint limiter
   │  → 2.5× lower lag than trajectory mode, no faults, chatter gone  (R5.10–R5.15)
   │  Residual "catches" → NOT the robot: Touch pitch gimbal stick-slip (R5.16)
   ▼
V6  Strokes jam in singularities → directional singularity guard  (R6.1–R6.3)
   │  Laparoscopic work needs a port → software RCM               (R7.1–R7.3)
   ▼
V7  RCM near the port explodes (lever r/d ≈ 30×) → depth-invariant
   │  sideways scaling law                                         (R7.4–R7.6)
   │  Haptic cues → OpenHaptics renders NO force in any ROS 2
   │  process (lttng-ust/libnuma) → process split; unit's force
   │  output faulty                                                (R6.4–R6.7)
   │  NDI validation: controller holds its point to 0.05–0.23 mm,
   │  but true error 0.4–2.9 mm → it's the factory KINEMATICS      (R7.7–R7.10)
   ▼
V8  Modularity: kinematic quick-change (~0.1 mm, ~5 s), calibrated
   │  tool library, micro module revived, one stylus for both      (R8.1–R8.9)
   │  Buzz returns → it's speedj + POSE, not acceleration          (R5.17)
   ▼
After V8  Position (servoj) streaming without the re-basing crawl  (R5.18)
```

The recurring discovery is the punchline of the Discussion. **In almost every case the "obvious" suspect was
wrong, and the cause sat at an interface**:

| Symptom | Obvious suspect | Actual cause (interface) |
|---|---|---|
| V1 crawl | network / gains | Servo's position output re-bases on measured state (Servo ↔ controller) |
| ~140 ms lag | latency | 60 % displacement delivery (Servo trajectory ↔ UR interpolation) |
| Velocity-mode vibration | robot dynamics | tremor × feedforward + Servo not smoothing velocity output (operator ↔ control law ↔ Servo) |
| Joint faults | overspeed | speedj set-point steps → current chatter (controller ↔ UR speedj) |
| Robot "catches" | controller / Kalman | Touch pitch gimbal stick-slip (device mechanics) |
| No haptic force | wrong force commands | OpenHaptics + lttng-ust/libnuma in one process (SDK ↔ ROS 2 runtime) |
| RCM error 0.4–2.9 mm | RCM controller | factory kinematics vs the real arm (model ↔ hardware) |
| Buzz | acceleration / tool | speedj + arm pose (UR velocity loop ↔ structure) |
| Stale-pose kicks | controller | orphaned `ros2_control_node`s publishing old `/joint_states` (process management) |

**This table is probably the single most valuable figure/table in the thesis.** Put a version of it in the
Discussion (§9.3). It also answers the Thesis A gap that the literature validates subsystems in isolation
(see §1.4 below).

---

## 1.3 Contributions (use as the Chapter 1 list)

Phrase each one as *what* + *evidence*. In the thesis, say "to the author's knowledge" only after doing
the literature check in `06_literature_to_find.md`.

**C1. A diagnosis of why MoveIt Servo teleoperation of a UR arm is slow, and a validated emulator to study it.**
Position output crawls because Servo re-bases on the measured state. Trajectory output delivers only ~60 % of the
commanded displacement on both the real UR16e and the emulator, so the P-loop makes up the rest as lag. The
emulator (16 ms dead time + 15 ms first-order lag) reproduces the real arm to within 3 % (144 vs 140 ms;
12.2 vs 11.9 mm). *(Ch 5; R5.1–R5.4)*

**C2. A failure analysis of velocity streaming (`speedj`) for teleoperation, with a fix.**
Servo does not smooth its velocity output. The velocity feedforward amplifies 8–12 Hz physiological tremor by 2πf.
The Touch updates only every ~4 ms. Command-rate staircases at 100/250 Hz produce wrist torque chatter, and in
all three recorded faults the faulting joint had its highest current chatter of the run. The fix is a
constant-velocity Kalman filter on the stylus, a command low-pass, a 500 Hz chain and a joint velocity /
acceleration limiter with critically damped smoothing after the caps. The result is 2.5× lower lag than
trajectory output, less than half the position error, no faults and chatter back to trajectory-mode levels.
*(Ch 5; R5.5–R5.15)*

**C3. A hybrid "integrated-velocity position streaming" output mode.** Servo velocities are integrated over real
elapsed time into the limiter's own joint positions and sent with a 30 ms lead as `servoj`. This avoids both V1's
re-basing crawl and speedj's pose-dependent buzz. *(Ch 5; R5.17–R5.18; real-arm validation is still pending,
see `07_remaining_experiments.md`)*

**C4. A quantified lag–smoothness trade-off and a practical bound.** Removing tremor with a causal filter costs
≈ 0.225/f_c s of group delay (2nd-order Butterworth): 28 ms at 8 Hz, 37 ms at 6 Hz, 45 ms at 5 Hz. Added to the
arm's own ~31 ms response, this means a 50 ms end-to-end bound and a ≥30 % tremor-band reduction cannot both be
met without prediction on this arm. This explains the Thesis A threshold result honestly, and is itself a
design insight. *(Ch 5 + Ch 9; R5.12, R5.15)*

**C5. A directional singularity guard for Cartesian teleoperation.** It uses the analytic gradient of
m = (Σσᵢ⁻²)^(−½) so that it removes only motion heading into the singularity. The tool slides along the
workspace edge instead of stalling (Servo scales the whole twist), and the arm backs out of a singularity Servo
would halt in permanently. The same gradient, mapped through (J_dls⁺)ᵀ, gives the direction of a haptic
"push-out" cue. *(Ch 6; R6.1–R6.3, R6.4)*

**C6. Software RCM teleoperation with a depth-invariant sideways scaling law.**
- The twist is built about the RCM point, filtered before the lever arm is formed, and predicted one loop delay
  ahead. This cut the offline error from 5.8 mm to 0.45 mm.
- The law tan θ = k·|s|/r(d) makes a sideways stylus movement move tool0 by the same amount at every depth from
  0 to 130 mm. The pivot rate stays bounded at the port (0.09–0.16°/mm), where the naive mapping needs ~30× arm
  motion at 10 mm, and changing depth with a sideways offset no longer swings the arm.
- On the real arm the controller holds its own RCM to 0.05–0.23 mm rms. *(Ch 7; R7.1–R7.6)*

**C7. Independent optical validation that separates controller error from kinematic-model error.**
Using an NDI Vega and repeatable stylus replays, the true RCM error is 0.4–2.9 mm rms, against 0.05–0.23 mm
self-reported. The gap is pose-dependent and comes from the UR16e factory kinematics (0.1–0.25° TF-vs-controller
tool0 mismatch). A methodological point follows: **controller-internal error metrics under-report
port-constraint error by an order of magnitude on uncalibrated arms.** *(Ch 7; R7.7–R7.10)*

**C8. A practical finding for the open-source haptics community:** OpenHaptics 3.4 silently renders no force in
any process that loads liblttng-ust or libnuma, which every ROS 2 node does. A lift-test matrix isolated the
cause, and a process-split driver (`touch_server` + `touch_bridge` over UDP) fixed it. *(Ch 6; R6.5)*

**C9. A modular tooling chain.**
- A printed kinematic quick-change coupling with cam preload: 20-swap sessions re-seat the tool tip within
  0.02–0.06 mm rms (max 0.19 mm), with swaps of ~3–9 s *(preliminary, protocol to confirm)*.
- Tool calibration without a divot: wrist-jog hand-eye, and a single-ball method for tools without a wand.
- A calibrated tool library switchable at run time with safety interlocks.
- The inherited micro module revived: startup deadlock, lost handshake and a rad/deg bug fixed; settling 69 ms
  vs 1.1 s in simulation.
- A one-stylus macro/micro mode switch. *(Ch 8; R8.1–R8.9)*

**C10. A reproducible evaluation framework for teleoperation development.**
- Every version is git-tagged, with full settings in each run's META.
- Stylus replay gives benchmark repeatability better than 1 %.
- The emulator supports offline tuning; the sim predicted the real tuned benchmarks almost exactly.
- The recorder detects wrong kinematics (TCP/TF drift check) and missing NDI.

*(Ch 4 + Appendix A/H; R4.1–R4.5)*

You probably won't keep all ten as top-level items. **Suggested top five for the abstract and introduction:**
C2 (+C1), C6, C7, C9 and C4. Present C5 and C8 as strong secondary contributions, and C3 as "proposed, sim-validated".

---

## 1.4 How the contributions answer the literature gaps (from your Thesis A review)

Your Thesis A synthesis found four gaps. Map them explicitly in the Literature Review's last section and again
in the Discussion. Examiners look for this loop to be closed.

| Thesis A gap | What this thesis provides | Chapters |
|---|---|---|
| **G1** How macro-stage dynamics (vibration, compliance) propagate, and how to get smooth, delay-robust teleop without hurting responsiveness | C1, C2, C3, C4: the full chain from stylus to joint current, with causes isolated at each interface and the lag–smoothness trade-off quantified | 5 |
| **G2** Rapid, repeatable exchange with consistent calibration across swaps | C9: kinematic coupling repeatability, calibrated tool library, divot-free calibration | 8 |
| **G3** Safety insufficiently integrated with task-level behaviour | C5 (singularity), C6 (RCM with max-error hold, exit interlock), haptic awareness (C8 + cue design), joint limiter, watchdog, tool-switch interlocks, orphan-driver guard | 6, 7 |
| **G4** No unified experimental framework evaluating control, mechanics and reconfiguration together with repeatable metrics | C10 + C7: replay benchmarks, emulator, independent NDI tracking, versioned stack | 4, 7 |

Add one more framing sentence. Your Thesis A synthesis said these areas "have been validated in isolation".
This thesis shows *why that matters*: most problems appeared only when the subsystems were exercised together
(SDK + ROS runtime, Servo + UR interpolation, controller + calibration).

---

## 1.5 What is genuinely novel vs. good engineering (be honest with yourself)

| Item | Novelty level | How to argue it |
|---|---|---|
| Depth-invariant RCM sideways scaling (C6) | **Probably novel** as a specific mapping; RCM motion scaling is studied, but check | Derive it, show the alternatives you tried failed, show the bounded pivot rate table. Literature check: "RCM teleoperation motion scaling insertion depth", "fulcrum effect compensation" |
| Controller vs model error separation via NDI (C7) | **Methodologically valuable**; the observation is known in calibration literature but rarely shown for software RCM on cobots | Present as an evaluation insight: self-reported constraint error is not enough |
| OpenHaptics + ROS 2 incompatibility (C8) | **Novel practical finding** (very likely undocumented) | Lift-test matrix is clean, reproducible evidence; frame as a contribution to open-source surgical robotics |
| MoveIt Servo 60 % delivery / re-basing (C1) | **Novel diagnosis** for this widely used stack; may be known informally on forums | Emulator validation + real-arm ratio; cite MoveIt Servo docs/source version (2.12.4) |
| speedj chatter → faults; Servo doesn't smooth velocity output (C2) | **Novel diagnosis** in this context | Source-code reference + spectra + fault table |
| Directional singularity guard (C5) | **Incremental**: DLS and manipulability gradients are classic | Novelty is the *combination* for teleop (slide along edge + back-out + haptic direction); compare with Servo's behaviour explicitly |
| Kalman over One Euro for quantised stylus (C2) | **Incremental, well-justified** | The comparison table (noise −20 to −42 % at no added lag; One Euro added 30–50 ms) |
| Quick-change coupling (C9) | **Engineering**, but the measured repeatability vs Ko et al. (15 s automated) and your Thesis A ≤20 s / ≤1 mm threshold is a clear result | Present against literature numbers |
| Replay framework (C10) | **Methodological** | Repeatability < 1 % is the selling point |

**Avoid** claiming "first ever macro–micro system" or "novel Kalman filter". Claim novel *findings* and the
*mapping law*, plus an *integrated, validated platform*.

---

## 1.6 Title options

1. **Smooth, Safe and Modular: Diagnosing and Removing the Limits of Low-Cost Macro–Micro Surgical Teleoperation**
2. **Where the Lag Lives: Interface-Level Diagnosis of Open-Architecture Surgical Teleoperation on a UR16e**
3. **From Proof-of-Concept to Measured Platform: Low-Latency, Port-Constrained and Modular Teleoperation of a
   Macro–Micro Surgical Robot**
4. **Low-Latency Remote-Centre-of-Motion Teleoperation of a Low-Cost Macro–Micro Surgical Robot**

Option 1 is the safest fit to the Thesis A question (smoothness + safety + modularity). Option 3 is the most
honest about the scope.

---

## 1.7 Framing the honest negatives (they strengthen the thesis if handled well)

- **Latency ≤ 50 ms not met with filtering on** (real arm: 54–74 ms). Present this as the trade-off bound (C4),
  not as a failure. Unfiltered velocity mode reached ~30 ms in sim, so the bound is the filter, not the stack.
- **RCM ≤ 1.0 mm met by the controller (0.05–0.23 mm) and by 2 of 4 trajectories under NDI (0.42, 0.89–0.94 mm),
  but not by 2 others (1.58–1.77, 2.80–2.92 mm).** The evidence points to kinematic calibration. State the
  calibrated-kinematics test as the decisive next experiment, and run it if time allows.
- **Haptics not evaluated with users**, because the device's force output is faulty (staircase test). This is
  a characterised hardware limitation and a documented reason.
- **Micro module not validated on hardware** after the fixes (sim/fakes only). Be explicit.
- **Position mode not yet run on the real arm.** Run it (it is priority 1 in `07_remaining_experiments.md`) or
  present it as proposed + sim-validated.
- **Many real-arm tests are single runs by one operator (you).** Replays make the comparisons repeatable, but
  no user study was done. Discuss this in limitations.
