# 4. Research question scorecard

Your Thesis A question (from `~/Desktop/Thesis A/Lit Review/Lit Review.tex`):

> *Can a UR5e based macro–micro robotic system satisfy predefined performance thresholds under controlled
> laboratory conditions by simultaneously achieving: (i) at least 30 % reduction in distal tip vibration amplitude
> in the peak frequency band relative to an unfiltered baseline, (ii) median end effector exchange cycle time of
> ≤ 20 s with post swap positional repeatability error of ≤ 1.0 mm, and (iii) RCM tracking error of ≤ 1.0 mm
> without increasing command to motion latency beyond 50 ms?*
>
> Hypothesis: zero-phase smoothing, repeatable exchange and constraint-aware monitoring will satisfy all three;
> failing any one rejects it.

Two wording issues to fix in the thesis: the arm is a **UR16e**, and the smoothing is causal (real-time
teleop cannot be zero-phase). Say why in Ch 3.

---

## Scorecard (current evidence)

| Threshold | Evidence | Verdict now | What would settle it |
|---|---|---|---|
| **(i) ≥ 30 % peak-band vibration reduction vs unfiltered baseline** | Unfiltered velocity mode → V5 filtering: tool rotation 15–25 Hz 0.0155 → 0.0036 rad/s (**−77 %**), 19.5/39 Hz peaks gone (R5.13). Wrist torque chatter 45–250 Hz 0.34–0.98 Nm → 0.03–0.08 Nm (**≈ −90 %**, back to trajectory-mode levels) (R5.9, R5.15). Buzz: pose-dependent 1.81 A; position-mode fix untested on the arm (R5.17–18) | **Met** (for velocity mode vs its unfiltered baseline) — but the baseline run was not kept and the measure is robot-side, not "distal tip" | One controlled replay set: same stylus run through (a) unfiltered velocity, (b) V5 tuned, (c) position mode, with the **NDI on the tip** (independent tip vibration up to 30 Hz) + joint current for 45–250 Hz. Gives a clean "distal tip" number. See `07_remaining_experiments.md` E2 |
| **(ii) median exchange ≤ 20 s, post-swap repeatability ≤ 1.0 mm** | 3 × 20 swaps: gaps 3.3–9.3 s (median ≈ 4.8 s); tip re-seat ≤ 0.19 mm max, 0.02–0.06 mm rms (R8.2) | **Met, preliminary** — protocol inferred from the data | Confirm what was done in those recordings (fully removed? lever cycled? arm static?). Ideally one more session with video for timing and **robot-frame** repeatability (tip via robot + NDI after each swap, so the calibration chain is included, not just the coupling). E3 |
| **(iii) RCM error ≤ 1.0 mm** | Controller's own: 0.05–0.23 mm (R7.8). Mock: 0.39 mean / 1.35 max (R7.2). **NDI (independent)**: RCMOnly 0.42–0.44 ✓, WholeRun 0.89–0.94 ✓, FastChecking 1.58–1.77 ✗, SlowChecking 2.80–2.92 ✗ (R7.7) | **Partially met**: met by the controller and 2 of 4 trajectories; the misses are explained by factory kinematics (R7.9) | Offline prediction from the controller TCP pose in the bags (E0), then calibrated kinematics + RCM replays (E4, RD1). If 2.9 mm drops below ~1 mm, the hypothesis holds for the controller and the thesis has a decisive result either way |
| **(iii) … without latency > 50 ms** | Real arm: 54–58 ms (hand runs), 62/58 ms (V5 benchmarks), 74/66 ms (tuned). Sim position mode 58 ms. Sim unfiltered velocity mode 30 ms | **Not met with smoothing on**; met without it (sim) | It is a trade-off with (i): see the bound below. Report it as the quantified conflict between (i) and (iii) |

**Overall:** the strict all-or-nothing hypothesis is **rejected on the latency clause**. Be upfront about that.
It is a *good* result if framed correctly: the thesis shows *why* (i) and the 50 ms clause conflict on this arm,
and what it would take to meet both (prediction).

---

## The latency–smoothness bound (the argument for Ch 9)

1. Physiological tremor sits at ~8–12 Hz (cite Riviere et al. or Elble). In your data the stylus tremor was at
   7–13 Hz (R5.11).
2. The velocity feedforward multiplies stylus motion by 2πf, so tremor dominates the command unless removed.
   A constant-velocity Kalman filter cannot remove it, because tremor looks like real motion (R5.11).
3. A causal 2nd-order Butterworth low-pass at cutoff f_c has a low-frequency group delay of
   τ ≈ √2 / (2π f_c) ≈ 0.225 / f_c seconds:

   | f_c | 10 Hz | 8 Hz | 6 Hz | 5 Hz |
   |---|---|---|---|---|
   | τ (formula) | 22.5 ms | 28.1 ms | 37.5 ms | 45.0 ms |
   | measured added lag (R5.12) | 22 ms | 28 ms | 37 ms | – |
   | left at 10 Hz / 20 Hz | – | 53 % / 12 % | 33 % / 7 % | – |

   The formula matches your offline measurements exactly. That is a nice validation to put in the thesis.
4. The UR16e's own response to a streamed set point is ~16 ms dead time + 15 ms lag ≈ 31 ms (R4.2), and the
   teleop + Servo chain adds more (~36 ms before Servo's output in the position-mode sim, filter included).
5. So with a filter that removes ≥ 30 % at 10 Hz (f_c ≲ 10 Hz → ≥ 22 ms) **and** the arm's ~31 ms, you are at
   ≥ 53 ms before any processing. **The 50 ms clause cannot be met together with (i) by causal filtering on this
   arm.** The Thesis A thresholds were set before the arm's response was measured.
6. Ways around it (future work): prediction. Position mode's 30 ms lead already cancels the arm's lag at steady
   speed, and a tremor-aware estimator (e.g. a Kalman filter with an explicit oscillator state for 8–12 Hz,
   Riviere-style weighted-frequency Fourier linear combiner) could remove tremor with less delay.

---

## How to phrase the answer (suggested structure for §9.1)

1. Restate each threshold → evidence → verdict (the table above, cleaned up).
2. "The hypothesis, as stated, is rejected because of the latency clause. The analysis in §9.2 shows that the
   clause conflicts with threshold (i) for any causal filter on this arm, so the more useful result is the
   quantified trade-off: X ms buys Y % of tremor-band reduction."
3. "Thresholds (ii) and (iii) are met at the controller and coupling level; the residual RCM error measured
   independently is attributable to the arm's uncalibrated kinematic model (§7.6.3)."

---

## If you decide to revise the thresholds in Ch 3

Only do this if your supervisor agrees. Revise (iii)'s latency bound to "≤ 50 ms *added* by the teleoperation
software beyond the arm's own response" or "≤ 80 ms end-to-end". **Disclose the change and the reason in Ch 3.**
The honest-rejection framing above is stronger, and I'd recommend it.
