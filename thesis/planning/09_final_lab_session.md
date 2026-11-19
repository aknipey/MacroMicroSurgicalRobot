# 9. Final lab session: tests that close the report's gaps

Written 2026-11-16 for the last arm session before submission. **No development:** every test uses existing
scripts and settings only (`teleop_settings.py` values, `config/tools.yaml` entries, `replay_batch.sh`,
`wrist_jog.py`, `wand_tool_calibration.py`). Tests run in priority order. If time runs out, stop after any test:
each one is complete on its own.

New data goes under **`logs/V9c/`** (session **D**, camera placement **D**), so it never mixes with session C in
`V9b/`. The analysis scripts get a session-D block afterwards (see section 4).

| Value | Test | Gap it closes | Arm time | Effect on the thesis | Est. mark |
|---|---|---|---|---|---|
| 1 | T2 Latch-offset cause | Largest RCM error term has no cause (Execution's main hole) | 60 min | "Cause open" → mechanism shown with a measured sensitivity, or a suspect ruled out | +1–2 if clear |
| 2 | T1 20 Hz filter | Latency clause "not met"; model prediction untested | 50 min | Prediction tested on the arm; latency verdict may become Met | ≈ +1 |
| 3 | T3 Tip vibration | Threshold (i) measured only at flange/joints | 40 min | "Met (tip not tracked)" → "Met at the tip", with an aliasing caveat | +0.5 |
| 4 | T7 Redo C3 exchanges | C3 set suspect: possible leaning on the table; 82 % of its spread was slow drift | 15 min | Clean C3 numbers; slow part attributed or removed; Fig 4.11 and C2-vs-C3 comparison firmer | +0.3 |
| 5 | T4 Exchange → RCM | Calibration survives an exchange? (G3 remainder) | 25 min | Last open clause of G3 closed | small |
| 6 | T5 Position lead | "Lead not tuned on the arm" | 20 min | One limitation removed | small |
| 7 | T6 Second Touch | Haptics not evaluated | 15 min | Only matters if forces work: a bench cue demo for G2d | small, likely 0 |

Mark estimates are overall (weighted) points, judged against the rubric. They are judgement, not arithmetic.
For comparison, the presentation fixes outside the lab (placeholders, empty heading, broken reference, 5 pages
over the limit) are worth about **+2–3**, more than any single test. Do those regardless.

Core (T1–T4 and T7) is about 3¼ h of arm time plus 30 min setup. The order on the day (section 3) puts **T2 first**
because it is worth the most. The wand tests are grouped together and the bare-flange T1 comes after them.

**Risk of late data:** every new result has to reach the scorecard, abstract, Ch 4, Ch 5 and the limitations, or
the report contradicts itself. New text also adds pages to a report already over the limit. Integration
(section 4) lists every place each result lands, so nothing is missed.

---

## 1. Ground rules for the whole session

- [ ] **Camera:** place it once (wand tool in view over the RCM work area *and* at the T3 pose) and do not touch the
      tripod again. Bump it → write down the time; runs after that are their own placement.
- [ ] **Wand:** once mounted and calibrated (T2a), it is not touched until T7. T7 and T4 come last among the wand
      tests because they re-seat it.
- [ ] **Hands off the table** during every recording. Lean only on the floor or a separate stand.
- [ ] Exactly one `/joint_states` publisher before the first run (`pgrep -fa ros2_control_node` is empty before
      launching). No `ros2` CLI while the arm moves.
- [ ] NDI running (`python3 -m ndi_tracking run`) for every wand run. Check visibility in each `NDI_META.yaml`
      before moving on (≥ 95 %).
- [ ] Never replay LargeVibrations in velocity output. Unfiltered velocity **at 1:5 only**.
- [ ] Every settings change is **committed** before its runs, so each run's `code_version` names it. Use
      `set_v9_mode.sh` for the base mode, then edit the one extra value by hand and commit with a message
      saying what it is (e.g. `V9c: position, filter 20 Hz`).
- [ ] Keep a short `logs/V9c/REPLAYS_<date>.md` as you go: folder, mode, code version, anything odd.
- [ ] Back up `logs/V9c` off the laptop at the end.

---

## 2. Test procedures

### T1. 20 Hz command filter (value rank 2)

**Why.** The latency clause misses by 2–12 ms. The validated loop model (folder 02) predicts that a 20 Hz cutoff
at the current gain (Kp 8) lags **50.0 ms** at 1:5 and removes **65 %** of 15–25 Hz power. That prediction was
never tested on the arm. Either result is useful: met = the clause passes; missed = an honest test of the model.

**Setup.** Bare flange (as in session C, the 1:1 inputs need it). No NDI needed. `tools.yaml` default as in
session C (tool 3).

**Runs** (`--no-ndi`, 2 replays each):

| Block | Mode | Inputs | Dest |
|---|---|---|---|
| T1a anchor | `set_v9_mode.sh position` (5 Hz) | Scaled5_Fast, Scaled5_Slow | `V9c/NoTool/PositionMode` |
| T1b baseline | `set_v9_mode.sh velocity-unfiltered` | Scaled5_Fast, Scaled5_Slow **only** | `V9c/NoTool/VelocityUnfiltered` |
| T1c test | `position`, then `TOUCH_COMMAND_FILTER = 20.0`, commit | all six: LongBenchmark, ShortBenchmark, NoBuzzArea, LargeVibrations, Scaled5_Fast, Scaled5_Slow | `V9c/NoTool/Position20Hz` |
| T1d (if time) | `position`, `TOUCH_COMMAND_FILTER = 10.0`, commit | Scaled5_Fast, Scaled5_Slow | `V9c/NoTool/Position10Hz` |

Use the same original inputs that session C replayed (`run_v9b.sh` steps 7–10): `V3/LongBenchmark
V3/ShortBenchmark V3/Scaled5_Slow V3/Scaled5_Fast V7/NoBuzzArea V7/LargeVibrations`. For example:

```
./scripts/replay_batch.sh --no-ndi --repeats=2 --dest=V9c/NoTool/Position20Hz \
  V3/LongBenchmark V3/ShortBenchmark V3/Scaled5_Slow V3/Scaled5_Fast V7/NoBuzzArea V7/LargeVibrations
./scripts/replay_batch.sh --no-ndi --repeats=2 --dest=V9c/NoTool/VelocityUnfiltered V3/Scaled5_Slow V3/Scaled5_Fast
```

**Watch for.** At 20 Hz more tremor reaches the arm. If anything buzzes or the pendant shows a protective stop:
Ctrl+C, note it, and drop the 1:1 inputs (the 1:5 inputs are what the clause is judged on).

**Pass.** At 1:5: translation lag ≤ 50 ms **and** 15–25 Hz flange power ≥ 51 % below T1b on the same input.
T1a gives the same-day anchor, so a day effect can be separated from the filter effect. T1d adds a third
point to Fig 4.3's measured curve.

**Effect on the thesis (≈ +1).** The value is in testing a stated prediction, not in passing. The model says
exactly 50.0 ms, so it is a coin toss.
- *Passes:* scorecard latency row → Met at 1:5. The hypothesis is still rejected unless RCM (iii) is also met,
  but the miss narrows to the RCM alone. Abstract, 4.5.1 and Ch 5 (a) change from "predicts" to "measured".
- *Misses by a few ms:* the model is confirmed to within its 4.4 ms error. That strengthens contribution 2
  (validated loop model) and the 4.5.1 argument that the lag–smoothness trade-off is structural.
- *Buzzes or faults at 20 Hz:* a real result too. The filter cutoff is bounded from below by the arm, and the
  Future-work line on tremor-aware prediction gains evidence.
- 1:1 lag at 20 Hz will likely still exceed 50 ms. Report it, but the clause is judged at 1:5.

### T2. Latch-offset cause (value rank 1)

**Why.** The latch offset (up to 2.69 mm) is the largest RCM term and its cause is open. It changed between
days, and between days the **wand was re-seated and recalibrated** as well as the camera moved. The camera
test showed no camera effect, so the tool calibration is the main remaining suspect. Only a lateral tip error
(across the shaft) can move a pivot off the shaft line, so a perturbation tells us directly how sensitive the
latch offset is to it.

**Setup.** Wand on the coupling, camera fixed, NDI on, final camera placement.

1. **T2a: Calibration repeatability (3 fits, nothing touched).** Run the wrist-jog hand–eye calibration three
   times back to back (`ndi_tracking robot-tcp --wand-is-tool -d 90` with `scripts/wrist_jog.py`, then
   `scripts/wand_tool_calibration.py --tool 3`). Add each fit to `tools.yaml` as tools 4, 5 and 6 (name
   `wand_D_fit1..3`, note the time). This gives the spread of the calibration itself.
2. **NDI floor.** 30 s, arm still, wand in view (dest `V9c/NDIStaticFloor`).
3. **T2b: Baseline.** Default tool = 4 (fit 1). Mode `position` (5 Hz, as session C). Replay
   `V7/SlowCheckingRCMSimpleHolder` ×3 and `V7/RCMOnlySimpleHolder:--rcm` ×2 (RCMOnly last, it ends latched).
   Dest `V9c/RCM_fit1`.
4. **T2c: Other fits.** Default tool 5, SlowChecking ×2 → `V9c/RCM_fit2`; tool 6, SlowChecking ×2 → `V9c/RCM_fit3`.
5. **T2d: Perturbations.** Copy tool 4 into two new entries and change only `tip_tool0_mm`:
   - tool 7 `wand_D_plus1mm_x`: x + 1.0 mm (across the shaft);
   - tool 8 `wand_D_plus2mm_axial`: tip moved 2.0 mm along `direction_tool0` (should do almost nothing: a control).

   SlowChecking ×2 each → `V9c/RCM_perturb_x1`, `V9c/RCM_perturb_ax2`.

**Safety.** The perturbed RCM point is only 1–2 mm off the real tip, in free air. `TOUCH_RCM_MAX_ERROR` (5 mm) still
holds the arm if the shaft wanders.

**Reading the result.**

| What happens | Conclusion for the report |
|---|---|
| Latch offset moves ~1 mm with the 1 mm lateral shift, not with the axial | Mechanism shown: offset = lateral tool-calibration error |
| ...and fit1–fit3 differ by about the day-to-day change (0.5–1.5 mm) | Day-to-day change explained by re-calibration |
| Fits agree to ≪ 0.5 mm, offset same as session C | Calibration ruled out; cause narrowed to the latch itself or the arm |
| Offset moves with neither | Calibration ruled out as a cause |

**Effect on the thesis (+1–2 if clear).** This is the most valuable test. The marker's main criticism of
Execution is that the headline RCM finding ends with "cause not established".
- *Calibration is the mechanism:* 4.3.3 gets a cause and a sensitivity ("1 mm of lateral tool-calibration error
  gives X mm of RCM error"). The G2c/G4 boxes change from "cause open" to "addressed". Ch 5 (d) gains a concrete
  rule: software RCM accuracy is bounded by tool calibration across the shaft. This is the kind of original
  insight the top Execution band asks for, and it makes the result publishable.
- *Calibration ruled out:* still progress. The cause narrows to the latch or the arm, and Limitations and Future
  work become specific instead of open.
- The T2a fit spread is also a new number for the error budget (calibration repeatability), whatever happens.

### T3. Vibration at the tool tip (value rank 3)

**Why.** Threshold (i) is about the distal tip. The session C 1:5 benchmarks put the markers out of view, so only
flange and joints were measured.

**Setup.** Wand on, same camera. Default tool 4.

1. **Record a new 1:5 input in view.** In `position` mode, with the NDI live view open, find a pose where the wand
   is well tracked. Record ~15 s at 1:5 of quick small sweeps and twists, similar to Scaled5_Fast (≈ 50 mm/s target
   speed). Check `NDI_META.yaml` visibility ≥ 95 % before using it. Save as `V9c/TipVib_input`.
2. **Replay it ×3** in each mode: `velocity-unfiltered` → `V9c/TipVib/VelocityUnfiltered`; `position` (5 Hz) →
   `V9c/TipVib/Position`; position 20 Hz (if T1 ran without trouble) → `V9c/TipVib/Position20Hz`.

**Metric.** NDI tip-position PSD in 15–25 Hz against unfiltered, with the flange PSD from `/joint_states` of the
same runs alongside. The Vega samples at 60 Hz, so 30–45 Hz content aliases into the band. The flange
comparison on the same runs shows how much that matters.

**Effect on the thesis (≈ +0.5).** The scorecard row (i) loses its "tip not tracked" qualifier, and 3.1's
"adaptation" paragraph and one Limitations sentence shrink.
- Aliasing means a careful marker can still question the tip number, so present it as agreeing with the
  flange result, not replacing it.
- If the NDI floor is too close to the vibration amplitude, the result is "below the tracker's resolution",
  which still bounds tip vibration.
- A first estimate is available offline from session C RCMOnly runs (section 4), so this test adds confidence
  rather than a new finding.

### T7. Redo the C3 exchange set (value rank 4, before T4)

**Why.** You may have been leaning on the table during the 2026-10-30 C3 set. That fits the data: 82 % of C3's
spread was a steady 35 µm/min drift, against 10–18 % for C1 and C2. The report currently says the slow part
"cannot be attributed".

**Setup.** C3 coupling, wand on its tool plate, arm stationary and not commanded (as in the earlier sets), same
fixed camera. Nobody touches the table except to work the lever and the tool. Stand clear between exchanges.

**Protocol** (identical to `logs/Exchange/PROTOCOL.md`, plus one longer still period):
1. `python3 -m ndi_tracking run`, start recording.
2. **60 s seated and still, hands off.** This is longer than before, so the drift with nobody touching anything
   is measured.
3. 20 exchanges: open the lever, take the tool off, move it as far out of the camera's view as possible, refit,
   close the lever fully, step back, leave it seated ≥ 5 s.
4. 60 s seated and still, hands off. Stop recording.
5. Move the folder to `logs/Exchange/<date_time>_QC_V3_redo` (the analysis globs `2026*_QC_V*`).

**Optional, +15 min: C2 in the same session.** Swap to the C2 tool plate and run the same protocol (`..._QC_V2_redo`).
This removes the "each version recorded in a different session" confound from the C2-vs-C3 comparison.

**Effect on the thesis (≈ +0.3).**
- *Drift gone:* the C3 slow part was the table, not the coupling. 4.4.1 can say so, the "cannot be attributed"
  paragraph shrinks, the C3 total spread should fall towards its per-swap 16 µm, and Fig 4.11's C3 panel is
  replaced. Use the redo as the C3 result and mention the original as discarded with its reason.
- *Drift still there with hands off:* the slow part belongs to the setup (camera, arm or coupling), and the
  still periods measure how much. The upper-bound wording stays, now with evidence behind it.
- Either way, threshold (ii) stays met. This firms up a number rather than changing a verdict.

### T4. Exchange, then RCM through the arm (value rank 5, last wand test)

**Why.** The coupling re-seats within 0.04 mm in the camera frame. What is still open is whether a tool's
calibration still holds after an exchange when measured **through the robot's kinematics**, which is what the RCM uses.

**Runs.** Default tool 4. Repeat 5 times: open the lever, take the tool off, refit, close the lever, then replay
`V7/RCMOnlySimpleHolder:--rcm` ×1 (dest `V9c/ExchangeRCM`, replays numbered 1–5). Time each exchange with a phone
stopwatch as well.

**Result.** Tracked RCM error and robot–tracker tip residual per exchange against T2b. Within the T2b replay
spread (≈ 0.07 mm) means the calibration survives the exchange.

**Effect on the thesis (small).** The G3 box loses "remains open", and 4.4.1's last paragraph and Ch 5 (c) can
say that a calibration made once travels with the tool. That supports the modularity claim end to end. The
5 phone-timed exchanges are also an independent check on the 5.5 s median.

### T5. Position-output lead (optional)

Bare flange or wand, either. `position` mode with `SERVO_POSITION_LEAD = 0.02`, then `0.04` (commit each).
Scaled5_Fast and LongBenchmark ×2 each → `V9c/NoTool/Lead20`, `V9c/NoTool/Lead40`. Compare lag and
lag-removed error with T1a. Run only if T1–T4 are done. Note that LongBenchmark needs the bare flange.

**Effect on the thesis (small).** It removes "the 30 ms lead was not tuned on the arm" from Limitations and adds
one sentence to 3.3.2(b). If a different lead is clearly better, report it, but don't change the final-stack
numbers this late.

### T6. Second Touch force check (optional, off the arm)

Swap in Touch SN 19039000821 (rerun Touch_HeadlessSetup), run the staircase force test from `touch_diagnostics`
through the split OpenHaptics process. If the force scales with the command, the cues can at least be shown
working on the bench. If not, skip it: nothing in the report changes.

**Effect on the thesis (likely 0).** If forces scale, G2d can say the cues render correctly on a working unit,
with Appendix F.1 extended. That is a bench demonstration only: whether the cues help the operator stays open.
The shoulder encoder on this unit is stuck, so it can't drive the arm.

### Not worth arm time

- **High-frequency buzz:** it is speedj-only and pose-dependent, and replaying the buzzing input in velocity is
  ruled out. Write up 4.1.4(a) from the existing folder 03 data instead.
- **Micro-tip accuracy:** needs markers on the continuum tip. Not possible without hardware work.
- **Guard on other singularities:** `singularity_input.py` scripts only the wrist case. Changing it is development.

---

## 3. Order on the day

Ordered by value, with tool changes kept to one: all wand tests first, then the bare flange.

1. **Setup (30 min):** pre-flight checks (section 1), camera placed, calibration file checked, NDI running.
2. **Mount the wand** (coupling C3) → T2a calibrations → NDI floor → T2b → T2c → T2d.
3. T3 (record, then replays; the 20 Hz block only if you accept running it before T1 has checked 20 Hz on the
   bare flange. Otherwise skip that block).
4. T7 (C3 exchange redo, optional C2). Stop the stack first so the arm is not commanded, then restart it for T4.
5. T4 (exchanges + RCMOnly replays).
6. **Wand off, bare flange:** T1a → T1b → T1c → (T1d) → (T5).
7. `set_v9_mode.sh final`, back up `logs/V9c`, T6 if wanted.

If the session is cut short, the minimum useful set is **T2b + T2d** (about 25 min after calibration), then
**T1b + T1c at 1:5 only** (about 15 min).

After each block, check before moving on: replay count matches, NDI visibility ≥ 95 %, no `NEW_RUN` left over,
nothing in the stack log about faults or program drops.

---

## 4. Analysis and integration (same day)

Everything below is read-only on `logs/` and writes into `thesis/data_analysis/`. I can run it as soon as the
folders exist.

| Test | Analysis | Report changes |
|---|---|---|
| T1 | 05 and 01: add a session-D block reading `V9c/NoTool/*`; 02: plot measured T1c/T1d lag on Fig 4.3 against the prediction | Table 4.1 (new column or session-D table), Fig 4.3, Table 4.3 latency row, 4.5.1, Abstract, Ch 5 (a), Table 3.4 row |
| T2 | 09 and 10: include `V9c/RCM_*` as day D; new folder 20_latch_offset_sensitivity (offset against tool-calibration shift, fit spread) | 4.3.3 cause paragraph, Fig 4.9 (day D rows), gap boxes G2c/G4, Ch 5 (b), Limitations, Future work |
| T3 | New folder 21_tip_vibration (NDI PSD, flange cross-check) | 4.1.4 threshold (i) sentence, Table 4.3 row (i), 3.1 "adaptations" paragraph, Limitations |
| T7 | 13: add the `_redo` sets (keep the 10-30 C3 set in RESULTS.md as excluded, with the reason); still-period drift reported separately | 4.4.1 numbers and drift paragraph, Fig 4.11 C3 panel, Fig F.10, G3 box, Ch 5 (c) and Abstract "0.19 mm" if the max changes, `PROTOCOL.md` |
| T4 | New folder 22_exchange_through_robot | 4.4.1 last paragraph and G3 box, Ch 5 (c) |
| T5 | 05 session-D block | 3.3.2(b) one sentence, Limitations |

**Offline checks to do now, no arm needed:**
- [ ] Tip speed at the latch instant in existing RCM runs: at 12.5 ms tracker delay, a tip moving 50 mm/s at
      latch gives 0.6 mm of offset. If that explains it, T2 matters less.
- [ ] Tip 15–25 Hz from NDI in existing session C RCMOnly runs (position vs unfiltered velocity, 1:5, wand on): a
      first tip-vibration number even if T3 does not run.
- [ ] Check whether the raw hand–eye recordings store joint angles. If they do, refit the tool through the nominal
      model to separate the kinematic share (limitation in 4.3.3) without the arm.

---

## 5. What Claude sets up when asked in the lab

Say "set up T<n>" and these are prepared, with each command shown before anything moves the arm:

- **Before the session:** a `scripts/run_v9c.sh` in the style of `run_v9b.sh`, one step per block above
  (mode switch + commit, stack restart, batch, recording check, stop to ask). Dry-run it on mock hardware on
  a private ROS domain, then kill every test-domain process.
- **T1/T5:** the `teleop_settings.py` edits (`TOUCH_COMMAND_FILTER`, `SERVO_POSITION_LEAD`) and their commits.
- **T2:** the `tools.yaml` entries for fits 1–3 from `wand_tool_calibration.py` output, and the perturbed tools
  7–8 computed from tool 4 (the axial shift along `direction_tool0`).
- **T7:** the recording start/stop and folder move, then a first-pass run of analysis 13 on the redo before the
  wand is touched again.
- **T2–T4:** setting `default:` in `tools.yaml` per block, then checking the first replay's `META.yaml` `tool:`.
- **After each block:** replay counts, NDI visibility, gap and camera-step checks, and a first-pass number from
  the analysis so a bad block can be redone before the setup changes.
- **After the session:** the section 4 analysis, and the report edits with a backup of the LaTeX first.

Pending offline answer that may change T2: the tip speed at the latch instant (section 4).
