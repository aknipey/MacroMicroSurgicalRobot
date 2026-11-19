# R05 Cause of the latch offset: the tool's line, and a measured correction

**Report:** 4.3 (cause of the latch offset, session D rows of Table 4.2), scorecard (iii), G2c / G4 boxes, 4.5
limitations, Ch 5. **Question:** why does the shaft pivot about a point other than the one latched (folder R03:
latch offset median 0.96 mm, up to 2.69 mm), and does correcting that cause reduce the independently measured
RCM error on new replays?

**Prior claims to test** (from the 11-19 session notes and `docs/VERSIONS.md`; reproduce, don't reuse):
offset not related to tip speed at the latch (r = 0.008); one line fixed in the wand frame explains slow runs to
0.13-0.25 mm; line angle ~0.3 deg (sessions B/C) and ~1.05 deg (session D tool 4); corrected tools from tools 4-8
agree to ~0.15 mm / 0.01 deg; phase L: corrected tool <= 0.35 mm on phantom RCM and slow probing (prediction written
before the run), fast probing improves, phantom task gets worse.

## Data

| Session | Runs | Tool |
|---|---|---|
| A, B, C | the folder R02 run set (`V7`, `V8/CalibratedKinematics`, `V9/*`, `V9b/*`) | tools 1 / 3 |
| D | `V9c/RCM/<mode>` (phantom RCM x3 in six output/filter settings, slow probing x3 at P5), `V9c/RCM_fit2`, `RCM_fit3` (tools 5, 6), `RCM_perturb_lateral1` / `axial2` (tools 7, 8: tool 4 moved 1 mm across / 2 mm along the shaft) | 4-8 |
| D, latch test | `V9c/Latch/Train` (tool 10, training), `V9c/Latch/A_tool10`, `V9c/Latch/B_tool11` (interleaved A B B A A B, four trajectories, three replays each) | 10 (fresh hand-eye fit), 11 (corrected from Train only) |

## Method

- **Line model.** The controller holds its virtual tip still and slides the tool along its own shaft direction,
  both fixed in the tool frame. Fit one line fixed in the wand frame (2 offset + 2 angle parameters, shared by the
  runs in a group) and one held point per run, by least squares on the distance of the line from the point over
  the longest RCM segment. Residual = what is left once the line error is removed. Angle = between the fitted line
  and the tracked shaft axis. Code: `scripts/latch_line_fit.py` (the same code built tool 11).
- **Timing test.** Tip speed at the latch (linear fit over the 0.5 s before) against the latch offset (latched
  point -> best-fit pivot, as folder R02).
- **Cross-validation.** Fit the line on training runs, then fit only the held point on held-out runs.
- **Recovery of known errors.** Corrected tools from the slow-probing runs of tools 4-8 should coincide.
- **Phase L.** As-run NDI RCM error per trajectory and tool, depth slope, line angle, controller error (bag).
- **Exclusions (derived here):** folder R02's camera check (persistent FK-vs-NDI step > 3 mm), tracked < 95 % of
  the RCM segment, and the arm path check: TCP path length < 80 % of the median for the same trajectory and group
  (a replay that did not run as recorded, e.g. at the wrong motion scale).

## Figures

1. RCM error against insertion depth, slow probing, tool A vs B (the depth-proportional term).
2. RCM error per replay by trajectory, tool A vs B, 1 mm threshold.
