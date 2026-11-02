# Thesis planning

Planning notes for the thesis write-up, drawn from `docs/VERSIONS.md`, the run logs, the Thesis A literature
review and plan (`~/Desktop/Thesis A/`) and Lachlan Scott's 2023 report. Nothing here is thesis text. It is the
map for writing it.

| File | What it's for |
|---|---|
| [01_story_and_novelty.md](01_story_and_novelty.md) | **Start here.** The one-sentence thesis, the narrative arc, the contribution list, the novelty argument, how it closes the Thesis A gaps, title options, how to frame the negatives |
| [02_results_inventory.md](02_results_inventory.md) | Every result with numbers, evidence type, data location, strength rating and chapter (IDs R4.x–R8.x used everywhere) |
| [03_chapter_plan.md](03_chapter_plan.md) | Chapter-by-chapter: job, section contents (matching the LaTeX headings), figures, hand-offs, page budget, writing order |
| [04_research_question_scorecard.md](04_research_question_scorecard.md) | Thesis A question and thresholds vs evidence; the latency–smoothness bound; how to phrase the answer |
| [05_figures_and_tables.md](05_figures_and_tables.md) | Every figure/table to make, its data and how to make it; ⚠ marks analyses to re-create as scripts |
| [06_literature_to_find.md](06_literature_to_find.md) | Literature the results now need, with candidate sources **to verify before citing**; where existing bib entries go |
| [07_remaining_experiments.md](07_remaining_experiments.md) | Testing plan: the factory-kinematics fix (why `ur_calibration` failed + the one-line fix), tests left (E0–E10), tests to redo (RD1–RD8), lab session plan, data hygiene |
| [analysis/exchange_repeatability.py](analysis/exchange_repeatability.py) | The preliminary quick-change repeatability analysis (R8.2) |

## The short version

- **Story:** each fix exposed the next limit, and the cause was almost always at an *interface* between
  off-the-shelf components (Servo ↔ UR, SDK ↔ ROS runtime, device mechanics, kinematic model), not in the
  high-level controller.
- **Headline numbers:**
  - Lag ~150 → ~60 ms (2.5×) with no joint faults; wrist torque chatter back to trajectory-mode levels.
  - Tremor-band vibration −77 %.
  - Software RCM held to 0.05–0.23 mm by the controller, and 0.4–2.9 mm measured independently. The gap is
    factory kinematics.
  - Quick-change re-seats within ≤ 0.2 mm in ~5 s.
- **Against the Thesis A thresholds:**
  - Vibration: met.
  - Exchange: met (preliminary).
  - RCM: met by the controller and by 2 of 4 trajectories under NDI.
  - **Latency ≤ 50 ms: not met with filtering**, and the thesis can show why (filter group delay + the arm's own
    ~31 ms response).
- **Next steps:**
  1. Offline: predict the calibrated-kinematics result from the controller TCP pose already in every bag (E0).
  2. Lab session A: fix `ur_calibration` (one line), load the arm's calibration, redo the hand-eye, replay the RCM
     set (E4, RD1).
  3. Lab session B: 3-mode paired benchmark with NDI tip vibration (E1, E2).
  4. Exchange protocol confirmation (E3).

## Things to check with your supervisor

1. The Thesis C page limit, required front matter (originality, AI-use statement) and submission date.
2. Report against the Thesis A thresholds as written (recommended) or revise them, with disclosure.
3. Whether any study involving other people needs ethics approval (probably skip it).
4. Which contributions to lead with. Suggested: C2 (+C1), C6, C7, C9, C4.
