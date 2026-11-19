# Quick-change exchange repeatability: protocol

As stated by the user on 2026-11-06, about the three sets here. Checked against the recordings the same day.

**Protocol, each set:**
1. 30 s seated and still.
2. 20 swaps. Each swap: lever opened, wand taken off and moved as far out of the camera's view as possible, put back,
   lever closed fully. Left seated for 5 s.
3. 30 s seated and still.

**What the data shows:** 21 still segments per set (20 swaps), 31-42 s at each end, 281-305 s total. Matches.

| Set | Tip spread rms | Max |
|---|---|---|
| 20261027_095016_QC_V1 | 0.06 mm | 0.19 mm |
| 20261027_100249_QC_V2 | 0.02 mm | 0.05 mm |
| 20261030_115609_QC_V3 | 0.05 mm | 0.12 mm |

Analysis: `thesis/planning/analysis/exchange_repeatability.py`. Camera-frame positions, so this is coupling re-seat repeatability
only, and it assumes the arm and camera stayed still. Robot-frame repeatability (through the kinematics) was not measured.

**Still to confirm:** what V1 / V2 / V3 refer to, and that the arm stayed still during each set.
