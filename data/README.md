# Recordings used in the thesis

`recordings/` holds every recording that the results in [`thesis/results/`](../thesis/results/README.md) read:

- 373 teleoperation runs from stack versions V1–V9c and the emulator study
- the three quick-change exchange sessions
- the macro–micro demonstration's node logs

That is about 17 GB, 11 GB of it rosbags.

| File | What it is |
|---|---|
| [`RECORDINGS.md`](RECORDINGS.md) | One row per recording: date, output mode, input, scale, tracker, bag, which results read it, and notes on odd folder names |
| [`used_by.json`](used_by.json) | For each result folder, the recordings and files it read; written by `thesis/results/run_all.py` |
| `MANIFEST.sha256` | SHA-256 of every file under `recordings/`, to check a downloaded or unpacked copy |
| [`make_index.py`](make_index.py) | Regenerates `RECORDINGS.md` and `MANIFEST.sha256` |

## What is in git and what is not

Git holds only the small per-run files:

- `META.yaml`: code version, kinematics, tool and every setting
- `REPLAY.yaml`: which original a replay reproduces
- `NDI_META.yaml`, `NDI_EVENTS.csv`: tracker summary and events
- the session notes
- the demonstration logs

These are enough to see what every run was. The per-sample CSVs (6 GB) and the rosbags (11 GB) are too large for
git; `.gitignore` keeps them out, and they are distributed as one archive:

```bash
# pack (from the repository root)
tar -cf macro_micro_recordings.tar data/recordings data/MANIFEST.sha256
# unpack into a clone and check every file
tar -xf macro_micro_recordings.tar && (cd data/recordings && sha256sum -c --quiet ../MANIFEST.sha256)
```

Archive location: *add the link here once uploaded.*

## Layout

The layout is `recordings/<stack version>/[<condition>/]<input>[_Replay<n>]`, the same as the lab's `logs/`
folder. The run keys in every `RESULTS.md` (for example `V9b/NoTool/PositionMode/LongBenchmark_Replay1`) are
paths in here.

- A folder ending `_Replay<n>` is a replay of a recorded stylus input. Its `REPLAY.yaml` names the original.
- Folders without `_Replay` are live sessions.

The report's sessions A–D are V7, V8 + V9, V9b and V9c respectively.

The files in a run folder are described in [`thesis/results/README.md`](../thesis/results/README.md#facts-about-the-raw-data).
Times share one clock per run (seconds since the recorder started). Positions are in metres, in base_link for
the arm, and in millimetres in the camera frame for the tracker.

## Where it came from, and what was left out

The recorder (`src/trajectory_recorder.py`) writes to `logs/` at the repository root. That folder is the lab
working copy: it is not in git and is never edited. On 19 November 2026 the recordings the results read were
copied from it here, unchanged.

"Read" was established by running every analysis with `thesis/results/run_all.py`, which logs every file each
one opens. A run whose `META.yaml` was only scanned while searching for runs does not count.

On the same Linux disk the copy uses hard links, so it takes no extra space; deleting either copy leaves the
other intact.

Left out of the copy:

| Item | Why |
|---|---|
| `V9c/Warmup/RCMOnlySimpleHolder_Replay1` | warm-up before the session D RCM set; never analysed |
| `V9c/RCM/P5/SlowCheckingRCMSimpleHolder_Replay{1,2}_NO_NDI` | tracker not following; repeated as Replay1–3 |
| `V9c/NDIStaticFloor`, `V9c/NDIStaticFloor_camera1` | static tracker checks; the report uses the session C floor (`V9b/NDIStaticFloor`) |
| `METRICS.yaml` in 28 early runs, `V1–V3/METRICS_SUMMARY.csv` | summaries written by an old plotting script, not recordings |
| `V9b/stack_logs/`, `V9c/stack_logs/` | console output of the lab session scripts |
| `V9c/latch_tool1{1,2}.json`, `V9c/LATCH_TEST*.txt` | scratch output of the latch fit; the corrected tools are in `config/tools.yaml` |

Some folders with odd names are kept because results read them. Their notes in `RECORDINGS.md` say why: for
example the `_failed` V4 runs are the joint faults themselves, and `UNSAVED_…` is a live session the recorder
saved under its default name.

## Not here

These are elsewhere in the repository:

- the hand–eye and tool calibrations: [`config/calibrations/`](../config/calibrations) and [`config/tools.yaml`](../config/tools.yaml)
- the arm kinematics: `config/calibrations/ur16e_kinematics_2026-11-03.yaml`

The raw wrist-jog recordings behind the hand–eye fits, and the stylus files fed to the trajectory player, are
in the `ndi_tracking` package and `~/ros2_ws/trajectories/` on the lab computer. Every replay folder here
carries its own copy of the input it replayed (`STYLUS_RAW.csv`, `TRAJECTORY.*`).
