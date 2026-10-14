#!/usr/bin/env python3
"""Plot the Touch target path against the UR's actual tool0 path for one logged run.

    python3 scripts/plot_paths.py                      # logs/NEW_RUN, translation
    python3 scripts/plot_paths.py logs/sine_baseline_1 # a renamed run
    python3 scripts/plot_paths.py logs/NEW_RUN -a      # angles only
    python3 scripts/plot_paths.py logs/NEW_RUN -t -a   # translation and angles
    python3 scripts/plot_paths.py logs/NEW_RUN --save  # also write PNGs into the run folder
    python3 scripts/plot_paths.py logs/V2              # every run in V2: metrics table, no plots
    python3 scripts/plot_paths.py logs/V2/*_10CM logs/V3/*_10CM   # compare runs across versions

Reads TOUCH_PATH.csv and UR_PATH.csv written by trajectory_recorder.py (both in
base_link, both stamped with the recorder's clock `t`).

-t (translation, the default): the two paths in 3D, then x / y / z against time
and the distance between the paths.
-a (angles): the tool orientation as a rotation since the start of the run,
about base_link x / y / z (deg, rotation vector components), against time, and
the angle between target and robot orientation.

Errors are taken at each Touch sample, with the UR pose interpolated at the same
time. The robot follows the target with some delay, so the script also finds the
lag that best lines them up (smallest RMS error) and shows the error with that
lag removed: what is left is shape error, not delay. Translation and angles get
their own lag.

UR_PATH rows are tool0, like the Touch target: the recorder moves the RTDE
pendant TCP (tool0_controller) back to tool0. Runs logged before it did that
(no tcp_offset in META.yaml), or where it could not (source 'rtde_tcp'), are
corrected here instead, assuming the robot was at the target when WHITE first
went down (the teleop engages at the robot's own pose, so it is).

Every run analysed gets a METRICS.yaml (the numbers printed, plus where the run
came from); a folder of runs also gets METRICS_SUMMARY.csv, one row per run.
Besides the errors and lag above:
  jitter        RMS of the motion above --jitter-hz (default 5 Hz: tremor, encoder
                noise, oscillation) while engaged, for the robot and for the Touch
                target. A filter should lower the target's and the robot's jitter
                without raising the lag.
  final error   robot vs the last target, at the end of the log (after it settled)
  overshoot     how far the robot went past the last target along the overall
                direction of motion; only for runs that move >= 10 mm net in a
                roughly straight line (the single-axis moves)
Compare metrics between replays of the same run (stylus_replay.py), not between
two hand-made runs: only then is the input the same.
"""

import argparse
import csv
import glob
import os
import sys

import numpy as np
import matplotlib.pyplot as plt
import yaml
from scipy.signal import butter, sosfiltfilt
from scipy.spatial.transform import Rotation, Slerp

DEFAULT_RUN = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "logs", "NEW_RUN")
TOUCH_COLOR = "#2a78d6"     # target
UR_COLOR = "#eb6834"        # actual
ERROR_COLOR = "#52514e"
LAGGED_COLOR = "#a8a7a0"
MAX_LAG = 1.5               # s, longest delay searched for (V1 lagged ~0.5 s)
GAP = 0.05                  # s, Touch rows further apart than this are separate strokes
JITTER_HZ = 5.0             # default high-pass cutoff for the jitter metric
RESAMPLE_HZ = 500.0         # paths are resampled to this before filtering
MIN_STROKE = 0.5            # s, shorter strokes are left out of the jitter metric
OVERSHOOT_MIN_MOVE = 0.01   # m, net target motion needed for an overshoot figure
OVERSHOOT_MIN_STRAIGHTNESS = 0.8   # net / path length of the target, also needed
# Lag is only reported when the target gets at least this far from where it started
# (mm / deg);
# with less, the best-fit lag is just noise.
LAG_MIN_MOTION = 5.0


def read_csv(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    # UR_PATH: keep the RTDE rows when there are any (TF fallback only fills gaps)
    rtde = [r for r in rows if r.get("source") in ("rtde", "rtde_tcp")]
    rows = rtde or rows
    if not rows:
        return None
    t = np.array([float(r["t"]) for r in rows])
    xyz = np.array([[float(r["x"]), float(r["y"]), float(r["z"])] for r in rows])
    quat = np.array([[float(r[c]) for c in ("qx", "qy", "qz", "qw")] for r in rows])
    order = np.argsort(t, kind="stable")
    return t[order], xyz[order], quat[order], rows[0].get("source")


def ur_is_pendant_tcp(run, ur):
    """True if UR_PATH still holds the pendant TCP rather than tool0 (older runs)."""
    source = ur[3]
    if source != "rtde":
        return source == "rtde_tcp"     # 'tf' rows are tool0 already
    try:
        with open(os.path.join(run, "META.yaml")) as f:
            meta = yaml.safe_load(f) or {}
    except OSError:
        return False
    return "tcp_offset" not in meta


def tcp_to_tool0(ur, touch):
    """Move pendant TCP rows to tool0 using the first Touch sample, where target == tool0.

    Returns (xyz, quat) of tool0."""
    ur_t, ur_xyz, ur_quat = ur[:3]
    i = min(np.searchsorted(ur_t, touch[0][0]), len(ur_t) - 1)
    tcp = Rotation.from_quat(ur_quat)
    target = Rotation.from_quat(touch[2][0])
    offset_rotation = target.inv() * tcp[i]                          # tool0 -> TCP
    offset = tcp[i].inv().apply(ur_xyz[i] - touch[1][0])            # in the TCP frame
    print(f"  UR_PATH is the pendant TCP: moved back to tool0 by {np.round(offset * 1000, 1)} mm (TCP frame), "
          f"{np.degrees(offset_rotation.magnitude()):.1f} deg")
    return ur_xyz - tcp.apply(offset), (tcp * offset_rotation.inv()).as_quat()


def with_gaps(t, xyz, gap=GAP):
    """Insert NaN rows between strokes so lines are not drawn across clutch releases."""
    breaks = np.where(np.diff(t) > gap)[0] + 1
    return np.insert(t, breaks, np.nan), np.insert(xyz, breaks, np.nan, axis=0)


def interpolate(t_query, t, xyz):
    """UR position at t_query; NaN outside the logged range."""
    out = np.column_stack([np.interp(t_query, t, xyz[:, i], left=np.nan, right=np.nan) for i in range(3)])
    return out


class RotationTrack:
    """Slerp through UR orientations; NaN-safe lookups outside the logged range."""

    def __init__(self, t, quat):
        t, first = np.unique(t, return_index=True)
        self.t = t
        self.slerp = Slerp(t, Rotation.from_quat(quat[first]))

    def angle_to(self, t_query, target):
        """Angle (deg) between `target` (Rotation, one per t_query) and the UR at t_query."""
        valid = (t_query >= self.t[0]) & (t_query <= self.t[-1])
        out = np.full(len(t_query), np.nan)
        if valid.any():
            out[valid] = np.degrees((target[valid] * self.slerp(t_query[valid]).inv()).magnitude())
        return out


def best_lag(error_at):
    """Delay (s) of the UR behind the target that minimises RMS of error_at(lag)."""
    lags = np.arange(0.0, MAX_LAG, 0.002)
    rms = []
    for lag in lags:
        d = error_at(lag)
        rms.append(np.sqrt(np.nanmean(d ** 2)) if np.isfinite(d).sum() > 10 else np.inf)
    i = int(np.argmin(rms))
    return lags[i], rms[i], rms[0]


def set_equal_3d(ax, xyz):
    centre = np.nanmean([np.nanmin(xyz, axis=0), np.nanmax(xyz, axis=0)], axis=0)
    half = np.nanmax(np.nanmax(xyz, axis=0) - np.nanmin(xyz, axis=0)) / 2 or 1.0
    ax.set_xlim(centre[0] - half, centre[0] + half)
    ax.set_ylim(centre[1] - half, centre[1] + half)
    ax.set_zlim(centre[2] - half, centre[2] + half)


def plot_error(ax, touch_t, gt, error_at, lag, unit):
    """Error at the same time and with the best lag (s) removed."""
    d_raw, d_lag = error_at(0.0), error_at(lag)
    ax.plot(gt, with_gaps(touch_t, d_raw[:, None])[1], color=ERROR_COLOR, linewidth=1.5, label="same time")
    ax.plot(gt, with_gaps(touch_t, d_lag[:, None])[1], color=LAGGED_COLOR, linewidth=1.0,
            label=f"UR shifted back {lag * 1000:.0f} ms")
    ax.set_ylabel(f"error ({unit})")
    ax.grid(alpha=0.3)
    ax.legend(loc="best")


def plot_translation(name, ur_t, ur_xyz, touch, metrics):
    """3D paths + x / y / z over time; returns the figures."""
    ur_xyz = ur_xyz * 1000.0       # mm
    fig3d = plt.figure(f"{name} - paths", figsize=(8, 7))
    ax = fig3d.add_subplot(projection="3d")
    ax.plot(*ur_xyz.T, color=UR_COLOR, linewidth=2, label="UR actual (tool0)")
    all_xyz = ur_xyz
    if touch is not None:
        touch_t, touch_xyz = touch[0], touch[1] * 1000.0
        gt, gxyz = with_gaps(touch_t, touch_xyz)
        ax.plot(*gxyz.T, color=TOUCH_COLOR, linewidth=2, label="Touch target")
        all_xyz = np.vstack([ur_xyz, touch_xyz])
        # Thin connectors from target to where the robot was at the same moment
        ur_at = interpolate(touch_t, ur_t, ur_xyz)
        for i in np.linspace(0, len(touch_t) - 1, min(60, len(touch_t))).astype(int):
            if np.all(np.isfinite(ur_at[i])):
                ax.plot(*np.vstack([touch_xyz[i], ur_at[i]]).T, color=ERROR_COLOR, linewidth=0.6, alpha=0.6)
    ax.scatter(*ur_xyz[0], color=UR_COLOR, s=40, marker="o", label="start")
    ax.scatter(*ur_xyz[-1], color=UR_COLOR, s=40, marker="s", label="end")
    set_equal_3d(ax, all_xyz)
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")
    ax.set_zlabel("z (mm)")
    ax.set_title(f"{name}: Touch target vs UR path (base_link)")
    ax.legend(loc="upper left")

    rows = 4 if touch is not None else 3
    fig_t, axes = plt.subplots(rows, 1, sharex=True, figsize=(10, 2.2 * rows), num=f"{name} - over time")
    for i, axis in enumerate("xyz"):
        axes[i].plot(ur_t, ur_xyz[:, i], color=UR_COLOR, linewidth=1.5, label="UR actual")
        if touch is not None:
            axes[i].plot(gt, gxyz[:, i], color=TOUCH_COLOR, linewidth=1.5, label="Touch target")
        axes[i].set_ylabel(f"{axis} (mm)")
        axes[i].grid(alpha=0.3)
    axes[0].legend(loc="upper right")

    if touch is not None:
        error_at = lambda lag: np.linalg.norm(interpolate(touch_t + lag, ur_t, ur_xyz) - touch_xyz, axis=1)
        plot_error(axes[3], touch_t, gt, error_at, (metrics["translation"]["lag_ms"] or 0) / 1000, "mm")
    axes[-1].set_xlabel("time since recording start (s)")
    fig_t.suptitle(f"{name}: position and distance between paths")
    fig_t.tight_layout()
    return {"paths_3d": fig3d, "paths_time": fig_t}


def plot_angles(name, ur_t, ur_quat, touch, metrics):
    """Rotation since the start of the run about base_link x / y / z, over time; returns the figure."""
    ur_rot = Rotation.from_quat(ur_quat)
    reference = ur_rot[0].inv()        # both curves start from the robot's first logged orientation
    ur_angles = np.degrees((ur_rot * reference).as_rotvec())
    rows = 4 if touch is not None else 3
    fig, axes = plt.subplots(rows, 1, sharex=True, figsize=(10, 2.2 * rows), num=f"{name} - angles")
    if touch is not None:
        touch_t, touch_rot = touch[0], Rotation.from_quat(touch[2])
        gt, gangles = with_gaps(touch_t, np.degrees((touch_rot * reference).as_rotvec()))
    for i, axis in enumerate("xyz"):
        axes[i].plot(ur_t, ur_angles[:, i], color=UR_COLOR, linewidth=1.5, label="UR actual")
        if touch is not None:
            axes[i].plot(gt, gangles[:, i], color=TOUCH_COLOR, linewidth=1.5, label="Touch target")
        axes[i].set_ylabel(f"about {axis} (deg)")
        axes[i].grid(alpha=0.3)
    axes[0].legend(loc="upper right")

    if touch is not None:
        track = RotationTrack(ur_t, ur_quat)
        error_at = lambda lag: track.angle_to(touch_t + lag, touch_rot)
        plot_error(axes[3], touch_t, gt, error_at, (metrics["angles"]["lag_ms"] or 0) / 1000, "deg")
    axes[-1].set_xlabel("time since recording start (s)")
    fig.suptitle(f"{name}: tool rotation since start (base_link) and angle between target and robot")
    fig.tight_layout()
    return {"angles_time": fig}


def strokes(t, gap=GAP):
    """(start, end) times of the engaged stretches in the Touch rows."""
    breaks = np.where(np.diff(t) > gap)[0]
    starts = np.concatenate([[t[0]], t[breaks + 1]])
    ends = np.concatenate([t[breaks], [t[-1]]])
    return list(zip(starts, ends))


def jitter(t, values, spans, cutoff):
    """RMS (norm over columns) of `values` above `cutoff` Hz, inside the given time spans.

    Each span is resampled to RESAMPLE_HZ and high-passed forwards and backwards (no
    phase shift); spans shorter than MIN_STROKE are left out. NaN if none are long enough."""
    sos = butter(2, cutoff, btype="highpass", fs=RESAMPLE_HZ, output="sos")
    squares = []
    for start, end in spans:
        if end - start < MIN_STROKE:
            continue
        grid = np.arange(start, end, 1.0 / RESAMPLE_HZ)
        resampled = np.column_stack([np.interp(grid, t, values[:, i]) for i in range(values.shape[1])])
        high = sosfiltfilt(sos, resampled, axis=0)
        squares.append(np.sum(high ** 2, axis=1))
    return float(np.sqrt(np.mean(np.concatenate(squares)))) if squares else float("nan")


def rnd(value, digits=2):
    value = float(value)
    return round(value, digits) if np.isfinite(value) else None


def compute_metrics(ur_t, ur_xyz, ur_quat, touch, jitter_hz):
    """Tracking numbers for one run (mm, deg, ms); None without Touch target rows."""
    if touch is None:
        return None
    touch_t, touch_xyz, touch_quat = touch[0], touch[1], touch[2]
    spans = strokes(touch_t)
    metrics = {"strokes": len(spans), "engaged_s": rnd(sum(e - s for s, e in spans), 3),
               "jitter_cutoff_hz": jitter_hz}

    # Translation
    ur_mm, touch_mm = ur_xyz * 1000.0, touch_xyz * 1000.0
    error_at = lambda lag: np.linalg.norm(interpolate(touch_t + lag, ur_t, ur_mm) - touch_mm, axis=1)
    path_length = np.sum(np.linalg.norm(np.diff(touch_mm, axis=0), axis=1))
    lag, rms_lagged, rms_raw = best_lag(error_at)
    if np.max(np.linalg.norm(touch_mm - touch_mm[0], axis=1)) < LAG_MIN_MOTION:
        lag, rms_lagged = None, rms_raw
    d_raw, d_lag = error_at(0.0), error_at(lag or 0.0)
    offset = np.nanmean(interpolate(touch_t + (lag or 0.0), ur_t, ur_mm) - touch_mm, axis=0)
    translation = {
        "rms_mm": rnd(rms_raw), "max_mm": rnd(np.nanmax(d_raw)),
        "lag_ms": None if lag is None else rnd(lag * 1000, 0),
        "rms_lag_removed_mm": rnd(rms_lagged), "max_lag_removed_mm": rnd(np.nanmax(d_lag)),
        "mean_offset_mm": [rnd(v) for v in offset],
        "final_error_mm": rnd(np.linalg.norm(ur_mm[-1] - touch_mm[-1])),
        "jitter_ur_mm": rnd(jitter(ur_t, ur_mm, spans, jitter_hz), 3),
        "jitter_target_mm": rnd(jitter(touch_t, touch_mm, spans, jitter_hz), 3),
        "overshoot_mm": None,
    }
    net = touch_mm[-1] - touch_mm[0]
    if (np.linalg.norm(net) >= OVERSHOOT_MIN_MOVE * 1000
            and np.linalg.norm(net) >= OVERSHOOT_MIN_STRAIGHTNESS * path_length):
        direction = net / np.linalg.norm(net)
        after_start = ur_t >= touch_t[0]
        translation["overshoot_mm"] = rnd(max(0.0, np.max((ur_mm[after_start] - touch_mm[-1]) @ direction)))
    metrics["translation"] = translation

    # Angles (jitter on the rotation vector since the start, in deg)
    ur_rot, touch_rot = Rotation.from_quat(ur_quat), Rotation.from_quat(touch_quat)
    track = RotationTrack(ur_t, ur_quat)
    error_at = lambda lag: track.angle_to(touch_t + lag, touch_rot)
    lag, rms_lagged, rms_raw = best_lag(error_at)
    if np.degrees(np.max((touch_rot * touch_rot[0].inv()).magnitude())) < LAG_MIN_MOTION:
        lag, rms_lagged = None, rms_raw
    d_raw, d_lag = error_at(0.0), error_at(lag or 0.0)
    reference = ur_rot[0].inv()
    metrics["angles"] = {
        "rms_deg": rnd(rms_raw), "max_deg": rnd(np.nanmax(d_raw)),
        "lag_ms": None if lag is None else rnd(lag * 1000, 0),
        "rms_lag_removed_deg": rnd(rms_lagged), "max_lag_removed_deg": rnd(np.nanmax(d_lag)),
        "final_error_deg": rnd(np.degrees((touch_rot[-1] * ur_rot[-1].inv()).magnitude())),
        "jitter_ur_deg": rnd(jitter(ur_t, np.degrees((ur_rot * reference).as_rotvec()), spans, jitter_hz), 3),
        "jitter_target_deg": rnd(jitter(touch_t, np.degrees((touch_rot * reference).as_rotvec()), spans, jitter_hz), 3),
    }
    return metrics


def print_metrics(metrics):
    tr, an = metrics["translation"], metrics["angles"]
    fmt = lambda v, unit: "   n/a" if v is None else f"{v:7.2f} {unit}"
    print("  translation")
    print(f"    RMS distance, same time:      {fmt(tr['rms_mm'], 'mm')}   (max {tr['max_mm']} mm)")
    print(f"    best lag:                     {fmt(tr['lag_ms'], 'ms')}")
    print(f"    RMS distance, lag removed:    {fmt(tr['rms_lag_removed_mm'], 'mm')}   (max {tr['max_lag_removed_mm']} mm)")
    print(f"    mean error UR - target (xyz): {tr['mean_offset_mm']} mm"
          + ("   <- constant offset? check the pendant TCP"
             if np.linalg.norm([v or 0 for v in tr["mean_offset_mm"]]) > 5 else ""))
    print(f"    final error:                  {fmt(tr['final_error_mm'], 'mm')}")
    print(f"    overshoot:                    {fmt(tr['overshoot_mm'], 'mm')}")
    print(f"    jitter > {metrics['jitter_cutoff_hz']:g} Hz, UR / target:   "
          f"{fmt(tr['jitter_ur_mm'], 'mm')} / {fmt(tr['jitter_target_mm'], 'mm')}")
    print("  angles")
    print(f"    RMS angle error, same time:   {fmt(an['rms_deg'], 'deg')}  (max {an['max_deg']} deg)")
    print(f"    best lag:                     {fmt(an['lag_ms'], 'ms')}")
    print(f"    RMS angle error, lag removed: {fmt(an['rms_lag_removed_deg'], 'deg')}  (max {an['max_lag_removed_deg']} deg)")
    print(f"    final error:                  {fmt(an['final_error_deg'], 'deg')}")
    print(f"    jitter > {metrics['jitter_cutoff_hz']:g} Hz, UR / target:   "
          f"{fmt(an['jitter_ur_deg'], 'deg')} / {fmt(an['jitter_target_deg'], 'deg')}")


def read_yaml(path):
    try:
        with open(path) as f:
            return yaml.safe_load(f) or {}
    except OSError:
        return {}


def run_info(run):
    """Where a run came from: version folder, code version, replay source."""
    meta, replay = read_yaml(os.path.join(run, "META.yaml")), read_yaml(os.path.join(run, "REPLAY.yaml"))
    parent = os.path.basename(os.path.dirname(os.path.abspath(run)))
    return {
        "run": os.path.basename(os.path.abspath(run)),
        "version": parent if parent.upper().startswith("V") and parent[1:].isdigit() else None,
        "date": meta.get("date"),
        "code_version": meta.get("code_version"),
        "replay_of": replay.get("replay_of"),
    }


SUMMARY_COLUMNS = [
    ("translation", "rms_mm"), ("translation", "lag_ms"), ("translation", "rms_lag_removed_mm"),
    ("translation", "final_error_mm"), ("translation", "overshoot_mm"),
    ("translation", "jitter_ur_mm"), ("translation", "jitter_target_mm"),
    ("angles", "rms_deg"), ("angles", "lag_ms"), ("angles", "rms_lag_removed_deg"),
    ("angles", "final_error_deg"), ("angles", "jitter_ur_deg"), ("angles", "jitter_target_deg"),
]


def summary_row(info, metrics):
    row = {"run": info["run"], "version": info["version"] or "", "replay_of": info["replay_of"] or ""}
    for group, key in SUMMARY_COLUMNS:
        value = metrics[group][key] if metrics else None
        row[f"{'ang_' if group == 'angles' else ''}{key}"] = "" if value is None else value
    return row


def print_table(rows):
    columns = ["run", "version"] + [k for k in rows[0] if k not in ("run", "version", "replay_of")]
    widths = {c: max(len(c), *(len(str(r[c])) for r in rows)) for c in columns}
    print("  ".join(c.ljust(widths[c]) for c in columns))
    for r in rows:
        print("  ".join(str(r[c] if r[c] != "" else "-").ljust(widths[c]) for c in columns))


def expand_runs(paths):
    """Run folders (have UR_PATH.csv) as given; other folders -> the runs inside them.

    -> [(run, folder it was found in or None)]"""
    runs = []
    for path in paths:
        path = os.path.normpath(path)
        if os.path.isfile(os.path.join(path, "UR_PATH.csv")):
            runs.append((path, None))
        else:
            inside = sorted(os.path.dirname(p) for p in glob.glob(os.path.join(path, "*", "UR_PATH.csv")))
            if not inside:
                sys.exit(f"{path}: no UR_PATH.csv in it or in its subfolders")
            runs += [(run, path) for run in inside]
    return runs


def analyse(run, args, plot):
    """Metrics (+ plots) for one run; returns (info, metrics or None, figures)."""
    touch = read_csv(os.path.join(run, "TOUCH_PATH.csv"))
    ur = read_csv(os.path.join(run, "UR_PATH.csv"))
    info = run_info(run)
    name = info["run"]
    if ur is None:
        print(f"{name}: UR_PATH.csv has no rows - skipped")
        return info, None, {}
    if touch is not None:
        print(f"{name}: {len(touch[0])} Touch rows, {len(ur[0])} UR rows"
              + (f" (replay of {info['replay_of']})" if info["replay_of"] else ""))
    else:
        print(f"{name}: no Touch target rows (keyboard run, or WHITE never held) - UR path only")
    drift = (read_yaml(os.path.join(run, "META.yaml")).get("tcp_offset") or {}).get("check_max_mm")
    if drift is not None and drift > 5.0:
        print(f"  WARNING: pendant TCP vs TF drifted {drift} mm during the run - TF used the wrong arm "
              "kinematics (ur_type?), so the robot was driven on a wrong model: re-record it")
    ur_t, ur_xyz, ur_quat = ur[:3]
    if touch is not None and ur_is_pendant_tcp(run, ur):
        ur_xyz, ur_quat = tcp_to_tool0(ur, touch)

    metrics = compute_metrics(ur_t, ur_xyz, ur_quat, touch, args.jitter_hz)
    if metrics:
        print_metrics(metrics)
        with open(os.path.join(run, "METRICS.yaml"), "w") as f:
            yaml.safe_dump({**info, **metrics}, f, default_flow_style=None, sort_keys=False)

    figures = {}
    if plot and args.translation:
        figures.update(plot_translation(name, ur_t, ur_xyz, touch, metrics))
    if plot and args.angles:
        figures.update(plot_angles(name, ur_t, ur_quat, touch, metrics))
    return info, metrics, figures


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("runs", nargs="*", default=[DEFAULT_RUN],
                        help="run folders, or folders of runs (default logs/NEW_RUN)")
    parser.add_argument("-t", "--translation", action="store_true", help="plot translation (default)")
    parser.add_argument("-a", "--angles", action="store_true", help="plot angles")
    parser.add_argument("--save", action="store_true", help="save PNGs into the run folder")
    parser.add_argument("--plot", action="store_true", help="plot even when several runs are given")
    parser.add_argument("--no-plot", action="store_true", help="metrics only")
    parser.add_argument("--jitter-hz", type=float, default=JITTER_HZ,
                        help=f"jitter = motion above this frequency (default {JITTER_HZ:g} Hz)")
    args = parser.parse_args()
    if not args.angles:
        args.translation = True

    runs = expand_runs(args.runs)
    plot = not args.no_plot and (len(runs) == 1 or args.plot)
    rows, folders = [], {}
    for run, folder in runs:
        info, metrics, figures = analyse(run, args, plot)
        rows.append(summary_row(info, metrics))
        if folder:
            folders.setdefault(folder, []).append(rows[-1])
        if args.save:
            for file, fig in figures.items():
                fig.savefig(os.path.join(run, f"{file}.png"), dpi=150)
            print(f"  saved {', '.join(f + '.png' for f in figures)} in {run}")
        print()

    for folder, folder_rows in folders.items():
        with open(os.path.join(folder, "METRICS_SUMMARY.csv"), "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(folder_rows[0]))
            writer.writeheader()
            writer.writerows(folder_rows)
        print(f"Wrote {os.path.join(folder, 'METRICS_SUMMARY.csv')}")
    if len(rows) > 1:
        print()
        print_table(rows)
    if plot:
        plt.show()


if __name__ == "__main__":
    main()
