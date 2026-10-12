#!/usr/bin/env python3
"""Plot the Touch target path against the UR's actual TCP path for one logged run.

    python3 scripts/plot_paths.py                      # logs/NEW_RUN
    python3 scripts/plot_paths.py logs/sine_baseline_1 # a renamed run
    python3 scripts/plot_paths.py logs/NEW_RUN --save  # also write PNGs into the run folder

Reads TOUCH_PATH.csv and UR_PATH.csv written by trajectory_recorder.py (both in
base_link, both stamped with the recorder's clock `t`). Figure 1 is the two paths
in 3D; figure 2 is x / y / z against time and the distance between the paths.

The distance is taken at each Touch sample, with the UR position interpolated at
the same time. The robot follows the target with some delay, so the script also
finds the lag that best lines the paths up (smallest RMS distance) and shows the
distance with that lag removed: what is left is shape error, not delay.

Note: UR_PATH rows with source 'rtde' are the pendant TCP (tool0_controller),
while the Touch target is tool0. If the pendant TCP is not zero, the two differ
by a constant offset - the script prints the mean error vector so you can spot it.
"""

import argparse
import csv
import os
import sys

import numpy as np
import matplotlib.pyplot as plt

DEFAULT_RUN = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "logs", "NEW_RUN")
TOUCH_COLOR = "#2a78d6"     # target
UR_COLOR = "#eb6834"        # actual
ERROR_COLOR = "#52514e"
LAGGED_COLOR = "#a8a7a0"
MAX_LAG = 0.5               # s, longest delay searched for
GAP = 0.05                  # s, Touch rows further apart than this are separate strokes


def read_csv(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    # UR_PATH: TF fallback rows track tool0, not the pendant TCP - drop them when RTDE rows exist
    if any(r.get("source") == "rtde" for r in rows):
        rows = [r for r in rows if r["source"] == "rtde"]
    if not rows:
        return None
    t = np.array([float(r["t"]) for r in rows])
    xyz = np.array([[float(r["x"]), float(r["y"]), float(r["z"])] for r in rows])
    order = np.argsort(t, kind="stable")
    return t[order], xyz[order]


def with_gaps(t, xyz, gap=GAP):
    """Insert NaN rows between strokes so lines are not drawn across clutch releases."""
    breaks = np.where(np.diff(t) > gap)[0] + 1
    return np.insert(t, breaks, np.nan), np.insert(xyz, breaks, np.nan, axis=0)


def interpolate(t_query, t, xyz):
    """UR position at t_query; NaN outside the logged range."""
    out = np.column_stack([np.interp(t_query, t, xyz[:, i], left=np.nan, right=np.nan) for i in range(3)])
    return out


def best_lag(touch_t, touch_xyz, ur_t, ur_xyz):
    """Delay (s) of the UR behind the Touch target that minimises RMS distance."""
    lags = np.arange(0.0, MAX_LAG, 0.002)
    rms = []
    for lag in lags:
        d = np.linalg.norm(interpolate(touch_t + lag, ur_t, ur_xyz) - touch_xyz, axis=1)
        rms.append(np.sqrt(np.nanmean(d ** 2)) if np.isfinite(d).sum() > 10 else np.inf)
    i = int(np.argmin(rms))
    return lags[i], rms[i], rms[0]


def set_equal_3d(ax, xyz):
    centre = np.nanmean([np.nanmin(xyz, axis=0), np.nanmax(xyz, axis=0)], axis=0)
    half = np.nanmax(np.nanmax(xyz, axis=0) - np.nanmin(xyz, axis=0)) / 2 or 1.0
    ax.set_xlim(centre[0] - half, centre[0] + half)
    ax.set_ylim(centre[1] - half, centre[1] + half)
    ax.set_zlim(centre[2] - half, centre[2] + half)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run", nargs="?", default=DEFAULT_RUN, help="run folder (default logs/NEW_RUN)")
    parser.add_argument("--save", action="store_true", help="save PNGs into the run folder")
    args = parser.parse_args()

    run = os.path.normpath(args.run)
    touch = read_csv(os.path.join(run, "TOUCH_PATH.csv"))
    ur = read_csv(os.path.join(run, "UR_PATH.csv"))
    if ur is None:
        sys.exit(f"{run}/UR_PATH.csv has no rows")
    ur_t, ur_xyz = ur[0], ur[1] * 1000.0       # mm
    name = os.path.basename(run)

    # ---- 3D paths
    fig3d = plt.figure(f"{name} - paths", figsize=(8, 7))
    ax = fig3d.add_subplot(projection="3d")
    ax.plot(*ur_xyz.T, color=UR_COLOR, linewidth=2, label="UR actual (TCP)")
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

    # ---- time series + difference
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
        lag, rms_lagged, rms_raw = best_lag(touch_t, touch_xyz, ur_t, ur_xyz)
        d_raw = np.linalg.norm(interpolate(touch_t, ur_t, ur_xyz) - touch_xyz, axis=1)
        d_lag = np.linalg.norm(interpolate(touch_t + lag, ur_t, ur_xyz) - touch_xyz, axis=1)
        _, g_raw = with_gaps(touch_t, d_raw[:, None])
        _, g_lag = with_gaps(touch_t, d_lag[:, None])
        axes[3].plot(gt, g_raw, color=ERROR_COLOR, linewidth=1.5, label="same time")
        axes[3].plot(gt, g_lag, color=LAGGED_COLOR, linewidth=1.0,
                     label=f"UR shifted back {lag * 1000:.0f} ms")
        axes[3].set_ylabel("distance (mm)")
        axes[3].grid(alpha=0.3)
        axes[3].legend(loc="best")

        offset = np.nanmean(interpolate(touch_t + lag, ur_t, ur_xyz) - touch_xyz, axis=0)
        print(f"{name}: {len(touch_t)} Touch rows, {len(ur_t)} UR rows")
        print(f"  RMS distance, same time:      {rms_raw:7.2f} mm   (max {np.nanmax(d_raw):.2f} mm)")
        print(f"  best lag:                     {lag * 1000:7.0f} ms")
        print(f"  RMS distance, lag removed:    {rms_lagged:7.2f} mm   (max {np.nanmax(d_lag):.2f} mm)")
        print(f"  mean error UR - target (xyz): {offset.round(2)} mm"
              + ("   <- constant offset? check the pendant TCP" if np.linalg.norm(offset) > 5 else ""))
    else:
        print(f"{name}: no Touch target rows (keyboard run, or WHITE never held) - UR path only")
    axes[-1].set_xlabel("time since recording start (s)")
    fig_t.suptitle(f"{name}: position and distance between paths")
    fig_t.tight_layout()

    if args.save:
        fig3d.savefig(os.path.join(run, "paths_3d.png"), dpi=150)
        fig_t.savefig(os.path.join(run, "paths_time.png"), dpi=150)
        print(f"  saved paths_3d.png and paths_time.png in {run}")
    plt.show()


if __name__ == "__main__":
    main()
