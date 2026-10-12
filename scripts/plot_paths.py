#!/usr/bin/env python3
"""Plot the Touch target path against the UR's actual tool0 path for one logged run.

    python3 scripts/plot_paths.py                      # logs/NEW_RUN, translation
    python3 scripts/plot_paths.py logs/sine_baseline_1 # a renamed run
    python3 scripts/plot_paths.py logs/NEW_RUN -a      # angles only
    python3 scripts/plot_paths.py logs/NEW_RUN -t -a   # translation and angles
    python3 scripts/plot_paths.py logs/NEW_RUN --save  # also write PNGs into the run folder

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
"""

import argparse
import csv
import os
import sys

import numpy as np
import matplotlib.pyplot as plt
import yaml
from scipy.spatial.transform import Rotation, Slerp

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


def plot_error(ax, touch_t, gt, error_at, unit):
    """Error at the same time and with the best lag removed; returns (lag, rms_lagged, rms_raw, d_raw, d_lag)."""
    lag, rms_lagged, rms_raw = best_lag(error_at)
    d_raw, d_lag = error_at(0.0), error_at(lag)
    ax.plot(gt, with_gaps(touch_t, d_raw[:, None])[1], color=ERROR_COLOR, linewidth=1.5, label="same time")
    ax.plot(gt, with_gaps(touch_t, d_lag[:, None])[1], color=LAGGED_COLOR, linewidth=1.0,
            label=f"UR shifted back {lag * 1000:.0f} ms")
    ax.set_ylabel(f"error ({unit})")
    ax.grid(alpha=0.3)
    ax.legend(loc="best")
    return lag, rms_lagged, rms_raw, d_raw, d_lag


def plot_translation(name, ur_t, ur_xyz, touch):
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
        lag, rms_lagged, rms_raw, d_raw, d_lag = plot_error(axes[3], touch_t, gt, error_at, "mm")
        offset = np.nanmean(interpolate(touch_t + lag, ur_t, ur_xyz) - touch_xyz, axis=0)
        print("  translation")
        print(f"    RMS distance, same time:      {rms_raw:7.2f} mm   (max {np.nanmax(d_raw):.2f} mm)")
        print(f"    best lag:                     {lag * 1000:7.0f} ms")
        print(f"    RMS distance, lag removed:    {rms_lagged:7.2f} mm   (max {np.nanmax(d_lag):.2f} mm)")
        print(f"    mean error UR - target (xyz): {offset.round(2)} mm"
              + ("   <- constant offset? check the pendant TCP" if np.linalg.norm(offset) > 5 else ""))
    axes[-1].set_xlabel("time since recording start (s)")
    fig_t.suptitle(f"{name}: position and distance between paths")
    fig_t.tight_layout()
    return {"paths_3d": fig3d, "paths_time": fig_t}


def plot_angles(name, ur_t, ur_quat, touch):
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
        lag, rms_lagged, rms_raw, d_raw, d_lag = plot_error(axes[3], touch_t, gt, error_at, "deg")
        print("  angles")
        print(f"    RMS angle error, same time:   {rms_raw:7.2f} deg  (max {np.nanmax(d_raw):.2f} deg)")
        print(f"    best lag:                     {lag * 1000:7.0f} ms")
        print(f"    RMS angle error, lag removed: {rms_lagged:7.2f} deg  (max {np.nanmax(d_lag):.2f} deg)")
    axes[-1].set_xlabel("time since recording start (s)")
    fig.suptitle(f"{name}: tool rotation since start (base_link) and angle between target and robot")
    fig.tight_layout()
    return {"angles_time": fig}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run", nargs="?", default=DEFAULT_RUN, help="run folder (default logs/NEW_RUN)")
    parser.add_argument("-t", "--translation", action="store_true", help="plot translation (default)")
    parser.add_argument("-a", "--angles", action="store_true", help="plot angles")
    parser.add_argument("--save", action="store_true", help="save PNGs into the run folder")
    args = parser.parse_args()
    if not args.angles:
        args.translation = True

    run = os.path.normpath(args.run)
    touch = read_csv(os.path.join(run, "TOUCH_PATH.csv"))
    ur = read_csv(os.path.join(run, "UR_PATH.csv"))
    if ur is None:
        sys.exit(f"{run}/UR_PATH.csv has no rows")
    name = os.path.basename(run)
    if touch is not None:
        print(f"{name}: {len(touch[0])} Touch rows, {len(ur[0])} UR rows")
    else:
        print(f"{name}: no Touch target rows (keyboard run, or WHITE never held) - UR path only")
    ur_t, ur_xyz, ur_quat = ur[:3]
    if touch is not None and ur_is_pendant_tcp(run, ur):
        ur_xyz, ur_quat = tcp_to_tool0(ur, touch)

    figures = {}
    if args.translation:
        figures.update(plot_translation(name, ur_t, ur_xyz, touch))
    if args.angles:
        figures.update(plot_angles(name, ur_t, ur_quat, touch))

    if args.save:
        for file, fig in figures.items():
            fig.savefig(os.path.join(run, f"{file}.png"), dpi=150)
        print(f"  saved {', '.join(f + '.png' for f in figures)} in {run}")
    plt.show()


if __name__ == "__main__":
    main()
