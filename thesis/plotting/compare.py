#!/usr/bin/env python3
"""One-page comparison sheet for any NDI runs (or folders of runs), e.g. a new replay against the reference.

    cd ~/ros2_ws/src/MacroMicroSurgicalRobot
    python3 thesis/plotting/compare.py logs/V7/SlowCheckingRCMSimpleHolder_Replay1 logs/V9/PositionMode/SlowCheckingRCMSimpleHolder_Replay1
    python3 thesis/plotting/compare.py logs/V8/CalibratedKinematics/RCMOnly* --name rcmonly_v8
    python3 thesis/plotting/compare.py logs/V9/PositionMode logs/V9/TrajectoryMode --group   # one colour per folder

Writes out/compare_<name>.png/.pdf:
  top row     a target (shaft crossings of the RCM plane) per run or group
  bottom      error vs time since the latch (overlaid) | share of time within X mm | a numbers table
Up to 8 runs/groups (the categorical palette's length); more should be split, not recoloured.
"""
import argparse
import glob
import os

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.gridspec import GridSpec
from matplotlib.patches import Circle

from common import GRID, INK, INK2, LOGS, MUTED, Run, bias_spread, pierce_points, rms, save, style

PALETTE = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4', '#008300', '#6250d6', '#e34948']
MARKERS = ['o', 's', '^', 'D', 'v', 'P', 'X', 'h']


def expand(path):
    path = os.path.abspath(path)
    if os.path.exists(os.path.join(path, 'NDI_META.yaml')):
        return [path]
    return sorted(os.path.dirname(p) for p in glob.glob(os.path.join(path, '*', 'NDI_META.yaml')))


def short_labels(names):
    """Path parts that differ between entries (shared ones dropped), 'SimpleHolder' trimmed."""
    parts = [n.replace('SimpleHolder', '').replace('CalibratedKinematics', 'CalKin').split('/') for n in names]
    width = max(len(p) for p in parts)
    parts = [[''] * (width - len(p)) + p for p in parts]
    keep = [k for k in range(width) if len({p[k] for p in parts}) > 1] or [width - 1]
    return ['/'.join(p[k] for k in keep if p[k]) for p in parts]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('paths', nargs='+', help='run folders, or folders holding runs')
    ap.add_argument('--group', action='store_true', help='one entry per given folder (its runs pooled)')
    ap.add_argument('--name', help='output name (default: from the first entry)')
    args = ap.parse_args()

    if args.group:
        entries = [(os.path.relpath(os.path.abspath(p), LOGS), [Run(r) for r in expand(p)]) for p in args.paths]
    else:
        entries = [(os.path.relpath(r, LOGS), [Run(r)]) for p in args.paths for r in expand(p)]
    entries = [(n, [r for r in rs if r.has_ndi]) for n, rs in entries]
    entries = [(n, rs) for n, rs in entries if rs]
    if not entries:
        raise SystemExit('no runs with NDI RCM segments')
    if len(entries) > len(PALETTE):
        raise SystemExit(f'{len(entries)} entries: at most {len(PALETTE)} per sheet (split them, or use --group)')

    style()
    n = len(entries)
    fig = plt.figure(figsize=(max(9, 2.1 * n), 6.4))
    gs = GridSpec(2, max(n, 4), height_ratios=[1, 1.15], hspace=0.45, wspace=0.4, figure=fig)
    lim = 4.0
    stats = []
    labels = short_labels([name for name, _ in entries])
    for i, (name, runs) in enumerate(entries):
        color, marker = PALETTE[i], MARKERS[i]
        ax = fig.add_subplot(gs[0, i])
        for radius in (1, 2, 3):
            ax.add_patch(Circle((0, 0), radius, fill=False, color=MUTED if radius == 1 else GRID,
                                lw=0.8, ls='-' if radius == 1 else '--'))
        xy = np.concatenate([pierce_points(r.ndi) for r in runs])
        ax.scatter(xy[::2, 0], xy[::2, 1], s=1.2, color=color, alpha=0.35, lw=0, rasterized=True)
        m = np.nanmean(xy, 0)
        ax.plot([0, m[0]], [0, m[1]], color=INK, lw=1)
        ax.plot(*m, marker='+', color=INK, ms=7, mew=1.2)
        ax.plot(0, 0, marker=marker, color=color, ms=5, mec='white')
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        ax.set_aspect('equal')
        ax.grid(False)
        ax.set_title(f'{i + 1}. {labels[i][:30]}', fontsize=7, color=INK2)
        err = np.concatenate([r.ndi['error'] for r in runs])
        b, sp = bias_spread(xy)
        swing = np.median([r.ndi['seg'].get('shaft_swing_deg', np.nan) for r in runs])
        stats.append((i, name, len(runs), rms(err), np.nanpercentile(err, 95), 100 * np.nanmean(err <= 1),
                      100 * np.nanmean(err <= 2), b, sp, swing))

    ax_t = fig.add_subplot(gs[1, 0])
    ax_c = fig.add_subplot(gs[1, 1])
    for i, (name, runs) in enumerate(entries):
        for r in runs:
            k = np.ones(9) / 9
            ax_t.plot(r.ndi['t'], np.convolve(np.nan_to_num(r.ndi['error']), k, mode='same'), color=PALETTE[i],
                      lw=1.0, alpha=0.85)
        err = np.sort(np.concatenate([r.ndi['error'] for r in runs]))
        err = err[np.isfinite(err)]
        ax_c.plot(err, 100 * np.arange(1, len(err) + 1) / len(err), color=PALETTE[i], lw=1.6)
        ax_c.plot(err[len(err) // 2], 50, marker=MARKERS[i], color=PALETTE[i], ms=5, mec='white')
    ax_t.axhline(1, color=MUTED, lw=0.7, ls='--')
    ax_t.set_xlabel('time since latch (s)')
    ax_t.set_ylabel('RCM error (mm)')
    ax_t.set_title('error over time')
    ax_t.set_ylim(bottom=0)
    for x in (1, 2):
        ax_c.axvline(x, color=MUTED, lw=0.7, ls='--')
    ax_c.set_xlim(0, None)
    ax_c.set_ylim(0, 100)
    ax_c.set_xlabel('RCM error (mm)')
    ax_c.set_ylabel('% of time within')
    ax_c.set_title('time within X mm')

    ax_n = fig.add_subplot(gs[1, 2:])
    ax_n.axis('off')
    cols = ['#', 'runs', 'rms', 'p95', '≤1', '≤2', 'bias', 'spr.', 'swing']
    cells = [[str(i + 1), str(k), f'{a:.2f}', f'{p:.2f}', f'{w1:.0f}%', f'{w2:.0f}%', f'{b:.2f}', f'{s:.2f}',
              f'{sw:.0f}'] for i, _, k, a, p, w1, w2, b, s, sw in stats]
    table = ax_n.table(cellText=cells, colLabels=cols, loc='upper left', cellLoc='center', colLoc='center')
    table.auto_set_font_size(False)
    table.set_fontsize(7)
    table.scale(1, 1.25)
    for (row, col), cell in table.get_celld().items():
        cell.set_edgecolor(GRID)
        if row == 0:
            cell.set_text_props(color=INK2, weight='bold')
        elif col == 0:
            cell.set_text_props(color=PALETTE[row - 1], weight='bold')
    ax_n.set_title('mm (≤1/≤2: % of time; swing: deg)', loc='left')

    fig.suptitle('RCM comparison: ' + ' vs '.join(f'{i + 1}' for i in range(n)) + '  (rings 1/2/3 mm; '
                 '+ = mean crossing; marker = latched point)', x=0.01, ha='left', fontsize=9)
    legend = ';  '.join(f'{i + 1} = {name}' for i, (name, _) in enumerate(entries))
    out = args.name or entries[0][0].replace('/', '_')
    save(fig, f'compare_{out}', note=legend)


if __name__ == '__main__':
    main()
