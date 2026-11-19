#!/usr/bin/env python3
"""Teleop tracking (Touch target vs robot tool0) compared across conditions, from the same replays.

    cd ~/ros2_ws/src/MacroMicroSurgicalRobot
    python3 thesis/earlier/plotting/tracking_plots.py

Metrics are thesis/run_tools/plot_paths.py's own (compute_metrics), computed in memory and cached in
out/cache/tracking.json; nothing is written into logs/ (plot_paths.py itself writes METRICS.yaml there).

tracking_tradeoff   lag vs high-frequency jitter per replay, per set: the speed/smoothness trade-off per mode
tracking_profile    each condition's profile (lag, shape error, jitter, rotation lag, RCM rms) as a dot matrix
"""
import json
import os
import sys

import matplotlib.pyplot as plt
import numpy as np

from common import CONDITIONS, GRID, INK, INK2, MUTED, OUT, PKG, SETS, all_runs, legend_handles, save, style

sys.path.insert(0, os.path.join(PKG, 'thesis', 'run_tools'))
import plot_paths  # noqa: E402

SET_NAMES = list(SETS.values())
CACHE = os.path.join(OUT, 'cache', 'tracking.json')


def metrics_for(run):
    """plot_paths' translation/angle metrics for a run (cached on the run's file times)."""
    os.makedirs(os.path.dirname(CACHE), exist_ok=True)
    cache = json.load(open(CACHE)) if os.path.exists(CACHE) else {}
    stamp = max(os.path.getmtime(os.path.join(run.path, f)) for f in ('UR_PATH.csv', 'TOUCH_PATH.csv'))
    hit = cache.get(run.key)
    if hit and hit['stamp'] == stamp:
        return hit['metrics']
    touch = plot_paths.read_csv(os.path.join(run.path, 'TOUCH_PATH.csv'))
    ur = plot_paths.read_csv(os.path.join(run.path, 'UR_PATH.csv'))
    m = plot_paths.compute_metrics(ur[0], ur[1], ur[2], touch, plot_paths.JITTER_HZ) if ur and touch else None
    cache[run.key] = {'stamp': stamp, 'metrics': m}
    json.dump(cache, open(CACHE, 'w'))
    return m


def collect(data):
    rows = []
    for c, runs in data.items():
        for r in runs:
            m = metrics_for(r)
            if not m:
                continue
            seg = max(r.ndi_meta['rcm_segments'], key=lambda g: g['samples'])
            rows.append(dict(condition=c, set=r.set, run=r.key,
                             lag=m['translation']['lag_ms'], jitter=m['translation']['jitter_ur_mm'],
                             jitter_target=m['translation']['jitter_target_mm'],
                             shape=m['translation']['rms_lag_removed_mm'], rms=m['translation']['rms_mm'],
                             ang_lag=m['angles']['lag_ms'], ang_jitter=m['angles']['jitter_ur_deg'],
                             rcm=seg['error_rms_mm']))
    return rows


def tradeoff(rows):
    fig, axes = plt.subplots(1, len(SET_NAMES), figsize=(9, 2.9), sharey=True)
    for ax, s in zip(axes, SET_NAMES):
        sel = [r for r in rows if r['set'] == s and r['lag'] is not None]
        for c in CONDITIONS:
            pts = np.array([(r['lag'], r['jitter']) for r in sel if r['condition'] == c], float)
            if len(pts):
                ax.scatter(pts[:, 0], pts[:, 1], s=26, color=CONDITIONS[c]['color'], marker=CONDITIONS[c]['marker'],
                           edgecolor='white', linewidth=0.6, zorder=3)
        target = [r['jitter_target'] for r in sel if r['jitter_target'] is not None]
        if target:   # the input's own jitter: what an ideal follower would reproduce
            ax.axhline(np.median(target), color=MUTED, lw=0.8, ls='--')
            ax.text(ax.get_xlim()[1] if False else 0.98, np.median(target), 'stylus input', fontsize=6.5,
                    color=MUTED, ha='right', va='bottom', transform=ax.get_yaxis_transform())
        ax.set_title(s)
        ax.set_xlabel('lag (ms)')
        ax.set_xlim(left=0)
    axes[0].set_ylabel('robot jitter > 5 Hz (mm rms)')
    axes[0].set_ylim(bottom=0)
    fig.legend(handles=legend_handles(list(CONDITIONS)), loc='lower center', ncol=4, bbox_to_anchor=(0.5, -0.1))
    fig.suptitle('Speed vs smoothness per output mode: lower-left is better (one marker per replay)', x=0.01,
                 ha='left', fontsize=9)
    fig.tight_layout()
    save(fig, 'tracking_tradeoff', note='lag = shift that best aligns tool0 with the Touch target; jitter = rms of '
         'tool0 motion above 5 Hz while engaged (thesis/run_tools/plot_paths.py definitions). Dashed: the target\'s own jitter.')


def profile(rows):
    """Dot matrix: rows = metrics, columns = sets; within a cell one dot per condition (median of replays).

    Each metric row is scaled to its own range so rows are comparable at a glance; the value is printed."""
    metrics = [('lag', 'lag (ms)', '{:.0f}'), ('shape', 'shape error, lag removed (mm)', '{:.1f}'),
               ('jitter', 'jitter > 5 Hz (mm)', '{:.2f}'), ('ang_lag', 'rotation lag (ms)', '{:.0f}'),
               ('rcm', 'NDI RCM rms (mm)', '{:.2f}')]
    conds = list(CONDITIONS)
    fig, ax = plt.subplots(figsize=(9, 3.6))
    for i, (key, label, fmt) in enumerate(metrics):
        vals = {}
        for j, s in enumerate(SET_NAMES):
            for c in conds:
                v = [r[key] for r in rows if r['set'] == s and r['condition'] == c and r[key] is not None]
                if v:
                    vals[(j, c)] = float(np.median(v))
        if not vals:
            continue
        lo, hi = min(vals.values()), max(vals.values())
        for (j, c), v in vals.items():
            k = conds.index(c)
            x = j + (k - 1.5) * 0.2
            size = 20 + 200 * (v - lo) / (hi - lo + 1e-9)       # bigger = worse
            ax.scatter(x, -i, s=size, color=CONDITIONS[c]['color'], marker=CONDITIONS[c]['marker'],
                       edgecolor='white', linewidth=0.6, zorder=3)
            ax.text(x, -i - 0.32, fmt.format(v), fontsize=5.8, color=INK2, ha='center', va='top')
    ax.set_yticks([-i for i in range(len(metrics))], [m[1] for m in metrics])
    ax.set_xticks(range(len(SET_NAMES)), SET_NAMES)
    ax.xaxis.tick_top()
    ax.set_xlim(-0.6, len(SET_NAMES) - 0.4)
    ax.set_ylim(-len(metrics) + 0.4, 0.6)
    ax.grid(False)
    for j in range(len(SET_NAMES) - 1):
        ax.axvline(j + 0.5, color=GRID, lw=0.8)
    for spine in ax.spines.values():
        spine.set_visible(False)
    ax.tick_params(length=0)
    ax.legend(handles=legend_handles(conds), loc='upper center', bbox_to_anchor=(0.5, -0.02), ncol=4)
    fig.suptitle('Condition profile (median of replays; marker size = worse, scaled per row)', x=0.01, ha='left',
                 fontsize=9, y=1.04)
    save(fig, 'tracking_profile')


def main():
    style()
    rows = collect(all_runs())
    print(f'{len(rows)} runs with tracking metrics')
    tradeoff(rows)
    profile(rows)


if __name__ == '__main__':
    main()
