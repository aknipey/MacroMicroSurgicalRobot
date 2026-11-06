#!/usr/bin/env python3
"""NDI-measured RCM comparisons across conditions (kinematics x output mode) and trajectory sets.

    cd ~/ros2_ws/src/MacroMicroSurgicalRobot
    python3 thesis/plotting/rcm_plots.py              # all figures
    python3 thesis/plotting/rcm_plots.py target ecdf  # some of them

rcm_replay_strip   every replay's rms as a dot, per set and condition: the summary table, with repeatability
rcm_target         where the shaft crosses the RCM plane (a target), sets x conditions, coloured by time
rcm_bias_spread    rms split into bias (latched point off the real pivot) and spread (scatter about it)
rcm_ecdf           share of time within X mm, per set: reads "within 1 mm / 2 mm" straight off
rcm_timeline       error vs time since the latch, median and range of the replays
rcm_tilt           error vs shaft tilt from the latch direction: does error grow with swing?
"""
import sys

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.patches import Circle

from common import (CONDITIONS, EXCLUDE, GRID, INK, INK2, MUTED, SETS, all_runs, bias_spread, exclusion_note,
                    legend_handles, pierce_points, rms, save, style, tilt_deg)

# single-hue sequential ramp (reference palette blue 150 -> 700) for time
TIME_CMAP = LinearSegmentedColormap.from_list('time', ['#b7d3f6', '#5598e7', '#256abf', '#0d366b'])
SET_NAMES = list(SETS.values())


def by_set(runs):
    return {s: [r for r in runs if r.set == s] for s in SET_NAMES}


def replay_strip(data):
    fig, ax = plt.subplots(figsize=(7.5, 3.4))
    conds = list(CONDITIONS)
    width = 0.8 / len(conds)
    used = []
    for i, s in enumerate(SET_NAMES):
        for j, c in enumerate(conds):
            x0 = i + (j - (len(conds) - 1) / 2) * width
            vals = [r.ndi_meta['rcm_segments'][0]['error_rms_mm'] if len(r.ndi_meta['rcm_segments']) == 1
                    else max(r.ndi_meta['rcm_segments'], key=lambda g: g['samples'])['error_rms_mm']
                    for r in by_set(data[c])[s]]
            used += [r.key for r in by_set(data[c])[s]]
            if not vals:
                continue
            jitter = np.linspace(-width * 0.25, width * 0.25, len(vals))
            ax.scatter(x0 + jitter, vals, s=22, color=CONDITIONS[c]['color'], marker=CONDITIONS[c]['marker'],
                       edgecolor='white', linewidth=0.6, zorder=3)
            ax.hlines(np.median(vals), x0 - width * 0.4, x0 + width * 0.4, color=INK, lw=1.0, zorder=4)
    # excluded runs, hollow, so they are seen but not counted
    for key, why in EXCLUDE.items():
        from common import Run
        r = Run(f'{__import__("common").LOGS}/{key}')
        c = next(k for k, v in CONDITIONS.items() if key.startswith(v['folder'] + '/'))
        j = conds.index(c)
        x0 = SET_NAMES.index(r.set) + (j - (len(conds) - 1) / 2) * width
        val = max(r.ndi_meta['rcm_segments'], key=lambda g: g['samples'])['error_rms_mm']
        ax.scatter([x0], [min(val, 3.4)], s=22, facecolor='none', edgecolor=CONDITIONS[c]['color'],
                   marker=CONDITIONS[c]['marker'], zorder=3)
        ax.annotate(f'{val:.1f} mm, excluded', (x0, min(val, 3.4)), xytext=(6, -2), textcoords='offset points',
                    fontsize=6.5, color=MUTED)
    ax.axhline(1.0, color=MUTED, lw=0.8, ls='--')
    ax.text(len(SET_NAMES) - 0.55, 1.02, 'target (iii): 1 mm', fontsize=7, color=MUTED, ha='right', va='bottom')
    ax.set_xticks(range(len(SET_NAMES)), SET_NAMES)
    ax.grid(axis='x', visible=False)
    ax.set_ylim(0, 3.5)
    ax.set_ylabel('NDI RCM error, rms (mm)')
    ax.set_title('Every replay, every condition (bar = median)')
    ax.legend(handles=legend_handles(conds), loc='upper center', bbox_to_anchor=(0.5, -0.1), ncol=4, fontsize=7)
    save(fig, 'rcm_replay_strip', note='Hollow = excluded: ' + '; '.join(f'{k.split("/")[-1]}: {v}'
                                                                           for k, v in EXCLUDE.items()))


def target(data):
    conds = list(CONDITIONS)
    fig, axes = plt.subplots(len(SET_NAMES), len(conds), figsize=(1.9 * len(conds) + 0.6, 1.9 * len(SET_NAMES)),
                             sharex=True, sharey=True)
    lim = 4.0
    used = []
    for i, s in enumerate(SET_NAMES):
        for j, c in enumerate(conds):
            ax = axes[i, j]
            runs = by_set(data[c])[s]
            used += [r.key for r in runs]
            for radius in (1, 2, 3):
                ax.add_patch(Circle((0, 0), radius, fill=False, color=MUTED if radius == 1 else GRID,
                                    lw=0.8 if radius == 1 else 0.6, ls='-' if radius == 1 else '--', zorder=1))
            all_xy = []
            for r in runs:
                xy = pierce_points(r.ndi)
                tt = r.ndi['t'] / r.ndi['t'][-1]
                ax.scatter(xy[::3, 0], xy[::3, 1], c=tt[::3], cmap=TIME_CMAP, s=1.2, lw=0, zorder=2, rasterized=True)
                all_xy.append(xy)
            if all_xy:
                xy = np.concatenate(all_xy)
                b, sp = bias_spread(xy)
                m = np.nanmean(xy, 0)
                ax.plot([0, m[0]], [0, m[1]], color=INK, lw=1.0, zorder=4)
                ax.plot(*m, marker='+', color=INK, ms=7, mew=1.2, zorder=4)
                stats = f'rms {rms(np.linalg.norm(xy, axis=1)):.2f}  bias {b:.2f}  spr {sp:.2f}'
            else:
                stats = 'no runs'
            head = CONDITIONS[c]['label'].replace(' kin.,', '\n') + '\n' if i == 0 else ''
            ax.set_title(head + stats, fontsize=6.5, color=INK2, pad=3)
            ax.plot(0, 0, marker='o', color=CONDITIONS[c]['color'], ms=4, zorder=5)
            ax.set_xlim(-lim, lim)
            ax.set_ylim(-lim, lim)
            ax.set_aspect('equal')
            ax.grid(False)
            if j == 0:
                ax.set_ylabel(f'{s}\nv (mm)', fontsize=8)
            if i == len(SET_NAMES) - 1:
                ax.set_xlabel('u (mm)', fontsize=8)
    sm = plt.cm.ScalarMappable(cmap=TIME_CMAP)
    cb = fig.colorbar(sm, ax=axes, shrink=0.35, pad=0.02)
    cb.set_label('time in RCM segment (start -> end)', fontsize=7)
    cb.set_ticks([0, 1], labels=['latch', 'end'])
    fig.suptitle('Where the shaft crosses the RCM plane (dot = latched point; + = mean; rings 1/2/3 mm)',
                 x=0.01, ha='left', fontsize=9)
    note = ('Plane through the NDI-latched point, normal to the mean shaft; u along camera +y. All replays of a cell '
            'overlaid. bias = how far the mean crossing is from the latched point; spread = scatter about the mean; '
            'rms^2 = bias^2 + spread^2. ' + (exclusion_note(EXCLUDE) or ''))
    save(fig, 'rcm_target', note=note)


def bias_spread_plane(data):
    fig, axes = plt.subplots(1, len(SET_NAMES), figsize=(9, 2.5), sharex=True, sharey=True)
    th = np.linspace(0, np.pi / 2, 100)
    for ax, s in zip(axes, SET_NAMES):
        for r_ in (0.5, 1.0, 1.5, 2.0, 2.5, 3.0):
            ax.plot(r_ * np.cos(th), r_ * np.sin(th), color=MUTED if r_ == 1.0 else GRID,
                    lw=1.0 if r_ == 1.0 else 0.7, ls='--' if r_ == 1.0 else '-', zorder=1)
            ax.text(r_ + 0.03, 0.03, f'{r_:g}', fontsize=6, color=MUTED, ha='left')
        for c, runs in data.items():
            pts = np.array([bias_spread(pierce_points(r.ndi)) for r in by_set(runs)[s]])
            if len(pts):
                ax.scatter(pts[:, 0], pts[:, 1], s=24, color=CONDITIONS[c]['color'], marker=CONDITIONS[c]['marker'],
                           edgecolor='white', linewidth=0.6, zorder=3)
        ax.set_title(s)
        ax.set_aspect('equal')
        ax.set_xlim(0, 3.1)
        ax.set_ylim(0, 1.5)
        ax.set_xlabel('bias (mm)')
    axes[0].set_ylabel('spread (mm)')
    fig.legend(handles=legend_handles(list(CONDITIONS)), loc='lower center', ncol=4, bbox_to_anchor=(0.5, -0.1))
    fig.suptitle('Is the RCM error an offset or a wobble? (arcs = total rms; dashed = 1 mm)', x=0.01, ha='left',
                 fontsize=9)
    fig.tight_layout()
    save(fig, 'rcm_bias_spread', note='One marker per replay. bias = mean shaft crossing vs the latched point (the '
         'latch was taken somewhere the arm then left: E0); spread = scatter about that mean (wobble). '
         'rms^2 = bias^2 + spread^2.')


def ecdf(data):
    fig, axes = plt.subplots(1, len(SET_NAMES), figsize=(9, 2.8), sharey=True)
    for ax, s in zip(axes, SET_NAMES):
        for c, runs in data.items():
            err = np.concatenate([r.ndi['error'] for r in by_set(runs)[s]] or [np.array([])])
            err = np.sort(err[np.isfinite(err)])
            if not len(err):
                continue
            y = np.arange(1, len(err) + 1) / len(err)
            ax.plot(err, 100 * y, color=CONDITIONS[c]['color'], lw=1.6)
            k = len(err) // 2
            ax.plot(err[k], 50, marker=CONDITIONS[c]['marker'], color=CONDITIONS[c]['color'], ms=4.5,
                    mec='white', mew=0.6)
        for x in (1, 2):
            ax.axvline(x, color=MUTED, lw=0.8, ls='--')
        ax.set_xlim(0, 4.5)
        ax.set_title(s)
        ax.set_xlabel('RCM error (mm)')
    axes[0].set_ylabel('% of RCM time within')
    axes[0].set_ylim(0, 100)
    fig.legend(handles=legend_handles(list(CONDITIONS)), loc='lower center', ncol=4, bbox_to_anchor=(0.5, -0.12))
    fig.suptitle('Share of time the shaft stays within X mm of the RCM point (marker = median; replays pooled)',
                 x=0.01, ha='left', fontsize=9)
    fig.tight_layout()
    save(fig, 'rcm_ecdf', note=exclusion_note(EXCLUDE))


def smooth(y, n):
    if n < 2:
        return y
    k = np.ones(n) / n
    return np.convolve(np.nan_to_num(y, nan=np.nanmean(y)), k, mode='same')


def timeline(data):
    fig, axes = plt.subplots(len(SET_NAMES), 1, figsize=(8, 1.8 * len(SET_NAMES)), sharex=False)
    for ax, s in zip(axes, SET_NAMES):
        tmax = 0
        for c, runs in data.items():
            rs = by_set(runs)[s]
            if not rs:
                continue
            end = min(r.ndi['t'][-1] for r in rs)
            grid = np.arange(0, end, 1 / 30)
            curves = np.array([smooth(np.interp(grid, r.ndi['t'], r.ndi['error']), 9) for r in rs])
            ax.fill_between(grid, curves.min(0), curves.max(0), color=CONDITIONS[c]['color'], alpha=0.18, lw=0)
            ax.plot(grid, np.median(curves, 0), color=CONDITIONS[c]['color'], lw=1.2)
            tmax = max(tmax, end)
        ax.axhline(1.0, color=MUTED, lw=0.7, ls='--')
        ax.set_xlim(0, tmax)
        ax.set_ylim(0, None)
        ax.set_ylabel(f'{s}\nerror (mm)', fontsize=8)
    axes[-1].set_xlabel('time since the RCM latch (s)')
    axes[0].legend(handles=legend_handles(list(CONDITIONS)), ncol=4, loc='lower left', bbox_to_anchor=(0, 1.02), fontsize=6.8)
    fig.suptitle('RCM error over the run: median of the replays (line) and their range (band)', x=0.01, ha='left',
                 fontsize=9)
    fig.tight_layout()
    save(fig, 'rcm_timeline', note='0.3 s moving average. Replays of one set share the stylus input, so their '
         'curves line up in time. ' + (exclusion_note(EXCLUDE) or ''))


def tilt(data):
    fig, axes = plt.subplots(1, len(SET_NAMES), figsize=(9, 2.8), sharey=True)
    for ax, s in zip(axes, SET_NAMES):
        for c, runs in data.items():
            rs = by_set(runs)[s]
            if not rs:
                continue
            a = np.concatenate([tilt_deg(r.ndi) for r in rs])
            e = np.concatenate([r.ndi['error'] for r in rs])
            edges = np.arange(0, np.nanpercentile(a, 99) + 2.5, 2.5)
            mid, med, q1, q3 = [], [], [], []
            for lo, hi in zip(edges[:-1], edges[1:]):
                sel = (a >= lo) & (a < hi) & np.isfinite(e)
                if sel.sum() >= 30:
                    mid.append((lo + hi) / 2)
                    med.append(np.median(e[sel]))
                    q1.append(np.percentile(e[sel], 25))
                    q3.append(np.percentile(e[sel], 75))
            ax.fill_between(mid, q1, q3, color=CONDITIONS[c]['color'], alpha=0.15, lw=0)
            ax.plot(mid, med, color=CONDITIONS[c]['color'], marker=CONDITIONS[c]['marker'], ms=3, lw=1.3)
        ax.axhline(1.0, color=MUTED, lw=0.7, ls='--')
        ax.set_title(s)
        ax.set_xlabel('shaft tilt from latch (deg)')
    axes[0].set_ylabel('RCM error (mm), median & IQR')
    axes[0].set_ylim(0, None)
    fig.legend(handles=legend_handles(list(CONDITIONS)), loc='lower center', ncol=4, bbox_to_anchor=(0.5, -0.12))
    fig.suptitle('Does the error grow as the shaft swings away from where it was latched?', x=0.01, ha='left',
                 fontsize=9)
    fig.tight_layout()
    save(fig, 'rcm_tilt', note='2.5 deg bins with >= 30 samples. Rising = pose-dependent error (model, '
         'measurement chain); flat = noise or controller. ' + (exclusion_note(EXCLUDE) or ''))


FIGURES = {'strip': replay_strip, 'target': target, 'bias': bias_spread_plane, 'ecdf': ecdf,
           'timeline': timeline, 'tilt': tilt}


def main(names):
    style()
    data = all_runs()
    print('runs: ' + ', '.join(f'{c} {len(v)}' for c, v in data.items()))
    for n in names or FIGURES:
        FIGURES[n](data)


if __name__ == '__main__':
    main(sys.argv[1:])
