#!/usr/bin/env python3
"""05 Same-day output-mode benchmark (logs/V9b/NoTool). Writes figures/ and RESULTS.md here.

    cd thesis/data_analysis && source /opt/ros/jazzy/setup.bash && python3 -m 05_output_mode_benchmark.analysis
(or: python3 05_output_mode_benchmark/analysis.py from thesis/data_analysis)
"""
import os
import sys

import numpy as np
from scipy import stats

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from common import runs, track  # noqa: E402
from common.style import FULL, HALF, INK2, MODE, MUTED, save, style  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

GAP_EXCLUDE_S = 0.05        # a gap in the recorded arm path longer than this inside a stroke excludes the run
ORDER = ['trajectory', 'position', 'velocity_optimised', 'velocity_unfiltered']
TRAJ_ORDER = ['LongBenchmark', 'ShortBenchmark', 'NoBuzzArea', 'LargeVibrations', 'Scaled5_Fast', 'Scaled5_Slow']


def collect():
    rs = runs.find_runs('V9b/NoTool/*/*_Replay*')
    rows, excluded = [], []
    for r in rs:
        m = track.cached_metrics(r)
        if m is None:
            excluded.append((r.key, 'no target/arm path'))
            continue
        big = [(a, g) for a, g in m['path_gaps'] if g > GAP_EXCLUDE_S]
        row = dict(run=r, mode=r.mode, traj=r.trajectory, rep=r.replay_n, scale=r.scale, code=r.code_version,
                   kin=r.kinematics[0], **m)
        if big:
            excluded.append((r.key, 'gap in recorded arm path: ' + ', '.join(f'{g * 1e3:.0f} ms at t = {a:.2f} s'
                                                                             for a, g in big)))
            row['excluded'] = True
        rows.append(row)
    return rows, excluded


def per_traj(rows, key):
    """{(traj, mode): mean over included replays}"""
    out = {}
    for t in TRAJ_ORDER:
        for mo in ORDER:
            v = [r[key] for r in rows if r['traj'] == t and r['mode'] == mo and not r.get('excluded')]
            if v:
                out[(t, mo)] = (np.mean(v), v)
    return out


def paired(pt, a, b):
    ts = [t for t in TRAJ_ORDER if (t, a) in pt and (t, b) in pt]
    d = np.array([pt[(t, a)][0] - pt[(t, b)][0] for t in ts])
    ratio = np.array([pt[(t, a)][0] / pt[(t, b)][0] for t in ts])
    if len(d) >= 2:
        ci = stats.t.interval(0.95, len(d) - 1, d.mean(), stats.sem(d))
    else:
        ci = (np.nan, np.nan)
    w = stats.wilcoxon(d).pvalue if len(d) >= 5 and np.any(d != 0) else np.nan
    return ts, d, ci, w, ratio


def fig_pareto(rows):
    fig, ax = plt.subplots(figsize=(FULL * 0.85, 3.0))
    inc = [r for r in rows if not r.get('excluded')]
    for t in TRAJ_ORDER:       # thin grey line joining each trajectory's mode means
        pts = []
        for mo in ORDER:
            v = [r for r in inc if r['traj'] == t and r['mode'] == mo]
            if v:
                pts.append((np.mean([r['lag_ms'] for r in v]), np.mean([r['jitter_ratio'] for r in v])))
        if len(pts) > 1:
            x, y = zip(*sorted(pts))
            ax.plot(x, y, color=MUTED, lw=0.6, zorder=1)
    for mo in ORDER:
        v = [r for r in inc if r['mode'] == mo]
        st = MODE[mo]
        ax.scatter([r['lag_ms'] for r in v], [r['jitter_ratio'] for r in v], s=26, color=st['color'],
                   marker=st['marker'], edgecolor='white', linewidth=0.6, label=st['label'], zorder=3)
    ax.axhline(1.0, color=INK2, lw=0.8, ls='--')
    ax.text(252, 1.02, 'Arm as rough as the hand', fontsize=8, color=INK2, va='bottom', ha='right')
    ax.axvspan(0, 50, color=MUTED, alpha=0.12, lw=0)
    ax.text(48, 0.05, r'$\leq$ 50 ms', fontsize=8, color=INK2, ha='right')
    ax.set_xlim(0, 260)
    ax.set_ylim(0, 2.3)
    ax.text(150, 1.75, '1:5 slow trajectory', fontsize=8, color=INK2)
    ax.set_xlabel('Translation lag (ms)')
    ax.set_ylabel('Jitter ratio, arm to input (> 5 Hz)')
    ax.legend(loc='lower left', ncol=4, bbox_to_anchor=(0, 1.0), handletextpad=0.2, columnspacing=1.0)
    return save(fig, HERE, 'pareto_lag_vs_smoothness')


def fig_freq_lag(rows):
    fig, ax = plt.subplots(figsize=(HALF * 1.4, 2.6))
    for mo in ORDER:
        curves = []
        for r in rows:
            if r['mode'] != mo or r.get('excluded') or r['scale'] != 1.0:
                continue
            res = track.freq_lag(r['run'])
            if res is not None:
                f, lag, coh = res
                curves.append(lag)
        if not curves:
            continue
        C = np.array(curves)
        med = np.nanmedian(C, 0)
        lo, hi = np.nanpercentile(C, 25, 0), np.nanpercentile(C, 75, 0)
        st = MODE[mo]
        ax.plot(f, med, color=st['color'], label=st['label'])
        ax.fill_between(f, lo, hi, color=st['color'], alpha=0.18, lw=0)
    ax.set_xscale('log')
    ax.set_xlabel('Frequency of the hand motion (Hz)')
    ax.set_ylabel('Lag (ms)')
    ax.set_ylim(0, 300)
    ax.legend(loc='upper right')
    return save(fig, HERE, 'lag_vs_frequency')


def fig_paired(pt, comps):
    fig, axes = plt.subplots(1, len(comps), figsize=(FULL, 2.2), sharey=True)
    for ax, (a, b, title) in zip(axes, comps):
        ts, d, ci, w, _ = paired(pt, a, b)
        y = np.arange(len(ts))
        ax.scatter(d, y, color=MODE[a]['color'], marker=MODE[a]['marker'], s=22, zorder=3)
        ax.axvline(0, color=INK2, lw=0.8)
        if np.isfinite(ci[0]):
            ax.errorbar(d.mean(), -1, xerr=[[d.mean() - ci[0]], [ci[1] - d.mean()]], fmt='o', color='k', ms=4,
                        capsize=3)
        ax.set_yticks(list(y) + [-1])
        ax.set_yticklabels(ts + ['mean, 95 % CI'])
        ax.set_title(title, fontsize=8)
        ax.set_xlabel('Lag difference (ms)')
    return save(fig, HERE, 'paired_lag_differences')


def main():
    style()
    rows, excluded = collect()
    pt = per_traj(rows, 'lag_ms')
    ptr = per_traj(rows, 'lag_rot_ms')
    ptv = per_traj(rows, 'lag_vel_ms')
    L = ['# 05 Results: same-day output-mode benchmark', '',
         'Generated by `analysis.py` (do not edit by hand). Metrics: `common/track.py` (validated in '
         '`common/selftest.py`: synthetic delays recovered to 0.1 ms).', '']
    L += ['## Runs', '', f'{len(rows)} replays found in `logs/V9b/NoTool`; modes from META: '
          + ', '.join(f"{mo} {sum(r['mode'] == mo for r in rows)}" for mo in ORDER) + '.',
          f"Kinematics: {sorted(set(r['kin'] for r in rows))}; code versions per mode: "
          + '; '.join(f"{mo} {sorted(set(r['code'] for r in rows if r['mode'] == mo))}" for mo in ORDER), '']
    L += ['**Excluded (found by the gap check, > %.0f ms inside a stroke):**' % (GAP_EXCLUDE_S * 1e3), '']
    L += [f'- `{k}`: {why}' for k, why in excluded] or ['- none']
    L += ['', '## Lag per trajectory (ms; translation position-shift / velocity-xcorr / rotation; replays)', '',
          '| Trajectory | scale | ' + ' | '.join(MODE[m]['label'] for m in ORDER) + ' |',
          '|---|---|' + '---|' * len(ORDER)]
    for t in TRAJ_ORDER:
        sc = next((r['scale'] for r in rows if r['traj'] == t), None)
        cells = []
        for mo in ORDER:
            v = [r for r in rows if r['traj'] == t and r['mode'] == mo and not r.get('excluded')]
            if not v:
                cells.append('–')
                continue
            f = lambda k: '/'.join(f"{r[k]:.0f}" for r in v)
            cells.append(f"{f('lag_ms')} · {f('lag_vel_ms')} · {f('lag_rot_ms')}")
        L.append(f'| {t} | 1:{1 / sc:.0f} | ' + ' | '.join(cells) + ' |')
    L += ['', '## Tracking error and smoothness (mean of replays)', '',
          '| Trajectory | mode | lag-removed error (mm) | rotation error (°) | jitter arm (mm) | input (mm) | ratio |',
          '|---|---|---|---|---|---|---|']
    for t in TRAJ_ORDER:
        for mo in ORDER:
            v = [r for r in rows if r['traj'] == t and r['mode'] == mo and not r.get('excluded')]
            if v:
                g = lambda k: np.mean([r[k] for r in v])
                L.append(f"| {t} | {MODE[mo]['label']} | {g('err_mm'):.2f} | {g('err_rot_deg'):.2f} | "
                         f"{g('jitter_arm_mm'):.3f} | {g('jitter_tgt_mm'):.3f} | {g('jitter_ratio'):.2f} |")
    comps = [('trajectory', 'position', 'Trajectory − position'),
             ('trajectory', 'velocity_optimised', 'Trajectory − velocity (filtered)'),
             ('velocity_optimised', 'position', 'Velocity (filtered) − position'),
             ('position', 'velocity_unfiltered', 'Position − velocity (unfiltered), 1:5')]
    L += ['', '## Paired comparisons (unit = trajectory, mean of its replays)', '',
          '| Comparison | metric | n | per-trajectory differences | mean [95 % CI] | Wilcoxon p | ratio range |',
          '|---|---|---|---|---|---|---|']
    for a, b, title in comps:
        for name, tab in (('translation', pt), ('velocity xcorr', ptv), ('rotation', ptr)):
            ts, d, ci, w, ratio = paired(tab, a, b)
            if len(d) == 0:
                continue
            L.append(f"| {title} | {name} | {len(d)} | {', '.join(f'{x:.0f}' for x in d)} | {d.mean():.0f} "
                     f"[{ci[0]:.0f}, {ci[1]:.0f}] | {w:.3f} | {ratio.min():.2f}–{ratio.max():.2f} |")
    # ranges quoted in the report
    def rng(mo, scale, key='lag_ms'):
        v = [r[key] for r in rows if r['mode'] == mo and r['scale'] == scale and not r.get('excluded')]
        return (min(v), max(v)) if v else (np.nan, np.nan)
    L += ['', '## Ranges (included replays)', '']
    for mo in ORDER:
        for sc in (1.0, 0.2):
            a, b = rng(mo, sc)
            ra, rb = rng(mo, sc, 'lag_rot_ms')
            if np.isfinite(a):
                L.append(f"- {MODE[mo]['label']}, 1:{1 / sc:.0f}: translation {a:.0f}–{b:.0f} ms, "
                         f"rotation {ra:.0f}–{rb:.0f} ms")
    L += ['', '## Prior claims checked', '']
    pa, pb = rng('position', 1.0)
    va, vb = rng('velocity_optimised', 1.0)
    ta, tb = rng('trajectory', 1.0)
    L.append(f'- "1:1 lag 66–78 ms (position, filtered velocity)": measured position {pa:.0f}–{pb:.0f}, '
             f'velocity {va:.0f}–{vb:.0f} ms.')
    L.append(f'- "trajectory 170–248 ms at 1:1": measured {ta:.0f}–{tb:.0f} ms.')
    ua, ub = rng('velocity_unfiltered', 0.2)
    fa, fb = min(rng('position', 0.2)[0], rng('velocity_optimised', 0.2)[0]), max(rng('position', 0.2)[1],
                                                                                  rng('velocity_optimised', 0.2)[1])
    L.append(f'- "1:5 unfiltered 46–48 ms vs filtered 52–62 ms": measured unfiltered {ua:.0f}–{ub:.0f}, filtered '
             f'{fa:.0f}–{fb:.0f} ms.')
    L.append('- "334 ms gap in VelocityOptimised/ShortBenchmark_Replay2": see the exclusion list above.')
    L += ['', '## Figures', '']
    for p in (fig_pareto(rows), fig_freq_lag(rows), fig_paired(pt, comps[:3])):
        L.append(f'- `figures/{os.path.basename(p)}` (and .pdf)')
    open(os.path.join(HERE, 'RESULTS.md'), 'w').write('\n'.join(L) + '\n')
    print('\n'.join(L))


if __name__ == '__main__':
    main()
