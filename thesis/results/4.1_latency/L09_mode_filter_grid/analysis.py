#!/usr/bin/env python3
"""L09 Session D (V9c) output mode x command filter grid, bare flange, 1:5 (Scaled5_Fast, Scaled5_Slow).

Cells (data/recordings/V9c/NoTool/<cell>): P5 V5 P10 V10 P20 V20 Poff Voff (P = position output, V = velocity output, stock
speedj in every cell; 'off' = no command filter and no Kalman), P5_end (the P5 anchor repeated last: drift), and
P20_1to1 (the 1:1 inputs at position 20 Hz, against session C's position mode).

Metrics: lag / error / jitter from common/track.py (same as folder L08; a gap > 50 ms inside a stroke excludes a
replay, as in 05). Tremor band: folder L07's method (tool linear velocity from joint velocity and the calibrated
kinematics, moving windows from the operator's input, 3 Hz zero-phase detrend, Welch 1 s), band power 15-25 Hz
against Voff (unfiltered velocity, session C's baseline) and against Poff. Model predictions are read from folder
L06's RESULTS.md. Run from thesis/results with ROS sourced:  python3 4.1_latency/L09_mode_filter_grid/analysis.py
"""
import importlib.util
import os
import re
import sys

import numpy as np
from scipy import stats

HERE = os.path.dirname(os.path.abspath(__file__))
DA = os.path.dirname(os.path.dirname(HERE))  # thesis/results
sys.path.insert(0, DA)
from common import kin, runs, track  # noqa: E402
from common.style import BLUE, FULL, GREY, INK2, ORANGE, panel, save, style  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

_spec = importlib.util.spec_from_file_location('v01', os.path.join(DA, '4.1_latency', 'L07_vibration_reduction', 'analysis.py'))
v01 = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(v01)

GAP_EXCLUDE_S = 0.05
GRID = ['P5', 'V5', 'P10', 'V10', 'P20', 'V20', 'Poff', 'Voff']
CUT = {'5': 5.0, '10': 10.0, '20': 20.0, 'off': 0.0}
INPUTS = ['Scaled5_Fast', 'Scaled5_Slow']
ONE_TO_ONE = ['LongBenchmark', 'ShortBenchmark', 'NoBuzzArea', 'LargeVibrations']
LAG_REQ_MS = 50.0
VIB_REQ = v01.REQ          # 51 % power
RNG = np.random.default_rng(1)


def cell_of(run):
    return run.path.rstrip('/').split('/')[-2]


def out_cut(cell):
    m = re.match(r'([PV])(5|10|20|off)', cell)
    return ('position' if m.group(1) == 'P' else 'velocity'), CUT[m.group(2)]


def model_predictions():
    """Folder L06: closed-loop added lag per f_c (position plant), and the f_c 20 Hz / Kp 8 total lag and reduction."""
    txt = open(os.path.join(runs.folder('L06'), 'RESULTS.md')).read()
    added = {float(a): float(c) for a, b, c in re.findall(r'^\| (\d+) \| (\d+) \| ([\d.]+) \| \d+ % \|$', txt, re.M)}
    m = re.search(r'f_c 20 Hz / Kp 8 \(([\d.]+) ms, (\d+) %\)', txt)
    return added, (float(m.group(1)), float(m.group(2)) / 100) if m else (None, None)


def collect():
    rows, notes = [], []
    for r in runs.find_runs('V9c/NoTool/*/*_Replay*'):
        if r.path.endswith('_NO_NDI'):
            continue
        m = track.cached_metrics(r)
        if m is None:
            notes.append((r.key, 'no target/arm path'))
            continue
        # Session D rule: any gap > 50 ms in the arm path or the replayed stylus excludes the replay. A ~335 ms stall
        # before the motion still corrupted P20/Scaled5_Fast_Replay2 (237 ms lag), so folder L08's in-stroke rule is
        # not enough here. TOUCH_PATH is not checked: its breaks are the input's own button releases.
        gmax = max(1e3 * np.diff(r.csv(f)['t']).max() for f in ('UR_PATH.csv', 'STYLUS_RAW.csv'))
        big = [(a, g) for a, g in m['path_gaps'] if g > GAP_EXCLUDE_S]
        if not big and gmax > GAP_EXCLUDE_S * 1e3:
            big = [(float('nan'), gmax / 1e3)]
        row = dict(run=r, key=r.key, cell=cell_of(r), traj=r.trajectory, rep=r.replay_n, code=r.code_version,
                   gap_ms=gmax, **m)
        if big:
            row['excluded'] = True
            notes.append((r.key, 'excluded: gap ' + ', '.join(f'{g * 1e3:.0f} ms' for a, g in big)))
        rows.append(row)
    return rows, notes


def vals(rows, cell, traj, key):
    return [r[key] for r in rows if r['cell'] == cell and r['traj'] == traj and not r.get('excluded')]


def msd(v, fmt='{:.1f}'):
    if not v:
        return '–'
    return (fmt.format(np.mean(v)) + (' ± ' + fmt.format(np.std(v, ddof=1)) if len(v) > 1 else '') + f' ({len(v)})')


def vibration(rows):
    """15-25 Hz tool-linear band power per included 1:5 replay (folder L07 method), pooled per cell and input."""
    K = kin.load('calibrated')
    rs = [r['run'] for r in rows if r['traj'] in INPUTS and r['cell'] in GRID + ['P5_end'] and not r.get('excluded')]
    thr = {tr: v01.speed_threshold([r for r in rs if cell_of(r) == 'Voff' and r.trajectory == tr])[0]
           for tr in INPUTS if any(cell_of(r) == 'Voff' and r.trajectory == tr for r in rs)}
    sig = {}
    for r in rs:
        try:
            sig[r.key] = v01.signals(r, K)
        except Exception as exc:  # noqa: BLE001
            print(f'  {r.key}: no bag signals ({exc})')
    span = {}
    for tr in thr:
        g0 = [sig[r.key][0] for r in rs if r.trajectory == tr and r.key in sig]
        span[tr] = (max(g[0] for g in g0) + 0.5, min(g[-1] for g in g0) - 0.5)
    segs = {}
    for r in rs:
        if r.key not in sig or r.trajectory not in thr:
            continue
        lo, hi = span[r.trajectory]
        wins = [(max(a, lo), min(b, hi)) for a, b in v01.moving_windows(r, thr[r.trajectory])]
        wins = [(a, b) for a, b in wins if b - a >= 1.0]
        g, q, qd, _, _ = sig[r.key]
        s = v01.segments(g, v01.to_tool(K, q, qd)[:, :3], wins)
        if s:
            f, P = v01.psd_of(s)
            segs.setdefault((cell_of(r), r.trajectory), []).append(v01.band(f, P, 15, 25))
    return {k: np.concatenate(v) for k, v in segs.items()}, thr


def reduction(bp, cell, base, traj):
    a, b = bp.get((cell, traj)), bp.get((base, traj))
    if a is None or b is None:
        return None
    red = 1 - a.mean() / b.mean()
    boot = [1 - RNG.choice(a, len(a)).mean() / RNG.choice(b, len(b)).mean() for _ in range(2000)]
    return red, np.percentile(boot, [2.5, 97.5])


def main():
    style()
    rows, notes = collect()
    have = sorted({r['cell'] for r in rows})
    added_model, (lag20_model, red20_model) = model_predictions()
    L = ['# L09 Results: session D output mode × command filter grid (V9c, bare flange)', '',
         'Generated by `analysis.py` (do not edit by hand). Metrics: `common/track.py` (as folder L08) and folder L07\'s '
         'tremor-band method. Cells found: ' + ', '.join(have) + '.', '',
         f'{len(rows)} replays; {sum(1 for r in rows if r.get("excluded"))} excluded (gap > 50 ms in the arm path or the replayed stylus).',
         'Code versions: ' + '; '.join(f'{c} {sorted({r["code"] for r in rows if r["cell"] == c})}' for c in have), '']
    if notes:
        L += ['**Gaps:**', ''] + [f'- `{k}`: {n}' for k, n in notes] + ['']

    # --- lag table
    L += ['## Lag, error and jitter per cell (mean ± sd (n) of included replays)', '',
          '| Cell | output | f_c | input | lag, position (ms) | lag, velocity xcorr (ms) | lag, rotation (ms) | '
          'lag-removed error (mm) | jitter ratio |', '|---|---|---|---|---|---|---|---|---|']
    for c in [x for x in GRID + ['P5_end'] if x in have]:
        o, fc = out_cut(c.replace('_end', ''))
        for tr in INPUTS:
            L.append(f'| {c} | {o} | {fc:g} | {tr} | {msd(vals(rows, c, tr, "lag_ms"))} | '
                     f'{msd(vals(rows, c, tr, "lag_vel_ms"))} | {msd(vals(rows, c, tr, "lag_rot_ms"))} | '
                     f'{msd(vals(rows, c, tr, "err_mm"), "{:.2f}")} | {msd(vals(rows, c, tr, "jitter_ratio"), "{:.2f}")} |')

    # --- drift
    if 'P5_end' in have and 'P5' in have:
        L += ['', '## Drift over the session: P5 (first) vs P5_end (last), same settings', '',
              '| Input | lag P5 | lag P5_end | difference (ms) | Welch t p |', '|---|---|---|---|---|']
        for tr in INPUTS:
            a, b = vals(rows, 'P5', tr, 'lag_ms'), vals(rows, 'P5_end', tr, 'lag_ms')
            if a and b:
                p = stats.ttest_ind(a, b, equal_var=False).pvalue if len(a) > 1 and len(b) > 1 else np.nan
                L.append(f'| {tr} | {np.mean(a):.1f} | {np.mean(b):.1f} | {np.mean(b) - np.mean(a):+.1f} | {p:.2f} |')

    # --- factorial effects
    L += ['', '## Effects (difference of cell means, ms; position lag)', '',
          '| Comparison | input | difference | Welch t p |', '|---|---|---|---|']
    comps = [(f'V{k} − P{k} (output, f_c {k})', f'V{k}', f'P{k}') for k in ('5', '10', '20', 'off')]
    comps += [(f'P{k} − Poff (filter, position)', f'P{k}', 'Poff') for k in ('5', '10', '20')]
    comps += [(f'V{k} − Voff (filter, velocity)', f'V{k}', 'Voff') for k in ('5', '10', '20')]
    for lab, a_, b_ in comps:
        for tr in INPUTS:
            a, b = vals(rows, a_, tr, 'lag_ms'), vals(rows, b_, tr, 'lag_ms')
            if a and b:
                p = stats.ttest_ind(a, b, equal_var=False).pvalue if len(a) > 1 and len(b) > 1 else np.nan
                L.append(f'| {lab} | {tr} | {np.mean(a) - np.mean(b):+.1f} | {p:.3f} |')

    # --- model check: filter-added lag, position
    L += ['', '## Filter-added lag against the folder L06 model (position plant)', '',
          '| f_c (Hz) | input | measured P(f_c) − Poff (ms) | model, closed loop (ms) |', '|---|---|---|---|']
    for k in ('5', '10', '20'):
        for tr in INPUTS:
            a, b = vals(rows, f'P{k}', tr, 'lag_ms'), vals(rows, 'Poff', tr, 'lag_ms')
            if a and b:
                mdl = added_model.get(float(k))
                L.append(f'| {k} | {tr} | {np.mean(a) - np.mean(b):.1f} | {mdl if mdl is not None else "–"} |')

    import json
    meas = {tr: {k: float(np.mean(vals(rows, f'P{k}', tr, 'lag_ms')) - np.mean(vals(rows, 'Poff', tr, 'lag_ms')))
                 for k in ('5', '10', '20') if vals(rows, f'P{k}', tr, 'lag_ms') and vals(rows, 'Poff', tr, 'lag_ms')}
            for tr in INPUTS}
    json.dump(meas, open(os.path.join(HERE, 'measured_filter_lag.json'), 'w'), indent=1)

    # --- tremor band
    bp, thr = ({}, {})
    if 'Voff' in have:
        bp, thr = vibration(rows)
        L += ['', '## 15–25 Hz tool linear velocity power: reduction (1 − P_cell / P_base, 95 % bootstrap CI)', '',
              'Moving threshold from the Voff replays: ' + ', '.join(f'{k} > {v:.2f} mm/s' for k, v in thr.items()), '',
              '| Cell | input | vs Voff (unfiltered velocity) | vs Poff |', '|---|---|---|---|']
        for c in [x for x in GRID + ['P5_end'] if x in have]:
            for tr in INPUTS:
                cells = []
                for base in ('Voff', 'Poff'):
                    rr = reduction(bp, c, base, tr) if c != base else None
                    cells.append(f'{100 * rr[0]:.0f} % [{100 * rr[1][0]:.0f}, {100 * rr[1][1]:.0f}]' if rr else '–')
                L.append(f'| {c} | {tr} | {cells[0]} | {cells[1]} |')

    # --- verdict on the 20 Hz prediction (threshold: lag <= 50 ms AND >= 51 % below unfiltered velocity)
    L += ['', '## The 20 Hz test (planning/09 T1): lag ≤ 50 ms and ≥ 51 % 15–25 Hz power below Voff, at 1:5', '',
          f'Model (folder L06): {lag20_model} ms, {100 * red20_model:.0f} % reduction.' if lag20_model else '', '',
          '| Input | lag P20 (ms) | lag ≤ 50 | reduction vs Voff | ≥ 51 % | both |', '|---|---|---|---|---|---|']
    for tr in INPUTS:
        lag = vals(rows, 'P20', tr, 'lag_ms')
        rr = reduction(bp, 'P20', 'Voff', tr) if bp else None
        if lag:
            ok1 = np.mean(lag) <= LAG_REQ_MS
            ok2 = rr is not None and rr[0] >= VIB_REQ
            L.append(f'| {tr} | {np.mean(lag):.1f} | {"yes" if ok1 else "no"} | '
                     f'{f"{100 * rr[0]:.0f} %" if rr else "–"} | {"yes" if ok2 else ("no" if rr else "–")} | '
                     f'{"MET" if ok1 and ok2 else "not met"} |')

    # --- 1:1 at 20 Hz vs session C position (5 Hz)
    if 'P20_1to1' in have:
        L += ['', '## 1:1 inputs at position 20 Hz (P20_1to1) against session C position 5 Hz (V9b)', '',
              '| Input | lag P20 (ms) | lag V9b position (ms) | jitter ratio P20 | jitter ratio V9b |', '|---|---|---|---|---|']
        c_rows = []
        for r in runs.find_runs('V9b/NoTool/PositionMode/*_Replay*'):
            m = track.cached_metrics(r)
            if m:
                c_rows.append(dict(traj=r.trajectory, **m))
        for tr in ONE_TO_ONE:
            a = vals(rows, 'P20_1to1', tr, 'lag_ms')
            b = [r['lag_ms'] for r in c_rows if r['traj'] == tr]
            ja = vals(rows, 'P20_1to1', tr, 'jitter_ratio')
            jb = [r['jitter_ratio'] for r in c_rows if r['traj'] == tr]
            L.append(f'| {tr} | {msd(a)} | {msd(b)} | {msd(ja, "{:.2f}")} | {msd(jb, "{:.2f}")} |')

    # --- figure: lag and tremor reduction against cutoff, both outputs
    fig, axes = plt.subplots(1, 2, figsize=(FULL, 2.6))
    xs = {'5': 5, '10': 10, '20': 20, 'off': 40}
    for o, col, mk in (('P', BLUE, '^'), ('V', ORANGE, 'o')):
        for tr, ls in (('Scaled5_Fast', '-'), ('Scaled5_Slow', '--')):
            pts = [(xs[k], np.mean(vals(rows, f'{o}{k}', tr, 'lag_ms'))) for k in xs if vals(rows, f'{o}{k}', tr, 'lag_ms')]
            if pts:
                axes[0].plot(*zip(*pts), ls=ls, marker=mk, color=col, ms=5,
                             label=f'{"Position" if o == "P" else "Velocity"}, {tr.replace("Scaled5_", "1:5 ").lower()}')
            rp = [(xs[k], 100 * reduction(bp, f'{o}{k}', 'Voff', tr)[0]) for k in xs
                  if bp and f'{o}{k}' != 'Voff' and reduction(bp, f'{o}{k}', 'Voff', tr)]
            if rp:
                axes[1].plot(*zip(*rp), ls=ls, marker=mk, color=col, ms=5)
    axes[0].axhline(LAG_REQ_MS, color=INK2, ls=':', lw=0.9)
    axes[1].axhline(100 * VIB_REQ, color=INK2, ls=':', lw=0.9)
    if lag20_model:
        axes[0].plot([20], [lag20_model], 'x', color=GREY, ms=7, mew=1.5, label='Model, 20 Hz')
    for ax in axes:
        ax.set_xscale('log')
        ax.set_xticks(list(xs.values()))
        ax.set_xticklabels(['5', '10', '20', 'off'])
        ax.minorticks_off()
        ax.set_xlabel('Command filter cutoff (Hz)')
    axes[0].set_ylabel('Translation lag (ms)')
    axes[1].set_ylabel('15–25 Hz power below\nunfiltered velocity (%)')
    panel(axes[0], 'a', 'Lag')
    panel(axes[1], 'b', '15–25 Hz reduction')
    fig.legend(*axes[0].get_legend_handles_labels(), loc='lower center', bbox_to_anchor=(0.5, 1.0), ncol=3)
    save(fig, HERE, 'grid_lag_and_vibration')

    open(os.path.join(HERE, 'RESULTS.md'), 'w').write('\n'.join(L) + '\n')
    print('\n'.join(L))


if __name__ == '__main__':
    main()
