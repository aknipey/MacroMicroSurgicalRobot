#!/usr/bin/env python3
"""R03 RCM error budget: controller, kinematic model, latch offset, measurement chain; variance decomposition.

Inputs: folder R02's per-run results (R02_tracked_rcm_error/cache/runs.json, run 09 first), each run's bag
(/joint_states, teleop rcm_point / rcm_active) and META tool geometry, both kinematic models, and
data/recordings/V9b/NDIStaticFloor for the tracker's own floor on the RCM metric.

Budget per run (quadrature, rms):  total^2 = latch^2 + fitted^2   (latch = latched point vs best fixed pivot)
                                   fitted^2 = controller^2 + kinematic^2 + chain^2   (chain = the rest)
Kinematic term: the controller held model A's shaft line through its latched point. With the same joints, model B
places the line elsewhere; the distance of B's line from B's latched point (same depth at the latch) is the RCM
error that the model difference alone produces. For runs on nominal kinematics (10-26), B = calibrated is taken as
the truth, so this is the kinematic term. For calibrated runs the controller matched the arm's own calibration
(checked to 0.000 mm in common/selftest), so their kinematic term is set to 0 and the nominal counterfactual is
reported separately as "what nominal kinematics would have added".
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))  # thesis/results
from common import bags, kin, ndi, runs  # noqa: E402
from common.style import FULL, HALF, INK2, MUTED, save, style, day_label  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

RNG = np.random.default_rng(10)
SETS = ['RCMOnly', 'WholeRunDrinkBottle', 'FastChecking', 'SlowChecking']
SET_LABEL = {'RCMOnly': 'RCM only', 'WholeRunDrinkBottle': 'Whole run', 'FastChecking': 'Fast checking',
             'SlowChecking': 'Slow checking'}


def kinematic_term(run):
    tool = (run.meta.get('tool') or {}).get('at_start') or {}
    s = run.settings
    if 'tip_tool0_mm' in tool:
        tip = np.array(tool['tip_tool0_mm']) / 1000
        dr = np.array(tool['direction_tool0'])
    elif 'TOUCH_RCM_SHAFT_START' in s:            # before the tool library (V7): the run's own RCM shaft settings
        dr = np.array(s['TOUCH_RCM_SHAFT_DIRECTION']); dr = dr / np.linalg.norm(dr)
        tip = np.array(s['TOUCH_RCM_SHAFT_START']) + s['TOUCH_RCM_SHAFT_LENGTH'] * dr
    else:
        return None
    b = bags.load(run, ['/joint_states', 'teleop/rcm_point', 'teleop/rcm_active'])
    if 'teleop/rcm_active' not in b or 'teleop/rcm_point' not in b:
        return None
    act, pt = b['teleop/rcm_active'], b['teleop/rcm_point']
    on = act['tr'][act['data'] > 0.5]
    if not len(on):
        return None
    t0 = on[0]
    # active span: from first True to the next False
    offs = act['tr'][(act['tr'] > t0) & (act['data'] < 0.5)]
    js_end = b['/joint_states']['tr'][-1]
    t1 = offs[0] if len(offs) else js_end            # rcm_active can be published once (latched)
    k = np.searchsorted(pt['tr'], t0)
    c = pt['p'][min(k, len(pt['p']) - 1)]
    js = b['/joint_states']
    m = (js['tr'] >= t0) & (js['tr'] <= t1)
    Q = js['q'][m][::10]
    if len(Q) < 10:
        return None
    model_a = run.kinematics[0] or 'nominal'
    model_b = 'calibrated' if model_a == 'nominal' else 'nominal'
    KA, KB = kin.load(run.kinematics[1] or model_a), kin.load(model_b)

    def line(Kx, q):
        T = kin.tool0(Kx, q)
        return T[:3, :3] @ tip + T[:3, 3], T[:3, :3] @ dr
    pa, da = line(KA, Q[0])
    depth = (c - pa) @ da
    pb, db = line(KB, Q[0])
    cb = pb + depth * db
    e = []
    for q in Q:
        p, d = line(KB, q)
        v = cb - p
        e.append(np.linalg.norm(v - (v @ d) * d))
    e = np.array(e) * 1000
    return dict(model=model_a, other=model_b, rms=float(np.sqrt(np.mean(e ** 2))), n=len(e))


def static_floor():
    r = runs.Run(os.path.join(runs.LOGS, 'V9b', 'NDIStaticFloor'))
    n = ndi.load(r)
    m = n['visible'] == 1
    P = np.c_[n['tip_x'], n['tip_y'], n['tip_z']][m]
    A = np.c_[n['axis_x'], n['axis_y'], n['axis_z']][m]
    A /= np.linalg.norm(A, axis=1, keepdims=True)
    am = A.mean(0); am /= np.linalg.norm(am)
    pm = P.mean(0)
    c_rec = np.array([n['rcm_x'][m][0], n['rcm_y'][m][0], n['rcm_z'][m][0]])
    c = pm + ((c_rec - pm) @ am) * am                     # point on the mean shaft at the recorded RCM depth
    e = ndi.line_dist(P, A, c)
    e_rec = ndi.line_dist(P, A, c_rec)
    return dict(rms=float(np.sqrt(np.mean(e ** 2))), p95=float(np.percentile(e, 95)), n=int(m.sum()),
                rec_rms=float(np.sqrt(np.mean(e_rec ** 2))), dur=float(n['t'][-1] - n['t'][0]))


def ols_ss(y, X):
    beta, *_ = np.linalg.lstsq(X, y, rcond=None)
    r = y - X @ beta
    return float(r @ r)


def design(rows, terms):
    cols = [np.ones(len(rows))]
    for term in terms:
        if ':' in term:
            a, b = term.split(':')
            levels = sorted({(r[a], r[b]) for r in rows})[1:]
            cols += [np.array([float((r[a], r[b]) == l) for r in rows]) for l in levels]
        else:
            levels = sorted({r[term] for r in rows})[1:]
            cols += [np.array([float(r[term] == l) for r in rows]) for l in levels]
    return np.column_stack(cols)


def eta2(rows, factors=('set', 'day', 'mode'), inter=('set:day',)):
    y = np.log([r['rms'] for r in rows])
    full = list(factors) + list(inter)
    ss_res = ols_ss(y, design(rows, full))
    out = {}
    for f in full:
        if ':' in f:
            reduced = list(factors)
            ss_f = ols_ss(y, design(rows, reduced)) - ss_res
        else:
            reduced = [x for x in factors if x != f]
            ss_f = ols_ss(y, design(rows, reduced)) - ols_ss(y, design(rows, list(factors)))
        out[f] = ss_f / (ss_f + ss_res) if ss_f + ss_res > 0 else np.nan
    out['residual (replays)'] = ss_res
    return out


def main():
    style()
    data = json.load(open(os.path.join(runs.folder('R02'), 'cache', 'runs.json')))
    rows = [o for o in data if not o['excluded']]
    for o in rows:
        o['kmodel'] = o.pop('kin')
        r = runs.Run(os.path.join(runs.LOGS, o['key']))
        kt = kinematic_term(r)
        o['kin_cf'] = kt['rms'] if kt else np.nan
        o['kin'] = 0.0      # NOT separated: see the note in RESULTS.md (same tool through both models is invalid)
        o['latch'] = float(np.sqrt(max(o['rms'] ** 2 - o['rms_fitted'] ** 2, 0)))
        o['chain'] = float(np.sqrt(max(o['rms_fitted'] ** 2 - (o['ctrl_mm'] or 0) ** 2 - o['kin'] ** 2, 0)))
        print(o['key'], f"total {o['rms']:.2f} ctrl {o['ctrl_mm']:.2f} kin {o['kin']:.2f} (cf {o['kin_cf']:.2f}) "
              f"latch {o['latch']:.2f} chain {o['chain']:.2f}")
    floor = static_floor()

    L = ['# R03 Results: RCM error budget and variance decomposition', '',
         'Generated by `analysis.py` from folder R02\'s per-run results plus bags, META tool geometry and both kinematic '
         'models. mm rms unless stated. Budget in quadrature: total² = latch² + fitted²; fitted² = controller² + '
         'kinematic² + chain².', '',
         f"**Tracker floor on the RCM metric** (`V9b/NDIStaticFloor`, arm still, {floor['dur']:.0f} s, {floor['n']} "
         f"samples): {floor['rms']:.3f} mm rms, p95 {floor['p95']:.3f} mm, about a point on the mean shaft. The "
         f"recorder's {floor['rec_rms']:.2f} mm for this run is distance to an older latched point (a constant offset, "
         'not noise).', '', '## Budget per set and condition (median over replays)', '',
         '| Set | day · kinematics | total | controller | latch offset | rest (kinematics + measurement chain) | '
         'model difference, same tool (INVALID, see note) |', '|---|---|---|---|---|---|---|']
    groups = sorted({(o['set'], o['day'], o['kmodel']) for o in rows}, key=lambda g: (SETS.index(g[0]), g[1]))
    for s, d, k in groups:
        v = [o for o in rows if (o['set'], o['day'], o['kmodel']) == (s, d, k)]
        g = lambda key: np.nanmedian([o[key] for o in v])
        L.append(f"| {SET_LABEL[s]} | {d} · {k} | {g('rms'):.2f} | {g('ctrl_mm'):.2f} | "
                 f"{g('latch'):.2f} | {g('chain'):.2f} | {g('kin_cf'):.2f} |")
    L += ['', '**Note: the kinematic term is not separated here.** Pushing the same tool geometry through the nominal and '
          'the calibrated model gives a "model difference" of 0.6–1.0 mm, larger than the *total* measured error on '
          'some runs (e.g. RCM only, 10-26), which is impossible for a real error component. The tool (hand–eye) '
          'calibration was itself fitted through one model, so swapping the model without refitting the tool creates '
          'an artificial difference. A valid kinematic term needs the tool refitted per model from the raw hand–eye '
          'recordings (`src/ndi_tracking/calibrations/raw/robot_tcp_*.npz`, joints recovered by IK), as planned in '
          'ANALYSIS.md; until then kinematics and the measurement chain are reported together as "rest".']
    # variance decomposition on the calibrated days (balanced on 11-03; 11-06 bridges)
    cal = [o for o in rows if o['kmodel'] == 'calibrated']
    for o in cal:
        o['mode_'] = o['mode'].replace('_optimised', '').replace('_unfiltered', '')
    e = eta2(cal, factors=('set', 'day', 'mode'), inter=('set:day',))
    boots = {k: [] for k in e}
    cells = {}
    for o in cal:
        cells.setdefault((o['set'], o['day'], o['mode']), []).append(o)
    for _ in range(500):
        smp = [RNG.choice(v) for v in cells.values() for _ in v]
        try:
            eb = eta2(smp)
        except Exception:
            continue
        for k in e:
            boots[k].append(eb[k])
    L += ['', '## Variance decomposition of log(NDI rms), calibrated days (11-03 and 11-06)', '',
          f'{len(cal)} runs. Partial η² per factor (type II, OLS on log rms), 95 % bootstrap interval over replays '
          'within cells.', '', '| Factor | partial η² | 95 % interval |', '|---|---|---|']
    for k in ('set', 'day', 'mode', 'set:day'):
        lo, hi = np.nanpercentile(boots[k], [2.5, 97.5])
        L.append(f'| {k} | {e[k]:.3f} | {lo:.3f}–{hi:.3f} |')
    allc = [o for o in rows]
    L += ['', '## Summary', '',
          f"- Controller's own error {np.nanmin([o['ctrl_mm'] for o in allc]):.2f}–{np.nanmax([o['ctrl_mm'] for o in allc]):.2f} mm; "
          f"tracker floor {floor['rms']:.3f} mm.",
          "- Kinematic term: not separated (see note); E0's 0.1–0.35 mm is untested here.",
          f"- Latch offset: median {np.median([o['latch'] for o in allc]):.2f} mm, max {max(o['latch'] for o in allc):.2f} mm.",
          '', '## Prior claims checked', '',
          '- "kinematics explain only 0.1–0.35 mm (E0)": see the kinematic and counterfactual columns.',
          '- "a ~2.5 mm latch-pose offset dominates SlowChecking (10-26)": see the latch column.',
          '- "trajectory and camera dominate, mode is negligible": see the η² table.', '', '## Figures', '']

    # fig 1: budget bars by variance share
    fig, ax = plt.subplots(figsize=(FULL * 0.8, 0.3 * len(groups) + 0.9))
    cols = [('Controller', '#0072B2'), ('Latch offset', '#E69F00'), ('Kinematics and measurement', '#8a8983')]
    for i, (s, d, k) in enumerate(groups):
        v = [o for o in rows if (o['set'], o['day'], o['kmodel']) == (s, d, k)]
        g = lambda key: np.nanmedian([o[key] for o in v])
        comps = [g('ctrl_mm') ** 2, g('latch') ** 2, g('chain') ** 2]
        tot = np.sqrt(sum(comps))
        x = 0
        for (lab, c), v2 in zip(cols, comps):
            w = tot * v2 / sum(comps)
            ax.barh(i, w, left=x, color=c, height=0.65, edgecolor='white', lw=0.8, label=lab if i == 0 else None)
            x += w
        ax.text(tot + 0.03, i, f'{g("rms"):.2f}', va='center', fontsize=8, color=INK2,
                bbox=dict(fc='white', ec='none', pad=0.5))
    ax.axvline(1.0, color=INK2, ls='--', lw=0.8)
    ax.axvspan(0, floor['rms'], color=MUTED, alpha=0.3, lw=0, label='Tracker floor')
    ax.set_yticks(range(len(groups)))
    ax.set_yticklabels([f"{SET_LABEL[s]}, {day_label(d)} ({k})" for s, d, k in groups])
    ax.invert_yaxis(); ax.set_xlabel('RCM error, rms (mm)')
    ax.legend(ncol=2, loc='lower left', bbox_to_anchor=(-0.45, 1.0))
    L.append(f"- `figures/{os.path.basename(save(fig, HERE, 'error_budget'))}`")

    # fig 2: eta^2 dot + interval
    fig, ax = plt.subplots(figsize=(HALF * 1.2, 1.9))
    for i, k in enumerate(('set', 'day', 'set:day', 'mode')):
        lo, hi = np.nanpercentile(boots[k], [2.5, 97.5])
        ax.plot([lo, hi], [i, i], color=INK2, lw=1.2)
        ax.plot(e[k], i, 'o', color='#0072B2', ms=5)
    ax.set_yticks(range(4)); ax.set_yticklabels(['Trajectory', 'Day (camera)', 'Trajectory × day', 'Output mode'])
    ax.invert_yaxis(); ax.set_xlim(0, 1); ax.set_xlabel(r'Partial $\eta^2$ of log RCM error')
    L.append(f"- `figures/{os.path.basename(save(fig, HERE, 'variance_shares'))}`")
    open(os.path.join(HERE, 'RESULTS.md'), 'w').write('\n'.join(L) + '\n')
    print('\n'.join(L))


if __name__ == '__main__':
    main()
