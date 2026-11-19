#!/usr/bin/env python3
"""R02 RCM error measured independently with the NDI tracker, for every RCM replay (V7, V8, V9, V9b).

Per run: RCM error recomputed from the tracked shaft line (common.ndi; cross-checked against the recorder's
column), crossing points, error about the fitted pivot, the controller's own error from the bag, and a
camera-moved check (rigid fit of FK tip -> NDI tip; a step in the residual = the camera or wand moved).
Writes figures/, RESULTS.md and cache/runs.json (consumed by folder R03).
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))  # thesis/results
from common import bags, kin, ndi, runs  # noqa: E402
from common.style import FULL, INK2, MODE, MUTED, save, style, day_label  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

SETS = ['RCMOnly', 'WholeRunDrinkBottle', 'FastChecking', 'SlowChecking']
SET_LABEL = {'RCMOnly': 'Phantom RCM', 'WholeRunDrinkBottle': 'Phantom task', 'FastChecking': 'Fast probing',
             'SlowChecking': 'Slow probing'}
PATTERNS = ['V7/*SimpleHolder_Replay*', 'V8/CalibratedKinematics/*SimpleHolder_Replay*',
            'V9/*/*SimpleHolder_Replay*', 'V9b/*/*SimpleHolder_Replay*']
CAM_STEP_MM = 3.0


def kabsch(A, B):
    """R, t minimising |R A + t - B|."""
    ca, cb = A.mean(0), B.mean(0)
    U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb))
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    return R, cb - R @ ca


def camera_check(run):
    """Rigid fit of FK tip (base, mm) to NDI tip (camera, mm) over the run; returns residual series and the
    largest change of its 2 s rolling median (a step = camera or wand moved)."""
    tool = (run.meta.get('tool') or {}).get('at_start') or {}
    tip = tool.get('tip_tool0_mm')
    n = ndi.load(run)
    b = bags.load(run, ['/joint_states'])
    if tip is None or n is None or '/joint_states' not in b:
        return None
    K = kin.load(run.kinematics[1] or run.kinematics[0] or 'nominal')
    js = b['/joint_states']
    m = (n['visible'] == 1) & (n['unix_time'] > js['tr'][0]) & (n['unix_time'] < js['tr'][-1])
    tu = n['unix_time'][m][::3]
    Pn = np.c_[n['tip_x'], n['tip_y'], n['tip_z']][m][::3]
    best = None
    for delay in np.arange(0.0, 0.0405, 0.0025):           # NDI delay fitted per run (grid 0-40 ms), not assumed
        Q = np.column_stack([np.interp(tu - delay, js['tr'], js['q'][:, k]) for k in range(6)])
        Pr_ = np.array([(kin.tool0(K, q) @ np.r_[np.array(tip) / 1000, 1])[:3] for q in Q[::4]]) * 1000
        R_, t_ = kabsch(Pr_, Pn[::4])
        e = np.sqrt(np.mean(np.sum(((R_ @ Pr_.T).T + t_ - Pn[::4]) ** 2, 1)))
        if best is None or e < best[0]:
            best = (e, delay)
    delay = best[1]
    Q = np.column_stack([np.interp(tu - delay, js['tr'], js['q'][:, k]) for k in range(6)])
    Pr = np.array([(kin.tool0(K, q) @ np.r_[np.array(tip) / 1000, 1])[:3] for q in Q]) * 1000
    R, t = kabsch(Pr, Pn)
    res = np.linalg.norm((R @ Pr.T).T + t - Pn, axis=1)
    # persistent step: largest |median(next 4 s) - median(previous 4 s)| (a brief spike does not count)
    tt = tu - tu[0]
    step = 0.0
    for c in np.arange(4.0, tt[-1] - 4.0, 0.5):
        a_ = res[(tt >= c - 4) & (tt < c)]; b_ = res[(tt >= c) & (tt < c + 4)]
        if len(a_) > 10 and len(b_) > 10:
            step = max(step, abs(np.median(b_) - np.median(a_)))
    step = float(step)
    return dict(t=tu - tu[0], res=res, rms=float(np.sqrt(np.mean(res ** 2))), step=step, delay_ms=delay * 1e3,
                cam_pos=(-R.T @ t).tolist())


def condition(run):
    k = run.kinematics[0] or 'nominal'
    day = run.date
    mode = run.mode
    return f'{k} holder · {mode}', day, mode, k


# Strip-figure styles: one marker per output mode. Stock and ramped speedj are different settings and
# must be distinguishable; unfiltered velocity is its own mode (its name also contains 'velocity').
STRIP_STYLE = {
    'position':            dict(color=MODE['position']['color'], marker='^', label='Position'),
    'trajectory':          dict(color=MODE['trajectory']['color'], marker='D', label='Trajectory'),
    'velocity':            dict(color=MODE['velocity']['color'], marker='o', label='Stock speedj'),
    'velocity_optimised':  dict(color=MODE['velocity']['color'], marker='s', label='Ramped speedj'),
    'velocity_unfiltered': dict(color=MODE['velocity_unfiltered']['color'], marker='v', label='Velocity, unfiltered'),
}


def setting_label(kmodel, day, cal_days):
    """Short tick label: the kinematic model, and for the calibrated holder tool which of its days."""
    if kmodel == 'nominal':
        return 'Nominal holder'
    return 'Calibrated holder' if len(cal_days) >= 1 else 'Calibrated holder'


def main():
    style()
    out = []
    for pat in PATTERNS:
        for r in runs.find_runs(pat):
            if 'WRONG_SCALE' in r.key:
                continue
            s = next((x for x in SETS if r.trajectory.startswith(x)), None)
            if s is None or not r.has('NDI_PATH.csv'):
                continue
            e = ndi.rcm_errors(r)
            if e is None:
                continue
            b = bags.load(r, ['teleop/rcm_error', 'teleop/rcm_active'])
            ce = b.get('teleop/rcm_error')
            ctrl = float(np.sqrt(np.mean(ce['data'][np.isfinite(ce['data'])] ** 2)) * 1000) if ce else np.nan
            cam = camera_check(r)
            label, day, mode, kmodel = condition(r)
            excl = None
            if cam and cam['step'] > CAM_STEP_MM:
                excl = f"camera/wand moved: FK-vs-NDI residual stepped {cam['step']:.1f} mm during the run"
            if abs(r.scale - 0.2) > 1e-6 and r.scale != 1.0:
                pass
            out.append(dict(key=r.key, set=s, day=day, mode=mode, kin=kmodel, scale=r.scale, label=label,
                            rms=e['rms'], rec=e['recorded_rms'], p95=e['p95'], max=e['max'], within1=e['within1'],
                            bias=e['bias'], spread=e['spread'], rms_fitted=e['rms_fitted'], swing=e['swing_deg'],
                            ctrl_mm=ctrl, cam_rms=cam['rms'] if cam else np.nan, cam_step=cam['step'] if cam else np.nan,
                            cam_pos=cam['cam_pos'] if cam else None, ndi_delay_ms=cam['delay_ms'] if cam else np.nan, excluded=excl, cross=e['cross'].tolist()[::5],
                            visible=e['visible_pct']))
            print(r.key, f"{e['rms']:.3f}", excl or '')
    os.makedirs(os.path.join(HERE, 'cache'), exist_ok=True)
    json.dump(out, open(os.path.join(HERE, 'cache', 'runs.json'), 'w'))

    conds = sorted({(o['day'], o['mode'], o['kin']) for o in out})
    L = ['# R02 Results: RCM error measured with the NDI tracker', '',
         'Generated by `analysis.py`. RCM error = distance from the tracked shaft line to the RCM point latched at '
         'the start, over the longest RCM segment, recomputed from `NDI_PATH.csv` (matches the recorder\'s own '
         f"column: max |difference| {max(abs(o['rms'] - o['rec']) for o in out):.4f} mm over {len(out)} runs).", '',
         '**Excluded (found by the camera check: persistent step > %.0f mm between 4 s medians of the FK-vs-NDI residual):**' % CAM_STEP_MM, '']
    L += [f"- `{o['key']}`: {o['excluded']} (RCM rms {o['rms']:.2f} mm)" for o in out if o['excluded']] or ['- none']
    L += ['', '## NDI RCM error, rms mm (range over replays); controller error in brackets (mm)', '',
          '| Day | mode | kinematics | ' + ' | '.join(SET_LABEL[s] for s in SETS) + ' |', '|---|---|---|' + '---|' * 4]
    for c in conds:
        cells = []
        for s in SETS:
            v = [o for o in out if (o['day'], o['mode'], o['kin']) == c and o['set'] == s and not o['excluded']]
            if v:
                a = [o['rms'] for o in v]; cc = [o['ctrl_mm'] for o in v]
                cells.append(f"{min(a):.2f}–{max(a):.2f} ({np.nanmin(cc):.2f}–{np.nanmax(cc):.2f}) n={len(v)}")
            else:
                cells.append('–')
        L.append(f'| {c[0]} | {c[1]} | {c[2]} | ' + ' | '.join(cells) + ' |')
    inc = [o for o in out if not o['excluded']]
    L += ['', '## Decomposition per set (median over all included runs)', '',
          '| Set | NDI rms | bias (latched point → crossing centroid) | spread | about fitted pivot | % within 1 mm | swing (°) |',
          '|---|---|---|---|---|---|---|']
    for s in SETS:
        v = [o for o in inc if o['set'] == s]
        g = lambda k: np.median([o[k] for o in v])
        L.append(f"| {SET_LABEL[s]} | {g('rms'):.2f} | {g('bias'):.2f} | {g('spread'):.2f} | {g('rms_fitted'):.2f} | "
                 f"{100 * g('within1'):.0f} | {g('swing'):.0f} |")
    allc = np.array([o['ctrl_mm'] for o in inc]); alln = np.array([o['rms'] for o in inc])
    L += ['', f"Controller's own RCM error: {np.nanmin(allc):.2f}–{np.nanmax(allc):.2f} mm rms; NDI / controller "
          f"ratio {np.nanmin(alln / allc):.1f}–{np.nanmax(alln / allc):.1f}.",
          f"Phantom RCM under 1 mm in every included run: {all(o['rms'] < 1 for o in inc if o['set'] == 'RCMOnly')}.",
          '', '## Camera placement per day (fitted camera position in base, mm; median over runs)', '']
    for d in sorted({o['day'] for o in inc}):
        P = np.array([o['cam_pos'] for o in inc if o['day'] == d and o['cam_pos']])
        if len(P):
            L.append(f"- {d}: {np.median(P, 0).round(0).tolist()} (spread {np.ptp(P, 0).max():.0f} mm over {len(P)} runs)")
    dl = np.array([o['ndi_delay_ms'] for o in out if np.isfinite(o['ndi_delay_ms'])])
    L += ['', f'NDI delay fitted per run (FK vs NDI tip): median {np.median(dl):.1f} ms, range {dl.min():.1f}–{dl.max():.1f} ms '
          f'over {len(dl)} runs.']
    L += ['', '## Figures', '']

    # strip
    fig, ax = plt.subplots(figsize=(FULL, 2.9))
    cl = conds
    cal_days = sorted({c[0] for c in cl if c[2] != 'nominal'})
    for i, s in enumerate(SETS):
        v = [o for o in out if o['set'] == s]
        cc = [o['ctrl_mm'] for o in v if np.isfinite(o['ctrl_mm'])]
        if cc:
            ax.fill_between([i - 0.42, i + 0.42], min(cc), max(cc), color=MUTED, alpha=0.35, lw=0)
        for j, c in enumerate(cl):
            vv = [o for o in v if (o['day'], o['mode'], o['kin']) == c]
            x = i - 0.33 + 0.66 * j / max(1, len(cl) - 1)
            st = STRIP_STYLE.get(c[1], STRIP_STYLE['position'])
            for o in vv:
                ax.scatter(x + np.random.default_rng(len(o['key'])).uniform(-0.02, 0.02), o['rms'], s=16,
                           marker=st['marker'], facecolor='none' if o['excluded'] else st['color'],
                           edgecolor=st['color'], lw=0.8, zorder=3)
            if vv:
                ax.text(x, -0.2, setting_label(c[2], c[0], cal_days), rotation=90, fontsize=6.5, color=INK2,
                        ha='center', va='top')
    ax.axhline(1.0, color=INK2, ls='--', lw=0.8)
    ax.text(-0.45, 1.04, '1 mm threshold', fontsize=8, color=INK2, ha='left', va='bottom')
    ax.set_xticks(range(len(SETS))); ax.set_xticklabels([SET_LABEL[s] for s in SETS])
    ax.tick_params(axis='x', pad=72, length=0); ax.set_xlabel('Recorded trajectory')
    ax.set_ylabel('RCM error, rms (mm)'); ax.set_ylim(-0.05, None)
    hs = [plt.Line2D([], [], ls='', marker=STRIP_STYLE[m]['marker'], color=STRIP_STYLE[m]['color'],
                     label=STRIP_STYLE[m]['label']) for m in STRIP_STYLE if m in {c[1] for c in cl}]
    hs.append(plt.Rectangle((0, 0), 1, 1, color=MUTED, alpha=0.35, label="Controller's own error"))
    ax.legend(handles=hs, loc='lower left', bbox_to_anchor=(0, 1.0), ncol=3)
    L.append(f"- `figures/{os.path.basename(save(fig, HERE, 'rcm_strip'))}`")

    # crossing targets: set x day small multiples
    days = sorted({o['day'] for o in inc})
    fig, axes = plt.subplots(len(days), len(SETS), figsize=(FULL, 1.55 * len(days)), sharex=True, sharey=True)
    for i, d in enumerate(days):
        for j, s in enumerate(SETS):
            ax = axes[i, j]
            ax.add_patch(plt.Circle((0, 0), 1.0, fill=False, color=INK2, lw=0.6, ls='--'))
            for o in [o for o in inc if o['day'] == d and o['set'] == s]:
                X = np.array(o['cross'])
                # project onto two axes orthogonal to the plane normal: first two principal directions
                if len(X) > 3:
                    U = np.linalg.svd(X - X.mean(0), full_matrices=False)[2]
                    Y = X @ U[:2].T
                    ax.scatter(Y[:, 0], Y[:, 1], s=0.6, color=MODE.get(o['mode'], MODE['position'])['color'],
                               alpha=0.35, lw=0)
            ax.set_aspect('equal'); ax.set_xlim(-3.5, 3.5); ax.set_ylim(-3.5, 3.5)
            ax.plot(0, 0, '+', color='k', ms=5)
            if i == 0:
                ax.set_title(SET_LABEL[s], fontsize=8)
            if j == 0:
                nominal_at_day = any(o["kin"] == "nominal" for o in inc if o["day"] == d)
                ax.set_ylabel("Nominal holder" if nominal_at_day else "Calibrated holder", fontsize=8)
    fig.suptitle('Shaft crossing of the RCM plane (mm); +: latched point; dashed: 1 mm', fontsize=9, x=0.02,
                 ha='left')
    L.append(f"- `figures/{os.path.basename(save(fig, HERE, 'rcm_crossing_targets'))}`")
    open(os.path.join(HERE, 'RESULTS.md'), 'w').write('\n'.join(L) + '\n')
    print('\n'.join(L))


if __name__ == '__main__':
    main()
