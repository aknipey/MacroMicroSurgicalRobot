#!/usr/bin/env python3
"""R05 Cause of the latch offset (a tool-frame line error) and its measured correction (phase L).

Run from thesis/results with ROS and the workspace sourced (folder R02's camera check reads bags).
Writes RESULTS.md, figures/ and cache/ here; the recordings are read-only. See ANALYSIS.md.
"""
import glob
import json
import os
import re
import sys

import numpy as np
import yaml
from scipy.optimize import least_squares

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))  # thesis/results
from common import bags, runs  # noqa: E402
from common.style import BLUE, FULL, GREY, GREY_DARK, INK2, VERMILLION, panel, save, style  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, os.path.join(runs.PKG, 'scripts', 'calibration'))
import latch_line_fit as llf  # noqa: E402
import importlib.util  # noqa: E402
_spec = importlib.util.spec_from_file_location('rcm09', os.path.join(runs.folder('R02'), 'analysis.py'))
_rcm09 = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_rcm09)
camera_check, CAM_STEP_MM = _rcm09.camera_check, _rcm09.CAM_STEP_MM   # folder R02's check, same threshold

SETS = ['RCMOnly', 'SlowChecking', 'FastChecking', 'WholeRunDrinkBottle']
LABEL = {'RCMOnly': 'RCM only', 'SlowChecking': 'Slow checking', 'FastChecking': 'Fast checking',
         'WholeRunDrinkBottle': 'Whole run'}
GROUPS = {   # group -> data/recordings/ globs; holder and flange mounting groups
    'Fresh fit': ['V7/*SimpleHolder_Replay*'],
    'Calibrated holder': ['V8/CalibratedKinematics/*SimpleHolder_Replay*', 'V9/*/*SimpleHolder_Replay*'],
    'FLANGE_FRESH': ['V9b/*/*SimpleHolder_Replay*'],
    'FLANGE_TEST': ['V9c/RCM/*/*SimpleHolder_Replay*', 'V9c/RCM_fit*/*SimpleHolder_Replay*',
          'V9c/RCM_perturb*/*SimpleHolder_Replay*'],
    'L-train': ['V9c/Latch/Train/*SimpleHolder_Replay*'],
    'FRESH': ['V9c/Latch/A_tool10/*SimpleHolder_Replay*'],
    'CORRECTED': ['V9c/Latch/B_tool11/*SimpleHolder_Replay*'],
    'ALL_FOUR': ['V9c/Latch/C_tool12/*SimpleHolder_Replay*'],          # follow-up, chosen after the CORRECTED result
    'DRIFT': ['V9c/Latch/A_tool10_drift/*SimpleHolder_Replay*'],  # fresh fit again, between the corrected-on-all-four tool blocks
}
PREDICTION_C = {'RCMOnly': 0.31, 'SlowChecking': 0.22, 'FastChecking': 0.50, 'WholeRunDrinkBottle': 0.63}  # VERSIONS.md, before
PATH_MIN = 0.8          # method parameter: TCP path < 80 % of the group/trajectory median = did not run as recorded
VIS_MIN = 0.95          # method parameter: tracked share of the whole recording (same rule as replay_batch.sh)
PREDICTION_MM = 0.35    # written before phase L ran (docs/VERSIONS.md, run_v9c.sh phase L header)
CACHE = os.path.join(HERE, 'cache', 'runs.json')


def tcp_path_mm(path):
    d = llf.load_csv(os.path.join(path, 'UR_PATH.csv'))
    P = np.column_stack([d['x'], d['y'], d['z']]) * 1000
    return float(np.sum(np.linalg.norm(np.diff(P, axis=0), axis=1)))


def ctrl_rms_mm(run):
    b = bags.load(run, ['teleop/rcm_error'])
    ce = b.get('teleop/rcm_error')
    if not ce:
        return np.nan
    v = ce['data'][np.isfinite(ce['data'])]
    return float(np.sqrt(np.mean(v ** 2)) * 1000)


def latch_speed_and_offset(r):
    """Tip speed at the latch (mm/s, fit over 0.5 s before) and latched point -> best pivot (mm)."""
    d = llf.load_csv(os.path.join(r.path, 'NDI_PATH.csv'))
    act = np.nan_to_num(d['rcm_active']).astype(int)
    s0 = int(np.flatnonzero(np.diff(np.r_[0, act]) == 1)[0])
    t0 = d['t'][s0]
    m = (d['t'] > t0 - 0.5) & (d['t'] <= t0 + 0.05) & (d['visible'] == 1)
    if m.sum() < 5:
        return np.nan, np.nan
    v = np.array([np.polyfit(d['t'][m], d[k][m], 1)[0] for k in ('tip_x', 'tip_y', 'tip_z')])
    tips = np.einsum('nij,j->ni', r.R, r.o) + r.t
    u = np.einsum('nij,j->ni', r.R, r.a)
    A = np.zeros((3, 3)); b = np.zeros(3)
    for p, a in zip(tips, u):
        M = np.eye(3) - np.outer(a, a); A += M; b += M @ p
    return float(np.linalg.norm(v)), float(np.linalg.norm(np.linalg.solve(A, b) - r.P))


def collect():
    old = json.load(open(CACHE)) if os.path.exists(CACHE) else {}
    rows = []
    for g, pats in GROUPS.items():
        for pat in pats:
            for path in sorted(glob.glob(os.path.join(runs.LOGS, pat))):
                key = os.path.relpath(path, runs.LOGS)
                name = os.path.basename(path)
                if g in ('Fresh fit', 'Calibrated holder', 'FLANGE_FRESH', 'FLANGE_TEST') and not re.search(r'_Replay\d+$', name):
                    continue          # as folder R02: suffixed folders (_WRONG_SCALE, _NO_NDI) were set aside at the time
                s = next((x for x in SETS if name.startswith(x)), None)
                if s is None or not os.path.exists(os.path.join(path, 'NDI_PATH.csv')):
                    continue
                try:
                    r = llf.Run(path)
                except (ValueError, KeyError) as exc:
                    rows.append(dict(key=key, group=g, set=s, excluded=f'no usable RCM segment ({exc})'))
                    continue
                c = old.get(key) or {}
                if 'cam_step' not in c:
                    run = runs.Run(path)
                    cam = camera_check(run)
                    c = dict(cam_step=cam['step'] if cam else None, ctrl=ctrl_rms_mm(run), path=tcp_path_mm(path))
                    old[key] = c
                rms, slope = r.as_run()
                _, w, _ = llf.fit_line([r])
                speed, offset = latch_speed_and_offset(r)
                seg = llf.load_csv(os.path.join(path, 'NDI_PATH.csv'))
                vis = float(np.mean(seg['visible'] == 1))   # whole recording, as the recording batch's 95 % rule
                rows.append(dict(key=key, group=g, set=s, tool=r.tool, tool_name=r.tool_name,
                                 mode=key.split('/')[2] if key.startswith('V9c/RCM/') else '',
                                 rms=rms, slope=slope, angle=float(np.degrees(np.arccos(np.clip(np.dot(w, r.a), -1, 1)))),
                                 speed=speed, offset=offset, vis=vis, ctrl=c['ctrl'], path=c['path'],
                                 cam_step=c['cam_step'], excluded=None, _run=r))
    os.makedirs(os.path.dirname(CACHE), exist_ok=True)
    json.dump(old, open(CACHE, 'w'), indent=1)
    # exclusions, all derived here
    for o in rows:
        if o.get('excluded'):
            continue
        same = [x['path'] for x in rows if x.get('path') and x['group'] == o['group'] and x['set'] == o['set']]
        if o['cam_step'] is not None and o['cam_step'] > CAM_STEP_MM:
            o['excluded'] = f"camera/wand moved (FK-vs-NDI residual stepped {o['cam_step']:.1f} mm, folder R02 check)"
        elif o['vis'] < VIS_MIN:
            o['excluded'] = f"wand tracked in {100 * o['vis']:.0f} % of the recording"
        elif o['path'] < PATH_MIN * np.median(same):
            o['excluded'] = (f"arm path {o['path']:.0f} mm against a median of {np.median(same):.0f} mm for this "
                             'trajectory: the replay did not run as recorded')
    return rows


def rng(v, f='{:.2f}'):
    v = np.asarray(v, float)
    return (f + '–' + f).format(v.min(), v.max()) if len(v) else '–'


def fixed_line_residual(train, test):
    c, w, _ = llf.fit_line(train)
    out = []
    for r in test:
        s = least_squares(lambda x: r.miss(c, w, r.P + x).reshape(-1), np.zeros(3))
        out.append(float(np.sqrt(np.mean(np.sum(r.miss(c, w, r.P + s.x) ** 2, axis=1)))))
    return out


def main():
    style()
    rows = collect()
    inc = [o for o in rows if not o.get('excluded')]
    L = ['# R05 Results: cause of the latch offset and its measured correction', '',
         'Generated by `analysis.py` (method in ANALYSIS.md; line model from `scripts/latch_line_fit.py`). NDI RCM error = '
         'rms distance of the tracked shaft from the latched point over the longest RCM segment, recomputed from '
         'NDI_PATH.csv. mm unless stated. Settings: holder, nominal; holder, calibrated; flange-mounted fresh fit; flange-mounted tool-line test; '
         '(Fresh fit; corrected from the training runs only).', '',
         f'**Excluded (checks run here: folder R02 camera step > {CAM_STEP_MM:.0f} mm, tracked < {100 * VIS_MIN:.0f} %, '
         f'arm path < {100 * PATH_MIN:.0f} % of the median):**', '']
    L += [f"- `{o['key']}`: {o['excluded']}" for o in rows if o.get('excluded')] or ['- none']
    L += ['', f'Included: {len(inc)} runs.', '']

    # --- 1. timing
    t = [o for o in inc if o['group'] in 'ABCD' and np.isfinite(o['speed'])]
    r_ = np.corrcoef([o['speed'] for o in t], [o['offset'] for o in t])[0, 1]
    L += ['## 1. Is the offset a latch-timing effect?', '',
          f"Latch offset against tip speed at the latch, holder-tool and flange-mounted runs, {len(t)} runs with the tip tracked before the "
          f"latch: r = {r_:.3f}. Tip speed {rng([o['speed'] for o in t], '{:.1f}')} mm/s, offset "
          f"{rng([o['offset'] for o in t])} mm. RCM-only runs latch at rest and have no pre-latch speed.", '']

    # --- 2. line model per mounting and set
    L += ['## 2. One line fixed in the wand frame, per mounting and trajectory', '',
          '| Setting | Trajectory | n | as run (median) | fixed-line residual (median) | line angle (°, median) | depth slope (mm/100 mm, median) |',
          '|---|---|---|---|---|---|---|']
    base = {g: [o for o in inc if o['group'] == g and (g != 'FLANGE_TEST' or o['tool'] == 4)] for g in ('Fresh fit', 'Calibrated holder', 'FLANGE_FRESH', 'FLANGE_TEST')}
    for g, rs in base.items():
        for s in SETS:
            v = [o for o in rs if o['set'] == s]
            if not v:
                continue
            res = [fixed_line_residual([o['_run']], [o['_run']])[0] for o in v]
            L.append(f"| {g} | {LABEL[s]} | {len(v)} | {np.median([o['rms'] for o in v]):.2f} | {np.median(res):.2f} | "
                     f"{np.median([o['angle'] for o in v]):.2f} | {np.median([o['slope'] for o in v]):.2f} |")
    L += ['', 'The flange-mounted fresh-fit rows use the initial flange-mounted fit; RCM only spans six output/filter settings.', '']

    # --- 3. flange-mounted fresh-fit baseline (Table 4.2 rows)
    d4 = [o for o in inc if o['group'] == 'FLANGE_TEST' and o['tool'] == 4]
    L += ['## 3. Flange-mounted fresh fit (tool 4), for Table 4.2', '',
          f"- RCM only, P5: {rng([o['rms'] for o in d4 if o['set'] == 'RCMOnly' and o['mode'] == 'P5'])} "
          f"(n = {sum(o['set'] == 'RCMOnly' and o['mode'] == 'P5' for o in d4)}); all six settings: "
          f"{rng([o['rms'] for o in d4 if o['set'] == 'RCMOnly'])} (n = {sum(o['set'] == 'RCMOnly' for o in d4)}, "
          f"settings {', '.join(sorted({o['mode'] for o in d4 if o['set'] == 'RCMOnly'}))})",
          f"- Slow checking, P5: {rng([o['rms'] for o in d4 if o['set'] == 'SlowChecking'])} "
          f"(n = {sum(o['set'] == 'SlowChecking' for o in d4)})",
          f"- Controller's own error, all flange-mounted runs: {rng([o['ctrl'] for o in d4])}", '']
    other = [o for o in inc if o['group'] == 'FLANGE_TEST' and o['tool'] in (5, 6)]
    L += [f"- Fits 2 and 3 (tools 5, 6), slow checking: {rng([o['rms'] for o in other])} (n = {len(other)})"]
    for tn, lab in ((7, '+1 mm across the shaft'), (8, '+2 mm along the shaft')):
        v = [o for o in inc if o['group'] == 'FLANGE_TEST' and o['tool'] == tn]
        L.append(f"- Tool 4 moved {lab} (tool {tn}), slow checking: {rng([o['rms'] for o in v])} (n = {len(v)})")
    L.append('')

    # --- 4. cross-validation (flange-mounted tool 4; holder-tool and flange-mounted runs)
    L += ['## 4. Does a line fitted on some runs predict others?', '',
          '| Trained on | Tested on (held out) | n | as run | predicted residual |', '|---|---|---|---|---|']

    def runs_of(sel):
        return [o['_run'] for o in sel]
    tr = [o for o in d4 if o['mode'] == 'P5' and re.search(r'_Replay[12]$', o['key'])]
    te = [o for o in d4 if o not in tr]
    for s in ('RCMOnly', 'SlowChecking'):
        v = [o for o in te if o['set'] == s]
        p = fixed_line_residual(runs_of(tr), runs_of(v))
        L.append(f"| D: RCM only + slow checking, P5 replays 1–2 | D {LABEL[s]}, every other replay and setting | {len(v)} | "
                 f"{rng([o['rms'] for o in v])} | {rng(p)} |")
    for g in ('Calibrated holder', 'FLANGE_FRESH'):
        for sub in sorted({o['key'].split('/')[1] for o in base[g]}):
            trn = [o for o in base[g] if o['key'].split('/')[1] == sub and o['set'] == 'RCMOnly']
            tst = [o for o in base[g] if o['key'].split('/')[1] == sub and o['set'] != 'RCMOnly']
            if not trn or not tst:
                continue
            for s in ('SlowChecking', 'FastChecking', 'WholeRunDrinkBottle'):
                v = [o for o in tst if o['set'] == s]
                if v:
                    L.append(f"| {g} {sub}: RCM only | {LABEL[s]} | {len(v)} | {rng([o['rms'] for o in v])} | "
                             f"{rng(fixed_line_residual(runs_of(trn), runs_of(v)))} |")
    L.append('')

    # --- 5. recovery of known tool errors; hand-eye fit spread
    he4 = (yaml.safe_load(open(os.path.join(runs.PKG, 'config', 'tools.yaml')))['tools'][4]['calibration'])
    tips, dirs, labs = [], [], []
    for tn in (4, 5, 6, 7, 8):
        v = [o['_run'] for o in inc if o['group'] == 'FLANGE_TEST' and o['tool'] == tn and o['set'] == 'SlowChecking']
        cal = v[0].calibration if tn in (4, 5, 6) else he4
        tip, dr, _ = llf.corrected_tool(v, llf.handeye_X(cal))
        tips.append(tip); dirs.append(dr); labs.append(tn)
    tips, dirs = np.array(tips), np.array(dirs)
    ctip, cdir = tips.mean(0), dirs.mean(0) / np.linalg.norm(dirs.mean(0))
    L += ['## 5. Does the correction recover the same tool from different starting errors?', '',
          '| Starting tool | corrected tip in tool0 (mm) | from the mean (mm) | direction from the mean (°) |', '|---|---|---|---|']
    for tn, tp, dr in zip(labs, tips, dirs):
        L.append(f"| {tn} | {np.round(tp, 2).tolist()} | {np.linalg.norm(tp - ctip):.2f} | "
                 f"{np.degrees(np.arccos(np.clip(dr @ cdir, -1, 1))):.3f} |")
    lib = yaml.safe_load(open(os.path.join(runs.PKG, 'config', 'tools.yaml')))['tools']
    t11 = np.array(lib[11]['tip_tool0_mm']); d11 = np.array(lib[11]['direction_tool0']); d11 /= np.linalg.norm(d11)
    t4, t10 = np.array(lib[4]['tip_tool0_mm']), np.array(lib[10]['tip_tool0_mm'])
    d4_, d10 = (np.array(lib[k]['direction_tool0']) for k in (4, 10))
    ang = lambda a, b: float(np.degrees(np.arccos(np.clip(a @ b / np.linalg.norm(a) / np.linalg.norm(b), -1, 1))))  # noqa: E731
    L += ['', f"Tools 7 and 8 use tool 4's hand-eye fit. Spread of the five corrected tools: tip "
          f"{max(np.linalg.norm(tips - ctip, axis=1)):.2f} mm, direction {max(ang(d, cdir) for d in dirs):.3f}° from their mean.",
          f"- Tool 11 (corrected in the latch test, after the wand was re-seated and refitted) from that mean: tip "
          f"{np.linalg.norm(t11 - ctip):.2f} mm, direction {ang(d11, cdir):.3f}°.",
          f"- The two hand-eye fits themselves (fresh flange fit vs tool-line fresh fit): tips {np.linalg.norm(t4 - t10):.2f} mm apart, "
          f"directions {ang(d4_, d10):.2f}° apart.", '']

    # --- 6. latch test
    L += ['## 6. Latch test: fresh fit against corrected tool, interleaved', '',
          f'Prediction written before the run: Corrected tool <= {PREDICTION_MM} mm on RCM only and slow checking.', '',
          '| Trajectory | tool | n | NDI rms mean [range] | depth slope (mm/100 mm) | line angle (°) | controller rms |',
          '|---|---|---|---|---|---|---|']
    lt = {}
    for s in SETS:
        for g, tl in (('FRESH', 'Fresh fit'), ('CORRECTED', 'Corrected tool')):
            v = [o for o in inc if o['group'] == g and o['set'] == s]
            lt[(s, tl)] = v
            if v:
                L.append(f"| {LABEL[s]} | {tl} | {len(v)} | {np.mean([o['rms'] for o in v]):.2f} [{rng([o['rms'] for o in v])}] | "
                         f"{np.mean([o['slope'] for o in v]):.2f} | {np.mean([o['angle'] for o in v]):.2f} | "
                         f"{rng([o['ctrl'] for o in v])} |")
    L += ['']
    for s in SETS:
        a, b = (np.array([o['rms'] for o in lt[(s, k)]]) for k in ('Fresh fit', 'Corrected tool'))
        if len(a) and len(b):
            verdict = ''
            if s in ('RCMOnly', 'SlowChecking'):
                verdict = ' (prediction met)' if b.max() <= PREDICTION_MM else ' (prediction NOT met)'
            L.append(f"- {LABEL[s]}: Corrected/fresh = {b.mean() / a.mean():.2f}, Corrected − fresh = {b.mean() - a.mean():+.2f} mm{verdict}")
    L += ['', '### Follow-up, chosen after the result above: corrected-on-all-four tool = fresh fit corrected on all four trajectories', '',
          'Trained on the four training runs plus the first tool-10 fast checking and whole run replays; tested on new '
          'replays (blocks 12 12 10 12). Offline prediction written before the run: '
          + ', '.join('%s %.2f' % (LABEL[k], v) for k, v in PREDICTION_C.items()) + ' mm.', '',
          '| Trajectory | n | NDI rms mean [range] | predicted | replays over 1 mm | line angle (°) | fresh-fit drift block vs fresh-fit mean |',
          '|---|---|---|---|---|---|---|']
    for s in SETS:
        v = [o['rms'] for o in inc if o['group'] == 'ALL_FOUR' and o['set'] == s]
        ang = [o['angle'] for o in inc if o['group'] == 'ALL_FOUR' and o['set'] == s]
        dr = [o['rms'] for o in inc if o['group'] == 'DRIFT' and o['set'] == s]
        a = [o['rms'] for o in lt[(s, 'Fresh fit')]]
        if v:
            L.append(f"| {LABEL[s]} | {len(v)} | {np.mean(v):.2f} [{rng(v)}] | {PREDICTION_C[s]:.2f} | {sum(x > 1.0 for x in v)} of {len(v)} | "
                     f"{np.mean(ang):.2f} | {(np.mean(dr) - np.mean(a)):+.2f} |" if dr else '| – |')
    allc = [o for o in inc if o['group'] == 'ALL_FOUR']
    L += ['', f"- All four trajectories <= 1.0 mm with corrected-on-all-four tool: {'yes' if allc and max(o['rms'] for o in allc) <= 1.0 else 'no'} "
          f"(largest {max(o['rms'] for o in allc):.2f} mm)" if allc else '- no corrected-on-all-four tool runs', '']
    trn = [o for o in inc if o['group'] == 'L-train']
    L += ['- Training runs (fresh fit): ' + ', '.join('%s %.2f' % (LABEL[o['set']], o['rms']) for o in trn), '']

    # --- 7. pose dependence
    L += ['## 7. Is the line error the same on every trajectory?', '',
          'Line angle (°) per trajectory with the same tool, median over replays:', '',
          '| Tool | ' + ' | '.join(LABEL[s] for s in SETS) + ' |', '|---|' + '---|' * len(SETS)]
    for lab, g, tn in (('Fresh fit', 'FRESH', 10), ('Corrected tool', 'CORRECTED', 11), ('Flange-mounted fresh fit', 'FLANGE_TEST', 4)):
        cells = []
        for s in SETS:
            v = [o['angle'] for o in inc if o['group'] == g and o['tool'] == tn and o['set'] == s]
            cells.append(f'{np.median(v):.2f}' if v else '–')
        L.append(f'| {lab} | ' + ' | '.join(cells) + ' |')
    L.append('')

    # --- prior claims
    L += ['## Prior claims checked', '',
          '- "r = 0.008 between offset and latch speed": section 1.',
          '- "fixed line explains slow runs to 0.13–0.25 mm; ~0.3° (B/C), ~1.05° (D)": section 2.',
          '- "corrected tools from the corrected tools agree to ~0.15 mm / 0.01°": section 5.',
          '- "phase L: Corrected tool <= 0.35 mm on RCM only and slow checking; fast better; whole run worse": section 6.',
          '- "corrected-on-all-four tool (all four): 0.31 / 0.22 / 0.50 / 0.63 mm" (prediction): section 6, follow-up.', '']

    # --- figures
    figs = [fig_depth(inc), fig_ab(inc)]
    L += ['## Figures', ''] + [f'- `figures/{os.path.basename(p)}`' for p in figs]
    open(os.path.join(HERE, 'RESULTS.md'), 'w').write('\n'.join(L) + '\n')
    print('\n'.join(L))


def fig_depth(inc):
    fig, ax = plt.subplots(figsize=(FULL / 2, 2.3), layout='constrained')
    for g, col, lab in (('FRESH', VERMILLION, 'Fresh fit'), ('CORRECTED', BLUE, 'Corrected tool')):
        v = [o['_run'] for o in inc if o['group'] == g and o['set'] == 'SlowChecking']
        D = np.concatenate([r.depth() for r in v])
        E = np.concatenate([np.linalg.norm(r.miss(r.o, r.a, r.P), axis=1) for r in v])
        bins = np.arange(np.floor(D.min() / 10) * 10, D.max() + 10, 10)
        k = np.digitize(D, bins)
        x = [D[k == i].mean() for i in np.unique(k) if (k == i).sum() > 30]
        y = [np.median(E[k == i]) for i in np.unique(k) if (k == i).sum() > 30]
        q1 = [np.percentile(E[k == i], 25) for i in np.unique(k) if (k == i).sum() > 30]
        q3 = [np.percentile(E[k == i], 75) for i in np.unique(k) if (k == i).sum() > 30]
        ax.fill_between(x, q1, q3, color=col, alpha=0.2, lw=0)
        ax.plot(x, y, color=col, lw=1.4, label=lab)
    ax.axhline(1.0, color=GREY, ls='--', lw=0.8)
    ax.set_xlabel('Tip depth below the RCM point (mm)')
    ax.set_ylabel('RCM error (mm)')
    ax.set_ylim(bottom=0)
    ax.legend(frameon=False, loc='upper left')
    panel(ax, 'a', 'Slow checking')
    fig_b = fig
    return save(fig_b, HERE, 'latch_depth')


def fig_ab(inc):
    fig, ax = plt.subplots(figsize=(FULL / 2, 2.3), layout='constrained')
    for i, s in enumerate(SETS):
        for g, col, mk, dx, lab in (('FRESH', VERMILLION, 'o', -0.2, 'Fresh fit'), ('CORRECTED', BLUE, '^', 0.0, 'Corrected tool')):
            v = [o['rms'] for o in inc if o['group'] == g and o['set'] == s]
            ax.scatter(np.full(len(v), i + dx), v, color=col, marker=mk, s=14, zorder=3, label=lab if i == 0 else None)
    ax.axhline(1.0, color=GREY, ls='--', lw=0.8)
    ax.set_xticks(range(len(SETS)))
    ax.set_xticklabels([LABEL[s].replace(' ', '\n', 1) for s in SETS])
    ax.set_ylabel('RCM error, rms (mm)')
    ax.set_ylim(bottom=0)
    ax.tick_params(axis='x', colors=INK2)
    ax.set_ylim(0, 2.45)
    ax.legend(frameon=False, loc='upper left', handletextpad=0.2, borderaxespad=0.2)
    panel(ax, 'b', 'Per replay')
    return save(fig, HERE, 'latch_ab')


if __name__ == '__main__':
    main()
