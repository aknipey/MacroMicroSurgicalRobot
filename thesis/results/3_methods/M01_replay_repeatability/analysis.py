#!/usr/bin/env python3
"""M01 Replay repeatability: how identical are replays of the same input in the same configuration?

Groups are built from the files: same input (REPLAY.yaml replay_of, else trajectory label), same settings hash,
same kinematics, same code version, same parent folder. Path-level: replays aligned on the recorded stylus, then
the arm path compared sample by sample. Metric-level: lag, error, jitter (common.track) and NDI RCM rms
(common.ndi) per replay. Writes figures/ and RESULTS.md here.
"""
import os
import sys
from collections import defaultdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))  # thesis/results
from common import ndi, runs, track  # noqa: E402
from common.style import FULL, INK2, MODE, MUTED, save, style  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

PATTERNS = ['V9b/NoTool/*/*_Replay*', 'V9b/Singularity/*/*_Replay*', 'V9b/PositionBridge/*_Replay*',
            'V9b/VelocityOptimised/*_Replay*', 'V9b/VelocityUnfiltered/*_Replay*', 'V9/*/*_Replay*',
            'V8/CalibratedKinematics/*_Replay?', 'V7/*_Replay*']


def groups():
    g = defaultdict(list)
    for pat in PATTERNS:
        for r in runs.find_runs(pat):
            key = (os.path.dirname(r.key), r.trajectory, r.settings_hash(), r.kinematics[0], r.code_version)
            g[key].append(r)
    return {k: sorted(v, key=lambda r: r.replay_n) for k, v in g.items() if len(v) >= 2}


def stylus(run):
    s = run.csv('STYLUS_RAW.csv', ['t', 'x', 'y', 'z'])
    return s['t'], np.c_[s['x'], s['y'], s['z']]


def align(ref, other):
    """Shift (s) to add to `other`'s clock so its stylus speed best matches `ref`'s (xcorr on a 2 ms grid)."""
    (t1, p1), (t2, p2) = ref, other
    g = np.arange(max(t1[0], t2[0]), min(t1[-1], t2[-1]), 0.002)
    v = lambda t, p: np.linalg.norm(np.gradient(np.column_stack([np.interp(g, t, p[:, k]) for k in range(3)]),
                                                0.002, axis=0), axis=1)
    a = v(t1, p1)
    best = (-np.inf, 0.0)
    for s in np.arange(-0.5, 0.5, 0.002):
        b = np.linalg.norm(np.gradient(np.column_stack([np.interp(g + s, t2, p2[:, k]) for k in range(3)]),
                                       0.002, axis=0), axis=1)
        c = np.corrcoef(a, b)[0, 1]
        best = max(best, (c, s))
    return best[1]


def _refine(g, t1, X1, t2, X2, s0):
    best = (np.inf, s0)
    for s in np.arange(s0 - 0.006, s0 + 0.006, 0.0001):
        A = np.column_stack([np.interp(g, t1, X1[:, k], left=np.nan, right=np.nan) for k in range(3)])
        B = np.column_stack([np.interp(g + s, t2, X2[:, k], left=np.nan, right=np.nan) for k in range(3)])
        e = np.nanmean(np.sum((A - B) ** 2, 1))
        best = min(best, (e, s))
    return best[1]


def path_diff(ref, other, shift):
    """Arm-path (tool0) and stylus differences, each after its own sub-ms alignment, over the reference's strokes."""
    (tt, _, _), (ut, up, _) = track.load_paths(ref)
    (_, _, _), (ut2, up2, _) = track.load_paths(other)
    st1, st2 = stylus(ref), stylus(other)
    g = np.concatenate([np.arange(a, b, 0.002) for a, b in track.strokes(tt)])
    sa = _refine(g[::5], ut, up, ut2, up2, shift)
    ss = _refine(g[::5], st1[0], st1[1], st2[0], st2[1], shift)
    A = np.column_stack([np.interp(g, ut, up[:, k], left=np.nan, right=np.nan) for k in range(3)])
    B = np.column_stack([np.interp(g + sa, ut2, up2[:, k], left=np.nan, right=np.nan) for k in range(3)])
    d = np.linalg.norm(A - B, axis=1)
    S1 = np.column_stack([np.interp(g, st1[0], st1[1][:, k], left=np.nan, right=np.nan) for k in range(3)])
    S2 = np.column_stack([np.interp(g + ss, st2[0], st2[1][:, k], left=np.nan, right=np.nan) for k in range(3)])
    ds = np.linalg.norm(S1 - S2, axis=1)
    m = np.isfinite(d)
    return g[m], A[m], B[m], d[m], ds[np.isfinite(ds)], (sa - ss) * 1e3


def main():
    style()
    G = groups()
    rows = []
    example = None
    for key, rs in sorted(G.items()):
        folder, traj, sh, kin, code = key
        ref = rs[0]
        st_ref = stylus(ref)
        mets = [track.cached_metrics(r) for r in rs]
        # 1. same gap rule as folder L08: a gap > 50 ms in the recorded arm path inside a stroke excludes the replay
        keep = [(r, m) for r, m in zip(rs, mets) if m and not any(g_ > 0.05 for _, g_ in m['path_gaps'])]
        dropped = [r.key for r, m in zip(rs, mets) if (r, m) not in keep]
        if len(keep) < 2:
            continue
        rs, mets = [k[0] for k in keep], [k[1] for k in keep]
        ref = rs[0]
        st_ref = stylus(ref)
        rcm = [ndi.rcm_errors(r) if r.has('NDI_PATH.csv') else None for r in rs]
        pd, sd, toff = [], [], []
        for o in rs[1:]:
            sft = align(st_ref, stylus(o))
            g, A, B, d, ds, dt_ms = path_diff(ref, o, sft)
            pd.append(d); sd.append(ds); toff.append(dt_ms)
            if example is None and folder.endswith('PositionMode') and traj == 'LongBenchmark':
                example = (ref, o, g, A, B, d)
        d = np.concatenate(pd); ds = np.concatenate(sd)
        row = dict(dropped=dropped, timing_ms=toff, folder=folder, traj=traj, n=len(rs), mode=rs[0].mode, path_rms=np.sqrt(np.mean(d ** 2)) * 1e3,
                   path_max=d.max() * 1e3, stylus_rms=np.sqrt(np.mean(ds ** 2)) * 1e3)
        for k in ('lag_ms', 'lag_rot_ms', 'err_mm', 'jitter_arm_mm'):
            v = [m[k] for m in mets if m]
            row[k] = v
        tg = ref.csv('TOUCH_PATH.csv')
        from scipy.spatial.transform import Rotation as _R
        Rq = _R.from_quat(np.c_[tg['qx'], tg['qy'], tg['qz'], tg['qw']])
        rot_range = float(np.degrees(np.max((Rq[0].inv() * Rq).magnitude())))
        row['rot_range'] = rot_range
        if rot_range < 10.0:              # too little rotation for a rotation lag to mean anything
            row['lag_rot_ms'] = []
        row['rcm'] = [e['rms'] for e in rcm if e]
        rows.append(row)

    # pooled within-group SD per metric (pairs/triples), and coefficient of repeatability
    def pooled(key):
        num = den = 0
        for r in rows:
            v = np.array(r[key], float)
            v = v[np.isfinite(v)]
            if len(v) >= 2:
                num += np.sum((v - v.mean()) ** 2); den += len(v) - 1
        s = np.sqrt(num / den) if den else np.nan
        return s, 2.77 * s, den
    L = ['# M01 Results: replay repeatability', '', 'Generated by `analysis.py`. Groups = same input, settings hash, '
         'kinematics, code version and folder (from META/REPLAY). Path difference: tool0 position of replay k minus '
         'replay 1, after aligning on the recorded stylus, over engaged strokes (µm).', '',
         '## Floors (pooled within-group SD; repeatability coefficient = 2.77 SD)', '',
         '| Metric | within-group SD | repeatability coefficient | df |', '|---|---|---|---|']
    for key, lab in (('lag_ms', 'translation lag (ms)'), ('lag_rot_ms', 'rotation lag (ms)'),
                     ('err_mm', 'lag-removed error (mm)'), ('jitter_arm_mm', 'arm jitter > 5 Hz (mm)'),
                     ('rcm', 'NDI RCM rms (mm)')):
        s, cr, df = pooled(key)
        L.append(f'| {lab} | {s:.3f} | {cr:.3f} | {df} |')
    L += ['', '## Per group', '', '| Folder | input | mode | n | stylus diff rms (µm) | arm path diff rms / max (µm) | '
          'lag (ms) | rot lag (ms) | NDI RCM rms (mm) |', '|---|---|---|---|---|---|---|---|---|']
    for r in rows:
        f = lambda v, p=0: '/'.join(f'{x:.{p}f}' for x in v) if v else '–'
        L.append(f"| {r['folder']} | {r['traj']} | {r['mode']} | {r['n']} | {r['stylus_rms']:.1f} | "
                 f"{r['path_rms']:.0f} / {r['path_max']:.0f} | {f(r['timing_ms'], 1)} | {f(r['lag_ms'])} | {f(r['lag_rot_ms'])} | "
                 f"{f(r['rcm'], 2)} |")
    pr = np.array([r['path_rms'] for r in rows])
    L += ['', f'Arm-path difference between replays: median {np.median(pr):.0f} µm rms, range {pr.min():.0f}–'
          f'{pr.max():.0f} µm, over {len(rows)} groups. After the best constant shift per pair, what remains is mostly '
          'replay timing jitter of a few ms, which shows as path difference in proportion to speed (largest on the fast '
          'pivoting FastChecking runs). The metric floors above are the numbers to compare effects against.', '',
          '## Prior claims checked', '', '- "replays match within 1 %": see the lag SD relative to ~60–200 ms lags.',
          '- "RCM replays repeat to ~0.1 mm": see the NDI RCM row of the floor table.', '', '## Figures', '']

    # figure 1: example overlay with magnified difference
    if example:
        ref, o, g, A, B, d = example
        fig, (a1, a2) = plt.subplots(2, 1, figsize=(FULL, 3.0), sharex=True, gridspec_kw=dict(height_ratios=[2, 1]))
        k = np.argmax(np.ptp(A, 0))
        t0 = g[0]
        a1.plot(g - t0, A[:, k], color=MODE['position']['color'], lw=1.6, label='replay 1')
        a1.plot(g - t0, B[:, k], color=INK2, lw=0.7, ls='--', label='replay 2')
        a1.set_ylabel(f'tool0 {"xyz"[k]} (mm)'); a1.legend(loc='upper right', ncol=2)
        a1.set_title(f'Two replays of the same input ({o.trajectory}, position output)')
        a2.plot(g - t0, d * 1e3, color=INK2, lw=0.6)
        a2.set_ylabel('|difference| (µm)'); a2.set_xlabel('time (s, engaged strokes joined)')
        L.append(f"- `figures/{os.path.basename(save(fig, HERE, 'replay_overlay'))}`")
    # figure 2: Bland-Altman style for lag across all pairs, with effect sizes as reference
    fig, axes = plt.subplots(1, 3, figsize=(FULL, 2.2))
    for ax, (key, lab, eff) in zip(axes, (('lag_ms', 'lag (ms)', 120), ('err_mm', 'lag-removed error (mm)', None),
                                          ('rcm', 'NDI RCM rms (mm)', 1.0))):
        for r in rows:
            v = np.array(r[key], float)
            if len(v) >= 2:
                for x in v[1:]:
                    ax.scatter((v[0] + x) / 2, x - v[0], s=10, color=MODE.get(r['mode'], MODE['position'])['color'])
        s, cr, _ = pooled(key)
        for y in (cr, -cr):
            ax.axhline(y, color=INK2, ls='--', lw=0.7)
        ax.axhline(0, color=MUTED, lw=0.6)
        ax.set_xlabel(f'mean {lab}'); ax.set_ylabel('replay k − replay 1')
        ax.set_title(lab, fontsize=7.5)
    L.append(f"- `figures/{os.path.basename(save(fig, HERE, 'bland_altman'))}`")
    open(os.path.join(HERE, 'RESULTS.md'), 'w').write('\n'.join(L) + '\n')
    print('\n'.join(L))


if __name__ == '__main__':
    main()
