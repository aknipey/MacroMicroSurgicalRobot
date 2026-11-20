#!/usr/bin/env python3
"""L07 Threshold (i): tremor-band vibration with the final filtering vs the unfiltered baseline (V9b/NoTool, 1:5).

Windows: 'moving' from the operator's input (TOUCH_PATH target speed above a threshold picked from the speed
histogram of the unfiltered replays), >= 1 s, 0.25 s trimmed each end; identical for every mode because the input
is identical. Signals (bag, 500 Hz, on the CSV clock): measured joint velocity, tool-space velocity J(q) qd
(calibrated kinematics), the joint-velocity command (velocity/position modes), and the stylus. Motion is removed
with a 3 Hz zero-phase low-pass before the PSD (offline, so zero-phase is fine here). Welch, Hann, 1 s segments,
50 % overlap; band power 15-25 Hz (primary, fixed before computing), 8-12 Hz and 30-60 Hz (secondary).
Uncertainty: bootstrap over Welch segments (2000). Writes figures/ and RESULTS.md.
"""
import os
import sys

import numpy as np
from scipy.signal import butter, sosfiltfilt, welch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))  # thesis/results
from common import bags, kin, runs, track  # noqa: E402
from common.style import FULL, HALF, INK2, MODE, MUTED, panel, save, style  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

FS = 500.0
NPER = 500
BANDS = {'tremor 8–12 Hz': (8, 12), 'primary 15–25 Hz': (15, 25), '30–60 Hz': (30, 60)}
PRIMARY = 'primary 15–25 Hz'
REQ = 1 - 0.7 ** 2   # threshold (i): 30 % in amplitude = 51 % in power, judged on the mean
ORDER = ['velocity_unfiltered', 'velocity_optimised', 'position', 'trajectory']
RNG = np.random.default_rng(1)
SOS_LP = butter(4, 3.0, 'lowpass', fs=FS, output='sos')


def moving_windows(run, thr):
    tg = run.csv('TOUCH_PATH.csv')
    t = tg['t']
    P = np.c_[tg['x'], tg['y'], tg['z']] * 1000
    out = []
    for a, b in track.strokes(t):
        g = np.arange(a, b, 0.01)
        p = np.column_stack([np.interp(g, t, P[:, k]) for k in range(3)])
        v = np.linalg.norm(np.gradient(p, 0.01, axis=0), axis=1)
        v = np.convolve(v, np.ones(20) / 20, mode='same')
        mv = v > thr
        i = 0
        while i < len(g):
            if mv[i]:
                j = i
                while j < len(g) and mv[j]:
                    j += 1
                s, e = g[i] + 0.25, g[j - 1] - 0.25
                if e - s >= 1.0:
                    out.append((s, e))
                i = j
            else:
                i += 1
    return out


def speed_threshold(rs):
    """Valley of the log target-speed histogram of the given runs (engaged samples)."""
    v = []
    for r in rs:
        tg = r.csv('TOUCH_PATH.csv')
        for a, b in track.strokes(tg['t']):
            g = np.arange(a, b, 0.01)
            p = np.column_stack([np.interp(g, tg['t'], tg[c] * 1000) for c in 'xyz'])
            v.extend(np.convolve(np.linalg.norm(np.gradient(p, 0.01, axis=0), axis=1), np.ones(20) / 20, 'same'))
    lv = np.log10(np.clip(v, 1e-3, None))
    h, e = np.histogram(lv, bins=60)
    hs = np.convolve(h, np.ones(5) / 5, 'same')
    peaks = [i for i in range(1, len(hs) - 1) if hs[i] >= hs[i - 1] and hs[i] >= hs[i + 1]]
    peaks = sorted(peaks, key=lambda i: -hs[i])[:2]
    if len(peaks) == 2:
        a, b = sorted(peaks)
        k = a + int(np.argmin(hs[a:b + 1]))
        return 10 ** ((e[k] + e[k + 1]) / 2), 'valley between the two modes of the speed histogram'
    return float(np.percentile(v, 25)), 'no valley: 25th percentile'


def signals(run, K):
    b = bags.load(run)
    off = bags.to_run_clock(run, b)
    js = b['/joint_states']
    t = js['th'] - (np.median(js['th'] - js['tr']) * 0) - 0
    t = js['tr'] - off
    g = np.arange(t[0], t[-1], 1 / FS)
    qd = np.column_stack([np.interp(g, t, js['qd'][:, k]) for k in range(6)])
    q = np.column_stack([np.interp(g, t, js['q'][:, k]) for k in range(6)])
    cmd = None
    for topic in ('/joint_velocity_limiter/commands', '/forward_velocity_controller/commands'):
        if topic in b and run.mode != 'trajectory':
            c = b[topic]
            tc = c['tr'] - off
            cmd = np.column_stack([np.interp(g, tc, c['data'][:, k]) for k in range(6)])
            break
    st = b['/phantom/state']
    ts = st['tr'] - off
    sty = np.column_stack([np.interp(g, ts, st['p'][:, k] * 1000) for k in range(3)])
    return g, q, qd, cmd, sty


def to_tool(K, q, qd, every=10):
    """Tool linear (mm/s) and angular (deg/s) velocity from joint velocity, Jacobian refreshed every `every` samples."""
    v = np.zeros((len(q), 6))
    J = None
    for i in range(len(q)):
        if i % every == 0:
            J = kin.jacobian(K, q[i])
        v[i] = J @ qd[i]
    v[:, :3] *= 1000
    v[:, 3:] = np.degrees(v[:, 3:])
    return v


def segments(g, X, wins):
    """Welch segments (detrended by the 3 Hz low-pass) from all moving windows, as a list of (NPER, ch) arrays."""
    segs = []
    for a, b in wins:
        m = (g >= a) & (g <= b)
        if m.sum() < NPER:
            continue
        x = X[m]
        x = x - sosfiltfilt(SOS_LP, x, axis=0)
        for s in range(0, len(x) - NPER + 1, NPER // 2):
            segs.append(x[s:s + NPER])
    return segs


def psd_of(segs):
    f, P = welch(np.stack(segs), fs=FS, nperseg=NPER, axis=1)       # (nseg, nf, ch)
    return f, P


def band(f, P, lo, hi):
    m = (f >= lo) & (f <= hi)
    return np.trapz(P[..., m, :], f[m], axis=-2).sum(-1)          # summed over channels -> per segment


def main():
    style()
    K = kin.load('calibrated')
    rs = [r for r in runs.find_runs('V9b/NoTool/*/Scaled5_*_Replay*')]
    thr = {}
    for tr in sorted({r.trajectory for r in rs}):
        thr[tr] = speed_threshold([r for r in rs if r.mode == 'velocity_unfiltered' and r.trajectory == tr])
    sig = {r.key: signals(r, K) for r in rs}
    span = {}
    for tr in thr:
        g0 = [sig[r.key][0] for r in rs if r.trajectory == tr]
        span[tr] = (max(g[0] for g in g0) + 0.5, min(g[-1] for g in g0) - 0.5)
    res = {}
    for r in rs:
        lo_, hi_ = span[r.trajectory]
        wins = [(max(a, lo_), min(b, hi_)) for a, b in moving_windows(r, thr[r.trajectory][0])]
        wins = [(a, b) for a, b in wins if b - a >= 1.0]
        g, q, qd, cmd, sty = sig[r.key]
        tool = to_tool(K, q, qd)
        out = dict(run=r, wins=wins, moving_s=sum(b - a for a, b in wins))
        for name, X in (('joint', np.degrees(qd)), ('tool_lin', tool[:, :3]), ('tool_ang', tool[:, 3:]),
                        ('cmd', np.degrees(cmd) if cmd is not None else None), ('stylus', sty)):
            if X is None:
                continue
            s = segments(g, X, wins)
            if s:
                f, P = psd_of(s)
                out[name] = (f, P)
        res[r.key] = out
        print(r.key, f"{out['moving_s']:.1f} s moving, {len(out['joint'][1]) if 'joint' in out else 0} segments")

    trajs = sorted({r.trajectory for r in rs})
    L = ['# L07 Results: vibration reduction against the unfiltered baseline (threshold (i))', '',
         'Generated by `analysis.py`. V9b/NoTool, 1:5 scale (the only scale with an unfiltered baseline), bare flange, '
         'calibrated kinematics.', '',
         'Moving threshold per trajectory: ' + '; '.join(f'{k} > {v[0]:.2f} mm/s ({v[1]})' for k, v in thr.items())
         + '; windows ≥ 1 s after trimming 0.25 s each end, clipped to the time span every run of the trajectory has '
         'in its bag (the bag starts a few seconds after the CSV recorder). '
         'Same windows for every mode of a trajectory because the input is identical.', '',
         '## Moving time and Welch segments per replay', '', '| Run | mode | moving (s) | segments |', '|---|---|---|---|']
    for k, o in res.items():
        L.append(f"| {k} | {o['run'].mode} | {o['moving_s']:.1f} | {len(o['joint'][1]) if 'joint' in o else 0} |")

    def pooled(traj, mode, sig):
        P = [res[k][sig][1] for k in res if res[k]['run'].trajectory == traj and res[k]['run'].mode == mode
             and sig in res[k]]
        if not P:
            return None, None
        return res[next(iter(res))]['joint'][0], np.concatenate(P)

    L += ['', '## Band power reduction vs unfiltered (1 − P_mode / P_unfiltered; 95 % bootstrap CI over segments)', '',
          '| Trajectory | signal | band | ' + ' | '.join(MODE[m]['label'] for m in ORDER[1:]) + ' |',
          '|---|---|---|' + '---|' * (len(ORDER) - 1)]
    summary = {}
    for tr in trajs:
        for sig, slab in (('joint', 'joint velocity'), ('tool_lin', 'tool linear'), ('tool_ang', 'tool angular'),
                          ('cmd', 'command')):
            f, Pb = pooled(tr, 'velocity_unfiltered', sig)
            if Pb is None:
                continue
            for bn, (lo, hi) in BANDS.items():
                cells = []
                for m in ORDER[1:]:
                    _, Pm = pooled(tr, m, sig)
                    if Pm is None:
                        cells.append('–'); continue
                    bb, bm = band(f, Pb, lo, hi), band(f, Pm, lo, hi)
                    red = 1 - bm.mean() / bb.mean()
                    boots = [1 - RNG.choice(bm, len(bm)).mean() / RNG.choice(bb, len(bb)).mean() for _ in range(2000)]
                    lo_, hi_ = np.percentile(boots, [2.5, 97.5])
                    cells.append(f'{100 * red:.0f} % [{100 * lo_:.0f}, {100 * hi_:.0f}]')
                    summary[(tr, sig, bn, m)] = (red, lo_, hi_)
                L.append(f'| {tr} | {slab} | {bn} | ' + ' | '.join(cells) + ' |')
    L += ['', '## Verdict on threshold (i) (primary band, joint velocity and tool space)', '',
          'Threshold (i) is a 30 % cut in vibration amplitude, i.e. ≥ 51 % in band power (0.7² = 0.49), '
          'judged on the mean band power over the replays of each condition; the CI is reported alongside.', '']
    for m in ORDER[1:]:
        v = [(k[0], k[1], summary[k]) for k in summary if k[2] == PRIMARY and k[3] == m and k[1] != 'cmd']
        if v:
            lows = [x[2][1] for x in v]
            reds = [x[2][0] for x in v]
            L.append(f"- {MODE[m]['label']}: reduction of the mean {100 * min(reds):.0f}–{100 * max(reds):.0f} %, "
                     f"lowest CI bound {100 * min(lows):.0f} % → {'meets' if min(reds) >= REQ else 'does NOT meet'} "
                     f"≥ {100 * REQ:.0f} % (judged on the mean) in every trajectory/signal.")
    lowf = []
    for tr in trajs:
        f, Pb = pooled(tr, 'velocity_unfiltered', 'tool_lin')
        for m in ('velocity_optimised', 'position'):
            _, Pm = pooled(tr, m, 'tool_lin')
            if Pm is not None:
                r_ = Pm.mean(0).sum(-1) / Pb.mean(0).sum(-1)
                mm = (f >= 1) & (f <= 8)
                lowf.append(f"{tr} {MODE[m]['label']}: max {r_[mm].max():.1f}× at {f[mm][np.argmax(r_[mm])]:.0f} Hz")
    L += ['', 'Joint-side and flange (tool0) measurements: the distal tip was not tracked on these runs.', '',
          '## Finding: more low-frequency motion with filtering', '',
          'Below ~8 Hz the filtered modes carry *more* flange motion than unfiltered (ratio > 1 in `psd_ratio`), and the '
          'same excess is already in the joint-velocity command. A low-pass filter inside a proportional loop adds phase '
          'lag that can make the closed loop peak below the cutoff; folder L06 tests this. Peak ratio 1–8 Hz: '
          + '; '.join(lowf) + '. This is why the 8–12 Hz flange band does not fall on the slow trajectory (CI wide: 4 '
          'Welch segments per replay).', '',
          '## Prior claims checked', '', '- "15–25 Hz joint-velocity power down 76–88 % (position) and 64–68 % '
          '(filtered velocity)": see the joint velocity rows.', '', '## Figures', '']

    # fig 1: PSD ratio
    fig, axes = plt.subplots(1, len(trajs), figsize=(FULL, 2.6), sharey=True)
    for ax, tr, letter in zip(np.atleast_1d(axes), trajs, 'abcdef'):
        f, Pb = pooled(tr, 'velocity_unfiltered', 'tool_lin')
        for m in ORDER[1:]:
            _, Pm = pooled(tr, m, 'tool_lin')
            if Pm is None:
                continue
            ratio = Pm.mean(0).sum(-1) / Pb.mean(0).sum(-1)
            boots = np.array([RNG.choice(Pm, len(Pm)).mean(0).sum(-1) / RNG.choice(Pb, len(Pb)).mean(0).sum(-1)
                              for _ in range(300)])
            lo, hi = np.percentile(boots, [2.5, 97.5], axis=0)
            mm = (f >= 1) & (f <= 100)
            ax.plot(f[mm], ratio[mm], color=MODE[m]['color'], label=MODE[m]['label'])
            ax.fill_between(f[mm], lo[mm], hi[mm], color=MODE[m]['color'], alpha=0.18, lw=0)
        ax.axvspan(15, 25, color=MUTED, alpha=0.15, lw=0)
        ax.axhline(1 - REQ, color=INK2, ls='--', lw=0.8, label=f'Threshold (−{100 * REQ:.0f} % power)')
        ax.axhline(1.0, color=MUTED, lw=0.6)
        ax.set_xscale('log'); ax.set_yscale('log'); ax.set_ylim(0.01, 5)
        panel(ax, letter, tr.replace('Scaled5_', '1:5 ').replace('Fast', 'fast').replace('Slow', 'slow') + ' trajectory')
        ax.set_xlabel('Frequency (Hz)')
    np.atleast_1d(axes)[0].set_ylabel('Flange power ratio to unfiltered velocity')
    np.atleast_1d(axes)[0].legend(loc='lower left')
    L.append(f"- `figures/{os.path.basename(save(fig, HERE, 'psd_ratio'))}`")

    # fig 2: spectra chain for the fast trajectory: stylus -> command -> arm (tool linear)
    tr = [t for t in trajs if 'Fast' in t][0] if any('Fast' in t for t in trajs) else trajs[0]
    fig, axes = plt.subplots(3, 1, figsize=(HALF * 1.6, 4.4), sharex=True)
    for ax, (sig, lab) in zip(axes, (('stylus', 'Stylus PSD\n(mm²/Hz)'), ('cmd', 'Command PSD\n((°/s)²/Hz)'),
                                     ('tool_lin', 'Flange PSD\n((mm/s)²/Hz)'))):
        for m in ('velocity_unfiltered', 'position'):
            f, Pm = pooled(tr, m, sig)
            if Pm is None:
                continue
            mm = (f >= 1) & (f <= 100)
            ax.plot(f[mm], Pm.mean(0).sum(-1)[mm], color=MODE[m]['color'], label=MODE[m]['label'], lw=1)
        ax.axvspan(15, 25, color=MUTED, alpha=0.15, lw=0); ax.axvspan(8, 12, color=MUTED, alpha=0.07, lw=0)
        ax.set_yscale('log'); ax.set_xscale('log'); ax.set_ylabel(lab)
    axes[0].legend()
    axes[-1].set_xlabel('Frequency (Hz)')
    L.append(f"- `figures/{os.path.basename(save(fig, HERE, 'spectra_chain'))}`")

    # fig 3: paired dots, primary band, tool linear
    fig, ax = plt.subplots(figsize=(HALF * 1.3, 2.4))
    for i, m in enumerate(ORDER):
        for tr in trajs:
            f, P = pooled(tr, m, 'tool_lin')
            if P is None:
                continue
            ax.scatter(i, band(f, P, 15, 25).mean(), color=MODE[m]['color'], marker=MODE[m]['marker'], s=26, zorder=3)
    for tr in trajs:
        ys = []
        for i, m in enumerate(ORDER):
            f, P = pooled(tr, m, 'tool_lin')
            ys.append((i, band(f, P, 15, 25).mean()) if P is not None else None)
        ys = [y for y in ys if y]
        ax.plot(*zip(*ys), color=MUTED, lw=0.7)
        b0 = ys[0][1]
        ax.plot([0, len(ORDER) - 1], [0.7 * b0, 0.7 * b0], color=INK2, ls=':', lw=0.6)
    ax.set_yscale('log'); ax.set_xticks(range(len(ORDER))); ax.set_xticklabels([MODE[m]['label'] for m in ORDER],
                                                                               rotation=20)
    ax.set_ylabel('Flange power, 15–25 Hz ((mm/s)²)')
    L.append(f"- `figures/{os.path.basename(save(fig, HERE, 'band_power_paired'))}`")
    open(os.path.join(HERE, 'RESULTS.md'), 'w').write('\n'.join(L) + '\n')
    print('\n'.join(L[-40:]))


if __name__ == '__main__':
    main()
