#!/usr/bin/env python3
"""Quick-change exchange: full 6-DOF re-seat, the tracker's own floor, bedding-in and swap times (R8.2, 4.4.1).

    cd ~/ros2_ws/src/MacroMicroSurgicalRobot
    python3 thesis/planning/analysis/exchange_6dof.py          # numbers + thesis/plotting/out/exchange_reseat.{png,pdf}

Extends exchange_repeatability.py (same still-segment rule, so the segment count and tip numbers match it):
- the tracker floor measured inside each recording: 5 s medians of the 30 s still periods at each end;
- the re-seat rotation in the wand frame (not just the shaft tilt), and the tip error it alone would cause on a
  tool of length L (the 436 mm micro tool);
- bedding-in: the first 3 swaps of each set against the rest;
- pooled n = 60 statistics against threshold (ii).
Camera-frame data, so it measures the coupling with the arm and camera assumed still (PROTOCOL.md).
Only reads logs/Exchange.
"""
import csv
import glob
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, os.path.join(PKG, 'thesis', 'plotting'))
from common import INK2, MUTED, save, style  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

SETS = sorted(glob.glob(os.path.join(PKG, 'logs', 'Exchange', '2026*_QC_*')))
COLORS = [('#2a78d6', 'o'), ('#eb6834', 's'), ('#1baf7a', '^')]   # slots 1-3, as in plotting/common.py
MICRO_TOOL_MM = 436.0      # micro tool tip from tool0 (config/calibrations/micro_tool_2026-11-06_134732.json)


def quat_to_R(q):
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def load(d):
    rows = list(csv.DictReader(open(os.path.join(d, 'NDI_PATH.csv'))))
    col = lambda k: np.array([float(r[k]) if r[k] else np.nan for r in rows])
    t = col('t')
    vis = np.array([r['visible'] == '1' for r in rows])
    P = np.c_[col('tip_x'), col('tip_y'), col('tip_z')]
    Q = np.c_[col('q0'), col('qx'), col('qy'), col('qz')]
    return t, vis, P, Q


def still_segments(t, vis, P):
    """Same rule as exchange_repeatability.py: >= 2 s, visible, tip moves < 0.3 mm over every 1 s window."""
    still = np.zeros(len(t), bool)
    for i in range(len(t)):
        if not vis[i]:
            continue
        j = np.searchsorted(t, t[i] + 1.0)
        seg = P[i:j][vis[i:j]]
        if len(seg) > 30 and np.ptp(seg, 0).max() < 0.3 and vis[i:j].mean() > 0.9:
            still[i:j] = True
    segs, i = [], 0
    while i < len(t):
        if still[i]:
            j = i
            while j < len(t) and still[j]:
                j += 1
            if t[j - 1] - t[i] >= 2.0:
                segs.append((i, j))
            i = j
        else:
            i += 1
    return segs


def analyse(d):
    t, vis, P, Q = load(d)
    segs = still_segments(t, vis, P)
    med = lambda a, b, X: np.nanmedian(X[a:b][vis[a:b]], 0)
    # tracker floor: 5 s medians inside the long still periods at both ends
    floor_dev = []
    for a, b in (segs[0], segs[-1]):   # each end separately: they are different seats
        w, k = [], a
        while k < b:
            e = min(np.searchsorted(t, t[k] + 5.0), b)
            if t[e - 1] - t[k] > 4:
                w.append(med(k, e, P))
            k = e
        w = np.array(w)
        floor_dev.extend(np.linalg.norm(w - w.mean(0), axis=1))
    floor_dev = np.array(floor_dev)
    C = np.array([med(a, b, P) for a, b in segs])
    dev = np.linalg.norm(C - np.median(C, 0), axis=1)
    Rs = [quat_to_R(med(a, b, Q)) for a, b in segs]
    rv = []
    for R in Rs:      # small-angle rotation vector of each seat relative to the first, wand frame (deg)
        dR = Rs[0].T @ R
        rv.append(np.degrees([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]]) / 2)
    rv = np.array(rv)
    rot = np.linalg.norm(rv - np.median(rv, 0), axis=1)
    gaps = np.array([t[segs[k][0]] - t[segs[k - 1][1] - 1] for k in range(1, len(segs))])
    # slow drift over the set (tracker warm-up or the arm warming): linear fit of seat position vs time
    tc = np.array([t[a] for a, b in segs])
    A = np.c_[tc, np.ones_like(tc)]
    coef, *_ = np.linalg.lstsq(A, C, rcond=None)
    res = C - A @ coef
    r2 = 1 - (res ** 2).sum() / ((C - C.mean(0)) ** 2).sum()
    detr = np.linalg.norm(res, axis=1)
    return dict(name=os.path.basename(d), n=len(segs), dev=dev, rot=rot, gaps=gaps, detr=detr,
                drift_um_min=np.linalg.norm(coef[0]) * 60e3, drift_r2=r2,
                floor_rms=np.sqrt((floor_dev ** 2).mean()), floor_max=floor_dev.max(),
                floor_sd=np.nanstd(P[segs[0][0]:segs[0][1]][vis[segs[0][0]:segs[0][1]]], 0))


def sequence_checks(d, rng):
    """Are the seats independent, and is any of it measurement artefact? (added 2026-11-13, report 4.4.1)

    - direction correlation: mean cosine between successive seat offsets vs a shuffled-order null;
    - split: independent swap-to-swap part = rms(successive differences)/sqrt(2), slow part = the rest;
    - change-point: largest persistent mean shift in the seat sequence, permutation p;
    - wander of the measured point in the still periods (nothing touched), um/min;
    - edge trim 0-2 s and mid-seat steps (1 s trimmed both ends) vs steps inside the still periods.
    """
    t, vis, P, Q = load(d)
    segs = still_segments(t, vis, P)
    C = np.array([np.nanmedian(P[a:b][vis[a:b]], 0) for a, b in segs])
    D = C - np.median(C, 0)

    def cos1(X):
        u = X / np.linalg.norm(X, axis=1, keepdims=True)
        return np.mean(np.sum(u[1:] * u[:-1], 1))
    obs = cos1(D)
    null = np.array([cos1(D[rng.permutation(len(D))]) for _ in range(5000)])
    p_cos = (np.sum(null >= obs) + 1) / 5001

    def split(X):
        fast = np.sqrt((np.linalg.norm(np.diff(X, axis=0), axis=1) ** 2).mean() / 2)
        tot = np.sqrt((np.linalg.norm(X - X.mean(0), axis=1) ** 2).mean())
        return tot, fast, np.sqrt(max(tot ** 2 - fast ** 2, 0))

    def best_split(X):
        best = (0, None)
        for k in range(3, len(X) - 3):
            s = np.linalg.norm(X[:k].mean(0) - X[k:].mean(0)) * np.sqrt(k * (len(X) - k) / len(X))
            if s > best[0]:
                best = (s, k)
        return best
    s, k = best_split(D)
    cnull = np.array([best_split(D[rng.permutation(len(D))])[0] for _ in range(2000)])
    shift = np.linalg.norm(D[:k].mean(0) - D[k:].mean(0))

    wander = []
    for a, b in (segs[0], segs[-1]):
        m = vis[a:b]
        tt, X = t[a:b][m], P[a:b][m]
        sl = np.linalg.lstsq(np.c_[tt - tt.mean(), np.ones_like(tt)], X, rcond=None)[0][0]
        wander.append(np.linalg.norm(sl) * 60e3)

    trims = []
    for trim in (0.0, 1.0, 2.0):
        Ct = []
        for a, b in segs:
            m = (t[a:b] >= t[a] + trim) & (t[a:b] <= t[b - 1] - trim) & vis[a:b]
            Ct.append(np.nanmedian(P[a:b][m], 0))
        dv = np.linalg.norm(np.array(Ct) - np.median(Ct, 0), axis=1)
        trims.append((trim, np.sqrt((dv ** 2).mean()), dv.max()))

    def maxstep(lo, hi, w=0.75):
        best = 0.0
        for c in np.arange(lo + w, hi - w, 0.1):
            i0, i1, i2 = np.searchsorted(t, [c - w, c, c + w])
            m0, m1 = vis[i0:i1], vis[i1:i2]
            if m0.sum() >= 10 and m1.sum() >= 10:
                best = max(best, np.linalg.norm(np.median(P[i0:i1][m0], 0) - np.median(P[i1:i2][m1], 0)))
        return best
    ref = []
    for a, b in (segs[0], segs[-1]):
        c = t[a]
        while c + 4 < t[b - 1]:
            ref.append(maxstep(c, c + 4))
            c += 4
    inner = [maxstep(t[a] + 1.0, t[b - 1] - 1.0) for a, b in segs[1:-1]]

    print(f"== {os.path.basename(d)}: sequence checks")
    print(f"  successive offsets: mean cosine {obs:+.2f} (shuffled {null.mean():+.2f}, p = {p_cos:.3f})")
    for lab, X in (('all seats', C), ('from seat 4', C[4:])):
        tot, fast, slow = split(X)
        print(f"  {lab}: total {tot * 1e3:.1f} um rms = independent {fast * 1e3:.1f} (+) slow {slow * 1e3:.1f} um")
    print(f"  change-point: {shift * 1e3:.0f} um before seat {k}, p = {(np.sum(cnull >= s) + 1) / 2001:.3f}")
    print(f"  still-period wander (nothing touched): {wander[0]:.1f} / {wander[1]:.1f} um/min (start / end)")
    print('  edge trim: ' + '; '.join(f"{tr:.0f} s -> rms {r * 1e3:.1f}, max {mx * 1e3:.1f} um" for tr, r, mx in trims))
    print(f"  mid-seat steps: max {max(inner) * 1e3:.0f} um vs still periods max {max(ref) * 1e3:.0f} um; "
          f"seats above: {[i + 1 for i, v in enumerate(inner) if v > max(ref)]}")


def main():
    res = [analyse(d) for d in SETS]
    rng = np.random.default_rng(1)
    for d in SETS:
        sequence_checks(d, rng)
    for r in res:
        dv, ro = r['dev'], r['rot']
        print(f"== {r['name']}: {r['n']} seated states ({r['n'] - 1} swaps)")
        print(f"  floor: per-sample SD {np.round(r['floor_sd'], 3)} mm; 5 s medians rms {r['floor_rms']:.3f}, "
              f"max {r['floor_max']:.3f} mm")
        print(f"  tip re-seat: rms {np.sqrt((dv ** 2).mean()):.3f}, max {dv.max():.3f} mm "
              f"({np.sqrt((dv ** 2).mean()) / r['floor_rms']:.0f}x floor)")
        print(f"  rotation re-seat: rms {np.sqrt((ro ** 2).mean()):.4f}, max {ro.max():.4f} deg -> "
              f"{np.radians(ro.max()) * MICRO_TOOL_MM:.3f} mm at {MICRO_TOOL_MM:.0f} mm")
        print(f"  bedding-in: first 3 mean {dv[:3].mean():.3f} vs rest {dv[3:].mean():.3f} mm")
        de = r['detr']
        print(f"  drift: {r['drift_um_min']:.1f} um/min, explains {r['drift_r2']:.0%} of the spread; "
              f"detrended re-seat rms {np.sqrt((de ** 2).mean()):.3f}, max {de.max():.3f} mm")
        g = r['gaps']
        print(f"  swap time: median {np.median(g):.1f}, IQR {np.percentile(g, 25):.1f}-{np.percentile(g, 75):.1f}, "
              f"max {g.max():.1f} s")
    # pooled: the 60 swaps (seated states after the first of each set)
    dv = np.concatenate([r['dev'][1:] for r in res])
    ro = np.concatenate([r['rot'][1:] for r in res])
    g = np.concatenate([r['gaps'] for r in res])
    late = np.concatenate([r['dev'][4:] for r in res])
    print(f"== pooled, n = {len(dv)} swaps")
    print(f"  tip re-seat: mean {dv.mean():.3f}, SD {dv.std(ddof=1):.3f}, rms {np.sqrt((dv ** 2).mean()):.3f}, "
          f"p95 {np.percentile(dv, 95):.3f}, max {dv.max():.3f} mm; after the first 3 swaps: p95 "
          f"{np.percentile(late, 95):.3f}, max {late.max():.3f} mm")
    print(f"  rotation: p95 {np.percentile(ro, 95):.4f}, max {ro.max():.4f} deg -> "
          f"{np.radians(ro.max()) * MICRO_TOOL_MM:.3f} mm at {MICRO_TOOL_MM:.0f} mm")
    print(f"  swap time: median {np.median(g):.1f}, IQR {np.percentile(g, 25):.1f}-{np.percentile(g, 75):.1f}, "
          f"max {g.max():.1f} s (threshold 20 s)")
    print(f"  floor (5 s medians): {min(r['floor_rms'] for r in res):.3f}-{max(r['floor_rms'] for r in res):.3f} "
          f"mm rms")

    style()
    fig, ax = plt.subplots(figsize=(6.0, 2.6))
    for k, r in enumerate(res):
        c, m = COLORS[k]
        x = np.arange(len(r['dev']))
        ax.plot(x, r['dev'] * 1000, color=c, marker=m, ms=4.5, lw=1.0,
                label=f"Set {k + 1}", markeredgecolor='white', markeredgewidth=0.6)
    fl = max(r['floor_max'] for r in res) * 1000
    ax.axhspan(0, fl, color=MUTED, alpha=0.18, lw=0)
    ax.text(20.3, fl, 'tracker floor\n(5 s average)', fontsize=7, color=INK2, va='bottom', ha='right')
    ax.set_xlabel('Seated state (0 = before the first swap)')
    ax.set_ylabel('Tip offset from set median (µm)')
    ax.set_xlim(-0.5, 20.5)
    ax.set_ylim(0, None)
    ax.set_xticks(range(0, 21, 2))
    ax.legend(ncol=3, loc='upper right')
    save(fig, 'exchange_reseat', note='NDI Vega, camera frame, wand on the tool plate; 20 swaps per set. '
         'Source: thesis/planning/analysis/exchange_6dof.py')


if __name__ == '__main__':
    main()
