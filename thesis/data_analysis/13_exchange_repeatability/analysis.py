#!/usr/bin/env python3
"""13 Quick-change coupling: re-seat repeatability and exchange time per version (V1, V2, V3).

    cd thesis/data_analysis && python3 13_exchange_repeatability/analysis.py
Reads logs/Exchange/*_QC_V*/NDI_PATH.csv (read-only). Writes figures/ and RESULTS.md here.
"""
import glob
import os
import sys

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from common import runs  # noqa: E402
from common.style import FULL, HALF, INK2, MUTED, VERSION, save, style, panel  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Ellipse  # noqa: E402

RNG = np.random.default_rng(13)
STILL_RANGE_MM, STILL_WIN_S, MIN_SEAT_S = 0.3, 1.0, 2.0      # still rule (tip range within any 1 s window)
AWAY_MM = 1.0                     # "left the seat" / "back at the seat"
SETTLE_WIN = 0.5                  # settled: 0.5 s rolling median inside the tracker's own 99th-percentile stray
TRIM_S = 1.0                      # trimmed both ends for the bump test
# Tool geometry for the tip-equivalent error is READ from config/tools.yaml, not typed in:
#  - the wand-holder tool whose calibration date is the latest on or before the session (the exchange logs do not
#    record which tool was mounted); the other holder tool is reported as a sensitivity check, because the coupling
#    stack between tool0 and the holder may differ between coupling versions;
#  - the micro module (tool named 'Micro module') for the micro-tip lever.
TOOLS = yaml.safe_load(open(os.path.join(runs.PKG, 'config', 'tools.yaml')))['tools']


def holder_tools():
    return {n: t for n, t in TOOLS.items() if 'wand' in t['name'].lower() or 'holder' in t['name'].lower()}


def tip_dist(tool):
    return float(np.linalg.norm(tool['tip_tool0_mm']))


def holder_for(date):
    """(number, tool) of the latest holder calibration on or before `date` (YYYY-MM-DD), else the earliest."""
    hs = sorted(holder_tools().items(), key=lambda kv: str(kv[1]['date']))
    ok = [kv for kv in hs if str(kv[1]['date']) <= date]
    return ok[-1] if ok else hs[0]


MICRO = next(t for t in TOOLS.values() if t['name'].lower().startswith('micro'))
MICRO_TIP_FROM_TOOL0_MM = tip_dist(MICRO)
LABELS = {'V1': 'C1 printed', 'V2': 'C2 full disc', 'V3': 'C3 wide base'}  # coupling versions are C1-C3 in the report (V1-V9 are stack versions)


def load(d):
    r = runs.read_csv(os.path.join(d, 'NDI_PATH.csv'))
    t = r['t']
    vis = r['visible'] == 1
    P = np.c_[r['tip_x'], r['tip_y'], r['tip_z']]
    A = np.c_[r['axis_x'], r['axis_y'], r['axis_z']]
    Q = np.c_[r['qx'], r['qy'], r['qz'], r['q0']]          # scipy order x, y, z, w
    return t, vis, P, A, Q, r['fit_error']


def still_segments(t, vis, P):
    still = np.zeros(len(t), bool)
    for i in range(len(t)):
        if not vis[i]:
            continue
        j = np.searchsorted(t, t[i] + STILL_WIN_S)
        seg = P[i:j][vis[i:j]]
        if len(seg) > 30 and np.ptp(seg, 0).max() < STILL_RANGE_MM and vis[i:j].mean() > 0.9:
            still[i:j] = True
    segs, i = [], 0
    while i < len(t):
        if still[i]:
            j = i
            while j < len(t) and still[j]:
                j += 1
            if t[j - 1] - t[i] >= MIN_SEAT_S:
                segs.append((i, j))
            i = j
        else:
            i += 1
    return segs


def rotmean(Rs):
    """Chordal L2 mean of rotations."""
    M = sum(R.as_matrix() for R in Rs)
    U, _, Vt = np.linalg.svd(M)
    return Rotation.from_matrix(U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt)


def analyse(d):
    name = os.path.basename(d)
    v = name.split('_')[-1]
    t, vis, P, A, Q, fe = load(d)
    segs = still_segments(t, vis, P)
    med = lambda a, b, X: np.nanmedian(X[a:b][vis[a:b]], 0)

    # ---- settling threshold from the tracker itself: in the long still periods, how far a 0.5 s rolling median
    #      strays from the 5 s median (99th percentile). A seat is settled once its rolling median stays inside it.
    k = int(SETTLE_WIN * 60)
    def rolling_med(pp):
        return np.array([np.median(pp[max(0, i - k + 1):i + 1], 0) for i in range(len(pp))])
    stray = []
    for a, b in (segs[0], segs[-1]):
        c0 = a
        while c0 < b:
            e = min(np.searchsorted(t, t[c0] + 5.0), b)
            pp = P[c0:e][vis[c0:e]]
            if len(pp) > 4 * 60:
                rm = rolling_med(pp)[k:]
                stray.extend(np.linalg.norm(rm - np.median(pp, 0), axis=1))
            c0 = e
    settle_thr = max(np.percentile(stray, 99), 0.005)
    settle, seats_P, seats_R, seats_t, fit = [], [], [], [], []
    for a, b in segs:
        tt, pp = t[a:b][vis[a:b]], P[a:b][vis[a:b]]
        p_end = np.median(pp[min(len(pp) // 2, 3 * 60):min(len(pp), 6 * 60)], 0)   # reference: 3-6 s (or 2nd half)
        rm = rolling_med(pp)
        off = np.linalg.norm(rm - p_end, axis=1) > settle_thr
        half = min(len(off) // 2, 3 * 60)   # settling = the first 3 s at most (end bleed and the slow wander of the
                                             # 30 s still periods are not settling)
        bad = np.where(off[:half])[0]
        ts = tt[bad[-1] + 1] - tt[0] if len(bad) else 0.0
        settle.append(ts)
        s0 = np.searchsorted(t, t[a] + ts)
        seats_P.append(med(s0, b, P))
        seats_R.append(rotmean(Rotation.from_quat(Q[s0:b][vis[s0:b]][::3])))
        seats_t.append(t[a])
        fit.append(np.nanmedian(fe[s0:b][vis[s0:b]]))
    C = np.array(seats_P)
    tc = np.array(seats_t)
    Rm = rotmean(seats_R)

    # ---- offsets in the wand frame (z ~ shaft ~ coupling axis); rotations as rotation vectors in the wand frame
    off_w = np.array([Rm.inv().apply(p - np.median(C, 0)) for p in C]) * 1000          # um
    rv = np.array([(Rm.inv() * R).as_rotvec() for R in seats_R])
    rv = np.degrees(rv - np.median(rv, 0))                                              # deg, about median
    # tip-equivalent at tool0 and at the micro tool tip: rigid motion of each seat applied to those points
    ax_w = np.array([0.0, 0.0, 1.0])
    def point_disp(L_from_tip):      # L along wand +z from the tip (+z points from the tip back to the markers/tool0)
        out = []
        for i in range(len(C)):
            w = np.radians(rv[i])
            out.append(off_w[i] + np.cross(w, ax_w * L_from_tip) * 1000)
        return np.array(out)
    date = str(yaml.safe_load(open(os.path.join(d, 'NDI_META.yaml')))['date'])[:10]
    hn, ht = holder_for(date)
    lever = tip_dist(ht)
    at_tool0 = point_disp(lever)
    at_micro = point_disp(lever - MICRO_TIP_FROM_TOOL0_MM)
    alt = {n: np.linalg.norm(point_disp(tip_dist(x) - MICRO_TIP_FROM_TOOL0_MM), axis=1).max()
           for n, x in holder_tools().items()}

    # ---- floors from the still periods at both ends (each end separately)
    floor_meds, wander = [], []
    for a, b in (segs[0], segs[-1]):
        w, k = [], a
        while k < b:
            e = min(np.searchsorted(t, t[k] + 5.0), b)
            if t[e - 1] - t[k] > 4:
                w.append(med(k, e, P))
            k = e
        w = np.array(w)
        floor_meds.extend(np.linalg.norm(w - w.mean(0), axis=1))
        m = vis[a:b]
        sl = np.linalg.lstsq(np.c_[t[a:b][m] - t[a:b][m].mean(), np.ones(m.sum())], P[a:b][m], rcond=None)[0][0]
        wander.append(np.linalg.norm(sl) * 60e3)
    floor_rms = np.sqrt(np.mean(np.square(floor_meds))) * 1000
    rot_floor = []
    for a, b in (segs[0], segs[-1]):
        w, k2 = [], a
        while k2 < b:
            e = min(np.searchsorted(t, t[k2] + 5.0), b)
            if t[e - 1] - t[k2] > 4:
                w.append(rotmean(Rotation.from_quat(Q[k2:e][vis[k2:e]][::3])))
            k2 = e
        m0 = rotmean(w)
        rot_floor.extend(np.degrees([(m0.inv() * x).magnitude() for x in w]))
    rot_floor_rms = np.sqrt(np.mean(np.square(rot_floor)))
    sample_sd = np.nanstd(P[segs[0][0]:segs[0][1]][vis[segs[0][0]:segs[0][1]]], 0) * 1000

    # ---- spread components
    dev = np.linalg.norm(C - np.median(C, 0), axis=1) * 1000
    def split(X):
        fast = np.sqrt((np.linalg.norm(np.diff(X, axis=0), axis=1) ** 2).mean() / 2)
        tot = np.sqrt((np.linalg.norm(X - X.mean(0), axis=1) ** 2).mean())
        return tot * 1000, fast * 1000, np.sqrt(max(tot ** 2 - fast ** 2, 0)) * 1000
    tot, fast, slow = split(C)
    tot4, fast4, slow4 = split(C[4:])
    A_ = np.c_[tc, np.ones_like(tc)]
    coef = np.linalg.lstsq(A_, C, rcond=None)[0]
    res = C - A_ @ coef
    drift = np.linalg.norm(coef[0]) * 60e3
    drift_r2 = 1 - (res ** 2).sum() / ((C - C.mean(0)) ** 2).sum()
    # direction correlation of successive offsets vs shuffled order
    D = C - np.median(C, 0)
    def cos1(X):
        u = X / np.linalg.norm(X, axis=1, keepdims=True)
        return np.mean(np.sum(u[1:] * u[:-1], 1))
    cobs = cos1(D)
    cnull = np.array([cos1(D[RNG.permutation(len(D))]) for _ in range(5000)])
    p_cos = (np.sum(cnull >= cobs) + 1) / 5001
    # change point
    def best_split(X):
        best = (0, None)
        for k in range(3, len(X) - 3):
            s = np.linalg.norm(X[:k].mean(0) - X[k:].mean(0)) * np.sqrt(k * (len(X) - k) / len(X))
            best = max(best, (s, k))
        return best
    s, kcp = best_split(D)
    cpn = np.array([best_split(D[RNG.permutation(len(D))])[0] for _ in range(2000)])
    p_cp = (np.sum(cpn >= s) + 1) / 2001
    cp_shift = np.linalg.norm(D[:kcp].mean(0) - D[kcp:].mean(0)) * 1000

    # ---- bumps: max step between adjacent 0.75 s medians, seats trimmed 1 s each end vs still periods
    def maxstep(lo, hi, w=0.75):
        best = 0.0
        for c in np.arange(lo + w, hi - w, 0.1):
            i0, i1, i2 = np.searchsorted(t, [c - w, c, c + w])
            m0, m1 = vis[i0:i1], vis[i1:i2]
            if m0.sum() >= 10 and m1.sum() >= 10:
                best = max(best, np.linalg.norm(np.median(P[i0:i1][m0], 0) - np.median(P[i1:i2][m1], 0)))
        return best * 1000
    ref = []
    for a, b in (segs[0], segs[-1]):
        c = t[a]
        while c + 4 < t[b - 1]:
            ref.append(maxstep(c, c + 4)); c += 4
    inner = [maxstep(t[a] + TRIM_S, t[b - 1] - TRIM_S) for a, b in segs[1:-1]]
    bumps = [i + 1 for i, x in enumerate(inner) if x > max(ref)]
    # trim sensitivity
    trims = []
    for tr in (0.0, 1.0, 2.0):
        Ct = np.array([med(np.searchsorted(t, t[a] + tr), np.searchsorted(t, t[b - 1] - tr), P) for a, b in segs])
        dv = np.linalg.norm(Ct - np.median(Ct, 0), axis=1) * 1000
        trims.append((tr, np.sqrt(np.mean(dv ** 2)), dv.max()))

    # ---- exchange phases between consecutive seats
    phases = []
    for k in range(1, len(segs)):
        a_end = segs[k - 1][1] - 1
        b0 = segs[k][0]
        p_prev, p_next = C[k - 1], C[k]
        ii = np.arange(a_end, b0 + 1)
        dprev = np.linalg.norm(P[ii] - p_prev, axis=1)
        dnext = np.linalg.norm(P[ii] - p_next, axis=1)
        leave = ii[np.argmax(np.nan_to_num(dprev, nan=1e9) > AWAY_MM)]
        li = int(np.where(ii == leave)[0][0])
        back_idx = np.where(vis[ii] & (dnext < AWAY_MM) & (np.arange(len(ii)) > li))[0]
        back_idx = back_idx[back_idx > li]
        # the last return: after the final excursion beyond AWAY_MM
        far = np.where(np.nan_to_num(dnext, nan=1e9) >= AWAY_MM)[0]
        back = ii[far[-1] + 1] if len(far) and far[-1] + 1 < len(ii) else b0
        # after `leave`, last time it was outside AWAY_MM of the next seat
        out_of_view = np.sum(~vis[leave:back]) / 60.0
        phases.append(dict(unlock=t[leave] - t[a_end], away=t[back] - t[leave], out_of_view=out_of_view,
                           seating=t[b0] - t[back], settle=settle[k], total=t[b0] - t[a_end] + settle[k]))

    return dict(holder=(hn, ht['name'], ht['date'], lever), micro_alt=alt, v=v, name=name, n_seats=len(segs), C=C, tc=tc, dev=dev, off_w=off_w, rv=rv, at_tool0=at_tool0,
                at_micro=at_micro, floor_rms=floor_rms, floor_max=np.max(floor_meds) * 1000, rot_floor=rot_floor_rms, settle_thr=settle_thr * 1000, sample_sd=sample_sd,
                wander=wander, tot=tot, fast=fast, slow=slow, tot4=tot4, fast4=fast4, slow4=slow4, drift=drift,
                drift_r2=drift_r2, cobs=cobs, cnull=cnull.mean(), p_cos=p_cos, cp=(kcp, cp_shift, p_cp),
                bumps=bumps, bump_max=max(inner), bump_ref=max(ref), trims=trims, phases=phases,
                settle=np.array(settle), fit=np.array(fit), date=yaml.safe_load(
                    open(os.path.join(d, 'NDI_META.yaml')))['date'])


def boot_ratio(a, b, n=5000):
    """Bootstrap CI of the ratio of per-swap rms (successive differences) between two versions."""
    da = np.linalg.norm(np.diff(a['C'], axis=0), axis=1)
    db = np.linalg.norm(np.diff(b['C'], axis=0), axis=1)
    r = []
    for _ in range(n):
        x = RNG.choice(da, len(da)); y = RNG.choice(db, len(db))
        r.append(np.sqrt(np.mean(x ** 2)) / np.sqrt(np.mean(y ** 2)))
    obs = np.sqrt(np.mean(da ** 2)) / np.sqrt(np.mean(db ** 2))
    return obs, np.percentile(r, [2.5, 97.5])


# ------------------------------------------------------------------------------------------- figures
def fig_swimlane(res):
    cols = {'unlock': '#8a8983', 'away': '#0072B2', 'seating': '#E69F00', 'settle': '#56B4E9'}
    fig, ax = plt.subplots(figsize=(FULL, 4.6))
    y = 0
    ticks, labels = [], []
    for r in res:
        for k, ph in enumerate(r['phases']):
            x = 0
            for key in ('unlock', 'away', 'seating', 'settle'):
                ax.barh(y, ph[key], left=x, color=cols[key], height=0.8, lw=0)
                if key == 'away' and ph['out_of_view'] > 0:
                    ax.barh(y, ph['out_of_view'], left=x + (ph['away'] - ph['out_of_view']) / 2, height=0.25,
                            color='white', lw=0)
                x += ph[key]
            y += 1
        med = {k: np.median([p[k] for p in r['phases']]) for k in cols}
        x = 0
        for key in cols:
            ax.barh(y + 0.3, med[key], left=x, color=cols[key], height=1.3, lw=0.6, edgecolor='k')
            x += med[key]
        ticks.append(y + 0.3); labels.append(f"{LABELS[r['v']]}\nmedian")
        y += 3
    ax.set_yticks(ticks); ax.set_yticklabels(labels, fontsize=8)
    ax.invert_yaxis()
    ax.set_xlabel('Time from the seat being disturbed (s)')
    ax.axvline(20, color=INK2, ls='--', lw=0.8); ax.text(19.8, -1, '20 s threshold', ha='right', fontsize=8)
    hs = [plt.Rectangle((0, 0), 1, 1, color=c) for c in cols.values()]
    ax.legend(hs, ['Unlock (< 1 mm moved)', 'Away (white: out of view)', 'Seating (within 1 mm → still)',
                   'Settling'], ncol=2, loc='lower center', bbox_to_anchor=(0.5, 1.0))
    ax.set_xlim(0, 21)
    return save(fig, HERE, 'exchange_phases_swimlane')


def fig_radial(res):
    fig, axes = plt.subplots(1, 3, figsize=(FULL, 2.35), sharex=True, sharey=True)
    lim = max(np.abs(r['off_w'][:, :2]).max() for r in res) * 1.15
    for ax, r in zip(axes, res):
        st = VERSION[r['v']]
        o = r['off_w']
        for i in range(len(o) - 1):
            ax.annotate('', xy=o[i + 1, :2], xytext=o[i, :2],
                        arrowprops=dict(arrowstyle='->', color=MUTED, lw=0.5, alpha=0.7))
        sc = ax.scatter(o[:, 0], o[:, 1], c=np.arange(len(o)), cmap='thesis_seq', s=18, marker=st['marker'],
                        edgecolor='white', linewidth=0.4, zorder=3)
        fl = r['floor_rms']
        ax.add_patch(Ellipse((0, 0), 2 * 2.45 * fl, 2 * 2.45 * fl, fc=MUTED, alpha=0.25, lw=0))
        ax.set_aspect('equal')
        ax.set_title(f"{LABELS[r['v']]}\nPer-swap {r['fast']:.0f} µm rms", fontsize=8)
        ax.set_xlabel('Tip, wand $x$ (µm)')
        ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim)
        # inset: the same cloud at the scale of the 1 mm threshold
        ins = ax.inset_axes([0.68, 0.68, 0.3, 0.3])
        ins.add_patch(plt.Circle((0, 0), 1000, fill=False, color=INK2, lw=0.7))
        ins.scatter(o[:, 0], o[:, 1], s=1, color=st['color'])
        ins.set_xlim(-1100, 1100); ins.set_ylim(-1100, 1100); ins.set_aspect('equal')
        ins.set_xticks([]); ins.set_yticks([]); ins.set_title('1 mm', fontsize=6, pad=1)
    axes[0].set_ylabel('Tip, wand $y$ (µm)')
    cb = fig.colorbar(sc, ax=axes, shrink=0.8, pad=0.01)
    cb.set_label('Seat number')
    return save(fig, HERE, 'reseat_radial_plane')


def fig_sequence(res):
    """One row per version on a shared y axis (so sizes compare directly); every seat kept.
    Each row is labelled with its per-swap rms (successive differences), the measure the versions are compared on."""
    fig, axes = plt.subplots(3, 1, figsize=(FULL * 0.85, 3.6), sharex=True, sharey=True)
    top = max(r['dev'].max() for r in res) * 1.12
    for ax, r in zip(axes, res):
        st = VERSION[r['v']]
        x = np.arange(len(r['dev']))
        ax.axvspan(-0.5, 3.5, color=MUTED, alpha=0.12, lw=0)
        ax.axhspan(0, r['floor_max'], color=MUTED, alpha=0.3, lw=0)
        ax.plot(x, r['dev'], color=st['color'], marker=st['marker'], ms=3.5, lw=1)
        k, sh, p = r['cp']
        if p < 0.05:
            ax.axvline(k - 0.5, color=INK2, ls=':', lw=1)
        ax.text(0.6, 0.92, f"{LABELS[r['v']]}: per-swap {r['fast']:.0f} µm rms", transform=ax.transAxes,
                ha='center', va='top', fontsize=8.5, color='#52514e',
                bbox=dict(fc='white', ec='none', alpha=0.85, pad=1))
        ax.set_ylim(0, top); ax.set_yticks([0, 50, 100, 150])
    axes[0].text(1.5, top * 0.92, 'Bedding in', ha='center', va='top', fontsize=8, color=INK2)
    fig.supylabel('Tip offset (µm)', fontsize=9)
    axes[-1].set_xlabel('Seated state (0: before the first exchange)')
    axes[-1].set_xticks(range(0, 21, 2)); axes[-1].set_xlim(-0.5, max(len(r['dev']) for r in res) - 0.5)
    return save(fig, HERE, 'reseat_sequence')


def fig_components(res):
    """Bar length = total rms of the seats; split by variance share (rms add in quadrature, so shares of the
    squared total are the honest split). per-swap coupling = sqrt(per-swap^2 - floor^2)."""
    fig, ax = plt.subplots(figsize=(FULL * 0.85, 1.9))
    for i, r in enumerate(res):
        floor2 = r['floor_rms'] ** 2
        coup2 = max(r['fast'] ** 2 - floor2, 0)
        slow2 = r['slow'] ** 2
        tot2 = floor2 + coup2 + slow2
        total = np.sqrt(tot2)
        x = 0
        for lab, v2, c, h in (('Tracker floor', floor2, '#c9c8c3', None),
                              ('Per-swap (coupling)', coup2, VERSION[r['v']]['color'], None),
                              ('Slow (unattributed)', slow2, '#e4e3df', '///')):
            w = total * v2 / tot2
            ax.barh(i, w, left=x, color=c, height=0.6, edgecolor='white', lw=1, hatch=h, label=lab if i == 0 else None)
            if v2 / tot2 > 0.12:
                ax.text(x + w / 2, i, f'{100 * v2 / tot2:.0f} %', ha='center', va='center', fontsize=8,
                        color='white' if c == VERSION[r['v']]['color'] else INK2)
            x += w
        ax.text(total + 1.5, i, f"{total:.0f} µm (micro-tool tip max "
                f"{np.linalg.norm(r['at_micro'], axis=1).max():.0f} µm)", va='center', fontsize=8, color=INK2)
    ax.set_yticks(range(len(res))); ax.set_yticklabels([LABELS[r['v']] for r in res])
    ax.invert_yaxis()
    ax.set_xlabel('Tip re-seat error, rms (µm)')
    ax.legend(ncol=3, loc='lower left', bbox_to_anchor=(0, 1.0))
    ax.set_xlim(0, max(np.sqrt(r['floor_rms'] ** 2 + max(r['fast'] ** 2 - r['floor_rms'] ** 2, 0) + r['slow'] ** 2)
                       for r in res) * 1.9)
    return save(fig, HERE, 'spread_components')


def fig_settling(res):
    fig, ax = plt.subplots(figsize=(HALF * 1.3, 2.3))
    for r in res:
        st = VERSION[r['v']]
        s = np.sort(r['settle'][1:-1])
        ax.step(s, np.arange(1, len(s) + 1) / len(s), where='post', color=st['color'], label=LABELS[r['v']])
    ax.set_xlabel('Settling time after the still rule fires (s)')
    ax.set_ylabel('Share of seats settled')
    ax.legend(loc='lower right')
    return save(fig, HERE, 'settling_ecdf')


def main():
    style()
    sets = sorted(glob.glob(os.path.join(runs.LOGS, 'Exchange', '2026*_QC_V*')))
    res = [analyse(d) for d in sets]
    L = ['# 13 Results: quick-change coupling re-seat and exchange time', '',
         'Generated by `analysis.py`. Camera-frame NDI data, wand on the tool plate, arm still; only the wand '
         'is tracked. Offsets are expressed in the wand frame (z = shaft ≈ coupling axis). µm unless stated. Lever '
         f"arms are read from `config/tools.yaml` (micro tool tip {MICRO_TIP_FROM_TOOL0_MM:.1f} mm from tool0, tool "
         f"'{MICRO['name']}', {MICRO['date']}); every other number comes from NDI_PATH.csv.", '',
         '## Per version', '',
         '| | ' + ' | '.join(LABELS[r['v']] for r in res) + ' |', '|---|' + '---|' * len(res)]
    def row(label, f):
        L.append(f'| {label} | ' + ' | '.join(f(r) for r in res) + ' |')
    row('recording', lambda r: f"{r['name']} ({r['date'][:16]})")
    row('seated states / exchanges', lambda r: f"{r['n_seats']} / {r['n_seats'] - 1}")
    row('tracker per-sample SD (x, y, z)', lambda r: ', '.join(f'{x:.0f}' for x in r['sample_sd']))
    row('tracker floor, 5 s medians (rms / max)', lambda r: f"{r['floor_rms']:.1f} / {r['floor_max']:.1f}")
    row('still-period wander (µm/min, start / end)', lambda r: ' / '.join(f'{w:.0f}' for w in r['wander']))
    row('tip re-seat: rms / max', lambda r: f"{np.sqrt(np.mean(r['dev'] ** 2)):.0f} / {r['dev'].max():.0f}")
    row('… exchanges only (first seating excluded, as in the pooled lines)',
        lambda r: f"{np.sqrt(np.mean(r['dev'][1:] ** 2)):.0f} / {r['dev'][1:].max():.0f}")
    row('× tracker floor (rms)', lambda r: f"{np.sqrt(np.mean(r['dev'] ** 2)) / r['floor_rms']:.0f}×")
    row('axial (seating depth) SD', lambda r: f"{r['off_w'][:, 2].std():.0f}")
    row('radial SD (x, y)', lambda r: f"{r['off_w'][:, 0].std():.0f}, {r['off_w'][:, 1].std():.0f}")
    row('tilt SD (°, x, y)', lambda r: f"{r['rv'][:, 0].std():.4f}, {r['rv'][:, 1].std():.4f}")
    row('clocking SD (°)', lambda r: f"{r['rv'][:, 2].std():.4f}")
    row('rotation floor, 5 s means (°, rms)', lambda r: f"{r['rot_floor']:.4f}")
    row('holder tool used for the levers (tools.yaml: number, name, date, tip→tool0 mm)', lambda r: f"{r['holder'][0]}, {r['holder'][1]}, {r['holder'][2]}, {r['holder'][3]:.1f}")
    row('→ floor at tool0 from rotation alone (µm)', lambda r: f"{np.radians(r['rot_floor']) * r['holder'][3] * 1000:.0f}")
    row('settling threshold from still periods (µm)', lambda r: f"{r['settle_thr']:.0f}")
    row('max rotation (°)', lambda r: f"{np.linalg.norm(r['rv'], axis=1).max():.4f}")
    row('at tool0: rms / max', lambda r: f"{np.sqrt(np.mean(np.sum(r['at_tool0'] ** 2, 1))):.0f} / "
                                         f"{np.linalg.norm(r['at_tool0'], axis=1).max():.0f}")
    row('at micro tool tip: rms / max', lambda r: f"{np.sqrt(np.mean(np.sum(r['at_micro'] ** 2, 1))):.0f} / "
                                                   f"{np.linalg.norm(r['at_micro'], axis=1).max():.0f}")
    row('sensitivity: micro-tip max with each holder tool (µm)', lambda r: ', '.join(f'tool {n}: {v:.0f}' for n, v in r['micro_alt'].items()))
    row('total / per-swap / slow (rms)', lambda r: f"{r['tot']:.0f} / {r['fast']:.0f} / {r['slow']:.0f}")
    row('per-swap coupling part (floor removed)', lambda r: f"{np.sqrt(max(r['fast'] ** 2 - r['floor_rms'] ** 2, 0)):.0f}")
    row('… from seat 4 (bedding-in removed)', lambda r: f"{r['tot4']:.0f} / {r['fast4']:.0f} / {r['slow4']:.0f}")
    row('first 3 seats vs rest (mean offset)', lambda r: f"{r['dev'][:3].mean():.0f} vs {r['dev'][3:].mean():.0f}")
    row('linear drift (µm/min, share of spread)', lambda r: f"{r['drift']:.0f} ({r['drift_r2']:.0%})")
    row('successive offsets: mean cosine (shuffled), p', lambda r: f"{r['cobs']:+.2f} ({r['cnull']:+.2f}), "
                                                                     f"{r['p_cos']:.4f}")
    row('largest persistent shift: before seat, size, p', lambda r: f"{r['cp'][0]}, {r['cp'][1]:.0f}, {r['cp'][2]:.3f}")
    row('mid-seat steps: max vs still-period max; seats above', lambda r: f"{r['bump_max']:.0f} vs {r['bump_ref']:.0f}; "
                                                                           f"{r['bumps'] or 'none'}")
    row('trim 0 / 1 / 2 s: rms', lambda r: ' / '.join(f'{x[1]:.1f}' for x in r['trims']))
    row('marker fit error per seat (mm, median)', lambda r: f"{np.median(r['fit']):.3f}")
    row('settling time (s): median / max', lambda r: f"{np.median(r['settle'][1:-1]):.2f} / {r['settle'][1:-1].max():.2f}")
    for key, lab in (('total', 'exchange time (s)'), ('unlock', 'unlock'), ('away', 'away'),
                     ('out_of_view', 'of which out of view'), ('seating', 'seating')):
        row(f'{lab}: median [IQR], max', lambda r, k=key: (lambda x: f"{np.median(x):.1f} [{np.percentile(x, 25):.1f}–"
                                                          f"{np.percentile(x, 75):.1f}], {np.max(x):.1f}")(
            np.array([p[k] for p in r['phases']])))
    # pooled
    dev = np.concatenate([r['dev'][1:] for r in res])
    tot = np.concatenate([[p['total'] for p in r['phases']] for r in res])
    micro = np.concatenate([np.linalg.norm(r['at_micro'][1:], axis=1) for r in res])
    L += ['', '## Pooled over the three versions (60 exchanges)', '',
          f'- Tip re-seat: rms {np.sqrt(np.mean(dev ** 2)):.0f} µm, p95 {np.percentile(dev, 95):.0f} µm, max '
          f'{dev.max():.0f} µm. Seats are serially correlated (above), so no CI is put on the percentile.',
          f'- At the micro tool tip: max {micro.max():.0f} µm.',
          f'- Exchange time (seat disturbed → settled): median {np.median(tot):.1f} s, max {tot.max():.1f} s '
          f'(threshold 20 s).', '']
    # headline for the coupling as built: the steel versions only (C1 was the printed prototype)
    steel = [r for r in res if r['v'] != res[0]['v']]
    if steel:
        sdev = np.concatenate([r['dev'][1:] for r in steel])
        stot = np.concatenate([[p['total'] for p in r['phases']] for r in steel])
        smic = np.concatenate([np.linalg.norm(r['at_micro'][1:], axis=1) for r in steel])
        L += [f'## Pooled over the steel versions ({" + ".join(LABELS[r["v"]] for r in steel)}, {len(sdev)} exchanges)', '',
              f'- Tip re-seat: rms {np.sqrt(np.mean(sdev ** 2)):.0f} µm, p95 {np.percentile(sdev, 95):.0f} µm, max '
              f'{sdev.max():.0f} µm (mean of the per-version rms: '
              f'{np.mean([np.sqrt(np.mean(r["dev"][1:] ** 2)) for r in steel]):.0f} µm).',
              f'- At the micro tool tip: max {smic.max():.0f} µm.',
              f'- Exchange time: median {np.median(stot):.1f} s, max {stot.max():.1f} s.', '']
    L += ['## Versions compared on the drift-robust per-swap component', '',
          'Ratio of per-swap rms (successive differences), bootstrap 95 % CI. Each version is one session, so a '
          'difference is version + day + camera pose + handling.', '']
    for a, b in ((0, 1), (0, 2), (1, 2)):
        if max(a, b) < len(res):
            obs, ci = boot_ratio(res[a], res[b])
            L.append(f"- {LABELS[res[a]['v']]} / {LABELS[res[b]['v']]}: {obs:.2f} [{ci[0]:.2f}, {ci[1]:.2f}]")
    L += ['', '## Prior claims checked', '',
          '- "pooled re-seat ≤ 0.19 mm, rms 0.043 mm": see the pooled line (note: seats here are medians of the '
          '*settled* part, which can differ slightly from whole-segment medians).',
          '- "tracker 5 s floor 0.003–0.006 mm": see the floor row.',
          '- "successive seats correlated, cosine +0.62 to +0.70": see the cosine row.',
          '- "median exchange 4.8 s": earlier figure was the gap between still segments; here exchange time runs '
          'from the seat being disturbed to settled, so it is longer by the unlock and settling phases.', '',
          '## Figures', '']
    for p in (fig_swimlane(res), fig_radial(res), fig_sequence(res), fig_components(res), fig_settling(res)):
        L.append(f'- `figures/{os.path.basename(p)}`')
    open(os.path.join(HERE, 'RESULTS.md'), 'w').write('\n'.join(L) + '\n')
    print('\n'.join(L))


if __name__ == '__main__':
    main()
