"""Tracking metrics written from scratch for this folder (not plot_paths): strokes, lag, error, jitter.

Conventions
- Target = TOUCH_PATH.csv (exists only while WHITE is held), arm = UR_PATH.csv rows with source 'rtde',
  both on the CSV 't' clock, positions in m -> mm.
- A stroke = a run of target samples with gaps < 0.1 s, at least 0.5 s long.
- lag_pos: shift s >= 0 minimising rms |arm(t+s) - target(t)| over strokes (1 ms grid, parabolic refine).
- lag_vel: shift maximising the correlation of arm and target velocity (5 ms-smoothed), same grid.
- err: rms position error after shifting by lag_pos. jitter: rms of the >5 Hz part (4th-order zero-phase
  Butterworth high-pass) of arm and target, per stroke on a uniform 500 Hz grid; ratio = arm / target.
- Rotation: same with the rotation angle between quaternions (deg) for err, and angular velocity for lag_vel;
  lag_rot uses the rms angle between orientations.
"""
import numpy as np
from scipy.signal import butter, sosfiltfilt
from scipy.spatial.transform import Rotation, Slerp

FS = 500.0


def strokes(t, gap=0.1, min_len=0.5):
    if len(t) < 2:
        return []
    br = np.where(np.diff(t) > gap)[0]
    starts = np.r_[0, br + 1]
    ends = np.r_[br, len(t) - 1]
    return [(t[a], t[b]) for a, b in zip(starts, ends) if t[b] - t[a] >= min_len]


def load_paths(run):
    tg = run.csv('TOUCH_PATH.csv')
    ur = run.csv('UR_PATH.csv')
    if not tg or not ur:
        return None
    keep = ur['source'] == 'rtde' if 'source' in ur else np.ones(len(ur['t']), bool)
    if keep.sum() < 0.5 * len(keep):          # older runs: tf fallback only
        keep = np.ones(len(ur['t']), bool)
    def pack(d, m):
        t = d['t'][m]
        o = np.argsort(t)
        P = np.c_[d['x'], d['y'], d['z']][m][o] * 1000.0
        Q = np.c_[d['qx'], d['qy'], d['qz'], d['qw']][m][o]
        t = t[o]
        u = np.r_[True, np.diff(t) > 0]
        return t[u], P[u], Q[u]
    return pack(tg, np.ones(len(tg['t']), bool)), pack(ur, keep)


def gaps(t, factor=3.0):
    """(start, length) of gaps longer than factor x median dt."""
    dt = np.diff(t)
    med = np.median(dt)
    idx = np.where(dt > factor * med)[0]
    return [(t[i], dt[i]) for i in idx]


def _grid(spans):
    return [np.arange(a, b, 1 / FS) for a, b in spans]


def _interp(t, P, tq):
    return np.column_stack([np.interp(tq, t, P[:, k], left=np.nan, right=np.nan) for k in range(P.shape[1])])


def _slerp(t, Q, tq):
    tq = np.clip(tq, t[0], t[-1])
    return Slerp(t, Rotation.from_quat(Q))(tq)


def best_shift(cost, lo=0.0, hi=1.6, step=0.001):
    s = np.arange(lo, hi, step)
    c = np.array([cost(x) for x in s])
    i = int(np.nanargmin(c))
    if 0 < i < len(s) - 1 and np.all(np.isfinite(c[i - 1:i + 2])):
        a, b, d = c[i - 1], c[i], c[i + 1]
        den = a - 2 * b + d
        off = 0.5 * (a - d) / den if den > 0 else 0.0
        return s[i] + off * step, c[i]
    return s[i], c[i]


def metrics(run, coarse=0.004):
    paths = load_paths(run)
    if paths is None:
        return None
    (tt, tp, tq), (ut, up, uq) = paths
    return metrics_arrays(tt, tp, tq, ut, up, uq, coarse)


def metrics_arrays(tt, tp, tq, ut, up, uq, coarse=0.004):
    spans = strokes(tt)
    if not spans:
        return None
    grids = _grid(spans)
    tq_all = np.concatenate(grids)
    tgt = _interp(tt, tp, tq_all)
    rt = _slerp(tt, tq, tq_all)

    def pos_cost(s):
        a = _interp(ut, up, tq_all + s)
        e = np.linalg.norm(a - tgt, axis=1)
        return np.sqrt(np.nanmean(e ** 2))

    def rot_cost(s):
        ra = _slerp(ut, uq, tq_all + s)
        return np.sqrt(np.mean(np.degrees((ra.inv() * rt).magnitude()) ** 2))

    # coarse then fine search
    s0, _ = best_shift(pos_cost, 0.0, 1.6, coarse)
    lag_p, err_p = best_shift(pos_cost, max(0, s0 - 2 * coarse), s0 + 2 * coarse, 0.0005)
    r0, _ = best_shift(rot_cost, 0.0, 1.6, coarse)
    lag_r, err_r = best_shift(rot_cost, max(0, r0 - 2 * coarse), r0 + 2 * coarse, 0.0005)

    # velocity lag: correlation of 5 ms-smoothed velocities on each stroke
    def vel(P, t):
        return np.gradient(P, t, axis=0)
    vt = [vel(_interp(tt, tp, g), g) for g in grids]

    def vel_cost(s):
        num = den1 = den2 = 0.0
        for g, v in zip(grids, vt):
            va = vel(_interp(ut, up, g + s), g)
            m = np.isfinite(va).all(1) & np.isfinite(v).all(1)
            num += np.sum(va[m] * v[m]); den1 += np.sum(va[m] ** 2); den2 += np.sum(v[m] ** 2)
        return -num / np.sqrt(den1 * den2) if den1 > 0 else np.nan
    v0, _ = best_shift(vel_cost, 0.0, 1.6, coarse)
    lag_v, corr_v = best_shift(vel_cost, max(0, v0 - 2 * coarse), v0 + 2 * coarse, 0.0005)

    sos = butter(4, 5.0, 'highpass', fs=FS, output='sos')
    ja, jt = [], []
    for g in grids:
        if len(g) < 3 * 50:
            continue
        a = _interp(ut, up, g)
        b = _interp(tt, tp, g)
        if not (np.isfinite(a).all() and np.isfinite(b).all()):
            continue
        ja.append(sosfiltfilt(sos, a, axis=0)); jt.append(sosfiltfilt(sos, b, axis=0))
    jit_arm = np.sqrt(np.mean(np.concatenate([np.sum(x ** 2, 1) for x in ja]))) if ja else np.nan
    jit_tgt = np.sqrt(np.mean(np.concatenate([np.sum(x ** 2, 1) for x in jt]))) if jt else np.nan
    raw = pos_cost(0.0)
    return dict(strokes=len(spans), engaged_s=float(sum(b - a for a, b in spans)),
                lag_ms=lag_p * 1e3, lag_vel_ms=lag_v * 1e3, lag_rot_ms=lag_r * 1e3,
                err_mm=err_p, err_raw_mm=raw, err_rot_deg=err_r, vel_corr=-corr_v,
                jitter_arm_mm=jit_arm, jitter_tgt_mm=jit_tgt, jitter_ratio=jit_arm / jit_tgt,
                path_gaps=[(float(a), float(b)) for a, b in gaps(ut) if any(s <= a <= e for s, e in spans)])


def cached_metrics(run):
    """metrics(run), cached in common/cache/track/<key>.json keyed on the CSVs' mtimes and this file's mtime."""
    import json
    import os
    from .runs import HERE
    d = os.path.join(HERE, 'cache', 'track')
    os.makedirs(d, exist_ok=True)
    stamp = [os.path.getmtime(os.path.join(run.path, f)) for f in ('UR_PATH.csv', 'TOUCH_PATH.csv')
             if run.has(f)] + [os.path.getmtime(__file__)]
    p = os.path.join(d, run.key.replace('/', '__') + '.json')
    if os.path.exists(p):
        c = json.load(open(p))
        if c.get('stamp') == stamp:
            return c['m']
    m = metrics(run)
    json.dump({'stamp': stamp, 'm': m}, open(p, 'w'))
    return m


def freq_lag(run, fmax=8.0, nper=1024):
    """Lag vs frequency from the cross-spectrum of target and arm velocity (per axis, pooled over strokes).

    Returns f, lag_ms (phase / 2 pi f, unwrapped), coherence; bins with coherence < 0.5 are nan."""
    from scipy.signal import csd, welch
    paths = load_paths(run)
    if paths is None:
        return None
    (tt, tp, tq), (ut, up, uq) = paths
    Sxy = Sxx = Syy = 0
    f = None
    for a, b in strokes(tt):
        g = np.arange(a, b, 1 / FS)
        if len(g) < nper:
            continue
        x = np.gradient(_interp(tt, tp, g), g, axis=0)
        y = np.gradient(_interp(ut, up, g), g, axis=0)
        if not (np.isfinite(x).all() and np.isfinite(y).all()):
            continue
        for k in range(3):
            f, pxy = csd(x[:, k], y[:, k], fs=FS, nperseg=nper)
            _, pxx = welch(x[:, k], fs=FS, nperseg=nper)
            _, pyy = welch(y[:, k], fs=FS, nperseg=nper)
            Sxy = Sxy + pxy; Sxx = Sxx + pxx; Syy = Syy + pyy
    if f is None:
        return None
    coh = np.abs(Sxy) ** 2 / (Sxx * Syy)
    ph = -np.unwrap(np.angle(Sxy))
    lag = ph / (2 * np.pi * np.where(f > 0, f, np.nan)) * 1e3
    m = (f > 0.15) & (f <= fmax)
    lag = np.where(coh >= 0.5, lag, np.nan)
    return f[m], lag[m], coh[m]
