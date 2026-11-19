"""RCM error recomputed from NDI geometry (not from NDI_META): shaft line (tip + axis) to the latched point.

Per sample in an RCM segment (rcm_active == 1, visible): distance from the RCM point to the line through
tip_* along axis_*. Also crossing points on the plane through the point normal to the mean axis, the
best fixed pivot (least-squares point closest to all lines) and the error about it.
"""
import numpy as np

from .runs import read_csv


def load(run):
    p = run.path + '/NDI_PATH.csv'
    import os
    if not os.path.exists(p):
        return None
    return read_csv(p)


def segments(n):
    act = (n['rcm_active'] == 1) & (n['visible'] == 1) & np.isfinite(n['rcm_x'])
    pts = np.c_[n['rcm_x'], n['rcm_y'], n['rcm_z']]
    out, i = [], 0
    while i < len(act):
        if act[i]:
            j = i
            while j < len(act) and act[j] and np.allclose(pts[j], pts[i]):
                j += 1
            out.append((i, j))
            i = j
        else:
            i += 1
    return out


def line_dist(P, A, c):
    A = A / np.linalg.norm(A, axis=1, keepdims=True)
    d = c - P
    return np.linalg.norm(d - np.sum(d * A, 1)[:, None] * A, axis=1)


def best_pivot(P, A):
    A = A / np.linalg.norm(A, axis=1, keepdims=True)
    M = np.zeros((3, 3)); b = np.zeros(3)
    for p, a in zip(P, A):
        K = np.eye(3) - np.outer(a, a)
        M += K; b += K @ p
    return np.linalg.solve(M, b)


def rcm_errors(run, longest=True):
    """dict for the longest RCM segment: e (mm per sample), t, rms, p95, max, within1, bias, spread,
    rms_fitted, swing_deg, recorded (NDI_PATH rcm_error column rms, for the cross-check)."""
    n = load(run)
    if n is None or 'rcm_active' not in n:
        return None
    segs = segments(n)
    if not segs:
        return None
    a, b = max(segs, key=lambda s: s[1] - s[0]) if longest else segs[0]
    P = np.c_[n['tip_x'], n['tip_y'], n['tip_z']][a:b]
    A = np.c_[n['axis_x'], n['axis_y'], n['axis_z']][a:b]
    c = np.array([n['rcm_x'][a], n['rcm_y'][a], n['rcm_z'][a]])
    e = line_dist(P, A, c)
    An = A / np.linalg.norm(A, axis=1, keepdims=True)
    am = An.mean(0); am /= np.linalg.norm(am)
    # crossing points on the plane through c normal to the mean axis
    s = ((c - P) @ am) / (An @ am)
    X = P + s[:, None] * An - c
    bias_v = X.mean(0)
    piv = best_pivot(P, A)
    ef = line_dist(P, A, piv)
    swing = np.degrees(np.arccos(np.clip(An @ am, -1, 1)))
    rec = n.get('rcm_error')
    return dict(t=n['t'][a:b], e=e, rms=float(np.sqrt(np.mean(e ** 2))), p95=float(np.percentile(e, 95)),
                max=float(e.max()), within1=float(np.mean(e <= 1.0)), within2=float(np.mean(e <= 2.0)),
                bias=float(np.linalg.norm(bias_v)), spread=float(np.sqrt(np.mean(np.sum((X - bias_v) ** 2, 1)))),
                cross=X, rms_fitted=float(np.sqrt(np.mean(ef ** 2))), pivot_offset=float(np.linalg.norm(piv - c)),
                swing_deg=float(2 * np.percentile(swing, 99)), n=int(b - a), duration=float(n['t'][b - 1] - n['t'][a]),
                visible_pct=float(100 * np.mean(n['visible'] == 1)),
                recorded_rms=float(np.sqrt(np.nanmean(rec[a:b] ** 2))) if rec is not None else np.nan)
