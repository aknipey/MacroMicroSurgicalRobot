#!/usr/bin/env python3
"""16 The stylus signal and the filters, offline, with the teleop node's own code.

1. Stylus signal (STYLUS_RAW.csv, the omni_state pose as published): publish interval, interval between real
   updates (rows whose pose changed) while moving, real updates per 10 ms window, and the error of a 10 ms
   difference against a smoothed reference velocity.
2. Tremor: stylus orientation deviation (rotation vector from a 3 Hz zero-phase smoothed orientation) band-passed
   7-13 Hz and 15-25 Hz, rms (deg) over engaged strokes.
3. Kalman filter: the node's ConstantVelocityKalman (src/ur16e_touch_teleop.py, settings from
   macro_micro_robot/teleop_settings.py), fed the recorded samples with their header stamps as the node does.
   Feedforward velocity = difference of the latest target at the publish rate (META of the run). Noise = rms of that
   velocity above 10 Hz (4th-order zero-phase high-pass) while engaged; lag = cross-correlation shift against the
   raw velocity smoothed zero-phase at 10 Hz. Tremor band 7-13 Hz reported separately.
4. One Euro filter (Casiez et al. 2012) on the same samples, grid of min_cutoff x beta: the lag of the settings that
   remove at least as much noise as the Kalman filter.
5. Command filter: the node's LowPass (macro_micro_robot/ur16e_common.py, 2nd-order Butterworth) run at 500 Hz on
   the recorded unfiltered twist commands (V9b/NoTool/VelocityUnfiltered, /servo_node/delta_twist_cmds); amplitude
   left per band (sqrt of Welch power ratio) and low-frequency group delay vs sqrt(2)/(2 pi f_c).
Writes RESULTS.md.
"""
import os
import sys

import numpy as np
from scipy.signal import butter, group_delay, sosfiltfilt, welch
from scipy.spatial.transform import Rotation

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from common import bags, runs, track  # noqa: E402
from common.style import FULL, INK2, MUTED, save, style, panel  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
sys.path.insert(0, os.path.join(runs.PKG, 'src'))
import ur16e_touch_teleop as node  # noqa: E402
from macro_micro_robot import teleop_settings as cfg  # noqa: E402
from macro_micro_robot.ur16e_common import LowPass  # noqa: E402

RUNS = ['V3/LongBenchmark', 'V3/ShortBenchmark', 'V3/Scaled5_Slow', 'V2/V2_90_Degrees_Twist']


def stylus(run):
    s = run.csv('STYLUS_RAW.csv')
    P = np.c_[s['x'], s['y'], s['z']]                                   # mm
    Q = np.c_[s['qx'], s['qy'], s['qz'], s['qw']]
    return s['t'], s['stamp'], P, Q


def engaged(run):
    return track.strokes(run.csv('TOUCH_PATH.csv')['t'])


def in_spans(t, spans):
    m = np.zeros(len(t), bool)
    for a, b in spans:
        m |= (t >= a) & (t <= b)
    return m


# ---------------------------------------------------------------- 1-2 stylus signal and tremor
def stylus_stats(run):
    t, st, P, Q = stylus(run)
    sp = engaged(run)
    m = in_spans(t, sp)
    pub = np.median(np.diff(t[m]))
    changed = np.r_[True, np.any(np.diff(np.c_[P, Q], axis=0) != 0, axis=1)]
    # moving: 20 ms-smoothed speed of the changed samples > 5 mm/s or > 0.05 rad/s
    tc, Pc, Qc = t[changed & m], P[changed & m], Q[changed & m]
    iv = np.diff(tc) * 1000
    g = np.arange(tc[0], tc[-1], 0.001)
    Pg = np.column_stack([np.interp(g, tc, Pc[:, k]) for k in range(3)])
    v = np.linalg.norm(np.gradient(Pg, 0.001, axis=0), axis=1)
    v = np.convolve(v, np.ones(20) / 20, 'same')
    mov = np.interp(tc[1:], g, v) > 5.0
    ivm = iv[mov & (iv < 100)]
    # updates per 10 ms window while moving
    w = np.arange(tc[0], tc[-1], 0.01)
    cnt = np.histogram(tc, bins=w)[0]
    movw = np.interp(w[:-1] + 0.005, g, v) > 5.0
    cnt = cnt[movw]
    # 10 ms difference of the latest sample vs reference velocity (zero-phase 10 Hz smoothed)
    idx = np.searchsorted(t, w, side='right') - 1
    Pl = P[np.clip(idx, 0, len(P) - 1)]
    v10 = np.diff(Pl, axis=0) / 0.01
    sos = butter(4, 10, fs=1000, output='sos')
    vref_g = np.gradient(sosfiltfilt(sos, Pg, axis=0), 0.001, axis=0)
    vref = np.column_stack([np.interp(w[:-1] + 0.005, g, vref_g[:, k]) for k in range(3)])
    nr = np.linalg.norm(vref, axis=1)
    sel = movw & (nr > 20)
    rel = np.linalg.norm(v10[sel] - vref[sel], axis=1) / nr[sel]
    if len(ivm) == 0 or len(rel) == 0:
        return None
    return dict(pub_ms=pub * 1000, upd_med=np.median(ivm), upd_p95=np.percentile(ivm, 95), upd_p99=np.percentile(ivm, 99),
                upd_max=ivm.max(), share_changed=changed[m].mean(), cnt=np.bincount(cnt, minlength=6)[:6] / max(len(cnt), 1),
                rel_med=np.median(rel), rel_p95=np.percentile(rel, 95))


def tremor(run):
    t, st, P, Q = stylus(run)
    out = {'7–13 Hz': [], '15–25 Hz': []}
    for a, b in engaged(run):
        m = (t >= a) & (t <= b)
        if m.sum() < 2000:
            continue
        g = np.arange(a, b, 0.001)
        R = track._slerp(t[m], Q[m], g)
        rv = R.as_rotvec()
        rs = sosfiltfilt(butter(4, 3, fs=1000, output='sos'), np.unwrap(rv, axis=0), axis=0)
        dev = (Rotation.from_rotvec(rs).inv() * R).as_rotvec()
        for bn, (lo, hi) in (('7–13 Hz', (7, 13)), ('15–25 Hz', (15, 25))):
            x = sosfiltfilt(butter(4, [lo, hi], 'bandpass', fs=1000, output='sos'), dev, axis=0)
            out[bn].append(np.degrees(np.linalg.norm(x[250:-250], axis=1)))
    return {k: float(np.sqrt(np.mean(np.concatenate(v) ** 2))) for k, v in out.items() if v}


# ---------------------------------------------------------------- 3-4 Kalman and One Euro
def node_rate(run):
    return float(run.settings.get('TOUCH_PUBLISH_RATE', 100.0))


def feedforward(tq, values, rotation):
    """Velocity from differencing the target at the node rate (tq), as update_target_velocity does (before its filter)."""
    dt = np.diff(tq)
    if rotation:
        R = values
        return (R[1:] * R[:-1].inv()).as_rotvec() / dt[:, None]
    return np.diff(values, axis=0) / dt[:, None]


def run_kalman(t, st, P, Q, tq, spans):
    """Node's filter fed every sample (header stamp); estimate read at the node ticks tq. Restarts per stroke."""
    kp = node.ConstantVelocityKalman(cfg.TOUCH_KALMAN_ACCEL_NOISE, cfg.TOUCH_KALMAN_POSITION_NOISE)
    kr = node.ConstantVelocityKalman(cfg.TOUCH_KALMAN_ANGULAR_ACCEL_NOISE, cfg.TOUCH_KALMAN_ROTATION_NOISE, rotation=True)
    Pm = P / 1000.0
    outP = np.full((len(tq), 3), np.nan)
    outQ = [None] * len(tq)
    j = 0
    off = np.median(st - t)                       # header stamp -> CSV t
    for a, b in spans:
        kp.clear(); kr.clear()
        sel = np.where((t >= a - 0.05) & (t <= b))[0]
        k = np.searchsorted(tq, t[sel[0]])
        for i in sel:
            kp.feed(Pm[i], Pm[i], st[i]); kr.feed(Rotation.from_quat(Q[i]), Q[i], st[i])
            nxt = t[i + 1] if i + 1 < len(t) else np.inf
            while k < len(tq) and tq[k] < nxt:
                if tq[k] >= t[i]:
                    outP[k] = kp.predict(tq[k] + off) * 1000.0
                    outQ[k] = kr.predict(tq[k] + off)
                k += 1
    return outP, outQ


def one_euro(t, x, min_cutoff, beta, d_cutoff=1.0):
    def alpha(dt, fc):
        tau = 1.0 / (2 * np.pi * fc)
        return 1.0 / (1.0 + tau / dt)
    y = np.empty_like(x)
    y[0] = x[0]
    dx = np.zeros(x.shape[1])
    for i in range(1, len(x)):
        dt = max(t[i] - t[i - 1], 1e-4)
        dxi = (x[i] - x[i - 1]) / dt
        a_d = alpha(dt, d_cutoff)
        dx = a_d * dxi + (1 - a_d) * dx
        fc = min_cutoff + beta * np.linalg.norm(dx)
        a = alpha(dt, fc)
        y[i] = a * x[i] + (1 - a) * y[i - 1]
    return y


def latest(t, X, tq):
    i = np.clip(np.searchsorted(t, tq, side='right') - 1, 0, len(t) - 1)
    return X[i]


def noise_lag(tq, V, ref, mask, fs):
    sos = butter(4, 10, 'highpass', fs=fs, output='sos')
    ok = mask & np.isfinite(V).all(1)
    Vf = np.where(np.isfinite(V), V, 0.0)
    hp = sosfiltfilt(sos, Vf, axis=0)
    noise = float(np.sqrt(np.mean(np.sum(hp[ok] ** 2, axis=1))))
    bp = sosfiltfilt(butter(4, [7, 13], 'bandpass', fs=fs, output='sos'), Vf, axis=0)
    trem = float(np.sqrt(np.mean(np.sum(bp[ok] ** 2, axis=1))))
    best, lag = -np.inf, 0
    for s in range(0, int(0.25 * fs)):
        a = Vf[s:][ok[s:]]
        b = ref[:len(ref) - s][ok[s:]]
        c = np.sum(a * b) / np.sqrt(np.sum(a * a) * np.sum(b * b))
        if c > best:
            best, lag = c, s
    return noise, trem, 1000.0 * lag / fs


def kalman_study(run):
    t, st, P, Q = stylus(run)
    spans = engaged(run)
    fs = node_rate(run)
    tq = np.arange(spans[0][0], spans[-1][1], 1.0 / fs)
    mask = in_spans(tq, spans)[1:]
    # raw: latest sample at each tick
    Pr = latest(t, P, tq)
    Rr = Rotation.from_quat(latest(t, Q, tq))
    # reference: zero-phase 10 Hz smoothed raw position / orientation on a 1 kHz grid
    g = np.arange(tq[0], tq[-1], 0.001)
    sos = butter(4, 10, fs=1000, output='sos')
    Pg = sosfiltfilt(sos, np.column_stack([np.interp(g, t, P[:, k]) for k in range(3)]), axis=0)
    rvg = np.unwrap(track._slerp(t, Q, g).as_rotvec(), axis=0)
    Rg = Rotation.from_rotvec(sosfiltfilt(sos, rvg, axis=0))
    ref_lin = np.column_stack([np.interp(tq[1:], g, np.gradient(Pg, 0.001, axis=0)[:, k]) for k in range(3)])
    wg = (Rg[1:] * Rg[:-1].inv()).as_rotvec() / 0.001
    ref_ang = np.column_stack([np.interp(tq[1:], g[1:], wg[:, k]) for k in range(3)])
    Pk, Qk = run_kalman(t, st, P, Q, tq, spans)
    okq = np.array([q is not None for q in Qk])
    Qk = [q if q is not None else Rotation.identity() for q in Qk]
    Rk = Rotation.concatenate(Qk)
    res = {}
    for name, (Vl, Va) in {'raw': (feedforward(tq, Pr, False), feedforward(tq, Rr, True)),
                           'kalman': (feedforward(tq, Pk, False), feedforward(tq, Rk, True))}.items():
        if name == 'kalman':
            bad = ~(okq[1:] & okq[:-1])
            Va[bad] = np.nan
        res[name] = dict(lin=noise_lag(tq[1:], Vl, ref_lin, mask, fs), ang=noise_lag(tq[1:], Va, ref_ang, mask, fs))
    # One Euro on position, grid
    passthrough = noise_lag(tq[1:], feedforward(tq, latest(t[np.r_[True, np.any(np.diff(P, axis=0) != 0, axis=1)]],
                                                       one_euro(t[np.r_[True, np.any(np.diff(P, axis=0) != 0, axis=1)]],
                                                                P[np.r_[True, np.any(np.diff(P, axis=0) != 0, axis=1)]], 1e4, 0.0), tq), False),
                            ref_lin, mask, fs)
    grid = []
    changed = np.r_[True, np.any(np.diff(P, axis=0) != 0, axis=1)]
    for mc in (0.1, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0):
        for beta in (0.0, 0.001, 0.005, 0.02, 0.1, 0.5):
            # run over the whole continuous stream (real updates only), as a stateless-start filter would;
            # resetting per stroke would hold the last value across the gap and create a spike at each stroke start
            Y = one_euro(t[changed], P[changed], mc, beta)
            Yq = latest(t[changed], Y, tq)
            n, tr, lag = noise_lag(tq[1:], feedforward(tq, Yq, False), ref_lin, mask, fs)
            grid.append((mc, beta, n, tr, lag))
    res['oneeuro'] = grid
    res['oe_passthrough'] = passthrough
    return fs, res


# ---------------------------------------------------------------- 5 command filter
def command_filter_study():
    out = []
    rs = runs.find_runs('V9b/NoTool/VelocityUnfiltered/Scaled5_*_Replay*')
    cmds = []
    for r in rs:
        b = bags.load(r, ['/servo_node/delta_twist_cmds'])
        c = b['/servo_node/delta_twist_cmds']
        X = np.c_[c['lin'] * 1000, np.degrees(c['ang'])]
        th = c['th']
        g = np.arange(th[0], th[-1], 1 / 500.0)
        cmds.append(np.column_stack([np.interp(g, th, X[:, k]) for k in range(6)]))
    for fc in (5.0, 6.0, 8.0, 10.0):
        b_, a_ = butter(2, fc, fs=500.0)
        w, gd = group_delay((b_, a_), w=[0.5, 1.0, 2.0], fs=500.0)
        Pin = {}; Pout = {}
        for X in cmds:
            lp = LowPass(fc, 500.0, 6)
            Y = np.array([lp(x) for x in X])
            for nm, sl in (('lin', slice(0, 3)), ('ang', slice(3, 6))):
                f, pi = welch(X[:, sl], fs=500.0, nperseg=500, axis=0)
                _, po = welch(Y[:, sl], fs=500.0, nperseg=500, axis=0)
                Pin.setdefault(nm, []).append(pi.sum(1)); Pout.setdefault(nm, []).append(po.sum(1))
        row = dict(fc=fc, gd_ms=1000 * gd / 500.0, formula_ms=1000 * np.sqrt(2) / (2 * np.pi * fc))
        for nm in ('lin', 'ang'):
            pi, po = np.sum(Pin[nm], 0), np.sum(Pout[nm], 0)
            for bn, (lo, hi) in (('8–12', (8, 12)), ('15–25', (15, 25)), ('30–50', (30, 50))):
                m = (f >= lo) & (f <= hi)
                row[(nm, bn)] = float(np.sqrt(po[m].sum() / pi[m].sum()))
        out.append(row)
    return out


def main():
    style()
    L = ['# 16 Results: the stylus signal and the filters, offline (teleop node code)', '',
         'Generated by `analysis.py`. Node code: `src/ur16e_touch_teleop.py` (ConstantVelocityKalman), '
         '`macro_micro_robot/ur16e_common.py` (LowPass); settings `macro_micro_robot/teleop_settings.py`: Kalman '
         f'accel noise {cfg.TOUCH_KALMAN_ACCEL_NOISE}, position noise {cfg.TOUCH_KALMAN_POSITION_NOISE} m, angular accel '
         f'noise {cfg.TOUCH_KALMAN_ANGULAR_ACCEL_NOISE}, rotation noise {cfg.TOUCH_KALMAN_ROTATION_NOISE} rad.', '',
         '## 1. Stylus signal (engaged strokes)', '',
         '| Run | publish interval (ms) | share of rows with a new pose | interval between real updates while moving: median / p95 / p99 / max (ms) | '
         'real updates per 10 ms window: share with 1 / 2 / 3 / 4 | 10 ms difference vs reference velocity: median / p95 relative error |',
         '|---|---|---|---|---|---|']
    allrun = RUNS + ['V5/LongBenchmark', 'V9b/NoTool/PositionMode/LongBenchmark_Replay1']
    for k in allrun:
        r = runs.Run(os.path.join(runs.LOGS, k))
        s = stylus_stats(r)
        if s is None:
            L.append(f'| {k} | no moving stylus samples (replay: stylus not recorded live) | | | | |')
            continue
        c = s['cnt']
        L.append(f"| {k} | {s['pub_ms']:.2f} | {100 * s['share_changed']:.0f} % | {s['upd_med']:.1f} / {s['upd_p95']:.1f} / "
                 f"{s['upd_p99']:.1f} / {s['upd_max']:.1f} | {100 * c[1]:.0f} / {100 * c[2]:.0f} / {100 * c[3]:.0f} / {100 * c[4]:.0f} % | "
                 f"{100 * s['rel_med']:.0f} % / {100 * s['rel_p95']:.0f} % |")
    L += ['', '## 2. Stylus orientation tremor (rms of the band-passed deviation, deg)', '',
          'Only slow inputs: on the 1:1 benchmarks the 7–25 Hz band is dominated by deliberate fast pen twists, not tremor.', '',
          '| Run | 7–13 Hz | 15–25 Hz |', '|---|---|---|']
    for k in ('V3/Scaled5_Slow', 'V3/Scaled20_Slow', 'V5/JerkyAngling', 'V2/General_Motion_Slow'):
        r = runs.Run(os.path.join(runs.LOGS, k))
        tr = tremor(r)
        L.append(f"| {k} | {tr.get('7–13 Hz', np.nan):.3f} | {tr.get('15–25 Hz', np.nan):.3f} |")
    L += ['', 'A velocity feedforward multiplies an oscillation of frequency f by 2πf: '
          f'{2 * np.pi * 10:.0f} /s at 10 Hz, {2 * np.pi * 20:.0f} /s at 20 Hz.', '',
          '## 3. Kalman filter (node code) on the recorded stylus, at the run\'s publish rate', '',
          '| Run | rate (Hz) | linear noise > 10 Hz raw → Kalman (mm/s) | change | angular noise raw → Kalman (deg/s) | change | '
          'linear 7–13 Hz raw → Kalman | angular 7–13 Hz raw → Kalman | lag raw → Kalman, linear / angular (ms) |',
          '|---|---|---|---|---|---|---|---|---|']
    kal = {}
    for k in RUNS:
        r = runs.Run(os.path.join(runs.LOGS, k))
        fs, res = kalman_study(r)
        kal[k] = res
        rl, kl = res['raw']['lin'], res['kalman']['lin']
        ra, ka = res['raw']['ang'], res['kalman']['ang']
        L.append(f"| {k} | {fs:.0f} | {rl[0]:.1f} → {kl[0]:.1f} | {100 * (kl[0] / rl[0] - 1):+.0f} % | {ra[0]:.2f} → {ka[0]:.2f} | "
                 f"{100 * (ka[0] / ra[0] - 1):+.0f} % | {rl[1]:.2f} → {kl[1]:.2f} ({100 * (kl[1] / rl[1] - 1):+.0f} %) | "
                 f"{ra[1]:.3f} → {ka[1]:.3f} ({100 * (ka[1] / ra[1] - 1):+.0f} %) | {rl[2]:.0f} → {kl[2]:.0f} / {ra[2]:.0f} → {ka[2]:.0f} |")
    L += ['', '## 4. One Euro filter on the same samples (linear), settings that remove at least as much > 10 Hz noise as the Kalman filter', '',
          '| Run | Kalman: noise, lag (ms) | matching One Euro settings (min_cutoff Hz, beta) | their lag (ms): min–max | added lag vs Kalman (ms) |',
          '|---|---|---|---|---|']
    for k, res in kal.items():
        kn, _, kl = res['kalman']['lin']
        match = [x for x in res['oneeuro'] if x[2] <= kn]
        if match:
            lags = [x[4] for x in match]
            L.append(f"| {k} | {kn:.1f} mm/s, {kl:.0f} | " + ', '.join(f'({x[0]:g}, {x[1]:g})' for x in match) +
                     f" | {min(lags):.0f}–{max(lags):.0f} | {min(lags) - kl:+.0f} to {max(lags) - kl:+.0f} |")
        else:
            best = min(res['oneeuro'], key=lambda x: x[2])
            L.append(f"| {k} | {kn:.1f} mm/s, {kl:.0f} | none in the grid (lowest noise {best[2]:.1f} at {best[0]:g}, {best[1]:g}, lag {best[4]:.0f}) | – | – |")
    L.append('')
    L.append('Check: One Euro with min_cutoff 10 kHz (pass-through) reproduces raw differencing: ' + '; '.join(
        f"{k.split('/')[-1]} {res['oe_passthrough'][0]:.1f} vs {res['raw']['lin'][0]:.1f} mm/s" for k, res in kal.items()))
    L += ['', '## 5. Command filter (node LowPass, 2nd-order Butterworth, 500 Hz) on the recorded unfiltered twist (V9b VelocityUnfiltered, 1:5)', '',
          '| f_c (Hz) | group delay at 0.5 / 1 / 2 Hz (ms) | √2/(2π f_c) (ms) | amplitude left, linear: 8–12 / 15–25 / 30–50 Hz | angular: 8–12 / 15–25 / 30–50 Hz |',
          '|---|---|---|---|---|']
    cf = command_filter_study()
    for row in cf:
        L.append(f"| {row['fc']:g} | " + ' / '.join(f'{x:.1f}' for x in row['gd_ms']) + f" | {row['formula_ms']:.1f} | " +
                 ' / '.join(f"{100 * row[('lin', b)]:.0f} %" for b in ('8–12', '15–25', '30–50')) + ' | ' +
                 ' / '.join(f"{100 * row[('ang', b)]:.0f} %" for b in ('8–12', '15–25', '30–50')) + ' |')
    # figures
    L += ['', '## Figures', '']
    show = [k for k in kal if kal[k]['raw']['lin'][0] > 5]           # inputs with translation
    fig, axes = plt.subplots(1, len(show), figsize=(FULL, 2.3), sharey=False)
    names = {'LongBenchmark': 'Long benchmark', 'ShortBenchmark': 'Short benchmark', 'Scaled5_Slow': '1:5 slow'}
    for ax, k, letter in zip(np.atleast_1d(axes), show, 'abcdef'):
        res = kal[k]
        oe = np.array([(x[4], x[2]) for x in res['oneeuro']])
        ax.plot(oe[:, 0], oe[:, 1], 'o', ms=3, color=MUTED, mfc='none', label='One Euro (grid)')
        ax.plot(res['raw']['lin'][2], res['raw']['lin'][0], 's', ms=6, color='#D55E00', label='Raw differencing')
        ax.plot(res['kalman']['lin'][2], res['kalman']['lin'][0], 'D', ms=6, color='#0072B2', label='Kalman (node)')
        ax.axhline(res['kalman']['lin'][0], color='#0072B2', lw=0.6, ls=':')
        panel(ax, letter, names.get(k.split('/')[-1], k.split('/')[-1])); ax.set_xlabel('Lag (ms)')
    np.atleast_1d(axes)[0].set_ylabel('Feedforward noise > 10 Hz (mm/s)')
    np.atleast_1d(axes)[0].legend(fontsize=7)
    L.append(f"- `figures/{os.path.basename(save(fig, HERE, 'stylus_filter_tradeoff'))}`: noise above 10 Hz in the "
             "feedforward velocity against lag, raw differencing vs the node's Kalman filter vs a One Euro grid "
             "(dotted = the Kalman level)")
    fig, ax = plt.subplots(figsize=(FULL * 0.6, 2.4))
    fcs = [r['fc'] for r in cf]
    for (nm, b), c, mk in ((('lin', '15–25'), '#0072B2', 'o'), (('ang', '15–25'), '#0072B2', 's'),
                           (('lin', '8–12'), '#E69F00', 'o'), (('lin', '30–50'), '#D55E00', 'o')):
        ax.plot(fcs, [100 * r[(nm, b)] for r in cf], marker=mk, color=c, mfc='white' if nm == 'ang' else c,
                label=f"{b} Hz, {'linear' if nm == 'lin' else 'angular'}")
    for r in cf:
        ax.annotate(f"{r['gd_ms'][0]:.0f} ms", (r['fc'], 100 * r[('lin', '8–12')]), xytext=(0, 5),
                    textcoords='offset points', fontsize=7, ha='center', color=INK2)
    ax.set_xlabel('Filter cutoff $f_c$ (Hz)'); ax.set_ylabel('Amplitude left (%)')
    ax.legend(fontsize=7)
    L.append(f"- `figures/{os.path.basename(save(fig, HERE, 'command_filter_offline'))}`: amplitude left per band by "
             "the node's 2nd-order Butterworth at each cutoff, on the recorded unfiltered commands; labels = low-frequency "
             "group delay")
    L += ['', '## Prior claims checked', '',
          '- "pose updates every 4 ms (sometimes 12 ms) while publishing at 1 kHz; up to ±33 % noise on a 10 ms difference"; '
          '"tremor 0.025° rms at 7–13 Hz"; "Kalman cut linear noise 23–42 %, angular 17–37 %, no added lag"; '
          '"One Euro settings that removed as much noise added 30–50 ms"; "Kalman alone left the tremor band unchanged"; '
          '"10 Hz leaves 23 % of 15–25 Hz for 22 ms, 8 Hz 14 % / 28 ms, 6 Hz 8 % / 37 ms; τ ≈ √2/(2π f_c)": see the tables.']
    open(os.path.join(HERE, 'RESULTS.md'), 'w').write('\n'.join(L) + '\n')
    print('\n'.join(L))


if __name__ == '__main__':
    main()
