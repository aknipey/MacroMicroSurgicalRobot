#!/usr/bin/env python3
"""L06 Closed-loop latency model (rebuilt 2026-11-13: causal time-domain simulation).

Why rebuilt: the first version computed the loop in the frequency domain. With the delays its fit chose, that loop
was unstable, and an FFT of an unstable loop silently returns a non-causal answer; it also left out the
correction-speed cap, the feedforward filter, the delayed-target correction and the clutch resets. That version
failed validation (62 / 211 ms rms).

Model (nothing is tuned to match lag):
1. PLANT, measured per output mode: everything after the teleop node (MoveIt Servo, joint limiter, controller,
   arm), identified from the bags as  tool0 velocity (FK of /joint_states) = k * e^{-s Td} / (tau s + 1) * twist
   (/servo_node/delta_twist_cmds, linear part), grid search on header stamps. Each run is predicted with the median
   plant of the OTHER trajectories of its mode (leave-one-trajectory-out).
2. TELEOP LAW, replicated step by step from src/ur16e_touch_teleop.py update_motion() (non-RCM branch) at the run's
   publish rate, with the node's own LowPass: feedforward of the low-passed target velocity, correction
   Kp (target(t - loop_delay) - robot) capped at max_correction above the deadband, total capped at max_linear, the
   command filter, and every state reset at each clutch stroke. All values from the run's META.
3. Driven by the run's recorded target (TOUCH_PATH); the simulated tool0 path gets the same lag estimator as the
   real arm. Gate: the model's conclusions are reported as usable only if its rms error is within 10 ms.
Trajectory output is not modelled (not a velocity loop through this law).
"""
import os
import sys

import numpy as np
from scipy.signal import welch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))  # thesis/results
from common import bags, kin, runs, track  # noqa: E402
from common.style import FULL, HALF, INK2, MODE, MUTED, save, style  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, runs.PKG)
from macro_micro_robot.ur16e_common import LowPass  # noqa: E402


def clamp_norm(v, limit):          # identical to src/ur16e_touch_teleop.py clamp_norm (the node is not importable)
    n = np.linalg.norm(v)
    return v * (limit / n) if n > limit else v


MODES = ('position', 'velocity_optimised', 'velocity_unfiltered')
GATE_MS = 10.0


# ------------------------------------------------------------------ 1. plant
def identify_plant(run):
    b = bags.load(run, ['/joint_states', '/servo_node/delta_twist_cmds'])
    if '/servo_node/delta_twist_cmds' not in b or '/joint_states' not in b:
        return None
    tw, js = b['/servo_node/delta_twist_cmds'], b['/joint_states']
    K = kin.load(run.kinematics[1] or 'nominal')
    g = np.arange(max(tw['th'][0], js['th'][0]) + 0.2, min(tw['th'][-1], js['th'][-1]) - 0.2, 0.01)
    u = np.column_stack([np.interp(g, tw['th'], tw['lin'][:, k]) for k in range(3)])
    Q = np.column_stack([np.interp(g, js['th'], js['q'][:, k]) for k in range(6)])
    P = np.array([kin.tool0(K, q)[:3, 3] for q in Q])
    y = np.gradient(P, g, axis=0)                         # tool0 velocity at 100 Hz
    mv = np.linalg.norm(u, axis=1) > 1e-3
    if mv.sum() < 100:
        return None
    best = (np.inf, None)
    for Td in np.arange(0.0, 0.121, 0.01):
        sh = int(round(Td / 0.01))
        ud = np.vstack([np.repeat(u[:1], sh, 0), u[:len(u) - sh]]) if sh else u
        for tau in np.arange(0.0, 0.081, 0.004):
            a = np.exp(-0.01 / tau) if tau > 0 else 0.0
            f = np.zeros_like(ud)
            for i in range(1, len(ud)):
                f[i] = a * f[i - 1] + (1 - a) * ud[i]
            num, den = np.sum(f[mv] * y[mv]), np.sum(f[mv] * f[mv])
            k = num / den if den > 0 else 1.0
            e = np.mean(np.sum((k * f[mv] - y[mv]) ** 2, 1))
            if e < best[0]:
                best = (e, (k, Td, tau))
    k, Td, tau = best[1]
    # refine the dead time at 2 ms around the coarse optimum (sub-sample shift by linear interpolation)
    best2 = (np.inf, Td)
    for Td2 in np.arange(max(0, Td - 0.01), Td + 0.0101, 0.002):
        ud = np.column_stack([np.interp(g - Td2, g, u[:, j]) for j in range(3)])
        a = np.exp(-0.01 / tau) if tau > 0 else 0.0
        f = np.zeros_like(ud)
        for i in range(1, len(ud)):
            f[i] = a * f[i - 1] + (1 - a) * ud[i]
        e = np.mean(np.sum((k * f[mv] - y[mv]) ** 2, 1))
        best2 = min(best2, (e, Td2))
    r2 = 1 - best2[0] / np.mean(np.sum((y[mv] - y[mv].mean(0)) ** 2, 1))
    return dict(k=float(k), Td=float(best2[1]), tau=float(tau), r2=float(r2))


# ------------------------------------------------------------------ 2. teleop law + plant, step by step
def simulate(run, plant, fc=None, Kp=None):
    s = run.settings
    rate = float(s.get('TOUCH_PUBLISH_RATE', 500.0))
    dt = 1.0 / rate
    fc = s.get('TOUCH_COMMAND_FILTER', 0.0) if fc is None else fc
    Kp = s.get('TOUCH_POSITION_GAIN', 8.0) if Kp is None else Kp
    kff = s.get('TOUCH_FEEDFORWARD', 1.0)
    ffc = s.get('TOUCH_FEEDFORWARD_FILTER', 0.0)
    ld = s.get('TOUCH_LOOP_DELAY', 0.0)
    cap = s.get('TOUCH_MAX_CORRECTION_SPEED') or np.inf
    vmax = s.get('TOUCH_MAX_LINEAR_SPEED') or np.inf
    dead = s.get('TOUCH_DEADBAND', 0.0)
    p_ = track.load_paths(run)
    if p_ is None:
        return None
    (tt, tp, _), (ut, up, _) = p_
    tp = tp / 1000.0
    k, Td, tau = plant['k'], plant['Td'], plant['tau']
    nd = int(round(Td / dt))
    a = np.exp(-dt / tau) if tau > 0 else 0.0
    t_all, p_all, v_all = [], [], []
    strokes = track.strokes(tt)
    p = np.array([np.interp(strokes[0][0], ut, up[:, j]) for j in range(3)]) / 1000.0
    vel = np.zeros(3)                                   # plant output (tool0 velocity)
    hist = [np.zeros(3)] * (nd + 1)                     # twist history for the dead time
    t_prev_end = None
    for a0, b0 in strokes:
        if t_prev_end is not None:                      # between strokes: zero twist, the plant settles
            for t in np.arange(t_prev_end, a0, dt):
                hist.append(np.zeros(3)); u = hist.pop(0)
                vel = a * vel + (1 - a) * k * u
                p = p + vel * dt
                t_all.append(t); p_all.append(p.copy()); v_all.append(vel.copy())
        filt = LowPass(fc, rate, 3) if fc and fc > 0 else None       # reset at engage, as the node does
        tv = np.zeros(3)
        prev = None
        thist = []
        ts = np.arange(a0, b0, dt)
        targets = np.column_stack([np.interp(ts, tt, tp[:, j]) for j in range(3)])
        for t, target in zip(ts, targets):
            if prev is not None:
                tv += dt / (dt + ffc) * ((target - prev) / dt - tv)
            prev = target
            thist.append((t, target))
            while len(thist) > 1 and thist[1][0] <= t - ld:
                thist.pop(0)
            err = thist[0][1] - p
            lin = kff * tv
            if np.linalg.norm(err) >= dead:
                lin = lin + clamp_norm(Kp * err, cap)
            lin = clamp_norm(lin, vmax)
            if filt is not None:
                lin = clamp_norm(filt(lin), vmax)
            hist.append(lin); u = hist.pop(0)
            vel = a * vel + (1 - a) * k * u
            p = p + vel * dt
            t_all.append(t); p_all.append(p.copy()); v_all.append(vel.copy())
        t_prev_end = b0
    return np.array(t_all), np.array(p_all) * 1000.0, np.array(v_all) * 1000.0


def lag_fast(run, t_arm, P_arm, hi=0.6):
    """Shift minimising rms position error over the strokes (same definition as common.track lag_ms), 100 Hz grid."""
    (tt, tp, _), _ = track.load_paths(run)
    sg = np.concatenate([np.arange(a, b, 0.01) for a, b in track.strokes(tt)])
    tgt = np.column_stack([np.interp(sg, tt, tp[:, k]) for k in range(3)])

    def cost(s):
        a = np.column_stack([np.interp(sg + s, t_arm, P_arm[:, k], left=np.nan, right=np.nan) for k in range(3)])
        return np.sqrt(np.nanmean(np.sum((a - tgt) ** 2, 1)))
    s = np.arange(0, hi, 0.002)
    c = np.array([cost(x) for x in s])
    i = int(np.nanargmin(c))
    f = np.arange(max(0, s[i] - 0.003), s[i] + 0.003, 0.00025)
    return 1e3 * f[int(np.nanargmin([cost(x) for x in f]))]


def band_power(t, v, lo, hi):
    fs = 1.0 / np.median(np.diff(t))
    f, P = welch(v - np.mean(v, 0), fs=fs, nperseg=min(512, len(v)), axis=0)
    m = (f >= lo) & (f <= hi)
    return float(np.trapz(P[m].sum(1), f[m]))


def main():
    style()
    rs = [r for r in runs.find_runs('V9b/NoTool/*/*_Replay*') if r.mode in MODES]
    meas = {r.key: track.cached_metrics(r) for r in rs}
    rs = [r for r in rs if meas[r.key] and not any(g > 0.05 for _, g in meas[r.key]['path_gaps'])]

    plants = {}
    for r in rs:
        pl = identify_plant(r)
        if pl:
            plants[r.key] = pl
            print('plant', r.key, pl, flush=True)
    L = ['# L06 Results: closed-loop latency model (causal simulation)', '',
         'Generated by `analysis.py`. See its docstring for why it was rebuilt. Nothing is tuned to match lag.', '',
         '## 1. Plant per run: twist → tool0 velocity (Servo + limiter + controller + arm)', '',
         '| Run | mode | gain k | dead time (ms) | lag tau (ms) | R² |', '|---|---|---|---|---|---|']
    for r in rs:
        if r.key in plants:
            p = plants[r.key]
            L.append(f"| {r.key} | {r.mode} | {p['k']:.2f} | {p['Td'] * 1e3:.0f} | {p['tau'] * 1e3:.0f} | {p['r2']:.3f} |")

    def loto(r):
        others = [plants[o.key] for o in rs if o.mode == r.mode and o.trajectory != r.trajectory and o.key in plants]
        return {k: float(np.median([o[k] for o in others])) for k in ('k', 'Td', 'tau')} if others else None

    rows = []
    for r in rs:
        pl = loto(r)
        if pl is None:
            continue
        sim = simulate(r, pl)
        if sim is None:
            continue
        _, (ut, up, _) = track.load_paths(r)
        mlag = lag_fast(r, ut, up)
        plag = lag_fast(r, sim[0], sim[1])
        rows.append(dict(run=r, plant=pl, meas=mlag, pred=plag, track=meas[r.key]['lag_ms']))
        print(r.key, f'measured {mlag:.1f} predicted {plag:.1f}', flush=True)
    err = np.array([o['pred'] - o['meas'] for o in rows])
    rms = float(np.sqrt(np.mean(err ** 2)))
    est = np.array([o['meas'] - o['track'] for o in rows])
    L += ['', '## 2. Prediction of every run (plant from the other trajectories of the same mode)', '',
          f'Lag estimator check: `lag_fast` vs common.track on the real arm: median {np.median(est):+.1f} ms, max '
          f'|{np.max(np.abs(est)):.1f}| ms.', '',
          '| Run | mode | scale | measured (ms) | predicted (ms) | error (ms) |', '|---|---|---|---|---|---|']
    for o in rows:
        L.append(f"| {o['run'].key} | {o['run'].mode} | 1:{1 / o['run'].scale:.0f} | {o['meas']:.1f} | {o['pred']:.1f} | "
                 f"{o['pred'] - o['meas']:+.1f} |")

    def by(sel):
        e = [o['pred'] - o['meas'] for o in rows if sel(o)]
        return float(np.sqrt(np.mean(np.square(e)))) if e else np.nan
    rms15 = by(lambda o: o['run'].scale < 1)
    L += ['', f'rms error: all {rms:.1f} ms (n = {len(rows)}); 1:5 runs {rms15:.1f} ms; 1:1 runs '
          f'{by(lambda o: o["run"].scale == 1):.1f} ms. Replay floor (folder M01): 0.7 ms SD.']
    ok15, ok = rms15 <= GATE_MS, rms <= GATE_MS
    L += ['', f"**Validation gate (rms ≤ {GATE_MS:.0f} ms):** all runs {'PASSED' if ok else 'FAILED'}; 1:5 runs "
          f"{'PASSED' if ok15 else 'FAILED'}. Sections 3–4 are usable for the report only if the 1:5 gate passed "
          "(they are computed on 1:5 inputs)."]

    # 3. filter cost inside the loop: same plant and input, filter on vs off, each filtered 1:5 run
    L += ['', '## 3. Filter cost inside the loop (same plant and input; only the command filter changes)', '',
          '| Run | mode | f_c (Hz) | lag with filter (ms) | without (ms) | added by the filter (ms) | open-loop 0.225/f_c (ms) |',
          '|---|---|---|---|---|---|---|']
    costs = []
    for o in rows:
        r = o['run']
        fc = r.settings.get('TOUCH_COMMAND_FILTER', 0.0)
        if r.scale == 1 or not fc:
            continue
        on = lag_fast(r, *simulate(r, o['plant'])[:2])
        off = lag_fast(r, *simulate(r, o['plant'], fc=0.0)[:2])
        costs.append(on - off)
        L.append(f'| {r.key} | {r.mode} | {fc} | {on:.1f} | {off:.1f} | {on - off:.1f} | {225 / fc:.0f} |')
    ref = next(o for o in rows if o['run'].mode == 'position' and 'Scaled5_Fast' in o['run'].key)
    fcs = [2, 3, 4, 5, 6, 8, 10, 12, 15]
    base_t, base_p, base_v = simulate(ref['run'], ref['plant'], fc=0.0)
    base_lag = lag_fast(ref['run'], base_t, base_p)
    base_bp = band_power(base_t, base_v, 15, 25)
    curve = []
    for fc in fcs:
        t_, p_, v_ = simulate(ref['run'], ref['plant'], fc=fc)
        curve.append((fc, lag_fast(ref['run'], t_, p_) - base_lag, 1 - band_power(t_, v_, 15, 25) / base_bp))
    L += ['', f"Curve on `{ref['run'].key}` (its leave-one-out position plant):", '',
          '| f_c (Hz) | open loop 0.225/f_c (ms) | closed loop, model (ms) | 15–25 Hz reduction of model tool velocity |',
          '|---|---|---|---|']
    L += [f'| {fc} | {225 / fc:.0f} | {c:.1f} | {100 * red:.0f} % |' for fc, c, red in curve]
    t5, p5, v5 = simulate(ref['run'], ref['plant'])
    lowr = band_power(t5, v5, 1, 8) / band_power(base_t, base_v, 1, 8)
    L += ['', 'Folder L07 measured more 1–8 Hz flange motion with filtering (up to 2.6–3.7× power). The model, same '
          f'input: 1–8 Hz tool-velocity power ratio filtered/unfiltered = {lowr:.2f}'
          + (' → reproduced.' if lowr > 1.3 else ' → not reproduced by the teleop law + plant alone.')]
    fgrid = [2, 4, 6, 8, 10, 12, 15, 20]; kgrid = [2, 4, 6, 8, 12, 16, 20]
    lag = np.zeros((len(kgrid), len(fgrid))); red = np.zeros_like(lag)
    for i, Kp in enumerate(kgrid):
        b_t, b_p, b_v = simulate(ref['run'], ref['plant'], fc=0.0, Kp=Kp)
        bb = band_power(b_t, b_v, 15, 25)
        for j, fc in enumerate(fgrid):
            t_, p_, v_ = simulate(ref['run'], ref['plant'], fc=fc, Kp=Kp)
            lag[i, j] = lag_fast(ref['run'], t_, p_)
            red[i, j] = 1 - band_power(t_, v_, 15, 25) / bb
    # Threshold (i): 30 % less amplitude = 1 - 0.7**2 = 51 % less band power (user decision 2026-11-15)
    REQ = 1 - 0.7 ** 2
    feas = (lag <= 50) & (red >= REQ)
    s = ref['run'].settings
    L += ['', '## 4. Design map (position plant, same 1:5 input)', '',
          f'Feasible (lag ≤ 50 ms and ≥ {100 * REQ:.0f} % 15–25 Hz power reduction, i.e. 30 % in amplitude): {int(feas.sum())} of {feas.size}: '
          + (', '.join(f'f_c {fgrid[j]} Hz / Kp {kgrid[i]} ({lag[i, j]:.1f} ms, {100 * red[i, j]:.0f} %)' for i, j in zip(*np.where(feas))) or 'none') + '.',
          'Reduction at f_c 20 Hz by Kp: ' + ', '.join(f'Kp {kgrid[i]}: {100 * red[i, -1]:.0f} % / {lag[i, -1]:.1f} ms' for i in range(len(kgrid))) + '.',
          f"Shipped: f_c {s.get('TOUCH_COMMAND_FILTER')} Hz, Kp {s.get('TOUCH_POSITION_GAIN')}. Reduction here is of the "
          'simulated tool velocity (the command path through the plant), not measured arm vibration (folder L07).', '',
          '## Figures', '']
    if not ok15:
        L.insert(2, '> **MODEL NOT VALIDATED on the 1:5 runs (gate %.0f ms).** Sections 3–4 are not usable for the report.\n'
                 % GATE_MS)

    fig, ax = plt.subplots(figsize=(FULL * 0.55, 2.6))
    for o in rows:
        st = MODE[o['run'].mode]
        ax.scatter(o['meas'], o['pred'], marker=st['marker'], edgecolor=st['color'], s=18,
                   facecolor=st['color'] if o['run'].scale < 1 else 'none')
    for m in dict.fromkeys(o['run'].mode for o in rows):
        ax.scatter([], [], marker=MODE[m]['marker'], color=MODE[m]['color'], s=18, label=MODE[m]['label'])
    lim = [0, max(max(o['meas'], o['pred']) for o in rows) * 1.1]
    ax.plot(lim, lim, color=INK2, lw=0.7)
    ax.fill_between(lim, [x - GATE_MS for x in lim], [x + GATE_MS for x in lim], color=MUTED, alpha=0.2, lw=0)
    ax.set_xlabel('Measured lag (ms)'); ax.set_ylabel('Predicted lag (ms)'); ax.legend(loc='upper left')
    L.append(f"- `figures/{os.path.basename(save(fig, HERE, 'predicted_vs_measured'))}`")

    fig, ax = plt.subplots(figsize=(FULL * 0.6, 2.4))
    ax.plot(fcs, [225 / fc for fc in fcs], color=INK2, ls='--', label='Open loop, $0.225/f_c$')
    ax.plot(fcs, [c for _, c, _ in curve], color=MODE['position']['color'], marker='o', ms=4,
            label='Closed loop (model)')
    if costs:
        ax.scatter([5] * len(costs), costs, color='k', s=10, zorder=3, label='Filtered 1:5 replays (model)')
    # Measured on the arm (session D, folder L09): position output, lag with the filter minus lag without it
    mpath = os.path.join(runs.folder('L09'), 'measured_filter_lag.json')
    if os.path.exists(mpath):
        import json
        meas = json.load(open(mpath))
        for tr, face, lab in (('Scaled5_Fast', MODE['position']['color'], 'Measured, 1:5 fast'),
                              ('Scaled5_Slow', 'none', 'Measured, 1:5 slow')):
            pts = sorted((float(k), v) for k, v in meas.get(tr, {}).items())
            if pts:
                ax.scatter(*zip(*pts), marker='^', s=30, zorder=4, facecolor=face,
                           edgecolor=MODE['position']['color'], label=lab)
    ax.set_xlabel('Filter cutoff $f_c$ (Hz)'); ax.set_ylabel('Added lag (ms)'); ax.legend()
    L.append(f"- `figures/{os.path.basename(save(fig, HERE, 'filter_cost'))}`")

    fig, ax = plt.subplots(figsize=(HALF * 1.5, 2.8))
    cs = ax.contourf(fgrid, kgrid, lag, levels=20, cmap='thesis_seq')
    ax.contour(fgrid, kgrid, lag, levels=[50], colors='white', linewidths=1.5)
    ax.contourf(fgrid, kgrid, red, levels=[-10, REQ], colors='none', hatches=['///'])
    ax.plot(s.get('TOUCH_COMMAND_FILTER'), s.get('TOUCH_POSITION_GAIN'), '*', color='white', ms=10, mec='k')
    fig.colorbar(cs, ax=ax, label='Model lag (ms)')
    ax.set_xlabel('Filter cutoff $f_c$ (Hz)'); ax.set_ylabel('Position gain $K_p$ (s$^{-1}$)')
    ax.set_title(r'White: 50 ms; hatched: < 51 % power reduction; $\star$: shipped', fontsize=8)
    L.append(f"- `figures/{os.path.basename(save(fig, HERE, 'design_map'))}`")
    open(os.path.join(HERE, 'RESULTS.md'), 'w').write('\n'.join(L) + '\n')
    print('\n'.join(L[-45:]))


if __name__ == '__main__':
    main()
