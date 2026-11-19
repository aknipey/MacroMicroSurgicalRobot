#!/usr/bin/env python3
"""17 Slow-speed 'catches': robot speed dips vs the Touch's own gimbal encoders.

Data: /phantom/joint_states (waist, shoulder, elbow, yaw, pitch, roll encoder angles, ~1 kHz; recorded from
V5/JerkyAngling on), read here directly (own cache), on the CSV clock via common.bags.to_run_clock.
1. Encoder stalls, every run with the topic: for each Touch joint, gaps between successive count changes >= 15 ms
   while the joint moves (its speed, 50 ms centred difference over the gap's neighbours, >= 2 deg/s on both sides);
   reported with the joint speed just before the stall. Slow / fast passes compared.
2. Robot catches in V5/JerkyAngling (pen rocking, 1:1): dips in the robot's tool angular speed (UR_PATH orientation,
   15 Hz zero-phase smoothed) below 50 % of the larger local peak within +-0.3 s, with that peak >= 10 deg/s. For each,
   the Touch joint speeds at the dip time minus the run's rotation lag (common.track).
3. What smoothing would remove them: the target's angular speed through the node's command low-pass (LowPass,
   2nd order) at several cutoffs; dips left (same rule, on the filtered target) and the filter's delay sqrt(2)/(2 pi fc).
Writes RESULTS.md.
"""
import glob
import os
import sys

import numpy as np
from scipy.signal import butter, sosfiltfilt
from scipy.spatial.transform import Rotation

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from common import bags, runs, track  # noqa: E402
from macro_micro_robot.ur16e_common import LowPass  # noqa: E402
from common.style import FULL, INK2, MUTED, save, style  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

NAMES = ['waist', 'shoulder', 'elbow', 'yaw', 'pitch', 'roll']
CACHE = os.path.join(HERE, 'cache')


def touch_joints(run):
    os.makedirs(CACHE, exist_ok=True)
    bagdir = os.path.join(run.path, 'rosbag')
    cp = os.path.join(CACHE, run.key.replace('/', '__') + f'_{int(os.path.getmtime(bagdir))}.npz')
    if os.path.exists(cp):
        z = np.load(cp)
        return z['t'], z['q']
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bagdir), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    if '/phantom/joint_states' not in types:
        return None, None
    r.set_filter(rosbag2_py.StorageFilter(topics=['/phantom/joint_states']))
    cls = get_message(types['/phantom/joint_states'])
    T, Q = [], []
    while r.has_next():
        _, raw, tr = r.read_next()
        m = deserialize_message(raw, cls)
        idx = [list(m.name).index(n) for n in NAMES]
        T.append(tr * 1e-9); Q.append([m.position[i] for i in idx])
    T, Q = np.array(T), np.degrees(np.array(Q))
    np.savez_compressed(cp, t=T, q=Q)
    return T, Q


def runs_with_topic():
    out = []
    for r in runs.find_runs('*/*') + runs.find_runs('*/*/*') + runs.find_runs('*/*/*/*'):
        m = os.path.join(r.path, 'rosbag', 'metadata.yaml')
        if os.path.exists(m) and '/phantom/joint_states' in open(m).read() and r.has('TOUCH_PATH.csv'):
            if r.replay or r.replay_n:           # replays reproduce the stylus pose, not the encoders
                continue
            out.append(r)
    return out


def stalls(t, q, spans):
    """Per joint: stalls = gaps between successive encoder count changes that are >= 15 ms AND >= 3x the interval
    expected at the joint's speed on both sides (speed from the 15 Hz zero-phase smoothed angle 30 ms before the gap
    and 30 ms after it, both >= 5 deg/s). Returns {joint: [(gap ms, speed before, speed after)]} and the smoothed
    speeds of all moving samples, plus the count size per joint (smallest step)."""
    res = {n: [] for n in NAMES}
    held = {n: 0 for n in NAMES}
    moving = {n: [] for n in NAMES}
    count = {}
    g = np.arange(t[0], t[-1], 0.002)
    chg_all = [t[np.where(np.abs(np.diff(q[:, k])) > 1e-4)[0] + 1] for k in range(6)]
    vs_all = [np.abs(np.gradient(sosfiltfilt(butter(2, 15, fs=500, output='sos'), np.interp(g, t, q[:, k])), 0.002))
              for k in range(6)]
    for j, n in enumerate(NAMES):
        x = q[:, j]
        dx = np.abs(np.diff(x))
        count[n] = float(dx[dx > 1e-4].min()) if (dx > 1e-4).any() else np.nan
        xs = sosfiltfilt(butter(2, 15, fs=500, output='sos'), np.interp(g, t, x))
        vs = np.abs(np.gradient(xs, 0.002))
        ch = np.where(np.abs(np.diff(x)) > 1e-4)[0] + 1
        tc = t[ch]
        for a, b in spans:
            sel = np.where((tc >= a + 0.1) & (tc <= b - 0.1))[0]
            mg = (g >= a) & (g <= b)
            moving[n] += list(vs[mg][vs[mg] >= 5])
            for i in sel[1:]:
                t0, t1 = t[ch[i - 1]], tc[i]
                gap = t1 - t0
                vb = np.interp(t0 - 0.03, g, vs); va = np.interp(t1 + 0.03, g, vs)
                if vb >= 5 and va >= 5 and gap >= 0.015 and gap >= 3 * count[n] / min(vb, va):
                    # joint-specific only if another moving joint changed count inside the gap and the stream had no hole
                    msg_hole = np.max(np.diff(t[(t >= t0) & (t <= t1)]), initial=0) > 0.01
                    others = [k for k in range(6) if k != j and np.interp((t0 + t1) / 2, g, vs_all[k]) >= 5]
                    other_moved = any(((chg_all[k] > t0) & (chg_all[k] < t1)).any() for k in others)
                    if msg_hole or not other_moved:
                        held[n] += 1
                    else:
                        res[n].append((gap * 1000, vb, va))
    return res, moving, count, held


def angular_speed(t, Q, fs=500.0, fc=15.0):
    g = np.arange(t[0], t[-1], 1 / fs)
    R = track._slerp(t, Q, g)
    w = np.linalg.norm((R[1:] * R[:-1].inv()).as_rotvec(), axis=1) * fs
    w = sosfiltfilt(butter(2, fc, fs=fs, output='sos'), np.degrees(w))
    return g[1:], w


def dips(g, w, thr=10.0, frac=0.5, win=0.3, fs=500.0):
    n = int(win * fs)
    out = []
    i = n
    while i < len(w) - n:
        if w[i] == w[i - 5:i + 6].min():
            pk = max(w[i - n:i].max(), w[i:i + n].max())
            if pk >= thr and w[i] < frac * pk:
                out.append((g[i], w[i] / pk))
                i += n // 2
                continue
        i += 1
    return out


def main():
    style()
    rates = {}
    L = ['# 17 Results: slow-speed catches vs the Touch gimbal encoders', '',
         'Generated by `analysis.py`. Encoder data: /phantom/joint_states (deg), live recordings only (replays reproduce the '
         'stylus pose, not the encoders).', '']
    # 1. stalls in every run with the topic
    allr = runs_with_topic()
    pooled = {n: [] for n in NAMES}
    mov = {n: [] for n in NAMES}
    holds = {n: 0 for n in NAMES}
    jerky = None
    for r in allr:
        t, q = touch_joints(r)
        if t is None:
            continue
        b = bags.load(r, ['/tcp_pose_broadcaster/pose'])
        off = bags.to_run_clock(r, b)
        if off is None:
            continue
        spans = track.strokes(r.csv('TOUCH_PATH.csv')['t'])
        st, ms, cnt, hd = stalls(t - off, q, spans)
        if r.key == 'V5/JerkyAngling':
            jerky = {n: (len(st[n]), len(ms[n]) * 0.002) for n in NAMES}
        for n in NAMES:
            pooled[n] += st[n]; mov[n] += ms[n]; holds[n] += hd[n]
    L += [f'## 1. Encoder stalls ({len(allr)} live runs: ' + ', '.join(r.key for r in allr) + ')', '',
          'Stall = gap between count changes ≥ 15 ms and ≥ 3× the interval expected at the joint speed on both sides '
          '(15 Hz-smoothed speed 30 ms before and after, both ≥ 5 deg/s), and joint-specific: another moving joint changed '
          'count during it and the message stream had no hole > 10 ms (otherwise counted as a whole-device hold). Count size per joint: '
          + ', '.join(f'{n} {c:.4f}°' for n, c in cnt.items()) + '.', '',
          '| Touch joint | joint-specific stalls (whole-device holds) | per minute moving | V5/JerkyAngling: stalls / moving s | duration: median / max (ms) | speed before → after a stall: medians (deg/s) | speed before: range (deg/s) | '
          'moving time ≥ 5 deg/s: speed p10–p90 (deg/s) |', '|---|---|---|---|---|---|---|---|']
    for n in NAMES:
        s = np.array(pooled[n]) if pooled[n] else np.zeros((0, 3))
        m = np.array(mov[n])
        rng = f'{s[:, 1].min():.0f}–{s[:, 1].max():.0f}' if len(s) else '–'
        dur = f'{np.median(s[:, 0]):.0f} / {s[:, 0].max():.0f}' if len(s) else '–'
        ba = f'{np.median(s[:, 1]):.0f} → {np.median(s[:, 2]):.0f}' if len(s) else '–'
        mp = f'{np.percentile(m, 10):.0f}–{np.percentile(m, 90):.0f}' if len(m) else '–'
        mins = len(m) * 0.002 / 60
        jk = jerky.get(n, (0, 0)) if jerky else (0, 0)
        rates[n] = len(s) / mins
        L.append(f'| {n} | {len(s)} ({holds[n]}) | {len(s) / mins:.1f} | {jk[0]} / {jk[1]:.1f} | {dur} | {ba} | {rng} | {mp} |')

    # 2. catches in JerkyAngling
    r = runs.Run(os.path.join(runs.LOGS, 'V5/JerkyAngling'))
    m = track.cached_metrics(r)
    lag = m['lag_rot_ms'] / 1000.0
    (tt, tp, tq), (ut, up, uq) = track.load_paths(r)
    spans = track.strokes(tt)
    gr, wr = angular_speed(ut, uq)
    gt, wt = angular_speed(tt, tq)
    keep = np.zeros(len(gr), bool)
    for a, b in spans:
        keep |= (gr >= a + 0.3) & (gr <= b + lag - 0.3)
    dd = [d for d in dips(gr, wr) if keep[np.searchsorted(gr, d[0]) - 1]]
    t, q = touch_joints(r)
    off = bags.to_run_clock(r, bags.load(r, ['/tcp_pose_broadcaster/pose']))
    t = t - off
    g = np.arange(t[0], t[-1], 0.002)
    qs = np.column_stack([np.interp(g, t, q[:, k]) for k in range(6)])
    v = np.abs(np.gradient(sosfiltfilt(butter(2, 15, fs=500, output='sos'), qs, axis=0), 0.002, axis=0))
    L += ['', f'## 2. Robot catches in V5/JerkyAngling (rotation lag {1000 * lag:.0f} ms, common.track)', '',
          '| robot dip at (s) | robot speed at dip (% of local peak) | pitch speed 0.15 s before → at → 0.15 s after (deg/s, at dip − lag) | '
          'yaw / roll max over ±0.1 s (deg/s) | pitch stopped (< 1 deg/s) |', '|---|---|---|---|---|']
    stopped = 0
    stops = []
    for td, fr in dd:
        tc = td - lag
        i0, i1, i2 = (np.searchsorted(g, x) for x in (tc - 0.15, tc, tc + 0.15))
        w = (g > tc - 0.1) & (g < tc + 0.1)
        pmin = v[w, 4].min()
        stopped += pmin < 1.0
        if pmin < 1.0:
            stops.append((fr, td))
        L.append(f'| {td:.2f} | {100 * fr:.0f} % | {v[i0, 4]:.1f} → {pmin:.1f} → {v[i2, 4]:.1f} | '
                 f'{v[w, 3].max():.1f} / {v[w, 5].max():.1f} | {"yes" if pmin < 1.0 else "no"} |')
    L.append(f'\n{len(dd)} robot dips; the pitch encoder stopped in {stopped}.')
    # target dips (are the dips already in the target the robot follows?)
    tdips = [d for d in dips(gt, wt) if any(a + 0.3 <= d[0] <= b - 0.3 for a, b in spans)]
    match = sum(any(abs(x[0] - (d[0] - lag)) < 0.06 for x in tdips) for d in dd)
    L.append(f'{len(tdips)} dips in the target (operator side); {match} of the {len(dd)} robot dips have a target dip within 60 ms '
             'after removing the lag.')

    # 3. smoothing needed
    L += ['', '## 3. Low-pass needed to remove them (target angular speed through the node LowPass, causal)', '',
          '| f_c (Hz) | target dips left | delay √2/(2π f_c) (ms) |', '|---|---|---|']
    fs = 500.0
    for fc in (1.0, 2.0, 3.0, 4.0, 6.0, 10.0):
        g2 = np.arange(tt[0], tt[-1], 1 / fs)
        R = track._slerp(tt, tq, g2)
        wv = (R[1:] * R[:-1].inv()).as_rotvec() * fs
        lp = LowPass(fc, fs, 3)
        wf = np.array([lp(x) for x in wv])
        sp = np.degrees(np.linalg.norm(wf, axis=1))
        left = [d for d in dips(g2[1:], sp) if any(a + 0.3 <= d[0] <= b - 0.3 for a, b in spans)]
        L.append(f'| {fc:g} | {len(left)} | {1000 * np.sqrt(2) / (2 * np.pi * fc):.0f} |')
    # figures
    L += ['', '## Figures', '']
    fr0, td0 = min(stops)
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(FULL, 2.4), gridspec_kw=dict(width_ratios=[2.2, 1]))
    w0, w1 = td0 - lag - 0.6, td0 - lag + 0.6
    m = (g >= w0) & (g <= w1)
    a1.plot(g[m] - (td0 - lag), v[m, 4], color='#E69F00', lw=1.2, label='Touch pitch encoder')
    mt = (gt >= w0) & (gt <= w1)
    a1.plot(gt[mt] - (td0 - lag), wt[mt], color=MUTED, lw=2.0, label='Stylus target')
    mr = (gr >= w0 + lag) & (gr <= w1 + lag)
    a1.plot(gr[mr] - td0, wr[mr], color='#0072B2', lw=1.2, label=f'Robot tool (shifted {1000 * lag:.0f} ms)')
    a1.axvline(0, color=INK2, lw=0.6, ls=':')
    a1.set_xlabel('Time from the dip (s)'); a1.set_ylabel('Angular speed (°/s)')
    a1.set_title('(a) One slow-speed catch')
    a1.legend(fontsize=7, loc='upper left', frameon=True, facecolor='white', edgecolor='none', framealpha=0.9)
    a2.bar(range(6), [rates[n] for n in NAMES], color=['#8a8983'] * 4 + ['#E69F00', '#8a8983'])
    a2.set_xticks(range(6)); a2.set_xticklabels([n.capitalize() for n in NAMES], rotation=45, ha='right', rotation_mode='anchor')
    a2.set_xlabel('Touch joint')
    a2.set_ylabel('Stalls per minute of motion'); a2.set_title('(b) Encoder stalls')
    L.append(f"- `figures/{os.path.basename(save(fig, HERE, 'catch_example'))}`: (a) the deepest catch with a pitch "
             "stall: Touch pitch encoder, stylus target and robot (shifted back by the lag); (b) joint-specific encoder "
             "stalls per minute of motion, per Touch joint")
    L += ['', '## Prior claims checked', '',
          '- "robot dipped to 7–20 % of its speed; pitch stopped then burst at 2–3× (16.6 → 0.0 → 46.5 °/s); stalls only at '
          '8–24 °/s, never on other joints; robot follows within ~55 ms; filtering would need 2–3 Hz and ~70 ms more lag": '
          'see the tables.']
    open(os.path.join(HERE, 'RESULTS.md'), 'w').write('\n'.join(L) + '\n')
    print('\n'.join(L))


if __name__ == '__main__':
    main()
