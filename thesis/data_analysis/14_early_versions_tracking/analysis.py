#!/usr/bin/env python3
"""14 Early versions (V1-V3) and the emulator latency study: lag and tracking error with the folder-05 method.

Every metric is common.track (the method of folders 05 and 06): lag = best position shift, error = rms position
(mm) / angle (deg) error after removing that lag; 'raw' = the same error with no shift. Velocity cross-correlation
lag is also given because position shift is ill-posed on single step-and-hold moves (folder 06).
Motion scale per stroke comes from the bag's teleop/motion_scale (the scale was switched at run time in V3), so
CyclingScaling is split by scale. Emulator runs: logs/SIM_latency, settings read from META. Writes RESULTS.md.
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from common import bags, kin, runs, track  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402
from common.style import FULL, INK2, MODE, MUTED, save, style  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

SIM_KEYS = ('SERVO_OUTPUT', 'SERVO_LOOKAHEAD', 'TOUCH_POSE_SOURCE', 'TOUCH_PUBLISH_RATE', 'TOUCH_FEEDFORWARD_FILTER',
            'TOUCH_LOOP_DELAY')


def needs_fk(run):
    """UR_PATH holds the pendant TCP, not tool0, when the recorder had no tcp_offset and the rows are RTDE
    (V1-V3 era). Emulator runs log TF tool0 ('tf' rows) and are used as they are."""
    if run.meta.get('tcp_offset'):
        return False
    ur = run.csv('UR_PATH.csv', ['source'])
    return ur is not None and any(str(s).startswith('rtde') for s in ur['source'])


def fk_path(run, kmodel='nominal'):
    """tool0 from FK of the bag's /joint_states on the CSV clock (UR16e kinematics), as (t, P mm, Q xyzw)."""
    b = bags.load(run)
    off = bags.to_run_clock(run, b)
    js = b['/joint_states']
    t = js['tr'] - off
    K = kin.load(kmodel)
    T = kin.tool0_many(K, js['q'])
    P = T[:, :3, 3] * 1000.0
    Q = Rotation.from_matrix(T[:, :3, :3]).as_quat()
    o = np.argsort(t)
    t, P, Q = t[o], P[o], Q[o]
    u = np.r_[True, np.diff(t) > 0]
    return t[u], P[u], Q[u]


OFFSET_SOURCE = 'V3/LongBenchmark'       # first run whose META has a measured pendant-TCP offset


def pendant_offset():
    m = runs.Run(os.path.join(runs.LOGS, OFFSET_SOURCE)).meta['tcp_offset']
    return np.array(m['xyz_mm'], float), m


def paths(run):
    """CSV paths; for pendant-TCP runs the UR_PATH pose is moved back to tool0 by the measured offset."""
    (tt, tp, tq), (ut, up, uq) = track.load_paths(run)
    if needs_fk(run):
        off, _ = pendant_offset()
        up = up - Rotation.from_quat(uq).apply(off)
    return (tt, tp, tq), (ut, up, uq)


_MCACHE = {}


def metrics(run):
    if run.key not in _MCACHE:
        if needs_fk(run):
            (tt, tp, tq), (ut, up, uq) = paths(run)
            _MCACHE[run.key] = track.metrics_arrays(tt, tp, tq, ut, up, uq)
        else:
            _MCACHE[run.key] = track.cached_metrics(run)
    return _MCACHE[run.key]


def fk_check(run):
    """Median / p99 distance between the arm path used here (paths) and FK of /joint_states (nominal UR16e)."""
    (_, _, _), (ut, up, uq) = paths(run)
    ft, fp, fq = fk_path(run)
    fpi = np.column_stack([np.interp(ut, ft, fp[:, k], left=np.nan, right=np.nan) for k in range(3)])
    d = np.linalg.norm(up - fpi, axis=1)
    return float(np.nanmedian(d)), float(np.nanpercentile(d, 99))


def raw_rot_err(run):
    p = paths(run)
    (tt, tp, tq), (ut, up, uq) = p
    g = np.concatenate([np.arange(a, b, 1 / track.FS) for a, b in track.strokes(tt)])
    ra = track._slerp(ut, uq, g)
    rt = track._slerp(tt, tq, g)
    return float(np.sqrt(np.mean(np.degrees((ra.inv() * rt).magnitude()) ** 2)))


def scale_of(run, t_csv):
    """Motion scale in force at each CSV time (from teleop/motion_scale), or the META scale."""
    b = bags.load(run, ['teleop/motion_scale', '/tcp_pose_broadcaster/pose'])
    ms = b.get('teleop/motion_scale')
    off = bags.to_run_clock(run, b) if b else None
    if ms is None or off is None:
        return np.full(len(t_csv), run.scale)
    tm = ms['tr'] - off
    idx = np.searchsorted(tm, t_csv, side='right') - 1
    vals = ms['data']
    out = np.where(idx >= 0, vals[np.clip(idx, 0, len(vals) - 1)], vals[0])
    return out


def per_scale(run):
    """common.track metrics per motion scale (strokes grouped by the scale at their midpoint)."""
    (tt, tp, tq), (ut, up, uq) = paths(run)
    sp = track.strokes(tt)
    mids = np.array([(a + b) / 2 for a, b in sp])
    sc = scale_of(run, mids)
    out = {}
    for s in sorted(set(np.round(sc, 4))):
        keep = [x for x, v in zip(sp, sc) if np.isclose(v, s)]
        mt = np.zeros(len(tt), bool)
        for a, b in keep:
            mt |= (tt >= a) & (tt <= b)
        m = track.metrics_arrays(tt[mt], tp[mt], tq[mt], ut, up, uq)
        if m:
            m['n_strokes'] = len(keep)
            out[float(s)] = m
    return out


def fmt(x, d=0):
    return '–' if x is None or not np.isfinite(x) else f'{x:.{d}f}'


def main():
    style()
    L = ['# 14 Results: early versions and emulator study, lag and tracking error (folder-05 method)', '',
         'Generated by `analysis.py`. common.track metrics: lag = position shift (translation) / orientation shift '
         '(rotation); error = rms after removing that lag; raw = no shift. Search limit 1.6 s.', '',
         '**Arm path source.** Runs whose META has no `tcp_offset` and RTDE rows (V1, V2, V3 scaled runs) logged the '
         'pendant TCP in UR_PATH, not tool0. Here those poses are moved back to tool0 by the pendant offset measured '
         f'in `{OFFSET_SOURCE}` META ({pendant_offset()[0].tolist()} mm, |offset| {np.linalg.norm(pendant_offset()[0]):.1f} mm). '
         'Check: distance between that path and FK of the bag\'s `/joint_states` (nominal UR16e), median / p99 mm, '
         'per run below (bag overlap only). Emulator runs log TF tool0 and are used as they are.', '']
    chk = []
    for r in runs.find_runs('V1/V1_*') + runs.find_runs('V2/*') + runs.find_runs('V3/*'):
        if needs_fk(r):
            try:
                a, b = fk_check(r)
                chk.append((r.key, a, b))
            except Exception as e:  # noqa: BLE001
                chk.append((r.key, np.nan, np.nan))
    L.append('| Run | FK check median / p99 (mm) |')
    L.append('|---|---|')
    L += [f'| {k} | {a:.2f} / {b:.2f} |' for k, a, b in chk]
    L.append('')
    # 1. V1 / V2 single moves and general motion
    L += ['## 1. V1 (position straight from Servo) vs V2 (trajectory output)', '',
          '| Run | mode | lag pos / vel-xcorr / rot (ms) | error lag-removed: mm / deg | error raw: mm / deg |',
          '|---|---|---|---|---|']
    sec1 = {}
    for r in runs.find_runs('V1/V1_*') + runs.find_runs('V2/*'):
        m = metrics(r)
        if not m:
            continue
        rr = raw_rot_err(r)
        sec1[r.key] = (m, rr)
        lim = lambda x: fmt(x) + ('≥' if x > 1590 else '')
        L.append(f"| {r.key} | {r.mode} | {lim(m['lag_ms'])} / {lim(m['lag_vel_ms'])} / {lim(m['lag_rot_ms'])} | "
                 f"{m['err_mm']:.1f} / {m['err_rot_deg']:.1f} | {m['err_raw_mm']:.1f} / {rr:.1f} |")
    for grp, pat in (('V1 90° rotations', 'V1/V1_90'), ('V2 90° rotations', 'V2/V2_90')):
        rot = [v[0]['err_rot_deg'] for k, v in sec1.items() if k.startswith(pat)]
        raw = [v[1] for k, v in sec1.items() if k.startswith(pat)]
        if rot:
            L.append(f"\n{grp}: rotation error lag-removed {min(rot):.1f}–{max(rot):.1f}°, raw {min(raw):.1f}–{max(raw):.1f}°")
    # 2. V3 by motion scale
    L += ['', '## 2. V3 trajectory output by motion scale (strokes grouped by teleop/motion_scale)', '',
          '| Run | scale | strokes | lag pos / rot (ms) | error lag-removed (mm) | error raw (mm) |', '|---|---|---|---|---|---|']
    by_scale = {}
    for r in runs.find_runs('V3/*'):
        if r.name == 'NoMotion':
            continue
        try:
            ps = per_scale(r)
        except Exception as e:  # noqa: BLE001
            L.append(f'| {r.key} | error: {e} | | | | |')
            continue
        for s, m in ps.items():
            by_scale.setdefault(s, []).append(m)
            L.append(f"| {r.key} | 1:{1 / s:.0f} | {m['n_strokes']} | {m['lag_ms']:.0f} / {m['lag_rot_ms']:.0f} | "
                     f"{m['err_mm']:.2f} | {m['err_raw_mm']:.2f} |")
    L += ['', '| Scale | runs | lag translation (ms) range | lag rotation (ms) range | raw error (mm) range |', '|---|---|---|---|---|']
    for s in sorted(by_scale, reverse=True):
        ms = by_scale[s]
        g = lambda k: (min(x[k] for x in ms), max(x[k] for x in ms))
        a, b, c = g('lag_ms'), g('lag_rot_ms'), g('err_raw_mm')
        L.append(f"| 1:{1 / s:.0f} | {len(ms)} | {a[0]:.0f}–{a[1]:.0f} | {b[0]:.0f}–{b[1]:.0f} | {c[0]:.2f}–{c[1]:.2f} |")
    # 3. emulator latency study
    L += ['', '## 3. Emulator study (logs/SIM_latency; mock UR16e + ur_emulator, replay of V3/CyclingScaling)', '',
          '| Run | ' + ' | '.join(SIM_KEYS) + ' | lag pos / rot (ms) | error lag-removed (mm) |',
          '|---|' + '---|' * len(SIM_KEYS) + '---|---|']
    sim = {}
    for r in runs.find_runs('SIM_latency/*'):
        m = metrics(r)
        if not m:
            continue
        sim[r.name] = (r, m)
        L.append(f"| {r.name} | " + ' | '.join(str(r.settings.get(k)) for k in SIM_KEYS) +
                 f" | {m['lag_ms']:.0f} / {m['lag_rot_ms']:.0f} | {m['err_mm']:.2f} |")
    # paired: only pose source differs
    L += ['', '### Pairs differing in one setting', '']
    for a in sim:
        for b in sim:
            ra, rb = sim[a][0], sim[b][0]
            d = [k for k in set(ra.settings) | set(rb.settings)
                 if k not in ('REPLAY_LEAD_IN',) and ra.settings.get(k) != rb.settings.get(k)]
            if a < b and len(d) == 1:
                ma, mb = sim[a][1], sim[b][1]
                L.append(f"- {a} → {b}: {d[0]} {ra.settings.get(d[0])} → {rb.settings.get(d[0])}: lag "
                         f"{ma['lag_ms']:.0f} → {mb['lag_ms']:.0f} ms, error {ma['err_mm']:.2f} → {mb['err_mm']:.2f} mm")
    # figures
    L += ['', '## Figures', '']
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(FULL * 0.86, 2.5), gridspec_kw=dict(width_ratios=[1, 1.1]))
    tr = MODE['trajectory']['color']
    for i, sc in enumerate(sorted(by_scale, reverse=True)):
        ms_ = by_scale[sc]
        x = np.full(len(ms_), i) + np.random.default_rng(i).uniform(-0.12, 0.12, len(ms_))
        a1.plot(x, [m['lag_ms'] for m in ms_], 'D', ms=4, color=tr, label='Translation' if i == 0 else None)
        a1.plot(x, [m['lag_rot_ms'] for m in ms_], 'D', ms=4, mfc='white', color=tr, label='Rotation' if i == 0 else None)
    a1.set_xticks(range(len(by_scale))); a1.set_xticklabels([f'1:{1 / s_:.0f}' for s_ in sorted(by_scale, reverse=True)])
    a1.set_ylim(0, 260); a1.set_ylabel('Lag (ms)'); a1.set_xlabel('Motion scale')
    a1.set_title('(a) Real arm, V3'); a1.legend(loc='lower left')
    order = [k for k in ('A_baseline', 'B_look20', 'C_ff10', 'D_250hz', 'E_fk', 'B_look13', 'H_vel_final') if k in sim]
    labels = {'A_baseline': 'Lookahead 0.05 s (V3)', 'B_look20': 'Lookahead 0.02 s',
              'C_ff10': '+ feedforward filter 0.01 s', 'D_250hz': '+ teleop at 250 Hz',
              'E_fk': '+ pose from /joint_states', 'B_look13': 'Lookahead 0.013 s',
              'H_vel_final': 'Velocity output'}
    vals = [sim[k][1]['lag_ms'] for k in order]
    cols = [MODE['velocity']['color'] if sim[k][0].settings.get('SERVO_OUTPUT') == 'velocity' else tr for k in order]
    y = np.arange(len(order))[::-1]
    a2.barh(y, vals, color=cols, alpha=0.8, height=0.6)
    for yy, v_ in zip(y, vals):
        a2.text(v_ + 3, yy, f'{v_:.0f}', va='center', fontsize=8)
    a2.set_yticks(y); a2.set_yticklabels([labels[k] for k in order])
    a2.set_xlim(0, 200); a2.set_xlabel('Lag (ms)'); a2.grid(axis='y', visible=False)
    a2.set_title('(b) Emulator replay')
    L.append(f"- `figures/{os.path.basename(save(fig, HERE, 'trajectory_route_lag'))}`: (a) trajectory-output lag at each "
             "motion scale on the real arm (V3); (b) emulator lag for the settings tried on the trajectory route, against "
             "velocity output")
    L += ['', '## Prior claims checked', '',
          '- "V2 rms angle error 27–36° (V1) → 6–24°", "lag ~140 ms at every scale, < 1 mm gap at small scales", '
          '"lookahead 0.013 s floors lag at ~114 ms", "joint_states pose made trajectory output worse, 114 → 130 ms": '
          'see the tables.']
    open(os.path.join(HERE, 'RESULTS.md'), 'w').write('\n'.join(L) + '\n')
    print('\n'.join(L))


if __name__ == '__main__':
    main()
