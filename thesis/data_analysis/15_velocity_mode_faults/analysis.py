#!/usr/bin/env python3
"""15 Velocity output on the real arm: vibration, joint faults and what the 500 Hz chain + limiter changed.

Same recorded inputs (V3/LongBenchmark, V3/ShortBenchmark) through three stacks, all real arm, 1:1:
  trajectory (the V3 originals), velocity without limiter (V4 *_failed, 100 Hz, 8 Hz command filter),
  velocity with the 500 Hz chain and joint_velocity_limiter (V5 replays). Plus V5/WristFault (a hand-driven
  velocity run, 250 Hz, no limiter, that faulted inside its recording).
Fault = the first /joint_states sample from which the published state is frozen (position, velocity and current
repeated exactly for >= 0.5 s): at a joint fault the driver keeps republishing the last state, so the frozen
values are the arm's state at the fault. Checks reported alongside: speed scaling dropping to 0 % and the
folder-06 rule (arm stops following).
Chatter = 45-250 Hz content of the motor current (/joint_states effort, A; 4th-order zero-phase high-pass at 45 Hz
on the 500 Hz grid), rms per 0.1 s window. Vibration = folder 01's method (tool-space velocity J(q) qd, 3 Hz
motion removal, Welch 1 s) on the moving part of the input before any fault. Joint speed limits are read from
ur_description (UR16e joint_limits.yaml); the limiter's caps from META. Writes RESULTS.md.
"""
import importlib.util
import os
import sys

import numpy as np
import yaml
from scipy.signal import butter, sosfiltfilt

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
from common import bags, kin, runs, track  # noqa: E402
from common.style import INK, FULL, INK2, MODE, save, style  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402


def load_module(folder, name):
    spec = importlib.util.spec_from_file_location(name, os.path.join(ROOT, folder, 'analysis.py'))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


F01 = load_module('01_vibration_threshold_i', 'f01')
F06 = load_module('06_lag_by_version', 'f06')
FS = 500.0
SOS_HP = butter(4, 45.0, 'highpass', fs=FS, output='sos')
LIMITS_FILE = '/opt/ros/jazzy/share/ur_description/config/ur16e/joint_limits.yaml'
SHORT = ['base', 'shoulder', 'elbow', 'wrist 1', 'wrist 2', 'wrist 3']

STACKS = [('V3/LongBenchmark', 'trajectory'), ('V3/ShortBenchmark', 'trajectory'),
          ('V4/LongBenchmark_failed', 'velocity, no limiter'), ('V4/ShortBenchmark_failed', 'velocity, no limiter'),
          ('V5/LongBenchmark', 'velocity, 500 Hz + limiter'), ('V5/ShortBenchmark', 'velocity, 500 Hz + limiter'),
          ('V5/WristFault', 'velocity, 250 Hz, no limiter (by hand)')]


class _Loader(yaml.SafeLoader):
    pass


_Loader.add_constructor('!degrees', lambda l, n: np.radians(float(l.construct_scalar(n))))


def speed_limits():
    d = yaml.load(open(LIMITS_FILE), Loader=_Loader)['joint_limits']
    return np.array([d[j]['max_velocity'] for j in runs.JOINTS])


def run_data(run):
    b = bags.load(run)
    off = bags.to_run_clock(run, b)
    js = b['/joint_states']
    t = js['tr'] - off
    g = np.arange(t[0], t[-1], 1 / FS)
    qd = np.column_stack([np.interp(g, t, js['qd'][:, k]) for k in range(6)])
    cur = np.column_stack([np.interp(g, t, js['current'][:, k]) for k in range(6)])
    cmds = {}
    for topic in ('/joint_velocity_limiter/commands', '/forward_velocity_controller/commands'):
        if topic in b and len(b[topic]['data']):
            c = b[topic]
            cmds[topic] = (c['tr'] - off, c['data'])
    ss = b.get('/speed_scaling_state_broadcaster/speed_scaling')
    ss = (ss['tr'] - off, ss['data']) if ss is not None else None
    return g, qd, cur, cmds, ss


def freeze_time(run):
    b = bags.load(run)
    off = bags.to_run_clock(run, b)
    js = b['/joint_states']
    t = js['tr'] - off
    X = np.hstack([js['q'], js['qd'], js['current']])
    same = np.r_[False, np.all(X[1:] == X[:-1], axis=1)]
    i = len(t) - 1
    while i > 0 and same[i]:
        i -= 1                       # i = last changing sample; frozen from i on
    if t[-1] - t[i] >= 0.5:
        return float(t[i]), X[i]
    return None, None


def engaged_mask(run, g, end):
    tg = run.csv('TOUCH_PATH.csv')
    m = np.zeros(len(g), bool)
    for a, b in track.strokes(tg['t']):
        m |= (g >= a) & (g <= min(b, end))
    return m


def chatter_windows(g, cur, mask, win=0.1):
    hp = sosfiltfilt(SOS_HP, cur, axis=0)
    n = int(win * FS)
    idx = [i for i in range(0, len(g) - n, n) if mask[i:i + n].all()]
    rms = np.array([np.sqrt(np.mean(hp[i:i + n] ** 2, axis=0)) for i in idx])
    return np.array([g[i + n] for i in idx]), rms, hp


def main():
    style()
    lim = speed_limits()
    rows = {}
    for key, label in STACKS:
        r = runs.Run(os.path.join(runs.LOGS, key))
        g, qd, cur, cmds, ss = run_data(r)
        fault06 = F06.fault(r)
        ts_, v_ = ss
        z = np.where(v_ < 1.0)[0]
        fault_ss = float(ts_[z[0]]) if len(z) else None
        fault, frozen = freeze_time(r)
        end = fault if fault else g[-1]
        mask = engaged_mask(r, g, end)
        tw, ch, hp = chatter_windows(g, cur, mask)
        o = dict(run=r, label=label, fault=fault, fault06=fault06, fault_ss=fault_ss, frozen=frozen, end_input=float(r.csv('TOUCH_PATH.csv')['t'][-1]),
                 peak=np.abs(qd[mask]).max(0), chatter_med=np.median(ch, 0), chatter=ch, tw=tw,
                 g=g, qd=qd, hp=hp)
        if fault:
            o['qd_at'] = np.abs(qd[(g > fault - 0.02) & (g <= fault)]).max(0)     # peak in the last 20 ms
            o['cur_at'] = frozen[12:18]
            # chatter in the last full 0.1 s window ending at or before the stop, ranked within the run
            k = int(np.searchsorted(tw, fault, side='right')) - 1
            k = max(0, min(k, len(tw) - 1))
            o['k_end'] = float(tw[k])
            o['ch_pre'] = ch[k]
            o['ch_rank'] = np.array([100.0 * np.mean(ch[:, j] >= ch[k, j]) for j in range(6)])
        for topic, (tc, d) in cmds.items():
            sel = (tc >= g[0]) & (tc <= end)
            if sel.any():
                o.setdefault('cmd_peak', {})[topic] = np.abs(d[sel]).max(0)
        if ss is not None:
            ts, v = ss
            sel = np.interp(ts, g, mask.astype(float)) > 0.5
            sel &= ts <= end
            if sel.any():
                o['ss_min'], o['ss_p1'] = float(v[sel].min()), float(np.percentile(v[sel], 1))
                o['ss_92'] = float(100 * np.mean(v[sel] >= 92))
        rows[key] = o

    L = ['# 15 Results: velocity output on the real arm: vibration, faults, 500 Hz chain + limiter', '',
         'Generated by `analysis.py`. Same recorded inputs through trajectory (V3), velocity without limiter (V4) and '
         'velocity with the 500 Hz chain + limiter (V5); plus V5/WristFault (by hand). Joint speed limits from '
         f'`{LIMITS_FILE}`: ' + ', '.join(f'{n} {v:.2f}' for n, v in zip(SHORT, lim)) + ' rad/s.', '',
         '## 1. Faults (arm stopped following before the input ended)', '',
         '| Run | stack | fault at (s) / input end (s) [speed scaling 0 % at; folder-06 rule] | peak joint speed before fault (rad/s), fastest joint | '
         'peak joint speed in the last 20 ms before the fault (current in the frozen state) | speed scaling while engaged, before fault: min / p1 / share ≥ 92 % |', '|---|---|---|---|---|---|']
    for k, o in rows.items():
        j = int(np.argmax(o['peak'] / lim))
        ssm = f"{o['ss_min']:.0f} % / {o['ss_p1']:.0f} % / {o['ss_92']:.1f} %" if 'ss_min' in o else '–'
        if o['fault']:
            jf = int(np.argmax(o['qd_at'] / lim))
            atf = f"{SHORT[jf]} {o['qd_at'][jf]:.2f} ({100 * o['qd_at'][jf] / lim[jf]:.0f} %), current {o['cur_at'][jf]:.1f} A"
        else:
            atf = '–'
        f6 = (('%.2f' % o['fault_ss']) if o['fault_ss'] else '–') + '; ' + (('%.1f' % o['fault06']) if o['fault06'] else '–')
        L.append(f"| {k} | {o['label']} | {('%.2f' % o['fault']) if o['fault'] else '–'} / {o['end_input']:.1f} [{f6}] | "
                 f"{SHORT[j]} {o['peak'][j]:.2f} ({100 * o['peak'][j] / lim[j]:.0f} % of limit) | {atf} | {ssm} |")

    L += ['', '## 2. At each fault: joint speed and current chatter in the last 0.1 s window before the stop', '',
          'Rank = share of the run\'s engaged 0.1 s windows (before the fault) with chatter at least as high; '
          '"top 1 window" = the run maximum. Two joints per fault: the highest-ranked.', '', '| Run | joint | peak speed, last 20 ms (rad/s, % of limit) | current at fault (A) | chatter in last 0.1 s (A) | run median (A) | rank |',
          '|---|---|---|---|---|---|---|']
    for k, o in rows.items():
        if not o['fault']:
            continue
        nwin = len(o['chatter'])
        for j in np.argsort(o['ch_rank'])[:2]:
            L.append(f"| {k} | {SHORT[j]} | {o['qd_at'][j]:.2f} ({100 * o['qd_at'][j] / lim[j]:.0f} %) | "
                     f"{o['cur_at'][j]:.2f} | {o['ch_pre'][j]:.2f} | {o['chatter_med'][j]:.2f} | top {o['ch_rank'][j]:.1f} % of {nwin} |")

    L += ['', '## 3. Current chatter while engaged (45–250 Hz, A rms per 0.1 s window, median over windows; before any fault)', '',
          '| Run | stack | ' + ' | '.join(SHORT) + ' |', '|---|---|' + '---|' * 6]
    for k, o in rows.items():
        L.append(f"| {k} | {o['label']} | " + ' | '.join(f'{x:.3f}' for x in o['chatter_med']) + ' |')
    by = {}
    for o in rows.values():
        by.setdefault(o['label'], []).append(o['chatter_med'][3:])
    L.append('')
    for lab, v in by.items():
        v = np.concatenate(v)
        L.append(f'- {lab}: wrist chatter medians {v.min():.3f}–{v.max():.3f} A')

    L += ['', '## 4. Commanded vs measured joint speed (max over the engaged part before any fault, rad/s)', '',
          '| Run | Servo output (max joint, % of limit) | limiter output | measured | TOUCH_MAX_ANGULAR_SPEED (tool cap, rad/s) |',
          '|---|---|---|---|---|']
    for k, o in rows.items():
        cp = o.get('cmd_peak', {})
        servo_topic = '/joint_velocity_limiter/commands' if '/joint_velocity_limiter/commands' in cp else \
            '/forward_velocity_controller/commands'
        def cell(v):
            if v is None:
                return '–'
            j = int(np.argmax(v / lim))
            return f'{SHORT[j]} {v[j]:.2f} ({100 * v[j] / lim[j]:.0f} %)'
        limo = cp.get('/forward_velocity_controller/commands') if servo_topic != '/forward_velocity_controller/commands' else None
        L.append(f"| {k} | {cell(cp.get(servo_topic))} | {cell(limo)} | {cell(o['peak'])} | "
                 f"{o['run'].settings.get('TOUCH_MAX_ANGULAR_SPEED')} |")
    caps = rows['V5/LongBenchmark']['run'].settings
    L.append(f"\nLimiter settings (META, V5): speed fraction {caps.get('VELOCITY_LIMITER_SPEED_FRACTION')} → wrist cap "
             f"{caps.get('VELOCITY_LIMITER_SPEED_FRACTION', np.nan) * lim[3]:.2f} rad/s; accelerations "
             f"{caps.get('VELOCITY_LIMITER_MAX_ACCELERATION')} rad/s².")

    # 5. vibration: tool-space band power, same input, before any fault
    L += ['', '## 5. Tool vibration, same input (folder-01 method, moving part before any fault)', '',
          '| Input | stack | moving (s) | 15–25 Hz linear / angular power ratio vs trajectory | 30–60 Hz linear / angular |',
          '|---|---|---|---|---|']
    for src in ('LongBenchmark', 'ShortBenchmark'):
        trio = [rows[k] for k in rows if k.endswith('/' + src) or k.endswith('/' + src + '_failed')]
        rr = [o['run'] for o in trio]
        thr = F01.speed_threshold(rr)[0]
        pw = {}
        for o in trio:
            r = o['run']
            K = kin.load(r.kinematics[0] or 'nominal')
            gg, q, qd, cmd, sty = F01.signals(r, K)
            end = o['fault'] if o['fault'] else gg[-1]
            wins = [(a, min(b, end - 0.5)) for a, b in F01.moving_windows(r, thr)]
            wins = [(a, b) for a, b in wins if b - a >= 1.0 and a >= gg[0] and b <= gg[-1]]
            tool = F01.to_tool(K, q, qd)
            res = {}
            for nm, X in (('lin', tool[:, :3]), ('ang', tool[:, 3:])):
                s = F01.segments(gg, X, wins)
                if s:
                    f, P = F01.psd_of(s)
                    res[nm] = {bn: F01.band(f, P, lo, hi).mean() for bn, (lo, hi) in (('a', (15, 25)), ('b', (30, 60)))}
            pw[r.key] = (o['label'], sum(b - a for a, b in wins), res)
        base = next(v for k, v in pw.items() if k.startswith('V3'))
        for k, (lab, mv, res) in pw.items():
            if not res or not base[2]:
                continue
            rat = lambda nm, bn: res[nm][bn] / base[2][nm][bn]
            L.append(f"| {src} | {lab} ({k}) | {mv:.1f} | {rat('lin', 'a'):.1f}× / {rat('ang', 'a'):.1f}× | "
                     f"{rat('lin', 'b'):.1f}× / {rat('ang', 'b'):.1f}× |")
    # 6. clean same-input comparison: unfiltered velocity vs trajectory, V9b 1:5 (folder 01 data and windows)
    L += ['', '## 6. Unfiltered velocity vs trajectory output, same input, V9b/NoTool 1:5 (folder-01 method and windows)', '',
          '| Input | signal | 15–25 Hz power ratio velocity-unfiltered / trajectory | 30–60 Hz | 8–12 Hz |',
          '|---|---|---|---|---|']
    K = kin.load('calibrated')
    rs = runs.find_runs('V9b/NoTool/VelocityUnfiltered/Scaled5_*_Replay*') + runs.find_runs('V9b/NoTool/TrajectoryMode/Scaled5_*_Replay*')
    for tr in sorted({r.trajectory for r in rs}):
        sub = [r for r in rs if r.trajectory == tr]
        thr = F01.speed_threshold([r for r in sub if r.mode == 'velocity_unfiltered'])[0]
        sig = {r.key: F01.signals(r, K) for r in sub}
        lo_ = max(v[0][0] for v in sig.values()) + 0.5
        hi_ = min(v[0][-1] for v in sig.values()) - 0.5
        pooled = {}
        for r in sub:
            gg, q, qd, cmd, sty = sig[r.key]
            wins = [(max(a, lo_), min(b, hi_)) for a, b in F01.moving_windows(r, thr)]
            wins = [(a, b) for a, b in wins if b - a >= 1.0]
            tool = F01.to_tool(K, q, qd)
            for nm, X in (('tool linear', tool[:, :3]), ('tool angular', tool[:, 3:])):
                sg = F01.segments(gg, X, wins)
                f, P = F01.psd_of(sg)
                pooled.setdefault((r.mode, nm), []).append(P)
        for nm in ('tool linear', 'tool angular'):
            Pu = np.concatenate(pooled[('velocity_unfiltered', nm)]); Pt = np.concatenate(pooled[('trajectory', nm)])
            cells = [F01.band(f, Pu, lo, hi).mean() / F01.band(f, Pt, lo, hi).mean() for lo, hi in ((15, 25), (30, 60), (8, 12))]
            L.append(f'| {tr} | {nm} | ' + ' | '.join(f'{c:.0f}× (amplitude {np.sqrt(c):.1f}×)' for c in cells) + ' |')
    L += ['', 'Power ratios; amplitude ratio = square root.', '', '## Figures', '']
    col = {'trajectory': MODE['trajectory']['color'], 'velocity, no limiter': MODE['velocity_unfiltered']['color'],
           'velocity, 500 Hz + limiter': MODE['velocity_optimised']['color'],
           'velocity, 250 Hz, no limiter (by hand)': MODE['velocity_unfiltered']['color']}
    fig, ax = plt.subplots(figsize=(FULL * 0.75, 2.7))
    keys = list(rows)
    data = [rows[k]['chatter'][:, 3:].max(1) for k in keys]
    bp = ax.boxplot(data, vert=True, widths=0.55, showfliers=False, patch_artist=True, whis=(5, 95),
                    medianprops=dict(color=INK, lw=1.2))
    for patch, k in zip(bp['boxes'], keys):
        patch.set_facecolor(col[rows[k]['label']]); patch.set_alpha(0.55)
    for i, (k, d) in enumerate(zip(keys, data), 1):
        ax.plot(np.full(len(d), i) + np.random.default_rng(i).uniform(-0.18, 0.18, len(d)), d, '.', ms=1.5,
                color=col[rows[k]['label']], alpha=0.5)
    ax.set_yscale('log'); ax.set_ylim(5e-3, 5); ax.set_ylabel('Wrist current chatter, 45–250 Hz (A rms)')
    ax.set_xticks(range(1, len(keys) + 1))
    tick = lambda k: ('V5 by hand\n(faulted)' if 'WristFault' in k else
                      k.replace('Benchmark', '').replace('_failed', '\n(faulted)').replace('/', ' ', 1))
    ax.set_xticklabels([tick(k) for k in keys])
    ax.set_xlabel('Replay (version and input)')
    for lab, c in list(col.items())[:3]:
        ax.plot([], [], 's', color=c, alpha=0.6, label=lab[0].upper() + lab[1:])
    ax.legend(loc='lower center', bbox_to_anchor=(0.5, 1.0), ncol=3, handletextpad=0.2, columnspacing=1.0)
    L.append(f"- `figures/{os.path.basename(save(fig, HERE, 'wrist_chatter_by_stack'))}`: wrist current chatter per "
             "0.1 s engaged window (worst wrist), same inputs through trajectory (V3), velocity without limiter (V4, "
             "faulted) and velocity with the 500 Hz chain + limiter (V5); box = quartiles, whiskers = 5th–95th percentile")
    o = rows['V4/LongBenchmark_failed']
    j = 3
    w = (o['g'] >= o['fault'] - 1.5) & (o['g'] <= o['fault'] + 0.3)
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(FULL * 0.65, 2.8), sharex=True)
    a1.plot(o['g'][w] - o['fault'], np.abs(o['qd'][w, j]), color='#0072B2', lw=1.0)
    a1.axhline(lim[j], color=INK2, lw=0.7, ls='--'); a1.text(-1.48, lim[j] * 1.03, 'Speed limit', fontsize=8, color=INK2, va='bottom')
    a1.set_ylabel('Wrist 1 speed\n(rad/s)')
    a2.plot(o['g'][w] - o['fault'], o['hp'][w, j], color='#D55E00', lw=0.6)
    a2.set_ylabel('Wrist 1 current,\n45–250 Hz (A)'); a2.set_xlabel('Time from the fault (s)')
    for a in (a1, a2):
        a.axvline(0, color='k', lw=0.8, ls=':')
    L.append(f"- `figures/{os.path.basename(save(fig, HERE, 'fault_timeline'))}`: wrist 1 speed and its 45–250 Hz "
             "current before the fault (dotted = start of the frozen /joint_states)")
    L += ['',
          '## Prior claims checked', '',
          '- "three benchmark replays faulted 4–5 s in"; "wrist at 3.12 and 3.17 rad/s vs 3.14"; "a later fault at 1.31 rad/s, '
          '41 % of the limit"; "faulting joint had its highest current chatter (top 0.1–1.5 %)"; "chatter 0.34–0.98 vs '
          '0.01–0.05 N m" (old values were labelled N m; /joint_states effort is motor current, A); "speed scaling above 92 %"; '
          '"tool vibrated ~5× more at 15–25 Hz, ~7× at 30–60 Hz"; "the same replays did not fault with the limiter": see the tables.']
    open(os.path.join(HERE, 'RESULTS.md'), 'w').write('\n'.join(L) + '\n')
    print('\n'.join(L))


if __name__ == '__main__':
    main()
