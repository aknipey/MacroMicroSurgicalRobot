#!/usr/bin/env python3
"""Figures for E0 from results/e0_results.yaml (run e0_offline.py first).

    source /opt/ros/jazzy/setup.bash
    cd ~/ros2_ws/src/MacroMicroSurgicalRobot
    python3 thesis/planning/analysis/E0_calibrated_kinematics/e0_figures.py

e0_budget.png       RCM error per trajectory set: measured vs model-only predictions, both days
e0_latch_offset.png SlowChecking: measured-minus-model offset through approach, latch and run
e0_camera_move.png  11-03 FastChecking_Replay1: wand-vs-robot residual steps ~13 mm at 28 s
"""
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import e0_offline as e  # noqa: E402

OUT = os.path.join(HERE, 'results')
SERIES = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100']     # categorical slots 1-4 (dataviz palette)
INK, MUTED, GRID = '#1f1f1e', '#6b6a66', '#e4e3df'
SETS = ['RCMOnly', 'WholeRunDrinkBottle', 'FastCheckingRCM', 'SlowCheckingRCM']
EXCLUDE = {'V8/CalibratedKinematics/FastCheckingRCMSimpleHolder_Replay1'}   # camera moved mid-run

plt.rcParams.update({'font.size': 9, 'axes.edgecolor': MUTED, 'axes.labelcolor': INK, 'xtick.color': MUTED,
                     'ytick.color': MUTED, 'axes.spines.top': False, 'axes.spines.right': False,
                     'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.6, 'axes.axisbelow': True})


def median(res, day_prefix, s, model, key):
    v = [r[model][key] for n, r in res.items()
         if n.startswith(day_prefix) and f'/{s}SimpleHolder_Replay' in n and n not in EXCLUDE]
    return np.median(v), np.min(v), np.max(v)


def budget(res):
    bars = [('10-26 measured (nominal on arm)', 'V7/', 'nominal', 'measured'),
            ('10-26 predicted, calibrated model', 'V7/', 'calibrated', 'predicted'),
            ('11-03 predicted, nominal model', 'V8/', 'nominal', 'predicted'),
            ('11-03 measured (calibrated on arm)', 'V8/', 'calibrated', 'measured')]
    fig, ax = plt.subplots(figsize=(8, 3.8))
    w = 0.19
    x = np.arange(len(SETS))
    for k, (label, day, model, key) in enumerate(bars):
        vals = np.array([median(res, day, s, model, key) for s in SETS])
        pos = x + (k - 1.5) * (w + 0.01)
        ax.bar(pos, vals[:, 0], w, color=SERIES[k], label=label, edgecolor='white', linewidth=1)
        ax.errorbar(pos, vals[:, 0], yerr=[vals[:, 0] - vals[:, 1], vals[:, 2] - vals[:, 0]], fmt='none',
                    ecolor=MUTED, elinewidth=0.8, capsize=0)
        for p, v in zip(pos, vals[:, 0]):
            ax.text(p, v + 0.06, f'{v:.1f}', ha='center', va='bottom', fontsize=7, color=INK)
    ax.axhline(1.0, color=MUTED, lw=0.8, ls='--')
    ax.text(len(SETS) - 0.5, 1.03, 'threshold (iii) 1 mm', ha='right', va='bottom', fontsize=7, color=MUTED)
    ax.set_xticks(x, ['RCMOnly', 'WholeRun', 'FastChecking', 'SlowChecking'])
    ax.set_ylabel('NDI RCM error, rms (mm)')
    ax.grid(axis='x', visible=False)
    ax.legend(fontsize=7, frameon=False, ncol=2, loc='upper left')
    ax.set_ylim(0, 3.6)
    ax.set_title('Kinematic model alone vs measured (median of replays, range bars)', fontsize=9, color=INK, loc='left')
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'e0_budget.png'), dpi=150)


def offsets_whole_run(res, day, name):
    """Measured-minus-calibrated-model offset at the segment pivot for the whole recording."""
    kin = yaml.safe_load(open(e.MODELS['calibrated']))['kinematics']
    run = e.load_run(os.path.join(e.PKG, 'logs', name))
    r = res[name]['calibrated']
    X = np.array(res['_handeye'][f'{day}/calibrated']['tool0_to_wand'])
    Y = np.array(r['base_to_camera'])
    q = e.joints_at(run, run['nt'] - r['delay_s'])
    ok = ~np.isnan(q).any(1)
    A, B, t = e.fk(kin, q[ok]), run['B'][ok], run['nt'][ok]
    tip, axis, _ = e.wand_points()
    P = np.array(r['pivot'])
    k_tip, k_dir = (A @ X @ np.r_[tip, 1])[:, :3], A[:, :3, :3] @ X[:3, :3] @ axis
    m_tip, m_dir = (Y @ B @ np.r_[tip, 1])[:, :3], Y[:3, :3] @ B[:, :3, :3] @ axis
    at = lambda p, d: p + np.einsum('ij,ij->i', P - p, d)[:, None] * d
    o = e.perp(at(m_tip, m_dir) - at(k_tip, k_dir), k_dir)
    o -= o[run['seg'][ok]].mean(0)
    speed = np.degrees(np.linalg.norm(np.gradient(q[ok], t, axis=0), axis=1))
    return t, np.linalg.norm(o, axis=1), speed, run['seg_meta']['t_start'], run['seg_meta']['t_end']


def smooth(y, n=15):
    return np.convolve(y, np.ones(n) / n, mode='same')


def latch_offset(res):
    runs = [('10-26', 'V7/SlowCheckingRCMSimpleHolder_Replay1', '10-26 (tool 1)'),
            ('11-03', 'V8/CalibratedKinematics/SlowCheckingRCMSimpleHolder_Replay1', '11-03 (tool 3)')]
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(8, 4.6), sharex=True, gridspec_kw={'height_ratios': [2, 1]})
    for k, (day, name, label) in enumerate(runs):
        t, o, speed, t0, t1 = offsets_whole_run(res, day, name)
        a1.plot(t, smooth(o), color=SERIES[k], lw=1.5, label=label)
        a2.plot(t, smooth(speed), color=SERIES[k], lw=1.0)
    for a in (a1, a2):
        a.axvline(t0, color=MUTED, lw=0.8, ls='--')
        a.axvspan(t0, t1, color=GRID, alpha=0.5, lw=0)
    a1.text(t0 + 0.3, a1.get_ylim()[1] * 0.92, 'RCM latched', fontsize=7, color=MUTED)
    a1.set_ylabel('|offset| from segment mean (mm)')
    a1.legend(fontsize=7, frameon=False)
    a1.set_title('SlowChecking: NDI shaft vs calibrated-model shaft at the pivot (shaded: RCM segment)',
                 fontsize=9, color=INK, loc='left')
    a2.set_ylabel('joint speed (deg/s)')
    a2.set_xlabel('recording time (s)')
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'e0_latch_offset.png'), dpi=150)


def camera_move(res):
    name = 'V8/CalibratedKinematics/FastCheckingRCMSimpleHolder_Replay1'
    kin = yaml.safe_load(open(e.MODELS['calibrated']))['kinematics']
    run = e.load_run(os.path.join(e.PKG, 'logs', name))
    r = res[name]['calibrated']
    X = np.array(res['_handeye']['11-03/calibrated']['tool0_to_wand'])
    q = e.joints_at(run, run['nt'] - r['delay_s'])
    ok = ~np.isnan(q).any(1)
    err, _ = e.tip_errors(e.fk(kin, q[ok]), X, np.array(r['base_to_camera']), run['B'][ok])
    t = run['nt'][ok]
    fig, ax = plt.subplots(figsize=(8, 2.6))
    ax.plot(t, smooth(err), color=SERIES[0], lw=1.2)
    ax.set_ylabel('tip residual (mm)')
    ax.set_xlabel('recording time (s)')
    ax.set_title('11-03 FastChecking_Replay1: robot and NDI disagree by ~13 mm from 28 s on (camera moved)',
                 fontsize=9, color=INK, loc='left')
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'e0_camera_move.png'), dpi=150)


if __name__ == '__main__':
    res = yaml.safe_load(open(os.path.join(OUT, 'e0_results.yaml')))
    budget(res)
    latch_offset(res)
    camera_move(res)
    print(f'wrote figures to {OUT}')
