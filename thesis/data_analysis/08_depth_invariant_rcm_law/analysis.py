#!/usr/bin/env python3
"""08 The depth-invariant sideways law, evaluated by driving the shipped code (macro_micro_robot.ur16e_common.RcmTool:
lever, pivot_slope, target_shaft) - not a re-derivation.

Laws compared (parameters READ from run META files, except the fade, which no run used live):
  - fixed tip scale: what V6 ran (V6 META has no law settings) = pivot_slope with tool0_gain <= 0, no fade;
  - smootherstep fade: tool0_gain off, TOUCH_RCM_PIVOT_FADE_DEPTH = 30 mm (illustrative: an alternative tried offline);
  - the law: TOUCH_RCM_TOOL0_GAIN and TOUCH_RCM_TOOL0_GAIN_DEPTH from V7/V9 META.
Tool geometry from the V7 run's own TOUCH_RCM_SHAFT_* settings. For each insertion depth d the stylus asks for a
small sideways tip move s; the code returns the new shaft; tool0 is placed so the shaft passes the RCM point at the
returned depth. Reported: flange (tool0) motion per mm of s, pivot per mm of s, and their continuity through d = 0.
"""
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from common import runs  # noqa: E402
from common.style import FULL, HALF, INK2, MUTED, save, style  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, runs.PKG)
from macro_micro_robot.ur16e_common import RcmTool  # noqa: E402

S = 0.001              # 1 mm sideways stylus request
FADE_ILLUSTRATIVE = 0.030


def flange_response(tool, slope_fn, depths, s=S):
    axis0 = np.array([0.0, 0.0, -1.0])        # shaft pointing down into the port
    rcm = np.zeros(3)
    R0 = RcmTool.align(tool.direction, axis0)
    perp = np.array([1.0, 0.0, 0.0])
    gain, pivot = [], []
    for d in depths:
        p0 = rcm + d * axis0 - R0.apply(tool.tip_offset)
        tip_target = rcm + d * axis0 + s * perp
        axis, dep = tool.target_shaft(tip_target, rcm, axis0, 1.0, slope_fn, min_depth=-0.05)
        R = RcmTool.align(axis0, axis) * R0
        p = rcm + dep * axis - R.apply(tool.tip_offset)
        gain.append(np.linalg.norm(p - p0) / s)
        pivot.append(np.degrees(np.arccos(np.clip(axis @ axis0, -1, 1))) / (s * 1000))
    return np.array(gain), np.array(pivot)


FS_SCRIPT = 500.0


def flange_path(tool, slope_fn, tip_targets):
    """tool0 positions (m) for a sequence of tip targets, through target_shaft (same placement as flange_response)."""
    axis0 = np.array([0.0, 0.0, -1.0])
    rcm = np.zeros(3)
    R0 = RcmTool.align(tool.direction, axis0)
    out = []
    for tip_target in tip_targets:
        axis, dep = tool.target_shaft(tip_target, rcm, axis0, 1.0, slope_fn, min_depth=-0.05)
        R = RcmTool.align(axis0, axis) * R0
        out.append(rcm + dep * axis - R.apply(tool.tip_offset))
    return np.array(out)


def kinematics_of(P, fs=FS_SCRIPT):
    v = np.gradient(P, 1 / fs, axis=0)
    a = np.gradient(v, 1 / fs, axis=0)
    return np.linalg.norm(v, axis=1), np.linalg.norm(a, axis=1)


def scripted(tool, laws):
    """Three scripted stylus inputs (tip targets) through each law: peak flange speed and acceleration."""
    axis0 = np.array([0.0, 0.0, -1.0]); perp = np.array([1.0, 0.0, 0.0]); perp2 = np.array([0.0, 1.0, 0.0])
    t1 = np.arange(0, 3.5, 1 / FS_SCRIPT)
    d1 = -0.010 + 0.020 * t1                                          # -10 -> +60 mm at 20 mm/s
    A = [d * axis0 + 0.005 * perp for d in d1]                       # insert with a 5 mm side offset
    A0 = [d * axis0 for d in d1]
    t2 = np.arange(0, 2.0, 1 / FS_SCRIPT)
    B = [0.010 * axis0 + 0.005 * (np.cos(2 * np.pi * x) * perp + np.sin(2 * np.pi * x) * perp2) for x in t2]
    t3 = np.arange(0, 1.0, 1 / FS_SCRIPT)
    C = [0.020 * axis0 + (-0.012 + 0.024 * x) * perp for x in t3]     # 24 mm/s sideways sweep at 20 mm depth
    rows = {}
    for k, f in laws.items():
        PA, PA0 = flange_path(tool, f, A), flange_path(tool, f, A0)
        vA, aA = kinematics_of(PA)
        off = np.linalg.norm(PA - PA0, axis=1) * 1000
        deep = (d1 >= 0.030) & (d1 <= 0.130)
        vB, aB = kinematics_of(flange_path(tool, f, B))
        vC, aC = kinematics_of(flange_path(tool, f, C))
        edge = slice(5, -5)                                           # drop the finite-difference ends
        rows[k] = dict(stepA=1000 * np.linalg.norm(np.diff(PA, axis=0), axis=1).max(), aA=aA[edge].max(), vA=vA[edge].max(), off=(off[deep].min(), off[deep].max()) if deep.any() else (np.nan, np.nan),
                       vB=vB[edge].max(), aB=aB[edge].max(), vC=np.median(vC[edge]), aC=aC[edge].max(),
                       series=dict(A=(d1[edge] * 1000, vA[edge] * 1000), B=(t2[edge], vB[edge] * 1000),
                                   C=(t3[edge], vC[edge] * 1000)))
    return rows


def main():
    style()
    v7 = runs.Run(os.path.join(runs.LOGS, 'V7', 'RCMOnlySimpleHolder_Replay1'))
    v6 = runs.Run(os.path.join(runs.LOGS, 'V6', 'BasicInRCM'))
    s7 = v7.settings
    tool = RcmTool(s7['TOUCH_RCM_SHAFT_START'], s7['TOUCH_RCM_SHAFT_DIRECTION'], s7['TOUCH_RCM_SHAFT_LENGTH'])
    g, gd = s7['TOUCH_RCM_TOOL0_GAIN'], s7['TOUCH_RCM_TOOL0_GAIN_DEPTH']
    laws = {
        'fixed tip scale (V6)': lambda a: tool.pivot_slope(a, 0.0, gd, 0.0),
        f'smootherstep fade {FADE_ILLUSTRATIVE * 1e3:.0f} mm (offline alternative)':
            lambda a: tool.pivot_slope(a, 0.0, gd, FADE_ILLUSTRATIVE),
        f'depth-invariant law (gain {g}, to {gd * 1e3:.0f} mm; V7–V9b)': lambda a: tool.pivot_slope(a, g, gd, 0.0),
    }
    cols = ['#8A8983', '#E69F00', '#0072B2']
    depths = np.linspace(-0.01, 0.15, 321)
    res = {k: flange_response(tool, f, depths) for k, f in laws.items()}
    v6_law = 'TOUCH_RCM_TOOL0_GAIN' not in v6.settings
    L = ['# 08 Results: the depth-invariant sideways law', '',
         'Generated by `analysis.py`, which drives `RcmTool.target_shaft` / `pivot_slope` from the repository code. Tool '
         f"geometry from `{v7.key}` META (shaft start {s7['TOUCH_RCM_SHAFT_START']} m, length {s7['TOUCH_RCM_SHAFT_LENGTH']} m; "
         f"tip {np.linalg.norm(tool.tip_offset) * 1e3:.1f} mm from tool0). Law settings from META: TOOL0_GAIN = {g}, "
         f"GAIN_DEPTH = {gd} m (V7 and V9 identical). V6 META has no law settings: {v6_law} → V6 ran the fixed tip scale.",
         '', '**Note for the report:** the gain in the runs is TOUCH_RCM_TOOL0_GAIN = %s, not the k = 0.5 written in the '
         'planning notes and the Chapter 4 draft.' % g, '',
         '## Flange (tool0) motion per mm of sideways stylus, and pivot per mm', '',
         '| Depth (mm) | ' + ' | '.join(f'{k}: flange mm/mm, pivot °/mm' for k in laws) + ' |', '|---|' + '---|' * len(laws)]
    for dmm in (0, 5, 10, 30, 60, 100, 130, 150):
        i = int(np.argmin(np.abs(depths - dmm / 1000)))
        L.append(f'| {dmm} | ' + ' | '.join(f'{res[k][0][i]:.2f}, {res[k][1][i]:.3f}' for k in laws) + ' |')
    L += ['', '## Continuity through the port (largest jump in flange gain between adjacent depths, −10…+10 mm)', '']
    m = np.abs(depths) <= 0.01
    for k in laws:
        L.append(f"- {k}: {np.max(np.abs(np.diff(res[k][0][m]))):.3f} mm/mm per 0.5 mm of depth")
    lo = (depths >= 0.005) & (depths <= gd)
    law = list(laws)[2]
    L += ['', f"Over 5–{gd * 1e3:.0f} mm the law's flange gain stays within {res[law][0][lo].min():.3f}–{res[law][0][lo].max():.3f} mm/mm "
          f"and its pivot within {res[law][1][lo].min():.3f}–{res[law][1][lo].max():.3f} °/mm; the fixed tip scale's "
          f"flange gain spans {res[list(laws)[0]][0][lo].min():.1f}–{res[list(laws)[0]][0][lo].max():.1f} mm/mm.", '',
          '## Scripted inputs through each law (offline, 500 Hz; flange = tool0)', '',
          'A: insert from −10 to +60 mm at 20 mm/s with the tip target 5 mm to the side. B: 5 mm-radius circle of the tip '
          'target at 10 mm depth, 1 Hz (31 mm/s). C: sideways sweep of the tip target at 24 mm/s at 20 mm depth.', '',
          '| Law | A: largest flange step in one 2 ms sample (mm) | A: peak flange acceleration (m/s²) | A: flange offset from the no-offset path, 30–60 mm deep (mm) | '
          'B: peak flange speed (mm/s) / acceleration (m/s²) | C: flange speed (mm/s, median) |', '|---|---|---|---|---|---|']
    srows = scripted(tool, laws)
    for k, row in srows.items():
        L.append(f"| {k} | {row['stepA']:.2f} | {row['aA']:.2f} | {row['off'][0]:.2f}–{row['off'][1]:.2f} | {1000 * row['vB']:.0f} / {row['aB']:.2f} | "
                 f"{1000 * row['vC']:.0f} |")
    L += ['', '## Not done yet', '', '- Replaying recorded RCM inputs through each law offline (flange acceleration for the same '
          'hand motion) and the on-arm overlay from the V6/V7 bags (ANALYSIS.md figures 2–3).', '',
          '## Prior claims checked', '', '- "fixed scale 30× at 10 mm, 9× at 30 mm, 1.4× at 130 mm; law 0.50 mm/mm at every '
          'depth; pivot 0.09–0.16 °/mm": see the table (gain here is the shipped value).', '', '## Figures', '']

    short = lambda k: {'fixed': 'Fixed tip scale (V6)', 'smoot': 'Smootherstep fade',
                       'depth': 'Depth-invariant law'}[k[:5]]
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(FULL * 0.8, 3.4), sharex=True)
    for (k, (gain, piv)), c in zip(res.items(), cols):
        a1.plot(depths * 1000, gain, color=c, label=short(k))
        a2.plot(depths * 1000, piv, color=c)
    for a in (a1, a2):
        a.axvline(0, color=INK2, lw=0.7, ls=':')
        a.axvspan(gd * 1000, 150, color=MUTED, alpha=0.1, lw=0)
    a1.text(-2, 40, 'Port', fontsize=8, color=INK2, ha='right')
    a1.set_yscale('log'); a1.set_ylim(0.3, 300); a1.set_ylabel('Flange motion\n(mm per stylus mm)')
    a1.legend(loc='lower center', bbox_to_anchor=(0.45, 1.0), ncol=3, handlelength=1.5, columnspacing=1.0)
    a2.set_yscale('log'); a2.set_ylabel('Pivot rotation\n(° per stylus mm)'); a2.set_xlabel('Insertion depth past the port (mm)')
    L.append(f"- `figures/{os.path.basename(save(fig, HERE, 'law_vs_depth'))}`")

    # scripted inputs: flange speed through each law
    fig, axes = plt.subplots(1, 3, figsize=(FULL, 2.3))
    titles = {'A': '(A) Insertion, 5 mm offset', 'B': '(B) Circle at 10 mm depth', 'C': '(C) Sweep at 20 mm depth'}
    xl = {'A': 'Insertion depth (mm)', 'B': 'Time (s)', 'C': 'Time (s)'}
    for ax, key in zip(axes, 'ABC'):
        for (k, row), c in zip(srows.items(), cols):
            x, v = row['series'][key]
            ax.plot(x, np.clip(v, 1e-2, None), color=c, lw=1.1, label=short(k))
        ax.set_yscale('log'); ax.set_title(titles[key]); ax.set_xlabel(xl[key])
    axes[0].set_ylabel('Flange speed (mm/s)')
    axes[2].axhline(24, color=INK2, lw=0.7, ls=':')
    h, l = axes[0].get_legend_handles_labels()
    fig.legend(h, l, loc='lower center', ncol=3, bbox_to_anchor=(0.5, 0.98))
    L.append(f"- `figures/{os.path.basename(save(fig, HERE, 'scripted_inputs'))}`: flange speed for three scripted "
             "stylus inputs through each law (log scale; dotted = the 24 mm/s stylus speed in C)")
    open(os.path.join(HERE, 'RESULTS.md'), 'w').write('\n'.join(L) + '\n')
    print('\n'.join(L))


if __name__ == '__main__':
    main()
