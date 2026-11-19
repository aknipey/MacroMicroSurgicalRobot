#!/usr/bin/env python3
"""Latch-offset test: fit the line the controller really held, build a corrected tool, report A/B runs.

The NDI RCM error in the SlowChecking / RCMOnly replays is mostly a tool-frame line mismatch: the controller
slides the tool along its own shaft direction (tools.yaml, from the hand-eye fit) and holds its own virtual tip,
and these differ from the wand's tracked shaft by ~0.3-1 deg and ~1-4 mm. The tool slides 100-150 mm through
the RCM point, so the tracked shaft misses the point by depth x angle (2026-11-19 analysis, VERSIONS.md).

  fit     RUN...  one line fixed in the wand frame (2 offset + 2 angle, shared) + one held point per run,
                  from the longest RCM segment of each run's NDI_PATH.csv. The held point at the latch, in the
                  wand frame, is the controller's virtual tip. The corrected tool undoes the fitted error to
                  first order and is mapped into tool0 with the hand-eye fit (tool0_to_wand). Prints the tool
                  and the error each run would have had with it (the fit residual), and with --add writes it.
  report  RUN...  per run: tool, NDI RCM rms as run, depth slope, fitted angle; then a mean per tool.

  python3 scripts/latch_line_fit.py fit logs/V9c/Latch/Train/RCMOnly*_Replay[0-9] --add 11 wand_L_corrected
  python3 scripts/latch_line_fit.py report logs/V9c/Latch/*/*_Replay[0-9]*

The hand-eye fit comes from the runs' META tool calibration; --handeye JSON overrides it (needed for tools made
by hand, e.g. the perturbed tools 7 and 8, which share tool 4's mount and fit).
"""
import argparse
import glob
import json
import os
import sys

import numpy as np
import yaml
from scipy.optimize import least_squares

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def quat_to_R(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def load_csv(path):
    import csv
    with open(path) as fh:
        r = csv.reader(fh)
        head = next(r)
        rows = list(r)
    out = {}
    for i, k in enumerate(head):
        try:
            out[k] = np.array([float(x[i]) if x[i] != '' else np.nan for x in rows])
        except ValueError:
            pass
    return out


class Run:
    """The longest RCM segment of one recording, wand poses in the camera frame."""

    def __init__(self, path):
        self.path = path.rstrip('/')
        self.name = os.path.relpath(self.path, os.path.join(PKG, 'logs')) if self.path.startswith(PKG) else self.path
        ndi_meta = yaml.safe_load(open(os.path.join(self.path, 'NDI_META.yaml')))
        cal = ndi_meta['calibration']
        self.o = np.array(cal['tip_offset_mm'], float)                 # NDI tip, wand (ROM) frame
        a = np.array(cal['shaft_axis'], float)
        self.a = a / np.linalg.norm(a)                                 # tip -> handle
        meta = yaml.safe_load(open(os.path.join(self.path, 'META.yaml')))
        tool = (meta.get('tool') or {}).get('at_start') or {}
        self.tool = tool.get('number')
        self.tool_name = tool.get('name', '?')
        self.tip_tool0 = np.array(tool.get('tip_tool0_mm', [np.nan] * 3), float)
        d = np.array(tool.get('direction_tool0', [np.nan] * 3), float)
        self.dir_tool0 = d / np.linalg.norm(d)
        self.calibration = tool.get('calibration')
        d = load_csv(os.path.join(self.path, 'NDI_PATH.csv'))
        act = np.nan_to_num(d['rcm_active']).astype(int)
        edges = np.flatnonzero(np.diff(np.r_[0, act, 0]))
        segs = list(zip(edges[::2], edges[1::2]))
        if not segs:
            raise ValueError(f'{self.name}: no RCM segment')
        s0, s1 = max(segs, key=lambda s: s[1] - s[0])
        idx = np.arange(s0, s1)
        idx = idx[d['visible'][idx] == 1]
        if len(idx) < 60:
            raise ValueError(f'{self.name}: only {len(idx)} tracked RCM frames')
        self.R = np.array([quat_to_R(d['q0'][i], d['qx'][i], d['qy'][i], d['qz'][i]) for i in idx])
        self.t = np.column_stack([d[k][idx] for k in ('x', 'y', 'z')])
        self.P = np.array([d[k][s0] for k in ('rcm_x', 'rcm_y', 'rcm_z')])   # NDI latched point
        self.n = len(idx)

    def miss(self, c, w, p):
        """Distance vectors from point p to the line (c, w) (wand frame) at every frame."""
        q = np.einsum('nij,j->ni', self.R, c) + self.t
        u = np.einsum('nij,j->ni', self.R, w / np.linalg.norm(w))
        r = p - q
        return r - np.einsum('ni,ni->n', r, u)[:, None] * u

    def depth(self):
        tips = np.einsum('nij,j->ni', self.R, self.o) + self.t
        u = np.einsum('nij,j->ni', self.R, self.a)
        return np.einsum('ni,ni->n', self.P - tips, u)

    def as_run(self):
        e = np.linalg.norm(self.miss(self.o, self.a, self.P), axis=1)
        slope = np.polyfit(self.depth(), e, 1)[0] * 100
        return float(np.sqrt(np.mean(e ** 2))), float(slope)


def basis(a):
    e1 = np.cross(a, [1.0, 0, 0])
    if np.linalg.norm(e1) < 0.1:
        e1 = np.cross(a, [0, 1.0, 0])
    e1 /= np.linalg.norm(e1)
    return e1, np.cross(a, e1)


def fit_line(runs):
    """Shared line (wand frame, relative to the NDI shaft) + per-run held point (camera frame)."""
    o, a = runs[0].o, runs[0].a
    e1, e2 = basis(a)

    def unpack(x):
        c = o + x[0] * e1 + x[1] * e2
        w = a + x[2] * e1 + x[3] * e2
        return c, w / np.linalg.norm(w), [r.P + x[4 + 3 * k:7 + 3 * k] for k, r in enumerate(runs)]

    def resid(x):
        c, w, ps = unpack(x)
        return np.concatenate([r.miss(c, w, p).reshape(-1) for r, p in zip(runs, ps)])

    sol = least_squares(resid, np.zeros(4 + 3 * len(runs)), x_scale='jac')
    c, w, ps = unpack(sol.x)
    return c, w, ps


def handeye_X(path):
    with open(path) as fh:
        return np.array(json.load(fh)['handeye']['tool0_to_wand'], float)


def corrected_tool(runs, X):
    """Move the controller's virtual tip onto the NDI tip and its line onto the NDI shaft. Returns tip, direction
    (holder -> tip) in tool0, info. Only the small wand-frame difference goes through the hand-eye rotation, so a
    hand-eye rotation error of e deg changes the correction by about e/57 of itself."""
    c, w, ps = fit_line(runs)
    Rx = X[:3, :3]
    r0 = runs[0]
    o, a = r0.o, r0.a
    # the controller's virtual tip seen in the wand frame: the held point at each run's latch frame
    v_w = np.mean([r.R[0].T @ (p - r.t[0]) for r, p in zip(runs, ps)], axis=0)
    tip = r0.tip_tool0 + Rx @ (o - v_w)
    direction = r0.dir_tool0 + Rx @ (w - a)          # w, a point tip -> handle; tool direction is holder -> tip
    w_est = -(Rx.T @ r0.dir_tool0)
    # consistency: is the virtual tip on the fitted line?
    off_line = np.linalg.norm((v_w - c) - np.dot(v_w - c, w) * w)
    resid = [float(np.sqrt(np.mean(np.sum(r.miss(c, w, p) ** 2, axis=1)))) for r, p in zip(runs, ps)]
    info = dict(angle_deg=float(np.degrees(np.arccos(np.clip(np.dot(w, w_est), -1, 1)))),
                tip_shift_mm=float(np.linalg.norm(tip - r0.tip_tool0)),
                virtual_tip_off_line_mm=float(off_line), predicted_rms_mm=resid,
                as_run_rms_mm=[r.as_run()[0] for r in runs])
    return tip, direction / np.linalg.norm(direction), info


def expand(paths):
    out = []
    for p in paths:
        hits = sorted(glob.glob(p)) or [p]
        out += [h for h in hits if os.path.exists(os.path.join(h, 'NDI_PATH.csv')) and not h.rstrip('/').endswith('_NO_NDI')]
    return out


def cmd_fit(args):
    runs = [Run(p) for p in expand(args.runs)]
    if not runs:
        sys.exit('no runs with NDI_PATH.csv')
    tools = {r.tool for r in runs}
    if len(tools) != 1:
        sys.exit(f'runs use different tools {tools}: fit one tool at a time')
    cal = args.handeye or runs[0].calibration
    if not cal or not os.path.exists(cal):
        sys.exit(f'no hand-eye JSON for tool {runs[0].tool} ({cal}); pass --handeye')
    tip, direction, info = corrected_tool(runs, handeye_X(cal))
    r0 = runs[0]
    print(f'fitted on {len(runs)} run(s) with tool {r0.tool} ({r0.tool_name}), hand-eye {os.path.basename(cal)}')
    for r, pr, ar in zip(runs, info['predicted_rms_mm'], info['as_run_rms_mm']):
        print(f'  {r.name:60s} as run {ar:5.2f} mm rms -> with the fitted line {pr:5.2f} mm')
    print(f'  line angle to the tool used: {info["angle_deg"]:.2f} deg; virtual tip {info["virtual_tip_off_line_mm"]:.2f} mm '
          f'off the fitted line (consistency)')
    print(f'  tool used:  tip {np.round(r0.tip_tool0, 3).tolist()}  direction {np.round(r0.dir_tool0, 6).tolist()}')
    print(f'  corrected:  tip {np.round(tip, 3).tolist()}  direction {np.round(direction, 6).tolist()}  '
          f'(tip moved {info["tip_shift_mm"]:.2f} mm, axis turned '
          f'{np.degrees(np.arccos(np.clip(np.dot(direction, r0.dir_tool0), -1, 1))):.2f} deg)')
    if args.json:
        json.dump(dict(tip_tool0_mm=tip.tolist(), direction_tool0=direction.tolist(), from_tool=r0.tool,
                       handeye=cal, runs=[r.name for r in runs], **info), open(args.json, 'w'), indent=1)
        print(f'  wrote {args.json}')
    if args.add:
        num, name = int(args.add[0]), args.add[1]
        import subprocess
        cmd = ['ros2', 'run', 'macro_micro_robot', 'tool_library.py'] + (['--library', args.library] if args.library else []) + ['add', str(num), name,
               '--tip', *[f'{v:.4f}' for v in tip], '--direction', *[f'{v:.6f}' for v in direction],
               '--length', str(args.length), '--force',
               '--notes', f'latch-line correction of tool {r0.tool} from {len(runs)} run(s): '
                          f'{info["angle_deg"]:.2f} deg, {info["tip_shift_mm"]:.2f} mm']
        subprocess.run(cmd, check=True)


def cmd_report(args):
    rows = []
    for p in expand(args.runs):
        try:
            r = Run(p)
        except (ValueError, OSError, KeyError) as exc:
            print(f'  skipped {p}: {exc}')
            continue
        rms, slope = r.as_run()
        c, w, _ = fit_line([r])
        ang = float(np.degrees(np.arccos(np.clip(np.dot(w, r.a), -1, 1))))
        rows.append((r, rms, slope, ang))
        print(f'  {r.name:62s} tool {r.tool!s:>3} {rms:5.2f} mm rms  slope {slope:5.2f} mm/100 mm  line angle {ang:4.2f} deg')
    groups = {}
    for r, rms, slope, ang in rows:
        kind = os.path.basename(r.path).split('_Replay')[0]
        groups.setdefault((kind, r.tool, r.tool_name), []).append((rms, slope, ang))
    print('\n  per trajectory and tool: n, NDI rms mean [min-max], slope, angle')
    for (kind, tool, name), v in sorted(groups.items(), key=lambda kv: (kv[0][0], str(kv[0][1]))):
        v = np.array(v)
        print(f'  {kind:34s} tool {tool!s:>3} {name:22s} n={len(v)}  {v[:, 0].mean():5.2f} [{v[:, 0].min():.2f}-{v[:, 0].max():.2f}] mm'
              f'  {v[:, 1].mean():5.2f} mm/100 mm  {v[:, 2].mean():4.2f} deg')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    f = sub.add_parser('fit')
    f.add_argument('runs', nargs='+')
    f.add_argument('--handeye', help='hand-eye JSON (default: the tool calibration in META.yaml)')
    f.add_argument('--add', nargs=2, metavar=('NUMBER', 'NAME'), help='write the corrected tool to tools.yaml')
    f.add_argument('--length', type=float, default=158.6)
    f.add_argument('--json', help='also write the result here')
    f.add_argument('--library', help='tools.yaml to write (default: the configured one)')
    r = sub.add_parser('report')
    r.add_argument('runs', nargs='+')
    args = ap.parse_args()
    {'fit': cmd_fit, 'report': cmd_report}[args.cmd](args)


if __name__ == '__main__':
    main()
