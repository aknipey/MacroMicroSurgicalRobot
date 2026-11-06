"""Shared pieces for the thesis plots: style, the condition registry, run loading, saving.

Read-only on logs/: every figure is written to thesis/plotting/out/ (PNG for viewing, PDF for LaTeX).
"""
import csv
import glob
import os
import re

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.abspath(os.path.join(HERE, '..', '..'))
LOGS = os.path.join(PKG, 'logs')
OUT = os.path.join(HERE, 'out')

# Ink and surfaces (light theme of the dataviz reference palette)
INK, INK2, MUTED, GRID, SURFACE = '#1f1f1e', '#52514e', '#8a8983', '#e4e3df', '#ffffff'

# One colour + marker per condition, fixed everywhere (colour follows the condition, never its rank).
# Categorical slots 1-4 of the validated reference palette; the marker is the secondary encoding.
CONDITIONS = {
    'nominal-velocity': dict(label='V7 nominal kin., velocity', folder='V7', color='#2a78d6', marker='o',
                             date='2026-10-26'),
    'calibrated-velocity': dict(label='V8 calibrated kin., velocity', folder='V8/CalibratedKinematics',
                                color='#eb6834', marker='s', date='2026-11-03'),
    'calibrated-position': dict(label='V9 calibrated kin., position', folder='V9/PositionMode',
                                color='#1baf7a', marker='^', date='2026-11-03'),
    'calibrated-trajectory': dict(label='V9 calibrated kin., trajectory', folder='V9/TrajectoryMode',
                                  color='#eda100', marker='D', date='2026-11-03'),
}
SETS = {   # folder stem -> short name, in reading order (slowest/simplest first)
    'RCMOnlySimpleHolder': 'RCMOnly',
    'WholeRunDrinkBottleSimpleHolder': 'WholeRun',
    'FastCheckingRCMSimpleHolder': 'FastChecking',
    'SlowCheckingRCMSimpleHolder': 'SlowChecking',
}
# Runs whose NDI numbers are not robot error (reason kept with them, shown in figure notes)
EXCLUDE = {
    'V8/CalibratedKinematics/FastCheckingRCMSimpleHolder_Replay1':
        'NDI camera moved at t = 28 s, mid-segment (E0, thesis/planning/analysis/E0_calibrated_kinematics)',
}


def style():
    plt.rcParams.update({
        'font.size': 9, 'axes.titlesize': 9, 'axes.labelsize': 9, 'legend.fontsize': 7.5,
        'xtick.labelsize': 8, 'ytick.labelsize': 8,
        'axes.edgecolor': MUTED, 'axes.labelcolor': INK, 'text.color': INK,
        'xtick.color': INK2, 'ytick.color': INK2,
        'axes.spines.top': False, 'axes.spines.right': False,
        'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.6, 'axes.axisbelow': True,
        'axes.titlelocation': 'left', 'figure.facecolor': SURFACE, 'axes.facecolor': SURFACE,
        'legend.frameon': False, 'lines.linewidth': 1.6, 'savefig.bbox': 'tight',
    })


def save(fig, name, note=None):
    """out/<name>.png and .pdf; an optional grey footnote under the figure."""
    import textwrap
    os.makedirs(OUT, exist_ok=True)
    if note:   # wrapped to the figure width, and below everything already drawn (legends included)
        width = max(40, int(fig.get_figwidth() * 19))
        bottom = fig.get_tightbbox(fig.canvas.get_renderer()).y0 / fig.get_figheight()
        fig.text(0.01, bottom - 0.02, '\n'.join(textwrap.wrap(note, width)), fontsize=6.5, color=MUTED,
                 ha='left', va='top')
    for ext in ('png', 'pdf'):
        fig.savefig(os.path.join(OUT, f'{name}.{ext}'), dpi=160)
    plt.close(fig)
    print(f'  out/{name}.png')


def exclusion_note(runs_used):
    hit = [f'{k.split("/")[-1]} ({v})' for k, v in EXCLUDE.items() if k in runs_used]
    return 'Excluded: ' + '; '.join(hit) if hit else None


# ---------------------------------------------------------------- runs
class Run:
    """One replay folder; NDI data loaded on first use."""

    def __init__(self, path, condition=None):
        self.path = os.path.abspath(path)
        self.key = os.path.relpath(self.path, LOGS)
        self.name = os.path.basename(self.path)
        m = re.match(r'(.*)_Replay(\d+)$', self.name)
        self.stem, self.replay = (m.group(1), int(m.group(2))) if m else (self.name, 0)
        self.set = SETS.get(self.stem, self.stem)
        self.condition = condition
        self._ndi = None

    def __repr__(self):
        return f'Run({self.key})'

    @property
    def meta(self):
        return yaml.safe_load(open(os.path.join(self.path, 'META.yaml')))

    @property
    def ndi_meta(self):
        p = os.path.join(self.path, 'NDI_META.yaml')
        return yaml.safe_load(open(p)) if os.path.exists(p) else None

    @property
    def has_ndi(self):
        m = self.ndi_meta
        return bool(m and m.get('rcm_segments'))

    @property
    def ndi(self):
        """RCM segment (largest) of NDI_PATH.csv, visible rows, camera frame, mm."""
        if self._ndi is None:
            seg = max(self.ndi_meta['rcm_segments'], key=lambda s: s.get('samples', 0))
            with open(os.path.join(self.path, 'NDI_PATH.csv'), newline='') as f:
                rows = [r for r in csv.DictReader(f) if r['visible'] == '1']
            col = lambda keys: np.array([[float(r[k]) if r[k] else np.nan for k in keys] for r in rows])
            t = col(['t'])[:, 0]
            inseg = (t >= seg['t_start']) & (t <= seg['t_end'])
            tip, axis = col(('tip_x', 'tip_y', 'tip_z'))[inseg], col(('axis_x', 'axis_y', 'axis_z'))[inseg]
            P = np.array(seg['rcm_point_mm'])
            self._ndi = dict(t=t[inseg] - seg['t_start'], tip=tip, axis=axis, P=P, seg=seg,
                             error=col(['rcm_error'])[inseg, 0], depth=col(['rcm_depth'])[inseg, 0],
                             fit_error=col(['fit_error'])[inseg, 0])
        return self._ndi


def condition_runs(condition, include_excluded=False):
    folder = os.path.join(LOGS, CONDITIONS[condition]['folder'])
    runs = [Run(p, condition) for p in sorted(glob.glob(os.path.join(folder, '*_Replay[0-9]')))]
    return [r for r in runs if r.set in SETS.values() and r.has_ndi
            and (include_excluded or r.key not in EXCLUDE)]


def all_runs(include_excluded=False):
    return {c: condition_runs(c, include_excluded) for c in CONDITIONS}


# ---------------------------------------------------------------- RCM geometry
def plane_basis(n):
    """Two unit vectors spanning the plane normal to n; u keeps camera +y's direction (stable across runs)."""
    n = n / np.linalg.norm(n)
    ref = np.array([0.0, 1.0, 0.0]) if abs(n[1]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u = ref - (ref @ n) * n
    u /= np.linalg.norm(u)
    return u, np.cross(n, u)


def pierce_points(ndi):
    """Where each shaft line crosses the plane through the latched RCM point normal to the mean shaft.

    Returns (N, 2) in mm, origin = latched point. The distance from the origin is close to rcm_error
    for small tilts (exactly rcm_error / cos(tilt))."""
    n = ndi['axis'].mean(0)
    n /= np.linalg.norm(n)
    u, v = plane_basis(n)
    tip, d, P = ndi['tip'], ndi['axis'], ndi['P']
    s = ((P - tip) @ n) / (d @ n)
    x = tip + s[:, None] * d - P
    return np.c_[x @ u, x @ v]


def tilt_deg(ndi):
    """Shaft angle from its direction at the latch (deg)."""
    a0 = ndi['axis'][:6].mean(0)
    a0 /= np.linalg.norm(a0)
    return np.degrees(np.arccos(np.clip(ndi['axis'] @ a0, -1, 1)))


def rms(x):
    x = np.asarray(x)
    return float(np.sqrt(np.nanmean(x ** 2)))


def bias_spread(xy):
    """rms^2 = bias^2 + spread^2: bias = |mean pierce point| (latch offset), spread = scatter about it."""
    m = np.nanmean(xy, 0)
    return float(np.linalg.norm(m)), rms(np.linalg.norm(xy - m, axis=1))


def legend_handles(conditions):
    from matplotlib.lines import Line2D
    return [Line2D([], [], color=CONDITIONS[c]['color'], marker=CONDITIONS[c]['marker'], ms=5, lw=1.6,
                   label=CONDITIONS[c]['label']) for c in conditions]
