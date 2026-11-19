"""One figure style for every folder: one colour + marker per condition, report widths, PDF + PNG."""
import glob
import os

import matplotlib
import numpy as np
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402

# The report is set in Latin Modern Roman (lmodern, 10 pt); the figures use the same face so text matches.
for _f in glob.glob('/usr/share/texmf/fonts/opentype/public/lm/lmroman10-*.otf'):
    font_manager.fontManager.addfont(_f)

INK, INK2, MUTED, GRID, SURFACE = '#1f1f1e', '#52514e', '#8a8983', '#e4e3df', '#ffffff'
FULL, HALF = 5.85, 2.9         # inches: the 150 mm text width (less 1 mm for the tight bbox), and half
# Figures are made at the width they are printed at (LaTeX includes them at natural size, no scaling), so every
# figure's text comes out at the same point size: 9 pt labels, 8 pt ticks and legends (body text is 10 pt).

# Palette (11-19), after simplifiedsciencepublishing.com "Best color palettes for scientific figures": contrast mainly
# through lightness, complementary hues (blue / orange) for the main pair, grey as the default, colour only on what the
# figure is about, the same hex codes in every figure, ColorBrewer for sequential scales. Hues are Okabe-Ito
# (colour-blind safe). Checked by simulation (Machado 2009, deutan / protan / tritan, OKLab): every pair used together
# is >= 8 Delta E for all three; the two pairs close in greyscale (position / trajectory, velocity filtered /
# position V1) always differ by marker too. Green is not used: it collapses into grey for deuteranopes.
BLUE, ORANGE, VERMILLION, SKY = '#0072B2', '#E69F00', '#D55E00', '#56B4E9'
GREY, GREY_DARK, GREY_LIGHT = MUTED, '#5F5E5A', '#C9C8C3'
GOOD, BAD = BLUE, VERMILLION   # on / off, pass / fail, adopted / rejected
MODE = {
    'trajectory':          dict(color=GREY_DARK, marker='D', label='Trajectory'),
    'position':            dict(color=BLUE, marker='^', label='Position'),
    'velocity_optimised':  dict(color=ORANGE, marker='o', label='Velocity, filtered'),
    'velocity':            dict(color=ORANGE, marker='o', label='Velocity, filtered'),
    'velocity_unfiltered': dict(color=VERMILLION, marker='v', label='Velocity, unfiltered'),
    'position_v1':         dict(color=SKY, marker='s', label='Position (V1)'),
}
# Ordered versions: ColorBrewer Blues, 18-21 % lightness steps (the article's 15-30 % rule)
VERSION = {'V1': dict(color='#6BAED6', marker='o'), 'V2': dict(color='#2171B5', marker='s'),
           'V3': dict(color='#08306B', marker='^')}
# Sequential: ColorBrewer Blues without its near-white end, so the lightest values still show on white
from matplotlib.colors import LinearSegmentedColormap  # noqa: E402
SEQ = LinearSegmentedColormap.from_list('thesis_seq', matplotlib.colormaps['Blues'](np.linspace(0.3, 1.0, 256)))
if 'thesis_seq' not in matplotlib.colormaps:
    matplotlib.colormaps.register(SEQ)


def style():
    plt.rcParams.update({
        'font.family': 'serif', 'font.serif': ['Latin Modern Roman', 'DejaVu Serif'],
        'mathtext.fontset': 'cm', 'axes.unicode_minus': True,
        'font.size': 9, 'axes.titlesize': 9, 'axes.labelsize': 9, 'legend.fontsize': 8,
        'xtick.labelsize': 8, 'ytick.labelsize': 8, 'figure.titlesize': 9,
        'axes.edgecolor': MUTED, 'axes.labelcolor': INK, 'text.color': INK,
        'xtick.color': INK2, 'ytick.color': INK2, 'axes.spines.top': False, 'axes.spines.right': False,
        'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.5, 'axes.axisbelow': True,
        'axes.titlelocation': 'left', 'axes.titlepad': 4, 'figure.facecolor': SURFACE, 'axes.facecolor': SURFACE,
        'axes.prop_cycle': matplotlib.cycler(color=[BLUE, ORANGE, VERMILLION, SKY, GREY_DARK]),
        'legend.frameon': False, 'lines.linewidth': 1.4, 'savefig.bbox': 'tight', 'savefig.pad_inches': 0.02,
        'pdf.fonttype': 42, 'figure.constrained_layout.use': True,
        'figure.constrained_layout.h_pad': 0.02, 'figure.constrained_layout.w_pad': 0.02,
    })


def panel(ax, letter, text=''):
    """Panel label in the report's style: '(a) short name'."""
    ax.set_title(f'({letter}) {text}'.rstrip(), fontsize=9)


def save(fig, folder, name):
    out = os.path.join(folder, 'figures')
    os.makedirs(out, exist_ok=True)
    for ext in ('pdf', 'png'):
        fig.savefig(os.path.join(out, f'{name}.{ext}'), dpi=170)
    w = fig.get_tightbbox(fig.canvas.get_renderer()).width + 2 * plt.rcParams['savefig.pad_inches']
    print(f'  {name}: {w:.2f} in wide' + ('  ** wider than the 150 mm text width **' if w > 150 / 25.4 else ''))
    plt.close(fig)
    return os.path.join(out, name + '.png')


def day_label(day):
    """'2026-10-26' -> '26 Oct', the form the report text and tables use."""
    import datetime
    return datetime.date.fromisoformat(str(day)[:10]).strftime('%-d %b')
