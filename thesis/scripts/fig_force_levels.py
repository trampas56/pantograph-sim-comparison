#!/usr/bin/env python3
"""
fig_force_levels.py - Τα τρία σχήματα σύγκρισης του ελέγχου δύναμης
(0,5 N, 1 N, 2 N), ένα ανά επίπεδο, με τέσσερα πλαίσια όπως οι διαφάνειες
17-19 της παρουσίασης: Gazebo, Webots / MuJoCo, Unity.

Δεδομένα: thesis/data/force/<προσομοιωτής>_<επίπεδο>N.csv
  gazebo, mujoco : εκτελέσεις Σεπτεμβρίου 2026 (κοινός ελεγκτής)
  webots         : 0,5 και 2 N της 24/9, 1 N του Ιουνίου
  unity          : επαναλήψεις Οκτωβρίου 2026
Έξοδος: thesis/figures/common/fig_force_{05,1,2}N.pdf

Χρήση: python3 scripts/fig_force_levels.py   (από τον φάκελο thesis/)
"""
import csv
import os

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, '..', 'data', 'force')
FIG = os.path.join(HERE, '..', 'figures', 'common')

RED = '#d62728'
SIM = {
    'gazebo': ('Gazebo (DART)', '#2a78d6'),
    'webots': ('Webots (ODE)', '#eb6834'),
    'mujoco': ('MuJoCo', '#1baf7a'),
    'unity':  ('Unity (PhysX)', '#4a3aa7'),
}
LIMIT = {'gazebo': 1.341, 'webots': 1.341, 'mujoco': 1.341, 'unity': 1.476}

plt.rcParams.update({
    'font.family': 'Noto Sans', 'font.size': 8.5, 'axes.titlesize': 9,
    'axes.labelsize': 8.5, 'legend.fontsize': 8, 'xtick.labelsize': 7.5,
    'ytick.labelsize': 7.5, 'axes.grid': True, 'grid.alpha': 0.25,
    'grid.linewidth': 0.5, 'axes.spines.top': False,
    'axes.spines.right': False, 'pdf.fonttype': 42, 'axes.unicode_minus': False,
})


def load(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}


def series(sim, lvl):
    """Χρόνος από την αρχή της ράμπας, F_des, F_est, μέτρηση."""
    c = load(os.path.join(DATA, f'{sim}_{lvl}N.csv'))
    if sim == 'unity':                       # η ράμπα ξεκινά στο t = 0
        return c['t'], c['F_cmd_N'], c['F_est_N'], c['contactF_N']
    meas = c['f_contact_x'] if sim == 'webots' else c['f_sensor_x']
    return c['t'] - 2.0, c['f_des_x'], c['f_est_x'], meas


def xlim(sim, lvl):
    if sim == 'webots':
        return {'0.5': (-2, 118), '1': (-2, 20), '2': (-2, 18.8)}[lvl]
    if sim == 'unity':
        return (0, 12)
    return (-2, 12)


def panel(ax, sim, lvl):
    name, color = SIM[sim]
    t, fd, fe, fm = series(sim, lvl)
    lo, hi = xlim(sim, lvl)
    sel = (t >= lo) & (t <= hi)
    top = {'0.5': 0.72, '1': 1.32, '2': 2.35}[lvl]
    ax.plot(t[sel], fd[sel], color='black', lw=0.9, ls=(0, (4, 2.5)))
    ax.plot(t[sel], fe[sel], color='#8a8f98', lw=1.0)
    ax.plot(t[sel], fm[sel], color=RED, lw=1.5)
    if lvl == '2':
        ax.axhline(LIMIT[sim], color='#8a8f98', lw=0.7, ls=(0, (1, 2)))
        ax.text(hi, LIMIT[sim] + 0.05, f'όριο ροπής {LIMIT[sim]:.3f} N'.replace('.', ','),
                ha='right', va='bottom', fontsize=7, color='#5d636e')
    peak = fm[sel].max()
    if peak > top:                          # αιχμή πρόσκρουσης έξω από την κλίμακα
        tp = t[sel][np.argmax(fm[sel])]
        ax.annotate(f'αιχμή {peak:.1f} N'.replace('.', ','), xy=(tp, top * 0.97),
                    xytext=(tp + 1.0, top * 0.92), fontsize=7, color=RED, va='center',
                    arrowprops=dict(arrowstyle='->', color=RED, lw=0.8))
    ax.set_xlim(lo, hi)
    ax.set_ylim(-0.05 * top, top)
    ax.set_title(name, color=color, loc='left', fontweight='bold')
    if sim == 'webots':                     # ο αισθητήρας δίνει 0, η κόκκινη είναι το ισοζύγιο
        ax.set_title('δύναμη από ισοζύγιο', loc='right', fontsize=7.5, color='#5d636e')


def figure(lvl):
    fig, axes = plt.subplots(2, 2, figsize=(5.8, 3.9))
    for ax, sim in zip(axes.flat, ('gazebo', 'webots', 'mujoco', 'unity')):
        panel(ax, sim, lvl)
    for ax in axes[:, 0]:
        ax.set_ylabel('δύναμη (N)')
    for ax in axes[1, :]:
        ax.set_xlabel('χρόνος από την αρχή της ράμπας (s)')
    h = [Line2D([0], [0], color='black', lw=0.9, ls=(0, (4, 2.5))),
         Line2D([0], [0], color='#8a8f98', lw=1.0),
         Line2D([0], [0], color=RED, lw=1.5)]
    lab = ['$F_{des}$, η εντολή', '$F_{est}$, από τις ροπές',
           '$F_{sensor}$, η δύναμη στον τοίχο']
    fig.tight_layout(rect=(0, 0.07, 1, 1), h_pad=0.8, w_pad=1.2)
    fig.legend(h, lab, loc='lower center', ncol=3, frameon=False,
               bbox_to_anchor=(0.5, 0.0), handlelength=2.4)
    out = os.path.join(FIG, f"fig_force_{lvl.replace('.', '')}N.pdf")
    fig.savefig(out, bbox_inches='tight')
    plt.close(fig)
    print(out)


if __name__ == '__main__':
    for lvl in ('0.5', '1', '2'):
        figure(lvl)
