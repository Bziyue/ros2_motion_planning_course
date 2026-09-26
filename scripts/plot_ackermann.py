#!/usr/bin/env python3
"""Plot actual Ackermann planning/tracking experiment; preserve geometric aspect."""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root = Path(__file__).resolve().parents[1]
a = np.genfromtxt(root/'tmp/ackermann_planning/tracking.csv', delimiter=',', names=True)
plt.rcParams.update({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False})
fig = plt.figure(figsize=(7.2, 4.6), layout='constrained')
gs = fig.add_gridspec(2, 2)
ax = fig.add_subplot(gs[0, :])
ax.plot(a['ref_x'], a['ref_y'], '--', color='#e19c34', lw=2.5, label='Flat reference')
ax.plot(a['x'], a['y'], color='#087e8b', lw=1, label='Force-driven plant')
ax.set(xlabel='x [m]', ylabel='y [m]', xlim=(-.1, 2.1), ylim=(-.15, .55))
ax.set_aspect('equal', adjustable='box')
ax.legend(loc='upper left', ncol=2, frameon=False)
for j, (key, label) in enumerate([('force', 'Applied force [N]'), ('steering', 'Front steering [rad]')]):
    ax = fig.add_subplot(gs[1, j])
    ax.plot(a['t'], a[key], color='#087e8b')
    ax.set(xlabel='t [s]', ylabel=label)
    ax.grid(alpha=.2)
fig.savefig(root/'textbook/figures/ackermann_tracking.pdf')
plt.close(fig)
