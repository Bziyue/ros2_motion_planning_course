#!/usr/bin/env python3
"""Compare laser sample-and-hold and actual IMU/laser fusion at IMU timestamps."""
from pathlib import Path
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

source = Path(sys.argv[1] if len(sys.argv) > 1 else 'tmp/ch09_fusion')
target = Path(sys.argv[2] if len(sys.argv) > 2 else 'textbook/figures')
fig, axes = plt.subplots(2, 2, figsize=(8, 5.3), constrained_layout=True)
for col, (name, omega) in enumerate([('slow', .4), ('fast', 1.2)]):
    data = np.genfromtxt(source / (name+'.csv'), delimiter=',', names=True)
    for row, field in enumerate(['error', 'yaw_error']):
        ax = axes[row, col]
        for prefix, label, color in [('laser', 'Laser, held', '#c66530'), ('fusion', 'IMU + laser', '#12334a')]:
            ax.plot(data['time'], np.abs(data[prefix+'_'+field]), label=label, color=color, linewidth=1.)
        ax.axvspan(3, 3.5, color='gray', alpha=.18)
        ax.set(xlabel='Time / s', ylabel='Position error / m' if row == 0 else 'Absolute yaw error / rad')
        ax.grid(alpha=.2)
        if row == 0:
            ax.set_title(f'Angular speed {omega} rad/s')
            ax.legend(loc='upper right', fontsize=8)
fig.savefig(target / 'ch09_fusion.pdf')
fig.savefig(target / 'ch09_fusion.png', dpi=180)
