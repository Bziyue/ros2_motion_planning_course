#!/usr/bin/env python3
"""Plot first-frame-aligned evaluation data; preserve equal physical axes."""
from pathlib import Path
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

source = Path(sys.argv[1] if len(sys.argv) > 1 else 'tmp/ch08_results')
target = Path(sys.argv[2] if len(sys.argv) > 2 else 'textbook/figures')
target.mkdir(parents=True, exist_ok=True)
fig, axes = plt.subplots(1, 2, figsize=(8, 3.3), constrained_layout=True)
for beams, color in ((90, '#c66530'), (360, '#508288'), (720, '#12334a')):
    data = np.genfromtxt(source / f'path_{beams}_noisy.csv', delimiter=',', names=True,
                         dtype=None, encoding='utf-8')
    axes[0].plot(data['estimated_x'], data['estimated_y'], label=f'{beams} beams', color=color)
    axes[1].plot(data['time'], data['error'], label=f'{beams} beams', color=color)
    failed = ~np.isin(data['status'], ['converged', 'initialized'])
    axes[1].scatter(data['time'][failed], data['error'][failed], marker='x', s=20, color=color)
axes[0].plot(data['truth_x'], data['truth_y'], '--', color='black', lw=.8, label='Truth (evaluation)')
axes[0].set(xlabel='x / m', ylabel='y / m', title='Range noise sigma = 0.03 m', aspect='equal')
axes[1].set(xlabel='Time / s', ylabel='Position error / m', title='Crosses: rejected scans, held estimate')
for ax in axes:
    ax.grid(alpha=.2)
    ax.legend(fontsize=7)
fig.savefig(target / 'ch08_odometry.pdf')
fig.savefig(target / 'ch08_odometry.png', dpi=180)
