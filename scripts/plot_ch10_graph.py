#!/usr/bin/env python3
"""Plot a synthetic relative-pose graph experiment, not a SLAM accuracy claim."""
from pathlib import Path
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

source = Path(sys.argv[1] if len(sys.argv) > 1 else 'tmp/ch10_graph')
target = Path(sys.argv[2] if len(sys.argv) > 2 else 'textbook/figures')
data = np.genfromtxt(source/'ch10_graph_trace.csv', delimiter=',', names=True)
fig, axes = plt.subplots(1, 2, figsize=(8, 3.6), constrained_layout=True)
for prefix, name, color, style in [('truth', 'Reference (evaluation)', 'black', '--'),
                                  ('odom', 'Odometry constraints', '#c66530', '-'),
                                  ('optimized', 'With loop constraint', '#087f8c', '-')]:
    axes[0].plot(data[prefix+'_x'], data[prefix+'_y'], style, color=color, label=name)
axes[0].plot([data['odom_x'][-1], 0], [data['odom_y'][-1], 0], ':', color='#8a487d', linewidth=2.)
axes[0].scatter([0], [0], marker='s', color='black', s=25, zorder=4)
axes[0].set(xlabel='x / m', ylabel='y / m', aspect='equal', title='Synthetic square graph, seed 1010')
axes[0].legend(loc='upper center', bbox_to_anchor=(.5, -.2), ncol=1, fontsize=8)
axes[1].plot(data['index'], data['odom_error'], color='#c66530', label='Odometry constraints')
axes[1].plot(data['index'], data['optimized_error'], color='#087f8c', label='With loop constraint')
axes[1].set(xlabel='Keyframe index', ylabel='Position error / m')
axes[1].legend(fontsize=8)
for ax in axes:
    ax.grid(alpha=.2)
fig.savefig(target/'ch10_graph.pdf')
fig.savefig(target/'ch10_graph.png', dpi=180)
