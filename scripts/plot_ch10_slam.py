#!/usr/bin/env python3
"""Plot sensor-derived keyframe SLAM and reconstructed occupancy maps."""
from pathlib import Path
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm

source = Path(sys.argv[1] if len(sys.argv) > 1 else 'tmp/ch10_slam')
target = Path(sys.argv[2] if len(sys.argv) > 2 else 'textbook/figures')
data = np.genfromtxt(source/'ch10_slam_trace.csv', delimiter=',', names=True)
fig, axes = plt.subplots(1, 2, figsize=(8, 3.6), constrained_layout=True)
for prefix, label, color, style in [('truth','Reference (evaluation)','black','--'),
    ('odom','Without loops','#c66530','-'), ('optimized','With validated loop','#087f8c','-')]:
    axes[0].plot(data[prefix+'_x'], data[prefix+'_y'], style, color=color, label=label)
axes[0].set(xlabel='x / m', ylabel='y / m', aspect='equal', title='Raw laser + IMU, fixed first frame')
axes[0].legend(loc='upper center', bbox_to_anchor=(.5,-.2), fontsize=8)
axes[1].plot(data['time'], data['odom_error'], color='#c66530', label='Without loops')
axes[1].plot(data['time'], data['optimized_error'], color='#087f8c', label='With validated loop')
axes[1].set(xlabel='Keyframe acquisition time / s', ylabel='Position error / m')
axes[1].legend(loc='upper left', fontsize=8)
for ax in axes:
    ax.grid(alpha=.2)
fig.savefig(target/'ch10_slam.pdf', bbox_inches='tight', pad_inches=.12)
fig.savefig(target/'ch10_slam.png', dpi=180, bbox_inches='tight', pad_inches=.12)

fig, axes = plt.subplots(1, 2, figsize=(8, 4), constrained_layout=True)
cmap = ListedColormap(['#ffffff','#dec899','#183447','#b6c7cb'])
norm = BoundaryNorm([0,35,65,101,256], cmap.N)
for ax, name, title in zip(axes, ['no_loop','loop'], ['Without loops','Rebuilt after validated loop']):
    values = np.loadtxt(source/(name+'.pgm'), skiprows=3).reshape(220,220)
    ax.imshow(values, origin='lower', extent=[-11,11,-11,11], cmap=cmap, norm=norm, interpolation='none')
    ax.set(xlabel='x / m', ylabel='y / m', aspect='equal', xlim=(-4.5,10.5), ylim=(-4.5,8.5), title=title)
fig.savefig(target/'ch10_maps.pdf')
fig.savefig(target/'ch10_maps.png', dpi=180)
