#!/usr/bin/env python3
"""Show whole-cell configuration-space inflation and paths on a supplied grid."""
from pathlib import Path
import csv
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Circle

source = Path(sys.argv[1] if len(sys.argv) > 1 else 'tmp/ch11_astar')
target = Path(sys.argv[2] if len(sys.argv) > 2 else 'textbook/figures')
fig, axes = plt.subplots(1, 3, figsize=(9, 3.6), constrained_layout=True)
for index, radius in enumerate([0., .15, .3]):
    data = np.genfromtxt(source/f'grid_{index}.csv', delimiter=',', names=True)
    values = np.where(data['source'] > 0, 2, data['blocked']).reshape(31,41)
    ax = axes[index]
    ax.imshow(values, origin='lower', extent=[0,4.1,0,3.1], interpolation='none',
              cmap=ListedColormap(['#ffffff','#bbd5db','#183447']), vmin=0, vmax=2)
    for neighbors, color, label in [(4,'#ba7727','4-neighbor'), (8,'#4560ad','8-neighbor')]:
        with open(source/f'path_{index}_{neighbors}.csv') as file:
            rows = list(csv.DictReader(file))
        raw = np.array([[float(r['x']),float(r['y'])] for r in rows if r['type'] == 'raw'])
        simple = np.array([[float(r['x']),float(r['y'])] for r in rows if r['type'] == 'simplified'])
        if raw.size:
            ax.plot(raw[:,0],raw[:,1],color=color,linewidth=1,alpha=.75,label=label)
        if neighbors == 8 and simple.size:
            ax.plot(simple[:,0],simple[:,1],color='#c53e48',marker='o',markersize=3,linewidth=1.4,label='Simplified')
    ax.scatter([.75],[.75],color='#087f8c',marker='s',s=22,zorder=5)
    ax.scatter([3.25],[2.25],color='#c53e48',marker='*',s=45,zorder=5)
    if radius:
        ax.add_patch(Circle((.75,.75), radius, edgecolor='#087f8c',fill=False))
    if index == 2:
        ax.text(2.9,.6,'Unreachable',ha='center',fontsize=9,color='#963440')
    ax.set(xlabel='x / m',ylabel='y / m',aspect='equal',title=f'Radius {radius:.2f} m, margin 0')
handles, labels = axes[0].get_legend_handles_labels()
fig.legend(handles,labels,loc='outside lower center',ncol=3,fontsize=9)
fig.savefig(target/'ch11_astar.pdf',bbox_inches='tight',pad_inches=.12)
fig.savefig(target/'ch11_astar.png',dpi=180,bbox_inches='tight',pad_inches=.12)
