#!/usr/bin/env python3
"""Explicit observed-grid fixtures; unknown, inflated cells and physical occupancy differ."""
from pathlib import Path
import shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
root=Path(__file__).resolve().parents[1]
d=np.genfromtxt(root/'tmp/ch19_replanner/curves.csv',delimiter=',',names=True)
m=np.genfromtxt(root/'tmp/ch19_replanner/maps.csv',delimiter=',',names=True)
shutil.copyfile(root/'tmp/ch19_replanner/summary.csv',root/'textbook/data/ch19_replanner.csv')
fig,axes=plt.subplots(1,2,figsize=(8.3,3.6),layout='constrained')
for scene,ax in enumerate(axes):
 raw=m[m['scene']==scene]['occupancy'].reshape(40,60);blocked=m[m['scene']==scene]['blocked'].reshape(40,60)
 colors=np.ones_like(raw);colors[blocked>0]=2;colors[raw<0]=0;colors[raw>=50]=3
 ax.imshow(colors,origin='lower',extent=(0,6,0,4),cmap=ListedColormap(['#b4bcc2','#ffffff','#f3ded0','#183447']),vmin=0,vmax=3,interpolation='nearest')
 for mode,label,color in [(0,'Certified stop fallback','#a9774f'),(1,'Certified Spline','#087f8c')]:
  s=d[(d['scene']==scene)&(d['mode']==mode)];ax.plot(s['x'],s['y'],color=color,lw=1.4,label=label)
 ax.scatter([.75],[2. if scene==0 else .75],s=22,color='#183447',zorder=5)
 ax.scatter([5.35],[2. if scene==0 else .75],marker='*',s=65,color='#c94749',zorder=5)
 ax.set(xlabel='x / m',ylabel='y / m',title='Unknown goal' if scene==0 else 'Known wall, local prefix');ax.set_aspect('equal')
fig.legend(*axes[0].get_legend_handles_labels(),loc='outside lower center',ncol=2,frameon=False)
fig.savefig(root/'textbook/figures/ch19_replanner.pdf',bbox_inches='tight',pad_inches=.12)
