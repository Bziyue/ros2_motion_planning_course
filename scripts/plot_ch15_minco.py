#!/usr/bin/env python3
"""Same waypoint/time problem: zero internal derivatives versus minimum jerk."""
from pathlib import Path
import shutil
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
root=Path(__file__).resolve().parents[1]
d=np.genfromtxt(root/'tmp/ch15_minco/samples.csv',delimiter=',',names=True)
shutil.copyfile(root/'tmp/ch15_minco/summary.csv',root/'textbook/data/ch15_minco.csv')
fig,axes=plt.subplots(1,3,figsize=(8.3,2.9),layout='constrained')
for mode,color,label in [(0,'#a07755','Stop at knots'),(1,'#087f8c','Minimum jerk')]:
    m=d[d['mode']==mode];axes[0].plot(m['x'],m['y'],color=color,label=label)
    axes[1].plot(m['t'],np.hypot(m['vx'],m['vy']),color=color)
    axes[2].plot(m['t'],m['jy'],color=color)
axes[0].scatter([0,1,2,3,4],[0,1,-.2,.8,0],s=18,c='white',edgecolors='#183447',zorder=5)
axes[0].set(xlabel='x / m',ylabel='y / m');axes[0].set_aspect('equal')
axes[1].set(xlabel='t / s',ylabel='Speed / (m/s)');axes[2].set(xlabel='t / s',ylabel='Jerk y / (m/s³)')
for ax in axes:ax.grid(alpha=.2)
for ax in axes[1:]:
    for t in [1.5,3,4.5]:ax.axvline(t,color='#c9d4d9',ls=':',lw=.8)
fig.legend(*axes[0].get_legend_handles_labels(),loc='outside lower center',ncol=2,frameon=False)
for ext in ['pdf','png']:fig.savefig(root/f'textbook/figures/ch15_minco.{ext}',dpi=160,bbox_inches='tight',pad_inches=.12)
