#!/usr/bin/env python3
"""Soft optimization can reduce cost and still violate a verified corridor."""
from pathlib import Path
import shutil
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
root=Path(__file__).resolve().parents[1]
d=np.genfromtxt(root/'tmp/ch15_optimization/samples.csv',delimiter=',',names=True)
r=np.genfromtxt(root/'tmp/ch13_corridor/regions_1.csv',delimiter=',',names=True)
fig,ax=plt.subplots(1,2,figsize=(8.3,3.1),layout='constrained')
for k in np.unique(r['region']):
    v=r[r['region']==k];ax[0].fill(v['x'],v['y'],color='#087f8c',alpha=.1);ax[0].plot(np.r_[v['x'],v['x'][0]],np.r_[v['y'],v['y'][0]],color='#8fadb5',lw=.8)
for mode,color,label in [(0,'#677687','Initial'),(1,'#ba763d','Position only'),(2,'#087f8c','Position + time')]:
    v=d[d['mode']==mode];ax[0].plot(v['x'],v['y'],color=color,label=label);ax[1].plot(v['t'],v['speed'],color=color)
ax[0].plot([3.05,3.05],[0,2.9],color='#183447',lw=5)
ax[0].set(xlabel='x / m',ylabel='y / m');ax[0].set_aspect('equal');ax[1].set(xlabel='t / s',ylabel='Speed / (m/s)')
ax[1].axhline(.8,color='#c7c7c7',ls=':',label='Soft target')
for a in ax:a.grid(alpha=.2)
fig.legend(*ax[0].get_legend_handles_labels(),loc='outside lower center',ncol=3,frameon=False)
fig.savefig(root/'textbook/figures/ch15_optimization.pdf',bbox_inches='tight',pad_inches=.12)
shutil.copyfile(root/'tmp/ch15_optimization/summary.csv',root/'textbook/data/ch15_optimization.csv')
