#!/usr/bin/env python3
"""Visualize the unchanged prefix and C2 moving-boundary handover."""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parents[1]
s=np.genfromtxt(root/'tmp/ch19_handover/samples.csv',delimiter=',',names=True)
fig=plt.figure(figsize=(8.3,3.4),layout='constrained');g=fig.add_gridspec(2,2,width_ratios=[1,1.35])
xy=fig.add_subplot(g[:,0]);v=fig.add_subplot(g[0,1]);a=fig.add_subplot(g[1,1])
xy.plot(s['old_x'],s['old_y'],ls='--',color='#83949f',label='Old reference')
xy.plot(s['x'],s['y'],color='#087f8c',label='Scheduled reference')
k=np.argmin(abs(s['t']-1.5));xy.scatter(s['x'][k],s['y'][k],color='#c94749',s=24,zorder=5)
xy.set(xlabel='x / m',ylabel='y / m');xy.set_aspect('equal')
v.plot(s['t'],s['vx'],color='#087f8c');a.plot(s['t'],s['ax'],color='#087f8c')
v.set(ylabel='vx / (m/s)');a.set(ylabel='ax / (m/s²)',xlabel='t / s')
for ax in (v,a):ax.axvline(1.5,color='#c94749',ls=':',lw=1);ax.set_xlim(0,6)
for ax in (xy,v,a):ax.grid(alpha=.2)
fig.legend(*xy.get_legend_handles_labels(),loc='outside lower center',ncol=2,frameon=False)
fig.savefig(root/'textbook/figures/ch19_handover.pdf',bbox_inches='tight',pad_inches=.12)
