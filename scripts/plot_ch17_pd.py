#!/usr/bin/env python3
"""Explicit zero-communication-delay controller experiment; ROS measured separately."""
from pathlib import Path
import shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parents[1]
d=np.genfromtxt(root/'tmp/ch17_pd/samples.csv',delimiter=',',names=True)
shutil.copyfile(root/'tmp/ch17_pd/summary.csv',root/'textbook/data/ch17_pd.csv')
fig,axes=plt.subplots(1,2,figsize=(8.3,3.4),layout='constrained')
labels=['Perfect reference','FF + PD','PD only','Mass mismatch','Force limited'];colors=['#183447','#087f8c','#a9774f','#7b62a3','#c94749']
for mode in range(5):
 s=d[(d['shape']==1)&(d['mode']==mode)];axes[0].plot(s['x'],s['y'],color=colors[mode],label=labels[mode],lw=1.2)
 if mode:axes[1].semilogy(s['t'],np.maximum(s['error'],1e-12),color=colors[mode])
axes[0].set(xlabel='x / m',ylabel='y / m');axes[0].set_aspect('equal');axes[1].set(xlabel='t / s',ylabel='Position error / m',ylim=(1e-5,3))
for a in axes:a.grid(alpha=.2)
fig.legend(*axes[0].get_legend_handles_labels(),loc='outside lower center',ncol=3,frameon=False)
fig.savefig(root/'textbook/figures/ch17_pd.pdf',bbox_inches='tight',pad_inches=.12)
