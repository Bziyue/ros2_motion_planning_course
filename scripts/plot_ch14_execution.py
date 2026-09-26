#!/usr/bin/env python3
"""Observed-route ROS execution; actual state markers over analytic references."""
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
root=Path(__file__).resolve().parents[1]
d=np.genfromtxt(root/'tmp/ch14_execution.csv',delimiter=',',names=True)
fig,axes=plt.subplots(1,3,figsize=(8.3,2.6),layout='constrained')
for ax,key,label in zip(axes,['x','v','a'],['x / m','vx / (m/s)','ax / (m/s²)']):
    ax.plot(d['t'],d[key+'_ref'],color='#087f8c',label='Analytic reference')
    ax.plot(d['t'][::14],d[key][::14],'o',mfc='none',mec='#d86629',ms=3,label='ROS state')
    for t in [.75,2.]:ax.axvline(t,color='#9bb0b8',ls=':',lw=.8)
    ax.set(xlabel='Simulation time / s',ylabel=label);ax.grid(alpha=.2)
fig.legend(*axes[0].get_legend_handles_labels(),loc='outside lower center',ncol=2,frameon=False)
for ext in ('pdf','png'):fig.savefig(root/f'textbook/figures/ch14_execution.{ext}',dpi=160,bbox_inches='tight',pad_inches=.12)
