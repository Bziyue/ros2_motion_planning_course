#!/usr/bin/env python3
"""Plot the measured exact wheel-rate circle with equal geometric scales."""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parents[1]
d=np.genfromtxt(root/'textbook/data/appB_circle.csv',delimiter=',',names=True)
fig,ax=plt.subplots(figsize=(5,3.4),layout='constrained')
ax.plot(d['x'],d['y'],color='#087f8c',label='Left 2 / right 6 rad/s')
ax.scatter([0],[.4],s=20,marker='+',color='#c94749',label='Centre (0, 0.4) m')
for k in [0,100,200,300]:
    ax.annotate('',xy=(d['x'][k]+.12*np.cos(d['yaw'][k]),d['y'][k]+.12*np.sin(d['yaw'][k])),
                xytext=(d['x'][k],d['y'][k]),arrowprops=dict(arrowstyle='->',color='#c94749',lw=1.3))
ax.set(xlabel='x / m',ylabel='y / m',xlim=(-.56,.56),ylim=(-.12,.92))
ax.set_aspect('equal');ax.grid(alpha=.2)
fig.legend(*ax.get_legend_handles_labels(),loc='outside lower center',ncol=2,frameon=False,fontsize=8)
fig.savefig(root/'textbook/figures/appB_circle.pdf',bbox_inches='tight',pad_inches=.1)
