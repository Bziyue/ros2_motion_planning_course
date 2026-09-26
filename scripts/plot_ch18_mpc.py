#!/usr/bin/env python3
"""Measured MPC/PD comparison: geometry uses equal axes, costs are not safety proofs."""
from pathlib import Path
import shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parents[1]
d=np.genfromtxt(root/'tmp/ch18_mpc/samples.csv',delimiter=',',names=True)
shutil.copyfile(root/'tmp/ch18_mpc/summary.csv',root/'textbook/data/ch18_mpc.csv')
shutil.copyfile(root/'tmp/ch18_mpc/failures.csv',root/'textbook/data/ch18_mpc_failures.csv')
fig,axes=plt.subplots(1,2,figsize=(8.3,3.3),layout='constrained')
s=d[d['mode']==6];axes[0].plot(s['x_ref'],s['y_ref'],color='#183447',ls='--',lw=1,label='Reference')
for mode,label,color in [(5,'PD, 0.35 N','#c94749'),(6,'MPC N=20, 0.35 N','#087f8c')]:
 s=d[d['mode']==mode];axes[0].plot(s['x'],s['y'],label=label,color=color,lw=1.2)
 axes[1].plot(s['t'],s['error'],color=color,lw=1.2)
axes[0].set_aspect('equal');axes[0].set(xlabel='x / m',ylabel='y / m');axes[1].set(xlabel='t / s',ylabel='Position error / m')
for a in axes:a.grid(alpha=.2)
fig.legend(*axes[0].get_legend_handles_labels(),loc='outside lower center',ncol=3,frameon=False)
fig.savefig(root/'textbook/figures/ch18_mpc.pdf',bbox_inches='tight',pad_inches=.12)
