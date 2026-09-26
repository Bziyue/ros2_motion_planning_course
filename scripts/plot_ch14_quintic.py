#!/usr/bin/env python3
"""Plot actual C++ polynomial samples and duration scaling; no ROS timing implied."""
from pathlib import Path
import shutil
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
root=Path(__file__).resolve().parents[1]
data=np.genfromtxt(root/'tmp/ch14_quintic/samples.csv',delimiter=',',names=True)
shutil.copyfile(root/'tmp/ch14_quintic/summary.csv',root/'textbook/data/ch14_quintic.csv')
fig,axes=plt.subplots(1,3,figsize=(8.3,2.6),layout='constrained')
for case,color in [(0,'#087f8c'),(1,'#d86629')]:
    d=data[data['case']==case]
    for ax,key,label in zip(axes,['x','vx','ax'],['Position / m','Velocity / (m/s)','Acceleration / (m/s²)']):
        ax.plot(d['t'],d[key],color=color,label=f'T = {int(d["t"][-1])} s')
        ax.set(xlabel='t / s',ylabel=label);ax.grid(alpha=.2)
fig.legend(*axes[0].get_legend_handles_labels(),loc='outside lower center',ncol=2,frameon=False)
for ext in ['pdf','png']:fig.savefig(root/f'textbook/figures/ch14_scaling.{ext}',dpi=160,bbox_inches='tight',pad_inches=.12)
d=data[data['case']==2]
fig,axes=plt.subplots(1,3,figsize=(8.3,2.6),layout='constrained')
axes[0].plot(d['x'],d['y'],color='#087f8c');axes[0].plot([0,1,2],[0,1,0],'o',color='#d86629')
axes[0].set(xlabel='x / m',ylabel='y / m',title='C² curve');axes[0].set_aspect('equal');axes[0].grid(alpha=.2)
for ax,keys,units in zip(axes[1:],[['vx','vy'],['ax','ay']],['m/s','m/s²']):
    for key,color in zip(keys,['#087f8c','#d86629']):ax.plot(d['t'],d[key],color=color,label=key[-1])
    ax.axvline(4,color='#526c76',ls=':',lw=.8);ax.set(xlabel='t / s',ylabel=units);ax.grid(alpha=.2)
fig.legend(*axes[1].get_legend_handles_labels(),loc='outside lower center',ncol=2,frameon=False)
for ext in ['pdf','png']:fig.savefig(root/f'textbook/figures/ch14_join.{ext}',dpi=160,bbox_inches='tight',pad_inches=.12)
