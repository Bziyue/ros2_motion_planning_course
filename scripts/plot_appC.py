#!/usr/bin/env python3
"""Measured standalone gyro bias path, independent of the white-noise stream."""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parents[1]
d=np.genfromtxt(root/'textbook/data/appC_bias_path.csv',delimiter=',',names=True)
fig,ax=plt.subplots(figsize=(8,2.7),layout='constrained')
ax.plot(d['t'],d['measurement'],color='#adbcc6',lw=.6,label='Bias + white noise')
ax.plot(d['t'],d['bias'],color='#087f8c',lw=1.5,label='Persistent bias')
ax.axhline(0,color='#263f50',lw=.6);ax.grid(alpha=.2)
ax.set(xlabel='t / s',ylabel='Gyro error / (rad/s)',xlim=(0,10))
fig.legend(*ax.get_legend_handles_labels(),loc='outside lower center',ncol=2,frameon=False)
fig.savefig(root/'textbook/figures/appC_bias.pdf',bbox_inches='tight',pad_inches=.1)
