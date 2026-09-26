#!/usr/bin/env python3
"""Data from the C++ ESDF demo; equal metric axes and separate interpolation error."""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parents[1]
source = root/'tmp/ch12_esdf'
fig, axes = plt.subplots(1, 2, figsize=(8, 3.6), layout='constrained')
d = np.genfromtxt(source/'field_10.csv', delimiter=',', names=True)
x = d['x'].reshape(97,97); y = d['y'].reshape(97,97); z = d['distance'].reshape(97,97)
h = axes[0].imshow(z, origin='lower', interpolation='nearest',
    extent=(x.min()-.015,x.max()+.015,y.min()-.015,y.max()+.015), cmap='RdBu',vmin=-1,vmax=1)
axes[0].contour(x,y,z,levels=[0],colors=['#17384b'],linewidths=1.2)
axes[0].quiver(x[::8,::8],y[::8,::8],d['gx'].reshape(97,97)[::8,::8],d['gy'].reshape(97,97)[::8,::8],
               color='#17384b',scale=25,width=.003)
axes[0].set(xlabel='x / m', ylabel='y / m', aspect='equal',title='Signed centre field, resolution 0.10 m')
fig.colorbar(h,ax=axes[0],label='Interpolated distance / m',shrink=.82)
for resolution in [20,10,5]:
    data = np.genfromtxt(source/f'field_{resolution}.csv', delimiter=',', names=True)
    row = data.reshape(97,97)[48]
    axes[1].plot(row['x'],row['distance']-row['analytic'],label=f'{resolution/100:.2f} m')
axes[1].axhline(0,color='#87969e',lw=.7)
axes[1].set(xlabel='x / m',ylabel='Field minus analytic circle / m',title='A slice at y = 0.011 m')
axes[1].grid(alpha=.2)
handles, labels = axes[1].get_legend_handles_labels()
fig.legend(handles, labels, loc='outside lower center', ncol=3, fontsize=9)
fig.savefig(root/'textbook/figures/ch12_esdf.pdf',bbox_inches='tight',pad_inches=.12)
fig.savefig(root/'textbook/figures/ch12_esdf.png',dpi=160,bbox_inches='tight',pad_inches=.12)
