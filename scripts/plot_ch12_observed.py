#!/usr/bin/env python3
"""Plot actual ROS distance and gradient messages, without a graphics-driver dependency."""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

root = Path(__file__).resolve().parents[1]
m = np.load(root/'tmp/ch12_observed.npz'); a=m['cloud']; r=float(m['resolution']); ox,oy=m['origin']
fig,ax=plt.subplots(figsize=(5,4),layout='constrained')
im=ax.imshow(a[:,:,3],origin='lower',extent=(ox,ox+a.shape[1]*r,oy,oy+a.shape[0]*r),
             interpolation='nearest',cmap='RdBu',vmin=-.5,vmax=2.)
starts,ends=m['starts'],m['ends']; delta=ends-starts
segments=[]
for start,end,vector in zip(starts,ends,delta):
    length=np.linalg.norm(vector); direction=vector/length
    normal=np.array([-direction[1],direction[0]]); head=min(.06,.3*length)
    segments.extend([(start,end),(end,end-head*direction+.4*head*normal),
                     (end,end-head*direction-.4*head*normal)])
ax.add_collection(LineCollection(segments,colors='#27353a',linewidths=.65))
ax.set(xlim=(-10.5,1.5),ylim=(-10.5,1.5),aspect='equal',xlabel='x / m',ylabel='y / m',title='Observed map at 0.5 s')
fig.colorbar(im,ax=ax,label='Signed centre distance / m',shrink=.85)
fig.savefig(root/'textbook/figures/ch12_observed.pdf',bbox_inches='tight',pad_inches=.12)
fig.savefig(root/'textbook/figures/ch12_observed.png',dpi=160,bbox_inches='tight',pad_inches=.12)
