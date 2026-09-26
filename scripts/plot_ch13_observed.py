#!/usr/bin/env python3
"""Plot the actual ROS snapshot; outlines correspond to published Marker vertices."""
import json
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import numpy as np
root=Path(__file__).resolve().parents[1]
m=json.loads((root/'tmp/ch13_observed.json').read_text())
w,h,r=m['width'],m['height'],m['resolution']; ox,oy=m['origin']
fig,ax=plt.subplots(figsize=(6.4,4.3),layout='constrained')
ax.imshow(np.array(m['blocked']).reshape(h,w),origin='lower',extent=(ox,ox+w*r,oy,oy+h*r),
          cmap=ListedColormap(['white','#c7d6dc']),vmin=0,vmax=100,interpolation='nearest')
colors=['#0d80a6','#d9660d','#26994d','#9940a6']
for i,vertices in enumerate(m['polygons']):
    p=np.array(vertices); ax.fill(p[:,0],p[:,1],color=colors[i%4],alpha=.09)
    p=np.vstack([p,p[0]]); ax.plot(p[:,0],p[:,1],color=colors[i%4],lw=1.3)
x,y=np.array(m['raw']).T; ax.plot(x,y,'--',color='#677d87',lw=1,label='Raw A*')
x,y=np.array(m['route']).T; ax.plot(x,y,'o-',color='#ce334a',ms=4,lw=1.6,label='Assigned route')
ax.set(xlim=(-9.5,-1.5),ylim=(-9.5,.5),xlabel='x / m',ylabel='y / m',title='Observed map: 73 A* points, 4 safe regions')
ax.set_aspect('equal')
fig.legend(*ax.get_legend_handles_labels(),loc='outside lower center',ncol=2,frameon=False)
for ext in ('pdf','png'):
    fig.savefig(root/f'textbook/figures/ch13_observed.{ext}',dpi=160,bbox_inches='tight',pad_inches=.12)
