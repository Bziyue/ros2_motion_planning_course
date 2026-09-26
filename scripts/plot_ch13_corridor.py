#!/usr/bin/env python3
"""Equal-scale configuration cells, overlapping regions and assigned waypoint segments."""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.colors import ListedColormap

root=Path(__file__).resolve().parents[1]; source=root/'tmp/ch13_corridor'
grid=np.genfromtxt(source/'grid.csv',delimiter=',',names=True)
astar=np.genfromtxt(source/'astar.csv',delimiter=',',names=True)
fig,axes=plt.subplots(1,2,figsize=(9,3.8),layout='constrained')
for mode,ax in enumerate(axes):
    ax.imshow(grid['blocked'].reshape(41,61),origin='lower',extent=(0,6.1,0,4.1),
              interpolation='nearest',cmap=ListedColormap(['white','#243f50']),vmin=0,vmax=1)
    vertices=np.genfromtxt(source/f'regions_{mode}.csv',delimiter=',',names=True)
    for i in np.unique(vertices['region']):
        group=vertices[vertices['region']==i]; color=plt.get_cmap('tab10')(int(i)%10)
        ax.add_patch(Polygon(np.column_stack([group['x'],group['y']]),facecolor=color,edgecolor=color,alpha=.22,lw=1))
    waypoints=np.genfromtxt(source/f'route_{mode}.csv',delimiter=',',names=True)
    ax.plot(astar['x'],astar['y'],color='#66757f',lw=.9,ls='--',label='A* path')
    ax.plot(waypoints['x'],waypoints['y'],'o-',color='#c9344d',lw=1.4,ms=3,label='Assigned segments')
    ax.set(xlabel='x / m',ylabel='y / m',aspect='equal',title=['Local rectangles','Validated convex hulls'][mode])
handles,labels=axes[0].get_legend_handles_labels();fig.legend(handles,labels,loc='outside lower center',ncol=2)
fig.savefig(root/'textbook/figures/ch13_corridor.pdf',bbox_inches='tight',pad_inches=.12)
fig.savefig(root/'textbook/figures/ch13_corridor.png',dpi=160,bbox_inches='tight',pad_inches=.12)
