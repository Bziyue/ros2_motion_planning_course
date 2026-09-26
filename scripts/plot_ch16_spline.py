#!/usr/bin/env python3
"""Measured construction + energy + energy-gradient, fixed identical problems."""
from pathlib import Path
import shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parents[1]
d=np.genfromtxt(root/'tmp/ch16_upstream/comparison.csv',delimiter=',',names=True)
shutil.copyfile(root/'tmp/ch16_upstream/comparison.csv',root/'textbook/data/ch16_comparison.csv')
fig,ax=plt.subplots(figsize=(7.8,2.8),layout='constrained');x=np.arange(len(d));w=.23
for k,key,label,color in [(0,'minco_us','Course MINCO','#a87751'),(1,'spline_us','Course Spline2D','#087f8c'),(2,'upstream_us','Pinned upstream 2D','#183447')]:
 ax.bar(x+(k-1)*w,d[key],w,label=label,color=color)
ax.set_xticks(x,[str(int(v)) for v in d['segments']]);ax.set(xlabel='Number of segments',ylabel='Median batch time / µs',yscale='log');ax.grid(axis='y',alpha=.2)
fig.legend(*ax.get_legend_handles_labels(),loc='outside lower center',ncol=3,frameon=False)
fig.savefig(root/'textbook/figures/ch16_comparison.pdf',bbox_inches='tight',pad_inches=.12)
