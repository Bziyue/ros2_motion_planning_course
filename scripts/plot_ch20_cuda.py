#!/usr/bin/env python3
"""Warm, resident scanner timing; kernel and complete scan are distinct curves."""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
values=np.genfromtxt('textbook/data/ch20_cuda.csv',delimiter=',',names=True)
fig,axes=plt.subplots(1,2,figsize=(9.2,3.7),layout='constrained')
for ax,count in zip(axes,[16,512]):
    rows=values[values['obstacles']==count]
    ax.loglog(rows['beams'],rows['cpu_p50_ms'],'o-',label='CPU scan',color='#c65039')
    ax.loglog(rows['beams'],rows['gpu_scan_p50_ms'],'s-',label='GPU scan + copy',color='#007f8b')
    ax.loglog(rows['beams'],rows['kernel_p50_ms'],'^--',label='GPU kernel',color='#705ca5')
    ax.set_title(f'{count} obstacles');ax.set_xlabel('Beams');ax.set_ylabel('P50 time [ms]');ax.grid(which='both',alpha=.15)
handles,labels=axes[0].get_legend_handles_labels();fig.legend(handles,labels,loc='outside lower center',ncol=3,frameon=False)
fig.savefig('textbook/figures/ch20_cuda.pdf')
