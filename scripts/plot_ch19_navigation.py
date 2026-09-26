#!/usr/bin/env python3
"""Plot measured navigation paths and errors, without claiming a GUI screenshot."""
import argparse
import csv
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser=argparse.ArgumentParser();parser.add_argument('input',nargs='?',default='tmp/ch19_batch');args=parser.parse_args();root=Path(args.input)
    fig,axes=plt.subplots(1,3,figsize=(10.4,3.4),layout='constrained');colors={'A':'#007f8b','B':'#c65039','C':'#705ca5'}
    for ax,seed in zip(axes,[42,43,44]):
        for route in ['A','B','C']:
            data=np.genfromtxt(root/f'{route}_{seed}'/'samples.csv',delimiter=',',names=True)
            ax.plot(data['truth_x'],data['truth_y'],color=colors[route],lw=1.3,label=route)
        ax.plot(-8,-8,'o',color='#223944',ms=4);ax.plot(-4,-4,'*',color='#223944',ms=9)
        ax.set_title(f'Seed {seed}');ax.set_xlabel('World x [m]');ax.set_ylabel('World y [m]');ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.2)
    handles,labels=axes[0].get_legend_handles_labels();fig.legend(handles,labels,loc='outside lower center',ncol=3,frameon=False)
    fig.savefig('textbook/figures/ch19_navigation.pdf');plt.close(fig)


if __name__=='__main__':main()
