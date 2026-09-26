#!/usr/bin/env python3
"""Display the synthetic-pose EKF experiment, separate from lidar validation."""
from pathlib import Path
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

source = Path(sys.argv[1] if len(sys.argv) > 1 else 'tmp/ch09_filter')
target = Path(sys.argv[2] if len(sys.argv) > 2 else 'textbook/figures')
target.mkdir(parents=True, exist_ok=True)
data = np.genfromtxt(source / 'ch09_filter_trace.csv', delimiter=',', names=True)
fig = plt.figure(figsize=(8, 5.5), constrained_layout=True)
gs = fig.add_gridspec(2, 2)
error = fig.add_subplot(gs[0, :])
for field, label, color in [('imu_error', 'IMU only', '#c66530'),
                            ('zero_bias_error', 'Bias fixed at zero', '#578690'),
                            ('estimated_bias_error', 'Bias estimated', '#12334a')]:
    error.semilogy(data['time'], data[field], label=label, color=color)
error.axvspan(4, 5, color='gray', alpha=.15, label='Pose dropout')
error.set(xlabel='Time / s', ylabel='Position error / m', title='Synthetic pose measurements (not lidar)')
error.legend(ncol=2, fontsize=8)
accel = fig.add_subplot(gs[1, 0])
for field, target_value, color in [('bax', .08, '#087f8c'), ('bay', -.04, '#c66530')]:
    accel.plot(data['time'], data[field], color=color, label=field)
    accel.axhline(target_value, color=color, linestyle='--', linewidth=.8)
accel.set(xlabel='Time / s', ylabel='Accel. bias / (m/s²)')
accel.legend(fontsize=8)
gyro = fig.add_subplot(gs[1, 1])
gyro.plot(data['time'], data['bg'], color='#12334a', label='Estimated bg')
gyro.axhline(.01, color='black', linestyle='--', linewidth=.8, label='Injected bias (evaluation)')
gyro.set(xlabel='Time / s', ylabel='Gyro bias / (rad/s)')
gyro.legend(fontsize=8)
for ax in (error, accel, gyro):
    ax.grid(alpha=.2)
fig.savefig(target / 'ch09_filter.pdf')
fig.savefig(target / 'ch09_filter.png', dpi=180)
