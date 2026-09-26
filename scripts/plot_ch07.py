#!/usr/bin/env python3
"""Plot mapping_demo data with physical axes; never feeds geometry into mapping.

Usage: python3 scripts/plot_ch07.py tmp/ch07_eval textbook/figures
Dependencies: host python3-matplotlib and python3-numpy.
"""
import argparse
import csv
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm
import numpy as np


def read_grid(path):
    """PGM stores probability percentages; 255 is unknown, row zero is minimum y."""
    return np.asarray(plt.imread(path), dtype=np.int16)


def show_grid(ax, path, extent):
    values = read_grid(path)
    classes = np.full(values.shape, 3)
    classes[(values >= 0) & (values <= 35)] = 0
    classes[(values > 35) & (values < 65)] = 1
    classes[(values >= 65) & (values <= 100)] = 2
    colors = ListedColormap(["#f6fafb", "#80bcc2", "#183447", "#b6c5ca"])
    ax.imshow(classes, origin="lower", extent=extent, interpolation="nearest",
              cmap=colors, norm=BoundaryNorm([-.5, .5, 1.5, 2.5, 3.5], 4))
    ax.set_xlabel("x / m")
    ax.set_ylabel("y / m")
    ax.spines[["top", "right"]].set_visible(False)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("data", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({"font.size": 12, "axes.titlesize": 13, "axes.labelsize": 12,
                         "savefig.dpi": 180, "font.family": "DejaVu Sans"})
    extent = [-11, 11, -11, 11]
    fig, axes = plt.subplots(1, 2, figsize=(10, 4.8), layout="constrained")
    for ax, name, title in zip(axes, ("truth.pgm", "map_100_0.pgm"),
                              ("Geometry reference (evaluation only)", "Observed map (truth pose)")):
        show_grid(ax, args.data / name, extent)
        ax.set_title(title)
        t = np.linspace(0, 15.9, 160)
        ax.plot(-8 + np.sin(.4*t), -7 - np.cos(.4*t), color="#df6737", linewidth=1.2)
        ax.set_xticks([-10, 0, 10]); ax.set_yticks([-10, 0, 10])
    fig.savefig(args.output / "ch07_truth_observed.png")
    plt.close(fig)

    fig, axes = plt.subplots(2, 3, figsize=(10, 7), layout="constrained")
    for row, noise in enumerate((0, 50)):
        for col, resolution in enumerate((50, 100, 200)):
            ax = axes[row, col]
            show_grid(ax, args.data / f"map_{resolution}_{noise}.pgm", extent)
            ax.set_xlim(-10.5, -2); ax.set_ylim(-10.5, -2)
            ax.set_xticks([-10, -6, -2]); ax.set_yticks([-10, -6, -2])
            ax.set_title(rf"$\rho={resolution/1000:g}$ m, $\sigma={noise/1000:g}$ m")
    fig.savefig(args.output / "ch07_resolution.png")
    plt.close(fig)

    with (args.data / "ch07_wall.csv").open() as stream:
        rows = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(stream)]
    fig, axes = plt.subplots(1, 2, figsize=(10, 4.6), layout="constrained")
    for resolution, color in zip((.05, .1, .2), ("#087f8c", "#df6737", "#526db1")):
        selected = [r for r in rows if abs(r["resolution"]-resolution) < 1e-9]
        axes[0].plot([r["sigma"] for r in selected], [r["mean_width"] for r in selected],
                     "o-", label=rf"$\rho={resolution:g}$ m", color=color)
    axes[0].set_xlabel("Range noise sigma / m")
    axes[0].set_ylabel("Mean occupied span / m")
    axes[0].set_xticks([0, .05, .1])
    axes[0].set_ylim(bottom=0)
    axes[0].grid(alpha=.2)
    axes[0].legend()
    axes[0].set_title("160 scans, fixed wall at x = 4.03 m")
    show_grid(axes[1], args.data / "wall_50_100.pgm", [-5, 5, -5, 5])
    axes[1].set_xlim(3.4, 4.6); axes[1].set_ylim(-1, 1)
    axes[1].axvline(4.03, color="#df6737", linestyle="--", linewidth=1.5)
    axes[1].set_title(r"$\rho=0.05$ m, $\sigma=0.10$ m")
    fig.savefig(args.output / "ch07_wall.png")
    plt.close(fig)
    print("Wrote three ch07 figures; gray unknown, white free, teal uncertain, navy occupied.")


if __name__ == "__main__":
    main()
