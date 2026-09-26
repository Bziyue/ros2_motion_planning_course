#!/usr/bin/env python3
"""Plot the actual ROS message snapshot saved by check_ch11.py."""
import json
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap

root = Path(__file__).resolve().parents[1]
m = json.loads((root / "tmp/ch11_observed.json").read_text())
w, h, r = m["width"], m["height"], m["resolution"]
ox, oy = m["origin"]; extent = (ox, ox+w*r, oy, oy+h*r)
raw = np.array(m["observed"]).reshape(h, w)
mask = np.array(m["blocked"]).reshape(h, w)
fig, axes = plt.subplots(1, 2, figsize=(8, 3.7), layout="constrained")
shown = np.where(raw < 0, 1, np.where(raw <= 35, 0, 2))
axes[0].imshow(shown, origin="lower", extent=extent, interpolation="nearest",
               cmap=ListedColormap(["white", "#b9c4c9", "#17384b"]), vmin=0, vmax=2)
axes[1].imshow(mask, origin="lower", extent=extent, interpolation="nearest",
               cmap=ListedColormap(["white", "#bfd5dc"]), vmin=0, vmax=100)
for ax, title in zip(axes, ["Observed map: unknown stays gray", "Disk centre: certified free cells"]):
    x, y = np.array(m["path"]).T
    ax.plot(x, y, "o-", color="#df3546", lw=2, ms=4)
    ax.set(xlim=(-10.5, -4), ylim=(-10.5, -4), xlabel="x / m", ylabel="y / m", title=title)
    ax.set_aspect("equal")
fig.savefig(root / "textbook/figures/ch11_observed.pdf", bbox_inches="tight", pad_inches=.12)
fig.savefig(root / "textbook/figures/ch11_observed.png", dpi=160, bbox_inches="tight", pad_inches=.12)
