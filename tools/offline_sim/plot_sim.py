"""画离线仿真结果: python3 plot_sim.py static.csv onlane.csv dynamic.csv -o sim.png"""
import argparse

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd

ap = argparse.ArgumentParser()
ap.add_argument("csv", nargs="+")
ap.add_argument("-o", "--out", default="sim.png")
args = ap.parse_args()

fig, axes = plt.subplots(len(args.csv), 2, figsize=(14, 3.6 * len(args.csv)),
                         gridspec_kw={"width_ratios": [2.2, 1]}, squeeze=False)
for row, path in enumerate(args.csv):
    d = pd.read_csv(path)
    ax = axes[row][0]
    ax.plot(d.x, d.y, color="#e08a00", lw=2, label="main car")
    for i, c in zip((1, 2, 3), ("#2b5fd9", "#7a3fd1", "#16a085")):
        ax.plot(d[f"o{i}x"], d[f"o{i}y"], color=c, lw=1, ls="--")
        ax.scatter(d[f"o{i}x"].iloc[0], d[f"o{i}y"].iloc[0], color=c, s=25, label=f"obs{i} start")
    # 每2s标一个主车位置
    for _, r in d.iloc[::20].iterrows():
        ax.annotate(f"{r.t:.0f}", (r.x, r.y), fontsize=6, color="#555")
    ax.set_title(f"{path}: XY")
    ax.set_aspect("equal", adjustable="datalim")
    ax.legend(fontsize=7, loc="best")
    ax.grid(alpha=0.3)

    ax2 = axes[row][1]
    ax2.plot(d.t, d.v, label="v (m/s)")
    ax2.plot(d.t, d.l, label="l (m)")
    ax2.plot(d.t, d.a, label="a (m/s²)", alpha=0.6)
    ax2.set_xlabel("t (s)")
    ax2.set_title("speed / lateral offset")
    ax2.legend(fontsize=7)
    ax2.grid(alpha=0.3)

fig.tight_layout()
fig.savefig(args.out, dpi=110)
print("saved", args.out)
