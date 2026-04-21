#!/usr/bin/env python3
"""Plot closed-loop roll torque impulse response from impulse_response.csv."""

import argparse
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

def main():
    parser = argparse.ArgumentParser(description="Plot quadrotor impulse response")
    parser.add_argument("csv", nargs="?", default="impulse_response.csv")
    args = parser.parse_args()

    try:
        data = np.genfromtxt(args.csv, delimiter=",", names=True)
    except Exception as e:
        sys.exit(f"Error reading {args.csv}: {e}")

    t           = data["t_s"]
    roll_deg    = data["roll_deg"]
    pitch_deg   = data["pitch_deg"]
    roll_rate   = data["roll_rate_dps"]
    pitch_rate  = data["pitch_rate_dps"]
    pos_z       = data["pos_z_m"]
    dist_nm     = data["disturbance_nm"]

    impulse_t = 1.0  # known impulse time

    fig = plt.figure(figsize=(11, 9))
    fig.suptitle("Closed-Loop Roll Torque Impulse Response\n"
                 "(0.1 N·m / 10 ms impulse at t = 1 s, I_x = 0.0035 kg·m²)",
                 fontsize=13, fontweight="bold")
    gs = gridspec.GridSpec(4, 1, hspace=0.50)

    def add_impulse_line(ax):
        ax.axvline(impulse_t, color="gray", linestyle=":", linewidth=0.9)

    ax1 = fig.add_subplot(gs[0])
    ax1.plot(t, roll_deg,  label="Roll",  color="tab:blue")
    ax1.plot(t, pitch_deg, label="Pitch", color="tab:orange")
    ax1.set_ylabel("Angle (deg)")
    ax1.legend(loc="upper right", fontsize=8)
    ax1.grid(True, alpha=0.3)
    add_impulse_line(ax1)

    ax2 = fig.add_subplot(gs[1])
    ax2.plot(t, roll_rate,  label="Roll rate",  color="tab:blue")
    ax2.plot(t, pitch_rate, label="Pitch rate", color="tab:orange")
    ax2.set_ylabel("Rate (deg/s)")
    ax2.legend(loc="upper right", fontsize=8)
    ax2.grid(True, alpha=0.3)
    add_impulse_line(ax2)

    ax3 = fig.add_subplot(gs[2])
    ax3.plot(t, -pos_z, color="tab:green")  # flip NED z → altitude above ground
    ax3.set_ylabel("Altitude (m)")
    ax3.grid(True, alpha=0.3)
    add_impulse_line(ax3)

    ax4 = fig.add_subplot(gs[3])
    ax4.plot(t, dist_nm, color="tab:purple")
    ax4.set_ylabel("Disturbance (N·m)")
    ax4.set_xlabel("Time (s)")
    ax4.grid(True, alpha=0.3)
    add_impulse_line(ax4)

    for ax in [ax1, ax2, ax3]:
        ax.set_xticklabels([])

    out = args.csv.replace(".csv", ".png")
    plt.savefig(out, dpi=150, bbox_inches="tight")
    print(f"Saved {out}")
    plt.show()

if __name__ == "__main__":
    main()
