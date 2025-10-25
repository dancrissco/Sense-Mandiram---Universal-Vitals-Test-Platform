#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Convert an IMU CSV (t_s, ax, ay, az) into a Fall Index plot image.
Usage:
    python3 csv_to_fall_plot.py imu6_run31.csv
Outputs:
    imu6_run31_plot.png
"""

import sys, csv
import numpy as np
from collections import deque
from pathlib import Path
import matplotlib.pyplot as plt

# --- Parameters ---
MA_SEC     = 1.5      # moving-mean (gravity removal)
RMS_SEC    = 0.25     # RMS window for fall index
THRESH_RMS = 2.0      # adjustable threshold (m/s^2)

def moving_mean(values, times, win_sec):
    out, dq_v, dq_t = [], deque(), deque(); acc = 0.0
    for v, t in zip(values, times):
        dq_v.append(v); dq_t.append(t); acc += v
        while dq_t and (t - dq_t[0]) > win_sec:
            acc -= dq_v.popleft(); dq_t.popleft()
        out.append(acc / max(1, len(dq_v)))
    return out

def rolling_rms(values, times, win_sec):
    out, dq_v, dq_t = [], deque(), deque(); acc2 = 0.0
    for v, t in zip(values, times):
        dq_v.append(v); dq_t.append(t); acc2 += v*v
        while dq_t and (t - dq_t[0]) > win_sec:
            vv = dq_v.popleft(); dq_t.popleft(); acc2 -= vv*vv
        val = acc2 / max(1, len(dq_v))
        if val < 0.0: val = 0.0   # guard small negatives
        out.append(val**0.5)
    return out

def accel_mag(ax, ay, az): return float(np.sqrt(ax*ax + ay*ay + az*az))

def compute_fall_index(ts, ax, ay, az):
    mag = [accel_mag(x, y, z) for x, y, z in zip(ax, ay, az)]
    ghat = moving_mean(mag, ts, MA_SEC)
    dyn  = [m - g for m, g in zip(mag, ghat)]
    fi   = rolling_rms(dyn, ts, RMS_SEC)
    return ts, fi

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 csv_to_fall_plot.py imu6_runXX.csv")
        sys.exit(1)

    csv_path = Path(sys.argv[1])
    if not csv_path.exists():
        print("File not found:", csv_path)
        sys.exit(1)

    ts, ax, ay, az = [], [], [], []
    with open(csv_path, "r") as f:
        r = csv.DictReader(f)
        for row in r:
            try:
                ts.append(float(row["t_s"]))
                ax.append(float(row["ax"]))
                ay.append(float(row["ay"]))
                az.append(float(row["az"]))
            except Exception:
                continue

    if not ts:
        print("No data in file.")
        sys.exit(1)

    t0 = ts[0]
    ts = [t - t0 for t in ts]  # relative time
    ts, fi = compute_fall_index(ts, ax, ay, az)

    # --- Plot ---
    plt.figure(figsize=(7, 4))
    plt.plot(ts, fi, lw=1.6, label="Fall Index (RMS of dynamic |a|)")
    plt.axhline(THRESH_RMS, color="red", linestyle="--", alpha=0.7,
                label=f"Threshold = {THRESH_RMS}")
    plt.xlabel("Time (s)")
    plt.ylabel("Fall Index (m/s²)")
    plt.title(f"Offline Fall Index — {csv_path.name}")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()

    out_png = csv_path.with_name(csv_path.stem + "_plot.png")
    plt.savefig(out_png, bbox_inches="tight")
    print("Saved:", out_png)

if __name__ == "__main__":
    main()
