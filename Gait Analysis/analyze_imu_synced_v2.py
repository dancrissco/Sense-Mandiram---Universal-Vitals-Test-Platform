#!/usr/bin/env python3
"""
analyze_imu_synced_v2.py
Robust multi-IMU analyzer for XIAO MG24 Sense logs.

Auto-detects common column names:
- time:  'timestamp','time','t','t_s','secs','seconds','ms','millis','milliseconds'
- device:'device','sensor','name','id','label','mac','addr'

Aligns all IMUs on a shared *overlap* time window so traces run together.
Computes/plots angular-velocity magnitude and a smoothed cadence trend,
and prints cadence (steps/min) per device.

Example:
  python3 analyze_imu_synced_v2.py --imu_csv imu6_run.csv --fs 50 --out_png imu_synced.png
"""

import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt

# -------- helpers --------
TIME_CANDIDATES   = ("timestamp","time","t","t_s","secs","seconds","ms","millis","milliseconds")
DEVICE_CANDIDATES = ("device","sensor","name","id","label","mac","addr")

def find_col(df, candidates):
    for c in df.columns:
        if c.lower() in candidates:
            return c
    return None

def standardize_time(series, fs):
    s = pd.to_numeric(series, errors="coerce")
    s = s.dropna()
    if len(s) < 3:
        # synthesize if unusable
        return None, "synth"
    rng = float(s.max() - s.min())
    # Heuristics: if it looks like milliseconds range, convert
    if rng > 1e5:
        t = (series - series.iloc[0]) / 1000.0
        unit = "ms->s"
    else:
        t = series - series.iloc[0]
        unit = "s"
    return t.astype(float), unit

def butter_lowpass_filter(x, cutoff, fs, order=3):
    b, a = butter(order, cutoff/(0.5*fs), btype="low")
    return filtfilt(b, a, x)

def ang_mag(gx, gy, gz):
    return np.sqrt(gx*gx + gy*gy + gz*gz)

def estimate_cadence(y, fs):
    if len(y) < 5:
        return np.nan
    # crude peak count: negative-going zero of 1st-derivative's sign change
    peaks = (np.diff(np.sign(np.diff(y))) < 0).sum()
    dur = len(y) / float(fs)
    return peaks * (60.0 / dur) if dur > 0 else np.nan

# -------- main --------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--imu_csv", required=True, help="Path to CSV")
    ap.add_argument("--fs", type=float, default=50.0, help="Sampling rate (Hz), used for resampling")
    ap.add_argument("--cutoff", type=float, default=3.0, help="LPF cutoff (Hz) for cadence trend")
    ap.add_argument("--out_png", default=None, help="Optional path to save plot")
    args = ap.parse_args()

    df = pd.read_csv(args.imu_csv)
    # detect essential columns
    time_col   = find_col(df, TIME_CANDIDATES)
    device_col = find_col(df, DEVICE_CANDIDATES)

    # gyro columns (exact names expected for values)
    for col in ("gx","gy","gz"):
        if col not in df.columns:
            raise ValueError(f"Required column '{col}' not found in CSV.")

    if device_col is None:
        raise ValueError(f"No device column found. Looked for: {DEVICE_CANDIDATES}")
    if time_col is None:
        # we can still proceed by synthesizing per-device time
        print("Warning: No explicit time column found; synthesizing per-device time from row order.")
    else:
        print(f"Using time column: '{time_col}'")
    print(f"Using device column: '{device_col}'")

    # clean numeric gyros
    for col in ("gx","gy","gz"):
        df[col] = pd.to_numeric(df[col], errors="coerce")

    # build per-device frames with their own time (normalized) and global clock if present
    devices = sorted(df[device_col].astype(str).unique().tolist())
    per_dev = {}
    for dev in devices:
        g = df[df[device_col].astype(str) == dev].copy()
        g = g.dropna(subset=["gx","gy","gz"])
        # time handling
        if time_col is not None:
            g[time_col] = pd.to_numeric(g[time_col], errors="coerce")
            g = g.dropna(subset=[time_col])
            g["t_global"] = g[time_col].astype(float)
            # normalize per-device
            t_norm, unit = standardize_time(g[time_col], args.fs)
            if t_norm is None:
                # synthesize if bad
                n = len(g)
                g["t_norm"] = np.arange(n)/float(args.fs)
                print(f"Device {dev}: synthesized time (bad/short time column).")
            else:
                g["t_norm"] = t_norm.values
        else:
            # no time column at all → synthesize
            n = len(g)
            g["t_global"] = np.nan
            g["t_norm"] = np.arange(n)/float(args.fs)
        per_dev[dev] = g.reset_index(drop=True)

    # choose alignment mode: prefer *global overlap* if we have global time for all devices; else use normalized overlap
    have_global = all(per_dev[d]["t_global"].notna().any() for d in devices)
    mode = "global" if have_global else "normalized"
    print(f"Alignment mode: {mode}")

    if mode == "global":
        # compute overlapping window using global time
        starts = [gd["t_global"].iloc[0] for gd in per_dev.values()]
        ends   = [gd["t_global"].iloc[-1] for gd in per_dev.values()]
        t0 = max(starts)
        t1 = min(ends)
        if not np.isfinite(t0) or not np.isfinite(t1) or t1 <= t0:
            # fallback to normalized
            mode = "normalized"
            print("Global overlap invalid; falling back to normalized alignment.")
        else:
            t_common = np.arange(t0, t1, 1.0/args.fs)

    if mode == "normalized":
        # overlap on normalized (start-at-0) time
        starts = [gd["t_norm"].iloc[0] for gd in per_dev.values()]
        ends   = [gd["t_norm"].iloc[-1] for gd in per_dev.values()]
        t0 = max(starts)
        t1 = min(ends)
        if not np.isfinite(t0) or not np.isfinite(t1) or t1 <= t0:
            raise RuntimeError("Could not find a valid overlapping duration across devices.")
        t_common = np.arange(t0, t1, 1.0/args.fs)

    # plot + cadence
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    cmap = plt.cm.get_cmap("tab10", max(len(devices), 10))
    cadence = {}

    for i, dev in enumerate(devices):
        gd = per_dev[dev]
        # choose the time vector based on mode
        t_src = gd["t_global"].values if mode == "global" else gd["t_norm"].values
        y_mag = ang_mag(gd["gx"].values, gd["gy"].values, gd["gz"].values)
        # robust interpolation (guard against tiny monotonic issues)
        order = np.argsort(t_src)
        t_src = t_src[order]
        y_mag = y_mag[order]

        # deduplicate times (np.interp requires ascending unique x)
        uniq_mask = np.concatenate(([True], np.diff(t_src) > 0))
        t_src = t_src[uniq_mask]
        y_mag = y_mag[uniq_mask]

        y_i = np.interp(t_common, t_src, y_mag)
        try:
            y_s = butter_lowpass_filter(y_i, args.cutoff, args.fs)
        except Exception:
            k = max(3, int(args.fs//2))
            y_s = np.convolve(y_i, np.ones(k)/k, mode="same")

        ax1.plot(t_common, y_i, alpha=0.4, color=cmap(i), label=f"{dev} raw")
        ax2.plot(t_common, y_s, lw=2, color=cmap(i), label=f"{dev} smooth")

        cadence[dev] = round(estimate_cadence(y_s, args.fs), 1)

    ax1.set_title("Instantaneous Angular Velocity Magnitude (rad/s)")
    ax1.set_ylabel("rad/s")
    ax1.grid(True, alpha=0.3)
    ax1.legend(ncol=2)

    ax2.set_title("Smoothed Rhythm (Cadence Trend)")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("filtered magnitude")
    ax2.grid(True, alpha=0.3)
    ax2.legend(ncol=2)

    plt.tight_layout()

    print("\nCadence summary (steps/min):")
    for dev in devices:
        print(f"  {dev}: {cadence.get(dev, np.nan)} spm")

    # save/show
    if args.out_png:
        plt.savefig(args.out_png, dpi=150)
        print(f"\nSaved → {args.out_png}")
    else:
        plt.show()

if __name__ == "__main__":
    main()
