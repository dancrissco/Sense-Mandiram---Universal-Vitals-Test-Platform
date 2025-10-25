#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Chest-only IMU recorder (BLE) for XIAO MG24 Sense.

- Uses the existing config.yaml -> ble.devices.chest
- Writes the SAME CSV schema as your current recorder:
    t_s, sensor, ax, ay, az, gx, gy, gz
- No other changes besides restricting to the 'chest' sensor.

Usage:
    python3 imu6_recorder_chest_only.py --duration 60 --outfile imu6_run_chest.csv
"""

import asyncio
import csv
import time
import argparse
from pathlib import Path

import yaml

# your existing client module
from ble_imu_client import ImuDevice


async def record_chest(duration_s: float, out_csv: Path):
    # Load config
    cfg = yaml.safe_load(open("config.yaml", "r"))
    ble_cfg = cfg.get("ble", {})
    dev_cfg = ble_cfg.get("devices", {}).get("chest", {})

    dev = ImuDevice(
        dev_cfg.get("name", "MG24-CHEST"),
        dev_cfg.get("service_uuid", "c0de0001-7e6d-4b5c-9a10-0000feedcafe"),
        dev_cfg.get("char_uuid",    "c0de0002-7e6d-4b5c-9a10-0000feedcafe"),
        "chest",
        address=dev_cfg.get("address"),
        adapter=ble_cfg.get("adapter", "default"),
        packet_format=ble_cfg.get("packet_format", "auto15"),
    )

    print("[chest] connecting…")
    await dev.connect()
    print("[chest] connected")

    out_csv.parent.mkdir(parents=True, exist_ok=True)
    f = open(out_csv, "w", newline="")
    w = csv.writer(f)
    # CSV schema unchanged:
    w.writerow(["t_s", "sensor", "ax", "ay", "az", "gx", "gy", "gz"])

    t0 = time.time()
    next_rate = t0 + 1.0
    pkt_count = 0

    try:
        while (time.time() - t0) < duration_s:
            batch = await dev.read_batch(1024)
            if not batch:
                await asyncio.sleep(0.01)
                continue

            for pkt in batch:
                ts = pkt.get("ts_ns", 0) * 1e-9
                ax, ay, az = pkt.get("acc", (0, 0, 0))
                gx, gy, gz = pkt.get("gyr", (0, 0, 0))
                # EXACT same row format:
                w.writerow([ts, "chest", ax, ay, az, gx, gy, gz])
                pkt_count += 1

            if time.time() >= next_rate:
                print(f"[chest] ~1s packets: {pkt_count}")
                pkt_count = 0
                next_rate = time.time() + 1.0
    finally:
        try:
            f.close()
        except Exception:
            pass
        try:
            await dev.disconnect()
        except Exception:
            pass

    print(f"[chest] done. CSV saved: {out_csv}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=60.0, help="seconds to record")
    ap.add_argument("--outfile", type=str, default="imu6_run_chest.csv", help="output CSV path")
    args = ap.parse_args()

    out_csv = Path(args.outfile)
    asyncio.run(record_chest(args.duration, out_csv))


if __name__ == "__main__":
    main()
