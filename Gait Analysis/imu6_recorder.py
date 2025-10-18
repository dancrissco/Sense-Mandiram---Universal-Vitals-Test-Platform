# imu6_recorder.py — record 6 IMUs (ankles, wrists, chest, neck) to one CSV
import asyncio, time, csv, tkinter as tk
from tkinter import ttk, filedialog
import yaml
from collections import defaultdict

from ble_imu_client import ImuDevice  # your existing decoder (packet_format: auto15)

DEFAULT_DURATION_S = 10.0
SENSORS_ORDER = ("ankle_L","ankle_R","wrist_L","wrist_R","chest","neck")

def load_cfg(path="config.yaml"):
    with open(path, "r") as f: return yaml.safe_load(f)

class IMU6RecorderApp:
    def __init__(self, root):
        self.root = root
        self.root.title("IMU Recorder (6 sensors)")
        self.loop_running = False

        top = ttk.Frame(root); top.pack(side=tk.TOP, fill=tk.X, padx=8, pady=8)
        ttk.Label(top, text="Duration (s):").pack(side=tk.LEFT)
        self.duration_var = tk.StringVar(value=str(DEFAULT_DURATION_S))
        ttk.Entry(top, textvariable=self.duration_var, width=7).pack(side=tk.LEFT, padx=6)

        self.btn = ttk.Button(top, text="Start recording", command=self.start)
        self.btn.pack(side=tk.LEFT, padx=8)

        self.status = ttk.Label(root, text="Idle")
        self.status.pack(anchor="w", padx=8, pady=4)

        self.rates_lbl = ttk.Label(root, text="")
        self.rates_lbl.pack(anchor="w", padx=8)

        self.progress = ttk.Progressbar(root, mode="determinate", length=360)
        self.progress.pack(fill="x", padx=8, pady=8)

    def start(self):
        if self.loop_running: return
        path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV","*.csv")],
            initialfile="imu6_run.csv",
            title="Save IMU CSV"
        )
        if not path: return
        try:
            dur = float(self.duration_var.get())
        except:
            dur = DEFAULT_DURATION_S

        self.loop_running = True
        self.btn.config(state="disabled")
        self.status.config(text=f"Recording to {path} for {dur:.1f}s …")
        # run the async recorder without blocking Tk
        self.root.after(50, lambda: asyncio.run(self._record(path, dur)))

    async def _record(self, out_path, duration_s):
        cfg = load_cfg()
        adapter = cfg["ble"].get("adapter","hci0")
        pformat = cfg["ble"].get("packet_format","auto15")
        devs_cfg = cfg["ble"]["devices"]

        # Build device list in a fixed order (skips any missing in config)
        want = []
        for loc in SENSORS_ORDER:
            if loc in devs_cfg:
                d = devs_cfg[loc]
                want.append(ImuDevice(
                    d["name"], d.get("service_uuid",""), d["char_uuid"], loc,
                    address=d.get("address"), adapter=adapter, packet_format=pformat
                ))

        devices = []
        # Connect sequentially (more reliable for BLE)
        for dev in want:
            try:
                self.status.config(text=f"Connecting {dev.loc} ({dev.name}) …")
                self._tk_pump()
                await dev.connect()
                devices.append(dev)
                self.status.config(text=f"✔ Connected: {dev.loc}")
                self._tk_pump()
                await asyncio.sleep(0.3)
            except Exception as e:
                self.status.config(text=f"✖ Skipped {dev.loc}: {e}")
                self._tk_pump()

        if not devices:
            self.status.config(text="No devices connected. Aborting.")
            self.btn.config(state="normal")
            self.loop_running = False
            return

        # Open CSV and stream for duration
        t0 = time.time()
        next_rate = t0 + 1.0
        counts = defaultdict(int)

        with open(out_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_s","sensor","ax","ay","az","gx","gy","gz"])  # SI units
            while True:
                now = time.time()
                # stop condition
                if now - t0 >= duration_s:
                    break

                # read batches
                for dev in devices:
                    batch = await dev.read_batch(1024)
                    if batch:
                        counts[dev.loc] += len(batch)
                        for pkt in batch:
                            ts = pkt["ts_ns"] * 1e-9
                            ax, ay, az = pkt["acc"]
                            gx, gy, gz = pkt["gyr"]
                            w.writerow([ts, dev.loc, ax, ay, az, gx, gy, gz])

                # update UI roughly every 50 ms
                self.progress["maximum"] = duration_s
                self.progress["value"] = now - t0
                if now >= next_rate:
                    # show per-sensor packet rate (Hz approx)
                    rate_txt = "  ".join(f"{k}:{v}" for k,v in counts.items())
                    self.rates_lbl.config(text=f"Rates (Hz, approx): {rate_txt}")
                    counts = defaultdict(int)
                    next_rate = now + 1.0
                self._tk_pump()
                await asyncio.sleep(0.01)

        # cleanup
        for dev in devices:
            try: await dev.disconnect()
            except: pass

        self.status.config(text=f"Done: {out_path}")
        self.btn.config(state="normal")
        self.loop_running = False

    def _tk_pump(self):
        # keep the UI responsive
        try:
            self.root.update_idletasks()
            self.root.update()
        except tk.TclError:
            pass

if __name__ == "__main__":
    root = tk.Tk()
    IMU6RecorderApp(root)
    root.mainloop()
