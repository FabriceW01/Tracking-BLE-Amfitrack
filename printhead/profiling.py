"""
Real-time timing profiler
==========================

Instruments a position-based print pass to answer one question: *are the
columns being sent (and delivered) fast enough to keep up with the moving
head?* If not, the print becomes speed-dependent even though the column is
chosen from the measured position — because the BLE link / firmware cannot
deliver columns as fast as the head crosses them.

The profiler records, per column write:
  * the BLE write latency (how long ``await write_column`` took), and
  * the head speed at that moment (mm/s) -> the *demanded* column rate.

From those it derives the sustained BLE column rate and the maximum head speed
at which columns still keep up, and prints a verdict at the end of the pass.

Note: without per-frame firmware feedback we cannot prove a column was
physically *printed* on time; the write latency + backlog are the best proxy.
Pair this with ``--ble-benchmark`` (which uses write-*with-response* to measure
the true GATT round-trip) for the delivery-latency side.
"""

from __future__ import annotations

import time
from typing import List, Optional

import numpy as np


class PassProfiler:
    def __init__(self, mm_per_column: float, live: bool = True,
                 csv_path: Optional[str] = None, live_every_s: float = 0.5):
        self.mm_per_column = mm_per_column
        self.live = live
        self.csv_path = csv_path
        self.live_every_s = live_every_s

        self.n_cols = 0
        self.total_write_time = 0.0
        self.write_latencies: List[float] = []
        self.speeds: List[float] = []
        self._t0 = 0.0
        self._t_end = 0.0
        self._last_live = 0.0
        self._csv = None

    def start(self) -> None:
        self._t0 = time.perf_counter()
        self._last_live = self._t0
        if self.csv_path:
            try:
                self._csv = open(self.csv_path, "w")
                self._csv.write("t_s,column,advance_mm,write_latency_ms,speed_mm_s\n")
            except OSError as exc:
                print(f"[profile] cannot open CSV {self.csv_path!r}: {exc}")
                self._csv = None

    def record_write(self, column: int, advance_mm: float, latency_s: float,
                     speed_mm_s: Optional[float]) -> None:
        self.n_cols += 1
        self.total_write_time += latency_s
        self.write_latencies.append(latency_s)
        if speed_mm_s is not None:
            self.speeds.append(speed_mm_s)

        if self._csv is not None:
            t = time.perf_counter() - self._t0
            self._csv.write(f"{t:.4f},{column},{advance_mm:.3f},"
                            f"{latency_s * 1000:.3f},{speed_mm_s or 0.0:.2f}\n")

        if self.live:
            now = time.perf_counter()
            if now - self._last_live >= self.live_every_s:
                self._last_live = now
                self._print_live(speed_mm_s, latency_s)

    def _sustained_rate(self) -> float:
        """Columns per second the BLE writes actually sustained."""
        return self.n_cols / self.total_write_time if self.total_write_time else 0.0

    def _print_live(self, speed: Optional[float], latency: float) -> None:
        speed = speed or 0.0
        demand = speed / self.mm_per_column if self.mm_per_column else 0.0
        sustained = self._sustained_rate()
        load = demand / sustained if sustained else 0.0
        flag = "   <-- BLE can't keep up" if load > 1.0 else ""
        print(f"[profile] v={speed:6.1f} mm/s  demand={demand:6.0f} cols/s  "
              f"ble~{sustained:6.0f} cols/s  wlat={latency * 1000:5.1f} ms  "
              f"load={load:4.2f}{flag}", flush=True)

    def finish(self) -> None:
        self._t_end = time.perf_counter()
        if self._csv is not None:
            self._csv.close()
            self._csv = None
        self._report()

    def _report(self) -> None:
        if not self.write_latencies:
            print("[profile] no column writes recorded.")
            return
        dur = max(1e-9, self._t_end - self._t0)
        lat_ms = np.array(self.write_latencies) * 1000.0
        sustained = self._sustained_rate()
        max_safe_speed = sustained * self.mm_per_column
        peak_speed = max(self.speeds) if self.speeds else 0.0
        peak_demand = peak_speed / self.mm_per_column if self.mm_per_column else 0.0

        print("---- timing profile ----")
        print(f"  pass duration      : {dur:6.2f} s")
        print(f"  columns written    : {self.n_cols}  "
              f"(avg {self.n_cols / dur:.1f} cols/s output)")
        print(f"  BLE write latency  : avg {lat_ms.mean():.1f} ms  "
              f"p95 {np.percentile(lat_ms, 95):.1f} ms  max {lat_ms.max():.1f} ms")
        print(f"  BLE sustained rate : ~{sustained:.0f} cols/s  -> keeps up to "
              f"~{max_safe_speed:.1f} mm/s at {self.mm_per_column:.3f} mm/col")
        print(f"  peak head speed    : {peak_speed:.1f} mm/s  "
              f"(demand {peak_demand:.0f} cols/s)")
        if peak_demand > sustained * 1.05:
            print(f"  VERDICT: the head outran the BLE link -> columns lagged, so "
                  f"the print depends on speed.\n"
                  f"           Keep speed below ~{max_safe_speed:.1f} mm/s, increase "
                  f"--mm-per-column, or speed up BLE (connection interval / MTU).")
        else:
            print("  VERDICT: BLE kept up with the head at the observed speeds.")
