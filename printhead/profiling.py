"""
Real-time timing profiler
==========================

Instruments a print pass to answer one question: *is the BLE link keeping up
with the moving head?* If not, the print becomes speed-dependent even though
the column/pattern is chosen from the measured position -- because the BLE
link / firmware cannot deliver updates as fast as the head moves.

Two modes, both on the same ``PassProfiler`` (extended, not a separate
class, so the CSV/live-printing/CLI plumbing is shared):

  * ``"line"`` (default) -- one column write per crossing. Records, per
    write: the BLE write latency (``await write_column`` timed directly at
    the call site) and the head speed -> the *demanded* column rate. Derives
    the *sustained* rate from the measured latencies.
  * ``"page"`` -- freehand mode has no per-write latency to time at the call
    site: ``PatternSender.send()`` hands a pattern to a background task and
    returns immediately (that asynchrony is the whole point of "latest
    wins", see ``pattern_sender.py``). So instead of measuring sustained
    throughput directly, page mode records how often the pattern actually
    changes (``CoverageEngine.step()`` reporting ``changed=True``) and
    compares that *demanded update rate* against a known BLE throughput
    ceiling (``ble_write_ceiling``, the same physical write-without-response
    limit as line mode, measured with ``--ble-benchmark``).

Note: without per-frame firmware feedback we cannot prove a column was
physically *printed* on time; the write latency + backlog (line) or update
rate + ceiling (page) are the best proxy. Pair this with ``--ble-benchmark``
(which uses write-*with-response* to measure the true GATT round-trip) for
the delivery-latency side.
"""

from __future__ import annotations

import time
from typing import List, Optional

import numpy as np

# Empirical BLE write-without-response ceiling (see diagnostics.ble_benchmark),
# used as page mode's throughput reference since PatternSender's background
# sends are not individually timed at the call site the way line mode's writes
# are. Override with the real number from --ble-benchmark on your hardware.
DEFAULT_BLE_WRITE_CEILING_PER_S = 270.0


class PassProfiler:
    def __init__(self, mm_per_column: float, live: bool = True,
                 csv_path: Optional[str] = None, live_every_s: float = 0.5,
                 mode: str = "line",
                 ble_write_ceiling: float = DEFAULT_BLE_WRITE_CEILING_PER_S):
        self.mm_per_column = mm_per_column
        self.live = live
        self.csv_path = csv_path
        self.live_every_s = live_every_s
        self.mode = mode
        self.ble_write_ceiling = ble_write_ceiling

        self.n_cols = 0
        self.total_write_time = 0.0
        self.write_latencies: List[float] = []
        self.speeds: List[float] = []
        self._t0 = 0.0
        self._t_end = 0.0
        self._last_live = 0.0
        self._csv = None

        # page mode only
        self.page_events = 0
        self.page_speeds: List[float] = []
        self.page_write_rates: List[float] = []
        self._page_last_t: Optional[float] = None

    def start(self) -> None:
        self._t0 = time.perf_counter()
        self._last_live = self._t0
        if self.csv_path:
            try:
                self._csv = open(self.csv_path, "w")
                if self.mode == "page":
                    # qx,qy,qz,qw: raw orientation quaternion, logged purely for
                    # offline correlation -- investigating whether cart rotation,
                    # combined with the fixed sensor->nozzle-bar lever arm
                    # (geometry.SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM, see also
                    # PageMapper), explains observed freehand misalignment.
                    # Not read back or used to correct anything live yet.
                    self._csv.write(
                        "t_s,row,col,u_mm,v_mm,speed_mm_s,writes_per_s,qx,qy,qz,qw\n")
                else:
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

    def record_page_sample(self, u_mm: float, v_mm: float,
                           speed_mm_s: Optional[float],
                           quat: Optional[np.ndarray] = None) -> None:
        """
        Log one page-mode pattern update. Call this only when
        ``CoverageEngine.step()`` reports ``changed=True`` (i.e. a pattern
        was actually handed to ``PatternSender.send()``), mirroring
        ``record_write()``'s per-write-event cadence rather than logging
        every poll tick -- most ticks do not change anything.

        ``quat`` is the raw ``(qx, qy, qz, qw)`` orientation for this sample
        (see ``AmfitrackTracker.read_pose``), or ``None`` when no orientation
        packet was drained this tick -- logged as-is (not yet used for any
        live correction) so a real print pass can later be checked offline
        for cart rotation coinciding with misalignment, given the fixed
        sensor->nozzle-bar lever arm (see ``geometry.
        SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM`` / ``tracking.PageMapper``).
        """
        from .geometry import NOZZLE_PITCH_MM

        col = int(round(u_mm / self.mm_per_column)) if self.mm_per_column else 0
        row = int(round(v_mm / NOZZLE_PITCH_MM))
        now = time.perf_counter()
        writes_per_s = 1.0 / (now - self._page_last_t) if self._page_last_t else 0.0
        self._page_last_t = now

        self.page_events += 1
        if speed_mm_s is not None:
            self.page_speeds.append(speed_mm_s)
        self.page_write_rates.append(writes_per_s)

        if self._csv is not None:
            t = now - self._t0
            # Blank (not 0) when quat is None: 0,0,0,0 would look like a real
            # (degenerate) quaternion and could mislead an offline analysis.
            if quat is not None:
                quat_fields = (f"{float(quat[0]):.4f},{float(quat[1]):.4f},"
                               f"{float(quat[2]):.4f},{float(quat[3]):.4f}")
            else:
                quat_fields = ",,,"
            self._csv.write(f"{t:.4f},{row},{col},{u_mm:.3f},{v_mm:.3f},"
                            f"{speed_mm_s or 0.0:.2f},{writes_per_s:.1f},"
                            f"{quat_fields}\n")

        if self.live and now - self._last_live >= self.live_every_s:
            self._last_live = now
            self._print_live_page(speed_mm_s, writes_per_s)

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

    def _print_live_page(self, speed: Optional[float], writes_per_s: float) -> None:
        speed = speed or 0.0
        load = writes_per_s / self.ble_write_ceiling if self.ble_write_ceiling else 0.0
        flag = "   <-- pattern updates may be outrunning BLE" if load > 1.0 else ""
        print(f"[profile] v={speed:6.1f} mm/s  updates~{writes_per_s:6.1f}/s  "
              f"ceiling~{self.ble_write_ceiling:.0f}/s  load={load:4.2f}{flag}",
              flush=True)

    def finish(self) -> None:
        self._t_end = time.perf_counter()
        if self._csv is not None:
            self._csv.close()
            self._csv = None
        if self.mode == "page":
            self._report_page()
        else:
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

    def _report_page(self) -> None:
        if not self.page_write_rates:
            print("[profile] no pattern updates recorded.")
            return
        dur = max(1e-9, self._t_end - self._t0)
        rates = np.array(self.page_write_rates)
        peak_speed = max(self.page_speeds) if self.page_speeds else 0.0
        p95_rate = float(np.percentile(rates, 95))

        print("---- page-mode timing profile ----")
        print(f"  pass duration        : {dur:6.2f} s")
        print(f"  pattern updates sent : {self.page_events}  "
              f"(avg {self.page_events / dur:.1f} updates/s)")
        print(f"  update rate          : avg {rates.mean():.1f}/s  "
              f"p95 {p95_rate:.1f}/s  max {rates.max():.1f}/s")
        print(f"  peak head speed      : {peak_speed:.1f} mm/s")
        print(f"  BLE write ceiling    : ~{self.ble_write_ceiling:.0f}/s (from "
              f"--ble-benchmark; PatternSender sends are 'latest wins', so "
              f"exceeding this delays which pattern is currently live -- it "
              f"does not lose or corrupt data, unlike line mode's FIFO).")
        if p95_rate > self.ble_write_ceiling:
            print("  VERDICT: pattern updates are being requested faster than BLE "
                  "can plausibly deliver them -> the printed pattern may lag "
                  "behind the live coverage state.")
        else:
            print("  VERDICT: pattern update rate stayed within the known BLE ceiling.")
