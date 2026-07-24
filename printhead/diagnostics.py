"""
Debug / diagnostics
===================

Standalone bring-up checks, each wired to its own CLI flag. They reuse the
normal building blocks (tracker, framing, BLE client) but run independently of
a print pass: connect, report/act, then exit. Every check degrades gracefully
with a friendly message when the hardware or a vendor library is missing.
"""

from __future__ import annotations

import asyncio
import json
import time
from typing import Optional

import numpy as np

from .ble_client import PrintheadBLE
from .config import BleSettings, NozzleMapSettings, TrackingSettings
from .geometry import BLANK_FRAME, IMAGE_HEIGHT
from .nozzle_map import remap_rows
from .rendering import frames_from_ink
from .tracking import _AXIS_INDEX, PositionFilter, make_tracker


# ============================================================================
# --pos : live Amfitrack position readout
# ============================================================================
async def monitor_position(tracking: TrackingSettings, simulate: bool,
                           hz: float = 15.0, ndjson: bool = False) -> None:
    """Continuously print the sensor position (x/y/z), the travel-axis value and
    the resulting column, until Ctrl+C. Doubles as an axis / mm-per-column aid.

    ``ndjson=True`` prints one newline-terminated JSON object per sample instead
    of the live single-line readout, so tools (the web UI) can parse the stream."""
    tracker = make_tracker(tracking, simulate)
    try:
        tracker.open()
    except Exception as exc:
        if ndjson:
            print(json.dumps({"event": "error", "message": str(exc)}), flush=True)
        else:
            print(f"Cannot open Amfitrack tracker: {exc}")
        return

    axis = _AXIS_INDEX[tracking.advance_axis]
    origin = None
    pos_filter = PositionFilter(tracking.smooth_ms / 1000.0)
    if ndjson:
        print(json.dumps({"event": "connected", "axis": tracking.advance_axis,
                          "mm_per_column": tracking.mm_per_column}), flush=True)
    else:
        print(f"Live Amfitrack position (axis '{tracking.advance_axis}', "
              f"{tracking.mm_per_column:.3f} mm/col). Ctrl+C to stop.")
    try:
        while True:
            pos = tracker.read_position()
            if pos is not None:
                pos = pos_filter.update(pos, time.monotonic())
                if origin is None:
                    origin = pos.copy()
                advance = tracking.axis_sign * float(pos[axis] - origin[axis])
                col = int(round(advance / tracking.mm_per_column))
                if ndjson:
                    print(json.dumps({
                        "event": "position",
                        "x": round(float(pos[0]), 3), "y": round(float(pos[1]), 3),
                        "z": round(float(pos[2]), 3),
                        "advance": round(advance, 3), "col": col}), flush=True)
                else:
                    print(f"x={pos[0]:9.2f}  y={pos[1]:9.2f}  z={pos[2]:9.2f} mm  |  "
                          f"advance={advance:9.2f} mm  |  col={col:5d}",
                          end="\r", flush=True)
            await asyncio.sleep(1.0 / hz)
    except (KeyboardInterrupt, asyncio.CancelledError):
        pass
    finally:
        if not ndjson:
            print()                   # leave the live line intact
        tracker.close()
        if ndjson:
            print(json.dumps({"event": "stopped"}), flush=True)
        else:
            print("Stopped position monitor.")


# ============================================================================
# --list-nodes : enumerate Amfitrack USB nodes
# ============================================================================
def list_nodes(tracking: TrackingSettings) -> None:
    """Connect to the dongle and list every node so the 'Sensor' match is visible."""
    try:
        import amfiprot
    except ImportError:
        print("amfiprot is not installed (pip install amfiprot amfiprot-amfitrack).")
        return

    s = tracking
    try:
        conn = amfiprot.USBConnection(s.vendor_id, s.product_id)
    except Exception:
        try:
            conn = amfiprot.USBConnection(s.vendor_id, s.product_id_source)
        except Exception as exc:
            print(f"Cannot open USB dongle "
                  f"(vendor 0x{s.vendor_id:04X}): {exc}")
            return

    try:
        nodes = conn.find_nodes()
        print(f"Found {len(nodes)} node(s):")
        for node in nodes:
            name = getattr(node, "name", "?")
            marker = "  <- sensor" if "Sensor" in str(name) else ""
            print(f"  name={name!r}  uuid={getattr(node, 'uuid', '?')}  "
                  f"tx_id={getattr(node, 'tx_id', '?')}{marker}")
        if not any("Sensor" in str(getattr(n, 'name', '')) for n in nodes):
            print("No node name contains 'Sensor' -> the tracker would find none.")
    finally:
        for method in ("stop", "close"):
            try:
                getattr(conn, method)()
            except Exception:
                pass


# ============================================================================
# --scan-ble : list nearby BLE devices
# ============================================================================
async def scan_ble(ble: BleSettings) -> None:
    """Scan and print BLE devices (address + name) to find the printhead."""
    try:
        from bleak import BleakScanner
    except ImportError:
        print("bleak is not installed (pip install bleak).")
        return

    print(f"Scanning BLE for {ble.scan_timeout:.0f}s ...")
    try:
        devices = await BleakScanner.discover(timeout=ble.scan_timeout)
    except Exception as exc:
        print(f"BLE scan failed: {exc}")
        return

    if not devices:
        print("No BLE devices found.")
        return
    for dev in devices:
        name = dev.name or "(no name)"
        marker = "  <- printhead" if dev.name == ble.device_name else ""
        print(f"  {dev.address}  {name}{marker}")


# ============================================================================
# --nozzle-test : fire a diagnostic pattern on the cartridge
# ============================================================================
async def nozzle_test(ble: BleSettings, nozzle_map: Optional[NozzleMapSettings] = None,
                      on_seconds: float = 2.0, sweep_step: float = 0.02) -> None:
    """All nozzles on briefly, then a single nozzle swept down all 164 rows.

    If ``nozzle_map`` is given, it is applied first, so the sweep lets you
    visually confirm a block remap fixes the physical firing order."""
    all_on_ink = np.ones((IMAGE_HEIGHT, 1), dtype=bool)
    sweep_ink = np.eye(IMAGE_HEIGHT, dtype=bool)      # 164 single-nozzle frames
    if nozzle_map is not None and nozzle_map.block_size:
        all_on_ink = remap_rows(all_on_ink, nozzle_map.block_size, nozzle_map.order)
        sweep_ink = remap_rows(sweep_ink, nozzle_map.block_size, nozzle_map.order)
    all_on = frames_from_ink(all_on_ink)[0]
    sweep = frames_from_ink(sweep_ink)

    try:
        async with PrintheadBLE(ble) as client:
            print(f"All {IMAGE_HEIGHT} nozzles ON for {on_seconds:.1f}s ...")
            await client.write_column(all_on)
            await asyncio.sleep(on_seconds)

            print("Sweeping a single nozzle down all rows ...")
            for frame in sweep:
                await client.write_column(frame)
                await asyncio.sleep(sweep_step)
            await client.write_blank()
        print("Nozzle test done.")
    except Exception as exc:
        print(f"Nozzle test failed (BLE): {exc}")


# ============================================================================
# --ble-benchmark : measure the BLE column throughput / latency ceiling
# ============================================================================
async def ble_benchmark(ble: BleSettings, tracking: TrackingSettings,
                        n_fast: int = 400, n_probe: int = 60) -> None:
    """
    Measure how fast columns can actually be pushed over BLE. This is the ceiling
    that makes position printing speed-dependent: if the head crosses columns
    faster than this, they lag no matter how good the position is.

      * throughput: ``n_fast`` write-without-response frames as fast as possible.
      * latency:    ``n_probe`` write-*with-response* frames -> true GATT
        round-trip (~ the connection interval), i.e. real delivery latency.

    Blank frames are used so nothing is actually printed.
    """
    loop = asyncio.get_event_loop()
    mmpc = tracking.mm_per_column
    try:
        async with PrintheadBLE(ble) as client:
            print(f"Throughput: sending {n_fast} frames (no response) ...")
            t0 = loop.time()
            for _ in range(n_fast):
                await client.write_column(BLANK_FRAME)
            dt = loop.time() - t0
            thr = n_fast / dt if dt > 0 else 0.0

            print(f"Latency: {n_probe} frames (with response) ...")
            lat = []
            for _ in range(n_probe):
                t = loop.time()
                await client.write_column(BLANK_FRAME, response=True)
                lat.append((loop.time() - t) * 1000.0)
            await client.write_blank()

            lat.sort()
            avg = sum(lat) / len(lat)
            p95 = lat[min(len(lat) - 1, int(0.95 * len(lat)))]
            max_speed = thr * mmpc

            print("---- BLE benchmark ----")
            print(f"  no-response throughput : {thr:.0f} cols/s "
                  f"({1000.0 / thr:.1f} ms/col)" if thr else "  throughput: n/a")
            print(f"  with-response latency  : avg {avg:.1f} ms  "
                  f"p95 {p95:.1f} ms  max {lat[-1]:.1f} ms")
            print(f"  => at {mmpc:.3f} mm/col, columns keep up to ~{max_speed:.1f} "
                  f"mm/s. Above that, position printing will lag / depend on speed.")
    except Exception as exc:
        print(f"BLE benchmark failed: {exc}")
