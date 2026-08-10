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
import math
import time
from typing import Optional

import numpy as np

from .ble_client import PrintheadBLE
from .calibration import PageCalibration
from .config import BleSettings, NozzleMapSettings, TrackingSettings
from .geometry import (
    BLANK_FRAME,
    IMAGE_HEIGHT,
    NOZZLE_MODE_LINE,
    SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM,
    SENSOR_TO_NOZZLE_COL_MM,
)
from .nozzle_map import remap_rows
from .rendering import frames_from_ink
from .tracking import _AXIS_INDEX, PageMapper, PositionFilter, make_tracker


# ============================================================================
# --pos : live Amfitrack position readout
# ============================================================================
async def monitor_position(tracking: TrackingSettings, simulate: bool,
                           hz: float = 15.0, ndjson: bool = False,
                           page_calibration_path: Optional[str] = None,
                           sensor_offset_row_mm: float = SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM,
                           sensor_offset_col_mm: float = SENSOR_TO_NOZZLE_COL_MM,
                           boresight_deg: float = 0.0) -> None:
    """Continuously print the sensor position (x/y/z), the travel-axis value and
    the resulting column, until Ctrl+C. Doubles as an axis / mm-per-column aid.

    ``ndjson=True`` prints one newline-terminated JSON object per sample instead
    of the live single-line readout, so tools (the web UI) can parse the stream.

    ``page_calibration_path``, if given, loads a ``PageCalibration`` (see
    ``calibration.py``) and additionally reports the live page-plane
    ``(page_u, page_v, page_z)`` for that calibration -- lets a page-mode
    calibration be sanity-checked (known hand motion -> plausible u/v) before
    anything is printed with it. A bad/missing path aborts the same way a
    failed tracker connection does, since the caller asked for it explicitly.

    ``sensor_offset_row_mm``/``sensor_offset_col_mm``/``boresight_deg`` are
    forwarded into the same :class:`~printhead.tracking.PageMapper` a real
    pass builds (see ``PrintController._print_freehand_pass``), so this
    diagnostic reports the exact same ``(page_u, page_v)`` -- and, when a
    boresight has been captured, the exact same live yaw -- a real pass
    would use. Reporting the live yaw (``yaw_deg`` below) is exactly what
    lets a boresight be verified before printing with it: held in the
    reference pose (nozzle bar along the traced row edge), ``yaw_deg``
    should read close to 0. ``roll_deg``/``pitch_deg`` (see
    ``rotation.cart_rotation_angles``) are reported alongside it for the
    same live-monitoring purpose, but are diagnostic only -- unlike yaw,
    neither feeds any position correction (see that function's docstring)."""
    tracker = make_tracker(tracking, simulate)
    try:
        tracker.open()
    except Exception as exc:
        if ndjson:
            print(json.dumps({"event": "error", "message": str(exc)}), flush=True)
        else:
            print(f"Cannot open Amfitrack tracker: {exc}")
        return

    page_mapper = None
    if tracking.page_frame == "simple":
        # Calibration-free frame (see PageCalibration.simple_frame): page
        # axes = tracker x/y, yaw about tracker z. Unlike a print pass, the
        # origin is NOT zeroed here -- this diagnostic is for watching raw
        # tracker-frame u/v/yaw, and re-zeroing to wherever --pos happened to
        # start would only obscure that. page_u/page_v therefore read as
        # absolute tracker x/y (plus the sensor->nozzle offset).
        page_mapper = PageMapper(PageCalibration.simple_frame(),
                                 sensor_offset_row_mm=sensor_offset_row_mm,
                                 sensor_offset_col_mm=sensor_offset_col_mm,
                                 boresight_offset_rad=math.radians(boresight_deg))
    elif page_calibration_path is not None:
        try:
            page_mapper = PageMapper(PageCalibration.load(page_calibration_path),
                                     sensor_offset_row_mm=sensor_offset_row_mm,
                                     sensor_offset_col_mm=sensor_offset_col_mm,
                                     boresight_offset_rad=math.radians(boresight_deg))
        except Exception as exc:
            if ndjson:
                print(json.dumps({"event": "error",
                                  "message": f"Cannot load page calibration: {exc}"}),
                      flush=True)
            else:
                print(f"Cannot load page calibration '{page_calibration_path}': {exc}")
            tracker.close()
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
            pos, quat = tracker.read_pose()
            if pos is not None:
                pos = pos_filter.update(pos, time.monotonic())
                if origin is None:
                    origin = pos.copy()
                advance = tracking.axis_sign * float(pos[axis] - origin[axis])
                col = int(round(advance / tracking.mm_per_column))
                # quat (qx,qy,qz,qw) -- see AmfitrackTracker._extract_pose -- is
                # included only when the connected hardware/SDK actually reports it,
                # so this line looks exactly as before on setups that don't.
                # page_mapper.project() applies the same rotation correction
                # (or lack of it -- see PageMapper.project's docstring) as a
                # real freehand pass, and caches this sample's yaw on
                # page_mapper.last_yaw_rad as a side effect (read below) --
                # see controller._print_freehand_pass for the identical
                # "compute once, reuse" pattern.
                page_uvz = page_mapper.project(pos, quat) if page_mapper is not None else None
                yaw_deg = (math.degrees(page_mapper.last_yaw_rad)
                          if page_mapper is not None else None)
                # Diagnostic-only tilt readout (see rotation.cart_rotation_angles /
                # tracking.PageMapper.project) -- reported alongside yaw_deg but,
                # like it, never fed back into any correction; last_roll_rad/
                # last_pitch_rad are valid floats (0.0 default) whenever a
                # page_mapper is active at all, same as last_yaw_rad.
                roll_deg = (math.degrees(page_mapper.last_roll_rad)
                           if page_mapper is not None else None)
                pitch_deg = (math.degrees(page_mapper.last_pitch_rad)
                            if page_mapper is not None else None)
                if ndjson:
                    event = {
                        "event": "position",
                        "x": round(float(pos[0]), 3), "y": round(float(pos[1]), 3),
                        "z": round(float(pos[2]), 3),
                        "advance": round(advance, 3), "col": col}
                    if quat is not None:
                        event.update(qx=round(float(quat[0]), 4), qy=round(float(quat[1]), 4),
                                     qz=round(float(quat[2]), 4), qw=round(float(quat[3]), 4))
                    if page_uvz is not None:
                        event.update(page_u=round(page_uvz[0], 3),
                                     page_v=round(page_uvz[1], 3),
                                     page_z=round(page_uvz[2], 3),
                                     yaw_deg=round(yaw_deg, 3),
                                     roll_deg=round(roll_deg, 3),
                                     pitch_deg=round(pitch_deg, 3))
                    print(json.dumps(event), flush=True)
                else:
                    line = (f"x={pos[0]:9.2f}  y={pos[1]:9.2f}  z={pos[2]:9.2f} mm  |  "
                           f"advance={advance:9.2f} mm  |  col={col:5d}")
                    if quat is not None:
                        line += (f"  |  quat=[{quat[0]:+.2f} {quat[1]:+.2f} "
                                f"{quat[2]:+.2f} {quat[3]:+.2f}]")
                    if page_uvz is not None:
                        line += (f"  |  page u={page_uvz[0]:8.2f}  v={page_uvz[1]:8.2f}  "
                                f"z={page_uvz[2]:6.2f} mm  yaw={yaw_deg:+6.2f} deg  "
                                f"roll={roll_deg:+6.2f} deg  pitch={pitch_deg:+6.2f} deg")
                    print(line, end="\r", flush=True)
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


def _print_start_button_hint() -> None:
    """
    Firmware only drains its BLE receive FIFO into the nozzle output queue
    while ``process_running == 1`` -- and that flag is set *exclusively* by
    a physical button press in the firmware's ``mainloop()`` (see
    ``main.c``: ``button_poll_press_event()`` -> ``i2s_parallel_start()``).
    BLE writes from here always succeed and land in the FIFO regardless, so
    without the button this command reports success while physically
    nothing fires and 0 mA flows -- confirmed against a real serial log.
    There is no BLE-visible signal wired up to gate on here (see the
    module docstring / README for why), so a message that cannot be missed
    is the fix.
    """
    print("=" * 72)
    print("IMPORTANT: the printhead only fires while its print process is")
    print("running, and that is only ever started by a PHYSICAL PRESS of the")
    print("START button on the device itself -- this command cannot do that")
    print("for you. Press and hold the device's START button now, for the")
    print("duration of this test, or nothing will physically happen even")
    print("though every BLE write below reports success.")
    print("=" * 72)


# ============================================================================
# --nozzle-test : fire a diagnostic pattern on the cartridge
# ============================================================================
async def nozzle_test(ble: BleSettings, nozzle_map: Optional[NozzleMapSettings] = None,
                      on_seconds: float = 2.0, sweep_step: float = 0.02) -> None:
    """All nozzles on briefly, then a single nozzle swept down all 152 rows.

    If ``nozzle_map`` is given, it is applied first, so the sweep lets you
    visually confirm a block remap fixes the physical firing order."""
    all_on_ink = np.ones((IMAGE_HEIGHT, 1), dtype=bool)
    sweep_ink = np.eye(IMAGE_HEIGHT, dtype=bool)      # 152 single-nozzle frames
    if nozzle_map is not None and nozzle_map.block_size:
        all_on_ink = remap_rows(all_on_ink, nozzle_map.block_size, nozzle_map.order)
        sweep_ink = remap_rows(sweep_ink, nozzle_map.block_size, nozzle_map.order)
    all_on = frames_from_ink(all_on_ink)[0]
    sweep = frames_from_ink(sweep_ink)

    try:
        async with PrintheadBLE(ble) as client:
            # This tool bypasses _run_ble(), so nothing else pins the firmware to
            # line mode here. If it is still in page mode from an earlier --mode
            # page run, the "all on" write below would not fire 3 times like line
            # mode intends -- it becomes a held pattern re-fired every
            # PATTERN_STRIDE ticks, i.e. ~120 times over on_seconds=2.0s, dumping
            # ~40x the intended ink through all nozzles at once. required=False:
            # this must still run against older firmware without MODE_UUID, where
            # line mode is the only behaviour anyway.
            await client.set_print_mode(NOZZLE_MODE_LINE, required=False)
            _print_start_button_hint()
            print(f"All {IMAGE_HEIGHT} nozzles ON for {on_seconds:.1f}s ...")
            await client.write_column(all_on)
            await asyncio.sleep(on_seconds)

            print("Sweeping a single nozzle down all rows ...")
            for frame in sweep:
                await client.write_column(frame)
                await asyncio.sleep(sweep_step)
            await client.write_blank()
        # Not "done" / success -- the frames were sent over BLE, that's all
        # this can confirm from here. If nothing visibly fired or no current
        # was drawn, the START button was most likely not pressed/held.
        print("Nozzle test: all frames sent. If nothing fired and no current "
              "was drawn, the physical START button on the device was most "
              "likely not pressed (or not held) throughout the test.")
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
            # Measure in a known mode, so the reported cols/s means something.
            await client.set_print_mode(NOZZLE_MODE_LINE, required=False)
            _print_start_button_hint()
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
