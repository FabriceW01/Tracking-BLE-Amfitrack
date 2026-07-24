"""
Command-line interface
======================

Parses arguments into the three settings dataclasses and runs a
:class:`PrintController`. Keeps every text/render option from the original
script and adds the Amfitrack position-tracking options.
"""

from __future__ import annotations

import argparse
import asyncio
from typing import Optional

from . import patterns
from .config import BleSettings, NozzleMapSettings, RenderSettings, TrackingSettings
from .controller import PrintController
from .geometry import DEVICE_NAME, IMAGE_HEIGHT
from .nozzle_map import parse_order
from .rendering import render_text


def _auto_int(value: str) -> int:
    """Parse an int that may be given as decimal or 0x-hex (for USB ids)."""
    return int(value, 0)


def parse_args(argv=None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        prog="printhead",
        description="Render text to a 164px-tall B/W image and print it "
                    "column-by-column on an HP302 cartridge via the PrintheadBLE "
                    "ESP32, driven either by Amfitrack position or by a timer.")
    ap.add_argument("text", nargs="?",
                    help="Text to print. Alternative content sources: "
                         "--calibrate or --pattern NAME (not needed for a "
                         "--pos/--scan-ble/... debug run)")

    # --- text / render -----------------------------------------------------
    g = ap.add_argument_group("text rendering")
    g.add_argument("--font", help="Path to a .ttf font file")
    g.add_argument("--render-size", type=int, default=220,
                   help="Font pixel size for the initial render (default 220)")
    g.add_argument("--threshold", type=int, default=128,
                   help="Black/white threshold 0..255 (default 128)")
    g.add_argument("--margin", type=int, default=0,
                   help="Vertical margin in px, top+bottom (default 0)")
    g.add_argument("--invert", action="store_true",
                   help="Invert ink (print white text on black)")
    g.add_argument("--flip-y", action="store_true",
                   help="Flip vertically if the print is upside-down")
    g.add_argument("--mirror-x", action="store_true",
                   help="Reverse column order if the print is mirrored")

    # --- test patterns (alternative to text) --------------------------------
    g = ap.add_argument_group("test patterns (alternative to text)")
    g.add_argument("--calibrate", action="store_true",
                   help="Print a calibration ruler instead of text: a continuous "
                        "baseline with full-height ticks every --calib-major-mm "
                        "and short ticks every --calib-minor-mm, to measure the "
                        "real mm/column against --mm-per-column/--dpi")
    g.add_argument("--pattern", choices=sorted(patterns.PATTERNS),
                   help="Print a test pattern instead of text; runs through the "
                        "same tracking/time pipeline as text")
    g.add_argument("--pattern-length-mm", type=float, default=200.0,
                   help="Physical length of --calibrate/--pattern (default 200)")
    g.add_argument("--pattern-square-mm", type=float, default=10.0,
                   help="Column period in mm for checkerboard/v-stripes/diagonal "
                        "(default 10)")
    g.add_argument("--pattern-square-rows", type=int, default=20,
                   help="Row period for checkerboard/h-stripes (default 20)")
    g.add_argument("--calib-major-mm", type=float, default=10.0,
                   help="Distance between full-height ruler ticks (default 10 = 1cm)")
    g.add_argument("--calib-minor-mm", type=float, default=1.0,
                   help="Distance between short ruler ticks (default 1 = 1mm)")

    # --- nozzle mapping (correct scrambled wiring) --------------------------
    g = ap.add_argument_group("nozzle mapping (correct scrambled wiring)")
    g.add_argument("--nozzle-block-size", type=int,
                   help="Size of a repeating physical nozzle block; enables "
                        "remapping (must be given together with --nozzle-order)")
    g.add_argument("--nozzle-order",
                   help="Comma-separated new order within each block, e.g. "
                        "'2,3,4,1,5' for a block size of 5 (1-indexed, must be "
                        "a permutation)")

    # --- printing mode -----------------------------------------------------
    g = ap.add_argument_group("printing mode")
    g.add_argument("--mode", choices=("position", "time"), default="position",
                   help="position = Amfitrack closed loop (default); "
                        "time = stream one column every --period seconds")
    g.add_argument("--no-track", dest="track", action="store_false",
                   help="Disable tracking (forces time mode)")
    g.add_argument("--period", type=float, default=0.03,
                   help="Seconds per column in time mode (default 0.03)")

    # --- Amfitrack ---------------------------------------------------------
    g = ap.add_argument_group("Amfitrack positioning")
    g.add_argument("--advance-axis", choices=("x", "y", "z"), default="y",
                   help="Sensor axis that is the travel direction. Default y "
                        "because the sensor is mounted rotated (travel in Y/Z).")
    g.add_argument("--axis-sign", type=int, choices=(1, -1), default=1,
                   help="Flip the travel direction (default 1)")
    g.add_argument("--auto-calibrate", action="store_true",
                   help="Learn the travel direction from the first motion "
                        "instead of using a fixed axis (robust to any rotation)")
    g.add_argument("--calib-distance", type=float, default=5.0,
                   help="mm of motion before auto-calibration locks (default 5)")
    scale = g.add_mutually_exclusive_group()
    scale.add_argument("--mm-per-column", type=float, default=0.2,
                       help="Physical width of one printed column in mm "
                            "(default 0.2)")
    scale.add_argument("--dpi", type=float,
                       help="Horizontal resolution; sets mm/column = 25.4/DPI")
    g.add_argument("--origin", choices=("button", "startpoint"), default="button",
                   help="What zeroes the position: START press or the startpoint "
                        "characteristic (default button)")
    g.add_argument("--min-move", type=float, default=0.05,
                   help="Deadband in mm; below this the head counts as stopped "
                        "(default 0.05)")
    g.add_argument("--timeout", type=float, default=30.0,
                   help="Abort a position pass after this many seconds (default 30)")
    g.add_argument("--vendor-id", type=_auto_int, default=0x0C17,
                   help="Amfitrack USB vendor id (default 0x0C17)")
    g.add_argument("--product-id", type=_auto_int, default=0x0D12,
                   help="Amfitrack USB product id (default 0x0D12)")
    g.add_argument("--sensor-id", type=_auto_int,
                   help="optional tx_id filter among the 'Sensor' nodes "
                        "(default: use all)")
    g.add_argument("--simulate", action="store_true",
                   help="Use a fake tracker (no hardware) to test the loop")

    # --- timing / profiling ------------------------------------------------
    g = ap.add_argument_group("timing / profiling (position mode)")
    g.add_argument("--profile", action="store_true",
                   help="Instrument the position pass: log head speed, demanded "
                        "vs. sustained BLE column rate and write latency, and a "
                        "verdict on whether columns kept up with the head")
    g.add_argument("--profile-csv",
                   help="Also write a per-column timing log to this CSV path")
    g.add_argument("--record",
                   help="Reconstruct what is actually deposited on paper: record "
                        "every sent frame + head position and save a PNG (intended "
                        "vs. sent-mapped-to-position) to this path after the pass")

    # --- BLE / run ---------------------------------------------------------
    g = ap.add_argument_group("BLE / run")
    g.add_argument("--device-name", default=DEVICE_NAME,
                   help=f"BLE device name (default {DEVICE_NAME})")
    g.add_argument("--address",
                   help="Connect directly to a BLE MAC/UUID and skip scanning")
    g.add_argument("--scan-timeout", type=float, default=10.0)
    g.add_argument("--auto-start", action="store_true",
                   help="Start immediately without waiting for the button")
    g.add_argument("--once", action="store_true",
                   help="Exit after one print (default: keep listening)")
    g.add_argument("--preview", help="Save a PNG preview of the rendered image")
    g.add_argument("--dry-run", action="store_true",
                   help="Render (and optionally preview/simulate) only; no BLE")
    g.add_argument("--verbose", action="store_true")

    # --- debug / diagnostics (each runs a standalone check and exits) -------
    g = ap.add_argument_group("debug / diagnostics (each runs a check and exits)")
    mx = g.add_mutually_exclusive_group()
    mx.add_argument("--pos", action="store_true",
                    help="Live-print the Amfitrack position (x/y/z + advance + "
                         "column); works with --simulate. Ctrl+C to stop")
    g.add_argument("--pos-json", action="store_true",
                   help="With --pos: emit one JSON object per sample (newline "
                        "terminated) instead of the live line (used by the web UI)")
    mx.add_argument("--list-nodes", action="store_true",
                    help="List the Amfitrack USB nodes (name/uuid/tx_id) and exit")
    mx.add_argument("--scan-ble", action="store_true",
                    help="Scan for BLE devices (address + name) and exit")
    mx.add_argument("--nozzle-test", action="store_true",
                    help="Fire a nozzle test pattern over BLE and exit")
    mx.add_argument("--ble-benchmark", action="store_true",
                    help="Measure BLE column throughput + round-trip latency "
                         "(the ceiling that makes printing speed-dependent) and exit")

    args = ap.parse_args(argv)
    if not _debug_mode(args):
        n = _content_mode_count(args)
        if n == 0:
            ap.error("provide 'text', or --calibrate, or --pattern NAME "
                     "(or use a debug flag like --pos)")
        if n > 1:
            ap.error("choose only one of: text, --calibrate, --pattern")
    if bool(args.nozzle_block_size) != bool(args.nozzle_order):
        ap.error("--nozzle-block-size and --nozzle-order must be given together")
    if args.nozzle_block_size is not None and args.nozzle_block_size <= 0:
        ap.error("--nozzle-block-size must be a positive integer")
    return args


def _debug_mode(args: argparse.Namespace) -> bool:
    return bool(args.pos or args.list_nodes or args.scan_ble or args.nozzle_test
                or args.ble_benchmark)


def _content_mode_count(args: argparse.Namespace) -> int:
    return int(args.text is not None) + int(args.calibrate) + int(args.pattern is not None)


def build_ble(args: argparse.Namespace) -> BleSettings:
    return BleSettings(
        device_name=args.device_name, address=args.address,
        scan_timeout=args.scan_timeout, auto_start=args.auto_start,
        once=args.once, period=args.period, verbose=args.verbose)


def build_tracking(args: argparse.Namespace) -> TrackingSettings:
    # --no-track forces time mode; otherwise honour --mode.
    mode = args.mode if args.track else "time"
    tracking = TrackingSettings(
        enabled=args.track, mode=mode,
        advance_axis=args.advance_axis, axis_sign=args.axis_sign,
        auto_calibrate=args.auto_calibrate, calib_distance_mm=args.calib_distance,
        origin=args.origin, min_move_mm=args.min_move, timeout_s=args.timeout,
        vendor_id=args.vendor_id, product_id=args.product_id,
        sensor_id=args.sensor_id)
    tracking.mm_per_column = tracking.resolve_mm_per_column(args.dpi)
    return tracking


def build_ink(args: argparse.Namespace, mm_per_column: float):
    """Return (ink, label) from whichever content source was selected."""
    if args.calibrate:
        ink = patterns.ruler_pattern(
            args.pattern_length_mm, mm_per_column,
            major_every_mm=args.calib_major_mm, minor_every_mm=args.calib_minor_mm)
        return ink, f"[calibrate {args.pattern_length_mm:.0f}mm]"
    if args.pattern:
        ink = patterns.PATTERNS[args.pattern](
            args.pattern_length_mm, mm_per_column,
            square_mm=args.pattern_square_mm, square_rows=args.pattern_square_rows)
        return ink, f"[pattern {args.pattern} {args.pattern_length_mm:.0f}mm]"
    render = RenderSettings(
        text=args.text, font=args.font, render_size=args.render_size,
        threshold=args.threshold, margin=args.margin, invert=args.invert,
        flip_y=args.flip_y, mirror_x=args.mirror_x)
    return render_text(render), args.text


def build_nozzle_map(args: argparse.Namespace) -> Optional[NozzleMapSettings]:
    if args.nozzle_block_size is None:
        return None
    try:
        order = parse_order(args.nozzle_order, args.nozzle_block_size)
    except ValueError as exc:
        raise SystemExit(f"printhead: error: {exc}")
    if IMAGE_HEIGHT % args.nozzle_block_size:
        leftover = IMAGE_HEIGHT % args.nozzle_block_size
        print(f"NOTE: {IMAGE_HEIGHT} rows is not a multiple of block size "
              f"{args.nozzle_block_size}; the trailing {leftover} row(s) are "
              f"left unmapped.")
    return NozzleMapSettings(block_size=args.nozzle_block_size, order=order)


def build_controller(args: argparse.Namespace) -> PrintController:
    tracking = build_tracking(args)
    ink, label = build_ink(args, tracking.mm_per_column)
    render = RenderSettings(text=label)
    return PrintController(render, build_ble(args), tracking,
                           simulate=args.simulate, preview=args.preview,
                           dry_run=args.dry_run, ink=ink,
                           nozzle_map=build_nozzle_map(args),
                           profile=args.profile, profile_csv=args.profile_csv,
                           record=args.record)


def _run_debug(args: argparse.Namespace) -> None:
    """Dispatch a standalone diagnostic; each connects, reports/acts, then exits."""
    from . import diagnostics
    if args.pos:
        asyncio.run(diagnostics.monitor_position(
            build_tracking(args), args.simulate, ndjson=args.pos_json))
    elif args.list_nodes:
        diagnostics.list_nodes(build_tracking(args))
    elif args.scan_ble:
        asyncio.run(diagnostics.scan_ble(build_ble(args)))
    elif args.nozzle_test:
        asyncio.run(diagnostics.nozzle_test(build_ble(args), build_nozzle_map(args)))
    elif args.ble_benchmark:
        asyncio.run(diagnostics.ble_benchmark(build_ble(args), build_tracking(args)))


def main(argv=None) -> None:
    args = parse_args(argv)
    try:
        if _debug_mode(args):
            _run_debug(args)
        else:
            asyncio.run(build_controller(args).run())
    except KeyboardInterrupt:
        print("\nInterrupted.")


if __name__ == "__main__":
    main()
