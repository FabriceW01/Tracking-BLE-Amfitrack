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

from .config import BleSettings, RenderSettings, TrackingSettings
from .controller import PrintController
from .geometry import DEVICE_NAME


def _auto_int(value: str) -> int:
    """Parse an int that may be given as decimal or 0x-hex (for USB ids)."""
    return int(value, 0)


def parse_args(argv=None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        prog="printhead",
        description="Render text to a 164px-tall B/W image and print it "
                    "column-by-column on an HP302 cartridge via the PrintheadBLE "
                    "ESP32, driven either by Amfitrack position or by a timer.")
    ap.add_argument("text", help="Text to print")

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
    g.add_argument("--advance-axis", choices=("x", "y", "z"), default="z",
                   help="Sensor axis that is the travel direction. Default z "
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
                   help="tx_id of the sensor node (default: first found)")
    g.add_argument("--simulate", action="store_true",
                   help="Use a fake tracker (no hardware) to test the loop")

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

    return ap.parse_args(argv)


def build_controller(args: argparse.Namespace) -> PrintController:
    render = RenderSettings(
        text=args.text, font=args.font, render_size=args.render_size,
        threshold=args.threshold, margin=args.margin, invert=args.invert,
        flip_y=args.flip_y, mirror_x=args.mirror_x)

    ble = BleSettings(
        device_name=args.device_name, address=args.address,
        scan_timeout=args.scan_timeout, auto_start=args.auto_start,
        once=args.once, period=args.period, verbose=args.verbose)

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

    return PrintController(render, ble, tracking, simulate=args.simulate,
                           preview=args.preview, dry_run=args.dry_run)


def main(argv=None) -> None:
    args = parse_args(argv)
    controller = build_controller(args)
    try:
        asyncio.run(controller.run())
    except KeyboardInterrupt:
        print("\nInterrupted.")


if __name__ == "__main__":
    main()
