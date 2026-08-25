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
from .controller import DEFAULT_SPEED_WARNING_MM_S, PrintController
from .geometry import (
    DEVICE_NAME,
    IMAGE_HEIGHT,
    NOZZLE_PITCH_MM,
    SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM,
    SENSOR_TO_NOZZLE_COL_MM,
)
from .nozzle_map import parse_order
from .rendering import render_text


def _auto_int(value: str) -> int:
    """Parse an int that may be given as decimal or 0x-hex (for USB ids)."""
    return int(value, 0)


def parse_args(argv=None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        prog="printhead",
        description="Render text to a 152px-tall B/W image and print it "
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
    g.add_argument("--pattern-image", type=str, default=None,
                   help="Image file for --pattern drill_pattern (any "
                        "PIL-readable format: PNG, JPG, BMP, ...), rasterised "
                        "to the requested --pattern-length-mm/--pattern-height-mm "
                        "size. No image ships with this repo -- without this "
                        "flag drill_pattern looks for one at "
                        "assets/drill_pattern.png next to the printhead/ "
                        "package (see README); if it's not there, place one "
                        "or pass this flag instead. Ignored by every other "
                        "--pattern")
    g.add_argument("--pattern-length-mm", type=float, default=200.0,
                   help="Physical length of --calibrate/--pattern (default 200)")
    g.add_argument("--pattern-square-mm", type=float, default=10.0,
                   help="Column period in mm for checkerboard/v-stripes/diagonal "
                        "(default 10)")
    g.add_argument("--pattern-square-rows", type=int, default=20,
                   help="Row period for checkerboard/h-stripes (default 20)")
    g.add_argument("--pattern-height-mm", type=float, default=None,
                   help="Total physical height of --calibrate/--pattern in mm "
                        "(rows = max(1, round(height_mm / NOZZLE_PITCH_MM))). "
                        "Page mode only: line/time mode packs fixed frames via "
                        "frames_from_ink(), which requires exactly IMAGE_HEIGHT "
                        "rows, so this is rejected outside --mode page. Without "
                        "it the pattern is capped at IMAGE_HEIGHT rows (13.2mm).")
    g.add_argument("--pattern-line-rows", type=int, default=1,
                   help="--pattern precision-check only: how many nozzle rows "
                        "thick each printed line is (default 1 = a single "
                        "nozzle row, ~0.087mm -- the sharpest test). Raise it "
                        "to measure gap resolution without a single weak "
                        "nozzle confusing the result")
    g.add_argument("--pattern-gap-start", type=int, default=1,
                   help="--pattern precision-check only: the FIRST gap between "
                        "lines, in nozzle rows; it doubles after every line. "
                        "1 gives gaps 1,2,4,8,16..., 2 gives 2,4,8,16..., "
                        "4 gives 4,8,16,32... (default 1). Counted in rows, "
                        "not mm, because that is the unit the answer is "
                        "quantised to -- the mm equivalents are printed as a "
                        "table when the pattern is generated")
    g.add_argument("--pattern-square-height-mm", type=float, default=None,
                   help="Row period in mm for checkerboard/h-stripes; overrides "
                        "--pattern-square-rows (square_rows = max(1, round(v / "
                        "NOZZLE_PITCH_MM))). A raw row is ~0.087mm, so this "
                        "is usually what you want for actually-square tiles.")
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
    g.add_argument("--mode", choices=("line", "page", "time"), default="page",
                   help="page = freehand 2D closed loop (default), needs "
                        "--page-calibration; "
                        "line = 1D Amfitrack closed loop; "
                        "time = stream one column every --period seconds")
    g.add_argument("--no-track", dest="track", action="store_false",
                   help="Disable tracking (forces time mode)")
    g.add_argument("--period", type=float, default=0.03,
                   help="Seconds per column in time mode (default 0.03)")
    g.add_argument("--dose-hold-s", type=float, default=None,
                   help="Page mode: seconds a nozzle must continuously hold a "
                        "pixel before it counts as printed (default: "
                        "coverage.DEFAULT_DOSE_HOLD_S = 0.00405, measured "
                        "against a real print at ~17 mm/s median hand speed "
                        "-- must track the firmware's PATTERN_STRIDE "
                        "(src/ble_dose.h): DEFAULT_DOSE_HOLD_S ~= "
                        "3 * PATTERN_STRIDE * 450us; changing one without "
                        "the other and re-flashing breaks the ~3-drop-per-"
                        "pixel target. MUST also stay below 1/--poll-hz "
                        "(the poll interval): at or above it, two "
                        "consecutive samples cannot complete a dose and "
                        "coverage collapses -- PrintController warns at "
                        "runtime if this holds)")
    g.add_argument("--spray-radius-mm", type=float, default=None,
                   help="Page mode ink-spread model: physical radius (mm) "
                        "around a completed pixel that also receives a "
                        "partial dose, since a real drop wets more than its "
                        "own grid cell. In MILLIMETRES, not pixels -- a cell "
                        "is ~0.087mm tall but --mm-per-column (0.2 default) "
                        "wide, so the same pixel count would mean two "
                        "different physical distances per axis. Default 0 "
                        "(off, exactly the pre-spray behaviour); pair with "
                        "--spray-strength, both must be > 0 to take effect. "
                        "Reduces re-printing when a return pass lands a "
                        "fraction of a mm off the outbound one")
    g.add_argument("--spray-strength", type=float, default=None,
                   help="Page mode ink-spread model: how much dose a "
                        "neighbour just next to a completed pixel picks up, "
                        "0.0..1.0, falling off linearly to 0 at "
                        "--spray-radius-mm. 1.0 would mark a neighbour fully "
                        "printed from one drop alone; a pixel counts as "
                        "printed once its accumulated dose reaches 1.0, so "
                        "e.g. 0.5 needs two neighbouring drops. Default 0 "
                        "(off). Too high under-prints (real gaps left "
                        "unfilled), so raise it gradually and check the "
                        "printed sheet, not just --record")
    g.add_argument("--nozzle-group", type=int, choices=(1, 2), default=1,
                   help="Page mode only: tie N adjacent nozzles into one "
                        "addressable unit that always fires (and doses) "
                        "together or not at all -- for coarser VERTICAL "
                        "addressing. N=2 halves the smallest addressable "
                        "vertical step from NOZZLE_PITCH_MM (~0.087mm) to "
                        "~0.174mm; the physical nozzle pitch itself does "
                        "not change. A group fires if ANY member's pixel "
                        "still wants ink (OR rule), so a group straddling "
                        "an ink/no-ink boundary also inks the no-ink side. "
                        "Default 1 = today's behaviour, every nozzle "
                        "addressed individually. NOT the same feature as "
                        "--nozzle-block-size/--nozzle-order above: those "
                        "permute which image ROW an individually-addressed "
                        "nozzle receives, for a rig wired out of order -- "
                        "this option does not reorder anything, it only "
                        "ties adjacent nozzles' firing together")
    g.add_argument("--progress-json", action="store_true",
                   help="Page mode: emit one JSON progress event per sample "
                        "(current u/v/row/col + newly-covered cells) instead "
                        "of the plain-text status lines -- used by the web UI's "
                        "live coverage view")
    g.add_argument("--speed-warning-mm-s", type=float, default=None,
                   help="Page mode: cart speed (mm/s) above which the client "
                        "warns the firmware over BLE that it is moving too "
                        "fast to print reliably (default: "
                        f"controller.DEFAULT_SPEED_WARNING_MM_S = "
                        f"{DEFAULT_SPEED_WARNING_MM_S:g}, the speed at which "
                        "dose-tuning measurements found coverage had already "
                        "fallen to ~60%%). Hysteresis: the warning clears "
                        "again only once speed drops 20%% below this "
                        "threshold, to avoid chattering the characteristic "
                        "right at the boundary. Advisory only -- drives a "
                        "status LED on the firmware, never affects dosing")

    # --- Amfitrack ---------------------------------------------------------
    g = ap.add_argument_group("Amfitrack positioning")
    g.add_argument("--advance-axis", choices=("x", "y", "z"), default="x",
                   help="Sensor axis that is the travel direction. Default x, "
                        "the rig's measured travel axis; change this if your "
                        "sensor is mounted rotated relative to that.")
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
    g.add_argument("--smooth-ms", type=float, default=12.0,
                   help="Low-pass time constant (ms) for the noisy Amfitrack "
                        "position; 0 = off, larger = smoother but more lag "
                        "(default 12)")
    g.add_argument("--min-move", type=float, default=0.05,
                   help="Deadband in mm; below this the head counts as stopped "
                        "(default 0.05)")
    g.add_argument("--poll-hz", type=float, default=500.0,
                   help="Position polling rate. A column crossing can only be "
                        "noticed once per poll, so this bounds how precisely a "
                        "column is placed: at 500 Hz and 20 mm/s that is 0.04 mm "
                        "= a fifth of a column (default 500)")
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
    g = ap.add_argument_group("timing / profiling (line/page mode)")
    g.add_argument("--profile", action="store_true",
                   help="Instrument the pass: line mode logs head speed, demanded "
                        "vs. sustained BLE column rate and write latency; page mode "
                        "logs the pattern-update rate against --ble-write-ceiling. "
                        "Either way, ends in a verdict on whether BLE kept up")
    g.add_argument("--profile-csv",
                   help="Also write a per-write timing log to this CSV path")
    g.add_argument("--ble-write-ceiling", type=float, default=None,
                   help="Page mode --profile only: known BLE write-without-response "
                        "ceiling (writes/s) to compare the pattern-update rate "
                        "against (default: an untuned guess -- measure the real "
                        "number for your hardware with --ble-benchmark)")
    g.add_argument("--record",
                   help="Reconstruct what is actually deposited on paper: line mode "
                        "records every sent frame + head position and compares "
                        "intended-vs-sent; page mode saves the coverage engine's "
                        "printed mask directly (intended/covered/missed). PNG")

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
    g.add_argument("--batch-cols", type=int, default=0, metavar="N",
                   help="Columns per BLE write (default 0 = derive from the "
                        "negotiated MTU). Use 1 for firmware without batching")
    g.add_argument("--preview", help="Save a PNG preview of the rendered image")
    g.add_argument("--dry-run", action="store_true",
                   help="Render (and optionally preview/simulate) only; no BLE")
    g.add_argument("--verbose", action="store_true",
                   help="--mode line/page: print a live, self-overwriting "
                        "position status line (x/y/z, page u/v/yaw/roll/"
                        "pitch, covered/total) WHILE actually printing -- "
                        "the --pos equivalent usable during a real pass, "
                        "unlike --pos itself which cannot be combined with "
                        "one. --mode time: verbose BLE write logging "
                        "(unchanged)")

    # --- debug / diagnostics (each runs a standalone check and exits) -------
    g = ap.add_argument_group("debug / diagnostics (each runs a check and exits)")
    mx = g.add_mutually_exclusive_group()
    mx.add_argument("--pos", action="store_true",
                    help="Live-print the Amfitrack position (x/y/z + advance + "
                         "column); works with --simulate. Ctrl+C to stop")
    mx.add_argument("--calibration-check", action="store_true",
                    help="Calibration health check: streams live page u/v/yaw/"
                         "roll/pitch like --pos, then (Ctrl+C) prints a summary "
                         "of yaw drift while the cart only TRANSLATES -- slide "
                         "it flat over the page WITHOUT rotating it. Reports "
                         "travelled u/v, yaw min/max/span (the headline "
                         "number -- should stay near 0 with no rotation), "
                         "roll/pitch span, and yaw's correlation against u/v "
                         "(separates noise from systematic drift with "
                         "position). Needs --page-calibration PATH or "
                         "--page-frame simple: there is no page u/v to check "
                         "drift against otherwise")
    g.add_argument("--pos-json", action="store_true",
                   help="With --pos or --calibration-check: emit one JSON "
                        "object per sample (newline terminated) instead of "
                        "the live line (used by the web UI)")
    g.add_argument("--page-frame", choices=("calibrated", "simple"),
                   default="calibrated",
                   help="Which 2D frame --mode page prints into. calibrated "
                        "(default) = a traced --page-calibration with a captured "
                        "boresight, most accurate. simple = no calibration at "
                        "all: the tracker's own x/y axes are the page axes, the "
                        "origin is zeroed wherever the cart is at START, and yaw "
                        "is measured relative to the pose held at START (or, "
                        "better, --simple-boresight) -- assumes the sheet lies "
                        "square with the tracker")
    g.add_argument("--simple-boresight", nargs=4, type=float,
                   metavar=("QX", "QY", "QZ", "QW"),
                   help="Page mode with --page-frame simple only: pin the yaw "
                        "reference pose instead of auto-capturing whatever "
                        "orientation the cart happens to be in at START. "
                        "Auto-capture can silently lock in a wrong pose (BLE "
                        "still settling, hand not yet still), with no way to "
                        "notice; capture the real value first with --pos "
                        "--page-frame simple (printed live as 'quat=[...]'), "
                        "confirm roll/pitch/yaw read ~0 while the cart is "
                        "genuinely flat, then pass that exact quaternion here "
                        "as four space-separated numbers (NOT comma-joined -- "
                        "a single comma-joined value trips argparse's "
                        "negative-number heuristic and is rejected)")
    g.add_argument("--page-calibration", metavar="PATH",
                   help="Load a page calibration JSON (see "
                        "printhead.calibration.PageCalibration). Required for "
                        "--mode page unless --page-frame simple. With --pos, "
                        "also reports live page-plane u/v/z, to sanity-check a "
                        "calibration against known hand motion before printing "
                        "with it")
    g.add_argument("--sensor-offset-row-mm", type=float, default=None,
                   help="Page mode: distance in mm, along the row axis (along "
                        "the nozzle bar, perpendicular to travel), from the "
                        "tracked Amfitrack sensor to the CENTRE of the "
                        "152-nozzle bar -- the sensor is not physically at the "
                        "printhead. Default: "
                        f"geometry.SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM = "
                        f"{SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM:g}, a real "
                        "physical measurement (not a guess like the initial "
                        "dose-tuning constants were). If a test print comes "
                        "out shifted the wrong way along v, negate this "
                        "value -- nothing else needs to change")
    g.add_argument("--sensor-offset-col-mm", type=float, default=None,
                   help="Page mode: same idea as --sensor-offset-row-mm but "
                        "along the column axis (the travel/sweep direction). "
                        "Default: geometry.SENSOR_TO_NOZZLE_COL_MM = "
                        f"{SENSOR_TO_NOZZLE_COL_MM:g}, currently assumed 0 "
                        "(no measurement has shown otherwise); a wrong-signed "
                        "test print is fixed the same way -- negate this value")
    g.add_argument("--boresight-deg", type=float, default=None,
                   help="Page mode: additive fine-tuning offset in degrees, "
                        "added to the yaw computed from the captured "
                        "boresight quaternion (PageCalibration.boresight_quat, "
                        "see --page-calibration / the Calibration tab's "
                        "'Capture boresight' button) -- lets a print that "
                        "comes out rotated be corrected without "
                        "re-calibrating, same 'adjust rather than rebuild' "
                        "idea as --sensor-offset-row-mm/-col-mm above. Has "
                        "no effect on a calibration with no boresight "
                        "captured (default: 0.0, i.e. trust the captured "
                        "boresight exactly)")
    g.add_argument("--latency-compensate-s", type=float, default=None,
                   help="Page mode: linearly extrapolate the position fed to "
                        "the coverage engine forward by this many seconds, "
                        "using the current sample's own velocity estimate "
                        "((u,v) - previous (u,v), divided by the elapsed "
                        "time) -- a heuristic correction for the measured "
                        "~13ms typical BLE-connection-interval + firmware-"
                        "queue + fire-slot pipeline delay between 'position "
                        "read' and 'ink actually placed', which otherwise "
                        "shows up as a position lag whose direction flips "
                        "with the direction of travel. Only the coordinates "
                        "passed to CoverageEngine.step() are shifted -- "
                        "--record's path panels, the out-of-page bounds "
                        "check and the speed warning all keep using the "
                        "real, uncompensated position, since those exist to "
                        "show where the cart actually was. Default 0.0 (off, "
                        "today's behaviour): this is a heuristic tuned to a "
                        "measured pipeline delay, not a general-purpose "
                        "smoothing knob, and a wrong value can overshoot "
                        "(most visibly right as the cart stops or reverses) "
                        "as easily as it corrects -- start small and check "
                        "against a real print before trusting it")
    mx.add_argument("--list-nodes", action="store_true",
                    help="List the Amfitrack USB nodes (name/uuid/tx_id) and exit")
    mx.add_argument("--scan-ble", action="store_true",
                    help="Scan for BLE devices (address + name) and exit")
    mx.add_argument("--nozzle-test", action="store_true",
                    help="Fire a nozzle test pattern over BLE and exit")
    mx.add_argument("--ble-benchmark", action="store_true",
                    help="Measure BLE column throughput + round-trip latency "
                         "(the ceiling that makes printing speed-dependent) and exit")
    mx.add_argument("--straightness", metavar="PROFILE_CSV",
                    help="Offline: analyse a --mode page --profile-csv file "
                         "from a pass run along a straight edge (a ruler). "
                         "Fits a total-least-squares line through the logged "
                         "u/v path and reports how far the points deviate "
                         "from it -- overall (RMS/p95/max, also in nozzle "
                         "rows), split into a systematic bend vs random "
                         "jitter, and binned by position along the line. "
                         "Also reports how far the cart ROTATED and how much "
                         "apparent deviation that alone explains through the "
                         "62mm sensor->nozzle-bar lever arm (1 deg ~ 1.1 mm), "
                         "which is usually the largest term. No hardware "
                         "needed; reads the file and exits")
    g.add_argument("--straightness-bins", type=int, default=10, metavar="N",
                   help="With --straightness: how many equal-width bins to "
                        "split the travelled line into for the "
                        "deviation-by-position table (default 10)")

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
    if not _debug_mode(args) and args.track and args.mode == "page" and args.nozzle_block_size is not None:
        ap.error("--nozzle-block-size/--nozzle-order are not supported with "
                 "--mode page: the block permutation is indexed by image row, "
                 "but page mode's nozzle-to-row alignment shifts with vertical "
                 "travel, so the permutation would only be correct at multiples "
                 "of the block size")
    if (not _debug_mode(args) and args.track and args.mode == "page"
            and args.page_frame == "calibrated" and not args.page_calibration):
        ap.error("--mode page requires --page-calibration PATH (trace the page "
                 "edges with calibration.calibrate_page() first, then save()), "
                 "or --page-frame simple to print without a calibration")
    if args.page_frame == "simple" and args.page_calibration:
        # Both given = two different page frames requested at once. Refuse
        # rather than silently picking one: the traced file is exactly what
        # --page-frame simple exists to bypass, so quietly honouring either
        # choice would be a surprise in one direction or the other.
        ap.error("--page-frame simple and --page-calibration are mutually "
                 "exclusive: simple takes the tracker's own x/y axes as the "
                 "page frame and ignores any traced calibration")
    if args.calibration_check and args.page_frame != "simple" and not args.page_calibration:
        # Unlike --pos (which tolerates no page frame at all -- it just
        # omits the page_u/page_v fields), --calibration-check has nothing
        # to measure drift IN without one: yaw is only ever reported
        # relative to a page normal, see rotation.yaw_about_normal.
        ap.error("--calibration-check needs a page frame to measure drift "
                 "against: pass --page-calibration PATH or --page-frame simple")
    if args.spray_radius_mm is not None and args.spray_radius_mm < 0.0:
        ap.error("--spray-radius-mm cannot be negative (0 disables the "
                 "ink-spread model)")
    if args.spray_strength is not None and not 0.0 <= args.spray_strength <= 1.0:
        # Above 1.0 a single drop would mark whole neighbourhoods printed
        # outright, which stops the pass firing over real gaps -- a silent
        # under-print rather than an obvious error, so reject it here.
        ap.error("--spray-strength must be between 0.0 and 1.0 (a pixel counts "
                 "as printed at an accumulated dose of 1.0; 0 disables the "
                 "ink-spread model)")
    if args.simple_boresight and args.page_frame != "simple":
        # nargs=4/type=float already guarantee exactly 4 numbers -- nothing
        # left to validate here except that this only makes sense paired
        # with --page-frame simple.
        ap.error("--simple-boresight only applies to --page-frame simple "
                 "(the calibrated frame's yaw reference comes from the traced "
                 "calibration's own boresight_quat instead)")
    if (not _debug_mode(args) and args.track and args.mode != "page"
            and args.pattern_height_mm is not None):
        ap.error("--pattern-height-mm is only valid with --mode page: line/time "
                 "mode packs fixed frames via frames_from_ink(), which requires "
                 "exactly IMAGE_HEIGHT rows, so the pattern height can't be "
                 "changed there")
    if (not _debug_mode(args) and args.track and args.mode != "page"
            and args.nozzle_group != 1):
        ap.error("--nozzle-group 2 is only valid with --mode page: line/time "
                 "mode packs fixed frames via rendering.frames_from_ink() "
                 "instead of going through CoverageEngine, so nozzle grouping "
                 "has no effect there (this is unrelated to "
                 "--nozzle-block-size/--nozzle-order, which IS supported "
                 "outside page mode -- see that option's help text)")
    return args


def _debug_mode(args: argparse.Namespace) -> bool:
    # --straightness is included here for the same reason as every other
    # entry: these all run a standalone check and exit, so none of them
    # needs the text/--calibrate/--pattern content that a real print does.
    # Unlike the others it touches no hardware at all -- it only reads a
    # CSV written by an earlier pass.
    return bool(args.pos or args.calibration_check or args.list_nodes or args.scan_ble
                or args.nozzle_test or args.ble_benchmark or args.straightness)


def _content_mode_count(args: argparse.Namespace) -> int:
    return int(args.text is not None) + int(args.calibrate) + int(args.pattern is not None)


def build_ble(args: argparse.Namespace) -> BleSettings:
    return BleSettings(
        device_name=args.device_name, address=args.address,
        scan_timeout=args.scan_timeout, auto_start=args.auto_start,
        once=args.once, period=args.period, verbose=args.verbose,
        batch_cols=args.batch_cols)


def build_tracking(args: argparse.Namespace) -> TrackingSettings:
    # --no-track forces time mode; otherwise honour --mode.
    mode = args.mode if args.track else "time"
    tracking = TrackingSettings(
        enabled=args.track, mode=mode, page_frame=args.page_frame,
        advance_axis=args.advance_axis, axis_sign=args.axis_sign,
        auto_calibrate=args.auto_calibrate, calib_distance_mm=args.calib_distance,
        origin=args.origin, min_move_mm=args.min_move, timeout_s=args.timeout,
        smooth_ms=args.smooth_ms, poll_hz=args.poll_hz,
        vendor_id=args.vendor_id, product_id=args.product_id,
        sensor_id=args.sensor_id,
        # CORRECTION: this constructor call used to omit mm_per_column
        # entirely, so TrackingSettings fell back to its own dataclass
        # default (0.2) regardless of --mm-per-column -- only --dpi ever
        # had any effect, via resolve_mm_per_column below. --mm-per-column
        # is a real, documented CLI flag (and the default value in its own
        # --help text), so silently ignoring it is a bug, not a feature:
        # confirmed on a real command (200mm pattern, --mm-per-column 0.1)
        # rendering 1000 columns instead of the requested 2000 -- exactly
        # the 0.2mm/column default, and exactly why a page-mode checkerboard
        # meant to be square (--pattern-square-mm == --pattern-square-height-mm,
        # --mm-per-column == NOZZLE_PITCH_MM) still rendered 2x too tall in
        # coverage.png: every column was twice as wide as intended.
        mm_per_column=args.mm_per_column)
    tracking.mm_per_column = tracking.resolve_mm_per_column(args.dpi)
    return tracking


def build_ink(args: argparse.Namespace, mm_per_column: float):
    """Return (ink, label) from whichever content source was selected."""
    # --pattern-height-mm (page mode only, see parse_args) picks the row count;
    # otherwise every generator defaults to IMAGE_HEIGHT, same as text.
    rows = IMAGE_HEIGHT
    if args.pattern_height_mm is not None:
        rows = max(1, round(args.pattern_height_mm / NOZZLE_PITCH_MM))
    height_mm = rows * NOZZLE_PITCH_MM

    # --pattern-square-height-mm is just a mm-based alternative unit for
    # --pattern-square-rows -- a raw row is NOZZLE_PITCH_MM (~0.087mm), so mm
    # is usually what you actually want for a square tile.
    square_rows = args.pattern_square_rows
    if args.pattern_square_height_mm is not None:
        square_rows = max(1, round(args.pattern_square_height_mm / NOZZLE_PITCH_MM))

    if args.calibrate:
        ink = patterns.ruler_pattern(
            args.pattern_length_mm, mm_per_column,
            major_every_mm=args.calib_major_mm, minor_every_mm=args.calib_minor_mm,
            rows=rows)
        return ink, f"[calibrate {args.pattern_length_mm:.0f}mm x {height_mm:.0f}mm]"
    if args.pattern:
        ink = patterns.PATTERNS[args.pattern](
            args.pattern_length_mm, mm_per_column,
            square_mm=args.pattern_square_mm, square_rows=square_rows,
            line_rows=args.pattern_line_rows, gap_start=args.pattern_gap_start,
            rows=rows, pattern_image=args.pattern_image)
        if args.pattern == "precision-check":
            # A resolution target is unreadable on paper without knowing
            # which gap is which -- every gap just looks like "some white
            # space" once printed. Printed here rather than folded into the
            # one-line label below because it is a table, and because it is
            # exactly as useful under --dry-run/--preview (where you check
            # the target before burning ink) as during a real pass.
            print(patterns.format_precision_check_layout(
                patterns.precision_check_layout(
                    rows, args.pattern_line_rows, args.pattern_gap_start),
                NOZZLE_PITCH_MM))
        return ink, f"[pattern {args.pattern} {args.pattern_length_mm:.0f}mm x {height_mm:.0f}mm]"
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


def build_page_calibration(args: argparse.Namespace):
    """Load the PageCalibration named by --page-calibration, or None if it
    was not given (only --mode page requires it -- see parse_args).

    ``--page-frame simple`` instead synthesises the calibration-free frame
    (tracker x/y axes, no boresight unless --simple-boresight pinned one --
    see ``PageCalibration.simple_frame``) rather than reading any file.
    parse_args() has already rejected asking for both --page-frame simple
    and --page-calibration together; --simple-boresight's own shape
    (exactly 4 numbers) is validated by argparse itself (nargs=4, type=float)."""
    from .calibration import PageCalibration
    if args.page_frame == "simple":
        return PageCalibration.simple_frame(boresight_quat=args.simple_boresight)
    if not args.page_calibration:
        return None
    try:
        return PageCalibration.load(args.page_calibration)
    except Exception as exc:
        raise SystemExit(f"printhead: error: cannot load page calibration "
                         f"'{args.page_calibration}': {exc}")


def build_controller(args: argparse.Namespace) -> PrintController:
    tracking = build_tracking(args)
    ink, label = build_ink(args, tracking.mm_per_column)
    render = RenderSettings(text=label)
    kwargs = {}
    if args.dose_hold_s is not None:
        kwargs["dose_hold_s"] = args.dose_hold_s
    if args.spray_radius_mm is not None:
        kwargs["spray_radius_mm"] = args.spray_radius_mm
    if args.spray_strength is not None:
        kwargs["spray_strength"] = args.spray_strength
    # --nozzle-group has a real default of 1 (not None like the args above),
    # so it always reaches the controller -- there is no "unset" state to
    # distinguish from the default here.
    kwargs["nozzle_group"] = args.nozzle_group
    if args.ble_write_ceiling is not None:
        kwargs["ble_write_ceiling"] = args.ble_write_ceiling
    if args.speed_warning_mm_s is not None:
        kwargs["speed_warning_mm_s"] = args.speed_warning_mm_s
    if args.sensor_offset_row_mm is not None:
        kwargs["sensor_offset_row_mm"] = args.sensor_offset_row_mm
    if args.sensor_offset_col_mm is not None:
        kwargs["sensor_offset_col_mm"] = args.sensor_offset_col_mm
    if args.boresight_deg is not None:
        kwargs["boresight_deg"] = args.boresight_deg
    if args.latency_compensate_s is not None:
        kwargs["latency_compensate_s"] = args.latency_compensate_s
    return PrintController(render, build_ble(args), tracking,
                           simulate=args.simulate, preview=args.preview,
                           dry_run=args.dry_run, ink=ink,
                           nozzle_map=build_nozzle_map(args),
                           profile=args.profile, profile_csv=args.profile_csv,
                           record=args.record,
                           page_calibration=build_page_calibration(args),
                           progress_json=args.progress_json,
                           **kwargs)


def _run_debug(args: argparse.Namespace) -> None:
    """Dispatch a standalone diagnostic; each connects, reports/acts, then exits."""
    from . import diagnostics
    if args.pos:
        pos_kwargs = {}
        if args.sensor_offset_row_mm is not None:
            pos_kwargs["sensor_offset_row_mm"] = args.sensor_offset_row_mm
        if args.sensor_offset_col_mm is not None:
            pos_kwargs["sensor_offset_col_mm"] = args.sensor_offset_col_mm
        if args.boresight_deg is not None:
            pos_kwargs["boresight_deg"] = args.boresight_deg
        if args.simple_boresight:
            pos_kwargs["simple_boresight"] = args.simple_boresight
        asyncio.run(diagnostics.monitor_position(
            build_tracking(args), args.simulate, ndjson=args.pos_json,
            page_calibration_path=args.page_calibration, **pos_kwargs))
    elif args.calibration_check:
        # Same kwargs as --pos above (offsets/boresight don't change the
        # yaw-drift statistics -- a constant u/v shift and a fixed yaw
        # offset don't affect a SPAN or a CORRELATION -- but forwarding them
        # anyway keeps this diagnostic's page_u/page_v consistent with what
        # a real pass would report, same reasoning as --pos).
        check_kwargs = {}
        if args.sensor_offset_row_mm is not None:
            check_kwargs["sensor_offset_row_mm"] = args.sensor_offset_row_mm
        if args.sensor_offset_col_mm is not None:
            check_kwargs["sensor_offset_col_mm"] = args.sensor_offset_col_mm
        if args.boresight_deg is not None:
            check_kwargs["boresight_deg"] = args.boresight_deg
        if args.simple_boresight:
            check_kwargs["simple_boresight"] = args.simple_boresight
        asyncio.run(diagnostics.calibration_check(
            build_tracking(args), args.simulate, ndjson=args.pos_json,
            page_calibration_path=args.page_calibration, **check_kwargs))
    elif args.list_nodes:
        diagnostics.list_nodes(build_tracking(args))
    elif args.scan_ble:
        asyncio.run(diagnostics.scan_ble(build_ble(args)))
    elif args.nozzle_test:
        asyncio.run(diagnostics.nozzle_test(build_ble(args), build_nozzle_map(args)))
    elif args.ble_benchmark:
        asyncio.run(diagnostics.ble_benchmark(build_ble(args), build_tracking(args)))
    elif args.straightness:
        # Imported here, not at module scope: this is the only code path
        # that needs it, and keeping it local matches how the other
        # offline/optional analyses in this file stay out of the import
        # cost of a plain print run.
        from . import straightness
        print(straightness.analyze_csv(args.straightness,
                                       n_bins=args.straightness_bins))


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
