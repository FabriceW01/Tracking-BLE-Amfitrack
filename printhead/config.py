"""
Configuration dataclasses
=========================

Grouping the many CLI options into three small dataclasses keeps the function
signatures across the package short and self-documenting. ``cli.py`` builds
these from ``argparse`` and hands them to the ``PrintController``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional


@dataclass
class RenderSettings:
    """Everything needed to turn a text string into a 164 px-tall ink mask."""
    text: str
    font: Optional[str] = None
    render_size: int = 220          # font pixel size for the initial render
    threshold: int = 128            # black/white cut-off 0..255
    margin: int = 0                 # vertical margin (top+bottom) in px
    invert: bool = False            # print white text on black
    flip_y: bool = False            # flip vertically (upside-down print)
    mirror_x: bool = False          # reverse column order (mirrored print)


@dataclass
class BleSettings:
    """How to reach the PrintheadBLE ESP32 and how to pace the time-based mode."""
    device_name: str = "PrintheadBLE"
    address: Optional[str] = None   # connect directly, skip scanning
    scan_timeout: float = 10.0
    auto_start: bool = False        # start without waiting for the START button
    once: bool = False              # exit after a single pass
    period: float = 0.03            # seconds per column (time-based mode only)
    verbose: bool = False


@dataclass
class TrackingSettings:
    """
    Amfitrack positioning configuration.

    ``mode`` selects the print strategy:
      * ``"position"`` - closed loop: the printed column follows the measured
        position, so the horizontal scale is independent of cart speed.
      * ``"time"``     - legacy behaviour: stream one column every ``period`` s.

    The sensor is mounted rotated (travel happens in Y/Z instead of X/Y), so the
    axis that drives column advancement is configurable. Two strategies exist:
      * fixed axis   - use ``advance_axis`` (default ``"y"``) times ``axis_sign``.
      * auto-calibrate - measure the real direction of motion at start and
        project the position onto it (robust against any rotation).
    """
    enabled: bool = True
    mode: str = "position"          # "position" | "time"

    # --- axis mapping for the rotated sensor -------------------------------
    advance_axis: str = "y"         # "x" | "y" | "z" (which axis = travel dir.)
    axis_sign: int = 1              # +1 or -1 (flip travel direction)
    auto_calibrate: bool = False    # derive travel direction from first motion
    calib_distance_mm: float = 5.0  # motion needed before auto-calibration locks

    # --- horizontal scale --------------------------------------------------
    mm_per_column: float = 0.2      # physical width of one printed column (mm)

    # --- run behaviour -----------------------------------------------------
    origin: str = "button"          # "button" | "startpoint" (what zeroes pos.)
    min_move_mm: float = 0.05       # deadband: below this, treat head as stopped
    timeout_s: float = 30.0         # give up a pass after this long with no end
    poll_hz: float = 200.0          # position polling rate

    # --- USB dongle (amfiprot) --------------------------------------------
    vendor_id: int = 0x0C17          # Amfitech USB vendor id
    product_id: int = 0x0D12         # sensor dongle product id (tried first)
    product_id_source: int = 0x0D01  # source dongle product id (fallback)
    sensor_id: Optional[int] = None  # optional tx_id filter among "Sensor" nodes

    def resolve_mm_per_column(self, dpi: Optional[float]) -> float:
        """If a DPI was given on the CLI, derive mm/column from it (25.4/DPI)."""
        if dpi:
            return 25.4 / dpi
        return self.mm_per_column


@dataclass
class NozzleMapSettings:
    """
    Corrects for physically-scrambled nozzle wiring: the printhead's nozzles are
    wired in repeating blocks of ``block_size``, and ``order`` gives the source
    row (0-indexed) within the block for each output row -- reorders ink rows in
    repeating blocks before framing so the printed image comes out correct.
    """
    block_size: Optional[int] = None
    order: Optional[List[int]] = None
