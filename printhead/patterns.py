"""
Printable test patterns
========================

Generates ``(rows, W)`` boolean ink masks, exactly like ``rendering.render_text``
(whose output is always ``(IMAGE_HEIGHT, W)``), so they flow through the same
framing/BLE/tracking pipeline as text (position or time mode, ``--simulate``,
``--dry-run``, ``--preview`` all just work).

``rows`` defaults to ``IMAGE_HEIGHT`` (matching text and line/time mode's fixed
frame packing), but every generator accepts an explicit override -- needed for
``--mode page``, whose whole point is that the target image is *not* capped to
the 152-nozzle bar height: ``PrintController`` skips ``frames_from_ink()``
there and reaches extra rows through vertical travel instead.

Two CLI flags use these:
  * ``--calibrate``  -> :func:`ruler_pattern`, a printed ruler to measure the
    real mm/column against ``--mm-per-column``/``--dpi``.
  * ``--pattern NAME`` -> one of :data:`PATTERNS`, general bring-up patterns.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from PIL import Image

from .geometry import IMAGE_HEIGHT

# Default location drill_pattern looks for its source image: next to the
# printhead/ PACKAGE, not wherever the process happens to be running from
# (Path(__file__).resolve() anchors to this file's own on-disk location, so
# ``cd /somewhere/else && python /path/to/main.py`` still finds it). No image
# ships in this repo -- the hardware owner supplies their own here, or points
# --pattern-image at a different file (see README).
DEFAULT_DRILL_PATTERN_PATH = Path(__file__).resolve().parent.parent / "assets" / "drill_pattern.png"


def _columns(length_mm: float, mm_per_column: float) -> int:
    return max(1, round(length_mm / mm_per_column))


# ============================================================================
# --calibrate : printed ruler
# ============================================================================
def ruler_pattern(length_mm: float, mm_per_column: float,
                  major_every_mm: float = 10.0, minor_every_mm: float = 1.0,
                  rows: int = IMAGE_HEIGHT, **_) -> np.ndarray:
    """A continuous baseline plus full-height ticks every ``major_every_mm`` and
    short ticks every ``minor_every_mm`` -- print it and measure with a ruler to
    calibrate ``--mm-per-column``/``--dpi`` against the real cart motion."""
    width = _columns(length_mm, mm_per_column)
    ink = np.zeros((rows, width), dtype=bool)

    mid = rows // 2
    ink[mid, :] = True                                        # continuous baseline

    minor_half = max(1, round(rows * 0.15))
    minor_step = max(1, round(minor_every_mm / mm_per_column))
    major_step = max(1, round(major_every_mm / mm_per_column))

    for col in range(0, width, minor_step):
        ink[mid - minor_half:mid + minor_half + 1, col] = True
    for col in range(0, width, major_step):
        ink[:, col] = True                                    # full-height major tick
    return ink


# ============================================================================
# --pattern : general bring-up presets
# ============================================================================
def checkerboard_pattern(length_mm: float, mm_per_column: float,
                         square_mm: float = 10.0, square_rows: int = 20,
                         rows: int = IMAGE_HEIGHT, **_) -> np.ndarray:
    """Checkerboard tiles: catches row/column swaps and alignment errors."""
    width = _columns(length_mm, mm_per_column)
    square_cols = max(1, round(square_mm / mm_per_column))
    row_band = (np.arange(rows) // square_rows) % 2
    cols = (np.arange(width) // square_cols) % 2
    return (row_band[:, None] ^ cols[None, :]).astype(bool)


def h_stripes_pattern(length_mm: float, mm_per_column: float,
                      square_rows: int = 20, rows: int = IMAGE_HEIGHT,
                      **_) -> np.ndarray:
    """Alternating full-width row bands: each nozzle fires continuously for its
    whole band, so a dead nozzle shows as a gap along the entire length."""
    width = _columns(length_mm, mm_per_column)
    band = (np.arange(rows) // square_rows) % 2 == 0
    return np.tile(band[:, None], (1, width))


def v_stripes_pattern(length_mm: float, mm_per_column: float,
                      square_mm: float = 10.0, rows: int = IMAGE_HEIGHT,
                      **_) -> np.ndarray:
    """Alternating full-height column bands: checks column/tracking timing --
    uneven feed shows up as uneven stripe widths."""
    width = _columns(length_mm, mm_per_column)
    square_cols = max(1, round(square_mm / mm_per_column))
    band = (np.arange(width) // square_cols) % 2 == 0
    return np.tile(band[None, :], (rows, 1))


def diagonal_pattern(length_mm: float, mm_per_column: float,
                     square_mm: float = 10.0, rows: int = IMAGE_HEIGHT,
                     **_) -> np.ndarray:
    """Repeating sawtooth diagonal (period ``square_mm``): a swapped/scrambled
    nozzle row shows up as an obvious kink or jump in the line."""
    width = _columns(length_mm, mm_per_column)
    period = max(2, round(square_mm / mm_per_column))
    ink = np.zeros((rows, width), dtype=bool)
    for x in range(width):
        y = int((x % period) * (rows - 1) / (period - 1))
        ink[y, x] = True
        if y + 1 < rows:
            ink[y + 1, x] = True             # 2px thick so it prints visibly
    return ink


def solid_pattern(length_mm: float, mm_per_column: float,
                  rows: int = IMAGE_HEIGHT, **_) -> np.ndarray:
    """Solid fill: checks ink coverage / banding over a run."""
    width = _columns(length_mm, mm_per_column)
    return np.ones((rows, width), dtype=bool)


def precision_check_layout(width: int, line_cols: int = 1,
                           gap_start: int = 1) -> "list[dict]":
    """
    Column layout for :func:`precision_check_pattern` -- where each line
    (parallel to the nozzle bar) sits and how big the gap before it is.

    The gap DOUBLES after every line, so ``gap_start`` selects the whole
    progression: 1 -> 1,2,4,8,16..., 2 -> 2,4,8,16..., 4 -> 4,8,16,32...
    Lines are laid down from column 0 until the next one would no longer
    fit in ``width``; a line is never half-drawn.

    Kept separate from the ink generation on purpose: the printed result is
    only readable if you know which gap is which, so the CLI prints this
    layout as a table (in mm) alongside the pattern. Being a pure function
    of three ints it is also directly unit-testable, unlike a mask you
    would have to reverse-engineer the geometry back out of.

    Returns one dict per line, in travel order:
      * ``index``      -- 0-based line number.
      * ``start``      -- first column of the line.
      * ``cols``       -- its thickness (always ``line_cols``).
      * ``gap_before`` -- unprinted columns between the previous line and
        this one; 0 for the first line, which has nothing before it.
    """
    line_cols = max(1, int(line_cols))
    gap = max(1, int(gap_start))

    bands = []
    col = 0
    gap_before = 0
    while col + line_cols <= width:
        bands.append({"index": len(bands), "start": col,
                      "cols": line_cols, "gap_before": gap_before})
        gap_before = gap
        col += line_cols + gap
        gap *= 2
    return bands


def precision_check_pattern(length_mm: float, mm_per_column: float,
                            line_cols: int = 1, gap_start: int = 1,
                            rows: int = IMAGE_HEIGHT, **_) -> np.ndarray:
    """
    Full-height lines PARALLEL TO THE NOZZLE BAR, separated by DOUBLING
    gaps along the travel direction -- a resolution target for "how close
    can two lines get before they smear into one".

    Each line spans the whole bar height, so one line is every nozzle
    firing together for one brief moment as the cart passes that column;
    the gaps between them grow 1,2,4,8,... columns (see
    :func:`precision_check_layout` and ``gap_start``). Reading the print is
    then a single question: scan from the tight end and find the first gap
    that still shows white. That gap is the practical resolution of the
    WHOLE system ALONG TRAVEL at the speed you drove -- tracking accuracy,
    dose timing and ink spread combined -- which no single measurement in
    isolation gives you.

    Deliberately oriented across travel rather than along it: a line drawn
    along travel is one nozzle firing continuously, which measures the
    nozzle bar's own row spacing and says little about the moving parts.
    Across travel, every line is a timing/position event -- the same axis
    the position lag and the dose interval act on -- so this is the
    orientation that actually exercises the tracking.

    ``line_cols`` sets how thick each printed line is (adjacent columns
    printed together). 1 is the sharpest test; raise it if the thinnest
    lines come out too faint to judge.

    Both parameters count COLUMNS, not millimetres -- along travel the
    grid is quantised to ``mm_per_column`` (0.2mm by default, set with
    ``--mm-per-column``/``--dpi``), and that is the unit the answer lands
    on. The CLI prints the mm equivalents so the printed result can still
    be read with a ruler.
    """
    width = _columns(length_mm, mm_per_column)
    ink = np.zeros((rows, width), dtype=bool)
    for band in precision_check_layout(width, line_cols, gap_start):
        ink[:, band["start"]:band["start"] + band["cols"]] = True
    return ink


def format_precision_check_layout(bands: "list[dict]",
                                  mm_per_column: float) -> str:
    """
    Render :func:`precision_check_layout`'s output as a readable table.

    Printed by the CLI when this pattern is selected: a precision target
    whose gap sizes you cannot look up is unreadable on paper, since every
    gap looks like "some white space" once printed.

    Gaps convert to mm through ``mm_per_column`` (not the nozzle pitch):
    these lines are spaced along TRAVEL, where one grid step is one
    column.
    """
    if not bands:
        return ("[precision-check] no line fits in the requested length -- "
                "reduce --pattern-line-cols/--pattern-gap-start or raise "
                "--pattern-length-mm.")
    out = [f"[precision-check] {len(bands)} lines parallel to the nozzle bar, "
           f"{bands[0]['cols']} column(s) thick "
           f"({bands[0]['cols'] * mm_per_column:.3f} mm):",
           "  line   gap before (cols)   gap before (mm)   at col"]
    for b in bands:
        if b["gap_before"] == 0:
            gap_cols, gap_mm = "-", "-"
        else:
            gap_cols = f"{b['gap_before']}"
            gap_mm = f"{b['gap_before'] * mm_per_column:.3f}"
        out.append(f"  {b['index']:>4}   {gap_cols:>17}   {gap_mm:>15}   "
                   f"{b['start']:>6}")
    return "\n".join(out)


def drill_pattern(length_mm: float, mm_per_column: float,
                  rows: int = IMAGE_HEIGHT, pattern_image: "str | None" = None,
                  **_) -> np.ndarray:
    """Rasterise an external image (e.g. a drill/crosshair alignment target)
    to the requested physical size, instead of drawing a procedural pattern.

    ``pattern_image`` (CLI: ``--pattern-image PATH``) overrides
    :data:`DEFAULT_DRILL_PATTERN_PATH`. No image ships with this repo -- the
    hardware owner supplies their own, either at the default path or via
    ``--pattern-image`` (see README). Missing-file is therefore the COMMON
    case here, not a rare edge case, so it gets a clear, actionable error
    (SystemExit, not a raw traceback/stack dump) naming the exact path that
    was checked.
    """
    path = Path(pattern_image) if pattern_image else DEFAULT_DRILL_PATTERN_PATH
    if not path.is_file():
        raise SystemExit(
            f"printhead: error: --pattern drill_pattern needs an image, but "
            f"none was found at '{path.resolve()}'. Place an image there "
            f"(any PIL-readable format: PNG, JPG, BMP, ...), or point at a "
            f"different one with --pattern-image PATH.")
    img = Image.open(path).convert("L")

    width = _columns(length_mm, mm_per_column)
    # Deliberately resized to (width, rows) INDEPENDENTLY rather than
    # preserving the source image's own aspect ratio -- this looks like a
    # bug (the image visibly stretches/squashes on screen) but is not one:
    # a printed CELL is mm_per_column wide but NOZZLE_PITCH_MM tall, two
    # *different* physical sizes, so matching the pixel counts to the
    # requested (width, rows) -- not to the source W:H ratio -- is exactly
    # what makes the PRINTED result physically correct/square on paper.
    img = img.resize((width, rows), Image.LANCZOS)
    arr = np.asarray(img)
    # Fixed mid-grey cut-off, same default as --threshold (see
    # config.RenderSettings.threshold): the source is expected to already
    # be close to black/white, and LANCZOS resampling only blurs the
    # transition at edges, so a plain 50% split reproduces it faithfully
    # without needing a tunable parameter here too.
    return arr < 128


PATTERNS = {
    "checkerboard": checkerboard_pattern,
    "h-stripes": h_stripes_pattern,
    "v-stripes": v_stripes_pattern,
    "diagonal": diagonal_pattern,
    "solid": solid_pattern,
    "precision-check": precision_check_pattern,
    "drill_pattern": drill_pattern,
}
