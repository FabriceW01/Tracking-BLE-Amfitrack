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
    "drill_pattern": drill_pattern,
}
