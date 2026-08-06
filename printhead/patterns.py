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

import numpy as np

from .geometry import IMAGE_HEIGHT


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


PATTERNS = {
    "checkerboard": checkerboard_pattern,
    "h-stripes": h_stripes_pattern,
    "v-stripes": v_stripes_pattern,
    "diagonal": diagonal_pattern,
    "solid": solid_pattern,
}
