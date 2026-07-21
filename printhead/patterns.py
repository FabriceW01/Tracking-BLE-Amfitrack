"""
Printable test patterns
========================

Generates ``(IMAGE_HEIGHT, W)`` boolean ink masks, exactly like
``rendering.render_text``, so they flow through the same framing/BLE/tracking
pipeline as text (position or time mode, ``--simulate``, ``--dry-run``,
``--preview`` all just work).

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
                  **_) -> np.ndarray:
    """A continuous baseline plus full-height ticks every ``major_every_mm`` and
    short ticks every ``minor_every_mm`` -- print it and measure with a ruler to
    calibrate ``--mm-per-column``/``--dpi`` against the real cart motion."""
    width = _columns(length_mm, mm_per_column)
    ink = np.zeros((IMAGE_HEIGHT, width), dtype=bool)

    mid = IMAGE_HEIGHT // 2
    ink[mid, :] = True                                        # continuous baseline

    minor_half = max(1, round(IMAGE_HEIGHT * 0.15))
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
                         **_) -> np.ndarray:
    """Checkerboard tiles: catches row/column swaps and alignment errors."""
    width = _columns(length_mm, mm_per_column)
    square_cols = max(1, round(square_mm / mm_per_column))
    rows = (np.arange(IMAGE_HEIGHT) // square_rows) % 2
    cols = (np.arange(width) // square_cols) % 2
    return (rows[:, None] ^ cols[None, :]).astype(bool)


def h_stripes_pattern(length_mm: float, mm_per_column: float,
                      square_rows: int = 20, **_) -> np.ndarray:
    """Alternating full-width row bands: each nozzle fires continuously for its
    whole band, so a dead nozzle shows as a gap along the entire length."""
    width = _columns(length_mm, mm_per_column)
    band = (np.arange(IMAGE_HEIGHT) // square_rows) % 2 == 0
    return np.tile(band[:, None], (1, width))


def v_stripes_pattern(length_mm: float, mm_per_column: float,
                      square_mm: float = 10.0, **_) -> np.ndarray:
    """Alternating full-height column bands: checks column/tracking timing --
    uneven feed shows up as uneven stripe widths."""
    width = _columns(length_mm, mm_per_column)
    square_cols = max(1, round(square_mm / mm_per_column))
    band = (np.arange(width) // square_cols) % 2 == 0
    return np.tile(band[None, :], (IMAGE_HEIGHT, 1))


def diagonal_pattern(length_mm: float, mm_per_column: float,
                     square_mm: float = 10.0, **_) -> np.ndarray:
    """Repeating sawtooth diagonal (period ``square_mm``): a swapped/scrambled
    nozzle row shows up as an obvious kink or jump in the line."""
    width = _columns(length_mm, mm_per_column)
    period = max(2, round(square_mm / mm_per_column))
    ink = np.zeros((IMAGE_HEIGHT, width), dtype=bool)
    for x in range(width):
        y = int((x % period) * (IMAGE_HEIGHT - 1) / (period - 1))
        ink[y, x] = True
        if y + 1 < IMAGE_HEIGHT:
            ink[y + 1, x] = True             # 2px thick so it prints visibly
    return ink


def solid_pattern(length_mm: float, mm_per_column: float, **_) -> np.ndarray:
    """Solid fill: checks ink coverage / banding over a run."""
    width = _columns(length_mm, mm_per_column)
    return np.ones((IMAGE_HEIGHT, width), dtype=bool)


PATTERNS = {
    "checkerboard": checkerboard_pattern,
    "h-stripes": h_stripes_pattern,
    "v-stripes": v_stripes_pattern,
    "diagonal": diagonal_pattern,
    "solid": solid_pattern,
}
