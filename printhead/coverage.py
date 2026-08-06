"""
Freehand coverage / per-nozzle dose engine
============================================

The 1D pipeline fires a column once, exactly when the head crosses it, and
never again (a monotonic ``frontier`` index -- see ``controller.py``). Free
movement breaks that: the cart can revisit the same spot, loop, or sweep
vertically to reach rows beyond the fixed 152-nozzle span. This module
replaces the frontier with a ``printed`` coverage mask and a per-nozzle dose
tracker, so "is this pixel done yet" is a live question answered every
sample rather than something decided once and forgotten.

Ink is dosed per *nozzle* (152 trackers), not per pixel -- a nozzle remembers
only the one pixel it is currently over and how long it has continuously
been there. It fires (bit=1 in the returned pattern) whenever that pixel is
wanted and not yet printed; once the dwell has lasted ``dose_hold_s``, the
pixel is marked printed and the nozzle stops firing for it. If a nozzle
leaves before that, the pixel simply stays unprinted -- a later pass (by
this nozzle, or by a different one, if vertical movement changed which
nozzle covers that row) catches it up. This gives full coverage under loops
and revisits without ever accumulating dose across pixels.

This assumes the nozzle bar stays perpendicular to the page's column axis
(``e_col``) -- no cart-rotation correction yet. The orientation quaternion
is confirmed available on real hardware (see ``tracking.AmfitrackTracker.
_extract_pose``) and is the intended eventual source for that correction,
but it needs a measured boresight offset first (``PageCalibration.
boresight_quat`` exists as a field but nothing yet has a procedure to
measure it) -- deliberately deferred rather than half-wired in here.
"""

from __future__ import annotations

from typing import List, Optional, Tuple

import numpy as np

from .geometry import NOZZLE_PITCH_MM, NUM_NOZZLES
from .rendering import pack_nozzle_bits

# Minimum continuous time (s) a nozzle must stay over a not-yet-printed ink
# pixel before that pixel counts as fully dosed -- the client-side analogue
# of the firmware's BLE_DOSE_HOLD_FACTOR, preventing a slow/stalled nozzle
# from firing forever. A first guess, not yet tuned against real hardware:
# every dose constant in this project so far (BLE_FIRE_MIN/MAX etc.) needed
# several rounds of hardware iteration before it was right, and this one
# almost certainly will too.
DEFAULT_DOSE_HOLD_S = 0.05

_Pixel = Optional[Tuple[int, int]]


class CoverageEngine:
    """
    Tracks what has been printed onto a target image -- possibly taller than
    the 152-nozzle bar, reached via vertical travel -- as the cart moves
    freely over the page.

    ``ink``/``printed`` are ``(H, W)`` bool arrays: ``ink`` is the target
    (never modified), ``printed`` is the live coverage mask this builds up.
    """

    def __init__(self, ink: np.ndarray, mm_per_column: float,
                 dose_hold_s: float = DEFAULT_DOSE_HOLD_S):
        ink = np.asarray(ink, dtype=bool)
        if ink.ndim != 2:
            raise ValueError(f"ink must be a 2-D (H, W) array, got shape {ink.shape}")
        self.ink = ink
        self.printed = np.zeros_like(ink, dtype=bool)
        self.mm_per_column = mm_per_column
        self.dose_hold_s = dose_hold_s

        # Per-nozzle dwell state: which pixel each nozzle is currently over
        # (None if not over a wanted, unprinted pixel) and when it arrived.
        self._nozzle_pixel: List[_Pixel] = [None] * NUM_NOZZLES
        self._nozzle_since: List[Optional[float]] = [None] * NUM_NOZZLES
        self._last_pattern: Optional[bytes] = None

        # Whether the most recent step() had ANY nozzle in bounds of the
        # target image. A fully out-of-page pass (cart never over the page
        # at all) and a fully-covered pass both end up with an all-zero
        # `active` pattern and no further `changed` updates -- from the
        # pattern alone a caller cannot tell "nothing left to do" from
        # "nothing here was ever reachable". This is the cheap signal that
        # lets a caller (controller._print_freehand_pass) tell them apart
        # and warn instead of silently finishing with 0 pixels covered.
        self.last_in_bounds: bool = False

    @property
    def height(self) -> int:
        return self.ink.shape[0]

    @property
    def width(self) -> int:
        return self.ink.shape[1]

    @property
    def done(self) -> bool:
        """True once every wanted ink pixel has been printed."""
        return bool(np.all(self.printed[self.ink]))

    def step(self, u_mm: float, v_mm: float, t: float) -> "Tuple[bytes, bool]":
        """
        Advance the engine with one live page-plane sample (see
        ``tracking.PageMapper.project``) at time ``t`` (seconds; only
        differences between calls matter, so ``time.monotonic()`` is fine).

        Returns ``(pattern, changed)``: ``pattern`` is the current 19-byte /
        152-bit nozzle frame (same format as ``rendering.frames_from_ink``),
        and ``changed`` is True if it differs from the pattern returned by
        the previous call, so a caller only needs to send a BLE update when
        this is True.

        Also updates ``self.last_in_bounds`` as a side effect (see its
        docstring) -- kept as an attribute rather than a third return value
        so this signature stays untouched for existing callers/tests.
        """
        col = int(round(u_mm / self.mm_per_column))
        base_row = int(round(v_mm / NOZZLE_PITCH_MM))

        active = np.zeros(NUM_NOZZLES, dtype=bool)
        in_bounds_any = False
        for p in range(NUM_NOZZLES):
            row = base_row + p
            in_bounds = 0 <= row < self.height and 0 <= col < self.width
            if in_bounds:
                in_bounds_any = True
            wanted = in_bounds and bool(self.ink[row, col]) and not self.printed[row, col]

            if not wanted:
                self._nozzle_pixel[p] = None
                self._nozzle_since[p] = None
                continue

            pixel = (row, col)
            if self._nozzle_pixel[p] != pixel:
                self._nozzle_pixel[p] = pixel
                self._nozzle_since[p] = t

            active[p] = True
            if t - self._nozzle_since[p] >= self.dose_hold_s:
                self.printed[row, col] = True

        self.last_in_bounds = in_bounds_any
        pattern = pack_nozzle_bits(active)
        changed = pattern != self._last_pattern
        self._last_pattern = pattern
        return pattern, changed
