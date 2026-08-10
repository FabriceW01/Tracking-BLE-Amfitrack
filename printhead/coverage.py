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

Cart yaw about the page normal is corrected here on a per-nozzle basis: with
the bar rotated by ``yaw_rad``, nozzle ``p`` is no longer at the same column
as nozzle 0 (see ``step()``'s ``yaw_rad`` parameter). Measured from a real
pass (``pass5.csv``), yaw spans 75.6 deg, which spreads the 15mm bar across
~14.5mm (~72 columns at 0.2 mm/col) -- ignoring that put nozzles that were
nowhere near a wanted pixel into the fired pattern, and vice versa. Only yaw
is corrected: tilt (pitch/roll) is small by comparison in the same data
(median 2.7 deg, max 7.8 deg) and is a deliberate, measured non-goal, not an
oversight -- see ``rotation.yaw_about_normal``'s docstring for the yaw
extraction itself and ``tracking.PageMapper`` for the matching lever-arm
correction (the sensor->nozzle-bar offset is a cart-frame vector, not a
page-frame constant, for the same reason).
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

import numpy as np

from .geometry import NOZZLE_PITCH_MM, NUM_NOZZLES
from .rendering import pack_nozzle_bits

# Minimum continuous time (s) a nozzle must stay over a not-yet-printed ink
# pixel before that pixel counts as fully dosed -- the client-side analogue
# of the firmware's BLE_DOSE_HOLD_FACTOR, preventing a slow/stalled nozzle
# from firing forever.
#
# Measured once against a real 200x100 mm checkerboard print. The old value
# (0.05 s = 50 ms) required the cart to be slower than 0.2/0.05 = 4 mm/s for
# a pixel to ever be marked printed -- but the real hand speed was median
# 17.3 mm/s / p95 46.1 mm/s, so only 5.5% of samples were ever that slow, and
# the finished pass reported "Covered 224/503500 ink pixels" = 0.044%. With
# `printed` staying False almost everywhere, every revisit re-fired the same
# pixels at a slightly different hand position -- CoverageEngine's whole
# purpose (tolerating imprecise freehand repositioning without double-firing
# a covered pixel) was non-functional, and that repeated re-firing is what
# produced the ghosting/doubling visible on paper.
#
# This value is derived from, and MUST be kept in sync with, the firmware's
# PATTERN_STRIDE (src/ble_dose.h in the firmware repo):
#   DEFAULT_DOSE_HOLD_S ~= 3 * PATTERN_STRIDE * 450e-6
# (450 us = the firmware print loop tick; 3 = BLE_DROPS_PER_COLUMN, line
# mode's long-validated per-column dose target). At PATTERN_STRIDE = 3 that
# is 3 * 3 * 450e-6 = 0.00405 s, i.e. a pixel gets ~3 drops before the
# coverage mask marks it printed. Changing this constant without changing
# PATTERN_STRIDE to match (and re-flashing) breaks that ~3-drop target: a
# shorter hold with an unchanged stride gives fewer than 3 drops, a longer
# hold gives more. See tests/test_coverage.py for a test that pins this
# relationship so an edit to one side fails loudly here.
#
# CORRECTION (this value replaces an earlier 0.0054 s / PATTERN_STRIDE=4
# pick that also hit the 3-drop target but ignored polling quantization and
# landed on a cliff): `step()` only marks a pixel printed on a *sample* that
# finds elapsed dwell >= dose_hold_s, where "elapsed" is measured from the
# first sample on that pixel -- so completing a dose costs whole poll
# intervals, not continuous time. With the default --poll-hz 200 (5.00 ms
# interval), a hold just *above* one interval (the old 5.4 ms) forces a
# THIRD sample to land on the same column before it completes, because a
# second sample at +5.00 ms is still short of 5.4 ms. Measured directly by
# sweeping dose_hold_s at poll_hz=200 over a realistic hand-speed pass:
#   dose_hold  4.90 ms -> 100.0 % coverage
#   dose_hold  5.40 ms ->  31.0 %   <-- the value that shipped in the first pass
#   dose_hold  7.00 ms ->  31.0 %
#   dose_hold 10.00 ms ->   6.5 %
# i.e. crossing the poll interval does not degrade coverage gracefully, it
# collapses it. The additional constraint this adds, on top of the ~3-drop
# target above: dose_hold_s MUST stay below the poll interval (1/poll_hz),
# so that two consecutive samples are always enough to complete a dose --
# cross that line and coverage falls off a cliff rather than sloping down.
# 0.00405 s is 19% below the 5.00 ms default poll interval, giving some
# margin rather than sitting on the edge again.
#
# At the measured median hand speed (17.3 mm/s) a full simulated pass at
# poll_hz=200 gives 100% coverage with this default; coverage falls off
# above that as hand speed rises (60% at 25 mm/s, 14% at 35 mm/s, 0% at
# 46 mm/s) because fewer poll samples land on each column before the hand
# moves on. That falloff is BY DESIGN, not a bug -- an unfinished pixel
# simply stays open for a later pass (see CoverageEngine's docstring) -- but
# it means this default is not universally sufficient at all hand speeds;
# it is tuned to the measured median, not the tail.
#
# Like PATTERN_STRIDE, this is a measured-once value from one real print at
# ~17 mm/s median hand speed, not a finished calibration -- a large,
# evidence-based improvement on the previous untested guess, but still
# expect iteration once more prints are measured, the same way every other
# dose constant in this project (BLE_FIRE_MIN/MAX etc.) needed several
# rounds of hardware iteration before it was right.
DEFAULT_DOSE_HOLD_S = 0.00405

_Pixel = Optional[Tuple[int, int]]


def bar_offset_uv(offset_along_bar_mm: float, yaw_rad: float) -> Tuple[float, float]:
    """
    ``(du, dv)`` from nozzle 0's ``(u, v)`` to a point ``offset_along_bar_mm``
    further along the bar's CURRENT (yaw-rotated) direction.

    Mirrors -- but is deliberately NOT called by -- the per-nozzle formula
    inside :meth:`CoverageEngine.step`'s hot loop (nozzle ``p`` sits at this
    evaluated at ``offset_along_bar_mm = p * NOZZLE_PITCH_MM``): that loop
    runs all ``NUM_NOZZLES`` nozzles on every sample, up to ``poll_hz`` times
    a second, and the "zero_yaw fast path" right above it already exists
    purely to avoid a cheaper equivalent of the per-call overhead a shared
    function here would add across 152 calls/sample -- not worth the
    dedup for a formula this short. This standalone copy exists for a
    caller that evaluates it only ONCE per sample instead: ``controller.
    _print_freehand_pass``'s ``--record`` path tracking, which needs the
    nozzle BAR's centre (a single point), not any specific nozzle.

    See ``step``'s "Sign derivation" docstring note for the derivation this
    must stay bit-for-bit consistent with; ``tests/test_coverage.py``
    cross-checks that consistency directly (this function evaluated at the
    bar's centre offset must match ``step()``'s own placement of the centre
    nozzle).
    """
    sin_yaw = math.sin(yaw_rad)
    cos_yaw = math.cos(yaw_rad)
    return (-offset_along_bar_mm * sin_yaw, offset_along_bar_mm * cos_yaw)


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

    def step(self, u_mm: float, v_mm: float, t: float,
             yaw_rad: float = 0.0) -> "Tuple[bytes, bool]":
        """
        Advance the engine with one live page-plane sample (see
        ``tracking.PageMapper.project``) at time ``t`` (seconds; only
        differences between calls matter, so ``time.monotonic()`` is fine).

        ``yaw_rad`` is the cart's current yaw about the page normal (see
        ``rotation.yaw_about_normal``, computed once per sample by the
        caller -- see ``controller._print_freehand_pass``), 0.0 by default
        (no rotation correction, e.g. no boresight captured for this pass's
        calibration). With the bar rotated by ``yaw_rad``, nozzle ``p`` sits
        at its own ``(row, col)`` -- offset ``p * NOZZLE_PITCH_MM`` along the
        bar's CURRENT direction from nozzle 0 -- rather than sharing nozzle
        0's column the way a zero-yaw bar (perpendicular to travel) does.

        Sign derivation (check this before ever touching it again): in the
        right-handed page basis ``{e_col, e_row, n}`` with ``n = e_col x
        e_row``, a rotation by ``+theta`` about ``n`` maps an in-plane vector
        ``(a, b)`` to ``(a*cos(theta) - b*sin(theta), a*sin(theta) +
        b*cos(theta))``. At ``yaw_rad == 0`` the bar points along ``+e_row``
        -- that follows directly from the ``row = base_row + p`` convention
        below (increasing nozzle index ``p`` -> increasing ``row`` ->
        increasing ``v``), i.e. the bar's direction vector is ``(0, 1)`` in
        ``(u, v)``. Rotating ``(0, 1)`` by ``+theta`` gives ``(-sin(theta),
        cos(theta))``, so nozzle ``p`` at distance ``d = p * NOZZLE_PITCH_MM``
        along the bar sits at ``u_p = u_mm - d*sin(yaw_rad)``, ``v_p = v_mm +
        d*cos(yaw_rad)`` -- MINUS on the u term. This must match
        ``tracking.PageMapper``'s offset rotation (``du = col_offset*cos(yaw)
        - row_offset*sin(yaw)``, same minus sign on the sin term for a
        row-offset vector): both describe body-fixed vectors on the same
        cart, so both must rotate the same way, or a rotating pass has the
        bar-tilt correction here pulling one way while the lever-arm
        correction in PageMapper pulls the other -- see
        ``tests/test_coverage.py``'s cross-consistency test against
        PageMapper, which exists specifically to catch that.

        Returns ``(pattern, changed)``: ``pattern`` is the current 19-byte /
        152-bit nozzle frame (same format as ``rendering.frames_from_ink``),
        and ``changed`` is True if it differs from the pattern returned by
        the previous call, so a caller only needs to send a BLE update when
        this is True.

        Also updates ``self.last_in_bounds`` as a side effect (see its
        docstring) -- kept as an attribute rather than a third return value
        so this signature stays untouched for existing callers/tests.
        """
        # yaw_rad == 0.0 gets its own exact-integer path: col/row for every
        # nozzle must be BIT-IDENTICAL to the pre-rotation formula (a single
        # shared `col`, `row = base_row + p`), not merely close to it, since
        # every coverage test written before this change depends on that
        # exact behaviour. Mathematically sin(0)=0/cos(0)=1 already collapse
        # the general per-nozzle formula below to the same u_p/v_p -- but
        # recomputing row_p as round(v_p / NOZZLE_PITCH_MM) from a
        # floating-point v_p = v_mm + p*NOZZLE_PITCH_MM, instead of the exact
        # integer base_row + p, can drift by one nozzle purely from the extra
        # add-then-divide rounding, which is exactly the drift this fast path
        # exists to avoid.
        zero_yaw = yaw_rad == 0.0
        if zero_yaw:
            col_fixed = int(round(u_mm / self.mm_per_column))
            base_row = int(round(v_mm / NOZZLE_PITCH_MM))
        else:
            sin_yaw = math.sin(yaw_rad)
            cos_yaw = math.cos(yaw_rad)

        active = np.zeros(NUM_NOZZLES, dtype=bool)
        in_bounds_any = False
        for p in range(NUM_NOZZLES):
            if zero_yaw:
                col = col_fixed
                row = base_row + p
            else:
                offset_along_bar = p * NOZZLE_PITCH_MM
                u_p = u_mm - offset_along_bar * sin_yaw
                v_p = v_mm + offset_along_bar * cos_yaw
                col = int(round(u_p / self.mm_per_column))
                row = int(round(v_p / NOZZLE_PITCH_MM))

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
