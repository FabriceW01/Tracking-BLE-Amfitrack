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
pass (``pass5.csv``), yaw spans 75.6 deg, which spreads the 15.1mm nozzle-0-
to-nozzle-151 span (``NOZZLE_BAR_SPAN_MM``, see geometry.py) across
~14.6mm (~73 columns at 0.2 mm/col) -- ignoring that put nozzles that were
nowhere near a wanted pixel into the fired pattern, and vice versa. Only yaw
is corrected: tilt (pitch/roll) is small by comparison in the same data
(median 2.7 deg, max 7.8 deg) and is a deliberate, measured non-goal, not an
oversight -- see ``rotation.yaw_about_normal``'s docstring for the yaw
extraction itself and ``tracking.PageMapper`` for the matching lever-arm
correction (the sensor->nozzle-bar offset is a cart-frame vector, not a
page-frame constant, for the same reason).

Nozzles can optionally be tied together into fixed-size groups (``__init__``'s
``nozzle_group``, CLI ``--nozzle-group``): with ``nozzle_group = N``, nozzles
``N*k .. N*k+N-1`` form group ``k`` and always fire together or not at all --
see ``step()`` for the exact per-group OR/dwell/deposit rule. This is a
coarser-vertical-addressing option requested by the hardware owner, nothing
more: it is NOT a fix for the repeated-overprint behaviour ``spray_radius_mm``
/ ``spray_strength`` above address, and firing nozzles in pairs does not
meaningfully change ``step()``'s own CPU cost (measured ~46.9us/call = 2.3%
of a core at 500Hz) -- the per-nozzle loop below still runs all 152 nozzles
either way, just grouped. Only ``N in (1, 2)`` is supported; ``N = 1``
reduces to exactly the ungrouped behaviour described above.
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
                 dose_hold_s: float = DEFAULT_DOSE_HOLD_S,
                 spray_radius_mm: float = 0.0,
                 spray_strength: float = 0.0,
                 nozzle_group: int = 1):
        ink = np.asarray(ink, dtype=bool)
        if ink.ndim != 2:
            raise ValueError(f"ink must be a 2-D (H, W) array, got shape {ink.shape}")
        if nozzle_group < 1:
            raise ValueError(f"nozzle_group must be >= 1, got {nozzle_group}")
        self.ink = ink
        self.printed = np.zeros_like(ink, dtype=bool)
        self.mm_per_column = mm_per_column
        self.dose_hold_s = dose_hold_s
        self.spray_radius_mm = spray_radius_mm
        self.spray_strength = spray_strength
        # Ties nozzles nozzle_group*k .. nozzle_group*k+nozzle_group-1 into
        # group k, which always fires together or not at all -- see step()
        # for the exact rule. 1 (default) = today's behaviour, every nozzle
        # addressed individually; only 1 and 2 are supported by the CLI, but
        # nothing below assumes NUM_NOZZLES divides evenly by this, so an
        # untested N is at worst a short trailing group, never a crash.
        self.nozzle_group = nozzle_group

        # Fractional dose per pixel, 0.0 .. 1.0 (1.0 == printed). Without
        # spray this only ever holds 0.0/1.0 and `printed` is simply its
        # threshold, i.e. behaviour is bit-identical to the pre-spray
        # engine; with spray it also carries the partial doses neighbours
        # pick up from a drop landing next to them (see _build_spray_kernel).
        self.dose = np.zeros(ink.shape, dtype=np.float32)
        self._spray_kernel = self._build_spray_kernel()

        # Per-GROUP dwell state: which pixel each group is currently over
        # (None if the group is not wanted -- see step()) and when it
        # arrived, keyed on the group's FIRST member's (row, col). At
        # nozzle_group=1 a group is exactly one nozzle, so this is the same
        # per-nozzle state the engine has always kept (bit-identical
        # behaviour -- see step()'s docstring).
        self._num_groups = -(-NUM_NOZZLES // nozzle_group)   # ceil division
        self._group_pixel: List[_Pixel] = [None] * self._num_groups
        self._group_since: List[Optional[float]] = [None] * self._num_groups
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

    def _build_spray_kernel(self) -> "List[Tuple[int, int, float]]":
        """
        Precompute the ``(d_row, d_col, weight)`` splat a single completed
        drop deposits around itself -- an "ink spread" / dot-gain model.

        A real thermal-inkjet drop does not land inside exactly one grid
        cell: it wets a small area around where it was aimed. Without
        modelling that, a return pass whose ``v`` sits a fraction of a
        millimetre off the outbound one addresses DIFFERENT row indices,
        finds them unprinted, and fires again over paper that already has
        ink on it -- the repeated over-printing seen on the rig when driving
        back and forth over the same strip.

        The radius is given in MILLIMETRES, not pixels, and converted per
        axis, because the grid is strongly anisotropic: a cell is
        ``NOZZLE_PITCH_MM`` (0.1mm) tall but ``mm_per_column`` (0.2mm by
        default) wide, so a physically round drop is about 2:1 elliptical in
        pixel space. A pixel-count radius would silently mean two different
        physical distances on the two axes.

        Weight falls off linearly with physical distance, NORMALISED so the
        CLOSEST neighbour in the kernel receives exactly ``spray_strength``
        and the falloff reaches 0 at ``spray_radius_mm``. That normalisation
        is what makes the parameter mean something operationally: at
        ``spray_strength = 1.0`` an immediately adjacent pixel is treated as
        fully printed (dose 1.0) by a single drop, so a return pass drifting
        one row over will not fire it again; at 0.5 it takes two drops.

        Without the normalisation ``strength`` would scale a raw
        ``1 - d/radius`` term, and since the nearest neighbour already sits
        about half a radius out, even ``strength = 1.0`` could only ever give
        it ~0.5 -- never enough to complete a pixel from spray alone, and
        measurably useless for the repeat-printing case this exists to fix
        (verified: it changed the per-pass firing counts by exactly zero).

        The centre itself is not in the kernel -- it always receives a full
        1.0 dose from the dwell that completed it (see ``_deposit``).

        Returns an empty kernel (spray disabled, behaviour identical to the
        pre-spray engine) when either parameter is <= 0.
        """
        if self.spray_radius_mm <= 0.0 or self.spray_strength <= 0.0:
            return []
        r_rows = int(self.spray_radius_mm / NOZZLE_PITCH_MM)
        r_cols = int(self.spray_radius_mm / self.mm_per_column) if self.mm_per_column else 0
        raw = []
        for dr in range(-r_rows, r_rows + 1):
            for dc in range(-r_cols, r_cols + 1):
                if dr == 0 and dc == 0:
                    continue
                d_mm = math.hypot(dr * NOZZLE_PITCH_MM, dc * self.mm_per_column)
                if d_mm > self.spray_radius_mm:
                    continue           # outside the round drop, not the box
                f = 1.0 - d_mm / self.spray_radius_mm
                if f > 0.0:
                    raw.append((dr, dc, f))
        if not raw:
            return []
        f_max = max(f for _, _, f in raw)      # the closest neighbour's falloff
        return [(dr, dc, self.spray_strength * f / f_max) for dr, dc, f in raw]

    def _deposit(self, row: int, col: int) -> None:
        """Land a completed drop at ``(row, col)``: full dose at the centre,
        plus the spray kernel's partial doses around it. Any pixel whose
        accumulated dose reaches 1.0 counts as printed.

        Called exactly once per completing pixel: the centre always reaches
        1.0 here, so the pixel is `printed` on return, which makes it
        unwanted and releases the nozzle -- it cannot deposit twice."""
        h, w = self.ink.shape
        self.dose[row, col] = 1.0
        self.printed[row, col] = True
        if not self._spray_kernel:
            return
        for dr, dc, weight in self._spray_kernel:
            r, c = row + dr, col + dc
            if 0 <= r < h and 0 <= c < w and not self.printed[r, c]:
                acc = self.dose[r, c] + weight
                if acc >= 1.0:
                    self.dose[r, c] = 1.0
                    self.printed[r, c] = True
                else:
                    self.dose[r, c] = acc

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

        ``self.nozzle_group`` (default 1, i.e. off) ties nozzles into
        fixed-size groups that fire and dose as one unit, for coarser
        vertical addressing -- unrelated to (and not to be confused with)
        ``NozzleMapSettings``/``--nozzle-block-size``+``--nozzle-order``,
        which permutes which image ROW an individually-addressed nozzle
        receives, for a rig wired out of order; grouping changes nothing
        about row order, it only ties adjacent nozzles' firing together.
        With ``nozzle_group = N``, nozzles ``N*k .. N*k+N-1`` form group
        ``k``. Each member's own ``(row, col)`` and ``wanted`` are computed
        exactly as in the ungrouped case above (under yaw, members can
        legitimately land in different columns -- that is expected, not a
        bug, and is exercised under yaw the same way the ungrouped path is).
        Per group:

          * the group is wanted if ANY member is wanted (OR) -- so a group
            never skips a pixel that still needs ink; the cost is that a
            group straddling an ink/no-ink boundary also inks the no-ink
            side (it cannot fire only half of itself).
          * if wanted, every member's ``active[p]`` is set True -- the
            defining behaviour: they always fire together.
          * dwell is tracked ONCE per group (not per nozzle), keyed on the
            group's FIRST member's ``(row, col)``; the key resets whenever
            that pixel changes or the group stops being wanted, same as the
            per-nozzle key does in the ungrouped case.
          * on completion, ``_deposit`` is called for EVERY in-bounds member
            (not just the first) -- every member physically received ink, so
            every member's pixel must be marked printed.

        At ``nozzle_group = 1`` every group has exactly one member, so all of
        the above collapses to precisely the ungrouped per-nozzle rule --
        this is what makes ``N = 1`` bit-identical to the engine's original
        behaviour rather than a special case of it.
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
        group_size = self.nozzle_group
        for k in range(self._num_groups):
            lo = k * group_size
            hi = min(lo + group_size, NUM_NOZZLES)

            # Step 1+2: every member's own (row, col)/wanted, exactly as the
            # ungrouped formula above; group_wanted is the OR across members
            # (see step()'s docstring for the group_size=1 collapse and the
            # OR-rule's boundary-fattening tradeoff).
            members = []                      # (p, row, col, in_bounds)
            group_wanted = False
            for p in range(lo, hi):
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
                members.append((p, row, col, in_bounds))
                group_wanted = group_wanted or wanted

            if not group_wanted:
                self._group_pixel[k] = None
                self._group_since[k] = None
                continue

            # Step 4: dwell tracked once per group, keyed on the FIRST
            # member's (row, col) -- at group_size=1 this is that one
            # nozzle's own (row, col), same key the ungrouped code used.
            first_row, first_col = members[0][1], members[0][2]
            pixel = (first_row, first_col)
            if self._group_pixel[k] != pixel:
                self._group_pixel[k] = pixel
                self._group_since[k] = t

            # Step 3: the whole group fires together.
            for p, _row, _col, _in_bounds in members:
                active[p] = True

            # Step 5: on completion, every in-bounds member actually
            # received ink and must be marked printed -- not just the first.
            if t - self._group_since[k] >= self.dose_hold_s:
                for _p, row, col, in_bounds in members:
                    if in_bounds:
                        self._deposit(row, col)

        self.last_in_bounds = in_bounds_any
        pattern = pack_nozzle_bits(active)
        changed = pattern != self._last_pattern
        self._last_pattern = pattern
        return pattern, changed
