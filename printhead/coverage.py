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
only the one pixel it is currently over and how much of a dose that pixel has
received. It fires (bit=1 in the returned pattern) whenever that pixel is
wanted and still owed ink; once ``drops_per_pixel`` drops have actually gone
out for it, the nozzle stops firing for it. Note "gone out", not "elapsed":
the firmware fires each column it receives exactly once and never repeats, so
ink is counted in drops the client really sent, not in time a bit was held
(see ``DEFAULT_DROPS_PER_PIXEL``). If a nozzle leaves before that, the pixel
simply stays unprinted -- a later pass (by this nozzle, or by a different
one, if vertical movement changed which nozzle covers that row) catches it
up. This gives full coverage under loops and revisits without ever
accumulating dose across pixels.

``printed`` is set a hair earlier than the nozzle is released, by exactly one
poll sample; ``step()``'s Step 5 has the measurement that forces those two to
be separate thresholds.

Cart yaw about the page normal is corrected here on a per-nozzle basis: with
the bar rotated by ``yaw_rad``, nozzle ``p`` is no longer at the same column
as nozzle 0 (see ``step()``'s ``yaw_rad`` parameter). Measured from a real
pass (``pass5.csv``), yaw spans 75.6 deg, which spreads the ~13.1mm nozzle-0-
to-nozzle-151 span (``NOZZLE_BAR_SPAN_MM``, see geometry.py) across
~12.7mm (~63 columns at 0.2 mm/col) -- ignoring that put nozzles that were
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
see ``step()`` for the exact per-group OR/drop/deposit rule. This is a
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

# How many drops a pixel must receive before it counts as fully dosed.
#
# This replaced a time-based hold (``DEFAULT_DOSE_HOLD_S``) when the firmware
# changed its page-mode firing model. The old firmware HELD the last written
# pattern and re-fired it every ``PATTERN_STRIDE`` ticks, so ink was set by
# how LONG the client held a nozzle bit at 1, and the two sides were coupled
# by ``DOSE_HOLD_S ~= 3 * PATTERN_STRIDE * tick``. The firmware now fires each
# column it receives EXACTLY ONCE and never repeats (``PATTERN_STRIDE`` and
# ``pattern_dose_should_fire()`` are gone from ble_dose.h), so there is no
# repeat rate left to hold against: ink is decided entirely on this side, by
# how many copies of a column the client queues.
#
# Counting drops instead of seconds is therefore not a refactor but the only
# model that still corresponds to the hardware. It also removes the old
# model's sharpest edge -- ``dose_hold_s`` had to stay below the poll interval
# or coverage collapsed off a cliff (measured: 100% at 4.90 ms against 31% at
# 5.40 ms with a 5.00 ms interval) -- because a drop count has no relationship
# to the polling rate at all.
#
# CORRECTION: this shipped as 3, copied from line mode's firmware constant
# ``BLE_DROPS_PER_COLUMN``, and it over-inked real prints by exactly that
# factor -- reported from hardware as "jetzt kommt zu viel raus", against a
# pre-conversion print that came out lighter AND sharper.
#
# The mistake was treating 3 as a property of the ink when it is a property
# of the COLUMN WIDTH it was validated at. What the paper cares about is
# drops per millimetre of travel, and this constant is divided by
# ``mm_per_column`` to get there:
#
#     drops/mm = drops_per_pixel / mm_per_column
#     3 / 0.200 = 15.0     line mode's validated 0.2 mm column
#     3 / 0.087 = 34.5     the same 3 at today's 0.087 mm column  <- 2.3x
#     1 / 0.087 = 11.5     this default
#
# 11.5 drops/mm is not a guess: it is exactly what the PRE-conversion client
# delivered at low speed, i.e. the density the operator has actually judged
# on paper. Simulated against the old "send only on pattern change" rule on
# the fire-once firmware, a slow pass put down 120 columns over 120 columns
# of travel -- 1.00 drops per column, 11.5 per mm. (It fell off a cliff
# above that: 0.51 drops/pixel at 17.3 mm/s with 49% of pixels getting no
# ink at all, 0.01 at 25 mm/s. Reproducing its DENSITY, not its
# speed-dependence, is the goal.)
#
# The physics agrees: a drop spreads to roughly 60-120 um, so one drop
# already covers an 87 um column. Three only made sense for a column two to
# three times wider.
#
# Still a first calibration, not a finished one -- raise it if a print comes
# out faint. Fractional values are accepted precisely so this can be tuned
# in both directions without the column width having to move with it.
#
# Who supplies ``drops``: ``PrintController._print_freehand_pass`` converts
# the cart's travel since the last send into a number of copies to queue
# (``drops = DROPS_PER_PIXEL * travel / mm_per_column``, carried across
# samples in an accumulator so no fraction is lost) and passes the same count
# to ``step()``. Ink per pixel is then independent of hand speed by
# construction: move twice as fast and twice as many copies go out per
# second, in half the time over the same pixel.
DEFAULT_DROPS_PER_PIXEL = 1.0

# Slack on the "is this pixel fully dosed" comparison -- see step()'s Step 5
# for why a float dose needs one at all.
_DOSE_EPS = 1e-9


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
                 drops_per_pixel: float = DEFAULT_DROPS_PER_PIXEL,
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
        # Where ink PHYSICALLY landed: set for every in-bounds nozzle on
        # every sample it actually fired, which is the instant its pixel is
        # wanted and any ink at all is owed -- NOT gated on dose completion
        # the way `printed` is.
        #
        # The two diverge where a pixel gets SOME ink but not a full
        # `drops_per_pixel` dose: mostly the last partial column before the
        # cart turns or leaves the page, and any column the cart crosses
        # faster than the poll rate can sample it. Simulated over a 120-column
        # solid block at the rig's real settings (mm_per_column 0.087,
        # drops_per_pixel 3, poll_hz 500):
        #
        #   speed     samples/col   printed   fired
        #   17.3 mm/s     2.51       100.0%   100.0%
        #   30.0 mm/s     1.45       100.0%   100.0%
        #   43.5 mm/s     1.00       100.0%   100.0%
        #   50.0 mm/s     0.87        86.7%    86.7%
        #   60.0 mm/s     0.72        72.5%    72.5%
        #
        # Note what the columns say TOGETHER. Up to one sample per column
        # (43.5 mm/s = mm_per_column * poll_hz) both are 100%: dose no longer
        # depends on how long a nozzle lingers, only on how far the cart
        # travelled, so speed cannot thin the print. Past that the two fall
        # in step, because the failure is no longer under-dosing -- whole
        # columns are simply never sampled, so nothing fires there at all.
        # `fired` tracking `printed` down is the signature of that: ink that
        # is missing from the paper, not merely from the bookkeeping.
        #
        # Under the previous time-based dose the same sweep gave 73.3%
        # printed against 99.3% fired at 25 mm/s and 44.2% against 99.3% at
        # 30 -- a fully inked page reported as heavily striped, which is how
        # it was reported from hardware ("the real print's fill is perfect,
        # the coverage image looks nothing like it"). This mask was added to
        # show what the paper looks like when the two disagree; it still
        # earns its place at the top end, where they agree for a much worse
        # reason. See recording.render_coverage's THIN panel.
        self.fired = np.zeros_like(ink, dtype=bool)
        self.mm_per_column = mm_per_column
        # Fractional on purpose: the dose is a density (drops per mm, once
        # divided by mm_per_column), and clamping it to whole drops would
        # make the smallest step below the default a 100% jump. The floor is
        # a hair above zero rather than zero, since a dose of 0 would mark
        # every visited pixel printed without any ink going out at all.
        self.drops_per_pixel = max(_DOSE_EPS, float(drops_per_pixel))
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

        self._num_groups = -(-NUM_NOZZLES // nozzle_group)   # ceil division

        # Per-PIXEL accumulated dose, in drops (fractional -- see step()),
        # keyed on (row, col) -- NOT per-group-slot and NOT reset when a
        # group briefly stops being wanted or switches to a different pixel.
        # Why: NOZZLE_PITCH_MM (~0.087mm) is finer than realistic tracker
        # position noise, so a nozzle sitting near a row boundary has its
        # rounded (row, col) key flap between two neighbours sample to
        # sample. A per-group-slot accumulator that resets on every key
        # change (the engine's original design) never reaches a full dose in
        # that case -- the nozzle fires every single sample (see `active`
        # below, set unconditionally once `wanted`), but NEITHER neighbour
        # ever collects `drops_per_pixel`, so `printed` stays False and the
        # pixel never completes, no matter how long the cart sits there.
        # Reproduced directly: 200/200 samples of real firing, 0 pixels ever
        # marked printed, with jitter of only +-0.001mm (two orders of
        # magnitude below plausible tracker noise) straddling one row
        # boundary. A synthetic noise sweep (0 to 0.2mm) confirmed the
        # direction gets WORSE, not better, with more realistic noise:
        # recorded coverage can drop even as the number of samples that
        # actually fired goes up, because more noise means more
        # boundary-crossing flips.
        #
        # Entries live here only while a pixel is wanted-but-incomplete;
        # `_deposit` (via step()) removes the entry once the dose is
        # complete, so this dict's size is bounded by "distinct pixels
        # currently in progress", not by image size or pass length.
        self._pixel_drops: "dict[Tuple[int, int], float]" = {}
        self._last_pattern: Optional[bytes] = None

        # ------------------------------------------------ incremental tallies
        # Everything below replaces a full-image numpy pass that used to run
        # once per poll SAMPLE. That was not a micro-optimisation: measured on
        # the README's own page-mode example (2299x1152 = 2.65M pixels), the
        # per-sample cost was `done` 1279us + `(ink & fired).sum()` 1536us +
        # `ink.sum()` 977us, against a 2000us budget at --poll-hz 500. The
        # freehand loop's achieved rate collapsed to ~71 Hz under
        # --progress-json (measured; ~208 Hz without it), and since the ink
        # model's column-skipping edge is `mm_per_column * poll_hz`, that is a
        # ~6.2 mm/s speed limit on the print itself -- well under the measured
        # 17.3 mm/s median hand speed. These counters remove all of it.
        #
        # They can be exact rather than approximate because each mask has
        # exactly ONE writer: `printed` only in `_deposit`, `fired` only in
        # `step()`. Both increment right where the bit flips, guarded on the
        # bit having been 0 before.
        self._ink_total = int(ink.sum())
        # Ink pixels whose dose has completed -- what `done` used to scan the
        # whole image for.
        self._ink_printed = 0
        # Ink pixels that have physically received ink -- what the caller used
        # to get from `(ink & fired).sum()`.
        self._ink_fired = 0
        # Cells whose `fired` bit flipped since the caller last drained this
        # (see `drain_new_cells`). Replaces the caller diffing `fired` against
        # its own copy of the previous mask, which cost a further ~723us per
        # sample and needed a full-image copy to hold the baseline.
        #
        # NOT filtered by `ink`: the diff it replaces ran on `fired` alone, so
        # it also reported cells fired over non-ink pixels, and
        # tests/test_freehand_pass.py reconstructs the printed mask from this
        # stream. Only the COUNTERS above are ink-filtered.
        self._new_cells: "List[Tuple[int, int]]" = []

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
    def ink_total(self) -> int:
        """How many pixels the target asks for. Constant for this engine."""
        return self._ink_total

    @property
    def ink_fired(self) -> int:
        """Ink pixels that have physically received ink (``ink & fired``)."""
        return self._ink_fired

    @property
    def ink_printed(self) -> int:
        """Ink pixels whose dose has completed (``ink & printed``)."""
        return self._ink_printed

    def drain_new_cells(self) -> "List[Tuple[int, int]]":
        """
        Take the ``(row, col)`` cells whose ``fired`` bit flipped since the
        last call, and clear the buffer.

        Drain semantics, not peek: each flipped cell is handed out EXACTLY
        ONCE over the engine's lifetime, which is the contract
        ``--progress-json``'s ``new_cells`` stream is built on (see
        ``tests/test_freehand_pass.py``'s exactly-once/no-duplicates test).
        That holds however often -- or seldom -- a caller drains, so a caller
        may batch several samples into one update without losing a cell.
        """
        cells = self._new_cells
        self._new_cells = []
        return cells

    @property
    def done(self) -> bool:
        """True once every wanted ink pixel has been printed."""
        # Counter, not `np.all(self.printed[self.ink])`. This is read once per
        # poll sample by controller._print_freehand_pass, and that scan cost a
        # measured 1279us per call on a 2299x1152 target -- 64% of the whole
        # 2000us budget at --poll-hz 500, spent re-deriving a number
        # `_deposit` already knows exactly. See __init__'s tallies.
        return self._ink_printed >= self._ink_total

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
        ``NOZZLE_PITCH_MM`` (~0.087mm) tall but ``mm_per_column`` (0.2mm by
        default) wide, so a physically round drop is about 2.3:1 elliptical
        in pixel space. A pixel-count radius would silently mean two
        different physical distances on the two axes.

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
        1.0 dose from the drops that completed it (see ``_deposit``).

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

        Called exactly once per completing pixel. The centre always reaches
        1.0 here, so the pixel is `printed` on return -- but under the
        drop-count model `printed` no longer releases the nozzle by itself
        (see ``step()``'s Step 5: a pixel is reported up to one sample before
        its dose finishes, and keeps firing until it does). The
        call-once-only guarantee therefore comes from ``step()``'s explicit
        ``not self.printed[row, col]`` guard at the call site, not from the
        pixel going unwanted. It matters: the spray kernel below adds a
        partial dose to neighbours, and calling twice would splash it twice.

        CORRECTION: the spray loop used to mark a neighbour ``printed`` on
        dose alone, without checking ``self.ink[r, c]`` -- i.e. it could
        mark a pixel that was never ``wanted`` (no ink asked for there) as
        printed anyway. Confirmed harmless for the CENTRE (only ever called
        on a pixel that WAS wanted -- see ``step()``), but the neighbours it
        splats onto get no such guarantee. Concretely, a completed pixel
        sitting near a pattern boundary (e.g. a checkerboard square's edge)
        could spray a "printed" mark onto the far side of that boundary --
        a pixel that was never fired at all, now permanently skipped if a
        later pass tries to reach it (``wanted`` requires ``not
        self.printed``), silently eating a real corner/edge of the pattern.
        Reproduced directly: a 1-pixel-wide unwanted neighbour one row over
        from a completed wanted pixel came back ``printed=True`` with
        ``dose=1.0`` despite ``ink`` being False there and no fire ever
        having reached it. Now gated on ``self.ink[r, c]`` the same way the
        centre already effectively is (via ``wanted`` in ``step()``): spray
        only ever finishes a pixel that was already asked for."""
        h, w = self.ink.shape
        self.dose[row, col] = 1.0
        # Tally before the store, while the previous value is still readable.
        # Gated on BOTH conditions for reasons that are easy to get wrong:
        #   - `not printed` because a double-count would make `done` fire
        #     early and end the pass with ink still owed;
        #   - `ink` because under --nozzle-group 2 the OR rule deposits on
        #     every in-bounds member of a firing group, including members
        #     sitting on pixels the target never asked for. `_ink_total`
        #     counts only ink, so counting those would let `_ink_printed`
        #     overshoot it.
        if self.ink[row, col] and not self.printed[row, col]:
            self._ink_printed += 1
        self.printed[row, col] = True
        if not self._spray_kernel:
            return
        for dr, dc, weight in self._spray_kernel:
            r, c = row + dr, col + dc
            if (0 <= r < h and 0 <= c < w and self.ink[r, c]
                    and not self.printed[r, c]):
                acc = self.dose[r, c] + weight
                if acc >= 1.0:
                    self.dose[r, c] = 1.0
                    # Already guarded on `ink` and `not printed` by the `if`
                    # above -- spray only ever finishes a pixel that was
                    # asked for and is not finished yet.
                    self._ink_printed += 1
                    self.printed[r, c] = True
                else:
                    self.dose[r, c] = acc

    def step(self, u_mm: float, v_mm: float, t: float = 0.0,
             drops: float = 1.0,
             yaw_rad: float = 0.0) -> "Tuple[bytes, bool]":
        """
        Advance the engine with one live page-plane sample (see
        ``tracking.PageMapper.project``).

        ``drops`` is how much of a full dose this sample delivers to whatever
        pixel the bar is over, in units where ``drops_per_pixel`` is one
        finished pixel. It is deliberately FRACTIONAL: the caller
        (``controller._print_freehand_pass``) computes it from the distance
        travelled since the last sample, ``drops_per_pixel * travel /
        mm_per_column``, so summing it over one column of travel gives
        exactly one dose no matter how fast the cart moved or how the poll
        samples happened to land. Do not pass the integer number of BLE
        columns that went out on this sample instead -- that is a transport
        quantity, and crediting it produces speed-dependent false stripes
        (measured; see Step 5's comment in the body).

        ``t`` (seconds) is accepted but no longer used. Dosing was time-based
        while the firmware re-fired a held pattern at a fixed rate; it is
        drop-based now, and nothing else in the engine needs a clock. Kept in
        the signature -- with a default, so new callers can omit it -- rather
        than removed, because every existing caller and test passes it
        positionally and none of them would read any differently without it.

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
          * the dose is tracked ONCE per group (not per nozzle), keyed on
            the group's FIRST member's ``(row, col)`` -- but accumulated in
            ``self._pixel_drops`` (see ``__init__``), which does NOT reset
            when the group switches to a different pixel or stops being
            wanted for a while: it only grows, and only while this exact
            pixel is the group's current key, until the full
            ``drops_per_pixel`` has gone out and the entry is removed. A
            later revisit of the SAME pixel (by this group or, since the
            dict is keyed on the pixel itself rather than a per-group slot,
            by any group) resumes from where it left off rather than
            starting over.
          * once the dose is reported complete, ``_deposit`` is called for
            EVERY in-bounds member (not just the first) -- every member
            physically received ink, so every member's pixel must be marked
            printed.

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
                # `printed` alone no longer releases a nozzle: a pixel is
                # REPORTED printed up to one sample before its dose is
                # actually complete (Step 5), and firing has to continue
                # through that last sample or the report would be paid for
                # in real ink. An open ledger entry means "dose still owed",
                # so it keeps the nozzle wanted; Step 5 removes the entry at
                # the moment the full dose has gone out, and the nozzle
                # releases then.
                wanted = in_bounds and bool(self.ink[row, col]) and (
                    not self.printed[row, col]
                    or (row, col) in self._pixel_drops)
                members.append((p, row, col, in_bounds))
                group_wanted = group_wanted or wanted

            if not group_wanted:
                # Deliberately does NOT touch self._pixel_drops: whatever
                # pixel this group was accumulating drops on (if any) simply
                # is not visited THIS sample. Its progress must survive --
                # see __init__'s docstring for why resetting it here was the
                # bug (a nozzle sitting near a row boundary flaps between
                # "wanted here" and "wanted next door" every sample, and a
                # reset-on-any-change rule then never lets either accumulate
                # a full dose, no matter how long it fires).
                continue

            # Step 4: drops tracked once per group, keyed on the FIRST
            # member's (row, col) -- at group_size=1 this is that one
            # nozzle's own (row, col), same key the ungrouped code used.
            # Accumulated in the persistent per-pixel dict (see __init__),
            # not reset by a key change -- only removed on completion below.
            # Surviving a key change is what this dict exists for: a nozzle
            # sitting near a row boundary flaps between "wanted here" and
            # "wanted next door" every sample, and a reset-on-any-change rule
            # would then never let either side accumulate its full dose, no
            # matter how long the nozzle fires there.
            first_row, first_col = members[0][1], members[0][2]
            pixel = (first_row, first_col)
            # Ink is counted in DROPS, not in dwell time: the firmware fires
            # each column it receives exactly once and never repeats, so how
            # much ink a pixel gets is decided entirely on this side -- see
            # the module docstring.
            #
            # `drops` is FRACTIONAL on purpose (see step()'s docstring). It is
            # the share of a full dose this sample's travel is worth, not the
            # integer number of BLE columns that happened to go out on this
            # particular sample. Those differ: at 17.3 mm/s a sample is worth
            # 1.2 drops, so the integer stream reads 1,1,2,1,1,2,... and a
            # column that happens to collect 1+1 before the cart leaves it
            # reads a drop short although the paper got its share. Crediting
            # the fraction makes the credit a pixel receives while the bar
            # crosses it sum to exactly `drops_per_pixel`, by construction,
            # at any speed; crediting the integer leaves it at the mercy of
            # where the rounding happened to land. Simulated over a
            # 120-column solid block at the default dose, the integer version
            # reported 88.3% of columns printed at 25 mm/s, 75.0% at 30,
            # 75.8% at 35 and 84.2% at 43.5 (against a flat 100% for the
            # fraction), and delivered 257 columns/s at 30 mm/s against the
            # budget's 343. `drops = 0` (a stationary cart) credits nothing
            # and fires nothing either way.
            treffer = self._pixel_drops.get(pixel, 0.0) + drops

            # Step 3: the whole group fires together -- and every in-bounds
            # member that fires physically puts ink on its own cell, right
            # now, whether or not the dose ever completes there (see
            # self.fired in __init__). Recorded per member, not just for the
            # key pixel, for the same reason _deposit is called for every
            # member on completion: they all really did fire.
            for p, row, col, in_bounds in members:
                active[p] = True
                if in_bounds and drops > 0 and not self.fired[row, col]:
                    self.fired[row, col] = True
                    # Every flip is recorded once, here, at the only place
                    # `fired` is ever written -- see drain_new_cells(). The
                    # caller used to recover the same set by diffing `fired`
                    # against a full-image copy of its previous state, at a
                    # measured ~723us per sample on a 2299x1152 target.
                    #
                    # Deliberately NOT filtered by `ink`: the diff it replaces
                    # ran on `fired` alone and so also reported cells fired
                    # over non-ink pixels. Only the tally below is ink-gated.
                    self._new_cells.append((row, col))
                    if self.ink[row, col]:
                        self._ink_fired += 1

            # Step 5: two thresholds on the one ledger, for two different
            # questions. They are deliberately not the same number.
            #
            #   `fertig` -- the full dose has now gone out. This RELEASES the
            #       nozzle: the entry is dropped, and with it (see `wanted`
            #       above) the group's claim on this pixel. It has to be the
            #       strict threshold, because everything past it is ink that
            #       does not get printed.
            #
            #   `melden` -- the pixel is within ONE more sample of its full
            #       dose. This is what marks it `printed`, i.e. what the
            #       coverage report and `done` are built on. It has to carry
            #       one sample of slack, because credit arrives in whole poll
            #       samples: a crossing worth m samples deposits floor(m) or
            #       ceil(m) of them inside the pixel, so a fully swept pixel
            #       can end up to one sample short of a strict full dose
            #       through nothing but how the sample grid happened to line
            #       up with the column grid. Measured on a 120-column solid
            #       block (mm_per_column 0.087, poll_hz 500), reporting on
            #       the strict threshold gave 70.0% of columns printed at
            #       5 mm/s, 35.0% at 10, 51.7% at 17.3, 74.2% at 25 and 9.2%
            #       at 40 -- against 100% fired throughout, and swinging with
            #       speed for no physical reason. On `melden` all of those
            #       read 100%.
            #
            # Reporting one sample early costs nothing on paper precisely
            # because it does not release the nozzle -- that is what the two
            # thresholds buy. Reporting AND releasing on `melden` was tried
            # and is measurably wrong: it cuts each crossing short by up to a
            # sample of real ink. Measured at the default dose against the
            # budget's column rate: 142/s instead of 199 at 17.3 mm/s, 74/s
            # instead of 285 at 25, 189/s instead of 343 at 30 -- a 30-75%
            # under-ink that also stops being monotonic in speed, while
            # coverage still cheerfully reports 100%.
            #
            # The epsilon is float hygiene, not policy: a credit built from
            # summed fractions lands on its target only up to rounding (three
            # exact 1.0 shares can total 2.9999999999999996), and a single
            # ulp short would otherwise strand a column forever.
            ziel = self.drops_per_pixel - _DOSE_EPS
            fertig = treffer >= ziel
            melden = treffer + drops >= ziel
            if fertig:
                self._pixel_drops.pop(pixel, None)
            else:
                self._pixel_drops[pixel] = treffer

            if melden:
                for _p, row, col, in_bounds in members:
                    # Guarded, because `melden` stays true for the samples
                    # between the report and the release: _deposit must run
                    # exactly once per pixel or its spray kernel would splash
                    # a neighbour's partial dose on repeatedly.
                    if in_bounds and not self.printed[row, col]:
                        self._deposit(row, col)

        self.last_in_bounds = in_bounds_any
        pattern = pack_nozzle_bits(active)
        changed = pattern != self._last_pattern
        self._last_pattern = pattern
        return pattern, changed
