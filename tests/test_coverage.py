"""
Freehand coverage engine tests (no hardware): mirrors the scenarios the
firmware's tests/test_column_fifo.c already proved out for the 1D dose
pipeline, generalised to CoverageEngine's per-nozzle, per-pixel model.

Run with:  python tests/test_coverage.py
"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from printhead.calibration import PageCalibration                      # noqa: E402
from printhead.coverage import (                                       # noqa: E402
    CoverageEngine, DEFAULT_DROPS_PER_PIXEL, bar_offset_uv,
)
from printhead.geometry import (                                       # noqa: E402
    NOZZLE_BAR_SPAN_MM, NOZZLE_PITCH_MM, NUM_NOZZLES, ROW_BYTES,
)
from printhead.rendering import pack_nozzle_bits                       # noqa: E402
from printhead.tracking import PageMapper                              # noqa: E402

# Dose used by the tests that do not care about the exact number. With the
# default drops=1.0 per step() call, a pixel is REPORTED printed after
# DROPS-1 samples and the nozzle is RELEASED after DROPS -- see step()'s
# Step 5 for why those are one sample apart.
DROPS = 3


def _unpack(pattern: bytes) -> np.ndarray:
    return np.unpackbits(np.frombuffer(pattern, dtype=np.uint8), bitorder="little")


# ============================================== measured drop-count tuning
def _sweep(eng, speed_mm_s, mm_per_column, poll_hz=500.0, columns=None,
           drops_per_pixel=None, v_mm=0.0):
    """
    Drive a straight constant-speed sweep across ``columns`` columns, deriving
    ``drops`` from travel exactly the way
    ``controller._print_freehand_pass`` does (fractional share to the engine,
    whole columns to the link). Returns ``(delivered_columns, samples)``.
    """
    dpp = eng.drops_per_pixel if drops_per_pixel is None else drops_per_pixel
    columns = eng.width if columns is None else columns
    dt = 1.0 / poll_hz
    n = int(columns * mm_per_column / (speed_mm_s * dt)) + 1
    schuld = 0.0
    vorher = None
    geliefert = 0
    for i in range(n):
        u = i * speed_mm_s * dt
        anteil = float(dpp) if vorher is None else dpp * (u - vorher) / mm_per_column
        vorher = u
        schuld += anteil
        kopien = int(schuld)
        schuld -= kopien
        pattern, _ = eng.step(u_mm=u, v_mm=v_mm, t=i * dt, drops=anteil)
        if kopien and any(pattern):
            geliefert += kopien
    return geliefert, n


def test_default_dose_reproduces_the_ink_density_validated_on_paper():
    # REGRESSION, reported from hardware after the fire-once conversion:
    # "jetzt kommt zu viel raus" -- the print came out heavier and less sharp
    # than the pre-conversion one. The default had been copied from line
    # mode's firmware constant BLE_DROPS_PER_COLUMN = 3 without noticing that
    # 3 describes a 0.2 mm column, not this rig's 0.087 mm one.
    #
    # What the paper sees is drops per MILLIMETRE, which is the dose divided
    # by the column width. Pinned as that density, not as the raw number, so
    # a future change to either one has to face the product:
    #
    #     3 / 0.200 = 15.0   line mode's validated column
    #     3 / 0.087 = 34.5   the same 3 at today's column   <- 3x over-inked
    #     1 / 0.087 = 11.5   this default
    #
    # 11.5/mm is exactly what the pre-conversion client delivered at low
    # speed (simulated on the fire-once firmware: 120 columns of ink over
    # 120 columns of travel), i.e. the density that was judged acceptable on
    # real paper.
    rig_mm_per_column = 0.087
    dichte = DEFAULT_DROPS_PER_PIXEL / rig_mm_per_column
    assert abs(dichte - 11.5) < 0.1, (
        f"the default lays down {dichte:.1f} drops/mm at "
        f"{rig_mm_per_column} mm/column, not the ~11.5 that the "
        f"pre-conversion client delivered and the operator accepted on "
        f"paper -- if this is a deliberate re-calibration, say which print "
        f"it came from")


def test_the_dose_is_a_density_so_it_scales_with_the_column_width():
    # The trap the wrong default fell into, pinned directly: the same
    # `drops_per_pixel` means different amounts of ink at different column
    # widths, because a column of travel is what the dose is per. Anyone
    # changing --mm-per-column has to move the dose with it.
    mm_pro_spalte = 0.087
    for dose in (0.5, 1.0, 3.0):
        eng = CoverageEngine(np.ones((20, 60), dtype=bool),
                             mm_per_column=mm_pro_spalte,
                             drops_per_pixel=dose)
        geliefert, _ = _sweep(eng, 17.3, mm_pro_spalte)
        pro_mm = geliefert / (60 * mm_pro_spalte)
        assert abs(pro_mm - dose / mm_pro_spalte) <= 0.05 * dose / mm_pro_spalte, (
            f"drops_per_pixel={dose} delivered {pro_mm:.1f} drops/mm, "
            f"expected {dose / mm_pro_spalte:.1f}")


def test_a_fractional_dose_is_honoured_rather_than_rounded_up():
    # Whole drops are too coarse a dial below the default: the next step down
    # from 1 would be "no ink at all". A half dose must halve the ink.
    mm_pro_spalte = 0.087
    voll = CoverageEngine(np.ones((20, 60), dtype=bool),
                          mm_per_column=mm_pro_spalte, drops_per_pixel=1.0)
    halb = CoverageEngine(np.ones((20, 60), dtype=bool),
                          mm_per_column=mm_pro_spalte, drops_per_pixel=0.5)
    v, _ = _sweep(voll, 17.3, mm_pro_spalte)
    h, _ = _sweep(halb, 17.3, mm_pro_spalte)
    assert abs(h - v / 2) <= 0.1 * v / 2, (v, h)


def test_coverage_is_speed_independent_up_to_the_poll_rate_limit():
    # The headline property of the drop-count model, and the reason it
    # replaced the time-based hold: ink follows travel, so hand speed cannot
    # thin a print. Under the old model the same sweep fell to 60% coverage
    # by 25 mm/s and 14% by 35.
    #
    # The limit that IS left is the poll rate: a column the tracker never
    # sampled never fires. That edge sits at mm_per_column * poll_hz.
    mm_per_column, poll_hz = 0.087, 500.0
    grenze = mm_per_column * poll_hz                      # 43.5 mm/s
    for speed in (5.0, 17.3, 25.0, 30.0, 40.0, grenze):
        eng = CoverageEngine(np.ones((40, 120), dtype=bool),
                             mm_per_column=mm_per_column)
        _sweep(eng, speed, mm_per_column, poll_hz)
        assert eng.printed.all(), (
            f"{speed} mm/s left {int((~eng.printed).sum())} of "
            f"{eng.printed.size} pixels unprinted -- coverage must not "
            f"depend on speed below the {grenze:.1f} mm/s poll-rate limit")


def test_past_the_poll_rate_limit_whole_columns_are_skipped_not_thinned():
    # Above the limit the failure mode changes character, and the two masks
    # say so: `fired` falls with `printed` rather than staying at 100%,
    # because the columns are not under-dosed, they are never visited.
    mm_per_column, poll_hz = 0.087, 500.0
    eng = CoverageEngine(np.ones((40, 120), dtype=bool),
                         mm_per_column=mm_per_column)
    _sweep(eng, 60.0, mm_per_column, poll_hz)
    anteil = eng.printed.sum() / eng.printed.size
    assert 0.6 < anteil < 0.85, (
        f"expected roughly 0.087*500/60 = 72% of columns to be sampled at "
        f"60 mm/s, got {anteil:.1%}")
    assert (eng.fired == eng.printed).all(), (
        "past the poll-rate limit the loss is unfired columns, not partial "
        "doses -- fired and printed must agree")


def test_the_delivered_column_count_matches_the_drops_per_pixel_budget():
    # What actually reaches the paper, not just what the masks claim: a
    # 120-column sweep owes 120 * drops_per_pixel columns of ink. Pinned
    # because reporting a pixel printed one sample before its dose is
    # complete (step()'s Step 5) is only safe as long as it does not also
    # release the nozzle -- if it did, this count would drop by up to 40%.
    mm_per_column = 0.087
    for speed in (17.3, 25.0, 35.0):
        eng = CoverageEngine(np.ones((40, 120), dtype=bool),
                             mm_per_column=mm_per_column)
        geliefert, _ = _sweep(eng, speed, mm_per_column)
        soll = 120 * DEFAULT_DROPS_PER_PIXEL
        assert abs(geliefert - soll) <= 0.03 * soll, (
            f"{speed} mm/s delivered {geliefert} columns against a budget of "
            f"{soll} -- the pass is under- or over-inking")


# ======================================================= ink spread ("spray")
def _spray_engine(radius_mm, strength, mm_per_column=0.2, size=(41, 41)):
    return CoverageEngine(np.ones(size, dtype=bool), mm_per_column=mm_per_column,
                          drops_per_pixel=DROPS, spray_radius_mm=radius_mm,
                          spray_strength=strength)


def test_spray_is_off_by_default():
    # The whole feature must be opt-in: with no spray arguments the kernel is
    # empty and a deposit touches exactly one pixel, i.e. bit-identical to the
    # pre-spray engine.
    eng = CoverageEngine(np.ones((21, 21), dtype=bool), mm_per_column=0.2,
                         drops_per_pixel=DROPS)
    assert eng._spray_kernel == []
    eng._deposit(10, 10)
    assert eng.printed[10, 10]
    assert eng.printed.sum() == 1, "no spray configured must touch exactly one pixel"


def test_spray_needs_both_radius_and_strength_to_engage():
    assert _spray_engine(0.2, 0.0)._spray_kernel == []      # no strength
    assert _spray_engine(0.0, 1.0)._spray_kernel == []      # no radius
    assert _spray_engine(0.2, 1.0)._spray_kernel != []


def test_spray_kernel_gives_the_nearest_neighbour_exactly_the_strength():
    # The normalisation that makes the parameter mean something (see
    # _build_spray_kernel): without it the nearest neighbour sits ~half a
    # radius out and could never exceed ~0.5*strength, so strength=1.0 could
    # not complete a pixel and the feature had NO measurable effect.
    for strength in (0.25, 0.5, 1.0):
        kernel = _spray_engine(0.2, strength)._spray_kernel
        assert abs(max(w for _, _, w in kernel) - strength) < 1e-9, strength


def test_spray_radius_is_physical_so_the_kernel_is_anisotropic():
    # A cell is NOZZLE_PITCH_MM (~0.087mm) tall but mm_per_column (0.2mm)
    # wide, so a round drop must reach further in ROWS than in COLUMNS. A
    # pixel-count radius would silently mean two different real distances.
    kernel = _spray_engine(0.2, 1.0, mm_per_column=0.2)._spray_kernel
    max_dr = max(abs(dr) for dr, _, _ in kernel)
    max_dc = max(abs(dc) for _, dc, _ in kernel)
    assert max_dr > max_dc, (max_dr, max_dc)
    assert abs(max_dr - round(0.2 / NOZZLE_PITCH_MM)) <= 1

    # mm_per_column == NOZZLE_PITCH_MM exactly (whatever that constant's
    # current value is -- deliberately read from geometry.py, not a literal
    # here) means the cells are exactly square, so the kernel must be
    # EXACTLY symmetric, not just close -- both axes run the identical
    # radius/mm_per_column formula against the identical mm value.
    square = _spray_engine(0.2, 1.0, mm_per_column=NOZZLE_PITCH_MM)._spray_kernel
    square_max_dr = max(abs(dr) for dr, _, _ in square)
    square_max_dc = max(abs(dc) for _, dc, _ in square)
    assert square_max_dr == square_max_dc, (square_max_dr, square_max_dc)

    # The exact-match case above can't by itself rule out a bug that just
    # happens to look square whenever the two axes share one pixel-count
    # radius (e.g. converting both from raw pixel counts instead of real
    # mm) -- so also check a mm_per_column NARROWER than the row pitch
    # (finer columns than rows) and confirm the anisotropy actually
    # FLIPS DIRECTION (more columns than rows touched), the opposite of
    # the mm_per_column=0.2 case above. Only a genuine per-axis physical
    # conversion produces that flip; a pixel-count-based implementation
    # would not.
    narrow = _spray_engine(0.2, 1.0, mm_per_column=0.05)._spray_kernel
    narrow_max_dr = max(abs(dr) for dr, _, _ in narrow)
    narrow_max_dc = max(abs(dc) for _, dc, _ in narrow)
    assert narrow_max_dc > narrow_max_dr, (narrow_max_dr, narrow_max_dc)


def test_spray_at_full_strength_marks_the_adjacent_pixel_printed():
    # This is what actually stops a return pass, drifted one row over, from
    # re-firing paper that already has ink on it.
    eng = _spray_engine(0.2, 1.0)
    eng._deposit(20, 20)
    assert eng.printed[20, 20]
    assert eng.printed[19, 20] and eng.printed[21, 20], (
        "at strength 1.0 an immediately adjacent pixel must count as printed")


def test_spray_at_half_strength_needs_two_drops():
    # Both drops must be IMMEDIATELY adjacent to the pixel under test: at
    # radius 0.2mm a two-row gap is already ~0.199mm, i.e. right at the edge
    # of the kernel where the linear falloff has taken the weight to ~0.007.
    eng = _spray_engine(0.2, 0.5)
    eng._deposit(19, 20)
    assert not eng.printed[20, 20], "one drop at 0.5 must not complete a neighbour"
    assert abs(eng.dose[20, 20] - 0.5) < 1e-6, eng.dose[20, 20]
    eng._deposit(21, 20)                       # second drop, other side
    assert eng.printed[20, 20], "two half-strength drops must complete it"


def test_spray_never_lowers_an_already_printed_pixel_or_leaves_the_page():
    # Deposit hard against a corner: must not raise, must not wrap around to
    # the far edge, and must leave an already-printed pixel printed.
    eng = _spray_engine(0.3, 1.0)
    eng._deposit(0, 0)
    assert eng.printed[0, 0]
    assert not eng.printed[-1, -1], "spray must not wrap to the opposite edge"
    eng._deposit(1, 0)
    assert eng.printed[0, 0], "an already-printed pixel must stay printed"


def test_spray_dose_never_exceeds_one():
    eng = _spray_engine(0.3, 1.0)
    for r in range(8, 14):
        eng._deposit(r, 20)
    assert eng.dose.max() <= 1.0 + 1e-6, eng.dose.max()


def test_spray_can_only_ever_add_coverage_never_remove_it():
    # Correctness property under identical input: everything the plain engine
    # marks must also be marked with spray on (spray only ever ADDS dose).
    # Driven off a fixed sample grid, not wall-clock, so it is deterministic
    # -- a timing-driven comparison is far too noisy to show this (measured:
    # run-to-run spread larger than the effect itself).
    ink = np.ones((60, 40), dtype=bool)
    samples = [(u, 2.0 + 0.02 * i, 0.001 * i)
               for i, u in enumerate(np.linspace(0.0, 6.0, 400))]

    def run(radius, strength):
        eng = CoverageEngine(ink, mm_per_column=0.2, drops_per_pixel=2,
                             spray_radius_mm=radius, spray_strength=strength)
        for u, v, t in samples:
            eng.step(u_mm=u, v_mm=v, t=t)
        return eng.printed.copy()

    plain = run(0.0, 0.0)
    sprayed = run(0.15, 1.0)
    assert plain.any(), "test is vacuous if the plain run printed nothing"
    assert np.all(sprayed | ~plain), "spray must never un-print a pixel"
    assert sprayed.sum() >= plain.sum()


def test_spray_never_marks_a_pixel_that_was_never_wanted():
    # REGRESSION, found while analysing a real checkerboard print
    # (pass_spray.csv, --dose-hold-s 0.001 --spray-radius-mm 0.3
    # --spray-strength 0.8): the physical print lost the checkerboard's
    # black/white alternation and came out as one solid inked blob. The
    # dominant cause turned out to be BLE write backlog, NOT this bug --
    # 0.3mm cannot bridge a 10mm checkerboard square either way -- but while
    # ruling spray out, _deposit's spray loop turned out to mark a
    # neighbour `printed` on accumulated dose alone, with no check of
    # `self.ink[r, c]`. A pixel that was never `wanted` -- e.g. a white
    # checkerboard cell one row across a pattern boundary from a completed
    # black cell -- could come back `printed=True` despite never having
    # been fired at. Harmless for the print itself (a pixel with ink=False
    # was never going to fire regardless of `printed`), but it corrupts the
    # COVERAGE BOOKKEEPING: if that same (row, col) is later wanted by a
    # DIFFERENT pattern reusing this engine, or the false "printed" bit
    # otherwise gets trusted, a genuinely wanted pixel would be silently
    # skipped, having never actually been inked.
    ink = np.zeros((41, 41), dtype=bool)
    ink[20, 20] = True                       # the only wanted pixel
    # ink[21, 20] stays False -- one row over, inside the spray radius below.
    eng = CoverageEngine(ink, mm_per_column=0.2, drops_per_pixel=1,
                         spray_radius_mm=0.25, spray_strength=1.0)
    eng._deposit(20, 20)
    assert eng.printed[20, 20], "the actually-wanted centre must still print"
    assert not eng.printed[21, 20], (
        "a pixel with ink=False was marked printed by spray alone, despite "
        "never being fired at")
    assert eng.dose[21, 20] == 0.0, (
        "an unwanted neighbour must not accumulate spray dose either")


# ============================================================== basic dosing
def test_single_pass_completes_once_the_drops_have_gone_out():
    ink = np.ones((10, 5), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=1.0, drops_per_pixel=DROPS)

    eng.step(u_mm=0.0, v_mm=0.0, t=0.0)                # 1 of 3 drops
    assert not eng.printed[0, 0]                        # not yet dosed
    eng.step(u_mm=0.0, v_mm=0.0, t=0.1)                # 2 of 3, within one of full
    assert eng.printed[0, 0]


def test_nozzle_keeps_firing_through_the_last_drop_after_being_reported():
    # `printed` is set one sample before the dose is complete; the nozzle
    # must NOT be released with the last drop still owed, or the report is
    # paid for in real ink (see step()'s Step 5).
    ink = np.ones((10, 5), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=1.0, drops_per_pixel=DROPS)

    eng.step(u_mm=0.0, v_mm=0.0, t=0.0)                        # drop 1
    pattern, _ = eng.step(u_mm=0.0, v_mm=0.0, t=0.1)           # drop 2, reported
    assert eng.printed[0, 0]
    assert _unpack(pattern)[0] == 1
    pattern, _ = eng.step(u_mm=0.0, v_mm=0.0, t=0.2)           # drop 3, the last
    assert _unpack(pattern)[0] == 1, "released before its final drop went out"
    pattern, changed = eng.step(u_mm=0.0, v_mm=0.0, t=0.3)
    assert _unpack(pattern)[0] == 0                             # then stops
    assert changed


def test_a_stationary_cart_owes_no_drops_and_never_completes():
    # The anti-blob property, and the sharpest break from the dwell model:
    # a parked cart used to complete a pixel purely by sitting there. Ink now
    # follows travel, so `drops=0` samples pile up forever with no effect --
    # no matter how much wall-clock time passes.
    ink = np.ones((10, 5), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=1.0, drops_per_pixel=DROPS)

    for i in range(500):
        eng.step(u_mm=0.0, v_mm=0.0, t=i * 0.1, drops=0.0)      # 50 s parked
    assert not eng.printed[0, 0]
    assert not eng.fired[0, 0], "a sample owing no drops must not ink either"

    eng.step(u_mm=0.0, v_mm=0.0, t=50.0, drops=float(DROPS))
    assert eng.printed[0, 0]


def test_revisit_does_not_refire_a_fully_dosed_pixel():
    ink = np.ones((10, 5), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=1.0, drops_per_pixel=DROPS)

    for i in range(DROPS):
        eng.step(u_mm=0.0, v_mm=0.0, t=0.1 * i)          # full dose to (0,0)
    assert eng.printed[0, 0]

    eng.step(u_mm=3.0, v_mm=0.0, t=0.5)                  # move away
    pattern, _ = eng.step(u_mm=0.0, v_mm=0.0, t=0.6)     # revisit
    assert _unpack(pattern)[0] == 0                      # never refires
    assert eng.printed[0, 0]                             # stays printed


def test_loop_with_a_second_pass_finishes_a_partially_dosed_pixel():
    # First pass leaves the pixel short of a dose -> not printed. A "loop
    # back" pass finishes it. Models "keep going in circles until it takes"
    # rather than one continuous crossing.
    ink = np.ones((10, 5), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=1.0, drops_per_pixel=6)

    eng.step(u_mm=0.0, v_mm=0.0, t=0.0)                   # 1 of 6, left early
    assert not eng.printed[0, 0]

    eng.step(u_mm=3.0, v_mm=0.0, t=1.0)                   # loop away
    for i in range(4):                                    # 2..5 of 6 on return
        eng.step(u_mm=0.0, v_mm=0.0, t=2.0 + 0.1 * i)
    assert eng.printed[0, 0]


def test_completion_depends_on_drops_delivered_not_on_elapsed_time():
    # The inverse of the rule this engine used to follow. Fifty samples each
    # carrying a tenth of a drop are five drops short of the dose however
    # much or little time they span; a single sample carrying the whole dose
    # completes it instantly.
    ink = np.ones((10, 5), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=1.0, drops_per_pixel=DROPS)

    for i in range(50):
        eng.step(u_mm=0.0, v_mm=0.0, t=1000.0 * i, drops=0.01)   # 13+ hours
    assert not eng.printed[0, 0]

    eng.step(u_mm=0.0, v_mm=0.0, t=0.0, drops=float(DROPS))      # no time at all
    assert eng.printed[0, 0]


def test_dose_does_not_accumulate_across_different_pixels():
    ink = np.ones((10, 5), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=1.0, drops_per_pixel=DROPS)

    eng.step(u_mm=0.0, v_mm=0.0, t=0.0, drops=1.0)           # pixel (0,0), 1 of 3
    eng.step(u_mm=1.0, v_mm=0.0, t=0.1, drops=1.0)           # pixel (0,1), 1 of 3

    assert not eng.printed[0, 0]
    assert not eng.printed[0, 1]


def test_a_parked_nozzle_fires_exactly_its_dose_and_no_more():
    # Ink volume per pixel, pinned directly rather than inferred from the
    # masks. A nozzle sitting on one pixel must hold its fire bit for exactly
    # `drops_per_pixel` samples at one drop a sample: fewer means the report
    # released it early (the failure mode that made "coverage.png looks
    # fuller than the real print" a hardware complaint under the old model),
    # more means the dose gate is not stopping it at all.
    for dose in (1, 3, 6):
        ink = np.ones((NUM_NOZZLES + 5, 3), dtype=bool)
        eng = CoverageEngine(ink, mm_per_column=0.2, drops_per_pixel=dose)
        fired = 0
        for i in range(dose + 6):
            pattern, _ = eng.step(u_mm=0.4, v_mm=0.0, t=i * 0.002, drops=1.0)
            if pattern[0] & 0x01:
                fired += 1
        assert fired == dose, (
            f"drops_per_pixel={dose}: nozzle held its fire bit for {fired} "
            f"samples, expected {dose} -- ink volume per pixel is wrong")


# ============================== incremental tallies vs the full-image truth
def _tally_invariant(eng, mm_per_column, speed=17.3, poll_hz=500.0):
    """Drive a sweep and check every counter against its O(N) definition
    after EVERY step -- the definitions these counters replaced."""
    gesammelt = []
    dt = 1.0 / poll_hz
    dpp = eng.drops_per_pixel
    n = int(eng.width * mm_per_column / (speed * dt)) + 1
    vorher = None
    for i in range(n):
        u = i * speed * dt
        anteil = float(dpp) if vorher is None else dpp * (u - vorher) / mm_per_column
        vorher = u
        eng.step(u_mm=u, v_mm=0.0, t=i * dt, drops=anteil)
        gesammelt += eng.drain_new_cells()

        assert eng.ink_total == int(eng.ink.sum())
        assert eng.ink_fired == int((eng.ink & eng.fired).sum()), i
        assert eng.ink_printed == int((eng.ink & eng.printed).sum()), i
        assert eng.done == bool(np.all(eng.printed[eng.ink])), i
    return gesammelt


def _checkerboard(h, w, feld=5):
    a = np.zeros((h, w), dtype=bool)
    for r in range(h):
        for c in range(w):
            a[r, c] = ((r // feld) + (c // feld)) % 2 == 0
    return a


def test_tallies_match_the_full_image_counts_they_replaced():
    # These counters exist to keep a full-image numpy pass out of the poll
    # loop: measured on a 2299x1152 target, `np.all(printed[ink])` alone cost
    # 1279 us per sample against a 2000 us budget, and the loop's achieved
    # rate had collapsed to ~71 Hz under --progress-json -- a ~6 mm/s speed
    # limit on the print itself, since the column-skipping edge is
    # mm_per_column * poll_hz.
    #
    # Checked after EVERY step rather than once at the end: a counter that
    # drifts and then happens to converge would pass an end-only check.
    mm = 0.087
    for label, kwargs in (
            ("plain", {}),
            # spray: _deposit marks NEIGHBOURS printed without firing them,
            # so ink_printed must move without ink_fired following.
            ("spray", dict(spray_radius_mm=0.25, spray_strength=1.0)),
            # nozzle_group=2: the OR rule fires and deposits on members whose
            # own pixel has no ink, so `fired` is NOT a subset of `ink` and an
            # unfiltered count would overshoot ink_total.
            ("group2", dict(nozzle_group=2)),
            ("spray+group2", dict(spray_radius_mm=0.25, spray_strength=1.0,
                                  nozzle_group=2)),
    ):
        eng = CoverageEngine(_checkerboard(60, 200), mm_per_column=mm, **kwargs)
        gesammelt = _tally_invariant(eng, mm)
        assert eng.ink_fired > 0, label
        # The drained stream must reconstruct `fired` exactly -- it is what
        # --progress-json's new_cells is built from, and tests elsewhere
        # rebuild the printed mask from it.
        rekonstruiert = np.zeros_like(eng.ink, dtype=bool)
        for r, c in gesammelt:
            rekonstruiert[r, c] = True
        assert np.array_equal(rekonstruiert, eng.fired), label
        assert len(gesammelt) == len(set(gesammelt)), f"{label}: duplicates"


def test_depositing_the_same_pixel_twice_counts_it_once():
    # step()'s Step 5 already guards `_deposit` with `not printed`, so this
    # path is unreachable through step() today -- which is exactly why it
    # needs its own test: the guard inside _deposit is the one that keeps
    # ink_printed honest if that caller-side check is ever relaxed, and a
    # double count would make `done` fire with ink still owed.
    eng = CoverageEngine(np.ones((10, 10), dtype=bool), mm_per_column=0.087)
    eng._deposit(3, 4)
    assert eng.ink_printed == 1
    eng._deposit(3, 4)
    assert eng.ink_printed == 1, "a re-deposit must not count twice"
    assert eng.ink_printed == int((eng.ink & eng.printed).sum())


def test_deposit_does_not_count_a_pixel_the_target_never_asked_for():
    # Reachable for real: under nozzle_group=2 the OR rule deposits on every
    # in-bounds member of a firing group, including one sitting on a non-ink
    # pixel. ink_total counts only ink, so counting those would let
    # ink_printed overshoot it and `done` never agree with the mask.
    ink = np.zeros((10, 10), dtype=bool)
    ink[3, 4] = True
    eng = CoverageEngine(ink, mm_per_column=0.087)
    eng._deposit(7, 7)                      # no ink asked for here
    assert eng.ink_printed == 0
    assert eng.ink_printed == int((eng.ink & eng.printed).sum())


def test_drain_hands_every_cell_out_exactly_once():
    # Drain, not peek: the exactly-once property must survive an IRREGULAR
    # drain cadence, because the caller throttles its emissions and drains in
    # batches of whatever accumulated since the last one.
    eng = CoverageEngine(np.ones((40, 120), dtype=bool), mm_per_column=0.087)
    gesammelt = []
    vorher = None
    for i in range(900):
        u = i * 17.3 / 500.0
        anteil = 1.0 if vorher is None else (u - vorher) / 0.087
        vorher = u
        eng.step(u_mm=u, v_mm=0.0, t=i / 500.0, drops=anteil)
        if i % 7 == 0:                      # deliberately uneven
            gesammelt += eng.drain_new_cells()
    gesammelt += eng.drain_new_cells()      # final flush

    assert len(gesammelt) == len(set(gesammelt)), "a cell was handed out twice"
    assert len(gesammelt) == int(eng.fired.sum()), "a cell was never handed out"
    assert eng.drain_new_cells() == [], "drained buffer must come back empty"


def test_group_2_reports_non_ink_cells_but_does_not_count_them():
    # The two rules pull in opposite directions on purpose, and this pins
    # both: `new_cells` mirrors `fired` (unfiltered, so a consumer can
    # reconstruct the mask), while ink_fired counts only ink (so it can never
    # exceed ink_total). Under nozzle_group=2 the OR rule makes them differ.
    ink = np.zeros((NUM_NOZZLES, 40), dtype=bool)
    ink[::2, :] = True                      # every other row -> groups straddle
    eng = CoverageEngine(ink, mm_per_column=0.087, nozzle_group=2)
    gesammelt = _tally_invariant(eng, 0.087)

    assert len(gesammelt) > eng.ink_fired, (
        "group 2 must report cells it fired over non-ink pixels")
    assert eng.ink_fired <= eng.ink_total
    assert int(eng.fired.sum()) > int((eng.ink & eng.fired).sum())


def test_a_dose_summed_from_fractions_completes_without_an_extra_sample():
    # The float-hygiene epsilon in step()'s Step 5, pinned as the ink it
    # saves. Ten samples of a tenth of a dose each is exactly one dose in
    # real arithmetic and 0.9999999999999999 in floating point, so a strict
    # `>=` leaves the nozzle firing for an eleventh sample it does not owe.
    # Ten fractional samples per column is an ordinary crossing, not a
    # contrived one -- 0.087 mm columns at 500 Hz reach it below 4.4 mm/s.
    ink = np.ones((NUM_NOZZLES + 5, 3), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=0.2, drops_per_pixel=1)
    fired = 0
    for i in range(20):
        pattern, _ = eng.step(u_mm=0.4, v_mm=0.0, t=i * 0.002, drops=0.1)
        if pattern[0] & 0x01:
            fired += 1
    assert fired == 10, (
        f"nozzle fired {fired} times for a dose worth 10 samples -- a "
        "rounding-short total is being read as a genuinely unfinished dose")


def test_dose_survives_flapping_between_two_neighbouring_rows():
    # REGRESSION: found analysing a real freehand print whose recorded
    # coverage.png showed far less than what was actually inked on paper.
    # NOZZLE_PITCH_MM (~0.087mm) is finer than realistic tracker position
    # noise, so a nozzle sitting near a row boundary has its rounded row
    # index flap between two neighbours sample to sample. The engine used
    # to key the dose on a per-group slot that RESET on every key change --
    # so neither neighbour ever collected a full dose, even though `active`
    # (and therefore real firmware firing) was True on literally every
    # sample. Reproduced directly against the pre-fix engine: 200/200
    # samples fired, 0 pixels ever completed, from jitter of only +-0.001mm
    # -- two orders of magnitude below plausible tracker noise. The dose
    # must be tracked per PIXEL (persists across a key flap) rather than
    # per group-slot (reset by one).
    ink = np.zeros((200, 50), dtype=bool)
    ink[50, 10] = True
    ink[51, 10] = True
    eng = CoverageEngine(ink, mm_per_column=0.2, drops_per_pixel=DROPS)
    v_center = 50.5 * NOZZLE_PITCH_MM       # exact row-50/51 rounding boundary
    t = 0.0
    fired = 0
    for i in range(200):
        jitter = 0.001 if i % 2 == 0 else -0.001
        pattern, _ = eng.step(u_mm=2.0, v_mm=v_center + jitter, t=t)
        if any(pattern):
            fired += 1
        t += 0.002                          # 500 Hz poll

    # Firing must have happened at all -- the pre-fix engine fired on every
    # single one of these 200 samples (see the MUTATION check below) while
    # completing nothing; the fixed engine fires only until both pixels
    # complete, then correctly stops (nothing left wanted) -- so this is a
    # lower bound, not the full 200.
    assert fired >= 2, "sanity: the nozzle must have fired at least once"
    assert eng.printed[50, 10], (
        "row 50 never completed despite firing on every single sample -- "
        "its dose was lost to key-flapping between the two boundary rows")
    assert eng.printed[51, 10], (
        "row 51 never completed despite firing on every single sample -- "
        "its dose was lost to key-flapping between the two boundary rows")


def test_dose_resumes_after_the_group_stops_being_wanted_for_a_while():
    # A pixel partially dosed, then the cart wanders away (group not wanted
    # for many samples -- e.g. off the page, or over an already-printed/blank
    # stretch), then returns to the SAME still-unfinished pixel: the earlier
    # partial dose must still count. Distinct from the flapping test above
    # (that one never leaves the group_wanted branch at all); this one
    # specifically exercises the "not wanted" continue path not silently
    # discarding progress.
    ink = np.zeros((10, 10), dtype=bool)
    ink[0, 0] = True
    eng = CoverageEngine(ink, mm_per_column=1.0, drops_per_pixel=6)

    eng.step(u_mm=0.0, v_mm=0.0, t=0.0)                 # 1 of 6 at (0,0)
    eng.step(u_mm=0.0, v_mm=0.0, t=0.006)               # 2 of 6, not yet done
    assert not eng.printed[0, 0]

    # Wander off to unwanted territory for a while (group_wanted False).
    for i in range(20):
        eng.step(u_mm=5.0, v_mm=5.0, t=0.006 + i * 0.001)

    # Come back to (0,0): only 3 more drops needed to reach the report point.
    for i in range(3):
        eng.step(u_mm=0.0, v_mm=0.0, t=0.030 + 0.004 * i)
    assert eng.printed[0, 0], (
        "the dose accumulated before the excursion was discarded instead of "
        "resumed on return")


def test_dose_flap_MUTATION_check_resetting_on_key_change_reintroduces_the_bug():
    # Proof the regression test above actually exercises the fix: reverting
    # to the old reset-on-key-change rule (a per-group slot dropped the
    # moment the key moves, exactly what step() did before this fix)
    # reproduces the original failure -- fires every sample, completes
    # neither pixel.
    ink = np.zeros((200, 50), dtype=bool)
    ink[50, 10] = True
    ink[51, 10] = True

    printed = np.zeros_like(ink, dtype=bool)
    pixel, konto = None, 0.0
    v_center = 50.5 * NOZZLE_PITCH_MM
    fired = 0
    for i in range(200):
        v = v_center + (0.001 if i % 2 == 0 else -0.001)
        row = int(round(v / NOZZLE_PITCH_MM))
        col = int(round(2.0 / 0.2))
        wanted = bool(ink[row, col]) and not printed[row, col]
        if wanted:
            key = (row, col)
            if pixel != key:
                pixel, konto = key, 0.0        # <-- the reverted, buggy reset
            konto += 1.0
            fired += 1
            if konto + 1.0 >= DROPS:
                printed[row, col] = True
        else:
            pixel, konto = None, 0.0

    assert fired == 200, "sanity: same firing pattern as the fixed engine"
    assert not printed[50, 10] and not printed[51, 10], (
        "the old reset-on-key-change rule was expected to still fail here -- "
        "if this now passes, the mutation no longer reproduces the original "
        "bug and this guard should be revisited")


def test_ink_not_requested_never_fires_or_prints():
    ink = np.zeros((10, 5), dtype=bool)          # nothing wanted anywhere
    eng = CoverageEngine(ink, mm_per_column=1.0, drops_per_pixel=DROPS)

    pattern, changed = eng.step(u_mm=0.0, v_mm=0.0, t=0.0)
    assert not _unpack(pattern).any()
    assert changed                                # first call always reports a change
    pattern, changed = eng.step(u_mm=0.0, v_mm=0.0, t=1.0)
    assert not changed                            # still all-zero -> no change this time
    eng.step(u_mm=0.0, v_mm=0.0, t=10.0)
    assert not eng.printed[0, 0]


# ============================================================== bar geometry
def test_multiple_nozzles_fire_together_when_all_wanted():
    ink = np.ones((NUM_NOZZLES, 1), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=1.0, drops_per_pixel=DROPS)
    pattern, _ = eng.step(u_mm=0.0, v_mm=0.0, t=0.0)
    assert _unpack(pattern)[:NUM_NOZZLES].all()


def test_tall_image_needs_vertical_travel_to_reach_all_rows():
    height = NUM_NOZZLES + 60
    ink = np.ones((height, 1), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=1.0, drops_per_pixel=DROPS)

    # base_row=0 -> nozzle bar spans rows [0, NUM_NOZZLES). Two drops of
    # three is enough to report (see step()'s Step 5).
    eng.step(u_mm=0.0, v_mm=0.0, t=0.0)
    eng.step(u_mm=0.0, v_mm=0.0, t=0.1)
    assert eng.printed[0, 0]
    assert eng.printed[NUM_NOZZLES - 1, 0]
    assert not eng.printed[NUM_NOZZLES, 0]           # just out of the bar's reach
    assert not eng.printed[height - 1, 0]            # far out of reach

    # Shift vertically so the bar's top edge now reaches the last row.
    v_shift_mm = (height - NUM_NOZZLES) * NOZZLE_PITCH_MM
    eng.step(u_mm=0.0, v_mm=v_shift_mm, t=1.0)
    eng.step(u_mm=0.0, v_mm=v_shift_mm, t=1.1)
    assert eng.printed[height - 1, 0]


def test_out_of_bounds_position_does_not_crash_or_print_anything():
    ink = np.ones((10, 5), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=1.0, drops_per_pixel=DROPS)

    # Far off the image in both u and v.
    pattern, _ = eng.step(u_mm=1000.0, v_mm=1000.0, t=0.0)
    eng.step(u_mm=1000.0, v_mm=1000.0, t=10.0)
    assert not _unpack(pattern).any()
    assert not eng.printed.any()

    # Negative column.
    eng.step(u_mm=-5.0, v_mm=0.0, t=0.0)
    eng.step(u_mm=-5.0, v_mm=0.0, t=10.0)
    assert not eng.printed.any()


# ================================================================= bookkeeping
def test_pattern_is_row_bytes_long():
    ink = np.ones((10, 5), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=1.0, drops_per_pixel=DROPS)
    pattern, _ = eng.step(u_mm=0.0, v_mm=0.0, t=0.0)
    assert len(pattern) == ROW_BYTES


def test_done_property_tracks_full_coverage():
    ink = np.ones((3, 1), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=1.0, drops_per_pixel=DROPS)
    assert not eng.done

    eng.step(u_mm=0.0, v_mm=0.0, t=0.0)
    eng.step(u_mm=0.0, v_mm=0.0, t=0.1)
    assert eng.done                 # bar (152 nozzles) covers all 3 rows at once


def test_done_is_true_for_a_blank_target():
    ink = np.zeros((10, 5), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=1.0, drops_per_pixel=DROPS)
    assert eng.done                 # nothing wanted -> trivially done


# ============================================== yaw / cart-rotation correction
def test_step_default_yaw_rad_reduces_exactly_to_the_pre_rotation_behaviour():
    # Regression guard (every test above depends on this): calling step()
    # without yaw_rad at all must place every nozzle at the same column and
    # at row = base_row + p, exactly as before this feature existed.
    ink = np.ones((NUM_NOZZLES + 10, 5), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=1.0, drops_per_pixel=1)
    eng.step(u_mm=2.0, v_mm=0.0, t=0.0)
    assert eng.printed[0:NUM_NOZZLES, 2].all()
    assert not eng.printed[:, [0, 1, 3, 4]].any()


def test_step_zero_yaw_is_bit_identical_across_a_sweep_of_v_offsets_incl_rounding_boundaries():
    # Floating-point drift guard: recomputing row_p as
    # round((v_mm + p*NOZZLE_PITCH_MM) / NOZZLE_PITCH_MM) instead of the
    # exact integer base_row + p measurably drifts by one nozzle right at
    # rounding boundaries (empirically confirmed while building this fix --
    # about half of exact .5-boundary v_mm values disagree). Sweep including
    # those boundaries and require BIT-IDENTICAL row placement between
    # explicit yaw_rad=0.0 and the pre-rotation base_row + p formula.
    height = NUM_NOZZLES + 5
    for k in range(-50, 50):
        v_mm = (k + 0.5) * NOZZLE_PITCH_MM        # exact rounding boundary
        ink = np.ones((height, 3), dtype=bool)
        eng = CoverageEngine(ink, mm_per_column=1.0, drops_per_pixel=1)
        eng.step(u_mm=1.0, v_mm=v_mm, t=0.0, yaw_rad=0.0)
        base_row = int(round(v_mm / NOZZLE_PITCH_MM))
        expected_rows = {r for r in range(base_row, base_row + NUM_NOZZLES) if 0 <= r < height}
        actual_rows = set(np.nonzero(eng.printed[:, 1])[0].tolist())
        assert actual_rows == expected_rows, (v_mm, base_row, actual_rows, expected_rows)


def test_step_nonzero_yaw_spreads_nozzles_across_columns_by_the_expected_amount():
    mm_per_column = 0.5
    width_cols = 200
    height_rows = 300
    ink = np.ones((height_rows, width_cols), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=mm_per_column, drops_per_pixel=1)
    yaw = math.radians(45.0)

    eng.step(u_mm=50.0, v_mm=5.0, t=0.0, yaw_rad=yaw)

    touched_cols = np.nonzero(eng.printed.any(axis=0))[0]
    assert touched_cols.size > 1, "a nonzero yaw must spread ink across more than one column"
    spread_cols = int(touched_cols.max() - touched_cols.min())

    # bar_length == (NUM_NOZZLES - 1) * NOZZLE_PITCH_MM == NOZZLE_BAR_SPAN_MM
    # exactly (see geometry.py) -- the u-extent across the whole bar is
    # bar_length * sin(yaw_rad), converted to columns.
    expected_spread_cols = NOZZLE_BAR_SPAN_MM * math.sin(yaw) / mm_per_column
    assert abs(spread_cols - expected_spread_cols) <= 1.0, (spread_cols, expected_spread_cols)


def test_step_90deg_yaw_swings_the_bar_to_lower_columns_not_higher():
    # Absolute-geometry pin, not just "the two signs disagree" (that weaker
    # property is what the old, WRONG version of this test checked, and it
    # would have passed just as happily with the sign inverted -- see the
    # PR body / coverage.py's step() docstring for the derivation this
    # value is checked against).
    #
    # In the right-handed page basis {e_col, e_row, n=e_col x e_row}, the
    # bar points along +e_row at yaw_rad==0 (from the row = base_row + p
    # convention: increasing nozzle index p -> increasing row -> increasing
    # v), i.e. its direction vector is (0, 1) in (u, v). Rotating (0, 1) by
    # +theta gives (-sin(theta), cos(theta)) -- so a POSITIVE yaw must swing
    # the far end of the bar to LOWER u (lower columns), not higher.
    #
    # yaw = +90 deg makes this exact and hand-checkable: sin(90deg) == 1.0,
    # cos(90deg) == 0.0 (both exact in IEEE 754), so nozzle NUM_NOZZLES-1
    # (the far end of the bar, offset_along_bar == NOZZLE_BAR_SPAN_MM
    # exactly -- see geometry.py) lands at u = u_mm - NOZZLE_BAR_SPAN_MM,
    # v == v_mm unchanged -- the whole bar stays on nozzle 0's row and
    # spreads purely in u.
    mm_per_column = 0.5
    u_mm, v_mm = 50.0, 0.0
    yaw = math.pi / 2.0

    ink = np.ones((10, 200), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=mm_per_column, drops_per_pixel=1)
    eng.step(u_mm=u_mm, v_mm=v_mm, t=0.0, yaw_rad=yaw)

    row0 = int(round(v_mm / NOZZLE_PITCH_MM))
    col0 = int(round(u_mm / mm_per_column))
    assert eng.printed[row0, col0], "nozzle 0 must stay at its un-rotated (row, col)"

    expected_last_col = col0 - int(round(NOZZLE_BAR_SPAN_MM / mm_per_column))
    touched_cols = np.nonzero(eng.printed[row0])[0]
    actual_last_col = int(touched_cols.min())        # bar swept toward LOWER columns

    assert touched_cols.max() == col0, "nozzle 0 must be the HIGHEST column touched"
    assert abs(actual_last_col - expected_last_col) <= 1, (
        actual_last_col, expected_last_col, "within one column of rounding")
    # explicitly reject the old (wrong) direction, so a re-inversion of the
    # sign fails loudly here even if the tolerance above were ever loosened
    assert actual_last_col < col0


# =============================================================== bar_offset_uv
def test_bar_offset_uv_is_zero_at_zero_offset():
    assert bar_offset_uv(0.0, math.radians(37.0)) == (0.0, 0.0)


def test_bar_offset_uv_stays_pure_v_at_zero_yaw():
    du, dv = bar_offset_uv(12.3, 0.0)
    assert du == 0.0
    assert dv == 12.3


def test_bar_offset_uv_matches_the_bar_centre_of_steps_own_spread():
    # Cross-consistency against CoverageEngine.step()'s own (independently
    # written, NOT calling this function -- see bar_offset_uv's docstring)
    # per-nozzle placement: NUM_NOZZLES is even, so no single nozzle sits
    # exactly at the geometric bar centre (offset_along_bar_mm =
    # NOZZLE_BAR_SPAN_MM / 2, halfway between nozzles 75 and 76) -- but that
    # centre must fall at the midpoint of the column range step() actually
    # spreads across. Same 90 deg exact-arithmetic setup (sin=1, cos=0
    # exactly in IEEE 754) as the "swings to lower columns" test above, so
    # the comparison isn't itself blurred by rounding.
    mm_per_column = 0.5
    u_mm, v_mm = 50.0, 0.0
    yaw = math.pi / 2.0

    ink = np.ones((10, 200), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=mm_per_column, drops_per_pixel=1)
    eng.step(u_mm=u_mm, v_mm=v_mm, t=0.0, yaw_rad=yaw)

    row0 = int(round(v_mm / NOZZLE_PITCH_MM))
    touched_cols = np.nonzero(eng.printed[row0])[0]
    observed_centre_col = (touched_cols.min() + touched_cols.max()) / 2.0

    du, dv = bar_offset_uv(NOZZLE_BAR_SPAN_MM / 2.0, yaw)
    predicted_centre_col = (u_mm + du) / mm_per_column
    assert abs(predicted_centre_col - observed_centre_col) <= 1.0, (
        predicted_centre_col, observed_centre_col)
    # math.pi/2 isn't bit-exact (math.pi is a finite approximation of pi),
    # so cos(yaw) is ~6e-17, not exactly 0.0 -- negligible against
    # NOZZLE_PITCH_MM (~0.087mm) and rounds away in row0 above, but not
    # literally zero; assert "negligible", not "==".
    assert abs(dv) < 1e-9, "bar centre must stay on nozzle 0's row at 90 deg yaw"


def test_step_and_page_mapper_rotate_a_body_fixed_row_vector_the_same_direction():
    # The property that actually broke here: PageMapper's sensor->nozzle
    # lever-arm offset and CoverageEngine's per-nozzle bar placement are
    # both body-fixed vectors on the same cart (a fixed offset along the
    # +row/+p direction from a reference point on the cart), so under the
    # SAME yaw they must shift u the SAME way -- left uncorrected, the two
    # rotating corrections would pull u in opposite directions on a
    # rotating pass, which is worse than applying neither.
    yaw = math.radians(30.0)
    offset_mm = 10.0

    # PageMapper side: du for a pure +row-offset vector (col offset 0) at
    # this yaw -- same construction as test_page_mapper.py's 90-degree test.
    cal = PageCalibration(origin=np.zeros(3), e_col=np.array([1.0, 0.0, 0.0]),
                          e_row=np.array([0.0, 1.0, 0.0]),
                          boresight_quat=np.array([0.0, 0.0, 0.0, 1.0]))
    mapper = PageMapper(cal, sensor_offset_row_mm=offset_mm + NOZZLE_BAR_SPAN_MM / 2.0,
                        sensor_offset_col_mm=0.0)
    half = yaw / 2.0
    quat = (0.0, 0.0, math.sin(half), math.cos(half))
    pos = np.zeros(3)
    u_raw, _, _ = cal.project(pos)
    u_mapped, _, _ = mapper.project(pos, quat=quat)
    mapper_du = u_mapped - u_raw
    assert mapper_du != 0.0, "test is vacuous if the offset doesn't move u at all"

    # CoverageEngine side: at zero yaw the bar points along +e_row (the same
    # sense as the positive row offset above -- see step()'s docstring), so
    # the far end of the bar (nozzle NUM_NOZZLES-1, the largest along-bar
    # offset) must shift u the same SIGN as mapper_du under the same yaw.
    mm_per_column = 0.5
    u_mm, v_mm = 50.0, 5.0
    ink = np.ones((300, 200), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=mm_per_column, drops_per_pixel=1)
    eng.step(u_mm=u_mm, v_mm=v_mm, t=0.0, yaw_rad=yaw)
    col0 = int(round(u_mm / mm_per_column))
    touched_cols = np.nonzero(eng.printed.any(axis=0))[0]
    farthest_col = int(touched_cols[np.argmax(np.abs(touched_cols - col0))])
    engine_du_sign_negative = farthest_col < col0

    assert (mapper_du < 0.0) == engine_du_sign_negative, (
        mapper_du, farthest_col, col0,
        "PageMapper and CoverageEngine disagree about which way a "
        "body-fixed row-axis vector on the cart rotates -- exactly the "
        "inconsistency the original sign error introduced")


# ========================================================= nozzle grouping
def test_nozzle_group_1_is_bit_identical_to_no_group_param_at_all():
    # G=1 must reduce to EXACTLY today's per-nozzle behaviour -- the most
    # important correctness property of this feature. Drive two engines (one
    # built with nozzle_group=1 explicit, one with the parameter omitted
    # entirely, i.e. whatever the class default is) through the SAME
    # scripted multi-sample run and require every returned pattern, and both
    # final printed masks, to match exactly.
    height = NUM_NOZZLES + 40
    ink = np.zeros((height, 40), dtype=bool)
    ink[::3, ::2] = True   # deterministic scattered target (not all-true/all-false)

    # Same v-step/sample-count shape as
    # test_spray_can_only_ever_add_coverage_never_remove_it above, which is
    # known to actually accumulate coverage at this dose -- a much faster v
    # sweep (or too few samples) leaves each (row, col) key too short-lived
    # to ever complete a dose, making the comparison vacuous.
    samples = [(u, 2.0 + 0.02 * i, 0.001 * i)
               for i, u in enumerate(np.linspace(0.0, 6.0, 400))]

    def run(**kwargs):
        eng = CoverageEngine(ink, mm_per_column=0.2, drops_per_pixel=2, **kwargs)
        seen = [eng.step(u_mm=u, v_mm=v, t=t)[0] for u, v, t in samples]
        return seen, eng.printed.copy()

    patterns_default, printed_default = run()
    patterns_g1, printed_g1 = run(nozzle_group=1)

    assert printed_default.any(), "test is vacuous if nothing got printed"
    assert patterns_default == patterns_g1, (
        "nozzle_group=1 must return byte-identical patterns to the "
        "no-argument default, sample for sample")
    assert np.array_equal(printed_default, printed_g1), (
        "nozzle_group=1 must leave an identical printed mask to the "
        "no-argument default")

    # The check above only proves the default IS 1 -- both runs go through
    # the same grouped code path, so on its own it cannot catch the grouping
    # rewrite changing behaviour. Compare against an INDEPENDENT
    # reimplementation of the per-nozzle rule (the code this replaced) to
    # actually pin the equivalence.
    #
    # The dose here is accumulated PER PIXEL (a plain dict, keyed on
    # (row, col), never reset by a key change), matching step()'s own
    # per-pixel model -- NOT the earlier per-nozzle slot that reset to zero
    # the instant a nozzle's rounded key changed. That earlier version is
    # what this test used to pin, and it was itself the bug: with
    # NOZZLE_PITCH_MM (~0.087mm) finer than realistic tracker noise, a nozzle
    # hovering near a row boundary flaps its key every sample, so a
    # reset-on-change accumulator never reaches a full dose -- the nozzle
    # fires every sample (see `active` below, set unconditionally once
    # `wanted`) but no pixel is ever marked printed. Reproduced directly
    # against the pre-fix engine: 200/200 samples fired, 0 pixels completed,
    # from +-0.001mm jitter alone. See coverage.py's module/__init__
    # docstrings.
    def reference(yaw_rad):
        """The per-nozzle rule, written out standalone, with the same
        never-reset-on-flap per-pixel dose accumulation step() now uses --
        including the two thresholds: `printed` is reported one drop early,
        the ledger entry (and with it the nozzle) is only released on the
        full dose."""
        dose_soll = 2                      # drops_per_pixel used above
        printed = np.zeros_like(ink, dtype=bool)
        konten: "dict[tuple[int, int], float]" = {}
        out = []
        for u_mm, v_mm, t in samples:
            active = np.zeros(NUM_NOZZLES, dtype=bool)
            zero_yaw = yaw_rad == 0.0
            if zero_yaw:
                col_fixed = int(round(u_mm / 0.2))
                base_row = int(round(v_mm / NOZZLE_PITCH_MM))
            for p in range(NUM_NOZZLES):
                if zero_yaw:
                    col, row = col_fixed, base_row + p
                else:
                    d = p * NOZZLE_PITCH_MM
                    col = int(round((u_mm - d * math.sin(yaw_rad)) / 0.2))
                    row = int(round((v_mm + d * math.cos(yaw_rad)) / NOZZLE_PITCH_MM))
                in_bounds = 0 <= row < ink.shape[0] and 0 <= col < ink.shape[1]
                pixel = (row, col)
                wanted = in_bounds and bool(ink[row, col]) and (
                    not printed[row, col] or pixel in konten)
                if not wanted:
                    continue
                treffer = konten.get(pixel, 0.0) + 1.0     # step()'s default drops
                active[p] = True
                if treffer >= dose_soll:
                    konten.pop(pixel, None)
                else:
                    konten[pixel] = treffer
                if treffer + 1.0 >= dose_soll:
                    printed[row, col] = True
            out.append(pack_nozzle_bits(active))
        return out, printed

    for yaw in (0.0, math.radians(30.0)):
        eng = CoverageEngine(ink, mm_per_column=0.2, drops_per_pixel=2,
                             nozzle_group=1)
        got = [eng.step(u_mm=u, v_mm=v, t=t, yaw_rad=yaw)[0] for u, v, t in samples]
        ref_patterns, ref_printed = reference(yaw)
        assert got == ref_patterns, (
            f"nozzle_group=1 diverged from the original per-nozzle rule at "
            f"yaw={yaw}")
        assert np.array_equal(eng.printed, ref_printed), (
            f"nozzle_group=1 printed mask diverged from the original rule at "
            f"yaw={yaw}")


def test_nozzle_group_2_fires_both_members_when_only_one_row_is_wanted():
    # Single ink row -> only nozzle 0 (row 0) individually "wants" ink;
    # nozzle 1 (row 1, same group) does not. The OR rule must still fire both.
    ink = np.zeros((NUM_NOZZLES, 1), dtype=bool)
    ink[0, 0] = True
    eng = CoverageEngine(ink, mm_per_column=1.0, drops_per_pixel=DROPS, nozzle_group=2)

    pattern, _ = eng.step(u_mm=0.0, v_mm=0.0, t=0.0)
    active = _unpack(pattern)
    assert active[0] and active[1], "both nozzles of the group must fire together"
    assert not active[2:].any(), "only group 0 (rows 0/1) should be firing"


def test_nozzle_group_2_marks_both_rows_printed_on_dose_completion():
    ink = np.zeros((NUM_NOZZLES, 1), dtype=bool)
    ink[0, 0] = True                        # row 1 is not itself wanted ink
    eng = CoverageEngine(ink, mm_per_column=1.0, drops_per_pixel=DROPS, nozzle_group=2)

    eng.step(u_mm=0.0, v_mm=0.0, t=0.0)
    eng.step(u_mm=0.0, v_mm=0.0, t=0.1)          # 2 of 3 drops -> reported

    assert eng.printed[0, 0], "the actually-wanted row must be printed"
    assert eng.printed[1, 0], (
        "row 1 physically received ink too (tied to row 0's nozzle), so it "
        "must also be marked printed even though it wasn't itself wanted")


def test_nozzle_group_2_neither_member_fires_when_neither_is_wanted():
    ink = np.zeros((NUM_NOZZLES, 1), dtype=bool)   # nothing wanted anywhere
    eng = CoverageEngine(ink, mm_per_column=1.0, drops_per_pixel=DROPS, nozzle_group=2)

    pattern, _ = eng.step(u_mm=0.0, v_mm=0.0, t=0.0)
    active = _unpack(pattern)
    assert not active[0] and not active[1]
    eng.step(u_mm=0.0, v_mm=0.0, t=0.1)
    assert not eng.printed.any()


def test_nozzle_group_2_under_yaw_fires_both_members_without_crashing():
    # Under yaw, group members can legitimately land in different columns
    # (see step()'s docstring) -- that must not crash the grouping logic,
    # and the group must still fire together.
    mm_per_column = 0.5
    ink = np.ones((300, 200), dtype=bool)
    yaw = math.radians(45.0)
    eng = CoverageEngine(ink, mm_per_column=mm_per_column, drops_per_pixel=DROPS,
                         nozzle_group=2)

    pattern, _ = eng.step(u_mm=50.0, v_mm=5.0, t=0.0, yaw_rad=yaw)
    active = _unpack(pattern)
    assert active[0] and active[1], "both members of group 0 must fire together under yaw"


def test_nozzle_group_2_MUTATION_check_firing_only_the_first_member_breaks_the_both_fire_test():
    # Inlines a broken alternative implementation: a step() that computes
    # the group-wanted OR correctly but only sets active[] for the group's
    # FIRST member, not every member -- reproduced directly here (not by
    # editing coverage.py), so this regression stays covered by the test
    # suite even if nobody re-derives it by hand.
    ink = np.zeros((NUM_NOZZLES, 1), dtype=bool)
    ink[0, 0] = True                # only nozzle 0 (group 0's first member) wants ink
    eng = CoverageEngine(ink, mm_per_column=1.0, drops_per_pixel=DROPS, nozzle_group=2)

    pattern, _ = eng.step(u_mm=0.0, v_mm=0.0, t=0.0)
    real_active = _unpack(pattern)[:2].copy()

    mutated_active = real_active.copy()
    mutated_active[1] = 0           # the mutation: group's second nozzle never fires

    assert real_active[1] == 1, "the real engine must fire BOTH members of the group"
    assert not np.array_equal(real_active, mutated_active), (
        "the mutated (first-member-only) firing pattern must disagree with "
        "the real engine's -- if this ever matches, the 'both fire' test "
        "above has stopped actually exercising nozzle_group's OR/fire-"
        "together rule")


# =============================================== mutation check (see PR body)
def test_step_MUTATION_check_ignoring_yaw_rad_breaks_the_spread_test():
    # Inlines the mutation described in the PR: a step() that accepts
    # yaw_rad but never uses it (drops back to the single shared `col`)
    # would place every nozzle in the same column regardless of yaw --
    # reproduced here directly rather than by editing coverage.py, so the
    # regression stays covered by the test suite.
    mm_per_column = 0.5
    ink = np.ones((300, 200), dtype=bool)
    yaw = math.radians(45.0)
    u_mm, v_mm = 50.0, 5.0

    col_ignoring_yaw = int(round(u_mm / mm_per_column))
    mutated_touched_cols = {col_ignoring_yaw}          # every nozzle, same column

    eng = CoverageEngine(ink, mm_per_column=mm_per_column, drops_per_pixel=1)
    eng.step(u_mm=u_mm, v_mm=v_mm, t=0.0, yaw_rad=yaw)
    real_touched_cols = set(np.nonzero(eng.printed.any(axis=0))[0].tolist())

    assert real_touched_cols != mutated_touched_cols, (
        "the mutated (yaw-ignoring) engine must disagree with the real one "
        "-- if this ever matches, the spread test above has stopped "
        "actually exercising yaw_rad")


# ============================================ fired (physical ink) vs printed
def test_fired_marks_a_pixel_on_its_very_first_sample_before_any_dose_completes():
    # The core semantic: a nozzle puts ink down the instant its pixel is
    # wanted (active[p]), NOT when the dose finishes. `fired` records that;
    # `printed` deliberately does not.
    ink = np.zeros((NUM_NOZZLES, 3), dtype=bool)
    ink[0, 0] = True
    eng = CoverageEngine(ink, mm_per_column=1.0, drops_per_pixel=DROPS)

    pattern, _ = eng.step(0.0, 0.0, 0.0)          # one single sample
    assert _unpack(pattern)[0] == 1, "nozzle 0 must fire on the first sample"
    assert eng.fired[0, 0], "fired must record that first firing"
    assert not eng.printed[0, 0], "dose cannot be complete after one sample"


def test_fired_never_marks_a_pixel_no_nozzle_ever_visited():
    ink = np.ones((NUM_NOZZLES, 4), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=1.0, drops_per_pixel=DROPS)
    for i in range(5):
        eng.step(0.0, 0.0, i * 0.05)              # parked on column 0 only
    assert eng.fired[:, 0].any()
    assert not eng.fired[:, 1:].any(), "columns never driven over must stay clean"


def test_fired_equals_printed_on_a_realistic_pass():
    # On a healthy pass driven the way the controller drives it, the two
    # pictures are identical. Under the old time-based dose this only held
    # at low speed; it now holds all the way to the poll-rate limit.
    mm_per_column = 0.087
    for speed in (5.0, 17.3, 30.0, 40.0):
        eng = CoverageEngine(np.ones((NUM_NOZZLES, 60), dtype=bool),
                             mm_per_column=mm_per_column)
        _sweep(eng, speed, mm_per_column)
        assert np.array_equal(eng.fired, eng.printed), (
            speed, int(eng.fired.sum()), int(eng.printed.sum()))


def test_a_partial_dose_inks_the_pixel_while_printed_still_says_no():
    # What `fired` is FOR, in the one case that still produces it: the cart
    # crosses a column and turns away before the dose is complete (a
    # direction reversal, or the page edge). The nozzle fired, so there is
    # ink on the paper, but the pixel is not fully dosed -- COVERED and THIN
    # must be able to say both.
    ink = np.ones((NUM_NOZZLES, 3), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=1.0, drops_per_pixel=6)

    eng.step(0.0, 0.0, 0.0, drops=1.0)            # 1 of 6, then gone
    assert eng.fired[0, 0], "the nozzle fired, so ink landed"
    assert not eng.printed[0, 0], "one drop of six is not a completed dose"


def test_MUTATION_check_crediting_integer_columns_reintroduces_speed_stripes():
    # Guards the fractional-credit rule in step()'s Step 4. Re-drives the
    # same 25 mm/s sweep crediting the engine with whole BLE columns (what
    # the link actually carried that sample) instead of the exact share of a
    # dose, and requires that to come out visibly worse -- otherwise the
    # distinction the code makes is not being exercised here.
    mm_per_column, poll_hz, speed = 0.087, 500.0, 25.0
    dt = 1.0 / poll_hz
    dpp = DEFAULT_DROPS_PER_PIXEL

    def run(integer_credit):
        eng = CoverageEngine(np.ones((40, 120), dtype=bool),
                             mm_per_column=mm_per_column)
        schuld, vorher = 0.0, None
        n = int(120 * mm_per_column / (speed * dt)) + 1
        for i in range(n):
            u = i * speed * dt
            anteil = float(dpp) if vorher is None else dpp * (u - vorher) / mm_per_column
            vorher = u
            schuld += anteil
            kopien = int(schuld)
            schuld -= kopien
            eng.step(u_mm=u, v_mm=0.0, t=i * dt,
                     drops=(kopien if integer_credit else anteil))
        return eng.printed.sum() / eng.printed.size

    exakt = run(False)
    ganzzahlig = run(integer_credit=True)
    assert exakt == 1.0, f"the shipped rule must cover everything, got {exakt:.1%}"
    assert ganzzahlig < exakt, (
        "crediting whole BLE columns was expected to leave columns unprinted "
        f"at {speed} mm/s but reported {ganzzahlig:.1%} -- if the two rules "
        "now agree, Step 4's fractional-credit argument needs revisiting")


def test_fired_records_every_member_of_a_firing_group_not_just_the_first():
    # nozzle_group=2 fires both members together (OR rule), so both really
    # do put ink down -- fired must say so, same reasoning as _deposit being
    # called for every in-bounds member on completion.
    ink = np.zeros((NUM_NOZZLES, 2), dtype=bool)
    ink[0, 0] = True                              # only nozzle 0's pixel wanted
    eng = CoverageEngine(ink, mm_per_column=1.0, drops_per_pixel=DROPS,
                         nozzle_group=2)
    eng.step(0.0, 0.0, 0.0)
    assert eng.fired[0, 0] and eng.fired[1, 0], \
        "both group members fire, so both cells receive ink"



if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print(f"OK: {name}")
    print("All coverage tests passed.")
