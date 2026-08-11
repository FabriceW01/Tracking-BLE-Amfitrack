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
    CoverageEngine, DEFAULT_DOSE_HOLD_S, bar_offset_uv,
)
from printhead.geometry import (                                       # noqa: E402
    NOZZLE_BAR_WIDTH_MM, NOZZLE_PITCH_MM, NUM_NOZZLES, ROW_BYTES,
)
from printhead.rendering import pack_nozzle_bits                       # noqa: E402
from printhead.tracking import PageMapper                              # noqa: E402

DOSE_HOLD_S = 0.1


def _unpack(pattern: bytes) -> np.ndarray:
    return np.unpackbits(np.frombuffer(pattern, dtype=np.uint8), bitorder="little")


# ================================================== measured dose-hold tuning
def test_default_dose_hold_tracks_the_firmware_pattern_stride():
    # DEFAULT_DOSE_HOLD_S is derived from -- and MUST stay in sync with --
    # the firmware's PATTERN_STRIDE (src/ble_dose.h in the firmware repo):
    # DOSE_HOLD_S ~= 3 * PATTERN_STRIDE * 450e-6 (3 = BLE_DROPS_PER_COLUMN,
    # 450us = the firmware print loop tick, PATTERN_STRIDE = 3). If someone
    # changes DEFAULT_DOSE_HOLD_S here without changing PATTERN_STRIDE (and
    # re-flashing) to match, or vice versa, the ~3-drop-per-pixel target this
    # pair was tuned to breaks silently on hardware -- this test is the loud
    # failure meant to catch that on the client side.
    #
    # CORRECTION: an earlier pick (PATTERN_STRIDE=4, DOSE_HOLD_S=0.0054 s)
    # also hit this ~3-drop target but landed just ABOVE the 5.00 ms poll
    # interval of the default --poll-hz 200, which requires a third sample
    # to land on the same column to complete a dose -- measured coverage
    # collapsed from 100% (at 4.90 ms) to 31% (at 5.40 ms). The additional
    # constraint that adds, on top of the 3-drop target: DOSE_HOLD_S must
    # stay below 1/poll_hz (the poll interval), or two consecutive samples
    # are not enough to complete a dose. 0.00405 s is 19% below the 5.00 ms
    # default poll interval.
    firmware_pattern_stride = 3
    drops_per_column = 3          # BLE_DROPS_PER_COLUMN, line mode's dose target
    tick_s = 450e-6                # firmware print loop tick
    expected = drops_per_column * firmware_pattern_stride * tick_s
    assert abs(DEFAULT_DOSE_HOLD_S - expected) < 1e-6, (
        f"DEFAULT_DOSE_HOLD_S={DEFAULT_DOSE_HOLD_S} no longer matches "
        f"3 * PATTERN_STRIDE({firmware_pattern_stride}) * 450us = {expected} "
        "-- update the firmware's PATTERN_STRIDE (src/ble_dose.h) to match, "
        "or this comment/test, and re-flash the firmware")

    # The additional poll-interval constraint above, pinned directly: with
    # the default poll_hz=200 (5.00 ms interval), the hold must stay below
    # it or coverage collapses (see coverage.py's DEFAULT_DOSE_HOLD_S
    # comment for the measured cliff).
    default_poll_hz = 200.0
    assert DEFAULT_DOSE_HOLD_S < 1.0 / default_poll_hz, (
        f"DEFAULT_DOSE_HOLD_S={DEFAULT_DOSE_HOLD_S} is not below the "
        f"{1.0 / default_poll_hz}s poll interval at poll_hz={default_poll_hz} "
        "-- this is the quantization cliff that made the previous 0.0054s "
        "value collapse coverage to ~31%")


def test_realistic_median_dwell_completes_with_the_new_default():
    # 0.2 mm column at the measured median hand speed (17.3 mm/s) dwells for
    # about 11.6 ms -- comfortably above the new 4.05 ms default, so the
    # pixel must be marked printed.
    ink = np.ones((10, 5), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=1.0, dose_hold_s=DEFAULT_DOSE_HOLD_S)
    median_dwell_s = 0.2 / 17.3

    eng.step(u_mm=0.0, v_mm=0.0, t=0.0)
    eng.step(u_mm=0.0, v_mm=0.0, t=median_dwell_s)
    assert eng.printed[0, 0]


def test_short_dwell_above_49mms_stays_unprinted_with_the_new_default():
    # A 3 ms dwell corresponds to roughly 0.2/0.003 ~= 67 mm/s, well above
    # the ~49 mm/s point (0.2 / 0.00405) where the new default hold no
    # longer fits inside one column's crossing time -- the pixel must stay
    # open for a later pass rather than being marked printed early.
    ink = np.ones((10, 5), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=1.0, dose_hold_s=DEFAULT_DOSE_HOLD_S)
    short_dwell_s = 0.003

    eng.step(u_mm=0.0, v_mm=0.0, t=0.0)
    eng.step(u_mm=0.0, v_mm=0.0, t=short_dwell_s)
    assert not eng.printed[0, 0]


# ======================================================= ink spread ("spray")
def _spray_engine(radius_mm, strength, mm_per_column=0.2, size=(41, 41)):
    return CoverageEngine(np.ones(size, dtype=bool), mm_per_column=mm_per_column,
                          dose_hold_s=DOSE_HOLD_S, spray_radius_mm=radius_mm,
                          spray_strength=strength)


def test_spray_is_off_by_default():
    # The whole feature must be opt-in: with no spray arguments the kernel is
    # empty and a deposit touches exactly one pixel, i.e. bit-identical to the
    # pre-spray engine.
    eng = CoverageEngine(np.ones((21, 21), dtype=bool), mm_per_column=0.2,
                         dose_hold_s=DOSE_HOLD_S)
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
    # A cell is NOZZLE_PITCH_MM (~0.0993mm) tall but mm_per_column (0.2mm)
    # wide, so a round drop must reach further in ROWS than in COLUMNS. A
    # pixel-count radius would silently mean two different real distances.
    kernel = _spray_engine(0.2, 1.0, mm_per_column=0.2)._spray_kernel
    max_dr = max(abs(dr) for dr, _, _ in kernel)
    max_dc = max(abs(dc) for _, dc, _ in kernel)
    assert max_dr > max_dc, (max_dr, max_dc)
    assert abs(max_dr - round(0.2 / NOZZLE_PITCH_MM)) <= 1

    # Square-ish cells (0.1mm/col ~= the 0.0993mm row pitch) must instead
    # give a near-symmetric kernel -- proving it tracks physical mm, not
    # an axis-specific fudge.
    square = _spray_engine(0.2, 1.0, mm_per_column=0.1)._spray_kernel
    assert abs(max(abs(dr) for dr, _, _ in square)
               - max(abs(dc) for _, dc, _ in square)) <= 1


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
        eng = CoverageEngine(ink, mm_per_column=0.2, dose_hold_s=0.0018,
                             spray_radius_mm=radius, spray_strength=strength)
        for u, v, t in samples:
            eng.step(u_mm=u, v_mm=v, t=t)
        return eng.printed.copy()

    plain = run(0.0, 0.0)
    sprayed = run(0.15, 1.0)
    assert plain.any(), "test is vacuous if the plain run printed nothing"
    assert np.all(sprayed | ~plain), "spray must never un-print a pixel"
    assert sprayed.sum() >= plain.sum()


# ============================================================== basic dosing
def test_single_pass_completes_after_dose_hold_elapses():
    ink = np.ones((10, 5), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=1.0, dose_hold_s=DOSE_HOLD_S)

    eng.step(u_mm=0.0, v_mm=0.0, t=0.0)               # first touch
    assert not eng.printed[0, 0]                       # not yet dosed
    eng.step(u_mm=0.0, v_mm=0.0, t=DOSE_HOLD_S + 0.05)  # dwelled long enough
    assert eng.printed[0, 0]


def test_nozzle_stops_firing_once_its_pixel_is_printed():
    ink = np.ones((10, 5), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=1.0, dose_hold_s=DOSE_HOLD_S)

    eng.step(u_mm=0.0, v_mm=0.0, t=0.0)
    pattern, _ = eng.step(u_mm=0.0, v_mm=0.0, t=DOSE_HOLD_S + 0.05)
    assert _unpack(pattern)[0] == 1          # fires through the completing sample
    pattern, changed = eng.step(u_mm=0.0, v_mm=0.0, t=DOSE_HOLD_S + 0.1)
    assert _unpack(pattern)[0] == 0          # then stops
    assert changed


def test_revisit_does_not_refire_an_already_printed_pixel():
    ink = np.ones((10, 5), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=1.0, dose_hold_s=DOSE_HOLD_S)

    eng.step(u_mm=0.0, v_mm=0.0, t=0.0)
    eng.step(u_mm=0.0, v_mm=0.0, t=DOSE_HOLD_S + 0.05)   # (0,0) printed
    assert eng.printed[0, 0]

    eng.step(u_mm=3.0, v_mm=0.0, t=0.5)                  # move away
    pattern, _ = eng.step(u_mm=0.0, v_mm=0.0, t=0.5 + DOSE_HOLD_S + 0.5)  # revisit, dwell plenty
    assert _unpack(pattern)[0] == 0                      # never refires
    assert eng.printed[0, 0]                             # stays printed


def test_loop_with_a_longer_second_pass_achieves_coverage():
    # First pass is too fast (dwell < threshold) -> not printed. A slower
    # "loop back" pass completes it. Models "keep going in circles until it
    # takes" rather than a single continuous dwell.
    ink = np.ones((10, 5), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=1.0, dose_hold_s=DOSE_HOLD_S)

    eng.step(u_mm=0.0, v_mm=0.0, t=0.0)
    eng.step(u_mm=0.0, v_mm=0.0, t=DOSE_HOLD_S * 0.5)     # left too early
    assert not eng.printed[0, 0]

    eng.step(u_mm=3.0, v_mm=0.0, t=1.0)                   # loop away
    eng.step(u_mm=0.0, v_mm=0.0, t=2.0)                   # loop back
    eng.step(u_mm=0.0, v_mm=0.0, t=2.0 + DOSE_HOLD_S + 0.05)  # this time it dwells
    assert eng.printed[0, 0]


def test_completion_depends_on_wall_clock_dwell_not_sample_count():
    # Many samples packed into a tiny time window must not fool the engine
    # into completing the dose early -- only elapsed time matters.
    ink = np.ones((10, 5), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=1.0, dose_hold_s=DOSE_HOLD_S)

    for i in range(50):
        eng.step(u_mm=0.0, v_mm=0.0, t=0.001 * i)          # 50 samples, 49ms span
    assert not eng.printed[0, 0]

    eng.step(u_mm=0.0, v_mm=0.0, t=DOSE_HOLD_S + 0.2)       # real dwell time passes
    assert eng.printed[0, 0]


def test_dose_does_not_accumulate_across_different_pixels():
    ink = np.ones((10, 5), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=1.0, dose_hold_s=DOSE_HOLD_S)

    eng.step(u_mm=0.0, v_mm=0.0, t=0.0)                      # pixel (0,0)
    eng.step(u_mm=0.0, v_mm=0.0, t=DOSE_HOLD_S * 0.5)         # 50% dwelled
    eng.step(u_mm=1.0, v_mm=0.0, t=DOSE_HOLD_S * 0.5)         # pixel (0,1)
    eng.step(u_mm=1.0, v_mm=0.0, t=DOSE_HOLD_S * 1.0)         # 50% dwelled

    assert not eng.printed[0, 0]
    assert not eng.printed[0, 1]


def test_ink_not_requested_never_fires_or_prints():
    ink = np.zeros((10, 5), dtype=bool)          # nothing wanted anywhere
    eng = CoverageEngine(ink, mm_per_column=1.0, dose_hold_s=DOSE_HOLD_S)

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
    eng = CoverageEngine(ink, mm_per_column=1.0, dose_hold_s=DOSE_HOLD_S)
    pattern, _ = eng.step(u_mm=0.0, v_mm=0.0, t=0.0)
    assert _unpack(pattern)[:NUM_NOZZLES].all()


def test_tall_image_needs_vertical_travel_to_reach_all_rows():
    height = NUM_NOZZLES + 60
    ink = np.ones((height, 1), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=1.0, dose_hold_s=DOSE_HOLD_S)

    # base_row=0 -> nozzle bar spans rows [0, NUM_NOZZLES). Dwell long enough.
    eng.step(u_mm=0.0, v_mm=0.0, t=0.0)
    eng.step(u_mm=0.0, v_mm=0.0, t=DOSE_HOLD_S + 0.05)
    assert eng.printed[0, 0]
    assert eng.printed[NUM_NOZZLES - 1, 0]
    assert not eng.printed[NUM_NOZZLES, 0]           # just out of the bar's reach
    assert not eng.printed[height - 1, 0]            # far out of reach

    # Shift vertically so the bar's top edge now reaches the last row.
    v_shift_mm = (height - NUM_NOZZLES) * NOZZLE_PITCH_MM
    eng.step(u_mm=0.0, v_mm=v_shift_mm, t=1.0)
    eng.step(u_mm=0.0, v_mm=v_shift_mm, t=1.0 + DOSE_HOLD_S + 0.05)
    assert eng.printed[height - 1, 0]


def test_out_of_bounds_position_does_not_crash_or_print_anything():
    ink = np.ones((10, 5), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=1.0, dose_hold_s=DOSE_HOLD_S)

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
    eng = CoverageEngine(ink, mm_per_column=1.0, dose_hold_s=DOSE_HOLD_S)
    pattern, _ = eng.step(u_mm=0.0, v_mm=0.0, t=0.0)
    assert len(pattern) == ROW_BYTES


def test_done_property_tracks_full_coverage():
    ink = np.ones((3, 1), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=1.0, dose_hold_s=DOSE_HOLD_S)
    assert not eng.done

    eng.step(u_mm=0.0, v_mm=0.0, t=0.0)
    eng.step(u_mm=0.0, v_mm=0.0, t=DOSE_HOLD_S + 0.05)
    assert eng.done                 # bar (152 nozzles) covers all 3 rows at once


def test_done_is_true_for_a_blank_target():
    ink = np.zeros((10, 5), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=1.0, dose_hold_s=DOSE_HOLD_S)
    assert eng.done                 # nothing wanted -> trivially done


# ============================================== yaw / cart-rotation correction
def test_step_default_yaw_rad_reduces_exactly_to_the_pre_rotation_behaviour():
    # Regression guard (every test above depends on this): calling step()
    # without yaw_rad at all must place every nozzle at the same column and
    # at row = base_row + p, exactly as before this feature existed.
    ink = np.ones((NUM_NOZZLES + 10, 5), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=1.0, dose_hold_s=0.0)
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
        eng = CoverageEngine(ink, mm_per_column=1.0, dose_hold_s=0.0)
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
    eng = CoverageEngine(ink, mm_per_column=mm_per_column, dose_hold_s=0.0)
    yaw = math.radians(45.0)

    eng.step(u_mm=50.0, v_mm=5.0, t=0.0, yaw_rad=yaw)

    touched_cols = np.nonzero(eng.printed.any(axis=0))[0]
    assert touched_cols.size > 1, "a nonzero yaw must spread ink across more than one column"
    spread_cols = int(touched_cols.max() - touched_cols.min())

    # bar_length == (NUM_NOZZLES - 1) * NOZZLE_PITCH_MM == NOZZLE_BAR_WIDTH_MM
    # exactly (see geometry.py) -- the u-extent across the whole bar is
    # bar_length * sin(yaw_rad), converted to columns.
    expected_spread_cols = NOZZLE_BAR_WIDTH_MM * math.sin(yaw) / mm_per_column
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
    # (the far end of the bar, offset_along_bar == NOZZLE_BAR_WIDTH_MM
    # exactly -- see geometry.py) lands at u = u_mm - NOZZLE_BAR_WIDTH_MM,
    # v == v_mm unchanged -- the whole bar stays on nozzle 0's row and
    # spreads purely in u.
    mm_per_column = 0.5
    u_mm, v_mm = 50.0, 0.0
    yaw = math.pi / 2.0

    ink = np.ones((10, 200), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=mm_per_column, dose_hold_s=0.0)
    eng.step(u_mm=u_mm, v_mm=v_mm, t=0.0, yaw_rad=yaw)

    row0 = int(round(v_mm / NOZZLE_PITCH_MM))
    col0 = int(round(u_mm / mm_per_column))
    assert eng.printed[row0, col0], "nozzle 0 must stay at its un-rotated (row, col)"

    expected_last_col = col0 - int(round(NOZZLE_BAR_WIDTH_MM / mm_per_column))
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
    # NOZZLE_BAR_WIDTH_MM / 2, halfway between nozzles 75 and 76) -- but that
    # centre must fall at the midpoint of the column range step() actually
    # spreads across. Same 90 deg exact-arithmetic setup (sin=1, cos=0
    # exactly in IEEE 754) as the "swings to lower columns" test above, so
    # the comparison isn't itself blurred by rounding.
    mm_per_column = 0.5
    u_mm, v_mm = 50.0, 0.0
    yaw = math.pi / 2.0

    ink = np.ones((10, 200), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=mm_per_column, dose_hold_s=0.0)
    eng.step(u_mm=u_mm, v_mm=v_mm, t=0.0, yaw_rad=yaw)

    row0 = int(round(v_mm / NOZZLE_PITCH_MM))
    touched_cols = np.nonzero(eng.printed[row0])[0]
    observed_centre_col = (touched_cols.min() + touched_cols.max()) / 2.0

    du, dv = bar_offset_uv(NOZZLE_BAR_WIDTH_MM / 2.0, yaw)
    predicted_centre_col = (u_mm + du) / mm_per_column
    assert abs(predicted_centre_col - observed_centre_col) <= 1.0, (
        predicted_centre_col, observed_centre_col)
    # math.pi/2 isn't bit-exact (math.pi is a finite approximation of pi),
    # so cos(yaw) is ~6e-17, not exactly 0.0 -- negligible against
    # NOZZLE_PITCH_MM (~0.099mm) and rounds away in row0 above, but not
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
    mapper = PageMapper(cal, sensor_offset_row_mm=offset_mm + NOZZLE_BAR_WIDTH_MM / 2.0,
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
    eng = CoverageEngine(ink, mm_per_column=mm_per_column, dose_hold_s=0.0)
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
    # known to actually accumulate coverage at this dose_hold_s -- a much
    # faster v sweep (or too few samples) leaves each (row, col) key too
    # short-lived to ever complete a dose, making the comparison vacuous.
    samples = [(u, 2.0 + 0.02 * i, 0.001 * i)
               for i, u in enumerate(np.linspace(0.0, 6.0, 400))]

    def run(**kwargs):
        eng = CoverageEngine(ink, mm_per_column=0.2, dose_hold_s=0.0018, **kwargs)
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
    # reimplementation of the original per-nozzle rule (the code this
    # replaced) to actually pin the equivalence.
    def reference(yaw_rad):
        """The pre-grouping per-nozzle rule, written out standalone."""
        printed = np.zeros_like(ink, dtype=bool)
        nozzle_pixel = [None] * NUM_NOZZLES
        nozzle_since = [None] * NUM_NOZZLES
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
                wanted = in_bounds and bool(ink[row, col]) and not printed[row, col]
                if not wanted:
                    nozzle_pixel[p] = None
                    nozzle_since[p] = None
                    continue
                if nozzle_pixel[p] != (row, col):
                    nozzle_pixel[p] = (row, col)
                    nozzle_since[p] = t
                active[p] = True
                if t - nozzle_since[p] >= 0.0018:
                    printed[row, col] = True
            out.append(pack_nozzle_bits(active))
        return out, printed

    for yaw in (0.0, math.radians(30.0)):
        eng = CoverageEngine(ink, mm_per_column=0.2, dose_hold_s=0.0018,
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
    eng = CoverageEngine(ink, mm_per_column=1.0, dose_hold_s=DOSE_HOLD_S, nozzle_group=2)

    pattern, _ = eng.step(u_mm=0.0, v_mm=0.0, t=0.0)
    active = _unpack(pattern)
    assert active[0] and active[1], "both nozzles of the group must fire together"
    assert not active[2:].any(), "only group 0 (rows 0/1) should be firing"


def test_nozzle_group_2_marks_both_rows_printed_on_dose_completion():
    ink = np.zeros((NUM_NOZZLES, 1), dtype=bool)
    ink[0, 0] = True                        # row 1 is not itself wanted ink
    eng = CoverageEngine(ink, mm_per_column=1.0, dose_hold_s=DOSE_HOLD_S, nozzle_group=2)

    eng.step(u_mm=0.0, v_mm=0.0, t=0.0)
    eng.step(u_mm=0.0, v_mm=0.0, t=DOSE_HOLD_S + 0.05)   # dwell completes

    assert eng.printed[0, 0], "the actually-wanted row must be printed"
    assert eng.printed[1, 0], (
        "row 1 physically received ink too (tied to row 0's nozzle), so it "
        "must also be marked printed even though it wasn't itself wanted")


def test_nozzle_group_2_neither_member_fires_when_neither_is_wanted():
    ink = np.zeros((NUM_NOZZLES, 1), dtype=bool)   # nothing wanted anywhere
    eng = CoverageEngine(ink, mm_per_column=1.0, dose_hold_s=DOSE_HOLD_S, nozzle_group=2)

    pattern, _ = eng.step(u_mm=0.0, v_mm=0.0, t=0.0)
    active = _unpack(pattern)
    assert not active[0] and not active[1]
    eng.step(u_mm=0.0, v_mm=0.0, t=DOSE_HOLD_S + 0.05)
    assert not eng.printed.any()


def test_nozzle_group_2_under_yaw_fires_both_members_without_crashing():
    # Under yaw, group members can legitimately land in different columns
    # (see step()'s docstring) -- that must not crash the grouping logic,
    # and the group must still fire together.
    mm_per_column = 0.5
    ink = np.ones((300, 200), dtype=bool)
    yaw = math.radians(45.0)
    eng = CoverageEngine(ink, mm_per_column=mm_per_column, dose_hold_s=DOSE_HOLD_S,
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
    eng = CoverageEngine(ink, mm_per_column=1.0, dose_hold_s=DOSE_HOLD_S, nozzle_group=2)

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

    eng = CoverageEngine(ink, mm_per_column=mm_per_column, dose_hold_s=0.0)
    eng.step(u_mm=u_mm, v_mm=v_mm, t=0.0, yaw_rad=yaw)
    real_touched_cols = set(np.nonzero(eng.printed.any(axis=0))[0].tolist())

    assert real_touched_cols != mutated_touched_cols, (
        "the mutated (yaw-ignoring) engine must disagree with the real one "
        "-- if this ever matches, the spread test above has stopped "
        "actually exercising yaw_rad")


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print(f"OK: {name}")
    print("All coverage tests passed.")
